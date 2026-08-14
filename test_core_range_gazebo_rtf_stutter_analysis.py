from __future__ import annotations

import json

import pytest

import core_range_gazebo_rtf_stutter_analysis as analysis


def _synthetic_trace(rtf_sequence, start_wall=1000.0):
    rows = []
    for i, rtf in enumerate(rtf_sequence):
        rows.append({
            "elapsed_s": float(i), "wall_clock_s": start_wall + i,
            "gazebo_rtf": rtf, "cpu_freq_mean_mhz": 1000.0 + i,
            "thermal_zone_max_c": 50.0, "mem_available_mb": 4000.0,
            "disk_write_kb_per_s": 10.0, "context_switches_per_s": 5000.0,
            "gpu_util_pct": 20.0, "gpu_clock_sm_mhz": 500.0,
            "process_thread_count": None,
        })
    return rows


def test_detect_transitions_finds_stable_to_stutter():
    sequence = [0.99] * 10 + [0.05] * 10
    trace = _synthetic_trace(sequence)
    transitions = analysis.detect_transitions(trace)
    assert len(transitions) == 1
    assert transitions[0]["kind"] == "stable_to_stutter"
    assert transitions[0]["transition_elapsed_s"] == 10.0


def test_detect_transitions_finds_stutter_to_stable():
    sequence = [0.05] * 10 + [0.99] * 10
    trace = _synthetic_trace(sequence)
    transitions = analysis.detect_transitions(trace)
    assert len(transitions) == 1
    assert transitions[0]["kind"] == "stutter_to_stable"


def test_detect_transitions_ignores_short_runs():
    # A 2-sample dip is below MIN_RUN_LENGTH_S=3, should not register.
    sequence = [0.99] * 10 + [0.05] * 2 + [0.99] * 10
    trace = _synthetic_trace(sequence)
    transitions = analysis.detect_transitions(trace)
    assert transitions == []


def test_detect_transitions_no_transition_when_all_stable():
    trace = _synthetic_trace([0.95] * 20)
    assert analysis.detect_transitions(trace) == []


def test_nearest_rtf_matches_closest_sample():
    trace = _synthetic_trace([0.99, 0.5, 0.05], start_wall=1000.0)
    assert analysis._nearest_rtf(trace, 1000.4) == 0.99
    assert analysis._nearest_rtf(trace, 1001.6) == 0.05
    assert analysis._nearest_rtf(trace, 5000.0) is None  # beyond max_gap_s


def test_rtf_context_comparison_labels_rows_by_nearest_rtf(tmp_path):
    # Trace: wall_clock_s 1000 (stable), 1001 (stutter).
    trace = _synthetic_trace([0.99, 0.03], start_wall=1000.0)

    def _diag_row(monotonic_ts, raw_m, gt_m):
        return {
            "stage": "raw_range_computed",
            "timestamp_stages": {
                "frame_receipt": {"timestamp_s": monotonic_ts},
                "consume": {"timestamp_s": monotonic_ts + 0.05},
            },
            "raw_range": {"physics_slant_range_m": raw_m},
            "ground_truth": {"distance_m": gt_m},
            "record_sha256": "x", "trace_identity_sha256": "y",
            "anchors": {"per_grid_point": [{}] * 96},
        }

    # offset chosen so monotonic 0 -> wall 1000 (stable sample) and
    # monotonic 1 -> wall 1001 (stutter sample).
    offset = 1000.0
    rows = [_diag_row(0.0, 5.0, 5.1), _diag_row(1.0, 6.0, 6.2)]
    sidecar = tmp_path / "physical_diagnostics.jsonl"
    sidecar.write_text("\n".join(json.dumps(r) for r in rows) + "\n")
    camera_trace = tmp_path / "camera_source_trace.jsonl"
    camera_trace.write_text("")

    result = analysis.rtf_context_comparison(trace, sidecar, camera_trace, monotonic_to_wall_offset=offset)
    assert result["stable"]["raw_range_row_count"] == 1
    assert result["stutter"]["raw_range_row_count"] == 1
    assert result["stable"]["raw_range_vs_gt_abs_error_m"]["median"] == pytest.approx(0.1, abs=1e-6)
    assert result["stutter"]["raw_range_vs_gt_abs_error_m"]["median"] == pytest.approx(0.2, abs=1e-6)


def test_resource_correlation_perfect_negative():
    # cpu_freq_mean_mhz increases as rtf decreases -> strong negative corr.
    trace = _synthetic_trace([1.0 - 0.01 * i for i in range(20)])
    corr = analysis.resource_correlation(trace)
    assert corr["cpu_freq_mean_mhz"] is not None
    assert corr["cpu_freq_mean_mhz"] < -0.9
