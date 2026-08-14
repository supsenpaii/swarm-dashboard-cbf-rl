import math
from pathlib import Path

import pytest

from range_v2_baseline_eval import (
    UNSUPPORTED_METRICS,
    assign_core_bin,
    build_per_bin_metrics,
    build_per_group_metrics,
    build_policy_slice_metrics,
    file_sha256,
    metric_summary,
    stationary_metrics,
    verify_source_checksums,
)


def _row(
    error_m: float,
    *,
    group: str = "run_a/2",
    timestamp_s: float = 0.0,
    distance_m: float = 8.0,
    policy: str = "accepted",
    frame_index: int = 0,
) -> dict:
    raw = distance_m + error_m
    return {
        "source_dataset_id": group.split("/")[0],
        "source_dataset_root": f"artifacts/{group.split('/')[0]}",
        "source_manifest_sha256": "a" * 64,
        "source_samples_sha256": "b" * 64,
        "source_line_number": frame_index + 1,
        "source_record_sha256": "c" * 64,
        "run_id": group.split("/")[0],
        "session_id": int(group.split("/")[1]),
        "group_id": f"UAV-01.{group.split('/')[1]}",
        "atomic_group_key": group,
        "frame_index": frame_index,
        "measurement_timestamp_s": timestamp_s,
        "source_sim_timestamp_s": timestamp_s,
        "core_bin": assign_core_bin(distance_m),
        "ground_truth_distance_m": distance_m,
        "raw_physical_range_m": raw,
        "signed_error_m": error_m,
        "absolute_error_m": abs(error_m),
        "absolute_relative_error": abs(error_m) / distance_m,
        "catastrophic_error": abs(error_m) > 3.0,
        "deterministic_policy_label": policy,
        "deterministic_policy_reason": (
            "applicable" if policy == "accepted" else "test_rejection"
        ),
        "validity_gt": "unknown",
        "correction_mode": "disabled",
    }


def test_metric_summary_is_deterministic_and_exact_for_simple_values():
    rows = [
        _row(-1.0, timestamp_s=0.0, frame_index=0),
        _row(0.0, timestamp_s=1.0, frame_index=1),
        _row(1.0, timestamp_s=2.0, frame_index=2),
    ]
    first = metric_summary(rows)
    second = metric_summary(rows)
    assert first == second
    assert first["signed_bias_m"] == pytest.approx(0.0)
    assert first["median_absolute_error_m"] == pytest.approx(1.0)
    assert first["mae_m"] == pytest.approx(2.0 / 3.0)
    assert first["minimum_error_m"] == -1.0
    assert first["maximum_error_m"] == 1.0


@pytest.mark.parametrize(
    ("distance_m", "expected"),
    (
        (3.0, "3-4m"),
        (3.999, "3-4m"),
        (4.0, "4-5m"),
        (9.999, "9-10m"),
        (10.0, "10-11m"),
        (11.999, "11-12m"),
        (12.0, "11-12m"),
    ),
)
def test_core_bin_boundary_assignment(distance_m, expected):
    assert assign_core_bin(distance_m) == expected


@pytest.mark.parametrize("distance_m", (2.999, 12.001))
def test_non_core_distance_rejected(distance_m):
    with pytest.raises(ValueError, match="distance_outside_core"):
        assign_core_bin(distance_m)


def test_per_group_metrics_do_not_mix_groups():
    rows = [
        _row(1.0, group="run_a/2", timestamp_s=0.0, frame_index=0),
        _row(1.0, group="run_a/2", timestamp_s=1.0, frame_index=1),
        _row(-2.0, group="run_b/3", timestamp_s=0.0, frame_index=0),
        _row(-2.0, group="run_b/3", timestamp_s=1.0, frame_index=1),
    ]
    groups = build_per_group_metrics(rows)
    assert len(groups) == 2
    by_key = {group["atomic_group_key"]: group for group in groups}
    assert by_key["run_a/2"]["signed_bias_m"] == pytest.approx(1.0)
    assert by_key["run_b/3"]["signed_bias_m"] == pytest.approx(-2.0)


def test_frame_weighted_and_equal_group_aggregation_differ_as_expected():
    rows = [
        _row(10.0, group="long/2", timestamp_s=float(index), frame_index=index)
        for index in range(100)
    ]
    rows.append(_row(0.0, group="short/2", timestamp_s=0.0, frame_index=0))
    summary = metric_summary(rows)
    assert summary["mae_m"] == pytest.approx(1000.0 / 101.0)
    assert summary["equal_group_mae_m"] == pytest.approx(5.0)


def test_policy_accepted_and_rejected_slices_are_separate():
    rows = [
        _row(1.0, policy="accepted", frame_index=0),
        _row(2.0, policy="accepted", frame_index=1, timestamp_s=1.0),
        _row(-4.0, policy="rejected", frame_index=2, timestamp_s=2.0),
    ]
    slices = build_policy_slice_metrics(rows)
    aggregate = {
        row["slice"]: row
        for row in slices
        if row["scope"] == "aggregate"
    }
    assert aggregate["deterministic_policy_accepted"]["frame_count"] == 2
    assert aggregate["deterministic_policy_accepted"]["mae_m"] == pytest.approx(1.5)
    assert aggregate["deterministic_policy_rejected"]["frame_count"] == 1
    assert aggregate["deterministic_policy_rejected"]["mae_m"] == pytest.approx(4.0)
    assert "test_rejection" in aggregate["deterministic_policy_rejected"]["reason_counts"]
    rejected_bin = next(
        row
        for row in slices
        if row["slice"] == "deterministic_policy_rejected"
        and row["scope"] == "bin"
        and row["core_bin"] == "8-9m"
    )
    assert rejected_bin["same_bin_comparison_status"] == (
        "DESCRIPTIVE_ONLY_INSUFFICIENT_INDEPENDENT_GROUPS"
    )


def test_catastrophic_error_is_strictly_greater_than_three_metres():
    summary = metric_summary(
        [
            _row(3.0, group="a/2"),
            _row(-3.000001, group="b/2"),
        ]
    )
    assert summary["catastrophic_error_rate"] == pytest.approx(0.5)


def test_static_drift_metrics():
    rows = [
        _row(-3.0, timestamp_s=0.0, frame_index=0),
        _row(-2.0, timestamp_s=5.0, frame_index=1),
        _row(-1.0, timestamp_s=10.0, frame_index=2),
    ]
    metrics = stationary_metrics(rows)
    assert metrics["sample_duration_s"] == pytest.approx(10.0)
    assert metrics["median_absolute_frame_delta_m"] == pytest.approx(1.0)
    assert metrics["p95_absolute_frame_delta_m"] == pytest.approx(1.0)
    assert metrics["linear_drift_slope_m_s"] == pytest.approx(0.2)
    assert metrics["maximum_drift_m"] == pytest.approx(2.0)
    assert metrics["maximum_10_second_drift_m"] == pytest.approx(2.0)


def test_single_frame_group_reports_unavailable_temporal_differences():
    metrics = stationary_metrics([_row(0.0)])
    assert metrics["frame_count"] == 1
    assert metrics["sample_duration_s"] == 0.0
    assert metrics["output_std_m"] == 0.0
    assert metrics["median_absolute_frame_delta_m"] is None
    assert metrics["p95_absolute_frame_delta_m"] is None
    assert metrics["linear_drift_slope_m_s"] is None
    assert metrics["maximum_10_second_drift_m"] is None


@pytest.mark.parametrize("bad_value", (float("nan"), float("inf"), -float("inf")))
def test_nan_and_inf_fail_closed(bad_value):
    row = _row(0.0)
    row["raw_physical_range_m"] = bad_value
    with pytest.raises(ValueError, match="raw_physical_range_m_invalid"):
        metric_summary([row])


def test_missing_bins_are_explicit_and_not_interpolated():
    rows = [_row(0.2, distance_m=3.5)]
    bins = {item["core_bin"]: item for item in build_per_bin_metrics(rows)}
    assert bins["3-4m"]["status"] == "CHARACTERIZED"
    assert bins["10-11m"] == {
        "core_bin": "10-11m",
        "status": "MISSING",
        "frame_count": 0,
        "independent_group_count": 0,
        "reason": "no_locked_static_development_rows_no_interpolation",
    }
    assert bins["11-12m"]["status"] == "MISSING"


def test_unsupported_dynamic_metrics_remain_na_or_unknown():
    assert UNSUPPORTED_METRICS
    assert set(UNSUPPORTED_METRICS.values()) <= {"N/A", "UNKNOWN"}
    assert UNSUPPORTED_METRICS["range_response_lag"] == "N/A"
    assert UNSUPPORTED_METRICS["calibration_event_jump"] == "UNKNOWN"


def test_source_checksum_mismatch_aborts_evaluation(tmp_path):
    manifest = tmp_path / "manifest.json"
    samples = tmp_path / "samples.jsonl"
    manifest.write_text("{}\n", encoding="utf-8")
    samples.write_text("{}\n", encoding="utf-8")
    source = {
        "manifest_path": "manifest.json",
        "manifest_sha256": file_sha256(manifest),
        "samples_path": "samples.jsonl",
        "samples_sha256": file_sha256(samples),
    }
    verify_source_checksums(tmp_path, [source])
    samples.write_text('{"changed":true}\n', encoding="utf-8")
    with pytest.raises(ValueError, match="source_checksum_mismatch"):
        verify_source_checksums(tmp_path, [source])


def test_stationary_timestamp_must_be_strictly_increasing():
    rows = [
        _row(0.0, timestamp_s=1.0, frame_index=0),
        _row(0.1, timestamp_s=1.0, frame_index=1),
    ]
    with pytest.raises(ValueError, match="timestamp_not_strictly_increasing"):
        stationary_metrics(rows)
