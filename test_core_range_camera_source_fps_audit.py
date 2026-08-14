from __future__ import annotations

import csv
import json

import core_range_camera_source_fps_audit as audit


def test_interval_metrics_basic():
    metrics = audit._interval_metrics([0.0, 0.1, 0.2, 0.3])
    assert metrics["frame_count"] == 4
    assert abs(metrics["median_fps"] - 10.0) < 1e-6
    assert metrics["duplicate_timestamp_count"] == 0


def test_interval_metrics_handles_duplicates_and_empty():
    metrics = audit._interval_metrics([1.0, 1.0, 1.1])
    assert metrics["duplicate_timestamp_count"] == 1

    empty = audit._interval_metrics([])
    assert empty["frame_count"] == 0
    assert empty["median_fps"] == 0.0


def test_parse_dmon_log_uses_header_to_locate_sm_column(tmp_path):
    log = tmp_path / "dmon.log"
    log.write_text(
        "# gpu    pwr  gtemp  mtemp     sm    mem    enc    dec    jpg    ofa   mclk   pclk  pviol  tviol \n"
        "# Idx      W      C      C      %      %      %      %      %      %    MHz    MHz      %   bool \n"
        "    0      3     44      -     29      5      0      0      0      0    405    210      0      0 \n"
        "    0      3     44      -     18      3      0      0      0      0    405    210     99      0 \n"
    )
    result = audit._parse_dmon_log(log)
    assert result["gpu_util_median_pct"] == 23.5
    assert result["gpu_util_samples"] == 2
    assert result["gpu_power_violation_median_pct"] == 49.5


def test_sequence_drop_count():
    assert audit._sequence_drop_count([1, 2, 3, 4]) == 0
    assert audit._sequence_drop_count([1, 2, 5]) == 2
    assert audit._sequence_drop_count([7]) == 0
    assert audit._sequence_drop_count([]) == 0


def _write_camera_trace_for_validate(run_dir, n=200, fps=45.0, dequeue_ratio=0.9):
    trace_path = run_dir / "camera_source_trace.jsonl"
    rows = []
    interval = 1.0 / fps
    for i in range(n):
        t = i * interval
        rows.append({
            "event": "camera_source_receipt", "trace_monotonic_s": t, "drone_id": "UAV-01",
            "frame_seq": i + 1, "sim_timestamp_s": t, "monotonic_receipt_s": t,
        })
        rows.append({
            "event": "camera_mailbox_store", "trace_monotonic_s": t + 0.001, "drone_id": "UAV-01",
            "frame_seq": i + 1, "monotonic_store_s": t + 0.001, "raw_frame_version": i + 1,
        })
        if i % int(1 / (1 - dequeue_ratio) if dequeue_ratio < 1 else 1) != 0 or dequeue_ratio >= 1:
            rows.append({
                "event": "camera_mailbox_dequeue", "trace_monotonic_s": t + 0.01, "drone_id": "UAV-01",
                "raw_frame_version": i + 1, "dequeue_monotonic_s": t + 0.01,
            })
            rows.append({
                "event": "tracker_output", "trace_monotonic_s": t + 0.015, "drone_id": "UAV-01",
                "raw_frame_version": i + 1, "tracker_output_monotonic_s": t + 0.015,
            })
    trace_path.write_text("\n".join(json.dumps(r) for r in rows) + "\n")


def test_validate_tracking_over_source_fraction_ignores_untracked_drone(tmp_path):
    # Two cameras publish (UAV-01 tracked, UAV-02 not); H5/tracker_output
    # only exists for UAV-01. The fraction must compare H5 against UAV-01's
    # own H2, not H2 summed across both drones (which would halve it).
    run_dir = tmp_path / "run"
    run_dir.mkdir()
    rows = []
    for i in range(100):
        t = i * (1.0 / 45.0)
        for drone_id in ("UAV-01", "UAV-02"):
            rows.append({
                "event": "camera_source_receipt", "trace_monotonic_s": t, "drone_id": drone_id,
                "frame_seq": i + 1, "sim_timestamp_s": t, "monotonic_receipt_s": t,
            })
        rows.append({
            "event": "camera_mailbox_dequeue", "trace_monotonic_s": t + 0.01, "drone_id": "UAV-01",
            "raw_frame_version": i + 1, "dequeue_monotonic_s": t + 0.01,
        })
        rows.append({
            "event": "tracker_output", "trace_monotonic_s": t + 0.015, "drone_id": "UAV-01",
            "raw_frame_version": i + 1, "tracker_output_monotonic_s": t + 0.015,
        })
    (run_dir / "camera_source_trace.jsonl").write_text("\n".join(json.dumps(r) for r in rows) + "\n")
    (run_dir / "capture_result.json").write_text(json.dumps({"session_id": 1}))

    output = tmp_path / "output"
    row = audit.validate(run_dir, output, "two_drone", prewarm_skip_s=0.0)

    # Nearly every UAV-01 frame reaches the tracker -> fraction should be
    # close to 1.0, not ~0.5 (which the pre-fix bug would have produced by
    # summing UAV-01+UAV-02 H2 frame counts as the denominator).
    assert row["tracking_over_source_fraction"] > 0.9


def test_validate_passes_gate_on_fast_synthetic_run(tmp_path):
    run_dir = tmp_path / "run"
    run_dir.mkdir()
    _write_camera_trace_for_validate(run_dir, n=300, fps=45.0, dequeue_ratio=0.95)

    diag_rows = []
    for i in range(50):
        diag_rows.append({
            "session_id": 1, "stage": "raw_range_computed",
            "timestamp_order_valid": True, "record_sha256": "abc123",
            "timestamp_stages": {
                "frame_receipt": {"timestamp_s": i * 0.02},
                "consume": {"timestamp_s": i * 0.02 + 0.05},
            },
            "timestamps": {"depth_inference_ms": 20.0},
        })
    (run_dir / "physical_diagnostics.jsonl").write_text(
        "\n".join(json.dumps(r) for r in diag_rows) + "\n"
    )
    (run_dir / "capture_result.json").write_text(json.dumps({"session_id": 1}))

    output = tmp_path / "output"
    row = audit.validate(run_dir, output, "fast_run", prewarm_skip_s=0.0)

    assert row["gate_pass"] is True
    assert row["raw_range_count"] == 50
    assert row["camera_source_median_fps"] > 25.0
    assert row["capture_consume_median_ms"] == 50.0

    with (output / "validation_smoke.csv").open() as stream:
        rows = list(csv.DictReader(stream))
    assert len(rows) == 1
    assert rows[0]["label"] == "fast_run"


def test_analyze_run_end_to_end(tmp_path):
    run_dir = tmp_path / "run"
    run_dir.mkdir()
    trace_path = run_dir / "camera_source_trace.jsonl"

    rows = []
    for i in range(20):
        t = i * 0.033
        rows.append({
            "event": "camera_source_receipt", "trace_monotonic_s": t, "drone_id": "UAV-01",
            "frame_seq": i + 1, "sim_timestamp_s": t, "monotonic_receipt_s": t,
        })
        rows.append({
            "event": "camera_mailbox_store", "trace_monotonic_s": t + 0.001, "drone_id": "UAV-01",
            "frame_seq": i + 1, "monotonic_store_s": t + 0.001, "raw_frame_version": i + 1,
        })
        rows.append({
            "event": "camera_mailbox_dequeue", "trace_monotonic_s": t + 0.05, "drone_id": "UAV-01",
            "raw_frame_version": i + 1, "dequeue_monotonic_s": t + 0.05,
        })
        rows.append({
            "event": "tracker_output", "trace_monotonic_s": t + 0.06, "drone_id": "UAV-01",
            "raw_frame_version": i + 1, "tracker_output_monotonic_s": t + 0.06,
        })
    trace_path.write_text("\n".join(json.dumps(r) for r in rows) + "\n")

    output = tmp_path / "output"
    result = audit.analyze_run(run_dir, output, "test_label", prewarm_skip_s=0.0)

    assert result["trace_events"] == len(rows)
    assert "UAV-01" in result["drone_ids"]

    with (output / "hop_fps_metrics.csv").open() as stream:
        hop_rows = list(csv.DictReader(stream))
    h2_rows = [r for r in hop_rows if r["hop"] == "H2_backend_gztransport_receipt"]
    assert len(h2_rows) == 1
    assert abs(float(h2_rows[0]["median_fps"]) - (1.0 / 0.033)) < 0.5

    delay_rows = [r for r in hop_rows if r["hop"] == "H3_to_H4_callback_delay_ms"]
    assert len(delay_rows) == 1
    assert float(delay_rows[0]["p5_interframe_interval_s"]) > 0
