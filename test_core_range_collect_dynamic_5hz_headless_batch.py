from __future__ import annotations

import json

import core_range_collect_dynamic_5hz_headless_batch as batch


def test_sessions_have_expected_shape_and_headless_suffix():
    values = batch.sessions()
    assert len(values) == 8
    ids = {row["session_id"] for row in values}
    assert all(sid.endswith("_5hz_headless") for sid in ids)
    assert all(row["group_id"] == row["session_id"] for row in values)
    assert all(row["dataset_role"] == batch.DATASET_ROLE for row in values)

    counts = {}
    for row in values:
        counts[row["scenario_type"]] = counts.get(row["scenario_type"], 0) + 1
    assert counts == {"approaching": 3, "receding": 3, "stop_and_hold": 2}

    recovery_rows = [row for row in values if row["roi_policy"] == "central_quantile_region"]
    assert len(recovery_rows) == 1
    assert recovery_rows[0]["session_id"] == "cdr_recede_left_yaw_5hz_headless"
    assert all(row["roi_policy"] == "default" for row in values if row is not recovery_rows[0])


def test_camera_source_median_fps_from_synthetic_trace(tmp_path):
    root = tmp_path / "run"
    root.mkdir()
    rows = []
    for i in range(200):
        t = i * (1.0 / 40.0)
        rows.append({
            "event": "camera_source_receipt", "trace_monotonic_s": t,
            "drone_id": "UAV-01", "monotonic_receipt_s": t, "frame_seq": i + 1,
        })
    (root / "camera_source_trace.jsonl").write_text("\n".join(json.dumps(r) for r in rows) + "\n")
    fps = batch._camera_source_median_fps(root)
    assert fps is not None
    assert abs(fps - 40.0) < 2.0


def test_camera_source_median_fps_none_when_no_trace(tmp_path):
    root = tmp_path / "run"
    root.mkdir()
    assert batch._camera_source_median_fps(root) is None


def test_gazebo_rtf_median_from_synthetic_log(tmp_path):
    root = tmp_path / "run"
    root.mkdir()
    log = root / "gz_world_stats.log"
    log.write_text(
        "--- 1.0 ---\nreal_time_factor: 0.99\niterations: 100\n"
        "--- 2.0 ---\nreal_time_factor: 0.97\niterations: 200\n"
    )
    median = batch._gazebo_rtf_median(root)
    assert median is not None
    assert 0.9 < median < 1.0


def test_gazebo_rtf_median_none_when_no_log(tmp_path):
    root = tmp_path / "run"
    root.mkdir()
    assert batch._gazebo_rtf_median(root) is None


def test_gazebo_rtf_quality_rejects_long_stutter(tmp_path):
    log = tmp_path / "stats.log"
    log.write_text("\n".join(
        f"real_time_factor: {value}" for value in (1.0, 1.0, 0.2, 0.1, 0.3, 1.0, 1.0)
    ))
    quality = batch._gazebo_rtf_quality(log)
    assert quality["longest_consecutive_rtf_le_0_3_samples"] == 3
    assert quality["long_stutter_pass"] is False


def test_rtf_precheck_requires_three_samples_and_stable_median(tmp_path):
    log = tmp_path / "stats.log"
    output = tmp_path / "precheck.json"
    log.write_text("\n".join(
        f"real_time_factor: {value}" for value in (0.1, 1.0, 0.99, 0.98, 1.0)
    ))
    result = batch.rtf_check(log, output, tail_samples=5)
    assert result["pass"] is True
    assert json.loads(output.read_text()) == result


def test_max_attempts_matches_continuation_contract():
    assert batch.MAX_ATTEMPTS == 8
