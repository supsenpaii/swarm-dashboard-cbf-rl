from __future__ import annotations

import ast
import csv
import json
import os
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

import numpy as np

import core_range_direct_dynamic_replay as replay
import core_range_dynamic_capture as capture


WORKSPACE = Path(__file__).resolve().parent
OUTPUT = WORKSPACE / "artifacts/core_range_3_12m/direct_dynamic_replay"


def feature_row() -> dict[str, float]:
    return {name: float(index + 1) for index, name in enumerate(replay.FEATURE_NAMES)}


def test_frozen_candidate_checksums_match_selected_direct_artifacts() -> None:
    frozen = replay._candidate_sources(WORKSPACE)
    assert len(frozen["models"]) == 3
    assert len(frozen["preprocessing"]) == 3
    assert all(item["sha256"] == replay.sha256_file(WORKSPACE / item["path"]) for item in frozen["models"])
    assert len(frozen["candidate_checksum"]) == 64


def test_replay_module_has_no_model_fit_or_training_call() -> None:
    tree = ast.parse(Path(replay.__file__).read_text(encoding="utf-8"))
    called = []
    for node in ast.walk(tree):
        if isinstance(node, ast.Call):
            if isinstance(node.func, ast.Attribute): called.append(node.func.attr)
            elif isinstance(node.func, ast.Name): called.append(node.func.id)
    assert "train" not in called
    assert "fit" not in called


def test_precommitted_sessions_are_independent_and_balanced() -> None:
    sessions = replay._sessions()
    assert len(sessions) == 8
    assert len({row["session_id"] for row in sessions}) == 8
    assert len({row["group_id"] for row in sessions}) == 8
    assert sum(row["scenario_type"] == "approaching" for row in sessions) == 3
    assert sum(row["scenario_type"] == "receding" for row in sessions) == 3
    assert sum(row["scenario_type"] == "stop_and_hold" for row in sessions) == 2


def test_precommitted_throughput_amendment_changes_collection_timing_only() -> None:
    plan, candidate = replay.verify_frozen_candidate(WORKSPACE, OUTPUT)
    amendment = plan["effective_amendment"]
    assert amendment["amendment_id"] == "core_range_direct_dynamic_collection_throughput_repair_v001"
    assert len(amendment["sha256"]) == 64
    assert candidate["frozen"]["candidate_checksum"] == plan["candidate_checksum"]
    assert len(plan["sessions"]) == 8
    assert plan["effective_amendment_v002"]["amendment_id"] == "core_range_direct_dynamic_endpoint_observability_repair_v002"
    assert plan["effective_amendment_v003"]["amendment_id"] == "core_range_direct_dynamic_prewarm_retry_v003"
    assert plan["effective_amendment_v004"]["amendment_id"] == "core_range_direct_dynamic_gazebo_service_retry_v004"
    assert plan["effective_amendment_v005"]["amendment_id"] == "core_range_direct_dynamic_near_lateral_roi_repair_v005"
    assert plan["effective_amendment_v006"]["amendment_id"] == "core_range_dynamic_roi_recovery_repair_and_replay_v006"
    assert {row["session_id"] for row in plan["sessions"]} == {
        "cdr_approach_center_r1",
        "cdr_approach_left_r2",
        "cdr_approach_right_yaw_r3",
        "cdr_recede_center_r3",
        "cdr_recede_left_yaw_r4",
        "cdr_recede_right_r5",
        "cdr_stop_approach_6_r1",
        "cdr_stop_recede_9_r2",
    }
    assert all(row["speed_m_s"] == 0.125 for row in plan["sessions"])
    assert all(
        row["hold_duration_s"] == 15.0
        for row in plan["sessions"]
        if row["session_id"] != "cdr_approach_center_r1"
    )
    assert plan["development_gates"]["equal_group_dynamic_mae_m_max"] == 1.0
    assert plan["smoothing_policy"]["time_constant_s"] == 0.20
    near_lateral = {
        row["session_id"]: row["initial_bbox_normalized"]
        for row in plan["sessions"]
        if row["session_id"] in {
            "cdr_recede_left_yaw_r4", "cdr_recede_right_r5", "cdr_stop_recede_9_r2"
        }
    }
    assert len(near_lateral) == 3
    # v006 reverted all three near-lateral bboxes to their original (pre-v005)
    # geometry: the accepted fix is the opt-in ROI recovery-policy statistic
    # swap (per-geometry), not a bbox scale change.
    assert all(np.isclose(box[2], 0.204) and np.isclose(box[3], 0.279) for box in near_lateral.values())


def test_gazebo_pose_service_retry_is_bounded_and_preserves_command() -> None:
    responses = [
        SimpleNamespace(returncode=0, stdout="", stderr="timeout"),
        SimpleNamespace(returncode=0, stdout="data: true\n", stderr=""),
    ]
    with patch.object(capture.subprocess, "run", side_effect=responses) as invoked, patch.object(capture.time, "sleep"):
        result = capture.set_target_pose(5.0, 0.0, 1.0, 0.0)
    assert invoked.call_count == 2
    assert result["range_m"] == 5.0
    assert result["x_m"] == 5.0
    assert result["gazebo_service_attempts"] == 2


def test_causal_smoothing_uses_only_current_and_past() -> None:
    timestamps = [0.0, 0.2, 0.4, 0.6, 0.8]
    values = [8.0, 7.0, 6.0, 9.0, 3.0]
    complete = replay.causal_smooth(timestamps, values)
    prefix = replay.causal_smooth(timestamps[:3], values[:3])
    assert np.allclose(complete[:3], prefix)
    assert complete[0] == values[0]


def test_causal_smoothing_rejects_nonmonotonic_timestamp() -> None:
    try:
        replay.causal_smooth([0.0, 0.0], [5.0, 6.0])
    except ValueError as error:
        assert "timestamp" in str(error)
    else:
        raise AssertionError("nonmonotonic timestamp accepted")


def test_bbox_perturbation_is_deterministic_and_only_changes_bbox_features() -> None:
    original = feature_row()
    original.update({
        "bbox_center_x_fraction": 0.5, "bbox_center_y_fraction": 0.4,
        "bbox_width_fraction": 0.2, "bbox_height_fraction": 0.1,
        "bbox_area_fraction": 0.02, "bbox_aspect_ratio": 2.0,
    })
    first = replay.perturb_bbox_features(original, "scale_plus_5")
    second = replay.perturb_bbox_features(original, "scale_plus_5")
    assert first == second
    assert np.isclose(first["bbox_width_fraction"], 0.21)
    assert np.isclose(first["bbox_height_fraction"], 0.105)
    changed = {name for name in replay.FEATURE_NAMES if first[name] != original[name]}
    assert changed <= {
        "bbox_width_fraction", "bbox_height_fraction",
        "bbox_area_fraction", "bbox_aspect_ratio",
    }


def test_partial_box_and_center_jitter_geometry() -> None:
    original = feature_row()
    original.update({
        "bbox_center_x_fraction": 0.5, "bbox_center_y_fraction": 0.4,
        "bbox_width_fraction": 0.2, "bbox_height_fraction": 0.1,
        "bbox_area_fraction": 0.02, "bbox_aspect_ratio": 2.0,
    })
    cropped = replay.perturb_bbox_features(original, "crop_left_10")
    assert np.isclose(cropped["bbox_width_fraction"], 0.18)
    assert np.isclose(cropped["bbox_center_x_fraction"], 0.51)
    centered = replay.perturb_bbox_features(original, "center_y_minus_5")
    assert np.isclose(centered["bbox_center_y_fraction"], 0.35)
    assert centered["bbox_width_fraction"] == original["bbox_width_fraction"]


def test_equal_group_aggregation_is_not_frame_weighted() -> None:
    keys = ("signed_bias_m", "median_absolute_error_m", "mae_m", "rmse_m", "p90_abs_error_m", "p95_abs_error_m", "mean_absolute_relative_error", "error_gt_1m_fraction", "error_gt_2m_fraction", "error_gt_3m_fraction")
    rows = [{key: 1.0 for key in keys}, {key: 3.0 for key in keys}]
    assert replay._equal_group(rows)["mae_m"] == 2.0


def test_alignment_lag_recovers_known_causal_delay() -> None:
    timestamps = np.arange(0.0, 10.0, 0.05)
    truth = 7.0 + np.sin(timestamps)
    delayed = np.interp(timestamps - 0.20, timestamps, truth)
    lag = replay.alignment_lag_s(timestamps, truth, delayed, maximum_s=1.0, step_s=0.05)
    assert abs(lag - 0.20) <= 0.051


def test_stop_response_calculation() -> None:
    timestamps = np.arange(0.0, 8.0, 0.2)
    truth = np.maximum(6.0, 8.0 - 0.5 * timestamps)
    rows = [
        {"scenario_type": "stop_and_hold", "session_id": "stop", "measurement_timestamp_s": float(t), "ground_truth_range_m": float(gt)}
        for t, gt in zip(timestamps, truth, strict=True)
    ]
    result = replay._stop_row(rows, truth.copy(), "perfect")
    assert result is not None
    assert result["status"] == "PASS_OBSERVED"
    assert result["settling_time_s"] == 0.0
    assert result["stationary_std_m"] == 0.0


def test_feature_and_prediction_checksum_are_deterministic() -> None:
    features = feature_row()
    first = replay.canonical_sha256([features[name] for name in replay.FEATURE_NAMES])
    second = replay.canonical_sha256([features[name] for name in replay.FEATURE_NAMES])
    assert first == second and len(first) == 64


def test_collection_scripts_have_no_vehicle_controller_command() -> None:
    shell = (WORKSPACE / "core_range_collect_dynamic_scenario.sh").read_text().lower()
    capture = (WORKSPACE / "core_range_dynamic_capture.py").read_text().lower()
    assert "/api/follow" not in shell + capture
    assert "/api/control" not in shell + capture
    assert "offboard" not in shell + capture
    assert "trajectory_setpoint" not in shell + capture
    assert "x500_custom_1" in capture
    assert "gazebo_set_pose_independent_script" in capture


def test_residual_correction_remains_default_off() -> None:
    assert os.environ.get("SWARM_RANGE_RESIDUAL_MODE", "off").lower() == "off"
    shell = (WORKSPACE / "core_range_collect_dynamic_scenario.sh").read_text()
    assert "SWARM_RANGE_RESIDUAL_MODE=off" in shell


def test_completed_replay_artifacts_if_present() -> None:
    manifest_path = OUTPUT / "dynamic_replay_manifest.json"
    if not manifest_path.exists():
        return
    manifest = json.loads(manifest_path.read_text())
    if manifest["conclusion"] == "DYNAMIC_REPLAY_BLOCKED_BY_DATA_INTEGRITY":
        assert manifest["counts"]["planned_sessions"] == 8
        assert manifest["counts"]["prediction_rows"] == 0
        assert manifest["model_loaded_for_prediction"] is False
    else:
        assert manifest["counts"]["sessions"] == 8
    assert manifest["scope_guards"]["retrain_model"] is False
    for relative, checksum in manifest["artifacts"].items():
        assert replay.sha256_file(OUTPUT / relative) == checksum
    with (OUTPUT / "prediction_rows.csv").open() as stream:
        rows = list(csv.DictReader(stream))
    expected = manifest["counts"].get("prediction_rows_including_stress", manifest["counts"].get("prediction_rows"))
    assert len(rows) == expected
    assert all(len(row["feature_vector_checksum"]) == 64 for row in rows)
    assert all(len(row["prediction_checksum"]) == 64 for row in rows)
    assert all(row["model_checksum"] == manifest["candidate_checksum"] for row in rows)
