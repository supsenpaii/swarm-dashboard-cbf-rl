"""Frozen direct-range candidate replay on dynamic development sessions.

The module loads existing XGBoost artifacts for offline prediction only.  It
contains no fitting path and has no runtime/controller integration.
"""

from __future__ import annotations

import argparse
import copy
import csv
import hashlib
import json
import math
import os
from pathlib import Path
from statistics import mean, median
from typing import Any, Iterable, Mapping, Sequence

import numpy as np

from core_range_logging_eval import load_jsonl, sha256_file, verify_record
from core_range_xgboost_benchmark import (
    FEATURE_NAMES,
    FoldPreprocessor,
    canonical_sha256,
    extract_features,
    metric_values,
)
from range_physical_diagnostics import validate_timestamp_stages


GATE_ID = "core_range_direct_dynamic_replay_20260804_v001"
DATASET_ROLE = "core_range_dynamic_development"
SEED = 52
MODEL_KIND = "direct_C_shallow_three_fold_mean_ensemble"
CLIP_BOUNDS = (3.0, 12.0)
SMOOTHING = {
    "kind": "causal_exponential_time_constant",
    "time_constant_s": 0.20,
    "initialization": "first_current_prediction",
    "session_reset": True,
    "future_frames": False,
}
DIRECTION_DEADBAND_M_S = 0.10
SETTLING_ERROR_BAND_M = 0.50
SETTLING_STABLE_FRAMES = 3
GATES = {
    "equal_group_dynamic_mae_m_max": 1.0,
    "equal_group_dynamic_p90_m_max": 2.0,
    "worst_session_mae_m_max": 1.5,
    "error_gt_3m_fraction_max": 0.01,
    "median_absolute_lag_s_max": 0.30,
    "worst_stop_settling_time_s_max": 1.0,
    "worst_stop_stationary_std_m_max": 0.30,
    "bbox_pm5_mae_degradation_m_max": 0.30,
    "bbox_pm5_median_prediction_shift_m_max": 0.50,
    "bbox_pm10_catastrophic_error_fraction_max": 0.0,
}
STRESS_VARIANTS = (
    "original",
    "scale_minus_2", "scale_plus_2", "scale_minus_5", "scale_plus_5",
    "scale_minus_10", "scale_plus_10", "scale_minus_15", "scale_plus_15",
    "center_x_minus_2", "center_x_plus_2", "center_x_minus_5", "center_x_plus_5",
    "center_y_minus_2", "center_y_plus_2", "center_y_minus_5", "center_y_plus_5",
    "crop_left_5", "crop_right_5", "crop_top_5", "crop_bottom_5",
    "crop_left_10", "crop_right_10", "crop_top_10", "crop_bottom_10",
)


def _json(path: Path) -> Any:
    return json.loads(path.read_text(encoding="utf-8"))


def _write_json(path: Path, value: Any) -> None:
    path.write_text(json.dumps(value, indent=2, sort_keys=True, allow_nan=False) + "\n", encoding="utf-8")


def _write_csv(path: Path, rows: Sequence[Mapping[str, Any]]) -> None:
    values = list(rows)
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(values[0]) if values else [])
        writer.writeheader()
        writer.writerows(values)


def _write_empty_csv(path: Path, fieldnames: Sequence[str]) -> None:
    with path.open("w", encoding="utf-8", newline="") as stream:
        csv.DictWriter(stream, fieldnames=list(fieldnames)).writeheader()


def _candidate_sources(workspace: Path) -> dict[str, Any]:
    benchmark = workspace / "artifacts/core_range_3_12m/xgboost_benchmark"
    benchmark_manifest = _json(benchmark / "benchmark_manifest.json")
    if benchmark_manifest.get("conclusion") != "DIRECT_XGBOOST_BEST_DEVELOPMENT_CANDIDATE":
        raise ValueError("direct_candidate_conclusion_mismatch")
    if benchmark_manifest.get("selected_configs", {}).get("direct") != "C_shallow":
        raise ValueError("direct_candidate_config_mismatch")
    frozen: dict[str, Any] = {
        "benchmark_manifest": {
            "path": str((benchmark / "benchmark_manifest.json").relative_to(workspace)),
            "sha256": sha256_file(benchmark / "benchmark_manifest.json"),
        },
        "benchmark_plan": {
            "path": str((benchmark / "benchmark_plan.json").relative_to(workspace)),
            "sha256": sha256_file(benchmark / "benchmark_plan.json"),
        },
        "feature_contract": {
            "path": str((benchmark / "feature_contract.json").relative_to(workspace)),
            "sha256": sha256_file(benchmark / "feature_contract.json"),
        },
        "fold_assignments": {
            "path": str((benchmark / "fold_assignments.csv").relative_to(workspace)),
            "sha256": sha256_file(benchmark / "fold_assignments.csv"),
        },
        "models": [],
        "preprocessing": [],
    }
    artifact_checksums = benchmark_manifest.get("artifacts") or {}
    for fold in range(3):
        model = benchmark / f"models/direct_fold_{fold}.json"
        preprocessing = benchmark / f"preprocessing/direct_C_shallow_fold_{fold}.json"
        for path in (model, preprocessing):
            relative = str(path.relative_to(benchmark))
            if not path.is_file() or artifact_checksums.get(relative) != sha256_file(path):
                raise ValueError(f"frozen_candidate_artifact_mismatch:{relative}")
        frozen["models"].append({"fold": fold, "path": str(model.relative_to(workspace)), "sha256": sha256_file(model)})
        frozen["preprocessing"].append({"fold": fold, "path": str(preprocessing.relative_to(workspace)), "sha256": sha256_file(preprocessing)})
    frozen["candidate_checksum"] = canonical_sha256({
        "kind": MODEL_KIND,
        "models": frozen["models"],
        "preprocessing": frozen["preprocessing"],
        "feature_contract": frozen["feature_contract"],
        "clip_bounds_m": CLIP_BOUNDS,
        "smoothing": SMOOTHING,
        "seed": SEED,
    })
    return frozen


def _sessions() -> list[dict[str, Any]]:
    base = {
        "dataset_role": DATASET_ROLE,
        "observer_pose": {"x_m": 0.0, "y_m": 0.0, "z_m": 1.0},
        "target_z_m": 1.0,
        "speed_m_s": 0.25,
        "pose_update_rate_hz": 5.0,
        "prewarm_timeout_s": 90,
        "minimum_raw_frames": 40,
        "logging_schema_version": "core_range_logging_3_12m_v001",
        "seed": SEED,
    }
    specifications = [
        ("cdr_approach_center", "approaching", 11.5, 3.5, 0.0, -10.0, 0.0, 0.0, [0.465, 0.250, 0.070, 0.095], 0.0),
        ("cdr_approach_left", "approaching", 11.5, 3.5, -0.75, -11.0, 0.0, 0.0, [0.390, 0.240, 0.070, 0.095], 0.0),
        ("cdr_approach_right_yaw", "approaching", 11.5, 3.5, 0.75, -12.0, -15.0, 15.0, [0.545, 0.230, 0.070, 0.095], 0.0),
        ("cdr_recede_center", "receding", 3.5, 11.5, 0.0, -10.0, 0.0, 0.0, [0.398, 0.155, 0.204, 0.279], 0.0),
        ("cdr_recede_left_yaw", "receding", 3.5, 11.5, -0.75, -11.0, 15.0, -15.0, [0.535, 0.145, 0.204, 0.279], 0.0),
        ("cdr_recede_right", "receding", 3.5, 11.5, 0.75, -12.0, 0.0, 0.0, [0.308, 0.135, 0.204, 0.279], 0.0),
        ("cdr_stop_approach_6", "stop_and_hold", 11.5, 6.0, 0.0, -10.0, 0.0, 0.0, [0.465, 0.250, 0.070, 0.095], 6.0),
        ("cdr_stop_recede_9", "stop_and_hold", 3.5, 9.0, 0.50, -12.0, -10.0, 10.0, [0.308, 0.135, 0.204, 0.279], 6.0),
    ]
    result = []
    for session_id, kind, start, end, lateral, pitch, yaw_start, yaw_end, bbox, hold in specifications:
        result.append({
            **base,
            "session_id": session_id,
            "group_id": session_id,
            "scenario_type": kind,
            "start_range_m": start,
            "end_range_m": end,
            "lateral_offset_m": lateral,
            "gimbal_pitch_deg": pitch,
            "target_yaw_start_deg": yaw_start,
            "target_yaw_end_deg": yaw_end,
            "initial_bbox_normalized": bbox,
            "movement_duration_s": abs(end - start) / base["speed_m_s"],
            "hold_duration_s": hold,
            "context": (
                "center" if lateral == 0.0 and yaw_start == yaw_end
                else "lateral_with_yaw_variation" if yaw_start != yaw_end
                else "lateral"
            ),
        })
    return result


def prepare(workspace: Path, output: Path) -> dict[str, Any]:
    if output.exists() and any(output.iterdir()):
        raise ValueError("dynamic_replay_output_not_empty")
    output.mkdir(parents=True, exist_ok=True)
    frozen = _candidate_sources(workspace)
    candidate_manifest = {
        "candidate_id": "direct_C_shallow_static_development_candidate_v001",
        "model_kind": MODEL_KIND,
        "ensemble_policy": "arithmetic_mean_of_three_fold_predictions_then_clip",
        "frozen": frozen,
        "retraining_permitted": False,
        "runtime_integration": False,
    }
    candidate_path = output / "candidate_model_manifest.json"
    _write_json(candidate_path, candidate_manifest)
    sessions = _sessions()
    plan = {
        "gate_id": GATE_ID,
        "created_before_collection_or_replay": True,
        "seed": SEED,
        "candidate_model_manifest_sha256": sha256_file(candidate_path),
        "candidate_checksum": frozen["candidate_checksum"],
        "prediction_clip_bounds_m": list(CLIP_BOUNDS),
        "smoothing_policy": SMOOTHING,
        "primary_gated_output": "direct_ensemble_causal_smoothed",
        "comparisons": ["raw_physical_range", "direct_ensemble_unsmoothed", "direct_ensemble_causal_smoothed"],
        "direction_deadband_m_s": DIRECTION_DEADBAND_M_S,
        "direction_deadband_basis": "0.25m/s scripted motion, simulation-exact GT, conservative 0.10m/s controller-neutral deadband",
        "lag_policy": {
            "primary": "causal_alignment_MAE_grid_search",
            "grid_step_s": 0.05,
            "maximum_absolute_lag_s": 2.0,
            "secondary": "range_rate_cross_correlation_on_median_sample_grid",
        },
        "stop_policy": {
            "settling_error_band_m": SETTLING_ERROR_BAND_M,
            "stable_frames": SETTLING_STABLE_FRAMES,
            "stop_detected_from_GT_rate_deadband": True,
        },
        "bbox_stress": {
            "variants": list(STRESS_VARIANTS),
            "scale_changes_apply_to_width_and_height_about_fixed_center": True,
            "center_offsets_are_absolute_image_fractions": True,
            "partial_box_crop_is_fraction_of_original_side": True,
            "physical_non_bbox_features_held_constant": True,
            "seed": SEED,
        },
        "development_gates": GATES,
        "failure_precedence": [
            "DYNAMIC_REPLAY_BLOCKED_BY_DATA_INTEGRITY",
            "DIRECT_CANDIDATE_FAILS_BBOX_ROBUSTNESS",
            "DIRECT_CANDIDATE_FAILS_TEMPORAL_RESPONSE",
            "DIRECT_CANDIDATE_FAILS_DYNAMIC_ACCURACY",
            "DIRECT_CANDIDATE_DYNAMIC_REPLAY_PASS",
        ],
        "sessions": sessions,
        "scope_guards": {
            "retrain_model": False,
            "hyperparameter_or_feature_tuning": False,
            "runtime_backend_controller_px4_change": False,
            "follow_offboard_arm_takeoff_mode_closed_loop": False,
            "residual_runtime_default": "off",
            "final_holdout": False,
            "target_motion_source": "independent_gazebo_set_pose_script_not_model_output",
        },
    }
    plan_path = output / "dynamic_replay_plan.json"
    _write_json(plan_path, plan)
    print(json.dumps({
        "phase": "DYNAMIC_REPLAY_PRECOMMIT_COMPLETE_NO_COLLECTION_NO_REPLAY",
        "plan_sha256": sha256_file(plan_path),
        "candidate_checksum": frozen["candidate_checksum"],
        "sessions": len(sessions),
    }, indent=2))
    return plan


def verify_frozen_candidate(workspace: Path, output: Path) -> tuple[dict[str, Any], dict[str, Any]]:
    plan_path = output / "dynamic_replay_plan.json"
    candidate_path = output / "candidate_model_manifest.json"
    plan, candidate = _json(plan_path), _json(candidate_path)
    if plan.get("gate_id") != GATE_ID or plan.get("seed") != SEED:
        raise ValueError("dynamic_replay_plan_invalid")
    if plan.get("candidate_model_manifest_sha256") != sha256_file(candidate_path):
        raise ValueError("candidate_manifest_changed_after_precommit")
    current = _candidate_sources(workspace)
    if current["candidate_checksum"] != plan.get("candidate_checksum"):
        raise ValueError("candidate_checksum_changed_after_precommit")
    if candidate.get("retraining_permitted") is not False:
        raise ValueError("candidate_retraining_policy_invalid")
    amendment_path = output / "dynamic_replay_plan_amendment_v001.json"
    if amendment_path.exists():
        amendment = _json(amendment_path)
        if amendment.get("parent_plan_sha256") != sha256_file(plan_path):
            raise ValueError("dynamic_plan_amendment_parent_mismatch")
        if amendment.get("created_before_replacement_capture") is not True:
            raise ValueError("dynamic_plan_amendment_not_precommitted")
        if amendment.get("model_predictions_or_raw_errors_inspected") is not False:
            raise ValueError("dynamic_plan_amendment_result_leakage")
        if amendment.get("candidate_or_gate_change") is not False:
            raise ValueError("dynamic_plan_amendment_candidate_change")
        changes = amendment.get("session_changes") or {}
        if set(changes) != {row["session_id"] for row in plan["sessions"]}:
            raise ValueError("dynamic_plan_amendment_session_set_mismatch")
        effective_sessions = []
        for session in plan["sessions"]:
            changed = dict(session)
            changed.update(changes[session["session_id"]])
            if changed.get("replacement_for") != session["session_id"]:
                raise ValueError("dynamic_plan_amendment_trace_mismatch")
            effective_sessions.append(changed)
        if len({row["session_id"] for row in effective_sessions}) != 8:
            raise ValueError("dynamic_plan_amendment_duplicate_session")
        plan = dict(plan)
        plan["sessions"] = effective_sessions
        plan["effective_amendment"] = {
            "amendment_id": amendment["amendment_id"],
            "sha256": sha256_file(amendment_path),
            "reason": amendment["reason"],
        }
    second_amendment_path = output / "dynamic_replay_plan_amendment_v002.json"
    if second_amendment_path.exists():
        second = _json(second_amendment_path)
        if second.get("parent_plan_sha256") != sha256_file(plan_path):
            raise ValueError("dynamic_plan_second_amendment_parent_mismatch")
        if second.get("parent_amendment_sha256") != sha256_file(amendment_path):
            raise ValueError("dynamic_plan_second_amendment_chain_mismatch")
        if second.get("created_before_replacement_capture") is not True:
            raise ValueError("dynamic_plan_second_amendment_not_precommitted")
        if second.get("model_predictions_or_raw_errors_inspected") is not False:
            raise ValueError("dynamic_plan_second_amendment_result_leakage")
        if second.get("candidate_or_gate_change") is not False:
            raise ValueError("dynamic_plan_second_amendment_candidate_change")
        current = {row["session_id"]: row for row in plan["sessions"]}
        changes = second.get("session_changes") or {}
        unchanged = set(second.get("unchanged_session_ids") or [])
        if set(changes) | unchanged != set(current) or set(changes) & unchanged:
            raise ValueError("dynamic_plan_second_amendment_session_set_mismatch")
        effective_sessions = []
        for session in plan["sessions"]:
            if session["session_id"] in unchanged:
                effective_sessions.append(session)
                continue
            changed = dict(session)
            changed.update(changes[session["session_id"]])
            if changed.get("replacement_for") != session["session_id"]:
                raise ValueError("dynamic_plan_second_amendment_trace_mismatch")
            effective_sessions.append(changed)
        if len({row["session_id"] for row in effective_sessions}) != 8:
            raise ValueError("dynamic_plan_second_amendment_duplicate_session")
        plan = dict(plan)
        plan["sessions"] = effective_sessions
        plan["effective_amendment_v002"] = {
            "amendment_id": second["amendment_id"],
            "sha256": sha256_file(second_amendment_path),
            "reason": second["reason"],
        }
    third_amendment_path = output / "dynamic_replay_plan_amendment_v003.json"
    if third_amendment_path.exists():
        third = _json(third_amendment_path)
        if third.get("parent_plan_sha256") != sha256_file(plan_path):
            raise ValueError("dynamic_plan_third_amendment_parent_mismatch")
        if third.get("parent_amendment_sha256") != sha256_file(second_amendment_path):
            raise ValueError("dynamic_plan_third_amendment_chain_mismatch")
        if third.get("created_before_replacement_capture") is not True:
            raise ValueError("dynamic_plan_third_amendment_not_precommitted")
        if third.get("model_predictions_or_raw_errors_inspected") is not False:
            raise ValueError("dynamic_plan_third_amendment_result_leakage")
        if third.get("candidate_or_gate_change") is not False:
            raise ValueError("dynamic_plan_third_amendment_candidate_change")
        current = {row["session_id"]: row for row in plan["sessions"]}
        changes = third.get("session_changes") or {}
        unchanged = set(third.get("unchanged_session_ids") or [])
        if set(changes) | unchanged != set(current) or set(changes) & unchanged:
            raise ValueError("dynamic_plan_third_amendment_session_set_mismatch")
        effective_sessions = []
        for session in plan["sessions"]:
            if session["session_id"] in unchanged:
                effective_sessions.append(session)
                continue
            changed = dict(session)
            changed.update(changes[session["session_id"]])
            if changed.get("replacement_for") != session["session_id"]:
                raise ValueError("dynamic_plan_third_amendment_trace_mismatch")
            effective_sessions.append(changed)
        if len({row["session_id"] for row in effective_sessions}) != 8:
            raise ValueError("dynamic_plan_third_amendment_duplicate_session")
        plan = dict(plan)
        plan["sessions"] = effective_sessions
        plan["effective_amendment_v003"] = {
            "amendment_id": third["amendment_id"],
            "sha256": sha256_file(third_amendment_path),
            "reason": third["reason"],
        }
    fourth_amendment_path = output / "dynamic_replay_plan_amendment_v004.json"
    if fourth_amendment_path.exists():
        fourth = _json(fourth_amendment_path)
        if fourth.get("parent_plan_sha256") != sha256_file(plan_path):
            raise ValueError("dynamic_plan_fourth_amendment_parent_mismatch")
        if fourth.get("parent_amendment_sha256") != sha256_file(third_amendment_path):
            raise ValueError("dynamic_plan_fourth_amendment_chain_mismatch")
        if fourth.get("created_before_replacement_capture") is not True:
            raise ValueError("dynamic_plan_fourth_amendment_not_precommitted")
        if fourth.get("model_predictions_or_raw_errors_inspected") is not False:
            raise ValueError("dynamic_plan_fourth_amendment_result_leakage")
        if fourth.get("candidate_or_gate_change") is not False:
            raise ValueError("dynamic_plan_fourth_amendment_candidate_change")
        current = {row["session_id"]: row for row in plan["sessions"]}
        changes = fourth.get("session_changes") or {}
        unchanged = set(fourth.get("unchanged_session_ids") or [])
        if set(changes) | unchanged != set(current) or set(changes) & unchanged:
            raise ValueError("dynamic_plan_fourth_amendment_session_set_mismatch")
        effective_sessions = []
        for session in plan["sessions"]:
            if session["session_id"] in unchanged:
                effective_sessions.append(session)
                continue
            changed = dict(session)
            changed.update(changes[session["session_id"]])
            if changed.get("replacement_for") != session["session_id"]:
                raise ValueError("dynamic_plan_fourth_amendment_trace_mismatch")
            effective_sessions.append(changed)
        if len({row["session_id"] for row in effective_sessions}) != 8:
            raise ValueError("dynamic_plan_fourth_amendment_duplicate_session")
        plan = dict(plan)
        plan["sessions"] = effective_sessions
        plan["effective_amendment_v004"] = {
            "amendment_id": fourth["amendment_id"],
            "sha256": sha256_file(fourth_amendment_path),
            "reason": fourth["reason"],
        }
    fifth_amendment_path = output / "dynamic_replay_plan_amendment_v005.json"
    if fifth_amendment_path.exists():
        fifth = _json(fifth_amendment_path)
        if fifth.get("parent_plan_sha256") != sha256_file(plan_path):
            raise ValueError("dynamic_plan_fifth_amendment_parent_mismatch")
        if fifth.get("parent_amendment_sha256") != sha256_file(fourth_amendment_path):
            raise ValueError("dynamic_plan_fifth_amendment_chain_mismatch")
        if fifth.get("created_before_replacement_capture") is not True:
            raise ValueError("dynamic_plan_fifth_amendment_not_precommitted")
        if fifth.get("model_predictions_or_raw_errors_inspected") is not False:
            raise ValueError("dynamic_plan_fifth_amendment_result_leakage")
        if fifth.get("candidate_or_gate_change") is not False:
            raise ValueError("dynamic_plan_fifth_amendment_candidate_change")
        current = {row["session_id"]: row for row in plan["sessions"]}
        changes = fifth.get("session_changes") or {}
        unchanged = set(fifth.get("unchanged_session_ids") or [])
        if set(changes) | unchanged != set(current) or set(changes) & unchanged:
            raise ValueError("dynamic_plan_fifth_amendment_session_set_mismatch")
        effective_sessions = []
        for session in plan["sessions"]:
            if session["session_id"] in unchanged:
                effective_sessions.append(session)
                continue
            changed = dict(session)
            changed.update(changes[session["session_id"]])
            if changed.get("replacement_for") != session["session_id"]:
                raise ValueError("dynamic_plan_fifth_amendment_trace_mismatch")
            effective_sessions.append(changed)
        if len({row["session_id"] for row in effective_sessions}) != 8:
            raise ValueError("dynamic_plan_fifth_amendment_duplicate_session")
        plan = dict(plan)
        plan["sessions"] = effective_sessions
        plan["effective_amendment_v005"] = {
            "amendment_id": fifth["amendment_id"],
            "sha256": sha256_file(fifth_amendment_path),
            "reason": fifth["reason"],
        }
    sixth_amendment_path = output / "dynamic_replay_plan_amendment_v006.json"
    if sixth_amendment_path.exists():
        sixth = _json(sixth_amendment_path)
        if sixth.get("parent_plan_sha256") != sha256_file(plan_path):
            raise ValueError("dynamic_plan_sixth_amendment_parent_mismatch")
        if sixth.get("parent_amendment_sha256") != sha256_file(fifth_amendment_path):
            raise ValueError("dynamic_plan_sixth_amendment_chain_mismatch")
        if sixth.get("created_before_replacement_capture") is not True:
            raise ValueError("dynamic_plan_sixth_amendment_not_precommitted")
        if sixth.get("model_predictions_or_raw_errors_inspected") is not False:
            raise ValueError("dynamic_plan_sixth_amendment_result_leakage")
        if sixth.get("candidate_or_gate_change") is not False:
            raise ValueError("dynamic_plan_sixth_amendment_candidate_change")
        current = {row["session_id"]: row for row in plan["sessions"]}
        changes = sixth.get("session_changes") or {}
        unchanged = set(sixth.get("unchanged_session_ids") or [])
        if set(changes) | unchanged != set(current) or set(changes) & unchanged:
            raise ValueError("dynamic_plan_sixth_amendment_session_set_mismatch")
        effective_sessions = []
        for session in plan["sessions"]:
            if session["session_id"] in unchanged:
                effective_sessions.append(session)
                continue
            changed = dict(session)
            changed.update(changes[session["session_id"]])
            if changed.get("replacement_for") != session["session_id"]:
                raise ValueError("dynamic_plan_sixth_amendment_trace_mismatch")
            effective_sessions.append(changed)
        if len({row["session_id"] for row in effective_sessions}) != 8:
            raise ValueError("dynamic_plan_sixth_amendment_duplicate_session")
        plan = dict(plan)
        plan["sessions"] = effective_sessions
        plan["effective_amendment_v006"] = {
            "amendment_id": sixth["amendment_id"],
            "sha256": sha256_file(sixth_amendment_path),
            "reason": sixth["reason"],
        }
    return plan, candidate


class FrozenDirectEnsemble:
    def __init__(self, workspace: Path, candidate: Mapping[str, Any]) -> None:
        import xgboost as xgb

        self.xgb = xgb
        self.members: list[tuple[Any, FoldPreprocessor]] = []
        frozen = candidate["frozen"]
        for model_spec, preprocessing_spec in zip(frozen["models"], frozen["preprocessing"], strict=True):
            model_path = workspace / model_spec["path"]
            preprocessing_path = workspace / preprocessing_spec["path"]
            if sha256_file(model_path) != model_spec["sha256"] or sha256_file(preprocessing_path) != preprocessing_spec["sha256"]:
                raise ValueError("candidate_member_checksum_mismatch")
            booster = xgb.Booster()
            booster.load_model(str(model_path))
            preprocessing = _json(preprocessing_path)
            medians = np.asarray([preprocessing["medians"][name] for name in FEATURE_NAMES], dtype=np.float64)
            self.members.append((booster, FoldPreprocessor(medians)))
        self.checksum = str(frozen["candidate_checksum"])

    def predict(self, feature_rows: Sequence[Mapping[str, Any]]) -> tuple[np.ndarray, np.ndarray]:
        member_predictions = []
        for booster, preprocessing in self.members:
            matrix = preprocessing.transform(feature_rows)
            member_predictions.append(np.asarray(booster.predict(self.xgb.DMatrix(matrix)), dtype=np.float64))
        members = np.vstack(member_predictions)
        ensemble = np.mean(members, axis=0)
        return np.clip(ensemble, *CLIP_BOUNDS), np.std(members, axis=0)


def causal_smooth(timestamps: Sequence[float], values: Sequence[float], tau_s: float = 0.20) -> np.ndarray:
    if len(timestamps) != len(values) or not values:
        raise ValueError("causal_smoothing_input_invalid")
    output = np.empty(len(values), dtype=np.float64)
    output[0] = float(values[0])
    for index in range(1, len(values)):
        dt = float(timestamps[index]) - float(timestamps[index - 1])
        if not math.isfinite(dt) or dt <= 0.0:
            raise ValueError("causal_smoothing_timestamp_invalid")
        alpha = 1.0 - math.exp(-dt / float(tau_s))
        output[index] = output[index - 1] + alpha * (float(values[index]) - output[index - 1])
    return output


def perturb_bbox_features(features: Mapping[str, Any], variant: str) -> dict[str, Any]:
    if variant not in STRESS_VARIANTS:
        raise ValueError(f"bbox_variant_invalid:{variant}")
    result = dict(features)
    cx = float(result["bbox_center_x_fraction"])
    cy = float(result["bbox_center_y_fraction"])
    width = float(result["bbox_width_fraction"])
    height = float(result["bbox_height_fraction"])
    if variant.startswith("scale_"):
        _, sign, magnitude = variant.split("_")
        factor = 1.0 + (1.0 if sign == "plus" else -1.0) * float(magnitude) / 100.0
        width *= factor
        height *= factor
    elif variant.startswith("center_"):
        _, axis, sign, magnitude = variant.split("_")
        delta = (1.0 if sign == "plus" else -1.0) * float(magnitude) / 100.0
        if axis == "x": cx += delta
        else: cy += delta
    elif variant.startswith("crop_"):
        _, side, magnitude = variant.split("_")
        fraction = float(magnitude) / 100.0
        if side == "left":
            cx += 0.5 * fraction * width
            width *= 1.0 - fraction
        elif side == "right":
            cx -= 0.5 * fraction * width
            width *= 1.0 - fraction
        elif side == "top":
            cy += 0.5 * fraction * height
            height *= 1.0 - fraction
        else:
            cy -= 0.5 * fraction * height
            height *= 1.0 - fraction
    result["bbox_center_x_fraction"] = min(1.0, max(0.0, cx))
    result["bbox_center_y_fraction"] = min(1.0, max(0.0, cy))
    result["bbox_width_fraction"] = width
    result["bbox_height_fraction"] = height
    result["bbox_area_fraction"] = width * height
    result["bbox_aspect_ratio"] = width / height
    return result


def _load_dynamic_rows(workspace: Path, output: Path, plan: Mapping[str, Any]) -> tuple[list[dict[str, Any]], dict[str, Any]]:
    session_manifest = _json(output / "dynamic_session_manifest.json")
    accepted = session_manifest.get("accepted_sessions") or []
    if len(accepted) != len(plan["sessions"]) or session_manifest.get("failed_sessions"):
        raise ValueError("dynamic_sessions_incomplete")
    planned = {row["session_id"]: row for row in plan["sessions"]}
    all_rows: list[dict[str, Any]] = []
    traces: set[str] = set()
    groups: set[str] = set()
    for source in accepted:
        session_id = source["session_id"]
        if session_id not in planned:
            raise ValueError(f"unplanned_dynamic_session:{session_id}")
        root = workspace / source["source_root"]
        sidecar = root / "physical_diagnostics.jsonl"
        if sha256_file(sidecar) != source["source_sidecar_sha256"]:
            raise ValueError(f"dynamic_sidecar_checksum_mismatch:{session_id}")
        loaded, malformed = load_jsonl(sidecar)
        if malformed:
            raise ValueError(f"dynamic_sidecar_malformed:{session_id}")
        runtime_session_id = int(source["runtime_session_id"])
        rows = [row for row in loaded if row.get("stage") == "raw_range_computed" and int(row.get("session_id", -1)) == runtime_session_id]
        if len(rows) < int(planned[session_id]["minimum_raw_frames"]):
            raise ValueError(f"dynamic_frame_count_failed:{session_id}")
        identities = {(str(row.get("run_id")), int(row.get("session_id")), str(row.get("group_id"))) for row in rows}
        if len(identities) != 1 or session_id in groups:
            raise ValueError(f"dynamic_group_independence_failed:{session_id}")
        groups.add(session_id)
        previous_timestamp = -math.inf
        for row in rows:
            record_ok, trace_ok = verify_record(row)
            timestamp_ok, _ = validate_timestamp_stages(row.get("timestamp_stages") or {})
            if not record_ok or not trace_ok or not timestamp_ok or row.get("ground_truth_trace_valid") is not True:
                raise ValueError(f"dynamic_integrity_failed:{session_id}")
            if len(((row.get("anchors") or {}).get("per_grid_point") or [])) != 96:
                raise ValueError(f"dynamic_anchor_count_failed:{session_id}")
            trace = str(row["trace_identity_sha256"])
            if trace in traces:
                raise ValueError(f"dynamic_duplicate_trace:{trace}")
            traces.add(trace)
            timestamp = float(row["measurement_timestamp_s"])
            if timestamp <= previous_timestamp:
                raise ValueError(f"dynamic_measurement_timestamp_nonmonotonic:{session_id}")
            previous_timestamp = timestamp
            features = extract_features(row)
            feature_checksum = canonical_sha256([features[name] for name in FEATURE_NAMES])
            gt = float(row["ground_truth"]["distance_m"])
            all_rows.append({
                "session_id": session_id,
                "group_id": session_id,
                "scenario_type": planned[session_id]["scenario_type"],
                "context": planned[session_id]["context"],
                "run_id": str(row["run_id"]),
                "runtime_session_id": int(row["session_id"]),
                "runtime_group_id": str(row["group_id"]),
                "frame_index": int(row["frame_index"]),
                "measurement_timestamp_s": timestamp,
                "ground_truth_timestamp_s": float(row["ground_truth"]["timestamp_s"]),
                "ground_truth_range_m": gt,
                "raw_physical_range_m": float(row["raw_range"]["physics_slant_range_m"]),
                "trace_identity_sha256": trace,
                "record_sha256": str(row["record_sha256"]),
                "feature_vector_checksum": feature_checksum,
                "features": features,
                "bbox_original": {
                    "center_x_fraction": features["bbox_center_x_fraction"],
                    "center_y_fraction": features["bbox_center_y_fraction"],
                    "width_fraction": features["bbox_width_fraction"],
                    "height_fraction": features["bbox_height_fraction"],
                },
            })
    all_rows.sort(key=lambda row: (row["session_id"], row["measurement_timestamp_s"]))
    return all_rows, session_manifest


def alignment_lag_s(timestamps: Sequence[float], truth: Sequence[float], prediction: Sequence[float], maximum_s: float = 2.0, step_s: float = 0.05) -> float:
    t = np.asarray(timestamps, dtype=np.float64)
    gt = np.asarray(truth, dtype=np.float64)
    pred = np.asarray(prediction, dtype=np.float64)
    candidates = np.arange(-maximum_s, maximum_s + 0.5 * step_s, step_s)
    scored = []
    for lag in candidates:
        shifted = t - lag
        mask = (shifted >= t[0]) & (shifted <= t[-1])
        if int(np.count_nonzero(mask)) < max(4, len(t) // 2):
            continue
        aligned = np.interp(shifted[mask], t, gt)
        scored.append((float(np.mean(np.abs(pred[mask] - aligned))), abs(float(lag)), float(lag)))
    if not scored:
        return float("nan")
    return min(scored)[2]


def cross_correlation_lag_s(timestamps: Sequence[float], truth: Sequence[float], prediction: Sequence[float], maximum_s: float = 2.0) -> float | None:
    t = np.asarray(timestamps, dtype=np.float64)
    if len(t) < 6:
        return None
    dt = float(np.median(np.diff(t)))
    if not math.isfinite(dt) or dt <= 0.0:
        return None
    grid = np.arange(t[0], t[-1] + 0.5 * dt, dt)
    gt_rate = np.gradient(np.interp(grid, t, truth), dt)
    pred_rate = np.gradient(np.interp(grid, t, prediction), dt)
    maximum_steps = max(1, int(round(maximum_s / dt)))
    scores = []
    for shift in range(-maximum_steps, maximum_steps + 1):
        if shift > 0:
            left, right = pred_rate[shift:], gt_rate[:-shift]
        elif shift < 0:
            left, right = pred_rate[:shift], gt_rate[-shift:]
        else:
            left, right = pred_rate, gt_rate
        if len(left) < 4 or np.std(left) <= 1e-12 or np.std(right) <= 1e-12:
            continue
        scores.append((-float(np.corrcoef(left, right)[0, 1]), abs(shift), shift * dt))
    return None if not scores else float(min(scores)[2])


def _directions(rate: np.ndarray) -> np.ndarray:
    return np.where(rate < -DIRECTION_DEADBAND_M_S, -1, np.where(rate > DIRECTION_DEADBAND_M_S, 1, 0))


def maximum_consecutive_bad_duration(timestamps: np.ndarray, absolute_error: np.ndarray, threshold_m: float = 2.0) -> float:
    longest = 0.0
    start: int | None = None
    for index, bad in enumerate(absolute_error > threshold_m):
        if bad and start is None:
            start = index
        if (not bad or index == len(absolute_error) - 1) and start is not None:
            end = index if bad and index == len(absolute_error) - 1 else index - 1
            duration = float(timestamps[end] - timestamps[start]) if end > start else 0.0
            longest = max(longest, duration)
            start = None
    return longest


def _group_metric(session_rows: Sequence[Mapping[str, Any]], prediction: np.ndarray, method: str) -> dict[str, Any]:
    truth = np.asarray([float(row["ground_truth_range_m"]) for row in session_rows])
    timestamps = np.asarray([float(row["measurement_timestamp_s"]) for row in session_rows])
    metrics = metric_values(truth, prediction)
    return {
        "session_id": session_rows[0]["session_id"],
        "group_id": session_rows[0]["group_id"],
        "scenario_type": session_rows[0]["scenario_type"],
        "context": session_rows[0]["context"],
        "method": method,
        "frame_count": len(session_rows),
        "duration_s": float(timestamps[-1] - timestamps[0]),
        **metrics,
        "prediction_std_m": float(np.std(prediction)),
        "p95_frame_delta_m": float(np.quantile(np.abs(np.diff(prediction)), 0.95)) if len(prediction) > 1 else 0.0,
        "maximum_span_m": float(np.max(prediction) - np.min(prediction)),
        "maximum_consecutive_bad_duration_s": maximum_consecutive_bad_duration(timestamps, np.abs(prediction - truth)),
    }


def _equal_group(group_rows: Sequence[Mapping[str, Any]]) -> dict[str, float]:
    keys = ("signed_bias_m", "median_absolute_error_m", "mae_m", "rmse_m", "p90_abs_error_m", "p95_abs_error_m", "mean_absolute_relative_error", "error_gt_1m_fraction", "error_gt_2m_fraction", "error_gt_3m_fraction")
    return {key: mean(float(row[key]) for row in group_rows) for key in keys}


def _direction_row(session_rows: Sequence[Mapping[str, Any]], prediction: np.ndarray, method: str) -> dict[str, Any]:
    timestamps = np.asarray([float(row["measurement_timestamp_s"]) for row in session_rows])
    truth = np.asarray([float(row["ground_truth_range_m"]) for row in session_rows])
    gt_rate = np.gradient(truth, timestamps)
    pred_rate = np.gradient(prediction, timestamps)
    gt_direction, pred_direction = _directions(gt_rate), _directions(pred_rate)
    return {
        "session_id": session_rows[0]["session_id"], "scenario_type": session_rows[0]["scenario_type"], "method": method,
        "alignment_lag_s": alignment_lag_s(timestamps, truth, prediction),
        "cross_correlation_lag_s": cross_correlation_lag_s(timestamps, truth, prediction),
        "range_rate_mae_m_s": float(np.mean(np.abs(pred_rate - gt_rate))),
        "direction_accuracy": float(np.mean(pred_direction == gt_direction)),
        "approaching_recall": float(np.mean(pred_direction[gt_direction == -1] == -1)) if np.any(gt_direction == -1) else None,
        "receding_recall": float(np.mean(pred_direction[gt_direction == 1] == 1)) if np.any(gt_direction == 1) else None,
        "stationary_recall": float(np.mean(pred_direction[gt_direction == 0] == 0)) if np.any(gt_direction == 0) else None,
        "gt_approaching_frames": int(np.count_nonzero(gt_direction == -1)),
        "gt_receding_frames": int(np.count_nonzero(gt_direction == 1)),
        "gt_stationary_frames": int(np.count_nonzero(gt_direction == 0)),
    }


def _stop_row(session_rows: Sequence[Mapping[str, Any]], prediction: np.ndarray, method: str) -> dict[str, Any] | None:
    if session_rows[0]["scenario_type"] != "stop_and_hold":
        return None
    timestamps = np.asarray([float(row["measurement_timestamp_s"]) for row in session_rows])
    truth = np.asarray([float(row["ground_truth_range_m"]) for row in session_rows])
    gt_rate = np.gradient(truth, timestamps)
    moving = np.where(np.abs(gt_rate) > DIRECTION_DEADBAND_M_S)[0]
    if len(moving) == 0 or int(moving[-1]) >= len(truth) - SETTLING_STABLE_FRAMES:
        return {
            "session_id": session_rows[0]["session_id"], "method": method,
            "status": "STOP_NOT_OBSERVED", "settling_time_s": None,
            "overshoot_m": None, "stationary_std_m": None,
        }
    stop_index = int(moving[-1]) + 1
    final_truth = float(np.median(truth[stop_index:]))
    settling_index: int | None = None
    within = np.abs(prediction - final_truth) <= SETTLING_ERROR_BAND_M
    for index in range(stop_index, len(prediction) - SETTLING_STABLE_FRAMES + 1):
        if bool(np.all(within[index:index + SETTLING_STABLE_FRAMES])):
            settling_index = index
            break
    settling = None if settling_index is None else float(timestamps[settling_index] - timestamps[stop_index])
    post = prediction[stop_index:]
    return {
        "session_id": session_rows[0]["session_id"], "method": method,
        "status": "PASS_OBSERVED" if settling is not None else "NEVER_SETTLED",
        "stop_timestamp_s": float(timestamps[stop_index]),
        "final_gt_range_m": final_truth,
        "settling_time_s": settling,
        "overshoot_m": float(np.max(np.abs(post - final_truth))),
        "stationary_std_m": float(np.std(post)),
        "post_stop_frame_count": len(post),
    }


def evaluate(workspace: Path, output: Path) -> dict[str, Any]:
    plan, candidate = verify_frozen_candidate(workspace, output)
    rows, session_manifest = _load_dynamic_rows(workspace, output, plan)
    ensemble = FrozenDirectEnsemble(workspace, candidate)
    direct, disagreement = ensemble.predict([row["features"] for row in rows])
    by_session = sorted({str(row["session_id"]) for row in rows})
    smoothed = np.empty(len(rows), dtype=np.float64)
    for session_id in by_session:
        indices = [index for index, row in enumerate(rows) if row["session_id"] == session_id]
        smoothed[indices] = causal_smooth(
            [float(rows[index]["measurement_timestamp_s"]) for index in indices],
            [float(direct[index]) for index in indices],
            float(plan["smoothing_policy"]["time_constant_s"]),
        )
    prediction_rows: list[dict[str, Any]] = []
    stress_predictions: dict[str, np.ndarray] = {"original": direct.copy()}
    for variant in STRESS_VARIANTS[1:]:
        features = [perturb_bbox_features(row["features"], variant) for row in rows]
        stress_predictions[variant] = ensemble.predict(features)[0]
    for index, row in enumerate(rows):
        for variant in STRESS_VARIANTS:
            variant_features = row["features"] if variant == "original" else perturb_bbox_features(row["features"], variant)
            perturbed_bbox = {
                "center_x_fraction": variant_features["bbox_center_x_fraction"],
                "center_y_fraction": variant_features["bbox_center_y_fraction"],
                "width_fraction": variant_features["bbox_width_fraction"],
                "height_fraction": variant_features["bbox_height_fraction"],
            }
            feature_checksum = canonical_sha256([variant_features[name] for name in FEATURE_NAMES])
            payload = {
                "run_id": row["run_id"], "session_id": row["session_id"], "group_id": row["group_id"],
                "runtime_session_id": row["runtime_session_id"], "runtime_group_id": row["runtime_group_id"],
                "frame_index": row["frame_index"], "measurement_timestamp_s": row["measurement_timestamp_s"],
                "ground_truth_timestamp_s": row["ground_truth_timestamp_s"], "ground_truth_range_m": row["ground_truth_range_m"],
                "raw_physical_range_m": row["raw_physical_range_m"],
                "direct_prediction_m": float(stress_predictions[variant][index]),
                "smoothed_prediction_m": float(smoothed[index]) if variant == "original" else None,
                "ensemble_disagreement_std_m": float(disagreement[index]) if variant == "original" else None,
                "bbox_variant": variant,
                "bbox_original_json": json.dumps(row["bbox_original"], sort_keys=True, separators=(",", ":")),
                "bbox_perturbed_json": json.dumps(perturbed_bbox, sort_keys=True, separators=(",", ":")),
                "feature_vector_checksum": feature_checksum,
                "model_checksum": ensemble.checksum,
                "trace_identity_sha256": row["trace_identity_sha256"],
                "record_sha256": row["record_sha256"],
            }
            payload["prediction_checksum"] = canonical_sha256(payload)
            prediction_rows.append(payload)
    group_metrics: list[dict[str, Any]] = []
    direction_metrics: list[dict[str, Any]] = []
    stop_metrics: list[dict[str, Any]] = []
    methods = {
        "raw_physical_range": np.asarray([float(row["raw_physical_range_m"]) for row in rows]),
        "direct_ensemble_unsmoothed": direct,
        "direct_ensemble_causal_smoothed": smoothed,
    }
    for session_id in by_session:
        indices = [index for index, row in enumerate(rows) if row["session_id"] == session_id]
        selected_rows = [rows[index] for index in indices]
        for method, values in methods.items():
            selected_prediction = values[indices]
            group_metrics.append(_group_metric(selected_rows, selected_prediction, method))
            direction_metrics.append(_direction_row(selected_rows, selected_prediction, method))
            stop = _stop_row(selected_rows, selected_prediction, method)
            if stop is not None: stop_metrics.append(stop)
    truth = np.asarray([float(row["ground_truth_range_m"]) for row in rows])
    base_error = np.abs(direct - truth)
    bbox_metrics: list[dict[str, Any]] = []
    sensitivity_rows: list[dict[str, Any]] = []
    for variant in STRESS_VARIANTS[1:]:
        prediction = stress_predictions[variant]
        shift = prediction - direct
        error = np.abs(prediction - truth)
        group_mae_degradations = []
        for session_id in by_session:
            indices = [index for index, row in enumerate(rows) if row["session_id"] == session_id]
            group_mae_degradations.append(float(np.mean(error[indices]) - np.mean(base_error[indices])))
        bbox_metrics.append({
            "bbox_variant": variant,
            "equal_group_mae_degradation_m": mean(group_mae_degradations),
            "frame_weighted_p90_increase_m": float(np.quantile(error, 0.90) - np.quantile(base_error, 0.90)),
            "median_absolute_prediction_shift_m": float(np.median(np.abs(shift))),
            "maximum_absolute_prediction_shift_m": float(np.max(np.abs(shift))),
            "prediction_shift_gt_0_5m_fraction": float(np.mean(np.abs(shift) > 0.5)),
            "prediction_shift_gt_1m_fraction": float(np.mean(np.abs(shift) > 1.0)),
            "catastrophic_error_gt_3m_fraction": float(np.mean(error > 3.0)),
        })
        if variant.startswith("scale_"):
            sign = 1.0 if "plus" in variant else -1.0
            magnitude = float(variant.rsplit("_", 1)[1]) / 100.0
            for index, row in enumerate(rows):
                delta_width = sign * magnitude * float(row["features"]["bbox_width_fraction"])
                sensitivity_rows.append({
                    "session_id": row["session_id"], "frame_index": row["frame_index"],
                    "trace_identity_sha256": row["trace_identity_sha256"], "bbox_variant": variant,
                    "delta_bbox_width_fraction": delta_width,
                    "delta_prediction_m": float(shift[index]),
                    "sensitivity_m_per_bbox_width_fraction": float(shift[index] / delta_width),
                })
    primary_groups = [row for row in group_metrics if row["method"] == plan["primary_gated_output"]]
    primary_aggregate = _equal_group(primary_groups)
    primary_direction = [row for row in direction_metrics if row["method"] == plan["primary_gated_output"]]
    primary_stops = [row for row in stop_metrics if row["method"] == plan["primary_gated_output"]]
    accuracy_checks = {
        "equal_group_mae": primary_aggregate["mae_m"] <= GATES["equal_group_dynamic_mae_m_max"],
        "equal_group_p90": primary_aggregate["p90_abs_error_m"] <= GATES["equal_group_dynamic_p90_m_max"],
        "worst_session_mae": max(float(row["mae_m"]) for row in primary_groups) <= GATES["worst_session_mae_m_max"],
        "error_gt_3m": primary_aggregate["error_gt_3m_fraction"] <= GATES["error_gt_3m_fraction_max"],
    }
    finite_lags = [abs(float(row["alignment_lag_s"])) for row in primary_direction if row["alignment_lag_s"] is not None and math.isfinite(float(row["alignment_lag_s"]))]
    stop_settling = [float(row["settling_time_s"]) for row in primary_stops if row.get("settling_time_s") is not None]
    stop_std = [float(row["stationary_std_m"]) for row in primary_stops if row.get("stationary_std_m") is not None]
    temporal_checks = {
        "median_absolute_lag": bool(finite_lags) and median(finite_lags) <= GATES["median_absolute_lag_s_max"],
        "all_stops_settled": len(stop_settling) == 2,
        "worst_stop_settling_time": len(stop_settling) == 2 and max(stop_settling) <= GATES["worst_stop_settling_time_s_max"],
        "worst_stop_stationary_std": len(stop_std) == 2 and max(stop_std) <= GATES["worst_stop_stationary_std_m_max"],
    }
    stress_by_name = {row["bbox_variant"]: row for row in bbox_metrics}
    pm5 = [stress_by_name["scale_minus_5"], stress_by_name["scale_plus_5"]]
    pm10 = [stress_by_name["scale_minus_10"], stress_by_name["scale_plus_10"]]
    bbox_checks = {
        "pm5_mae_degradation": max(float(row["equal_group_mae_degradation_m"]) for row in pm5) <= GATES["bbox_pm5_mae_degradation_m_max"],
        "pm5_median_prediction_shift": max(float(row["median_absolute_prediction_shift_m"]) for row in pm5) <= GATES["bbox_pm5_median_prediction_shift_m_max"],
        "pm10_no_catastrophic_error": max(float(row["catastrophic_error_gt_3m_fraction"]) for row in pm10) <= GATES["bbox_pm10_catastrophic_error_fraction_max"],
    }
    if not all(bbox_checks.values()):
        conclusion = "DIRECT_CANDIDATE_FAILS_BBOX_ROBUSTNESS"
    elif not all(temporal_checks.values()):
        conclusion = "DIRECT_CANDIDATE_FAILS_TEMPORAL_RESPONSE"
    elif not all(accuracy_checks.values()):
        conclusion = "DIRECT_CANDIDATE_FAILS_DYNAMIC_ACCURACY"
    else:
        conclusion = "DIRECT_CANDIDATE_DYNAMIC_REPLAY_PASS"
    failures = sorted(
        (
            {
                "session_id": row["session_id"], "method": row["method"],
                "mae_m": row["mae_m"], "p90_abs_error_m": row["p90_abs_error_m"],
                "maximum_consecutive_bad_duration_s": row["maximum_consecutive_bad_duration_s"],
                "reason": "primary_session_mae_exceeds_gate",
            }
            for row in primary_groups if float(row["mae_m"]) > GATES["worst_session_mae_m_max"]
        ),
        key=lambda row: float(row["mae_m"]), reverse=True,
    )
    _write_csv(output / "prediction_rows.csv", prediction_rows)
    _write_csv(output / "per_group_metrics.csv", group_metrics)
    _write_csv(output / "direction_metrics.csv", direction_metrics)
    _write_csv(output / "stop_response_metrics.csv", stop_metrics)
    _write_csv(output / "bbox_stress_metrics.csv", bbox_metrics)
    _write_csv(output / "bbox_sensitivity.csv", sensitivity_rows)
    _write_csv(output / "failure_cases.csv", failures)
    summary = {
        "gate_id": GATE_ID, "conclusion": conclusion,
        "candidate_checksum": ensemble.checksum,
        "counts": {"sessions": len(by_session), "base_frames": len(rows), "prediction_rows_including_stress": len(prediction_rows)},
        "primary_equal_group_metrics": primary_aggregate,
        "accuracy_checks": accuracy_checks, "temporal_checks": temporal_checks,
        "bbox_checks": bbox_checks,
        "median_absolute_lag_s": median(finite_lags) if finite_lags else None,
        "worst_stop_settling_time_s": max(stop_settling) if len(stop_settling) == 2 else None,
        "worst_stop_stationary_std_m": max(stop_std) if len(stop_std) == 2 else None,
        "scope_guards": plan["scope_guards"],
        "limitations": [
            "development sessions only; no final holdout or promotion claim",
            "target motion uses independent simulator set_pose and no model output",
            "bbox stress holds physical non-bbox features constant by design",
        ],
    }
    report = [
        "# CORE RANGE DIRECT DYNAMIC REPLAY", "", f"Conclusion: `{conclusion}`", "",
        "Frozen direct C_shallow three-fold mean ensemble; offline development replay only.", "",
        "## Primary gated output", "", "`direct_ensemble_causal_smoothed`", "",
        "## Equal-group dynamic metrics", "",
        "```json", json.dumps(primary_aggregate, indent=2, sort_keys=True), "```", "",
        "## Gate checks", "", "```json", json.dumps({"accuracy": accuracy_checks, "temporal": temporal_checks, "bbox": bbox_checks}, indent=2, sort_keys=True), "```", "",
        "No runtime integration, shadow, controller or PX4 effect was performed.", "",
    ]
    (output / "dynamic_replay_report.md").write_text("\n".join(report), encoding="utf-8")
    artifacts = [path for path in output.rglob("*") if path.is_file() and path.name != "dynamic_replay_manifest.json"]
    manifest = {
        **summary,
        "dynamic_replay_plan_sha256": sha256_file(output / "dynamic_replay_plan.json"),
        "candidate_model_manifest_sha256": sha256_file(output / "candidate_model_manifest.json"),
        "dynamic_session_manifest_sha256": sha256_file(output / "dynamic_session_manifest.json"),
        "evaluator_source_sha256": sha256_file(workspace / "core_range_direct_dynamic_replay.py"),
        "versions": {"python": os.sys.version.split()[0], "numpy": np.__version__, "xgboost": ensemble.xgb.__version__},
        "artifacts": {str(path.relative_to(output)): sha256_file(path) for path in sorted(artifacts)},
    }
    _write_json(output / "dynamic_replay_manifest.json", manifest)
    print(json.dumps(summary, indent=2, sort_keys=True))
    return manifest


def finalize_blocked(workspace: Path, output: Path) -> dict[str, Any]:
    plan, candidate = verify_frozen_candidate(workspace, output)
    session_manifest = _json(output / "dynamic_session_manifest.json")
    accepted = session_manifest.get("accepted_sessions") or []
    failed = session_manifest.get("failed_sessions") or []
    if session_manifest.get("collection_complete") is not False or not failed:
        raise ValueError("blocked_finalization_requires_incomplete_collection")
    accepted_ids = {str(row["session_id"]) for row in accepted}
    planned = {str(row["session_id"]): row for row in plan["sessions"]}
    if not accepted_ids <= set(planned):
        raise ValueError("blocked_accepted_session_not_in_effective_plan")
    accepted_scenario_counts = {
        scenario: sum(planned[session_id]["scenario_type"] == scenario for session_id in accepted_ids)
        for scenario in ("approaching", "receding", "stop_and_hold")
    }
    required_scenario_counts = {"approaching": 3, "receding": 3, "stop_and_hold": 2}
    missing_scenario_counts = {
        key: required_scenario_counts[key] - accepted_scenario_counts[key]
        for key in required_scenario_counts
    }
    source_integrity = {}
    for row in accepted:
        root = workspace / str(row["source_root"])
        checks = {
            "sidecar": sha256_file(root / "physical_diagnostics.jsonl") == row["source_sidecar_sha256"],
            "capture": sha256_file(root / "capture_events.jsonl") == row["source_capture_sha256"],
            "trajectory": sha256_file(root / "trajectory_events.jsonl") == row["source_trajectory_sha256"],
            "archive": sha256_file(root / "runtime_logs.tar.zst") == row["runtime_archive_sha256"],
        }
        if not all(checks.values()):
            raise ValueError(f"blocked_accepted_source_changed:{row['session_id']}")
        source_integrity[str(row["session_id"])] = checks

    _write_empty_csv(output / "prediction_rows.csv", (
        "run_id", "session_id", "group_id", "frame_index",
        "measurement_timestamp_s", "ground_truth_timestamp_s",
        "ground_truth_range_m", "raw_physical_range_m", "direct_prediction_m",
        "smoothed_prediction_m", "bbox_variant", "feature_vector_checksum",
        "model_checksum", "prediction_checksum", "trace_identity_sha256",
    ))
    _write_empty_csv(output / "per_group_metrics.csv", ("session_id", "method", "status", "mae_m", "p90_abs_error_m"))
    _write_empty_csv(output / "direction_metrics.csv", ("session_id", "method", "status", "alignment_lag_s", "range_rate_mae_m_s"))
    _write_empty_csv(output / "stop_response_metrics.csv", ("session_id", "method", "status", "settling_time_s", "overshoot_m", "stationary_std_m"))
    _write_empty_csv(output / "bbox_stress_metrics.csv", ("bbox_variant", "status", "equal_group_mae_degradation_m", "median_absolute_prediction_shift_m"))
    _write_empty_csv(output / "bbox_sensitivity.csv", ("session_id", "frame_index", "bbox_variant", "status", "sensitivity_m_per_bbox_width_fraction"))
    failure_rows = [
        {
            "session_id": row.get("session_id"),
            "stage": "dynamic_collection_integrity",
            "reason": row.get("reason"),
            "quarantine": row.get("quarantine"),
            "model_prediction_run": False,
        }
        for row in failed
    ]
    _write_csv(output / "failure_cases.csv", failure_rows)
    conclusion = "DYNAMIC_REPLAY_BLOCKED_BY_DATA_INTEGRITY"
    summary = {
        "gate_id": GATE_ID,
        "conclusion": conclusion,
        "candidate_checksum": candidate["frozen"]["candidate_checksum"],
        "counts": {
            "planned_sessions": len(plan["sessions"]),
            "accepted_sessions": len(accepted),
            "accepted_frames": sum(int(row["frame_count"]) for row in accepted),
            "prediction_rows": 0,
        },
        "accepted_scenario_counts": accepted_scenario_counts,
        "required_scenario_counts": required_scenario_counts,
        "missing_scenario_counts": missing_scenario_counts,
        "metrics_status": "N/A_INCOMPLETE_GROUP_COVERAGE",
        "accuracy_checks": "N/A",
        "temporal_checks": "N/A",
        "bbox_checks": "N/A",
        "source_integrity": source_integrity,
        "scope_guards": plan["scope_guards"],
        "model_loaded_for_prediction": False,
        "retraining_performed": False,
        "runtime_or_controller_integration": False,
        "blocker": {
            "failed_session": failed[-1],
            "repeated_failure_mode": "near-start lateral bbox produced zero accepted raw-range rows after center-preserving 80-percent ROI correction",
        },
    }
    report = [
        "# CORE RANGE DIRECT DYNAMIC REPLAY", "",
        f"Conclusion: `{conclusion}`", "",
        "The frozen direct C_shallow candidate was not replayed because the precommitted dynamic collection did not satisfy the minimum group-integrity contract.", "",
        "## Coverage at the blocker", "",
        f"- Accepted: {len(accepted)}/8 independent sessions, {summary['counts']['accepted_frames']} frames.",
        f"- Approaching: {accepted_scenario_counts['approaching']}/3.",
        f"- Receding: {accepted_scenario_counts['receding']}/3.",
        f"- Stop-and-hold: {accepted_scenario_counts['stop_and_hold']}/2.", "",
        "The lateral-left receding scenario produced zero accepted raw-range rows both before and after the precommitted center-preserving 80% ROI correction. The repeated reason was `inverse_depth_uncertainty_too_large`. All failed attempts remain quarantined.", "",
        "## Metrics", "",
        "Dynamic accuracy, temporal response and bbox robustness are `N/A`: evaluating four partial groups would violate the frozen gate and could create selection bias.", "",
        "No model prediction, retraining, runtime integration, shadow, Follow, OFFBOARD, arm, takeoff or controller action was performed. Residual correction remains default-off.", "",
    ]
    (output / "dynamic_replay_report.md").write_text("\n".join(report), encoding="utf-8")
    artifacts = [path for path in output.rglob("*") if path.is_file() and path.name != "dynamic_replay_manifest.json"]
    manifest = {
        **summary,
        "dynamic_replay_plan_sha256": sha256_file(output / "dynamic_replay_plan.json"),
        "candidate_model_manifest_sha256": sha256_file(output / "candidate_model_manifest.json"),
        "dynamic_session_manifest_sha256": sha256_file(output / "dynamic_session_manifest.json"),
        "evaluator_source_sha256": sha256_file(workspace / "core_range_direct_dynamic_replay.py"),
        "exact_command": ".venv/bin/python core_range_direct_dynamic_replay.py finalize-blocked --workspace /home/sup/swarm_dashboard --output artifacts/core_range_3_12m/direct_dynamic_replay",
        "artifacts": {str(path.relative_to(output)): sha256_file(path) for path in sorted(artifacts)},
    }
    _write_json(output / "dynamic_replay_manifest.json", manifest)
    print(json.dumps(summary, indent=2, sort_keys=True))
    return manifest


def main() -> int:
    parser = argparse.ArgumentParser()
    subparsers = parser.add_subparsers(dest="command", required=True)
    for command in ("prepare", "evaluate", "finalize-blocked"):
        child = subparsers.add_parser(command)
        child.add_argument("--workspace", required=True, type=Path)
        child.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    workspace, output = args.workspace.resolve(), args.output.resolve()
    if args.command == "prepare": prepare(workspace, output)
    elif args.command == "evaluate":
        evaluate(workspace, output)
    else:
        finalize_blocked(workspace, output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
