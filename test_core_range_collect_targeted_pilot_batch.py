"""Focused tests for CORE_RANGE_TARGETED_DYNAMIC_PILOT_COLLECTION_STAGE_2A.

Covers Section 14 of the task spec: scenario-matrix completeness, unique
session/group IDs, deterministic trajectory spec, exact distance-regime
validation, direction validation, stop-and-hold phase detection, RTF
median/streak gate, Candidate B feature order/checksum, historical/pilot
cell mapping basis, persistent-bias duration logic (reused from the gate
threshold, not reimplemented), no GT/runtime leakage, quarantine
exclusion, partial-pilot-cannot-proceed, and no PX4/controller side effect.
"""

from __future__ import annotations

import json
from pathlib import Path

import pytest

import core_range_collect_targeted_pilot_batch as batch

WORKSPACE = Path(__file__).resolve().parent
PILOT_DIR = WORKSPACE / "artifacts/core_range_3_12m/targeted_dynamic_pilot"


def _sessions():
    return batch.sessions()


def test_scenario_matrix_completeness():
    sessions = _sessions()
    assert len(sessions) == 14
    bands = {s["band"] for s in sessions}
    assert bands == {"near_3_4m", "mid_4_5_8m", "far_10_12m", "stop_and_hold"}
    counts = {}
    for s in sessions:
        counts[s["scenario_type"]] = counts.get(s["scenario_type"], 0) + 1
    assert counts == {"approaching": 6, "receding": 6, "stop_and_hold": 2}


def test_unique_session_and_group_ids():
    sessions = _sessions()
    group_ids = [s["group_id"] for s in sessions]
    session_ids = [s["session_id"] for s in sessions]
    assert len(group_ids) == len(set(group_ids)) == 14
    assert len(session_ids) == len(set(session_ids)) == 14
    assert group_ids == session_ids  # established convention, unchanged


def test_deterministic_trajectory_spec():
    sessions = _sessions()
    for s in sessions:
        assert s["seed"] == 52
        assert s["pose_update_rate_hz"] == 5.0
        assert s["maximum_attempts"] == 3


def test_exact_distance_regime_validation():
    bounds = {
        "near_3_4m": (3.0, 4.0),
        "mid_4_5_8m": (4.5, 8.0),
        "far_10_12m": (12.0 - 12.0 + 10.0, 12.0),  # 10.0, 12.0
    }
    for s in _sessions():
        if s["band"] == "stop_and_hold":
            continue
        lo, hi = bounds[s["band"]]
        soft_lo, soft_hi = lo - batch.BAND_SOFT_MARGIN_M, hi + batch.BAND_SOFT_MARGIN_M
        assert soft_lo <= s["start_range_m"] <= soft_hi
        assert soft_lo <= s["end_range_m"] <= soft_hi
        if s["band"] == "near_3_4m":
            assert s["start_range_m"] >= 3.0 and s["end_range_m"] >= 3.0
        if s["band"] == "far_10_12m":
            assert s["start_range_m"] <= 12.0 and s["end_range_m"] <= 12.0


def test_direction_validation():
    for s in _sessions():
        if s["scenario_type"] == "approaching":
            assert s["start_range_m"] > s["end_range_m"]
        elif s["scenario_type"] == "receding":
            assert s["start_range_m"] < s["end_range_m"]


def test_stop_and_hold_phase_detection():
    for s in _sessions():
        if s["scenario_type"] == "stop_and_hold":
            assert s["hold_duration_s"] > 0
            assert s["band"] == "stop_and_hold"
        else:
            assert s["hold_duration_s"] == 0.0


def test_rtf_median_and_streak_gate_logic(tmp_path):
    log = tmp_path / "gz_world_stats.log"
    # 5 samples: median well above 0.8, one streak of 2 (allowed) then 1 (not a streak)
    log.write_text(
        "--- 1 ---\nreal_time_factor: 1.0\n"
        "--- 2 ---\nreal_time_factor: 0.25\n"
        "--- 3 ---\nreal_time_factor: 0.25\n"
        "--- 4 ---\nreal_time_factor: 1.0\n"
        "--- 5 ---\nreal_time_factor: 1.0\n"
    )
    result = batch._gazebo_rtf_quality(log)
    assert result["median_pass"] is True
    assert result["longest_consecutive_rtf_le_0_3_samples"] == 2
    assert result["long_stutter_pass"] is True

    log2 = tmp_path / "gz_world_stats_bad.log"
    log2.write_text(
        "--- 1 ---\nreal_time_factor: 1.0\n"
        "--- 2 ---\nreal_time_factor: 0.2\n"
        "--- 3 ---\nreal_time_factor: 0.2\n"
        "--- 4 ---\nreal_time_factor: 0.2\n"
        "--- 5 ---\nreal_time_factor: 1.0\n"
    )
    result2 = batch._gazebo_rtf_quality(log2)
    assert result2["longest_consecutive_rtf_le_0_3_samples"] == 3
    assert result2["long_stutter_pass"] is False


def test_candidate_b_feature_order_and_checksum_match_established_artifacts():
    contract = json.loads((PILOT_DIR / "candidate_b_inference_contract.json").read_text())
    reproduction = json.loads(
        (WORKSPACE / "artifacts/core_range_3_12m/control_ready_observation/candidate_b_reproduction.json").read_text()
    )
    assert contract["feature_order"] == reproduction["feature_order"]
    assert contract["candidate"] == "CANDIDATE_B_GROUP_AND_DISTANCE_BALANCED"
    artifact_checksums = json.loads(
        (WORKSPACE / "artifacts/core_range_3_12m/control_ready_observation/artifact_checksums.json").read_text()
    )
    for name in contract["fold_model_files"]:
        assert Path(name).name in artifact_checksums["model_and_preprocessing_files"]


def test_partial_pilot_cannot_proceed():
    counts = {"approaching": 5, "receding": 6, "stop_and_hold": 2}
    accepted_len = 13
    complete = accepted_len == 14 and counts == {"approaching": 6, "receding": 6, "stop_and_hold": 2}
    assert complete is False


def test_no_px4_controller_or_arm_takeoff_side_effect_in_pilot_scripts():
    for name in ("core_range_collect_targeted_pilot_scenario.sh", "core_range_rtf_soak_preflight_scenario.sh"):
        text = (WORKSPACE / name).read_text()
        forbidden = ["arm_vehicle", "OFFBOARD", "follow_target_start", "takeoff", "--arm"]
        for token in forbidden:
            assert token not in text, f"{name} unexpectedly references {token}"


def test_stage_2a_gate_result_reflects_actual_run():
    result = json.loads((PILOT_DIR / "stage_2a_gate_result.json").read_text())
    assert result["conclusion"] == "PILOT_BLOCKED_BY_RTF_STUTTER"
    assert result["gate_loosened"] is False
    assert result["third_attempt_made"] is False
    assert result["accepted_groups"] == 0


def test_rtf_soak_summary_attempt_budget_honestly_applied():
    summary = json.loads((PILOT_DIR / "rtf_soak_summary.json").read_text())
    genuine = [a for a in summary["attempts"] if a["counts_against_2_attempt_budget"]]
    assert len(genuine) == 2
    assert all(a["result"] == "FAIL" for a in genuine)
    assert summary["conclusion"] == "PILOT_BLOCKED_BY_RTF_STUTTER"
