from __future__ import annotations

import hashlib
from pathlib import Path

from cbf_command_gate import CbfCommandGate
from cbf_rl_env import sparrow_20m_cbf_config
from cbf_rl_policy import MODEL_FORMAT_SPARROW_20M_V1, ProximityCbfRlPolicy
from cbf_rl_shadow import CbfRlShadow, _validate_active_contract
from cbf_rl_sparrow_safety_matrix import (
    _horizontal_case,
    _vertical_case,
    cases,
    evaluate_case,
)
from cbf_rl_train import sparrow_training_scenarios


MODEL = Path(__file__).with_name("cbf_rl_policy_sparrow_20m_10ms_v1.json")
MODEL_SHA256 = "bb61f15e6832a0afad8ee6ded649c3210fbdf521a20a2056742c4c9ef509e1ef"


def test_sparrow_policy_artifact_and_active_contract_are_pinned() -> None:
    assert hashlib.sha256(MODEL.read_bytes()).hexdigest() == MODEL_SHA256
    policy = ProximityCbfRlPolicy.load(MODEL)

    assert policy.as_dict()["format"] == MODEL_FORMAT_SPARROW_20M_V1
    assert policy.vehicle_profile == "sparrow"
    assert policy.minimum_separation_m == 20.0
    assert policy.maximum_velocity_m_s == 10.0
    _validate_active_contract(
        CbfCommandGate("UAV-01", ("UAV-02",), sparrow_20m_cbf_config()),
        policy,
    )


def test_sparrow_matrix_covers_speed_angle_lag_and_vertical_envelope() -> None:
    selected = cases()

    assert len(selected) == 280
    assert {case.speed_m_s for case in selected} == set(map(float, range(1, 11)))
    assert {case.peer_age_ms for case in selected} == {0.0, 100.0}
    assert all(case.vehicle_profile == "sparrow" for case in selected)
    assert any(case.encounter_angle_deg is None for case in selected)
    assert sparrow_20m_cbf_config().relative_braking_acceleration_m_s2 == 8.0
    assert sparrow_20m_cbf_config().design_margin_buffer_m == 0.0


def test_sparrow_training_suite_uses_the_20m_high_speed_envelope() -> None:
    selected = sparrow_training_scenarios()

    assert len(selected) == 16
    assert any(scenario.name == "vertical_swap" for scenario in selected)
    assert any("worst_lag" in scenario.name for scenario in selected)


def test_worst_horizontal_and_vertical_cases_reach_goals_above_20m() -> None:
    if not MODEL.exists():
        return
    policy = ProximityCbfRlPolicy.load(MODEL)
    selected = (
        _horizontal_case(10.0, 180.0, 0.75, 100.0),
        _vertical_case(10.0, 0.75, 100.0),
    )

    results = [evaluate_case((policy, case)) for case in selected]

    assert all(result["success"] for result in results)
    assert min(result["minimum_distance_m"] for result in results) >= 20.0
