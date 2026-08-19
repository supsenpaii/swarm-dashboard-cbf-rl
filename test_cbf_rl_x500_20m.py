from __future__ import annotations

import hashlib
from pathlib import Path

from cbf_command_gate import CbfCommandGate
from cbf_rl_env import x500_20m_cbf_config
from cbf_rl_policy import MODEL_FORMAT_X500_20M_V1, ProximityCbfRlPolicy
from cbf_rl_shadow import CbfRlShadow, _validate_active_contract
from cbf_rl_x500_safety_matrix import (
    _horizontal_case,
    _vertical_case,
    cases,
    evaluate_case,
)
from cbf_rl_train import OrbitScenario, x500_training_scenarios


MODEL = Path(__file__).with_name("cbf_rl_policy_x500_20m_v1.json")
MODEL_SHA256 = "83aef0cf98e3feb7e27853737221a8e751627864eedd8eacaf27939776d8ebc1"


def test_x500_policy_artifact_and_active_contract_are_pinned() -> None:
    assert hashlib.sha256(MODEL.read_bytes()).hexdigest() == MODEL_SHA256
    policy = ProximityCbfRlPolicy.load(MODEL)

    assert policy.as_dict()["format"] == MODEL_FORMAT_X500_20M_V1
    assert policy.vehicle_profile == "x500"
    assert policy.minimum_separation_m == 20.0
    assert policy.maximum_velocity_m_s == 10.0
    _validate_active_contract(
        CbfCommandGate("UAV-01", ("UAV-02",), x500_20m_cbf_config()),
        policy,
    )


def test_x500_active_runtime_accepts_only_the_validated_150ms_age(monkeypatch) -> None:
    monkeypatch.setenv("SWARM_CBF_RL_MODE", "active")
    monkeypatch.setenv("SWARM_CBF_RL_ACTIVE_ACK", CbfRlShadow.ACTIVE_ACK)
    monkeypatch.setenv("SWARM_CBF_RL_MODEL", str(MODEL))
    monkeypatch.setenv("SWARM_CBF_RL_MODEL_SHA256", MODEL_SHA256)
    monkeypatch.setenv("SWARM_PEER_STATE_MAX_AGE_MS", "150")

    active = CbfRlShadow.from_environment()

    assert active.policy is not None
    assert active.load_error == ""

    monkeypatch.setenv("SWARM_PEER_STATE_MAX_AGE_MS", "100")
    rejected = CbfRlShadow.from_environment()
    assert rejected.policy is None
    assert rejected.load_error == "active_peer_age_contract_mismatch"


def test_x500_matrix_covers_the_1_to_10ms_envelope() -> None:
    selected = cases()

    # 10 speeds x 3 lag variants x (13 angles + one vertical + one climbing).
    assert len(selected) == 450
    assert {case.speed_m_s for case in selected} == set(map(float, range(1, 11)))
    assert {case.peer_age_ms for case in selected} == {0.0, 100.0, 150.0}
    assert all(case.vehicle_profile == "x500" for case in selected)
    assert x500_20m_cbf_config().relative_braking_acceleration_m_s2 == 6.0
    assert x500_20m_cbf_config().design_margin_buffer_m == 5.0


def test_x500_matrix_can_extend_to_12ms_without_changing_the_10ms_default() -> None:
    selected = cases(12)

    assert len(selected) == 540
    assert {case.speed_m_s for case in selected} == set(map(float, range(1, 13)))
    assert x500_20m_cbf_config(12.0).maximum_velocity_m_s == 12.0


def test_x500_15ms_training_orbit_stays_inside_the_acceleration_envelope() -> None:
    orbits = [
        scenario
        for scenario in x500_training_scenarios(15.0)
        if isinstance(scenario, OrbitScenario)
    ]

    assert len(orbits) == 2
    assert all(scenario.speed_m_s < 10.0 for scenario in orbits)
    assert all(scenario.speed_m_s**2 / scenario.radius_m < 3.0 for scenario in orbits)


def test_x500_worst_10ms_cases_reach_goals_above_20m() -> None:
    policy = ProximityCbfRlPolicy.load(MODEL)
    selected = (
        _horizontal_case(10.0, 180.0, 0.75, 150.0),
        _vertical_case(10.0, 0.75, 150.0),
    )

    results = [evaluate_case((policy, case)) for case in selected]

    assert all(result["success"] for result in results)
    assert min(result["minimum_distance_m"] for result in results) >= 20.0
