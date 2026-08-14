from cbf_rl_evaluate import fault_evaluation, run_active_trajectory_gate, run_gate
from cbf_rl_policy import ProximityCbfRlPolicy


MODEL = "cbf_rl_policy_v1.json"
MODEL_V2 = "cbf_rl_policy_v2.json"
MODEL_MISSION = "cbf_rl_policy_mission_v1.json"
MODEL_X500 = "cbf_rl_policy_x500_20m_v1.json"


def test_old_policy_fails_gate_when_any_cbf_margin_is_negative() -> None:
    report = run_gate(MODEL)

    assert report["verdict"] == "FAIL"
    assert any(
        run["minimum_cbf_margin_m"] < 0.0
        for run in report["learned"]["scenario_results"]
    )


def test_faults_fail_closed_and_covariance_recovers() -> None:
    faults = fault_evaluation(ProximityCbfRlPolicy.load(MODEL))

    for name in ("missing_covariance", "stale_state", "invalid_source"):
        assert faults[name]["fail_closed"]
        assert faults[name]["safe_velocity_zero"]
    recovery = faults["covariance_gap_then_recovery"]
    assert recovery["gap"]["fail_closed"]
    assert recovery["recovered"]
    assert recovery["recovery_steps"] == 1


def test_frozen_policy_fails_extended_active_trajectory_gate() -> None:
    report = run_active_trajectory_gate(MODEL)

    assert report["verdict"] == "FAIL"
    result = report["scenario_result"]
    assert not result["success"]
    assert result["minimum_cbf_margin_m"] < 0.0


def test_v2_policy_passes_offline_and_active_trajectory_gates() -> None:
    assert run_gate(MODEL_V2)["verdict"] == "FAIL"

    report = run_active_trajectory_gate(MODEL_V2)
    assert report["verdict"] == "PASS"
    result = report["scenario_result"]
    assert result["steps"] <= report["maximum_steps"]
    assert result["minimum_cbf_margin_m"] >= 0.0


def test_mission_policy_passes_point_fault_trajectory_and_orbit_gates() -> None:
    report = run_gate(MODEL_MISSION)
    assert report["verdict"] == "PASS"
    orbit = next(
        run
        for run in report["learned"]["scenario_results"]
        if run["scenario"] == "unseen_same_orbit_phase"
    )
    assert orbit["success"]
    assert orbit["minimum_cbf_margin_m"] >= 0.0

    assert run_active_trajectory_gate(MODEL_MISSION)["verdict"] == "PASS"


def test_x500_policy_uses_a_parallel_scenario_outside_its_20m_barrier() -> None:
    report = run_active_trajectory_gate(MODEL_X500)

    assert report["verdict"] == "PASS"
    result = report["scenario_result"]
    assert result["scenario"] == "active_parallel_x500_20m"
    assert result["minimum_distance_m"] >= 20.0
