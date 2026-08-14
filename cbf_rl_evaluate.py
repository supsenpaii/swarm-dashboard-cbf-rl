#!/usr/bin/env python3
"""Offline acceptance gate for the frozen CBF-RL nominal policy."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

from cbf_rl_env import CbfRlEnvironment, DRONE_IDS
from cbf_rl_policy import ProximityCbfRlPolicy
from cbf_rl_train import OrbitScenario, TrainingScenario, evaluate


def evaluation_scenarios() -> tuple[TrainingScenario, ...]:
    return (
        TrainingScenario(
            "perpendicular_crossing",
            {"UAV-01": (-8.0, 0.0, 9.0), "UAV-02": (0.0, -8.0, 9.0)},
            {"UAV-01": (8.0, 0.0, 9.0), "UAV-02": (0.0, 8.0, 9.0)},
        ),
        TrainingScenario(
            "offset_swap",
            {"UAV-01": (-7.0, -1.0, 9.0), "UAV-02": (7.0, 1.0, 9.0)},
            {"UAV-01": (7.0, 1.0, 9.0), "UAV-02": (-7.0, -1.0, 9.0)},
        ),
        TrainingScenario(
            "unequal_crossing",
            {"UAV-01": (-8.0, -4.0, 9.0), "UAV-02": (-3.0, 7.0, 9.0)},
            {"UAV-01": (8.0, 4.0, 9.0), "UAV-02": (5.0, -7.0, 9.0)},
        ),
        TrainingScenario(
            "vertical_separation",
            {"UAV-01": (-6.0, 0.0, 8.0), "UAV-02": (6.0, 0.0, 13.0)},
            {"UAV-01": (6.0, 0.0, 8.0), "UAV-02": (-6.0, 0.0, 13.0)},
        ),
        OrbitScenario("unseen_same_orbit_phase", phase_waypoints=3),
    )


def active_trajectory_scenario(
    policy: ProximityCbfRlPolicy | None = None,
) -> TrainingScenario:
    if policy is not None and policy.minimum_separation_m >= 20.0:
        return TrainingScenario(
            "active_parallel_x500_20m",
            {"UAV-01": (-20.0, 0.0, 20.0), "UAV-02": (-20.0, 30.0, 20.0)},
            {"UAV-01": (20.0, 0.0, 20.0), "UAV-02": (20.0, 30.0, 20.0)},
            maximum_steps=700,
        )
    return TrainingScenario(
        "active_parallel_20m",
        {"UAV-01": (0.0, 0.0, 9.0), "UAV-02": (-5.0, 2.0, 9.0)},
        {"UAV-01": (20.0, 0.0, 9.0), "UAV-02": (15.0, 2.0, 9.0)},
        maximum_steps=700,
    )


def _one_step(
    environment: CbfRlEnvironment,
    policy: ProximityCbfRlPolicy,
) -> dict[str, dict[str, Any]]:
    observations = environment.observations()
    _, _, _, _, info = environment.step(
        {drone: policy.act(observations[drone]) for drone in DRONE_IDS}
    )
    return info


def _fault_result(info: dict[str, dict[str, Any]]) -> dict[str, Any]:
    return {
        "fail_closed": all(not info[drone]["cbf"]["active"] for drone in DRONE_IDS),
        "safe_velocity_zero": all(
            info[drone]["safe_velocity_enu_m_s"] == (0.0, 0.0, 0.0)
            for drone in DRONE_IDS
        ),
        "reason_by_drone": {
            drone: info[drone]["cbf"]["reason"] for drone in DRONE_IDS
        },
    }


def fault_evaluation(policy: ProximityCbfRlPolicy) -> dict[str, Any]:
    spawn = {"UAV-01": (0.0, 0.0, 9.0), "UAV-02": (0.0, 8.0, 9.0)}
    goals = {"UAV-01": (10.0, 0.0, 9.0), "UAV-02": (10.0, 8.0, 9.0)}
    covariance = (0.041, 0.041, 0.071)

    missing = CbfRlEnvironment(goals)
    missing.reset(
        spawn,
        covariance_by_drone={"UAV-01": covariance, "UAV-02": None},
    )
    stale = CbfRlEnvironment(goals)
    stale.reset(
        spawn,
        message_age_ms_by_drone={"UAV-01": 101.0, "UAV-02": 101.0},
    )
    invalid = CbfRlEnvironment(goals)
    invalid.reset(spawn, valid_by_drone={"UAV-01": True, "UAV-02": False})

    recovery = CbfRlEnvironment(goals)
    recovery.reset(spawn)
    recovery.covariances["UAV-02"] = None
    gap = _fault_result(_one_step(recovery, policy))
    recovery.covariances["UAV-02"] = recovery.config.default_position_covariance_m2
    recovered_info = _one_step(recovery, policy)
    recovered = all(recovered_info[drone]["cbf"]["active"] for drone in DRONE_IDS)

    return {
        "missing_covariance": _fault_result(_one_step(missing, policy)),
        "stale_state": _fault_result(_one_step(stale, policy)),
        "invalid_source": _fault_result(_one_step(invalid, policy)),
        "covariance_gap_then_recovery": {
            "gap": gap,
            "recovered": recovered,
            "recovery_steps": 1 if recovered else None,
        },
    }


def run_gate(model_path: str | Path) -> dict[str, Any]:
    policy = ProximityCbfRlPolicy.load(model_path)
    scenarios = evaluation_scenarios()
    learned_score, learned_runs = evaluate(policy, scenarios)
    baseline = ProximityCbfRlPolicy(
        goal_gain=policy.goal_gain,
        avoidance_gain=0.0,
        avoidance_radius_m=policy.avoidance_radius_m,
        avoidance_goal_taper_m=policy.avoidance_goal_taper_m,
    )
    baseline_score, baseline_runs = evaluate(baseline, scenarios)
    faults = fault_evaluation(policy)
    learned_successes = sum(run["success"] for run in learned_runs)
    baseline_successes = sum(run["success"] for run in baseline_runs)
    faults_pass = all(
        faults[name]["fail_closed"] and faults[name]["safe_velocity_zero"]
        for name in ("missing_covariance", "stale_state", "invalid_source")
    ) and bool(faults["covariance_gap_then_recovery"]["gap"]["fail_closed"])
    faults_pass = faults_pass and bool(
        faults["covariance_gap_then_recovery"]["recovered"]
    )
    passed = bool(
        learned_successes == len(scenarios)
        and all(run["minimum_distance_m"] >= 4.0 for run in learned_runs)
        and all(run["minimum_cbf_margin_m"] >= 0.0 for run in learned_runs)
        and all(run["hold_frames"] == 0 for run in learned_runs)
        and learned_successes > baseline_successes
        and faults_pass
    )
    return {
        "milestone": "CBF_RL_OFFLINE_EVALUATION",
        "verdict": "PASS" if passed else "FAIL",
        "model": str(model_path),
        "learned": {"score": learned_score, "scenario_results": learned_runs},
        "goal_only_baseline": {
            "score": baseline_score,
            "scenario_results": baseline_runs,
        },
        "faults": faults,
    }


def run_active_trajectory_gate(model_path: str | Path) -> dict[str, Any]:
    policy = ProximityCbfRlPolicy.load(model_path)
    scenario = active_trajectory_scenario(policy)
    _, runs = evaluate(
        policy,
        (scenario,),
        maximum_steps=800,
    )
    run = runs[0]
    passed = bool(
        run["success"]
        and run["minimum_distance_m"] >= policy.minimum_separation_m
        and run["minimum_cbf_margin_m"] >= 0.0
        and run["hold_frames"] == 0
    )
    return {
        "milestone": "CBF_RL_ACTIVE_TRAJECTORY_OFFLINE",
        "verdict": "PASS" if passed else "FAIL",
        "model": str(model_path),
        "maximum_steps": scenario.maximum_steps,
        "scenario_result": run,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", default="cbf_rl_policy_v1.json")
    parser.add_argument("--output")
    parser.add_argument("--active-trajectory", action="store_true")
    arguments = parser.parse_args()
    report = (
        run_active_trajectory_gate(arguments.model)
        if arguments.active_trajectory
        else run_gate(arguments.model)
    )
    encoded = json.dumps(report, indent=2, sort_keys=True) + "\n"
    if arguments.output:
        Path(arguments.output).write_text(encoded, encoding="utf-8")
    print(encoded, end="")
    return 0 if report["verdict"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
