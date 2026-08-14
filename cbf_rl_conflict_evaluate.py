"""Offline gate for the hybrid mission/RL/coordinator/CBF control path."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Any

from cbf_rl_env import CbfRlEnvConfig, CbfRlEnvironment, DRONE_IDS
from cbf_rl_policy import ProximityCbfRlPolicy
from conflict_coordinator import ConflictCoordinator
from formation_controller import FormationConfig
from trajectory_controller import ClosedPolylineTrajectory, TrajectoryTrackingController
from two_uav_mission_circle_flight import mission_waypoints_enu


SCENARIOS = (
    ("diagonal_cross", 0.8),
    ("head_on_swap", 0.6),
    ("opposite_orbit", 0.6),
    ("repeated_bow_tie", 0.7),
)
LAG_VALUES_S = (0.45, 0.55, 0.65, 0.70)
REQUIRED_MARGIN_M = 0.30
MINIMUM_PROGRESS_FRACTION = 0.45


def rollout(
    policy: ProximityCbfRlPolicy,
    scenario: str,
    speed_m_s: float,
    response_time_constant_s: float,
    *,
    maximum_steps: int = 1600,
) -> dict[str, Any]:
    points = mission_waypoints_enu(scenario)
    trajectories = {
        drone: ClosedPolylineTrajectory(points[drone], speed_m_s)
        for drone in DRONE_IDS
    }
    spawn = {drone: points[drone][0] for drone in DRONE_IDS}
    environment = CbfRlEnvironment(
        spawn,
        CbfRlEnvConfig(
            maximum_steps=maximum_steps,
            response_time_constant_s=response_time_constant_s,
        ),
    )
    environment.reset(spawn)
    controllers = {
        drone: TrajectoryTrackingController(
            drone,
            trajectories[drone],
            FormationConfig(),
            maximum_acceleration_m_s2=0.5,
        )
        for drone in DRONE_IDS
    }
    coordinators = ConflictCoordinator.pair(DRONE_IDS)
    previous_phase = {
        drone: trajectories[drone].nearest_time_s(spawn[drone])
        for drone in DRONE_IDS
    }
    lap_s = {drone: trajectories[drone].lap_duration_s() for drone in DRONE_IDS}
    progress_m = {drone: 0.0 for drone in DRONE_IDS}
    minimum_distance_m = math.inf
    minimum_margin_m = math.inf
    hold_frames = 0
    intervention_frames = 0
    maximum_cross_track_error_m = 0.0

    for step in range(maximum_steps):
        state = environment._swarm_state()
        mission = {
            drone: controllers[drone].command(step * environment.config.dt_s, state)
            for drone in DRONE_IDS
        }
        environment.goals = {
            drone: mission[drone].target_enu_m  # type: ignore[dict-item]
            for drone in DRONE_IDS
        }
        observations = environment.observations()
        actions = {}
        for drone in DRONE_IDS:
            action = policy.act(observations[drone])
            mission_velocity = mission[drone].velocity_enu_m_s
            maximum_action = math.sqrt(sum(value * value for value in mission_velocity))
            action_norm = math.sqrt(sum(value * value for value in action))
            if action_norm * 2.0 > maximum_action and action_norm > 0.0:
                action = tuple(
                    value * maximum_action / (2.0 * action_norm) for value in action
                )
            peer = next(other for other in DRONE_IDS if other != drone)
            coordinated, _ = coordinators[drone].filter(
                tuple(value * 2.0 for value in action),
                mission_velocity,
                state,
            )
            actions[drone] = tuple(value / 2.0 for value in coordinated)

        _, _, _, _, info = environment.step(actions)
        minimum_distance_m = min(
            minimum_distance_m,
            math.dist(environment.positions["UAV-01"], environment.positions["UAV-02"]),
        )
        minimum_margin_m = min(
            minimum_margin_m,
            *(float(info[drone]["cbf"]["minimum_margin_m"]) for drone in DRONE_IDS),
        )
        hold_frames += sum(not info[drone]["cbf"]["active"] for drone in DRONE_IDS)
        intervention_frames += sum(
            info[drone]["cbf"]["intervention_norm_m_s"] > 1.0e-6
            for drone in DRONE_IDS
        )
        for drone in DRONE_IDS:
            trajectory = trajectories[drone]
            phase = trajectory.nearest_time_s(environment.positions[drone])
            delta = phase - previous_phase[drone]
            if delta < -lap_s[drone] / 2.0:
                delta += lap_s[drone]
            elif delta > lap_s[drone] / 2.0:
                delta -= lap_s[drone]
            progress_m[drone] += delta * speed_m_s
            previous_phase[drone] = phase
            reference = trajectory.reference(phase).position_enu_m
            maximum_cross_track_error_m = max(
                maximum_cross_track_error_m,
                math.dist(environment.positions[drone], reference),
            )

    target_progress_m = maximum_steps * environment.config.dt_s * speed_m_s
    progress_fraction = min(progress_m.values()) / target_progress_m
    passed = bool(
        minimum_margin_m >= REQUIRED_MARGIN_M
        and hold_frames == 0
        and intervention_frames == 0
        and progress_fraction >= MINIMUM_PROGRESS_FRACTION
    )
    return {
        "scenario": scenario,
        "response_time_constant_s": response_time_constant_s,
        "verdict": "PASS" if passed else "FAIL",
        "minimum_distance_m": minimum_distance_m,
        "minimum_cbf_margin_m": minimum_margin_m,
        "hold_frames": hold_frames,
        "final_cbf_intervention_frames": intervention_frames,
        "progress_m": progress_m,
        "target_progress_m": target_progress_m,
        "minimum_progress_fraction": progress_fraction,
        "maximum_cross_track_error_m": maximum_cross_track_error_m,
    }


def run_gate(model_path: str | Path = "cbf_rl_policy_mission_v1.json") -> dict[str, Any]:
    policy = ProximityCbfRlPolicy.load(model_path)
    runs = [
        rollout(policy, scenario, speed, lag)
        for lag in LAG_VALUES_S
        for scenario, speed in SCENARIOS
    ]
    return {
        "milestone": "CBF_RL_HYBRID_CONFLICT_OFFLINE_GATE",
        "verdict": "PASS" if all(run["verdict"] == "PASS" for run in runs) else "FAIL",
        "model": str(model_path),
        "required_margin_m": REQUIRED_MARGIN_M,
        "minimum_progress_fraction": MINIMUM_PROGRESS_FRACTION,
        "runs": runs,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", default="cbf_rl_policy_mission_v1.json")
    arguments = parser.parse_args()
    report = run_gate(arguments.model)
    print(json.dumps(report, indent=2))
    return 0 if report["verdict"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
