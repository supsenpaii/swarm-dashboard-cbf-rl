#!/usr/bin/env python3
"""Seeded Cross-Entropy policy search for the offline CBF-RL environment."""

from __future__ import annotations

import argparse
import concurrent.futures
import json
import math
import os
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Sequence

import numpy as np

from cbf_rl_env import (
    CbfRlEnvConfig,
    CbfRlEnvironment,
    DRONE_IDS,
    x500_20m_cbf_config,
)
from cbf_rl_policy import ProximityCbfRlPolicy
from formation_controller import FormationConfig
from trajectory_controller import ClosedPolylineTrajectory, TrajectoryTrackingController


@dataclass(frozen=True)
class TrainingScenario:
    name: str
    spawn: dict[str, tuple[float, float, float]]
    goals: dict[str, tuple[float, float, float]]
    maximum_steps: int | None = None
    initial_velocity: dict[str, tuple[float, float, float]] | None = None
    message_age_ms: float = 0.0
    response_time_constant_s: float | None = None


@dataclass(frozen=True)
class OrbitScenario:
    name: str
    phase_waypoints: int
    radius_m: float = 10.0
    waypoint_count: int = 24
    speed_m_s: float = 1.0
    maximum_steps: int = 1000


Scenario = TrainingScenario | OrbitScenario


def x500_training_scenarios(
    maximum_velocity_m_s: float = 10.0,
    *,
    relative_braking_acceleration_m_s2: float = 6.0,
) -> tuple[Scenario, ...]:
    if maximum_velocity_m_s <= 0.0 or relative_braking_acceleration_m_s2 <= 0.0:
        raise ValueError("velocity and braking acceleration must be positive")
    angle = math.radians(15.0)
    relative_speed = 2.0 * maximum_velocity_m_s * math.sin(angle / 2.0)
    initial_distance = (
        20.0
        + 2.0
        + 0.10 * math.sqrt(2.0 * (0.041 + 0.041 + 0.071))
        + relative_speed * 0.75
        + relative_speed
        * relative_speed
        / (2.0 * relative_braking_acceleration_m_s2)
        + 15.0
    )
    time_to_conflict = initial_distance / relative_speed
    second_heading = (math.cos(angle), math.sin(angle), 0.0)
    shallow_spawn = {
        "UAV-01": (-maximum_velocity_m_s * time_to_conflict, 0.0, 20.0),
        "UAV-02": (
            -maximum_velocity_m_s * time_to_conflict * second_heading[0],
            -maximum_velocity_m_s * time_to_conflict * second_heading[1],
            20.0,
        ),
    }
    after_s = time_to_conflict + 15.0
    maximum_vehicle_acceleration_m_s2 = (
        relative_braking_acceleration_m_s2 / 2.0
    )
    orbit_speed_m_s = min(
        0.8 * maximum_velocity_m_s,
        0.85 * math.sqrt(maximum_vehicle_acceleration_m_s2 * 45.0),
    )

    def encounter(
        angle_deg: float, speed_m_s: float, tau_s: float, age_ms: float
    ) -> TrainingScenario:
        encounter_angle = math.radians(angle_deg)
        heading = (math.cos(encounter_angle), math.sin(encounter_angle), 0.0)
        closing = 2.0 * speed_m_s * math.sin(encounter_angle / 2.0)
        distance = (
            20.0
            + 2.0
            + 0.10 * math.sqrt(2.0 * (0.041 + 0.041 + 0.071))
            + closing * (0.65 + age_ms / 1000.0)
            + closing
            * closing
            / (2.0 * relative_braking_acceleration_m_s2)
            + 15.0
        )
        conflict_s = distance / closing
        end_s = conflict_s + 15.0
        return TrainingScenario(
            f"stress_{angle_deg:.0f}deg_{speed_m_s:.0f}ms_tau{tau_s:.2f}_age{age_ms:.0f}",
            {
                "UAV-01": (-speed_m_s * conflict_s, 0.0, 20.0),
                "UAV-02": (
                    -speed_m_s * conflict_s * heading[0],
                    -speed_m_s * conflict_s * heading[1],
                    20.0,
                ),
            },
            {
                "UAV-01": (speed_m_s * end_s, 0.0, 20.0),
                "UAV-02": (
                    speed_m_s * end_s * heading[0],
                    speed_m_s * end_s * heading[1],
                    20.0,
                ),
            },
            maximum_steps=1800,
            initial_velocity={
                "UAV-01": (speed_m_s, 0.0, 0.0),
                "UAV-02": (
                    speed_m_s * heading[0],
                    speed_m_s * heading[1],
                    0.0,
                ),
            },
            message_age_ms=age_ms,
            response_time_constant_s=tau_s,
        )
    return (
        TrainingScenario(
            "parallel",
            {"UAV-01": (-80.0, 0.0, 20.0), "UAV-02": (-80.0, 30.0, 20.0)},
            {"UAV-01": (80.0, 0.0, 20.0), "UAV-02": (80.0, 30.0, 20.0)},
            maximum_steps=2400,
        ),
        TrainingScenario(
            "perpendicular_crossing",
            {"UAV-01": (-70.0, 0.0, 20.0), "UAV-02": (0.0, -70.0, 20.0)},
            {"UAV-01": (70.0, 0.0, 20.0), "UAV-02": (0.0, 70.0, 20.0)},
            maximum_steps=2600,
        ),
        TrainingScenario(
            "diagonal_crossing",
            {"UAV-01": (-70.0, -35.0, 20.0), "UAV-02": (-70.0, 35.0, 20.0)},
            {"UAV-01": (70.0, 35.0, 20.0), "UAV-02": (70.0, -35.0, 20.0)},
            maximum_steps=2800,
        ),
        TrainingScenario(
            "head_on_swap",
            {"UAV-01": (-70.0, 0.0, 20.0), "UAV-02": (70.0, 0.0, 20.0)},
            {"UAV-01": (70.0, 0.0, 20.0), "UAV-02": (-70.0, 0.0, 20.0)},
            maximum_steps=3000,
        ),
        TrainingScenario(
            "stationary_intruder",
            {"UAV-01": (-90.0, 0.0, 20.0), "UAV-02": (0.0, 0.0, 20.0)},
            {"UAV-01": (90.0, 0.0, 20.0), "UAV-02": (0.0, 0.0, 20.0)},
            maximum_steps=3200,
        ),
        TrainingScenario(
            "vertical_swap",
            {"UAV-01": (0.0, 0.0, 5.0), "UAV-02": (0.0, 0.0, 75.0)},
            {"UAV-01": (0.0, 0.0, 75.0), "UAV-02": (0.0, 0.0, 5.0)},
            maximum_steps=2400,
        ),
        TrainingScenario(
            "shallow_15deg_full_speed_worst_lag",
            shallow_spawn,
            {
                "UAV-01": (maximum_velocity_m_s * after_s, 0.0, 20.0),
                "UAV-02": (
                    maximum_velocity_m_s * after_s * second_heading[0],
                    maximum_velocity_m_s * after_s * second_heading[1],
                    20.0,
                ),
            },
            maximum_steps=1600,
            initial_velocity={
                "UAV-01": (maximum_velocity_m_s, 0.0, 0.0),
                "UAV-02": (
                    maximum_velocity_m_s * second_heading[0],
                    maximum_velocity_m_s * second_heading[1],
                    0.0,
                ),
            },
            message_age_ms=100.0,
            response_time_constant_s=0.75,
        ),
        encounter(30.0, max(1.0, maximum_velocity_m_s - 1.0), 0.75, 100.0),
        encounter(45.0, max(1.0, maximum_velocity_m_s - 1.0), 0.75, 100.0),
        encounter(30.0, maximum_velocity_m_s, 0.45, 0.0),
        encounter(30.0, maximum_velocity_m_s, 0.75, 100.0),
        encounter(45.0, maximum_velocity_m_s, 0.45, 0.0),
        encounter(45.0, maximum_velocity_m_s, 0.75, 100.0),
        encounter(90.0, maximum_velocity_m_s, 0.75, 100.0),
        OrbitScenario(
            "same_orbit_close_phase",
            phase_waypoints=2,
            radius_m=45.0,
            speed_m_s=orbit_speed_m_s,
            maximum_steps=2400,
        ),
        OrbitScenario(
            "same_orbit_safe_phase",
            phase_waypoints=4,
            radius_m=45.0,
            speed_m_s=orbit_speed_m_s,
            maximum_steps=2400,
        ),
    )


def training_scenarios() -> tuple[Scenario, ...]:
    """Frozen legacy 4 m suite used to replay authenticated old policies."""
    return (
        TrainingScenario(
            "parallel",
            {"UAV-01": (0.0, 0.0, 9.0), "UAV-02": (0.0, 8.0, 9.0)},
            {"UAV-01": (10.0, 0.0, 9.0), "UAV-02": (10.0, 8.0, 9.0)},
        ),
        TrainingScenario(
            "active_parallel_20m",
            {"UAV-01": (0.0, 0.0, 9.0), "UAV-02": (-5.0, 2.0, 9.0)},
            {"UAV-01": (20.0, 0.0, 9.0), "UAV-02": (15.0, 2.0, 9.0)},
            maximum_steps=700,
        ),
        TrainingScenario(
            "diagonal_crossing",
            {"UAV-01": (-6.0, -3.0, 9.0), "UAV-02": (-6.0, 3.0, 9.0)},
            {"UAV-01": (6.0, 3.0, 9.0), "UAV-02": (6.0, -3.0, 9.0)},
        ),
        TrainingScenario(
            "head_on_swap",
            {"UAV-01": (-6.0, 0.0, 9.0), "UAV-02": (6.0, 0.0, 9.0)},
            {"UAV-01": (6.0, 0.0, 9.0), "UAV-02": (-6.0, 0.0, 9.0)},
        ),
        OrbitScenario(
            "same_orbit_close_phase",
            phase_waypoints=2,
            radius_m=10.0,
            speed_m_s=1.0,
            maximum_steps=1000,
        ),
        OrbitScenario(
            "same_orbit_safe_phase",
            phase_waypoints=4,
            radius_m=10.0,
            speed_m_s=1.0,
            maximum_steps=1000,
        ),
    )


def _policy(
    parameters: Sequence[float], *,
    maximum_velocity_m_s: float = 10.0,
    vehicle_profile: str = "x500",
    minimum_separation_m: float = 20.0,
) -> ProximityCbfRlPolicy:
    if len(parameters) != 6:
        raise ValueError("parameter vector shape is invalid")
    return ProximityCbfRlPolicy(
        goal_gain=max(0.1, min(20.0, float(parameters[0]))),
        avoidance_gain=max(0.0, min(30.0, float(parameters[1]))),
        # The avoidance radius has to sit outside the floor -- the policy's
        # proximity term divides by (radius - floor) -- so this lower clamp
        # follows the floor rather than the 20 m it used to assume.
        avoidance_radius_m=max(
            minimum_separation_m + 0.01, min(120.0, float(parameters[2]))
        ),
        avoidance_goal_taper_m=max(0.25, min(200.0, float(parameters[3]))),
        risk_slowdown_gain=max(0.0, min(2.0, float(parameters[4]))),
        closing_speed_scale_m_s=max(0.05, min(20.0, float(parameters[5]))),
        minimum_separation_m=minimum_separation_m,
        maximum_velocity_m_s=maximum_velocity_m_s,
        vehicle_profile=vehicle_profile,
    )


def initial_parameters() -> np.ndarray:
    # Start from the authenticated v2 policy, then let the mission scenarios
    # train every parameter instead of freezing the v1 avoidance shape.
    return np.array((3.0, 3.0, 90.0, 140.0, 1.0, 5.0))


def _environment_config(
    policy: ProximityCbfRlPolicy,
    maximum_steps: int,
    *,
    response_seed: int = 7,
    response_time_constant_s: float | None = None,
) -> CbfRlEnvConfig:
    if policy.avoids_vertically:
        return CbfRlEnvConfig(
            maximum_steps=maximum_steps,
            cbf=x500_20m_cbf_config(policy.maximum_velocity_m_s),
            response_time_constant_s=response_time_constant_s or 0.0,
            response_time_constant_range_s=(
                None
                if response_time_constant_s is not None
                else (0.45, 0.75)
            ),
            response_seed=response_seed,
            maximum_acceleration_m_s2=3.0,
        )
    return CbfRlEnvConfig(maximum_steps=maximum_steps)


def _orbit_rollout(
    policy: ProximityCbfRlPolicy,
    scenario: OrbitScenario,
    *,
    maximum_steps: int,
) -> dict[str, Any]:
    high_speed_contract = policy.avoids_vertically
    points = tuple(
        (
            scenario.radius_m * math.cos(2.0 * math.pi * index / scenario.waypoint_count),
            scenario.radius_m * math.sin(2.0 * math.pi * index / scenario.waypoint_count),
            20.0 if high_speed_contract else 9.0,
        )
        for index in range(scenario.waypoint_count)
    )
    trajectory = ClosedPolylineTrajectory(points, scenario.speed_m_s)
    spawn = {
        "UAV-01": points[0],
        "UAV-02": points[scenario.phase_waypoints],
    }
    steps = min(maximum_steps, scenario.maximum_steps)
    environment = CbfRlEnvironment(
        spawn,
        _environment_config(policy, steps),
    )
    environment.reset(spawn)
    controllers = {
        drone: TrajectoryTrackingController(
            drone,
            trajectory,
            (
                FormationConfig(maximum_velocity_m_s=policy.maximum_velocity_m_s)
                if high_speed_contract
                else FormationConfig()
            ),
            maximum_acceleration_m_s2=(
                3.0 if high_speed_contract else 0.5
            ),
            # Without this the corner branch never runs at all -- it needs both
            # an acceleration and a tolerance -- so the orbit was flown with no
            # curvature limiting whatsoever. Matches the runtime default in
            # companion_safety.
            corner_tracking_tolerance_m=1.0,
        )
        for drone in DRONE_IDS
    }
    lap_s = trajectory.lap_duration_s()
    previous_phase = {
        drone: trajectory.nearest_time_s(environment.positions[drone])
        for drone in DRONE_IDS
    }
    progress_m = {drone: 0.0 for drone in DRONE_IDS}
    cross_track_errors: list[float] = []
    minimum_distance = math.inf
    minimum_cbf_margin = math.inf
    holds = interventions = 0

    for step in range(steps):
        state = environment._swarm_state()
        commands = {
            drone: controllers[drone].command(
                step * environment.config.dt_s,
                state,
            )
            for drone in DRONE_IDS
        }
        targets = {drone: commands[drone].target_enu_m for drone in DRONE_IDS}
        environment.goals = {
            drone: targets[drone]  # type: ignore[dict-item]
            for drone in DRONE_IDS
        }
        observations = environment.observations()
        actions = {}
        for drone in DRONE_IDS:
            # The runtime caps the policy at the deterministic tracker's speed
            # for this frame (cbf_rl_shadow.py:174), which is what carries the
            # corner slowdown. Capping at a constant instead handed the policy
            # cruise speed through every corner and then scored it on the
            # cross-track that produced.
            tracker_speed_m_s = math.sqrt(
                sum(value * value for value in commands[drone].velocity_enu_m_s)
            )
            maximum_normalized_speed = min(
                scenario.speed_m_s, tracker_speed_m_s
            ) / environment.config.cbf.maximum_velocity_m_s
            action = policy.act(observations[drone])
            magnitude = math.sqrt(sum(component * component for component in action))
            actions[drone] = (
                tuple(
                    component * maximum_normalized_speed / magnitude
                    for component in action
                )
                if magnitude > maximum_normalized_speed
                else action
            )
        _, _, _, _, info = environment.step(actions)
        minimum_distance = min(
            minimum_distance,
            math.dist(environment.positions["UAV-01"], environment.positions["UAV-02"]),
        )
        minimum_cbf_margin = min(
            minimum_cbf_margin,
            *(float(info[drone]["cbf"]["minimum_margin_m"]) for drone in DRONE_IDS),
        )
        holds += sum(not info[drone]["cbf"]["active"] for drone in DRONE_IDS)
        interventions += sum(
            info[drone]["cbf"]["intervention_norm_m_s"] > 1.0e-6
            for drone in DRONE_IDS
        )
        for drone in DRONE_IDS:
            phase = trajectory.nearest_time_s(environment.positions[drone])
            delta = phase - previous_phase[drone]
            if delta < -lap_s / 2.0:
                delta += lap_s
            elif delta > lap_s / 2.0:
                delta -= lap_s
            progress_m[drone] += delta * scenario.speed_m_s
            previous_phase[drone] = phase
            nearest = trajectory.reference(phase).position_enu_m
            cross_track_errors.append(math.dist(environment.positions[drone], nearest))

    target_progress_m = steps * environment.config.dt_s * scenario.speed_m_s
    minimum_progress_m = min(progress_m.values())
    maximum_cross_track_error_m = max(cross_track_errors)
    success = bool(
        minimum_progress_m >= 0.80 * target_progress_m
        and minimum_distance >= environment.config.cbf.minimum_separation_m
        and minimum_cbf_margin >= 0.0
        and holds == 0
        and maximum_cross_track_error_m <= 1.0
    )
    score = (
        20.0 * sum(progress_m.values())
        - 20.0 * sum(cross_track_errors) / len(cross_track_errors)
        - 0.02 * interventions
        - 20.0 * holds
        - 10_000.0 * max(0.0, -minimum_cbf_margin)
        - (2_000.0 if not success else 0.0)
    )
    return {
        "scenario": scenario.name,
        "score": score,
        "success": success,
        "steps": steps,
        "minimum_distance_m": minimum_distance,
        "minimum_cbf_margin_m": minimum_cbf_margin,
        "hold_frames": holds,
        "intervention_frames": interventions,
        "progress_m": progress_m,
        "target_progress_m": target_progress_m,
        "mean_cross_track_error_m": sum(cross_track_errors) / len(cross_track_errors),
        "maximum_cross_track_error_m": maximum_cross_track_error_m,
    }


def rollout(
    policy: ProximityCbfRlPolicy,
    scenario: Scenario,
    *,
    maximum_steps: int = 800,
) -> dict[str, Any]:
    if isinstance(scenario, OrbitScenario):
        return _orbit_rollout(policy, scenario, maximum_steps=maximum_steps)
    environment = CbfRlEnvironment(
        scenario.goals,
        _environment_config(
            policy,
            min(maximum_steps, scenario.maximum_steps or maximum_steps),
            response_time_constant_s=scenario.response_time_constant_s,
        ),
    )
    observations = environment.reset(
        scenario.spawn,
        velocity_by_drone=scenario.initial_velocity,
        message_age_ms_by_drone={
            drone: scenario.message_age_ms for drone in DRONE_IDS
        },
    )
    total_reward = 0.0
    minimum_distance = math.inf
    minimum_cbf_margin = math.inf
    holds = interventions = 0
    terminated = truncated = False
    while not terminated and not truncated:
        actions = {drone: policy.act(observations[drone]) for drone in DRONE_IDS}
        observations, rewards, terminated, truncated, info = environment.step(actions)
        total_reward += sum(rewards.values())
        minimum_distance = min(
            minimum_distance,
            math.dist(environment.positions["UAV-01"], environment.positions["UAV-02"]),
        )
        holds += sum(not info[drone]["cbf"]["active"] for drone in DRONE_IDS)
        minimum_cbf_margin = min(
            minimum_cbf_margin,
            *(float(info[drone]["cbf"]["minimum_margin_m"]) for drone in DRONE_IDS),
        )
        interventions += sum(
            info[drone]["cbf"]["intervention_norm_m_s"] > 1.0e-6
            for drone in DRONE_IDS
        )
    final_goal_distance = {
        drone: math.dist(environment.positions[drone], scenario.goals[drone])
        for drone in DRONE_IDS
    }
    safe_success = bool(
        terminated
        and minimum_distance >= environment.config.cbf.minimum_separation_m
        and minimum_cbf_margin >= 0.0
        and holds == 0
    )
    if not safe_success:
        total_reward -= 500.0 + 25.0 * sum(final_goal_distance.values())
    total_reward -= 100.0 * max(
        0.0, environment.config.cbf.minimum_separation_m - minimum_distance
    )
    total_reward -= 10_000.0 * max(0.0, -minimum_cbf_margin)
    total_reward -= 20.0 * holds
    return {
        "scenario": scenario.name,
        "score": float(total_reward),
        "success": safe_success,
        "steps": environment.steps,
        "minimum_distance_m": minimum_distance,
        "minimum_cbf_margin_m": minimum_cbf_margin,
        "hold_frames": holds,
        "intervention_frames": interventions,
        "final_goal_distance_m": final_goal_distance,
    }


def evaluate(
    policy: ProximityCbfRlPolicy,
    scenarios: Sequence[Scenario],
    *,
    maximum_steps: int = 800,
) -> tuple[float, list[dict[str, Any]]]:
    runs = [rollout(policy, scenario, maximum_steps=maximum_steps) for scenario in scenarios]
    return sum(run["score"] for run in runs), runs


def _candidate_score(
    payload: tuple[np.ndarray, tuple[Scenario, ...], int, float, str, float],
) -> float:
    (
        parameters,
        scenarios,
        maximum_steps,
        maximum_velocity_m_s,
        vehicle_profile,
        minimum_separation_m,
    ) = payload
    score, _ = evaluate(
        _policy(parameters, maximum_velocity_m_s=maximum_velocity_m_s,
            vehicle_profile=vehicle_profile,
            minimum_separation_m=minimum_separation_m),
        scenarios,
        maximum_steps=maximum_steps,
    )
    return score


def train(
    *,
    seed: int = 7,
    generations: int = 12,
    population: int = 24,
    elite_count: int = 6,
    scenarios: Sequence[Scenario] | None = None,
    maximum_steps: int = 3200,
    workers: int = 1,
    maximum_velocity_m_s: float = 10.0,
    vehicle_profile: str = "x500",
    minimum_separation_m: float = 20.0,
) -> tuple[ProximityCbfRlPolicy, dict[str, Any]]:
    if (
        generations <= 0
        or population <= 1
        or not 1 <= elite_count < population
        or workers <= 0
        or not math.isfinite(maximum_velocity_m_s)
        or maximum_velocity_m_s <= 0.0
        or vehicle_profile != "x500"
    ):
        raise ValueError("training population configuration is invalid")
    selected_scenarios = tuple(
        scenarios
        or (
            x500_training_scenarios(maximum_velocity_m_s)
        )
    )
    if not selected_scenarios:
        raise ValueError("at least one training scenario is required")
    rng = np.random.default_rng(seed)
    mean = initial_parameters()
    deviation = np.array((1.0, 2.0, 15.0, 25.0, 0.30, 3.0))
    best_parameters = mean.copy()
    best_score, best_runs = evaluate(
        _policy(best_parameters, maximum_velocity_m_s=maximum_velocity_m_s,
            vehicle_profile=vehicle_profile,
            minimum_separation_m=minimum_separation_m),
        selected_scenarios,
        maximum_steps=maximum_steps,
    )
    history = []

    executor = (
        concurrent.futures.ProcessPoolExecutor(max_workers=workers)
        if workers > 1
        else None
    )
    try:
        for generation in range(generations):
            candidates = rng.normal(mean, deviation, size=(population, mean.size))
            candidates[0] = mean
            if executor is None:
                scores = np.asarray(
                    [
                        _candidate_score(
                            (
                                candidate,
                                selected_scenarios,
                                maximum_steps,
                                maximum_velocity_m_s,
                                vehicle_profile,
                                minimum_separation_m,
                            )
                        )
                        for candidate in candidates
                    ]
                )
            else:
                scores = np.fromiter(
                    executor.map(
                        _candidate_score,
                        (
                            (
                                candidate,
                                selected_scenarios,
                                maximum_steps,
                                maximum_velocity_m_s,
                                vehicle_profile,
                                minimum_separation_m,
                            )
                            for candidate in candidates
                        ),
                        chunksize=1,
                    ),
                    dtype=float,
                    count=population,
                )
            elite_indices = np.argsort(scores)[-elite_count:]
            elite = candidates[elite_indices]
            mean = elite.mean(axis=0)
            deviation = np.maximum(elite.std(axis=0), 0.03)
            generation_best = int(np.argmax(scores))
            if scores[generation_best] > best_score:
                best_score = float(scores[generation_best])
                best_parameters = candidates[generation_best].copy()
                _, best_runs = evaluate(
                    _policy(best_parameters, maximum_velocity_m_s=maximum_velocity_m_s,
            vehicle_profile=vehicle_profile,
            minimum_separation_m=minimum_separation_m),
                    selected_scenarios,
                    maximum_steps=maximum_steps,
                )
            history.append(
                {
                    "generation": generation,
                    "best_score": round(float(scores[generation_best]), 6),
                    "elite_mean_score": round(float(scores[elite_indices].mean()), 6),
                }
            )
    finally:
        if executor is not None:
            executor.shutdown()

    policy = _policy(best_parameters, maximum_velocity_m_s=maximum_velocity_m_s,
            vehicle_profile=vehicle_profile,
            minimum_separation_m=minimum_separation_m)
    speed_tag = f"{maximum_velocity_m_s:g}".replace(".", "P")
    report = {
        "milestone": f"CBF_RL_{vehicle_profile.upper()}_20M_{speed_tag}MS_OFFLINE_TRAINING",
        "algorithm": f"parallel_seeded_cross_entropy_{vehicle_profile}_20m_{speed_tag}ms_v1",
        "vehicle_profile": vehicle_profile,
        "seed": seed,
        "generations": generations,
        "population": population,
        "elite_count": elite_count,
        "workers": workers,
        "maximum_steps": maximum_steps,
        "maximum_velocity_m_s": maximum_velocity_m_s,
        "best_score": best_score,
        "all_training_scenarios_successful": all(run["success"] for run in best_runs),
        "scenario_results": best_runs,
        "history": history,
    }
    return policy, report


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seed", type=int, default=7)
    parser.add_argument("--generations", type=int, default=12)
    parser.add_argument("--population", type=int, default=24)
    parser.add_argument("--elite-count", type=int, default=6)
    parser.add_argument("--maximum-steps", type=int, default=3200)
    parser.add_argument("--maximum-velocity-m-s", type=float, default=10.0)
    parser.add_argument(
        "--vehicle-profile", choices=("x500",), default="x500"
    )
    parser.add_argument(
        "--minimum-separation-m",
        type=float,
        default=20.0,
        help="physical separation floor for the x500 high-speed contract",
    )
    parser.add_argument(
        "--workers", type=int, default=max(1, min(8, (os.cpu_count() or 2) - 2))
    )
    parser.add_argument("--output")
    arguments = parser.parse_args()
    policy, report = train(
        seed=arguments.seed,
        generations=arguments.generations,
        population=arguments.population,
        elite_count=arguments.elite_count,
        maximum_steps=arguments.maximum_steps,
        workers=arguments.workers,
        maximum_velocity_m_s=arguments.maximum_velocity_m_s,
        vehicle_profile=arguments.vehicle_profile,
        minimum_separation_m=arguments.minimum_separation_m,
    )
    speed_tag = f"{arguments.maximum_velocity_m_s:g}".replace(".", "p")
    floor_tag = f"{arguments.minimum_separation_m:g}".replace(".", "p")
    output = arguments.output or (
        f"cbf_rl_policy_{arguments.vehicle_profile}_{floor_tag}m_{speed_tag}ms_v1.json"
    )
    policy.save(output, training=report)
    print(json.dumps(report, indent=2))
    print(f"wrote {Path(output)}")
    return 0 if report["all_training_scenarios_successful"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
