#!/usr/bin/env python3
"""Deterministic 20 m safety gate over vehicle speeds, angles and lag bounds."""

from __future__ import annotations

import argparse
import concurrent.futures
import json
import math
import os
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any

from cbf_rl_env import CbfRlEnvConfig, CbfRlEnvironment, DRONE_IDS, x500_20m_cbf_config
from cbf_rl_policy import ProximityCbfRlPolicy


Vector3 = tuple[float, float, float]


@dataclass(frozen=True)
class SafetyCase:
    name: str
    speed_m_s: float
    encounter_angle_deg: float | None
    response_time_constant_s: float
    peer_age_ms: float
    spawn: dict[str, Vector3]
    velocity: dict[str, Vector3]
    goals: dict[str, Vector3]
    maximum_steps: int
    vehicle_profile: str = "x500"


def _required_distance(
    speed_m_s: float,
    angle_deg: float,
    age_ms: float,
    relative_braking_acceleration_m_s2: float = 6.0,
) -> float:
    relative_speed = 2.0 * speed_m_s * math.sin(math.radians(angle_deg) / 2.0)
    uncertainty = 0.10 * math.sqrt(2.0 * (0.041 + 0.041 + 0.071))
    return (
        20.0
        + 2.0
        + uncertainty
        + relative_speed * (0.65 + age_ms / 1000.0)
        + relative_speed
        * relative_speed
        / (2.0 * relative_braking_acceleration_m_s2)
    )


def _horizontal_case(
    speed: float,
    angle: float,
    tau: float,
    age_ms: float,
) -> SafetyCase:
    if angle == 0.0:
        spawn = {"UAV-01": (-80.0, 0.0, 20.0), "UAV-02": (-80.0, 30.0, 20.0)}
        velocity = {drone: (speed, 0.0, 0.0) for drone in DRONE_IDS}
        goals = {"UAV-01": (80.0, 0.0, 20.0), "UAV-02": (80.0, 30.0, 20.0)}
        horizon_s = 190.0 / speed
    else:
        first = (1.0, 0.0, 0.0)
        radians = math.radians(angle)
        second = (math.cos(radians), math.sin(radians), 0.0)
        relative_speed = 2.0 * speed * math.sin(radians / 2.0)
        initial_distance = _required_distance(speed, angle, age_ms) + 15.0
        time_to_conflict = initial_distance / relative_speed
        spawn = {
            "UAV-01": tuple(-speed * time_to_conflict * value for value in first),
            "UAV-02": tuple(-speed * time_to_conflict * value for value in second),
        }
        spawn = {
            drone: (position[0], position[1], position[2] + 20.0)
            for drone, position in spawn.items()
        }
        velocity = {
            "UAV-01": tuple(speed * value for value in first),
            "UAV-02": tuple(speed * value for value in second),
        }
        after_s = time_to_conflict + 15.0
        goals = {
            "UAV-01": (
                speed * after_s * first[0],
                speed * after_s * first[1],
                20.0,
            ),
            "UAV-02": (
                speed * after_s * second[0],
                speed * after_s * second[1],
                20.0,
            ),
        }
        horizon_s = 2.0 * time_to_conflict + 35.0
    return SafetyCase(
        name=f"horizontal_{angle:03.0f}deg_{speed:02.0f}ms_tau{tau:.2f}_age{age_ms:.0f}",
        speed_m_s=speed,
        encounter_angle_deg=angle,
        response_time_constant_s=tau,
        peer_age_ms=age_ms,
        spawn=spawn,
        velocity=velocity,
        goals=goals,
        maximum_steps=max(800, int(math.ceil(horizon_s / 0.05))) + 1200,
    )


def _vertical_case(speed: float, tau: float, age_ms: float) -> SafetyCase:
    initial_distance = _required_distance(speed, 180.0, age_ms) + 15.0
    half = initial_distance / 2.0
    center = 100.0
    spawn = {
        "UAV-01": (0.0, 0.0, center - half),
        "UAV-02": (0.0, 0.0, center + half),
    }
    velocity = {"UAV-01": (0.0, 0.0, speed), "UAV-02": (0.0, 0.0, -speed)}
    goals = {
        "UAV-01": (0.0, 0.0, center + half + 15.0),
        "UAV-02": (0.0, 0.0, center - half - 15.0),
    }
    horizon_s = 2.0 * half / speed + 35.0
    return SafetyCase(
        name=f"vertical_180deg_{speed:02.0f}ms_tau{tau:.2f}_age{age_ms:.0f}",
        speed_m_s=speed,
        encounter_angle_deg=None,
        response_time_constant_s=tau,
        peer_age_ms=age_ms,
        spawn=spawn,
        velocity=velocity,
        goals=goals,
        maximum_steps=max(800, int(math.ceil(horizon_s / 0.05))) + 1200,
    )


def cases(maximum_speed_m_s: int = 10) -> tuple[SafetyCase, ...]:
    if maximum_speed_m_s < 1:
        raise ValueError("maximum speed must be positive")
    result: list[SafetyCase] = []
    variants = ((0.45, 0.0), (0.75, 100.0), (0.75, 150.0))
    for speed in range(1, maximum_speed_m_s + 1):
        for tau, age_ms in variants:
            for angle in range(0, 181, 15):
                result.append(_horizontal_case(float(speed), float(angle), tau, age_ms))
            result.append(_vertical_case(float(speed), tau, age_ms))
    return tuple(result)


def _bounded_action(action: Vector3, maximum_normalized_speed: float) -> Vector3:
    magnitude = math.sqrt(sum(value * value for value in action))
    if magnitude <= maximum_normalized_speed:
        return action
    return tuple(
        value * maximum_normalized_speed / magnitude for value in action
    )  # type: ignore[return-value]


def evaluate_case(
    payload: tuple[ProximityCbfRlPolicy, SafetyCase]
) -> dict[str, Any]:
    policy, case = payload
    cbf = x500_20m_cbf_config(policy.maximum_velocity_m_s)
    # A vertical right-hand pass can trace almost one avoidance-radius orbit
    # before returning to the goal line. The direct-path horizon alone is not
    # a valid liveness bound at 1 m/s.
    avoidance_steps = math.ceil(
        2.0
        * math.pi
        * policy.avoidance_radius_m
        / (case.speed_m_s * 0.05)
    )
    evaluation_maximum_steps = case.maximum_steps + avoidance_steps
    environment = CbfRlEnvironment(
        case.goals,
        CbfRlEnvConfig(
            maximum_steps=evaluation_maximum_steps,
            cbf=cbf,
            response_time_constant_s=case.response_time_constant_s,
            maximum_acceleration_m_s2=3.0,
            state_max_age_ms=150.0,
        ),
    )
    observations = environment.reset(
        case.spawn,
        velocity_by_drone=case.velocity,
        message_age_ms_by_drone={drone: case.peer_age_ms for drone in DRONE_IDS},
    )
    minimum_distance = math.dist(case.spawn["UAV-01"], case.spawn["UAV-02"])
    minimum_margin = math.inf
    hold_frames = 0
    terminated = truncated = False
    maximum_normalized_speed = case.speed_m_s / cbf.maximum_velocity_m_s
    while not terminated and not truncated:
        actions = {
            drone: _bounded_action(
                policy.act(observations[drone]), maximum_normalized_speed
            )
            for drone in DRONE_IDS
        }
        observations, _, terminated, truncated, info = environment.step(actions)
        minimum_distance = min(
            minimum_distance,
            math.dist(
                environment.positions["UAV-01"], environment.positions["UAV-02"]
            ),
        )
        for drone in DRONE_IDS:
            command = info[drone]["cbf"]
            hold_frames += int(not command["active"])
            if command["minimum_margin_m"] is not None:
                minimum_margin = min(minimum_margin, command["minimum_margin_m"])
    physical_safe = minimum_distance >= 20.0 - 1.0e-6
    dynamic_safe = minimum_margin >= -1.0e-6
    return {
        **asdict(case),
        "evaluation_maximum_steps": evaluation_maximum_steps,
        "steps": environment.steps,
        "minimum_distance_m": round(minimum_distance, 6),
        "minimum_dynamic_margin_m": round(minimum_margin, 6),
        "hold_frames": hold_frames,
        "reached_goals": terminated,
        "physical_safe": physical_safe,
        "dynamic_safe": dynamic_safe,
        "success": physical_safe and dynamic_safe and hold_frames == 0 and terminated,
    }


def run(
    model: str | Path,
    *,
    workers: int = 1,
    maximum_speed_m_s: int | None = None,
) -> dict[str, Any]:
    policy = ProximityCbfRlPolicy.load(model)
    if policy.vehicle_profile != "x500":
        raise ValueError("the safety matrix requires an x500 policy")
    selected_maximum_speed = maximum_speed_m_s or round(policy.maximum_velocity_m_s)
    if (
        selected_maximum_speed < 1
        or selected_maximum_speed > policy.maximum_velocity_m_s
    ):
        raise ValueError("safety-matrix speed exceeds the policy contract")
    selected = cases(selected_maximum_speed)
    if workers == 1:
        results = [evaluate_case((policy, case)) for case in selected]
    else:
        with concurrent.futures.ProcessPoolExecutor(max_workers=workers) as executor:
            results = list(
                executor.map(
                    evaluate_case,
                    ((policy, case) for case in selected),
                    chunksize=1,
                )
            )
    failures = [result for result in results if not result["success"]]
    return {
        "milestone": f"CBF_RL_X500_20M_1_TO_{selected_maximum_speed}MS_SAFETY_MATRIX",
        "vehicle_profile": "x500",
        "model": str(model),
        "case_count": len(results),
        "workers": workers,
        "speed_values_m_s": list(range(1, selected_maximum_speed + 1)),
        "horizontal_angles_deg": list(range(0, 181, 15)),
        "response_time_constants_s": [0.45, 0.75],
        "peer_ages_ms": [0.0, 100.0, 150.0],
        "minimum_distance_m": min(result["minimum_distance_m"] for result in results),
        "minimum_dynamic_margin_m": min(
            result["minimum_dynamic_margin_m"] for result in results
        ),
        "failure_count": len(failures),
        "verdict": "PASS" if not failures else "FAIL",
        "failures": failures,
        "results": results,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("model")
    parser.add_argument(
        "--workers", type=int, default=max(1, min(8, (os.cpu_count() or 2) - 2))
    )
    parser.add_argument("--maximum-speed-m-s", type=int)
    parser.add_argument(
        "--output", default="artifacts/cbf_rl_x500_20m_safety_matrix.json"
    )
    arguments = parser.parse_args()
    report = run(
        arguments.model,
        workers=arguments.workers,
        maximum_speed_m_s=arguments.maximum_speed_m_s,
    )
    destination = Path(arguments.output)
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_text(
        json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    print(json.dumps({key: value for key, value in report.items() if key != "results"}, indent=2))
    print(f"wrote {destination}")
    return 0 if report["verdict"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
