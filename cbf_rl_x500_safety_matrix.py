#!/usr/bin/env python3
"""Deterministic 20 m safety gate over vehicle speeds, angles and lag bounds."""

from __future__ import annotations

import argparse
import concurrent.futures
import json
import math
import statistics
import os
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any

from cbf_command_gate import CbfConfig
from cbf_rl_env import (
    CbfRlEnvConfig,
    CbfRlEnvironment,
    DRONE_IDS,
    sparrow_10m_floor_cbf_config,
    sparrow_20m_cbf_config,
    x500_20m_cbf_config,
)
from cbf_rl_policy import ProximityCbfRlPolicy
from conflict_coordinator import sparrow_20m_conflict_config


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
    minimum_separation_m: float = 20.0


def profile_config(
    vehicle_profile: str,
    maximum_velocity_m_s: float = 10.0,
    minimum_separation_m: float = 20.0,
) -> CbfConfig:
    """The one contract a policy is judged against, for spawn and for solve.

    The floor is a parameter, not a property of the airframe. Reading it from
    the profile alone judged a policy trained against a 10 m floor with a 20 m
    gate: it approached to the distance it was taught was safe, the gate held
    it at twice that, and three separately-seeded policies all stopped at the
    same 22.055 m -- which is 20 + 2 + uncertainty, the wrong contract's
    requirement, and the tell that the wall was the harness rather than them.
    """
    if vehicle_profile == "sparrow":
        return (
            sparrow_10m_floor_cbf_config(maximum_velocity_m_s)
            if minimum_separation_m < 20.0
            else sparrow_20m_cbf_config(maximum_velocity_m_s)
        )
    return x500_20m_cbf_config(maximum_velocity_m_s)


def _required_distance(
    speed_m_s: float,
    angle_deg: float,
    age_ms: float,
    config: CbfConfig,
) -> float:
    """Spawn far enough apart that the encounter is real, read off the contract.

    Every term here used to be a literal that happened to equal the config it
    was meant to mirror. Two sources for one number is one too many: lower the
    floor in the config and the spawn geometry silently kept testing the old
    one.
    """
    relative_speed = 2.0 * speed_m_s * math.sin(math.radians(angle_deg) / 2.0)
    uncertainty = config.covariance_sigma * math.sqrt(2.0 * (0.041 + 0.041 + 0.071))
    return (
        config.minimum_separation_m
        + config.tracking_reserve_m
        + uncertainty
        + relative_speed * (config.command_latency_s + age_ms / 1000.0)
        + relative_speed
        * relative_speed
        / (2.0 * config.relative_braking_acceleration_m_s2)
    )


def _horizontal_case(
    speed: float,
    angle: float,
    tau: float,
    age_ms: float,
    vehicle_profile: str = "x500",
    minimum_separation_m: float = 20.0,
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
        initial_distance = (
            _required_distance(
                speed,
                angle,
                age_ms,
                profile_config(vehicle_profile, minimum_separation_m=minimum_separation_m),
            )
            + 15.0
        )
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
        vehicle_profile=vehicle_profile,
        minimum_separation_m=minimum_separation_m,
    )


def _vertical_case(
    speed: float,
    tau: float,
    age_ms: float,
    vehicle_profile: str = "x500",
    minimum_separation_m: float = 20.0,
) -> SafetyCase:
    initial_distance = (
        _required_distance(
            speed,
            180.0,
            age_ms,
            profile_config(vehicle_profile, minimum_separation_m=minimum_separation_m),
        )
        + 15.0
    )
    half = initial_distance / 2.0
    config = profile_config(
        vehicle_profile, minimum_separation_m=minimum_separation_m
    )
    floor_m = config.geofence_min_enu_m[2]
    ceiling_m = config.geofence_max_enu_m[2]
    center = (floor_m + ceiling_m) / 2.0
    # The goal has to sit inside the fence the barrier will hold the vehicle
    # to. This case scales its spawn distance with speed, and at 20 m/s the
    # 15 m goal margin put both goals 0.728 m OUTSIDE a 0-200 m fence: each
    # vehicle flew its encounter cleanly, passed, reached the fence, and then
    # hovered a fraction short of a goal it was forbidden to reach. That read
    # as two liveness failures in the Sparrow 20 m/s matrix for as long as the
    # matrix existed, and neither was one -- both held 22.3 m of separation
    # over 25,000 steps.
    goal_margin_m = min(15.0, (ceiling_m - floor_m) / 2.0 - half - 1.0)
    if goal_margin_m <= 0.0:
        raise ValueError(
            f"vertical case at {speed:g} m/s needs "
            f"{2.0 * (half + 1.0):.1f} m of fence and has "
            f"{ceiling_m - floor_m:.1f} m"
        )
    spawn = {
        "UAV-01": (0.0, 0.0, center - half),
        "UAV-02": (0.0, 0.0, center + half),
    }
    velocity = {"UAV-01": (0.0, 0.0, speed), "UAV-02": (0.0, 0.0, -speed)}
    goals = {
        "UAV-01": (0.0, 0.0, center + half + goal_margin_m),
        "UAV-02": (0.0, 0.0, center - half - goal_margin_m),
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
        vehicle_profile=vehicle_profile,
        minimum_separation_m=minimum_separation_m,
    )


def _climbing_case(
    speed: float,
    angle: float,
    tau: float,
    age_ms: float,
    vehicle_profile: str = "x500",
    minimum_separation_m: float = 20.0,
    climb_deg: float = 30.0,
) -> SafetyCase:
    """A crossing that is also a climb, which neither other family covers.

    `_horizontal_case` holds altitude and `_vertical_case` holds ground track,
    so between them every case in this matrix moves along exactly one of the
    two. Real missions do not: an aircraft changing level while crossing
    another's track is the ordinary case, and it is the one the coordinator has
    never been measured on. Its yield branches on `mission_horizontal_speed`,
    so a climbing crossing takes the HORIZONTAL branch -- which spends
    horizontal speed on the lane change and passes `mission[2]` through
    untouched. Whether that is enough when a third of the closing rate is
    vertical is the question, and nothing else here asks it.

    Total speed stays `speed` so the case is comparable to its neighbours: the
    climb tilts the velocity rather than adding to it. One vehicle climbs and
    the other descends, and both are spawned off-level by exactly the distance
    that tilt covers before the conflict, so they arrive at one point in three
    dimensions rather than merely passing near each other in two.
    """
    climb = math.radians(climb_deg)
    horizontal_speed = speed * math.cos(climb)
    vertical_speed = speed * math.sin(climb)
    radians = math.radians(angle)
    first = (1.0, 0.0)
    second = (math.cos(radians), math.sin(radians))

    horizontal_relative = 2.0 * horizontal_speed * math.sin(radians / 2.0)
    relative_speed = math.hypot(horizontal_relative, 2.0 * vertical_speed)
    # Reuse the one contract-derived spawn distance rather than restating its
    # terms: solve for the flat encounter angle that closes at the same rate.
    effective_angle = math.degrees(
        2.0 * math.asin(min(1.0, relative_speed / (2.0 * speed)))
    )
    config = profile_config(
        vehicle_profile, minimum_separation_m=minimum_separation_m
    )
    initial_distance = (
        _required_distance(speed, effective_angle, age_ms, config) + 15.0
    )
    time_to_conflict = initial_distance / relative_speed

    floor_m = config.geofence_min_enu_m[2]
    ceiling_m = config.geofence_max_enu_m[2]
    center = (floor_m + ceiling_m) / 2.0
    rise = vertical_speed * time_to_conflict
    # Fly on past the conflict far enough to prove the pair separated, but not
    # through the fence the barrier will hold them to. Each goal ends up
    # `vertical_speed * settle_s` from centre, so at 14 m/s a flat 15 s put it
    # at 205 m against a 200 m ceiling -- the same trap the vertical family
    # already carries a clamp for, and the reason this one is a raise rather
    # than a silent goal nobody can reach.
    settle_s = min(15.0, (ceiling_m - center - 1.0) / max(vertical_speed, 1e-9))
    after_s = time_to_conflict + settle_s
    spawn = {
        "UAV-01": (
            -horizontal_speed * time_to_conflict * first[0],
            -horizontal_speed * time_to_conflict * first[1],
            center - rise,
        ),
        "UAV-02": (
            -horizontal_speed * time_to_conflict * second[0],
            -horizontal_speed * time_to_conflict * second[1],
            center + rise,
        ),
    }
    velocity = {
        "UAV-01": (
            horizontal_speed * first[0],
            horizontal_speed * first[1],
            vertical_speed,
        ),
        "UAV-02": (
            horizontal_speed * second[0],
            horizontal_speed * second[1],
            -vertical_speed,
        ),
    }
    goals = {
        "UAV-01": (
            horizontal_speed * after_s * first[0],
            horizontal_speed * after_s * first[1],
            center - rise + vertical_speed * after_s,
        ),
        "UAV-02": (
            horizontal_speed * after_s * second[0],
            horizontal_speed * after_s * second[1],
            center + rise - vertical_speed * after_s,
        ),
    }
    for drone, goal in goals.items():
        if not floor_m < goal[2] < ceiling_m:
            raise ValueError(
                f"climbing case at {speed:g} m/s puts {drone}'s goal at "
                f"{goal[2]:.1f} m, outside the {floor_m:.0f}-{ceiling_m:.0f} m "
                "fence the barrier will hold it to"
            )
    horizon_s = 2.0 * time_to_conflict + 35.0
    return SafetyCase(
        name=(
            f"climbing_{angle:03.0f}deg_{climb_deg:02.0f}up"
            f"_{speed:02.0f}ms_tau{tau:.2f}_age{age_ms:.0f}"
        ),
        speed_m_s=speed,
        encounter_angle_deg=angle,
        response_time_constant_s=tau,
        peer_age_ms=age_ms,
        spawn=spawn,
        velocity=velocity,
        goals=goals,
        maximum_steps=max(800, int(math.ceil(horizon_s / 0.05))) + 1200,
        vehicle_profile=vehicle_profile,
        minimum_separation_m=minimum_separation_m,
    )


def cases(
    maximum_speed_m_s: int = 10,
    vehicle_profile: str = "x500",
    minimum_separation_m: float = 20.0,
) -> tuple[SafetyCase, ...]:
    if maximum_speed_m_s < 1 or vehicle_profile not in {"x500", "sparrow"}:
        raise ValueError("maximum speed and vehicle profile are invalid")
    result: list[SafetyCase] = []
    variants = (
        # 0.90 s, not the 0.75 s this used to carry. The 2026-08-15 Sparrow
        # flight measured the horizontal response at tau = 0.860 s against a
        # vertical 0.275 s, so the old upper bound was testing a plant faster
        # than the real one on the axis that matters most for a crossing.
        ((0.45, 0.0), (0.90, 100.0))
        if vehicle_profile == "sparrow"
        else ((0.45, 0.0), (0.75, 100.0), (0.75, 150.0))
    )
    for speed in range(1, maximum_speed_m_s + 1):
        for tau, age_ms in variants:
            for angle in range(0, 181, 15):
                result.append(
                    _horizontal_case(
                        float(speed),
                        float(angle),
                        tau,
                        age_ms,
                        vehicle_profile,
                        minimum_separation_m,
                    )
                )
            result.append(
                _vertical_case(
                    float(speed), tau, age_ms, vehicle_profile, minimum_separation_m
                )
            )
            # 90 degrees: the pure crossing, where neither vehicle's track
            # gives way to the other's by geometry alone, and the one angle at
            # which a lane change is least able to borrow from along-path
            # speed. One climbing case per speed and lag variant, the same
            # weight the vertical family carries.
            result.append(
                _climbing_case(
                    float(speed),
                    90.0,
                    tau,
                    age_ms,
                    vehicle_profile,
                    minimum_separation_m,
                )
            )
    return tuple(result)


def _bounded_action(action: Vector3, maximum_normalized_speed: float) -> Vector3:
    magnitude = math.sqrt(sum(value * value for value in action))
    if magnitude <= maximum_normalized_speed:
        return action
    return tuple(
        value * maximum_normalized_speed / magnitude for value in action
    )  # type: ignore[return-value]


def certified_speed_by_geometry(results: list[dict[str, Any]]) -> dict[str, int]:
    """Highest speed each geometry family clears with every rung below it clean.

    One number for the whole matrix hides the shape of a failure. Sparrow at
    20 m/s reads FAIL, but the four cases that fail are all vertical head-on,
    and horizontal -- which is what a drawn mission actually flies, since
    missions are polylines at one altitude -- is clean to the top. Reporting a
    single verdict there would either overstate the envelope or discard a
    result that is genuinely usable.
    """
    families: dict[str, dict[float, bool]] = {}
    for result in results:
        family = (
            "vertical_180deg"
            if result.get("encounter_angle_deg") is None
            else "horizontal"
        )
        speed = float(result["speed_m_s"])
        passed = families.setdefault(family, {})
        passed[speed] = passed.get(speed, True) and bool(result["success"])
    certified = {}
    for family, by_speed in families.items():
        top = 0
        for speed in sorted(by_speed):
            if not by_speed[speed]:
                break
            top = int(speed)
        certified[family] = top
    return certified


def evaluate_case(
    payload: tuple[ProximityCbfRlPolicy, SafetyCase]
) -> dict[str, Any]:
    policy, case = payload
    cbf = profile_config(
        case.vehicle_profile,
        policy.maximum_velocity_m_s,
        case.minimum_separation_m,
    )
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
            maximum_acceleration_m_s2=(
                4.0 if case.vehicle_profile == "sparrow" else 3.0
            ),
            state_max_age_ms=(100.0 if case.vehicle_profile == "sparrow" else 150.0),
            # Sparrow is judged against the stack it actually flies, which has
            # run this between the policy and the barrier since the mission
            # milestone. x500 keeps the older gate so its certified rungs still
            # mean what they meant when they were signed off.
            conflict_coordination=(
                sparrow_20m_conflict_config(policy.maximum_velocity_m_s)
                if case.vehicle_profile == "sparrow"
                else None
            ),
            # The policy is bounded to the case speed below, so the mission
            # the coordinator rebuilds its output from is bounded with it.
            mission_speed_m_s=case.speed_m_s,
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
    # How hard the barrier had to work, not just whether it held. A CBF is a
    # hard constraint: it holds the line whether the coordinator resolved the
    # encounter or left the whole job to it, so distance and margin cannot
    # tell those apart. Measured on the Sparrow 10 m/s cases, the yield
    # geometry that saturated at 5 m in flight moved the worst per-case margin
    # from 2.632 m to 1.701 m and the peak intervention from 2.401 to 2.603
    # m/s -- while minimum_distance_m stayed identical to seven digits,
    # because the aggregate minimum is set by cases where the barrier sits on
    # its boundary by construction. That is why 1260 cases certified PASS
    # through a broken lane change.
    barrier_frames = 0
    intervened_frames = 0
    peak_intervention_m_s = 0.0
    coordinated_frames = 0
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
            barrier_frames += 1
            norm = command.get("intervention_norm_m_s") or 0.0
            # 0.1 m/s is PX4's measured tracking noise floor, the same bound
            # the 2026-08-10 crossing sweep used to call an intervention real.
            intervened_frames += int(norm > 0.1)
            peak_intervention_m_s = max(peak_intervention_m_s, norm)
            coordinated_frames += int(
                bool((info[drone].get("conflict_coordination") or {}).get("active"))
            )
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
        "cbf_intervention_rate": (
            round(intervened_frames / barrier_frames, 4) if barrier_frames else None
        ),
        "peak_intervention_norm_m_s": round(peak_intervention_m_s, 3),
        "coordinated_frames": coordinated_frames,
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
    expected_vehicle_profile: str | None = None,
) -> dict[str, Any]:
    policy = ProximityCbfRlPolicy.load(model)
    if policy.vehicle_profile not in {"x500", "sparrow"}:
        raise ValueError("the safety matrix requires a high-speed policy")
    if expected_vehicle_profile and policy.vehicle_profile != expected_vehicle_profile:
        raise ValueError("the safety matrix vehicle profile does not match")
    selected_maximum_speed = maximum_speed_m_s or round(policy.maximum_velocity_m_s)
    if (
        selected_maximum_speed < 1
        or selected_maximum_speed > policy.maximum_velocity_m_s
    ):
        raise ValueError("safety-matrix speed exceeds the policy contract")
    selected = cases(
        selected_maximum_speed,
        policy.vehicle_profile,
        policy.minimum_separation_m,
    )
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
        "milestone": f"CBF_RL_{policy.vehicle_profile.upper()}_20M_1_TO_{selected_maximum_speed}MS_SAFETY_MATRIX",
        "vehicle_profile": policy.vehicle_profile,
        "model": str(model),
        "case_count": len(results),
        "certified_speed_by_geometry_m_s": certified_speed_by_geometry(results),
        "workers": workers,
        "speed_values_m_s": list(range(1, selected_maximum_speed + 1)),
        # The rung this sweep certifies. It was never emitted, so every
        # certification carried a hand-written copy -- two sources for one
        # number, which is exactly what _required_distance's own comment
        # warns about, and the ladder test reads this one.
        "maximum_speed_m_s": selected_maximum_speed,
        "horizontal_angles_deg": list(range(0, 181, 15)),
        "response_time_constants_s": sorted(
            {case.response_time_constant_s for case in selected}
        ),
        "peer_ages_ms": sorted({case.peer_age_ms for case in selected}),
        "minimum_distance_m": min(result["minimum_distance_m"] for result in results),
        "minimum_dynamic_margin_m": min(
            result["minimum_dynamic_margin_m"] for result in results
        ),
        # Barrier effort, aggregated so a coordinator regression is visible
        # even when every case still passes. The minima above cannot show one:
        # a CBF holds its constraint whether the coordinator did its job or
        # left the whole encounter to it, and minimum_distance_m is set by
        # cases that sit on the boundary by construction.
        # A minimum is the wrong statistic for a coordinator regression. It
        # is set by one case, and the case that sets it is one where the
        # barrier sits on its constraint boundary by construction -- 0.000 m
        # whether the lane change works or not. What a broken coordinator does
        # is make MANY cases worse at once, which only a robust central
        # statistic can see.
        #
        # Measured across the Sparrow 10 m/s matrix against the yield geometry
        # that saturated at 5 m in flight: 72 of 280 cases degraded, one from
        # 10.083 m of margin to 5.944, and every number the matrix reported
        # stayed bit-identical -- minimum_distance_m to seven digits. The
        # median moved 2.760 -> 1.957 and the count under 2 m moved 125 -> 143.
        # That is how 1260 cases certified PASS through a broken lane change.
        "median_dynamic_margin_m": round(
            statistics.median(
                result["minimum_dynamic_margin_m"] for result in results
            ),
            6,
        ),
        "cases_below_two_metres_of_margin": sum(
            1 for result in results if result["minimum_dynamic_margin_m"] < 2.0
        ),
        "coordinated_case_count": sum(
            1 for result in results if result["coordinated_frames"] > 0
        ),
        "maximum_cbf_intervention_rate": max(
            (result["cbf_intervention_rate"] or 0.0) for result in results
        ),
        "peak_intervention_norm_m_s": max(
            result["peak_intervention_norm_m_s"] for result in results
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
    parser.add_argument("--output")
    arguments = parser.parse_args()
    report = run(
        arguments.model,
        workers=arguments.workers,
        maximum_speed_m_s=arguments.maximum_speed_m_s,
    )
    destination = Path(
        arguments.output
        or f"artifacts/cbf_rl_{report['vehicle_profile']}_20m_safety_matrix.json"
    )
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_text(
        json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    print(json.dumps({key: value for key, value in report.items() if key != "results"}, indent=2))
    print(f"wrote {destination}")
    return 0 if report["verdict"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
