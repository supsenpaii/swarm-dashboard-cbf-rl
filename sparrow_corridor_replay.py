#!/usr/bin/env python3
"""Closed-loop replay of the real companion stack on a corridor geometry.

WHY THIS EXISTS
---------------
The speed matrix answers "is the stack safe" over a fan of encounter angles,
but it spawns each pair on 160 m of open runway with a 15 m head start, so at
low speed the vehicles resolve long before they are ever close: the 1 m/s
head-on case in `cbf_rl_sparrow_20m_certification.json` never gets nearer than
38.2 m. That is a real result about that scenario and a useless one about the
scenario a flight actually flies -- two vehicles at opposite ends of ONE 60 m
corridor that have to swap places. There the conflict cannot be dodged by
geometry; somebody has to give way, and the corridor is barely twice the
separation the barrier demands.

So this replay takes the flight profile itself (the same `.env` the SITL run
would load) and drives `CompanionSafetyMonitor.from_environment` -- the actual
runtime object, with its actual trajectory tracker, RL policy, conflict
coordinator and CBF gate -- against a first-order vehicle model. It is not a
substitute for flying. It is the thing that says whether flying is worth the
airframe, and it can answer that for a profile in seconds.

WHAT THE VEHICLE MODEL IS AND IS NOT
------------------------------------
`velocity += (command - velocity) * dt / tau`, with tau the response time
constant measured on the 2026-08-15 Sparrow flight (0.860 s horizontal). That
is the same first-order lag the CBF gate's `command_latency_s` is sized
against, which is the point: if the gate's own model of the vehicle is wrong,
this replay is wrong in exactly the same direction as the flight, rather than
flattering the gate. It has no attitude loop, no wind, no estimator noise, and
no PX4 mode logic -- a PASS here is a necessary condition for flight, never a
sufficient one.
"""

from __future__ import annotations

import argparse
import json
import math
import os
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Iterator

Vector3 = tuple[float, float, float]

ROOT = Path(__file__).parent
DRONE_IDS = ("UAV-01", "UAV-02")
# Measured on the 2026-08-15 Sparrow flight: horizontal step response tau,
# against 0.275 s vertical. The horizontal number is the binding one for a
# corridor swap, and it is the number the gate's latency was raised to match.
SPARROW_TAU_S = 0.860
# The profiles require published covariance, so a replay that omitted it
# would be refused by the gate for a reason no flight ever hits. These are
# the live SITL GPS accuracies (eph 0.9 m, epv 1.78 m) squared.
SPARROW_COVARIANCE_M2: Vector3 = (0.81, 0.81, 3.17)


def load_env_profile(path: str | Path) -> dict[str, str]:
    """Apply a run_all.sh override profile to `os.environ`, as the stack would.

    `${script_dir}` is expanded the way run_all.sh expands it, so a profile
    that pins a model by path keeps pointing at the same file here.
    """
    applied: dict[str, str] = {}
    for line in Path(path).read_text(encoding="utf-8").splitlines():
        line = line.strip()
        if not line or line.startswith("#") or "=" not in line:
            continue
        key, _, value = line.partition("=")
        value = value.strip().replace("${script_dir}", str(ROOT))
        applied[key.strip()] = value
        os.environ[key.strip()] = value
    return applied


@dataclass
class _Vehicle:
    position_enu_m: list[float]
    velocity_enu_m_s: list[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])

    def step(self, command_enu_m_s: Vector3, dt_s: float, tau_s: float) -> None:
        blend = min(1.0, dt_s / tau_s)
        for axis in range(3):
            self.velocity_enu_m_s[axis] += (
                command_enu_m_s[axis] - self.velocity_enu_m_s[axis]
            ) * blend
            self.position_enu_m[axis] += self.velocity_enu_m_s[axis] * dt_s


def _distance(first: list[float], second: list[float]) -> float:
    return math.sqrt(sum((a - b) ** 2 for a, b in zip(first, second)))


def replay(
    env_path: str | Path,
    *,
    duration_s: float = 240.0,
    dt_s: float = 0.05,
    tau_s: float = SPARROW_TAU_S,
    peer_age_ms: float = 20.0,
    covariance_m2: Vector3 = SPARROW_COVARIANCE_M2,
    missions: dict[str, Any] | None = None,
    trace: bool = False,
) -> dict[str, Any]:
    from companion_safety import CompanionSafetyMonitor
    from conflict_coordinator import reset_shared_conflict_state

    # Profiles differ in which keys they set, so a leaked geofence or ceiling
    # from the previous replay would silently judge the next one under the
    # wrong contract. Build the monitors under the profile, then put the
    # environment back.
    # The runtime keys its encounter ledger by drone pair in a module global,
    # so replay N+1 would otherwise inherit N's priority alternation and its
    # release latch -- and then answer a different question than the one asked.
    reset_shared_conflict_state(DRONE_IDS)

    saved_environment = dict(os.environ)
    try:
        profile = load_env_profile(env_path)
        monitors = {
            drone_id: CompanionSafetyMonitor.from_environment(drone_id, DRONE_IDS)
            for drone_id in DRONE_IDS
        }
        # A drawn mission arrives through the operator path, not the profile,
        # so a closed polyline has to be installed the way the runtime installs
        # one rather than described in an env file that has no syntax for it.
        for drone_id, mission in (missions or {}).items():
            monitors[drone_id].set_trajectory(mission)
    finally:
        os.environ.clear()
        os.environ.update(saved_environment)

    starts = {}
    goals = {}
    waypoints = {}
    for drone_id, monitor in monitors.items():
        start = monitor.trajectory_reference_start_enu_m
        if start is None:
            raise ValueError(f"{drone_id} has no trajectory configured")
        starts[drone_id] = list(start)
        trajectory = monitor.trajectory_tracking.trajectory
        goals[drone_id] = list(getattr(trajectory, "end_enu_m", start))
        # A closed loop has no end to arrive at, so "did it fly the mission"
        # means every corner was visited, not that a final point was reached.
        waypoints[drone_id] = [
            list(point) for point in getattr(trajectory, "waypoints_enu_m", ())
        ]

    vehicles = {
        drone_id: _Vehicle(list(starts[drone_id])) for drone_id in DRONE_IDS
    }

    minimum_distance_m = math.inf
    minimum_margin_m = math.inf
    maximum_required_m = 0.0
    intervened_frames = 0
    applied_frames = 0
    coordinated_frames = 0
    rejected = 0
    samples: list[dict[str, Any]] = []
    breach: dict[str, Any] | None = None
    visited: dict[str, set[int]] = {drone_id: set() for drone_id in DRONE_IDS}
    maximum_cross_track_m: dict[str, float] = {
        drone_id: 0.0 for drone_id in DRONE_IDS
    }
    # Cross-track while avoiding is a deliberate detour, not a tracking
    # error; lumping the two together would report the avoidance layer as
    # a failure of the corner work. Keep the clear-path number separate --
    # that is the one that answers 'does it still hug the drawn path'.
    clear_cross_track_m: dict[str, float] = {
        drone_id: 0.0 for drone_id in DRONE_IDS
    }
    maximum_speed_m_s: dict[str, float] = {
        drone_id: 0.0 for drone_id in DRONE_IDS
    }

    steps = int(duration_s / dt_s)
    for step in range(steps):
        now_s = step * dt_s
        swarm_state = {
            drone_id: {
                "position_enu_m": list(vehicle.position_enu_m),
                "velocity_enu_m_s": list(vehicle.velocity_enu_m_s),
                "position_covariance_m2": list(covariance_m2),
                "message_age_ms": peer_age_ms,
                "valid": True,
            }
            for drone_id, vehicle in vehicles.items()
        }

        outputs = {}
        for drone_id, monitor in monitors.items():
            status = monitor.evaluate(swarm_state, now_s, station_keeping=True)
            outputs[drone_id] = status
            if status.intervened:
                intervened_frames += 1
            shadow = status.cbf_rl_shadow or {}
            if shadow.get("applied"):
                applied_frames += 1
            if (shadow.get("conflict_coordination") or {}).get("active"):
                coordinated_frames += 1
            if shadow.get("valid") is False:
                rejected += 1

        # The gate reports the pair from whichever side is asked; take the
        # tighter of the two so a one-sided view cannot hide a breach.
        distance_m = _distance(
            vehicles[DRONE_IDS[0]].position_enu_m,
            vehicles[DRONE_IDS[1]].position_enu_m,
        )
        required_m = 0.0
        margin_m = math.inf
        for status in outputs.values():
            reported = status.command.as_dict()
            required_m = max(required_m, reported["critical_required_separation_m"] or 0.0)
            if reported["minimum_margin_m"] is not None:
                margin_m = min(margin_m, reported["minimum_margin_m"])
        if not math.isfinite(margin_m):
            margin_m = distance_m - required_m
        minimum_distance_m = min(minimum_distance_m, distance_m)
        minimum_margin_m = min(minimum_margin_m, margin_m)
        maximum_required_m = max(maximum_required_m, required_m)
        if margin_m < 0.0 and breach is None:
            breach = {
                "time_s": round(now_s, 2),
                "distance_m": round(distance_m, 3),
                "required_m": round(required_m, 3),
                "margin_m": round(margin_m, 3),
            }
        if trace and step % 40 == 0:
            samples.append(
                {
                    "t": round(now_s, 1),
                    "d": round(distance_m, 2),
                    "req": round(required_m, 2),
                    "margin": round(margin_m, 2),
                    **{
                        drone_id: [
                            round(value, 2)
                            for value in vehicles[drone_id].position_enu_m
                        ]
                        for drone_id in DRONE_IDS
                    },
                }
            )

        for drone_id, status in outputs.items():
            vehicle = vehicles[drone_id]
            vehicle.step(status.output_velocity_enu_m_s, dt_s, tau_s)
            maximum_speed_m_s[drone_id] = max(
                maximum_speed_m_s[drone_id],
                math.sqrt(sum(v * v for v in vehicle.velocity_enu_m_s)),
            )
            for index, waypoint in enumerate(waypoints[drone_id]):
                if _distance(vehicle.position_enu_m, waypoint) <= 5.0:
                    visited[drone_id].add(index)
            trajectory = monitors[drone_id].trajectory_tracking.trajectory
            nearest = getattr(trajectory, "nearest_time_s", None)
            if callable(nearest):
                # Cross-track against the drawn path is the number the corner
                # work exists to hold; measuring it only while the pair is
                # clear would measure the easy half.
                on_path = trajectory.reference(
                    float(nearest(tuple(vehicle.position_enu_m)))
                ).position_enu_m
                cross_track_m = _distance(vehicle.position_enu_m, list(on_path))
                maximum_cross_track_m[drone_id] = max(
                    maximum_cross_track_m[drone_id], cross_track_m
                )
                shadow = outputs[drone_id].cbf_rl_shadow or {}
                avoiding = outputs[drone_id].intervened or (
                    shadow.get("conflict_coordination") or {}
                ).get("active")
                # Right after a detour the vehicle is legitimately far off the
                # path and returning; the runtime says so by reading
                # "trajectory_entering" until it has rejoined. Only frames
                # where it claims to be tracking are frames where cross-track
                # is a tracking measurement.
                if not avoiding and (
                    outputs[drone_id].nominal_reason == "tracking_trajectory"
                ):
                    clear_cross_track_m[drone_id] = max(
                        clear_cross_track_m[drone_id], cross_track_m
                    )

    reached = {
        drone_id: round(
            _distance(vehicles[drone_id].position_enu_m, goals[drone_id]), 3
        )
        for drone_id in DRONE_IDS
    }
    # "Swapped" is the whole point of a corridor: arriving anywhere else is a
    # deadlock dressed up as a safe flight.
    swapped = all(value <= 3.0 for value in reached.values())
    if any(waypoints.values()):
        # Closed mission: flying it means visiting every corner.
        swapped = all(
            len(visited[drone_id]) == len(waypoints[drone_id])
            for drone_id in DRONE_IDS
        )

    return {
        "profile": str(env_path),
        "mode": profile.get("SWARM_CBF_RL_MODE", "off"),
        "model": Path(profile.get("SWARM_CBF_RL_MODEL", "")).name,
        "command_latency_s": float(profile.get("SWARM_CBF_COMMAND_LATENCY_S", 0.0)),
        "response_tau_s": tau_s,
        "duration_s": duration_s,
        "verdict": (
            "REPLAY_FAIL_SEPARATION"
            if breach is not None
            else "REPLAY_PASS" if swapped else "REPLAY_FAIL_DEADLOCK"
        ),
        "minimum_distance_m": round(minimum_distance_m, 3),
        "minimum_dynamic_margin_m": round(minimum_margin_m, 3),
        "maximum_required_distance_m": round(maximum_required_m, 3),
        "first_breach": breach,
        "distance_to_goal_m": reached,
        "waypoints_visited": {
            drone_id: f"{len(visited[drone_id])}/{len(waypoints[drone_id])}"
            for drone_id in DRONE_IDS
            if waypoints[drone_id]
        },
        "maximum_cross_track_m": {
            drone_id: round(value, 3)
            for drone_id, value in maximum_cross_track_m.items()
        },
        "maximum_cross_track_clear_of_conflict_m": {
            drone_id: round(value, 3)
            for drone_id, value in clear_cross_track_m.items()
        },
        "maximum_speed_m_s": {
            drone_id: round(value, 3)
            for drone_id, value in maximum_speed_m_s.items()
        },
        "swapped_ends": swapped,
        "cbf_intervention_rate": round(intervened_frames / (2 * steps), 4),
        "rl_applied_rate": round(applied_frames / (2 * steps), 4),
        "coordinated_rate": round(coordinated_frames / (2 * steps), 4),
        "rl_rejected_frames": rejected,
        "trace": samples,
    }


def counter_rotating_square(
    side_m: float, speed_m_s: float, altitude_m: float = 30.0
) -> dict[str, Any]:
    """The same drawn square, flown by both vehicles in opposite directions.

    This is the product's hardest honest scenario in one geometry: 90 degree
    corners force the tracker to brake hard for accuracy, and counter-rotation
    guarantees two head-on encounters per lap ON the path -- so corner slowdown
    and collision avoidance have to hold at the same time, on the same frames,
    rather than being demonstrated separately.
    """
    from trajectory_controller import ClosedPolylineTrajectory

    corners = (
        (0.0, 0.0, altitude_m),
        (side_m, 0.0, altitude_m),
        (side_m, side_m, altitude_m),
        (0.0, side_m, altitude_m),
    )
    return {
        "UAV-01": ClosedPolylineTrajectory(
            waypoints_enu_m=corners, speed_m_s=speed_m_s
        ),
        "UAV-02": ClosedPolylineTrajectory(
            waypoints_enu_m=tuple(reversed(corners)), speed_m_s=speed_m_s
        ),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("profile", help="run_all.sh env override profile to replay")
    parser.add_argument("--duration-s", type=float, default=240.0)
    parser.add_argument("--dt-s", type=float, default=0.05)
    parser.add_argument("--tau-s", type=float, default=SPARROW_TAU_S)
    parser.add_argument("--trace", action="store_true")
    parser.add_argument(
        "--counter-rotating-square-m",
        type=float,
        help="replace the profile trajectories with one square flown both ways",
    )
    parser.add_argument("--output")
    arguments = parser.parse_args()

    missions = None
    if arguments.counter_rotating_square_m:
        import os

        load_env_profile(arguments.profile)
        missions = counter_rotating_square(
            arguments.counter_rotating_square_m,
            float(os.environ["SWARM_TRAJECTORY_UAV_01_SPEED_M_S"]),
        )
    report = replay(
        arguments.profile,
        duration_s=arguments.duration_s,
        dt_s=arguments.dt_s,
        tau_s=arguments.tau_s,
        missions=missions,
        trace=arguments.trace,
    )
    if arguments.output:
        destination = Path(arguments.output)
        destination.parent.mkdir(parents=True, exist_ok=True)
        destination.write_text(
            json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8"
        )
    printable = {key: value for key, value in report.items() if key != "trace"}
    print(json.dumps(printable, indent=2))
    for sample in report["trace"]:
        print(sample)
    return 0 if report["verdict"] == "REPLAY_PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
