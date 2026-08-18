#!/usr/bin/env python3
"""Fly two SITL vehicles on one UI mission circle with safety guards."""

from __future__ import annotations

import argparse
import asyncio
import json
import math
import subprocess
import time
import urllib.request
from pathlib import Path
from typing import Any, Sequence

import websockets

from trajectory_controller import ClosedPolylineTrajectory

API_URL = "http://127.0.0.1:8000/api/drones"
WS_URL = "ws://127.0.0.1:8000/ws"
DRONE_IDS = ("UAV-01", "UAV-02")
OFFBOARD_NAV_STATE = 14
POSITION_NAV_STATE = 2
EARTH_RADIUS_M = 6_378_137.0
ORIGIN_LATITUDE_DEG = 47.397971057728974
ORIGIN_LONGITUDE_DEG = 8.546163739800146
DEFAULT_MODEL_SHA256 = (
    "458899a16cc6a60d916bf020be055fe4bcec78553e18576202de5ab7eb8ff5e1"
)
COMMANDER = "/mnt/px4ssd/PX4-Autopilot/build/px4_sitl_default/bin/px4-commander"


class FlightAbort(RuntimeError):
    pass


SPAWN_ENU_M = {
    "UAV-01": (0.0, 0.0, 9.0),
    "UAV-02": (-5.0, 2.0, 9.0),
}


def enu_waypoints_to_geodetic(
    points_by_drone: dict[str, list[tuple[float, float, float]]],
) -> dict[str, list[dict[str, float]]]:
    latitude_scale = math.degrees(1.0 / EARTH_RADIUS_M)
    longitude_scale = math.degrees(
        1.0
        / (EARTH_RADIUS_M * math.cos(math.radians(ORIGIN_LATITUDE_DEG)))
    )
    return {
        drone_id: [
            {
                "latitude_deg": ORIGIN_LATITUDE_DEG + north * latitude_scale,
                "longitude_deg": ORIGIN_LONGITUDE_DEG + east * longitude_scale,
            }
            for east, north, _altitude in points
        ]
        for drone_id, points in points_by_drone.items()
    }


def mission_waypoints_enu(
    scenario: str,
    *,
    center_enu_m: tuple[float, float] = (-1.5, 3.5),
    radius_m: float = 5.0,
) -> dict[str, list[tuple[float, float, float]]]:
    altitude = 9.0
    first = SPAWN_ENU_M["UAV-01"]
    second = SPAWN_ENU_M["UAV-02"]
    crossing = (-1.0, 4.75, altitude)
    if scenario == "circle":
        points = [
            (
                center_enu_m[0] + radius_m * math.cos(2.0 * math.pi * i / 24),
                center_enu_m[1] + radius_m * math.sin(2.0 * math.pi * i / 24),
                altitude,
            )
            for i in range(24)
        ]
        return {drone_id: points for drone_id in DRONE_IDS}
    if scenario == "diagonal_cross":
        return {
            "UAV-01": [first, crossing, (6.0, 5.0, altitude), (6.0, -4.0, altitude)],
            "UAV-02": [second, crossing, (-7.0, 8.0, altitude), (-10.0, -2.0, altitude)],
        }
    if scenario == "head_on_swap":
        return {
            "UAV-01": [first, second, (-5.0, -6.0, altitude), (2.0, -6.0, altitude)],
            "UAV-02": [second, first, (2.0, 8.0, altitude), (-7.0, 8.0, altitude)],
        }
    if scenario == "repeated_bow_tie":
        return {
            "UAV-01": [first, crossing, (6.0, 4.75, altitude), (-1.0, 10.0, altitude)],
            "UAV-02": [second, crossing, (-8.0, 4.75, altitude), (-1.0, -3.0, altitude)],
        }
    if scenario != "opposite_orbit":
        raise ValueError(f"unknown scenario: {scenario}")

    # One geometric circle through both spawn points.  Reversing UAV-02's
    # vertex order makes repeated head-on encounters without an entry dash.
    orbit_radius = 7.0
    chord = math.dist(first[:2], second[:2])
    midpoint = ((first[0] + second[0]) / 2.0, (first[1] + second[1]) / 2.0)
    offset = math.sqrt(orbit_radius * orbit_radius - (chord / 2.0) ** 2)
    center = (
        midpoint[0] + offset * (second[1] - first[1]) / chord,
        midpoint[1] - offset * (second[0] - first[0]) / chord,
    )
    start_angle = math.atan2(first[1] - center[1], first[0] - center[0])
    points = [
        (
            center[0] + orbit_radius * math.cos(start_angle + 2.0 * math.pi * i / 32),
            center[1] + orbit_radius * math.sin(start_angle + 2.0 * math.pi * i / 32),
            altitude,
        )
        for i in range(32)
    ]
    first_index = min(range(32), key=lambda index: math.dist(points[index], first))
    second_index = min(range(32), key=lambda index: math.dist(points[index], second))
    points[first_index] = first
    points[second_index] = second
    uav_01 = points[first_index:] + points[:first_index]
    reversed_points = list(reversed(points))
    reversed_second = reversed_points.index(second)
    return {
        "UAV-01": uav_01,
        "UAV-02": reversed_points[reversed_second:] + reversed_points[:reversed_second],
    }


def fit_to_spawns(
    points_by_drone: dict[str, list[tuple[float, float, float]]],
    actual_first: Sequence[float],
    actual_second: Sequence[float],
) -> dict[str, list[tuple[float, float, float]]]:
    """Rotate and scale the whole picture onto where the vehicles actually are.

    The scenarios below are drawn against SPAWN_ENU_M, two points 5.39 m apart
    -- a geometry from the 4 m separation envelope. Sparrow's certified
    envelope asks for 20 m of separation and its coordinator does not engage
    until 120 m, so at that scale the vehicles start inside the barrier and
    every scenario aborts before it flies.

    A similarity transform about the nominal spawn midpoint fixes that without
    touching a single scenario: every crossing angle, every bow-tie, every
    orbit is preserved exactly, and only the distances become real. Nothing
    here is scenario-specific, so a scenario added later scales for free.
    """
    nominal_first = SPAWN_ENU_M[DRONE_IDS[0]]
    nominal_second = SPAWN_ENU_M[DRONE_IDS[1]]
    nominal = (
        nominal_second[0] - nominal_first[0],
        nominal_second[1] - nominal_first[1],
    )
    actual = (
        float(actual_second[0]) - float(actual_first[0]),
        float(actual_second[1]) - float(actual_first[1]),
    )
    nominal_span = math.hypot(*nominal)
    actual_span = math.hypot(*actual)
    if nominal_span <= 1.0e-6 or actual_span <= 1.0e-6:
        # Fail closed: without two distinguishable spawns there is no frame to
        # fit to, and a silently unscaled scenario is one that flies the wrong
        # distances at the right angles.
        raise FlightAbort(
            f"cannot_fit_scenario_to_spawns:{nominal_span:.3f}:{actual_span:.3f}"
        )
    scale = actual_span / nominal_span
    angle = math.atan2(actual[1], actual[0]) - math.atan2(nominal[1], nominal[0])
    cos_a, sin_a = math.cos(angle), math.sin(angle)
    nominal_mid = (
        (nominal_first[0] + nominal_second[0]) / 2.0,
        (nominal_first[1] + nominal_second[1]) / 2.0,
    )
    actual_mid = (
        (float(actual_first[0]) + float(actual_second[0])) / 2.0,
        (float(actual_first[1]) + float(actual_second[1])) / 2.0,
    )

    def place(point: tuple[float, float, float]) -> tuple[float, float, float]:
        east = (point[0] - nominal_mid[0]) * scale
        north = (point[1] - nominal_mid[1]) * scale
        return (
            actual_mid[0] + east * cos_a - north * sin_a,
            actual_mid[1] + east * sin_a + north * cos_a,
            point[2],  # altitude is commanded separately, never scaled
        )

    return {
        drone_id: [place(point) for point in points]
        for drone_id, points in points_by_drone.items()
    }


def fetch_states() -> dict[str, dict[str, Any]]:
    with urllib.request.urlopen(API_URL, timeout=5) as response:
        payload = json.load(response)
    result: dict[str, dict[str, Any]] = {}
    for drone_id in DRONE_IDS:
        drone = payload["drones"][drone_id]
        safety = (
            payload["tracking_pose_streams"][drone_id].get("companion_safety")
            or {}
        )
        local = drone.get("local_position") or {}
        result[drone_id] = {
            "online": drone.get("online"),
            "armed": (drone.get("status") or {}).get("armed"),
            "nav_state": (drone.get("status") or {}).get("nav_state"),
            "preflight_checks_pass": (drone.get("status") or {}).get(
                "preflight_checks_pass"
            ),
            "failsafe": (drone.get("status") or {}).get("failsafe"),
            "altitude_m": (
                None
                if local.get("z_down_m") is None
                else -float(local["z_down_m"])
            ),
            "position_enu_m": safety.get("own_position_enu_m"),
            "mission": safety.get("mission") or {},
            "sender": safety.get("active_offboard_sender") or {},
            "conditions": safety.get("active_offboard_conditions") or [],
            "cbf": safety.get("cbf") or {},
            "cbf_margin_extrema": safety.get("cbf_margin_extrema") or {},
            "cbf_rl": safety.get("cbf_rl_shadow") or {},
            "tracking_error_m": safety.get("nominal_position_error_m"),
            "intervened": safety.get("intervened"),
        }
    return result


def physical_separation_m(states: dict[str, dict[str, Any]]) -> float | None:
    first = states[DRONE_IDS[0]]["position_enu_m"]
    second = states[DRONE_IDS[1]]["position_enu_m"]
    if first is None or second is None:
        return None
    return math.dist(first, second)


class Flight:
    def __init__(self, arguments: argparse.Namespace) -> None:
        self.arguments = arguments
        self.started = time.monotonic()
        self.records: list[dict[str, Any]] = []
        self.publish_errors: list[dict[str, Any]] = []
        self.stopping = False
        self.verdict = "FLIGHT_FAIL"

    def log(self, event: str, **values: Any) -> None:
        row = {
            "elapsed_s": round(time.monotonic() - self.started, 2),
            "event": event,
            **values,
        }
        self.records.append(row)
        print(json.dumps(row), flush=True)

    def guard(
        self, states: dict[str, dict[str, Any]], *, mission_running: bool = False
    ) -> None:
        separation = physical_separation_m(states)
        if separation is not None and separation < self.arguments.minimum_separation_m:
            raise FlightAbort(f"physical_separation:{separation:.3f}")
        for drone_id, state in states.items():
            if not state["online"]:
                raise FlightAbort(f"offline:{drone_id}")
            if state["failsafe"]:
                raise FlightAbort(f"px4_failsafe:{drone_id}")
            if state["sender"].get("latched_abort"):
                raise FlightAbort(
                    f"sender_latched:{drone_id}:{state['sender']['latched_abort']}"
                )
            if state["conditions"]:
                raise FlightAbort(
                    f"safety_conditions:{drone_id}:{state['conditions']}"
                )
            cbf_margin = state["cbf"].get("minimum_margin_m")
            if cbf_margin is not None and cbf_margin < 0.0:
                raise FlightAbort(f"cbf_margin:{drone_id}:{cbf_margin}")
            # The line above only sees the frames this poller asked about.
            # `cbf_margin_extrema` is accumulated at 20 Hz by the companion
            # and catches the breaches that fall between two polls.
            extrema = state["cbf_margin_extrema"]
            if (extrema.get("breach_frames") or 0) > 0:
                raise FlightAbort(
                    f"cbf_margin_between_samples:{drone_id}:"
                    f"{extrema.get('minimum_margin_m')}"
                )
            if not mission_running:
                continue
            if state["nav_state"] != OFFBOARD_NAV_STATE:
                raise FlightAbort(
                    f"left_offboard:{drone_id}:{state['nav_state']}"
                )
            if state["mission"].get("execution_state") not in {
                "entering",
                "running",
            }:
                raise FlightAbort(
                    f"mission_not_running:{drone_id}:{state['mission']}"
                )
            rl = state["cbf_rl"]
            if rl.get("mode") != self.arguments.expected_rl_mode:
                raise FlightAbort(f"rl_mode:{drone_id}:{rl.get('mode')}")
            if rl.get("model_sha256") != self.arguments.model_sha256:
                raise FlightAbort(
                    f"rl_hash:{drone_id}:{rl.get('model_sha256')}"
                )
            if not rl.get("model_loaded") or not rl.get("valid"):
                raise FlightAbort(
                    f"rl_invalid:{drone_id}:{rl.get('reason') or rl.get('load_error')}"
                )
            rl_margin = (rl.get("cbf") or {}).get("minimum_margin_m")
            if rl_margin is not None and rl_margin < 0.0:
                raise FlightAbort(f"rl_cbf_margin:{drone_id}:{rl_margin}")
            should_apply = self.arguments.expected_rl_mode == "active"
            if bool(rl.get("applied")) is not should_apply:
                raise FlightAbort(
                    f"rl_applied_contract:{drone_id}:{rl.get('applied')}"
                )

    async def wait_for(self, label: str, predicate: Any, timeout_s: float) -> Any:
        deadline = time.monotonic() + timeout_s
        last = None
        while time.monotonic() < deadline:
            if self.publish_errors:
                raise FlightAbort(f"publish_error:{self.publish_errors[-1]}")
            last = await asyncio.to_thread(fetch_states)
            self.guard(last)
            if predicate(last):
                self.log(label)
                return last
            await asyncio.sleep(0.5)
        raise FlightAbort(f"timeout:{label}:{last}")

    async def run(self) -> str:
        send_lock = asyncio.Lock()
        async with websockets.connect(WS_URL, max_size=8_000_000) as websocket:
            async def send(message: dict[str, Any]) -> None:
                async with send_lock:
                    await websocket.send(json.dumps(message))

            async def receive() -> None:
                try:
                    async for raw in websocket:
                        message = json.loads(raw)
                        if (
                            message.get("type") == "control_publish_result"
                            and not message.get("ok", False)
                        ):
                            self.publish_errors.append(message)
                except Exception as error:  # cleanup handles a lost socket
                    if not self.stopping:
                        self.publish_errors.append({"error": str(error)})

            async def neutral_manual_control() -> None:
                while not self.stopping:
                    for drone_id in DRONE_IDS:
                        await send(
                            {
                                "type": "manual_control",
                                "drone_id": drone_id,
                                "enabled": True,
                                "forward": 0.0,
                                "right": 0.0,
                                "up": 0.0,
                                "yaw": 0.0,
                            }
                        )
                    await asyncio.sleep(0.05)

            receiver = asyncio.create_task(receive())
            manual = asyncio.create_task(neutral_manual_control())
            try:
                states = await asyncio.to_thread(fetch_states)
                self.guard(states)
                if any(state["armed"] is not False for state in states.values()):
                    raise FlightAbort(f"precheck_state:{states}")
                # A previous cleanup may leave a disarmed PX4 reporting LAND
                # until Position is explicitly requested.  The UI uses the
                # same recovery action before enabling ARM.
                for drone_id in DRONE_IDS:
                    await send(
                        {
                            "type": "control_action",
                            "drone_id": drone_id,
                            "action": "position",
                        }
                    )
                states = await self.wait_for(
                    "PRECHECK_READY",
                    lambda value: all(
                        state["armed"] is False
                        and state["nav_state"] == POSITION_NAV_STATE
                        and state["preflight_checks_pass"] is True
                        for state in value.values()
                    ),
                    30.0,
                )
                self.log(
                    "PRECHECK_PASS",
                    separation_m=round(physical_separation_m(states) or 0.0, 3),
                )
                points = fit_to_spawns(
                    mission_waypoints_enu(
                        self.arguments.scenario,
                        center_enu_m=(
                            self.arguments.center_east_m,
                            self.arguments.center_north_m,
                        ),
                        radius_m=self.arguments.radius_m,
                    ),
                    states[DRONE_IDS[0]]["position_enu_m"],
                    states[DRONE_IDS[1]]["position_enu_m"],
                )
                self.log(
                    "SCENARIO_FITTED",
                    scenario=self.arguments.scenario,
                    spawn_separation_m=round(physical_separation_m(states) or 0.0, 3),
                    extent_m={
                        drone_id: round(
                            max(
                                math.dist(point[:2], legs[0][:2]) for point in legs
                            ),
                            1,
                        )
                        for drone_id, legs in points.items()
                    },
                )
                waypoints = enu_waypoints_to_geodetic(points)
                for drone_id in DRONE_IDS:
                    await send(
                        {
                            "type": "mission_path",
                            "drone_id": drone_id,
                            "altitude_m": self.arguments.altitude_m,
                            "speed_m_s": self.arguments.speed_m_s,
                            "waypoints": waypoints[drone_id],
                        }
                    )
                await self.wait_for(
                    "MISSIONS_INSTALLED",
                    lambda value: all(
                        state["mission"].get("state") == "installed"
                        for state in value.values()
                    ),
                    10.0,
                )
                for drone_id in DRONE_IDS:
                    await send(
                        {
                            "type": "control_action",
                            "drone_id": drone_id,
                            "action": "arm",
                        }
                    )
                await self.wait_for(
                    "ARMED",
                    lambda value: all(
                        state["armed"] is True for state in value.values()
                    ),
                    15.0,
                )
                for drone_id in DRONE_IDS:
                    await send(
                        {
                            "type": "control_action",
                            "drone_id": drone_id,
                            "action": "takeoff",
                            "altitude_m": self.arguments.altitude_m,
                        }
                    )
                await self.wait_for(
                    "MISSION_START_READY",
                    lambda value: all(
                        state["mission"].get("start_ready") is True
                        and (state["altitude_m"] or 0.0) > 7.0
                        for state in value.values()
                    ),
                    55.0,
                )
                for drone_id in DRONE_IDS:
                    await send({"type": "mission_start", "drone_id": drone_id})
                await self.wait_for(
                    "MISSIONS_RUNNING",
                    lambda value: all(
                        state["mission"].get("execution_state") == "running"
                        and state["nav_state"] == OFFBOARD_NAV_STATE
                        for state in value.values()
                    ),
                    15.0,
                )

                minimum_separation = math.inf
                minimum_cbf_margin = math.inf
                minimum_rl_margin = math.inf
                maximum_tracking_error = 0.0
                interventions = 0
                trajectories = {
                    drone_id: ClosedPolylineTrajectory(
                        tuple(points[drone_id]), self.arguments.speed_m_s
                    )
                    for drone_id in DRONE_IDS
                }
                previous_phase = {
                    drone_id: trajectories[drone_id].nearest_time_s(
                        tuple(states[drone_id]["position_enu_m"])
                    )
                    for drone_id in DRONE_IDS
                }
                progress_m = {drone_id: 0.0 for drone_id in DRONE_IDS}
                sample_count = 0
                deadline = time.monotonic() + self.arguments.hold_s
                next_report_s = 0.0
                while time.monotonic() < deadline:
                    states = await asyncio.to_thread(fetch_states)
                    self.guard(states, mission_running=True)
                    separation = physical_separation_m(states)
                    if separation is not None:
                        minimum_separation = min(minimum_separation, separation)
                    for drone_id, state in states.items():
                        margin = state["cbf_margin_extrema"].get("minimum_margin_m")
                        if margin is None:
                            margin = state["cbf"].get("minimum_margin_m")
                        if margin is not None:
                            minimum_cbf_margin = min(minimum_cbf_margin, margin)
                        rl_margin = (state["cbf_rl"].get("cbf") or {}).get(
                            "minimum_margin_m"
                        )
                        if rl_margin is not None:
                            minimum_rl_margin = min(minimum_rl_margin, rl_margin)
                        error = state["tracking_error_m"]
                        if error is not None:
                            maximum_tracking_error = max(
                                maximum_tracking_error, error
                            )
                        interventions += int(bool(state["intervened"]))
                        phase = trajectories[drone_id].nearest_time_s(
                            tuple(state["position_enu_m"])
                        )
                        lap_s = trajectories[drone_id].lap_duration_s()
                        delta = phase - previous_phase[drone_id]
                        if delta < -lap_s / 2.0:
                            delta += lap_s
                        elif delta > lap_s / 2.0:
                            delta -= lap_s
                        progress_m[drone_id] += delta * self.arguments.speed_m_s
                        previous_phase[drone_id] = phase
                    sample_count += 1
                    elapsed = self.arguments.hold_s - (deadline - time.monotonic())
                    if elapsed >= next_report_s:
                        self.log(
                            "MISSION_SAMPLE",
                            mission_elapsed_s=round(elapsed, 1),
                            separation_m=round(separation or 0.0, 3),
                            minimum_cbf_margin_m=round(minimum_cbf_margin, 3),
                            minimum_rl_margin_m=round(minimum_rl_margin, 3),
                        )
                        next_report_s += 5.0
                    await asyncio.sleep(0.5)
                self.verdict = "FLIGHT_PASS"
                self.log(
                    self.verdict,
                    samples=sample_count,
                    minimum_separation_m=round(minimum_separation, 3),
                    minimum_cbf_margin_m=round(minimum_cbf_margin, 3),
                    minimum_rl_margin_m=round(minimum_rl_margin, 3),
                    maximum_tracking_error_m=round(maximum_tracking_error, 3),
                    cbf_intervention_samples=interventions,
                    progress_m={
                        drone_id: round(value, 3)
                        for drone_id, value in progress_m.items()
                    },
                    target_progress_m=round(
                        self.arguments.hold_s * self.arguments.speed_m_s, 3
                    ),
                )
            except Exception as error:
                self.verdict = f"FLIGHT_ABORTED:{error}"
                self.log("ABORT", reason=str(error))
            finally:
                await self.cleanup(send)
                self.stopping = True
                manual.cancel()
                receiver.cancel()
                await asyncio.gather(manual, receiver, return_exceptions=True)
        return self.verdict

    async def cleanup(self, send: Any) -> None:
        for drone_id in DRONE_IDS:
            try:
                await send({"type": "mission_stop", "drone_id": drone_id})
            except Exception:
                pass
        await asyncio.sleep(2.0)
        for drone_id in DRONE_IDS:
            try:
                await send(
                    {
                        "type": "control_action",
                        "drone_id": drone_id,
                        "action": "land",
                    }
                )
            except Exception:
                pass
        deadline = time.monotonic() + 90.0
        states = None
        while time.monotonic() < deadline:
            try:
                states = await asyncio.to_thread(fetch_states)
                if all(state["armed"] is False for state in states.values()):
                    break
            except Exception:
                pass
            await asyncio.sleep(1.0)
        if states is None or not all(
            state["armed"] is False for state in states.values()
        ):
            for instance in ("0", "1"):
                subprocess.run(
                    [COMMANDER, "--instance", instance, "land"],
                    check=False,
                    capture_output=True,
                )
            await asyncio.sleep(8.0)
            states = await asyncio.to_thread(fetch_states)
        self.log(
            "FINAL_SAFE_STATE",
            armed={drone_id: state["armed"] for drone_id, state in states.items()},
            nav_state={
                drone_id: state["nav_state"] for drone_id, state in states.items()
            },
        )


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--expected-rl-mode", choices=("shadow", "active"), required=True)
    parser.add_argument(
        "--scenario",
        choices=(
            "circle",
            "diagonal_cross",
            "head_on_swap",
            "opposite_orbit",
            "repeated_bow_tie",
        ),
        default="circle",
    )
    parser.add_argument("--model-sha256", default=DEFAULT_MODEL_SHA256)
    parser.add_argument("--hold-s", type=float, default=45.0)
    parser.add_argument("--center-east-m", type=float, default=-1.5)
    parser.add_argument("--center-north-m", type=float, default=3.5)
    parser.add_argument("--radius-m", type=float, default=5.0)
    parser.add_argument("--altitude-m", type=float, default=9.0)
    parser.add_argument("--speed-m-s", type=float, default=1.0)
    parser.add_argument("--minimum-separation-m", type=float, default=4.0)
    parser.add_argument(
        "--output", default="artifacts/two_uav_mission_circle_flight.json"
    )
    return parser.parse_args()


def main() -> int:
    arguments = parse_arguments()
    flight = Flight(arguments)
    verdict = asyncio.run(flight.run())
    output = Path(arguments.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(
        json.dumps(
            {
                "verdict": verdict,
                "expected_rl_mode": arguments.expected_rl_mode,
                "model_sha256": arguments.model_sha256,
                "circle": {
                    "scenario": arguments.scenario,
                    "center_enu_m": [
                        arguments.center_east_m,
                        arguments.center_north_m,
                    ],
                    "radius_m": arguments.radius_m,
                    "altitude_m": arguments.altitude_m,
                    "speed_m_s": arguments.speed_m_s,
                },
                "records": flight.records,
            },
            indent=2,
        )
        + "\n",
        encoding="utf-8",
    )
    return 0 if verdict == "FLIGHT_PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
