#!/usr/bin/env python3
"""TWO_UAV_TRAJECTORY_TRACKING: fly UAV-01 AND UAV-02 on independent,
non-crossing trajectories at the same time.

Succeeds `two_uav_active_flight.py` (leader-hold + follower-to-slot) rather
than editing it: that script is TWO_UAV_ACTIVE_SITL_FLIGHT's own reviewed
record. This driver reuses its precheck/guard/abort shape but the vehicles
are no longer leader/follower -- both track a time-indexed trajectory
(`p_ref(t)`/`v_ref(t)`, see `trajectory_controller.py`), companion-side, in
place of formation/altitude-hold. Same downstream chain as before, unchanged
per this milestone's own roadmap: `u_nom -> local CBF -> EmergencySupervisor
-> PX4` (see companion_safety.py's trajectory wiring).

THIS SCRIPT DOES NOT CONFIGURE THE TRAJECTORIES ITSELF
--------------------------------------------------------
Trajectory selection is companion-side, environment-driven
(`SWARM_TRAJECTORY_<DRONE_ID>_KIND` and friends, read once when the stack
starts -- see companion_safety.py's `_trajectory_env`), exactly like
`SWARM_FORMATION_SLOT_UAV_02_ENU_M` already is. A flight driver process
cannot reach into an already-running worker's config. `DEFAULT_TRAJECTORY_ENV`
below is the pair this driver's guards/measurements assume; `--print-env`
prints it so an operator can diff it against their actual `.env` before
flying, and `precheck()` aborts early with a clear diagnosis rather than
silently holding station if the live system never entered trajectory mode.

WHY THESE TWO DEFAULT LEGS SPECIFICALLY
------------------------------------------
Both start at each vehicle's own real hover position (UAV-01 spawns (0,0),
UAV-02 spawns (-5,2) per run_all.sh -- see test_formation_spawn_geometry.py
for why that pair was chosen) and travel the same direction at the same
speed. Two parallel lines under equal-speed feed-forward tracking hold a
CONSTANT horizontal separation for the whole flight -- the offset at t=0 IS
the offset at every t, by construction, not by tuning. Simulated against the
real `CompanionSafetyMonitor` + trajectory wiring (not reimplemented math):
constant 5.385 m separation, min_cbf_margin_m 1.385 m throughout, both reach
"trajectory_reached". No crossing is possible with this pair, which is
exactly what this milestone's roadmap asks for before `ACTIVE_CBF_
CROSSING_TRAJECTORY` becomes the next, deliberately harder step.

WHY NO LEADER/FOLLOWER SEQUENCING IS REQUIRED ANYMORE
---------------------------------------------------------
`two_uav_active_flight.py` armed/took off/engaged the leader before the
follower because the follower's nominal command depended on the leader's
*live* position. A trajectory's nominal depends only on wall-clock-relative
mission time and its own vehicle's position -- never on a peer's state. Only
CBF (the safety layer, not the nominal) still reads peer state. UAV-01 is
still sequenced first below purely for deterministic, readable logs, not
because of any data dependency.

EACH RUN NEEDS A FRESHLY RESTARTED STACK
-------------------------------------------
The trajectory starts are ABSOLUTE shared-ENU coordinates, so they are only
valid from the spawn poses run_all.sh actually places the vehicles at. A
previous run that moved a vehicle -- even an aborted one -- leaves it parked
somewhere else, and it will take off from there next time. Measured: an
attempt that held OFFBOARD for ~1 s pushed UAV-01 2.05 m east, and the very
next run was correctly refused by the alignment gate below (2.047 m > 1.5 m)
before it could fly a trajectory it no longer started at. Restart the stack
between runs; the gate is what makes forgetting safe rather than silent.

MEASUREMENT, NOT JUST A GUARD
------------------------------
`nominal_position_error_m` (tracking error against `p_ref(t)`) is new
telemetry added for this milestone (see companion_safety.py's
`CompanionSafetyStatus.nominal_position_error_m`) -- neither the formation
nor the altitude-hold flights had anything to measure tracking error against.
It is reported, not guarded on: CBF is what's safety-critical here, same as
before, so `cbf_minimum_margin_m` keeps its abort-on-violation guard;
tracking error is instrumentation for judging PASS at the end.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import subprocess
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from two_uav_active_readiness import FlightEnvelope, PX4_FACTS, WarmupContract

API_URL = "http://127.0.0.1:8000/api/drones"
COMMANDER = (
    "/mnt/px4ssd/PX4-Autopilot/build/px4_sitl_default/bin/px4-commander"
)
UAV_01 = "UAV-01"
UAV_02 = "UAV-02"
DRONE_IDS: tuple[str, ...] = (UAV_01, UAV_02)
PX4_INSTANCE: dict[str, str] = {UAV_01: "0", UAV_02: "1"}

NAV_STATE_OFFBOARD = PX4_FACTS["nav_state_offboard"]["value"]
NAV_STATE_POSCTL = PX4_FACTS["nav_state_posctl"]["value"]

# "trajectory_entering" belongs here: it is what the companion reports while
# the vehicle flies toward a path it is further than trajectory_entry_radius_m
# from -- after a yield detour, or simply overshooting at cruise. It is a
# trajectory state, not the formation/altitude-hold fallback these checks
# exist to catch, and sparrow_corridor_replay.py:277 already reads it that
# way. Leaving it out cost the 15 m/s rung a flight: both vehicles crossed the
# entry radius back and forth every ~2.7 s, out of phase with each other, so
# the gate's demand that BOTH be tracking in the same 0.25 s poll almost never
# held and a longer timeout would not have helped. The fallback reasons
# ("formation_slot_unassigned", "trajectory_inactive",
# "trajectory_entry_state_invalid") still fail, which is the whole point.
TRAJECTORY_NOMINAL_REASONS = {
    "tracking_trajectory",
    "trajectory_reached",
    "trajectory_entering",
}

# What this driver's guards/measurements assume is configured in the live
# stack's .env before ./run_all.sh starts. Not enforced by this process --
# see the module docstring.
# The x/y starts are each vehicle's Gazebo spawn, which IS a shared-ENU
# coordinate here and not an assumption: .env's SWARM_ENU_ORIGIN_* equals the
# world's own <spherical_coordinates> block in default.sdf, with
# world_frame_orientation ENU. (.env.example documents why a mismatch there
# silently corrupts every separation, and how to re-derive it.)
#
# The z start is 9.0 because shared-ENU altitude is NOT the ~10 m the vehicle
# reports locally: measured altitude_reference_m was 8.973 (UAV-01) and 8.810
# (UAV-02) on the geometry-fix flight, and 9.452 on the trace flight -- the
# known GPS-vs-local-NED offset. 9.0 sits inside that measured spread; the
# alignment gate below is what confirms it per flight instead of trusting it.
DEFAULT_TRAJECTORY_ENV: dict[str, str] = {
    "SWARM_TRAJECTORY_UAV_01_KIND": "linear",
    "SWARM_TRAJECTORY_UAV_01_START_ENU_M": "0,0,9",
    "SWARM_TRAJECTORY_UAV_01_END_ENU_M": "20,0,9",
    "SWARM_TRAJECTORY_UAV_01_SPEED_M_S": "1.5",
    "SWARM_TRAJECTORY_UAV_02_KIND": "linear",
    "SWARM_TRAJECTORY_UAV_02_START_ENU_M": "-5,2,9",
    "SWARM_TRAJECTORY_UAV_02_END_ENU_M": "15,2,9",
    "SWARM_TRAJECTORY_UAV_02_SPEED_M_S": "1.5",
}


def configured_trajectory_env() -> dict[str, str]:
    return {
        key: os.environ.get(key, default)
        for key, default in DEFAULT_TRAJECTORY_ENV.items()
    }


# ENGINEERING GUARD, not a derived physical constant. Bounds chosen from
# measured numbers rather than picked round:
#
#   floor -- shared ENU altitude is GPS-derived while the driver's own
#     altitude_m is LOCAL_POSITION_NED, and the two disagree by a measured
#     ~0.91 m at hover (geometry-fix flight: altitude_reference_m 8.973 vs
#     entry_altitude_m 9.88; the 2026-08-10 trace flight root-caused this as
#     GPS/baro fusion lag, not a bug). Add hover wander and a limit below
#     ~1.2 m would abort healthy flights.
#   ceiling -- must stay far under the scale of a real frame error. The
#     vehicles spawn 5.39 m apart and a wrong ENU origin would be tens of
#     metres out, so 1.5 m still refuses those decisively.
#
# It deliberately permits brief velocity saturation (with k_p 0.6 and
# v_ref 1.5 m/s, saturation begins around 0.83 m of aligned error). That is
# acceptable: the cap is FlightEnvelope's own limit and CBF still bounds the
# result. This guard exists to catch a WRONG FRAME, not to enforce perfect
# alignment.
INITIAL_ERROR_LIMIT_DEFAULT_M = 1.5


class AltitudeTrendHoverGate:
    """Detect settled hover without trusting PX4's biased EKF `vz` signal."""

    def __init__(
        self,
        *,
        minimum_altitude_m: float = 7.0,
        window_s: float = 5.0,
        minimum_window_s: float = 4.0,
        maximum_slope_m_s: float = 0.15,
        maximum_spread_m: float = 0.40,
    ) -> None:
        self.minimum_altitude_m = minimum_altitude_m
        self.window_s = window_s
        self.minimum_window_s = minimum_window_s
        self.maximum_slope_m_s = maximum_slope_m_s
        self.maximum_spread_m = maximum_spread_m
        self._samples: list[tuple[float, float]] = []

    def reached(self, altitude_m: Any, *, now_s: float | None = None) -> bool:
        try:
            altitude = float(altitude_m)
        except (TypeError, ValueError):
            return False
        if not math.isfinite(altitude):
            return False
        now = time.monotonic() if now_s is None else float(now_s)
        if self._samples and now < self._samples[-1][0]:
            self._samples.clear()
        self._samples.append((now, altitude))
        cutoff = now - self.window_s
        self._samples = [sample for sample in self._samples if sample[0] >= cutoff]
        duration = self._samples[-1][0] - self._samples[0][0]
        if altitude <= self.minimum_altitude_m or duration < self.minimum_window_s:
            return False
        values = [sample[1] for sample in self._samples]
        slope = (values[-1] - values[0]) / duration
        return (
            abs(slope) <= self.maximum_slope_m_s
            and max(values) - min(values) <= self.maximum_spread_m
        )

    def __call__(self, state: dict[str, Any]) -> bool:
        return self.reached(state.get("altitude_m"))


# A hold can log clean numbers and still be hollow.  The 15 m/s corridor
# returned FLIGHT_PASS having touched 2.97 m/s of a commanded 14.76 and never
# reached its far end -- nothing gated on either number, so the rung went
# green without flying its case.  Separation on the measured numbers is wide:
# that hollow run sat at 0.20 of command while the polygon flight reached
# 1.03 and its parked partner 1.87, so half of command refuses the first
# without coming near the others.
HOLLOW_HOLD_SPEED_FRACTION = 0.5


def check_hold_was_flown(
    completed: dict[str, dict[str, Any]], trajectory_env: dict[str, str]
) -> list[str]:
    """Reasons the hold did not fly its case.  Empty means it did."""
    reasons = []
    for drone_id, flown in sorted(completed.items()):
        prefix = f"SWARM_TRAJECTORY_{drone_id.replace('-', '_')}"
        commanded = float(trajectory_env[f"{prefix}_SPEED_M_S"])
        reached = flown["max_horizontal_speed_m_s"]
        if reached is None or reached < HOLLOW_HOLD_SPEED_FRACTION * commanded:
            reasons.append(f"{drone_id} peaked at {reached} of {commanded} m/s")
        # A closed polyline has no end to reach: it laps until the hold runs
        # out, so only an open trajectory can be asked to finish.
        if trajectory_env[f"{prefix}_KIND"] == "linear" and not flown[
            "reached_trajectory_end"
        ]:
            reasons.append(f"{drone_id} never reached its trajectory end")
    return reasons


def initial_error_limit_m() -> float:
    """Operator override for the alignment limit, clamped to a sane band."""
    try:
        value = float(
            os.environ.get(
                "SWARM_TRAJECTORY_INITIAL_ERROR_MAX_M",
                str(INITIAL_ERROR_LIMIT_DEFAULT_M),
            )
        )
    except (TypeError, ValueError):
        return INITIAL_ERROR_LIMIT_DEFAULT_M
    if not math.isfinite(value) or value <= 0.0:
        return INITIAL_ERROR_LIMIT_DEFAULT_M
    return min(value, 50.0)


@dataclass(frozen=True)
class AlignmentResult:
    drone_id: str
    actual_start_enu_m: tuple[float, float, float] | None
    reference_start_enu_m: tuple[float, float, float] | None
    initial_tracking_error_m: float | None
    ok: bool
    reason: str

    def as_dict(self) -> dict[str, Any]:
        return {
            "drone_id": self.drone_id,
            "actual_start_enu": (
                [round(v, 3) for v in self.actual_start_enu_m]
                if self.actual_start_enu_m is not None
                else None
            ),
            "reference_start_enu": (
                [round(v, 3) for v in self.reference_start_enu_m]
                if self.reference_start_enu_m is not None
                else None
            ),
            "initial_tracking_error_m": (
                round(self.initial_tracking_error_m, 3)
                if self.initial_tracking_error_m is not None
                else None
            ),
            "ok": self.ok,
            "reason": self.reason,
        }


def _finite_vector3(value: Any) -> tuple[float, float, float] | None:
    if not isinstance(value, (list, tuple)) or len(value) != 3:
        return None
    try:
        vector = tuple(float(item) for item in value)
    except (TypeError, ValueError):
        return None
    return vector if all(math.isfinite(item) for item in vector) else None  # type: ignore[return-value]


def check_initial_frame_alignment(
    snapshot: dict[str, dict[str, Any]], *, limit_m: float
) -> tuple[tuple[AlignmentResult, ...], str | None]:
    """Compares where each vehicle actually is against where its trajectory
    says it starts, both in the SHARED ENU frame the trajectory controller
    itself consumes -- never LOCAL_POSITION_NED, whose origin is per-vehicle
    and not the frame any of this is expressed in.

    Pure and snapshot-driven so the decision is testable without a stack.
    Returns per-vehicle results plus an abort reason (None when all pass).

    Boundary semantics: an error EQUAL to the limit passes; only a strictly
    greater error fails.
    """
    results: list[AlignmentResult] = []
    failures: list[str] = []
    for drone_id in DRONE_IDS:
        state = snapshot.get(drone_id) or {}

        claimed = state.get("companion_drone_id")
        if claimed != drone_id:
            results.append(
                AlignmentResult(
                    drone_id, None, None, None, False,
                    f"identity_mismatch:{claimed!r}",
                )
            )
            failures.append(f"trajectory_identity_mismatch:{drone_id}:{claimed!r}")
            continue

        reference = _finite_vector3(state.get("trajectory_reference_start_enu_m"))
        if reference is None:
            results.append(
                AlignmentResult(drone_id, None, None, None, False, "trajectory_not_configured")
            )
            failures.append(f"trajectory_not_configured:{drone_id}")
            continue

        actual = _finite_vector3(state.get("own_position_enu_m"))
        if actual is None:
            results.append(
                AlignmentResult(
                    drone_id, None, reference, None, False,
                    "shared_enu_position_unavailable",
                )
            )
            failures.append(f"trajectory_initial_position_unavailable:{drone_id}")
            continue

        error = math.sqrt(sum((actual[i] - reference[i]) ** 2 for i in range(3)))
        within = error <= limit_m
        results.append(
            AlignmentResult(
                drone_id, actual, reference, error, within,
                "aligned" if within else "initial_error_exceeds_limit",
            )
        )
        if not within:
            failures.append(
                f"trajectory_initial_position_mismatch:{drone_id}:"
                f"{error:.3f}m>{limit_m:.3f}m"
            )

    return tuple(results), (",".join(failures) if failures else None)


class FlightAbort(Exception):
    """Raised on any failed verification. Always leads to POSCTL + land, both vehicles."""


class Flight:
    def __init__(self, hold_s: float, dry_run: bool) -> None:
        self.hold_s = hold_s
        self.dry_run = dry_run
        self.envelope = FlightEnvelope.from_environment()
        self.warmup = WarmupContract()
        self.records: list[dict[str, Any]] = []
        self.started = time.monotonic()
        self.commands: list[str] = []

    # -- telemetry --------------------------------------------------------

    def telemetry(self) -> dict[str, Any]:
        result = subprocess.run(
            ["curl", "-fsS", "-m", "5", API_URL],
            capture_output=True,
            text=True,
            check=False,
        )
        if result.returncode != 0:
            raise FlightAbort("telemetry_unavailable")
        return json.loads(result.stdout)

    def vehicle(
        self, drone_id: str, payload: dict[str, Any] | None = None
    ) -> dict[str, Any]:
        payload = payload or self.telemetry()
        drone = payload["drones"][drone_id]
        stream = payload["tracking_pose_streams"][drone_id]
        safety = stream.get("companion_safety") or {}
        local = drone.get("local_position") or {}
        cbf = safety.get("cbf") or {}
        return {
            "drone_id": drone_id,
            # Which vehicle the companion pipeline BELIEVES it is evaluating.
            # Compared against drone_id by the alignment gate: a stream
            # serving one vehicle's state under another's key would otherwise
            # let every other check pass against the wrong aircraft.
            "companion_drone_id": safety.get("drone_id"),
            "own_position_enu_m": safety.get("own_position_enu_m"),
            "trajectory_reference_start_enu_m": safety.get(
                "trajectory_reference_start_enu_m"
            ),
            "armed": drone["status"].get("armed"),
            "nav_state": drone["status"].get("nav_state"),
            "failsafe": drone["status"].get("failsafe"),
            "altitude_m": (
                None if local.get("z_down_m") is None else -float(local["z_down_m"])
            ),
            "vertical_velocity_m_s": (
                None if local.get("vz_m_s") is None else -float(local["vz_m_s"])
            ),
            "speed_m_s": (
                None
                if local.get("vx_m_s") is None
                else (local["vx_m_s"] ** 2 + local["vy_m_s"] ** 2) ** 0.5
            ),
            "offboard_signal_lost": (drone.get("failsafe_flags") or {}).get(
                "offboard_control_signal_lost"
            ),
            "sender": safety.get("active_offboard_sender") or {},
            "frame": safety.get("active_offboard_frame") or {},
            "conditions": safety.get("active_offboard_conditions") or [],
            "output_velocity_enu_m_s": safety.get("output_velocity_enu_m_s"),
            "nominal_reason": safety.get("nominal_reason"),
            "nominal_position_error_m": safety.get("nominal_position_error_m"),
            "station_keeping": safety.get("station_keeping"),
            "cbf_active": cbf.get("active"),
            "cbf_reason": cbf.get("reason"),
            "cbf_intervened": safety.get("intervened"),
            # Own measurement of the pair's separation margin -- the same
            # value CBF itself filters against (see two_uav_active_flight.py
            # for why this is not re-derived from local_position instead).
            "cbf_minimum_margin_m": cbf.get("minimum_margin_m"),
            "cbf_critical_peer_id": cbf.get("critical_peer_id"),
            "cbf_critical_distance_m": cbf.get("critical_distance_m"),
            "cbf_critical_required_separation_m": cbf.get(
                "critical_required_separation_m"
            ),
        }

    def snapshot(self) -> dict[str, dict[str, Any]]:
        payload = self.telemetry()
        return {drone_id: self.vehicle(drone_id, payload) for drone_id in DRONE_IDS}

    def log(self, step: str, **fields: Any) -> None:
        record = {"t_s": round(time.monotonic() - self.started, 2), "step": step, **fields}
        self.records.append(record)
        print(json.dumps(record), flush=True)

    # -- commands ---------------------------------------------------------

    def commander(self, drone_id: str, *arguments: str) -> str:
        command = [COMMANDER, "--instance", PX4_INSTANCE[drone_id], *arguments]
        self.commands.append(f"{drone_id}:" + " ".join(arguments))
        if self.dry_run:
            self.log("WOULD_COMMAND", drone_id=drone_id, command=" ".join(arguments))
            return ""
        result = subprocess.run(command, capture_output=True, text=True, check=False)
        self.log(
            "COMMAND", drone_id=drone_id, command=" ".join(arguments), rc=result.returncode
        )
        return result.stdout

    def await_vehicle_state(
        self,
        drone_id: str,
        description: str,
        predicate,
        timeout_s: float,
        poll_s: float = 0.5,
    ) -> dict[str, Any]:
        deadline = time.monotonic() + timeout_s
        last: dict[str, Any] = {}
        while time.monotonic() < deadline:
            snapshot = self.snapshot()
            self.guard(snapshot)
            last = snapshot[drone_id]
            if predicate(last):
                self.log("CONFIRMED", drone_id=drone_id, what=description, **self.brief(last))
                return last
            time.sleep(poll_s)
        if "hover" in description:
            # The overwhelmingly likely cause, and the one that cost three
            # days: PX4 ships MIS_TAKEOFF_ALT at 2.5 m while every driver
            # here waits for a ~10 m envelope, so the vehicle levels off
            # early and this reads as a dead vertical channel.
            raise FlightAbort(
                f"timeout_waiting_for:{drone_id}:{description}"
                " (check px4-param show MIS_TAKEOFF_ALT against the hover envelope)"
            )
        raise FlightAbort(f"timeout_waiting_for:{drone_id}:{description}")

    @staticmethod
    def brief(state: dict[str, Any]) -> dict[str, Any]:
        return {
            "armed": state["armed"],
            "nav_state": state["nav_state"],
            "alt_m": None if state["altitude_m"] is None else round(state["altitude_m"], 2),
            "decision": state["frame"].get("decision"),
            "tx": state["sender"].get("transmit_count"),
            "nominal_reason": state["nominal_reason"],
            "tracking_error_m": (
                None
                if state["nominal_position_error_m"] is None
                else round(state["nominal_position_error_m"], 2)
            ),
            "cbf_margin_m": (
                None
                if state["cbf_minimum_margin_m"] is None
                else round(state["cbf_minimum_margin_m"], 2)
            ),
            "cbf_distance_m": state["cbf_critical_distance_m"],
            "cbf_required_m": state["cbf_critical_required_separation_m"],
        }

    # -- continuous safety ------------------------------------------------

    def guard(self, snapshot: dict[str, dict[str, Any]]) -> None:
        """Same shape as two_uav_active_flight.py's guard: per-vehicle checks
        for both, joint CBF margin check. Tracking error is NOT guarded here
        -- CBF is the safety layer; tracking error is instrumentation."""
        if time.monotonic() - self.started > self.envelope.maximum_test_duration_s:
            raise FlightAbort("maximum_test_duration_exceeded")
        for drone_id, state in snapshot.items():
            if state["failsafe"]:
                raise FlightAbort(f"px4_failsafe:{drone_id}")
            latched = state["sender"].get("latched_abort")
            if latched:
                raise FlightAbort(f"sender_latched:{drone_id}:{latched}")
            altitude = state["altitude_m"]
            if altitude is not None and altitude > self.envelope.geofence_max_enu_m[2]:
                raise FlightAbort(f"geofence_altitude_exceeded:{drone_id}")
            speed = state["speed_m_s"]
            if (
                speed is not None
                and speed > self.envelope.maximum_horizontal_velocity_m_s + 0.5
            ):
                raise FlightAbort(f"horizontal_speed_exceeded:{drone_id}:{speed:.2f}")
        margins = [
            state["cbf_minimum_margin_m"]
            for state in snapshot.values()
            if state["cbf_minimum_margin_m"] is not None
        ]
        if margins and min(margins) < 0.0:
            raise FlightAbort(f"cbf_separation_violated:{min(margins):.2f}")

    def verify_initial_frame_alignment(self, snapshot: dict[str, dict[str, Any]]) -> None:
        """TRAJECTORY_INITIAL_FRAME_ALIGNMENT_VALIDATED.

        Reports every vehicle's numbers before deciding, so a refusal is
        diagnosable from the log alone rather than needing a re-run.
        """
        limit_m = initial_error_limit_m()
        results, failure = check_initial_frame_alignment(snapshot, limit_m=limit_m)
        self.log(
            "INITIAL_FRAME_ALIGNMENT",
            limit_m=limit_m,
            **{result.drone_id: result.as_dict() for result in results},
        )
        if failure:
            raise FlightAbort(failure)
        self.log("TRAJECTORY_INITIAL_FRAME_ALIGNMENT_VALIDATED", limit_m=limit_m)

    def await_trajectory_mode_engaged(self, timeout_s: float = 8.0) -> dict[str, dict[str, Any]]:
        """Trajectory mode is a TRANSITION to wait for, not an instantaneous
        property of having requested OFFBOARD.

        Measured on the first real attempt: this driver confirms OFFBOARD from
        `drones[].status.nav_state`, but the companion derives its own
        `station_keeping` -- which is what actually activates the trajectory
        nominal -- from the bridge worker's `last_px4_main_mode`, fed by a
        different MAVLink stream on its own cadence. UAV-01 had caught up
        0.52 s after its mode request and reported `tracking_trajectory`;
        UAV-02, checked 0.52 s after its own request, had not, and a
        single-shot check aborted a flight that was in fact healthy.

        Polling instead of sleeping keeps every guard live throughout, and a
        genuinely unconfigured stack still fails: its nominal_reason stays a
        formation one forever and the timeout fires. The stronger check
        against that -- a published trajectory reference -- has already run
        before OFFBOARD, in the alignment gate.
        """
        deadline = time.monotonic() + timeout_s
        snapshot: dict[str, dict[str, Any]] = {}
        while time.monotonic() < deadline:
            snapshot = self.snapshot()
            self.guard(snapshot)
            pending = [
                drone_id
                for drone_id, state in snapshot.items()
                if state["nominal_reason"] not in TRAJECTORY_NOMINAL_REASONS
            ]
            if not pending:
                return snapshot
            time.sleep(0.25)
        self.verify_trajectory_mode_engaged(snapshot)
        return snapshot

    def verify_trajectory_mode_engaged(self, snapshot: dict[str, dict[str, Any]]) -> None:
        """Checked once, right after both vehicles enter OFFBOARD. Companion-
        side trajectory selection is environment-driven and set BEFORE this
        process starts (see module docstring) -- if an operator forgot to set
        it, the live system silently falls back to formation/altitude-hold,
        which would otherwise produce a flight that "passes" while measuring
        nothing this milestone cares about."""
        misconfigured = [
            drone_id
            for drone_id, state in snapshot.items()
            if state["nominal_reason"] not in TRAJECTORY_NOMINAL_REASONS
        ]
        if misconfigured:
            raise FlightAbort(
                "trajectory_not_configured:" + ",".join(misconfigured)
                + " -- see DEFAULT_TRAJECTORY_ENV / --print-env"
            )

    # -- sequence -----------------------------------------------------------

    def precheck(self) -> None:
        snapshot = self.snapshot()
        failures: list[str] = []
        for drone_id, state in snapshot.items():
            sender = state["sender"]
            if state["armed"] is not False:
                failures.append(f"{drone_id}_not_disarmed")
            if state["offboard_signal_lost"] is not False:
                failures.append(f"{drone_id}_px4_does_not_see_the_setpoint_stream")
            if not sender.get("transmit_sink_attached"):
                failures.append(f"{drone_id}_no_transmit_sink")
            if not sender.get("explicit_opt_in"):
                failures.append(f"{drone_id}_no_explicit_opt_in")
            duration = sender.get("stream_duration_s")
            if duration is None or duration < self.warmup.minimum_duration_s:
                failures.append(f"{drone_id}_warmup_duration_insufficient:{duration}")
            if sender.get("transmit_count", 0) < self.warmup.minimum_valid_samples:
                failures.append(f"{drone_id}_warmup_samples_insufficient")
            if sender.get("max_transmit_gap_s", 9.9) > self.warmup.maximum_gap_s:
                failures.append(f"{drone_id}_warmup_gap:{sender.get('max_transmit_gap_s')}")
            if state["conditions"]:
                failures.append(f"{drone_id}_abort_conditions_active:{state['conditions']}")
        if failures:
            raise FlightAbort("precheck:" + ",".join(failures))
        self.log(
            "PRECHECK_PASS",
            **{drone_id: self.brief(state) for drone_id, state in snapshot.items()},
        )

    def run(self) -> str:
        self.log(
            "START",
            envelope_hover_m=self.envelope.hover_altitude_m,
            max_duration_s=self.envelope.maximum_test_duration_s,
            expected_trajectory_env=configured_trajectory_env(),
        )
        self.precheck()

        self.commander(UAV_01, "arm")
        self.await_vehicle_state(UAV_01, "armed", lambda s: s["armed"] is True, timeout_s=10.0)
        self.commander(UAV_02, "arm")
        self.await_vehicle_state(UAV_02, "armed", lambda s: s["armed"] is True, timeout_s=10.0)

        hover_gate = {
            UAV_01: AltitudeTrendHoverGate(),
            UAV_02: AltitudeTrendHoverGate(),
        }

        self.commander(UAV_01, "takeoff")
        self.await_vehicle_state(
            UAV_01, "hover_reached", hover_gate[UAV_01], timeout_s=45.0
        )
        self.commander(UAV_02, "takeoff")
        self.await_vehicle_state(
            UAV_02, "hover_reached", hover_gate[UAV_02], timeout_s=45.0
        )

        # THE GATE. Runs while BOTH vehicles hover under PX4's own POSCTL,
        # before either hands control to the companion. Checking after
        # OFFBOARD would be too late to be preventive: trajectory mode only
        # activates with station-keeping, so a mismatched p_ref(0) would
        # already be commanding a saturated velocity toward the wrong place
        # by the time the first sample arrived.
        #
        # Both vehicles are armed and hovering at this point, which cannot be
        # avoided -- on the ground their shared-ENU altitude is ~0 against a
        # ~9 m reference, so every check would fail spuriously. Arming alone
        # grants the companion nothing; OFFBOARD is where authority actually
        # transfers, and that is what this gate stands in front of, for BOTH
        # vehicles at once.
        self.verify_initial_frame_alignment(self.snapshot())

        self.commander(UAV_01, "mode", "offboard")
        self.await_vehicle_state(
            UAV_01,
            "offboard_engaged",
            lambda s: s["nav_state"] == NAV_STATE_OFFBOARD,
            timeout_s=10.0,
        )
        self.commander(UAV_02, "mode", "offboard")
        self.await_vehicle_state(
            UAV_02,
            "offboard_engaged",
            lambda s: s["nav_state"] == NAV_STATE_OFFBOARD,
            timeout_s=10.0,
        )

        entry = self.await_trajectory_mode_engaged()
        self.log(
            "TRAJECTORY_MODE_CONFIRMED",
            **{drone_id: self.brief(state) for drone_id, state in entry.items()},
        )

        deadline = time.monotonic() + self.hold_s
        samples: dict[str, list[dict[str, Any]]] = {drone_id: [] for drone_id in DRONE_IDS}
        while time.monotonic() < deadline:
            snapshot = self.snapshot()
            self.guard(snapshot)
            for drone_id, state in snapshot.items():
                if state["nav_state"] != NAV_STATE_OFFBOARD:
                    raise FlightAbort(f"px4_exited_offboard_unexpectedly:{drone_id}")
                samples[drone_id].append(state)
            time.sleep(0.5)

        completed: dict[str, dict[str, Any]] = {}
        for drone_id in DRONE_IDS:
            drone_samples = samples[drone_id]
            errors = [
                s["nominal_position_error_m"]
                for s in drone_samples
                if s["nominal_position_error_m"] is not None
            ]
            speeds = [s["speed_m_s"] for s in drone_samples if s["speed_m_s"] is not None]
            margins = [
                s["cbf_minimum_margin_m"]
                for s in drone_samples
                if s["cbf_minimum_margin_m"] is not None
            ]
            interventions = sum(1 for s in drone_samples if s["cbf_intervened"])
            completed[drone_id] = dict(
                seconds=self.hold_s,
                max_tracking_error_m=round(max(errors), 3) if errors else None,
                final_tracking_error_m=round(errors[-1], 3) if errors else None,
                max_horizontal_speed_m_s=round(max(speeds), 3) if speeds else None,
                min_cbf_margin_m=round(min(margins), 3) if margins else None,
                cbf_intervention_rate=(
                    round(interventions / len(drone_samples), 3) if drone_samples else None
                ),
                total_samples=len(drone_samples),
                nominal_reasons=sorted({str(s["nominal_reason"]) for s in drone_samples}),
                reached_trajectory_end=any(
                    s["nominal_reason"] == "trajectory_reached" for s in drone_samples
                ),
            )
            self.log("TRAJECTORY_HOLD_COMPLETE", drone_id=drone_id, **completed[drone_id])

        # Logged first, so the evidence survives whichever way this goes.
        hollow = check_hold_was_flown(completed, configured_trajectory_env())
        if hollow:
            raise FlightAbort("hold_did_not_fly_the_case:" + "; ".join(hollow))

        self.commander(UAV_01, "mode", "posctl")
        self.await_vehicle_state(
            UAV_01,
            "posctl_restored",
            lambda s: s["nav_state"] == NAV_STATE_POSCTL,
            timeout_s=10.0,
        )
        self.commander(UAV_02, "mode", "posctl")
        self.await_vehicle_state(
            UAV_02,
            "posctl_restored",
            lambda s: s["nav_state"] == NAV_STATE_POSCTL,
            timeout_s=10.0,
        )

        # Start both descents before waiting for either disarm.  Sequential
        # landing can leave one vehicle moving while the other is parked and
        # consumed the remaining separation in an aborted conflict run.
        for drone_id in DRONE_IDS:
            self.commander(drone_id, "land")
        for drone_id in DRONE_IDS:
            self.await_vehicle_state(
                drone_id,
                "disarmed",
                lambda s: s["armed"] is False,
                timeout_s=90.0,
            )
        return "FLIGHT_PASS"

    def abort(self, reason: str) -> None:
        self.log("ABORT", reason=reason)
        for drone_id in DRONE_IDS:
            try:
                self.commander(drone_id, "mode", "posctl")
            except Exception as error:  # noqa: BLE001 - abort must not raise
                self.log("ABORT_INCOMPLETE", drone_id=drone_id, stage="posctl", error=str(error))
        time.sleep(1.0)
        for drone_id in DRONE_IDS:
            try:
                state = self.vehicle(drone_id)
                if state["armed"]:
                    self.commander(drone_id, "land")
            except Exception as error:  # noqa: BLE001 - abort must not raise
                self.log("ABORT_INCOMPLETE", drone_id=drone_id, stage="land", error=str(error))
        self.log(
            "ABORT_DONE",
            note="POSCTL is the strongest implemented action; a human must take it from here",
        )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--hold-s", type=float, default=20.0)
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--output", default="artifacts/two_uav_trajectory_flight.json")
    parser.add_argument(
        "--print-env",
        action="store_true",
        help="Print the .env block this driver's guards assume, then exit.",
    )
    arguments = parser.parse_args()

    if arguments.print_env:
        for key, value in configured_trajectory_env().items():
            print(f"{key}={value}")
        return 0

    flight = Flight(hold_s=arguments.hold_s, dry_run=arguments.dry_run)
    verdict = "FLIGHT_FAIL"
    try:
        verdict = flight.run()
    except FlightAbort as error:
        flight.abort(str(error))
        verdict = f"FLIGHT_ABORTED:{error}"
    except KeyboardInterrupt:
        flight.abort("operator_interrupt")
        verdict = "FLIGHT_ABORTED:operator_interrupt"

    try:
        final = {drone_id: flight.vehicle(drone_id) for drone_id in DRONE_IDS}
        final_state = {drone_id: Flight.brief(state) for drone_id, state in final.items()}
        sender = {drone_id: state["sender"] for drone_id, state in final.items()}
    except FlightAbort as error:
        final_state = {"error": str(error)}
        sender = {}
    result = {
        "verdict": verdict,
        "records": flight.records,
        "commands_issued": flight.commands,
        "final_state": final_state,
        "sender": sender,
        "expected_trajectory_env": configured_trajectory_env(),
    }
    output = Path(arguments.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    print()
    print(
        json.dumps(
            {"verdict": verdict, "final": result["final_state"]},
            indent=2,
        )
    )
    return 0 if verdict == "FLIGHT_PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
