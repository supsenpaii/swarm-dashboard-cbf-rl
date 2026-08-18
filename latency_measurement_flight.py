#!/usr/bin/env python3
"""LATENCY_MEASUREMENT_FLIGHT: excite UAV-02 with a repeated velocity square
wave while hovering, so `CbfConfig.command_latency_s` -- the CBF margin
formula's assumed command-to-motion delay, defaulted to an unvalidated
0.10 s and never wired to a trustworthy measurement -- can finally be fit
against real PX4 tracking dynamics instead of guessed or inferred after the
fact from telemetry that was never designed to expose it.

WHY THIS FLIGHT EXISTS
-----------------------
`ACTIVE_CBF_CROSSING_TRAJECTORY` failed real flight twice (2026-08-10), both
times on a `cbf_minimum_margin_m < 0` abort during a hard CBF correction,
despite an idealized simulation (position = integral of COMMANDED velocity,
no PX4 tracking lag) predicting a comfortable positive margin both times.
Two follow-up investigations closed off the easy explanations: a wider
geometry sweep showed the simulated margin has a structural ceiling
(~0.14-0.15 m whenever CBF intervention is real) that is *below* the real
excess measured twice (~0.16-0.19 m) -- no geometry in the swept space fixes
this. A first-order lag model fit with tau=0.5s (estimated from the two
crossing flights' own noisy traces) did not reproduce the real margin drop
either -- that guess was rejected by data, not assumed correct.

Reading `cbf_command_gate.py`'s own `age_latency` term directly found the
actual gap: `command_latency_s` (default 0.10, wired to
`SWARM_CBF_COMMAND_LATENCY_S` but never measured) contributes 0.29-0.36 m to
`required_margin` at the exact frames the real flights went negative --
comparable to or larger than the entire real-vs-simulated discrepancy. Two
independent system-ID attempts on EXISTING telemetry (point-by-point lag fit
on the smooth parallel-trajectory flight, cross-correlation on both flights)
disagreed by more than 10x (0.05 s to 0.65 s) depending on axis/dataset --
neither dataset was designed to excite this dynamic, so neither estimate is
trustworthy. Picking a number that makes the crossing margin come out
positive would be reasoning backward from the desired answer, the same
mistake already ruled out for `minimum_separation_m` and `barrier_gain_s_inv`
earlier in this same investigation.

This flight is the honest fix: a scheduled step input, known exactly
independent of any tracking dynamics, repeated many times so the fit can be
robust (median across edges) instead of trusting one noisy correlation.

THIS SCRIPT DOES NOT CONFIGURE THE TRAJECTORY ITSELF
--------------------------------------------------------
Same reasoning as `two_uav_crossing_trajectory_flight.py`: trajectory
selection is companion-side and environment-driven
(`SWARM_TRAJECTORY_UAV_02_KIND=square_wave` and friends), read once when the
stack starts. `DEFAULT_TRAJECTORY_ENV` below is what this driver's
alignment gate and geometry assume; `--print-env` prints it for an operator
to diff against `.env`, and `precheck()`/the alignment gate abort early with
a clear diagnosis rather than silently measuring nothing.

WHY UAV-01 STAYS STATIC AND UAV-02 IS THE ONLY ONE EXCITED
-----------------------------------------------------------
CBF requires both vehicles present with valid state (see `CbfCommandGate`
peer requirements) or the gate holds at zero -- so UAV-01 must fly too, but
it needs no trajectory of its own: with no formation slot and no trajectory
configured, `companion_safety.py`'s leader branch gives it altitude-hold and
zero horizontal velocity automatically, exactly like every prior two-vehicle
flight's leader. UAV-02 alone carries the excitation signal.

WHY THE SQUARE WAVE POINTS AWAY FROM UAV-01, NOT ACROSS ITS PATH
-------------------------------------------------------------------
This flight measures PX4's own step response, not CBF -- CBF intervening
would inject exactly the confound this flight exists to avoid (a corrected,
not open-loop, velocity). `step_velocity_enu_m_s` points further away from
UAV-01's spawn (due west, same axis UAV-02's own spawn already sits along),
so separation only grows during the excited half and never drops below the
5.385 m spawn separation. Simulated with the real
`TrajectoryTrackingController` + `CbfCommandGate` (not reimplemented math,
same practice as every prior geometry decision in this project):
`test_latency_measurement_geometry.py` pins zero CBF intervention and a
minimum margin >=1.0 m throughout for `DEFAULT_TRAJECTORY_ENV`'s numbers --
CBF should never fire once during this flight; if it does, the trace's
`cbf_intervened`/`cbf_minimum_margin_m` fields make that visible rather than
silently corrupting the fit.

WHY A SQUARE WAVE AND NOT A SINGLE STEP
-------------------------------------------
A single step buys one edge per flight. `SquareWaveVelocityTrajectory`
never finishes and never drifts past one period's amplitude from its start
(see its own docstring) -- many edges fit inside one small, bounded hover
volume, so the post-flight fit can take a median across edges instead of
trusting whichever one flight happened to capture.

WHAT TO DO WITH THE RESULT
------------------------------
This driver only flies and records. It does not fit `command_latency_s` --
that belongs in a separate offline analysis over
`SWARM_COMPANION_SAFETY_LOG`'s JSONL trace (`own_velocity_enu_m_s`, the real
measured response, against the known scheduled step schedule -- see
`SquareWaveVelocityTrajectory.reference`), so the fit can be inspected,
re-run, and reasoned about before anyone edits `CbfConfig.command_latency_s`
on its strength. Start the stack with `SWARM_COMPANION_SAFETY_LOG` set to a
path before running this driver, or there is nothing to fit against
afterward.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import subprocess
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from two_uav_active_readiness import (
    FlightEnvelope,
    PX4_FACTS,
    WarmupContract,
    extrema_minimum_margin_m,
)

API_URL = "http://127.0.0.1:8000/api/drones"
COMMANDER = (
    "/mnt/px4ssd/PX4-Autopilot/build/px4_sitl_default/bin/px4-commander"
)
UAV_01 = "UAV-01"
UAV_02 = "UAV-02"
DRONE_IDS: tuple[str, ...] = (UAV_01, UAV_02)
PX4_INSTANCE: dict[str, str] = {UAV_01: "0", UAV_02: "1"}
EXCITED_DRONE = UAV_02

NAV_STATE_OFFBOARD = PX4_FACTS["nav_state_offboard"]["value"]
NAV_STATE_POSCTL = PX4_FACTS["nav_state_posctl"]["value"]

TRAJECTORY_NOMINAL_REASONS = {"tracking_trajectory", "trajectory_reached"}

# Only UAV-02 carries a trajectory -- see module docstring for why UAV-01
# stays on the ordinary leader (altitude-hold) path with none configured.
# start/step numbers verified safe by test_latency_measurement_geometry.py
# (real TrajectoryTrackingController + CbfCommandGate, zero intervention,
# minimum margin >=1.0 m). z=9.0 for the same measured GPS-vs-local-NED
# reason two_uav_trajectory_flight.py's DEFAULT_TRAJECTORY_ENV documents.
DEFAULT_TRAJECTORY_ENV: dict[str, str] = {
    "SWARM_TRAJECTORY_UAV_02_KIND": "square_wave",
    "SWARM_TRAJECTORY_UAV_02_START_ENU_M": "-5,2,9",
    "SWARM_TRAJECTORY_UAV_02_STEP_VELOCITY_ENU_M_S": "-1.5,0,0",
    "SWARM_TRAJECTORY_UAV_02_HALF_PERIOD_S": "2.5",
}


# Same reasoning and same value as two_uav_trajectory_flight.py's
# INITIAL_ERROR_LIMIT_DEFAULT_M -- see that module for the measured numbers
# behind it. Duplicated rather than imported: driver files in this project
# are deliberately self-contained (see two_uav_active_flight.py's PX4_INSTANCE
# comment for the same convention applied to a different constant).
INITIAL_ERROR_LIMIT_DEFAULT_M = 1.5


def initial_error_limit_m() -> float:
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
    actual_start_enu_m: tuple[float, float, float] | None
    reference_start_enu_m: tuple[float, float, float] | None
    initial_tracking_error_m: float | None
    ok: bool
    reason: str

    def as_dict(self) -> dict[str, Any]:
        return {
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
    state: dict[str, Any], *, limit_m: float
) -> AlignmentResult:
    """Only UAV-02 carries a trajectory here, so only UAV-02 needs this gate
    -- see two_uav_trajectory_flight.py's version of this check for the full
    reasoning (shared-ENU vs. LOCAL_POSITION_NED, why this must run before
    OFFBOARD not after)."""
    claimed = state.get("companion_drone_id")
    if claimed != EXCITED_DRONE:
        return AlignmentResult(None, None, None, False, f"identity_mismatch:{claimed!r}")

    reference = _finite_vector3(state.get("trajectory_reference_start_enu_m"))
    if reference is None:
        return AlignmentResult(None, None, None, False, "trajectory_not_configured")

    actual = _finite_vector3(state.get("own_position_enu_m"))
    if actual is None:
        return AlignmentResult(None, reference, None, False, "shared_enu_position_unavailable")

    error = math.sqrt(sum((actual[i] - reference[i]) ** 2 for i in range(3)))
    within = error <= limit_m
    return AlignmentResult(
        actual, reference, error, within, "aligned" if within else "initial_error_exceeds_limit"
    )


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
        extrema = safety.get("cbf_margin_extrema") or {}
        return {
            "drone_id": drone_id,
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
            "cbf_intervention_norm_m_s": cbf.get("intervention_norm_m_s"),
            "cbf_minimum_margin_m": cbf.get("minimum_margin_m"),
            # The line above is one 20 Hz frame sampled at the poll rate; the
            # two below are every frame, accumulated by the companion itself.
            # Guard and report from these -- see companion_safety's
            # _accumulate_margin for the flights that proved why.
            "cbf_extrema_minimum_margin_m": extrema.get("minimum_margin_m"),
            "cbf_extrema_breach_frames": extrema.get("breach_frames") or 0,
            "cbf_extrema_frames": extrema.get("frames") or 0,
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
        raise FlightAbort(f"timeout_waiting_for:{drone_id}:{description}")

    @staticmethod
    def brief(state: dict[str, Any]) -> dict[str, Any]:
        return {
            "armed": state["armed"],
            "nav_state": state["nav_state"],
            "alt_m": None if state["altitude_m"] is None else round(state["altitude_m"], 2),
            "decision": state["frame"].get("decision"),
            "nominal_reason": state["nominal_reason"],
            "cbf_margin_m": (
                None
                if state["cbf_minimum_margin_m"] is None
                else round(state["cbf_minimum_margin_m"], 2)
            ),
        }

    # -- continuous safety ------------------------------------------------

    def guard(self, snapshot: dict[str, dict[str, Any]]) -> None:
        """Same shape as two_uav_trajectory_flight.py's guard: per-vehicle
        checks for both, joint CBF margin check. CBF intervening at all is
        NOT guarded here -- test_latency_measurement_geometry.py already
        established it should never happen for DEFAULT_TRAJECTORY_ENV, so if
        it does, that is data about a bad assumption, not a safety violation;
        the trace makes it visible for the post-flight fit to discard."""
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
        breached = {
            drone_id: state["cbf_extrema_minimum_margin_m"]
            for drone_id, state in snapshot.items()
            if state["cbf_extrema_breach_frames"] > 0
        }
        if breached:
            worst = min(breached.items(), key=lambda item: item[1])
            raise FlightAbort(
                f"cbf_separation_violated_between_samples:{worst[0]}:{worst[1]:.2f}"
            )

    def verify_initial_frame_alignment(self, snapshot: dict[str, dict[str, Any]]) -> None:
        limit_m = initial_error_limit_m()
        result = check_initial_frame_alignment(snapshot[EXCITED_DRONE], limit_m=limit_m)
        self.log("INITIAL_FRAME_ALIGNMENT", limit_m=limit_m, **{EXCITED_DRONE: result.as_dict()})
        if not result.ok:
            raise FlightAbort(
                f"trajectory_initial_position_mismatch:{EXCITED_DRONE}:{result.reason}"
            )
        self.log("TRAJECTORY_INITIAL_FRAME_ALIGNMENT_VALIDATED", limit_m=limit_m)

    def await_trajectory_mode_engaged(self, timeout_s: float = 8.0) -> dict[str, dict[str, Any]]:
        """See two_uav_trajectory_flight.py's version for why this must poll
        a transition rather than sample `nominal_reason` once."""
        deadline = time.monotonic() + timeout_s
        snapshot: dict[str, dict[str, Any]] = {}
        while time.monotonic() < deadline:
            snapshot = self.snapshot()
            self.guard(snapshot)
            if snapshot[EXCITED_DRONE]["nominal_reason"] in TRAJECTORY_NOMINAL_REASONS:
                return snapshot
            time.sleep(0.25)
        misconfigured = snapshot.get(EXCITED_DRONE, {}).get("nominal_reason")
        raise FlightAbort(
            f"trajectory_not_configured:{EXCITED_DRONE}:{misconfigured!r}"
            " -- see DEFAULT_TRAJECTORY_ENV / --print-env"
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
            expected_trajectory_env=DEFAULT_TRAJECTORY_ENV,
        )
        self.precheck()

        self.commander(UAV_01, "arm")
        self.await_vehicle_state(UAV_01, "armed", lambda s: s["armed"] is True, timeout_s=10.0)
        self.commander(UAV_02, "arm")
        self.await_vehicle_state(UAV_02, "armed", lambda s: s["armed"] is True, timeout_s=10.0)

        def hover_reached(state: dict[str, Any]) -> bool:
            return (
                state["altitude_m"] is not None
                and state["altitude_m"] > 7.0
                and abs(state["vertical_velocity_m_s"] or 9.9) < 0.3
            )

        self.commander(UAV_01, "takeoff")
        self.await_vehicle_state(UAV_01, "hover_reached", hover_reached, timeout_s=45.0)
        self.commander(UAV_02, "takeoff")
        self.await_vehicle_state(UAV_02, "hover_reached", hover_reached, timeout_s=45.0)

        # Before either vehicle hands control to the companion -- see
        # two_uav_trajectory_flight.py's run() for why this must be here and
        # not after OFFBOARD.
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

        # THE MEASUREMENT. Coarse (0.5 s) HTTP polling here is only for the
        # safety guard and the summary below -- it cannot resolve a
        # sub-second step response. The actual fit reads
        # SWARM_COMPANION_SAFETY_LOG's per-frame JSONL trace, written by the
        # bridge itself at its own evaluation cadence; see module docstring.
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

        for drone_id in DRONE_IDS:
            drone_samples = samples[drone_id]
            speeds = [s["speed_m_s"] for s in drone_samples if s["speed_m_s"] is not None]
            margins = [
                s["cbf_minimum_margin_m"]
                for s in drone_samples
                if s["cbf_minimum_margin_m"] is not None
            ]
            interventions = sum(1 for s in drone_samples if s["cbf_intervened"])
            self.log(
                "HOLD_COMPLETE",
                drone_id=drone_id,
                seconds=self.hold_s,
                max_horizontal_speed_m_s=round(max(speeds), 3) if speeds else None,
                min_cbf_margin_m=extrema_minimum_margin_m(drone_samples),
                min_cbf_margin_sampled_m=round(min(margins), 3) if margins else None,
                cbf_intervention_rate=(
                    round(interventions / len(drone_samples), 3) if drone_samples else None
                ),
                total_samples=len(drone_samples),
                nominal_reasons=sorted({str(s["nominal_reason"]) for s in drone_samples}),
            )

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

        self.commander(UAV_01, "land")
        self.await_vehicle_state(
            UAV_01, "disarmed", lambda s: s["armed"] is False, timeout_s=90.0
        )
        self.commander(UAV_02, "land")
        self.await_vehicle_state(
            UAV_02, "disarmed", lambda s: s["armed"] is False, timeout_s=90.0
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
    parser.add_argument("--hold-s", type=float, default=40.0)
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--output", default="artifacts/latency_measurement_flight.json")
    parser.add_argument(
        "--print-env",
        action="store_true",
        help="Print the .env block this driver's guards assume, then exit.",
    )
    arguments = parser.parse_args()

    if arguments.print_env:
        for key, value in DEFAULT_TRAJECTORY_ENV.items():
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
        "expected_trajectory_env": DEFAULT_TRAJECTORY_ENV,
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
