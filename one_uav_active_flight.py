#!/usr/bin/env python3
"""ONE_UAV_ACTIVE_SITL_FLIGHT: first armed OFFBOARD flight of UAV-01.

What this driver does and, more importantly, what it does not.

It sends NO setpoints. Every setpoint in this flight comes from the
companion's own pipeline (nominal -> CBF -> EmergencySupervisor ->
validate_command -> ShadowOffboardSetpointSender.preview ->
ActiveOffboardSetpointSender), which is already running in the bridge at
20 Hz. This driver only issues mode and arming commands through PX4's own
console client, so no arming code exists anywhere in this repository and the
companion remains strictly a setpoint writer.

WHY PX4 PERFORMS THE ASCENT
---------------------------
UAV-01 is the formation leader and therefore has no slot: its nominal
controller reports `formation_slot_unassigned` and outputs zero velocity.
There is no climb command anywhere in the companion pipeline, so a
companion-driven ascent is not merely untested, it is not expressible.
PX4's own AUTO takeoff lifts the vehicle to MIS_TAKEOFF_ALT, and OFFBOARD is
handed over at a stable hover.

MIS_TAKEOFF_ALT IS NOT 9 m
--------------------------
This docstring used to claim 9.0 m and call it an unchanged firmware default.
Both halves were wrong: PX4 ships 2.5 m, and the value on a given instance is
whatever was last saved to its parameter file. Every driver here waits for a
~10 m hover envelope, so on a stock instance the vehicle levels off at 2.5 m
and the driver times out with `timeout_waiting_for:hover_reached` -- which
looks exactly like a broken vertical channel and was read as one for three
days, including the 2026-08-12 "vertical channel blocker". The vertical axis
was fine the whole time; on Sparrow it measures gain 1.060, tau 0.275 s.

Check it before blaming the aircraft:

    px4-param --instance 0 show MIS_TAKEOFF_ALT
    px4-param --instance 0 set MIS_TAKEOFF_ALT 10.0

That makes the flight a test of the thing actually in question: whether the
companion's setpoint stream can HOLD an armed, airborne vehicle in OFFBOARD.
Zero velocity is the correct command for a leader with no slot, and holding
station on it is a real result, not a degenerate one.

ABORT
-----
Any failed verification, any abort condition reported by the sender, or the
global timeout drops the vehicle to POSCTL and lands it. POSCTL is the
strongest action this project has -- there is no autonomous RTL anywhere in
it -- so a human retains the final say, exactly as ABORT_MATRIX records.
"""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
import time
from pathlib import Path
from typing import Any

from one_uav_active_readiness import (
    FIRST_ACTIVE_FLIGHT_DRONE_ID,
    FIRST_ACTIVE_FLIGHT_SYSTEM_ID,
    PX4_FACTS,
    FlightEnvelope,
    WarmupContract,
)

API_URL = "http://127.0.0.1:8000/api/drones"
COMMANDER = (
    "/mnt/px4ssd/PX4-Autopilot/build/px4_sitl_default/bin/px4-commander"
)
FLIGHT_INSTANCE = "0"  # PX4 -i 0 is UAV-01; -i 1 is UAV-02
SECOND_VEHICLE = "UAV-02"

NAV_STATE_OFFBOARD = PX4_FACTS["nav_state_offboard"]["value"]
NAV_STATE_POSCTL = PX4_FACTS["nav_state_posctl"]["value"]


class FlightAbort(Exception):
    """Raised on any failed verification. Always leads to POSCTL + land."""


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

    def vehicle(self, payload: dict[str, Any] | None = None) -> dict[str, Any]:
        payload = payload or self.telemetry()
        drone = payload["drones"][FIRST_ACTIVE_FLIGHT_DRONE_ID]
        stream = payload["tracking_pose_streams"][FIRST_ACTIVE_FLIGHT_DRONE_ID]
        safety = stream.get("companion_safety") or {}
        local = drone.get("local_position") or {}
        return {
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
            "station_keeping": safety.get("station_keeping"),
            "altitude_reference_m": safety.get("altitude_reference_m"),
            "second_vehicle_armed": payload["drones"][SECOND_VEHICLE]["status"].get(
                "armed"
            ),
        }

    def log(self, step: str, **fields: Any) -> None:
        record = {"t_s": round(time.monotonic() - self.started, 2), "step": step, **fields}
        self.records.append(record)
        print(json.dumps(record), flush=True)

    # -- commands ---------------------------------------------------------

    def commander(self, *arguments: str) -> str:
        """PX4's own console client. The only actuation path in this flight."""
        command = [COMMANDER, "--instance", FLIGHT_INSTANCE, *arguments]
        self.commands.append(" ".join(arguments))
        if self.dry_run:
            self.log("WOULD_COMMAND", command=" ".join(arguments))
            return ""
        result = subprocess.run(command, capture_output=True, text=True, check=False)
        self.log("COMMAND", command=" ".join(arguments), rc=result.returncode)
        return result.stdout

    def await_state(
        self, description: str, predicate, timeout_s: float, poll_s: float = 0.5
    ) -> dict[str, Any]:
        deadline = time.monotonic() + timeout_s
        last: dict[str, Any] = {}
        while time.monotonic() < deadline:
            last = self.vehicle()
            self.guard(last)
            if predicate(last):
                self.log("CONFIRMED", what=description, **self.brief(last))
                return last
            time.sleep(poll_s)
        if "hover" in description:
            # The overwhelmingly likely cause, and the one that cost three
            # days: PX4 ships MIS_TAKEOFF_ALT at 2.5 m while every driver
            # here waits for a ~10 m envelope, so the vehicle levels off
            # early and this reads as a dead vertical channel.
            raise FlightAbort(
                f"timeout_waiting_for:{description}"
                " (check px4-param show MIS_TAKEOFF_ALT against the hover envelope)"
            )
        raise FlightAbort(f"timeout_waiting_for:{description}")

    @staticmethod
    def brief(state: dict[str, Any]) -> dict[str, Any]:
        return {
            "armed": state["armed"],
            "nav_state": state["nav_state"],
            "alt_m": None if state["altitude_m"] is None else round(state["altitude_m"], 2),
            "decision": state["frame"].get("decision"),
            "tx": state["sender"].get("transmit_count"),
        }

    # -- continuous safety ------------------------------------------------

    def guard(self, state: dict[str, Any]) -> None:
        """Checked on every poll, not only at transitions."""
        if time.monotonic() - self.started > self.envelope.maximum_test_duration_s:
            raise FlightAbort("maximum_test_duration_exceeded")
        if state["second_vehicle_armed"]:
            raise FlightAbort("second_vehicle_armed")
        if state["failsafe"]:
            raise FlightAbort("px4_failsafe")
        latched = state["sender"].get("latched_abort")
        if latched:
            raise FlightAbort(f"sender_latched:{latched}")
        altitude = state["altitude_m"]
        if altitude is not None and altitude > self.envelope.geofence_max_enu_m[2]:
            raise FlightAbort("geofence_altitude_exceeded")
        speed = state["speed_m_s"]
        if (
            speed is not None
            and speed > self.envelope.maximum_horizontal_velocity_m_s + 0.5
        ):
            raise FlightAbort(f"horizontal_speed_exceeded:{speed:.2f}")

    # -- sequence ---------------------------------------------------------

    def precheck(self) -> None:
        state = self.vehicle()
        sender = state["sender"]
        failures = []
        if state["armed"] is not False:
            failures.append("uav01_not_disarmed")
        if state["second_vehicle_armed"] is not False:
            failures.append("uav02_not_disarmed")
        if state["offboard_signal_lost"] is not False:
            failures.append("px4_does_not_see_the_setpoint_stream")
        if not sender.get("transmit_sink_attached"):
            failures.append("no_transmit_sink")
        if not sender.get("explicit_opt_in"):
            failures.append("no_explicit_opt_in")
        # WarmupContract, measured on what actually reached the wire.
        duration = sender.get("stream_duration_s")
        if duration is None or duration < self.warmup.minimum_duration_s:
            failures.append(f"warmup_duration_insufficient:{duration}")
        if sender.get("transmit_count", 0) < self.warmup.minimum_valid_samples:
            failures.append("warmup_samples_insufficient")
        if sender.get("max_transmit_gap_s", 9.9) > self.warmup.maximum_gap_s:
            failures.append(f"warmup_gap:{sender.get('max_transmit_gap_s')}")
        if state["conditions"]:
            failures.append(f"abort_conditions_active:{state['conditions']}")
        if failures:
            raise FlightAbort("precheck:" + ",".join(failures))
        self.log("PRECHECK_PASS", **self.brief(state), stream_s=duration,
                 max_gap_s=sender.get("max_transmit_gap_s"))

    def run(self) -> str:
        self.log("START", envelope_hover_m=self.envelope.hover_altitude_m,
                 max_duration_s=self.envelope.maximum_test_duration_s)
        self.precheck()

        self.commander("arm")
        self.await_state("armed", lambda s: s["armed"] is True, timeout_s=10.0)

        self.commander("takeoff")
        # PX4 AUTO takeoff to MIS_TAKEOFF_ALT. Wait for altitude AND for the
        # climb to have settled: handing over to OFFBOARD mid-climb would ask
        # the companion to arrest a vertical rate it never commanded.
        self.await_state(
            "hover_reached",
            lambda s: (
                s["altitude_m"] is not None
                and s["altitude_m"] > 7.0
                and abs(s["vertical_velocity_m_s"] or 9.9) < 0.3
            ),
            timeout_s=45.0,
        )

        self.commander("mode", "offboard")
        self.await_state(
            "offboard_engaged",
            lambda s: s["nav_state"] == NAV_STATE_OFFBOARD,
            timeout_s=10.0,
        )

        # THE MEASUREMENT. The companion is now the only source of setpoints.
        entry = self.vehicle()
        entry_altitude = entry["altitude_m"]
        deadline = time.monotonic() + self.hold_s
        samples = []
        while time.monotonic() < deadline:
            state = self.vehicle()
            self.guard(state)
            if state["nav_state"] != NAV_STATE_OFFBOARD:
                raise FlightAbort("px4_exited_offboard_unexpectedly")
            samples.append(state)
            time.sleep(0.5)
        # Drift is measured against the altitude the companion is actually
        # holding against, not the entry sample: the bridge learns the mode
        # from a 1 Hz HEARTBEAT, so station-keeping begins up to a second
        # after OFFBOARD engages and the reference is captured then.
        references = [
            s["altitude_reference_m"] for s in samples if s["altitude_reference_m"]
        ]
        reference = references[0] if references else entry_altitude
        drift = [
            abs(s["altitude_m"] - reference)
            for s in samples
            if s["altitude_m"] is not None and s["station_keeping"]
        ]
        speeds = [s["speed_m_s"] for s in samples if s["speed_m_s"] is not None]
        self.log(
            "OFFBOARD_HOLD_COMPLETE",
            seconds=self.hold_s,
            entry_altitude_m=round(entry_altitude, 2),
            altitude_reference_m=round(reference, 3),
            max_altitude_drift_m=round(max(drift), 3) if drift else None,
            max_horizontal_speed_m_s=round(max(speeds), 3) if speeds else None,
            station_keeping_samples=sum(1 for s in samples if s["station_keeping"]),
            total_samples=len(samples),
            nominal_reasons=sorted({str(s["nominal_reason"]) for s in samples}),
            decisions=sorted({str(s["frame"].get("decision")) for s in samples}),
        )

        self.commander("mode", "posctl")
        self.await_state(
            "posctl_restored",
            lambda s: s["nav_state"] == NAV_STATE_POSCTL,
            timeout_s=10.0,
        )

        self.commander("land")
        self.await_state("disarmed", lambda s: s["armed"] is False, timeout_s=90.0)
        return "FLIGHT_PASS"

    def abort(self, reason: str) -> None:
        """POSCTL then land. The strongest action this project implements."""
        self.log("ABORT", reason=reason)
        try:
            self.commander("mode", "posctl")
            time.sleep(1.0)
            state = self.vehicle()
            if state["armed"]:
                self.commander("land")
        except Exception as error:  # noqa: BLE001 - abort must not raise
            self.log("ABORT_INCOMPLETE", error=str(error))
        self.log(
            "ABORT_DONE",
            note="POSCTL is the strongest implemented action; a human must take it from here",
        )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--hold-s", type=float, default=15.0)
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--output", default="artifacts/one_uav_active_flight.json")
    arguments = parser.parse_args()

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

    final = flight.vehicle()
    result = {
        "verdict": verdict,
        "records": flight.records,
        "commands_issued": flight.commands,
        "final_state": Flight.brief(final),
        "final_second_vehicle_armed": final["second_vehicle_armed"],
        "sender": final["sender"],
    }
    output = Path(arguments.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    print()
    print(json.dumps({"verdict": verdict, "final": result["final_state"],
                      "uav02_armed": result["final_second_vehicle_armed"]}, indent=2))
    return 0 if verdict == "FLIGHT_PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
