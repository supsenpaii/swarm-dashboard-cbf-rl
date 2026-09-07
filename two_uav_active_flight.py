#!/usr/bin/env python3
"""TWO_UAV_ACTIVE_SITL_FLIGHT: first armed OFFBOARD flight of UAV-01 AND UAV-02
together.

Succeeds `one_uav_active_flight.py` rather than editing it: that script is
the ONE_UAV_ACTIVE_SITL_FLIGHT milestone's own reviewed record (UAV-02 stayed
disarmed throughout, guarded explicitly) and stays exactly what it always
meant. This driver reuses everything vehicle-count-agnostic from it --
`FlightEnvelope`, `WarmupContract`, PX4_FACTS, the precheck/guard/abort
shape -- generalized from one vehicle to a fixed pair.

WHAT IS NEW HERE, NOT JUST DOUBLED
-----------------------------------
UAV-01 is still the formation leader with no slot: its nominal controller
still reports `formation_slot_unassigned` and holds zero velocity plus
altitude hold, exactly as in the first flight. UAV-02 is different --
`SWARM_FORMATION_SLOT_UAV_02_ENU_M` gives it a real slot 10 m from wherever
UAV-01 actually is, so once OFFBOARD engages, UAV-02's nominal controller
produces a real, non-zero velocity toward that slot
(`DeterministicFormationController.command`, reason "tracking_slot"), CBF-
filtered against `SWARM_CBF_MINIMUM_SEPARATION_M` (4.0 m). This is the first
time any command in this project actually moves a vehicle horizontally
toward another vehicle rather than holding station -- the thing this
milestone exists to measure.

WHY ARM/TAKEOFF/OFFBOARD ARE SEQUENCED LEADER-FIRST
------------------------------------------------------
UAV-02's target is UAV-01's *live* position (via peer-to-peer swarm state),
not a fixed point. Bringing the leader up first gives the follower a stable
reference to track from the moment its own OFFBOARD engages, rather than
tracking a leader that is itself still mid-takeoff.

ABORT IS ALWAYS BOTH VEHICLES
------------------------------
UAV-02's target follows UAV-01 in all three axes (the slot's z-offset is 0),
so if only the leader dropped out of OFFBOARD, the follower would still be
tracking wherever the leader ends up next under manual PX4 control -- not a
state to leave standing. Any guard failure aborts BOTH vehicles: POSCTL is
requested for both before landing is requested for either, so the pair stops
taking companion-driven relative commands as fast as possible, then each is
landed independently. There is no autonomous RTL/Land anywhere in this
project -- POSCTL is the strongest implemented action, matching ABORT_MATRIX.

A NEW GUARD, NOT PRESENT IN THE ONE-VEHICLE DRIVER
-----------------------------------------------------
`cbf_minimum_margin_m`, read from each vehicle's own `CompanionSafetyStatus`,
is checked every poll for both vehicles. CBF is a preventive filter, not a
detector -- if it ever fails to keep this non-negative (a PX4 tracking lag
the filter's own instant-response assumption did not account for, say), that
is a real separation violation between two real aircraft and this driver
aborts on it independently of anything CBF itself reports.
"""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
import time
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
LEADER = "UAV-01"
FOLLOWER = "UAV-02"
DRONE_IDS: tuple[str, ...] = (LEADER, FOLLOWER)
# PX4 -i 0 is UAV-01; -i 1 is UAV-02 (same mapping one_uav_active_flight.py
# documents; duplicated here rather than imported since that module has no
# constant for it -- it only ever addressed instance 0).
PX4_INSTANCE: dict[str, str] = {LEADER: "0", FOLLOWER: "1"}

NAV_STATE_OFFBOARD = PX4_FACTS["nav_state_offboard"]["value"]
NAV_STATE_POSCTL = PX4_FACTS["nav_state_posctl"]["value"]


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
            "cbf_active": cbf.get("active"),
            "cbf_reason": cbf.get("reason"),
            # This vehicle's own measurement of the pair's separation margin
            # -- the same value CBF itself filters against, not re-derived
            # from local_position (each vehicle's LOCAL_POSITION_NED origin
            # is its own EKF reference, not a frame shared between them; the
            # swarm's shared ENU frame is what CBF and this margin use).
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
        """Both vehicles from ONE telemetry fetch, so their readings are
        from the same instant -- required for the joint margin guard to be
        meaningful rather than comparing two different moments in time."""
        payload = self.telemetry()
        return {drone_id: self.vehicle(drone_id, payload) for drone_id in DRONE_IDS}

    def log(self, step: str, **fields: Any) -> None:
        record = {"t_s": round(time.monotonic() - self.started, 2), "step": step, **fields}
        self.records.append(record)
        print(json.dumps(record), flush=True)

    # -- commands ---------------------------------------------------------

    def commander(self, drone_id: str, *arguments: str) -> str:
        """PX4's own console client, addressed at one instance. The only
        actuation path in this flight."""
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
        """Waits on one vehicle's transition, but guards BOTH on every poll:
        the vehicle not being waited on must stay safe too while this one
        catches up."""
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
            "tx": state["sender"].get("transmit_count"),
            "cbf_margin_m": (
                None
                if state["cbf_minimum_margin_m"] is None
                else round(state["cbf_minimum_margin_m"], 2)
            ),
        }

    # -- continuous safety ------------------------------------------------

    def guard(self, snapshot: dict[str, dict[str, Any]]) -> None:
        """Checked on every poll, not only at transitions. Per-vehicle
        checks run for both; the CBF margin check is joint."""
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
            minimum_separation_hint_m="see SWARM_CBF_MINIMUM_SEPARATION_M",
        )
        self.precheck()

        # Leader first throughout: the follower's target tracks wherever the
        # leader actually is, so the leader should be a stable reference
        # before the follower does anything that depends on it.
        self.commander(LEADER, "arm")
        self.await_vehicle_state(LEADER, "armed", lambda s: s["armed"] is True, timeout_s=10.0)
        self.commander(FOLLOWER, "arm")
        self.await_vehicle_state(FOLLOWER, "armed", lambda s: s["armed"] is True, timeout_s=10.0)

        def hover_reached(state: dict[str, Any]) -> bool:
            return (
                state["altitude_m"] is not None
                and state["altitude_m"] > 7.0
                and abs(state["vertical_velocity_m_s"] or 9.9) < 0.3
            )

        self.commander(LEADER, "takeoff")
        self.await_vehicle_state(LEADER, "hover_reached", hover_reached, timeout_s=45.0)
        self.commander(FOLLOWER, "takeoff")
        self.await_vehicle_state(FOLLOWER, "hover_reached", hover_reached, timeout_s=45.0)

        self.commander(LEADER, "mode", "offboard")
        self.await_vehicle_state(
            LEADER,
            "offboard_engaged",
            lambda s: s["nav_state"] == NAV_STATE_OFFBOARD,
            timeout_s=10.0,
        )
        self.commander(FOLLOWER, "mode", "offboard")
        self.await_vehicle_state(
            FOLLOWER,
            "offboard_engaged",
            lambda s: s["nav_state"] == NAV_STATE_OFFBOARD,
            timeout_s=10.0,
        )

        # THE MEASUREMENT. Both vehicles are now companion-driven: the
        # leader holds station at zero velocity, the follower tracks its
        # slot 10 m away under real, non-zero, CBF-filtered velocity.
        entry = self.snapshot()
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
            entry_altitude = entry[drone_id]["altitude_m"]
            # UAV-02 has an assigned slot, so its own AltitudeHoldController
            # never activates (companion_safety.evaluate only calls it for an
            # inactive/unassigned nominal) -- its z target comes from the
            # formation controller instead. `reference` falls back to the
            # entry altitude for it, which is what "drift since station-
            # keeping began" means when there is no separate hold reference.
            references = [
                s["altitude_reference_m"] for s in drone_samples if s["altitude_reference_m"]
            ]
            reference = references[0] if references else entry_altitude
            drift = [
                abs(s["altitude_m"] - reference)
                for s in drone_samples
                if s["altitude_m"] is not None and reference is not None
            ]
            speeds = [s["speed_m_s"] for s in drone_samples if s["speed_m_s"] is not None]
            margins = [
                s["cbf_minimum_margin_m"]
                for s in drone_samples
                if s["cbf_minimum_margin_m"] is not None
            ]
            self.log(
                "OFFBOARD_HOLD_COMPLETE",
                drone_id=drone_id,
                seconds=self.hold_s,
                entry_altitude_m=(
                    None if entry_altitude is None else round(entry_altitude, 2)
                ),
                altitude_reference_m=None if reference is None else round(reference, 3),
                max_altitude_drift_m=round(max(drift), 3) if drift else None,
                max_horizontal_speed_m_s=round(max(speeds), 3) if speeds else None,
                min_cbf_margin_m=extrema_minimum_margin_m(drone_samples),
                min_cbf_margin_sampled_m=round(min(margins), 3) if margins else None,
                station_keeping_samples=sum(
                    1 for s in drone_samples if s["station_keeping"]
                ),
                total_samples=len(drone_samples),
                nominal_reasons=sorted({str(s["nominal_reason"]) for s in drone_samples}),
                decisions=sorted({str(s["frame"].get("decision")) for s in drone_samples}),
            )

        # Disengage follower then leader, land follower then leader -- not
        # safety-critical once both hold stably ~10 m apart in independent
        # PX4-only control, kept symmetric with the leader-first engage
        # order above purely for predictability.
        self.commander(FOLLOWER, "mode", "posctl")
        self.await_vehicle_state(
            FOLLOWER,
            "posctl_restored",
            lambda s: s["nav_state"] == NAV_STATE_POSCTL,
            timeout_s=10.0,
        )
        self.commander(LEADER, "mode", "posctl")
        self.await_vehicle_state(
            LEADER,
            "posctl_restored",
            lambda s: s["nav_state"] == NAV_STATE_POSCTL,
            timeout_s=10.0,
        )

        self.commander(FOLLOWER, "land")
        self.await_vehicle_state(
            FOLLOWER, "disarmed", lambda s: s["armed"] is False, timeout_s=90.0
        )
        self.commander(LEADER, "land")
        self.await_vehicle_state(
            LEADER, "disarmed", lambda s: s["armed"] is False, timeout_s=90.0
        )
        return "FLIGHT_PASS"

    def abort(self, reason: str) -> None:
        """POSCTL for both, then land for both. The strongest action this
        project implements, applied to both vehicles because the follower's
        target is only ever meaningful while the leader is a stable,
        companion-driven reference."""
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
    parser.add_argument("--hold-s", type=float, default=15.0)
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--output", default="artifacts/two_uav_active_flight.json")
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

    # Best-effort: telemetry can be unavailable here too (e.g. the stack was
    # never up at all, the scenario a --dry-run with no stack running
    # exercises), and a final-state read failing must not crash the summary
    # after abort() has already done everything it can.
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
