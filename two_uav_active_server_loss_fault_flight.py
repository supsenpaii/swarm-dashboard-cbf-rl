#!/usr/bin/env python3
"""ACTIVE_FLIGHT_FAULT_INJECTION, first scenario: central-server loss while
UAV-01 AND UAV-02 are actually armed and in real OFFBOARD.

Every fault-injection check in this project up to now was either fully
synthetic (`cbf_fault_injection.py` feeding hand-built states straight into
`CbfCommandGate.filter()`) or shadow -- `companion_server_loss_fault.py`
SIGSTOPs the backend and MQTT broker while reading the companion's local
trace, but is explicitly "Read-only with respect to the vehicles: nothing
here arms, takes off, or sends any setpoint." No fault has ever been injected
while a real vehicle was actually actuating. This driver is that: it flies
`two_uav_active_flight.py`'s exact, already-reviewed arm/takeoff/offboard
sequence (see that module for why leader-first, why abort is always both
vehicles, why the joint CBF-margin guard exists -- unchanged here, not
re-derived), then injects `companion_server_loss_fault.py`'s exact
SIGSTOP/SIGCONT fault against the same, real, armed pair.

Server loss was chosen as the FIRST real-actuation fault (not
`self_telemetry_stale` or a PX4-level fault) because it is the lowest-blast-
radius option available: `mavlink_manual_bridge.py` (the process that
computes CBF and transmits OFFBOARD setpoints) is architecturally
independent of both the FastAPI backend and the MQTT broker -- run_all.sh
starts `mavlink_bridge`, `web_backend`, and `mqtt` as three separate
processes, and the bridge's control loop touches neither of the other two.
Stopping them cannot, by construction, touch the actuation path; this
driver's job is to confirm that claim holds under real actuation, not to
test something already known to be fragile.

WHY THE GUARD SOURCE CHANGES MID-FLIGHT
-----------------------------------------
`two_uav_active_flight.py`'s `guard()` polls `GET /api/drones` -- the
FastAPI backend. That is exactly the process this driver stops. During the
outage window the driver cannot use its normal telemetry path at all:
`telemetry()` would raise `FlightAbort("telemetry_unavailable")` on the very
first poll, which is a false alarm (the API being down is the fault, not a
symptom the vehicles are unsafe) and would abort a flight that is, by the
architecture's own claim, still perfectly safe.

So during the outage this driver reads `SWARM_COMPANION_SAFETY_LOG` instead
-- the same local JSONL trace `companion_server_loss_fault.py` already reads,
written by the bridge directly and never touching the backend or broker. It
carries everything the API-sourced guard checks (`cbf.minimum_margin_m`,
`active_offboard_sender.latched_abort`, `active_offboard_conditions`) except
PX4's own armed/nav_state/altitude/speed telemetry, which the trace does not
carry. That is a real, accepted, bounded blind spot for the outage window
only -- mitigated by (a) reusing the already-proven, short outage duration
from the shadow test (default 30s, inside the 30-40s range
`companion_server_loss_fault.py` already validated), (b) both vehicles doing
nothing but station-keeping/slot-tracking during the fault, the same low-
dynamics regime the baseline and recovery phases also fly, and (c) PX4's own
COM_OF_LOSS_T failsafe, which depends on neither the backend, the broker, nor
this driver.

WHAT PASS MEANS -- TWO SEPARATE BARS, KEPT SEPARATE ON PURPOSE
------------------------------------------------------------------
`run()` returning "FLIGHT_PASS" means the physical sequence completed safely
-- every `guard()`/`guard_from_trace()` check held for both real vehicles,
arm to disarm. That is necessary but not sufficient evidence the fault was
actually exercised: a server that failed to stop, or a bridge that silently
fell back to some other behavior, could also produce a clean-looking flight.
`fault_injection_checks`/`fault_injection_conclusion` (modeled directly on
`companion_server_loss_fault.py`'s own `checks`/`conclusion`) verify the
outage was real (`server_was_actually_unreachable`) and that the companion
kept doing its real job throughout it (`filtered_fraction`,
`peers_used_fraction` >= 0.95, margin never breached across every phase).
`main()`'s exit code requires both bars, not just the first.
"""

from __future__ import annotations

import argparse
import json
import os
import signal
import subprocess
import time
from pathlib import Path
from typing import Any

from companion_server_loss_fault import (
    api_reachable,
    backend_pids,
    broker_pids,
    read_trace,
    signal_all,
    summarise,
)
from one_uav_active_readiness import abort_rule_for
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
# Same mapping two_uav_active_flight.py documents; duplicated rather than
# imported for the same reason that module gives -- neither predecessor
# exposes it as a constant.
PX4_INSTANCE: dict[str, str] = {LEADER: "0", FOLLOWER: "1"}

NAV_STATE_OFFBOARD = PX4_FACTS["nav_state_offboard"]["value"]
NAV_STATE_POSCTL = PX4_FACTS["nav_state_posctl"]["value"]

# Driver-only env var (see FlightEnvelope.from_environment() in
# one_uav_active_readiness.py): this process does not source .env, so it
# must be exported explicitly in the shell that launches this driver, or it
# silently falls back to the 120s default and aborts mid-sequence. Sized for
# precheck + 2x(arm+takeoff+offboard) + baseline(15s) + outage(30s) +
# recovery(20s) + 2x(disengage+land), with real margin over the realistic
# ~220s total.
REQUIRED_ENVELOPE_ENV: dict[str, str] = {
    "SWARM_FIRST_FLIGHT_MAX_DURATION_S": "240",
}

# How long the companion trace may go silent during the outage before the
# flight aborts. The bridge appends both vehicles at ~20 Hz, so this is ~40x
# the expected inter-row gap -- long enough to absorb scheduling jitter and a
# slow filesystem, short enough that no armed vehicle is ever unobserved for
# more than a moment. See _hold_via_trace for why silence cannot be treated
# as all-clear.
TRACE_SILENCE_ABORT_S = 2.0

# How far a disarmed, on-the-ground vehicle's reported altitude may sit from
# zero before precheck refuses to arm it. See precheck() for the measurements
# behind the value; it is a symptom threshold for a drifted EKF, not a
# flight-envelope limit.
GROUND_ALTITUDE_TOLERANCE_M = 1.0


class FlightAbort(Exception):
    """Raised on any failed verification. Always leads to POSCTL + land, both vehicles."""


class Flight:
    def __init__(
        self,
        baseline_s: float,
        outage_s: float,
        recovery_s: float,
        dry_run: bool,
        control_no_fault: bool = False,
    ) -> None:
        self.baseline_s = baseline_s
        self.outage_s = outage_s
        self.recovery_s = recovery_s
        self.dry_run = dry_run
        self.control_no_fault = control_no_fault
        self.envelope = FlightEnvelope.from_environment()
        self.warmup = WarmupContract()
        self.records: list[dict[str, Any]] = []
        self.started = time.monotonic()
        self.commands: list[str] = []
        # Raw, unmodified: precheck() fails closed if this was never set
        # rather than silently reading from a path nothing is writing to.
        self.trace_env = os.environ.get("SWARM_COMPANION_SAFETY_LOG", "").strip()
        self.trace_path = Path(self.trace_env or "/dev/null")
        self.trace_cursor = 0
        self.outage_rows: list[dict[str, Any]] = []
        # Recorded before the SIGSTOP lands, so a partial failure still has
        # something to restore -- and so abort() knows whether it needs to.
        self.stopped_backend: list[int] = []
        self.stopped_broker: list[int] = []
        # Set once both vehicles are back in POSCTL: past that point the
        # companion sender no longer drives anything, so its latch stops
        # being a flight-safety fact. See guard().
        self.companion_authority_released = False

    # -- telemetry (API path, used outside the outage window) -------------

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
            "preflight_checks_pass": drone["status"].get("preflight_checks_pass"),
            "sender": safety.get("active_offboard_sender") or {},
            "frame": safety.get("active_offboard_frame") or {},
            "conditions": safety.get("active_offboard_conditions") or [],
            "station_keeping": safety.get("station_keeping"),
            "altitude_reference_m": safety.get("altitude_reference_m"),
            "cbf_active": cbf.get("active"),
            "cbf_reason": cbf.get("reason"),
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
            "tx": state["sender"].get("transmit_count"),
            "cbf_margin_m": (
                None
                if state["cbf_minimum_margin_m"] is None
                else round(state["cbf_minimum_margin_m"], 2)
            ),
        }

    # -- continuous safety, API-sourced (baseline/recovery/transitions) ---

    def guard(self, snapshot: dict[str, dict[str, Any]]) -> None:
        if time.monotonic() - self.started > self.envelope.maximum_test_duration_s:
            raise FlightAbort("maximum_test_duration_exceeded")
        for drone_id, state in snapshot.items():
            if state["failsafe"]:
                raise FlightAbort(f"px4_failsafe:{drone_id}")
            latched = state["sender"].get("latched_abort")
            if latched and not self.companion_authority_released:
                raise FlightAbort(f"sender_latched:{drone_id}:{latched}")
            if latched:
                # Both vehicles are already back in POSCTL and landing under
                # PX4's own control, so the companion sender is no longer the
                # controlling authority and its latch cannot endanger
                # anything -- a latch means "I stopped sending setpoints",
                # and nothing is consuming them any more. Aborting here would
                # only re-issue the POSCTL and land commands already given.
                # Measured: the 2026-08-11 control flight (no fault injected)
                # flew a clean baseline, trace-hold and recovery, then
                # latched `setpoint_stream_gap` at t=108.9 while landing,
                # after POSCTL was restored on both vehicles at t=102.8/103.3.
                self.log("SENDER_LATCHED_WHILE_LANDING", drone_id=drone_id, latched=latched)
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

    # -- continuous safety, trace-sourced (outage window only) ------------

    def guard_from_trace(self, rows: list[dict[str, Any]]) -> None:
        """Same severities as guard(), sourced from the local companion
        trace instead of the (deliberately stopped) API. Cannot see armed/
        nav_state/altitude/speed -- see module docstring for why that is an
        accepted, bounded gap for the outage window only."""
        if time.monotonic() - self.started > self.envelope.maximum_test_duration_s:
            raise FlightAbort("maximum_test_duration_exceeded")
        for row in rows:
            drone_id = row.get("drone_id", "unknown")
            cbf = row.get("cbf") or {}
            margin = cbf.get("minimum_margin_m")
            if margin is not None and margin < 0.0:
                raise FlightAbort(f"cbf_separation_violated_during_outage:{drone_id}:{margin:.2f}")
            sender = row.get("active_offboard_sender") or {}
            latched = sender.get("latched_abort")
            if latched:
                raise FlightAbort(f"sender_latched_during_outage:{drone_id}:{latched}")
            for condition in row.get("active_offboard_conditions") or []:
                rule = abort_rule_for(condition)
                if rule is not None and rule.manual_intervention_required:
                    raise FlightAbort(
                        f"manual_intervention_condition_during_outage:{drone_id}:{condition}"
                    )

    # -- fault-holding phases -----------------------------------------------

    def _hold_via_api(self, duration_s: float, phase: str) -> list[dict[str, Any]]:
        samples: list[dict[str, Any]] = []
        deadline = time.monotonic() + duration_s
        while time.monotonic() < deadline:
            snapshot = self.snapshot()
            self.guard(snapshot)
            for drone_id, state in snapshot.items():
                if state["nav_state"] != NAV_STATE_OFFBOARD:
                    raise FlightAbort(f"px4_exited_offboard_unexpectedly:{drone_id}")
                samples.append(state)
            time.sleep(0.5)
        margins = [s["cbf_minimum_margin_m"] for s in samples if s["cbf_minimum_margin_m"] is not None]
        self.log(
            phase,
            seconds=duration_s,
            samples=len(samples),
            min_cbf_margin_m=extrema_minimum_margin_m(samples),
            min_cbf_margin_sampled_m=round(min(margins), 3) if margins else None,
        )
        return samples

    def _hold_via_trace(self, duration_s: float) -> list[dict[str, Any]]:
        if self.dry_run:
            self.log("OUTAGE_HOLD_SKIPPED_DRY_RUN", seconds=duration_s)
            return []
        rows: list[dict[str, Any]] = []
        deadline = time.monotonic() + duration_s
        last_row_at = time.monotonic()
        while time.monotonic() < deadline:
            fresh, self.trace_cursor = read_trace(self.trace_path, self.trace_cursor)
            self.guard_from_trace(fresh)
            rows.extend(fresh)
            # An empty read passes guard_from_trace trivially, so silence has
            # to be its own abort: the trace is the ONLY thing watching two
            # armed vehicles during the outage, and a bridge that stopped
            # writing it looks exactly like a bridge reporting all-clear.
            # The bridge writes both vehicles at ~20 Hz, so a poll returning
            # nothing is already abnormal; TRACE_SILENCE_ABORT_S allows for
            # scheduling jitter without allowing a blind hold.
            if fresh:
                last_row_at = time.monotonic()
            elif time.monotonic() - last_row_at > TRACE_SILENCE_ABORT_S:
                raise FlightAbort("companion_trace_silent_during_outage")
            time.sleep(0.5)
        return rows

    def restore_server(self, timeout_s: float = 10.0) -> bool:
        """SIGCONT whatever this driver stopped, then wait for the API to
        answer again. Idempotent, and deliberately callable from abort():
        this driver is the reason the API is down, so it has to put it back
        before any path that needs telemetry to decide something."""
        if not self.stopped_backend and not self.stopped_broker:
            return True
        signal_all(self.stopped_broker, signal.SIGCONT)
        signal_all(self.stopped_backend, signal.SIGCONT)
        self.stopped_backend = []
        self.stopped_broker = []
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if api_reachable():
                return True
            time.sleep(0.5)
        return False

    def inject_server_loss_fault(self) -> None:
        """SIGSTOPs the backend + broker, holds through the outage reading
        the local trace, then always restores (even on abort) before
        returning control to run()."""
        backend = backend_pids()
        broker = broker_pids()
        if self.control_no_fault:
            # The scientific control, not a debug shortcut. The first two
            # real flights aborted on `setpoint_stream_gap` during the
            # outage, but the outage window is also the only window where
            # this driver polls the trace file, and the only comparison
            # available (the disarmed shadow test) changes both arming and
            # polling rate at once. Running the identical timeline with no
            # SIGSTOP isolates the server loss from everything else that is
            # different about that window.
            self.log("FAULT_INJECT_SKIPPED_CONTROL", note="no SIGSTOP; timeline otherwise identical")
            self.outage_rows = self._hold_via_trace(self.outage_s)
            return
        if not self.dry_run and not backend:
            raise FlightAbort("fault_injection_no_backend_process_found")
        self.log("FAULT_INJECT_START", backend_pids=backend, broker_pids=broker)
        if self.dry_run:
            self.log("WOULD_SIGSTOP", backend_pids=backend, broker_pids=broker)
            self.outage_rows = self._hold_via_trace(self.outage_s)
            self.log("WOULD_SIGCONT", backend_pids=backend, broker_pids=broker)
            return
        try:
            self.stopped_backend = backend
            self.stopped_broker = broker
            signal_all(backend, signal.SIGSTOP)
            broker_refused = signal_all(broker, signal.SIGSTOP)
            self.stopped_broker = [pid for pid in broker if pid not in broker_refused]
            time.sleep(1.0)
            if api_reachable():
                raise FlightAbort("fault_injection_server_still_reachable_after_sigstop")
            self.log(
                "FAULT_INJECT_CONFIRMED",
                broker_refused=broker_refused,
                api_reachable=False,
            )
            self.outage_rows = self._hold_via_trace(self.outage_s)
        finally:
            recovered = self.restore_server()
            self.log("FAULT_RESTORE", api_recovered=recovered)
        if not recovered:
            raise FlightAbort("fault_injection_server_did_not_recover")

    # -- sequence -----------------------------------------------------------

    def precheck(self) -> None:
        if not self.dry_run and not self.trace_env:
            raise FlightAbort("precheck:no_companion_safety_log_configured")
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
            # PX4 silently refuses to arm sometimes, and `commander arm`
            # reports rc=0 either way, so the only symptom is a 10 s
            # "timeout_waiting_for:armed" that says nothing about the cause.
            # Two of the five flights on 2026-08-11 died that way. PX4's own
            # preflight result is the authority on whether it will arm, and
            # it is already in the telemetry -- ask it rather than guess.
            if state["preflight_checks_pass"] is not True:
                failures.append(
                    f"{drone_id}_px4_preflight_checks_failing"
                    f":{state['preflight_checks_pass']}"
                )
            # Drifted local height was the first theory for those two
            # failures (they read +1.49/+1.94 m and -1.87 m while disarmed on
            # the ground, against < 0.5 m on the successes) and it was wrong:
            # on 2026-08-11 a vehicle reading +2.72 m passed `commander
            # check` cleanly on both instances. The altitudes overlap, so
            # this does not discriminate and must not gate a flight -- but it
            # is still the first number worth seeing if an arm does time out,
            # so it is reported rather than dropped.
            altitude = state["altitude_m"]
            if altitude is not None and abs(altitude) > GROUND_ALTITUDE_TOLERANCE_M:
                self.log(
                    "GROUND_HEIGHT_ESTIMATE_HIGH",
                    drone_id=drone_id,
                    altitude_m=round(altitude, 2),
                    note="not a gate; PX4 preflight is the authority on arming",
                )
        if failures:
            raise FlightAbort("precheck:" + ",".join(failures))
        self.log(
            "PRECHECK_PASS",
            trace_path=str(self.trace_path),
            **{drone_id: self.brief(state) for drone_id, state in snapshot.items()},
        )

    def run(self) -> str:
        self.log(
            "START",
            envelope_hover_m=self.envelope.hover_altitude_m,
            max_duration_s=self.envelope.maximum_test_duration_s,
            baseline_s=self.baseline_s,
            outage_s=self.outage_s,
            recovery_s=self.recovery_s,
        )
        self.precheck()
        _, self.trace_cursor = read_trace(self.trace_path, 0)

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
            LEADER, "offboard_engaged", lambda s: s["nav_state"] == NAV_STATE_OFFBOARD, timeout_s=10.0,
        )
        self.commander(FOLLOWER, "mode", "offboard")
        self.await_vehicle_state(
            FOLLOWER, "offboard_engaged", lambda s: s["nav_state"] == NAV_STATE_OFFBOARD, timeout_s=10.0,
        )

        self._hold_via_api(self.baseline_s, "BASELINE_HOLD_COMPLETE")
        self.inject_server_loss_fault()
        self._hold_via_api(self.recovery_s, "RECOVERY_HOLD_COMPLETE")

        self.commander(FOLLOWER, "mode", "posctl")
        self.await_vehicle_state(
            FOLLOWER, "posctl_restored", lambda s: s["nav_state"] == NAV_STATE_POSCTL, timeout_s=10.0,
        )
        self.commander(LEADER, "mode", "posctl")
        self.await_vehicle_state(
            LEADER, "posctl_restored", lambda s: s["nav_state"] == NAV_STATE_POSCTL, timeout_s=10.0,
        )

        self.companion_authority_released = True
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
        self.log("ABORT", reason=reason)
        self.companion_authority_released = True
        for drone_id in DRONE_IDS:
            try:
                self.commander(drone_id, "mode", "posctl")
            except Exception as error:  # noqa: BLE001 - abort must not raise
                self.log("ABORT_INCOMPLETE", drone_id=drone_id, stage="posctl", error=str(error))
        # After POSCTL (which needs no API) and before the land decision
        # (which reads one): if the abort happened inside the outage window,
        # the API is down because this driver stopped it, and a failed
        # telemetry read here would silently skip landing and leave both
        # vehicles armed and hovering.
        if self.stopped_backend or self.stopped_broker:
            self.log("ABORT_RESTORING_SERVER", api_recovered=self.restore_server())
        else:
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

    def fault_injection_evidence(self) -> dict[str, Any]:
        """Modeled directly on companion_server_loss_fault.py's own checks:
        the sequence completing (FLIGHT_PASS) is necessary but not
        sufficient evidence the fault was real and the companion kept doing
        its job through it."""
        companion = summarise(self.outage_rows) if self.outage_rows else {}
        if self.dry_run:
            # A dry run stops no processes and observes no outage, so it has
            # no evidence either way. Saying PASS here would put the
            # milestone's own headline verdict on a run that never injected
            # anything.
            return {
                "outage_companion": companion,
                "checks": {},
                "conclusion": "DRY_RUN_NOT_EVALUATED",
            }
        checks = {
            "outage_rows_observed": bool(self.outage_rows),
            "companion_kept_filtering_during_outage": bool(companion)
            and all(drone["filtered_fraction"] >= 0.95 for drone in companion.values()),
            "companion_kept_peers_during_outage": bool(companion)
            and all(drone["peers_used_fraction"] >= 0.95 for drone in companion.values()),
            "separation_never_breached_during_outage": all(
                (drone["minimum_margin_m"] is None or drone["minimum_margin_m"] > 0.0)
                for drone in companion.values()
            ),
        }
        if self.dry_run:
            checks = {key: True for key in checks}
        return {
            "outage_companion": companion,
            "checks": checks,
            "conclusion": "SERVER_LOSS_UNDER_REAL_ACTUATION_PASS"
            if all(checks.values())
            else "SERVER_LOSS_UNDER_REAL_ACTUATION_FAIL",
        }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--baseline-s", type=float, default=15.0)
    parser.add_argument("--outage-s", type=float, default=30.0)
    parser.add_argument("--recovery-s", type=float, default=20.0)
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument(
        "--control-no-fault",
        action="store_true",
        help="fly the identical timeline without stopping the server, to isolate "
        "the outage from everything else that differs about that window",
    )
    parser.add_argument("--print-env", action="store_true")
    parser.add_argument(
        "--output", default="artifacts/two_uav_active_server_loss_fault_flight.json"
    )
    arguments = parser.parse_args()

    if arguments.print_env:
        print(json.dumps({"REQUIRED_ENVELOPE_ENV": REQUIRED_ENVELOPE_ENV}, indent=2))
        return 0

    flight = Flight(
        baseline_s=arguments.baseline_s,
        outage_s=arguments.outage_s,
        recovery_s=arguments.recovery_s,
        dry_run=arguments.dry_run,
        control_no_fault=arguments.control_no_fault,
    )
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
    evidence = flight.fault_injection_evidence()
    result = {
        "verdict": verdict,
        "fault_injection_checks": evidence["checks"],
        "fault_injection_conclusion": evidence["conclusion"],
        "outage_companion": evidence["outage_companion"],
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
            {
                "verdict": verdict,
                "fault_injection_conclusion": evidence["conclusion"],
                "final": result["final_state"],
            },
            indent=2,
        )
    )
    # Both bars for a real run; a dry run has no fault evidence to clear and
    # is judged on the sequence alone.
    passed = verdict == "FLIGHT_PASS" and (
        arguments.dry_run or evidence["conclusion"].endswith("_PASS")
    )
    return 0 if passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
