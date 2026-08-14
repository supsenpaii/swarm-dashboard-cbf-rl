"""Derives the abort conditions the active sender cannot see for itself.

`ActiveOffboardSetpointSender` decides seven of the fourteen ABORT_MATRIX
conditions from the preview it is handed. The other seven need knowledge the
preview does not carry -- MAVLink link health, PX4 mode transitions, CBF gate
state, own-telemetry validity, and the liveness of the loop producing the
previews. Those are this module's job.

Kept separate from both the sender and the bridge, and written as one pure
function over plain scalars, so the mapping can be tested without a vehicle,
a MAVLink connection, or the safety stack. The sender must not import the
safety stack (that is what makes bypassing CBF structurally impossible), and
the bridge is the least testable file in the project; neither is a good home
for this logic.

REACHABILITY -- the mission flight gate makes `px4_rejects_offboard` and
`px4_exits_offboard_unexpectedly` live: it records the mode COMMAND_ACK and
marks OFFBOARD expected only after PX4 is observed in that mode.

Structurally unreachable from the affected worker (measured, not assumed):
  `mavlink_connection_loss` and `companion_safety_loop_stops` cannot be
  reported by the worker whose link died, because that worker's loop is
  what stops. SIGSTOP on one UAV's PX4 froze that worker's
  `evaluated_monotonic_s` at 28484.764 for the full 7 s of the fault while
  the other worker kept evaluating at 20 Hz: the blocking MAVLink read never
  returns, so no frame is produced to carry the condition.

  This is not a safety hole. PX4's own COM_OF_LOSS_T (1.0 s) expires and
  falls back to Position mode, which is exactly the px4_policy ABORT_MATRIX
  already records for both rules. What is missing is the *observation*, not
  the protection.

  A watcher outside the worker now exists: worker_liveness_watchdog.py, run
  from main()'s own idle loop in mavlink_manual_bridge.py (a different
  thread, so a frozen worker cannot also freeze the check). It restores the
  missing observation -- logging and a retained MQTT status per drone -- but
  does not change reachability here: a genuinely frozen worker still cannot
  run this module's derivation any more than it could before, since that
  derivation is code the frozen loop itself would have to execute. The
  watchdog's report is a separate, independently-produced signal, not a way
  for the stalled worker to report these two conditions itself.

  The peer's loss is detected, and by the right vehicle: during that same
  fault the surviving UAV reported `cbf_invalid_or_infeasible` and held at
  zero velocity for the whole outage.
"""

from __future__ import annotations

from typing import Any


# Results PX4 returns for an accepted MAV_CMD_DO_SET_MODE. Anything else is a
# rejection. Mirrored as plain ints rather than importing pymavlink so this
# module stays testable without the transport library; the bridge passes its
# own pymavlink-derived set when it has one.
DEFAULT_ACCEPTED_ACK_RESULTS = frozenset({0, 5})  # ACCEPTED, IN_PROGRESS


def derive_reported_conditions(
    *,
    now_monotonic_s: float,
    self_state_valid: bool,
    cbf_active: bool,
    cbf_reason: str,
    last_heartbeat_monotonic_s: float | None,
    heartbeat_timeout_s: float,
    px4_main_mode: int | None,
    px4_offboard_main_mode: int,
    offboard_expected: bool,
    offboard_mode_ack_result: int | None,
    accepted_ack_results: frozenset[int] = DEFAULT_ACCEPTED_ACK_RESULTS,
    previous_evaluation_monotonic_s: float | None,
    maximum_command_age_s: float,
) -> tuple[str, ...]:
    """Return the active ABORT_MATRIX conditions, unordered.

    Ordering is not decided here: the sender resolves precedence using
    ABORT_MATRIX's own most-severe-first order, so a second ranking in this
    module could only ever disagree with it.

    Every check fails closed. In particular a `None` heartbeat timestamp
    means "never heard from", not "recently heard from".
    """
    conditions: list[str] = []

    # Broader than the matrix's original wording, deliberately: an own state
    # that is invalid for any reason (telemetry_stale, no_local_state,
    # enu_transform_failed) is equally unusable as the basis for a setpoint.
    if not self_state_valid:
        conditions.append("self_telemetry_stale")

    # The CBF gate holding for any reason -- infeasible constraints, invalid
    # peer state, missing own state -- means the commanded velocity is no
    # longer a filtered solution. `outside_geofence` also sets active=False,
    # so it is reported as both; the sender's matrix ordering picks the more
    # severe, which is what a single caller-side choice would get wrong.
    if not cbf_active:
        conditions.append("cbf_invalid_or_infeasible")
    if cbf_reason == "outside_geofence":
        conditions.append("geofence_violation_or_risk")

    if (
        last_heartbeat_monotonic_s is None
        or now_monotonic_s - last_heartbeat_monotonic_s > heartbeat_timeout_s
    ):
        conditions.append("mavlink_connection_loss")

    # Only meaningful once the sequence expects OFFBOARD. Before that, PX4
    # being in POSCTL is the normal resting state, not an unexpected exit.
    if offboard_expected and px4_main_mode != px4_offboard_main_mode:
        conditions.append("px4_exits_offboard_unexpectedly")

    if (
        offboard_mode_ack_result is not None
        and int(offboard_mode_ack_result) not in accepted_ack_results
    ):
        conditions.append("px4_rejects_offboard")

    # The loop that produces previews having stalled. Measured against the
    # previous evaluation rather than a wall clock, so it detects the loop
    # itself falling behind. No previous evaluation means this is the first
    # frame, which is not a stall.
    if (
        previous_evaluation_monotonic_s is not None
        and now_monotonic_s - previous_evaluation_monotonic_s > maximum_command_age_s
    ):
        conditions.append("companion_safety_loop_stops")

    return tuple(conditions)


def conditions_from_status(
    status_payload: dict[str, Any],
    *,
    now_monotonic_s: float,
    last_heartbeat_monotonic_s: float | None,
    heartbeat_timeout_s: float,
    px4_main_mode: int | None,
    px4_offboard_main_mode: int,
    offboard_expected: bool,
    offboard_mode_ack_result: int | None,
    accepted_ack_results: frozenset[int] = DEFAULT_ACCEPTED_ACK_RESULTS,
    previous_evaluation_monotonic_s: float | None,
    maximum_command_age_s: float,
) -> tuple[str, ...]:
    """Adapter from a published CompanionSafetyStatus dict.

    Reads the serialized payload rather than the dataclass so this module
    does not import the safety stack, and so the same derivation can be
    replayed from a recorded trace file. The cost of that decoupling is that
    the key names are a contract with no type checker behind them, so
    test_offboard_abort_conditions pins them against the real
    `CompanionSafetyStatus.as_dict()` rather than a hand-written sample --
    the first live run of this module reported a phantom CBF abort on every
    frame because a hand-written test agreed with a wrong key.
    """
    command = status_payload.get("cbf") or {}
    return derive_reported_conditions(
        now_monotonic_s=now_monotonic_s,
        # A missing self_state_reason is not evidence of health.
        self_state_valid=status_payload.get("self_state_reason") == "ok",
        cbf_active=bool(command.get("active")),
        cbf_reason=str(command.get("reason") or ""),
        last_heartbeat_monotonic_s=last_heartbeat_monotonic_s,
        heartbeat_timeout_s=heartbeat_timeout_s,
        px4_main_mode=px4_main_mode,
        px4_offboard_main_mode=px4_offboard_main_mode,
        offboard_expected=offboard_expected,
        offboard_mode_ack_result=offboard_mode_ack_result,
        accepted_ack_results=accepted_ack_results,
        previous_evaluation_monotonic_s=previous_evaluation_monotonic_s,
        maximum_command_age_s=maximum_command_age_s,
    )
