"""Readiness contract for the first armed + OFFBOARD flight of ONE UAV in SITL.

This module contains no transmit path and performs no flight. It defines,
in executable form, what the first active flight would require and how it
would abort, so the contract can be reviewed and dry-run before anything is
ever armed. The active sender it describes is deliberately NOT implemented
here (see `ActiveOffboardSetpointSenderPlan`).

Every PX4 value below was read from this build's own source/config, not
assumed -- see PX4_FACTS for provenance. PX4 v1.15.4-4-g85df8c2281, in which
every OFFBOARD-related parameter is at its firmware default (the build's
parameters.bson contains only 38 non-default entries, none of them COM_OF_*
or COM_OBL_*).
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
import math
import os
from typing import Any


# --------------------------------------------------------------------------
# PX4 facts, each with the file it was read from. Not defaults invented here.
# --------------------------------------------------------------------------
PX4_FACTS: dict[str, Any] = {
    "version": "v1.15.4-4-g85df8c2281",
    "offboard_setpoint_timeout_s": {
        "value": 1.0,
        "param": "COM_OF_LOSS_T",
        "source": "src/modules/commander/commander_params.c:319 (firmware default)",
    },
    "offboard_loss_action": {
        "value": 0,
        "meaning": "Position mode",
        "param": "COM_OBL_RC_ACT",
        "source": "src/modules/commander/commander_params.c:348 (firmware default)",
    },
    "external_setpoint_forwarding": {
        "value": 1,
        "param": "MAV_FWDEXTSP",
        "source": "src/modules/mavlink/mavlink_params.c:121 (firmware default)",
    },
    "offboard_mode_requirements": {
        "value": ["angular_velocity", "attitude", "offboard_signal"],
        "source": "src/modules/commander/ModeUtil/mode_requirements.cpp:132",
    },
    "offboard_signal_rule": {
        "value": (
            "offboard_control_mode newer than COM_OF_LOSS_T AND velocity set "
            "AND NOT local_velocity_invalid"
        ),
        "source": "src/modules/commander/HealthAndArmingChecks/checks/offboardCheck.cpp",
    },
    "broadcast_hazard": {
        "value": "target_system == 0 is accepted by ANY instance",
        "source": "src/modules/mavlink/mavlink_receiver.cpp:966",
    },
    "arming_state_enum": {"DISARMED": 1, "ARMED": 2, "source": "msg/VehicleStatus.msg"},
    "nav_state_offboard": {
        "value": 14,
        "source": "msg/VehicleStatus.msg NAVIGATION_STATE_OFFBOARD",
    },
    "nav_state_posctl": {"value": 2, "source": "msg/VehicleStatus.msg"},
}

# The one vehicle permitted to be armed in the first active gate. Hardcoded
# rather than configurable: the whole point of the first gate is that a
# second vehicle cannot be actuated even by misconfiguration.
FIRST_ACTIVE_FLIGHT_DRONE_ID = "UAV-01"
FIRST_ACTIVE_FLIGHT_SYSTEM_ID = 1


class ReadinessState(str, Enum):
    PRECHECK = "precheck"
    SETPOINT_WARMUP = "setpoint_warmup"
    ARM = "arm"
    OFFBOARD = "offboard"
    CONTROLLED_ASCENT = "controlled_ascent"
    HOVER = "hover"
    ZERO_VELOCITY = "zero_velocity"
    EXIT_OFFBOARD = "exit_offboard"
    LAND_DISARM = "land_disarm"
    ABORTED = "aborted"


STATE_SEQUENCE: tuple[ReadinessState, ...] = (
    ReadinessState.PRECHECK,
    ReadinessState.SETPOINT_WARMUP,
    ReadinessState.ARM,
    ReadinessState.OFFBOARD,
    ReadinessState.CONTROLLED_ASCENT,
    ReadinessState.HOVER,
    ReadinessState.ZERO_VELOCITY,
    ReadinessState.EXIT_OFFBOARD,
    ReadinessState.LAND_DISARM,
)


@dataclass(frozen=True)
class FlightEnvelope:
    """First-flight limits. Every value is inherited from an existing project
    constraint -- none is widened for the sake of the test.
    """

    maximum_horizontal_velocity_m_s: float
    maximum_vertical_velocity_m_s: float
    hover_altitude_m: float
    geofence_min_enu_m: tuple[float, float, float]
    geofence_max_enu_m: tuple[float, float, float]
    maximum_test_duration_s: float
    maximum_state_age_s: float
    maximum_command_age_s: float

    @classmethod
    def from_environment(cls) -> FlightEnvelope:
        def _f(name: str, default: float) -> float:
            try:
                return float(os.environ.get(name, str(default)))
            except (TypeError, ValueError):
                return default

        def _v(name: str, default: str) -> tuple[float, float, float]:
            values = tuple(
                float(part.strip())
                for part in os.environ.get(name, default).split(",")
            )
            if len(values) != 3:
                raise ValueError(f"{name} requires three ENU components")
            return values  # type: ignore[return-value]

        cbf_maximum = _f("SWARM_CBF_MAXIMUM_VELOCITY_M_S", 2.0)
        return cls(
            # Reuses the CBF limit: the active sender is downstream of CBF, so
            # it can never legitimately be asked for more than CBF allows.
            maximum_horizontal_velocity_m_s=cbf_maximum,
            # Deliberately half the horizontal limit. The §20
            # ALTITUDE_SEPARATION stage already commands vertical motion up
            # to the CBF limit; a first flight should not ascend that fast,
            # and a tighter limit can only ever reject commands the wider one
            # would allow.
            maximum_vertical_velocity_m_s=cbf_maximum / 2.0,
            # Matches the emergency ladder's own altitude base, so the first
            # hover sits exactly where the §20 deconfliction logic expects
            # this drone to be.
            hover_altitude_m=_f("SWARM_EMERGENCY_ALTITUDE_BASE_M", 10.0),
            geofence_min_enu_m=_v("SWARM_CBF_GEOFENCE_MIN_ENU_M", "-100,-100,0"),
            geofence_max_enu_m=_v("SWARM_CBF_GEOFENCE_MAX_ENU_M", "100,100,50"),
            maximum_test_duration_s=_f("SWARM_FIRST_FLIGHT_MAX_DURATION_S", 120.0),
            # Same freshness contract the companion safety stack already uses.
            maximum_state_age_s=_f("SWARM_PEER_STATE_MAX_AGE_MS", 500.0) / 1000.0,
            maximum_command_age_s=_f("SWARM_PEER_STATE_MAX_AGE_MS", 500.0) / 1000.0,
        )


@dataclass(frozen=True)
class WarmupContract:
    """Setpoint stream requirements before ARM/OFFBOARD may be requested.

    PX4 itself demands only that `offboard_control_mode` be newer than
    COM_OF_LOSS_T (1.0 s) at the instant of the mode request -- it does not
    require a sample count. The extra requirements below are this project's
    own, deliberately stricter, and are flagged as such: a stream that has
    only just started satisfies PX4 but gives no evidence it will *stay*
    healthy, and a first flight should not discover that after arming.
    """

    # PX4-derived, not chosen here.
    px4_setpoint_timeout_s: float = 1.0
    # Project choices, stricter than PX4. Rationale in the docstring.
    minimum_duration_s: float = 3.0
    minimum_valid_samples: int = 40  # 2 s at the companion's 20 Hz cadence
    maximum_gap_s: float = 0.25  # 5x the 50 ms nominal interval
    # If warmup is interrupted, the sequence restarts from PRECHECK rather
    # than resuming: a gap means the evidence collected so far no longer
    # demonstrates a continuously healthy stream.
    on_interruption: str = "restart_from_precheck"


class AbortAction(str, Enum):
    """What the companion does. Deliberately limited to actions that are
    actually implemented or are inaction -- no autonomous RTL/Land is claimed,
    because none is implemented anywhere in this codebase.
    """

    REFUSE_TO_PROCEED = "refuse_to_proceed"  # pre-arm only; nothing armed yet
    STOP_SENDING_SETPOINTS = "stop_sending_setpoints"
    SEND_ZERO_VELOCITY = "send_zero_velocity"
    REQUEST_POSITION_MODE = "request_position_mode"


@dataclass(frozen=True)
class AbortRule:
    condition: str
    detect: str
    immediate_action: AbortAction
    px4_policy: str
    manual_intervention_required: bool
    recovery_allowed: bool


# Ordered most-severe-first. `stop_sending_setpoints` is the strongest action
# the companion has: after COM_OF_LOSS_T (1.0 s) PX4 itself falls back to
# Position mode (COM_OBL_RC_ACT=0), which is a benign, already-configured
# failsafe -- not something invented here.
ABORT_MATRIX: tuple[AbortRule, ...] = (
    AbortRule(
        condition="self_telemetry_stale",
        # Broadened when the producer was written: an own state that is
        # invalid for any reason (telemetry_stale, no_local_state,
        # enu_transform_failed, unhealthy heartbeat) is equally unusable as
        # the basis for a setpoint, so the producer keys off validity rather
        # than the one reason string originally named here.
        detect="companion own_swarm_state reason != ok (see offboard_abort_conditions)",
        immediate_action=AbortAction.STOP_SENDING_SETPOINTS,
        px4_policy="PX4 holds last setpoint until COM_OF_LOSS_T=1.0s, then Position mode",
        manual_intervention_required=False,
        recovery_allowed=False,
    ),
    AbortRule(
        condition="command_stale",
        detect="OffboardSetpointPreview.inhibit_reason=command_stale",
        immediate_action=AbortAction.STOP_SENDING_SETPOINTS,
        px4_policy="COM_OF_LOSS_T timeout -> Position mode",
        manual_intervention_required=False,
        recovery_allowed=False,
    ),
    AbortRule(
        condition="setpoint_stream_gap",
        detect="interval between previews > WarmupContract.maximum_gap_s",
        immediate_action=AbortAction.STOP_SENDING_SETPOINTS,
        px4_policy="COM_OF_LOSS_T timeout -> Position mode",
        manual_intervention_required=False,
        recovery_allowed=False,
    ),
    AbortRule(
        condition="px4_rejects_offboard",
        detect="COMMAND_ACK for MAV_CMD_DO_SET_MODE != ACCEPTED",
        immediate_action=AbortAction.REFUSE_TO_PROCEED,
        px4_policy="vehicle never left POSCTL; still disarmed if abort precedes ARM",
        manual_intervention_required=True,
        recovery_allowed=True,
    ),
    AbortRule(
        condition="px4_exits_offboard_unexpectedly",
        detect="HEARTBEAT custom_mode main mode != OFFBOARD while sequence expects it",
        immediate_action=AbortAction.STOP_SENDING_SETPOINTS,
        px4_policy="PX4 already chose the mode itself; companion must not fight it",
        manual_intervention_required=True,
        recovery_allowed=False,
    ),
    AbortRule(
        condition="cbf_invalid_or_infeasible",
        detect="CbfCommand.active=False (any reason, incl. cbf_constraints_infeasible)",
        immediate_action=AbortAction.SEND_ZERO_VELOCITY,
        px4_policy="zero velocity is a valid OFFBOARD setpoint; vehicle holds",
        manual_intervention_required=False,
        recovery_allowed=True,
    ),
    AbortRule(
        condition="emergency_supervisor_hold",
        detect="EmergencyDecision.stage=hold",
        immediate_action=AbortAction.SEND_ZERO_VELOCITY,
        px4_policy="zero velocity setpoint maintained",
        manual_intervention_required=False,
        recovery_allowed=True,
    ),
    AbortRule(
        condition="emergency_recommends_position_hold",
        detect="EmergencyDecision.recommended_px4_mode=POSITION_HOLD",
        immediate_action=AbortAction.REQUEST_POSITION_MODE,
        px4_policy="explicit MAV_CMD_DO_SET_MODE to POSCTL via the bridge's existing request_position_mode()",
        manual_intervention_required=False,
        recovery_allowed=False,
    ),
    AbortRule(
        condition="emergency_recommends_rtl_or_land",
        detect="EmergencyDecision.recommended_px4_mode=RTL_OR_LAND",
        immediate_action=AbortAction.REQUEST_POSITION_MODE,
        # Honest limitation, not a silent invention: no autonomous RTL or
        # Land is implemented anywhere in this codebase. The strongest
        # implemented action is Position mode; a human must take it from
        # there. Claiming RTL here would be claiming behaviour that does not
        # exist.
        px4_policy="POSCTL only -- autonomous RTL/Land NOT implemented in this project",
        manual_intervention_required=True,
        recovery_allowed=False,
    ),
    AbortRule(
        condition="geofence_violation_or_risk",
        detect="CbfCommand.reason=outside_geofence, or position outside SWARM_CBF_GEOFENCE_*",
        immediate_action=AbortAction.SEND_ZERO_VELOCITY,
        px4_policy="CBF already refuses; zero velocity held",
        manual_intervention_required=False,
        recovery_allowed=True,
    ),
    AbortRule(
        condition="vertical_velocity_sign_mismatch",
        detect="sign(preview.output_velocity_ned_m_s[2]) != -sign(input_velocity_enu_m_s[2])",
        immediate_action=AbortAction.STOP_SENDING_SETPOINTS,
        px4_policy="frame conversion is untrustworthy; stop rather than command an inverted axis",
        manual_intervention_required=True,
        recovery_allowed=False,
    ),
    AbortRule(
        condition="nan_or_inf_command",
        detect="OffboardSetpointPreview.inhibit_reason=malformed_command",
        immediate_action=AbortAction.STOP_SENDING_SETPOINTS,
        px4_policy="COM_OF_LOSS_T timeout -> Position mode",
        manual_intervention_required=False,
        recovery_allowed=False,
    ),
    AbortRule(
        condition="mavlink_connection_loss",
        # Measured limitation: the affected worker's loop stalls on the
        # blocking MAVLink read, so it produces no frame to carry this
        # condition. The protection still holds -- PX4 times out on its own --
        # but the observation needs a watcher outside the worker, which does
        # not exist. See offboard_abort_conditions for the measurement.
        detect="no HEARTBEAT for VEHICLE_HEARTBEAT_TIMEOUT_S (3.0 s); NOT self-observable",
        immediate_action=AbortAction.STOP_SENDING_SETPOINTS,
        px4_policy="setpoints cannot reach PX4 anyway; PX4 times out -> Position mode",
        manual_intervention_required=True,
        recovery_allowed=False,
    ),
    AbortRule(
        condition="companion_safety_loop_stops",
        detect="no new CompanionSafetyStatus for maximum_command_age_s",
        immediate_action=AbortAction.STOP_SENDING_SETPOINTS,
        px4_policy="COM_OF_LOSS_T timeout -> Position mode",
        manual_intervention_required=False,
        recovery_allowed=False,
    ),
)


@dataclass(frozen=True)
class ActiveOffboardSetpointSenderPlan:
    """Design record for the active sender, frozen before it was written.

    Kept unchanged as a contract now that `active_offboard_setpoint_sender`
    implements it: `must_not` and `authorization_rule` are asserted against
    the real class by test, so the implementation cannot drift away from the
    design that was reviewed.

    `implemented` and `wired_into_live_loop` are separate on purpose. The
    class exists and is fully testable, but nothing in this repository
    constructs it with a real MAVLink sink -- that wiring belongs to the
    ONE_UAV_ACTIVE_SITL_FLIGHT gate.
    """

    consumes: str = "OffboardSetpointPreview (unchanged contract from the shadow sender)"
    layering: tuple[str, ...] = (
        "CompanionSafetyStatus",
        "validate_command",
        "ShadowOffboardSetpointSender.preview -> OffboardSetpointPreview",
        "ActiveOffboardSetpointSender (active_offboard_setpoint_sender.py)",
        "MAVLink set_position_target_local_ned_send",
    )
    must_not: tuple[str, ...] = (
        "bypass CBF",
        "bypass EmergencySupervisor",
        "use pre-CBF nominal command",
        "invent new safety logic",
        "transmit when preview.valid is False",
    )
    authorization_rule: str = (
        "transmit permitted only if drone_id == UAV-01 AND system_id == 1 AND "
        "an explicit opt-in is present AND preview.valid is True"
    )
    implemented: bool = True
    # The bridge now passes a real MAVLink sink -- but only to the vehicle
    # `companion_active_transmit_authorized` clears, so an unauthorized
    # sender holds no transmit path at all rather than merely declining to
    # use one.
    wired_into_live_loop: bool = True


@dataclass(frozen=True)
class VelocityReadiness:
    """Whether PX4's OFFBOARD velocity-mode precondition is satisfied.

    PX4 refuses OFFBOARD velocity control unless
    `failsafe_flags.local_velocity_invalid` is false
    (commander/HealthAndArmingChecks/checks/offboardCheck.cpp). This is a
    *reported* flag: it is never inferred from velocity values, because a
    plausible-looking velocity says nothing about whether the estimator
    considers it valid.

    Every non-false outcome fails closed. In particular `None` (flag absent
    from the telemetry payload) is NOT treated as "valid" -- an unreported
    precondition is an unknown one.
    """

    ready: bool
    reason: str
    observed_value: Any
    age_s: float | None

    def as_dict(self) -> dict[str, Any]:
        return {
            "ready": self.ready,
            "reason": self.reason,
            "observed_value": self.observed_value,
            "age_s": round(self.age_s, 3) if self.age_s is not None else None,
        }


def evaluate_velocity_readiness(
    failsafe_flags: Any,
    now_monotonic_s: float,
    maximum_age_s: float,
) -> VelocityReadiness:
    """Fail-closed evaluation of the OFFBOARD velocity precondition."""
    if not isinstance(failsafe_flags, dict):
        return VelocityReadiness(False, "failsafe_flags_missing", None, None)

    if "local_velocity_invalid" not in failsafe_flags:
        return VelocityReadiness(
            False, "local_velocity_invalid_not_reported", None, None
        )
    value = failsafe_flags["local_velocity_invalid"]
    if value is None:
        return VelocityReadiness(
            False, "local_velocity_invalid_not_reported", None, None
        )
    if not isinstance(value, bool):
        # A string, int, or anything else means the producer changed shape;
        # coercing it would risk turning a truthy "false" string into True.
        return VelocityReadiness(False, "local_velocity_invalid_malformed", value, None)

    received = failsafe_flags.get("received_monotonic_s")
    if not isinstance(received, (int, float)):
        return VelocityReadiness(False, "failsafe_flags_timestamp_missing", value, None)
    age_s = now_monotonic_s - float(received)
    if age_s < 0.0:
        return VelocityReadiness(
            False, "failsafe_flags_timestamp_regressed", value, age_s
        )
    if age_s > maximum_age_s:
        return VelocityReadiness(False, "failsafe_flags_stale", value, age_s)

    if value:
        return VelocityReadiness(False, "local_velocity_invalid", value, age_s)
    return VelocityReadiness(True, "", value, age_s)


def is_authorized_for_first_active_flight(
    drone_id: str, px4_system_id: int, explicit_opt_in: bool
) -> tuple[bool, str]:
    """Authorization gate for the first active flight.

    All three conditions are required. The drone identity is compared against
    a hardcoded constant rather than configuration so that a typo in a config
    file cannot authorize the wrong vehicle -- UAV-02 must be impossible to
    actuate in this gate, not merely disabled by default.
    """
    if drone_id != FIRST_ACTIVE_FLIGHT_DRONE_ID:
        return False, f"drone_not_authorized:{drone_id}"
    if int(px4_system_id) != FIRST_ACTIVE_FLIGHT_SYSTEM_ID:
        return False, f"system_id_not_authorized:{px4_system_id}"
    if int(px4_system_id) == 0:
        # Unreachable given the check above, kept as an explicit guard because
        # target_system 0 is a MAVLink broadcast accepted by every instance.
        return False, "broadcast_system_id_forbidden"
    if not explicit_opt_in:
        return False, "explicit_opt_in_absent"
    return True, ""


def abort_rule_for(condition: str) -> AbortRule | None:
    for rule in ABORT_MATRIX:
        if rule.condition == condition:
            return rule
    return None


def transition_guard(
    state: ReadinessState,
    evidence: dict[str, Any],
    envelope: FlightEnvelope,
    warmup: WarmupContract,
) -> tuple[bool, str]:
    """Evidence required to leave `state`. Returns (may_advance, reason)."""
    if state is ReadinessState.PRECHECK:
        for key in (
            "preflight_checks_pass",
            "local_position_invalid_false",
            "companion_safety_running",
            "peer_state_fresh",
        ):
            if not evidence.get(key):
                return False, f"precheck_missing:{key}"
        if evidence.get("armed") is not False:
            return False, "precheck_requires_disarmed"
        # PX4's own OFFBOARD velocity precondition. Absent evidence fails
        # closed rather than being skipped: this guard exists precisely
        # because the flag used to be unobservable.
        velocity = evidence.get("velocity_readiness")
        if not isinstance(velocity, VelocityReadiness):
            return False, "precheck_missing:velocity_readiness"
        if not velocity.ready:
            return False, f"velocity_not_ready:{velocity.reason}"
        # Single-writer interlock: exactly one OFFBOARD writer, and it must
        # be the companion for this gate.
        authority = evidence.get("offboard_authority")
        if not isinstance(authority, dict):
            return False, "precheck_missing:offboard_authority"
        if authority.get("active_offboard_writer_count", 99) > 1:
            return False, "multiple_offboard_writers_permitted"
        if not authority.get("companion_writer_permitted"):
            return False, f"companion_writer_not_permitted:{authority.get('reason','')}"
        return True, ""

    if state is ReadinessState.SETPOINT_WARMUP:
        if evidence.get("warmup_duration_s", 0.0) < warmup.minimum_duration_s:
            return False, "warmup_duration_insufficient"
        if evidence.get("valid_sample_count", 0) < warmup.minimum_valid_samples:
            return False, "warmup_samples_insufficient"
        if evidence.get("max_gap_s", math.inf) > warmup.maximum_gap_s:
            return False, "warmup_stream_gap_exceeded"
        if not evidence.get("all_previews_valid"):
            return False, "warmup_contains_invalid_preview"
        return True, ""

    if state is ReadinessState.ARM:
        if not evidence.get("armed"):
            return False, "arm_not_confirmed"
        if evidence.get("stream_still_healthy") is not True:
            return False, "arm_requires_healthy_stream"
        return True, ""

    if state is ReadinessState.OFFBOARD:
        if evidence.get("nav_state") != PX4_FACTS["nav_state_offboard"]["value"]:
            return False, "offboard_not_confirmed"
        return True, ""

    if state is ReadinessState.CONTROLLED_ASCENT:
        altitude = evidence.get("altitude_m", 0.0)
        if altitude < envelope.hover_altitude_m - 0.5:
            return False, "ascent_target_not_reached"
        if abs(evidence.get("vertical_velocity_m_s", 0.0)) > envelope.maximum_vertical_velocity_m_s + 1e-6:
            return False, "ascent_velocity_exceeds_envelope"
        return True, ""

    if state is ReadinessState.HOVER:
        if evidence.get("hover_duration_s", 0.0) < 5.0:
            return False, "hover_duration_insufficient"
        return True, ""

    if state is ReadinessState.ZERO_VELOCITY:
        speed = evidence.get("speed_m_s", math.inf)
        if speed > 0.1:
            return False, "not_at_zero_velocity"
        return True, ""

    if state is ReadinessState.EXIT_OFFBOARD:
        if evidence.get("nav_state") == PX4_FACTS["nav_state_offboard"]["value"]:
            return False, "still_in_offboard"
        return True, ""

    if state is ReadinessState.LAND_DISARM:
        if evidence.get("armed") is not False:
            return False, "still_armed"
        return True, ""

    return False, f"unknown_state:{state}"
