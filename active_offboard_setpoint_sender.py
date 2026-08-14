"""Active PX4 offboard setpoint sender.

The last stage of the companion pipeline (architecture doc §24), and the
only object in this project that is *capable* of putting a setpoint on the
wire:

    nominal -> local CBF -> EmergencySupervisor -> validate_command
            -> ShadowOffboardSetpointSender.preview -> OffboardSetpointPreview
            -> ActiveOffboardSetpointSender          <-- this module
            -> MAVLink set_position_target_local_ned_send

It implements exactly `one_uav_active_readiness.ActiveOffboardSetpointSenderPlan`,
which was frozen and reviewed before any of this code existed. The plan's own
plumbing -- CBF, EmergencySupervisor, ABORT_MATRIX, WarmupContract -- was
never vehicle-count-specific; only the identity gate this class consults was,
and that gate is now `two_uav_active_readiness.is_authorized_for_two_uav_active_flight`
(see `_authorization`), not the narrower one-vehicle gate the plan's name
still refers to.

WHY THE SAFETY STACK CANNOT BE BYPASSED HERE
--------------------------------------------
The plan forbids bypassing CBF, bypassing the EmergencySupervisor, and using
the pre-CBF nominal command. None of those is prevented by a check in this
file -- they are prevented by what this class is allowed to touch. `step()`
accepts an `OffboardSetpointPreview` and nothing else. It has no reference to
a CompanionSafetyStatus, a CbfCommandGate, an EmergencySupervisor, or a
nominal controller, so there is no expression it could evaluate that would
reach a pre-CBF velocity. A future edit that wanted to bypass CBF would have
to change this module's signature, which is a visible change rather than a
silent one.

The plan also forbids inventing new safety logic. This module therefore adds
no limits of its own: every threshold it applies is read from `ABORT_MATRIX`
or `WarmupContract`, and the velocity/staleness/authority rules are the ones
already enforced upstream. What it adds is *sequencing* -- the cross-frame
state (stream continuity, abort latching, counters) that no single-frame
evaluation can hold.

NO TRANSPORT IMPORT
-------------------
This module does not import pymavlink, sockets, or MQTT. The wire call is
injected as a callable whose signature is exactly
`set_position_target_local_ned_send`, so wiring it in the bridge is a
one-line, reviewable change and testing it needs no vehicle. A sender
constructed without a sink cannot transmit at all.

RUNTIME WIRING
--------------
The MAVLink bridge attaches the real sink only when both authority and active
flight readiness gates pass. Its mission flight gate then requires this
sender's measured warmup contract before requesting PX4 OFFBOARD.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from enum import Enum
import math
import time
from typing import Any, Callable, Iterable, Sequence

from offboard_authority import companion_active_transmit_authorized
from offboard_setpoint_sender import (
    INHIBIT_VEHICLE_NOT_READY,
    MAV_FRAME_LOCAL_NED,
    OFFBOARD_VELOCITY_TYPE_MASK,
    OffboardSetpointPreview,
    Vector3,
    ZERO_VELOCITY,
    enu_to_ned_velocity,
)
from one_uav_active_readiness import ABORT_MATRIX, AbortAction, AbortRule, WarmupContract
from two_uav_active_readiness import is_authorized_for_two_uav_active_flight


# Abort conditions this sender derives on its own, from the preview it is
# given plus its own timing. Kept as a mapping so the derivation for each
# condition is stated next to the condition name.
SELF_DERIVED_CONDITIONS = (
    "command_stale",
    "nan_or_inf_command",
    "setpoint_stream_gap",
    "vertical_velocity_sign_mismatch",
    "emergency_supervisor_hold",
    "emergency_recommends_position_hold",
    "emergency_recommends_rtl_or_land",
)

# Abort conditions the sender structurally cannot see and the caller must
# report. Each needs knowledge the preview does not carry: MAVLink link
# health, PX4 mode transitions, CBF gate state, own-telemetry freshness, or
# the liveness of the companion loop that produces previews in the first
# place. Enumerated rather than left implicit so that
# SELF_DERIVED_CONDITIONS + CALLER_REPORTED_CONDITIONS can be asserted to
# cover ABORT_MATRIX exactly -- a new rule with no handler fails the test.
CALLER_REPORTED_CONDITIONS = (
    "self_telemetry_stale",
    "px4_rejects_offboard",
    "px4_exits_offboard_unexpectedly",
    "cbf_invalid_or_infeasible",
    "geofence_violation_or_risk",
    "mavlink_connection_loss",
    "companion_safety_loop_stops",
)


# Frames retained for inspection. The sender runs at the companion's 20 Hz
# inside a process that stays up for days, so the history is a ring buffer:
# an unbounded list would be a slow leak. 200 frames is 10 s, enough to see
# what led to an abort. The counters below it are cumulative and unaffected.
MAX_RETAINED_FRAMES = 200


class TransmitDecision(str, Enum):
    TRANSMIT = "transmit"
    TRANSMIT_ZERO = "transmit_zero"
    TRANSMIT_WARMUP = "transmit_warmup"
    WITHHOLD = "withhold"
    REQUEST_POSITION_MODE = "request_position_mode"


@dataclass(frozen=True)
class SetpointFrame:
    """The record of one `step()`, whether or not anything went on the wire.

    Emitted for every frame including withheld ones: a flight log that only
    records transmissions cannot distinguish "healthy and quiet" from
    "aborted and silent".
    """

    sequence: int
    decision: TransmitDecision
    reason: str
    abort_condition: str | None
    abort_action: str | None
    transmitted: bool
    velocity_ned_m_s: Vector3
    latched: bool
    interval_s: float | None
    evaluated_monotonic_s: float

    def as_dict(self) -> dict[str, Any]:
        return {
            "sequence": self.sequence,
            "decision": self.decision.value,
            "reason": self.reason,
            "abort_condition": self.abort_condition,
            "abort_action": self.abort_action,
            "transmitted": self.transmitted,
            "velocity_ned_m_s": list(self.velocity_ned_m_s),
            "latched": self.latched,
            "interval_s": (
                round(self.interval_s, 4) if self.interval_s is not None else None
            ),
            "evaluated_monotonic_s": round(self.evaluated_monotonic_s, 4),
        }


TransmitSink = Callable[..., Any]
PositionModeSink = Callable[[], Any]


class ActiveOffboardSetpointSender:
    """Streams validated setpoints to PX4, subject to every gate in the plan.

    Deliberately a separate class from `ShadowOffboardSetpointSender` rather
    than a mode of it: the shadow sender's guarantee is that it *cannot*
    transmit, and that guarantee would be destroyed by adding an `if active:`
    branch to it.
    """

    def __init__(
        self,
        drone_id: str,
        px4_system_id: int,
        *,
        explicit_opt_in: bool = False,
        transmit: TransmitSink | None = None,
        request_position_mode: PositionModeSink | None = None,
        warmup: WarmupContract | None = None,
        environment: dict[str, str] | None = None,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        if not str(drone_id).strip():
            raise ValueError("drone_id is required")
        # `explicit_opt_in` is a constructor argument and never read from the
        # environment. Enabling the first active flight must be a code/CLI
        # decision at the construction site, not a flag a deploy could flip.
        # `is True` rather than truthiness: a stray non-empty string must not
        # authorize a flight.
        if explicit_opt_in is not True and explicit_opt_in is not False:
            raise ValueError("explicit_opt_in must be a bool")
        self.drone_id = str(drone_id)
        self.px4_system_id = int(px4_system_id)
        self.explicit_opt_in = explicit_opt_in
        self._transmit = transmit
        self._request_position_mode = request_position_mode
        self.warmup = warmup or WarmupContract()
        self._environment = environment
        self._clock = clock

        self.sequence = 0
        self.latched_abort: str | None = None
        self.last_step_monotonic_s: float | None = None
        self.transmit_count = 0
        self.zero_transmit_count = 0
        self.warmup_transmit_count = 0
        self.withheld_count = 0
        # Measured stream continuity, so WarmupContract can be evaluated
        # against what actually went on the wire rather than against how
        # often step() was called. A frame that was withheld is not part of
        # the stream PX4 sees.
        self.stream_started_monotonic_s: float | None = None
        self.last_transmit_monotonic_s: float | None = None
        self.max_transmit_gap_s = 0.0
        self.position_mode_request_count = 0
        # This class has no arming code path at all; the counter exists so the
        # zero-actuation proof can assert it rather than assume it.
        self.arm_command_count = 0
        self.frames: deque[SetpointFrame] = deque(maxlen=MAX_RETAINED_FRAMES)

    @property
    def transmit_sink_attached(self) -> bool:
        return self._transmit is not None

    @property
    def holds_offboard_authority(self) -> bool:
        """Whether this sender is currently the thing driving OFFBOARD.

        Consulted by the legacy path, which otherwise treats any observed
        OFFBOARD as a mode nothing is driving and recovers out of it. Goes
        false the moment an abort latches, so the legacy recovery becomes the
        backstop for an aborted companion rather than its adversary.
        """
        return self.transmit_sink_attached and self.latched_abort is None

    # -- gates ------------------------------------------------------------

    def _abort_condition(
        self,
        preview: OffboardSetpointPreview,
        interval_s: float | None,
        reported: Sequence[str],
    ) -> str | None:
        """First active condition in ABORT_MATRIX order.

        ABORT_MATRIX is documented as most-severe-first, so the matrix's own
        ordering is the severity ranking; none is invented here.
        """
        active: set[str] = {c for c in reported if c}

        if preview.inhibit_reason == "command_stale":
            active.add("command_stale")
        if preview.inhibit_reason == "malformed_command":
            active.add("nan_or_inf_command")
        # A negative interval is a broken clock, which is a broken stream:
        # the timing evidence the gap rule relies on is unusable either way.
        if interval_s is not None and (
            interval_s < 0.0 or interval_s > self.warmup.maximum_gap_s
        ):
            active.add("setpoint_stream_gap")
        if _vertical_sign_mismatch(preview):
            active.add("vertical_velocity_sign_mismatch")
        if preview.emergency_stage == "hold":
            active.add("emergency_supervisor_hold")
        if preview.recommended_px4_mode == "POSITION_HOLD":
            active.add("emergency_recommends_position_hold")
        if preview.recommended_px4_mode == "RTL_OR_LAND":
            active.add("emergency_recommends_rtl_or_land")

        for rule in ABORT_MATRIX:
            if rule.condition in active:
                return rule.condition
        return None

    def _authorization(self, preview: OffboardSetpointPreview) -> tuple[bool, str]:
        """Re-resolved every frame, never cached.

        Both gates are consulted because they answer different questions: the
        authority decides which of the two OFFBOARD writers may exist at all,
        the readiness gate decides whether this specific vehicle is one of
        the ones cleared for active flight (TWO_UAV_ACTIVE_SITL_FLIGHT, as of
        two_uav_active_readiness; ONE_UAV_ACTIVE_SITL_FLIGHT's own narrower
        gate in one_uav_active_readiness is unchanged but no longer the one
        consulted here). Either refusing is a refusal.
        """
        permitted, reason = companion_active_transmit_authorized(
            self.drone_id, self.px4_system_id, self._environment
        )
        if not permitted:
            return False, f"authority_refused:{reason}"

        cleared, reason = is_authorized_for_two_uav_active_flight(
            self.drone_id, self.px4_system_id, self.explicit_opt_in
        )
        if not cleared:
            return False, f"readiness_gate_refused:{reason}"

        # The preview must have been built for this vehicle. A preview is a
        # plain dataclass and could have been produced by another drone's
        # sender; transmitting it here would command the wrong airframe with
        # another airframe's CBF solution.
        if preview.drone_id != self.drone_id:
            return False, f"preview_drone_mismatch:{preview.drone_id}"
        if preview.px4_target_system != self.px4_system_id:
            return False, f"preview_system_id_mismatch:{preview.px4_target_system}"
        # Belt-and-braces at the wire boundary: MAVLink target_system 0 is a
        # broadcast that every PX4 instance accepts. Already refused above,
        # repeated here because this is the last line before the sink.
        if preview.px4_target_system == 0:
            return False, "broadcast_system_id_forbidden"
        return True, ""

    def _wire_consistent(self, preview: OffboardSetpointPreview) -> tuple[bool, str]:
        """The preview's own fields must still agree with each other.

        Not a new safety rule -- a re-derivation of the conversion the shadow
        sender already performed. It catches a hand-built or mutated preview,
        which is the only way a bad vector could reach this point given that
        `valid` was already checked.
        """
        if preview.px4_frame != MAV_FRAME_LOCAL_NED:
            return False, f"unexpected_frame:{preview.px4_frame}"
        if preview.px4_type_mask != OFFBOARD_VELOCITY_TYPE_MASK:
            return False, f"unexpected_type_mask:{preview.px4_type_mask}"
        if not all(math.isfinite(v) for v in preview.output_velocity_ned_m_s):
            return False, "non_finite_output_velocity"
        expected = enu_to_ned_velocity(preview.input_velocity_enu_m_s)
        if any(
            abs(a - b) > 1e-9
            for a, b in zip(expected, preview.output_velocity_ned_m_s)
        ):
            return False, "frame_conversion_inconsistent"
        return True, ""

    # -- loop -------------------------------------------------------------

    def step(
        self,
        preview: OffboardSetpointPreview,
        *,
        now_monotonic_s: float | None = None,
        reported_conditions: Iterable[str] = (),
    ) -> SetpointFrame:
        """Evaluate one frame and transmit if, and only if, every gate holds.

        `reported_conditions` carries the abort conditions in
        CALLER_REPORTED_CONDITIONS that only the caller can observe. Unknown
        strings are ignored rather than rejected: an unrecognised condition
        matches no rule and so cannot silently authorize anything.
        """
        now = self._clock() if now_monotonic_s is None else float(now_monotonic_s)
        self.sequence += 1
        interval_s = (
            None
            if self.last_step_monotonic_s is None
            else now - self.last_step_monotonic_s
        )
        self.last_step_monotonic_s = now

        # A latched abort outranks everything, including a subsequently
        # healthy frame. Without this, a condition that flaps would resume
        # transmission on its own, which is the one thing an abort must not
        # allow.
        if self.latched_abort is not None:
            return self._record(
                TransmitDecision.WITHHOLD,
                f"abort_latched:{self.latched_abort}",
                self.latched_abort,
                None,
                False,
                ZERO_VELOCITY,
                interval_s,
                now,
            )

        condition = self._abort_condition(
            preview, interval_s, tuple(reported_conditions)
        )
        if condition is not None:
            return self._handle_abort(condition, interval_s, now)

        # WARMUP. PX4 will not accept a switch to OFFBOARD unless it is
        # already receiving setpoints newer than COM_OF_LOSS_T, so a sender
        # that only transmits once the vehicle is armed and in OFFBOARD can
        # never bootstrap the mode it needs. WarmupContract anticipated this;
        # this is where it is implemented.
        #
        # The relaxation is deliberately narrow. Only the VEHICLE_NOT_READY
        # category qualifies -- the command is good, PX4 simply is not in a
        # state to use it -- and only a ZERO velocity is sent, never the real
        # vector. A real velocity streamed at a vehicle not yet in OFFBOARD
        # would take effect the instant the mode engaged; zero means the
        # vehicle holds at the moment of handover, which is the only safe
        # initial condition. COMMAND_INTEGRITY inhibits are untouched: an
        # untrustworthy command is not made trustworthy by warmup.
        warmup = (
            not preview.valid
            and preview.inhibit_category == INHIBIT_VEHICLE_NOT_READY
        )
        if not preview.valid and not warmup:
            return self._record(
                TransmitDecision.WITHHOLD,
                f"preview_invalid:{preview.inhibit_reason or 'unspecified'}",
                None,
                None,
                False,
                ZERO_VELOCITY,
                interval_s,
                now,
            )

        authorized, reason = self._authorization(preview)
        if not authorized:
            return self._record(
                TransmitDecision.WITHHOLD,
                reason,
                None,
                None,
                False,
                ZERO_VELOCITY,
                interval_s,
                now,
            )

        if warmup:
            # Built from the sender's own fields rather than the preview's,
            # for the same reason the abort hold is: a preview that could not
            # be validated is not a source to relay.
            transmitted = self._send_zero()
            self.warmup_transmit_count += 1
            return self._record(
                TransmitDecision.TRANSMIT_WARMUP,
                preview.inhibit_reason,
                None,
                None,
                transmitted,
                ZERO_VELOCITY,
                interval_s,
                now,
            )

        consistent, reason = self._wire_consistent(preview)
        if not consistent:
            return self._record(
                TransmitDecision.WITHHOLD,
                reason,
                None,
                None,
                False,
                ZERO_VELOCITY,
                interval_s,
                now,
            )

        transmitted = self._send(preview, preview.output_velocity_ned_m_s)
        return self._record(
            TransmitDecision.TRANSMIT,
            "" if transmitted else "no_transmit_sink",
            None,
            None,
            transmitted,
            preview.output_velocity_ned_m_s,
            interval_s,
            now,
        )

    def _handle_abort(
        self, condition: str, interval_s: float | None, now: float
    ) -> SetpointFrame:
        rule = _rule_for(condition)
        # Latching is read from the matrix rather than restated: a rule whose
        # recovery_allowed is False must never resume on its own.
        if not rule.recovery_allowed:
            self.latched_abort = condition

        if rule.immediate_action is AbortAction.SEND_ZERO_VELOCITY:
            transmitted = self._send_zero()
            return self._record(
                TransmitDecision.TRANSMIT_ZERO,
                condition,
                condition,
                rule.immediate_action.value,
                transmitted,
                ZERO_VELOCITY,
                interval_s,
                now,
            )

        if rule.immediate_action is AbortAction.REQUEST_POSITION_MODE:
            self.position_mode_request_count += 1
            if self._request_position_mode is not None:
                self._request_position_mode()
            return self._record(
                TransmitDecision.REQUEST_POSITION_MODE,
                condition,
                condition,
                rule.immediate_action.value,
                False,
                ZERO_VELOCITY,
                interval_s,
                now,
            )

        # STOP_SENDING_SETPOINTS and REFUSE_TO_PROCEED are both "send
        # nothing". They differ only in whether anything is airborne yet,
        # which the matrix records and this sender does not need to know:
        # PX4 falls back to Position mode after COM_OF_LOSS_T either way.
        return self._record(
            TransmitDecision.WITHHOLD,
            condition,
            condition,
            rule.immediate_action.value,
            False,
            ZERO_VELOCITY,
            interval_s,
            now,
        )

    # -- wire -------------------------------------------------------------

    def _send(self, preview: OffboardSetpointPreview, velocity_ned: Vector3) -> bool:
        """Call the injected sink. Returns whether anything was actually sent.

        A sender with no sink is inert: it evaluates and logs every gate but
        cannot transmit, which is how it is used everywhere in the repository
        today.
        """
        if self._transmit is None:
            return False
        north, east, down = velocity_ned
        # Argument order and units mirror pymavlink's
        # set_position_target_local_ned_send exactly, so wiring the real sink
        # is a substitution rather than a translation.
        self._transmit(
            preview.boot_time_ms,
            preview.px4_target_system,
            preview.px4_target_component,
            preview.px4_frame,
            preview.px4_type_mask,
            0.0,
            0.0,
            0.0,
            north,
            east,
            down,
            0.0,
            0.0,
            0.0,
            0.0,
            preview.yaw_rate_rad_s,
        )
        self._record_transmit()
        return True

    def _send_zero(self) -> bool:
        """Zero velocity is a valid OFFBOARD setpoint that holds the vehicle.

        Built here rather than reusing the preview's own fields because the
        preview that triggered the abort may itself be untrustworthy.
        """
        if self._transmit is None:
            return False
        self._transmit(
            int(self._clock() * 1000.0) & 0xFFFFFFFF,
            self.px4_system_id,
            1,
            MAV_FRAME_LOCAL_NED,
            OFFBOARD_VELOCITY_TYPE_MASK,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
        )
        self._record_transmit()
        self.zero_transmit_count += 1
        return True

    def _record_transmit(self) -> None:
        """Stream continuity, measured on what actually went on the wire."""
        self.transmit_count += 1
        now = self._clock() if self.last_step_monotonic_s is None else self.last_step_monotonic_s
        if self.stream_started_monotonic_s is None:
            self.stream_started_monotonic_s = now
        elif self.last_transmit_monotonic_s is not None:
            gap = now - self.last_transmit_monotonic_s
            if gap > self.max_transmit_gap_s:
                self.max_transmit_gap_s = gap
        self.last_transmit_monotonic_s = now

    def _record(
        self,
        decision: TransmitDecision,
        reason: str,
        abort_condition: str | None,
        abort_action: str | None,
        transmitted: bool,
        velocity: Vector3,
        interval_s: float | None,
        now: float,
    ) -> SetpointFrame:
        if decision in (
            TransmitDecision.WITHHOLD,
            TransmitDecision.REQUEST_POSITION_MODE,
        ):
            self.withheld_count += 1
        frame = SetpointFrame(
            sequence=self.sequence,
            decision=decision,
            reason=reason,
            abort_condition=abort_condition,
            abort_action=abort_action,
            transmitted=transmitted,
            velocity_ned_m_s=tuple(float(v) for v in velocity),  # type: ignore[arg-type]
            latched=self.latched_abort is not None,
            interval_s=interval_s,
            evaluated_monotonic_s=now,
        )
        self.frames.append(frame)
        return frame

    def status(self) -> dict[str, Any]:
        return {
            "drone_id": self.drone_id,
            "px4_system_id": self.px4_system_id,
            "explicit_opt_in": self.explicit_opt_in,
            "transmit_sink_attached": self._transmit is not None,
            "sequence": self.sequence,
            "transmit_count": self.transmit_count,
            "zero_transmit_count": self.zero_transmit_count,
            "warmup_transmit_count": self.warmup_transmit_count,
            "withheld_count": self.withheld_count,
            "stream_duration_s": (
                None
                if self.stream_started_monotonic_s is None
                or self.last_transmit_monotonic_s is None
                else round(
                    self.last_transmit_monotonic_s - self.stream_started_monotonic_s, 3
                )
            ),
            "max_transmit_gap_s": round(self.max_transmit_gap_s, 4),
            "position_mode_request_count": self.position_mode_request_count,
            "arm_command_count": self.arm_command_count,
            "latched_abort": self.latched_abort,
            "implementation": "active",
        }


def _sign(value: float) -> int:
    if value > 1e-9:
        return 1
    if value < -1e-9:
        return -1
    return 0


def _vertical_sign_mismatch(preview: OffboardSetpointPreview) -> bool:
    """ABORT_MATRIX condition `vertical_velocity_sign_mismatch`, literally:

        sign(output_velocity_ned[2]) != -sign(input_velocity_enu[2])

    An inverted vertical axis is the most dangerous frame bug available here
    -- it turns a climb command into a descent -- so it is re-checked at the
    wire boundary and not only where the conversion happens. Zero maps to
    zero, so a non-zero `down` paired with a zero `up` is a mismatch too.
    """
    up = preview.input_velocity_enu_m_s[2]
    down = preview.output_velocity_ned_m_s[2]
    if not math.isfinite(up) or not math.isfinite(down):
        return True
    return _sign(down) != -_sign(up)


def _rule_for(condition: str) -> AbortRule:
    for rule in ABORT_MATRIX:
        if rule.condition == condition:
            return rule
    raise KeyError(f"no abort rule for condition: {condition}")
