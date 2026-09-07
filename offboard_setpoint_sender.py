"""Shadow-only PX4 offboard setpoint sender.

Last stage of the companion pipeline (architecture doc §24):

    nominal -> local CBF -> EmergencySupervisor -> command validator
            -> OffboardSetpointSender -> (shadow serialization only)

This module builds *exactly* the PX4 setpoint that a future active sender
would transmit -- same MAVLink message, same frame, same type mask, same
units -- and then does not transmit it. It is the preview/serialization half
of the split, deliberately kept as a separate class from any future
`ActiveOffboardSetpointSender`.

HARD TRANSMIT LOCK (architecture requirement, not a config flag)
---------------------------------------------------------------
Transmission is impossible from this module by construction, not by an
`if enabled:` that a config typo could flip:

* This module does not import pymavlink, and never has. It cannot call
  `mav.set_position_target_local_ned_send` even if asked to.
* It never receives, stores, or is reachable from a MAVLink connection,
  socket, or MQTT client -- `preview()` takes only plain data.
* The MAVLink message type and frame are recorded as descriptive *strings*
  and integer constants for the preview record; there is no code path that
  turns them back into a wire message.
* `transmit_allowed` / `transmit_attempted` are constant `False` on the
  frozen dataclass, and are not settable from configuration, environment, or
  any `preview()` argument.

Reused rather than reinvented (see Phase 0 audit): the canonical PX4
mechanism already in `mavlink_manual_bridge.py` is
`set_position_target_local_ned_send` with a `MAV_FRAME_*` frame and one of
the `OFFBOARD_*_TYPE_MASK` constants. A world-frame 3-D velocity setpoint is
`MAV_FRAME_LOCAL_NED` combined with `OFFBOARD_VELOCITY_TYPE_MASK` (1479 --
uses vx/vy/vz + yaw_rate). That mask already exists; the bridge currently
only pairs LOCAL_NED with the altitude-hold mask 1507, which *ignores* vz and
so could not express §20's ALTITUDE_SEPARATION vertical command.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Any


Vector3 = tuple[float, float, float]

ZERO_VELOCITY: Vector3 = (0.0, 0.0, 0.0)

# Mirrors of the pymavlink constants the bridge already uses. Duplicated as
# plain ints on purpose: this module must not import pymavlink (see the hard
# transmit lock above). test_offboard_setpoint_sender.py asserts these still
# equal the real pymavlink values, so the mirror cannot silently drift.
MAV_FRAME_LOCAL_NED = 1
MAV_COMP_ID_AUTOPILOT1 = 1
OFFBOARD_VELOCITY_TYPE_MASK = 1479

INTENDED_MESSAGE_TYPE = "SET_POSITION_TARGET_LOCAL_NED"

# The single authority string produced by companion_safety. Anything else is
# refused: a setpoint must originate from the validated companion pipeline,
# not from the server's observability-only shadow gate (which carries
# "shadow_safety_gate_only") or any future source.
AUTHORIZED_SOURCE_AUTHORITY = "shadow_companion_only"

# §20 stages whose meaning is "recommend a PX4 mode change", which this task
# must not perform. The velocity preview stays inert (zero) and the
# recommendation is carried as an annotation only.
MODE_RECOMMENDATION_STAGES = frozenset(
    {"recommend_position_hold", "recommend_rtl_or_land"}
)

# Inhibit categories. See ShadowOffboardSetpointSender._inhibit for why the
# distinction exists and how each one treats the previewed vector.
INHIBIT_COMMAND_INTEGRITY = "command_integrity"
INHIBIT_VEHICLE_NOT_READY = "vehicle_not_ready"


def enu_to_ned_velocity(velocity_enu_m_s: Vector3) -> Vector3:
    """ENU (east, north, up) -> NED (north, east, down).

        north = enu_y      east = enu_x      down = -enu_z

    Same convention as `swarm_state.ned_to_enu`, which this inverts. Applies
    to velocity vectors only -- never to a position origin.
    """
    east, north, up = velocity_enu_m_s
    return north, east, -up


@dataclass(frozen=True)
class OffboardSetpointPreview:
    drone_id: str
    source_authority: str
    input_velocity_enu_m_s: Vector3
    output_velocity_ned_m_s: Vector3
    px4_target_system: int
    px4_target_component: int
    px4_frame: int
    px4_type_mask: int
    intended_message_type: str
    yaw_rate_rad_s: float
    boot_time_ms: int
    evaluated_monotonic_s: float
    command_age_ms: float | None
    emergency_stage: str
    recommended_px4_mode: str | None
    valid: bool
    inhibited: bool
    inhibit_reason: str
    inhibit_category: str

    # Constant by construction. Not fields, so no dataclass constructor
    # argument, environment variable, or preview() input can set them.
    @property
    def transmit_allowed(self) -> bool:
        return False

    @property
    def transmit_attempted(self) -> bool:
        return False

    def as_dict(self) -> dict[str, Any]:
        return {
            "drone_id": self.drone_id,
            "source_authority": self.source_authority,
            "input_frame": "ENU",
            "input_velocity_enu_m_s": list(self.input_velocity_enu_m_s),
            "output_frame": "NED",
            "output_velocity_ned_m_s": list(self.output_velocity_ned_m_s),
            "px4_target_system": self.px4_target_system,
            "px4_target_component": self.px4_target_component,
            "px4_frame": self.px4_frame,
            "px4_type_mask": self.px4_type_mask,
            "intended_message_type": self.intended_message_type,
            "yaw_rate_rad_s": self.yaw_rate_rad_s,
            "boot_time_ms": self.boot_time_ms,
            "evaluated_monotonic_s": round(self.evaluated_monotonic_s, 4),
            "command_age_ms": (
                round(self.command_age_ms, 2)
                if self.command_age_ms is not None
                else None
            ),
            "emergency_stage": self.emergency_stage,
            "recommended_px4_mode": self.recommended_px4_mode,
            "valid": self.valid,
            "inhibited": self.inhibited,
            "inhibit_reason": self.inhibit_reason,
            "inhibit_category": self.inhibit_category,
            "transmit_allowed": self.transmit_allowed,
            "transmit_attempted": self.transmit_attempted,
            "authority": "shadow_offboard_setpoint_preview_only",
        }


class ShadowOffboardSetpointSender:
    """Builds would-be PX4 setpoints and never transmits them.

    Deliberately NOT named `OffboardSetpointSender`: the name records that
    this is the shadow half. A future active sender is a separate class with
    a separate review, so that enabling transmission is an explicit code
    change rather than a flag flip.

    This class performs no safety reasoning. It converts an
    already-validated command into PX4 representation and decides only
    whether that representation is fresh/consistent enough to be previewed as
    valid. All collision-avoidance and fallback logic lives upstream in
    CbfCommandGate / EmergencySupervisor / validate_command.
    """

    def __init__(
        self,
        drone_id: str,
        px4_system_id: int,
        maximum_velocity_m_s: float,
        maximum_command_age_s: float,
    ) -> None:
        if not drone_id.strip():
            raise ValueError("drone_id is required")
        if not 1 <= int(px4_system_id) <= 255:
            raise ValueError("px4_system_id is out of range")
        if not math.isfinite(maximum_velocity_m_s) or maximum_velocity_m_s <= 0.0:
            raise ValueError("maximum_velocity_m_s must be positive and finite")
        if not math.isfinite(maximum_command_age_s) or maximum_command_age_s <= 0.0:
            raise ValueError("maximum_command_age_s must be positive and finite")
        self.drone_id = drone_id
        self.px4_system_id = int(px4_system_id)
        self.maximum_velocity_m_s = float(maximum_velocity_m_s)
        self.maximum_command_age_s = float(maximum_command_age_s)
        self.preview_count = 0
        self.inhibited_count = 0
        self.integrity_inhibited_count = 0
        self.not_ready_inhibited_count = 0
        # Counters that must stay at zero for the whole task; asserted by the
        # zero-actuation proof rather than merely assumed.
        self.transmit_attempt_count = 0
        self.arm_command_count = 0
        self.mode_change_count = 0

    def _inhibit(
        self,
        status: Any,
        command_age_s: float | None,
        px4_main_mode: int | None,
        px4_armed: bool | None,
        px4_offboard_main_mode: int,
    ) -> tuple[str, str]:
        """Return (reason, category). Empty reason means "would transmit".

        Two categories, because they demand opposite handling of the vector:

        COMMAND_INTEGRITY -- the command itself cannot be trusted (stale,
        malformed, over-limit, rejected by the validator, wrong authority, or
        a supervisor stage that is a mode recommendation rather than a
        velocity). The previewed vector is forced to zero: emitting the last
        good value here is exactly the stale-replay failure the gate exists
        to prevent.

        VEHICLE_NOT_READY -- the command is perfectly good, but PX4 is not in
        a state that could accept it (disarmed, not in OFFBOARD, mode not yet
        known). Zeroing here would destroy the very thing a shadow sender is
        for: showing what *would* be sent once the vehicle is ready. The real
        converted setpoint is preserved for inspection, and `valid` is still
        False so nothing downstream may treat it as transmittable.
        """
        authority = status.as_dict().get("authority", "")
        if authority != AUTHORIZED_SOURCE_AUTHORITY:
            return "unauthorized_source", INHIBIT_COMMAND_INTEGRITY
        if not status.output_valid:
            return "command_validator_rejected", INHIBIT_COMMAND_INTEGRITY
        velocity = status.output_velocity_enu_m_s
        if len(velocity) != 3 or not all(math.isfinite(v) for v in velocity):
            return "malformed_command", INHIBIT_COMMAND_INTEGRITY
        if math.sqrt(sum(v * v for v in velocity)) > self.maximum_velocity_m_s + 1e-6:
            return "command_exceeds_velocity_limit", INHIBIT_COMMAND_INTEGRITY
        if command_age_s is None:
            return "missing_command_timestamp", INHIBIT_COMMAND_INTEGRITY
        if command_age_s < 0.0:
            return "command_timestamp_regressed", INHIBIT_COMMAND_INTEGRITY
        if command_age_s > self.maximum_command_age_s:
            return "command_stale", INHIBIT_COMMAND_INTEGRITY
        if status.emergency.stage.value in MODE_RECOMMENDATION_STAGES:
            return "emergency_recommends_mode_change", INHIBIT_COMMAND_INTEGRITY
        # A future active sender may only stream while PX4 is actually armed
        # and in OFFBOARD. Unknown (None) is treated as not-satisfied: fail
        # closed rather than assume.
        if px4_armed is not True:
            return "px4_not_armed", INHIBIT_VEHICLE_NOT_READY
        if px4_main_mode is None:
            return "px4_mode_unknown", INHIBIT_VEHICLE_NOT_READY
        if px4_main_mode != px4_offboard_main_mode:
            return "px4_not_in_offboard", INHIBIT_VEHICLE_NOT_READY
        return "", ""

    def preview(
        self,
        status: Any,
        now_monotonic_s: float,
        command_monotonic_s: float | None,
        px4_main_mode: int | None,
        px4_armed: bool | None,
        px4_offboard_main_mode: int,
    ) -> OffboardSetpointPreview:
        """Build the would-be setpoint. Never transmits.

        `status` is a companion_safety.CompanionSafetyStatus. It is consumed
        through its validated output only (`output_velocity_enu_m_s`,
        `output_valid`) -- never `nominal_velocity_enu_m_s` (pre-CBF) and
        never `command.velocity_enu_m_s` (pre-supervisor).
        """
        self.preview_count += 1
        command_age_s = (
            None
            if command_monotonic_s is None
            else now_monotonic_s - command_monotonic_s
        )
        reason, category = self._inhibit(
            status, command_age_s, px4_main_mode, px4_armed, px4_offboard_main_mode
        )
        inhibited = bool(reason)
        if inhibited:
            self.inhibited_count += 1
        if category == INHIBIT_COMMAND_INTEGRITY:
            self.integrity_inhibited_count += 1
        elif category == INHIBIT_VEHICLE_NOT_READY:
            self.not_ready_inhibited_count += 1

        # Zero only when the command itself is untrustworthy. A trustworthy
        # command blocked purely by vehicle readiness is preserved so the
        # would-be setpoint stays inspectable; `valid` is False either way.
        input_velocity: Vector3 = (
            ZERO_VELOCITY
            if category == INHIBIT_COMMAND_INTEGRITY
            else tuple(float(v) for v in status.output_velocity_enu_m_s)  # type: ignore[assignment]
        )
        output_velocity = enu_to_ned_velocity(input_velocity)
        return OffboardSetpointPreview(
            drone_id=self.drone_id,
            source_authority=status.as_dict().get("authority", ""),
            input_velocity_enu_m_s=input_velocity,
            output_velocity_ned_m_s=output_velocity,
            px4_target_system=self.px4_system_id,
            px4_target_component=MAV_COMP_ID_AUTOPILOT1,
            px4_frame=MAV_FRAME_LOCAL_NED,
            px4_type_mask=OFFBOARD_VELOCITY_TYPE_MASK,
            intended_message_type=INTENDED_MESSAGE_TYPE,
            # No yaw authority is produced anywhere in the companion pipeline
            # today (CBF and the §20 ladder are velocity-only), so the
            # would-be setpoint commands zero yaw rate rather than inventing
            # one. Units are rad/s, matching the bridge's existing
            # math.radians() conversion at the transmit boundary.
            yaw_rate_rad_s=0.0,
            boot_time_ms=int(now_monotonic_s * 1000.0) & 0xFFFFFFFF,
            evaluated_monotonic_s=now_monotonic_s,
            command_age_ms=(
                None if command_age_s is None else command_age_s * 1000.0
            ),
            emergency_stage=status.emergency.stage.value,
            recommended_px4_mode=status.emergency.recommended_px4_mode,
            valid=not inhibited,
            inhibited=inhibited,
            inhibit_reason=reason,
            inhibit_category=category,
        )

    def status(self) -> dict[str, Any]:
        return {
            "drone_id": self.drone_id,
            "preview_count": self.preview_count,
            "inhibited_count": self.inhibited_count,
            "integrity_inhibited_count": self.integrity_inhibited_count,
            "not_ready_inhibited_count": self.not_ready_inhibited_count,
            "transmit_attempt_count": self.transmit_attempt_count,
            "arm_command_count": self.arm_command_count,
            "mode_change_count": self.mode_change_count,
            "transmit_allowed": False,
            "maximum_velocity_m_s": self.maximum_velocity_m_s,
            "maximum_command_age_s": self.maximum_command_age_s,
            "implementation": "shadow_only",
        }
