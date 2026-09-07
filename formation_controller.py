"""Deterministic ENU formation nominal controller.

This module produces only a bounded nominal velocity.  It has no PX4, MQTT,
or Offboard dependency: CBF/command-gate integration remains a separate step.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Any


Vector3 = tuple[float, float, float]


@dataclass(frozen=True)
class FormationSlot:
    drone_id: str
    offset_enu_m: Vector3

    def __post_init__(self) -> None:
        if not self.drone_id.strip() or not _finite_vector(self.offset_enu_m):
            raise ValueError("formation slot is invalid")


@dataclass(frozen=True)
class FormationConfig:
    position_gain_s_inv: float = 0.6
    maximum_velocity_m_s: float = 2.0
    arrival_radius_m: float = 0.25

    def __post_init__(self) -> None:
        values = (
            self.position_gain_s_inv,
            self.maximum_velocity_m_s,
            self.arrival_radius_m,
        )
        if not all(math.isfinite(value) and value > 0.0 for value in values):
            raise ValueError("formation controller configuration is invalid")


@dataclass(frozen=True)
class FormationCommand:
    drone_id: str
    target_enu_m: Vector3 | None
    velocity_enu_m_s: Vector3
    active: bool
    reason: str
    position_error_m: float | None

    def as_dict(self) -> dict[str, Any]:
        return {
            "drone_id": self.drone_id,
            "frame": "ENU",
            "target_enu_m": list(self.target_enu_m) if self.target_enu_m else None,
            "velocity_enu_m_s": list(self.velocity_enu_m_s),
            "active": self.active,
            "reason": self.reason,
            "position_error_m": round(self.position_error_m, 3)
            if self.position_error_m is not None
            else None,
            "authority": "shadow_nominal_only",
        }


class DeterministicFormationController:
    """P-controller for slots expressed in a common ENU frame."""

    def __init__(self, leader_id: str, slots: tuple[FormationSlot, ...], config: FormationConfig | None = None) -> None:
        if not leader_id.strip():
            raise ValueError("leader_id is required")
        slot_ids = [slot.drone_id for slot in slots]
        if len(slot_ids) != len(set(slot_ids)) or leader_id in slot_ids:
            raise ValueError("formation slots must be unique followers")
        self.leader_id = leader_id
        self.slots = {slot.drone_id: slot for slot in slots}
        self.config = config or FormationConfig()

    def command(self, drone_id: str, swarm_state: dict[str, Any]) -> FormationCommand:
        slot = self.slots.get(drone_id)
        if slot is None:
            return self._hold(drone_id, "formation_slot_unassigned")
        follower = swarm_state.get(drone_id)
        leader = swarm_state.get(self.leader_id)
        follower_position = _usable_vector(follower, "position_enu_m")
        leader_position = _usable_vector(leader, "position_enu_m")
        leader_velocity = _usable_vector(leader, "velocity_enu_m_s")
        if follower_position is None:
            return self._hold(drone_id, "follower_state_invalid")
        if leader_position is None or leader_velocity is None:
            return self._hold(drone_id, "leader_state_invalid")
        target = tuple(leader_position[index] + slot.offset_enu_m[index] for index in range(3))
        error = tuple(target[index] - follower_position[index] for index in range(3))
        error_norm = _norm(error)
        if error_norm <= self.config.arrival_radius_m:
            return FormationCommand(drone_id, target, (0.0, 0.0, 0.0), True, "slot_reached", error_norm)
        requested = tuple(
            leader_velocity[index] + self.config.position_gain_s_inv * error[index]
            for index in range(3)
        )
        return FormationCommand(
            drone_id,
            target,
            _limit_norm(requested, self.config.maximum_velocity_m_s),
            True,
            "tracking_slot",
            error_norm,
        )

    @staticmethod
    def _hold(drone_id: str, reason: str) -> FormationCommand:
        return _hold(drone_id, reason)


class TargetRelativeFormationController:
    """Formation slots anchored to an external target's ENU position.

    Sibling to `DeterministicFormationController`, which anchors slots to a
    fellow drone (the leader). A target is not a drone: it must never be
    looked up from `swarm_state` by drone id or passed to CBF as a peer,
    which would apply collision-avoidance separation against whatever is
    being followed rather than a squadmate. The target is instead passed in
    explicitly, in the same shape as any other swarm-state entry
    (`valid`/`position_enu_m`/`velocity_enu_m_s`).
    """

    def __init__(self, slots: tuple[FormationSlot, ...], config: FormationConfig | None = None) -> None:
        slot_ids = [slot.drone_id for slot in slots]
        if len(slot_ids) != len(set(slot_ids)):
            raise ValueError("target-relative formation slots must be unique")
        self.slots = {slot.drone_id: slot for slot in slots}
        self.config = config or FormationConfig()

    def command(self, drone_id: str, follower_state: Any, target_state: Any) -> FormationCommand:
        slot = self.slots.get(drone_id)
        if slot is None:
            return _hold(drone_id, "formation_slot_unassigned")
        follower_position = _usable_vector(follower_state, "position_enu_m")
        target_position = _usable_vector(target_state, "position_enu_m")
        if follower_position is None:
            return _hold(drone_id, "follower_state_invalid")
        if target_position is None:
            return _hold(drone_id, "target_state_invalid")
        # Target velocity degrades to zero feed-forward rather than
        # invalidating the slot when untrustworthy: this project's own
        # TargetEstimate already separates "position known" from
        # "velocity_valid" for exactly this reason (see tracking_web.py's
        # visual-follow projection, which does the same substitution).
        target_velocity = _usable_vector(target_state, "velocity_enu_m_s")
        if target_velocity is None:
            target_velocity = (0.0, 0.0, 0.0)
        target = tuple(target_position[index] + slot.offset_enu_m[index] for index in range(3))
        error = tuple(target[index] - follower_position[index] for index in range(3))
        error_norm = _norm(error)
        if error_norm <= self.config.arrival_radius_m:
            return FormationCommand(drone_id, target, (0.0, 0.0, 0.0), True, "target_slot_reached", error_norm)
        requested = tuple(
            target_velocity[index] + self.config.position_gain_s_inv * error[index]
            for index in range(3)
        )
        return FormationCommand(
            drone_id,
            target,
            _limit_norm(requested, self.config.maximum_velocity_m_s),
            True,
            "tracking_target_slot",
            error_norm,
        )


@dataclass(frozen=True)
class AltitudeHoldConfig:
    """Vertical station-keeping limits.

    No new tuning constants: the gain and arrival radius are the formation
    controller's own, and the velocity ceiling is FlightEnvelope's vertical
    limit (half the CBF horizontal limit), so a correction can never be
    faster than the flight envelope already permits.
    """

    gain_s_inv: float = 0.6
    maximum_velocity_m_s: float = 1.0
    arrival_radius_m: float = 0.25

    def __post_init__(self) -> None:
        values = (self.gain_s_inv, self.maximum_velocity_m_s, self.arrival_radius_m)
        if not all(math.isfinite(value) and value > 0.0 for value in values):
            raise ValueError("altitude hold configuration is invalid")


class AltitudeHoldController:
    """Holds the altitude a drone had when station-keeping began.

    WHY THIS EXISTS
    ---------------
    The first armed OFFBOARD flight held horizontal station to 0.028 m/s but
    drifted UP 1.36 m in 15 s on a commanded zero vertical velocity. That is
    what velocity-mode OFFBOARD does: it holds velocity, not position, and
    the vertical axis is the one with a standing disturbance (gravity, via
    PX4's hover-thrust estimate). Horizontally there is no equivalent bias,
    which is why only this axis needs closing.

    WHY THE REFERENCE IS LATCHED ON A TRANSITION
    --------------------------------------------
    Capturing it whenever own state is valid would latch the altitude the
    drone had while sitting on the ground. PX4's AUTO takeoff then lifts it
    to ~9 m, and the instant OFFBOARD engaged the companion would command a
    dive back to the ground reference. The reference is therefore captured
    only on the transition into station-keeping -- the moment the companion
    actually becomes the controlling authority -- and cleared when it stops
    being. The caller supplies that flag; this module stays free of any PX4
    concept.
    """

    def __init__(self, config: AltitudeHoldConfig | None = None) -> None:
        self.config = config or AltitudeHoldConfig()
        self.reference_altitude_m: float | None = None
        self.active = False

    def reset(self) -> None:
        self.reference_altitude_m = None
        self.active = False

    def command(
        self, drone_id: str, own_state: Any, station_keeping: bool
    ) -> FormationCommand:
        if not station_keeping:
            self.reset()
            return _hold(drone_id, "station_keeping_inactive")

        position = _usable_vector(own_state, "position_enu_m")
        if position is None:
            # An unusable state cannot be held against. The reference is
            # dropped rather than kept, so control resumes from wherever the
            # drone actually is once state returns -- holding against a
            # stale reference is how a small gap becomes a large correction.
            self.reset()
            return _hold(drone_id, "own_state_invalid")

        altitude = position[2]
        if self.reference_altitude_m is None:
            self.reference_altitude_m = altitude
            self.active = True
            return FormationCommand(
                drone_id,
                (position[0], position[1], altitude),
                (0.0, 0.0, 0.0),
                True,
                "altitude_reference_captured",
                0.0,
            )

        error = self.reference_altitude_m - altitude
        target = (position[0], position[1], self.reference_altitude_m)
        if abs(error) <= self.config.arrival_radius_m:
            return FormationCommand(
                drone_id, target, (0.0, 0.0, 0.0), True, "altitude_held", abs(error)
            )
        vertical = self.config.gain_s_inv * error
        vertical = max(
            -self.config.maximum_velocity_m_s,
            min(self.config.maximum_velocity_m_s, vertical),
        )
        return FormationCommand(
            drone_id, target, (0.0, 0.0, vertical), True, "altitude_hold", abs(error)
        )


def _hold(drone_id: str, reason: str) -> FormationCommand:
    return FormationCommand(drone_id, None, (0.0, 0.0, 0.0), False, reason, None)


def _usable_vector(state: Any, field: str) -> Vector3 | None:
    if not isinstance(state, dict) or not state.get("valid", False):
        return None
    value = state.get(field)
    if not isinstance(value, (list, tuple)) or len(value) != 3:
        return None
    try:
        vector = tuple(float(item) for item in value)
    except (TypeError, ValueError):
        return None
    return vector if _finite_vector(vector) else None


def _finite_vector(vector: Vector3) -> bool:
    return all(math.isfinite(item) for item in vector)


def _norm(vector: Vector3) -> float:
    return math.sqrt(sum(item * item for item in vector))


def _limit_norm(vector: Vector3, maximum: float) -> Vector3:
    magnitude = _norm(vector)
    if magnitude <= maximum:
        return vector
    scale = maximum / magnitude
    return tuple(item * scale for item in vector)  # type: ignore[return-value]
