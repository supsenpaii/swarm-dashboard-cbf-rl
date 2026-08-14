"""Companion-local emergency fallback ladder, per architecture doc §20.

§20 ("Khi QP không có nghiệm") gives an ordered list of six fallback actions
to take when the CBF/QP layer cannot produce a safe command, triggered by
§19's control loop: `if result.feasible: u_safe = result.command else:
u_safe = emergency_controller(own_state, neighbors)`. It does not specify
transition timings, hysteresis, or a recovery rule -- see the module-level
ambiguity notes below, each tied to the specific line of code that encodes
the resulting engineering default.

Position in the pipeline (per §24's minimal companion architecture):

    nominal command -> local CBF -> EmergencySupervisor -> command validator -> shadow output

This module never touches PX4. Every output is a shadow recommendation only.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
import math
from typing import Any

from cbf_command_gate import CbfCommand


Vector3 = tuple[float, float, float]

ZERO_VELOCITY: Vector3 = (0.0, 0.0, 0.0)


class EmergencyStage(str, Enum):
    NORMAL = "normal"
    STOP_HORIZONTAL = "stop_horizontal"
    REDUCE_VELOCITY = "reduce_velocity"
    ALTITUDE_SEPARATION = "altitude_separation"
    HOLD = "hold"
    RECOMMEND_POSITION_HOLD = "recommend_position_hold"
    RECOMMEND_RTL_OR_LAND = "recommend_rtl_or_land"


# Order is the one piece of §20 stated as an explicit contract ("Fallback
# theo thu tu"): a fault may only advance one rung at a time, never skip
# ahead, and recovery is all-or-nothing back to NORMAL (see EmergencySupervisor
# docstring) rather than a partial step down -- neither skip-ahead escalation
# nor partial recovery is described anywhere in §20.
STAGE_ORDER: tuple[EmergencyStage, ...] = (
    EmergencyStage.NORMAL,
    EmergencyStage.STOP_HORIZONTAL,
    EmergencyStage.REDUCE_VELOCITY,
    EmergencyStage.ALTITUDE_SEPARATION,
    EmergencyStage.HOLD,
    EmergencyStage.RECOMMEND_POSITION_HOLD,
    EmergencyStage.RECOMMEND_RTL_OR_LAND,
)


@dataclass(frozen=True)
class EmergencyConfig:
    """Timing and geometry the ladder needs but §20 does not specify.

    §20 states only an ORDER, not pacing or a recovery rule. Every field here
    is therefore an engineering default, not a value extracted from the
    architecture document -- each is documented at its point of use in
    EmergencySupervisor.evaluate().
    """

    # How long a fault must persist in the current stage before advancing to
    # the next rung. Absent from §20; chosen to match the time-based
    # convention already used throughout this codebase (PEER_STATE_MAX_AGE_S,
    # CbfConfig.command_latency_s) rather than a frame-count, since companion
    # loop rate is not fixed by contract.
    stage_dwell_s: float = 2.0

    # How long CBF must be continuously feasible before the ladder resets to
    # NORMAL. Absent from §20. Without this, a single feasible frame amid
    # otherwise-infeasible ones would reset the ladder every time, which
    # would both violate "no oscillation/chatter" and let a borderline
    # scenario bounce in and out of an emergency stage indefinitely.
    recovery_confirmation_s: float = 0.5

    # Stage 3 (ALTITUDE_SEPARATION) target altitude, per drone-ID index:
    # target = altitude_base_m + index * altitude_step_m. §20's own example
    # (12/15/18 m for drones 1/2/3) is for an unstated baseline that does not
    # match this project's ~10 m SITL hover altitude, so the *pattern*
    # (deterministic per-ID offset) is kept and the literal numbers are not.
    altitude_base_m: float = 10.0
    altitude_step_m: float = 3.0


@dataclass(frozen=True)
class EmergencyDecision:
    stage: EmergencyStage
    velocity_enu_m_s: Vector3
    active: bool
    reason: str
    recommended_px4_mode: str | None
    time_in_stage_s: float

    def as_dict(self) -> dict[str, Any]:
        return {
            "stage": self.stage.value,
            "velocity_enu_m_s": list(self.velocity_enu_m_s),
            "active": self.active,
            "reason": self.reason,
            "recommended_px4_mode": self.recommended_px4_mode,
            "time_in_stage_s": round(self.time_in_stage_s, 3),
            "authority": "shadow_emergency_supervisor_only",
        }


def _clip(value: float, maximum: float) -> float:
    return max(-maximum, min(maximum, value))


class EmergencySupervisor:
    """Deterministic §20 fallback ladder, evaluated once per companion frame.

    Reason-agnostic by design: §19's control loop branches only on
    `result.feasible`, never on *why* it is infeasible. Every CBF Hold reason
    (stale own state, stale peer, geofence, overlap, solver infeasibility,
    or -- via CbfCommandGate's own `_vector()` check -- a malformed nominal
    command) already collapses to `CbfCommand.active is False` upstream, so
    this supervisor does not need to special-case any of them; it faithfully
    mirrors the single feasible/else branch in §19's pseudocode.
    """

    def __init__(
        self,
        drone_id: str,
        peer_ids: tuple[str, ...],
        geofence_min_enu_m: Vector3,
        geofence_max_enu_m: Vector3,
        maximum_velocity_m_s: float,
        altitude_gain_s_inv: float,
        altitude_arrival_radius_m: float,
        config: EmergencyConfig | None = None,
    ) -> None:
        if not drone_id.strip():
            raise ValueError("drone_id is required")
        if not all(math.isfinite(value) and value > 0.0 for value in (maximum_velocity_m_s, altitude_gain_s_inv, altitude_arrival_radius_m)):
            raise ValueError("emergency supervisor configuration is invalid")
        self.drone_id = drone_id
        self.geofence_min_enu_m = geofence_min_enu_m
        self.geofence_max_enu_m = geofence_max_enu_m
        self.maximum_velocity_m_s = float(maximum_velocity_m_s)
        self.altitude_gain_s_inv = float(altitude_gain_s_inv)
        self.altitude_arrival_radius_m = float(altitude_arrival_radius_m)
        self.config = config or EmergencyConfig()

        # Deterministic altitude assignment "by drone ID" (§20.3's own
        # phrasing), from static configuration only -- it must not require a
        # live peer negotiation, since the whole point of separating by ID
        # rather than by coordination is that it still works when peer
        # comms are exactly what's down.
        fleet = tuple(sorted({drone_id, *peer_ids}))
        self._altitude_index = fleet.index(drone_id)

        self._stage: EmergencyStage = EmergencyStage.NORMAL
        self._stage_entered_monotonic_s: float | None = None
        self._feasible_streak_started_monotonic_s: float | None = None

    def _target_altitude_m(self) -> float:
        return self.config.altitude_base_m + self._altitude_index * self.config.altitude_step_m

    def _altitude_separation_precondition_met(self) -> bool:
        """Geofence check only. See module docstring: "airspace above" has no
        sensor anywhere in this codebase and cannot be verified here -- this
        is a known, reported limitation, not a completed §20 requirement."""
        target = self._target_altitude_m()
        return self.geofence_min_enu_m[2] <= target <= self.geofence_max_enu_m[2]

    def _next_stage(self, own_state_valid: bool) -> EmergencyStage | None:
        index = STAGE_ORDER.index(self._stage)
        if index + 1 >= len(STAGE_ORDER):
            return None
        candidate = STAGE_ORDER[index + 1]
        if candidate is EmergencyStage.ALTITUDE_SEPARATION and not (
            own_state_valid and self._altitude_separation_precondition_met()
        ):
            # Precondition unmet: skip the rung rather than block escalation,
            # since a stage this system cannot safely execute must not stall
            # the ladder before HOLD.
            if index + 2 >= len(STAGE_ORDER):
                return None
            return STAGE_ORDER[index + 2]
        return candidate

    def _enter(self, stage: EmergencyStage, now_monotonic_s: float) -> None:
        self._stage = stage
        self._stage_entered_monotonic_s = now_monotonic_s

    def evaluate(
        self,
        cbf_command: CbfCommand,
        now_monotonic_s: float,
        own_state_valid: bool,
        own_altitude_m: float | None,
    ) -> EmergencyDecision:
        if not math.isfinite(now_monotonic_s):
            raise ValueError("now_monotonic_s must be finite")

        if cbf_command.active:
            self._feasible_streak_started_monotonic_s = (
                now_monotonic_s
                if self._feasible_streak_started_monotonic_s is None
                else self._feasible_streak_started_monotonic_s
            )
            feasible_duration = now_monotonic_s - self._feasible_streak_started_monotonic_s
            if (
                self._stage is not EmergencyStage.NORMAL
                and feasible_duration >= self.config.recovery_confirmation_s
            ):
                self._enter(EmergencyStage.NORMAL, now_monotonic_s)
            # A feasible frame never escalates and never partially recovers:
            # while still inside the confirmation window the ladder holds its
            # current rung untouched, which is what makes a brief feasible
            # blip unable to cause chatter in either direction.
        else:
            self._feasible_streak_started_monotonic_s = None
            if self._stage is EmergencyStage.NORMAL:
                self._enter(STAGE_ORDER[1], now_monotonic_s)
            else:
                assert self._stage_entered_monotonic_s is not None
                dwell = now_monotonic_s - self._stage_entered_monotonic_s
                if dwell >= self.config.stage_dwell_s:
                    next_stage = self._next_stage(own_state_valid)
                    if next_stage is not None:
                        self._enter(next_stage, now_monotonic_s)
                    # else: already at the terminal stage; hold there.

        return self._decision(cbf_command, now_monotonic_s, own_altitude_m)

    def _decision(
        self,
        cbf_command: CbfCommand,
        now_monotonic_s: float,
        own_altitude_m: float | None,
    ) -> EmergencyDecision:
        time_in_stage = (
            0.0
            if self._stage_entered_monotonic_s is None
            else now_monotonic_s - self._stage_entered_monotonic_s
        )
        if self._stage is EmergencyStage.NORMAL:
            return EmergencyDecision(
                stage=self._stage,
                velocity_enu_m_s=cbf_command.velocity_enu_m_s,
                active=False,
                reason="cbf_feasible",
                recommended_px4_mode=None,
                time_in_stage_s=time_in_stage,
            )
        if self._stage in (EmergencyStage.STOP_HORIZONTAL, EmergencyStage.REDUCE_VELOCITY):
            # Both rungs command zero: this system has no continuous
            # velocity-ramp state to further "reduce" once horizontal is
            # already zeroed (each frame is an independent CBF solve), and
            # carrying forward any prior velocity would conflict with
            # "khong duoc dung lai lenh RL cu". Kept as distinct FSM states
            # for ladder fidelity and audit trail; see module docstring.
            return EmergencyDecision(
                stage=self._stage,
                velocity_enu_m_s=ZERO_VELOCITY,
                active=True,
                reason=f"emergency_{self._stage.value}",
                recommended_px4_mode=None,
                time_in_stage_s=time_in_stage,
            )
        if self._stage is EmergencyStage.ALTITUDE_SEPARATION:
            assert own_altitude_m is not None  # guaranteed by _next_stage's precondition gate
            error = self._target_altitude_m() - own_altitude_m
            if abs(error) <= self.altitude_arrival_radius_m:
                vertical = 0.0
            else:
                vertical = _clip(error * self.altitude_gain_s_inv, self.maximum_velocity_m_s)
            return EmergencyDecision(
                stage=self._stage,
                velocity_enu_m_s=(0.0, 0.0, vertical),
                active=True,
                reason="emergency_altitude_separation",
                recommended_px4_mode=None,
                time_in_stage_s=time_in_stage,
            )
        if self._stage is EmergencyStage.HOLD:
            return EmergencyDecision(
                stage=self._stage,
                velocity_enu_m_s=ZERO_VELOCITY,
                active=True,
                reason="emergency_hold",
                recommended_px4_mode=None,
                time_in_stage_s=time_in_stage,
            )
        if self._stage is EmergencyStage.RECOMMEND_POSITION_HOLD:
            return EmergencyDecision(
                stage=self._stage,
                velocity_enu_m_s=ZERO_VELOCITY,
                active=True,
                reason="emergency_recommend_position_hold",
                recommended_px4_mode="POSITION_HOLD",
                time_in_stage_s=time_in_stage,
            )
        return EmergencyDecision(
            stage=self._stage,
            velocity_enu_m_s=ZERO_VELOCITY,
            active=True,
            reason="emergency_recommend_rtl_or_land",
            recommended_px4_mode="RTL_OR_LAND",
            time_in_stage_s=time_in_stage,
        )
