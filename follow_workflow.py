from __future__ import annotations

import math
import time
from dataclasses import asdict, dataclass
from enum import Enum
from typing import Any


class FollowWorkflowState(str, Enum):
    IDLE = "IDLE"
    SELECTING = "SELECTING"
    POINTING = "POINTING"
    POINTING_STABLE = "POINTING_STABLE"
    ACQUIRING_RANGE = "ACQUIRING_RANGE"
    # Backward-compatible names are aliases; serialized telemetry always uses
    # the canonical workflow requested by the Visual Follow contract.
    READY_FOR_FOLLOW = "ACQUIRING_RANGE"
    RGB_BOOTSTRAP = "RGB_BOOTSTRAP"
    TARGET_INITIALIZED = "TARGET_INITIALIZED"
    TARGET_3D_READY = "TARGET_INITIALIZED"
    PRESTREAMING_TARGET = "PRESTREAMING_TARGET"
    FOLLOW_PRESTREAM = "PRESTREAMING_TARGET"
    REQUESTING_FOLLOW_MODE = "REQUESTING_FOLLOW_MODE"
    FOLLOW_MODE_REQUESTED = "REQUESTING_FOLLOW_MODE"
    FOLLOWING = "FOLLOWING"
    DEGRADED = "DEGRADED"
    HOLD = "HOLD"


@dataclass(frozen=True)
class SelectionSnapshot:
    session_id: int
    selection_timestamp_s: float
    selection_frame_index: int
    selection_vehicle_position_ned: tuple[float, float, float] | None
    selection_camera_position_ned: tuple[float, float, float] | None
    selection_vehicle_global_position: tuple[float, float, float] | None
    selection_camera_quaternion_xyzw: tuple[float, float, float, float] | None
    selection_bearing_ned: tuple[float, float, float] | None
    selection_bbox: tuple[float, float, float, float]
    selection_bbox_scale_px: float
    selection_heading_rad: float | None
    selection_altitude_m: float | None
    selection_pose_age_ms: float | None
    selection_gimbal_age_ms: float | None
    valid: bool
    reason: str

    def status(self) -> dict[str, Any]:
        return asdict(self)


@dataclass(frozen=True)
class WorkflowTransition:
    from_state: FollowWorkflowState
    to_state: FollowWorkflowState
    timestamp_s: float
    reason: str
    timeout_s: float | None

    def status(self) -> dict[str, Any]:
        result = asdict(self)
        result["from_state"] = self.from_state.value
        result["to_state"] = self.to_state.value
        return result


_ALLOWED_TRANSITIONS: dict[FollowWorkflowState, set[FollowWorkflowState]] = {
    FollowWorkflowState.IDLE: {FollowWorkflowState.SELECTING},
    FollowWorkflowState.SELECTING: {
        FollowWorkflowState.POINTING,
        FollowWorkflowState.IDLE,
        FollowWorkflowState.HOLD,
    },
    FollowWorkflowState.POINTING: {
        FollowWorkflowState.POINTING_STABLE,
        FollowWorkflowState.DEGRADED,
        FollowWorkflowState.HOLD,
        FollowWorkflowState.SELECTING,
        FollowWorkflowState.IDLE,
    },
    FollowWorkflowState.POINTING_STABLE: {
        FollowWorkflowState.POINTING,
        FollowWorkflowState.READY_FOR_FOLLOW,
        FollowWorkflowState.DEGRADED,
        FollowWorkflowState.HOLD,
        FollowWorkflowState.SELECTING,
        FollowWorkflowState.IDLE,
    },
    FollowWorkflowState.READY_FOR_FOLLOW: {
        FollowWorkflowState.POINTING,
        FollowWorkflowState.RGB_BOOTSTRAP,
        FollowWorkflowState.TARGET_3D_READY,
        FollowWorkflowState.DEGRADED,
        FollowWorkflowState.HOLD,
        FollowWorkflowState.SELECTING,
        FollowWorkflowState.IDLE,
    },
    FollowWorkflowState.RGB_BOOTSTRAP: {
        FollowWorkflowState.TARGET_3D_READY,
        FollowWorkflowState.DEGRADED,
        FollowWorkflowState.HOLD,
        FollowWorkflowState.IDLE,
    },
    FollowWorkflowState.TARGET_3D_READY: {
        FollowWorkflowState.FOLLOW_PRESTREAM,
        FollowWorkflowState.DEGRADED,
        FollowWorkflowState.HOLD,
        FollowWorkflowState.IDLE,
    },
    FollowWorkflowState.FOLLOW_PRESTREAM: {
        FollowWorkflowState.FOLLOW_MODE_REQUESTED,
        FollowWorkflowState.DEGRADED,
        FollowWorkflowState.HOLD,
        FollowWorkflowState.IDLE,
    },
    FollowWorkflowState.FOLLOW_MODE_REQUESTED: {
        FollowWorkflowState.FOLLOWING,
        FollowWorkflowState.DEGRADED,
        FollowWorkflowState.HOLD,
        FollowWorkflowState.IDLE,
    },
    FollowWorkflowState.FOLLOWING: {
        FollowWorkflowState.DEGRADED,
        FollowWorkflowState.HOLD,
        FollowWorkflowState.IDLE,
    },
    FollowWorkflowState.DEGRADED: {
        FollowWorkflowState.POINTING,
        FollowWorkflowState.RGB_BOOTSTRAP,
        FollowWorkflowState.FOLLOWING,
        FollowWorkflowState.HOLD,
        FollowWorkflowState.IDLE,
    },
    FollowWorkflowState.HOLD: {
        FollowWorkflowState.POINTING,
        FollowWorkflowState.SELECTING,
        FollowWorkflowState.IDLE,
    },
}


class FollowWorkflowMachine:
    """Single authoritative state machine for pointing-to-PX4 handover."""

    def __init__(self, *, history_limit: int = 32) -> None:
        self.state = FollowWorkflowState.IDLE
        self.entered_timestamp_s = time.monotonic()
        self.reason = "idle"
        self.timeout_s: float | None = None
        self.session_id = 0
        self._history_limit = max(4, int(history_limit))
        self.history: list[WorkflowTransition] = []

    def transition(
        self,
        to_state: FollowWorkflowState,
        reason: str,
        *,
        timestamp_s: float | None = None,
        timeout_s: float | None = None,
        force: bool = False,
    ) -> WorkflowTransition | None:
        now = time.monotonic() if timestamp_s is None else float(timestamp_s)
        if not math.isfinite(now):
            raise ValueError("transition timestamp must be finite")
        if timeout_s is not None and (
            not math.isfinite(float(timeout_s)) or float(timeout_s) <= 0.0
        ):
            raise ValueError("transition timeout must be positive")
        reason = str(reason).strip() or "unspecified"
        if to_state == self.state:
            self.reason = reason
            self.timeout_s = timeout_s
            return None
        if not force and to_state not in _ALLOWED_TRANSITIONS[self.state]:
            raise ValueError(
                f"invalid workflow transition {self.state.value}"
                f" -> {to_state.value}"
            )
        transition = WorkflowTransition(
            from_state=self.state,
            to_state=to_state,
            timestamp_s=now,
            reason=reason,
            timeout_s=timeout_s,
        )
        self.state = to_state
        self.entered_timestamp_s = now
        self.reason = reason
        self.timeout_s = timeout_s
        self.history.append(transition)
        if len(self.history) > self._history_limit:
            del self.history[: len(self.history) - self._history_limit]
        return transition

    def begin_tracking(self, timestamp_s: float | None = None) -> int:
        self.session_id += 1
        self.transition(
            FollowWorkflowState.SELECTING,
            "tracking_started",
            timestamp_s=timestamp_s,
            force=True,
        )
        return self.session_id

    def begin_target_session(self) -> int:
        """Invalidate commands and frames associated with the previous bbox."""

        self.session_id += 1
        return self.session_id

    def select_target(
        self,
        *,
        snapshot_valid: bool,
        reason: str,
        timestamp_s: float | None = None,
    ) -> None:
        self.transition(
            FollowWorkflowState.POINTING,
            reason if reason else (
                "bbox_selected"
                if snapshot_valid
                else "selection_pose_stale"
            ),
            timestamp_s=timestamp_s,
            force=self.state not in {
                FollowWorkflowState.SELECTING,
                FollowWorkflowState.POINTING,
                FollowWorkflowState.POINTING_STABLE,
                FollowWorkflowState.READY_FOR_FOLLOW,
                FollowWorkflowState.HOLD,
            },
        )

    def status(self, now_s: float | None = None) -> dict[str, Any]:
        now = time.monotonic() if now_s is None else float(now_s)
        elapsed_s = max(0.0, now - self.entered_timestamp_s)
        timeout_remaining_s = (
            None
            if self.timeout_s is None
            else max(0.0, self.timeout_s - elapsed_s)
        )
        return {
            "state": self.state.value,
            "reason": self.reason,
            "session_id": self.session_id,
            "entered_timestamp_s": self.entered_timestamp_s,
            "state_elapsed_s": elapsed_s,
            "timeout_s": self.timeout_s,
            "timeout_remaining_s": timeout_remaining_s,
            "timed_out": bool(
                self.timeout_s is not None and elapsed_s >= self.timeout_s
            ),
            "last_transition": (
                self.history[-1].status() if self.history else None
            ),
            "transition_history": [
                transition.status() for transition in self.history
            ],
        }
