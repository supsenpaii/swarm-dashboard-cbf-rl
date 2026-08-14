from __future__ import annotations

import math
import os
from dataclasses import dataclass


def _env_float(name: str, default: float, minimum: float, maximum: float) -> float:
    try:
        value = float(os.environ.get(name, str(default)))
    except ValueError:
        value = default
    return max(minimum, min(maximum, value))


def _env_bool(name: str, default: bool = False) -> bool:
    fallback = "true" if default else "false"
    return os.environ.get(name, fallback).strip().lower() in {
        "1",
        "true",
        "yes",
        "on",
    }


@dataclass(frozen=True)
class BodyYawRecenterConfig:
    enter_deg: float = 3.75
    exit_deg: float = 1.25
    enter_hold_s: float = 0.08
    filter_time_constant_s: float = 0.08
    proportional_gain: float = 1.6
    bbox_feedforward_gain: float = 0.20
    maximum_rate_deg_s: float = 20.0
    slew_rate_deg_s2: float = 120.0
    invert: bool = False

    @classmethod
    def from_environment(cls) -> "BodyYawRecenterConfig":
        exit_deg = _env_float(
            "SWARM_BODY_RECENTER_EXIT_DEG", 1.25, 0.2, 10.0
        )
        enter_deg = _env_float(
            "SWARM_BODY_RECENTER_ENTER_DEG", 3.75, exit_deg + 0.2, 30.0
        )
        return cls(
            enter_deg=enter_deg,
            exit_deg=exit_deg,
            enter_hold_s=_env_float(
                "SWARM_BODY_RECENTER_ENTER_HOLD_S", 0.08, 0.0, 2.0
            ),
            filter_time_constant_s=_env_float(
                "SWARM_BODY_RECENTER_FILTER_TAU_S", 0.08, 0.01, 3.0
            ),
            proportional_gain=_env_float(
                "SWARM_BODY_RECENTER_YAW_KP", 1.6, 0.0, 8.0
            ),
            bbox_feedforward_gain=_env_float(
                "SWARM_BODY_RECENTER_BBOX_FEEDFORWARD_GAIN", 0.20, 0.0, 1.0
            ),
            maximum_rate_deg_s=_env_float(
                "SWARM_BODY_RECENTER_MAX_YAW_RATE_DEG_S", 20.0, 0.5, 45.0
            ),
            slew_rate_deg_s2=_env_float(
                "SWARM_BODY_RECENTER_RATE_SLEW_DEG_S2", 120.0, 1.0, 360.0
            ),
            invert=_env_bool("SWARM_BODY_RECENTER_YAW_INVERT", False),
        )


@dataclass(frozen=True)
class BodyYawRecenterOutput:
    active: bool
    state: str
    filtered_gimbal_yaw_deg: float
    combined_error_deg: float
    requested_rate_deg_s: float
    limited_rate_deg_s: float
    enter_elapsed_s: float
    reason: str


class BodyYawRecenterController:
    """Slow outer loop that unloads yaw from the tracking gimbal.

    The gimbal remains the fast bbox controller. Body yaw only starts after a
    filtered gimbal deflection persists beyond the enter threshold.
    """

    def __init__(self, config: BodyYawRecenterConfig | None = None) -> None:
        self.config = config or BodyYawRecenterConfig.from_environment()
        self.reset()

    def reset(self) -> None:
        self.filtered_gimbal_yaw_deg: float | None = None
        self.active = False
        self.enter_elapsed_s = 0.0
        self.rate_deg_s = 0.0

    def update(
        self,
        *,
        gimbal_yaw_deg: float,
        bbox_horizontal_error_deg: float,
        tracking_valid: bool,
        dt_s: float,
        gimbal_fresh: bool = True,
    ) -> BodyYawRecenterOutput:
        cfg = self.config
        dt = max(0.001, min(0.2, float(dt_s)))
        values = (gimbal_yaw_deg, bbox_horizontal_error_deg)
        finite = all(math.isfinite(float(value)) for value in values)
        if not tracking_valid or not gimbal_fresh or not finite:
            self.active = False
            self.enter_elapsed_s = 0.0
            self.rate_deg_s = self._slew(self.rate_deg_s, 0.0, dt)
            return self._output(
                requested=0.0,
                combined=0.0,
                state="blocked",
                reason=(
                    "gimbal_stale"
                    if not gimbal_fresh
                    else "tracking_invalid"
                    if tracking_valid
                    else "tracking_blocked"
                ),
            )

        measured = float(gimbal_yaw_deg)
        if self.filtered_gimbal_yaw_deg is None:
            self.filtered_gimbal_yaw_deg = measured
        else:
            alpha = 1.0 - math.exp(-dt / max(0.01, cfg.filter_time_constant_s))
            self.filtered_gimbal_yaw_deg += alpha * (
                measured - self.filtered_gimbal_yaw_deg
            )

        filtered_abs = abs(self.filtered_gimbal_yaw_deg)
        if self.active:
            if filtered_abs <= cfg.exit_deg:
                self.active = False
                self.enter_elapsed_s = 0.0
        elif filtered_abs >= cfg.enter_deg:
            self.enter_elapsed_s += dt
            if self.enter_elapsed_s >= cfg.enter_hold_s:
                self.active = True
                self.enter_elapsed_s = 0.0
        else:
            self.enter_elapsed_s = 0.0

        combined = (
            self.filtered_gimbal_yaw_deg
            + cfg.bbox_feedforward_gain * float(bbox_horizontal_error_deg)
        )
        if cfg.invert:
            combined *= -1.0
        if self.active:
            effective_error = math.copysign(
                max(0.0, abs(combined) - cfg.exit_deg),
                combined,
            )
            requested = max(
                -cfg.maximum_rate_deg_s,
                min(cfg.maximum_rate_deg_s, cfg.proportional_gain * effective_error),
            )
        else:
            requested = 0.0

        # Never cross directly from positive to negative yaw in one update.
        # First brake to zero, then accelerate in the new direction.
        slew_target = requested
        if self.rate_deg_s * requested < 0.0:
            slew_target = 0.0
        self.rate_deg_s = self._slew(self.rate_deg_s, slew_target, dt)
        state = (
            "recentering"
            if self.active
            else "arming"
            if self.enter_elapsed_s > 0.0
            else "centered"
        )
        reason = (
            ""
            if self.active
            else "enter_hold"
            if self.enter_elapsed_s > 0.0
            else "gimbal_within_hysteresis"
        )
        return self._output(
            requested=requested,
            combined=combined,
            state=state,
            reason=reason,
        )

    def _slew(self, previous: float, target: float, dt_s: float) -> float:
        maximum_delta = self.config.slew_rate_deg_s2 * dt_s
        return previous + max(
            -maximum_delta,
            min(maximum_delta, target - previous),
        )

    def _output(
        self,
        *,
        requested: float,
        combined: float,
        state: str,
        reason: str,
    ) -> BodyYawRecenterOutput:
        return BodyYawRecenterOutput(
            active=self.active,
            state=state,
            filtered_gimbal_yaw_deg=float(self.filtered_gimbal_yaw_deg or 0.0),
            combined_error_deg=float(combined),
            requested_rate_deg_s=float(requested),
            limited_rate_deg_s=float(self.rate_deg_s),
            enter_elapsed_s=float(self.enter_elapsed_s),
            reason=reason,
        )
