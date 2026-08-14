from __future__ import annotations

import math
import os
from dataclasses import dataclass
from typing import Any


def _env_float(name: str, default: float, minimum: float, maximum: float) -> float:
    try:
        value = float(os.environ.get(name, str(default)))
    except ValueError:
        value = default
    return max(minimum, min(maximum, value))


@dataclass(frozen=True)
class BodyAttitudeRecenterConfig:
    enabled: bool
    home_roll_deg: float
    home_pitch_deg: float
    home_yaw_deg: float
    deadband_deg: float
    enter_deadband_deg: float
    filter_time_constant_s: float
    enter_hold_s: float
    bbox_deadband_deg: float
    bbox_roll_gain: float
    bbox_pitch_gain: float
    bbox_yaw_gain: float
    yaw_fast_threshold_deg: float
    yaw_fast_min_rate_deg_s: float
    yaw_fast_kp: float
    roll_kp: float
    pitch_kp: float
    yaw_kp: float
    yaw_kp_far: float
    yaw_adaptive_error_deg: float
    level_kp: float
    max_tilt_deg: float
    max_roll_rate_deg_s: float
    max_pitch_rate_deg_s: float
    max_yaw_rate_deg_s: float
    distance_pitch_rate_max_deg_s: float
    velocity_per_rate_m_s: float
    max_forward_velocity_m_s: float
    max_right_velocity_m_s: float
    max_horizontal_accel_m_s2: float
    horizontal_accel_jerk_m_s3: float
    velocity_damping: float
    follow_accel_max_m_s2: float
    altitude_pause_error_m: float
    altitude_exit_error_m: float
    rate_slew_deg_s2: float
    hover_thrust: float
    altitude_kp: float
    vertical_velocity_kd: float
    thrust_min: float
    thrust_max: float

    @classmethod
    def from_environment(cls) -> "BodyAttitudeRecenterConfig":
        enabled = os.environ.get(
            "SWARM_TRACKING_BODY_ATTITUDE_ENABLED",
            "true",
        ).strip().lower() in {"1", "true", "yes", "on"}
        return cls(
            enabled=enabled,
            home_roll_deg=_env_float(
                "SWARM_GIMBAL_HOME_ROLL_DEG", 0.0, -20.0, 20.0
            ),
            home_pitch_deg=_env_float(
                "SWARM_GIMBAL_HOME_PITCH_DEG", 0.0, -45.0, 20.0
            ),
            home_yaw_deg=_env_float(
                "SWARM_GIMBAL_HOME_YAW_DEG", 0.0, -45.0, 45.0
            ),
            deadband_deg=_env_float(
                "SWARM_BODY_RECENTER_EXIT_DEG",
                _env_float(
                    "SWARM_BODY_RECENTER_DEADBAND_DEG", 2.5, 0.2, 10.0
                ),
                0.2,
                10.0,
            ),
            enter_deadband_deg=_env_float(
                "SWARM_BODY_RECENTER_ENTER_DEG", 6.0, 0.4, 15.0
            ),
            filter_time_constant_s=_env_float(
                "SWARM_BODY_RECENTER_FILTER_TAU_S", 0.30, 0.02, 2.0
            ),
            enter_hold_s=_env_float(
                "SWARM_BODY_RECENTER_ENTER_HOLD_S", 0.25, 0.0, 0.5
            ),
            bbox_deadband_deg=_env_float(
                "SWARM_BODY_BBOX_DEADBAND_DEG", 0.5, 0.1, 5.0
            ),
            bbox_roll_gain=_env_float(
                "SWARM_BODY_BBOX_ROLL_GAIN", 0.35, 0.0, 2.0
            ),
            bbox_pitch_gain=_env_float(
                "SWARM_BODY_BBOX_PITCH_GAIN", 1.00, 0.0, 2.0
            ),
            bbox_yaw_gain=_env_float(
                "SWARM_BODY_BBOX_YAW_GAIN", 0.0, 0.0, 3.0
            ),
            yaw_fast_threshold_deg=_env_float(
                "SWARM_BODY_YAW_FAST_THRESHOLD_DEG", 20.0, 1.0, 30.0
            ),
            yaw_fast_min_rate_deg_s=_env_float(
                "SWARM_BODY_YAW_FAST_MIN_RATE_DEG_S", 0.0, 0.0, 45.0
            ),
            yaw_fast_kp=_env_float(
                "SWARM_BODY_YAW_FAST_KP", 0.0, 0.0, 8.0
            ),
            roll_kp=_env_float(
                "SWARM_BODY_RECENTER_ROLL_KP", 0.40, 0.0, 3.0
            ),
            pitch_kp=_env_float(
                "SWARM_BODY_RECENTER_PITCH_KP", 0.40, 0.0, 3.0
            ),
            yaw_kp=_env_float(
                "SWARM_BODY_RECENTER_YAW_KP", 1.0, 0.0, 3.0
            ),
            yaw_kp_far=_env_float(
                "SWARM_BODY_RECENTER_YAW_KP_FAR", 1.5, 0.0, 3.0
            ),
            yaw_adaptive_error_deg=_env_float(
                "SWARM_BODY_RECENTER_YAW_ADAPTIVE_ERROR_DEG",
                10.0,
                2.0,
                30.0,
            ),
            level_kp=_env_float(
                "SWARM_BODY_RECENTER_LEVEL_KP", 0.45, 0.0, 3.0
            ),
            max_tilt_deg=_env_float(
                "SWARM_BODY_RECENTER_MAX_TILT_DEG", 6.0, 1.0, 15.0
            ),
            max_roll_rate_deg_s=_env_float(
                "SWARM_BODY_RECENTER_MAX_ROLL_RATE_DEG_S", 10.0, 0.5, 30.0
            ),
            max_pitch_rate_deg_s=_env_float(
                "SWARM_BODY_RECENTER_MAX_PITCH_RATE_DEG_S", 10.0, 0.5, 30.0
            ),
            max_yaw_rate_deg_s=_env_float(
                "SWARM_BODY_RECENTER_MAX_YAW_RATE_DEG_S", 10.0, 1.0, 45.0
            ),
            distance_pitch_rate_max_deg_s=_env_float(
                "SWARM_BODY_RECENTER_DISTANCE_PITCH_RATE_MAX_DEG_S",
                2.0,
                0.0,
                8.0,
            ),
            velocity_per_rate_m_s=_env_float(
                "SWARM_BODY_RECENTER_VELOCITY_PER_RATE_M_S", 0.05, 0.0, 0.20
            ),
            max_forward_velocity_m_s=_env_float(
                "SWARM_BODY_RECENTER_MAX_FORWARD_M_S", 0.35, 0.05, 1.0
            ),
            max_right_velocity_m_s=_env_float(
                "SWARM_BODY_RECENTER_MAX_RIGHT_M_S", 0.25, 0.05, 1.0
            ),
            max_horizontal_accel_m_s2=_env_float(
                "SWARM_BODY_RECENTER_MAX_HORIZONTAL_ACCEL_M_S2",
                1.20,
                0.10,
                3.0,
            ),
            horizontal_accel_jerk_m_s3=_env_float(
                "SWARM_BODY_RECENTER_ACCEL_JERK_M_S3", 4.0, 0.1, 10.0
            ),
            velocity_damping=_env_float(
                "SWARM_BODY_RECENTER_VELOCITY_DAMPING", 0.50, 0.0, 3.0
            ),
            follow_accel_max_m_s2=_env_float(
                "SWARM_BODY_RECENTER_FOLLOW_ACCEL_MAX_M_S2",
                0.25,
                0.0,
                1.5,
            ),
            altitude_pause_error_m=_env_float(
                "SWARM_BODY_RECENTER_ALTITUDE_PAUSE_ERROR_M",
                0.25,
                0.10,
                1.0,
            ),
            altitude_exit_error_m=_env_float(
                "SWARM_BODY_RECENTER_ALTITUDE_EXIT_ERROR_M",
                0.50,
                0.20,
                2.0,
            ),
            rate_slew_deg_s2=_env_float(
                "SWARM_BODY_RECENTER_RATE_SLEW_DEG_S2", 30.0, 1.0, 180.0
            ),
            hover_thrust=_env_float(
                "SWARM_BODY_RECENTER_HOVER_THRUST", 0.50, 0.25, 0.75
            ),
            altitude_kp=_env_float(
                "SWARM_BODY_RECENTER_ALTITUDE_KP", 0.08, 0.0, 0.30
            ),
            vertical_velocity_kd=_env_float(
                "SWARM_BODY_RECENTER_VERTICAL_KD", 0.10, 0.0, 0.40
            ),
            thrust_min=_env_float(
                "SWARM_BODY_RECENTER_THRUST_MIN", 0.35, 0.10, 0.70
            ),
            thrust_max=_env_float(
                "SWARM_BODY_RECENTER_THRUST_MAX", 0.70, 0.40, 0.95
            ),
        )


class BodyAttitudeRecenterController:
    """Slow body-rate loop that unloads a tracking gimbal toward Home."""

    def __init__(self, config: BodyAttitudeRecenterConfig | None = None) -> None:
        self.config = config or BodyAttitudeRecenterConfig.from_environment()
        self.altitude_target_m: float | None = None
        self.rates_deg_s = {"roll": 0.0, "pitch": 0.0, "yaw": 0.0}
        self.acceleration_body_m_s2 = {"forward": 0.0, "right": 0.0}
        self.filtered_gimbal_angles_deg: dict[str, float] | None = None
        self.recenter_active = {"roll": False, "pitch": False, "yaw": False}
        self.recenter_enter_elapsed_s = {
            "roll": 0.0,
            "pitch": 0.0,
            "yaw": 0.0,
        }

    def reset(self) -> None:
        self.altitude_target_m = None
        self.rates_deg_s = {"roll": 0.0, "pitch": 0.0, "yaw": 0.0}
        self.acceleration_body_m_s2 = {"forward": 0.0, "right": 0.0}
        self.filtered_gimbal_angles_deg = None
        self.recenter_active = {"roll": False, "pitch": False, "yaw": False}
        self.recenter_enter_elapsed_s = {
            "roll": 0.0,
            "pitch": 0.0,
            "yaw": 0.0,
        }

    @staticmethod
    def _deadband(value: float, deadband: float) -> float:
        magnitude = abs(value)
        if magnitude <= deadband:
            return 0.0
        return math.copysign(magnitude - deadband, value)

    @staticmethod
    def _finite(mapping: dict[str, Any], key: str) -> float:
        value = float(mapping[key])
        if not math.isfinite(value):
            raise ValueError(f"{key} is not finite")
        return value

    def update(
        self,
        *,
        enabled: bool,
        gimbal_angles_deg: dict[str, Any],
        body_attitude_deg: dict[str, Any],
        altitude_m: float,
        vertical_velocity_down_m_s: float,
        horizontal_velocity_body_m_s: dict[str, Any] | None = None,
        bbox_image_error_deg: dict[str, Any] | None = None,
        dt: float,
        normalized_forward_command: float = 0.0,
    ) -> dict[str, Any]:
        cfg = self.config
        if not enabled or not cfg.enabled:
            self.reset()
            return self.output(False, 0.0, "disabled")

        roll_gimbal = self._finite(gimbal_angles_deg, "roll")
        pitch_gimbal = self._finite(gimbal_angles_deg, "pitch")
        yaw_gimbal = self._finite(gimbal_angles_deg, "yaw")
        body_roll = self._finite(body_attitude_deg, "roll")
        body_pitch = self._finite(body_attitude_deg, "pitch")
        horizontal_velocity_body_m_s = horizontal_velocity_body_m_s or {
            "forward": 0.0,
            "right": 0.0,
        }
        forward_velocity = self._finite(
            horizontal_velocity_body_m_s,
            "forward",
        )
        right_velocity = self._finite(
            horizontal_velocity_body_m_s,
            "right",
        )
        altitude_m = float(altitude_m)
        vertical_velocity_down_m_s = float(vertical_velocity_down_m_s)
        if not all(
            math.isfinite(value)
            for value in (altitude_m, vertical_velocity_down_m_s)
        ):
            raise ValueError("altitude feedback is not finite")

        if self.altitude_target_m is None:
            self.altitude_target_m = altitude_m

        dt = max(0.001, min(0.1, float(dt)))
        measured_angles = {
            "roll": roll_gimbal,
            "pitch": pitch_gimbal,
            "yaw": yaw_gimbal,
        }
        first_measurement = self.filtered_gimbal_angles_deg is None
        if first_measurement:
            # React immediately on acquisition. Subsequent IMU jitter is
            # attenuated by the time-based low-pass filter below.
            self.filtered_gimbal_angles_deg = dict(measured_angles)
        else:
            alpha = 1.0 - math.exp(-dt / cfg.filter_time_constant_s)
            for axis, measured in measured_angles.items():
                self.filtered_gimbal_angles_deg[axis] += alpha * (
                    measured - self.filtered_gimbal_angles_deg[axis]
                )

        home_angles = {
            "roll": cfg.home_roll_deg,
            "pitch": cfg.home_pitch_deg,
            "yaw": cfg.home_yaw_deg,
        }
        filtered_home_errors = {
            axis: self.filtered_gimbal_angles_deg[axis] - home_angles[axis]
            for axis in self.recenter_active
        }
        enter_deadband = max(cfg.enter_deadband_deg, cfg.deadband_deg + 0.2)
        for axis, error in filtered_home_errors.items():
            if axis != "yaw":
                self.recenter_active[axis] = False
                self.recenter_enter_elapsed_s[axis] = 0.0
                continue
            if self.recenter_active[axis]:
                if abs(error) <= cfg.deadband_deg:
                    self.recenter_active[axis] = False
                    self.recenter_enter_elapsed_s[axis] = 0.0
            elif abs(error) >= enter_deadband:
                self.recenter_enter_elapsed_s[axis] += (
                    cfg.enter_hold_s if first_measurement else dt
                )
                if self.recenter_enter_elapsed_s[axis] >= cfg.enter_hold_s:
                    self.recenter_active[axis] = True
                    self.recenter_enter_elapsed_s[axis] = 0.0
            else:
                self.recenter_enter_elapsed_s[axis] = 0.0

        errors = {
            axis: (
                self._deadband(error, cfg.deadband_deg)
                if axis == "yaw" and self.recenter_active[axis]
                else 0.0
            )
            for axis, error in filtered_home_errors.items()
        }
        bbox_image_error_deg = bbox_image_error_deg or {}
        bbox_horizontal_error = float(bbox_image_error_deg.get("yaw", 0.0))
        bbox_vertical_error = float(bbox_image_error_deg.get("pitch", 0.0))
        if not all(
            math.isfinite(value)
            for value in (bbox_horizontal_error, bbox_vertical_error)
        ):
            raise ValueError("bbox image error is not finite")
        bbox_errors = {
            # Body recenter is deliberately yaw-only. PX4 keeps roll and pitch
            # stable while the gimbal handles vertical image error.
            "roll": 0.0,
            "pitch": 0.0,
            "yaw": self._deadband(
                cfg.bbox_yaw_gain * bbox_horizontal_error,
                cfg.bbox_deadband_deg,
            ),
        }
        gimbal_errors = errors
        errors = {
            axis: gimbal_errors[axis] + bbox_errors[axis]
            for axis in gimbal_errors
        }
        # Let the body take over decisively once the gimbal has panned more
        # than +/-5 degrees. A proportional-only response is only a few
        # degrees/s at that error and allows the target to leave the image.
        yaw_error_abs = abs(filtered_home_errors["yaw"])
        if (
            cfg.yaw_fast_min_rate_deg_s > 0.0
            and yaw_error_abs >= cfg.yaw_fast_threshold_deg
        ):
            yaw_fast_excess = yaw_error_abs - cfg.yaw_fast_threshold_deg
            yaw_fast_rate = cfg.yaw_fast_min_rate_deg_s + (
                cfg.yaw_fast_kp * yaw_fast_excess
            )
            errors["yaw"] = math.copysign(
                max(abs(errors["yaw"]), yaw_fast_rate),
                filtered_home_errors["yaw"],
            )
        yaw_gain_blend = min(
            1.0,
            abs(errors["yaw"]) / cfg.yaw_adaptive_error_deg,
        )
        adaptive_yaw_kp = (
            cfg.yaw_kp
            + (cfg.yaw_kp_far - cfg.yaw_kp) * yaw_gain_blend
        )
        requested = {
            "roll": 0.0,
            "pitch": 0.0,
            "yaw": adaptive_yaw_kp * errors["yaw"],
        }
        normalized_forward_command = max(
            -1.0,
            min(1.0, float(normalized_forward_command)),
        )

        limits = {
            "roll": cfg.max_roll_rate_deg_s,
            "pitch": cfg.max_pitch_rate_deg_s,
            "yaw": cfg.max_yaw_rate_deg_s,
        }
        delta_limit = cfg.rate_slew_deg_s2 * dt
        for axis in self.rates_deg_s:
            target = max(-limits[axis], min(limits[axis], requested[axis]))
            if self.rates_deg_s[axis] * target < 0.0:
                target = 0.0
            delta = max(
                -delta_limit,
                min(delta_limit, target - self.rates_deg_s[axis]),
            )
            self.rates_deg_s[axis] += delta

        tilt_cosine = max(
            0.80,
            math.cos(math.radians(body_roll))
            * math.cos(math.radians(body_pitch)),
        )
        altitude_error = self.altitude_target_m - altitude_m
        thrust = (
            cfg.hover_thrust / tilt_cosine
            + cfg.altitude_kp * altitude_error
            + cfg.vertical_velocity_kd * vertical_velocity_down_m_s
        )
        thrust = max(cfg.thrust_min, min(cfg.thrust_max, thrust))

        desired_roll_deg = 0.0
        desired_pitch_deg = 0.0
        target_acceleration = {"forward": 0.0, "right": 0.0}

        altitude_error_abs = abs(altitude_error)
        altitude_exit = altitude_error_abs >= cfg.altitude_exit_error_m
        altitude_paused = altitude_error_abs >= cfg.altitude_pause_error_m
        if altitude_paused:
            target_acceleration = {"forward": 0.0, "right": 0.0}
        accel_delta_limit = cfg.horizontal_accel_jerk_m_s3 * dt
        for axis in self.acceleration_body_m_s2:
            delta = max(
                -accel_delta_limit,
                min(
                    accel_delta_limit,
                    target_acceleration[axis]
                    - self.acceleration_body_m_s2[axis],
                ),
            )
            self.acceleration_body_m_s2[axis] += delta
        if altitude_exit:
            self.acceleration_body_m_s2 = {"forward": 0.0, "right": 0.0}

        state = (
            "altitude_exit"
            if altitude_exit
            else "altitude_guard"
            if altitude_paused
            else "recentering"
        )
        result = self.output(not altitude_exit, thrust, state)
        result.update(
            {
                "gimbal_home_error_deg": gimbal_errors,
                "bbox_image_error_deg": {
                    "horizontal": round(bbox_horizontal_error, 3),
                    "vertical": round(bbox_vertical_error, 3),
                },
                "bbox_body_error_deg": {
                    axis: round(value, 3)
                    for axis, value in bbox_errors.items()
                },
                "body_control_error_deg": {
                    axis: round(value, 3)
                    for axis, value in errors.items()
                },
                "gimbal_filtered_deg": {
                    axis: round(value, 3)
                    for axis, value in self.filtered_gimbal_angles_deg.items()
                },
                "gimbal_raw_deg": {
                    axis: round(value, 3)
                    for axis, value in measured_angles.items()
                },
                "recenter_active_axes": dict(self.recenter_active),
                "recenter_enter_deg": round(enter_deadband, 3),
                "recenter_exit_deg": round(cfg.deadband_deg, 3),
                "recenter_enter_hold_s": round(cfg.enter_hold_s, 3),
                "altitude_target_m": round(self.altitude_target_m, 3),
                "altitude_error_m": round(altitude_error, 3),
                "tilt_compensation": round(1.0 / tilt_cosine, 4),
                "normalized_forward_command": round(
                    normalized_forward_command,
                    3,
                ),
                "body_velocity_m_s": {
                    "forward": round(forward_velocity, 4),
                    "right": round(right_velocity, 4),
                },
                "body_acceleration_m_s2": {
                    "forward": round(
                        self.acceleration_body_m_s2["forward"], 4
                    ),
                    "right": round(
                        self.acceleration_body_m_s2["right"], 4
                    ),
                },
                "desired_body_tilt_deg": {
                    "roll": round(desired_roll_deg, 3),
                    "pitch": round(desired_pitch_deg, 3),
                },
                "adaptive_yaw_kp": round(adaptive_yaw_kp, 4),
                "altitude_guard_active": altitude_paused,
                "vertical_control": "px4_absolute_z",
            }
        )
        return result

    def output(self, active: bool, thrust: float, state: str) -> dict[str, Any]:
        return {
            "ok": True,
            "active": active,
            "state": state,
            "body_rates_deg_s": dict(self.rates_deg_s),
            "thrust": round(float(thrust), 4),
        }
