"""Image-only range and virtual target helpers for PX4 Follow Target.

The range scale is empirical: it is valid only for the calibrated target,
camera FOV and image crop.  No lidar or ground-plane assumption is used.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Sequence


EARTH_RADIUS_M = 6_378_137.0


@dataclass(frozen=True)
class VisualRangeSample:
    distance_m: float
    width_px: float
    height_px: float

    @property
    def area_px2(self) -> float:
        return self.width_px * self.height_px

    @property
    def scale_px(self) -> float:
        return math.sqrt(self.area_px2)

    @property
    def aspect_ratio(self) -> float:
        return self.width_px / self.height_px


@dataclass(frozen=True)
class VisualRangeProfile:
    name: str = "target_uav_640x360"
    frame_width: int = 640
    frame_height: int = 360
    safe_distance_m: float = 10.0
    deadband_m: float = 0.5
    ema_alpha: float = 0.15
    stable_frames_required: int = 5
    max_scale_jump_ratio: float = 0.30
    # A distant UAV can look nearly square when viewed nose-on. Keep the
    # aspect check as an outlier guard, but allow that expected perspective.
    max_aspect_error_ratio: float = 0.60
    ttc_block_s: float = 3.0
    samples: tuple[VisualRangeSample, ...] = field(
        default_factory=lambda: (
            VisualRangeSample(5.0, 150.0, 90.0),
            VisualRangeSample(8.0, 95.0, 57.0),
            VisualRangeSample(10.0, 76.0, 46.0),
            VisualRangeSample(12.0, 63.0, 38.0),
            VisualRangeSample(15.0, 51.0, 31.0),
        )
    )

    def __post_init__(self) -> None:
        if self.frame_width <= 0 or self.frame_height <= 0:
            raise ValueError("calibration frame dimensions must be positive")
        if len(self.samples) < 2:
            raise ValueError("at least two visual range samples are required")
        if not 0.0 < self.ema_alpha <= 1.0:
            raise ValueError("ema_alpha must be in (0, 1]")
        distances = [sample.distance_m for sample in self.samples]
        scales = [sample.scale_px for sample in self.samples]
        if any(value <= 0.0 for value in distances + scales):
            raise ValueError("range samples must be positive")
        if distances != sorted(distances):
            raise ValueError("range samples must be ordered by distance")
        if any(scales[index] <= scales[index + 1] for index in range(len(scales) - 1)):
            raise ValueError("bbox scale must decrease as distance increases")


DEFAULT_VISUAL_RANGE_PROFILE = VisualRangeProfile()


@dataclass(frozen=True)
class VisualRangeEstimate:
    valid: bool
    ready: bool
    reason: str
    distance_raw_m: float | None
    distance_filtered_m: float | None
    scale_px: float | None
    area_px2: float | None
    aspect_ratio: float | None
    quality: float
    ttc_s: float | None
    range_state: str
    stable_frames: int


@dataclass(frozen=True)
class FilteredBBoxState:
    valid: bool
    accepted: bool
    ready: bool
    reason: str
    cx: float | None
    cy: float | None
    width: float | None
    height: float | None
    stable_frames: int
    rejected_frames: int
    center_residual_px: float | None


class VisualBBoxFilter:
    """Reject tracker jumps and smooth the bbox used by flight control."""

    def __init__(
        self,
        *,
        center_alpha: float = 0.22,
        size_alpha: float = 0.12,
        center_deadband_px: float = 2.5,
        stable_frames_required: int = 10,
        stable_center_step_px: float = 12.0,
        stable_scale_step_ratio: float = 0.12,
        max_center_rate_px_s: float = 240.0,
        max_scale_step_ratio: float = 0.28,
        hold_rejected_frames: int = 3,
    ) -> None:
        self.center_alpha = center_alpha
        self.size_alpha = size_alpha
        self.center_deadband_px = center_deadband_px
        self.stable_frames_required = stable_frames_required
        self.stable_center_step_px = stable_center_step_px
        self.stable_scale_step_ratio = stable_scale_step_ratio
        self.max_center_rate_px_s = max_center_rate_px_s
        self.max_scale_step_ratio = max_scale_step_ratio
        self.hold_rejected_frames = hold_rejected_frames
        self.reset()

    def reset(self) -> None:
        self.filtered: tuple[float, float, float, float] | None = None
        self.previous_raw: tuple[float, float, float, float] | None = None
        self.last_timestamp_s: float | None = None
        self.stable_frames = 0
        self.rejected_frames = 0

    def update(
        self,
        *,
        cx: float,
        cy: float,
        width: float,
        height: float,
        timestamp_s: float,
        tracking_valid: bool = True,
    ) -> FilteredBBoxState:
        values = (cx, cy, width, height, timestamp_s)
        if (
            not tracking_valid
            or not all(math.isfinite(float(value)) for value in values)
            or width <= 0.0
            or height <= 0.0
        ):
            return self._reject("tracking_not_confirmed")

        raw = (float(cx), float(cy), float(width), float(height))
        if self.filtered is None or self.previous_raw is None:
            self.filtered = raw
            self.previous_raw = raw
            self.last_timestamp_s = timestamp_s
            self.stable_frames = 1
            self.rejected_frames = 0
            return self._output(True, "stabilizing", 0.0)

        previous_timestamp = (
            timestamp_s
            if self.last_timestamp_s is None
            else self.last_timestamp_s
        )
        dt = max(
            1.0 / 60.0,
            min(0.2, timestamp_s - previous_timestamp),
        )
        raw_center_step = math.hypot(
            raw[0] - self.previous_raw[0],
            raw[1] - self.previous_raw[1],
        )
        previous_scale = math.sqrt(self.previous_raw[2] * self.previous_raw[3])
        raw_scale = math.sqrt(raw[2] * raw[3])
        scale_step = abs(raw_scale / max(previous_scale, 1e-6) - 1.0)
        max_center_step = max(18.0, self.max_center_rate_px_s * dt)
        if raw_center_step > max_center_step or scale_step > self.max_scale_step_ratio:
            return self._reject("bbox_jump", raw_center_step)

        filtered_cx, filtered_cy, filtered_w, filtered_h = self.filtered
        center_residual = math.hypot(raw[0] - filtered_cx, raw[1] - filtered_cy)
        if center_residual > self.center_deadband_px:
            filtered_cx += self.center_alpha * (raw[0] - filtered_cx)
            filtered_cy += self.center_alpha * (raw[1] - filtered_cy)
        filtered_w += self.size_alpha * (raw[2] - filtered_w)
        filtered_h += self.size_alpha * (raw[3] - filtered_h)
        self.filtered = (filtered_cx, filtered_cy, filtered_w, filtered_h)
        self.previous_raw = raw
        self.last_timestamp_s = timestamp_s
        self.rejected_frames = 0

        stable_sample = bool(
            raw_center_step <= self.stable_center_step_px
            and scale_step <= self.stable_scale_step_ratio
        )
        self.stable_frames = self.stable_frames + 1 if stable_sample else 0
        reason = (
            "ready"
            if self.stable_frames >= self.stable_frames_required
            else "stabilizing"
        )
        return self._output(True, reason, center_residual)

    def _reject(
        self,
        reason: str,
        residual_px: float | None = None,
    ) -> FilteredBBoxState:
        self.rejected_frames += 1
        if self.rejected_frames <= self.hold_rejected_frames and self.filtered is not None:
            return self._output(False, "holding_outlier", residual_px)
        self.stable_frames = 0
        return FilteredBBoxState(
            False, False, False, reason, None, None, None, None,
            0, self.rejected_frames, residual_px,
        )

    def _output(
        self,
        accepted: bool,
        reason: str,
        residual_px: float | None,
    ) -> FilteredBBoxState:
        assert self.filtered is not None
        return FilteredBBoxState(
            True,
            accepted,
            self.stable_frames >= self.stable_frames_required,
            reason,
            *self.filtered,
            self.stable_frames,
            self.rejected_frames,
            residual_px,
        )


class VisualBBoxRangeEstimator:
    def __init__(self, profile: VisualRangeProfile = DEFAULT_VISUAL_RANGE_PROFILE) -> None:
        self.profile = profile
        self.reset()

    def reset(self) -> None:
        self.filtered_distance_m: float | None = None
        self.previous_scale_px: float | None = None
        self.previous_area_px2: float | None = None
        self.previous_timestamp_s: float | None = None
        self.stable_frames = 0
        self.last_estimate = VisualRangeEstimate(
            False, False, "not_initialized", None, None, None, None,
            None, 0.0, None, "invalid", 0,
        )

    def _distance_from_scale(self, scale_px: float) -> tuple[float, str]:
        samples = self.profile.samples
        if scale_px >= samples[0].scale_px:
            return samples[0].distance_m, "too_close"
        if scale_px <= samples[-1].scale_px:
            return samples[-1].distance_m, "too_far"

        for near, far in zip(samples, samples[1:]):
            if near.scale_px >= scale_px >= far.scale_px:
                fraction = (
                    (near.scale_px - scale_px)
                    / (near.scale_px - far.scale_px)
                )
                distance = near.distance_m + fraction * (
                    far.distance_m - near.distance_m
                )
                return distance, "in_range"
        raise RuntimeError("visual range LUT is not monotonic")

    def update(
        self,
        *,
        width_px: float,
        height_px: float,
        frame_width: int,
        frame_height: int,
        timestamp_s: float,
        tracking_score: float,
        tracking_valid: bool = True,
    ) -> VisualRangeEstimate:
        values = (
            width_px, height_px, frame_width, frame_height,
            timestamp_s, tracking_score,
        )
        if not all(math.isfinite(float(value)) for value in values):
            return self._invalid("non_finite_bbox")
        if width_px <= 0.0 or height_px <= 0.0 or frame_width <= 0 or frame_height <= 0:
            return self._invalid("invalid_bbox")
        if not tracking_valid:
            return self._invalid("tracking_not_confirmed")

        width_cal = width_px * self.profile.frame_width / frame_width
        height_cal = height_px * self.profile.frame_height / frame_height
        area_px2 = width_cal * height_cal
        scale_px = math.sqrt(area_px2)
        aspect = width_cal / height_cal
        reference_aspect = sum(
            sample.aspect_ratio for sample in self.profile.samples
        ) / len(self.profile.samples)
        aspect_error = abs(aspect / reference_aspect - 1.0)
        if aspect_error > self.profile.max_aspect_error_ratio:
            return self._invalid(
                "aspect_ratio_outlier", scale_px, area_px2, aspect,
            )

        if self.previous_scale_px is not None:
            jump = abs(scale_px / self.previous_scale_px - 1.0)
            if jump > self.profile.max_scale_jump_ratio:
                return self._invalid(
                    "bbox_scale_jump", scale_px, area_px2, aspect,
                )

        raw_distance, range_state = self._distance_from_scale(scale_px)
        alpha = self.profile.ema_alpha
        filtered = (
            raw_distance
            if self.filtered_distance_m is None
            else (1.0 - alpha) * self.filtered_distance_m + alpha * raw_distance
        )

        ttc_s: float | None = None
        if self.previous_area_px2 is not None and self.previous_timestamp_s is not None:
            dt = timestamp_s - self.previous_timestamp_s
            if dt > 1e-3:
                area_rate = (area_px2 - self.previous_area_px2) / dt
                if area_rate > 1e-6:
                    ttc_s = max(0.0, 2.0 * area_px2 / area_rate)

        self.previous_scale_px = scale_px
        self.previous_area_px2 = area_px2
        self.previous_timestamp_s = timestamp_s
        self.filtered_distance_m = filtered
        self.stable_frames += 1
        ready = self.stable_frames >= self.profile.stable_frames_required
        score_quality = max(0.0, min(1.0, float(tracking_score)))
        quality = score_quality * max(0.0, 1.0 - aspect_error)
        if range_state != "in_range":
            quality *= 0.7
        reason = "ready" if ready else "stabilizing"
        if ttc_s is not None and ttc_s < self.profile.ttc_block_s:
            ready = False
            reason = "ttc_block"
            quality *= 0.4

        self.last_estimate = VisualRangeEstimate(
            True, ready, reason, raw_distance, filtered, scale_px,
            area_px2, aspect, quality, ttc_s, range_state,
            self.stable_frames,
        )
        return self.last_estimate

    def _invalid(
        self,
        reason: str,
        scale_px: float | None = None,
        area_px2: float | None = None,
        aspect_ratio: float | None = None,
    ) -> VisualRangeEstimate:
        self.stable_frames = 0
        self.last_estimate = VisualRangeEstimate(
            False, False, reason, None, self.filtered_distance_m,
            scale_px, area_px2, aspect_ratio, 0.0, None, "invalid", 0,
        )
        return self.last_estimate


def _normalize(vector: Sequence[float]) -> tuple[float, float, float]:
    if len(vector) != 3:
        raise ValueError("3D vector required")
    norm = math.sqrt(sum(float(value) ** 2 for value in vector))
    if not math.isfinite(norm) or norm <= 1e-9:
        raise ValueError("vector norm is invalid")
    return tuple(float(value) / norm for value in vector)  # type: ignore[return-value]


def rotate_vector_by_quaternion(
    vector: Sequence[float],
    quaternion_xyzw: Sequence[float],
) -> tuple[float, float, float]:
    if len(quaternion_xyzw) != 4:
        raise ValueError("quaternion must be [x, y, z, w]")
    x, y, z, w = (float(value) for value in quaternion_xyzw)
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if not math.isfinite(norm) or norm <= 1e-9:
        raise ValueError("quaternion is invalid")
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    vx, vy, vz = (float(value) for value in vector)
    # Optimized q * v * q^-1.
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    return (
        vx + w * tx + (y * tz - z * ty),
        vy + w * ty + (z * tx - x * tz),
        vz + w * tz + (x * ty - y * tx),
    )


class CameraRayProjector:
    """Project Gazebo camera pixels through the camera IMU pose into NED.

    Gazebo's camera sensor looks along +X, with image-right along -Y and
    image-down along -Z. The camera IMU in the x500 gimbal model has the
    same sensor pose as the RGB camera, so its quaternion rotates this FLU
    ray into Gazebo ENU. ENU is then converted to PX4 NED.
    """

    def __init__(self, fx: float, fy: float, cx: float | None = None, cy: float | None = None) -> None:
        if fx <= 0.0 or fy <= 0.0:
            raise ValueError("camera focal lengths must be positive")
        self.fx = float(fx)
        self.fy = float(fy)
        self.cx = cx
        self.cy = cy

    def pixel_to_ned_ray(
        self,
        u: float,
        v: float,
        frame_width: int,
        frame_height: int,
        camera_quaternion_xyzw: Sequence[float],
    ) -> tuple[float, float, float]:
        cx = frame_width / 2.0 if self.cx is None else float(self.cx)
        cy = frame_height / 2.0 if self.cy is None else float(self.cy)
        sensor_ray = _normalize((
            1.0,
            -(float(u) - cx) / self.fx,
            -(float(v) - cy) / self.fy,
        ))
        east, north, up = rotate_vector_by_quaternion(
            sensor_ray, camera_quaternion_xyzw,
        )
        return _normalize((north, east, -up))

    def target_ned(
        self,
        *,
        u: float,
        v: float,
        frame_width: int,
        frame_height: int,
        camera_quaternion_xyzw: Sequence[float],
        camera_position_ned: Sequence[float],
        distance_m: float,
    ) -> tuple[float, float, float]:
        if distance_m <= 0.0 or not math.isfinite(distance_m):
            raise ValueError("target distance must be positive")
        ray = self.pixel_to_ned_ray(
            u, v, frame_width, frame_height, camera_quaternion_xyzw,
        )
        if len(camera_position_ned) != 3:
            raise ValueError("camera_position_ned must have three components")
        return tuple(
            float(camera_position_ned[index]) + distance_m * ray[index]
            for index in range(3)
        )  # type: ignore[return-value]


@dataclass(frozen=True)
class FilteredTargetState:
    valid: bool
    reason: str
    position_ned: tuple[float, float, float] | None
    velocity_ned: tuple[float, float, float] | None
    age_s: float


class TargetStateFilter:
    def __init__(
        self,
        alpha: float = 0.20,
        beta: float = 0.02,
        max_jump_m: float = 3.0,
        max_speed_m_s: float = 2.5,
        max_position_rate_m_s: float = 3.0,
        position_deadband_m: float = 0.08,
        velocity_deadband_m_s: float = 0.30,
        stale_timeout_s: float = 0.5,
    ) -> None:
        self.alpha = alpha
        self.beta = beta
        self.max_jump_m = max_jump_m
        self.max_speed_m_s = max_speed_m_s
        self.max_position_rate_m_s = max_position_rate_m_s
        self.position_deadband_m = position_deadband_m
        self.velocity_deadband_m_s = velocity_deadband_m_s
        self.stale_timeout_s = stale_timeout_s
        self.reset()

    def reset(self) -> None:
        self.position: tuple[float, float, float] | None = None
        self.velocity = (0.0, 0.0, 0.0)
        self.last_timestamp_s: float | None = None

    def update(
        self,
        measurement_ned: Sequence[float],
        timestamp_s: float,
    ) -> FilteredTargetState:
        measurement = tuple(float(value) for value in measurement_ned)
        if len(measurement) != 3 or not all(math.isfinite(v) for v in measurement):
            return self.output(timestamp_s, "invalid_measurement")
        if self.position is None or self.last_timestamp_s is None:
            self.position = measurement  # type: ignore[assignment]
            self.velocity = (0.0, 0.0, 0.0)
            self.last_timestamp_s = timestamp_s
            return self.output(timestamp_s, "initialized")

        dt = timestamp_s - self.last_timestamp_s
        if dt <= 1e-3:
            return self.output(timestamp_s, "invalid_dt")
        if dt > 1.0:
            # Auto-follow can be safety-blocked for longer than the normal
            # filter cadence. Re-acquire from the current measurement instead
            # of remaining permanently stale on every retry.
            self.position = measurement  # type: ignore[assignment]
            self.velocity = (0.0, 0.0, 0.0)
            self.last_timestamp_s = timestamp_s
            return self.output(timestamp_s, "reinitialized")
        predicted = tuple(
            self.position[index] + self.velocity[index] * dt
            for index in range(3)
        )
        innovation = tuple(measurement[index] - predicted[index] for index in range(3))
        innovation_norm = math.sqrt(sum(value * value for value in innovation))
        if innovation_norm > self.max_jump_m:
            return self.output(timestamp_s, "position_jump")

        position = tuple(
            predicted[index] + self.alpha * innovation[index]
            for index in range(3)
        )
        position_step = tuple(
            position[index] - self.position[index]
            for index in range(3)
        )
        position_step_norm = math.sqrt(
            sum(value * value for value in position_step)
        )
        if position_step_norm <= self.position_deadband_m:
            position = self.position
        else:
            max_position_step = self.max_position_rate_m_s * dt
            if position_step_norm > max_position_step:
                factor = max_position_step / position_step_norm
                position = tuple(
                    self.position[index] + position_step[index] * factor
                    for index in range(3)
                )
        velocity = tuple(
            self.velocity[index] + self.beta * innovation[index] / dt
            for index in range(3)
        )
        speed = math.sqrt(sum(value * value for value in velocity))
        if speed < self.velocity_deadband_m_s:
            velocity = (0.0, 0.0, 0.0)
        elif speed > self.max_speed_m_s:
            factor = self.max_speed_m_s / speed
            velocity = tuple(value * factor for value in velocity)
        self.position = position
        self.velocity = velocity
        self.last_timestamp_s = timestamp_s
        return self.output(timestamp_s, "tracking")

    def output(self, now_s: float, reason: str = "tracking") -> FilteredTargetState:
        age = (
            math.inf if self.last_timestamp_s is None
            else max(0.0, now_s - self.last_timestamp_s)
        )
        valid = self.position is not None and age <= self.stale_timeout_s
        return FilteredTargetState(
            valid,
            reason if valid else "stale",
            self.position if valid else None,
            self.velocity if valid else None,
            age,
        )


def ned_target_to_wgs84(
    *,
    follower_lat_deg: float,
    follower_lon_deg: float,
    follower_alt_msl_m: float,
    follower_position_ned: Sequence[float],
    target_position_ned: Sequence[float],
) -> tuple[float, float, float]:
    if len(follower_position_ned) != 3 or len(target_position_ned) != 3:
        raise ValueError("NED positions must have three components")
    north = float(target_position_ned[0]) - float(follower_position_ned[0])
    east = float(target_position_ned[1]) - float(follower_position_ned[1])
    down = float(target_position_ned[2]) - float(follower_position_ned[2])
    latitude_rad = math.radians(float(follower_lat_deg))
    cos_latitude = math.cos(latitude_rad)
    if abs(cos_latitude) < 1e-6:
        raise ValueError("WGS84 conversion is unstable near the poles")
    target_lat = follower_lat_deg + math.degrees(north / EARTH_RADIUS_M)
    target_lon = follower_lon_deg + math.degrees(
        east / (EARTH_RADIUS_M * cos_latitude)
    )
    target_alt = follower_alt_msl_m - down
    return target_lat, target_lon, target_alt
