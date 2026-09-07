"""Object-agnostic visual safety, projection and target-coordinate helpers."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Sequence


EARTH_RADIUS_M = 6_378_137.0


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
    size_alpha_used: float = 0.0


class VisualBBoxFilter:
    """Reject tracker jumps and smooth the bbox used by flight control."""

    def __init__(
        self,
        *,
        center_alpha: float = 0.22,
        size_alpha: float = 0.20,
        max_size_alpha: float = 0.48,
        center_deadband_px: float = 2.5,
        stable_frames_required: int = 10,
        stable_center_step_px: float = 18.0,
        stable_scale_step_ratio: float = 0.12,
        max_center_rate_px_s: float = 240.0,
        max_scale_step_ratio: float = 0.28,
        hold_rejected_frames: int = 3,
    ) -> None:
        self.center_alpha = center_alpha
        self.size_alpha = max(0.01, min(1.0, size_alpha))
        self.max_size_alpha = max(
            self.size_alpha,
            min(1.0, max_size_alpha),
        )
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
        confidence: float = 1.0,
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
        confidence = (
            float(confidence)
            if math.isfinite(float(confidence))
            else 0.0
        )
        confidence = max(0.0, min(1.0, confidence))
        size_alpha = self.size_alpha + (
            self.max_size_alpha - self.size_alpha
        ) * confidence * confidence
        filtered_w += size_alpha * (raw[2] - filtered_w)
        filtered_h += size_alpha * (raw[3] - filtered_h)
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
        return self._output(True, reason, center_residual, size_alpha)

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
            0, self.rejected_frames, residual_px, 0.0,
        )

    def _output(
        self,
        accepted: bool,
        reason: str,
        residual_px: float | None,
        size_alpha_used: float | None = None,
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
            (
                self.size_alpha
                if size_alpha_used is None
                else float(size_alpha_used)
            ),
        )


@dataclass(frozen=True)
class BBoxMotionEstimate:
    valid: bool
    ready: bool
    reason: str
    scale_ratio: float | None
    area_ratio: float | None
    quality: float
    ttc_s: float | None
    stable_frames: int


class BBoxMotionSafetyEstimator:
    """Object-agnostic apparent-size/TTC guard with no metric-range output."""

    def __init__(
        self,
        *,
        maximum_scale_jump_ratio: float = 0.45,
        stable_frames_required: int = 4,
        ttc_block_s: float = 2.0,
    ) -> None:
        self.maximum_scale_jump_ratio = max(0.05, maximum_scale_jump_ratio)
        self.stable_frames_required = max(1, stable_frames_required)
        self.ttc_block_s = max(0.1, ttc_block_s)
        self.reset()

    def reset(self) -> None:
        self.reference_scale_ratio: float | None = None
        self.previous_scale_ratio: float | None = None
        self.previous_area_ratio: float | None = None
        self.previous_timestamp_s: float | None = None
        self.stable_frames = 0

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
    ) -> BBoxMotionEstimate:
        values = (
            width_px,
            height_px,
            frame_width,
            frame_height,
            timestamp_s,
            tracking_score,
        )
        if (
            not tracking_valid
            or not all(math.isfinite(float(value)) for value in values)
            or width_px <= 0.0
            or height_px <= 0.0
            or frame_width <= 0
            or frame_height <= 0
        ):
            self.stable_frames = 0
            return BBoxMotionEstimate(
                False, False, "tracking_invalid", None, None, 0.0, None, 0
            )
        area_ratio = (width_px * height_px) / float(frame_width * frame_height)
        scale_ratio = math.sqrt(max(1e-12, area_ratio))
        if self.reference_scale_ratio is None:
            self.reference_scale_ratio = scale_ratio
        if self.previous_scale_ratio is not None:
            jump = abs(scale_ratio / self.previous_scale_ratio - 1.0)
            if jump > self.maximum_scale_jump_ratio:
                self.stable_frames = 0
                return BBoxMotionEstimate(
                    False,
                    False,
                    "bbox_scale_jump",
                    scale_ratio,
                    area_ratio,
                    0.0,
                    None,
                    0,
                )

        ttc_s: float | None = None
        if self.previous_area_ratio is not None and self.previous_timestamp_s is not None:
            dt = timestamp_s - self.previous_timestamp_s
            if dt > 1e-3:
                area_rate = (area_ratio - self.previous_area_ratio) / dt
                if area_rate > 1e-9:
                    ttc_s = max(0.0, 2.0 * area_ratio / area_rate)
        self.previous_scale_ratio = scale_ratio
        self.previous_area_ratio = area_ratio
        self.previous_timestamp_s = timestamp_s
        self.stable_frames += 1
        ready = self.stable_frames >= self.stable_frames_required
        reason = "ready" if ready else "stabilizing"
        quality = max(0.0, min(1.0, float(tracking_score)))
        if ttc_s is not None and ttc_s < self.ttc_block_s:
            ready = False
            reason = "ttc_block"
            quality *= 0.4
        return BBoxMotionEstimate(
            True,
            ready,
            reason,
            scale_ratio,
            area_ratio,
            quality,
            ttc_s,
            self.stable_frames,
        )


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


def selection_anchored_target_ned(
    *,
    camera_position_ned: Sequence[float],
    vehicle_position_ned: Sequence[float],
    bearing_ned: Sequence[float],
    safe_horizontal_distance_m: float,
) -> tuple[tuple[float, float, float], float]:
    """Place a provisional target on the selected camera ray.

    The positive ray distance is chosen so the target's horizontal distance
    from the vehicle at selection is exactly ``safe_horizontal_distance_m``.
    This is an explicitly provisional monocular scale anchor, not a claim that
    RGB supplied an absolute range measurement.
    """

    camera = tuple(float(value) for value in camera_position_ned)
    vehicle = tuple(float(value) for value in vehicle_position_ned)
    bearing = tuple(float(value) for value in bearing_ned)
    distance = float(safe_horizontal_distance_m)
    if (
        len(camera) != 3
        or len(vehicle) != 3
        or len(bearing) != 3
        or not all(math.isfinite(value) for value in camera + vehicle + bearing)
    ):
        raise ValueError("selection target inputs must contain finite NED vectors")
    if not math.isfinite(distance) or distance < 3.0 or distance > 20.0:
        raise ValueError("safe horizontal distance must be within 3-20 m")
    bearing_norm = math.sqrt(sum(value * value for value in bearing))
    if bearing_norm <= 1e-9:
        raise ValueError("selection bearing has zero length")
    bearing = tuple(value / bearing_norm for value in bearing)
    horizontal_norm_sq = bearing[0] ** 2 + bearing[1] ** 2
    if horizontal_norm_sq <= 1e-6:
        raise ValueError("selection bearing has insufficient horizontal component")

    camera_offset_north = camera[0] - vehicle[0]
    camera_offset_east = camera[1] - vehicle[1]
    quadratic_b = 2.0 * (
        camera_offset_north * bearing[0]
        + camera_offset_east * bearing[1]
    )
    quadratic_c = (
        camera_offset_north**2
        + camera_offset_east**2
        - distance**2
    )
    discriminant = quadratic_b**2 - 4.0 * horizontal_norm_sq * quadratic_c
    if discriminant < 0.0:
        raise ValueError("selection ray does not intersect the safe-distance circle")
    root = math.sqrt(max(0.0, discriminant))
    candidates = (
        (-quadratic_b + root) / (2.0 * horizontal_norm_sq),
        (-quadratic_b - root) / (2.0 * horizontal_norm_sq),
    )
    positive = [candidate for candidate in candidates if candidate > 0.0]
    if not positive:
        raise ValueError("safe-distance intersection is behind the camera")
    slant_range_m = min(positive)
    target = tuple(
        camera[index] + slant_range_m * bearing[index]
        for index in range(3)
    )
    return target, slant_range_m  # type: ignore[return-value]


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
