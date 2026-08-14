from __future__ import annotations

import asyncio
import math
import os
import sys
import threading
import time
from collections import deque
from dataclasses import replace
from pathlib import Path
from typing import Any, Callable

from bearing_target_estimator import (
    BearingObservation,
    BearingTargetEstimator,
    TargetEstimate,
)
from body_yaw_recenter import BodyYawRecenterController
from camera_source_trace import record as _trace_camera_hop
from follow_workflow import (
    FollowWorkflowMachine,
    FollowWorkflowState,
    SelectionSnapshot,
)
from metric_target_fusion import MetricTargetFusion
from visual_follow_target import (
    BBoxMotionSafetyEstimator,
    CameraRayProjector,
    TargetStateFilter,
    VisualBBoxFilter,
    ned_target_to_wgs84,
    selection_anchored_target_ned,
)


SYSTEM_DIST_PACKAGES = Path("/usr/lib/python3/dist-packages")
REQUESTED_TRACKING_PACKAGE_ROOT = Path(
    os.environ.get(
        "SWARM_TRACKING_PACKAGE_ROOT",
        "/home/sup/ws_px4/src/lfc_gimbal_gazebo",
    )
)


def _finite_or_none(value: Any) -> float | None:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return number if math.isfinite(number) else None


def _finite_position(value: Any) -> tuple[float, float, float] | None:
    if not isinstance(value, (list, tuple)) or len(value) != 3:
        return None
    numbers = tuple(_finite_or_none(item) for item in value)
    if any(item is None for item in numbers):
        return None
    return tuple(float(item) for item in numbers)  # type: ignore[arg-type]


def _pose_sync_diagnostics(pose: dict[str, Any]) -> dict[str, Any]:
    numeric_fields = (
        "frame_timestamp_s",
        "telemetry_age_ms",
        "camera_age_ms",
        "pose_age_ms",
        "gimbal_age_ms",
        "pose_sample_offset_ms",
        "gimbal_sample_offset_ms",
        "pose_interpolation_span_ms",
        "gimbal_interpolation_span_ms",
        "pose_lower_delta_ms",
        "pose_upper_delta_ms",
        "gimbal_lower_delta_ms",
        "gimbal_upper_delta_ms",
    )
    diagnostics: dict[str, Any] = {
        key: _finite_or_none(pose.get(key))
        for key in numeric_fields
        if pose.get(key) is not None
    }
    for key in ("pose_sync_reason", "gimbal_sync_reason", "error"):
        if pose.get(key):
            diagnostics[key] = str(pose[key])
    diagnostics["available"] = bool(pose.get("available", False))
    return diagnostics


def camera_angular_error_deg(
    *,
    center_x_px: float,
    center_y_px: float,
    frame_width_px: int,
    frame_height_px: int,
    calibrated_fx_px: float,
    calibrated_fy_px: float,
    calibration_width_px: int,
    calibration_height_px: int,
) -> tuple[float, float]:
    """Convert image error to yaw/pitch error at any scaled resolution."""

    width = max(1.0, float(frame_width_px))
    height = max(1.0, float(frame_height_px))
    reference_width = max(1.0, float(calibration_width_px))
    reference_height = max(1.0, float(calibration_height_px))
    fx = max(1.0, float(calibrated_fx_px)) * width / reference_width
    fy = max(1.0, float(calibrated_fy_px)) * height / reference_height
    return (
        math.degrees(math.atan2(float(center_x_px) - width / 2.0, fx)),
        math.degrees(math.atan2(height / 2.0 - float(center_y_px), fy)),
    )


def apply_gimbal_yaw_direction(
    yaw_rate_rad_s: float,
    *,
    inverted: bool,
) -> float:
    """Map camera-controller yaw into the configured actuator convention."""

    rate = float(yaw_rate_rad_s)
    return -rate if inverted else rate


def apply_gimbal_pitch_direction(
    pitch_rate_rad_s: float,
    *,
    inverted: bool,
) -> float:
    """Map camera-controller pitch into the configured actuator convention."""

    rate = float(pitch_rate_rad_s)
    return -rate if inverted else rate


def lateral_bootstrap_unit_ned(
    bearing_ned: tuple[float, float, float],
    direction: str,
) -> tuple[float, float]:
    """Return local-NED lateral motion perpendicular to horizontal LOS."""

    north = float(bearing_ned[0])
    east = float(bearing_ned[1])
    norm = math.hypot(north, east)
    if not math.isfinite(norm) or norm < 1e-3:
        raise ValueError("horizontal target bearing is invalid")
    direction = str(direction).strip().lower()
    if direction not in {"left", "right"}:
        raise ValueError("bootstrap direction must be left or right")
    sign = 1.0 if direction == "right" else -1.0
    return sign * -east / norm, sign * north / norm


def predict_bbox_center_px(
    *,
    center_x_px: float,
    center_y_px: float,
    velocity_x_px_per_frame: float,
    velocity_y_px_per_frame: float,
    source_fps: float,
    latency_s: float,
    maximum_horizon_s: float,
    frame_width_px: int,
    frame_height_px: int,
) -> tuple[float, float, float]:
    """Predict a tracked center over a short, explicitly clamped horizon."""

    fps = max(1.0, min(120.0, float(source_fps)))
    horizon = max(0.0, min(float(maximum_horizon_s), float(latency_s)))
    frame_steps = min(3.0, horizon * fps)
    predicted_x = float(center_x_px) + float(velocity_x_px_per_frame) * frame_steps
    predicted_y = float(center_y_px) + float(velocity_y_px_per_frame) * frame_steps
    return (
        max(0.0, min(float(frame_width_px - 1), predicted_x)),
        max(0.0, min(float(frame_height_px - 1), predicted_y)),
        horizon,
    )


def _resolve_tracking_package_root(requested: Path) -> Path:
    candidates = [requested, Path(f"{requested} ")]
    parent = requested.parent
    if parent.exists():
        candidates.extend(
            path
            for path in parent.glob(f"{requested.name}*")
            if path.is_dir()
        )
    return next((path for path in candidates if path.is_dir()), requested)


TRACKING_PACKAGE_ROOT = _resolve_tracking_package_root(
    REQUESTED_TRACKING_PACKAGE_ROOT
)


def apparent_size_error(reference_scale_px: float, current_scale_px: float) -> float:
    """Positive when the target is smaller than the selected reference bbox."""
    reference = max(1.0, float(reference_scale_px))
    current = max(1.0, float(current_scale_px))
    return math.log(reference / current)


def safe_follow_band_error(
    measured_distance_m: float,
    minimum_distance_m: float,
    maximum_distance_m: float,
) -> float:
    """Signed distance to the nearest safe-band boundary.

    Positive means the target is too far and requires forward motion.
    Negative means it is too close and requires reverse motion.
    """
    measured = float(measured_distance_m)
    minimum = float(minimum_distance_m)
    maximum = float(maximum_distance_m)
    if not all(math.isfinite(value) for value in (measured, minimum, maximum)):
        return 0.0
    if maximum < minimum:
        minimum, maximum = maximum, minimum
    if measured < minimum:
        return measured - minimum
    if measured > maximum:
        return measured - maximum
    return 0.0


def braking_speed_limit_m_s(
    distance_to_boundary_m: float,
    deceleration_m_s2: float,
) -> float:
    """Maximum speed that can stop at a boundary with constant deceleration."""
    distance = max(0.0, float(distance_to_boundary_m))
    deceleration = max(0.0, float(deceleration_m_s2))
    return math.sqrt(2.0 * deceleration * distance)


def metric_follow_command_target(
    measured_distance_m: float,
    minimum_distance_m: float,
    maximum_distance_m: float,
    *,
    proportional_gain: float,
    maximum_command: float,
    maximum_velocity_m_s: float,
    brake_deceleration_m_s2: float,
    radial_velocity_m_s: float = 0.0,
    radial_velocity_gain: float = 0.0,
) -> tuple[float, float]:
    """Return signed safe-band error and a braking-limited command."""
    distance_error = safe_follow_band_error(
        measured_distance_m,
        minimum_distance_m,
        maximum_distance_m,
    )
    command = proportional_gain * distance_error
    if abs(distance_error) > 0.0:
        command += (
            radial_velocity_gain
            * radial_velocity_m_s
            * maximum_command
            / max(0.1, maximum_velocity_m_s)
        )
        braking_speed = braking_speed_limit_m_s(
            abs(distance_error),
            brake_deceleration_m_s2,
        )
        braking_command = (
            braking_speed
            * maximum_command
            / max(0.1, maximum_velocity_m_s)
        )
        command = math.copysign(
            min(abs(command), braking_command),
            distance_error,
        )
    else:
        command = 0.0
    return (
        distance_error,
        max(-maximum_command, min(maximum_command, command)),
    )


def slew_toward(
    previous: float,
    target: float,
    rate_per_second: float,
    dt_s: float,
) -> float:
    delta_limit = max(0.0, rate_per_second) * max(
        0.0,
        min(0.1, dt_s),
    )
    return previous + max(
        -delta_limit,
        min(delta_limit, target - previous),
    )


def visual_follow_center_error_deg(
    cx: float,
    cy: float,
    frame_width: int,
    frame_height: int,
    fx: float,
    fy: float,
) -> float:
    horizontal = math.degrees(
        math.atan2(float(cx) - frame_width / 2.0, max(1.0, float(fx)))
    )
    vertical = math.degrees(
        math.atan2(frame_height / 2.0 - float(cy), max(1.0, float(fy)))
    )
    return math.hypot(horizontal, vertical)


def native_precenter_yaw_rate_deg_s(
    gimbal_yaw_deg: float,
    bbox_horizontal_error_deg: float,
    threshold_deg: float = 5.0,
    minimum_rate_deg_s: float = 18.0,
    maximum_rate_deg_s: float = 45.0,
    proportional_gain: float = 1.5,
) -> float:
    """Yaw the body while native Follow waits for the bbox center lock."""
    gimbal_yaw_deg = float(gimbal_yaw_deg)
    bbox_horizontal_error_deg = float(bbox_horizontal_error_deg)
    if not all(
        math.isfinite(value)
        for value in (gimbal_yaw_deg, bbox_horizontal_error_deg)
    ):
        return 0.0
    if (
        abs(gimbal_yaw_deg) < threshold_deg
        and abs(bbox_horizontal_error_deg) < threshold_deg
    ):
        return 0.0
    line_of_sight_error = gimbal_yaw_deg + bbox_horizontal_error_deg
    if abs(line_of_sight_error) < 0.5:
        return 0.0
    rate = max(
        minimum_rate_deg_s,
        proportional_gain * max(0.0, abs(line_of_sight_error) - threshold_deg),
    )
    return math.copysign(min(maximum_rate_deg_s, rate), line_of_sight_error)


def unified_recenter_yaw_rate_deg_s(
    gimbal_yaw_deg: float,
    bbox_horizontal_error_deg: float,
    previous_rate_deg_s: float = 0.0,
    dt_s: float = 1.0 / 30.0,
    *,
    deadband_deg: float = 1.0,
    proportional_gain: float = 2.0,
    maximum_rate_deg_s: float = 60.0,
    slew_rate_deg_s2: float = 180.0,
    invert: bool = False,
) -> float:
    """Compute the single OFFBOARD body-yaw command with rate limiting."""
    values = (
        gimbal_yaw_deg,
        bbox_horizontal_error_deg,
        previous_rate_deg_s,
        dt_s,
    )
    if not all(math.isfinite(float(value)) for value in values):
        return 0.0
    error_deg = float(gimbal_yaw_deg) + float(bbox_horizontal_error_deg)
    if invert:
        error_deg *= -1.0
    deadband = max(0.0, float(deadband_deg))
    if abs(error_deg) <= deadband:
        target = 0.0
    else:
        target = math.copysign(
            min(
                max(0.0, float(maximum_rate_deg_s)),
                max(0.0, float(proportional_gain))
                * (abs(error_deg) - deadband),
            ),
            error_deg,
        )
    max_delta = (
        max(0.0, float(slew_rate_deg_s2))
        * max(0.001, min(0.2, float(dt_s)))
    )
    previous = float(previous_rate_deg_s)
    return previous + max(
        -max_delta,
        min(max_delta, target - previous),
    )


def px4_follow_mode_confirmed(nav_state: int, mode_request_ack: str) -> bool:
    """Require both MAVLink command ACK and PX4's observed nav state."""

    return bool(
        int(nav_state) == 19
        and str(mode_request_ack).strip().lower() == "accepted"
    )


def update_gimbal_yaw_recenter_latch(
    latched: bool,
    gimbal_yaw_deg: float,
    bbox_horizontal_error_deg: float,
    *,
    yaw_limit_deg: float = 10.0,
    saturated_outward: bool = False,
    home_exit_deg: float = 1.0,
    bbox_center_exit_deg: float = 1.0,
) -> bool:
    """Latch body recenter at the gimbal limit until both errors are small."""
    values = (
        gimbal_yaw_deg,
        bbox_horizontal_error_deg,
        yaw_limit_deg,
        home_exit_deg,
        bbox_center_exit_deg,
    )
    if not all(math.isfinite(float(value)) for value in values):
        return False
    yaw_abs = abs(float(gimbal_yaw_deg))
    image_abs = abs(float(bbox_horizontal_error_deg))
    if latched:
        return not (
            yaw_abs <= max(0.0, float(home_exit_deg))
            and image_abs <= max(0.0, float(bbox_center_exit_deg))
        )
    limit = max(0.1, abs(float(yaw_limit_deg)))
    return bool(saturated_outward or yaw_abs >= limit - 0.05)


def hold_visual_follow_radius(
    follower_ned: tuple[float, float, float],
    target_ned: tuple[float, float, float],
    target_velocity_ned: tuple[float, float, float],
    follow_distance_m: float,
    measured_distance_m: float,
    tolerance_m: float,
) -> tuple[tuple[float, float, float], tuple[float, float, float], bool]:
    """Deprecated compatibility helper that never creates a ghost target.

    Native PX4 Follow receives the physical target coordinate unchanged.
    Radial spacing is configured exclusively through ``FLW_TGT_DST``.
    """

    _ = (follower_ned, follow_distance_m, measured_distance_m, tolerance_m)
    return target_ned, target_velocity_ned, False


def soft_handover_range_m(
    follow_distance_m: float,
    measured_distance_m: float,
    elapsed_s: float,
    duration_s: float,
) -> tuple[float, float]:
    """Return the observed physical range; never blend target coordinates."""
    _ = float(follow_distance_m)
    measured_distance_m = float(measured_distance_m)
    duration_s = max(0.1, float(duration_s))
    blend = max(0.0, min(1.0, float(elapsed_s) / duration_s))
    return measured_distance_m, blend


def _bbox_crop(frame: Any, bbox: Any) -> Any:
    frame_height, frame_width = frame.shape[:2]
    x0 = max(0, min(frame_width - 1, int(math.floor(float(bbox.x)))))
    y0 = max(0, min(frame_height - 1, int(math.floor(float(bbox.y)))))
    x1 = max(x0 + 1, min(frame_width, int(math.ceil(float(bbox.x + bbox.w)))))
    y1 = max(y0 + 1, min(frame_height, int(math.ceil(float(bbox.y + bbox.h)))))
    return frame[y0:y1, x0:x1]


def object_pixel_scale_px(frame: Any, bbox: Any) -> tuple[float, str]:
    """Estimate target size from foreground object pixels inside the bbox."""
    crop = _bbox_crop(frame, bbox)
    if crop.size == 0:
        return 0.0, "empty"

    height, width = crop.shape[:2]
    bbox_area = max(1.0, float(width * height))
    if width < 10 or height < 10 or cv2 is None or np is None:
        return math.sqrt(bbox_area), "bbox_fallback"

    foreground_count = 0
    source = "object_contrast"
    min_pixels = max(24, int(bbox_area * 0.03))
    max_pixels = int(bbox_area * 0.95)
    try:
        gray = cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        border = np.concatenate(
            (
                blurred[0, :],
                blurred[-1, :],
                blurred[:, 0],
                blurred[:, -1],
            )
        )
        background_level = float(np.median(border))
        diff = cv2.absdiff(
            blurred,
            np.full_like(blurred, int(round(background_level))),
        )
        _, foreground = cv2.threshold(
            diff,
            0,
            255,
            cv2.THRESH_BINARY + cv2.THRESH_OTSU,
        )
        kernel = np.ones((3, 3), np.uint8)
        foreground = cv2.morphologyEx(foreground, cv2.MORPH_OPEN, kernel)
        foreground = cv2.morphologyEx(foreground, cv2.MORPH_CLOSE, kernel)
        foreground_count = int(cv2.countNonZero(foreground))
    except Exception:
        foreground_count = 0

    allow_grabcut = os.environ.get(
        "SWARM_OBJECT_PIXEL_SCALE_ALLOW_GRABCUT",
        "false",
    ).strip().lower() in {"1", "true", "yes", "on"}
    if (
        allow_grabcut
        and (foreground_count < min_pixels or foreground_count > max_pixels)
    ):
        source = "object_grabcut"
        try:
            margin_x = max(1, int(width * 0.08))
            margin_y = max(1, int(height * 0.08))
            rect_w = max(1, width - 2 * margin_x)
            rect_h = max(1, height - 2 * margin_y)
            mask = np.zeros((height, width), np.uint8)
            bgd_model = np.zeros((1, 65), np.float64)
            fgd_model = np.zeros((1, 65), np.float64)
            cv2.grabCut(
                crop,
                mask,
                (margin_x, margin_y, rect_w, rect_h),
                bgd_model,
                fgd_model,
                1,
                cv2.GC_INIT_WITH_RECT,
            )
            foreground = np.where(
                (mask == cv2.GC_FGD) | (mask == cv2.GC_PR_FGD),
                255,
                0,
            ).astype("uint8")
            foreground_count = int(cv2.countNonZero(foreground))
        except Exception:
            foreground_count = 0

    if foreground_count < min_pixels or foreground_count > max_pixels:
        return math.sqrt(bbox_area), "bbox_fallback"
    return math.sqrt(float(foreground_count)), source


if (
    SYSTEM_DIST_PACKAGES.exists()
    and str(SYSTEM_DIST_PACKAGES) not in sys.path
):
    sys.path.append(str(SYSTEM_DIST_PACKAGES))

if str(TRACKING_PACKAGE_ROOT) in sys.path:
    sys.path.remove(str(TRACKING_PACKAGE_ROOT))
if TRACKING_PACKAGE_ROOT.exists():
    sys.path.insert(0, str(TRACKING_PACKAGE_ROOT))

try:
    import cv2
    import numpy as np

    from lfc_gimbal_gazebo.config import GIMBAL as PACKAGE_GIMBAL
    from lfc_gimbal_gazebo.models.types import BBox, TrackResult, TrackState
    from lfc_gimbal_gazebo.services.gimbal_service import GimbalService
    from tracking_hybrid import HybridMemoryTrackerService

    TRACKING_IMPORT_ERROR = ""
except Exception as error:
    cv2 = None
    np = None
    BBox = None
    TrackResult = None
    TrackState = None
    PACKAGE_GIMBAL = None
    GimbalService = None
    HybridMemoryTrackerService = None
    TRACKING_IMPORT_ERROR = str(error)

try:
    from lfc_gimbal_gazebo.services.inference_service import InferenceService
    from lfc_gimbal_gazebo.services.tracker_service import TrackerService
except Exception:
    InferenceService = None
    TrackerService = None


class FastOpenCVTrackerService:
    def __init__(self, algorithm: str) -> None:
        self.algorithm = algorithm.upper()
        self.reset()

    def reset(self) -> None:
        self._tracker: Any = None
        self._bbox: Any = None
        self.frame_idx = 0
        self.state = TrackState.IDLE if TrackState is not None else None

    @property
    def active(self) -> bool:
        return self._tracker is not None

    def init(self, frame: Any, bbox: Any) -> None:
        factories = {
            "CSRT": cv2.TrackerCSRT_create,
            "KCF": cv2.TrackerKCF_create,
            "MOSSE": cv2.legacy.TrackerMOSSE_create,
        }
        factory = factories.get(self.algorithm)

        if factory is None:
            raise RuntimeError(
                f"Unsupported OpenCV tracker: {self.algorithm}"
            )

        self.reset()
        self._tracker = factory()
        initial_bbox = tuple(
            int(round(value))
            for value in (bbox.x, bbox.y, bbox.w, bbox.h)
        )
        result = self._tracker.init(frame, initial_bbox)

        if result is False:
            self.reset()
            raise RuntimeError(
                f"OpenCV {self.algorithm} could not initialize the bbox"
            )

        self._bbox = bbox
        self.state = TrackState.TRACKING

    def update(self, frame: Any) -> Any:
        if not self.active:
            raise RuntimeError("Tracker is not initialized")
        self.frame_idx += 1
        success, raw_bbox = self._tracker.update(frame)
        if success:
            self._bbox = BBox(*map(float, raw_bbox))
            self.state = TrackState.TRACKING
            score = 1.0
        else:
            self.state = TrackState.LOST
            score = 0.0
        return TrackResult(
            state=self.state,
            bbox=self._bbox,
            score=score,
            frame_idx=self.frame_idx,
            redetecting=False,
        )


class TrackingManager:
    def __init__(
        self,
        gazebo_bridge: Any,
        allowed_drones: set[str],
        gimbal_command: Callable[..., dict[str, Any]] | None = None,
        body_yaw_command: Callable[..., dict[str, Any]] | None = None,
        follow_command: Callable[..., dict[str, Any]] | None = None,
        motion_command: Callable[..., dict[str, Any]] | None = None,
        attitude_command: Callable[..., dict[str, Any]] | None = None,
        bootstrap_command: Callable[..., dict[str, Any]] | None = None,
        visual_target_command: Callable[..., dict[str, Any]] | None = None,
        pose_provider: Callable[..., dict[str, Any]] | None = None,
        ground_truth_provider: Callable[..., dict[str, Any]] | None = None,
    ) -> None:
        self.gazebo_bridge = gazebo_bridge
        self.allowed_drones = allowed_drones
        self.gimbal_command = gimbal_command
        self.body_yaw_command = body_yaw_command
        self.follow_command = follow_command
        self.motion_command = motion_command
        self.attitude_command = attitude_command
        self.bootstrap_command = bootstrap_command
        self.visual_target_command = visual_target_command
        self.pose_provider = pose_provider
        self.ground_truth_provider = ground_truth_provider
        self.backend = os.environ.get(
            "SWARM_TRACKING_BACKEND",
            "hybrid",
        ).strip().lower()
        self.board_ip = os.environ.get(
            "SWARM_TRACKING_BOARD_IP",
            "192.168.4.89",
        ).strip()
        self.opencv_tracker = os.environ.get(
            "SWARM_OPENCV_TRACKER",
            "KCF",
        ).strip().upper()
        try:
            self.gimbal_home_pitch_deg = max(
                -45.0,
                min(
                    20.0,
                    float(os.environ.get("SWARM_GIMBAL_HOME_PITCH_DEG", "0.0")),
                ),
            )
        except ValueError:
            self.gimbal_home_pitch_deg = 0.0
        try:
            self.jpeg_quality = max(
                45,
                min(
                    95,
                    int(os.environ.get("SWARM_TRACKING_JPEG_QUALITY", "72")),
                ),
            )
        except ValueError:
            self.jpeg_quality = 72
        try:
            self.preview_max_width = max(
                160,
                min(
                    1920,
                    int(
                        os.environ.get(
                            "SWARM_TRACKING_PREVIEW_MAX_WIDTH",
                            "640",
                        )
                    ),
                ),
            )
        except ValueError:
            self.preview_max_width = 640
        self.preview_publish_enabled = os.environ.get(
            "SWARM_TRACKING_PREVIEW_PUBLISH_ENABLED", "true"
        ).strip().lower() not in {"0", "false", "no", "off"}
        self.gimbal_enabled = os.environ.get(
            "SWARM_TRACKING_GIMBAL_ENABLED",
            "true",
        ).strip().lower() not in {"0", "false", "no", "off"}
        self.gimbal_yaw_invert = os.environ.get(
            "SWARM_TRACKING_GIMBAL_YAW_INVERT",
            "false",
        ).strip().lower() in {"1", "true", "yes", "on"}
        self.gimbal_pitch_invert = os.environ.get(
            "SWARM_TRACKING_GIMBAL_PITCH_INVERT",
            "false",
        ).strip().lower() in {"1", "true", "yes", "on"}
        try:
            self.configured_gimbal_yaw_limit_deg = max(
                5.0,
                min(
                    45.0,
                    float(
                        os.environ.get(
                            "SWARM_TRACKING_GIMBAL_YAW_LIMIT_DEG",
                            "10",
                        )
                    ),
                ),
            )
            self.gimbal_yaw_home_exit_deg = max(
                0.1,
                min(
                    5.0,
                    float(
                        os.environ.get(
                            "SWARM_TRACKING_GIMBAL_YAW_HOME_EXIT_DEG",
                            "1.0",
                        )
                    ),
                ),
            )
            self.gimbal_bbox_center_exit_deg = max(
                0.1,
                min(
                    5.0,
                    float(
                        os.environ.get(
                            "SWARM_TRACKING_GIMBAL_BBOX_CENTER_EXIT_DEG",
                            "1.0",
                        )
                    ),
                ),
            )
        except ValueError:
            self.configured_gimbal_yaw_limit_deg = 10.0
            self.gimbal_yaw_home_exit_deg = 1.0
            self.gimbal_bbox_center_exit_deg = 1.0
        self.body_yaw_enabled = os.environ.get(
            "SWARM_TRACKING_BODY_YAW_ENABLED",
            "false",
        ).strip().lower() in {"1", "true", "yes", "on"}
        self.follow_enabled = os.environ.get(
            "SWARM_TRACKING_FOLLOW_ENABLED",
            "false",
        ).strip().lower() in {"1", "true", "yes", "on"}
        self.visual_follow_feature_enabled = os.environ.get(
            "SWARM_VISUAL_FOLLOW_TARGET_ENABLED",
            "true",
        ).strip().lower() not in {"0", "false", "no", "off"}
        self.native_visual_follow_enabled = os.environ.get(
            "SWARM_NATIVE_FOLLOW_TARGET_ENABLED",
            "false",
        ).strip().lower() in {"1", "true", "yes", "on"}
        self.apparent_size_follow_enabled = os.environ.get(
            "SWARM_APPARENT_SIZE_FOLLOW_ENABLED",
            "false",
        ).strip().lower() in {"1", "true", "yes", "on"}
        self.legacy_automation_enabled = os.environ.get(
            "SWARM_LEGACY_AUTOMATION_ENABLED",
            "false",
        ).strip().lower() in {"1", "true", "yes", "on"}
        # Safe native Follow always requires a fresh explicit user action.
        # The deprecated auto-start environment variable is intentionally
        # ignored rather than retained as a hidden authority path.
        self.visual_follow_auto_start = False
        try:
            self.visual_follow_start_delay_s = max(
                0.0,
                min(
                    2.0,
                    float(
                        os.environ.get(
                            "SWARM_VISUAL_FOLLOW_START_DELAY_S",
                            "0.50",
                        )
                    ),
                ),
            )
        except ValueError:
            self.visual_follow_start_delay_s = 0.50
        try:
            self.visual_follow_center_deadband_deg = max(
                0.5,
                min(
                    15.0,
                    float(
                        os.environ.get(
                            "SWARM_VISUAL_FOLLOW_CENTER_DEADBAND_DEG",
                            "5.0",
                        )
                    ),
                ),
            )
            self.visual_follow_center_hold_s = max(
                0.1,
                min(
                    3.0,
                    float(
                        os.environ.get(
                            "SWARM_VISUAL_FOLLOW_CENTER_HOLD_S",
                            "0.15",
                        )
                    ),
                ),
            )
            self.visual_follow_min_stable_frames = max(
                1,
                min(
                    60,
                    int(
                        os.environ.get(
                            "SWARM_VISUAL_FOLLOW_MIN_STABLE_FRAMES",
                            "12",
                        )
                    ),
                ),
            )
        except ValueError:
            self.visual_follow_center_deadband_deg = 5.0
            self.visual_follow_center_hold_s = 0.15
            self.visual_follow_min_stable_frames = 12
        try:
            self.visual_follow_auto_retry_s = max(
                0.25,
                float(os.environ.get("SWARM_VISUAL_FOLLOW_AUTO_RETRY_S", "1.0")),
            )
            self.visual_follow_handover_s = max(
                0.5,
                min(
                    5.0,
                    float(os.environ.get("SWARM_VISUAL_FOLLOW_HANDOVER_S", "2.0")),
                ),
            )
        except ValueError:
            self.visual_follow_auto_retry_s = 1.0
            self.visual_follow_handover_s = 2.0
        self.body_attitude_enabled = os.environ.get(
            "SWARM_TRACKING_BODY_ATTITUDE_ENABLED",
            "true",
        ).strip().lower() in {"1", "true", "yes", "on"}
        self.body_yaw_invert = os.environ.get(
            "SWARM_TRACKING_BODY_YAW_INVERT",
            "false",
        ).strip().lower() in {"1", "true", "yes", "on"}
        try:
            camera_width = max(
                1,
                int(os.environ.get("SWARM_CAMERA_WIDTH", "640")),
            )
            camera_height = max(
                1,
                int(os.environ.get("SWARM_CAMERA_HEIGHT", "360")),
            )
            camera_horizontal_fov_rad = float(
                os.environ.get("SWARM_CAMERA_HORIZONTAL_FOV_RAD", "2.0")
            )
            derived_camera_focal_px = camera_width / (
                2.0 * math.tan(camera_horizontal_fov_rad / 2.0)
            )
            gimbal_fx = max(
                1.0,
                float(
                    os.environ.get(
                        "SWARM_TRACKING_GIMBAL_FX",
                        str(derived_camera_focal_px),
                    )
                ),
            )
            gimbal_fy = max(
                1.0,
                float(
                    os.environ.get(
                        "SWARM_TRACKING_GIMBAL_FY",
                        str(derived_camera_focal_px),
                    )
                ),
            )
            gimbal_kp = float(
                os.environ.get("SWARM_TRACKING_GIMBAL_KP", "6.0")
            )
            gimbal_ki = float(
                os.environ.get("SWARM_TRACKING_GIMBAL_KI", "0.10")
            )
            gimbal_kd = float(
                os.environ.get("SWARM_TRACKING_GIMBAL_KD", "0.12")
            )
            gimbal_ema_alpha = float(
                os.environ.get("SWARM_TRACKING_GIMBAL_EMA_ALPHA", "0.85")
            )
            gimbal_deadband_deg = float(
                os.environ.get("SWARM_TRACKING_GIMBAL_DEADBAND_DEG", "0.25")
            )
            gimbal_omega_max_deg = float(
                os.environ.get("SWARM_TRACKING_GIMBAL_OMEGA_MAX_DEG", "180")
            )
            gimbal_slew_max_deg = float(
                os.environ.get("SWARM_TRACKING_GIMBAL_SLEW_MAX_DEG", "900")
            )
            gimbal_publish_roll_rate_deg_s = float(
                os.environ.get(
                    "SWARM_TRACKING_GIMBAL_ROLL_RATE_DEG_S", "90"
                )
            )
            gimbal_publish_pitch_rate_deg_s = float(
                os.environ.get(
                    "SWARM_TRACKING_GIMBAL_MAX_PITCH_RATE_DEG_S", "180"
                )
            )
            gimbal_publish_yaw_rate_deg_s = float(
                os.environ.get(
                    "SWARM_TRACKING_GIMBAL_MAX_YAW_RATE_DEG_S", "180"
                )
            )
            gimbal_publish_accel_deg_s2 = float(
                os.environ.get(
                    "SWARM_TRACKING_GIMBAL_MAX_ACCEL_DEG_S2", "900"
                )
            )
            gimbal_publish_brake_accel_deg_s2 = float(
                os.environ.get(
                    "SWARM_TRACKING_GIMBAL_MAX_BRAKE_ACCEL_DEG_S2",
                    "1400",
                )
            )
            body_yaw_enter_deg = float(
                os.environ.get("SWARM_TRACKING_BODY_YAW_ENTER_DEG", "2.5")
            )
            body_yaw_exit_deg = float(
                os.environ.get("SWARM_TRACKING_BODY_YAW_EXIT_DEG", "1.0")
            )
            body_yaw_kp = float(
                os.environ.get("SWARM_TRACKING_BODY_YAW_KP", "0.018")
            )
            body_yaw_max = float(
                os.environ.get("SWARM_TRACKING_BODY_YAW_MAX", "0.55")
            )
            body_yaw_slew = float(
                os.environ.get("SWARM_TRACKING_BODY_YAW_SLEW", "1.4")
            )
            body_yaw_ema_alpha = float(
                os.environ.get("SWARM_TRACKING_BODY_YAW_EMA_ALPHA", "0.22")
            )
            body_yaw_min_score = float(
                os.environ.get("SWARM_TRACKING_BODY_YAW_MIN_SCORE", "0.35")
            )
            follow_kp = float(
                os.environ.get("SWARM_TRACKING_FOLLOW_KP", "0.12")
            )
            follow_max = float(
                os.environ.get("SWARM_TRACKING_FOLLOW_MAX", "0.30")
            )
            follow_slew = float(
                os.environ.get("SWARM_TRACKING_FOLLOW_SLEW", "0.15")
            )
            follow_deadband_m = float(
                os.environ.get("SWARM_TRACKING_FOLLOW_DEADBAND_M", "1.0")
            )
            follow_min_distance_m = float(
                os.environ.get("SWARM_TRACKING_FOLLOW_MIN_DISTANCE_M", "8.0")
            )
            follow_max_distance_m = float(
                os.environ.get("SWARM_TRACKING_FOLLOW_MAX_DISTANCE_M", "12.0")
            )
            follow_brake_m_s2 = float(
                os.environ.get("SWARM_TRACKING_FOLLOW_BRAKE_M_S2", "1.5")
            )
            follow_align_yaw_deg = float(
                os.environ.get("SWARM_TRACKING_FOLLOW_ALIGN_YAW_DEG", "5")
            )
            follow_radial_velocity_gain = float(
                os.environ.get(
                    "SWARM_TRACKING_RADIAL_VELOCITY_GAIN",
                    "1.0",
                )
            )
            follow_radial_velocity_alpha = float(
                os.environ.get(
                    "SWARM_TRACKING_RADIAL_VELOCITY_ALPHA",
                    "0.30",
                )
            )
            follow_ema_alpha = float(
                os.environ.get("SWARM_TRACKING_FOLLOW_EMA_ALPHA", "0.35")
            )
            follow_min_score = float(
                os.environ.get("SWARM_TRACKING_FOLLOW_MIN_SCORE", "0.70")
            )
            apparent_size_kp = float(
                os.environ.get("SWARM_APPARENT_SIZE_FOLLOW_KP", "1.15")
            )
            apparent_size_kd = float(
                os.environ.get("SWARM_APPARENT_SIZE_FOLLOW_KD", "0.12")
            )
            apparent_size_boost_max = float(
                os.environ.get("SWARM_APPARENT_SIZE_FOLLOW_BOOST_MAX", "0.20")
            )
            apparent_size_rate_deadband = float(
                os.environ.get("SWARM_APPARENT_SIZE_RATE_DEADBAND", "0.03")
            )
            apparent_size_deadband = float(
                os.environ.get("SWARM_APPARENT_SIZE_FOLLOW_DEADBAND", "0.005")
            )
            apparent_size_reverse_deadband = float(
                os.environ.get(
                    "SWARM_APPARENT_SIZE_REVERSE_DEADBAND",
                    "0.025",
                )
            )
            apparent_size_reference_warmup_s = float(
                os.environ.get(
                    "SWARM_APPARENT_SIZE_REFERENCE_WARMUP_S",
                    "1.2",
                )
            )
            apparent_size_slowband = float(
                os.environ.get("SWARM_APPARENT_SIZE_SLOWBAND", "0.20")
            )
            apparent_size_hold_ratio_tolerance = float(
                os.environ.get(
                    "SWARM_APPARENT_SIZE_HOLD_RATIO_TOLERANCE",
                    "0.12",
                )
            )
            bbox_forward_stop_ratio = float(
                os.environ.get("SWARM_BBOX_FOLLOW_FORWARD_STOP_RATIO", "0.97")
            )
            bbox_reverse_start_ratio = float(
                os.environ.get("SWARM_BBOX_FOLLOW_REVERSE_START_RATIO", "1.10")
            )
            object_scale_blend = float(
                os.environ.get("SWARM_OBJECT_SCALE_BLEND", "0.25")
            )
            vertical_recenter_kp = float(
                os.environ.get("SWARM_TRACKING_VERTICAL_RECENTER_KP", "0.035")
            )
            vertical_recenter_deadband_deg = float(
                os.environ.get(
                    "SWARM_TRACKING_VERTICAL_RECENTER_DEADBAND_DEG",
                    "2.0",
                )
            )
            vertical_recenter_max_down_m_s = float(
                os.environ.get(
                    "SWARM_TRACKING_VERTICAL_RECENTER_MAX_DOWN_M_S",
                    "0.45",
                )
            )
            vertical_recenter_slew_m_s2 = float(
                os.environ.get(
                    "SWARM_TRACKING_VERTICAL_RECENTER_SLEW_M_S2",
                    "0.70",
                )
            )
            vertical_recenter_invert = os.environ.get(
                "SWARM_TRACKING_VERTICAL_RECENTER_INVERT",
                "false",
            ).strip().lower() in {"1", "true", "yes", "on"}
            apparent_size_yaw_kp = float(
                os.environ.get("SWARM_APPARENT_SIZE_YAW_KP", "4.0")
            )
            apparent_size_yaw_deadband_deg = float(
                os.environ.get("SWARM_APPARENT_SIZE_YAW_DEADBAND_DEG", "0.5")
            )
            apparent_size_yaw_invert = os.environ.get(
                "SWARM_APPARENT_SIZE_YAW_INVERT",
                "false",
            ).strip().lower() in {"1", "true", "yes", "on"}
            offboard_max_forward_m_s = float(
                os.environ.get(
                    "SWARM_TRACKING_OFFBOARD_MAX_FORWARD_M_S",
                    "3.0",
                )
            )
            offboard_forward_sign = float(
                os.environ.get(
                    "SWARM_TRACKING_OFFBOARD_FORWARD_SIGN",
                    "1",
                )
            )
            offboard_max_yaw_rate_deg_s = float(
                os.environ.get(
                    "SWARM_TRACKING_OFFBOARD_MAX_YAW_RATE_DEG_S",
                    "35",
                )
            )
            offboard_yaw_slew_deg_s2 = float(
                os.environ.get(
                    "SWARM_TRACKING_OFFBOARD_YAW_SLEW_DEG_S2",
                    "120",
                )
            )
            bbox_size_alpha = float(
                os.environ.get("SWARM_TRACKING_BBOX_SIZE_ALPHA", "0.20")
            )
            bbox_max_size_alpha = float(
                os.environ.get(
                    "SWARM_TRACKING_BBOX_MAX_SIZE_ALPHA", "0.48"
                )
            )
            bbox_center_alpha = float(
                os.environ.get(
                    "SWARM_TRACKING_BBOX_CENTER_ALPHA", "0.55"
                )
            )
        except ValueError:
            camera_width = 640
            camera_height = 360
            camera_horizontal_fov_rad = 2.0
            gimbal_fx = 205.5
            gimbal_fy = 205.5
            gimbal_kp = 6.0
            gimbal_ki = 0.10
            gimbal_kd = 0.12
            gimbal_ema_alpha = 0.85
            gimbal_deadband_deg = 0.25
            gimbal_omega_max_deg = 60.0
            gimbal_slew_max_deg = 360.0
            gimbal_publish_roll_rate_deg_s = 90.0
            gimbal_publish_pitch_rate_deg_s = 45.0
            gimbal_publish_yaw_rate_deg_s = 60.0
            gimbal_publish_accel_deg_s2 = 360.0
            gimbal_publish_brake_accel_deg_s2 = 720.0
            body_yaw_enter_deg = 2.5
            body_yaw_exit_deg = 1.0
            body_yaw_kp = 0.018
            body_yaw_max = 0.55
            body_yaw_slew = 1.4
            body_yaw_ema_alpha = 0.22
            body_yaw_min_score = 0.35
            follow_kp = 0.12
            follow_max = 0.30
            follow_slew = 0.15
            follow_deadband_m = 0.40
            follow_min_distance_m = 8.0
            follow_max_distance_m = 12.0
            follow_brake_m_s2 = 1.5
            follow_align_yaw_deg = 5.0
            follow_radial_velocity_gain = 1.0
            follow_radial_velocity_alpha = 0.30
            follow_ema_alpha = 0.35
            follow_min_score = 0.70
            apparent_size_kp = 1.15
            apparent_size_kd = 0.12
            apparent_size_boost_max = 0.20
            apparent_size_rate_deadband = 0.03
            apparent_size_deadband = 0.005
            apparent_size_reverse_deadband = 0.025
            apparent_size_reference_warmup_s = 1.2
            apparent_size_slowband = 0.20
            apparent_size_hold_ratio_tolerance = 0.12
            bbox_forward_stop_ratio = 0.97
            bbox_reverse_start_ratio = 1.10
            object_scale_blend = 0.25
            vertical_recenter_kp = 0.035
            vertical_recenter_deadband_deg = 2.0
            vertical_recenter_max_down_m_s = 0.45
            vertical_recenter_slew_m_s2 = 0.70
            vertical_recenter_invert = False
            apparent_size_yaw_kp = 4.0
            apparent_size_yaw_deadband_deg = 0.5
            apparent_size_yaw_invert = False
            offboard_max_forward_m_s = 3.0
            offboard_forward_sign = 1.0
            offboard_max_yaw_rate_deg_s = 35.0
            offboard_yaw_slew_deg_s2 = 120.0
            bbox_size_alpha = 0.20
            bbox_max_size_alpha = 0.48
            bbox_center_alpha = 0.55

        self.body_yaw_exit_deg = max(0.2, min(10.0, body_yaw_exit_deg))
        self.body_yaw_enter_deg = max(
            self.body_yaw_exit_deg + 0.2,
            min(30.0, body_yaw_enter_deg),
        )
        self.body_yaw_kp = max(0.001, min(0.1, body_yaw_kp))
        self.body_yaw_max = max(0.05, min(0.55, body_yaw_max))
        self.body_yaw_slew = max(0.1, min(5.0, body_yaw_slew))
        self.body_yaw_ema_alpha = max(
            0.05,
            min(1.0, body_yaw_ema_alpha),
        )
        self.body_yaw_min_score = max(0.0, min(1.0, body_yaw_min_score))
        if self.apparent_size_follow_enabled:
            try:
                apparent_size_slew = float(
                    os.environ.get("SWARM_APPARENT_SIZE_FOLLOW_SLEW", "1.40")
                )
                apparent_size_ema = float(
                    os.environ.get("SWARM_APPARENT_SIZE_FOLLOW_EMA_ALPHA", "0.60")
                )
            except ValueError:
                apparent_size_slew = 1.40
                apparent_size_ema = 0.60
            follow_slew = max(follow_slew, apparent_size_slew)
            follow_ema_alpha = max(follow_ema_alpha, apparent_size_ema)
            follow_max = max(follow_max, 0.35)
        self.follow_kp = max(0.01, min(0.5, follow_kp))
        self.follow_max = max(0.05, min(0.35, follow_max))
        self.follow_slew = max(0.1, min(3.0, follow_slew))
        self.follow_deadband_m = max(0.1, min(3.0, follow_deadband_m))
        self.follow_min_distance_m = max(
            1.0,
            min(19.0, follow_min_distance_m),
        )
        self.follow_max_distance_m = max(
            self.follow_min_distance_m + 0.5,
            min(20.0, follow_max_distance_m),
        )
        self.follow_brake_m_s2 = max(
            0.1,
            min(5.0, follow_brake_m_s2),
        )
        self.follow_align_yaw_deg = max(
            5.0,
            min(60.0, follow_align_yaw_deg),
        )
        self.follow_ema_alpha = max(0.05, min(1.0, follow_ema_alpha))
        self.follow_radial_velocity_gain = max(
            0.0,
            min(2.0, follow_radial_velocity_gain),
        )
        self.follow_radial_velocity_alpha = max(
            0.05,
            min(1.0, follow_radial_velocity_alpha),
        )
        self.follow_min_score = max(0.0, min(1.0, follow_min_score))
        self.apparent_size_kp = max(0.01, min(2.0, apparent_size_kp))
        self.apparent_size_kd = max(0.0, min(0.5, apparent_size_kd))
        self.apparent_size_boost_max = max(
            0.0,
            min(0.35, apparent_size_boost_max),
        )
        self.apparent_size_rate_deadband = max(
            0.0,
            min(1.0, apparent_size_rate_deadband),
        )
        self.apparent_size_deadband = max(
            0.001,
            min(0.50, apparent_size_deadband),
        )
        self.apparent_size_reverse_deadband = max(
            self.apparent_size_deadband,
            min(0.50, apparent_size_reverse_deadband),
        )
        self.apparent_size_reference_warmup_s = max(
            0.0,
            min(5.0, apparent_size_reference_warmup_s),
        )
        self.apparent_size_slowband = max(
            self.apparent_size_reverse_deadband,
            min(1.0, apparent_size_slowband),
        )
        self.apparent_size_hold_ratio_tolerance = max(
            0.0,
            min(0.50, apparent_size_hold_ratio_tolerance),
        )
        self.bbox_forward_stop_ratio = max(
            0.50,
            min(1.0, bbox_forward_stop_ratio),
        )
        self.bbox_reverse_start_ratio = max(
            self.bbox_forward_stop_ratio,
            min(2.0, bbox_reverse_start_ratio),
        )
        self.object_scale_blend = max(0.0, min(1.0, object_scale_blend))
        self.vertical_recenter_kp = max(0.0, min(0.20, vertical_recenter_kp))
        self.vertical_recenter_deadband_deg = max(
            0.0,
            min(20.0, vertical_recenter_deadband_deg),
        )
        self.vertical_recenter_max_down_m_s = max(
            0.0,
            min(2.0, vertical_recenter_max_down_m_s),
        )
        self.vertical_recenter_slew_m_s2 = max(
            0.05,
            min(5.0, vertical_recenter_slew_m_s2),
        )
        self.vertical_recenter_invert = vertical_recenter_invert
        self.apparent_size_yaw_kp = max(
            0.1,
            min(8.0, apparent_size_yaw_kp),
        )
        self.camera_width = camera_width
        self.camera_height = camera_height
        self.camera_horizontal_fov_rad = camera_horizontal_fov_rad
        self.camera_fx = gimbal_fx
        self.camera_fy = gimbal_fy
        self.apparent_size_yaw_deadband_deg = max(
            0.5,
            min(15.0, apparent_size_yaw_deadband_deg),
        )
        self.apparent_size_yaw_invert = apparent_size_yaw_invert
        self.offboard_max_forward_m_s = max(
            0.1,
            min(3.0, offboard_max_forward_m_s),
        )
        self.offboard_forward_sign = (
            -1.0 if offboard_forward_sign < 0.0 else 1.0
        )
        self.offboard_max_yaw_rate_deg_s = max(
            5.0,
            min(120.0, offboard_max_yaw_rate_deg_s),
        )
        self.offboard_yaw_slew_deg_s2 = max(
            10.0,
            min(720.0, offboard_yaw_slew_deg_s2),
        )

        self.gimbal_controller = (
            GimbalService(
                replace(
                    PACKAGE_GIMBAL,
                    enabled=self.gimbal_enabled,
                    fx=gimbal_fx,
                    fy=gimbal_fy,
                    kp=max(0.1, min(20.0, gimbal_kp)),
                    ki=max(0.0, min(5.0, gimbal_ki)),
                    kd=max(0.0, min(2.0, gimbal_kd)),
                    ema_alpha=max(0.05, min(1.0, gimbal_ema_alpha)),
                    deadband_deg=max(
                        0.1,
                        min(5.0, gimbal_deadband_deg),
                    ),
                    omega_max_deg=max(
                        30.0,
                        min(360.0, gimbal_omega_max_deg),
                    ),
                    slew_max_deg=max(
                        60.0,
                        min(1200.0, gimbal_slew_max_deg),
                    ),
                )
            )
            if GimbalService is not None and PACKAGE_GIMBAL is not None
            else None
        )
        self.camera_reference_width = camera_width
        self.camera_reference_height = camera_height
        try:
            self.gimbal_prediction_latency_s = max(
                0.0,
                min(
                    0.25,
                    float(
                        os.environ.get(
                            "SWARM_TRACKING_GIMBAL_PREDICTION_LATENCY_S",
                            "0.08",
                        )
                    ),
                ),
            )
            self.gimbal_max_prediction_horizon_s = max(
                0.0,
                min(
                    0.25,
                    float(
                        os.environ.get(
                            "SWARM_TRACKING_GIMBAL_MAX_PREDICTION_HORIZON_S",
                            "0.12",
                        )
                    ),
                ),
            )
        except ValueError:
            self.gimbal_prediction_latency_s = 0.08
            self.gimbal_max_prediction_horizon_s = 0.12
        try:
            self.gimbal_command_rate_hz = max(
                30.0,
                min(
                    50.0,
                    float(
                        os.environ.get(
                            "SWARM_TRACKING_GIMBAL_COMMAND_RATE_HZ",
                            "50",
                        )
                    ),
                ),
            )
            self.gimbal_target_stale_s = max(
                0.08,
                min(
                    0.50,
                    float(
                        os.environ.get(
                            "SWARM_TRACKING_GIMBAL_TARGET_STALE_S",
                            "0.20",
                        )
                    ),
                ),
            )
        except ValueError:
            self.gimbal_command_rate_hz = 50.0
            self.gimbal_target_stale_s = 0.20
        try:
            self.selection_pose_max_age_ms = max(
                20.0,
                min(
                    500.0,
                    float(
                        os.environ.get(
                            "SWARM_FOLLOW_SELECTION_POSE_MAX_AGE_MS",
                            "200",
                        )
                    ),
                ),
            )
            self.selection_gimbal_max_age_ms = max(
                20.0,
                min(
                    500.0,
                    float(
                        os.environ.get(
                            "SWARM_FOLLOW_SELECTION_GIMBAL_MAX_AGE_MS",
                            "120",
                        )
                    ),
                ),
            )
            self.follow_min_altitude_m = max(
                1.0,
                min(
                    120.0,
                    float(
                        os.environ.get(
                            "SWARM_FOLLOW_MIN_ALTITUDE_M",
                            "8.0",
                        )
                    ),
                ),
            )
        except ValueError:
            self.selection_pose_max_age_ms = 200.0
            self.selection_gimbal_max_age_ms = 120.0
            self.follow_min_altitude_m = 8.0
        self.bbox_motion_safety_estimator = BBoxMotionSafetyEstimator()
        self.visual_bbox_filter = VisualBBoxFilter(
            center_alpha=max(0.05, min(1.0, bbox_center_alpha)),
            size_alpha=max(0.01, min(1.0, bbox_size_alpha)),
            max_size_alpha=max(
                0.01,
                min(1.0, bbox_max_size_alpha),
            ),
        )
        self.visual_ray_projector = CameraRayProjector(
            gimbal_fx,
            gimbal_fy,
        )
        self.metric_target_fusion = MetricTargetFusion(
            self.visual_ray_projector,
        )
        self.bearing_target_estimator = BearingTargetEstimator()
        self.provisional_target_filter = TargetStateFilter(
            alpha=0.16,
            beta=0.012,
            max_jump_m=4.0,
            max_speed_m_s=3.0,
            max_position_rate_m_s=2.0,
            position_deadband_m=0.10,
            velocity_deadband_m_s=0.25,
            stale_timeout_s=0.50,
        )
        try:
            self.follow_relative_angle_deg = max(
                -180.0,
                min(
                    180.0,
                    float(
                        os.environ.get(
                            "SWARM_FOLLOW_RELATIVE_ANGLE_DEG",
                            "180.0",
                        )
                    ),
                ),
            )
        except ValueError:
            self.follow_relative_angle_deg = 180.0
        try:
            self.bootstrap_maximum_baseline_m = max(
                self.bearing_target_estimator.config.bootstrap_lateral_distance_m,
                min(
                    3.0,
                    float(
                        os.environ.get(
                            "SWARM_RGB_BOOTSTRAP_MAX_BASELINE_M",
                            "1.25",
                        )
                    ),
                ),
            )
        except ValueError:
            self.bootstrap_maximum_baseline_m = 1.25
        self.body_yaw_recenter_controller = BodyYawRecenterController()

        self.lock = threading.RLock()
        # OpenCV/hybrid tracker updates can take tens of milliseconds.  Keep
        # that work serialized without holding the state/API lock.
        self.tracker_lock = threading.Lock()
        self.output_condition = threading.Condition(self.lock)
        self.shutdown_event = threading.Event()
        self.worker: threading.Thread | None = None
        self.pointing_worker: threading.Thread | None = None

        self.active = False
        self.drone_id = ""
        self.state = "inactive"
        self.error = TRACKING_IMPORT_ERROR
        self.tracker: Any = None
        self.inference: Any = None
        self.tracker_epoch = 0
        self.session_id = 0
        self.latest_raw_frame: Any = None
        self.latest_result: Any = None
        self.selected_bbox: list[float] | None = None
        self.output_frame: bytes | None = None
        self.output_version = 0
        self.last_output_monotonic: float | None = None
        self.fps = 0.0
        self.fps_window_started: float | None = None
        self.fps_window_frames = 0
        self.tracking_ms = 0.0
        self.encode_ms = 0.0
        self.controller_ms = 0.0
        self.main_loop_ms = 0.0
        self.frame_queue_age_ms = 0.0
        self.source_frame_index = 0
        self.current_source_timestamp_s = 0.0
        self.current_source_sim_timestamp_s: float | None = None
        self.source_frames_dropped = 0
        self.follow_workflow = FollowWorkflowMachine()
        self.selection_snapshot: SelectionSnapshot | None = None
        self.pointing_guard_reason = "tracking_inactive"
        self.bootstrap_direction = os.environ.get(
            "SWARM_RGB_BOOTSTRAP_DIRECTION",
            "right",
        ).strip().lower()
        if self.bootstrap_direction not in {"left", "right"}:
            self.bootstrap_direction = "right"
        self.bootstrap_corridor_confirmed = False
        self.bootstrap_started_monotonic: float | None = None
        self.bootstrap_start_position_ned: tuple[float, float, float] | None = None
        self.bootstrap_lateral_unit_ned: tuple[float, float] | None = None
        self.bootstrap_command_count = 0
        self.bootstrap_motion_error = ""
        self.bootstrap_command_velocity_ned = (0.0, 0.0)
        self.bootstrap_travel_m = 0.0
        self.initial_safe_distance_horizontal_m: float | None = None
        self.initial_safe_distance_slant_m: float | None = None
        self.initial_safe_distance_lock_timestamp_s: float | None = None
        self.initial_safe_distance_lock_count = 0
        self.initial_safe_distance_source = "none"
        self.initial_follow_angle_deg: float | None = None
        self.initial_follow_angle_source = "none"
        self.initial_vertical_offset_m: float | None = None
        self.initial_bbox_scale_px: float | None = None
        self.initial_range_std_m: float | None = None
        self.current_estimated_horizontal_distance_m: float | None = None
        self.provisional_target_estimate = TargetEstimate(
            timestamp_s=0.0,
            state="PROVISIONAL",
            valid=False,
            reason="not_initialized",
        )
        self.provisional_target_initial_slant_range_m: float | None = None
        self.provisional_target_update_count = 0
        self.provisional_target_accepted_count = 0
        self.provisional_target_source = "none"
        self.provisional_target_range_m: float | None = None
        self.provisional_target_metric_handover_started_s: float | None = None
        self.provisional_target_metric_handover_origin_ned: (
            tuple[float, float, float] | None
        ) = None
        self.provisional_target_metric_blend = 0.0
        self.provisional_target_metric_confirmed = False
        try:
            self.follow_prestream_min_messages = max(
                5,
                min(
                    30,
                    int(
                        os.environ.get(
                            "SWARM_FOLLOW_PRESTREAM_MIN_MESSAGES",
                            "5",
                        )
                    ),
                ),
            )
            self.follow_mode_confirmation_timeout_s = max(
                2.0,
                min(
                    15.0,
                    float(
                        os.environ.get(
                            "SWARM_FOLLOW_MODE_CONFIRM_TIMEOUT_S",
                            "6.0",
                        )
                    ),
                ),
            )
            self.visual_follow_dropout_grace_s = max(
                0.50,
                min(
                    3.0,
                    float(
                        os.environ.get(
                            "SWARM_VISUAL_FOLLOW_DROPOUT_GRACE_S",
                            "1.50",
                        )
                    ),
                ),
            )
        except ValueError:
            self.follow_prestream_min_messages = 5
            self.follow_mode_confirmation_timeout_s = 6.0
            self.visual_follow_dropout_grace_s = 1.50
        self.follow_prestream_count = 0
        self.follow_mode_request_attempts = 0
        self.follow_mode_requested_monotonic: float | None = None
        self.visual_target_stream_active = False
        self.follow_estimate_invalid_since_monotonic: float | None = None
        self.px4_nav_state = -1
        self.visual_follow_bridge_status: dict[str, Any] = {}
        self._timing_samples: dict[str, deque[float]] = {
            name: deque(maxlen=240)
            for name in (
                "frame_queue_age_ms",
                "tracking_ms",
                "controller_ms",
                "metric_fusion_ms",
                "ground_truth_provider_ms",
                "bearing_estimator_ms",
                "overlay_prepare_ms",
                "encode_ms",
                "main_loop_ms",
                "status_lock_wait_ms",
            )
        }
        self.gimbal_last_update_monotonic: float | None = None
        self.gimbal_last_command_monotonic: float | None = None
        self.gimbal_command_timestamps: deque[float] = deque(maxlen=200)
        self.gimbal_angles_deg = {
            "roll": 0.0,
            "pitch": 0.0,
            "yaw": 0.0,
        }
        self.gimbal_commanded_angles_deg = dict(self.gimbal_angles_deg)
        self.gimbal_maximum_command_lead_deg = 0.0
        self.gimbal_feedback_available = False
        self.gimbal_feedback_age_ms: float | None = None
        self.gimbal_measured_center_px: tuple[float, float] | None = None
        self.gimbal_predicted_center_px: tuple[float, float] | None = None
        self.gimbal_prediction_horizon_s = 0.0
        self.gimbal_rates_deg_s = {
            "roll": 0.0,
            "pitch": 0.0,
            "yaw": 0.0,
        }
        self.gimbal_raw_requested_rates_deg_s = dict(
            self.gimbal_rates_deg_s
        )
        self.gimbal_limited_rates_deg_s = dict(self.gimbal_rates_deg_s)
        self.gimbal_maximum_rates_deg_s = {
            "roll": max(
                1.0,
                min(
                    180.0,
                    gimbal_publish_roll_rate_deg_s,
                    gimbal_publish_pitch_rate_deg_s,
                ),
            ),
            "pitch": max(
                1.0,
                min(180.0, gimbal_publish_pitch_rate_deg_s),
            ),
            "yaw": max(
                1.0,
                min(180.0, gimbal_publish_yaw_rate_deg_s),
            ),
        }
        self.gimbal_maximum_acceleration_deg_s2 = max(
            1.0,
            min(2000.0, gimbal_publish_accel_deg_s2),
        )
        self.gimbal_maximum_braking_acceleration_deg_s2 = max(
            self.gimbal_maximum_acceleration_deg_s2,
            min(2000.0, gimbal_publish_brake_accel_deg_s2),
        )
        self.gimbal_error = ""
        self.gimbal_yaw_limit_deg = (
            self.configured_gimbal_yaw_limit_deg
        )
        self.gimbal_yaw_saturated_outward = False
        self.gimbal_yaw_recenter_latched = False
        self.gimbal_yaw_requested_rate_deg_s = 0.0
        self.body_yaw_target_error_deg = 0.0
        self.body_yaw_image_error_deg = 0.0
        self.body_pitch_image_error_deg = 0.0
        self.body_yaw_active = False
        self.body_yaw_latched = False
        self.body_yaw_filtered_deg = 0.0
        self.body_yaw_command_normalized = 0.0
        self.body_yaw_last_publish_monotonic: float | None = None
        self.body_yaw_error = ""
        self.follow_distance_target_m = 10.0
        self.follow_distance_m: float | None = None
        self.motion_command_created_timestamp_s: float | None = None
        self.target_estimate = TargetEstimate(
            timestamp_s=0.0,
            state="IDLE",
            valid=False,
            reason="idle",
        )
        self.multiview_target_estimate = TargetEstimate(
            timestamp_s=0.0,
            state="MULTIVIEW",
            valid=False,
            reason="idle",
        )
        self.target_estimator_error = ""
        self.target_tracking_gate_reason = "idle"
        self.target_pose_age_ms: float | None = None
        self.target_gimbal_age_ms: float | None = None
        self.target_pose_interpolation_span_ms: float | None = None
        self.target_gimbal_interpolation_span_ms: float | None = None
        self.target_pose_sync_diagnostics: dict[str, Any] = {}
        self.body_yaw_recenter_enter_elapsed_s = 0.0
        self.body_yaw_recenter_filtered_deg = 0.0
        self.follow_lidar_distance_m: float | None = None
        self.follow_distance_source = "none"
        self.follow_reference_area: float | None = None
        self.follow_reference_distance_m: float | None = None
        self.follow_reference_scale_px: float | None = None
        self.follow_reference_bbox_scale_px: float | None = None
        self.follow_filtered_scale_px: float | None = None
        self.follow_scale_ratio: float | None = None
        self.follow_bbox_scale_ratio: float | None = None
        self.follow_forward_blocked_by_bbox_guard = False
        self.follow_scale_error: float = 0.0
        self.follow_scale_error_rate: float = 0.0
        self.follow_object_scale_source = "none"
        self.follow_reference_scale_source = "none"
        self.follow_reference_warmup_until_monotonic: float | None = None
        self.follow_previous_scale_error: float | None = None
        self.follow_previous_scale_error_monotonic: float | None = None
        self.follow_previous_distance_m: float | None = None
        self.follow_previous_distance_monotonic: float | None = None
        self.follow_previous_depth_frame_index = 0
        self.follow_radial_velocity_m_s = 0.0
        self.follow_command_forward = 0.0
        self.follow_active = False
        self.follow_state = "idle"
        self.follow_error = ""
        self.follow_block_reason = "tracking_inactive"
        self.visual_follow_requested = False
        self.visual_follow_auto_start_pending = False
        self.visual_follow_ready = False
        self.visual_follow_active = False
        self.relative_visual_follow_state = "idle"
        self.relative_visual_follow_block_reason = ""
        self.visual_motion_gate_reason = "idle"
        self.visual_follow_state = "idle"
        self.visual_follow_error = ""
        self.visual_follow_last_stop_reason = ""
        self.visual_follow_last_stop_monotonic: float | None = None
        self.visual_follow_boundary_invalid_since_monotonic: float | None = None
        self.visual_follow_last_publish_monotonic: float | None = None
        self.visual_follow_last_auto_attempt_monotonic: float | None = None
        self.visual_follow_ready_since_monotonic: float | None = None
        self.visual_follow_center_since_monotonic: float | None = None
        self.visual_follow_handover_started_monotonic: float | None = None
        self.visual_follow_handover_blend = 0.0
        self.visual_follow_center_error_deg = 0.0
        self.visual_range_raw_m: float | None = None
        self.visual_range_filtered_m: float | None = None
        self.visual_bbox_scale_px: float | None = None
        self.visual_bbox_filter_state = "idle"
        self.visual_bbox_stable_frames = 0
        self.visual_bbox_rejected_frames = 0
        self.visual_bbox_center_residual_px: float | None = None
        self.visual_bbox_filtered_bbox: list[float] | None = None
        self.visual_bbox_filtered_area_px2: float | None = None
        self.visual_bbox_size_alpha_used = 0.0
        self.visual_range_quality = 0.0
        self.visual_ttc_s: float | None = None
        self.visual_target_ned: tuple[float, float, float] | None = None
        self.visual_target_velocity_ned: tuple[float, float, float] | None = None
        self.visual_target_global: tuple[float, float, float] | None = None
        self.follow_last_publish_monotonic: float | None = None
        self.motion_active = False
        self.motion_state = "idle"
        self.motion_forward_velocity_m_s = 0.0
        self.motion_down_velocity_m_s = 0.0
        self.motion_yaw_rate_deg_s = 0.0
        self.motion_requested_yaw_rate_deg_s = 0.0
        self.motion_combined_yaw_error_deg = 0.0
        self.yaw_authority = "none"
        self.yaw_block_reason = "tracking_inactive"
        self.gimbal_recenter_state = "inactive"
        self.motion_hold_z_down_m: float | None = None
        self.motion_vertical_pitch_error_deg = 0.0
        self.motion_vertical_recenter_active = False
        self.motion_error = ""
        self.motion_last_publish_monotonic: float | None = None
        self.motion_invalid_since_monotonic: float | None = None
        self.body_attitude_active = False
        self.body_attitude_state = "disabled"
        self.body_attitude_rates_deg_s = {
            "roll": 0.0,
            "pitch": 0.0,
            "yaw": 0.0,
        }
        self.body_attitude_thrust = 0.0
        self.body_recenter_velocity_m_s = {
            "forward": 0.0,
            "right": 0.0,
            "down": 0.0,
        }
        self.body_recenter_acceleration_m_s2 = {
            "forward": 0.0,
            "right": 0.0,
        }
        self.body_desired_tilt_deg = {"roll": 0.0, "pitch": 0.0}
        self.body_altitude_error_m = 0.0
        self.body_altitude_guard_active = False
        self.body_hold_z_down_m: float | None = None
        self.body_attitude_error = ""
        self.gimbal_home_error_deg = {
            "roll": 0.0,
            "pitch": 0.0,
            "yaw": 0.0,
        }

    @property
    def available(self) -> bool:
        return not TRACKING_IMPORT_ERROR and self.backend in {
            "opencv",
            "hybrid",
            "onnx",
            "board",
        }

    def start(
        self,
        drone_id: str,
        desired_distance_m: float = 10.0,
    ) -> dict[str, Any]:
        if drone_id not in self.allowed_drones:
            raise ValueError("Invalid drone ID")

        if not self.available:
            raise RuntimeError(
                TRACKING_IMPORT_ERROR
                or f"Unsupported tracking backend: {self.backend}"
            )
        if not 3.0 <= float(desired_distance_m) <= 20.0:
            raise ValueError("Desired tracking distance must be from 3 to 20 m")

        tracker, inference = self._create_tracker()
        self.metric_target_fusion.start()

        with self.lock:
            previous_drone_id = self.drone_id
            if previous_drone_id:
                self._disable_body_yaw_locked(previous_drone_id)
            self._close_tracker_locked()
            self.tracker = tracker
            self.inference = inference
            self.active = True
            self.drone_id = drone_id
            self.follow_distance_target_m = float(desired_distance_m)
            self.state = "selecting"
            self.error = ""
            self.session_id += 1
            self.latest_raw_frame = None
            self.latest_result = None
            self.selected_bbox = None
            self.output_frame = None
            self.last_output_monotonic = None
            self.fps = 0.0
            self.fps_window_started = None
            self.fps_window_frames = 0
            self.tracking_ms = 0.0
            self.encode_ms = 0.0
            self.controller_ms = 0.0
            self.frame_queue_age_ms = 0.0
            self.source_frame_index = 0
            self.source_frames_dropped = 0
            self.selection_snapshot = None
            self.pointing_guard_reason = "awaiting_bbox"
            self.follow_workflow.begin_tracking(time.monotonic())
            for samples in self._timing_samples.values():
                samples.clear()
            self._reset_gimbal_locked()
            self.output_version += 1
            self.output_condition.notify_all()

            if self.worker is None or not self.worker.is_alive():
                self.shutdown_event.clear()
                self.worker = threading.Thread(
                    target=self._run,
                    name="dashboard-tracking",
                    daemon=True,
                )
                self.worker.start()
            if self.pointing_worker is None or not self.pointing_worker.is_alive():
                self.pointing_worker = threading.Thread(
                    target=self._run_pointing,
                    name="dashboard-target-pointing",
                    daemon=True,
                )
                self.pointing_worker.start()

        self.gazebo_bridge.set_tracking_drone(drone_id)
        reset_pending = getattr(
            self.gazebo_bridge, "reset_ground_truth_pending", None
        )
        if reset_pending is not None:
            reset_pending()
        return self.status()

    def stop(self) -> dict[str, Any]:
        with self.lock:
            stopped_drone_id = self.drone_id
            self.active = False
            self.state = "inactive"
            self.error = ""
            self.session_id += 1
            self.latest_raw_frame = None
            self.latest_result = None
            self.selected_bbox = None
            self.selection_snapshot = None
            self.pointing_guard_reason = "tracking_inactive"
            self.follow_workflow.transition(
                FollowWorkflowState.IDLE,
                "tracking_stopped",
                timestamp_s=time.monotonic(),
                force=True,
            )
            self.output_frame = None
            self.last_output_monotonic = None
            self.fps = 0.0
            self.fps_window_started = None
            self.fps_window_frames = 0
            self.tracking_ms = 0.0
            self.encode_ms = 0.0
            self.controller_ms = 0.0
            self.frame_queue_age_ms = 0.0
            self._reset_gimbal_locked(stopped_drone_id)
            self.drone_id = ""
            self.output_version += 1
            self._close_tracker_locked()
            self.output_condition.notify_all()

        self.gazebo_bridge.set_tracking_drone(None)
        reset_pending = getattr(
            self.gazebo_bridge, "reset_ground_truth_pending", None
        )
        if reset_pending is not None:
            reset_pending()
        return self.status()

    def shutdown(self) -> None:
        self.shutdown_event.set()
        self.stop()
        self.metric_target_fusion.shutdown()

        worker = self.worker
        if worker is not None:
            worker.join(timeout=2.0)
        pointing_worker = self.pointing_worker
        if pointing_worker is not None:
            pointing_worker.join(timeout=2.0)

    def _capture_selection_snapshot_locked(
        self,
        bbox: Any,
        frame_width: int,
        frame_height: int,
    ) -> SelectionSnapshot:
        timestamp_s = float(
            self.current_source_timestamp_s or time.monotonic()
        )
        bbox_tuple = (
            float(bbox.x),
            float(bbox.y),
            float(bbox.w),
            float(bbox.h),
        )
        bbox_scale_px = math.sqrt(max(0.0, float(bbox.w * bbox.h)))
        invalid_arguments = {
            "session_id": self.follow_workflow.session_id,
            "selection_timestamp_s": timestamp_s,
            "selection_frame_index": int(self.source_frame_index),
            "selection_vehicle_position_ned": None,
            "selection_camera_position_ned": None,
            "selection_vehicle_global_position": None,
            "selection_camera_quaternion_xyzw": None,
            "selection_bearing_ned": None,
            "selection_bbox": bbox_tuple,
            "selection_bbox_scale_px": bbox_scale_px,
            "selection_heading_rad": None,
            "selection_altitude_m": None,
            "selection_pose_age_ms": None,
            "selection_gimbal_age_ms": None,
            "valid": False,
            "reason": "selection_pose_stale",
        }
        if self.pose_provider is None:
            invalid_arguments["reason"] = "selection_pose_provider_unavailable"
            return SelectionSnapshot(**invalid_arguments)
        try:
            try:
                pose = self.pose_provider(self.drone_id, timestamp_s)
            except TypeError:
                pose = self.pose_provider(self.drone_id)
            if not pose.get("available"):
                invalid_arguments["reason"] = "selection_pose_stale"
                return SelectionSnapshot(**invalid_arguments)
            pose_age_ms = float(pose.get("telemetry_age_ms", math.inf))
            gimbal_age_ms = float(pose.get("camera_age_ms", math.inf))
            invalid_arguments["selection_pose_age_ms"] = pose_age_ms
            invalid_arguments["selection_gimbal_age_ms"] = gimbal_age_ms
            if pose_age_ms > self.selection_pose_max_age_ms:
                invalid_arguments["reason"] = "selection_pose_stale"
                return SelectionSnapshot(**invalid_arguments)
            if gimbal_age_ms > self.selection_gimbal_max_age_ms:
                invalid_arguments["reason"] = "selection_gimbal_pose_stale"
                return SelectionSnapshot(**invalid_arguments)
            vehicle_position = _finite_position(
                pose.get("vehicle_position_ned_m")
            )
            camera_position = _finite_position(
                pose.get("camera_position_ned_m")
            )
            global_position = pose.get("synchronized_global_position")
            camera_quaternion = pose.get("camera_quaternion_xyzw")
            if (
                vehicle_position is None
                or camera_position is None
                or not isinstance(global_position, (tuple, list))
                or len(global_position) != 3
                or not isinstance(camera_quaternion, (tuple, list))
                or len(camera_quaternion) != 4
            ):
                raise ValueError("synchronized selection pose is incomplete")
            global_tuple = tuple(float(value) for value in global_position)
            quaternion_tuple = tuple(float(value) for value in camera_quaternion)
            if not all(
                math.isfinite(value)
                for value in global_tuple + quaternion_tuple
            ):
                raise ValueError("selection pose is not finite")
            bearing = self.visual_ray_projector.pixel_to_ned_ray(
                float(bbox.cx),
                float(bbox.cy),
                frame_width,
                frame_height,
                quaternion_tuple,
            )
            heading_rad = float(pose["synchronized_heading_rad"])
            if not math.isfinite(heading_rad):
                raise ValueError("selection heading is not finite")
            altitude_m = max(0.0, -float(vehicle_position[2]))
            return SelectionSnapshot(
                session_id=self.follow_workflow.session_id,
                selection_timestamp_s=timestamp_s,
                selection_frame_index=int(self.source_frame_index),
                selection_vehicle_position_ned=vehicle_position,
                selection_camera_position_ned=camera_position,
                selection_vehicle_global_position=global_tuple,
                selection_camera_quaternion_xyzw=quaternion_tuple,
                selection_bearing_ned=bearing,
                selection_bbox=bbox_tuple,
                selection_bbox_scale_px=bbox_scale_px,
                selection_heading_rad=heading_rad,
                selection_altitude_m=altitude_m,
                selection_pose_age_ms=pose_age_ms,
                selection_gimbal_age_ms=gimbal_age_ms,
                valid=True,
                reason="ok",
            )
        except (KeyError, TypeError, ValueError):
            invalid_arguments["reason"] = "selection_pose_invalid"
            return SelectionSnapshot(**invalid_arguments)

    def set_bbox(
        self,
        x: float,
        y: float,
        width: float,
        height: float,
    ) -> dict[str, Any]:
        values = (x, y, width, height)
        if not all(0.0 <= value <= 1.0 for value in values):
            raise ValueError("Bounding box values must be normalized from 0 to 1")
        if width <= 0.0 or height <= 0.0:
            raise ValueError("Bounding box width and height must be positive")
        if x + width > 1.0 or y + height > 1.0:
            raise ValueError("Bounding box exceeds the tracking frame")

        with self.lock:
            if not self.active or self.tracker is None:
                raise RuntimeError("Tracking is not active")
            if self.latest_raw_frame is None:
                raise RuntimeError("No tracking frame is available yet")

            frame_height, frame_width = self.latest_raw_frame.shape[:2]
            bbox = BBox(
                x * frame_width,
                y * frame_height,
                width * frame_width,
                height * frame_height,
            )

            if bbox.w < 6.0 or bbox.h < 6.0:
                raise ValueError("Bounding box must be at least 6 pixels wide and high")

            with self.tracker_lock:
                self.tracker.init(self.latest_raw_frame.copy(), bbox)
            self.tracker_epoch += 1
            self._reset_gimbal_locked(preserve_metric_calibration=True)
            self.bbox_motion_safety_estimator.reset()
            self.bearing_target_estimator.reset()
            self.target_estimate = self.bearing_target_estimator.last_estimate
            self.follow_distance_m = None
            self.follow_lidar_distance_m = None
            self.follow_distance_source = (
                "apparent_size"
                if self.apparent_size_follow_enabled
                else "bearing_triangulation"
            )
            self.follow_reference_area = float(bbox.w * bbox.h)
            self.follow_reference_distance_m = None
            self.follow_reference_bbox_scale_px = math.sqrt(
                max(36.0, self.follow_reference_area)
            )
            object_scale_px, object_scale_source = object_pixel_scale_px(
                self.latest_raw_frame,
                bbox,
            )
            scale_normalizer = math.sqrt(
                self.camera_reference_width
                * self.camera_reference_height
                / max(1.0, float(frame_width * frame_height))
            )
            self.follow_reference_scale_px = max(
                6.0,
                object_scale_px * scale_normalizer,
            )
            self.follow_filtered_scale_px = self.follow_reference_scale_px
            self.follow_scale_ratio = 1.0
            self.follow_bbox_scale_ratio = 1.0
            self.follow_forward_blocked_by_bbox_guard = False
            self.follow_scale_error = 0.0
            self.follow_scale_error_rate = 0.0
            self.follow_object_scale_source = object_scale_source
            self.follow_reference_scale_source = object_scale_source
            now = time.monotonic()
            self.follow_reference_warmup_until_monotonic = (
                now + self.apparent_size_reference_warmup_s
            )
            self.follow_previous_scale_error = 0.0
            self.follow_previous_scale_error_monotonic = now
            self.follow_state = (
                "acquiring"
            )
            self.visual_follow_state = "acquiring"
            self.visual_follow_ready = False
            self.visual_follow_requested = False
            self.visual_follow_auto_start_pending = (
                self.visual_follow_auto_start
            )
            self.visual_follow_active = False
            self.relative_visual_follow_state = "reference_warmup"
            self.relative_visual_follow_block_reason = ""
            self.visual_motion_gate_reason = "stabilizing"
            self.visual_follow_boundary_invalid_since_monotonic = None
            self.bootstrap_corridor_confirmed = False
            self.bootstrap_started_monotonic = None
            self.bootstrap_start_position_ned = None
            self.bootstrap_lateral_unit_ned = None
            self.bootstrap_command_count = 0
            self.bootstrap_motion_error = ""
            self.bootstrap_command_velocity_ned = (0.0, 0.0)
            self.bootstrap_travel_m = 0.0
            self.initial_safe_distance_horizontal_m = None
            self.initial_safe_distance_slant_m = None
            self.initial_safe_distance_lock_timestamp_s = None
            self.initial_safe_distance_lock_count = 0
            self.initial_safe_distance_source = "none"
            self.initial_follow_angle_deg = None
            self.initial_follow_angle_source = "none"
            self.initial_vertical_offset_m = None
            self.initial_bbox_scale_px = None
            self.initial_range_std_m = None
            self.current_estimated_horizontal_distance_m = None
            self.follow_prestream_count = 0
            self.follow_mode_request_attempts = 0
            self.follow_mode_requested_monotonic = None
            self.visual_target_stream_active = False
            self.follow_estimate_invalid_since_monotonic = None
            self.px4_nav_state = -1
            self.visual_follow_bridge_status = {}
            self.follow_workflow.begin_target_session()
            self.selection_snapshot = self._capture_selection_snapshot_locked(
                bbox,
                frame_width,
                frame_height,
            )
            self.pointing_guard_reason = (
                "pointing_acquiring"
                if self.selection_snapshot.valid
                else self.selection_snapshot.reason
            )
            self.follow_workflow.select_target(
                snapshot_valid=self.selection_snapshot.valid,
                reason=(
                    "bbox_selected"
                    if self.selection_snapshot.valid
                    else self.selection_snapshot.reason
                ),
                timestamp_s=time.monotonic(),
            )
            self.latest_result = None
            self.selected_bbox = bbox.as_list()
            self.state = "tracking"
            self.error = ""

        reset_pending = getattr(
            self.gazebo_bridge, "reset_ground_truth_pending", None
        )
        if reset_pending is not None:
            reset_pending()
        return self.status()

    def _authorize_metric_follow_locked(
        self,
        reason: str,
        *,
        timestamp_s: float | None = None,
    ) -> None:
        """Authorize native Follow only for a converged measured target."""

        source_kind = str(self.provisional_target_source).strip().lower()
        if (
            not self.provisional_target_metric_confirmed
            or not source_kind
            or any(
                token in source_kind
                for token in (
                    "selection",
                    "provisional",
                    "apparent",
                    "bootstrap",
                )
            )
        ):
            raise RuntimeError(
                "Native Follow requires a converged measured target"
            )
        if not self._save_initial_safe_distance_locked(self.target_estimate):
            raise RuntimeError(
                "Native Follow requires a valid measured safe distance"
            )

        self.visual_follow_requested = True
        self.visual_follow_auto_start_pending = False
        self.visual_follow_state = "target_initialized"
        self.visual_follow_error = ""
        self.follow_workflow.transition(
            FollowWorkflowState.TARGET_INITIALIZED,
            reason,
            timestamp_s=timestamp_s,
        )

    def set_visual_follow_requested(
        self,
        enabled: bool,
        *,
        bootstrap_direction: str | None = None,
        corridor_confirmed: bool = False,
    ) -> dict[str, Any]:
        with self.lock:
            if enabled:
                if not self.visual_follow_feature_enabled:
                    raise RuntimeError("Visual Follow Target is disabled")
                if not self.active or self.tracker is None or not self.tracker.active:
                    raise RuntimeError("Select and track a target first")
                expected_state = (
                    FollowWorkflowState.TARGET_INITIALIZED
                    if self.metric_target_fusion.operational
                    else FollowWorkflowState.ACQUIRING_RANGE
                )
                if self.follow_workflow.state != expected_state:
                    raise RuntimeError(
                        "Follow workflow is not ready for authorization: "
                        f"{self.follow_workflow.reason}"
                    )
                if (
                    self.selection_snapshot is None
                    or not self.selection_snapshot.valid
                ):
                    raise RuntimeError("Selection pose is stale or invalid")
                if self.visual_follow_requested:
                    return self.status()
                if self.metric_target_fusion.operational:
                    if not self.target_estimate.valid:
                        raise RuntimeError(
                            "Visual target estimate is not ready: "
                            f"{self.target_estimate.reason}"
                        )
                    self._authorize_metric_follow_locked(
                        "user_authorized_visual_target",
                        timestamp_s=time.monotonic(),
                    )
                else:
                    if not getattr(
                        self,
                        "legacy_automation_enabled",
                        False,
                    ):
                        raise RuntimeError(
                            "Legacy RGB bootstrap automation is disabled"
                        )
                    direction = (
                        self.bootstrap_direction
                        if bootstrap_direction is None
                        else str(bootstrap_direction).strip().lower()
                    )
                    if direction not in {"left", "right"}:
                        raise ValueError(
                            "Bootstrap direction must be left or right"
                        )
                    if not corridor_confirmed:
                        raise RuntimeError(
                            "Confirm a clear lateral corridor; obstacle "
                            "avoidance is not available"
                        )
                    assert (
                        self.selection_snapshot.selection_bearing_ned
                        is not None
                    )
                    self.bootstrap_direction = direction
                    self.bootstrap_corridor_confirmed = True
                    self.bootstrap_lateral_unit_ned = (
                        lateral_bootstrap_unit_ned(
                            self.selection_snapshot.selection_bearing_ned,
                            direction,
                        )
                    )
                    self.bootstrap_started_monotonic = time.monotonic()
                    self.bootstrap_start_position_ned = (
                        self._bootstrap_start_position_locked()
                    )
                    self.bootstrap_command_count = 0
                    self.bootstrap_motion_error = ""
                    self.visual_follow_requested = True
                    self.visual_follow_state = "rgb_bootstrap"
                    self.visual_follow_error = ""
                    self.bearing_target_estimator.reset()
                    self.target_estimate = (
                        self.bearing_target_estimator.last_estimate
                    )
                    self.follow_workflow.transition(
                        FollowWorkflowState.RGB_BOOTSTRAP,
                        "user_authorized_follow",
                        timestamp_s=time.monotonic(),
                        timeout_s=(
                            self.bearing_target_estimator.config.bootstrap_timeout_s
                        ),
                    )
            else:
                self.visual_follow_auto_start_pending = False
                self._stop_visual_follow_locked("user_disabled")
                self.visual_follow_requested = False
                if self.active and self.tracker is not None and self.tracker.active:
                    self.follow_workflow.transition(
                        FollowWorkflowState.POINTING,
                        "follow_stopped_by_user",
                        timestamp_s=time.monotonic(),
                        force=True,
                    )
                else:
                    self.follow_workflow.transition(
                        FollowWorkflowState.HOLD,
                        "follow_stopped_by_user",
                        timestamp_s=time.monotonic(),
                        force=True,
                    )
        return self.status()

    def _bootstrap_start_position_locked(
        self,
    ) -> tuple[float, float, float]:
        """Sample current follower pose when the operator authorizes motion.

        The immutable selection pose remains the sole D0 reference. Bootstrap
        travel starts at authorization time so a previous abort or Position
        drift cannot consume the next motion budget.
        """

        if self.pose_provider is None:
            raise RuntimeError("Bootstrap start pose provider is unavailable")
        try:
            try:
                pose = self.pose_provider(
                    self.drone_id,
                    self.current_source_timestamp_s,
                )
            except TypeError:
                pose = self.pose_provider(self.drone_id)
            position = _finite_position(
                pose.get("vehicle_position_ned_m")
            )
            pose_age_ms = float(
                pose.get("telemetry_age_ms", math.inf)
            )
            gimbal_age_ms = float(
                pose.get("camera_age_ms", math.inf)
            )
        except (KeyError, TypeError, ValueError) as error:
            raise RuntimeError(
                f"Bootstrap start pose is invalid: {error}"
            ) from error
        if not pose.get("available") or position is None:
            raise RuntimeError("Bootstrap start pose is unavailable")
        if pose_age_ms > self.selection_pose_max_age_ms:
            raise RuntimeError("Bootstrap start vehicle pose is stale")
        if gimbal_age_ms > self.selection_gimbal_max_age_ms:
            raise RuntimeError("Bootstrap start gimbal pose is stale")
        return position

    def controls_gimbal(self, drone_id: str) -> bool:
        with self.lock:
            return bool(
                self.gimbal_enabled
                and self.gimbal_controller is not None
                and self.active
                and self.drone_id == drone_id
                and self.tracker is not None
                and self.tracker.active
            )

    async def mjpeg_frames(self):
        last_version = -1

        stream_session_id = self.session_id

        while True:
            if (
                not self.active
                or self.session_id != stream_session_id
            ):
                return

            version = self.output_version
            frame = self.output_frame

            if frame is None or version == last_version:
                await asyncio.sleep(0.001)
                continue

            last_version = version

            yield (
                b"--frame\r\n"
                b"Content-Type: image/jpeg\r\n"
                + f"Content-Length: {len(frame)}\r\n\r\n".encode("ascii")
                + frame
                + b"\r\n"
            )

    def status(self) -> dict[str, Any]:
        lock_started = time.perf_counter()
        with self.lock:
            self._record_timing_locked(
                "status_lock_wait_ms",
                (time.perf_counter() - lock_started) * 1000.0,
            )
            now = time.monotonic()
            last_output = self.last_output_monotonic
            result = self.latest_result
            bbox = self.selected_bbox
            score = None
            redetecting = False
            target_estimator_status = (
                self.target_estimate.as_dict()
                if self.metric_target_fusion.operational
                else self.bearing_target_estimator.status(now)
            )

            if result is not None:
                bbox = result.bbox.as_list() if result.bbox else None
                score = round(float(result.score), 3)
                redetecting = bool(result.redetecting)

            return {
                "available": self.available,
                "active": self.active,
                "drone_id": self.drone_id,
                "backend": self.backend,
                "tracker": (
                    self.opencv_tracker
                    if self.backend == "opencv"
                    else (
                        f"HYBRID-{self.opencv_tracker}"
                        if self.backend == "hybrid"
                        else self.backend.upper()
                    )
                ),
                "state": self.state,
                "error": self.error or TRACKING_IMPORT_ERROR,
                "has_frame": self.output_frame is not None,
                "last_frame_age_ms": (
                    round((time.monotonic() - last_output) * 1000)
                    if last_output is not None
                    else None
                ),
                "fps": round(self.fps, 1),
                "tracking_fps": round(self.fps, 1),
                "source_fps": self._source_fps_locked(),
                "tracking_ms": round(self.tracking_ms, 2),
                "encode_ms": round(self.encode_ms, 2),
                "preview_max_width": self.preview_max_width,
                "preview_publish_enabled": self.preview_publish_enabled,
                "controller_ms": round(self.controller_ms, 2),
                "main_loop_ms": round(self.main_loop_ms, 2),
                "frame_queue_age_ms": round(self.frame_queue_age_ms, 2),
                "source_frame_index": self.source_frame_index,
                "source_frames_dropped": self.source_frames_dropped,
                "timing": self._timing_status_locked(),
                "gimbal_enabled": self.gimbal_enabled,
                "gimbal_yaw_invert": self.gimbal_yaw_invert,
                "gimbal_pitch_invert": self.gimbal_pitch_invert,
                "gimbal_active": bool(
                    self.gimbal_enabled
                    and self.gimbal_controller is not None
                    and self.tracker is not None
                    and self.tracker.active
                ),
                "gimbal_angles_deg": dict(self.gimbal_angles_deg),
                "gimbal_commanded_angles_deg": dict(
                    self.gimbal_commanded_angles_deg
                ),
                "gimbal_feedback_available": self.gimbal_feedback_available,
                "gimbal_feedback_age_ms": self.gimbal_feedback_age_ms,
                "gimbal_measured_center_px": self.gimbal_measured_center_px,
                "gimbal_predicted_center_px": self.gimbal_predicted_center_px,
                "gimbal_prediction_horizon_ms": round(
                    self.gimbal_prediction_horizon_s * 1000.0,
                    1,
                ),
                "gimbal_prediction_latency_ms": round(
                    self.gimbal_prediction_latency_s * 1000.0,
                    1,
                ),
                "gimbal_max_prediction_horizon_ms": round(
                    self.gimbal_max_prediction_horizon_s * 1000.0,
                    1,
                ),
                "gimbal_command_rate_hz": self.gimbal_command_rate_hz,
                "gimbal_command_fps": self._gimbal_command_fps_locked(now),
                "gimbal_target_stale_ms": round(
                    self.gimbal_target_stale_s * 1000.0,
                    1,
                ),
                "gimbal_rates_deg_s": dict(self.gimbal_rates_deg_s),
                "gimbal_raw_requested_rates_deg_s": dict(
                    self.gimbal_raw_requested_rates_deg_s
                ),
                "gimbal_limited_rates_deg_s": dict(
                    self.gimbal_limited_rates_deg_s
                ),
                "gimbal_maximum_rates_deg_s": dict(
                    self.gimbal_maximum_rates_deg_s
                ),
                "gimbal_maximum_acceleration_deg_s2": round(
                    self.gimbal_maximum_acceleration_deg_s2,
                    1,
                ),
                "gimbal_maximum_braking_acceleration_deg_s2": round(
                    self.gimbal_maximum_braking_acceleration_deg_s2,
                    1,
                ),
                "gimbal_maximum_command_lead_deg": round(
                    self.gimbal_maximum_command_lead_deg,
                    1,
                ),
                "gimbal_error": self.gimbal_error,
                "gimbal_yaw_limit_deg": round(
                    self.gimbal_yaw_limit_deg,
                    1,
                ),
                "gimbal_yaw_saturated_outward": (
                    self.gimbal_yaw_saturated_outward
                ),
                "gimbal_yaw_recenter_latched": (
                    self.gimbal_yaw_recenter_latched
                ),
                "gimbal_yaw_home_error_deg": round(
                    float(self.gimbal_angles_deg.get("yaw", 0.0)),
                    2,
                ),
                "gimbal_yaw_home_exit_deg": round(
                    self.gimbal_yaw_home_exit_deg,
                    2,
                ),
                "gimbal_bbox_center_exit_deg": round(
                    self.gimbal_bbox_center_exit_deg,
                    2,
                ),
                "gimbal_yaw_requested_rate_deg_s": round(
                    self.gimbal_yaw_requested_rate_deg_s,
                    1,
                ),
                "body_yaw_enabled": self.body_yaw_enabled,
                "body_yaw_active": self.body_yaw_active,
                "body_yaw_latched": self.body_yaw_latched,
                "body_yaw_control_mode": "unified_offboard_follow",
                "body_yaw_target_error_deg": round(
                    self.body_yaw_target_error_deg,
                    2,
                ),
                "body_yaw_image_error_deg": round(
                    self.body_yaw_image_error_deg,
                    2,
                ),
                "body_pitch_image_error_deg": round(
                    self.body_pitch_image_error_deg,
                    2,
                ),
                "body_yaw_filtered_error_deg": round(
                    self.body_yaw_filtered_deg,
                    2,
                ),
                "body_yaw_command": round(
                    self.body_yaw_command_normalized,
                    3,
                ),
                "body_yaw_error": self.body_yaw_error,
                "follow_enabled": self.follow_enabled,
                "follow_active": self.follow_active,
                "follow_state": self.follow_state,
                "follow_block_reason": self.follow_block_reason,
                "follow_target_distance_m": round(
                    self.follow_distance_target_m,
                    2,
                ),
                "follow_safe_min_distance_m": round(
                    self.follow_min_distance_m,
                    2,
                ),
                "follow_safe_max_distance_m": round(
                    self.follow_max_distance_m,
                    2,
                ),
                "follow_brake_m_s2": round(self.follow_brake_m_s2, 2),
                "follow_distance_m": (
                    round(self.follow_distance_m, 2)
                    if self.follow_distance_m is not None
                    else None
                ),
                "motion_command_created_timestamp": (
                    self.motion_command_created_timestamp_s
                ),
                "target_estimator": target_estimator_status,
                "target_estimator_state": target_estimator_status["state"],
                "target_estimate_valid": target_estimator_status["valid"],
                "target_estimator_reason": target_estimator_status["reason"],
                "invalid_reason": (
                    ""
                    if target_estimator_status["valid"]
                    else target_estimator_status["reason"]
                ),
                "target_range_m": target_estimator_status["range_m"],
                "range_std_m": target_estimator_status["range_std_m"],
                "target_position_ned": target_estimator_status["position_ned_m"],
                "target_velocity_ned": target_estimator_status["velocity_ned_m_s"],
                "position_covariance": target_estimator_status["covariance"],
                "baseline_m": target_estimator_status["baseline_m"],
                "intersection_angle_deg": (
                    target_estimator_status["intersection_angle_deg"]
                ),
                "condition_number": target_estimator_status.get(
                    "condition_number"
                ),
                "reprojection_error_px": (
                    target_estimator_status["reprojection_error_px"]
                ),
                "observation_count": target_estimator_status["observation_count"],
                "observability_score": (
                    target_estimator_status["observability_score"]
                ),
                "estimate_age_ms": target_estimator_status["estimate_age_ms"],
                "bootstrap_progress": target_estimator_status["bootstrap_progress"],
                "bootstrap_guidance": target_estimator_status.get(
                    "bootstrap_guidance",
                    {},
                ),
                "target_velocity_valid": target_estimator_status["velocity_valid"],
                "target_estimator_error": self.target_estimator_error,
                "metric_target_fusion": self.metric_target_fusion.status(),
                "range_ground_truth": (
                    self.ground_truth_provider.status()
                    if callable(
                        getattr(self.ground_truth_provider, "status", None)
                    )
                    else {
                        "source": "unavailable",
                        "enabled": False,
                        "load_error": "ground_truth_provider_status_unavailable",
                    }
                ),
                "multiview_target_estimate": (
                    self.multiview_target_estimate.as_dict()
                ),
                "target_tracking_gate_reason": (
                    self.target_tracking_gate_reason
                ),
                "target_pose_age_ms": self.target_pose_age_ms,
                "target_gimbal_age_ms": self.target_gimbal_age_ms,
                "target_pose_interpolation_span_ms": (
                    self.target_pose_interpolation_span_ms
                ),
                "target_gimbal_interpolation_span_ms": (
                    self.target_gimbal_interpolation_span_ms
                ),
                "target_pose_sync": dict(self.target_pose_sync_diagnostics),
                "follow_lidar_distance_m": (
                    round(self.follow_lidar_distance_m, 2)
                    if self.follow_lidar_distance_m is not None
                    else None
                ),
                "follow_distance_source": self.follow_distance_source,
                "apparent_size_follow_enabled": (
                    self.apparent_size_follow_enabled
                ),
                "follow_reference_scale_px": (
                    round(self.follow_reference_scale_px, 2)
                    if self.follow_reference_scale_px is not None
                    else None
                ),
                "follow_reference_bbox_scale_px": (
                    round(self.follow_reference_bbox_scale_px, 2)
                    if self.follow_reference_bbox_scale_px is not None
                    else None
                ),
                "follow_filtered_scale_px": (
                    round(self.follow_filtered_scale_px, 2)
                    if self.follow_filtered_scale_px is not None
                    else None
                ),
                "follow_scale_ratio": (
                    round(self.follow_scale_ratio, 4)
                    if self.follow_scale_ratio is not None
                    else None
                ),
                "follow_bbox_scale_ratio": (
                    round(self.follow_bbox_scale_ratio, 4)
                    if self.follow_bbox_scale_ratio is not None
                    else None
                ),
                "follow_forward_blocked_by_bbox_guard": (
                    self.follow_forward_blocked_by_bbox_guard
                ),
                "bbox_forward_stop_ratio": round(
                    self.bbox_forward_stop_ratio,
                    4,
                ),
                "bbox_reverse_start_ratio": round(
                    self.bbox_reverse_start_ratio,
                    4,
                ),
                "object_scale_blend": round(self.object_scale_blend, 3),
                "follow_scale_error": round(self.follow_scale_error, 4),
                "follow_scale_error_rate": round(
                    self.follow_scale_error_rate,
                    4,
                ),
                "follow_object_scale_source": self.follow_object_scale_source,
                "follow_reference_scale_source": (
                    self.follow_reference_scale_source
                ),
                "follow_reference_warmup_ms": (
                    max(
                        0,
                        round(
                            (
                                self.follow_reference_warmup_until_monotonic
                                - time.monotonic()
                            )
                            * 1000
                        ),
                    )
                    if self.follow_reference_warmup_until_monotonic is not None
                    else 0
                ),
                "follow_scale_deadband": round(
                    self.apparent_size_deadband,
                    4,
                ),
                "follow_reverse_deadband": round(
                    self.apparent_size_reverse_deadband,
                    4,
                ),
                "follow_slowband": round(self.apparent_size_slowband, 4),
                "follow_hold_ratio_tolerance": round(
                    self.apparent_size_hold_ratio_tolerance,
                    4,
                ),
                "apparent_size_kd": round(self.apparent_size_kd, 3),
                "apparent_size_boost_max": round(
                    self.apparent_size_boost_max,
                    3,
                ),
                "apparent_size_rate_deadband": round(
                    self.apparent_size_rate_deadband,
                    3,
                ),
                "apparent_size_yaw_kp": round(
                    self.apparent_size_yaw_kp,
                    3,
                ),
                "apparent_size_yaw_deadband_deg": round(
                    self.apparent_size_yaw_deadband_deg,
                    2,
                ),
                "apparent_size_yaw_invert": self.apparent_size_yaw_invert,
                "follow_forward_command": round(
                    self.follow_command_forward,
                    3,
                ),
                "requested_forward_velocity_m_s": round(
                    self.follow_command_forward
                    / max(0.01, self.follow_max)
                    * self.offboard_max_forward_m_s
                    * self.offboard_forward_sign,
                    3,
                ),
                "follow_error": self.follow_error,
                "visual_follow_feature_enabled": (
                    self.visual_follow_feature_enabled
                ),
                "native_visual_follow_enabled": (
                    self.native_visual_follow_enabled
                ),
                "visual_follow_auto_start": self.visual_follow_auto_start,
                "visual_follow_auto_start_pending": (
                    self.visual_follow_auto_start_pending
                ),
                "visual_follow_requested": self.visual_follow_requested,
                "visual_follow_ready": self.visual_follow_ready,
                "relative_visual_follow_enabled": bool(
                    self.apparent_size_follow_enabled
                    and self.visual_follow_feature_enabled
                ),
                "relative_visual_follow_active": bool(
                    self.apparent_size_follow_enabled
                    and not self.visual_follow_requested
                    and self.motion_active
                ),
                "relative_visual_follow_state": (
                    self.relative_visual_follow_state
                ),
                "relative_visual_follow_block_reason": (
                    self.relative_visual_follow_block_reason
                ),
                "visual_motion_gate_reason": self.visual_motion_gate_reason,
                "metric_handover_authorized": self.visual_follow_requested,
                "metric_handover_stream_active": (
                    self.visual_target_stream_active
                ),
                "follow_workflow": self.follow_workflow.status(now),
                "follow_workflow_state": self.follow_workflow.state.value,
                "follow_workflow_reason": self.follow_workflow.reason,
                "pointing_guard_reason": self.pointing_guard_reason,
                "selection_snapshot": (
                    self.selection_snapshot.status()
                    if self.selection_snapshot is not None
                    else None
                ),
                "selection_pose_valid": bool(
                    self.selection_snapshot is not None
                    and self.selection_snapshot.valid
                ),
                "follow_min_altitude_m": self.follow_min_altitude_m,
                "visual_follow_center_error_deg": round(
                    self.visual_follow_center_error_deg,
                    2,
                ),
                "visual_follow_center_stable_ms": (
                    None
                    if self.visual_follow_center_since_monotonic is None
                    else round(
                        max(
                            0.0,
                            time.monotonic()
                            - self.visual_follow_center_since_monotonic,
                        )
                        * 1000.0
                    )
                ),
                "visual_follow_center_deadband_deg": round(
                    self.visual_follow_center_deadband_deg,
                    2,
                ),
                "visual_follow_active": self.visual_follow_active,
                "visual_target_stream_active": (
                    self.visual_target_stream_active
                ),
                "bootstrap_direction": self.bootstrap_direction,
                "bootstrap_corridor_confirmed": (
                    self.bootstrap_corridor_confirmed
                ),
                "bootstrap_command_count": self.bootstrap_command_count,
                "bootstrap_command_velocity_ned_m_s": {
                    "north": round(
                        self.bootstrap_command_velocity_ned[0], 3
                    ),
                    "east": round(
                        self.bootstrap_command_velocity_ned[1], 3
                    ),
                },
                "bootstrap_travel_m": round(self.bootstrap_travel_m, 3),
                "bootstrap_maximum_baseline_m": (
                    self.bootstrap_maximum_baseline_m
                ),
                "bootstrap_motion_error": self.bootstrap_motion_error,
                "initial_safe_distance_horizontal_m": (
                    self.initial_safe_distance_horizontal_m
                ),
                "initial_safe_distance_slant_m": (
                    self.initial_safe_distance_slant_m
                ),
                "initial_safe_distance_lock_timestamp_s": (
                    self.initial_safe_distance_lock_timestamp_s
                ),
                "initial_safe_distance_lock_count": (
                    self.initial_safe_distance_lock_count
                ),
                "safe_distance_lock_source": (
                    self.initial_safe_distance_source
                ),
                "initial_follow_angle_deg": self.initial_follow_angle_deg,
                "initial_follow_angle_source": (
                    self.initial_follow_angle_source
                ),
                "initial_vertical_offset_m": self.initial_vertical_offset_m,
                "initial_bbox_scale_px": self.initial_bbox_scale_px,
                "initial_range_std_m": self.initial_range_std_m,
                "target_coordinate_source": self.provisional_target_source,
                "provisional_target_active": bool(
                    self.target_estimate.valid
                    and self.target_estimate.state
                    in {"PROVISIONAL", "METRIC_HANDOVER"}
                ),
                "provisional_target_estimate": (
                    self.provisional_target_estimate.as_dict()
                ),
                "provisional_target_initial_slant_range_m": (
                    self.provisional_target_initial_slant_range_m
                ),
                "provisional_target_range_m": (
                    self.provisional_target_range_m
                ),
                "provisional_target_update_count": (
                    self.provisional_target_update_count
                ),
                "provisional_target_accepted_count": (
                    self.provisional_target_accepted_count
                ),
                "metric_handover_blend": round(
                    self.provisional_target_metric_blend,
                    3,
                ),
                "metric_handover_confirmed": (
                    self.provisional_target_metric_confirmed
                ),
                "current_estimated_horizontal_distance_m": (
                    self.current_estimated_horizontal_distance_m
                ),
                "follow_distance_error_m": (
                    None
                    if (
                        self.current_estimated_horizontal_distance_m is None
                        or self.initial_safe_distance_horizontal_m is None
                    )
                    else (
                        self.current_estimated_horizontal_distance_m
                        - self.initial_safe_distance_horizontal_m
                    )
                ),
                "follow_prestream_count": self.follow_prestream_count,
                "follow_prestream_min_messages": (
                    self.follow_prestream_min_messages
                ),
                "follow_mode_request_attempts": (
                    self.follow_mode_request_attempts
                ),
                "px4_nav_state": self.px4_nav_state,
                "px4_parameter_ack": self.visual_follow_bridge_status.get(
                    "parameter_ack",
                    "not_reported",
                ),
                "mode_request_ack": self.visual_follow_bridge_status.get(
                    "mode_request_ack",
                    "not_reported",
                ),
                "follow_target_publish_rate_hz": (
                    self.visual_follow_bridge_status.get(
                        "publish_rate_hz",
                        0.0,
                    )
                ),
                "follow_target_est_capabilities": (
                    3 if self.target_estimate.velocity_valid else 1
                ),
                "target_stream_age_ms": (
                    self.visual_follow_bridge_status.get(
                        "command_age_ms"
                    )
                ),
                "target_stream_timeout_state": (
                    self.visual_follow_bridge_status.get(
                        "timeout_state",
                        "INACTIVE",
                    )
                ),
                "visual_follow_bridge": dict(
                    self.visual_follow_bridge_status
                ),
                "visual_follow_state": self.visual_follow_state,
                "visual_follow_error": self.visual_follow_error,
                "visual_follow_last_stop_reason": (
                    self.visual_follow_last_stop_reason
                ),
                "visual_follow_last_stop_age_ms": (
                    None
                    if self.visual_follow_last_stop_monotonic is None
                    else round(
                        max(
                            0.0,
                            time.monotonic()
                            - self.visual_follow_last_stop_monotonic,
                        )
                        * 1000.0
                    )
                ),
                "visual_follow_boundary_invalid_ms": (
                    None
                    if self.visual_follow_boundary_invalid_since_monotonic
                    is None
                    else round(
                        max(
                            0.0,
                            time.monotonic()
                            - self.visual_follow_boundary_invalid_since_monotonic,
                        )
                        * 1000.0
                    )
                ),
                "apparent_size_metric_range_enabled": False,
                "visual_range_raw_m": None,
                "visual_range_filtered_m": None,
                "visual_bbox_scale_px": self.visual_bbox_scale_px,
                "visual_bbox_filter_state": self.visual_bbox_filter_state,
                "visual_bbox_stable_frames": self.visual_bbox_stable_frames,
                "visual_bbox_rejected_frames": self.visual_bbox_rejected_frames,
                "visual_bbox_center_residual_px": (
                    round(self.visual_bbox_center_residual_px, 2)
                    if self.visual_bbox_center_residual_px is not None
                    else None
                ),
                "filtered_bbox": self.visual_bbox_filtered_bbox,
                "filtered_bbox_area_px2": (
                    round(self.visual_bbox_filtered_area_px2, 2)
                    if self.visual_bbox_filtered_area_px2 is not None
                    else None
                ),
                "bbox_size_alpha_used": round(
                    self.visual_bbox_size_alpha_used,
                    3,
                ),
                "visual_range_quality": round(self.visual_range_quality, 3),
                "visual_ttc_s": self.visual_ttc_s,
                "visual_target_ned": self.visual_target_ned,
                "visual_target_velocity_ned": self.visual_target_velocity_ned,
                "visual_target_global": self.visual_target_global,
                "motion_control_mode": (
                    "apparent_size_offboard"
                    if self.apparent_size_follow_enabled
                    else
                    "px4_follow_target"
                    if (
                        self.visual_follow_feature_enabled
                        and self.native_visual_follow_enabled
                    )
                    else "offboard_dynamic_range"
                ),
                "motion_forward_sign": self.offboard_forward_sign,
                "motion_max_forward_m_s": round(
                    self.offboard_max_forward_m_s,
                    2,
                ),
                "motion_max_yaw_rate_deg_s": round(
                    self.offboard_max_yaw_rate_deg_s,
                    2,
                ),
                "motion_yaw_slew_deg_s2": round(
                    self.offboard_yaw_slew_deg_s2,
                    2,
                ),
                "follow_radial_velocity_m_s": round(
                    self.follow_radial_velocity_m_s,
                    3,
                ),
                "motion_active": self.motion_active,
                "motion_state": self.motion_state,
                "motion_forward_velocity_m_s": round(
                    self.motion_forward_velocity_m_s,
                    3,
                ),
                "published_forward_velocity_m_s": round(
                    self.motion_forward_velocity_m_s,
                    3,
                ),
                "motion_down_velocity_m_s": round(
                    self.motion_down_velocity_m_s,
                    3,
                ),
                "motion_yaw_rate_deg_s": round(
                    self.motion_yaw_rate_deg_s,
                    2,
                ),
                "requested_yaw_rate_deg_s": round(
                    self.motion_requested_yaw_rate_deg_s,
                    2,
                ),
                "published_yaw_rate_deg_s": round(
                    self.motion_yaw_rate_deg_s,
                    2,
                ),
                "combined_yaw_error_deg": round(
                    self.motion_combined_yaw_error_deg,
                    2,
                ),
                "yaw_authority": self.yaw_authority,
                "yaw_block_reason": self.yaw_block_reason,
                "gimbal_recenter_state": self.gimbal_recenter_state,
                "body_yaw_recenter_active": (
                    self.body_yaw_recenter_controller.active
                ),
                "body_yaw_recenter_filtered_deg": round(
                    self.body_yaw_recenter_filtered_deg,
                    2,
                ),
                "body_yaw_recenter_hold_elapsed_s": round(
                    self.body_yaw_recenter_enter_elapsed_s,
                    3,
                ),
                "body_yaw_recenter_enter_deg": (
                    self.body_yaw_recenter_controller.config.enter_deg
                ),
                "body_yaw_recenter_exit_deg": (
                    self.body_yaw_recenter_controller.config.exit_deg
                ),
                "body_yaw_recenter_hold_s": (
                    self.body_yaw_recenter_controller.config.enter_hold_s
                ),
                "body_yaw_recenter_max_rate_deg_s": (
                    self.body_yaw_recenter_controller.config.maximum_rate_deg_s
                ),
                "body_yaw_recenter_slew_deg_s2": (
                    self.body_yaw_recenter_controller.config.slew_rate_deg_s2
                ),
                "body_yaw_bbox_feedforward_gain": (
                    self.body_yaw_recenter_controller.config.bbox_feedforward_gain
                ),
                "motion_hold_z_down_m": self.motion_hold_z_down_m,
                "motion_vertical_recenter_active": (
                    self.motion_vertical_recenter_active
                ),
                "motion_vertical_pitch_error_deg": round(
                    self.motion_vertical_pitch_error_deg,
                    2,
                ),
                "vertical_recenter_deadband_deg": round(
                    self.vertical_recenter_deadband_deg,
                    2,
                ),
                "vertical_recenter_max_down_m_s": round(
                    self.vertical_recenter_max_down_m_s,
                    3,
                ),
                "motion_error": self.motion_error,
                "body_attitude_enabled": self.body_attitude_enabled,
                "body_attitude_active": self.body_attitude_active,
                "body_attitude_state": self.body_attitude_state,
                "body_attitude_rates_deg_s": dict(
                    self.body_attitude_rates_deg_s
                ),
                "body_attitude_thrust": round(
                    self.body_attitude_thrust,
                    4,
                ),
                "body_recenter_velocity_m_s": dict(
                    self.body_recenter_velocity_m_s
                ),
                "body_recenter_acceleration_m_s2": dict(
                    self.body_recenter_acceleration_m_s2
                ),
                "body_desired_tilt_deg": dict(self.body_desired_tilt_deg),
                "body_altitude_error_m": round(
                    self.body_altitude_error_m, 3
                ),
                "body_altitude_guard_active": (
                    self.body_altitude_guard_active
                ),
                "body_hold_z_down_m": self.body_hold_z_down_m,
                "body_attitude_error": self.body_attitude_error,
                "gimbal_home_error_deg": dict(self.gimbal_home_error_deg),
                "gimbal_last_command_age_ms": (
                    round(
                        (
                            time.monotonic()
                            - self.gimbal_last_command_monotonic
                        )
                        * 1000
                    )
                    if self.gimbal_last_command_monotonic is not None
                    else None
                ),
                "bbox": bbox,
                "score": score,
                "redetecting": redetecting,
                "package_root": str(TRACKING_PACKAGE_ROOT),
                "advanced": self._tracker_diagnostics_locked(),
            }

    def _create_tracker(self) -> tuple[Any, Any]:
        if self.backend == "opencv":
            return FastOpenCVTrackerService(self.opencv_tracker), None

        if self.backend == "hybrid":
            if HybridMemoryTrackerService is None:
                raise RuntimeError("Hybrid tracking backend is unavailable")
            return HybridMemoryTrackerService(self.opencv_tracker), None

        if InferenceService is None or TrackerService is None:
            raise RuntimeError(
                f"{self.backend.upper()} inference backend is unavailable"
            )

        inference_kwargs = (
            {"board_ip": self.board_ip}
            if self.backend == "board"
            else {}
        )
        inference = InferenceService(
            self.backend,
            **inference_kwargs,
        )
        return TrackerService(inference), inference

    def _tracker_diagnostics_locked(self) -> dict[str, Any]:
        diagnostics = getattr(self.tracker, "diagnostics", None)
        if callable(diagnostics):
            diagnostics = diagnostics()
        return dict(diagnostics) if isinstance(diagnostics, dict) else {
            "kalman": self.backend in {"hybrid", "onnx", "board"},
            "memory": self.backend in {"hybrid", "onnx", "board"},
            "reid": self.backend in {"hybrid", "onnx", "board"},
        }

    def _close_tracker_locked(self) -> None:
        with self.tracker_lock:
            if self.tracker is not None:
                try:
                    self.tracker.reset()
                except Exception:
                    pass

            if self.inference is not None:
                try:
                    self.inference.close()
                except Exception:
                    pass

        self.tracker = None
        self.inference = None
        self.tracker_epoch += 1

    def _run(self) -> None:
        source_versions = {
            drone_id: -1
            for drone_id in self.allowed_drones
        }

        while not self.shutdown_event.is_set():
            with self.lock:
                active = self.active
                drone_id = self.drone_id
                session_id = self.session_id

            if not active or drone_id not in self.allowed_drones:
                self.shutdown_event.wait(0.1)
                continue

            with self.gazebo_bridge.frame_condition:
                self.gazebo_bridge.frame_condition.wait_for(
                    lambda: (
                        self.gazebo_bridge.raw_frame_versions[drone_id]
                        != source_versions[drone_id]
                    ),
                    timeout=0.5,
                )
                frame = self.gazebo_bridge.raw_frames.get(drone_id)
                previous_source_version = source_versions[drone_id]
                source_version = (
                    self.gazebo_bridge.raw_frame_versions[drone_id]
                )
                source_timestamp_s = (
                    self.gazebo_bridge.last_raw_frame_monotonic.get(
                        drone_id,
                        time.monotonic(),
                    )
                )
                source_sim_timestamp_s = (
                    self.gazebo_bridge.last_raw_frame_sim_timestamp_s.get(
                        drone_id
                    )
                )
                source_versions[drone_id] = source_version

            if frame is None:
                continue
            if source_version == previous_source_version:
                # The condition wait timed out without a new Gazebo frame.
                # Do not feed the same timestamp to the tracker or bearing EKF
                # as a second observation.
                continue

            _trace_camera_hop(
                "camera_mailbox_dequeue",
                drone_id=drone_id,
                raw_frame_version=source_version,
                source_sim_timestamp_s=source_sim_timestamp_s,
                source_timestamp_s=source_timestamp_s,
                dequeue_monotonic_s=time.monotonic(),
            )

            try:
                if previous_source_version >= 0:
                    dropped = max(
                        0,
                        source_version - previous_source_version - 1,
                    )
                    if dropped:
                        with self.lock:
                            self.source_frames_dropped += dropped
                self._process_frame(
                    frame,
                    drone_id,
                    session_id,
                    source_version,
                    source_timestamp_s,
                    source_sim_timestamp_s,
                )
            except Exception as error:
                with self.lock:
                    if self.active and self.session_id == session_id:
                        self.state = "error"
                        self.error = f"Tracking frame failed: {error}"

    def _run_pointing(self) -> None:
        period_s = 1.0 / self.gimbal_command_rate_hz
        while not self.shutdown_event.is_set():
            loop_started = time.monotonic()
            with self.lock:
                if (
                    self.active
                    and self.drone_id in self.allowed_drones
                    and self.latest_raw_frame is not None
                    and self.latest_result is not None
                    and self.current_source_timestamp_s > 0.0
                    and (
                        loop_started - self.current_source_timestamp_s
                        <= self.gimbal_target_stale_s
                    )
                ):
                    self._update_gimbal_locked(
                        self.latest_raw_frame,
                        self.latest_result,
                        loop_started,
                        update_frame_subsystems=False,
                    )

            remaining_s = period_s - (time.monotonic() - loop_started)
            if remaining_s > 0.0:
                self.shutdown_event.wait(remaining_s)

    def _gimbal_command_fps_locked(self, now: float) -> float:
        cutoff = now - 1.0
        while (
            self.gimbal_command_timestamps
            and self.gimbal_command_timestamps[0] < cutoff
        ):
            self.gimbal_command_timestamps.popleft()
        if len(self.gimbal_command_timestamps) < 2:
            return 0.0
        elapsed_s = (
            self.gimbal_command_timestamps[-1]
            - self.gimbal_command_timestamps[0]
        )
        if elapsed_s <= 0.0:
            return 0.0
        return round(
            (len(self.gimbal_command_timestamps) - 1) / elapsed_s,
            1,
        )

    def _process_frame(
        self,
        frame: Any,
        drone_id: str,
        session_id: int,
        source_frame_index: int,
        source_timestamp_s: float,
        source_sim_timestamp_s: float | None,
    ) -> None:
        main_loop_started = time.perf_counter()
        with self.lock:
            if (
                not self.active
                or self.drone_id != drone_id
                or self.session_id != session_id
                or self.tracker is None
            ):
                return

            tracker = self.tracker
            tracker_epoch = self.tracker_epoch
            tracker_active = tracker.active
            self.latest_raw_frame = frame
            self.source_frame_index = source_frame_index
            self.current_source_timestamp_s = source_timestamp_s
            self.current_source_sim_timestamp_s = source_sim_timestamp_s
            queue_age_ms = max(
                0.0,
                (time.monotonic() - source_timestamp_s) * 1000.0,
            )
            self.frame_queue_age_ms = queue_age_ms
            self._record_timing_locked("frame_queue_age_ms", queue_age_ms)
            jpeg_quality = self.jpeg_quality

        result = None
        tracking_error: Exception | None = None
        tracking_ms = 0.0
        if tracker_active:
            tracking_started = time.perf_counter()
            try:
                with self.tracker_lock:
                    result = tracker.update(frame)
            except Exception as error:
                tracking_error = error
            tracking_ms = (
                time.perf_counter() - tracking_started
            ) * 1000.0
            _trace_camera_hop(
                "tracker_output",
                drone_id=drone_id,
                raw_frame_version=source_frame_index,
                source_sim_timestamp_s=source_sim_timestamp_s,
                source_timestamp_s=source_timestamp_s,
                tracking_ms=tracking_ms,
                tracker_output_monotonic_s=time.monotonic(),
            )

        with self.lock:
            if (
                not self.active
                or self.drone_id != drone_id
                or self.session_id != session_id
                or self.tracker is not tracker
                or self.tracker_epoch != tracker_epoch
            ):
                return

            if tracker_active and tracking_error is None:
                try:
                    now = time.monotonic()
                    result = self._filter_bbox_locked(result, now)
                    self._update_timing_locked(
                        "tracking_ms",
                        tracking_ms,
                    )
                    self.latest_result = result
                    self.state = result.state.value
                    self.error = ""
                    controller_started = time.perf_counter()
                    self._update_gimbal_locked(
                        frame,
                        result,
                        now,
                        update_frame_subsystems=True,
                    )
                    self._update_timing_locked(
                        "controller_ms",
                        (time.perf_counter() - controller_started) * 1000.0,
                    )
                except Exception as error:
                    with self.tracker_lock:
                        tracker.reset()
                    self.tracker_epoch += 1
                    self._reset_gimbal_locked()
                    self.latest_result = None
                    self.selected_bbox = None
                    self.state = "error"
                    self.error = f"Tracking inference failed: {error}"
            elif tracker_active:
                with self.tracker_lock:
                    tracker.reset()
                self.tracker_epoch += 1
                self._reset_gimbal_locked()
                self.latest_result = None
                self.selected_bbox = None
                self.state = "error"
                self.error = f"Tracking inference failed: {tracking_error}"
            elif self.state != "error":
                self.state = "selecting"
                self._prewarm_metric_calibration_locked(
                    frame,
                    source_timestamp_s,
                    source_frame_index,
                )

            self._update_fps_locked(time.monotonic())

        if not self.preview_publish_enabled:
            with self.lock:
                self._update_timing_locked(
                    "main_loop_ms",
                    (time.perf_counter() - main_loop_started) * 1000.0,
                )
            return

        overlay_started = time.perf_counter()
        display = frame.copy()
        self._draw_overlay(display, result)
        overlay_prepare_ms = (
            time.perf_counter() - overlay_started
        ) * 1000.0
        encode_started = time.perf_counter()
        if display.shape[1] > self.preview_max_width:
            preview_height = max(
                1,
                round(
                    display.shape[0]
                    * self.preview_max_width
                    / display.shape[1]
                ),
            )
            display = cv2.resize(
                display,
                (self.preview_max_width, preview_height),
                interpolation=cv2.INTER_AREA,
            )
        success, encoded = cv2.imencode(
            ".jpg",
            display,
            [cv2.IMWRITE_JPEG_QUALITY, jpeg_quality],
        )
        encode_ms = (time.perf_counter() - encode_started) * 1000.0

        with self.lock:
            if (
                not self.active
                or self.drone_id != drone_id
                or self.session_id != session_id
            ):
                return
            self._update_timing_locked(
                "encode_ms",
                encode_ms,
            )
            self._record_timing_locked(
                "overlay_prepare_ms",
                overlay_prepare_ms,
            )
            self._update_timing_locked(
                "main_loop_ms",
                (time.perf_counter() - main_loop_started) * 1000.0,
            )

            if not success:
                raise RuntimeError("OpenCV could not encode the tracking frame")

            self.output_frame = encoded.tobytes()
            self.output_version += 1
            self.last_output_monotonic = time.monotonic()
            self.output_condition.notify_all()

    def _update_fps_locked(self, now: float) -> None:
        if self.fps_window_started is None:
            self.fps_window_started = now
            self.fps_window_frames = 1
            return

        self.fps_window_frames += 1
        elapsed = now - self.fps_window_started

        if elapsed < 0.75:
            return

        measured_fps = self.fps_window_frames / elapsed
        self.fps = (
            measured_fps
            if self.fps <= 0.0
            else self.fps * 0.65 + measured_fps * 0.35
        )
        self.fps_window_started = now
        self.fps_window_frames = 0

    def _update_timing_locked(self, field: str, measured_ms: float) -> None:
        previous_ms = getattr(self, field)
        setattr(
            self,
            field,
            measured_ms
            if previous_ms <= 0.0
            else previous_ms * 0.8 + measured_ms * 0.2,
        )
        self._record_timing_locked(field, measured_ms)

    def _record_timing_locked(self, field: str, measured_ms: float) -> None:
        samples = self._timing_samples.get(field)
        if samples is not None and math.isfinite(measured_ms):
            samples.append(max(0.0, float(measured_ms)))

    def _timing_status_locked(self) -> dict[str, dict[str, float]]:
        status: dict[str, dict[str, float]] = {}
        for name, sample_window in self._timing_samples.items():
            samples = sorted(sample_window)
            if not samples:
                status[name] = {"average_ms": 0.0, "p95_ms": 0.0}
                continue
            p95_index = max(0, math.ceil(0.95 * len(samples)) - 1)
            status[name] = {
                "average_ms": round(sum(samples) / len(samples), 2),
                "p95_ms": round(samples[p95_index], 2),
            }
        return status

    def _reset_gimbal_locked(
        self,
        drone_id: str | None = None,
        *,
        preserve_metric_calibration: bool = False,
    ) -> None:
        self._disable_body_yaw_locked(drone_id or self.drone_id)
        self._reset_follow_locked(drone_id or self.drone_id)
        self._stop_motion_locked(drone_id or self.drone_id, "idle")
        self._reset_visual_follow_locked(
            drone_id or self.drone_id,
            preserve_metric_calibration=preserve_metric_calibration,
        )
        self.motion_command_created_timestamp_s = None
        self.bearing_target_estimator.reset()
        self.target_estimate = self.bearing_target_estimator.last_estimate
        self.multiview_target_estimate = (
            self.bearing_target_estimator.last_estimate
        )
        self.target_estimator_error = ""
        self.target_pose_age_ms: float | None = None
        self.target_gimbal_age_ms: float | None = None
        self.target_pose_interpolation_span_ms: float | None = None
        self.target_gimbal_interpolation_span_ms: float | None = None
        self.target_pose_sync_diagnostics = {}
        self.body_yaw_recenter_controller.reset()
        self.body_yaw_recenter_enter_elapsed_s = 0.0
        self.body_yaw_recenter_filtered_deg = 0.0
        if self.gimbal_controller is not None:
            self.gimbal_controller.reset()
        self.visual_bbox_filter.reset()
        self.visual_bbox_filter_state = "idle"
        self.visual_bbox_stable_frames = 0
        self.visual_bbox_rejected_frames = 0
        self.visual_bbox_center_residual_px = None
        self.visual_bbox_filtered_bbox = None
        self.visual_bbox_filtered_area_px2 = None
        self.visual_bbox_size_alpha_used = 0.0
        self.gimbal_last_update_monotonic = None
        self.gimbal_last_command_monotonic = None
        self.gimbal_command_timestamps.clear()
        self.gimbal_angles_deg = {
            "roll": 0.0,
            "pitch": 0.0,
            "yaw": 0.0,
        }
        self.gimbal_commanded_angles_deg = dict(self.gimbal_angles_deg)
        self.gimbal_maximum_command_lead_deg = 0.0
        self.gimbal_feedback_available = False
        self.gimbal_feedback_age_ms = None
        self.gimbal_measured_center_px = None
        self.gimbal_predicted_center_px = None
        self.gimbal_prediction_horizon_s = 0.0
        self.gimbal_rates_deg_s = {
            "roll": 0.0,
            "pitch": 0.0,
            "yaw": 0.0,
        }
        self.gimbal_raw_requested_rates_deg_s = dict(
            self.gimbal_rates_deg_s
        )
        self.gimbal_limited_rates_deg_s = dict(self.gimbal_rates_deg_s)
        self.gimbal_error = ""
        self.gimbal_yaw_limit_deg = (
            self.configured_gimbal_yaw_limit_deg
        )
        self.gimbal_yaw_saturated_outward = False
        self.gimbal_yaw_recenter_latched = False
        self.gimbal_yaw_requested_rate_deg_s = 0.0
        self.body_yaw_target_error_deg = 0.0
        self.body_yaw_image_error_deg = 0.0
        self.body_pitch_image_error_deg = 0.0
        self.follow_block_reason = "tracking_inactive"
        self.motion_requested_yaw_rate_deg_s = 0.0
        self.motion_combined_yaw_error_deg = 0.0
        self.yaw_authority = "none"
        self.yaw_block_reason = "tracking_inactive"
        self.gimbal_recenter_state = "inactive"

    def _stop_visual_follow_locked(self, state: str) -> None:
        was_authorized = bool(
            self.visual_follow_active or self.visual_follow_requested
        )
        should_publish = bool(
            self.drone_id
            and self.visual_target_command is not None
            and was_authorized
        )
        if should_publish:
            try:
                self.visual_target_command(
                    self.drone_id,
                    False,
                    None,
                    state,
                )
            except (
                ArithmeticError,
                KeyError,
                RuntimeError,
                TypeError,
                ValueError,
            ) as error:
                self.visual_follow_error = (
                    f"target_disable_failed:{type(error).__name__}:{error}"
                )
        if was_authorized:
            self.visual_follow_last_stop_reason = state
            self.visual_follow_last_stop_monotonic = time.monotonic()
        self.visual_follow_requested = False
        self.visual_follow_auto_start_pending = False
        self.visual_follow_active = False
        self.visual_target_stream_active = False
        self.relative_visual_follow_state = state
        self.relative_visual_follow_block_reason = state
        self.visual_follow_boundary_invalid_since_monotonic = None
        self.visual_follow_state = state
        self.visual_follow_last_publish_monotonic = None
        self.visual_follow_ready_since_monotonic = None
        self.visual_follow_center_since_monotonic = None
        self.visual_follow_handover_started_monotonic = None
        self.visual_follow_handover_blend = 0.0
        self.visual_follow_center_error_deg = 0.0

    def _reset_visual_follow_locked(
        self,
        drone_id: str,
        *,
        preserve_metric_calibration: bool = False,
    ) -> None:
        self._stop_visual_follow_locked("idle")
        self.bbox_motion_safety_estimator.reset()
        self.bearing_target_estimator.reset()
        self.metric_target_fusion.reset(
            preserve_calibration=preserve_metric_calibration
        )
        self.provisional_target_filter.reset()
        self.provisional_target_estimate = TargetEstimate(
            timestamp_s=0.0,
            state="PROVISIONAL",
            valid=False,
            reason="not_initialized",
        )
        self.provisional_target_initial_slant_range_m = None
        self.provisional_target_update_count = 0
        self.provisional_target_accepted_count = 0
        self.provisional_target_source = "none"
        self.provisional_target_range_m = None
        self.provisional_target_metric_handover_started_s = None
        self.provisional_target_metric_handover_origin_ned = None
        self.provisional_target_metric_blend = 0.0
        self.provisional_target_metric_confirmed = False
        self.target_estimate = self.bearing_target_estimator.last_estimate
        self.visual_follow_ready = False
        self.visual_follow_error = ""
        self.visual_follow_last_auto_attempt_monotonic = None
        self.visual_follow_ready_since_monotonic = None
        self.visual_follow_center_since_monotonic = None
        self.visual_follow_handover_started_monotonic = None
        self.visual_follow_handover_blend = 0.0
        self.visual_follow_center_error_deg = 0.0
        self.visual_range_raw_m = None
        self.visual_range_filtered_m = None
        self.visual_bbox_scale_px = None
        self.visual_range_quality = 0.0
        self.visual_ttc_s = None
        self.visual_motion_gate_reason = "idle"
        self.relative_visual_follow_state = "idle"
        self.relative_visual_follow_block_reason = ""
        self.visual_target_ned = None
        self.visual_target_velocity_ned = None
        self.visual_target_global = None

    def _filter_bbox_locked(self, result: Any, now: float) -> Any:
        bbox = result.bbox
        state = getattr(result.state, "value", str(result.state))
        filtered = self.visual_bbox_filter.update(
            cx=float(bbox.cx) if bbox is not None else 0.0,
            cy=float(bbox.cy) if bbox is not None else 0.0,
            width=float(bbox.w) if bbox is not None else 0.0,
            height=float(bbox.h) if bbox is not None else 0.0,
            timestamp_s=now,
            tracking_valid=bool(state == "tracking" and bbox is not None),
            confidence=float(getattr(result, "score", 0.0)),
        )
        self.visual_bbox_filter_state = filtered.reason
        self.visual_bbox_stable_frames = filtered.stable_frames
        self.visual_bbox_rejected_frames = filtered.rejected_frames
        self.visual_bbox_center_residual_px = filtered.center_residual_px
        self.visual_bbox_size_alpha_used = filtered.size_alpha_used
        if not filtered.valid:
            self.visual_bbox_filtered_bbox = None
            self.visual_bbox_filtered_area_px2 = None
            return replace(result, bbox=None)
        assert None not in (
            filtered.cx,
            filtered.cy,
            filtered.width,
            filtered.height,
        )
        filtered_bbox = BBox.from_center(
            float(filtered.cx),
            float(filtered.cy),
            float(filtered.width),
            float(filtered.height),
        )
        self.visual_bbox_filtered_bbox = filtered_bbox.as_list()
        self.visual_bbox_filtered_area_px2 = float(
            filtered_bbox.w * filtered_bbox.h
        )
        return replace(result, bbox=filtered_bbox)

    def _update_bearing_target_locked(
        self,
        frame: Any,
        result: Any,
        gimbal_command_ok: bool,
        now: float,
    ) -> TargetEstimate:
        state = getattr(result.state, "value", str(result.state))
        score = float(getattr(result, "score", 0.0))
        bbox = result.bbox
        tracker_diagnostics = self._tracker_diagnostics_locked()
        tracking_valid = bool(
            state == "tracking"
            and bbox is not None
            and gimbal_command_ok
            and not self.gimbal_error
            and not bool(getattr(result, "redetecting", False))
            and not bool(tracker_diagnostics.get("ambiguous", False))
            and not str(tracker_diagnostics.get("reject_reason", ""))
            and self.visual_bbox_stable_frames
            >= self.visual_bbox_filter.stable_frames_required
        )
        if not tracking_valid:
            if state != "tracking":
                self.target_tracking_gate_reason = f"tracker_state_{state}"
            elif bbox is None:
                self.target_tracking_gate_reason = "bbox_unavailable"
            elif not gimbal_command_ok:
                self.target_tracking_gate_reason = "gimbal_command_not_ok"
            elif self.gimbal_error:
                self.target_tracking_gate_reason = "gimbal_error"
            elif bool(getattr(result, "redetecting", False)):
                self.target_tracking_gate_reason = "tracker_redetecting"
            elif bool(tracker_diagnostics.get("ambiguous", False)):
                self.target_tracking_gate_reason = "tracker_ambiguous"
            elif str(tracker_diagnostics.get("reject_reason", "")):
                self.target_tracking_gate_reason = (
                    "tracker_reject:"
                    f"{tracker_diagnostics.get('reject_reason')}"
                )
            else:
                self.target_tracking_gate_reason = "bbox_stabilizing"
            self.target_estimate = self.bearing_target_estimator.invalidate(
                now,
                "tracking_lost" if state != "tracking" else "tracking_invalid",
            )
            self.target_estimator_error = self.target_estimate.reason
            return self.target_estimate
        self.target_tracking_gate_reason = "valid"
        if self.pose_provider is None:
            self.target_estimate = self.bearing_target_estimator.invalidate(
                now, "pose_provider_unavailable"
            )
            self.target_estimator_error = self.target_estimate.reason
            return self.target_estimate
        try:
            pose = self.pose_provider(
                self.drone_id,
                self.current_source_timestamp_s,
            )
        except TypeError:
            # Compatibility for test doubles and external providers that still
            # expose the previous one-argument callback.
            pose = self.pose_provider(self.drone_id)
        self.target_pose_sync_diagnostics = _pose_sync_diagnostics(pose)
        if not pose.get("available"):
            self.target_estimate = self.bearing_target_estimator.invalidate(
                now, str(pose.get("error", "pose_unavailable"))
            )
            self.target_estimator_error = self.target_estimate.reason
            return self.target_estimate
        try:
            local = pose["local_position"]
            camera_position = pose.get("camera_position_ned_m") or (
                float(local["x_north_m"]),
                float(local["y_east_m"]),
                float(local["z_down_m"]),
            )
            frame_height, frame_width = frame.shape[:2]
            camera_ray = self.visual_ray_projector.pixel_to_ned_ray(
                float(bbox.cx),
                float(bbox.cy),
                frame_width,
                frame_height,
                pose["camera_quaternion_xyzw"],
            )
            pose_age_s = max(
                0.0, float(pose.get("telemetry_age_ms", 0.0)) / 1000.0
            )
            gimbal_age_s = max(
                0.0, float(pose.get("camera_age_ms", 0.0)) / 1000.0
            )
            self.target_pose_age_ms = pose_age_s * 1000.0
            self.target_gimbal_age_ms = gimbal_age_s * 1000.0
            self.target_pose_interpolation_span_ms = _finite_or_none(
                pose.get("pose_interpolation_span_ms")
            )
            self.target_gimbal_interpolation_span_ms = _finite_or_none(
                pose.get("gimbal_interpolation_span_ms")
            )
            observation = BearingObservation(
                timestamp_s=float(self.current_source_timestamp_s or now),
                camera_position_ned_m=tuple(
                    float(value) for value in camera_position
                ),
                bearing_ned_unit=camera_ray,
                bbox_center_px=(float(bbox.cx), float(bbox.cy)),
                bbox_size_px=(float(bbox.w), float(bbox.h)),
                tracking_score=score,
                focal_length_px=0.5 * (
                    float(self.visual_ray_projector.fx)
                    + float(self.visual_ray_projector.fy)
                ),
                pose_age_s=pose_age_s,
                gimbal_age_s=gimbal_age_s,
                frame_index=self.source_frame_index,
                redetecting=bool(getattr(result, "redetecting", False)),
                ambiguous=bool(tracker_diagnostics.get("ambiguous", False)),
                reject_reason=str(
                    tracker_diagnostics.get("reject_reason", "")
                ),
            )
            estimator_started = time.perf_counter()
            self.target_estimate = self.bearing_target_estimator.update(
                observation,
                now_s=now,
            )
            self._record_timing_locked(
                "bearing_estimator_ms",
                (time.perf_counter() - estimator_started) * 1000.0,
            )
            self.target_estimator_error = (
                "" if self.target_estimate.valid else self.target_estimate.reason
            )
        except (KeyError, TypeError, ValueError) as error:
            self.target_estimate = self.bearing_target_estimator.invalidate(
                now, f"bearing_projection_invalid:{error}"
            )
            self.target_estimator_error = self.target_estimate.reason
        return self.target_estimate

    def _physical_range_instrumentation_context(
        self,
        pose: dict[str, Any],
        ground_truth: dict[str, Any] | None = None,
    ) -> dict[str, Any]:
        local = pose.get("local_position") or {}
        vehicle_position = pose.get("vehicle_position_ned_m") or (
            local.get("x_north_m"),
            local.get("y_east_m"),
            local.get("z_down_m"),
        )
        camera_position = pose.get("camera_position_ned_m")
        camera_offset = pose.get("camera_offset_body_frd_m")
        provider = dict(ground_truth or {})
        gt_reference = None
        if provider:
            gt_reference = {
                "source": provider.get("ground_truth_source"),
                "camera_pose_source": provider.get("camera_pose_source"),
                "camera_optical_center_xyz_enu_m": provider.get(
                    "camera_position_xyz"
                ),
                "camera_link_origin_xyz_enu_m": provider.get(
                    "camera_link_origin_xyz"
                ),
                "camera_drone_center_xyz_enu_m": provider.get(
                    "camera_drone_center_xyz"
                ),
                "target_model_origin_xyz_enu_m": provider.get(
                    "target_position_xyz"
                ),
                "camera_optical_center_to_target_model_origin_m": provider.get(
                    "camera_to_target_center_m"
                ),
                "drone_center_to_target_model_origin_m": provider.get(
                    "drone_center_to_center_m"
                ),
                "frame": "gazebo_enu",
                "clock_domain": provider.get("clock_domain"),
                "ground_truth_contract_version": provider.get(
                    "ground_truth_contract_version"
                ),
                "ground_truth_contract_sha256": provider.get(
                    "ground_truth_contract_sha256"
                ),
            }
        return {
            "raw_geometry_center_source": (
                "vehicle_position_ned_plus_configured_camera_offset_body_frd"
            ),
            "ground_truth_reference_centers": gt_reference,
            "optical_center": {
                "position_ned_m": camera_position,
                "configured_offset_body_frd_m": camera_offset,
                "position_enu_m": provider.get("camera_optical_center_xyz"),
                "status": "gazebo_actual_camera_sensor_optical_center",
                "camera_link_to_sensor_optical_center": (
                    [-0.0412, 0.0, -0.162]
                ),
            },
            "target_reference": {
                "raw": "eroded_bbox_foreground_inverse_depth_statistic",
                "ground_truth": "target_model_origin_when_simulation_GT",
            },
            "reference_center_unavailable_fields": [
                "target_model_origin_to_visible_surface_vector",
            ],
            "camera_info": {
                "configured_width": self.camera_width,
                "configured_height": self.camera_height,
                "configured_horizontal_fov_rad": (
                    self.camera_horizontal_fov_rad
                ),
                "configured_fx": self.camera_fx,
                "configured_fy": self.camera_fy,
                "camera_info_source": "tracking_environment_configuration",
                "distortion_model": None,
                "distortion_coefficients": None,
                "rectified": None,
                "unavailable_fields": [
                    "distortion_model",
                    "distortion_coefficients",
                    "rectified",
                    "ros_camera_info_hash",
                ],
            },
            "extrinsics": {
                "vehicle_position_ned_m": vehicle_position,
                "vehicle_quaternion_xyzw": pose.get(
                    "vehicle_quaternion_xyzw"
                ),
                "camera_offset_body_frd_m": camera_offset,
                "camera_position_ned_m": camera_position,
                "camera_quaternion_xyzw": pose.get(
                    "camera_quaternion_xyzw"
                ),
                "vehicle_frame": "px4_ned_frd",
                "camera_orientation_source": "synchronized_camera_imu",
            },
            "timestamps": {
                "frame_receipt_monotonic_s": self.current_source_timestamp_s,
                "frame_source_sim_timestamp_s": (
                    self.current_source_sim_timestamp_s
                ),
                "pose_query_timestamp_s": pose.get("frame_timestamp_s"),
                "pose_sync_reason": pose.get("pose_sync_reason"),
                "pose_sample_offset_ms": pose.get("pose_sample_offset_ms"),
                "pose_interpolation_span_ms": pose.get(
                    "pose_interpolation_span_ms"
                ),
                "gimbal_sync_reason": pose.get("gimbal_sync_reason"),
                "gimbal_sample_offset_ms": pose.get(
                    "gimbal_sample_offset_ms"
                ),
                "gimbal_interpolation_span_ms": pose.get(
                    "gimbal_interpolation_span_ms"
                ),
                "ground_truth_pose_sim_timestamp_s": provider.get(
                    "pose_sim_timestamp_s"
                ),
                "ground_truth_pose_age_ms": provider.get("pose_age_ms"),
                "ground_truth_interpolation_span_ms": provider.get(
                    "interpolation_span_ms"
                ),
                "ground_truth_pose_bracket_lower_sim_timestamp_s": provider.get(
                    "pose_bracket_lower_sim_timestamp_s"
                ),
                "ground_truth_pose_bracket_upper_sim_timestamp_s": provider.get(
                    "pose_bracket_upper_sim_timestamp_s"
                ),
                "ground_truth_camera_pose_sim_timestamp_s": provider.get(
                    "camera_pose_sim_timestamp_s"
                ),
                "ground_truth_target_pose_sim_timestamp_s": provider.get(
                    "target_pose_sim_timestamp_s"
                ),
                "sensor_capture_timestamp_s": None,
                "sensor_capture_clock": "unavailable",
            },
            "ground_truth_provider": provider or None,
        }

    def _prewarm_metric_calibration_locked(
        self,
        frame: Any,
        source_timestamp_s: float,
        frame_index: int,
    ) -> None:
        """Warm only the scene metric calibration while awaiting a bbox."""

        if (
            not self.metric_target_fusion.operational
            or self.pose_provider is None
            or not self.drone_id
        ):
            return
        now = time.monotonic()
        try:
            try:
                pose = self.pose_provider(
                    self.drone_id,
                    float(source_timestamp_s),
                )
            except TypeError:
                pose = self.pose_provider(self.drone_id)
            if not pose.get("available"):
                return
            if float(pose.get("telemetry_age_ms", math.inf)) > (
                self.selection_pose_max_age_ms
            ):
                return
            if float(pose.get("camera_age_ms", math.inf)) > (
                self.selection_gimbal_max_age_ms
            ):
                return
            local = pose["local_position"]
            camera_position = pose.get("camera_position_ned_m") or (
                float(local["x_north_m"]),
                float(local["y_east_m"]),
                float(local["z_down_m"]),
            )
            camera_quaternion = pose["camera_quaternion_xyzw"]
            self.metric_target_fusion.prewarm(
                frame_bgr=frame,
                measurement_timestamp_s=float(source_timestamp_s),
                now_s=now,
                frame_index=int(frame_index),
                camera_position_ned_m=camera_position,
                camera_quaternion_xyzw=camera_quaternion,
                dataset_group_id=(
                    f"{self.drone_id}.{self.follow_workflow.session_id}"
                ),
                dataset_session_id=self.follow_workflow.session_id,
                source_sim_timestamp_s=self.current_source_sim_timestamp_s,
                instrumentation_context=(
                    self._physical_range_instrumentation_context(pose)
                ),
            )
        except (ArithmeticError, KeyError, TypeError, ValueError):
            # Prewarming is opportunistic. Selection and the fast bbox/gimbal
            # path must remain available even when synchronized pose is not.
            return

    def _update_metric_target_fusion_locked(
        self,
        frame: Any,
        result: Any,
        gimbal_command_ok: bool,
        now: float,
    ) -> TargetEstimate:
        """Submit depth work without running model inference on this thread."""

        state = getattr(result.state, "value", str(result.state))
        score = float(getattr(result, "score", 0.0))
        bbox = result.bbox
        diagnostics = self._tracker_diagnostics_locked()
        tracking_valid = bool(
            state == "tracking"
            and bbox is not None
            and score >= self.follow_min_score
            and gimbal_command_ok
            and not self.gimbal_error
            and not bool(getattr(result, "redetecting", False))
            and not bool(diagnostics.get("ambiguous", False))
            and not str(diagnostics.get("reject_reason", ""))
            and self.visual_bbox_filter_state != "holding_outlier"
            and self.visual_bbox_stable_frames
            >= self.visual_bbox_filter.stable_frames_required
        )
        if not tracking_valid or bbox is None:
            self.target_tracking_gate_reason = "metric_tracking_invalid"
            return self.metric_target_fusion.predict(now)
        if self.pose_provider is None:
            return TargetEstimate(
                timestamp_s=now,
                state="METRIC_FUSION",
                valid=False,
                reason="pose_provider_unavailable",
            )
        try:
            try:
                pose = self.pose_provider(
                    self.drone_id,
                    self.current_source_timestamp_s,
                )
            except TypeError:
                pose = self.pose_provider(self.drone_id)
            self.target_pose_sync_diagnostics = _pose_sync_diagnostics(pose)
            if not pose.get("available"):
                raise ValueError(str(pose.get("error", "pose_unavailable")))
            local = pose["local_position"]
            camera_position = pose.get("camera_position_ned_m") or (
                float(local["x_north_m"]),
                float(local["y_east_m"]),
                float(local["z_down_m"]),
            )
            camera_quaternion = pose["camera_quaternion_xyzw"]
            self.target_pose_age_ms = max(
                0.0,
                float(pose.get("telemetry_age_ms", 0.0)),
            )
            self.target_gimbal_age_ms = max(
                0.0,
                float(pose.get("camera_age_ms", 0.0)),
            )
            self.target_pose_interpolation_span_ms = _finite_or_none(
                pose.get("pose_interpolation_span_ms")
            )
            self.target_gimbal_interpolation_span_ms = _finite_or_none(
                pose.get("gimbal_interpolation_span_ms")
            )
            ground_truth_distance_m = None
            ground_truth_valid = False
            ground_truth_reason = "collector_disabled"
            ground_truth_uncertainty_m = None
            ground_truth_time_offset_ms = None
            ground_truth_quality = "unavailable"
            ground_truth_lever_arm_corrected = False
            ground_truth_diagnostics = None
            if bool(
                getattr(
                    self.metric_target_fusion,
                    "dataset_collection_enabled",
                    False,
                )
                or getattr(
                    self.metric_target_fusion,
                    "physical_diagnostics_enabled",
                    False,
                )
            ):
                ground_truth_reason = "ground_truth_provider_unavailable"
                ground_truth_provider = getattr(
                    self,
                    "ground_truth_provider",
                    None,
                )
                if ground_truth_provider is not None:
                    try:
                        ground_truth_started = time.perf_counter()
                        target_id = str(
                            self.metric_target_fusion.dataset_collector.target_id
                            or self.metric_target_fusion
                            .physical_diagnostics_collector.target_id
                        )
                        ground_truth = ground_truth_provider(
                            self.drone_id,
                            self.current_source_timestamp_s,
                            self.current_source_sim_timestamp_s,
                            target_id,
                        )
                        self._record_timing_locked(
                            "ground_truth_provider_ms",
                            (time.perf_counter() - ground_truth_started)
                            * 1000.0,
                        )
                        ground_truth_valid = bool(
                            ground_truth.get("available", False)
                        )
                        ground_truth_diagnostics = dict(ground_truth)
                        ground_truth_reason = (
                            "ok"
                            if ground_truth_valid
                            else str(
                                ground_truth.get(
                                    "error",
                                    "ground_truth_unavailable",
                                )
                            )
                        )
                        if ground_truth_valid:
                            expected_source = str(
                                self.metric_target_fusion.dataset_collector
                                .ground_truth_source
                            )
                            if str(
                                ground_truth["ground_truth_source"]
                            ) != expected_source:
                                raise ValueError(
                                    "ground_truth_source_mismatch"
                                )
                            ground_truth_distance_m = float(
                                ground_truth[
                                    "camera_to_target_center_m"
                                ]
                            )
                            ground_truth_uncertainty_m = float(
                                ground_truth[
                                    "ground_truth_uncertainty_m"
                                ]
                            )
                            ground_truth_time_offset_ms = float(
                                ground_truth[
                                    "ground_truth_time_offset_ms"
                                ]
                            )
                            ground_truth_quality = str(
                                ground_truth["ground_truth_quality"]
                            )
                            ground_truth_lever_arm_corrected = bool(
                                ground_truth[
                                    "ground_truth_lever_arm_corrected"
                                ]
                            )
                    except (KeyError, TypeError, ValueError) as error:
                        ground_truth_valid = False
                        ground_truth_reason = (
                            f"ground_truth_invalid:{error}"
                        )
            fusion_started = time.perf_counter()
            estimate = self.metric_target_fusion.update(
                frame_bgr=frame,
                bbox_xywh=(
                    float(bbox.x),
                    float(bbox.y),
                    float(bbox.w),
                    float(bbox.h),
                ),
                bbox_center_px=(float(bbox.cx), float(bbox.cy)),
                tracking_score=score,
                tracking_valid=True,
                measurement_timestamp_s=float(
                    self.current_source_timestamp_s or now
                ),
                now_s=now,
                frame_index=self.source_frame_index,
                camera_position_ned_m=camera_position,
                camera_quaternion_xyzw=camera_quaternion,
                dataset_group_id=(
                    f"{self.drone_id}."
                    f"{self.follow_workflow.session_id}"
                ),
                dataset_session_id=self.follow_workflow.session_id,
                source_sim_timestamp_s=(
                    self.current_source_sim_timestamp_s
                ),
                ground_truth_distance_m=ground_truth_distance_m,
                ground_truth_valid=ground_truth_valid,
                ground_truth_reason=ground_truth_reason,
                ground_truth_uncertainty_m=ground_truth_uncertainty_m,
                ground_truth_time_offset_ms=ground_truth_time_offset_ms,
                ground_truth_quality=ground_truth_quality,
                ground_truth_lever_arm_corrected=(
                    ground_truth_lever_arm_corrected
                ),
                instrumentation_context=(
                    self._physical_range_instrumentation_context(
                        pose,
                        ground_truth_diagnostics,
                    )
                ),
            )
            self._record_timing_locked(
                "metric_fusion_ms",
                (time.perf_counter() - fusion_started) * 1000.0,
            )
            self.target_tracking_gate_reason = (
                "valid" if estimate.valid else estimate.reason
            )
            self.target_estimator_error = (
                "" if estimate.valid else estimate.reason
            )
            return estimate
        except (KeyError, TypeError, ValueError) as error:
            self.target_estimator_error = f"metric_projection_invalid:{error}"
            return TargetEstimate(
                timestamp_s=now,
                state="METRIC_FUSION",
                valid=False,
                reason=self.target_estimator_error,
            )

    @staticmethod
    def _target_estimate_quality(estimate: TargetEstimate) -> float:
        return max(0.0, min(1.0, float(estimate.observability_score)))

    def _pointing_follow_guard_locked(
        self,
        frame: Any,
        result: Any,
        gimbal_command_ok: bool,
    ) -> tuple[bool, str]:
        state = getattr(result.state, "value", str(result.state))
        bbox = result.bbox
        diagnostics = self._tracker_diagnostics_locked()
        if state != "tracking" or bbox is None:
            return False, "tracker_not_tracking"
        if bool(getattr(result, "redetecting", False)):
            return False, "tracker_redetecting"
        if bool(diagnostics.get("ambiguous", False)):
            return False, "tracker_ambiguous"
        reject_reason = str(diagnostics.get("reject_reason", ""))
        if reject_reason:
            return False, f"tracker_rejected:{reject_reason}"
        if float(getattr(result, "score", 0.0)) < self.follow_min_score:
            return False, "tracker_score_low"
        stable_frame_gate = max(
            self.visual_bbox_filter.stable_frames_required,
            self.visual_follow_min_stable_frames,
        )
        if self.visual_bbox_stable_frames < stable_frame_gate:
            return False, "bbox_filter_not_stable"
        if not gimbal_command_ok or self.gimbal_error:
            return False, "gimbal_command_unavailable"
        if not self.gimbal_feedback_available:
            return False, "gimbal_feedback_unavailable"
        if (
            self.gimbal_feedback_age_ms is None
            or self.gimbal_feedback_age_ms
            > self.selection_gimbal_max_age_ms
        ):
            return False, "gimbal_feedback_stale"
        if self.selection_snapshot is None:
            return False, "selection_snapshot_missing"
        if not self.selection_snapshot.valid:
            return False, self.selection_snapshot.reason
        frame_height, frame_width = frame.shape[:2]
        yaw_error_deg, pitch_error_deg = camera_angular_error_deg(
            center_x_px=float(bbox.cx),
            center_y_px=float(bbox.cy),
            frame_width_px=frame_width,
            frame_height_px=frame_height,
            calibrated_fx_px=float(self.gimbal_controller.cfg.fx),
            calibrated_fy_px=float(self.gimbal_controller.cfg.fy),
            calibration_width_px=self.camera_reference_width,
            calibration_height_px=self.camera_reference_height,
        )
        self.visual_follow_center_error_deg = math.hypot(
            yaw_error_deg,
            pitch_error_deg,
        )
        if self.pose_provider is None:
            return False, "pose_provider_unavailable"
        try:
            try:
                pose = self.pose_provider(
                    self.drone_id,
                    self.current_source_timestamp_s,
                )
            except TypeError:
                pose = self.pose_provider(self.drone_id)
        except (KeyError, TypeError, ValueError):
            return False, "vehicle_pose_invalid"
        if not pose.get("available"):
            return False, "vehicle_pose_stale"
        if float(pose.get("telemetry_age_ms", math.inf)) > (
            self.selection_pose_max_age_ms
        ):
            return False, "vehicle_pose_stale"
        if float(pose.get("camera_age_ms", math.inf)) > (
            self.selection_gimbal_max_age_ms
        ):
            return False, "gimbal_pose_stale"
        if not bool(pose.get("armed", False)):
            return False, "vehicle_not_armed"
        if bool(pose.get("failsafe", True)):
            return False, "px4_failsafe"
        if not bool(pose.get("local_position_valid", False)):
            return False, "local_position_invalid"
        if not bool(pose.get("global_position_valid", False)):
            return False, "global_position_invalid"
        if bool(pose.get("manual_override", False)):
            return False, "manual_override"
        position = _finite_position(pose.get("vehicle_position_ned_m"))
        if position is None:
            return False, "vehicle_position_invalid"
        if -position[2] < self.follow_min_altitude_m:
            return False, "follow_altitude_below_minimum"
        return True, "pointing_guard_valid"

    def _update_pointing_workflow_locked(
        self,
        frame: Any,
        result: Any,
        now: float,
        gimbal_command_ok: bool,
    ) -> None:
        if self.follow_workflow.state not in {
            FollowWorkflowState.POINTING,
            FollowWorkflowState.POINTING_STABLE,
            FollowWorkflowState.ACQUIRING_RANGE,
        }:
            self.visual_follow_ready = (
                self.follow_workflow.state
                == FollowWorkflowState.TARGET_INITIALIZED
            )
            return
        bbox = getattr(result, "bbox", None)
        if (
            bbox is not None
            and self.selection_snapshot is not None
            and not self.selection_snapshot.valid
            and self.selection_snapshot.reason
            in {
                "selection_pose_stale",
                "selection_gimbal_pose_stale",
            }
        ):
            frame_height, frame_width = frame.shape[:2]
            refreshed_snapshot = self._capture_selection_snapshot_locked(
                bbox,
                frame_width,
                frame_height,
            )
            if refreshed_snapshot.valid:
                # The selected bbox and tracker remain untouched. Only the
                # timestamp-aligned selection pose is refreshed.
                self.selection_snapshot = refreshed_snapshot
                self.follow_workflow.transition(
                    FollowWorkflowState.POINTING,
                    "selection_snapshot_reacquired",
                    timestamp_s=now,
                    force=True,
                )
        valid, reason = self._pointing_follow_guard_locked(
            frame,
            result,
            gimbal_command_ok,
        )
        self.pointing_guard_reason = reason
        if not valid:
            self.visual_follow_center_since_monotonic = None
            self.visual_follow_ready_since_monotonic = None
            self.visual_follow_ready = False
            if self.follow_workflow.state != FollowWorkflowState.POINTING:
                self.follow_workflow.transition(
                    FollowWorkflowState.POINTING,
                    reason,
                    timestamp_s=now,
                )
            else:
                self.follow_workflow.transition(
                    FollowWorkflowState.POINTING,
                    reason,
                    timestamp_s=now,
                )
            return
        if self.follow_workflow.state == FollowWorkflowState.POINTING:
            self.visual_follow_center_since_monotonic = now
            self.follow_workflow.transition(
                FollowWorkflowState.POINTING_STABLE,
                reason,
                timestamp_s=now,
                timeout_s=self.visual_follow_center_hold_s,
            )
            self.visual_follow_ready = False
            return
        if self.follow_workflow.state == FollowWorkflowState.POINTING_STABLE:
            if (
                now - self.follow_workflow.entered_timestamp_s
                >= self.visual_follow_center_hold_s
            ):
                self.visual_follow_ready_since_monotonic = now
                self.follow_workflow.transition(
                    FollowWorkflowState.ACQUIRING_RANGE,
                    "pointing_stable_hold_complete",
                    timestamp_s=now,
                )
                self.visual_follow_ready = False
            return
        self.visual_follow_ready = False

    def _stop_bootstrap_motion_locked(self, reason: str) -> None:
        if self.bootstrap_command is not None and self.drone_id:
            try:
                self.bootstrap_command(
                    self.drone_id,
                    False,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    reason,
                )
            except (
                ArithmeticError,
                KeyError,
                RuntimeError,
                TypeError,
                ValueError,
            ) as error:
                self.bootstrap_motion_error = (
                    f"bootstrap_neutral_failed:{error}"
                )
        self.bootstrap_command_velocity_ned = (0.0, 0.0)
        self.motion_forward_velocity_m_s = 0.0
        self.motion_down_velocity_m_s = 0.0
        self.motion_yaw_rate_deg_s = 0.0
        self.motion_active = False
        self.motion_state = reason
        self.motion_last_publish_monotonic = None

    def _abort_follow_workflow_locked(self, reason: str, now: float) -> None:
        self._stop_bootstrap_motion_locked(reason)
        self._stop_visual_follow_locked(reason)
        self.visual_follow_error = reason
        self.visual_follow_ready = False
        self.follow_workflow.transition(
            FollowWorkflowState.HOLD,
            reason,
            timestamp_s=now,
            force=True,
        )

    def _stop_visual_follow_at_boundary_locked(
        self,
        reason: str,
        now: float,
    ) -> None:
        """Keep Follow authorization and workflow state fail-closed together."""

        transient_reasons = {
            "no_bbox",
            "tracking_lost",
            "tracking_redetecting",
        }
        grace_states = {
            FollowWorkflowState.TARGET_3D_READY,
            FollowWorkflowState.FOLLOW_PRESTREAM,
            FollowWorkflowState.FOLLOW_MODE_REQUESTED,
            FollowWorkflowState.FOLLOWING,
            FollowWorkflowState.DEGRADED,
        }
        if (
            reason in transient_reasons
            and self.visual_follow_requested
            and self.follow_workflow.state in grace_states
        ):
            grace_timeout_s = getattr(
                self,
                "visual_follow_dropout_grace_s",
                1.50,
            )
            if self.visual_follow_boundary_invalid_since_monotonic is None:
                self.visual_follow_boundary_invalid_since_monotonic = now
            invalid_elapsed_s = (
                now - self.visual_follow_boundary_invalid_since_monotonic
            )
            if invalid_elapsed_s <= grace_timeout_s:
                self.visual_follow_state = "boundary_grace"
                self.visual_follow_error = (
                    f"Transient {reason}; neutral grace before HOLD"
                )
                return

        authorized_states = {
            FollowWorkflowState.RGB_BOOTSTRAP,
            FollowWorkflowState.TARGET_3D_READY,
            FollowWorkflowState.FOLLOW_PRESTREAM,
            FollowWorkflowState.FOLLOW_MODE_REQUESTED,
            FollowWorkflowState.FOLLOWING,
            FollowWorkflowState.DEGRADED,
        }
        if (
            self.visual_follow_requested
            or self.visual_follow_active
            or self.follow_workflow.state in authorized_states
        ):
            self._abort_follow_workflow_locked(reason, now)
            return
        self._stop_visual_follow_locked(reason)

    def _publish_rgb_bootstrap_locked(
        self,
        result: Any,
        now: float,
        gimbal_command_ok: bool,
    ) -> bool:
        if self.bootstrap_command is None:
            self._abort_follow_workflow_locked(
                "bootstrap_command_unavailable",
                now,
            )
            return False
        if (
            not self.bootstrap_corridor_confirmed
            or self.bootstrap_lateral_unit_ned is None
            or self.bootstrap_start_position_ned is None
        ):
            self._abort_follow_workflow_locked(
                "bootstrap_corridor_not_confirmed",
                now,
            )
            return False
        state = getattr(result.state, "value", str(result.state))
        diagnostics = self._tracker_diagnostics_locked()
        abort_reason = ""
        if state != "tracking" or result.bbox is None:
            abort_reason = "bootstrap_tracker_lost"
        elif bool(getattr(result, "redetecting", False)):
            abort_reason = "bootstrap_tracker_redetecting"
        elif bool(diagnostics.get("ambiguous", False)):
            abort_reason = "bootstrap_tracker_ambiguous"
        elif str(diagnostics.get("reject_reason", "")):
            abort_reason = "bootstrap_bbox_rejected"
        elif not gimbal_command_ok or self.gimbal_error:
            abort_reason = "bootstrap_gimbal_invalid"
        if abort_reason:
            self._abort_follow_workflow_locked(abort_reason, now)
            return False
        if self.pose_provider is None:
            self._abort_follow_workflow_locked(
                "bootstrap_pose_provider_unavailable",
                now,
            )
            return False
        try:
            try:
                pose = self.pose_provider(
                    self.drone_id,
                    self.current_source_timestamp_s,
                )
            except TypeError:
                pose = self.pose_provider(self.drone_id)
            position = _finite_position(pose.get("vehicle_position_ned_m"))
            pose_age_ms = float(pose.get("telemetry_age_ms", math.inf))
            gimbal_age_ms = float(pose.get("camera_age_ms", math.inf))
        except (KeyError, TypeError, ValueError):
            pose = {}
            position = None
            pose_age_ms = math.inf
            gimbal_age_ms = math.inf
        if not pose.get("available") or position is None:
            abort_reason = "bootstrap_pose_stale"
        elif pose_age_ms > self.selection_pose_max_age_ms:
            abort_reason = "bootstrap_pose_stale"
        elif gimbal_age_ms > self.selection_gimbal_max_age_ms:
            abort_reason = "bootstrap_gimbal_stale"
        elif not bool(pose.get("armed", False)):
            abort_reason = "bootstrap_vehicle_not_armed"
        elif bool(pose.get("failsafe", True)):
            abort_reason = "bootstrap_px4_failsafe"
        elif bool(pose.get("manual_override", False)):
            abort_reason = "bootstrap_manual_override"
        elif not bool(pose.get("local_position_valid", False)):
            abort_reason = "bootstrap_local_position_invalid"
        elif not bool(pose.get("global_position_valid", False)):
            abort_reason = "bootstrap_global_position_invalid"
        elif -position[2] < self.follow_min_altitude_m:
            abort_reason = "bootstrap_altitude_below_minimum"
        if abort_reason:
            self._abort_follow_workflow_locked(abort_reason, now)
            return False
        lateral_north, lateral_east = self.bootstrap_lateral_unit_ned
        displacement_north = position[0] - self.bootstrap_start_position_ned[0]
        displacement_east = position[1] - self.bootstrap_start_position_ned[1]
        self.bootstrap_travel_m = max(
            0.0,
            displacement_north * lateral_north
            + displacement_east * lateral_east,
        )
        if self.bootstrap_travel_m >= self.bootstrap_maximum_baseline_m:
            self._abort_follow_workflow_locked(
                "bootstrap_maximum_baseline_reached",
                now,
            )
            return False
        previous_publish = self.motion_last_publish_monotonic
        if previous_publish is not None and now - previous_publish < 0.045:
            return True
        remaining_m = max(
            0.0,
            self.bearing_target_estimator.config.bootstrap_lateral_distance_m
            - self.bootstrap_travel_m,
        )
        maximum_speed = (
            self.bearing_target_estimator.config.bootstrap_lateral_speed_m_s
        )
        commanded_speed = min(
            maximum_speed,
            max(0.15, 1.5 * remaining_m),
        )
        yaw_dt = (
            1.0 / 20.0
            if previous_publish is None
            else max(0.001, min(0.1, now - previous_publish))
        )
        yaw_output = self.body_yaw_recenter_controller.update(
            gimbal_yaw_deg=float(self.gimbal_angles_deg.get("yaw", 0.0)),
            bbox_horizontal_error_deg=self.body_yaw_image_error_deg,
            tracking_valid=True,
            dt_s=yaw_dt,
            gimbal_fresh=self.gimbal_feedback_available,
        )
        north_velocity = lateral_north * commanded_speed
        east_velocity = lateral_east * commanded_speed
        response = self.bootstrap_command(
            self.drone_id,
            True,
            north_velocity,
            east_velocity,
            float(self.bootstrap_start_position_ned[2]),
            yaw_output.limited_rate_deg_s,
            "rgb_bootstrap",
        )
        self.motion_last_publish_monotonic = now
        self.bootstrap_command_count += 1
        self.bootstrap_motion_error = str(response.get("error", ""))
        self.motion_active = bool(response.get("ok"))
        self.motion_state = (
            "rgb_bootstrap" if self.motion_active else "bootstrap_blocked"
        )
        self.motion_yaw_rate_deg_s = (
            yaw_output.limited_rate_deg_s if self.motion_active else 0.0
        )
        self.motion_requested_yaw_rate_deg_s = (
            yaw_output.requested_rate_deg_s
        )
        self.yaw_authority = "rgb_bootstrap"
        self.bootstrap_command_velocity_ned = (
            north_velocity if self.motion_active else 0.0,
            east_velocity if self.motion_active else 0.0,
        )
        if not self.motion_active:
            self._abort_follow_workflow_locked(
                self.bootstrap_motion_error or "bootstrap_offboard_rejected",
                now,
            )
            return False
        return True

    def _initialize_provisional_target_locked(
        self,
        now: float,
    ) -> TargetEstimate:
        """Create the first native target from the selection ray and D0.

        Monocular RGB cannot observe absolute scale from one frame.  D0 is
        therefore used once as an explicit provisional scale anchor.  It is
        kept separate from the metric fusion quality state and is replaced
        with a rate-limited handover when a metric estimate converges.
        """

        if self.provisional_target_estimate.valid:
            return self.provisional_target_estimate
        snapshot = self.selection_snapshot
        if (
            snapshot is None
            or not snapshot.valid
            or snapshot.selection_camera_position_ned is None
            or snapshot.selection_vehicle_position_ned is None
            or snapshot.selection_bearing_ned is None
        ):
            return TargetEstimate(
                timestamp_s=now,
                state="PROVISIONAL",
                valid=False,
                reason="selection_snapshot_invalid",
            )
        safe_distance = float(self.follow_distance_target_m)
        if safe_distance < 3.0 or safe_distance > 20.0:
            return TargetEstimate(
                timestamp_s=now,
                state="PROVISIONAL",
                valid=False,
                reason="safe_distance_outside_3_20_m",
            )
        try:
            target_position, slant_range = selection_anchored_target_ned(
                camera_position_ned=snapshot.selection_camera_position_ned,
                vehicle_position_ned=snapshot.selection_vehicle_position_ned,
                bearing_ned=snapshot.selection_bearing_ned,
                safe_horizontal_distance_m=safe_distance,
            )
        except ValueError as error:
            return TargetEstimate(
                timestamp_s=now,
                state="PROVISIONAL",
                valid=False,
                reason=f"selection_anchor_invalid:{error}",
            )

        filtered = self.provisional_target_filter.update(target_position, now)
        if not filtered.valid or filtered.position_ned is None:
            return TargetEstimate(
                timestamp_s=now,
                state="PROVISIONAL",
                valid=False,
                reason=filtered.reason,
            )
        range_std = max(1.0, 0.35 * slant_range)
        variance = range_std * range_std
        self.provisional_target_initial_slant_range_m = slant_range
        self.provisional_target_range_m = slant_range
        self.provisional_target_update_count = 1
        self.provisional_target_accepted_count = 1
        self.provisional_target_source = "selection_anchor"
        self.initial_bbox_scale_px = snapshot.selection_bbox_scale_px
        self.provisional_target_estimate = TargetEstimate(
            timestamp_s=now,
            state="PROVISIONAL",
            valid=True,
            reason="selection_anchor_initialized",
            position_ned_m=filtered.position_ned,
            velocity_ned_m_s=(0.0, 0.0, 0.0),
            covariance=(
                (variance, 0.0, 0.0),
                (0.0, variance, 0.0),
                (0.0, 0.0, variance),
            ),
            range_m=slant_range,
            range_std_m=range_std,
            estimate_age_ms=0.0,
            velocity_valid=False,
            estimator_mode="PROVISIONAL_STATIONARY",
            stationary_probability=1.0,
            moving_probability=0.0,
        )
        return self.provisional_target_estimate

    def _update_provisional_target_locked(
        self,
        frame: Any,
        result: Any,
        now: float,
        gimbal_command_ok: bool,
    ) -> TargetEstimate:
        was_initialized = self.provisional_target_estimate.valid
        initial = self._initialize_provisional_target_locked(now)
        if not initial.valid:
            self.provisional_target_estimate = initial
            return initial
        if not was_initialized:
            return initial
        bbox = getattr(result, "bbox", None)
        state_object = getattr(result, "state", None)
        state = getattr(state_object, "value", str(state_object))
        tracker_diagnostics = self._tracker_diagnostics_locked()
        if (
            state != "tracking"
            or bbox is None
            or not gimbal_command_ok
            or self.gimbal_error
            or float(getattr(result, "score", 0.0)) < self.follow_min_score
            or self.visual_bbox_filter_state != "ready"
            or bool(getattr(result, "redetecting", False))
            or bool(tracker_diagnostics.get("ambiguous", False))
            or bool(tracker_diagnostics.get("reject_reason", ""))
        ):
            held = self.provisional_target_filter.output(
                now,
                "provisional_measurement_unavailable",
            )
            if not held.valid or held.position_ned is None:
                return TargetEstimate(
                    timestamp_s=now,
                    state="PROVISIONAL",
                    valid=False,
                    reason=held.reason,
                )
            self.provisional_target_estimate = replace(
                self.provisional_target_estimate,
                reason="provisional_measurement_unavailable",
                estimate_age_ms=held.age_s * 1000.0,
            )
            return self.provisional_target_estimate
        if self.pose_provider is None:
            return TargetEstimate(
                timestamp_s=now,
                state="PROVISIONAL",
                valid=False,
                reason="pose_provider_unavailable",
            )
        try:
            try:
                pose = self.pose_provider(
                    self.drone_id,
                    self.current_source_timestamp_s,
                )
            except TypeError:
                pose = self.pose_provider(self.drone_id)
            if not pose.get("available"):
                raise ValueError(
                    str(pose.get("error", "synchronized pose unavailable"))
                )
            if float(pose.get("telemetry_age_ms", math.inf)) > (
                self.selection_pose_max_age_ms
            ):
                raise ValueError("vehicle_pose_stale")
            if float(pose.get("camera_age_ms", math.inf)) > (
                self.selection_gimbal_max_age_ms
            ):
                raise ValueError("gimbal_pose_stale")
            camera_position = _finite_position(
                pose.get("camera_position_ned_m")
            )
            camera_quaternion = pose.get("camera_quaternion_xyzw")
            if (
                camera_position is None
                or not isinstance(camera_quaternion, (tuple, list))
                or len(camera_quaternion) != 4
            ):
                raise ValueError("camera_pose_invalid")
            frame_height, frame_width = frame.shape[:2]
            bearing = self.visual_ray_projector.pixel_to_ned_ray(
                float(bbox.cx),
                float(bbox.cy),
                frame_width,
                frame_height,
                tuple(float(value) for value in camera_quaternion),
            )
            current_scale = math.sqrt(
                max(1.0, float(bbox.w) * float(bbox.h))
            )
            reference_scale = float(self.initial_bbox_scale_px or 0.0)
            initial_slant = float(
                self.provisional_target_initial_slant_range_m or 0.0
            )
            if reference_scale <= 0.0 or initial_slant <= 0.0:
                raise ValueError("selection_scale_invalid")
            observed_range = max(
                1.0,
                min(
                    80.0,
                    initial_slant * reference_scale / current_scale,
                ),
            )
            measurement = tuple(
                camera_position[index] + observed_range * bearing[index]
                for index in range(3)
            )
        except (KeyError, TypeError, ValueError) as error:
            held = self.provisional_target_filter.output(
                now,
                f"provisional_measurement_rejected:{error}",
            )
            if held.valid:
                self.provisional_target_estimate = replace(
                    self.provisional_target_estimate,
                    reason=f"provisional_measurement_rejected:{error}",
                    estimate_age_ms=held.age_s * 1000.0,
                )
                return self.provisional_target_estimate
            return TargetEstimate(
                timestamp_s=now,
                state="PROVISIONAL",
                valid=False,
                reason=f"provisional_measurement_rejected:{error}",
            )

        self.provisional_target_update_count += 1
        filtered = self.provisional_target_filter.update(measurement, now)
        accepted = filtered.reason in {
            "initialized",
            "reinitialized",
            "tracking",
        }
        if accepted:
            self.provisional_target_accepted_count += 1
            self.provisional_target_range_m = observed_range
        if not filtered.valid or filtered.position_ned is None:
            return TargetEstimate(
                timestamp_s=now,
                state="PROVISIONAL",
                valid=False,
                reason=filtered.reason,
            )
        velocity = filtered.velocity_ned or (0.0, 0.0, 0.0)
        speed = math.sqrt(sum(value * value for value in velocity))
        velocity_valid = bool(
            self.provisional_target_accepted_count >= 12
            and filtered.reason == "tracking"
        )
        range_std = max(1.0, 0.30 * observed_range)
        variance = range_std * range_std
        accepted_timestamp = float(
            self.provisional_target_filter.last_timestamp_s or now
        )
        self.provisional_target_source = "selection_anchor_cv2"
        self.provisional_target_estimate = TargetEstimate(
            timestamp_s=accepted_timestamp,
            state="PROVISIONAL",
            valid=True,
            reason=(
                "selection_anchor_tracking"
                if accepted
                else filtered.reason
            ),
            position_ned_m=filtered.position_ned,
            velocity_ned_m_s=velocity,
            covariance=(
                (variance, 0.0, 0.0),
                (0.0, variance, 0.0),
                (0.0, 0.0, variance),
            ),
            range_m=observed_range,
            range_std_m=range_std,
            estimate_age_ms=max(
                0.0,
                (now - accepted_timestamp) * 1000.0,
            ),
            velocity_valid=velocity_valid,
            estimator_mode=(
                "PROVISIONAL_STATIONARY"
                if speed < 0.25
                else "PROVISIONAL_CONSTANT_VELOCITY"
            ),
            stationary_probability=0.9 if speed < 0.25 else 0.2,
            moving_probability=0.1 if speed < 0.25 else 0.8,
        )
        return self.provisional_target_estimate

    def _select_follow_target_estimate_locked(
        self,
        provisional: TargetEstimate,
        metric: TargetEstimate | None,
        now: float,
    ) -> TargetEstimate:
        metric_ready = bool(
            metric is not None
            and metric.valid
            and self.metric_target_fusion.ready_for_safe_distance_lock
            and metric.position_ned_m is not None
        )
        if not metric_ready:
            if (
                getattr(
                    self,
                    "provisional_target_metric_handover_started_s",
                    None,
                )
                is not None
            ):
                self.provisional_target_metric_handover_started_s = None
                self.provisional_target_metric_handover_origin_ned = None
                self.provisional_target_metric_blend = 0.0
            self.provisional_target_source = (
                "selection_anchor_fallback"
                if getattr(
                    self,
                    "provisional_target_metric_confirmed",
                    False,
                )
                else "selection_anchor_cv2"
            )
            return provisional
        assert metric is not None and metric.position_ned_m is not None
        if (
            getattr(
                self,
                "provisional_target_metric_confirmed",
                False,
            )
            or not provisional.valid
            or provisional.position_ned_m is None
        ):
            self.provisional_target_metric_confirmed = True
            self.provisional_target_metric_blend = 1.0
            self.provisional_target_source = (
                self.metric_target_fusion.active_metric_source
            )
            return metric
        if (
            getattr(
                self,
                "provisional_target_metric_handover_started_s",
                None,
            )
            is None
        ):
            self.provisional_target_metric_handover_started_s = now
            self.provisional_target_metric_handover_origin_ned = (
                provisional.position_ned_m
            )
        origin = self.provisional_target_metric_handover_origin_ned
        assert origin is not None
        blend = max(
            0.0,
            min(
                1.0,
                (
                    now - self.provisional_target_metric_handover_started_s
                )
                / max(0.5, self.visual_follow_handover_s),
            ),
        )
        self.provisional_target_metric_blend = blend
        if blend >= 1.0:
            self.provisional_target_metric_confirmed = True
            self.provisional_target_source = (
                self.metric_target_fusion.active_metric_source
            )
            return metric
        position = tuple(
            (1.0 - blend) * origin[index]
            + blend * metric.position_ned_m[index]
            for index in range(3)
        )
        provisional_variance = max(
            1.0,
            float(provisional.range_std_m or 3.0) ** 2,
        )
        metric_variance = (
            max(
                0.0,
                max(
                    float(metric.covariance[index][index])
                    for index in range(3)
                ),
            )
            if len(metric.covariance) >= 3
            and all(len(metric.covariance[index]) >= 3 for index in range(3))
            else provisional_variance
        )
        variance = (
            (1.0 - blend) * provisional_variance
            + blend * metric_variance
        )
        self.provisional_target_source = "metric_handover"
        return TargetEstimate(
            timestamp_s=now,
            state="METRIC_HANDOVER",
            valid=True,
            reason="provisional_to_metric_handover",
            position_ned_m=position,
            velocity_ned_m_s=(0.0, 0.0, 0.0),
            covariance=(
                (variance, 0.0, 0.0),
                (0.0, variance, 0.0),
                (0.0, 0.0, variance),
            ),
            range_m=(
                (1.0 - blend) * float(provisional.range_m or 0.0)
                + blend * float(metric.range_m or 0.0)
            ),
            range_std_m=math.sqrt(variance),
            estimate_age_ms=0.0,
            velocity_valid=False,
            estimator_mode="METRIC_HANDOVER",
            stationary_probability=None,
            moving_probability=None,
        )

    def _save_initial_safe_distance_locked(
        self,
        target_estimate: TargetEstimate,
    ) -> bool:
        if getattr(self, "initial_safe_distance_horizontal_m", None) is not None:
            # D0/FLW_TGT_DST is immutable for the selected-target session.
            return True
        metric_fusion = getattr(self, "metric_target_fusion", None)
        if (
            metric_fusion is not None
            and bool(getattr(metric_fusion, "operational", False))
            and not bool(
                getattr(
                    metric_fusion,
                    "ready_for_safe_distance_lock",
                    False,
                )
            )
        ):
            return False
        snapshot = self.selection_snapshot
        target_position = target_estimate.position_ned_m
        if (
            snapshot is None
            or not snapshot.valid
            or not target_estimate.valid
            or snapshot.selection_vehicle_position_ned is None
            or target_position is None
            or target_estimate.range_std_m is None
        ):
            return False
        delta = tuple(
            float(target_position[index])
            - float(snapshot.selection_vehicle_position_ned[index])
            for index in range(3)
        )
        horizontal = math.hypot(delta[0], delta[1])
        slant = math.sqrt(sum(value * value for value in delta))
        if not all(
            math.isfinite(value)
            for value in (
                horizontal,
                slant,
                delta[2],
                target_estimate.range_std_m,
            )
        ):
            return False
        if horizontal < 3.0 or horizontal > 20.0:
            return False
        self.initial_safe_distance_horizontal_m = horizontal
        self.initial_safe_distance_slant_m = slant
        self.initial_safe_distance_lock_timestamp_s = time.monotonic()
        self.initial_safe_distance_lock_count = (
            int(getattr(self, "initial_safe_distance_lock_count", 0)) + 1
        )
        self.initial_safe_distance_source = "metric_quality_gate"
        self.initial_follow_angle_deg = math.degrees(
            math.atan2(-delta[1], -delta[0])
        )
        self.initial_follow_angle_source = "selection_target_to_vehicle_bearing"
        self.initial_vertical_offset_m = delta[2]
        self.initial_bbox_scale_px = snapshot.selection_bbox_scale_px
        self.initial_range_std_m = float(target_estimate.range_std_m)
        return True

    def _update_visual_follow_locked(
        self,
        frame: Any,
        result: Any,
        now: float,
        gimbal_command_ok: bool,
    ) -> None:
        state = getattr(result.state, "value", str(result.state))
        score = float(getattr(result, "score", 0.0))
        bbox = result.bbox
        frame_height, frame_width = frame.shape[:2]
        metric_target_fusion = getattr(
            self,
            "metric_target_fusion",
            None,
        )
        metric_fusion_enabled = bool(
            metric_target_fusion is not None
            and metric_target_fusion.operational
        )
        if (
            self.visual_bbox_filter_state == "holding_outlier"
            and not (
                metric_fusion_enabled
                and self.visual_follow_requested
            )
        ):
            self.visual_follow_state = "holding_outlier"
            return
        estimate = self.bbox_motion_safety_estimator.update(
            width_px=float(bbox.w) if bbox is not None else 0.0,
            height_px=float(bbox.h) if bbox is not None else 0.0,
            frame_width=frame_width,
            frame_height=frame_height,
            timestamp_s=now,
            tracking_score=score,
            tracking_valid=bool(
                state == "tracking"
                and score >= self.follow_min_score
                and gimbal_command_ok
                and not self.gimbal_error
                and bbox is not None
                and self.visual_bbox_stable_frames
                >= self.visual_bbox_filter.stable_frames_required
            ),
        )
        self.visual_range_raw_m = None
        self.visual_range_filtered_m = None
        # BBoxMotionEstimate intentionally exposes only dimensionless
        # apparent-size ratios.  This diagnostic is the geometric bbox scale,
        # not a metric-range estimate.
        self.visual_bbox_scale_px = (
            math.sqrt(max(0.0, float(bbox.w) * float(bbox.h)))
            if bbox is not None
            else None
        )
        self.visual_range_quality = estimate.quality
        self.visual_ttc_s = estimate.ttc_s
        self.visual_motion_gate_reason = estimate.reason
        workflow_enabled = hasattr(self, "follow_workflow")
        if workflow_enabled:
            self._update_pointing_workflow_locked(
                frame,
                result,
                now,
                gimbal_command_ok,
            )
        metric_target_estimate: TargetEstimate | None = None
        if metric_fusion_enabled:
            metric_target_estimate = (
                self._update_metric_target_fusion_locked(
                    frame,
                    result,
                    gimbal_command_ok,
                    now,
                )
            )
            # Passive multi-view triangulation is an independent range
            # cross-check. It uses only naturally occurring baseline and never
            # starts bootstrap motion without operator authorization.
            self.multiview_target_estimate = (
                self._update_bearing_target_locked(
                    frame,
                    result,
                    gimbal_command_ok,
                    now,
                )
            )
            multiview_fused = metric_target_fusion.fuse_multiview(
                self.multiview_target_estimate,
                now,
            )
            if multiview_fused and metric_target_fusion.ready:
                # The passive triangulation path has its own temporal range
                # warm-up and covariance gate. Re-read the EKF state so the
                # same accepted frame can initialize/prestream the true target
                # without waiting for MiDaS/M52 to recover.
                multiview_metric_estimate = metric_target_fusion.predict(now)
                if multiview_metric_estimate.valid:
                    metric_target_estimate = multiview_metric_estimate
        provisional_states = {
            FollowWorkflowState.ACQUIRING_RANGE,
            FollowWorkflowState.TARGET_INITIALIZED,
            FollowWorkflowState.PRESTREAMING_TARGET,
            FollowWorkflowState.REQUESTING_FOLLOW_MODE,
            FollowWorkflowState.FOLLOWING,
            FollowWorkflowState.DEGRADED,
        }
        provisional_target_estimate = TargetEstimate(
            timestamp_s=now,
            state="PROVISIONAL",
            valid=False,
            reason="waiting_for_bbox_stability",
        )
        if (
            workflow_enabled
            and self.follow_workflow.state in provisional_states
        ):
            provisional_target_estimate = (
                self._update_provisional_target_locked(
                    frame,
                    result,
                    now,
                    gimbal_command_ok,
                )
            )
        if workflow_enabled:
            target_estimate = self._select_follow_target_estimate_locked(
                provisional_target_estimate,
                metric_target_estimate,
                now,
            )
        else:
            target_estimate = self._update_bearing_target_locked(
                frame,
                result,
                gimbal_command_ok,
                now,
            )
            self.provisional_target_source = "bearing_triangulation"
        self.target_estimate = target_estimate
        self.target_estimator_error = (
            "" if target_estimate.valid else target_estimate.reason
        )
        self.follow_distance_m = target_estimate.range_m
        self.follow_distance_source = self.provisional_target_source

        if workflow_enabled and not self.visual_follow_requested:
            if (
                self.follow_workflow.state
                == FollowWorkflowState.ACQUIRING_RANGE
                and provisional_target_estimate.valid
            ):
                self.follow_workflow.transition(
                    FollowWorkflowState.TARGET_INITIALIZED,
                    "selection_anchor_initialized",
                    timestamp_s=now,
                )
                self.visual_follow_ready = True
            if not self.visual_follow_requested:
                self.visual_follow_state = (
                    self.follow_workflow.state.value.lower()
                )
                self.visual_follow_error = (
                    ""
                    if (
                        self.follow_workflow.state
                        == FollowWorkflowState.TARGET_INITIALIZED
                        and self.target_estimate.valid
                    )
                    else self.target_estimate.reason
                )
                return
        # Apparent-size TTC gates radial Follow motion. RGB bootstrap is a
        # corridor-authorized lateral translation, so scale noise there must
        # not masquerade as a forward-collision command.
        if (
            estimate.reason == "ttc_block"
            and (
                not workflow_enabled
                or self.follow_workflow.state
                != FollowWorkflowState.RGB_BOOTSTRAP
            )
        ):
            self.visual_follow_state = "ttc_block"
            self.visual_follow_error = "Forward motion blocked by apparent-size TTC"
            self._stop_visual_follow_locked("ttc_block")
            self.follow_workflow.transition(
                FollowWorkflowState.HOLD,
                "ttc_block",
                timestamp_s=now,
                force=True,
            )
            return
        if not target_estimate.valid:
            if (
                workflow_enabled
                and self.follow_workflow.state
                != FollowWorkflowState.RGB_BOOTSTRAP
            ):
                grace_states = {
                    FollowWorkflowState.TARGET_3D_READY,
                    FollowWorkflowState.FOLLOW_PRESTREAM,
                    FollowWorkflowState.FOLLOW_MODE_REQUESTED,
                    FollowWorkflowState.FOLLOWING,
                    FollowWorkflowState.DEGRADED,
                }
                if self.follow_workflow.state in grace_states:
                    grace_timeout_s = getattr(
                        self,
                        "visual_follow_dropout_grace_s",
                        1.50,
                    )
                    if self.follow_estimate_invalid_since_monotonic is None:
                        self.follow_estimate_invalid_since_monotonic = now
                        if self.follow_workflow.state in {
                            FollowWorkflowState.FOLLOWING,
                            FollowWorkflowState.DEGRADED,
                        }:
                            self.follow_workflow.transition(
                                FollowWorkflowState.DEGRADED,
                                f"target_estimate_{target_estimate.reason}",
                                timestamp_s=now,
                                timeout_s=grace_timeout_s,
                                force=True,
                            )
                    if (
                        now - self.follow_estimate_invalid_since_monotonic
                        <= grace_timeout_s
                    ):
                        self.visual_follow_state = "degraded"
                        self.visual_follow_error = (
                            "Target estimate grace: "
                            f"{target_estimate.reason}"
                        )
                        return
                self._abort_follow_workflow_locked(
                    f"target_estimate_{target_estimate.reason}",
                    now,
                )
                return
            self.visual_follow_state = (
                "rgb_bootstrap"
                if workflow_enabled
                else target_estimate.state.lower()
            )
            self.visual_follow_error = (
                "RGB bootstrap: "
                f"{target_estimate.reason}"
            )
            expected_geometry_reasons = {
                "insufficient_observations",
                "insufficient_baseline",
                "weak_intersection_angle",
                "degenerate_geometry",
                "too_many_outliers",
                "range_uncertainty",
            }
            if (
                workflow_enabled
                and target_estimate.reason not in expected_geometry_reasons
            ):
                self._abort_follow_workflow_locked(
                    f"bootstrap_{target_estimate.reason}",
                    now,
                )
                return
            if (
                workflow_enabled
                and
                self.follow_workflow.timeout_s is not None
                and now - self.follow_workflow.entered_timestamp_s
                >= self.follow_workflow.timeout_s
            ):
                self._stop_visual_follow_locked("bootstrap_timeout")
                self.follow_workflow.transition(
                    FollowWorkflowState.HOLD,
                    "bootstrap_timeout",
                    timestamp_s=now,
                    force=True,
                )
                return
            if workflow_enabled:
                self._publish_rgb_bootstrap_locked(
                    result,
                    now,
                    gimbal_command_ok,
                )
            return
        self.follow_estimate_invalid_since_monotonic = None
        if (
            workflow_enabled
            and self.follow_workflow.state == FollowWorkflowState.DEGRADED
        ):
            self.follow_workflow.transition(
                FollowWorkflowState.FOLLOWING,
                "target_estimate_recovered",
                timestamp_s=now,
            )
        if (
            workflow_enabled
            and self.follow_workflow.state == FollowWorkflowState.RGB_BOOTSTRAP
        ):
            self._stop_bootstrap_motion_locked("target_3d_ready")
            if not self._save_initial_safe_distance_locked(target_estimate):
                self._abort_follow_workflow_locked(
                    "initial_safe_distance_invalid",
                    now,
                )
                return
            self.follow_workflow.transition(
                FollowWorkflowState.TARGET_3D_READY,
                "bearing_geometry_and_d0_valid",
                timestamp_s=now,
            )
            self.visual_follow_state = "target_3d_ready"
            self.visual_follow_error = ""
            # Keep the same valid triangulation sample for the first native
            # target publication.  Returning here creates a one-frame gap
            # between stopping bootstrap Offboard motion and starting Follow
            # Target prestream; any authority/gimbal boundary in that gap can
            # clear the explicit user authorization and strand the workflow in
            # TARGET_3D_READY with no target stream.
        if workflow_enabled:
            self.visual_follow_state = self.follow_workflow.state.value.lower()
        if self.visual_target_command is None or self.pose_provider is None:
            self.visual_follow_error = "Visual target callbacks are unavailable"
            self._stop_visual_follow_at_boundary_locked("unavailable", now)
            return

        source_kind = str(self.provisional_target_source).strip().lower()
        estimate_state = str(target_estimate.state).strip().upper()
        unsafe_source_tokens = (
            "selection",
            "provisional",
            "apparent",
            "bootstrap",
        )
        if (
            estimate_state in {"PROVISIONAL", "METRIC_HANDOVER"}
            or not source_kind
            or any(token in source_kind for token in unsafe_source_tokens)
            or not bool(
                getattr(self, "provisional_target_metric_confirmed", False)
            )
        ):
            self._abort_follow_workflow_locked(
                "measured_target_required",
                now,
            )
            return

        try:
            pose = self.pose_provider(
                self.drone_id,
                self.current_source_timestamp_s,
            )
        except TypeError:
            pose = self.pose_provider(self.drone_id)
        if not pose.get("available"):
            self.visual_follow_error = str(
                pose.get("error", "Visual target pose is unavailable")
            )
            self._abort_follow_workflow_locked("target_pose_invalid", now)
            return
        bridge_status = pose.get("visual_follow_bridge", {})
        self.visual_follow_bridge_status = (
            dict(bridge_status)
            if isinstance(bridge_status, dict)
            else {}
        )
        self.visual_follow_bridge_status["manual_loss_grace_state"] = str(
            pose.get("manual_loss_grace_state", "INACTIVE")
        )
        self.visual_follow_bridge_status["manual_loss_grace_age_ms"] = (
            pose.get("manual_loss_grace_age_ms")
        )
        parameter_ack = str(
            self.visual_follow_bridge_status.get(
                "parameter_ack",
                "not_reported",
            )
        )
        mode_ack = str(
            self.visual_follow_bridge_status.get(
                "mode_request_ack",
                "not_reported",
            )
        )
        if parameter_ack in {"rejected", "timeout"}:
            self._abort_follow_workflow_locked(
                f"px4_parameter_ack_{parameter_ack}",
                now,
            )
            return
        if mode_ack == "rejected":
            self._abort_follow_workflow_locked(
                "px4_follow_mode_ack_rejected",
                now,
            )
            return
        try:
            follower_ned = _finite_position(
                pose.get("vehicle_position_ned_m")
            )
            global_position = tuple(
                float(value)
                for value in pose["synchronized_global_position"]
            )
            if follower_ned is None or len(global_position) != 3:
                raise ValueError("synchronized follower pose is incomplete")
            target_ned = target_estimate.position_ned_m
            target_velocity = target_estimate.velocity_ned_m_s
            if target_ned is None or target_velocity is None:
                raise ValueError("bearing target state is incomplete")
            if not target_estimate.velocity_valid:
                target_velocity = (0.0, 0.0, 0.0)
            target_global = ned_target_to_wgs84(
                follower_lat_deg=global_position[0],
                follower_lon_deg=global_position[1],
                follower_alt_msl_m=global_position[2],
                follower_position_ned=follower_ned,
                target_position_ned=target_ned,
            )
        except (AssertionError, KeyError, TypeError, ValueError) as error:
            self.visual_follow_error = f"Visual target projection failed: {error}"
            self._abort_follow_workflow_locked("projection_invalid", now)
            return

        self.visual_target_ned = target_ned
        self.visual_target_velocity_ned = target_velocity
        self.visual_target_global = target_global
        self.current_estimated_horizontal_distance_m = math.hypot(
            float(target_ned[0]) - follower_ned[0],
            float(target_ned[1]) - follower_ned[1],
        )
        # Capture the selected target-to-vehicle azimuth with D0 so entering
        # native Follow does not command an immediate orbital relocation.
        # This changes only FLW_TGT_FA; the real target coordinate is never
        # translated to manufacture the stand-off distance.
        follow_angle_deg = (
            self.initial_follow_angle_deg
            if self.initial_follow_angle_deg is not None
            else self.follow_relative_angle_deg
        )
        if (
            self.visual_follow_last_publish_monotonic is not None
            and now - self.visual_follow_last_publish_monotonic < 0.10
        ):
            return
        if self.initial_safe_distance_horizontal_m is None:
            self._abort_follow_workflow_locked(
                "initial_safe_distance_missing",
                now,
            )
            return
        covariance = target_estimate.covariance
        position_covariance = (
            [
                max(0.0, float(covariance[index][index]))
                for index in range(3)
            ]
            if len(covariance) >= 3
            and all(len(covariance[index]) >= 3 for index in range(3))
            else [100.0, 100.0, 100.0]
        )
        estimate_capabilities = 1 | (
            2 if target_estimate.velocity_valid else 0
        )
        response = self.visual_target_command(
            self.drone_id,
            True,
            {
                "latitude_deg": target_global[0],
                "longitude_deg": target_global[1],
                "altitude_msl_m": target_global[2],
                "velocity_north_m_s": target_velocity[0],
                "velocity_east_m_s": target_velocity[1],
                "velocity_down_m_s": target_velocity[2],
                "distance_m": target_estimate.range_m,
                "follow_distance_m": (
                    self.initial_safe_distance_horizontal_m
                ),
                "follow_height_m": max(
                    self.follow_min_altitude_m,
                    -follower_ned[2],
                ),
                "follow_angle_deg": follow_angle_deg,
                "quality": self._target_estimate_quality(target_estimate),
                "position_covariance": position_covariance,
                "est_capabilities": estimate_capabilities,
                "ttc_s": estimate.ttc_s,
                "measurement_timestamp_s": target_estimate.timestamp_s,
                "estimate_age_ms": target_estimate.estimate_age_ms,
                "session_id": self.session_id,
                "source_kind": source_kind,
                "target_semantics": "measured_target",
            },
            state,
        )
        self.visual_follow_last_publish_monotonic = now
        self.visual_target_stream_active = bool(response.get("ok"))
        self.visual_follow_error = str(response.get("error", ""))
        if not self.visual_target_stream_active:
            self._abort_follow_workflow_locked(
                self.visual_follow_error or "target_prestream_rejected",
                now,
            )
            return
        bridge_prestream_count = self.visual_follow_bridge_status.get(
            "prestream_count"
        )
        bridge_prestream_required = self.visual_follow_bridge_status.get(
            "prestream_required"
        )
        bridge_prestream_confirmed = False
        try:
            if bridge_prestream_count is not None:
                self.follow_prestream_count = max(
                    0,
                    int(bridge_prestream_count),
                )
                required_count = max(
                    self.follow_prestream_min_messages,
                    int(bridge_prestream_required),
                )
                bridge_prestream_confirmed = bool(
                    self.follow_prestream_count >= required_count
                )
        except (TypeError, ValueError):
            bridge_prestream_confirmed = False
        if self.follow_workflow.state == FollowWorkflowState.TARGET_3D_READY:
            self.follow_workflow.transition(
                FollowWorkflowState.FOLLOW_PRESTREAM,
                "target_stream_started",
                timestamp_s=now,
            )
        if (
            self.follow_workflow.state == FollowWorkflowState.FOLLOW_PRESTREAM
            and bridge_prestream_confirmed
        ):
            self.follow_mode_requested_monotonic = now
            self.follow_mode_request_attempts = 1
            self.follow_workflow.transition(
                FollowWorkflowState.FOLLOW_MODE_REQUESTED,
                "target_prestream_complete",
                timestamp_s=now,
                timeout_s=self.follow_mode_confirmation_timeout_s,
            )
        nav_state = int(pose.get("px4_nav_state", -1))
        self.px4_nav_state = nav_state
        if px4_follow_mode_confirmed(nav_state, mode_ack):
            if self.follow_workflow.state in {
                FollowWorkflowState.TARGET_3D_READY,
                FollowWorkflowState.FOLLOW_PRESTREAM,
            }:
                self.follow_mode_requested_monotonic = now
                self.follow_mode_request_attempts = max(
                    1,
                    self.follow_mode_request_attempts,
                )
                self.follow_workflow.transition(
                    FollowWorkflowState.FOLLOW_MODE_REQUESTED,
                    "px4_follow_mode_observed_after_prestream",
                    timestamp_s=now,
                    timeout_s=self.follow_mode_confirmation_timeout_s,
                    force=True,
                )
            if self.follow_workflow.state != FollowWorkflowState.FOLLOWING:
                self.follow_workflow.transition(
                    FollowWorkflowState.FOLLOWING,
                    "px4_nav_state_19_confirmed",
                    timestamp_s=now,
                    force=True,
                )
            self.visual_follow_active = True
        else:
            self.visual_follow_active = False
            try:
                self.follow_mode_request_attempts = max(
                    self.follow_mode_request_attempts,
                    int(
                        self.visual_follow_bridge_status.get(
                            "mode_request_attempts",
                            0,
                        )
                    ),
                )
            except (TypeError, ValueError):
                pass
            if (
                self.follow_workflow.state
                == FollowWorkflowState.FOLLOW_MODE_REQUESTED
                and self.follow_mode_requested_monotonic is not None
            ):
                elapsed = now - self.follow_mode_requested_monotonic
                self.follow_mode_request_attempts = min(
                    3,
                    1 + int(elapsed / 1.0),
                )
                if elapsed >= self.follow_mode_confirmation_timeout_s:
                    self._abort_follow_workflow_locked(
                        "px4_follow_mode_confirmation_timeout",
                        now,
                    )
                    return
        self.visual_follow_state = self.follow_workflow.state.value.lower()

    def _update_visual_follow_safely_locked(
        self,
        frame: Any,
        result: Any,
        now: float,
        gimbal_command_ok: bool,
    ) -> None:
        """Isolate target geolocation/follow faults from the pointing loop."""

        try:
            self._update_visual_follow_locked(
                frame,
                result,
                now,
                gimbal_command_ok,
            )
        except (
            ArithmeticError,
            AttributeError,
            KeyError,
            RuntimeError,
            TypeError,
            ValueError,
        ) as error:
            reason = f"follow_workflow_error:{type(error).__name__}"
            self.target_estimator_error = f"{reason}:{error}"
            self.visual_follow_error = (
                "Target geolocation/follow degraded; pointing remains active: "
                f"{error}"
            )
            self.visual_follow_ready = False
            self.visual_follow_state = "degraded"
            if self.visual_follow_requested or self.visual_follow_active:
                self._stop_visual_follow_at_boundary_locked(reason, now)

    def _disable_body_yaw_locked(self, drone_id: str) -> None:
        should_publish = bool(
            drone_id
            and self.body_yaw_command is not None
            and (
                self.body_yaw_active
                or self.body_yaw_latched
                or abs(self.body_yaw_command_normalized) > 1e-6
            )
        )
        if should_publish:
            try:
                self.body_yaw_command(
                    drone_id,
                    False,
                    0.0,
                    self.body_yaw_filtered_deg,
                    "inactive",
                )
            except Exception:
                pass
        self.body_yaw_active = False
        self.body_yaw_latched = False
        self.body_yaw_filtered_deg = 0.0
        self.body_yaw_command_normalized = 0.0
        self.body_yaw_last_publish_monotonic = None
        self.body_yaw_error = ""

    def _stop_follow_locked(self, drone_id: str, state: str) -> None:
        should_publish = bool(
            drone_id
            and self.follow_command is not None
            and (
                self.follow_active
                or abs(self.follow_command_forward) > 1e-6
            )
        )
        if should_publish:
            try:
                self.follow_command(
                    drone_id,
                    False,
                    0.0,
                    self.follow_distance_m,
                    self.follow_distance_target_m,
                    self.follow_distance_source,
                    state,
                )
            except Exception:
                pass
        self.follow_active = False
        self.follow_command_forward = 0.0
        self.follow_last_publish_monotonic = None
        self.follow_state = state

    def _reset_follow_locked(self, drone_id: str) -> None:
        self._stop_follow_locked(drone_id, "idle")
        self.follow_distance_m = None
        self.follow_lidar_distance_m = None
        self.follow_distance_source = "none"
        self.follow_reference_area = None
        self.follow_reference_distance_m = None
        self.follow_reference_scale_px = None
        self.follow_reference_bbox_scale_px = None
        self.follow_filtered_scale_px = None
        self.follow_scale_ratio = None
        self.follow_bbox_scale_ratio = None
        self.follow_forward_blocked_by_bbox_guard = False
        self.follow_scale_error = 0.0
        self.follow_scale_error_rate = 0.0
        self.follow_object_scale_source = "none"
        self.follow_reference_scale_source = "none"
        self.follow_reference_warmup_until_monotonic = None
        self.follow_previous_scale_error = None
        self.follow_previous_scale_error_monotonic = None
        self.follow_previous_distance_m = None
        self.follow_previous_distance_monotonic = None
        self.follow_previous_depth_frame_index = 0
        self.follow_radial_velocity_m_s = 0.0
        self.follow_error = ""

    def _stop_motion_locked(self, drone_id: str, state: str) -> None:
        should_publish = bool(
            drone_id
            and self.motion_command is not None
            and self.motion_last_publish_monotonic is not None
        )
        if should_publish:
            try:
                self.motion_command(
                    drone_id,
                    False,
                    0.0,
                    0.0,
                    0.0,
                    self.follow_distance_m,
                    self.follow_distance_target_m,
                    self.follow_distance_source,
                    state,
                )
            except Exception:
                pass
        if (
            drone_id
            and self.body_attitude_enabled
            and self.attitude_command is not None
            and self.motion_last_publish_monotonic is not None
        ):
            try:
                self.attitude_command(drone_id, False, 0.0, state)
            except Exception:
                pass
        self.motion_active = False
        self.motion_state = state
        self.motion_forward_velocity_m_s = 0.0
        self.motion_down_velocity_m_s = 0.0
        self.motion_yaw_rate_deg_s = 0.0
        self.motion_requested_yaw_rate_deg_s = 0.0
        self.motion_combined_yaw_error_deg = 0.0
        self.yaw_authority = "none"
        self.yaw_block_reason = state
        self.gimbal_recenter_state = "blocked"
        self.motion_hold_z_down_m = None
        self.motion_vertical_pitch_error_deg = 0.0
        self.motion_vertical_recenter_active = False
        self.motion_error = ""
        self.motion_last_publish_monotonic = None
        self.motion_invalid_since_monotonic = None
        self.body_attitude_active = False
        self.body_attitude_state = state
        self.body_attitude_rates_deg_s = {
            "roll": 0.0,
            "pitch": 0.0,
            "yaw": 0.0,
        }
        self.body_attitude_thrust = 0.0
        self.body_recenter_velocity_m_s = {
            "forward": 0.0,
            "right": 0.0,
            "down": 0.0,
        }
        self.body_recenter_acceleration_m_s2 = {
            "forward": 0.0,
            "right": 0.0,
        }
        self.body_desired_tilt_deg = {"roll": 0.0, "pitch": 0.0}
        self.body_altitude_error_m = 0.0
        self.body_altitude_guard_active = False
        self.body_hold_z_down_m = None

    def _update_native_precenter_yaw_locked(
        self,
        result: Any,
        now: float,
        gimbal_command_ok: bool,
    ) -> None:
        state = getattr(result.state, "value", str(result.state))
        score = float(getattr(result, "score", 0.0))
        valid = bool(
            self.motion_command is not None
            and state == "tracking"
            and score >= self.body_yaw_min_score
            and gimbal_command_ok
            and not self.gimbal_error
            and result.bbox is not None
        )
        if not valid:
            self.body_yaw_recenter_controller.update(
                gimbal_yaw_deg=float(
                    self.gimbal_angles_deg.get("yaw", 0.0)
                ),
                bbox_horizontal_error_deg=self.body_yaw_image_error_deg,
                tracking_valid=False,
                dt_s=1.0 / 30.0,
            )
            self._stop_motion_locked(self.drone_id, "precenter_invalid")
            return
        previous = self.motion_last_publish_monotonic
        dt = (
            1.0 / 20.0
            if previous is None
            else max(0.001, min(0.1, now - previous))
        )
        yaw_output = self.body_yaw_recenter_controller.update(
            gimbal_yaw_deg=float(self.gimbal_angles_deg.get("yaw", 0.0)),
            bbox_horizontal_error_deg=self.body_yaw_image_error_deg,
            tracking_valid=True,
            dt_s=dt,
            gimbal_fresh=bool(
                getattr(self, "gimbal_feedback_available", True)
            ),
        )
        yaw_rate = yaw_output.limited_rate_deg_s
        self.gimbal_yaw_recenter_latched = yaw_output.active
        self.gimbal_recenter_state = yaw_output.state
        self.body_yaw_recenter_filtered_deg = (
            yaw_output.filtered_gimbal_yaw_deg
        )
        self.body_yaw_recenter_enter_elapsed_s = yaw_output.enter_elapsed_s
        self.motion_combined_yaw_error_deg = yaw_output.combined_error_deg
        self.motion_requested_yaw_rate_deg_s = yaw_rate
        self.yaw_authority = (
            "native_precenter" if yaw_output.active else "gimbal_only"
        )
        self.yaw_block_reason = yaw_output.reason
        if abs(yaw_rate) < 1e-6:
            self._stop_motion_locked(self.drone_id, "centered_wait")
            return
        if (
            self.motion_last_publish_monotonic is not None
            and now - self.motion_last_publish_monotonic < 0.045
        ):
            return
        response = self.motion_command(
            self.drone_id,
            True,
            0.0,
            yaw_rate,
            0.0,
            self.follow_distance_m,
            self.follow_distance_target_m,
            self.follow_distance_source,
            "precentering_yaw",
        )
        self.motion_last_publish_monotonic = now
        self.motion_active = bool(response.get("ok"))
        self.motion_state = (
            "precentering_yaw" if self.motion_active else "blocked"
        )
        self.motion_error = str(response.get("error", ""))
        self.motion_forward_velocity_m_s = 0.0
        self.motion_down_velocity_m_s = 0.0
        self.motion_yaw_rate_deg_s = yaw_rate if self.motion_active else 0.0
        hold_z = response.get("hold_z_down_m")
        self.motion_hold_z_down_m = (
            float(hold_z) if hold_z is not None else None
        )

    def _tracking_motion_gate_locked(
        self,
        result: Any,
        gimbal_command_ok: bool,
        minimum_score: float,
        *,
        require_bbox_stable: bool = True,
    ) -> tuple[bool, str]:
        state = getattr(result.state, "value", str(result.state))
        bbox = getattr(result, "bbox", None)
        if bbox is None:
            return False, "no_bbox"
        if state != "tracking":
            return False, (
                "tracker_redetecting"
                if bool(getattr(result, "redetecting", False))
                else "tracker_lost"
            )
        if bool(getattr(result, "redetecting", False)):
            return False, "tracker_redetecting"
        diagnostics = (
            self._tracker_diagnostics_locked()
            if hasattr(self, "tracker")
            else {}
        )
        if bool(diagnostics.get("ambiguous", False)):
            return False, "tracker_ambiguous"
        if diagnostics.get("reject_reason", ""):
            return False, "tracker_redetecting"
        if float(getattr(result, "score", 0.0)) < minimum_score:
            return False, "low_tracker_score"
        if require_bbox_stable:
            bbox_filter = getattr(self, "visual_bbox_filter", None)
            stable_required = int(
                getattr(bbox_filter, "stable_frames_required", 0)
            )
            if (
                getattr(self, "visual_bbox_stable_frames", stable_required)
                < stable_required
            ):
                return False, "bbox_stabilizing"
        if not gimbal_command_ok or self.gimbal_error:
            return False, "motion_safety_blocked"
        return True, ""

    def _update_motion_locked(
        self,
        result: Any,
        now: float,
        gimbal_command_ok: bool,
    ) -> None:
        if self.motion_command is None:
            self.follow_block_reason = "motion_safety_blocked"
            self.yaw_authority = "none"
            self.yaw_block_reason = "motion_safety_blocked"
            self._stop_motion_locked(self.drone_id, "disabled")
            return
        state = getattr(result.state, "value", str(result.state))
        score = float(getattr(result, "score", 0.0))
        yaw_tracking_valid, yaw_block_reason = (
            self._tracking_motion_gate_locked(
                result,
                gimbal_command_ok,
                self.body_yaw_min_score,
                require_bbox_stable=False,
            )
        )
        if not yaw_tracking_valid:
            self.body_yaw_recenter_controller.update(
                gimbal_yaw_deg=float(
                    getattr(self, "gimbal_angles_deg", {}).get("yaw", 0.0)
                ),
                bbox_horizontal_error_deg=getattr(
                    self, "body_yaw_image_error_deg", 0.0
                ),
                tracking_valid=False,
                dt_s=1.0 / 30.0,
            )
            self.follow_block_reason = yaw_block_reason
            self.yaw_authority = "none"
            self.yaw_block_reason = yaw_block_reason
            self.gimbal_recenter_state = "blocked"
            self.motion_requested_yaw_rate_deg_s = 0.0
            self._stop_motion_locked(self.drone_id, yaw_block_reason)
            return
        self.motion_invalid_since_monotonic = None

        previous_motion = self.motion_last_publish_monotonic
        motion_dt = (
            1.0 / 30.0
            if previous_motion is None
            else max(0.001, min(0.1, now - previous_motion))
        )
        gimbal_yaw_deg = float(
            getattr(self, "gimbal_angles_deg", {}).get("yaw", 0.0)
        )
        yaw_output = self.body_yaw_recenter_controller.update(
            gimbal_yaw_deg=gimbal_yaw_deg,
            bbox_horizontal_error_deg=getattr(
                self, "body_yaw_image_error_deg", 0.0
            ),
            tracking_valid=True,
            dt_s=motion_dt,
            gimbal_fresh=bool(
                getattr(self, "gimbal_feedback_available", True)
            ),
        )
        yaw_rate_target_deg_s = yaw_output.limited_rate_deg_s
        self.motion_combined_yaw_error_deg = yaw_output.combined_error_deg
        self.motion_requested_yaw_rate_deg_s = yaw_rate_target_deg_s
        self.body_yaw_recenter_filtered_deg = (
            yaw_output.filtered_gimbal_yaw_deg
        )
        self.body_yaw_recenter_enter_elapsed_s = yaw_output.enter_elapsed_s
        self.gimbal_yaw_recenter_latched = yaw_output.active
        self.yaw_authority = (
            "unified_offboard_follow"
            if yaw_output.active
            else "gimbal_only"
        )
        self.yaw_block_reason = yaw_output.reason
        self.gimbal_recenter_state = yaw_output.state
        forward_tracking_valid, forward_block_reason = (
            self._tracking_motion_gate_locked(
                result,
                gimbal_command_ok,
                self.follow_min_score,
            )
        )

        # Use one PX4 motion authority for all three body axes. Apparent-size
        # follow still supplies the normalized forward command, while the
        # attitude-recenter controller combines it with filtered gimbal
        # roll/pitch/yaw home errors. Previously this path was skipped in
        # apparent-size mode, so only yaw and vertical velocity recentered.
        if (
            self.body_attitude_enabled
            and self.apparent_size_follow_enabled
        ):
            if self.attitude_command is None:
                self._stop_motion_locked(self.drone_id, "attitude_unavailable")
                self.body_attitude_error = (
                    "Body attitude callback is unavailable"
                )
                return
            last_publish = self.motion_last_publish_monotonic
            if last_publish is not None and now - last_publish < 0.045:
                return
            response = self.attitude_command(
                self.drone_id,
                True,
                self.follow_command_forward / max(0.01, self.follow_max),
                state,
                bbox_image_error_deg={
                    "yaw": self.body_yaw_image_error_deg,
                    "pitch": self.body_pitch_image_error_deg,
                },
            )
            self.motion_last_publish_monotonic = now
            self.body_attitude_active = bool(response.get("active"))
            self.body_attitude_state = str(
                response.get("state", "blocked")
            )
            self.body_attitude_rates_deg_s = dict(
                response.get(
                    "body_rates_deg_s",
                    self.body_attitude_rates_deg_s,
                )
            )
            self.body_attitude_thrust = float(response.get("thrust", 0.0))
            self.body_recenter_velocity_m_s = dict(
                response.get(
                    "body_velocity_m_s",
                    self.body_recenter_velocity_m_s,
                )
            )
            self.body_recenter_acceleration_m_s2 = dict(
                response.get(
                    "body_acceleration_m_s2",
                    self.body_recenter_acceleration_m_s2,
                )
            )
            self.body_desired_tilt_deg = dict(
                response.get(
                    "desired_body_tilt_deg",
                    self.body_desired_tilt_deg,
                )
            )
            self.body_altitude_error_m = float(
                response.get("altitude_error_m", 0.0)
            )
            self.body_altitude_guard_active = bool(
                response.get("altitude_guard_active", False)
            )
            hold_z = response.get("hold_z_down_m")
            self.body_hold_z_down_m = (
                float(hold_z) if hold_z is not None else None
            )
            self.body_attitude_error = str(response.get("error", ""))
            self.gimbal_home_error_deg = dict(
                response.get(
                    "gimbal_home_error_deg",
                    self.gimbal_home_error_deg,
                )
            )
            self.motion_active = bool(response.get("ok"))
            self.motion_state = (
                "acceleration_recentering"
                if self.motion_active
                else "blocked"
            )
            self.motion_error = self.body_attitude_error
            self.motion_forward_velocity_m_s = float(
                self.body_recenter_velocity_m_s.get("forward", 0.0)
            )
            self.motion_yaw_rate_deg_s = float(
                self.body_attitude_rates_deg_s.get("yaw", 0.0)
            )
            return

        forward_velocity_m_s = (
            self.follow_command_forward
            / max(0.01, self.follow_max)
            * self.offboard_max_forward_m_s
            * self.offboard_forward_sign
        )
        down_velocity_m_s = 0.0
        if self.apparent_size_follow_enabled:
            # Recenter the gimbal with the vehicle body: if the gimbal yaws
            # away from home, immediately yaw the drone in the same direction
            # so the gimbal can unload back toward 0 deg while tracking.
            yaw_error_deg = float(self.gimbal_angles_deg.get("yaw", 0.0))
            if self.apparent_size_yaw_invert:
                yaw_error_deg *= -1.0
            yaw_error_abs = abs(yaw_error_deg)
            if yaw_error_abs <= self.apparent_size_yaw_deadband_deg:
                yaw_rate_deg_s = 0.0
            else:
                yaw_rate_deg_s = math.copysign(
                    min(
                        self.offboard_max_yaw_rate_deg_s,
                        self.apparent_size_yaw_kp
                        * (
                            yaw_error_abs
                            - self.apparent_size_yaw_deadband_deg
                        ),
                    ),
                    yaw_error_deg,
                )
            pitch_error_deg = (
                float(self.gimbal_angles_deg.get("pitch", 0.0))
                - self.gimbal_home_pitch_deg
            )
            if self.vertical_recenter_invert:
                pitch_error_deg *= -1.0
            pitch_error_abs = abs(pitch_error_deg)
            if pitch_error_abs <= self.vertical_recenter_deadband_deg:
                target_down_velocity_m_s = 0.0
            else:
                # BODY_NED down velocity is negative for climbing. Positive
                # pitch error means the gimbal is looking above HOME, so climb.
                target_down_velocity_m_s = -math.copysign(
                    min(
                        self.vertical_recenter_max_down_m_s,
                        self.vertical_recenter_kp
                        * (
                            pitch_error_abs
                            - self.vertical_recenter_deadband_deg
                        ),
                    ),
                    pitch_error_deg,
                )
            down_delta_limit = self.vertical_recenter_slew_m_s2 * motion_dt
            down_velocity_m_s = self.motion_down_velocity_m_s + max(
                -down_delta_limit,
                min(
                    down_delta_limit,
                    target_down_velocity_m_s - self.motion_down_velocity_m_s,
                ),
            )
        else:
            pitch_error_deg = 0.0
        yaw_rate_deg_s = yaw_rate_target_deg_s
        if yaw_output.active:
            forward_velocity_m_s = 0.0
            motion_state = "recentering_gimbal_yaw"
            self.follow_block_reason = "recentering_gimbal_yaw"
        elif abs(self.body_yaw_image_error_deg) > self.follow_align_yaw_deg:
            forward_velocity_m_s = 0.0
            motion_state = "aligning_yaw"
            self.follow_block_reason = "aligning_yaw"
        elif self.follow_state in {
            "moving_forward",
            "moving_backward",
            "braking",
        }:
            motion_state = self.follow_state
            self.follow_block_reason = ""
        elif abs(forward_velocity_m_s) > 1e-3:
            motion_state = "following"
        elif abs(down_velocity_m_s) > 1e-3:
            motion_state = "vertical_recentering"
        elif abs(yaw_rate_deg_s) > 1e-3:
            motion_state = "yawing"
        else:
            motion_state = "holding"
            self.follow_block_reason = "holding_safe_band"

        last_publish = self.motion_last_publish_monotonic
        if (
            last_publish is not None
            and now - last_publish < 0.045
        ):
            return
        response = self.motion_command(
            self.drone_id,
            True,
            forward_velocity_m_s,
            yaw_rate_deg_s,
            down_velocity_m_s,
            self.follow_distance_m,
            self.follow_distance_target_m,
            self.follow_distance_source,
            motion_state,
        )
        self.motion_last_publish_monotonic = now
        self.motion_active = bool(response.get("ok"))
        self.motion_error = str(response.get("error", ""))
        self.motion_state = motion_state if self.motion_active else "blocked"
        self.motion_forward_velocity_m_s = (
            forward_velocity_m_s if self.motion_active else 0.0
        )
        self.motion_down_velocity_m_s = (
            down_velocity_m_s if self.motion_active else 0.0
        )
        self.motion_yaw_rate_deg_s = (
            yaw_rate_deg_s if self.motion_active else 0.0
        )
        hold_z = response.get("hold_z_down_m")
        self.motion_hold_z_down_m = (
            float(hold_z) if hold_z is not None else None
        )
        self.motion_vertical_pitch_error_deg = pitch_error_deg
        self.motion_vertical_recenter_active = bool(
            self.motion_active and abs(down_velocity_m_s) > 1e-3
        )

    def _update_follow_locked(
        self,
        result: Any,
        now: float,
        dt: float,
        gimbal_command_ok: bool,
    ) -> None:
        if not self.follow_enabled or self.follow_command is None:
            self._stop_follow_locked(self.drone_id, "disabled")
            return
        state = getattr(result.state, "value", str(result.state))
        score = float(getattr(result, "score", 0.0))
        if (
            state != "tracking"
            or score < self.follow_min_score
            or not gimbal_command_ok
            or result.bbox is None
        ):
            self._stop_follow_locked(self.drone_id, state)
            return

        frame_height, frame_width = self.latest_raw_frame.shape[:2]
        width_cal = (
            float(result.bbox.w)
            * self.camera_reference_width
            / frame_width
        )
        height_cal = (
            float(result.bbox.h)
            * self.camera_reference_height
            / frame_height
        )
        bbox_scale_px = math.sqrt(max(36.0, width_cal * height_cal))
        self.follow_lidar_distance_m = None
        distance_error_is_metric = False

        if self.apparent_size_follow_enabled:
            self.follow_distance_source = "apparent_size"
            if self.follow_reference_scale_px is None:
                self.follow_reference_scale_px = bbox_scale_px
            if self.follow_reference_bbox_scale_px is None:
                self.follow_reference_bbox_scale_px = bbox_scale_px
            object_scale_px, object_scale_source = object_pixel_scale_px(
                self.latest_raw_frame,
                result.bbox,
            )
            scale_normalizer = math.sqrt(
                self.camera_reference_width
                * self.camera_reference_height
                / max(1.0, float(frame_width * frame_height))
            )
            scale_px = max(6.0, object_scale_px * scale_normalizer)
            self.follow_object_scale_source = object_scale_source
            alpha = self.follow_ema_alpha
            self.follow_filtered_scale_px = (
                scale_px
                if self.follow_filtered_scale_px is None
                else (
                    (1.0 - alpha) * self.follow_filtered_scale_px
                    + alpha * scale_px
                )
            )
            reference_scale = max(1.0, self.follow_reference_scale_px)
            current_scale = max(1.0, self.follow_filtered_scale_px)
            bbox_reference_scale = max(1.0, self.follow_reference_bbox_scale_px)
            self.follow_bbox_scale_ratio = bbox_scale_px / bbox_reference_scale
            warmup_active = bool(
                self.follow_reference_warmup_until_monotonic is not None
                and now < self.follow_reference_warmup_until_monotonic
            )
            fallback_scale = object_scale_source == "bbox_fallback"
            object_scale_reliable = bool(
                not fallback_scale
                and self.follow_reference_scale_source
                in {"none", object_scale_source}
            )
            if current_scale > reference_scale and (
                warmup_active or fallback_scale
            ):
                reference_scale = current_scale
                self.follow_reference_scale_px = reference_scale
                if not fallback_scale:
                    self.follow_reference_scale_source = object_scale_source
            self.follow_scale_ratio = current_scale / reference_scale
            near_reference = (
                abs(self.follow_scale_ratio - 1.0)
                <= self.apparent_size_hold_ratio_tolerance
            )
            # Positive error means the target appears smaller than it did at
            # selection time, so the follower should move forward.
            object_error = apparent_size_error(reference_scale, current_scale)
            if self.follow_bbox_scale_ratio < self.bbox_forward_stop_ratio:
                distance_error = math.log(
                    self.bbox_forward_stop_ratio
                    / max(1e-6, self.follow_bbox_scale_ratio)
                )
            elif self.follow_bbox_scale_ratio > self.bbox_reverse_start_ratio:
                distance_error = -math.log(
                    self.follow_bbox_scale_ratio
                    / max(1e-6, self.bbox_reverse_start_ratio)
                )
            else:
                distance_error = 0.0

            if object_scale_reliable and distance_error != 0.0:
                distance_error += self.object_scale_blend * object_error
                # The bbox guard is the safety authority: once bbox says the
                # initial range has been reached, object pixels cannot command
                # additional forward motion.
                if (
                    self.follow_bbox_scale_ratio
                    >= self.bbox_forward_stop_ratio
                    and distance_error > 0.0
                ):
                    distance_error = 0.0

            self.follow_forward_blocked_by_bbox_guard = bool(
                self.follow_bbox_scale_ratio >= self.bbox_forward_stop_ratio
                and distance_error > 0.0
            )
            if self.follow_forward_blocked_by_bbox_guard:
                distance_error = 0.0
            self.follow_scale_error = distance_error
            previous_error = self.follow_previous_scale_error
            previous_time = self.follow_previous_scale_error_monotonic
            if previous_error is None or previous_time is None:
                scale_error_rate = 0.0
            else:
                error_dt = max(1.0 / 60.0, min(0.2, now - previous_time))
                scale_error_rate = (distance_error - previous_error) / error_dt
            self.follow_scale_error_rate = (
                0.0
                if near_reference or distance_error == 0.0
                else max(-5.0, min(5.0, scale_error_rate))
            )
            self.follow_previous_scale_error = distance_error
            self.follow_previous_scale_error_monotonic = now
            self.follow_distance_m = None
        else:
            self._stop_follow_locked(self.drone_id, "native_follow_required")
            self.follow_error = (
                "Absolute range requires bearing triangulation and PX4 "
                "Follow Target"
            )
            return

        if abs(self.body_yaw_image_error_deg) > self.follow_align_yaw_deg:
            self._stop_follow_locked(self.drone_id, "aligning")
            self.follow_error = ""
            return
        if self.apparent_size_follow_enabled and not distance_error_is_metric:
            deadband = (
                self.apparent_size_reverse_deadband
                if distance_error < 0.0
                else self.apparent_size_deadband
            )
        else:
            # Metric/LUT distance already uses the explicit safe band.
            # Applying the legacy deadband again would silently expand it.
            deadband = 0.0
        rate_error = 0.0
        if self.apparent_size_follow_enabled and not distance_error_is_metric:
            rate = self.follow_scale_error_rate
            if abs(rate) > self.apparent_size_rate_deadband:
                rate_error = math.copysign(
                    abs(rate) - self.apparent_size_rate_deadband,
                    rate,
                )
        radial_feedforward_active = bool(
            distance_error_is_metric
            and abs(distance_error) > 0.0
            and abs(self.follow_radial_velocity_m_s) >= 0.15
        )
        if (
            abs(distance_error) <= deadband
            and not distance_error_is_metric
            and not radial_feedforward_active
        ):
            self._stop_follow_locked(self.drone_id, "holding")
            self.follow_error = ""
            return

        signed_error = (
            0.0
            if abs(distance_error) <= deadband
            else math.copysign(
                max(0.0, abs(distance_error) - deadband),
                distance_error,
            )
        )
        slowband_error = max(
            1e-6,
            self.apparent_size_slowband - deadband,
        )
        proximity = min(1.0, abs(signed_error) / slowband_error)
        taper = proximity * proximity * (3.0 - 2.0 * proximity)
        gain = (
            self.apparent_size_kp
            if self.apparent_size_follow_enabled and not distance_error_is_metric
            else self.follow_kp
        )
        if distance_error_is_metric:
            _, command_target = metric_follow_command_target(
                self.follow_distance_m,
                self.follow_min_distance_m,
                self.follow_max_distance_m,
                proportional_gain=gain,
                maximum_command=self.follow_max,
                maximum_velocity_m_s=self.offboard_max_forward_m_s,
                brake_deceleration_m_s2=self.follow_brake_m_s2,
                radial_velocity_m_s=self.follow_radial_velocity_m_s,
                radial_velocity_gain=self.follow_radial_velocity_gain,
            )
        else:
            command_target = gain * signed_error
        if self.apparent_size_follow_enabled and not distance_error_is_metric:
            command_target *= taper
        if self.apparent_size_follow_enabled and not distance_error_is_metric:
            boost = max(
                -self.apparent_size_boost_max,
                min(
                    self.apparent_size_boost_max,
                    self.apparent_size_kd * rate_error,
                ),
            )
            command_target += boost * taper
        target = max(
            -self.follow_max,
            min(self.follow_max, command_target),
        )
        self.follow_command_forward = slew_toward(
            self.follow_command_forward,
            target,
            self.follow_slew,
            max(0.001, dt),
        )

        last_publish = self.follow_last_publish_monotonic
        if last_publish is not None and now - last_publish < 0.045:
            return
        response = self.follow_command(
            self.drone_id,
            True,
            self.follow_command_forward,
            self.follow_distance_m,
            self.follow_distance_target_m,
            self.follow_distance_source,
            state,
        )
        self.follow_last_publish_monotonic = now
        self.follow_active = bool(response.get("ok"))
        self.follow_error = str(response.get("error", ""))
        if not self.follow_active:
            self.follow_state = "blocked"
        elif (
            distance_error > 0.0
            and self.follow_command_forward > 1e-3
        ):
            self.follow_state = "moving_forward"
        elif (
            distance_error < 0.0
            and self.follow_command_forward < -1e-3
        ):
            self.follow_state = "moving_backward"
        elif abs(self.follow_command_forward) > 1e-3:
            self.follow_state = "braking"
        else:
            self.follow_state = "holding"
        if not self.follow_active:
            self.follow_command_forward = 0.0

    def _update_body_yaw_locked(
        self,
        result: Any,
        now: float,
        dt: float,
        gimbal_command_ok: bool,
    ) -> None:
        # Unified offboard_follow is the sole body-motion authority.  Keep the
        # legacy yaw-only publisher disabled whenever that callback exists.
        if (
            self.motion_command is not None
            or not self.body_yaw_enabled
            or self.body_yaw_command is None
        ):
            self._disable_body_yaw_locked(self.drone_id)
            return

        state = getattr(result.state, "value", str(result.state))
        score = float(getattr(result, "score", 0.0))
        valid_tracking = bool(
            state == "tracking"
            and score >= self.body_yaw_min_score
            and gimbal_command_ok
            and not self.gimbal_error
        )
        if not valid_tracking:
            self.body_yaw_target_error_deg = 0.0
            self.body_yaw_image_error_deg = 0.0
            self.body_pitch_image_error_deg = 0.0
            self._disable_body_yaw_locked(self.drone_id)
            return

        alpha = self.body_yaw_ema_alpha
        self.body_yaw_filtered_deg = (
            (1.0 - alpha) * self.body_yaw_filtered_deg
            + alpha * self.body_yaw_target_error_deg
        )
        magnitude = abs(self.body_yaw_filtered_deg)

        if self.body_yaw_latched:
            if magnitude <= self.body_yaw_exit_deg:
                self.body_yaw_latched = False
        elif magnitude >= self.body_yaw_enter_deg:
            self.body_yaw_latched = True

        if not self.body_yaw_latched:
            if self.body_yaw_active:
                self._disable_body_yaw_locked(self.drone_id)
            return

        direction = -1.0 if self.body_yaw_filtered_deg < 0.0 else 1.0
        if self.body_yaw_invert:
            direction *= -1.0
        target = direction * min(
            self.body_yaw_max,
            self.body_yaw_kp
            * max(
                0.0,
                magnitude - self.body_yaw_exit_deg,
            ),
        )
        delta_limit = self.body_yaw_slew * max(0.001, min(0.1, dt))
        delta = max(
            -delta_limit,
            min(
                delta_limit,
                target - self.body_yaw_command_normalized,
            ),
        )
        self.body_yaw_command_normalized += delta

        last_publish = self.body_yaw_last_publish_monotonic
        if last_publish is not None and now - last_publish < 0.045:
            return

        response = self.body_yaw_command(
            self.drone_id,
            True,
            self.body_yaw_command_normalized,
            self.body_yaw_filtered_deg,
            state,
        )
        self.body_yaw_last_publish_monotonic = now
        self.body_yaw_active = bool(response.get("ok"))
        self.body_yaw_error = str(response.get("error", ""))
        if not self.body_yaw_active:
            self.body_yaw_command_normalized = 0.0

    def _update_after_pointing_locked(
        self,
        frame: Any,
        result: Any,
        now: float,
        dt: float,
        gimbal_ok: bool,
    ) -> None:
        passive_dataset_collection = bool(
            getattr(
                getattr(self, "metric_target_fusion", None),
                "dataset_collection_enabled",
                False,
            )
        )
        if passive_dataset_collection:
            # Dataset collection is an observation-only path. It must keep the
            # metric pipeline running even when native Follow is disabled, and
            # it must never retain any body-motion authority.
            self._update_visual_follow_safely_locked(
                frame,
                result,
                now,
                gimbal_ok,
            )
            self._disable_body_yaw_locked(self.drone_id)
            self._stop_follow_locked(self.drone_id, "dataset_collection")
            self._stop_motion_locked(self.drone_id, "dataset_collection")
            self.relative_visual_follow_state = "dataset_collection"
            self.relative_visual_follow_block_reason = ""
            return
        if (
            self.visual_follow_feature_enabled
            and self.native_visual_follow_enabled
        ):
            self._update_visual_follow_safely_locked(
                frame,
                result,
                now,
                gimbal_ok,
            )
            workflow_state = self.follow_workflow.state
            if workflow_state in {
                FollowWorkflowState.DEGRADED,
                FollowWorkflowState.HOLD,
            }:
                # HOLD is a terminal aircraft-motion boundary for the current
                # Follow attempt.  The bbox/gimbal loop remains active, but a
                # later frame must not silently reacquire Offboard authority
                # through the ordinary body-yaw recenter path.
                self._disable_body_yaw_locked(self.drone_id)
                self._stop_follow_locked(self.drone_id, "follow_hold")
                self._stop_motion_locked(self.drone_id, "follow_hold")
                self.relative_visual_follow_state = "hold"
                self.relative_visual_follow_block_reason = (
                    getattr(
                        self.follow_workflow,
                        "reason",
                        "follow_hold",
                    )
                )
                return
            if (
                not self.visual_follow_requested
                and not self.visual_follow_active
            ):
                # Stage 1: metric geolocation stays warm in the background,
                # while bbox center and apparent scale provide immediate
                # relative motion. This branch never invents metric range or a
                # target coordinate.
                self._update_body_yaw_locked(
                    result,
                    now,
                    dt,
                    gimbal_ok,
                )
                relative_ready = bool(
                    self.apparent_size_follow_enabled
                    and self.follow_workflow.state
                    == FollowWorkflowState.ACQUIRING_RANGE
                    and getattr(
                        self,
                        "visual_bbox_filter_state",
                        "ready",
                    )
                    == "ready"
                    and getattr(
                        self,
                        "visual_motion_gate_reason",
                        "ready",
                    )
                    in {"ready", "ttc_block"}
                    and (
                        self.follow_reference_warmup_until_monotonic is None
                        or now
                        >= self.follow_reference_warmup_until_monotonic
                    )
                )
                if relative_ready:
                    self._update_follow_locked(
                        result,
                        now,
                        dt,
                        gimbal_ok,
                    )
                    if (
                        self.visual_motion_gate_reason == "ttc_block"
                        and self.follow_command_forward > 0.0
                    ):
                        # TTC only blocks motion toward the target. A reverse
                        # escape remains possible when the bbox is too large.
                        self.follow_command_forward = 0.0
                        self.follow_state = "ttc_block"
                        self.follow_block_reason = "ttc_block"
                    self.relative_visual_follow_state = "tracking"
                    self.relative_visual_follow_block_reason = (
                        self.follow_block_reason
                        if self.follow_block_reason
                        not in {"holding_safe_band", "tracking_inactive"}
                        else ""
                    )
                else:
                    if not self.apparent_size_follow_enabled:
                        reason = "relative_visual_follow_disabled"
                    elif (
                        self.follow_workflow.state
                        != FollowWorkflowState.ACQUIRING_RANGE
                    ):
                        reason = self.pointing_guard_reason
                    elif (
                        getattr(
                            self,
                            "visual_bbox_filter_state",
                            "ready",
                        )
                        != "ready"
                    ):
                        reason = str(self.visual_bbox_filter_state)
                    elif getattr(
                        self,
                        "visual_motion_gate_reason",
                        "ready",
                    ) not in {
                        "ready",
                        "ttc_block",
                    }:
                        reason = self.visual_motion_gate_reason
                    else:
                        reason = "reference_warmup"
                    self._stop_follow_locked(self.drone_id, reason)
                    self.relative_visual_follow_state = reason
                    self.relative_visual_follow_block_reason = reason
                self._update_motion_locked(result, now, gimbal_ok)
                return
            self.relative_visual_follow_state = (
                "native_follow"
                if self.follow_workflow.state
                == FollowWorkflowState.FOLLOWING
                else "metric_handover"
            )
            self.relative_visual_follow_block_reason = ""
            self._disable_body_yaw_locked(self.drone_id)
            self._stop_follow_locked(self.drone_id, "native_follow")
            if workflow_state == FollowWorkflowState.RGB_BOOTSTRAP:
                # The bootstrap callback publishes one local-NED setpoint that
                # already combines lateral translation, altitude hold and yaw.
                return
            if workflow_state in {
                FollowWorkflowState.TARGET_3D_READY,
                FollowWorkflowState.FOLLOW_PRESTREAM,
                FollowWorkflowState.FOLLOW_MODE_REQUESTED,
                FollowWorkflowState.FOLLOWING,
            }:
                self._stop_motion_locked(self.drone_id, "native_follow")
                return
            self._stop_motion_locked(self.drone_id, "follow_hold")
            return

        self._stop_visual_follow_locked("offboard_range_follow")
        self.relative_visual_follow_state = "relative_only"
        self.relative_visual_follow_block_reason = ""
        self._update_body_yaw_locked(
            result,
            now,
            dt,
            gimbal_ok,
        )
        self._update_follow_locked(
            result,
            now,
            dt,
            gimbal_ok,
        )
        self._update_motion_locked(
            result,
            now,
            gimbal_ok,
        )

    def _update_gimbal_locked(
        self,
        frame: Any,
        result: Any,
        now: float,
        *,
        update_frame_subsystems: bool = True,
    ) -> None:
        if (
            result is None
            or result.bbox is None
        ):
            if update_frame_subsystems:
                if (
                    result is not None
                    and getattr(
                        getattr(self, "metric_target_fusion", None),
                        "operational",
                        False,
                    )
                    and self.visual_follow_requested
                ):
                    self._update_visual_follow_safely_locked(
                        frame,
                        result,
                        now,
                        False,
                    )
                    if self.target_estimate.valid:
                        self.follow_block_reason = "metric_dropout_prediction"
                        self.yaw_authority = "none"
                        self.yaw_block_reason = "bbox_dropout"
                        self.gimbal_recenter_state = "predicting"
                        self._stop_motion_locked(
                            self.drone_id,
                            "native_follow",
                        )
                        return
                self.gimbal_yaw_recenter_latched = False
                self.follow_block_reason = "no_bbox"
                self.yaw_authority = "none"
                self.yaw_block_reason = "no_bbox"
                self.gimbal_recenter_state = "blocked"
                self._stop_motion_locked(self.drone_id, "inactive")
                self._stop_visual_follow_at_boundary_locked("no_bbox", now)
            return
        self.visual_follow_boundary_invalid_since_monotonic = None
        frame_height, frame_width = frame.shape[:2]
        image_fx = (
            float(self.gimbal_controller.cfg.fx)
            if self.gimbal_controller is not None
            else float(self.visual_ray_projector.fx)
        )
        image_fy = (
            float(self.gimbal_controller.cfg.fy)
            if self.gimbal_controller is not None
            else float(self.visual_ray_projector.fy)
        )
        measured_center = (
            float(result.bbox.cx),
            float(result.bbox.cy),
        )
        self.gimbal_measured_center_px = measured_center
        (
            self.body_yaw_image_error_deg,
            self.body_pitch_image_error_deg,
        ) = camera_angular_error_deg(
            center_x_px=measured_center[0],
            center_y_px=measured_center[1],
            frame_width_px=frame_width,
            frame_height_px=frame_height,
            calibrated_fx_px=image_fx,
            calibrated_fy_px=image_fy,
            calibration_width_px=self.camera_reference_width,
            calibration_height_px=self.camera_reference_height,
        )
        tracker_diagnostics = self._tracker_diagnostics_locked()
        velocity = tracker_diagnostics.get("kalman_velocity_px", (0.0, 0.0))
        velocity_x = (
            _finite_or_none(velocity[0])
            if isinstance(velocity, (list, tuple)) and len(velocity) >= 2
            else None
        )
        velocity_y = (
            _finite_or_none(velocity[1])
            if isinstance(velocity, (list, tuple)) and len(velocity) >= 2
            else None
        )
        end_to_end_latency_s = self.gimbal_prediction_latency_s + max(
            0.0,
            (
                self.frame_queue_age_ms
                + self.tracking_ms
                + self.controller_ms
            )
            / 1000.0,
        )
        predicted_x, predicted_y, prediction_horizon = predict_bbox_center_px(
            center_x_px=measured_center[0],
            center_y_px=measured_center[1],
            velocity_x_px_per_frame=float(velocity_x or 0.0),
            velocity_y_px_per_frame=float(velocity_y or 0.0),
            source_fps=max(1.0, self._source_fps_locked()),
            latency_s=end_to_end_latency_s,
            maximum_horizon_s=self.gimbal_max_prediction_horizon_s,
            frame_width_px=frame_width,
            frame_height_px=frame_height,
        )
        self.gimbal_predicted_center_px = (predicted_x, predicted_y)
        self.gimbal_prediction_horizon_s = prediction_horizon
        post_dt = max(
            0.001,
            min(0.1, 1.0 / max(1.0, self._source_fps_locked())),
        )

        if not self.gimbal_enabled:
            if not update_frame_subsystems:
                return
            # A fixed or externally-controlled gimbal is still a valid camera.
            self.gimbal_error = ""
            if bool(
                getattr(
                    getattr(self, "metric_target_fusion", None),
                    "dataset_collection_enabled",
                    False,
                )
            ):
                self._update_after_pointing_locked(
                    frame,
                    result,
                    now,
                    post_dt,
                    True,
                )
                return
            if not self.follow_enabled:
                self.follow_active = False
                self.follow_state = "disabled"
                self.follow_command_forward = 0.0
                self.motion_active = False
                self.motion_state = "disabled"
                self.motion_forward_velocity_m_s = 0.0
                self.motion_down_velocity_m_s = 0.0
                self.motion_yaw_rate_deg_s = 0.0
                self._stop_visual_follow_locked("disabled")
                return
            self._stop_visual_follow_locked("offboard_range_follow")
            self._update_body_yaw_locked(result, now, post_dt, True)
            self._update_follow_locked(result, now, post_dt, True)
            self._update_motion_locked(result, now, True)
            return

        if self.gimbal_controller is None or self.gimbal_command is None:
            if update_frame_subsystems:
                self._stop_motion_locked(self.drone_id, "gimbal_unavailable")
                self._stop_visual_follow_at_boundary_locked(
                    "gimbal_unavailable",
                    now,
                )
            return

        command_interval_s = 1.0 / self.gimbal_command_rate_hz
        command_due = (
            self.gimbal_last_update_monotonic is None
            or now - self.gimbal_last_update_monotonic
            >= command_interval_s * 0.80
        )
        if not command_due:
            if update_frame_subsystems:
                last_command = self.gimbal_last_command_monotonic
                gimbal_ok = bool(
                    not self.gimbal_error
                    and last_command is not None
                    and now - last_command <= self.gimbal_target_stale_s
                )
                self._update_after_pointing_locked(
                    frame,
                    result,
                    now,
                    post_dt,
                    gimbal_ok,
                )
            return

        previous = self.gimbal_last_update_monotonic
        dt = command_interval_s if previous is None else now - previous
        dt = max(0.001, min(0.1, dt))
        self.gimbal_last_update_monotonic = now

        try:
            # GimbalService is calibrated in camera-reference pixels. Scaling
            # the predicted center and frame together preserves the pinhole
            # angular error when the runtime stream resolution changes.
            controller_x = (
                predicted_x * self.camera_reference_width / frame_width
            )
            controller_y = (
                predicted_y * self.camera_reference_height / frame_height
            )
            omega = self.gimbal_controller.update(
                controller_x,
                controller_y,
                result.state,
                dt,
                self.camera_reference_width,
                self.camera_reference_height,
            )

            if omega is None:
                self.gimbal_yaw_recenter_latched = False
                self.body_yaw_target_error_deg = 0.0
                self.body_yaw_image_error_deg = 0.0
                self.body_pitch_image_error_deg = 0.0
                if update_frame_subsystems:
                    self._disable_body_yaw_locked(self.drone_id)
                    state = getattr(result.state, "value", str(result.state))
                    self._stop_follow_locked(self.drone_id, state)
                    self._stop_motion_locked(self.drone_id, state)
                    self._stop_visual_follow_at_boundary_locked(
                        f"tracking_{state}",
                        now,
                    )
                return

            stable_required = int(
                getattr(
                    getattr(self, "visual_bbox_filter", None),
                    "stable_frames_required",
                    0,
                )
            )
            bbox_stable = bool(
                stable_required <= 0
                or getattr(self, "visual_bbox_stable_frames", 0)
                >= stable_required
            )
            if not bbox_stable:
                # Keep reacquisition conservative; unlock the high-rate
                # profile only after the filtered bbox is stable.
                omega = omega * 0.50

            command_result = self.gimbal_command(
                self.drone_id,
                apply_gimbal_yaw_direction(
                    float(omega[0]),
                    inverted=self.gimbal_yaw_invert,
                ),
                apply_gimbal_pitch_direction(
                    float(omega[1]),
                    inverted=self.gimbal_pitch_invert,
                ),
                dt,
            )
            self.gimbal_angles_deg = dict(
                command_result.get(
                    "angles_deg",
                    self.gimbal_angles_deg,
                )
            )
            self.gimbal_commanded_angles_deg = dict(
                command_result.get(
                    "commanded_angles_deg",
                    self.gimbal_commanded_angles_deg,
                )
            )
            self.gimbal_rates_deg_s = dict(
                command_result.get(
                    "rates_deg_s",
                    self.gimbal_rates_deg_s,
                )
            )
            self.gimbal_raw_requested_rates_deg_s = dict(
                command_result.get(
                    "raw_requested_rates_deg_s",
                    self.gimbal_raw_requested_rates_deg_s,
                )
            )
            self.gimbal_limited_rates_deg_s = dict(
                command_result.get(
                    "limited_rates_deg_s",
                    self.gimbal_limited_rates_deg_s,
                )
            )
            self.gimbal_maximum_rates_deg_s = dict(
                command_result.get(
                    "maximum_rates_deg_s",
                    self.gimbal_maximum_rates_deg_s,
                )
            )
            self.gimbal_maximum_acceleration_deg_s2 = float(
                command_result.get(
                    "maximum_acceleration_deg_s2",
                    self.gimbal_maximum_acceleration_deg_s2,
                )
            )
            self.gimbal_maximum_braking_acceleration_deg_s2 = float(
                command_result.get(
                    "maximum_braking_acceleration_deg_s2",
                    self.gimbal_maximum_braking_acceleration_deg_s2,
                )
            )
            self.gimbal_maximum_command_lead_deg = float(
                command_result.get(
                    "maximum_command_lead_deg",
                    self.gimbal_maximum_command_lead_deg,
                )
            )
            self.gimbal_error = str(
                command_result.get("error", "")
            )
            self.gimbal_yaw_limit_deg = float(
                command_result.get(
                    "yaw_limit_deg",
                    self.configured_gimbal_yaw_limit_deg,
                )
            )
            self.gimbal_yaw_saturated_outward = bool(
                command_result.get("yaw_saturated_outward", False)
            )
            self.gimbal_yaw_requested_rate_deg_s = float(
                command_result.get("yaw_requested_rate_deg_s", 0.0)
            )
            self.gimbal_feedback_available = bool(
                command_result.get("feedback_available", False)
            )
            self.gimbal_feedback_age_ms = _finite_or_none(
                command_result.get("feedback_age_ms")
            )
            self.body_yaw_target_error_deg = (
                float(self.gimbal_angles_deg.get("yaw", 0.0))
                + self.body_yaw_image_error_deg
            )
            if command_result.get("ok"):
                self.gimbal_last_command_monotonic = now
                self.gimbal_command_timestamps.append(now)
            if update_frame_subsystems:
                self._update_after_pointing_locked(
                    frame,
                    result,
                    now,
                    dt,
                    bool(command_result.get("ok")),
                )
        except (
            ArithmeticError,
            AttributeError,
            KeyError,
            RuntimeError,
            TypeError,
            ValueError,
        ) as error:
            self.gimbal_controller.reset()
            self.body_yaw_target_error_deg = 0.0
            self.body_yaw_image_error_deg = 0.0
            self.body_pitch_image_error_deg = 0.0
            self._disable_body_yaw_locked(self.drone_id)
            self._stop_follow_locked(self.drone_id, "gimbal_error")
            self._stop_motion_locked(self.drone_id, "gimbal_error")
            self._stop_visual_follow_at_boundary_locked("gimbal_error", now)
            self.gimbal_yaw_saturated_outward = False
            self.gimbal_yaw_recenter_latched = False
            self.gimbal_yaw_requested_rate_deg_s = 0.0
            self.gimbal_rates_deg_s = {
                "roll": 0.0,
                "pitch": 0.0,
                "yaw": 0.0,
            }
            self.gimbal_raw_requested_rates_deg_s = dict(
                self.gimbal_rates_deg_s
            )
            self.gimbal_limited_rates_deg_s = dict(
                self.gimbal_rates_deg_s
            )
            self.gimbal_error = f"Gimbal control failed: {error}"

    def _source_fps_locked(self) -> float:
        source_fps = self.gazebo_bridge.source_fps.get(
            self.drone_id,
            0.0,
        )
        return round(source_fps, 1)

    def _draw_overlay(self, display: Any, result: Any) -> None:
        if result is not None and result.bbox is not None:
            bbox = result.bbox
            x = int(round(bbox.x))
            y = int(round(bbox.y))
            width = int(round(bbox.w))
            height = int(round(bbox.h))
            state = result.state.value
            colors = {
                "tracking": (0, 255, 0),
                "occluded": (0, 255, 255),
                "lost": (0, 0, 255),
            }
            color = colors.get(state, (160, 160, 160))
            cv2.rectangle(
                display,
                (x, y),
                (x + width, y + height),
                color,
                2,
            )
            cv2.circle(
                display,
                (x + width // 2, y + height // 2),
                4,
                (0, 0, 255),
                -1,
            )
            frame_height, frame_width = display.shape[:2]
            frame_center = (frame_width // 2, frame_height // 2)
            target_center = (
                x + width // 2,
                y + height // 2,
            )
            cv2.drawMarker(
                display,
                frame_center,
                (230, 230, 230),
                cv2.MARKER_CROSS,
                18,
                1,
            )
            cv2.line(
                display,
                frame_center,
                target_center,
                color,
                1,
                cv2.LINE_AA,
            )

        fps_text = f"FPS {self.fps:.1f}"
        position = (10, 28)
        cv2.putText(
            display,
            fps_text,
            position,
            cv2.FONT_HERSHEY_SIMPLEX,
            0.58,
            (20, 20, 20),
            3,
            cv2.LINE_AA,
        )
        cv2.putText(
            display,
            fps_text,
            position,
            cv2.FONT_HERSHEY_SIMPLEX,
            0.58,
            (230, 230, 230),
            1,
            cv2.LINE_AA,
        )
