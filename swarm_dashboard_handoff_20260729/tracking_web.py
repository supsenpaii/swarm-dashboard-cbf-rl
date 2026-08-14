from __future__ import annotations

import asyncio
import math
import os
import sys
import threading
import time
from dataclasses import replace
from pathlib import Path
from typing import Any, Callable

from metric_depth_estimator import MetricDepthEstimator
from visual_follow_target import (
    CameraRayProjector,
    DEFAULT_VISUAL_RANGE_PROFILE,
    TargetStateFilter,
    VisualBBoxFilter,
    VisualBBoxRangeEstimator,
    ned_target_to_wgs84,
)


SYSTEM_DIST_PACKAGES = Path("/usr/lib/python3/dist-packages")
REQUESTED_TRACKING_PACKAGE_ROOT = Path(
    os.environ.get(
        "SWARM_TRACKING_PACKAGE_ROOT",
        "/home/sup/ws_px4/src/lfc_gimbal_gazebo",
    )
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


def hold_visual_follow_radius(
    follower_ned: tuple[float, float, float],
    target_ned: tuple[float, float, float],
    target_velocity_ned: tuple[float, float, float],
    follow_distance_m: float,
    measured_distance_m: float,
    tolerance_m: float,
) -> tuple[tuple[float, float, float], tuple[float, float, float], bool]:
    """Keep native Follow Target's radial error at zero inside the deadband."""
    if not (
        math.isfinite(follow_distance_m)
        and math.isfinite(measured_distance_m)
        and math.isfinite(tolerance_m)
    ):
        return target_ned, target_velocity_ned, False
    if abs(measured_distance_m - follow_distance_m) > tolerance_m:
        return target_ned, target_velocity_ned, False

    north = float(target_ned[0]) - float(follower_ned[0])
    east = float(target_ned[1]) - float(follower_ned[1])
    horizontal_distance = math.hypot(north, east)
    if horizontal_distance < 0.1:
        return target_ned, target_velocity_ned, False
    unit_north = north / horizontal_distance
    unit_east = east / horizontal_distance
    held_target = (
        float(follower_ned[0]) + follow_distance_m * unit_north,
        float(follower_ned[1]) + follow_distance_m * unit_east,
        float(target_ned[2]),
    )
    radial_velocity = (
        float(target_velocity_ned[0]) * unit_north
        + float(target_velocity_ned[1]) * unit_east
    )
    held_velocity = (
        float(target_velocity_ned[0]) - radial_velocity * unit_north,
        float(target_velocity_ned[1]) - radial_velocity * unit_east,
        float(target_velocity_ned[2]),
    )
    return held_target, held_velocity, True


def soft_handover_range_m(
    follow_distance_m: float,
    measured_distance_m: float,
    elapsed_s: float,
    duration_s: float,
) -> tuple[float, float]:
    """Ramp native Follow's first target from zero radial error."""
    follow_distance_m = float(follow_distance_m)
    measured_distance_m = float(measured_distance_m)
    duration_s = max(0.1, float(duration_s))
    blend = max(0.0, min(1.0, float(elapsed_s) / duration_s))
    return (
        follow_distance_m
        + blend * (measured_distance_m - follow_distance_m),
        blend,
    )


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
        visual_target_command: Callable[..., dict[str, Any]] | None = None,
        pose_provider: Callable[[str], dict[str, Any]] | None = None,
    ) -> None:
        self.gazebo_bridge = gazebo_bridge
        self.allowed_drones = allowed_drones
        self.gimbal_command = gimbal_command
        self.body_yaw_command = body_yaw_command
        self.follow_command = follow_command
        self.motion_command = motion_command
        self.attitude_command = attitude_command
        self.visual_target_command = visual_target_command
        self.pose_provider = pose_provider
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
        self.gimbal_enabled = os.environ.get(
            "SWARM_TRACKING_GIMBAL_ENABLED",
            "true",
        ).strip().lower() not in {"0", "false", "no", "off"}
        self.body_yaw_enabled = os.environ.get(
            "SWARM_TRACKING_BODY_YAW_ENABLED",
            "false",
        ).strip().lower() not in {"0", "false", "no", "off"}
        self.follow_enabled = os.environ.get(
            "SWARM_TRACKING_FOLLOW_ENABLED",
            "true",
        ).strip().lower() not in {"0", "false", "no", "off"}
        self.visual_follow_feature_enabled = os.environ.get(
            "SWARM_VISUAL_FOLLOW_TARGET_ENABLED",
            "true",
        ).strip().lower() not in {"0", "false", "no", "off"}
        self.native_visual_follow_enabled = os.environ.get(
            "SWARM_NATIVE_FOLLOW_TARGET_ENABLED",
            "false",
        ).strip().lower() not in {"0", "false", "no", "off"}
        self.apparent_size_follow_enabled = os.environ.get(
            "SWARM_APPARENT_SIZE_FOLLOW_ENABLED",
            "false",
        ).strip().lower() not in {"0", "false", "no", "off"}
        self.metric_depth_follow_control = os.environ.get(
            "SWARM_METRIC_DEPTH_FOLLOW_CONTROL",
            "true",
        ).strip().lower() not in {"0", "false", "no", "off"}
        self.visual_follow_auto_start = os.environ.get(
            "SWARM_VISUAL_FOLLOW_AUTO_START",
            "true",
        ).strip().lower() not in {"0", "false", "no", "off"}
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
                            "0.60",
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
            self.visual_follow_center_hold_s = 0.60
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
            gimbal_fx = max(
                1.0,
                float(os.environ.get("SWARM_TRACKING_GIMBAL_FX", "205.5")),
            )
            gimbal_fy = max(
                1.0,
                float(os.environ.get("SWARM_TRACKING_GIMBAL_FY", "205.5")),
            )
            gimbal_kp = float(
                os.environ.get("SWARM_TRACKING_GIMBAL_KP", "18.0")
            )
            gimbal_ki = float(
                os.environ.get("SWARM_TRACKING_GIMBAL_KI", "0.25")
            )
            gimbal_kd = float(
                os.environ.get("SWARM_TRACKING_GIMBAL_KD", "0.20")
            )
            gimbal_ema_alpha = float(
                os.environ.get("SWARM_TRACKING_GIMBAL_EMA_ALPHA", "0.92")
            )
            gimbal_deadband_deg = float(
                os.environ.get("SWARM_TRACKING_GIMBAL_DEADBAND_DEG", "0.35")
            )
            gimbal_omega_max_deg = float(
                os.environ.get("SWARM_TRACKING_GIMBAL_OMEGA_MAX_DEG", "360")
            )
            gimbal_slew_max_deg = float(
                os.environ.get("SWARM_TRACKING_GIMBAL_SLEW_MAX_DEG", "1200")
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
                    "60",
                )
            )
        except ValueError:
            gimbal_fx = 205.5
            gimbal_fy = 205.5
            gimbal_kp = 18.0
            gimbal_ki = 0.25
            gimbal_kd = 0.20
            gimbal_ema_alpha = 0.92
            gimbal_deadband_deg = 0.35
            gimbal_omega_max_deg = 360.0
            gimbal_slew_max_deg = 1200.0
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
            offboard_max_yaw_rate_deg_s = 60.0

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
        self.visual_range_estimator = VisualBBoxRangeEstimator(
            DEFAULT_VISUAL_RANGE_PROFILE
        )
        self.visual_bbox_filter = VisualBBoxFilter()
        self.visual_ray_projector = CameraRayProjector(
            gimbal_fx,
            gimbal_fy,
        )
        self.metric_depth_estimator = MetricDepthEstimator(
            fx=gimbal_fx,
            fy=gimbal_fy,
        )
        self.visual_target_filter = TargetStateFilter()

        self.lock = threading.RLock()
        self.output_condition = threading.Condition(self.lock)
        self.shutdown_event = threading.Event()
        self.worker: threading.Thread | None = None

        self.active = False
        self.drone_id = ""
        self.state = "inactive"
        self.error = TRACKING_IMPORT_ERROR
        self.tracker: Any = None
        self.inference: Any = None
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
        self.gimbal_last_update_monotonic: float | None = None
        self.gimbal_last_command_monotonic: float | None = None
        self.gimbal_angles_deg = {
            "roll": 0.0,
            "pitch": 0.0,
            "yaw": 0.0,
        }
        self.gimbal_rates_deg_s = {
            "roll": 0.0,
            "pitch": 0.0,
            "yaw": 0.0,
        }
        self.gimbal_error = ""
        self.gimbal_yaw_limit_deg = 15.0
        self.gimbal_yaw_saturated_outward = False
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
        self.metric_depth_distance_m: float | None = None
        self.metric_depth_raw_m: float | None = None
        self.metric_depth_optical_z_m: float | None = None
        self.metric_depth_valid_fraction = 0.0
        self.metric_depth_state = "disabled"
        self.metric_depth_error = ""
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
        self.follow_radial_velocity_m_s = 0.0
        self.follow_command_forward = 0.0
        self.follow_active = False
        self.follow_state = "idle"
        self.follow_error = ""
        self.visual_follow_requested = False
        self.visual_follow_ready = False
        self.visual_follow_active = False
        self.visual_follow_state = "idle"
        self.visual_follow_error = ""
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

    def preload_metric_depth(self) -> bool:
        """Warm metric depth before the first bbox can enter the frame loop."""
        loaded = self.metric_depth_estimator.preload()
        with self.lock:
            self.metric_depth_state = (
                "preloaded"
                if loaded
                else (
                    f"unavailable:{self.metric_depth_estimator.load_error}"
                    if self.metric_depth_estimator.enabled
                    else "disabled"
                )
            )
            self.metric_depth_error = (
                ""
                if loaded or not self.metric_depth_estimator.enabled
                else self.metric_depth_estimator.load_error
            )
        return loaded

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
        if not 1.0 <= float(desired_distance_m) <= 20.0:
            raise ValueError("Desired tracking distance must be from 1 to 20 m")

        tracker, inference = self._create_tracker()

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

        self.gazebo_bridge.set_tracking_drone(drone_id)
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
            self.output_frame = None
            self.last_output_monotonic = None
            self.fps = 0.0
            self.fps_window_started = None
            self.fps_window_frames = 0
            self.tracking_ms = 0.0
            self.encode_ms = 0.0
            self._reset_gimbal_locked(stopped_drone_id)
            self.drone_id = ""
            self.output_version += 1
            self._close_tracker_locked()
            self.output_condition.notify_all()

        self.gazebo_bridge.set_tracking_drone(None)
        return self.status()

    def shutdown(self) -> None:
        self.shutdown_event.set()
        self.stop()

        worker = self.worker
        if worker is not None:
            worker.join(timeout=2.0)

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

            self.tracker.init(self.latest_raw_frame.copy(), bbox)
            self._reset_gimbal_locked()
            # Image-only range uses the fixed LUT. Selecting a bbox never
            # calibrates from lidar and never starts flight automatically.
            self.visual_range_estimator.reset()
            self.visual_target_filter.reset()
            self.follow_distance_m = None
            self.follow_lidar_distance_m = None
            self.follow_distance_source = (
                "apparent_size"
                if self.apparent_size_follow_enabled
                else "visual_lut"
            )
            self.follow_reference_area = float(bbox.w * bbox.h)
            self.follow_reference_distance_m = None
            self.follow_reference_bbox_scale_px = math.sqrt(
                max(36.0, self.follow_reference_area)
            )
            profile = self.visual_range_estimator.profile
            object_scale_px, object_scale_source = object_pixel_scale_px(
                self.latest_raw_frame,
                bbox,
            )
            scale_normalizer = math.sqrt(
                profile.frame_width
                * profile.frame_height
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
            self.follow_state = "acquiring"
            self.visual_follow_state = "acquiring"
            self.visual_follow_ready = False
            self.visual_follow_requested = False
            self.visual_follow_active = False
            self.latest_result = None
            self.selected_bbox = bbox.as_list()
            self.state = "tracking"
            self.error = ""

        return self.status()

    def set_visual_follow_requested(self, enabled: bool) -> dict[str, Any]:
        with self.lock:
            if enabled:
                if not self.visual_follow_feature_enabled:
                    raise RuntimeError("Visual Follow Target is disabled")
                if not self.active or self.tracker is None or not self.tracker.active:
                    raise RuntimeError("Select and track a target first")
                if not self.visual_follow_ready:
                    raise RuntimeError("Visual target is not READY yet")
                self.visual_follow_requested = True
                self.visual_follow_state = "ready"
                self.visual_follow_error = ""
            else:
                self._stop_visual_follow_locked("user_disabled")
                self.visual_follow_requested = False
        return self.status()

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
        with self.lock:
            last_output = self.last_output_monotonic
            result = self.latest_result
            bbox = self.selected_bbox
            score = None
            redetecting = False

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
                "source_fps": self._source_fps_locked(),
                "tracking_ms": round(self.tracking_ms, 2),
                "encode_ms": round(self.encode_ms, 2),
                "gimbal_enabled": self.gimbal_enabled,
                "gimbal_active": bool(
                    self.gimbal_enabled
                    and self.gimbal_controller is not None
                    and self.tracker is not None
                    and self.tracker.active
                ),
                "gimbal_angles_deg": dict(self.gimbal_angles_deg),
                "gimbal_rates_deg_s": dict(self.gimbal_rates_deg_s),
                "gimbal_error": self.gimbal_error,
                "gimbal_yaw_limit_deg": round(
                    self.gimbal_yaw_limit_deg,
                    1,
                ),
                "gimbal_yaw_saturated_outward": (
                    self.gimbal_yaw_saturated_outward
                ),
                "gimbal_yaw_requested_rate_deg_s": round(
                    self.gimbal_yaw_requested_rate_deg_s,
                    1,
                ),
                "body_yaw_enabled": self.body_yaw_enabled,
                "body_yaw_active": self.body_yaw_active,
                "body_yaw_latched": self.body_yaw_latched,
                "body_yaw_control_mode": "bbox_primary",
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
                "metric_depth_enabled": self.metric_depth_estimator.enabled,
                "metric_depth_available": self.metric_depth_estimator.available,
                "metric_depth_follow_control": self.metric_depth_follow_control,
                "metric_depth_state": self.metric_depth_state,
                "metric_depth_distance_m": (
                    round(self.metric_depth_distance_m, 2)
                    if self.metric_depth_distance_m is not None
                    else None
                ),
                "metric_depth_raw_m": (
                    round(self.metric_depth_raw_m, 2)
                    if self.metric_depth_raw_m is not None
                    else None
                ),
                "metric_depth_optical_z_m": (
                    round(self.metric_depth_optical_z_m, 2)
                    if self.metric_depth_optical_z_m is not None
                    else None
                ),
                "metric_depth_valid_fraction": round(
                    self.metric_depth_valid_fraction,
                    3,
                ),
                "metric_depth_error": self.metric_depth_error,
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
                "follow_error": self.follow_error,
                "visual_follow_feature_enabled": (
                    self.visual_follow_feature_enabled
                ),
                "native_visual_follow_enabled": (
                    self.native_visual_follow_enabled
                ),
                "visual_follow_auto_start": self.visual_follow_auto_start,
                "visual_follow_requested": self.visual_follow_requested,
                "visual_follow_ready": self.visual_follow_ready,
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
                "visual_follow_state": self.visual_follow_state,
                "visual_follow_error": self.visual_follow_error,
                "visual_range_profile": {
                    "name": self.visual_range_estimator.profile.name,
                    "frame_width": self.visual_range_estimator.profile.frame_width,
                    "frame_height": self.visual_range_estimator.profile.frame_height,
                    "samples": [
                        {
                            "distance_m": sample.distance_m,
                            "width_px": sample.width_px,
                            "height_px": sample.height_px,
                        }
                        for sample in self.visual_range_estimator.profile.samples
                    ],
                },
                "visual_range_raw_m": self.visual_range_raw_m,
                "visual_range_filtered_m": self.visual_range_filtered_m,
                "visual_bbox_scale_px": self.visual_bbox_scale_px,
                "visual_bbox_filter_state": self.visual_bbox_filter_state,
                "visual_bbox_stable_frames": self.visual_bbox_stable_frames,
                "visual_bbox_rejected_frames": self.visual_bbox_rejected_frames,
                "visual_bbox_center_residual_px": (
                    round(self.visual_bbox_center_residual_px, 2)
                    if self.visual_bbox_center_residual_px is not None
                    else None
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
                "motion_down_velocity_m_s": round(
                    self.motion_down_velocity_m_s,
                    3,
                ),
                "motion_yaw_rate_deg_s": round(
                    self.motion_yaw_rate_deg_s,
                    2,
                ),
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
                source_versions[drone_id] = (
                    self.gazebo_bridge.raw_frame_versions[drone_id]
                )

            if frame is None:
                continue

            try:
                self._process_frame(frame, drone_id, session_id)
            except Exception as error:
                with self.lock:
                    if self.active and self.session_id == session_id:
                        self.state = "error"
                        self.error = f"Tracking frame failed: {error}"

    def _process_frame(
        self,
        frame: Any,
        drone_id: str,
        session_id: int,
    ) -> None:
        with self.lock:
            if (
                not self.active
                or self.drone_id != drone_id
                or self.session_id != session_id
                or self.tracker is None
            ):
                return

            self.latest_raw_frame = frame
            result = None

            if self.tracker.active:
                try:
                    tracking_started = time.perf_counter()
                    result = self.tracker.update(frame)
                    now = time.monotonic()
                    result = self._filter_bbox_locked(result, now)
                    self._update_timing_locked(
                        "tracking_ms",
                        (time.perf_counter() - tracking_started) * 1000.0,
                    )
                    self.latest_result = result
                    self.state = result.state.value
                    self.error = ""
                    self._update_gimbal_locked(
                        frame,
                        result,
                        now,
                    )
                except Exception as error:
                    self.tracker.reset()
                    self._reset_gimbal_locked()
                    self.latest_result = None
                    self.selected_bbox = None
                    self.state = "error"
                    self.error = f"Tracking inference failed: {error}"
            elif self.state != "error":
                self.state = "selecting"

            self._update_fps_locked(time.monotonic())
            display = frame.copy()
            self._draw_overlay(display, result)
            encode_started = time.perf_counter()
            success, encoded = cv2.imencode(
                ".jpg",
                display,
                [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality],
            )
            self._update_timing_locked(
                "encode_ms",
                (time.perf_counter() - encode_started) * 1000.0,
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

    def _reset_gimbal_locked(self, drone_id: str | None = None) -> None:
        self._disable_body_yaw_locked(drone_id or self.drone_id)
        self._reset_follow_locked(drone_id or self.drone_id)
        self._stop_motion_locked(drone_id or self.drone_id, "idle")
        self._reset_visual_follow_locked(drone_id or self.drone_id)
        self.metric_depth_estimator.reset()
        self.metric_depth_distance_m = None
        self.metric_depth_raw_m = None
        self.metric_depth_optical_z_m = None
        self.metric_depth_valid_fraction = 0.0
        self.metric_depth_state = (
            "disabled" if not self.metric_depth_estimator.enabled else "reset"
        )
        self.metric_depth_error = ""
        if self.gimbal_controller is not None:
            self.gimbal_controller.reset()
        self.visual_bbox_filter.reset()
        self.visual_bbox_filter_state = "idle"
        self.visual_bbox_stable_frames = 0
        self.visual_bbox_rejected_frames = 0
        self.visual_bbox_center_residual_px = None
        self.gimbal_last_update_monotonic = None
        self.gimbal_last_command_monotonic = None
        self.gimbal_angles_deg = {
            "roll": 0.0,
            "pitch": 0.0,
            "yaw": 0.0,
        }
        self.gimbal_rates_deg_s = {
            "roll": 0.0,
            "pitch": 0.0,
            "yaw": 0.0,
        }
        self.gimbal_error = ""
        self.gimbal_yaw_limit_deg = 15.0
        self.gimbal_yaw_saturated_outward = False
        self.gimbal_yaw_requested_rate_deg_s = 0.0
        self.body_yaw_target_error_deg = 0.0
        self.body_yaw_image_error_deg = 0.0
        self.body_pitch_image_error_deg = 0.0

    def _stop_visual_follow_locked(self, state: str) -> None:
        should_publish = bool(
            self.drone_id
            and self.visual_target_command is not None
            and (self.visual_follow_active or self.visual_follow_requested)
        )
        if should_publish:
            try:
                self.visual_target_command(
                    self.drone_id,
                    False,
                    None,
                    state,
                )
            except Exception:
                pass
        self.visual_follow_requested = False
        self.visual_follow_active = False
        self.visual_follow_state = state
        self.visual_follow_last_publish_monotonic = None
        self.visual_follow_ready_since_monotonic = None
        self.visual_follow_center_since_monotonic = None
        self.visual_follow_handover_started_monotonic = None
        self.visual_follow_handover_blend = 0.0
        self.visual_follow_center_error_deg = 0.0

    def _reset_visual_follow_locked(self, drone_id: str) -> None:
        self._stop_visual_follow_locked("idle")
        self.visual_range_estimator.reset()
        self.visual_target_filter.reset()
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
        )
        self.visual_bbox_filter_state = filtered.reason
        self.visual_bbox_stable_frames = filtered.stable_frames
        self.visual_bbox_rejected_frames = filtered.rejected_frames
        self.visual_bbox_center_residual_px = filtered.center_residual_px
        if not filtered.valid:
            return replace(result, bbox=None)
        assert None not in (
            filtered.cx,
            filtered.cy,
            filtered.width,
            filtered.height,
        )
        return replace(
            result,
            bbox=BBox.from_center(
                float(filtered.cx),
                float(filtered.cy),
                float(filtered.width),
                float(filtered.height),
            ),
        )

    def _update_metric_depth_locked(
        self,
        frame: Any,
        result: Any,
        gimbal_command_ok: bool,
    ) -> None:
        state = getattr(result.state, "value", str(result.state))
        score = float(getattr(result, "score", 0.0))
        estimate = self.metric_depth_estimator.update(
            frame,
            result.bbox,
            tracking_valid=bool(
                state == "tracking"
                and score >= self.follow_min_score
                and gimbal_command_ok
                and not self.gimbal_error
                and result.bbox is not None
                and self.visual_bbox_stable_frames
                >= self.visual_bbox_filter.stable_frames_required
            ),
        )
        self.metric_depth_state = estimate.reason
        self.metric_depth_raw_m = estimate.distance_raw_m
        self.metric_depth_distance_m = estimate.distance_filtered_m
        self.metric_depth_optical_z_m = estimate.optical_depth_m
        self.metric_depth_valid_fraction = estimate.roi_valid_fraction
        self.metric_depth_error = (
            estimate.reason
            if self.metric_depth_estimator.enabled and not estimate.valid
            else ""
        )

    def _best_metric_or_fallback_distance(
        self,
        fallback_distance_m: float | None,
    ) -> tuple[float | None, str]:
        if (
            self.metric_depth_estimator.enabled
            and self.metric_depth_estimator.last_estimate.valid
            and self.metric_depth_estimator.last_estimate.ready
            and self.metric_depth_distance_m is not None
        ):
            return self.metric_depth_distance_m, "metric_depth"
        return fallback_distance_m, "visual_lut"

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
        if self.visual_bbox_filter_state == "holding_outlier":
            self.visual_follow_state = "holding_outlier"
            return
        estimate = self.visual_range_estimator.update(
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
        self.visual_range_raw_m = estimate.distance_raw_m
        self.visual_range_filtered_m = estimate.distance_filtered_m
        self.visual_bbox_scale_px = estimate.scale_px
        self.visual_range_quality = estimate.quality
        self.visual_ttc_s = estimate.ttc_s
        selected_distance_m, selected_distance_source = (
            self._best_metric_or_fallback_distance(estimate.distance_filtered_m)
        )
        self.follow_distance_m = selected_distance_m
        self.follow_distance_source = selected_distance_source
        self.visual_follow_ready = bool(estimate.ready)

        metric_required = bool(
            self.metric_depth_follow_control
            and self.metric_depth_estimator.enabled
        )
        metric_ready = bool(
            self.metric_depth_estimator.last_estimate.valid
            and self.metric_depth_estimator.last_estimate.ready
            and self.metric_depth_distance_m is not None
        )
        if metric_required and not metric_ready:
            self.visual_follow_ready = False
            self.visual_follow_state = "waiting_metric_depth"
            self.visual_follow_error = (
                "Waiting for stable UniDepth range: "
                f"{self.metric_depth_estimator.last_estimate.reason}"
            )
            if self.visual_follow_requested or self.visual_follow_active:
                self._stop_visual_follow_locked("waiting_metric_depth")
            return

        if (
            not estimate.valid
            or not estimate.ready
            or selected_distance_m is None
        ):
            self.visual_follow_state = estimate.reason
            if self.visual_follow_requested or self.visual_follow_active:
                self.visual_follow_error = (
                    f"Visual Follow stopped: {estimate.reason}"
                )
                self._stop_visual_follow_locked(estimate.reason)
            return

        self.visual_follow_center_error_deg = visual_follow_center_error_deg(
            float(bbox.cx),
            float(bbox.cy),
            frame_width,
            frame_height,
            float(self.gimbal_controller.cfg.fx),
            float(self.gimbal_controller.cfg.fy),
        )
        stable_frame_gate = max(
            self.visual_bbox_filter.stable_frames_required,
            self.visual_follow_min_stable_frames,
        )
        centered_and_stable = bool(
            self.visual_follow_center_error_deg
            <= self.visual_follow_center_deadband_deg
            and self.visual_bbox_stable_frames >= stable_frame_gate
        )
        if not centered_and_stable:
            self.visual_follow_ready = False
            self.visual_follow_ready_since_monotonic = None
            self.visual_follow_center_since_monotonic = None
            self.visual_follow_state = "waiting_center"
            self.visual_follow_error = "Waiting for stable centered bbox"
            if self.visual_follow_requested or self.visual_follow_active:
                self._stop_visual_follow_locked("waiting_center")
            return
        if self.visual_follow_center_since_monotonic is None:
            self.visual_follow_center_since_monotonic = now
        if (
            now - self.visual_follow_center_since_monotonic
            < self.visual_follow_center_hold_s
        ):
            self.visual_follow_ready = False
            self.visual_follow_ready_since_monotonic = None
            self.visual_follow_state = "waiting_center"
            self.visual_follow_error = "Centering bbox"
            return
        self.visual_follow_ready = True

        if not self.visual_follow_requested:
            if not self.visual_follow_auto_start:
                self.visual_follow_state = "ready"
                self.visual_follow_error = ""
                return
            if self.visual_follow_ready_since_monotonic is None:
                self.visual_follow_ready_since_monotonic = now
            if (
                now - self.visual_follow_ready_since_monotonic
                < self.visual_follow_start_delay_s
            ):
                self.visual_follow_state = "ready"
                self.visual_follow_error = ""
                return
            if (
                self.visual_follow_last_auto_attempt_monotonic is not None
                and now - self.visual_follow_last_auto_attempt_monotonic
                < self.visual_follow_auto_retry_s
            ):
                self.visual_follow_state = "ready"
                return
            self.visual_follow_requested = True
            self.visual_follow_last_auto_attempt_monotonic = now
            self.visual_follow_handover_started_monotonic = now
            self.visual_follow_handover_blend = 0.0
            self.visual_follow_state = "starting"
            self.visual_follow_error = ""
        if self.visual_target_command is None or self.pose_provider is None:
            self.visual_follow_error = "Visual target callbacks are unavailable"
            self._stop_visual_follow_locked("unavailable")
            return

        pose = self.pose_provider(self.drone_id)
        if not pose.get("available"):
            self.visual_follow_error = str(
                pose.get("error", "Visual target pose is unavailable")
            )
            self._stop_visual_follow_locked("pose_invalid")
            return
        try:
            local = pose["local_position"]
            global_position = pose["global_position"]
            follower_ned = (
                float(local["x_north_m"]),
                float(local["y_east_m"]),
                float(local["z_down_m"]),
            )
            heading_rad = float(local["heading_rad"])
            if not math.isfinite(heading_rad):
                raise ValueError("vehicle heading is not finite")
            if self.visual_follow_handover_started_monotonic is None:
                self.visual_follow_handover_started_monotonic = now
            handover_distance_m, handover_blend = soft_handover_range_m(
                self.follow_distance_target_m,
                float(selected_distance_m),
                now - self.visual_follow_handover_started_monotonic,
                self.visual_follow_handover_s,
            )
            self.visual_follow_handover_blend = handover_blend
            # A centered bbox means the target lies on the current heading.
            # Keeping the same NED altitude and ramping only the radius avoids
            # the initial lateral/vertical step that used to jerk the vehicle.
            target_measurement = (
                follower_ned[0] + handover_distance_m * math.cos(heading_rad),
                follower_ned[1] + handover_distance_m * math.sin(heading_rad),
                follower_ned[2],
            )
            filtered = self.visual_target_filter.update(target_measurement, now)
            if not filtered.valid or filtered.reason in {
                "position_jump", "invalid_measurement", "invalid_dt"
            }:
                raise ValueError(f"target filter: {filtered.reason}")
            target_ned = filtered.position_ned
            # Do not feed the target-filter derivative into PX4 during native
            # handover. It includes observer/vehicle motion and creates a large
            # velocity step exactly when AUTO FOLLOW is entered.
            target_velocity = (0.0, 0.0, 0.0)
            assert target_ned is not None and target_velocity is not None
            target_global = ned_target_to_wgs84(
                follower_lat_deg=float(global_position["latitude_deg"]),
                follower_lon_deg=float(global_position["longitude_deg"]),
                follower_alt_msl_m=float(global_position["altitude_msl_m"]),
                follower_position_ned=follower_ned,
                target_position_ned=target_ned,
            )
            target_ned, target_velocity, distance_band_held = (
                hold_visual_follow_radius(
                    follower_ned,
                    target_ned,
                    target_velocity,
                    self.follow_distance_target_m,
                    float(selected_distance_m),
                    self.follow_deadband_m,
                )
            )
            if distance_band_held:
                target_global = ned_target_to_wgs84(
                    follower_lat_deg=float(global_position["latitude_deg"]),
                    follower_lon_deg=float(global_position["longitude_deg"]),
                    follower_alt_msl_m=float(global_position["altitude_msl_m"]),
                    follower_position_ned=follower_ned,
                    target_position_ned=target_ned,
                )
        except (AssertionError, KeyError, TypeError, ValueError) as error:
            self.visual_follow_error = f"Visual target projection failed: {error}"
            self._stop_visual_follow_locked("projection_invalid")
            return

        self.visual_target_ned = target_ned
        self.visual_target_velocity_ned = target_velocity
        self.visual_target_global = target_global
        if (
            self.visual_follow_last_publish_monotonic is not None
            and now - self.visual_follow_last_publish_monotonic < 0.10
        ):
            return
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
                "distance_m": selected_distance_m,
                "follow_distance_m": self.follow_distance_target_m,
                "follow_height_m": max(1.0, -follower_ned[2]),
                "quality": estimate.quality,
                "ttc_s": estimate.ttc_s,
            },
            state,
        )
        self.visual_follow_last_publish_monotonic = now
        self.visual_follow_active = bool(response.get("ok"))
        self.visual_follow_state = (
            "following" if self.visual_follow_active else "blocked"
        )
        self.visual_follow_error = str(response.get("error", ""))
        if not self.visual_follow_active:
            self.visual_follow_requested = False

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
        self.metric_depth_distance_m = None
        self.metric_depth_raw_m = None
        self.metric_depth_optical_z_m = None
        self.metric_depth_valid_fraction = 0.0
        self.metric_depth_state = (
            "disabled" if not self.metric_depth_estimator.enabled else "reset"
        )
        self.metric_depth_error = ""
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
            self._stop_motion_locked(self.drone_id, "precenter_invalid")
            return
        yaw_rate = native_precenter_yaw_rate_deg_s(
            float(self.gimbal_angles_deg.get("yaw", 0.0)),
            self.body_yaw_image_error_deg,
        )
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

    def _update_motion_locked(
        self,
        result: Any,
        now: float,
        gimbal_command_ok: bool,
    ) -> None:
        if self.motion_command is None:
            self._stop_motion_locked(self.drone_id, "disabled")
            return
        state = getattr(result.state, "value", str(result.state))
        score = float(getattr(result, "score", 0.0))
        valid_tracking = bool(
            state == "tracking"
            and score >= max(self.body_yaw_min_score, self.follow_min_score)
            and gimbal_command_ok
            and not self.gimbal_error
            and result.bbox is not None
        )
        if not valid_tracking:
            if self.motion_invalid_since_monotonic is None:
                self.motion_invalid_since_monotonic = now
            invalid_age = now - self.motion_invalid_since_monotonic
            # Keep a neutral velocity setpoint through short tracker dropouts.
            # This prevents OFFBOARD/POSITION oscillation frame-by-frame while
            # PX4 continues holding vertical speed at zero.
            if self.motion_active and invalid_age < 0.75:
                last_publish = self.motion_last_publish_monotonic
                if last_publish is None or now - last_publish >= 0.045:
                    if self.apparent_size_follow_enabled:
                        response = self.motion_command(
                            self.drone_id,
                            True,
                            0.0,
                            0.0,
                            0.0,
                            self.follow_distance_m,
                            self.follow_distance_target_m,
                            self.follow_distance_source,
                            "tracking_grace",
                        )
                    else:
                        response = self.attitude_command(
                            self.drone_id,
                            True,
                            0.0,
                            "tracking_grace",
                            True,
                        )
                    self.motion_last_publish_monotonic = now
                    self.motion_state = "tracking_grace"
                    self.motion_error = str(response.get("error", ""))
                return
            self._stop_motion_locked(self.drone_id, state)
            return
        self.motion_invalid_since_monotonic = None

        # Use one PX4 motion authority for all three body axes. Apparent-size
        # follow still supplies the normalized forward command, while the
        # attitude-recenter controller combines it with filtered gimbal
        # roll/pitch/yaw home errors. Previously this path was skipped in
        # apparent-size mode, so only yaw and vertical velocity recentered.
        if self.body_attitude_enabled and self.apparent_size_follow_enabled:
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
        previous_motion = self.motion_last_publish_monotonic
        motion_dt = (
            1.0 / 30.0
            if previous_motion is None
            else max(0.001, min(0.1, now - previous_motion))
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
            yaw_rate_deg_s = native_precenter_yaw_rate_deg_s(
                float(self.gimbal_angles_deg.get("yaw", 0.0)),
                self.body_yaw_image_error_deg,
                maximum_rate_deg_s=self.offboard_max_yaw_rate_deg_s,
            )
            pitch_error_deg = 0.0
        if abs(self.body_yaw_image_error_deg) > self.follow_align_yaw_deg:
            forward_velocity_m_s = 0.0
            motion_state = "aligning"
        elif abs(forward_velocity_m_s) > 1e-3:
            motion_state = "following"
        elif abs(down_velocity_m_s) > 1e-3:
            motion_state = "vertical_recentering"
        elif abs(yaw_rate_deg_s) > 1e-3:
            motion_state = "yawing"
        else:
            motion_state = "holding"

        last_publish = self.motion_last_publish_monotonic
        if last_publish is not None and now - last_publish < 0.045:
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
            state,
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
        profile = self.visual_range_estimator.profile
        width_cal = float(result.bbox.w) * profile.frame_width / frame_width
        height_cal = float(result.bbox.h) * profile.frame_height / frame_height
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
                profile.frame_width
                * profile.frame_height
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
            if (
                self.metric_depth_follow_control
                and self.metric_depth_estimator.last_estimate.valid
                and self.metric_depth_distance_m is not None
            ):
                self.follow_distance_m = self.metric_depth_distance_m
                self.follow_distance_source = "metric_depth"
                distance_error = safe_follow_band_error(
                    self.metric_depth_distance_m,
                    self.follow_min_distance_m,
                    self.follow_max_distance_m,
                )
                distance_error_is_metric = True
        else:
            measured_distance, _ = (
                self.visual_range_estimator._distance_from_scale(bbox_scale_px)
            )
            selected_distance_m, selected_distance_source = (
                self._best_metric_or_fallback_distance(measured_distance)
            )
            self.follow_distance_source = selected_distance_source
            alpha = self.follow_ema_alpha
            self.follow_distance_m = (
                selected_distance_m
                if self.follow_distance_m is None
                else (
                    (1.0 - alpha) * self.follow_distance_m
                    + alpha * float(selected_distance_m)
                )
            )
            previous_distance = self.follow_previous_distance_m
            previous_distance_time = self.follow_previous_distance_monotonic
            if previous_distance is None or previous_distance_time is None:
                measured_radial_velocity = 0.0
            else:
                range_dt = max(
                    1.0 / 60.0,
                    min(0.5, now - previous_distance_time),
                )
                measured_radial_velocity = max(
                    -self.offboard_max_forward_m_s,
                    min(
                        self.offboard_max_forward_m_s,
                        (self.follow_distance_m - previous_distance) / range_dt,
                    ),
                )
            rate_alpha = self.follow_radial_velocity_alpha
            self.follow_radial_velocity_m_s = (
                (1.0 - rate_alpha) * self.follow_radial_velocity_m_s
                + rate_alpha * measured_radial_velocity
            )
            self.follow_previous_distance_m = self.follow_distance_m
            self.follow_previous_distance_monotonic = now
            distance_error = safe_follow_band_error(
                self.follow_distance_m,
                self.follow_min_distance_m,
                self.follow_max_distance_m,
            )
            distance_error_is_metric = True

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
            # Metric/LUT distance already uses the explicit 8-12 m safe band.
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
        command_target = gain * signed_error
        if distance_error_is_metric:
            command_target += (
                self.follow_radial_velocity_gain
                * self.follow_radial_velocity_m_s
                * self.follow_max
                / max(0.1, self.offboard_max_forward_m_s)
            )
            braking_speed = braking_speed_limit_m_s(
                abs(signed_error),
                self.follow_brake_m_s2,
            )
            braking_command = (
                braking_speed
                * self.follow_max
                / max(0.1, self.offboard_max_forward_m_s)
            )
            command_target = math.copysign(
                min(abs(command_target), braking_command),
                command_target,
            )
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
        delta_limit = self.follow_slew * max(0.001, min(0.1, dt))
        self.follow_command_forward += max(
            -delta_limit,
            min(delta_limit, target - self.follow_command_forward),
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
        self.follow_state = (
            "holding"
            if abs(self.follow_command_forward) <= 1e-3
            and abs(distance_error) <= deadband
            else ("following" if self.follow_active else "blocked")
        )
        if not self.follow_active:
            self.follow_command_forward = 0.0

    def _update_body_yaw_locked(
        self,
        result: Any,
        now: float,
        dt: float,
        gimbal_command_ok: bool,
    ) -> None:
        if not self.body_yaw_enabled or self.body_yaw_command is None:
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

    def _update_gimbal_locked(
        self,
        frame: Any,
        result: Any,
        now: float,
    ) -> None:
        if (
            not self.gimbal_enabled
            or self.gimbal_controller is None
            or self.gimbal_command is None
            or result is None
            or result.bbox is None
        ):
            self._stop_motion_locked(self.drone_id, "inactive")
            self._stop_visual_follow_locked("inactive")
            return

        previous = self.gimbal_last_update_monotonic
        dt = 1.0 / 30.0 if previous is None else now - previous
        dt = max(0.001, min(0.1, dt))
        self.gimbal_last_update_monotonic = now
        frame_height, frame_width = frame.shape[:2]
        self.body_yaw_image_error_deg = math.degrees(
            math.atan2(
                float(result.bbox.cx) - frame_width / 2.0,
                max(1.0, float(self.gimbal_controller.cfg.fx)),
            )
        )
        self.body_pitch_image_error_deg = math.degrees(
            math.atan2(
                frame_height / 2.0 - float(result.bbox.cy),
                max(1.0, float(self.gimbal_controller.cfg.fy)),
            )
        )

        try:
            omega = self.gimbal_controller.update(
                result.bbox.cx,
                result.bbox.cy,
                result.state,
                dt,
                frame_width,
                frame_height,
            )

            if omega is None:
                self.body_yaw_target_error_deg = 0.0
                self.body_yaw_image_error_deg = 0.0
                self.body_pitch_image_error_deg = 0.0
                self._disable_body_yaw_locked(self.drone_id)
                state = getattr(result.state, "value", str(result.state))
                self._stop_follow_locked(self.drone_id, state)
                self._stop_motion_locked(self.drone_id, state)
                self._stop_visual_follow_locked(state)
                return

            command_result = self.gimbal_command(
                self.drone_id,
                float(omega[0]),
                float(omega[1]),
                dt,
            )
            self.gimbal_angles_deg = dict(
                command_result.get(
                    "angles_deg",
                    self.gimbal_angles_deg,
                )
            )
            self.gimbal_rates_deg_s = dict(
                command_result.get(
                    "rates_deg_s",
                    self.gimbal_rates_deg_s,
                )
            )
            self.gimbal_error = str(
                command_result.get("error", "")
            )
            self.gimbal_yaw_limit_deg = float(
                command_result.get("yaw_limit_deg", 15.0)
            )
            self.gimbal_yaw_saturated_outward = bool(
                command_result.get("yaw_saturated_outward", False)
            )
            self.gimbal_yaw_requested_rate_deg_s = float(
                command_result.get("yaw_requested_rate_deg_s", 0.0)
            )
            self.body_yaw_target_error_deg = (
                float(self.gimbal_angles_deg.get("yaw", 0.0))
                + self.body_yaw_image_error_deg
            )

            if command_result.get("ok"):
                self.gimbal_last_command_monotonic = now
            self._update_metric_depth_locked(
                frame,
                result,
                bool(command_result.get("ok")),
            )
            if (
                self.visual_follow_feature_enabled
                and self.native_visual_follow_enabled
                and not self.apparent_size_follow_enabled
            ):
                self._update_visual_follow_locked(
                    frame,
                    result,
                    now,
                    bool(command_result.get("ok")),
                )
                # Native PX4 Follow is the only aircraft-motion authority in
                # visual-follow mode, including READY/BLOCKED/retry periods.
                # Never let legacy recenter briefly re-enter Offboard between
                # native target updates.
                self._disable_body_yaw_locked(self.drone_id)
                self._stop_follow_locked(self.drone_id, "native_follow")
                if (
                    self.visual_follow_state == "waiting_center"
                    and not self.visual_follow_active
                ):
                    self._update_native_precenter_yaw_locked(
                        result,
                        now,
                        bool(command_result.get("ok")),
                    )
                else:
                    self._stop_motion_locked(self.drone_id, "native_follow")
            else:
                self._stop_visual_follow_locked("offboard_range_follow")
                self._update_body_yaw_locked(
                    result,
                    now,
                    dt,
                    bool(command_result.get("ok")),
                )
                self._update_follow_locked(
                    result,
                    now,
                    dt,
                    bool(command_result.get("ok")),
                )
                self._update_motion_locked(
                    result,
                    now,
                    bool(command_result.get("ok")),
                )
        except Exception as error:
            self.gimbal_controller.reset()
            self.body_yaw_target_error_deg = 0.0
            self.body_yaw_image_error_deg = 0.0
            self.body_pitch_image_error_deg = 0.0
            self._disable_body_yaw_locked(self.drone_id)
            self._stop_follow_locked(self.drone_id, "gimbal_error")
            self._stop_motion_locked(self.drone_id, "gimbal_error")
            self._stop_visual_follow_locked("gimbal_error")
            self.gimbal_yaw_saturated_outward = False
            self.gimbal_yaw_requested_rate_deg_s = 0.0
            self.gimbal_rates_deg_s = {
                "roll": 0.0,
                "pitch": 0.0,
                "yaw": 0.0,
            }
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
