from __future__ import annotations

import math
import os
import threading
import time
from dataclasses import dataclass
from typing import Any, Sequence

import numpy as np

from bearing_range_filter import RobustBearingRangeFilter
from bearing_target_estimator import TargetEstimate
from depth_model_adapter import (
    DepthModelAdapter,
    create_depth_adapter_from_environment,
)
from depth_worker import DepthJob, LatestDepthWorker
from follow_target_quality_gate import FollowTargetQualityGate
from m52_adapter import M52GroundAnchorAdapter
from metric_depth_calibrator import MetricDepthCalibrator
from range_applicability_gate import (
    RangeApplicabilityGate,
    RangeApplicabilityInputs,
)
from range_physical_diagnostics import (
    DIAGNOSTICS_SCHEMA_VERSION,
    RangePhysicalDiagnosticsCollector,
    canonical_sha256,
    timestamp_stage,
)
from range_residual_correction import (
    RangeResidualCorrector,
    RangeResidualFeatures,
)
from range_residual_dataset import RangeResidualDatasetCollector
from stage2a_range_model import (
    KNOWN_RISK_RANGE_STD_MULTIPLIER,
    SIZE_AGNOSTIC_RANGE_STD_M,
    is_known_risk_geometry,
    predict_range_m,
)
from target_depth_extractor import TargetDepthExtractor
from target_fusion_ekf import TargetFusionEKF
from visual_follow_target import CameraRayProjector


def _enabled(name: str, default: bool = False) -> bool:
    fallback = "true" if default else "false"
    return os.environ.get(name, fallback).strip().lower() in {
        "1", "true", "yes", "on",
    }


def _env_float(
    name: str,
    default: float,
    minimum: float,
    maximum: float,
) -> float:
    try:
        value = float(os.environ.get(name, str(default)))
    except ValueError:
        value = default
    return max(minimum, min(maximum, value))


def stage2a_model_features(
    *,
    target_depth: Any,
    inverse_result: Any,
    calibration: Any,
    normalized_x: float,
    normalized_y: float,
    camera_position_ned_m: Sequence[float],
    physics_slant_range_m: float,
) -> dict[str, float | None]:
    """Build the feature dict for `stage2a_range_model`, live-object version.

    Field-for-field the same construction as
    `core_range_stage2a_relative_range_model.extract_rows`, minus
    `gimbal_pitch_deg` (not exposed on the live fusion context; left as
    None so the model median-imputes it, same as any other missing feature).
    Keep this in sync with that script if either changes -- there is no
    shared schema module, only the parallel construction and the test in
    test_stage2a_model_features_matches_training_schema.
    """

    quantiles = calibration.anchor_inverse_depth_quantiles or ()
    return {
        "target_raw_inverse_depth": target_depth.relative_inverse_depth,
        "target_raw_inverse_depth_std": target_depth.relative_inverse_depth_std,
        "target_roi_optical_depth_m": target_depth.optical_depth_m,
        "target_roi_optical_depth_std_m": target_depth.optical_depth_std_m,
        "target_valid_fraction": target_depth.valid_fraction,
        "target_sample_count": target_depth.sample_count,
        "filtered_inverse_depth": inverse_result.filtered_inverse_depth,
        "inverse_depth_std": inverse_result.inverse_depth_std,
        "calibration_scale": calibration.scale,
        "calibration_offset": calibration.offset,
        "calibration_anchor_count": calibration.anchor_count,
        "calibration_inlier_count": calibration.inlier_count,
        "calibration_residual_m_inv": calibration.residual_m_inv,
        "calibration_condition_number": calibration.condition_number,
        "calibration_stable": 1.0 if calibration.stable else 0.0,
        "calibration_parameter_uncertainty": calibration.parameter_uncertainty,
        "anchor_inverse_depth_span": (
            (quantiles[5] - quantiles[1]) if len(quantiles) == 7 else None
        ),
        "image_ray_x": normalized_x,
        "image_ray_y": normalized_y,
        "camera_altitude_px4_m": -float(camera_position_ned_m[2]),
        "gimbal_pitch_deg": None,
        "physics_slant_range_m": physics_slant_range_m,
    }


@dataclass(frozen=True)
class FusionFrameContext:
    bbox_xywh: tuple[float, float, float, float]
    bbox_center_px: tuple[float, float]
    camera_position_ned_m: tuple[float, float, float]
    camera_quaternion_xyzw: tuple[float, float, float, float]
    tracking_score: float
    frame_width: int
    frame_height: int
    ground_down_m: float
    filtered_bearing_ned_unit: tuple[float, float, float]
    calibration_only: bool = False
    dataset_group_id: str = ""
    dataset_session_id: int = 0
    source_sim_timestamp_s: float | None = None
    ground_truth_distance_m: float | None = None
    ground_truth_valid: bool = False
    ground_truth_reason: str = "disabled"
    ground_truth_uncertainty_m: float | None = None
    ground_truth_time_offset_ms: float | None = None
    ground_truth_quality: str = "unavailable"
    ground_truth_lever_arm_corrected: bool = False
    instrumentation_context: dict[str, Any] | None = None


class MetricTargetFusion:
    """Async MiDaS + M52 metric calibration + target EKF coordinator."""

    def __init__(
        self,
        projector: CameraRayProjector,
        *,
        adapter: DepthModelAdapter | None = None,
        enabled: bool | None = None,
        range_residual_corrector: RangeResidualCorrector | None = None,
        dataset_collector: RangeResidualDatasetCollector | None = None,
        range_applicability_gate: RangeApplicabilityGate | None = None,
        physical_diagnostics_collector: (
            RangePhysicalDiagnosticsCollector | None
        ) = None,
    ) -> None:
        self.projector = projector
        self._diagnostic_consumer_receive_timestamp_s: float | None = None
        self._diagnostic_consume_timestamp_s: float | None = None
        self.enabled = (
            _enabled("SWARM_METRIC_TARGET_FUSION_ENABLED", True)
            if enabled is None
            else bool(enabled)
        )
        self.depth_rate_hz = _env_float(
            # Revalidated after removing the pose-history deepcopy/GIL
            # bottleneck. 5 Hz is the highest rate in the representative
            # 2/3/4/5/7.5 Hz sweep that meets every absolute tracking and
            # latency gate; see full_stack_contention_fix/depth_rate_sweep.csv.
            "SWARM_METRIC_TARGET_DEPTH_RATE_HZ", 5.0, 1.0, 20.0
        )
        self.maximum_depth_age_s = _env_float(
            "SWARM_METRIC_TARGET_MAX_DEPTH_AGE_S", 0.75, 0.1, 3.0
        )
        self.calibration_cache_ttl_s = _env_float(
            "SWARM_METRIC_CALIBRATION_CACHE_TTL_S", 2.5, 0.2, 3.0
        )
        self.calibration_scale_drift_fraction_per_s = _env_float(
            "SWARM_METRIC_CALIBRATION_SCALE_DRIFT_FRACTION_PER_S",
            0.08,
            0.0,
            0.5,
        )
        self.calibration_offset_drift_m_inv_per_s = _env_float(
            "SWARM_METRIC_CALIBRATION_OFFSET_DRIFT_M_INV_PER_S",
            0.003,
            0.0,
            0.05,
        )
        self.target_anchor_support_margin_iqr = _env_float(
            "SWARM_METRIC_TARGET_ANCHOR_SUPPORT_MARGIN_IQR",
            0.50,
            0.0,
            2.0,
        )
        self.dropout_timeout_s = _env_float(
            "SWARM_METRIC_TARGET_DROPOUT_TIMEOUT_S", 1.5, 0.2, 5.0
        )
        self.angular_noise_px = _env_float(
            "SWARM_METRIC_TARGET_BEARING_NOISE_PX", 1.5, 0.2, 20.0
        )
        self.multiview_minimum_baseline_m = _env_float(
            "SWARM_MULTIVIEW_MIN_BASELINE_M",
            0.5,
            0.5,
            5.0,
        )
        self.multiview_minimum_intersection_angle_deg = _env_float(
            "SWARM_MULTIVIEW_MIN_INTERSECTION_ANGLE_DEG",
            2.0,
            2.0,
            30.0,
        )
        self.multiview_maximum_reprojection_error_px = _env_float(
            "SWARM_MULTIVIEW_MAX_REPROJECTION_ERROR_PX",
            4.0,
            0.5,
            4.0,
        )
        self.multiview_maximum_condition_number = _env_float(
            "SWARM_MULTIVIEW_MAX_CONDITION_NUMBER",
            1.0e7,
            100.0,
            1.0e7,
        )
        self.ground_down_m = _env_float(
            "SWARM_M52_GROUND_DOWN_M", 0.0, -1000.0, 1000.0
        )
        self.ground_target_mode = _enabled(
            "SWARM_M52_GROUND_TARGET_MODE",
            False,
        )
        # Opt-in: replace the M52 physics_slant_range_m point estimate with
        # the learned Stage 2A model (core_range_stage2a_relative_range_model.py).
        # Cross-validated MAE 0.88 m on held-out scenario groups vs. ~4.27 m
        # for the physics formula on the same corpus (see
        # artifacts/core_range_3_12m/stage2a_relative_range_model/evaluation.json).
        # Default off: this has not been flight/stack validated, only
        # evaluated offline against logged Stage 2A captures.
        self.range_model_enabled = _enabled(
            "SWARM_METRIC_TARGET_RANGE_MODEL_ENABLED",
            False,
        )
        self.calibrator = MetricDepthCalibrator(
            ema_alpha=0.08,
            temporal_window=15,
            stable_samples_required=5,
            maximum_scale_step_fraction=0.18,
            maximum_offset_step_m_inv=0.04,
            temporal_uncertainty_floor_fraction=0.35,
        )
        self.extractor = TargetDepthExtractor()
        self.bearing_range_filter = RobustBearingRangeFilter(
            range_window=15,
            range_minimum_samples=7,
            range_tau_s=2.0,
            range_hampel_sigma=3.0,
            range_absolute_gate_m=0.75,
            range_relative_gate=0.06,
            maximum_range_rate_m_s=2.0,
            maximum_range_acceleration_m_s2=3.0,
            inverse_depth_window=15,
            inverse_depth_minimum_samples=7,
            inverse_depth_tau_s=1.5,
            inverse_depth_hampel_sigma=3.0,
            inverse_depth_relative_gate=0.18,
            inverse_depth_uncertainty_floor_fraction=0.35,
        )
        self.multiview_range_filter = RobustBearingRangeFilter(
            range_window=15,
            range_minimum_samples=7,
            range_tau_s=2.0,
            range_hampel_sigma=3.0,
            range_absolute_gate_m=0.75,
            range_relative_gate=0.06,
            maximum_range_rate_m_s=2.0,
            maximum_range_acceleration_m_s2=3.0,
        )
        self.ekf = TargetFusionEKF()
        self.quality_gate = FollowTargetQualityGate()
        self.multiview_quality_gate = FollowTargetQualityGate()
        self.anchor_adapter = M52GroundAnchorAdapter(projector)
        self.range_residual_corrector = (
            range_residual_corrector
            if range_residual_corrector is not None
            else RangeResidualCorrector.from_environment()
        )
        self.range_applicability_gate = (
            range_applicability_gate
            if range_applicability_gate is not None
            else RangeApplicabilityGate.from_environment()
        )
        self.dataset_collector = (
            dataset_collector
            if dataset_collector is not None
            else RangeResidualDatasetCollector.from_environment()
        )
        self.physical_diagnostics_collector = (
            physical_diagnostics_collector
            if physical_diagnostics_collector is not None
            else RangePhysicalDiagnosticsCollector.from_environment()
        )
        if (
            self.dataset_collector.enabled
            and self.range_residual_corrector.mode == "active"
        ):
            self.dataset_collector.load_error = (
                "dataset_collection_requires_off_or_shadow_mode"
            )
            self.dataset_collector.last_reason = (
                self.dataset_collector.load_error
            )
        self.worker: LatestDepthWorker | None = None
        self._adapter_error = ""
        if self.enabled:
            try:
                selected_adapter = (
                    adapter
                    if adapter is not None
                    else create_depth_adapter_from_environment()
                )
                self.worker = LatestDepthWorker(selected_adapter)
            except Exception as error:
                self._adapter_error = str(error)
        self._last_submit_timestamp_s: float | None = None
        self._consumed_depth_version = 0
        self._last_depth_measurement_s: float | None = None
        self._last_raw_target_range_m: float | None = None
        self._last_physics_distance_m: float | None = None
        self._last_target_range_m: float | None = None
        self._last_range_std_m: float | None = None
        self._last_range_measurement_accepted = False
        self._last_range_uncertainty_components: dict[str, float] = {}
        self._last_valid_calibration_timestamp_s: float | None = None
        self._calibration_source = "none"
        self._current_calibration_fit_reason = "not_calibrated"
        self._prewarm_submitted_count = 0
        self._prewarm_result_count = 0
        self._prewarm_accepted_count = 0
        self._prewarm_rejected_count = 0
        self._last_prewarm_reason = "not_started"
        self._last_prewarm_result_timestamp_s: float | None = None
        self._last_prewarm_latency_ms: float | None = None
        self._last_anchor_candidate_count = 0
        self._last_accepted_ground_anchor_count = 0
        self._last_anchor_rejected_reason_counts: dict[str, int] = {}
        self._last_anchor_inverse_depth_quantiles: tuple[float, ...] = ()
        self._last_anchor_metric_depth_quantiles_m: tuple[float, ...] = ()
        self._last_anchor_spatial_bin_count = 0
        self._last_anchor_spatial_coverage_fraction = 0.0
        self._last_target_inverse_depth_support: dict[
            str,
            float | bool | str | None,
        ] = {}
        self._last_multiview_reason = "unavailable"
        self._last_multiview_accepted = False
        self._last_multiview_discrepancy_m: float | None = None
        self._last_multiview_range_m: float | None = None
        self._last_multiview_range_std_m: float | None = None
        self._active_metric_source = "none"
        self._last_reason = (
            "disabled"
            if not self.enabled
            else ("adapter_unavailable" if self.worker is None else "waiting_depth")
        )
        self._track_epoch = 1
        self._calibration_epoch = 1

    @property
    def ready(self) -> bool:
        return bool(
            self.quality_gate.initialized
            or self.multiview_quality_gate.initialized
        )

    @property
    def ready_for_safe_distance_lock(self) -> bool:
        depth_ready = bool(
            self.quality_gate.last.ready
            and self._last_range_measurement_accepted
            and self._last_target_range_m is not None
        )
        multiview_ready = bool(
            self.multiview_quality_gate.last.ready
            and self._last_multiview_accepted
            and self._last_multiview_range_m is not None
        )
        return bool(depth_ready or multiview_ready)

    @property
    def active_metric_source(self) -> str:
        return self._active_metric_source

    @property
    def operational(self) -> bool:
        return bool(
            self.enabled
            and self.worker is not None
            and not self.worker.load_error
        )

    @property
    def dataset_collection_enabled(self) -> bool:
        return self.dataset_collector.enabled

    @property
    def physical_diagnostics_enabled(self) -> bool:
        return self.physical_diagnostics_collector.enabled

    def start(self) -> None:
        if self.worker is not None:
            self.worker.start()

    def reset(self, *, preserve_calibration: bool = False) -> None:
        preserved_timestamp = self._last_valid_calibration_timestamp_s
        calibration_preserved = bool(
            preserve_calibration
            and self.calibrator.restore_last_valid()
            and preserved_timestamp is not None
        )
        if not calibration_preserved:
            self._calibration_epoch += 1
            self.calibrator.reset()
            self._last_valid_calibration_timestamp_s = None
            self._calibration_source = "none"
            self._current_calibration_fit_reason = "not_calibrated"
        else:
            self._last_valid_calibration_timestamp_s = preserved_timestamp
            self._calibration_source = "prewarmed"
            self._current_calibration_fit_reason = "calibration_preloaded"
        self._track_epoch += 1
        self.bearing_range_filter.reset()
        self.multiview_range_filter.reset()
        self.ekf.reset()
        self.quality_gate.reset()
        self.multiview_quality_gate.reset()
        self.range_residual_corrector.reset()
        self.range_applicability_gate.reset()
        if self.worker is not None:
            self.worker.reset()
        self._last_submit_timestamp_s = None
        self._consumed_depth_version = 0
        self._last_depth_measurement_s = None
        self._last_raw_target_range_m = None
        self._last_physics_distance_m = None
        self._last_target_range_m = None
        self._last_range_std_m = None
        self._last_range_measurement_accepted = False
        self._last_range_uncertainty_components = {}
        self._last_anchor_candidate_count = 0
        self._last_accepted_ground_anchor_count = 0
        self._last_anchor_rejected_reason_counts = {}
        self._last_anchor_inverse_depth_quantiles = ()
        self._last_anchor_metric_depth_quantiles_m = ()
        self._last_anchor_spatial_bin_count = 0
        self._last_anchor_spatial_coverage_fraction = 0.0
        self._last_target_inverse_depth_support = {}
        self._last_multiview_reason = "unavailable"
        self._last_multiview_accepted = False
        self._last_multiview_discrepancy_m = None
        self._last_multiview_range_m = None
        self._last_multiview_range_std_m = None
        self._active_metric_source = "none"
        self._last_reason = (
            "calibration_preloaded"
            if calibration_preserved
            else ("waiting_depth" if self.enabled else "disabled")
        )

    def shutdown(self) -> None:
        if self.worker is not None:
            self.worker.stop()

    def prewarm(
        self,
        *,
        frame_bgr: np.ndarray,
        measurement_timestamp_s: float,
        now_s: float,
        frame_index: int,
        camera_position_ned_m: Sequence[float],
        camera_quaternion_xyzw: Sequence[float],
        dataset_group_id: str = "",
        dataset_session_id: int = 0,
        source_sim_timestamp_s: float | None = None,
        instrumentation_context: dict[str, Any] | None = None,
    ) -> None:
        """Calibrate MiDaS against visible ground before a bbox is selected.

        This only estimates the frame-wide inverse-depth-to-metric mapping.
        It never extracts a target range, initializes the EKF, or publishes a
        target coordinate.
        """

        if not self.enabled or self.worker is None:
            return
        camera = tuple(float(value) for value in camera_position_ned_m)
        quaternion = tuple(float(value) for value in camera_quaternion_xyzw)
        height, width = frame_bgr.shape[:2]
        submit_due = bool(
            self._last_submit_timestamp_s is None
            or measurement_timestamp_s - self._last_submit_timestamp_s
            >= 1.0 / self.depth_rate_hz
        )
        if submit_due:
            context = FusionFrameContext(
                bbox_xywh=(0.0, 0.0, 0.0, 0.0),
                bbox_center_px=(0.5 * width, 0.5 * height),
                camera_position_ned_m=(camera[0], camera[1], camera[2]),
                camera_quaternion_xyzw=(
                    quaternion[0],
                    quaternion[1],
                    quaternion[2],
                    quaternion[3],
                ),
                tracking_score=0.0,
                frame_width=width,
                frame_height=height,
                ground_down_m=self.ground_down_m,
                filtered_bearing_ned_unit=(1.0, 0.0, 0.0),
                calibration_only=True,
                dataset_group_id=str(dataset_group_id),
                dataset_session_id=int(dataset_session_id),
                source_sim_timestamp_s=source_sim_timestamp_s,
                instrumentation_context=(
                    None
                    if instrumentation_context is None
                    else dict(instrumentation_context)
                ),
            )
            self.worker.submit(
                DepthJob(
                    frame_bgr=frame_bgr,
                    measurement_timestamp_s=measurement_timestamp_s,
                    frame_index=frame_index,
                    context=context,
                )
            )
            self._last_submit_timestamp_s = measurement_timestamp_s
            self._prewarm_submitted_count += 1

        version, depth_result = self.worker.latest()
        if depth_result is not None and version != self._consumed_depth_version:
            consumer_receive_timestamp_s = time.monotonic()
            self._consumed_depth_version = version
            self._consume_depth_result(
                depth_result,
                now_s,
                consumer_receive_timestamp_s=consumer_receive_timestamp_s,
                consume_timestamp_s=time.monotonic(),
            )

    def update(
        self,
        *,
        frame_bgr: np.ndarray,
        bbox_xywh: Sequence[float],
        bbox_center_px: Sequence[float],
        tracking_score: float,
        tracking_valid: bool,
        measurement_timestamp_s: float,
        now_s: float,
        frame_index: int,
        camera_position_ned_m: Sequence[float],
        camera_quaternion_xyzw: Sequence[float],
        dataset_group_id: str = "",
        dataset_session_id: int = 0,
        source_sim_timestamp_s: float | None = None,
        ground_truth_distance_m: float | None = None,
        ground_truth_valid: bool = False,
        ground_truth_reason: str = "disabled",
        ground_truth_uncertainty_m: float | None = None,
        ground_truth_time_offset_ms: float | None = None,
        ground_truth_quality: str = "unavailable",
        ground_truth_lever_arm_corrected: bool = False,
        instrumentation_context: dict[str, Any] | None = None,
    ) -> TargetEstimate:
        if not self.enabled:
            return self._invalid(now_s, "disabled")
        if self.worker is None:
            return self._invalid(
                now_s,
                f"adapter_unavailable:{self._adapter_error}",
            )
        if not tracking_valid:
            estimate = self.ekf.estimate(
                now_s,
                stale_timeout_s=self.dropout_timeout_s,
            )
            if estimate.valid and self.ready:
                return self._as_target_estimate(
                    estimate,
                    now_s,
                    "predicting_dropout",
                )
            return self._invalid(now_s, "tracking_invalid")

        bbox = tuple(float(value) for value in bbox_xywh)
        center = tuple(float(value) for value in bbox_center_px)
        camera = tuple(float(value) for value in camera_position_ned_m)
        quaternion = tuple(float(value) for value in camera_quaternion_xyzw)
        height, width = frame_bgr.shape[:2]
        ray = self.projector.pixel_to_ned_ray(
            center[0],
            center[1],
            width,
            height,
            quaternion,
        )
        angular_std = self.angular_noise_px / max(
            1.0,
            0.5 * (self.projector.fx + self.projector.fy),
        )
        bearing_result = self.bearing_range_filter.update_bearing(
            ray,
            measurement_timestamp_s,
        )
        filtered_ray = (
            ray
            if bearing_result.bearing_ned_unit is None
            else bearing_result.bearing_ned_unit
        )
        self.ekf.update_bearing(
            camera,
            filtered_ray,
            angular_std,
            measurement_timestamp_s,
        )

        submit_due = bool(
            self._last_submit_timestamp_s is None
            or measurement_timestamp_s - self._last_submit_timestamp_s
            >= 1.0 / self.depth_rate_hz
        )
        if submit_due:
            context = FusionFrameContext(
                bbox_xywh=bbox,
                bbox_center_px=(center[0], center[1]),
                camera_position_ned_m=(camera[0], camera[1], camera[2]),
                camera_quaternion_xyzw=(
                    quaternion[0],
                    quaternion[1],
                    quaternion[2],
                    quaternion[3],
                ),
                tracking_score=float(tracking_score),
                frame_width=width,
                frame_height=height,
                ground_down_m=self.ground_down_m,
                filtered_bearing_ned_unit=tuple(
                    float(value) for value in filtered_ray
                ),
                dataset_group_id=str(dataset_group_id),
                dataset_session_id=int(dataset_session_id),
                source_sim_timestamp_s=source_sim_timestamp_s,
                ground_truth_distance_m=ground_truth_distance_m,
                ground_truth_valid=bool(ground_truth_valid),
                ground_truth_reason=str(ground_truth_reason),
                ground_truth_uncertainty_m=ground_truth_uncertainty_m,
                ground_truth_time_offset_ms=ground_truth_time_offset_ms,
                ground_truth_quality=str(ground_truth_quality),
                ground_truth_lever_arm_corrected=bool(
                    ground_truth_lever_arm_corrected
                ),
                instrumentation_context=(
                    None
                    if instrumentation_context is None
                    else dict(instrumentation_context)
                ),
            )
            self.worker.submit(
                DepthJob(
                    frame_bgr=frame_bgr,
                    measurement_timestamp_s=measurement_timestamp_s,
                    frame_index=frame_index,
                    context=context,
                )
            )
            self._last_submit_timestamp_s = measurement_timestamp_s

        version, depth_result = self.worker.latest()
        if (
            depth_result is not None
            and version != self._consumed_depth_version
        ):
            consumer_receive_timestamp_s = time.monotonic()
            self._consumed_depth_version = version
            self._consume_depth_result(
                depth_result,
                now_s,
                consumer_receive_timestamp_s=consumer_receive_timestamp_s,
                consume_timestamp_s=time.monotonic(),
            )

        estimate = self.ekf.estimate(
            now_s,
            stale_timeout_s=self.dropout_timeout_s,
        )
        if estimate.valid and self.ready:
            reason = (
                "ok"
                if self.quality_gate.last.ready
                else f"predicting:{self._last_reason}"
            )
            return self._as_target_estimate(estimate, now_s, reason)
        reason = self.quality_gate.last.reason
        if reason == "uninitialized":
            reason = self._last_reason
        return self._as_target_estimate(
            estimate,
            now_s,
            reason,
            force_invalid=True,
        )

    def status(self) -> dict[str, Any]:
        worker_status = (
            self.worker.status()
            if self.worker is not None
            else {
                "thread_alive": False,
                "latest_reason": self._adapter_error or "unavailable",
            }
        )
        return {
            "enabled": self.enabled,
            "operational": self.operational,
            "ready": self.ready,
            "reason": self._last_reason,
            "active_metric_source": self._active_metric_source,
            "depth_rate_hz": self.depth_rate_hz,
            "ground_target_mode": self.ground_target_mode,
            "stability_filter": {
                "calibration_temporal_window": (
                    self.calibrator.temporal_window
                ),
                "calibration_ema_alpha": self.calibrator.ema_alpha,
                "temporal_uncertainty_floor_fraction": (
                    self.calibrator.temporal_uncertainty_floor_fraction
                ),
                "inverse_depth_window": (
                    self.bearing_range_filter.inverse_depth_window
                ),
                "inverse_depth_minimum_samples": (
                    self.bearing_range_filter.inverse_depth_minimum_samples
                ),
                "inverse_depth_tau_s": (
                    self.bearing_range_filter.inverse_depth_tau_s
                ),
                "range_window": self.bearing_range_filter.range_window,
                "range_minimum_samples": (
                    self.bearing_range_filter.range_minimum_samples
                ),
                "range_tau_s": self.bearing_range_filter.range_tau_s,
                "maximum_range_rate_m_s": (
                    self.bearing_range_filter.maximum_range_rate_m_s
                ),
                "maximum_range_acceleration_m_s2": (
                    self.bearing_range_filter.maximum_range_acceleration_m_s2
                ),
                "maximum_measurement_std_m": (
                    self.bearing_range_filter.maximum_measurement_std_m
                ),
                "maximum_measurement_relative_std": (
                    self.bearing_range_filter.maximum_measurement_relative_std
                ),
            },
            "calibration_cache": {
                "source": self._calibration_source,
                "fit_reason": self._current_calibration_fit_reason,
                "age_ms": (
                    None
                    if self._last_valid_calibration_timestamp_s is None
                    else max(
                        0.0,
                        time.monotonic()
                        - self._last_valid_calibration_timestamp_s,
                    )
                    * 1000.0
                ),
                "ttl_ms": self.calibration_cache_ttl_s * 1000.0,
                "prewarm_submitted_count": self._prewarm_submitted_count,
                "prewarm_result_count": self._prewarm_result_count,
                "prewarm_accepted_count": self._prewarm_accepted_count,
                "prewarm_rejected_count": self._prewarm_rejected_count,
                "last_prewarm_reason": self._last_prewarm_reason,
                "last_prewarm_result_age_ms": (
                    None
                    if self._last_prewarm_result_timestamp_s is None
                    else max(
                        0.0,
                        time.monotonic()
                        - self._last_prewarm_result_timestamp_s,
                    )
                    * 1000.0
                ),
                "last_prewarm_latency_ms": self._last_prewarm_latency_ms,
            },
            "ready_for_safe_distance_lock": (
                self.ready_for_safe_distance_lock
            ),
            "calibration": {
                "valid": self.calibrator.last.valid,
                "reason": self.calibrator.last.reason,
                "scale": self.calibrator.last.scale,
                "offset": self.calibrator.last.offset,
                "inlier_count": self.calibrator.last.inlier_count,
                "anchor_count": self.calibrator.last.anchor_count,
                "residual_m_inv": self.calibrator.last.residual_m_inv,
                "raw_scale": self.calibrator.last.raw_scale,
                "raw_offset": self.calibrator.last.raw_offset,
                "filtered_scale": self.calibrator.last.scale,
                "filtered_offset": self.calibrator.last.offset,
                "condition_number": (
                    self.calibrator.last.condition_number
                ),
                "parameter_uncertainty": (
                    self.calibrator.last.parameter_uncertainty
                ),
                "parameter_covariance_ab": [
                    list(row)
                    for row in self.calibrator.last.parameter_covariance_ab
                ],
                "residual_std_m_inv": (
                    self.calibrator.last.residual_std_m_inv
                ),
                "stable_samples": self.calibrator.last.stable_samples,
                "stable": self.calibrator.last.stable,
                "measurement_accepted": (
                    self.calibrator.last.measurement_accepted
                ),
                "anchor_inverse_depth_quantiles": list(
                    self.calibrator.last.anchor_inverse_depth_quantiles
                ),
                "metric_depth_quantiles_m": list(
                    self.calibrator.last.metric_depth_quantiles_m
                ),
                "change_point_reseeded": (
                    self.calibrator.last.change_point_reseeded
                ),
                "recovery": self.calibrator.recovery_status(),
            },
            "m52_anchors": {
                "candidate_count": self._last_anchor_candidate_count,
                "accepted_ground_anchor_count": (
                    self._last_accepted_ground_anchor_count
                ),
                "rejected_reason_counts": dict(
                    self._last_anchor_rejected_reason_counts
                ),
                "inverse_depth_quantiles": list(
                    self._last_anchor_inverse_depth_quantiles
                ),
                "metric_depth_quantiles_m": list(
                    self._last_anchor_metric_depth_quantiles_m
                ),
                "spatial_bin_count": self._last_anchor_spatial_bin_count,
                "spatial_coverage_fraction": (
                    self._last_anchor_spatial_coverage_fraction
                ),
            },
            "target_inverse_depth_support": dict(
                self._last_target_inverse_depth_support
            ),
            "quality_gate": {
                "ready": self.quality_gate.last.ready,
                "initialized": self.quality_gate.initialized,
                "reason": self.quality_gate.last.reason,
                "quality": self.quality_gate.last.quality,
                "stable_samples": self.quality_gate.last.stable_samples,
            },
            "bearing_range_filter": self.bearing_range_filter.status(),
            "last_raw_target_range_m": self._last_raw_target_range_m,
            "last_physics_distance_m": self._last_physics_distance_m,
            "last_target_range_m": self._last_target_range_m,
            "last_range_std_m": self._last_range_std_m,
            "range_uncertainty_components": dict(
                self._last_range_uncertainty_components
            ),
            "range_residual_correction": (
                self.range_residual_corrector.status()
            ),
            "range_applicability_gate": (
                self.range_applicability_gate.status()
            ),
            "range_residual_dataset": self.dataset_collector.status(),
            "physical_range_diagnostics": {
                **self.physical_diagnostics_collector.status(),
                "track_epoch": self._track_epoch,
                "calibration_epoch": self._calibration_epoch,
            },
            "range_measurement_accepted": (
                self._last_range_measurement_accepted
            ),
            "estimator": self.ekf.status(),
            "multiview_crosscheck": {
                "accepted": self._last_multiview_accepted,
                "reason": self._last_multiview_reason,
                "position_discrepancy_m": (
                    self._last_multiview_discrepancy_m
                ),
                "range_m": self._last_multiview_range_m,
                "range_std_m": self._last_multiview_range_std_m,
                "range_filter": self.multiview_range_filter.status()["range"],
                "quality_gate": {
                    "ready": self.multiview_quality_gate.last.ready,
                    "initialized": (
                        self.multiview_quality_gate.initialized
                    ),
                    "reason": self.multiview_quality_gate.last.reason,
                    "quality": self.multiview_quality_gate.last.quality,
                    "stable_samples": (
                        self.multiview_quality_gate.last.stable_samples
                    ),
                },
                "geometry_gate": {
                    "minimum_baseline_m": (
                        self.multiview_minimum_baseline_m
                    ),
                    "minimum_intersection_angle_deg": (
                        self.multiview_minimum_intersection_angle_deg
                    ),
                    "maximum_reprojection_error_px": (
                        self.multiview_maximum_reprojection_error_px
                    ),
                    "maximum_condition_number": (
                        self.multiview_maximum_condition_number
                    ),
                },
            },
            "last_depth_age_ms": (
                None
                if self._last_depth_measurement_s is None
                else max(
                    0.0,
                    time.monotonic() - self._last_depth_measurement_s,
                )
                * 1000.0
            ),
            "worker": worker_status,
        }

    def fuse_multiview(
        self,
        estimate: TargetEstimate,
        now_s: float,
    ) -> bool:
        """Fuse passive triangulation as an independent metric measurement.

        Multi-view may initialize the metric EKF without MiDaS/M52, but only
        after its own range warm-up and position-covariance quality gate. It
        never commands bootstrap motion and a held/rejected estimate is never
        inserted into either filter.
        """

        self._last_multiview_accepted = False
        if not estimate.valid or estimate.position_ned_m is None:
            self._last_multiview_reason = estimate.reason
            return False
        if (
            estimate.range_m is None
            or estimate.range_std_m is None
        ):
            self._last_multiview_reason = "multiview_range_missing"
            return False
        if estimate.baseline_m < self.multiview_minimum_baseline_m:
            self._last_multiview_reason = "insufficient_baseline"
            return False
        if (
            estimate.intersection_angle_deg
            < self.multiview_minimum_intersection_angle_deg
        ):
            self._last_multiview_reason = "weak_intersection_angle"
            return False
        if (
            estimate.reprojection_error_px is None
            or estimate.reprojection_error_px
            > self.multiview_maximum_reprojection_error_px
        ):
            self._last_multiview_reason = "reprojection_error"
            return False
        if (
            estimate.condition_number is None
            or estimate.condition_number
            > self.multiview_maximum_condition_number
        ):
            self._last_multiview_reason = "degenerate_geometry"
            return False
        if (
            estimate.estimate_age_ms is not None
            and estimate.estimate_age_ms > 500.0
        ):
            self._last_multiview_reason = "multiview_estimate_stale"
            return False
        try:
            range_result = self.multiview_range_filter.update_range(
                float(estimate.range_m),
                float(estimate.range_std_m),
                float(estimate.timestamp_s),
            )
        except (ArithmeticError, TypeError, ValueError):
            self._last_multiview_reason = "multiview_range_invalid"
            return False
        if (
            not range_result.valid
            or not range_result.measurement_accepted
            or range_result.range_m is None
            or range_result.range_std_m is None
        ):
            self._last_multiview_reason = range_result.reason
            if not self.quality_gate.initialized:
                self._last_reason = f"multiview:{range_result.reason}"
            return False

        metric = self.ekf.estimate(
            now_s,
            stale_timeout_s=self.dropout_timeout_s,
        )
        candidate = np.asarray(
            estimate.position_ned_m,
            dtype=np.float64,
        )
        multiview_std = max(0.2, float(estimate.range_std_m or 2.0))
        if metric.valid and metric.position_ned_m is not None:
            current = np.asarray(metric.position_ned_m, dtype=np.float64)
            discrepancy = float(np.linalg.norm(candidate - current))
            self._last_multiview_discrepancy_m = discrepancy
            gate = max(
                1.0,
                3.0 * math.hypot(
                    float(metric.position_std_m or 2.5),
                    multiview_std,
                ),
            )
            if discrepancy > gate:
                self._last_multiview_reason = "multiview_disagreement"
                return False
        else:
            self._last_multiview_discrepancy_m = None
        covariance_source = np.asarray(
            estimate.covariance,
            dtype=np.float64,
        )
        covariance = (
            covariance_source[:3, :3]
            if covariance_source.ndim == 2
            and covariance_source.shape[0] >= 3
            and covariance_source.shape[1] >= 3
            and np.all(np.isfinite(covariance_source[:3, :3]))
            else np.eye(3) * multiview_std**2
        )
        covariance = covariance + np.eye(3) * 0.04
        try:
            self.ekf.update_position(
                candidate,
                covariance,
                float(estimate.timestamp_s),
            )
        except (ArithmeticError, TypeError, ValueError):
            self._last_multiview_reason = "multiview_covariance_invalid"
            return False
        fused = self.ekf.estimate(
            now_s,
            stale_timeout_s=self.dropout_timeout_s,
        )
        quality = self.multiview_quality_gate.update(
            fused,
            calibration_valid=True,
            target_depth_valid=True,
            calibration_stable=True,
            inverse_depth_ready=True,
            range_measurement_accepted=True,
            range_uncertainty_valid=True,
            tracking_quality_valid=True,
        )
        self._last_multiview_range_m = float(range_result.range_m)
        self._last_multiview_range_std_m = float(
            range_result.range_std_m
        )
        self._last_multiview_accepted = True
        self._last_multiview_reason = (
            "accepted" if quality.ready else quality.reason
        )
        if quality.ready:
            self._active_metric_source = "multiview"
            self._last_reason = "multiview_ready"
        elif not self.quality_gate.initialized:
            self._last_reason = f"multiview:{quality.reason}"
        return True

    def predict(self, now_s: float) -> TargetEstimate:
        estimate = self.ekf.estimate(
            now_s,
            stale_timeout_s=self.dropout_timeout_s,
        )
        if estimate.valid and self.ready:
            return self._as_target_estimate(
                estimate,
                now_s,
                "predicting_dropout",
            )
        return self._as_target_estimate(
            estimate,
            now_s,
            estimate.reason,
            force_invalid=True,
        )

    def _begin_prewarm_result(self, result: Any, now_s: float) -> bool:
        context = getattr(result, "context", None)
        if not (
            isinstance(context, FusionFrameContext)
            and context.calibration_only
        ):
            return False
        self._prewarm_result_count += 1
        self._last_prewarm_result_timestamp_s = float(now_s)
        completed = getattr(result, "completed_timestamp_s", None)
        measurement = getattr(result, "measurement_timestamp_s", None)
        try:
            latency_ms = (
                float(completed) - float(measurement)
            ) * 1000.0
            self._last_prewarm_latency_ms = max(0.0, latency_ms)
        except (TypeError, ValueError):
            self._last_prewarm_latency_ms = None
        return True

    def _finish_prewarm_result(
        self,
        is_prewarm: bool,
        reason: str,
        *,
        accepted: bool,
    ) -> None:
        if not is_prewarm:
            return
        self._last_prewarm_reason = str(reason)
        if accepted:
            self._prewarm_accepted_count += 1
        else:
            self._prewarm_rejected_count += 1

    @staticmethod
    def _calibration_diagnostics(calibration: Any | None) -> dict[str, Any] | None:
        if calibration is None:
            return None
        return {
            "valid": bool(calibration.valid),
            "reason": str(calibration.reason),
            "raw_scale": calibration.raw_scale,
            "raw_offset": calibration.raw_offset,
            "filtered_scale": calibration.scale,
            "filtered_offset": calibration.offset,
            "inlier_count": int(calibration.inlier_count),
            "anchor_count": int(calibration.anchor_count),
            "residual_m_inv": calibration.residual_m_inv,
            "residual_std_m_inv": calibration.residual_std_m_inv,
            "condition_number": calibration.condition_number,
            "parameter_uncertainty": calibration.parameter_uncertainty,
            "stable_samples": int(calibration.stable_samples),
            "stable": bool(calibration.stable),
            "measurement_accepted": bool(calibration.measurement_accepted),
            "parameter_covariance_ab": [
                list(row) for row in calibration.parameter_covariance_ab
            ],
            "anchor_inverse_depth_quantiles": list(
                calibration.anchor_inverse_depth_quantiles
            ),
            "metric_depth_quantiles_m": list(
                calibration.metric_depth_quantiles_m
            ),
            "change_point_reseeded": bool(
                calibration.change_point_reseeded
            ),
        }

    def _record_physical_diagnostics(
        self,
        *,
        result: Any,
        context: FusionFrameContext,
        now_s: float,
        stage: str,
        reason: str,
        anchors: Any | None = None,
        fit_calibration: Any | None = None,
        applied_calibration: Any | None = None,
        calibration_cache_age_s: float | None = None,
        target_depth: Any | None = None,
        inverse_result: Any | None = None,
        image_ray_x: float | None = None,
        image_ray_y: float | None = None,
        ray_scale: float | None = None,
        raw_roi_slant_range_m: float | None = None,
        filtered_optical_depth_m: float | None = None,
        physics_slant_range_m: float | None = None,
        applicability: Any | None = None,
        correction: Any | None = None,
    ) -> None:
        collector = self.physical_diagnostics_collector
        if not collector.enabled:
            return
        instrumentation = dict(context.instrumentation_context or {})
        reported_camera_info = dict(instrumentation.get("camera_info") or {})
        cx = (
            float(self.projector.cx)
            if self.projector.cx is not None
            else context.frame_width / 2.0
        )
        cy = (
            float(self.projector.cy)
            if self.projector.cy is not None
            else context.frame_height / 2.0
        )
        runtime_camera_info = {
            "width": int(context.frame_width),
            "height": int(context.frame_height),
            "fx": float(self.projector.fx),
            "fy": float(self.projector.fy),
            "cx": cx,
            "cy": cy,
            "pixel_convention": "u_right_v_down_pixel_center",
            "projection": "analytic_pinhole_no_K_inverse",
            "distortion_model": reported_camera_info.get(
                "distortion_model"
            ),
            "distortion_coefficients": reported_camera_info.get(
                "distortion_coefficients"
            ),
            "rectified": reported_camera_info.get("rectified"),
            "camera_info_source": reported_camera_info.get(
                "camera_info_source", "runtime_projector_configuration"
            ),
            "unavailable_fields": reported_camera_info.get(
                "unavailable_fields",
                ["distortion_model", "distortion_coefficients", "rectified"],
            ),
        }
        runtime_camera_info["fingerprint_sha256"] = canonical_sha256(
            runtime_camera_info
        )
        runtime_extrinsics = {
            "camera_position_ned_m": list(
                context.camera_position_ned_m
            ),
            "camera_quaternion_xyzw": list(
                context.camera_quaternion_xyzw
            ),
            **dict(instrumentation.get("extrinsics") or {}),
        }
        runtime_extrinsics["fingerprint_sha256"] = canonical_sha256(
            runtime_extrinsics
        )
        pipeline_config_fingerprint = canonical_sha256(
            {
                "diagnostics_schema_version": DIAGNOSTICS_SCHEMA_VERSION,
                "camera_info_fingerprint_sha256": runtime_camera_info[
                    "fingerprint_sha256"
                ],
                "extrinsics_fingerprint_sha256": runtime_extrinsics[
                    "fingerprint_sha256"
                ],
                "anchor_grid": [
                    self.anchor_adapter.grid_columns,
                    self.anchor_adapter.grid_rows,
                ],
                "anchor_range_m": [
                    self.anchor_adapter.minimum_range_m,
                    self.anchor_adapter.maximum_range_m,
                ],
                "calibration_cache_ttl_s": self.calibration_cache_ttl_s,
                "raw_range_formula": (
                    "physics_slant_range_m=filtered_optical_depth_m*"
                    "sqrt(1+x^2+y^2)"
                ),
                "residual_mode": self.range_residual_corrector.mode,
            }
        )
        anchor_payload = None
        if anchors is not None:
            anchor_payload = {
                "adapter_config": {
                    "grid_columns": self.anchor_adapter.grid_columns,
                    "grid_rows": self.anchor_adapter.grid_rows,
                    "minimum_range_m": self.anchor_adapter.minimum_range_m,
                    "maximum_range_m": self.anchor_adapter.maximum_range_m,
                    "minimum_ground_image_fraction": (
                        self.anchor_adapter.minimum_ground_image_fraction
                    ),
                    "maximum_local_inverse_depth_cv": (
                        self.anchor_adapter.maximum_local_inverse_depth_cv
                    ),
                    "ground_down_m": context.ground_down_m,
                },
                "candidate_count": anchors.candidate_count,
                "accepted_ground_anchor_count": (
                    anchors.accepted_ground_anchor_count
                ),
                "rejected_reason_counts": dict(
                    anchors.rejected_reason_counts
                ),
                "relative_inverse_depth_quantiles": list(
                    anchors.relative_inverse_depth_quantiles
                ),
                "metric_optical_depth_quantiles_m": list(
                    anchors.metric_depth_quantiles_m
                ),
                "spatial_bin_count": anchors.spatial_bin_count,
                "spatial_coverage_fraction": (
                    anchors.spatial_coverage_fraction
                ),
                "per_grid_point": [
                    item.as_dict() for item in anchors.diagnostics
                ],
            }
        target_payload = None
        if target_depth is not None:
            target_payload = {
                "valid": bool(target_depth.valid),
                "reason": str(target_depth.reason),
                "raw_roi_optical_depth_m": target_depth.optical_depth_m,
                "raw_roi_optical_depth_std_m": (
                    target_depth.optical_depth_std_m
                ),
                "raw_relative_inverse_depth": (
                    target_depth.relative_inverse_depth
                ),
                "raw_relative_inverse_depth_std": (
                    target_depth.relative_inverse_depth_std
                ),
                "valid_fraction": target_depth.valid_fraction,
                "sample_count": target_depth.sample_count,
                "raw_relative_inverse_depth_quantiles": list(
                    target_depth.roi_inverse_depth_quantiles
                ),
                "selected_foreground_statistic": (
                    target_depth.selected_foreground_statistic
                ),
                "output_semantics": "visible_surface",
                "surface_to_target_center_vector": {
                    "status": "unavailable_with_reason",
                    "reason": (
                        "target_model_surface_correspondence_not_observed"
                    ),
                },
            }
            if target_depth.variant_diagnostics is not None:
                target_payload["variant_diagnostics"] = (
                    target_depth.variant_diagnostics
                )
        inverse_payload = None
        if inverse_result is not None:
            inverse_payload = {
                "valid": bool(inverse_result.valid),
                "reason": str(inverse_result.reason),
                "raw_inverse_depth": inverse_result.raw_inverse_depth,
                "filtered_inverse_depth": (
                    inverse_result.filtered_inverse_depth
                ),
                "inverse_depth_std": inverse_result.inverse_depth_std,
                "robust_center": inverse_result.robust_center,
                "robust_sigma": inverse_result.robust_sigma,
                "sample_count": inverse_result.sample_count,
                "measurement_accepted": bool(
                    inverse_result.measurement_accepted
                ),
                "outlier": bool(inverse_result.outlier),
            }
        correction_payload = None
        if correction is not None:
            correction_payload = {
                "mode": self.range_residual_corrector.mode,
                "physics_distance_m": correction.physics_distance_m,
                "output_distance_m": correction.output_distance_m,
                "candidate_distance_m": correction.candidate_distance_m,
                "predicted_residual_m": correction.predicted_residual_m,
                "applied": bool(correction.applied),
                "reason": str(correction.reason),
                "measurement_usable": bool(correction.measurement_usable),
            }
        payload = {
            "diagnostics_schema_version": DIAGNOSTICS_SCHEMA_VERSION,
            "instrumentation_only": True,
            "run_id": collector.run_id,
            "target_id": collector.target_id,
            "group_id": context.dataset_group_id,
            "session_id": context.dataset_session_id,
            "frame_index": int(result.frame_index),
            "measurement_timestamp_s": float(
                result.measurement_timestamp_s
            ),
            "source_sim_timestamp_s": context.source_sim_timestamp_s,
            "track_epoch": self._track_epoch,
            "calibration_epoch": self._calibration_epoch,
            "stage": str(stage),
            "reason": str(reason),
            "reference_centers": {
                "raw_geometry_center_ned_m": list(
                    context.camera_position_ned_m
                ),
                "raw_geometry_center_source": instrumentation.get(
                    "raw_geometry_center_source",
                    "camera_position_ned_m_from_pose_provider",
                ),
                "ground_truth": instrumentation.get(
                    "ground_truth_reference_centers"
                ),
                "optical_center": instrumentation.get(
                    "optical_center"
                ),
                "target_reference": instrumentation.get(
                    "target_reference"
                ),
                "unavailable_fields": instrumentation.get(
                    "reference_center_unavailable_fields", []
                ),
            },
            "camera_info": runtime_camera_info,
            "extrinsics": runtime_extrinsics,
            "pipeline_config_fingerprint_sha256": (
                pipeline_config_fingerprint
            ),
            "timestamps": {
                "measurement_timestamp_s": result.measurement_timestamp_s,
                "measurement_clock": "dashboard_monotonic_receipt",
                "source_sim_timestamp_s": context.source_sim_timestamp_s,
                "source_sim_clock": "gazebo_sim_time",
                "depth_completed_timestamp_s": (
                    result.completed_timestamp_s
                ),
                "depth_completed_clock": "dashboard_monotonic",
                "depth_job_generation": getattr(
                    result, "generation", None
                ),
                "depth_submitted_timestamp_s": getattr(
                    result, "submitted_timestamp_s", None
                ),
                "depth_inference_started_timestamp_s": getattr(
                    result, "inference_started_timestamp_s", None
                ),
                "depth_queue_wait_ms": (
                    None
                    if getattr(result, "submitted_timestamp_s", None) is None
                    or getattr(
                        result, "inference_started_timestamp_s", None
                    ) is None
                    else max(
                        0.0,
                        result.inference_started_timestamp_s
                        - result.submitted_timestamp_s,
                    )
                    * 1000.0
                ),
                "depth_inference_ms": result.inference_ms,
                "depth_profiling_stages_s": (
                    None
                    if getattr(result, "profiling_stages", None) is None
                    else dict(result.profiling_stages)
                ),
                "consume_now_monotonic_s": now_s,
                "depth_result_age_ms": max(
                    0.0,
                    now_s - result.measurement_timestamp_s,
                )
                * 1000.0,
                "ground_truth_time_offset_ms": (
                    context.ground_truth_time_offset_ms
                ),
                **dict(instrumentation.get("timestamps") or {}),
            },
            "timestamp_stages": {
                "frame_receipt": timestamp_stage(
                    result.measurement_timestamp_s,
                    component="TrackingWeb",
                    execution_context="camera_frame_callback",
                    semantic="camera_frame_received_by_dashboard",
                ),
                "depth_submit": timestamp_stage(
                    getattr(result, "submitted_timestamp_s", None),
                    component="LatestDepthWorker",
                    execution_context="tracking_consumer_thread",
                    semantic="depth_job_owned_and_submitted",
                ),
                "depth_worker_start": timestamp_stage(
                    getattr(result, "inference_started_timestamp_s", None),
                    component="LatestDepthWorker",
                    execution_context=(
                        getattr(result, "worker_thread_name", None)
                        or "metric-depth-worker"
                    ),
                    semantic="depth_adapter_inference_started",
                ),
                "depth_complete": timestamp_stage(
                    result.completed_timestamp_s,
                    component="LatestDepthWorker",
                    execution_context=(
                        getattr(result, "worker_thread_name", None)
                        or "metric-depth-worker"
                    ),
                    semantic="depth_result_fully_constructed",
                ),
                "result_publish": timestamp_stage(
                    getattr(result, "result_publish_timestamp_s", None),
                    component="LatestDepthWorker",
                    execution_context=(
                        getattr(result, "publisher_thread_name", None)
                        or "metric-depth-worker"
                    ),
                    semantic="completed_result_published_to_latest_slot",
                ),
                "consumer_receive": timestamp_stage(
                    self._diagnostic_consumer_receive_timestamp_s,
                    component="MetricTargetFusion",
                    execution_context=threading.current_thread().name,
                    semantic="new_result_version_received_by_consumer",
                ),
                "consume": timestamp_stage(
                    self._diagnostic_consume_timestamp_s,
                    component="MetricTargetFusion",
                    execution_context=threading.current_thread().name,
                    semantic="completed_result_accepted_for_consumption",
                ),
            },
            "bbox": {
                "xywh_px": list(context.bbox_xywh),
                "center_px": list(context.bbox_center_px),
                "tracking_score": context.tracking_score,
            },
            "depth_model": {
                "source": (
                    None
                    if result.depth_map is None
                    else result.depth_map.source
                ),
                "model_version_or_checksum": instrumentation.get(
                    "depth_model_version_or_checksum"
                ),
            },
            "anchors": anchor_payload,
            "calibration": {
                "fit": self._calibration_diagnostics(fit_calibration),
                "applied": self._calibration_diagnostics(
                    applied_calibration
                ),
                "source": self._calibration_source,
                "fit_reason": self._current_calibration_fit_reason,
                "cache_age_s": calibration_cache_age_s,
                "cache_ttl_s": self.calibration_cache_ttl_s,
                "recovery": self.calibrator.recovery_status(),
                "raw_fit_timestamp_s": (
                    result.measurement_timestamp_s
                    if fit_calibration is not None
                    else None
                ),
                "applied_source_timestamp_s": (
                    self._last_valid_calibration_timestamp_s
                ),
                "calibration_age_s": (
                    None
                    if self._last_valid_calibration_timestamp_s is None
                    else max(
                        0.0,
                        result.measurement_timestamp_s
                        - self._last_valid_calibration_timestamp_s,
                    )
                ),
            },
            "target_depth": target_payload,
            "inverse_depth_filter": inverse_payload,
            "raw_range": {
                "image_ray_x": image_ray_x,
                "image_ray_y": image_ray_y,
                "ray_scale": ray_scale,
                "raw_roi_slant_range_m": raw_roi_slant_range_m,
                "filtered_optical_depth_m": filtered_optical_depth_m,
                "physics_slant_range_m": physics_slant_range_m,
                "formula": "physics_slant_range_m=filtered_optical_depth_m*sqrt(1+x^2+y^2)",
            },
            "applicability": (
                None
                if applicability is None
                else {
                    "applicable": bool(applicability.applicable),
                    "measurement_usable": bool(
                        applicability.measurement_usable
                    ),
                    "reason": str(applicability.reason),
                }
            ),
            "correction_observation": correction_payload,
            "ground_truth": {
                "distance_m": context.ground_truth_distance_m,
                "valid": context.ground_truth_valid,
                "reason": context.ground_truth_reason,
                "quality": context.ground_truth_quality,
                "uncertainty_m": context.ground_truth_uncertainty_m,
                "lever_arm_corrected": (
                    context.ground_truth_lever_arm_corrected
                ),
                "provider_diagnostics": instrumentation.get(
                    "ground_truth_provider"
                ),
                "timestamp_s": (
                    (instrumentation.get("timestamps") or {}).get(
                        "ground_truth_pose_sim_timestamp_s"
                    )
                ),
                "timestamp_clock_id": "gazebo_sim_time",
            },
        }
        collector.record(payload)

    def _consume_depth_result(
        self,
        result: Any,
        now_s: float,
        *,
        consumer_receive_timestamp_s: float | None = None,
        consume_timestamp_s: float | None = None,
    ) -> None:
        self._diagnostic_consumer_receive_timestamp_s = (
            time.monotonic()
            if consumer_receive_timestamp_s is None
            else float(consumer_receive_timestamp_s)
        )
        self._diagnostic_consume_timestamp_s = (
            time.monotonic()
            if consume_timestamp_s is None
            else float(consume_timestamp_s)
        )
        self._last_range_measurement_accepted = False
        self._last_range_uncertainty_components = {}
        is_prewarm = self._begin_prewarm_result(result, now_s)
        if not is_prewarm:
            self._last_target_inverse_depth_support = {}
        if not result.valid or result.depth_map is None:
            self._last_reason = result.reason
            early_context = getattr(result, "context", None)
            if isinstance(early_context, FusionFrameContext):
                self._record_physical_diagnostics(
                    result=result,
                    context=early_context,
                    now_s=now_s,
                    stage="depth_result_invalid",
                    reason=result.reason,
                )
            self._finish_prewarm_result(
                is_prewarm,
                result.reason,
                accepted=False,
            )
            return
        age_s = max(0.0, now_s - result.measurement_timestamp_s)
        if age_s > self.maximum_depth_age_s:
            self._last_reason = "depth_result_stale"
            stale_context = getattr(result, "context", None)
            if isinstance(stale_context, FusionFrameContext):
                self._record_physical_diagnostics(
                    result=result,
                    context=stale_context,
                    now_s=now_s,
                    stage="depth_result_stale",
                    reason=self._last_reason,
                )
            self._finish_prewarm_result(
                is_prewarm,
                self._last_reason,
                accepted=False,
            )
            return
        context = result.context
        if not isinstance(context, FusionFrameContext):
            self._last_reason = "depth_context_invalid"
            return
        inverse_depth = result.depth_map.inverse_depth
        try:
            anchors = self.anchor_adapter.anchors(
                inverse_depth,
                camera_position_ned_m=context.camera_position_ned_m,
                camera_quaternion_xyzw=context.camera_quaternion_xyzw,
                ground_down_m=context.ground_down_m,
                excluded_bbox_xywh=context.bbox_xywh,
            )
            self._last_anchor_candidate_count = anchors.candidate_count
            self._last_accepted_ground_anchor_count = (
                anchors.accepted_ground_anchor_count
            )
            self._last_anchor_rejected_reason_counts = dict(
                anchors.rejected_reason_counts
            )
            self._last_anchor_inverse_depth_quantiles = (
                anchors.relative_inverse_depth_quantiles
            )
            self._last_anchor_metric_depth_quantiles_m = (
                anchors.metric_depth_quantiles_m
            )
            self._last_anchor_spatial_bin_count = anchors.spatial_bin_count
            self._last_anchor_spatial_coverage_fraction = (
                anchors.spatial_coverage_fraction
            )
            calibration = self.calibrator.fit(
                anchors.relative_inverse_depth,
                anchors.metric_optical_depth_m,
            )
            fit_calibration = calibration
            if fit_calibration.change_point_reseeded:
                self._calibration_epoch += 1
            self._current_calibration_fit_reason = calibration.reason
            if calibration.valid and calibration.stable:
                self._last_valid_calibration_timestamp_s = (
                    result.measurement_timestamp_s
                )
                self._calibration_source = (
                    "prewarm" if context.calibration_only else "live"
                )
            if not calibration.valid:
                cache_age_s = (
                    math.inf
                    if self._last_valid_calibration_timestamp_s is None
                    else max(
                        0.0,
                        result.measurement_timestamp_s
                        - self._last_valid_calibration_timestamp_s,
                    )
                )
                cached = None
                if cache_age_s <= self.calibration_cache_ttl_s:
                    cached = self.calibrator.use_cached(
                        cache_age_s,
                        scale_drift_fraction_per_s=(
                            self.calibration_scale_drift_fraction_per_s
                        ),
                        offset_drift_m_inv_per_s=(
                            self.calibration_offset_drift_m_inv_per_s
                        ),
                    )
                if cached is None:
                    self._last_reason = calibration.reason
                    self._last_range_measurement_accepted = False
                    self._record_physical_diagnostics(
                        result=result,
                        context=context,
                        now_s=now_s,
                        stage="calibration_rejected",
                        reason=calibration.reason,
                        anchors=anchors,
                        fit_calibration=fit_calibration,
                        applied_calibration=None,
                        calibration_cache_age_s=(
                            None
                            if not math.isfinite(cache_age_s)
                            else cache_age_s
                        ),
                    )
                    self._finish_prewarm_result(
                        is_prewarm,
                        calibration.reason,
                        accepted=False,
                    )
                    if not context.calibration_only:
                        self.quality_gate.update(
                            self.ekf.estimate(now_s),
                            calibration_valid=False,
                            target_depth_valid=False,
                            calibration_reason=calibration.reason,
                        )
                    return
                calibration = cached
                self._calibration_source = "cached"
            else:
                cache_age_s = None
            if context.calibration_only:
                self._last_reason = (
                    "calibration_prewarmed"
                    if calibration.stable
                    else calibration.reason
                )
                self._finish_prewarm_result(
                    is_prewarm,
                    self._last_reason,
                    accepted=bool(calibration.valid and calibration.stable),
                )
                self._record_physical_diagnostics(
                    result=result,
                    context=context,
                    now_s=now_s,
                    stage="calibration_only",
                    reason=self._last_reason,
                    anchors=anchors,
                    fit_calibration=fit_calibration,
                    applied_calibration=calibration,
                    calibration_cache_age_s=cache_age_s,
                )
                return
            target_depth = self.extractor.extract(
                inverse_depth,
                context.bbox_xywh,
                self.calibrator,
            )
            if not target_depth.valid or target_depth.optical_depth_m is None:
                self._last_reason = target_depth.reason
                self._last_range_measurement_accepted = False
                self.quality_gate.update(
                    self.ekf.estimate(now_s),
                    calibration_valid=True,
                    target_depth_valid=False,
                    target_depth_reason=target_depth.reason,
                )
                self._record_physical_diagnostics(
                    result=result,
                    context=context,
                    now_s=now_s,
                    stage="target_depth_rejected",
                    reason=target_depth.reason,
                    anchors=anchors,
                    fit_calibration=fit_calibration,
                    applied_calibration=calibration,
                    calibration_cache_age_s=cache_age_s,
                    target_depth=target_depth,
                )
                return
            if (
                target_depth.relative_inverse_depth is None
                or target_depth.relative_inverse_depth_std is None
            ):
                self._last_reason = "target_inverse_depth_invalid"
                self._last_range_measurement_accepted = False
                self.quality_gate.update(
                    self.ekf.estimate(now_s),
                    calibration_valid=True,
                    calibration_stable=calibration.stable,
                    target_depth_valid=False,
                    inverse_depth_ready=False,
                )
                self._record_physical_diagnostics(
                    result=result,
                    context=context,
                    now_s=now_s,
                    stage="target_inverse_depth_rejected",
                    reason=self._last_reason,
                    anchors=anchors,
                    fit_calibration=fit_calibration,
                    applied_calibration=calibration,
                    calibration_cache_age_s=cache_age_s,
                    target_depth=target_depth,
                )
                return
            support = self.calibrator.target_inverse_depth_support(
                target_depth.relative_inverse_depth,
                margin_iqr=self.target_anchor_support_margin_iqr,
            )
            self._last_target_inverse_depth_support = support
            if not bool(support.get("supported", False)):
                support_reason = str(
                    support.get("reason")
                    or "target_inverse_depth_outside_anchor_support"
                )
                self._last_reason = support_reason
                self._last_range_measurement_accepted = False
                self.quality_gate.update(
                    self.ekf.estimate(now_s),
                    calibration_valid=True,
                    calibration_stable=calibration.stable,
                    target_depth_valid=False,
                    target_depth_reason=support_reason,
                    inverse_depth_ready=False,
                    range_measurement_accepted=False,
                )
                self._record_physical_diagnostics(
                    result=result,
                    context=context,
                    now_s=now_s,
                    stage="anchor_support_rejected",
                    reason=support_reason,
                    anchors=anchors,
                    fit_calibration=fit_calibration,
                    applied_calibration=calibration,
                    calibration_cache_age_s=cache_age_s,
                    target_depth=target_depth,
                )
                return
            inverse_result = self.bearing_range_filter.update_inverse_depth(
                target_depth.relative_inverse_depth,
                target_depth.relative_inverse_depth_std,
                result.measurement_timestamp_s,
            )
            if (
                not inverse_result.valid
                or inverse_result.filtered_inverse_depth is None
                or not inverse_result.measurement_accepted
            ):
                self._last_reason = inverse_result.reason
                self._last_range_measurement_accepted = False
                self.quality_gate.update(
                    self.ekf.estimate(now_s),
                    calibration_valid=True,
                    calibration_stable=calibration.stable,
                    target_depth_valid=True,
                    inverse_depth_ready=False,
                    range_measurement_accepted=False,
                )
                self._record_physical_diagnostics(
                    result=result,
                    context=context,
                    now_s=now_s,
                    stage="inverse_depth_filter_rejected",
                    reason=inverse_result.reason,
                    anchors=anchors,
                    fit_calibration=fit_calibration,
                    applied_calibration=calibration,
                    calibration_cache_age_s=cache_age_s,
                    target_depth=target_depth,
                    inverse_result=inverse_result,
                )
                return
            ray = np.asarray(
                context.filtered_bearing_ned_unit,
                dtype=np.float64,
            )
            normalized_x = (
                context.bbox_center_px[0] - context.frame_width / 2.0
            ) / self.projector.fx
            normalized_y = (
                context.bbox_center_px[1] - context.frame_height / 2.0
            ) / self.projector.fy
            ray_scale = math.sqrt(
                1.0 + normalized_x**2 + normalized_y**2
            )
            raw_ray_range = target_depth.optical_depth_m * ray_scale
            filtered_optical_depth = float(
                self.calibrator.metric_depth(
                    inverse_result.filtered_inverse_depth
                )
            )
            ray_range = filtered_optical_depth * ray_scale
            range_model_range_m: float | None = None
            range_model_risk_geometry = False
            if self.range_model_enabled:
                range_model_features = stage2a_model_features(
                    target_depth=target_depth,
                    inverse_result=inverse_result,
                    calibration=calibration,
                    normalized_x=normalized_x,
                    normalized_y=normalized_y,
                    camera_position_ned_m=context.camera_position_ned_m,
                    physics_slant_range_m=ray_range,
                )
                range_model_range_m = predict_range_m(range_model_features)
                if range_model_range_m is not None:
                    ray_range = range_model_range_m
                    # Known-unfixed failure mode: training scenarios with the
                    # target well off boresight showed the model tracking
                    # distance backwards. Not detectable from this frame alone,
                    # so matching geometry both inflates range_std below and is
                    # handed to the applicability gate, which abstains from the
                    # measurement outright in active mode -- an inflated sigma
                    # does not contain a sign error.
                    # See stage2a_range_model.py's module docstring.
                    range_model_risk_geometry = is_known_risk_geometry(
                        range_model_features
                    )
            inverse_metric = 1.0 / max(
                1e-6,
                filtered_optical_depth,
            )
            uncertainty = self.calibrator.inverse_metric_uncertainty(
                inverse_result.filtered_inverse_depth,
                float(inverse_result.inverse_depth_std or 0.0),
                inverse_result.sample_count,
            )
            inverse_metric_to_depth = 1.0 / max(
                1e-9,
                inverse_metric**2,
            )
            inverse_depth_depth_std = (
                uncertainty["inverse_depth_std_m_inv"]
                * inverse_metric_to_depth
            )
            parameter_depth_std = (
                uncertainty["parameter_std_m_inv"]
                * inverse_metric_to_depth
            )
            residual_depth_std = (
                uncertainty["residual_std_m_inv"]
                * inverse_metric_to_depth
            )
            roi_depth_std = float(
                target_depth.optical_depth_std_m or 0.5
            )
            # ROI metric MAD and propagated inverse-depth MAD describe the same
            # target-pixel spread. Use the more conservative one once, then add
            # independent calibration parameter and prediction uncertainties.
            sampling_depth_std = max(
                roi_depth_std,
                inverse_depth_depth_std,
            )
            metric_depth_std = math.sqrt(
                sampling_depth_std**2
                + parameter_depth_std**2
                + residual_depth_std**2
            )
            if range_model_range_m is not None:
                raw_range_std = max(0.15, SIZE_AGNOSTIC_RANGE_STD_M)
                if range_model_risk_geometry:
                    raw_range_std *= KNOWN_RISK_RANGE_STD_MULTIPLIER
            else:
                raw_range_std = max(0.15, metric_depth_std * ray_scale)
            anchor_depths = np.asarray(
                anchors.metric_optical_depth_m,
                dtype=np.float64,
            )
            anchor_median = (
                float(np.median(anchor_depths))
                if anchor_depths.size > 0
                else float("nan")
            )
            accepted_fraction = (
                float(anchors.accepted_ground_anchor_count)
                / max(1.0, float(anchors.candidate_count))
            )
            anchor_quality = max(
                0.0,
                min(
                    1.0,
                    accepted_fraction
                    * float(anchors.spatial_coverage_fraction),
                ),
            )
            bbox_width = max(0.0, float(context.bbox_xywh[2]))
            bbox_height = max(0.0, float(context.bbox_xywh[3]))
            bbox_area_fraction = min(
                1.0,
                bbox_width
                * bbox_height
                / max(1.0, float(context.frame_width * context.frame_height)),
            )
            bbox_center_x_fraction = (
                float(context.bbox_center_px[0])
                / max(1.0, float(context.frame_width))
            )
            bbox_center_y_fraction = (
                float(context.bbox_center_px[1])
                / max(1.0, float(context.frame_height))
            )
            camera_optical_axis = np.asarray(
                self.projector.pixel_to_ned_ray(
                    context.frame_width / 2.0,
                    context.frame_height / 2.0,
                    context.frame_width,
                    context.frame_height,
                    context.camera_quaternion_xyzw,
                ),
                dtype=np.float64,
            )
            calibration_condition_number = float(
                calibration.condition_number
                if calibration.condition_number is not None
                else float("nan")
            )
            calibration_residual = float(
                calibration.residual_m_inv
                if calibration.residual_m_inv is not None
                else float("nan")
            )
            calibration_inlier_fraction = (
                float(calibration.inlier_count)
                / max(1.0, float(calibration.anchor_count))
            )
            target_anchor_extrapolation_iqr = float(
                support.get("extrapolation_iqr", float("nan"))
            )
            ray_range_relative_std = raw_range_std / max(
                1.0e-6,
                abs(ray_range),
            )
            applicability = self.range_applicability_gate.evaluate(
                RangeApplicabilityInputs(
                    image_ray_x=normalized_x,
                    image_ray_y=normalized_y,
                    target_bearing_down=float(ray[2]),
                    camera_optical_axis_down=float(camera_optical_axis[2]),
                    calibration_condition_number=(
                        calibration_condition_number
                    ),
                    calibration_residual_m_inv=calibration_residual,
                    calibration_inlier_fraction=(
                        calibration_inlier_fraction
                    ),
                    anchor_spatial_coverage_fraction=float(
                        anchors.spatial_coverage_fraction
                    ),
                    target_anchor_extrapolation_iqr=(
                        target_anchor_extrapolation_iqr
                    ),
                    ray_range_relative_std=ray_range_relative_std,
                    range_model_risk_geometry=range_model_risk_geometry,
                )
            )
            previous_distance = (
                float(self._last_physics_distance_m)
                if self._last_physics_distance_m is not None
                else float("nan")
            )
            delta_time_s = (
                float(result.measurement_timestamp_s)
                - float(self._last_depth_measurement_s)
                if self._last_depth_measurement_s is not None
                else float("nan")
            )
            residual_features = RangeResidualFeatures(
                m52_anchor_median_m=anchor_median,
                m52_anchor_quality=anchor_quality,
                midas_target_inverse_depth_median=float(
                    inverse_result.filtered_inverse_depth
                ),
                midas_target_inverse_depth_spread=float(
                    inverse_result.inverse_depth_std or 0.0
                ),
                bbox_width_px=bbox_width,
                bbox_height_px=bbox_height,
                bbox_area_fraction=bbox_area_fraction,
                previous_physics_distance_m=previous_distance,
                delta_time_s=delta_time_s,
                bbox_center_x_fraction=bbox_center_x_fraction,
                bbox_center_y_fraction=bbox_center_y_fraction,
                image_ray_x=normalized_x,
                image_ray_y=normalized_y,
                target_bearing_down=float(ray[2]),
                camera_optical_axis_down=float(camera_optical_axis[2]),
                calibration_scale=float(
                    calibration.scale
                    if calibration.scale is not None
                    else float("nan")
                ),
                calibration_offset=float(
                    calibration.offset
                    if calibration.offset is not None
                    else float("nan")
                ),
                calibration_residual_m_inv=calibration_residual,
                calibration_condition_number_log10=math.log10(
                    max(1.0, calibration_condition_number)
                ),
                calibration_inlier_fraction=(
                    calibration_inlier_fraction
                ),
                anchor_spatial_coverage_fraction=float(
                    anchors.spatial_coverage_fraction
                ),
                target_anchor_extrapolation_iqr=(
                    target_anchor_extrapolation_iqr
                ),
                ray_range_relative_std=ray_range_relative_std,
            )
            correction = self.range_residual_corrector.correct(
                residual_features,
                ray_range,
            )
            self._record_physical_diagnostics(
                result=result,
                context=context,
                now_s=now_s,
                stage="raw_range_computed",
                reason="observed_before_range_filter",
                anchors=anchors,
                fit_calibration=fit_calibration,
                applied_calibration=calibration,
                calibration_cache_age_s=cache_age_s,
                target_depth=target_depth,
                inverse_result=inverse_result,
                image_ray_x=normalized_x,
                image_ray_y=normalized_y,
                ray_scale=ray_scale,
                raw_roi_slant_range_m=raw_ray_range,
                filtered_optical_depth_m=filtered_optical_depth,
                physics_slant_range_m=correction.physics_distance_m,
                applicability=applicability,
                correction=correction,
            )
            if self.dataset_collector.enabled:
                self.dataset_collector.record(
                    features=residual_features,
                    physics_distance_m=correction.physics_distance_m,
                    correction=correction,
                    group_id=context.dataset_group_id,
                    session_id=context.dataset_session_id,
                    frame_index=int(result.frame_index),
                    measurement_timestamp_s=float(
                        result.measurement_timestamp_s
                    ),
                    source_sim_timestamp_s=context.source_sim_timestamp_s,
                    ground_truth_distance_m=(
                        context.ground_truth_distance_m
                    ),
                    ground_truth_valid=context.ground_truth_valid,
                    ground_truth_reason=context.ground_truth_reason,
                    ground_truth_uncertainty_m=(
                        context.ground_truth_uncertainty_m
                    ),
                    ground_truth_time_offset_ms=(
                        context.ground_truth_time_offset_ms
                    ),
                    ground_truth_quality=context.ground_truth_quality,
                    ground_truth_lever_arm_corrected=(
                        context.ground_truth_lever_arm_corrected
                    ),
                )
            self._last_physics_distance_m = correction.physics_distance_m
            ray_range = correction.output_distance_m
            if correction.applied:
                raw_range_std = math.hypot(
                    raw_range_std,
                    correction.residual_prediction_std_m,
                )
            self._last_range_uncertainty_components = {
                **uncertainty,
                "inverse_depth_depth_std_m": inverse_depth_depth_std,
                "parameter_depth_std_m": parameter_depth_std,
                "residual_depth_std_m": residual_depth_std,
                "roi_depth_std_m": roi_depth_std,
                "sampling_depth_std_m": sampling_depth_std,
                "metric_depth_std_m": metric_depth_std,
                "ray_range_std_m": raw_range_std,
                "xgboost_residual_prediction_std_m": (
                    correction.residual_prediction_std_m
                ),
            }
            self._last_raw_target_range_m = raw_ray_range
            self._last_depth_measurement_s = result.measurement_timestamp_s
            if (
                not applicability.measurement_usable
                or not correction.measurement_usable
            ):
                rejection_reason = (
                    applicability.reason
                    if not applicability.measurement_usable
                    else correction.reason
                )
                self._last_reason = rejection_reason
                self._last_range_measurement_accepted = False
                self.quality_gate.update(
                    self.ekf.estimate(now_s),
                    calibration_valid=True,
                    target_depth_valid=True,
                    calibration_stable=calibration.stable,
                    inverse_depth_ready=True,
                    range_measurement_accepted=False,
                    range_uncertainty_valid=True,
                    range_rejection_reason=rejection_reason,
                )
                return
            range_result = self.bearing_range_filter.update_range(
                ray_range,
                raw_range_std,
                result.measurement_timestamp_s,
            )
            if not range_result.valid or range_result.range_m is None:
                self._last_reason = range_result.reason
                self._last_range_measurement_accepted = False
                self.quality_gate.update(
                    self.ekf.estimate(now_s),
                    calibration_valid=True,
                    target_depth_valid=True,
                    calibration_stable=calibration.stable,
                    inverse_depth_ready=True,
                    range_measurement_accepted=False,
                    range_uncertainty_valid=(
                        range_result.reason
                        != "range_uncertainty_too_large"
                    ),
                    range_rejection_reason=range_result.reason,
                )
                return
            if not range_result.measurement_accepted:
                # Predict through a short rejected-range burst. Feeding the
                # held value repeatedly would create false EKF confidence.
                self._last_reason = range_result.reason
                self._last_range_measurement_accepted = False
                self.quality_gate.update(
                    self.ekf.estimate(now_s),
                    calibration_valid=True,
                    calibration_stable=calibration.stable,
                    target_depth_valid=True,
                    inverse_depth_ready=True,
                    range_measurement_accepted=False,
                    range_uncertainty_valid=(
                        range_result.reason
                        != "range_uncertainty_too_large"
                    ),
                    range_rejection_reason=range_result.reason,
                )
                return
            ray_range = range_result.range_m
            range_std = float(
                range_result.range_std_m
                if range_result.range_std_m is not None
                else raw_range_std
            )
            camera = np.asarray(
                context.camera_position_ned_m,
                dtype=np.float64,
            )
            measured_position = camera + ray * ray_range
            angular_std = self.angular_noise_px / max(
                1.0,
                0.5 * (self.projector.fx + self.projector.fy),
            )
            perpendicular = np.eye(3) - np.outer(ray, ray)
            covariance = (
                np.outer(ray, ray) * range_std**2
                + perpendicular * (ray_range * angular_std) ** 2
                + np.eye(3) * 0.01
            )
            if self.ground_target_mode:
                bottom_v = (
                    context.bbox_xywh[1] + context.bbox_xywh[3]
                )
                ground_ray = np.asarray(
                    self.projector.pixel_to_ned_ray(
                        context.bbox_center_px[0],
                        bottom_v,
                        context.frame_width,
                        context.frame_height,
                        context.camera_quaternion_xyzw,
                    ),
                    dtype=np.float64,
                )
                if ground_ray[2] > 1e-4:
                    ground_range = (
                        context.ground_down_m - camera[2]
                    ) / ground_ray[2]
                    if 0.5 <= ground_range <= 100.0:
                        ground_position = camera + ground_ray * ground_range
                        # Bottom-center estimates the contact point. Keep the
                        # depth-derived horizontal position but constrain D.
                        measured_position[2] = ground_position[2]
                        covariance[2, 2] = min(covariance[2, 2], 0.25)
            self.ekf.update_position(
                measured_position,
                covariance,
                result.measurement_timestamp_s,
            )
            estimate = self.ekf.estimate(
                now_s,
                stale_timeout_s=self.dropout_timeout_s,
            )
            quality = self.quality_gate.update(
                estimate,
                calibration_valid=True,
                target_depth_valid=True,
                calibration_stable=calibration.stable,
                inverse_depth_ready=True,
                range_measurement_accepted=True,
                range_uncertainty_valid=True,
                tracking_quality_valid=(
                    context.tracking_score >= 0.70
                ),
            )
            self._last_reason = quality.reason
            self._last_target_range_m = ray_range
            self._last_range_std_m = range_std
            self._last_range_measurement_accepted = True
            if quality.ready:
                self._active_metric_source = "m52_midas"
        except (ArithmeticError, TypeError, ValueError) as error:
            self._last_reason = f"fusion_update_failed:{error}"
            self._finish_prewarm_result(
                is_prewarm,
                self._last_reason,
                accepted=False,
            )

    def _as_target_estimate(
        self,
        estimate: Any,
        now_s: float,
        reason: str,
        *,
        force_invalid: bool = False,
    ) -> TargetEstimate:
        measurement_timestamp = (
            self.ekf.last_measurement_timestamp_s
            if self.ekf.last_measurement_timestamp_s is not None
            else now_s
        )
        age_ms = max(0.0, now_s - measurement_timestamp) * 1000.0
        position = estimate.position_ned_m
        multiview_active = self._active_metric_source == "multiview"
        range_m = (
            self._last_multiview_range_m
            if multiview_active
            else self._last_target_range_m
        )
        range_std_m = (
            self._last_multiview_range_std_m
            if multiview_active
            else self._last_range_std_m
        )
        active_quality_gate = (
            self.multiview_quality_gate
            if multiview_active
            else self.quality_gate
        )
        return TargetEstimate(
            timestamp_s=float(measurement_timestamp),
            state="METRIC_FUSION",
            valid=bool(estimate.valid and self.ready and not force_invalid),
            reason=reason,
            position_ned_m=position,
            velocity_ned_m_s=estimate.velocity_ned_m_s,
            covariance=estimate.covariance,
            range_m=range_m,
            range_std_m=range_std_m,
            observation_count=int(estimate.update_count),
            observability_score=float(active_quality_gate.last.quality),
            estimate_age_ms=age_ms,
            bootstrap_progress=min(
                1.0,
                active_quality_gate.last.stable_samples
                / max(1, active_quality_gate.stable_samples_required),
            ),
            velocity_valid=bool(estimate.velocity_valid),
            estimator_mode=str(estimate.estimator_mode),
            stationary_probability=float(
                estimate.stationary_probability
            ),
            moving_probability=float(estimate.moving_probability),
        )

    def _invalid(self, now_s: float, reason: str) -> TargetEstimate:
        return TargetEstimate(
            timestamp_s=float(now_s),
            state="METRIC_FUSION",
            valid=False,
            reason=reason,
        )
