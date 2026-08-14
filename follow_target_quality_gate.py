from __future__ import annotations

from dataclasses import dataclass

from target_fusion_ekf import FusionEstimate


@dataclass(frozen=True)
class FollowTargetQuality:
    ready: bool
    reason: str
    quality: float
    stable_samples: int


class FollowTargetQualityGate:
    def __init__(
        self,
        *,
        stable_samples_required: int = 4,
        maximum_position_std_m: float = 2.5,
        maximum_velocity_std_m_s: float = 3.0,
    ) -> None:
        self.stable_samples_required = max(1, stable_samples_required)
        self.maximum_position_std_m = max(0.1, maximum_position_std_m)
        self.maximum_velocity_std_m_s = max(
            0.1,
            maximum_velocity_std_m_s,
        )
        self.stable_samples = 0
        self.initialized = False
        self.last = FollowTargetQuality(False, "uninitialized", 0.0, 0)

    def reset(self) -> None:
        self.stable_samples = 0
        self.initialized = False
        self.last = FollowTargetQuality(False, "uninitialized", 0.0, 0)

    def update(
        self,
        estimate: FusionEstimate,
        *,
        calibration_valid: bool,
        target_depth_valid: bool,
        calibration_stable: bool = True,
        inverse_depth_ready: bool = True,
        range_measurement_accepted: bool = True,
        range_uncertainty_valid: bool = True,
        tracking_quality_valid: bool = True,
        calibration_reason: str = "",
        target_depth_reason: str = "",
        range_rejection_reason: str = "",
    ) -> FollowTargetQuality:
        reason = "ok"
        valid = True
        if not calibration_valid:
            valid, reason = (
                False,
                calibration_reason or "metric_calibration_invalid",
            )
        elif not calibration_stable:
            valid, reason = False, "metric_calibration_stabilizing"
        elif not tracking_quality_valid:
            valid, reason = False, "tracking_quality_low"
        elif not inverse_depth_ready:
            valid, reason = False, "inverse_depth_filter_warming_up"
        elif not target_depth_valid:
            valid, reason = (
                False,
                target_depth_reason or "target_depth_invalid",
            )
        elif not range_uncertainty_valid:
            valid, reason = False, "range_uncertainty_too_large"
        elif not range_measurement_accepted:
            valid, reason = (
                False,
                range_rejection_reason or "range_measurement_rejected",
            )
        elif not estimate.valid:
            valid, reason = False, estimate.reason
        elif (
            estimate.position_std_m is None
            or estimate.position_std_m > self.maximum_position_std_m
        ):
            valid, reason = False, "position_uncertainty"
        elif (
            estimate.velocity_std_m_s is not None
            and estimate.velocity_std_m_s > self.maximum_velocity_std_m_s
            and estimate.update_count >= 2
        ):
            valid, reason = False, "velocity_uncertainty"
        self.stable_samples = self.stable_samples + 1 if valid else 0
        ready = bool(
            valid and self.stable_samples >= self.stable_samples_required
        )
        self.initialized = bool(self.initialized or ready)
        position_ratio = (
            0.0
            if estimate.position_std_m is None
            else max(
                0.0,
                1.0
                - estimate.position_std_m / self.maximum_position_std_m,
            )
        )
        quality = max(
            0.0,
            min(
                1.0,
                0.5 * position_ratio
                + 0.5 * min(
                    1.0,
                    self.stable_samples / self.stable_samples_required,
                ),
            ),
        )
        self.last = FollowTargetQuality(
            ready,
            "ready" if ready else reason,
            quality,
            self.stable_samples,
        )
        return self.last
