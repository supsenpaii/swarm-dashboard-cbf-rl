from __future__ import annotations

import math
import os
from dataclasses import asdict, dataclass
from typing import Any


VALID_APPLICABILITY_MODES = {"off", "shadow", "active"}


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


@dataclass(frozen=True)
class RangeApplicabilityInputs:
    image_ray_x: float
    image_ray_y: float
    target_bearing_down: float
    camera_optical_axis_down: float
    calibration_condition_number: float
    calibration_residual_m_inv: float
    calibration_inlier_fraction: float
    anchor_spatial_coverage_fraction: float
    target_anchor_extrapolation_iqr: float
    ray_range_relative_std: float
    # True only when the learned Stage 2A range model produced this frame's
    # range AND the frame matches the geometry of the training groups whose
    # within-group slope is negative (the model reports distance moving
    # opposite to the truth). Defaults False so callers that never enable the
    # learned model are unaffected. See stage2a_range_model.is_known_risk_geometry.
    range_model_risk_geometry: bool = False

    def validated(self) -> RangeApplicabilityInputs:
        values = asdict(self)
        if not all(math.isfinite(float(value)) for value in values.values()):
            raise ValueError("applicability_input_not_finite")
        if not -4.0 <= self.image_ray_x <= 4.0:
            raise ValueError("applicability_image_ray_x_invalid")
        if not -4.0 <= self.image_ray_y <= 4.0:
            raise ValueError("applicability_image_ray_y_invalid")
        if not -1.0 <= self.target_bearing_down <= 1.0:
            raise ValueError("applicability_target_bearing_invalid")
        if not -1.0 <= self.camera_optical_axis_down <= 1.0:
            raise ValueError("applicability_camera_axis_invalid")
        if self.calibration_condition_number <= 0.0:
            raise ValueError("applicability_calibration_condition_invalid")
        if self.calibration_residual_m_inv < 0.0:
            raise ValueError("applicability_calibration_residual_invalid")
        if not 0.0 <= self.calibration_inlier_fraction <= 1.0:
            raise ValueError("applicability_calibration_inlier_invalid")
        if not 0.0 <= self.anchor_spatial_coverage_fraction <= 1.0:
            raise ValueError("applicability_anchor_coverage_invalid")
        if self.target_anchor_extrapolation_iqr < 0.0:
            raise ValueError("applicability_anchor_extrapolation_invalid")
        if self.ray_range_relative_std < 0.0:
            raise ValueError("applicability_range_uncertainty_invalid")
        return self

    def as_dict(self) -> dict[str, float]:
        return {
            key: float(value)
            for key, value in asdict(self).items()
        }


@dataclass(frozen=True)
class RangeApplicabilityResult:
    applicable: bool
    enforced: bool
    measurement_usable: bool
    reason: str


class RangeApplicabilityGate:
    """Deterministic containment gate before range filtering and the EKF.

    The active envelope is deliberately conservative: it matches the locked
    front-facing evidence and abstains from the oblique geometry that produced
    catastrophic M52+MiDaS ranges. Shadow mode exposes the same decision
    without changing the physics pipeline.
    """

    def __init__(
        self,
        *,
        mode: str = "active",
        maximum_abs_image_ray_x: float = 0.45,
        maximum_abs_image_ray_y: float = 0.85,
        maximum_calibration_condition_number: float = 1.0e5,
        maximum_calibration_residual_m_inv: float = 0.025,
        minimum_calibration_inlier_fraction: float = 0.45,
        minimum_anchor_spatial_coverage_fraction: float = 0.25,
        maximum_target_anchor_extrapolation_iqr: float = 0.50,
        maximum_ray_range_relative_std: float = 0.30,
    ) -> None:
        selected_mode = str(mode).strip().lower()
        self.mode = (
            selected_mode
            if selected_mode in VALID_APPLICABILITY_MODES
            else "off"
        )
        self.load_error = (
            ""
            if selected_mode in VALID_APPLICABILITY_MODES
            else f"invalid_mode:{selected_mode}"
        )
        self.maximum_abs_image_ray_x = max(
            0.05, min(4.0, float(maximum_abs_image_ray_x))
        )
        self.maximum_abs_image_ray_y = max(
            0.05, min(4.0, float(maximum_abs_image_ray_y))
        )
        self.maximum_calibration_condition_number = max(
            10.0, float(maximum_calibration_condition_number)
        )
        self.maximum_calibration_residual_m_inv = max(
            1.0e-6, float(maximum_calibration_residual_m_inv)
        )
        self.minimum_calibration_inlier_fraction = max(
            0.0, min(1.0, float(minimum_calibration_inlier_fraction))
        )
        self.minimum_anchor_spatial_coverage_fraction = max(
            0.0,
            min(1.0, float(minimum_anchor_spatial_coverage_fraction)),
        )
        self.maximum_target_anchor_extrapolation_iqr = max(
            0.0, float(maximum_target_anchor_extrapolation_iqr)
        )
        self.maximum_ray_range_relative_std = max(
            0.01, float(maximum_ray_range_relative_std)
        )
        self.reset()

    @classmethod
    def from_environment(cls) -> RangeApplicabilityGate:
        return cls(
            mode=os.environ.get(
                "SWARM_RANGE_APPLICABILITY_MODE", "active"
            ),
            maximum_abs_image_ray_x=_env_float(
                "SWARM_RANGE_APPLICABILITY_MAX_ABS_IMAGE_RAY_X",
                0.45,
                0.05,
                4.0,
            ),
            maximum_abs_image_ray_y=_env_float(
                "SWARM_RANGE_APPLICABILITY_MAX_ABS_IMAGE_RAY_Y",
                0.85,
                0.05,
                4.0,
            ),
            maximum_calibration_condition_number=_env_float(
                "SWARM_RANGE_APPLICABILITY_MAX_CALIBRATION_CONDITION",
                1.0e5,
                10.0,
                1.0e9,
            ),
            maximum_calibration_residual_m_inv=_env_float(
                "SWARM_RANGE_APPLICABILITY_MAX_CALIBRATION_RESIDUAL_M_INV",
                0.025,
                1.0e-6,
                1.0,
            ),
            minimum_calibration_inlier_fraction=_env_float(
                "SWARM_RANGE_APPLICABILITY_MIN_CALIBRATION_INLIER_FRACTION",
                0.45,
                0.0,
                1.0,
            ),
            minimum_anchor_spatial_coverage_fraction=_env_float(
                "SWARM_RANGE_APPLICABILITY_MIN_ANCHOR_SPATIAL_COVERAGE",
                0.25,
                0.0,
                1.0,
            ),
            maximum_target_anchor_extrapolation_iqr=_env_float(
                "SWARM_RANGE_APPLICABILITY_MAX_TARGET_EXTRAPOLATION_IQR",
                0.50,
                0.0,
                5.0,
            ),
            maximum_ray_range_relative_std=_env_float(
                "SWARM_RANGE_APPLICABILITY_MAX_RANGE_RELATIVE_STD",
                0.30,
                0.01,
                5.0,
            ),
        )

    def reset(self) -> None:
        self.request_count = 0
        self.applicable_count = 0
        self.rejected_count = 0
        self.enforced_rejection_count = 0
        self.last_reason = "disabled" if self.mode == "off" else "waiting_sample"
        self.last_inputs: dict[str, float] = {}

    def evaluate(
        self,
        inputs: RangeApplicabilityInputs,
    ) -> RangeApplicabilityResult:
        self.request_count += 1
        try:
            values = inputs.validated()
            self.last_inputs = values.as_dict()
            reason = self._rejection_reason(values)
        except (TypeError, ValueError) as error:
            reason = str(error)
        applicable = not reason
        if applicable:
            self.applicable_count += 1
            self.last_reason = "applicable"
        else:
            self.rejected_count += 1
            self.last_reason = reason
        enforced = self.mode == "active" and not applicable
        if enforced:
            self.enforced_rejection_count += 1
        return RangeApplicabilityResult(
            applicable=applicable,
            enforced=enforced,
            measurement_usable=not enforced,
            reason=("applicable" if applicable else reason),
        )

    def _rejection_reason(
        self,
        values: RangeApplicabilityInputs,
    ) -> str:
        if abs(values.image_ray_x) > self.maximum_abs_image_ray_x:
            return "image_ray_x_outside_validated_envelope"
        if abs(values.image_ray_y) > self.maximum_abs_image_ray_y:
            return "image_ray_y_outside_validated_envelope"
        if (
            values.calibration_condition_number
            > self.maximum_calibration_condition_number
        ):
            return "calibration_condition_outside_limit"
        if (
            values.calibration_residual_m_inv
            > self.maximum_calibration_residual_m_inv
        ):
            return "calibration_residual_outside_limit"
        if (
            values.calibration_inlier_fraction
            < self.minimum_calibration_inlier_fraction
        ):
            return "calibration_inlier_fraction_too_low"
        if (
            values.anchor_spatial_coverage_fraction
            < self.minimum_anchor_spatial_coverage_fraction
        ):
            return "anchor_spatial_coverage_too_low"
        if (
            values.target_anchor_extrapolation_iqr
            > self.maximum_target_anchor_extrapolation_iqr
        ):
            return "target_anchor_extrapolation_outside_limit"
        if (
            values.ray_range_relative_std
            > self.maximum_ray_range_relative_std
        ):
            return "range_relative_uncertainty_too_large"
        if values.range_model_risk_geometry:
            # Inflated uncertainty alone does not contain a sign error: a
            # confidently-wrong direction still steers a follow controller the
            # wrong way. Abstain instead, consistent with how this gate treats
            # the oblique geometry it was built for.
            return "range_model_known_risk_geometry"
        return ""

    def status(self) -> dict[str, Any]:
        return {
            "mode": self.mode,
            "load_error": self.load_error,
            "request_count": self.request_count,
            "applicable_count": self.applicable_count,
            "rejected_count": self.rejected_count,
            "enforced_rejection_count": self.enforced_rejection_count,
            "last_reason": self.last_reason,
            "last_inputs": dict(self.last_inputs),
            "limits": {
                "maximum_abs_image_ray_x": self.maximum_abs_image_ray_x,
                "maximum_abs_image_ray_y": self.maximum_abs_image_ray_y,
                "maximum_calibration_condition_number": (
                    self.maximum_calibration_condition_number
                ),
                "maximum_calibration_residual_m_inv": (
                    self.maximum_calibration_residual_m_inv
                ),
                "minimum_calibration_inlier_fraction": (
                    self.minimum_calibration_inlier_fraction
                ),
                "minimum_anchor_spatial_coverage_fraction": (
                    self.minimum_anchor_spatial_coverage_fraction
                ),
                "maximum_target_anchor_extrapolation_iqr": (
                    self.maximum_target_anchor_extrapolation_iqr
                ),
                "maximum_ray_range_relative_std": (
                    self.maximum_ray_range_relative_std
                ),
            },
        }
