from __future__ import annotations

import hashlib
import json
import math
import os
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Mapping

import numpy as np


FEATURE_SCHEMA_VERSION = "m52_midas_range_features_v2"
FEATURE_NAMES = (
    "m52_anchor_median_m",
    "m52_anchor_quality",
    "midas_target_inverse_depth_median",
    "midas_target_inverse_depth_spread",
    "bbox_width_px",
    "bbox_height_px",
    "bbox_area_fraction",
    "previous_physics_distance_m",
    "delta_time_s",
    "bbox_center_x_fraction",
    "bbox_center_y_fraction",
    "image_ray_x",
    "image_ray_y",
    "target_bearing_down",
    "camera_optical_axis_down",
    "calibration_scale",
    "calibration_offset",
    "calibration_residual_m_inv",
    "calibration_condition_number_log10",
    "calibration_inlier_fraction",
    "anchor_spatial_coverage_fraction",
    "target_anchor_extrapolation_iqr",
    "ray_range_relative_std",
)
FEATURE_TYPES = ("float64",) * len(FEATURE_NAMES)
MISSING_VALUE_POLICY = "reject_sample"
BUNDLE_VERSION = "m52_midas_range_residual_bundle_v2"
VALID_MODES = {"off", "shadow", "active"}


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def feature_schema() -> dict[str, Any]:
    return {
        "version": FEATURE_SCHEMA_VERSION,
        "names": list(FEATURE_NAMES),
        "types": list(FEATURE_TYPES),
        "missing_value_policy": MISSING_VALUE_POLICY,
    }


@dataclass(frozen=True)
class RangeResidualFeatures:
    m52_anchor_median_m: float
    m52_anchor_quality: float
    midas_target_inverse_depth_median: float
    midas_target_inverse_depth_spread: float
    bbox_width_px: float
    bbox_height_px: float
    bbox_area_fraction: float
    previous_physics_distance_m: float
    delta_time_s: float
    bbox_center_x_fraction: float
    bbox_center_y_fraction: float
    image_ray_x: float
    image_ray_y: float
    target_bearing_down: float
    camera_optical_axis_down: float
    calibration_scale: float
    calibration_offset: float
    calibration_residual_m_inv: float
    calibration_condition_number_log10: float
    calibration_inlier_fraction: float
    anchor_spatial_coverage_fraction: float
    target_anchor_extrapolation_iqr: float
    ray_range_relative_std: float

    def values(self) -> tuple[float, ...]:
        return tuple(float(getattr(self, name)) for name in FEATURE_NAMES)

    def as_array(self) -> np.ndarray:
        values = self.values()
        if len(values) != len(FEATURE_NAMES):
            raise ValueError("feature_count_mismatch")
        if not all(math.isfinite(value) for value in values):
            raise ValueError("feature_not_finite")
        if self.m52_anchor_median_m <= 0.0:
            raise ValueError("m52_anchor_median_invalid")
        if not 0.0 <= self.m52_anchor_quality <= 1.0:
            raise ValueError("m52_anchor_quality_invalid")
        if self.midas_target_inverse_depth_median <= 0.0:
            raise ValueError("midas_inverse_depth_invalid")
        if self.midas_target_inverse_depth_spread < 0.0:
            raise ValueError("midas_inverse_depth_spread_invalid")
        if self.bbox_width_px <= 0.0 or self.bbox_height_px <= 0.0:
            raise ValueError("bbox_size_invalid")
        if not 0.0 < self.bbox_area_fraction <= 1.0:
            raise ValueError("bbox_area_fraction_invalid")
        if self.previous_physics_distance_m <= 0.0:
            raise ValueError("previous_physics_distance_invalid")
        if self.delta_time_s <= 0.0:
            raise ValueError("delta_time_invalid")
        if not 0.0 <= self.bbox_center_x_fraction <= 1.0:
            raise ValueError("bbox_center_x_fraction_invalid")
        if not 0.0 <= self.bbox_center_y_fraction <= 1.0:
            raise ValueError("bbox_center_y_fraction_invalid")
        if not -4.0 <= self.image_ray_x <= 4.0:
            raise ValueError("image_ray_x_invalid")
        if not -4.0 <= self.image_ray_y <= 4.0:
            raise ValueError("image_ray_y_invalid")
        if not -1.0 <= self.target_bearing_down <= 1.0:
            raise ValueError("target_bearing_down_invalid")
        if not -1.0 <= self.camera_optical_axis_down <= 1.0:
            raise ValueError("camera_optical_axis_down_invalid")
        if self.calibration_scale <= 0.0:
            raise ValueError("calibration_scale_invalid")
        if self.calibration_residual_m_inv < 0.0:
            raise ValueError("calibration_residual_invalid")
        if not 0.0 <= self.calibration_condition_number_log10 <= 12.0:
            raise ValueError("calibration_condition_invalid")
        if not 0.0 <= self.calibration_inlier_fraction <= 1.0:
            raise ValueError("calibration_inlier_fraction_invalid")
        if not 0.0 <= self.anchor_spatial_coverage_fraction <= 1.0:
            raise ValueError("anchor_spatial_coverage_invalid")
        if self.target_anchor_extrapolation_iqr < 0.0:
            raise ValueError("target_anchor_extrapolation_invalid")
        if self.ray_range_relative_std < 0.0:
            raise ValueError("ray_range_relative_std_invalid")
        return np.asarray(values, dtype=np.float64).reshape(1, -1)

    def as_dict(self) -> dict[str, float]:
        return dict(zip(FEATURE_NAMES, self.values(), strict=True))


class _XGBoostBoosterPredictor:
    def __init__(self, xgboost_module: Any, booster: Any) -> None:
        self._xgboost = xgboost_module
        self._booster = booster

    def predict(self, features: np.ndarray) -> np.ndarray:
        return np.asarray(
            self._booster.predict(self._xgboost.DMatrix(features)),
            dtype=np.float64,
        )


@dataclass(frozen=True)
class LoadedRangeResidualBundle:
    manifest: Mapping[str, Any]
    scaler: Any
    applicability_predictor: Any
    predictor: Any
    manifest_sha256: str

    @property
    def model_input(self) -> str:
        return str(self.manifest["model_input"])

    @property
    def maximum_absolute_z_score(self) -> float:
        return float(self.manifest["maximum_absolute_z_score"])

    @property
    def maximum_absolute_residual_m(self) -> float:
        return float(self.manifest["maximum_absolute_residual_m"])

    @property
    def residual_prediction_std_m(self) -> float:
        return float(self.manifest["residual_prediction_std_m"])

    @property
    def ood_mean(self) -> np.ndarray:
        return np.asarray(
            self.manifest["ood"]["scaled_feature_mean"],
            dtype=np.float64,
        )

    @property
    def ood_inverse_covariance(self) -> np.ndarray:
        return np.asarray(
            self.manifest["ood"]["scaled_inverse_covariance"],
            dtype=np.float64,
        )

    @property
    def maximum_mahalanobis_distance(self) -> float:
        return float(self.manifest["ood"]["maximum_mahalanobis_distance"])

    @property
    def minimum_applicability_probability(self) -> float:
        return float(self.manifest["minimum_applicability_probability"])


def _validated_artifact_path(bundle_dir: Path, filename: Any) -> Path:
    name = str(filename)
    if not name or Path(name).name != name:
        raise ValueError("bundle_artifact_filename_invalid")
    path = bundle_dir / name
    if not path.is_file():
        raise ValueError(f"bundle_artifact_missing:{name}")
    return path


def load_range_residual_bundle(bundle_dir: str | os.PathLike[str]) -> (
    LoadedRangeResidualBundle
):
    root = Path(bundle_dir)
    manifest_path = root / "manifest.json"
    if not manifest_path.is_file():
        raise ValueError("bundle_manifest_missing")
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    if not isinstance(manifest, dict):
        raise ValueError("bundle_manifest_invalid")
    expected = {
        "bundle_version": BUNDLE_VERSION,
        "feature_schema_version": FEATURE_SCHEMA_VERSION,
        "feature_names": list(FEATURE_NAMES),
        "feature_types": list(FEATURE_TYPES),
        "missing_value_policy": MISSING_VALUE_POLICY,
    }
    for key, value in expected.items():
        if manifest.get(key) != value:
            raise ValueError(f"bundle_{key}_mismatch")
    if manifest.get("model_input") not in {"raw", "scaled"}:
        raise ValueError("bundle_model_input_invalid")
    if not isinstance(manifest.get("training_manifest"), dict) or not (
        manifest["training_manifest"]
    ):
        raise ValueError("bundle_training_manifest_missing")
    if not isinstance(manifest.get("versions"), dict) or not manifest["versions"]:
        raise ValueError("bundle_versions_missing")
    try:
        maximum_z = float(manifest["maximum_absolute_z_score"])
        maximum_residual = float(manifest["maximum_absolute_residual_m"])
        residual_std = float(manifest["residual_prediction_std_m"])
    except (KeyError, TypeError, ValueError) as error:
        raise ValueError("bundle_safety_limits_invalid") from error
    if not math.isfinite(maximum_z) or maximum_z <= 0.0:
        raise ValueError("bundle_z_score_limit_invalid")
    if not math.isfinite(maximum_residual) or maximum_residual <= 0.0:
        raise ValueError("bundle_residual_limit_invalid")
    if not math.isfinite(residual_std) or residual_std < 0.0:
        raise ValueError("bundle_residual_std_invalid")
    try:
        minimum_applicability_probability = float(
            manifest["minimum_applicability_probability"]
        )
    except (KeyError, TypeError, ValueError) as error:
        raise ValueError("bundle_applicability_threshold_invalid") from error
    if not 0.0 < minimum_applicability_probability <= 1.0:
        raise ValueError("bundle_applicability_threshold_invalid")
    ood = manifest.get("ood")
    if not isinstance(ood, dict) or ood.get("method") != (
        "zscore_and_shrunk_mahalanobis"
    ):
        raise ValueError("bundle_ood_contract_invalid")
    try:
        ood_mean = np.asarray(
            ood["scaled_feature_mean"],
            dtype=np.float64,
        )
        ood_inverse_covariance = np.asarray(
            ood["scaled_inverse_covariance"],
            dtype=np.float64,
        )
        maximum_mahalanobis = float(
            ood["maximum_mahalanobis_distance"]
        )
    except (KeyError, TypeError, ValueError) as error:
        raise ValueError("bundle_ood_contract_invalid") from error
    feature_count = len(FEATURE_NAMES)
    if (
        ood_mean.shape != (feature_count,)
        or ood_inverse_covariance.shape != (feature_count, feature_count)
        or not np.all(np.isfinite(ood_mean))
        or not np.all(np.isfinite(ood_inverse_covariance))
        or not math.isfinite(maximum_mahalanobis)
        or maximum_mahalanobis <= 0.0
    ):
        raise ValueError("bundle_ood_contract_invalid")

    artifacts = manifest.get("artifacts")
    if not isinstance(artifacts, dict):
        raise ValueError("bundle_artifacts_missing")
    scaler_spec = artifacts.get("scaler")
    model_spec = artifacts.get("model")
    applicability_model_spec = artifacts.get("applicability_model")
    if not (
        isinstance(scaler_spec, dict)
        and isinstance(model_spec, dict)
        and isinstance(applicability_model_spec, dict)
    ):
        raise ValueError("bundle_artifact_spec_invalid")
    scaler_path = _validated_artifact_path(root, scaler_spec.get("filename"))
    model_path = _validated_artifact_path(root, model_spec.get("filename"))
    applicability_model_path = _validated_artifact_path(
        root,
        applicability_model_spec.get("filename"),
    )
    for path, spec in (
        (scaler_path, scaler_spec),
        (model_path, model_spec),
        (applicability_model_path, applicability_model_spec),
    ):
        expected_digest = str(spec.get("sha256", "")).lower()
        if len(expected_digest) != 64 or _sha256(path) != expected_digest:
            raise ValueError(f"bundle_checksum_mismatch:{path.name}")

    try:
        import joblib
        import xgboost
    except ImportError as error:
        raise ValueError(f"bundle_dependency_unavailable:{error.name}") from error

    scaler = joblib.load(scaler_path)
    if not callable(getattr(scaler, "transform", None)):
        raise ValueError("bundle_scaler_transform_missing")
    feature_count = getattr(scaler, "n_features_in_", len(FEATURE_NAMES))
    if int(feature_count) != len(FEATURE_NAMES):
        raise ValueError("bundle_scaler_feature_count_mismatch")
    booster = xgboost.Booster()
    booster.load_model(str(model_path))
    predictor = _XGBoostBoosterPredictor(xgboost, booster)
    applicability_booster = xgboost.Booster()
    applicability_booster.load_model(str(applicability_model_path))
    applicability_predictor = _XGBoostBoosterPredictor(
        xgboost,
        applicability_booster,
    )
    return LoadedRangeResidualBundle(
        manifest=manifest,
        scaler=scaler,
        applicability_predictor=applicability_predictor,
        predictor=predictor,
        manifest_sha256=_sha256(manifest_path),
    )


@dataclass(frozen=True)
class RangeCorrectionResult:
    output_distance_m: float
    physics_distance_m: float
    candidate_distance_m: float | None
    predicted_residual_m: float | None
    residual_prediction_std_m: float
    inference_valid: bool
    applied: bool
    reason: str
    measurement_usable: bool = True
    would_reject_measurement: bool = False


class RangeResidualCorrector:
    """Fail-closed residual range correction; never fits at runtime."""

    def __init__(
        self,
        *,
        mode: str = "off",
        bundle: LoadedRangeResidualBundle | None = None,
        load_error: str = "",
    ) -> None:
        selected_mode = str(mode).strip().lower()
        self.mode = selected_mode if selected_mode in VALID_MODES else "off"
        self.bundle = bundle
        self.load_error = (
            str(load_error)
            if selected_mode in VALID_MODES
            else f"invalid_mode:{selected_mode}"
        )
        self.reset()

    @classmethod
    def from_environment(cls) -> "RangeResidualCorrector":
        mode = os.getenv("SWARM_RANGE_RESIDUAL_MODE", "off").strip().lower()
        bundle_path = os.getenv("SWARM_RANGE_RESIDUAL_BUNDLE", "").strip()
        if mode == "off":
            return cls(mode=mode)
        if not bundle_path:
            return cls(mode=mode, load_error="bundle_path_missing")
        try:
            bundle = load_range_residual_bundle(bundle_path)
        except Exception as error:
            return cls(mode=mode, load_error=str(error))
        return cls(mode=mode, bundle=bundle)

    def reset(self) -> None:
        self.request_count = 0
        self.valid_inference_count = 0
        self.applied_count = 0
        self.shadow_count = 0
        self.rejected_count = 0
        self.would_reject_measurement_count = 0
        self.enforced_measurement_rejection_count = 0
        self.last_reason = "disabled" if self.mode == "off" else "waiting_sample"
        self.last_features: dict[str, float] = {}
        self.last_scaled_features: list[float] = []
        self.last_mahalanobis_distance: float | None = None
        self.last_applicability_probability: float | None = None
        self.last_physics_distance_m: float | None = None
        self.last_candidate_distance_m: float | None = None
        self.last_predicted_residual_m: float | None = None

    def correct(
        self,
        features: RangeResidualFeatures,
        physics_distance_m: float,
    ) -> RangeCorrectionResult:
        self.request_count += 1
        try:
            physics_distance = float(physics_distance_m)
            if not math.isfinite(physics_distance) or physics_distance <= 0.0:
                raise ValueError("physics_distance_invalid")
        except (TypeError, ValueError) as error:
            self.rejected_count += 1
            self.last_reason = str(error)
            return RangeCorrectionResult(
                float("nan"),
                float("nan"),
                None,
                None,
                0.0,
                False,
                False,
                self.last_reason,
                measurement_usable=False,
                would_reject_measurement=True,
            )
        self.last_physics_distance_m = physics_distance
        if self.mode == "off":
            self.last_reason = "disabled"
            return RangeCorrectionResult(
                physics_distance,
                physics_distance,
                None,
                None,
                0.0,
                False,
                False,
                self.last_reason,
            )
        if self.bundle is None:
            self.rejected_count += 1
            self.last_reason = self.load_error or "bundle_unavailable"
            return RangeCorrectionResult(
                physics_distance,
                physics_distance,
                None,
                None,
                0.0,
                False,
                False,
                self.last_reason,
            )
        try:
            raw = features.as_array()
            self.last_features = features.as_dict()
            scaled = np.asarray(
                self.bundle.scaler.transform(raw),
                dtype=np.float64,
            )
            if scaled.shape != raw.shape or not np.all(np.isfinite(scaled)):
                raise ValueError("scaled_features_invalid")
            self.last_scaled_features = [float(value) for value in scaled[0]]
            if float(np.max(np.abs(scaled))) > (
                self.bundle.maximum_absolute_z_score
            ):
                raise ValueError("feature_ood")
            centered = scaled[0] - self.bundle.ood_mean
            mahalanobis_squared = float(
                centered
                @ self.bundle.ood_inverse_covariance
                @ centered
            )
            if not math.isfinite(mahalanobis_squared):
                raise ValueError("feature_ood_distance_invalid")
            mahalanobis = math.sqrt(max(0.0, mahalanobis_squared))
            self.last_mahalanobis_distance = mahalanobis
            if mahalanobis > self.bundle.maximum_mahalanobis_distance:
                raise ValueError("feature_multivariate_ood")
            model_features = (
                scaled if self.bundle.model_input == "scaled" else raw
            )
            applicability_prediction = np.asarray(
                self.bundle.applicability_predictor.predict(model_features),
                dtype=np.float64,
            ).reshape(-1)
            if (
                applicability_prediction.size != 1
                or not np.isfinite(applicability_prediction[0])
            ):
                raise ValueError("applicability_prediction_invalid")
            applicability_probability = float(applicability_prediction[0])
            self.last_applicability_probability = applicability_probability
            if not 0.0 <= applicability_probability <= 1.0:
                raise ValueError("applicability_probability_invalid")
            if (
                applicability_probability
                < self.bundle.minimum_applicability_probability
            ):
                raise ValueError("prediction_not_applicable")
            prediction = np.asarray(
                self.bundle.predictor.predict(model_features),
                dtype=np.float64,
            ).reshape(-1)
            if prediction.size != 1 or not np.isfinite(prediction[0]):
                raise ValueError("prediction_invalid")
            residual = float(prediction[0])
            if abs(residual) > self.bundle.maximum_absolute_residual_m:
                raise ValueError("prediction_residual_outside_limit")
            candidate = physics_distance + residual
            if not math.isfinite(candidate) or not 0.5 <= candidate <= 80.0:
                raise ValueError("corrected_distance_outside_limits")
        except Exception as error:
            self.rejected_count += 1
            self.last_reason = str(error)
            would_reject = self.last_reason in {
                "feature_ood",
                "feature_multivariate_ood",
                "prediction_residual_outside_limit",
                "corrected_distance_outside_limits",
                "prediction_not_applicable",
            }
            if would_reject:
                self.would_reject_measurement_count += 1
            enforced_rejection = self.mode == "active" and would_reject
            if enforced_rejection:
                self.enforced_measurement_rejection_count += 1
            return RangeCorrectionResult(
                physics_distance,
                physics_distance,
                None,
                None,
                0.0,
                False,
                False,
                self.last_reason,
                measurement_usable=not enforced_rejection,
                would_reject_measurement=would_reject,
            )

        self.valid_inference_count += 1
        self.last_candidate_distance_m = candidate
        self.last_predicted_residual_m = residual
        if self.mode == "shadow":
            self.shadow_count += 1
            self.last_reason = "shadow_prediction"
            return RangeCorrectionResult(
                physics_distance,
                physics_distance,
                candidate,
                residual,
                self.bundle.residual_prediction_std_m,
                True,
                False,
                self.last_reason,
            )
        self.applied_count += 1
        self.last_reason = "residual_applied"
        return RangeCorrectionResult(
            candidate,
            physics_distance,
            candidate,
            residual,
            self.bundle.residual_prediction_std_m,
            True,
            True,
            self.last_reason,
        )

    def status(self) -> dict[str, Any]:
        return {
            "mode": self.mode,
            "bundle_loaded": self.bundle is not None,
            "load_error": self.load_error,
            "schema": feature_schema(),
            "model_input": (
                None if self.bundle is None else self.bundle.model_input
            ),
            "manifest_sha256": (
                None
                if self.bundle is None
                else self.bundle.manifest_sha256
            ),
            "request_count": self.request_count,
            "valid_inference_count": self.valid_inference_count,
            "shadow_count": self.shadow_count,
            "applied_count": self.applied_count,
            "rejected_count": self.rejected_count,
            "would_reject_measurement_count": (
                self.would_reject_measurement_count
            ),
            "enforced_measurement_rejection_count": (
                self.enforced_measurement_rejection_count
            ),
            "last_reason": self.last_reason,
            "last_features": dict(self.last_features),
            "last_scaled_features": list(self.last_scaled_features),
            "last_mahalanobis_distance": self.last_mahalanobis_distance,
            "last_applicability_probability": (
                self.last_applicability_probability
            ),
            "last_physics_distance_m": self.last_physics_distance_m,
            "last_candidate_distance_m": self.last_candidate_distance_m,
            "last_predicted_residual_m": self.last_predicted_residual_m,
        }
