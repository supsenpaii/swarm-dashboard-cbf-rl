from __future__ import annotations

import json
from pathlib import Path

import numpy as np
import pytest

from range_residual_correction import (
    BUNDLE_VERSION,
    FEATURE_NAMES,
    FEATURE_SCHEMA_VERSION,
    FEATURE_TYPES,
    MISSING_VALUE_POLICY,
    LoadedRangeResidualBundle,
    RangeResidualCorrector,
    RangeResidualFeatures,
    feature_schema,
    load_range_residual_bundle,
)


class TransformOnlyScaler:
    n_features_in_ = len(FEATURE_NAMES)

    def __init__(self, offset: float = 0.0) -> None:
        self.offset = float(offset)
        self.transform_count = 0

    def fit(self, *_args, **_kwargs):
        raise AssertionError("runtime must never fit the scaler")

    def transform(self, values: np.ndarray) -> np.ndarray:
        self.transform_count += 1
        return np.asarray(values, dtype=np.float64) + self.offset


class ConstantPredictor:
    def __init__(self, residual_m: float) -> None:
        self.residual_m = float(residual_m)
        self.last_input: np.ndarray | None = None

    def predict(self, values: np.ndarray) -> np.ndarray:
        self.last_input = np.asarray(values, dtype=np.float64)
        return np.asarray([self.residual_m], dtype=np.float64)


class FailingPredictor:
    def predict(self, _values: np.ndarray) -> np.ndarray:
        raise RuntimeError("inference exploded")


def valid_features() -> RangeResidualFeatures:
    return RangeResidualFeatures(
        m52_anchor_median_m=12.0,
        m52_anchor_quality=0.75,
        midas_target_inverse_depth_median=0.40,
        midas_target_inverse_depth_spread=0.02,
        bbox_width_px=80.0,
        bbox_height_px=120.0,
        bbox_area_fraction=0.05,
        previous_physics_distance_m=10.0,
        delta_time_s=0.10,
        bbox_center_x_fraction=0.50,
        bbox_center_y_fraction=0.40,
        image_ray_x=0.0,
        image_ray_y=-0.20,
        target_bearing_down=0.05,
        camera_optical_axis_down=0.17,
        calibration_scale=0.0002,
        calibration_offset=0.02,
        calibration_residual_m_inv=0.002,
        calibration_condition_number_log10=3.5,
        calibration_inlier_fraction=0.70,
        anchor_spatial_coverage_fraction=0.50,
        target_anchor_extrapolation_iqr=0.0,
        ray_range_relative_std=0.10,
    )


def loaded_bundle(
    *,
    scaler: TransformOnlyScaler | None = None,
    applicability_predictor: object | None = None,
    predictor: object | None = None,
    model_input: str = "scaled",
    maximum_z: float = 1000.0,
    maximum_mahalanobis: float = 1.0e9,
) -> LoadedRangeResidualBundle:
    return LoadedRangeResidualBundle(
        manifest={
            "model_input": model_input,
            "maximum_absolute_z_score": maximum_z,
            "maximum_absolute_residual_m": 3.0,
            "residual_prediction_std_m": 0.4,
            "minimum_applicability_probability": 0.80,
            "ood": {
                "method": "zscore_and_shrunk_mahalanobis",
                "scaled_feature_mean": [0.0] * len(FEATURE_NAMES),
                "scaled_inverse_covariance": np.eye(
                    len(FEATURE_NAMES)
                ).tolist(),
                "maximum_mahalanobis_distance": maximum_mahalanobis,
            },
        },
        scaler=scaler or TransformOnlyScaler(),
        applicability_predictor=(
            applicability_predictor or ConstantPredictor(1.0)
        ),
        predictor=predictor or ConstantPredictor(0.5),
        manifest_sha256="a" * 64,
    )


def test_feature_schema_is_versioned_deterministic_and_finite() -> None:
    schema = feature_schema()

    assert schema == {
        "version": FEATURE_SCHEMA_VERSION,
        "names": list(FEATURE_NAMES),
        "types": list(FEATURE_TYPES),
        "missing_value_policy": MISSING_VALUE_POLICY,
    }
    feature = valid_features()
    assert list(feature.as_dict()) == list(FEATURE_NAMES)
    assert feature.as_array().shape == (1, len(FEATURE_NAMES))


def test_feature_schema_rejects_missing_or_non_finite_values() -> None:
    invalid = RangeResidualFeatures(
        **{
            **valid_features().as_dict(),
            "previous_physics_distance_m": float("nan"),
        }
    )

    with pytest.raises(ValueError, match="feature_not_finite"):
        invalid.as_array()


def test_shadow_prediction_transforms_but_keeps_physics_output() -> None:
    scaler = TransformOnlyScaler(offset=1.0)
    predictor = ConstantPredictor(0.5)
    corrector = RangeResidualCorrector(
        mode="shadow",
        bundle=loaded_bundle(scaler=scaler, predictor=predictor),
    )

    result = corrector.correct(valid_features(), 10.0)

    assert result.inference_valid
    assert not result.applied
    assert result.output_distance_m == 10.0
    assert result.candidate_distance_m == 10.5
    assert result.predicted_residual_m == 0.5
    assert scaler.transform_count == 1
    assert predictor.last_input is not None
    np.testing.assert_allclose(
        predictor.last_input,
        valid_features().as_array() + 1.0,
    )
    assert corrector.status()["shadow_count"] == 1


def test_active_prediction_applies_residual_with_bundle_uncertainty() -> None:
    corrector = RangeResidualCorrector(
        mode="active",
        bundle=loaded_bundle(predictor=ConstantPredictor(-0.75)),
    )

    result = corrector.correct(valid_features(), 10.0)

    assert result.inference_valid
    assert result.applied
    assert result.output_distance_m == 9.25
    assert result.residual_prediction_std_m == 0.4
    assert corrector.status()["applied_count"] == 1


def test_missing_bundle_and_ood_features_fail_closed_to_physics() -> None:
    unavailable = RangeResidualCorrector(
        mode="active",
        load_error="bundle_path_missing",
    )
    missing = unavailable.correct(valid_features(), 10.0)
    assert missing.output_distance_m == 10.0
    assert not missing.inference_valid
    assert missing.reason == "bundle_path_missing"

    ood = RangeResidualCorrector(
        mode="active",
        bundle=loaded_bundle(maximum_z=0.01),
    ).correct(valid_features(), 10.0)
    assert ood.output_distance_m == 10.0
    assert not ood.inference_valid
    assert ood.reason == "feature_ood"

    failed = RangeResidualCorrector(
        mode="active",
        bundle=loaded_bundle(predictor=FailingPredictor()),
    ).correct(valid_features(), 10.0)
    assert failed.output_distance_m == 10.0
    assert not failed.inference_valid
    assert failed.reason == "inference exploded"


def test_multivariate_ood_and_large_residual_abstain_only_in_active() -> None:
    multivariate = RangeResidualCorrector(
        mode="active",
        bundle=loaded_bundle(maximum_mahalanobis=0.01),
    ).correct(valid_features(), 10.0)
    assert multivariate.reason == "feature_multivariate_ood"
    assert not multivariate.measurement_usable
    assert multivariate.would_reject_measurement

    active = RangeResidualCorrector(
        mode="active",
        bundle=loaded_bundle(predictor=ConstantPredictor(4.0)),
    ).correct(valid_features(), 10.0)
    assert active.reason == "prediction_residual_outside_limit"
    assert not active.measurement_usable
    assert active.would_reject_measurement

    shadow = RangeResidualCorrector(
        mode="shadow",
        bundle=loaded_bundle(predictor=ConstantPredictor(4.0)),
    ).correct(valid_features(), 10.0)
    assert shadow.measurement_usable
    assert shadow.would_reject_measurement


def test_applicability_classifier_abstains_before_residual_prediction() -> None:
    residual_predictor = ConstantPredictor(0.5)
    result = RangeResidualCorrector(
        mode="active",
        bundle=loaded_bundle(
            applicability_predictor=ConstantPredictor(0.25),
            predictor=residual_predictor,
        ),
    ).correct(valid_features(), 10.0)

    assert result.reason == "prediction_not_applicable"
    assert not result.measurement_usable
    assert result.would_reject_measurement
    assert residual_predictor.last_input is None


def test_bundle_manifest_mismatch_fails_before_loading_dependencies(
    tmp_path: Path,
) -> None:
    bundle_dir = tmp_path / "bundle"
    bundle_dir.mkdir()
    (bundle_dir / "manifest.json").write_text(
        json.dumps(
            {
                "bundle_version": BUNDLE_VERSION,
                "feature_schema_version": "wrong_schema",
                "feature_names": list(FEATURE_NAMES),
                "feature_types": list(FEATURE_TYPES),
                "missing_value_policy": MISSING_VALUE_POLICY,
            }
        ),
        encoding="utf-8",
    )

    with pytest.raises(ValueError, match="feature_schema_version_mismatch"):
        load_range_residual_bundle(bundle_dir)
