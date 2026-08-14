from __future__ import annotations

import argparse
import json
import math
import platform
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Callable, Mapping, Sequence

import numpy as np

from range_residual_correction import (
    BUNDLE_VERSION,
    FEATURE_NAMES,
    FEATURE_SCHEMA_VERSION,
    FEATURE_TYPES,
    MISSING_VALUE_POLICY,
    load_range_residual_bundle,
)
from range_residual_dataset import (
    RangeResidualDatasetSample,
    file_sha256,
    load_training_samples,
)
from range_applicability_gate import (
    RangeApplicabilityGate,
    RangeApplicabilityInputs,
)


@dataclass(frozen=True)
class GroupedDatasetSplit:
    train: tuple[RangeResidualDatasetSample, ...]
    validation: tuple[RangeResidualDatasetSample, ...]
    test: tuple[RangeResidualDatasetSample, ...]
    train_groups: tuple[str, ...]
    validation_groups: tuple[str, ...]
    test_groups: tuple[str, ...]


def _group_key(sample: RangeResidualDatasetSample) -> str:
    return f"{sample.run_id}/{sample.group_id}"


def _group_balanced_weights(
    samples: Sequence[RangeResidualDatasetSample],
) -> np.ndarray:
    """Give every independent run/session group equal total weight."""
    if not samples:
        raise ValueError("group_balanced_weights_empty")
    counts: dict[str, int] = {}
    for sample in samples:
        group = _group_key(sample)
        counts[group] = counts.get(group, 0) + 1
    group_total_weight = float(len(samples) / len(counts))
    weights = np.asarray(
        [group_total_weight / counts[_group_key(sample)] for sample in samples],
        dtype=np.float64,
    )
    if not np.all(np.isfinite(weights)) or np.any(weights <= 0.0):
        raise ValueError("group_balanced_weights_invalid")
    return weights


def _group_weight_totals(
    samples: Sequence[RangeResidualDatasetSample],
    weights: np.ndarray,
) -> dict[str, float]:
    values = np.asarray(weights, dtype=np.float64).reshape(-1)
    if values.shape != (len(samples),):
        raise ValueError("group_weight_shape_invalid")
    totals: dict[str, float] = {}
    for sample, weight in zip(samples, values, strict=True):
        group = _group_key(sample)
        totals[group] = totals.get(group, 0.0) + float(weight)
    return totals


def grouped_split(
    samples: Sequence[RangeResidualDatasetSample],
    *,
    seed: int = 52,
    validation_fraction: float = 0.15,
    test_fraction: float = 0.15,
) -> GroupedDatasetSplit:
    if not 0.0 < validation_fraction < 0.5:
        raise ValueError("validation_fraction_invalid")
    if not 0.0 < test_fraction < 0.5:
        raise ValueError("test_fraction_invalid")
    groups = sorted({_group_key(sample) for sample in samples})
    if len(groups) < 3:
        raise ValueError("at_least_three_independent_groups_required")

    import hashlib

    ordered = sorted(
        groups,
        key=lambda group: hashlib.sha256(
            f"{int(seed)}:{group}".encode("utf-8")
        ).hexdigest(),
    )
    test_count = max(1, int(round(len(ordered) * test_fraction)))
    validation_count = max(
        1, int(round(len(ordered) * validation_fraction))
    )
    if test_count + validation_count >= len(ordered):
        overflow = test_count + validation_count - len(ordered) + 1
        if validation_count > test_count:
            validation_count -= overflow
        else:
            test_count -= overflow
    test_groups = tuple(sorted(ordered[:test_count]))
    validation_groups = tuple(
        sorted(ordered[test_count:test_count + validation_count])
    )
    train_groups = tuple(
        sorted(ordered[test_count + validation_count:])
    )
    train_set = set(train_groups)
    validation_set = set(validation_groups)
    test_set = set(test_groups)
    if train_set & validation_set or train_set & test_set or validation_set & test_set:
        raise AssertionError("group_split_overlap")

    def select(selected: set[str]) -> tuple[RangeResidualDatasetSample, ...]:
        return tuple(sample for sample in samples if _group_key(sample) in selected)

    split = GroupedDatasetSplit(
        train=select(train_set),
        validation=select(validation_set),
        test=select(test_set),
        train_groups=train_groups,
        validation_groups=validation_groups,
        test_groups=test_groups,
    )
    if not split.train or not split.validation or not split.test:
        raise ValueError("group_split_empty_partition")
    return split


def applicability_stratified_grouped_split(
    samples: Sequence[RangeResidualDatasetSample],
    *,
    maximum_absolute_residual_m: float,
    seed: int = 52,
    validation_fraction: float = 0.15,
    test_fraction: float = 0.15,
) -> GroupedDatasetSplit:
    """Keep groups disjoint while exposing both applicability classes."""
    threshold = float(maximum_absolute_residual_m)
    if not math.isfinite(threshold) or threshold <= 0.0:
        raise ValueError("maximum_absolute_residual_m_invalid")
    baseline = grouped_split(
        samples,
        seed=seed,
        validation_fraction=validation_fraction,
        test_fraction=test_fraction,
    )
    groups = sorted({_group_key(sample) for sample in samples})

    import hashlib

    ordered = sorted(
        groups,
        key=lambda group: hashlib.sha256(
            f"{int(seed)}:{group}".encode("utf-8")
        ).hexdigest(),
    )
    class_counts: dict[str, tuple[int, int]] = {}
    for group in ordered:
        group_samples = tuple(
            sample
            for sample in samples
            if _group_key(sample) == group
        )
        correctable = sum(
            1
            for sample in group_samples
            if sample.ground_truth_distance_m is not None
            and abs(
                sample.ground_truth_distance_m
                - sample.physics_distance_m
            )
            <= threshold
        )
        class_counts[group] = (
            correctable,
            len(group_samples) - correctable,
        )
    remaining = list(ordered)

    def take_partition(target_count: int) -> list[str]:
        mixed = [
            group
            for group in remaining
            if class_counts[group][0] > 0
            and class_counts[group][1] > 0
        ]
        selected_groups: list[str] = []
        if mixed:
            selected_groups.append(mixed[0])
        else:
            correctable_only = [
                group
                for group in remaining
                if class_counts[group][0] > 0
                and class_counts[group][1] == 0
            ]
            uncorrectable_only = [
                group
                for group in remaining
                if class_counts[group][0] == 0
                and class_counts[group][1] > 0
            ]
            if not correctable_only or not uncorrectable_only:
                raise ValueError(
                    "independent_applicability_holdout_classes_unavailable"
                )
            selected_groups.extend(
                (correctable_only[0], uncorrectable_only[0])
            )
        for group in selected_groups:
            remaining.remove(group)
        while len(selected_groups) < target_count and remaining:
            selected_groups.append(remaining.pop(0))
        return selected_groups

    test_groups = take_partition(len(baseline.test_groups))
    validation_groups = take_partition(len(baseline.validation_groups))
    train_groups = remaining

    def select(
        selected_groups: Sequence[str],
    ) -> tuple[RangeResidualDatasetSample, ...]:
        selected_set = set(selected_groups)
        return tuple(
            sample
            for sample in samples
            if _group_key(sample) in selected_set
        )

    split = GroupedDatasetSplit(
        train=select(train_groups),
        validation=select(validation_groups),
        test=select(test_groups),
        train_groups=tuple(sorted(train_groups)),
        validation_groups=tuple(sorted(validation_groups)),
        test_groups=tuple(sorted(test_groups)),
    )
    for partition_name, partition in (
        ("train", split.train),
        ("validation", split.validation),
        ("test", split.test),
    ):
        labels = [
            sample.ground_truth_distance_m is not None
            and abs(
                sample.ground_truth_distance_m
                - sample.physics_distance_m
            )
            <= threshold
            for sample in partition
        ]
        if not any(labels) or all(labels):
            raise ValueError(
                f"applicability_{partition_name}_requires_both_classes"
            )
    return split


def deterministic_applicability_samples(
    samples: Sequence[RangeResidualDatasetSample],
) -> tuple[tuple[RangeResidualDatasetSample, ...], dict[str, Any]]:
    """Apply the same conservative pre-model envelope used at runtime."""
    gate = RangeApplicabilityGate(mode="active")
    accepted: list[RangeResidualDatasetSample] = []
    rejected_reasons: dict[str, int] = {}
    for sample in samples:
        features = sample.features
        try:
            condition_number = 10.0 ** float(
                features.calibration_condition_number_log10
            )
            result = gate.evaluate(
                RangeApplicabilityInputs(
                    image_ray_x=features.image_ray_x,
                    image_ray_y=features.image_ray_y,
                    target_bearing_down=features.target_bearing_down,
                    camera_optical_axis_down=(
                        features.camera_optical_axis_down
                    ),
                    calibration_condition_number=condition_number,
                    calibration_residual_m_inv=(
                        features.calibration_residual_m_inv
                    ),
                    calibration_inlier_fraction=(
                        features.calibration_inlier_fraction
                    ),
                    anchor_spatial_coverage_fraction=(
                        features.anchor_spatial_coverage_fraction
                    ),
                    target_anchor_extrapolation_iqr=(
                        features.target_anchor_extrapolation_iqr
                    ),
                    ray_range_relative_std=(
                        features.ray_range_relative_std
                    ),
                )
            )
        except (OverflowError, ValueError) as error:
            reason = f"deterministic_gate_input_invalid:{error}"
        else:
            reason = result.reason
            if result.applicable:
                accepted.append(sample)
                continue
        rejected_reasons[reason] = rejected_reasons.get(reason, 0) + 1
    status = gate.status()
    return tuple(accepted), {
        "policy": "runtime_default_deterministic_applicability_envelope",
        "input_samples": len(samples),
        "accepted_samples": len(accepted),
        "rejected_samples": len(samples) - len(accepted),
        "rejected_reason_counts": rejected_reasons,
        "limits": status["limits"],
    }


def _matrix(
    samples: Sequence[RangeResidualDatasetSample],
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    features = np.vstack([sample.features.as_array()[0] for sample in samples])
    physics = np.asarray(
        [sample.physics_distance_m for sample in samples],
        dtype=np.float64,
    )
    truth = np.asarray(
        [sample.ground_truth_distance_m for sample in samples],
        dtype=np.float64,
    )
    if not (
        np.all(np.isfinite(features))
        and np.all(np.isfinite(physics))
        and np.all(np.isfinite(truth))
    ):
        raise ValueError("training_matrix_not_finite")
    residual = truth - physics
    return features, physics, residual


def regression_metrics(
    ground_truth_m: Sequence[float] | np.ndarray,
    prediction_m: Sequence[float] | np.ndarray,
) -> dict[str, float | int]:
    truth = np.asarray(ground_truth_m, dtype=np.float64).reshape(-1)
    prediction = np.asarray(prediction_m, dtype=np.float64).reshape(-1)
    if truth.size == 0 or truth.shape != prediction.shape:
        raise ValueError("metric_input_shape_invalid")
    error = prediction - truth
    absolute = np.abs(error)
    return {
        "count": int(truth.size),
        "mae_m": float(np.mean(absolute)),
        "rmse_m": float(np.sqrt(np.mean(error**2))),
        "bias_m": float(np.mean(error)),
        "p95_absolute_error_m": float(np.quantile(absolute, 0.95)),
    }


def _comparison(
    physics: np.ndarray,
    residual_target: np.ndarray,
    residual_prediction: np.ndarray,
) -> dict[str, Any]:
    truth = physics + residual_target
    baseline = regression_metrics(truth, physics)
    corrected = regression_metrics(truth, physics + residual_prediction)
    return {
        "physics_baseline": baseline,
        "residual_corrected": corrected,
        "mae_improvement_m": float(
            baseline["mae_m"] - corrected["mae_m"]
        ),
        "rmse_improvement_m": float(
            baseline["rmse_m"] - corrected["rmse_m"]
        ),
    }


def _mahalanobis_distances(
    values: np.ndarray,
    mean: np.ndarray,
    inverse_covariance: np.ndarray,
) -> np.ndarray:
    centered = np.asarray(values, dtype=np.float64) - mean
    squared = np.einsum(
        "ij,jk,ik->i",
        centered,
        inverse_covariance,
        centered,
    )
    return np.sqrt(np.maximum(0.0, squared))


def _ood_contract(train_scaled: np.ndarray) -> dict[str, Any]:
    values = np.asarray(train_scaled, dtype=np.float64)
    if values.ndim != 2 or values.shape[0] < 2:
        raise ValueError("ood_training_matrix_invalid")
    feature_count = values.shape[1]
    mean = np.mean(values, axis=0)
    covariance = np.asarray(np.cov(values, rowvar=False), dtype=np.float64)
    if covariance.shape != (feature_count, feature_count):
        raise ValueError("ood_covariance_shape_invalid")
    shrinkage = 0.10
    regularized = (
        (1.0 - shrinkage) * covariance
        + shrinkage * np.eye(feature_count, dtype=np.float64)
    )
    inverse_covariance = np.linalg.pinv(regularized)
    distances = _mahalanobis_distances(
        values,
        mean,
        inverse_covariance,
    )
    if not (
        np.all(np.isfinite(mean))
        and np.all(np.isfinite(inverse_covariance))
        and np.all(np.isfinite(distances))
    ):
        raise ValueError("ood_contract_not_finite")
    maximum_distance = max(
        6.0,
        float(np.quantile(distances, 0.995)) + 1.0,
    )
    return {
        "method": "zscore_and_shrunk_mahalanobis",
        "covariance_shrinkage": shrinkage,
        "scaled_feature_mean": [float(value) for value in mean],
        "scaled_inverse_covariance": [
            [float(value) for value in row]
            for row in inverse_covariance
        ],
        "maximum_mahalanobis_distance": maximum_distance,
        "training_distance_quantiles": {
            "p50": float(np.quantile(distances, 0.50)),
            "p95": float(np.quantile(distances, 0.95)),
            "p995": float(np.quantile(distances, 0.995)),
            "maximum": float(np.max(distances)),
        },
    }


def _runtime_partition_report(
    physics: np.ndarray,
    residual_target: np.ndarray,
    residual_prediction: np.ndarray,
    applicability_probability: np.ndarray,
    *,
    minimum_applicability_probability: float,
    maximum_absolute_residual_m: float,
) -> tuple[dict[str, Any], np.ndarray]:
    prediction = np.asarray(residual_prediction, dtype=np.float64).reshape(-1)
    probability = np.asarray(
        applicability_probability,
        dtype=np.float64,
    ).reshape(-1)
    if not (
        prediction.shape == residual_target.shape
        and probability.shape == residual_target.shape
    ):
        raise ValueError("runtime_evaluation_shape_invalid")
    candidate_distance = physics + prediction
    accepted = (
        np.isfinite(prediction)
        & np.isfinite(probability)
        & (probability >= minimum_applicability_probability)
        & (np.abs(prediction) <= maximum_absolute_residual_m)
        & np.isfinite(candidate_distance)
        & (candidate_distance >= 0.5)
        & (candidate_distance <= 80.0)
    )
    if not np.any(accepted):
        raise ValueError("all_partition_predictions_rejected")
    ground_truth_correctable = (
        np.abs(residual_target) <= maximum_absolute_residual_m
    )
    true_positive = int(
        np.count_nonzero(accepted & ground_truth_correctable)
    )
    false_positive = int(
        np.count_nonzero(accepted & ~ground_truth_correctable)
    )
    true_negative = int(
        np.count_nonzero(~accepted & ~ground_truth_correctable)
    )
    false_negative = int(
        np.count_nonzero(~accepted & ground_truth_correctable)
    )

    def ratio(numerator: int, denominator: int) -> float:
        return (
            float(numerator / denominator)
            if denominator > 0
            else 0.0
        )

    report = _comparison(
        physics[accepted],
        residual_target[accepted],
        prediction[accepted],
    )
    report.update(
        {
            "total_samples": int(residual_target.size),
            "accepted_samples": int(np.count_nonzero(accepted)),
            "coverage_fraction": float(np.mean(accepted)),
            "classifier_rejected_samples": int(
                np.count_nonzero(
                    probability < minimum_applicability_probability
                )
            ),
            "residual_limit_rejected_samples": int(
                np.count_nonzero(
                    np.abs(prediction) > maximum_absolute_residual_m
                )
            ),
            "ground_truth_correctable_samples": int(
                np.count_nonzero(ground_truth_correctable)
            ),
            "ground_truth_uncorrectable_samples": int(
                np.count_nonzero(~ground_truth_correctable)
            ),
            "applicability_confusion": {
                "true_positive": true_positive,
                "false_positive": false_positive,
                "true_negative": true_negative,
                "false_negative": false_negative,
            },
            "applicability_precision": ratio(
                true_positive,
                true_positive + false_positive,
            ),
            "applicability_recall": ratio(
                true_positive,
                true_positive + false_negative,
            ),
            "applicability_specificity": ratio(
                true_negative,
                true_negative + false_positive,
            ),
            "uncorrectable_false_accept_rate": ratio(
                false_positive,
                false_positive + true_negative,
            ),
        }
    )
    return report, accepted


def _applicability_group_reports(
    samples: Sequence[RangeResidualDatasetSample],
    accepted: np.ndarray,
    correctable: np.ndarray,
    *,
    minimum_recall: float,
    maximum_false_accept_rate: float,
) -> dict[str, dict[str, Any]]:
    accepted_values = np.asarray(accepted, dtype=bool).reshape(-1)
    correctable_values = np.asarray(correctable, dtype=bool).reshape(-1)
    expected_shape = (len(samples),)
    if (
        accepted_values.shape != expected_shape
        or correctable_values.shape != expected_shape
    ):
        raise ValueError("applicability_group_report_shape_invalid")
    reports: dict[str, dict[str, Any]] = {}
    for group in sorted({_group_key(sample) for sample in samples}):
        selected = np.asarray(
            [_group_key(sample) == group for sample in samples],
            dtype=bool,
        )
        group_accepted = accepted_values[selected]
        group_correctable = correctable_values[selected]
        true_positive = int(
            np.count_nonzero(group_accepted & group_correctable)
        )
        false_positive = int(
            np.count_nonzero(group_accepted & ~group_correctable)
        )
        true_negative = int(
            np.count_nonzero(~group_accepted & ~group_correctable)
        )
        false_negative = int(
            np.count_nonzero(~group_accepted & group_correctable)
        )
        correctable_count = true_positive + false_negative
        uncorrectable_count = false_positive + true_negative
        recall = (
            float(true_positive / correctable_count)
            if correctable_count
            else None
        )
        false_accept_rate = (
            float(false_positive / uncorrectable_count)
            if uncorrectable_count
            else None
        )
        recall_passed = bool(
            recall is None or recall >= float(minimum_recall)
        )
        false_accept_passed = bool(
            false_accept_rate is None
            or false_accept_rate <= float(maximum_false_accept_rate)
        )
        reports[group] = {
            "sample_count": int(np.count_nonzero(selected)),
            "correctable_samples": correctable_count,
            "uncorrectable_samples": uncorrectable_count,
            "true_positive": true_positive,
            "false_positive": false_positive,
            "true_negative": true_negative,
            "false_negative": false_negative,
            "applicability_recall": recall,
            "uncorrectable_false_accept_rate": false_accept_rate,
            "recall_passed": recall_passed,
            "false_accept_rate_passed": false_accept_passed,
            "passed": bool(recall_passed and false_accept_passed),
        }
    return reports


def _default_dependencies() -> tuple[Any, Any, Any, Callable[[], Any]]:
    try:
        import joblib
        import sklearn
        import xgboost
        from sklearn.preprocessing import StandardScaler
    except ImportError as error:
        raise RuntimeError(
            "Offline training requires scikit-learn, xgboost and joblib"
        ) from error
    return joblib, sklearn, xgboost, StandardScaler


def train_range_residual_bundle(
    dataset_roots: Sequence[str | Path],
    output_dir: str | Path,
    *,
    seed: int = 52,
    validation_fraction: float = 0.15,
    test_fraction: float = 0.15,
    maximum_absolute_z_score: float = 8.0,
    maximum_absolute_residual_m: float = 3.0,
    minimum_applicability_probability: float = 0.80,
    num_boost_round: int = 400,
    early_stopping_rounds: int = 40,
    dependency_loader: Callable[
        [], tuple[Any, Any, Any, Callable[[], Any]]
    ] = _default_dependencies,
) -> dict[str, Any]:
    output = Path(output_dir)
    if output.exists():
        raise ValueError("output_bundle_already_exists")
    if (
        not math.isfinite(float(maximum_absolute_z_score))
        or float(maximum_absolute_z_score) <= 0.0
    ):
        raise ValueError("maximum_absolute_z_score_invalid")
    if (
        not math.isfinite(float(maximum_absolute_residual_m))
        or float(maximum_absolute_residual_m) <= 0.0
    ):
        raise ValueError("maximum_absolute_residual_m_invalid")
    if not 0.0 < float(minimum_applicability_probability) <= 1.0:
        raise ValueError("minimum_applicability_probability_invalid")
    loaded_samples, provenance = load_training_samples(dataset_roots)
    samples, deterministic_filter = deterministic_applicability_samples(
        loaded_samples
    )
    provenance["deterministic_applicability_filter"] = (
        deterministic_filter
    )
    if not samples:
        raise ValueError("no_samples_inside_deterministic_applicability_envelope")
    split = applicability_stratified_grouped_split(
        samples,
        maximum_absolute_residual_m=float(maximum_absolute_residual_m),
        seed=seed,
        validation_fraction=validation_fraction,
        test_fraction=test_fraction,
    )
    joblib, sklearn, xgboost, scaler_factory = dependency_loader()
    train_x, train_physics, train_y = _matrix(split.train)
    validation_x, validation_physics, validation_y = _matrix(
        split.validation
    )
    test_x, test_physics, test_y = _matrix(split.test)

    scaler = scaler_factory()
    train_scaled = np.asarray(
        scaler.fit_transform(train_x), dtype=np.float64
    )
    validation_scaled = np.asarray(
        scaler.transform(validation_x), dtype=np.float64
    )
    test_scaled = np.asarray(scaler.transform(test_x), dtype=np.float64)
    if int(getattr(scaler, "n_features_in_", -1)) != len(FEATURE_NAMES):
        raise ValueError("trained_scaler_feature_count_mismatch")
    if not all(
        np.all(np.isfinite(values))
        for values in (train_scaled, validation_scaled, test_scaled)
    ):
        raise ValueError("scaled_training_features_invalid")
    ood_contract = _ood_contract(train_scaled)

    correctable_train = (
        np.abs(train_y) <= float(maximum_absolute_residual_m)
    )
    correctable_validation = (
        np.abs(validation_y) <= float(maximum_absolute_residual_m)
    )
    correctable_test = (
        np.abs(test_y) <= float(maximum_absolute_residual_m)
    )
    if np.count_nonzero(correctable_train) < 2:
        raise ValueError("insufficient_correctable_training_samples")
    if not np.any(~correctable_train):
        raise ValueError("no_uncorrectable_training_samples")
    if not np.any(correctable_validation):
        raise ValueError("no_correctable_validation_samples")
    if not np.any(~correctable_validation):
        raise ValueError("no_uncorrectable_validation_samples")
    if not np.any(correctable_test) or not np.any(~correctable_test):
        raise ValueError("test_applicability_requires_both_classes")

    parameters = {
        "objective": "reg:squarederror",
        "eval_metric": "rmse",
        "max_depth": 4,
        "eta": 0.05,
        "subsample": 0.8,
        "colsample_bytree": 0.9,
        "min_child_weight": 5.0,
        "lambda": 1.0,
        "alpha": 0.0,
        "seed": int(seed),
        "nthread": 1,
    }
    applicability_parameters = {
        **parameters,
        "objective": "binary:logistic",
        "eval_metric": "logloss",
        "max_depth": 3,
    }
    applicability_train_weights = _group_balanced_weights(split.train)
    applicability_validation_weights = _group_balanced_weights(
        split.validation
    )
    correctable_train_samples = tuple(
        sample
        for sample, selected in zip(
            split.train,
            correctable_train,
            strict=True,
        )
        if selected
    )
    correctable_validation_samples = tuple(
        sample
        for sample, selected in zip(
            split.validation,
            correctable_validation,
            strict=True,
        )
        if selected
    )
    residual_train_weights = _group_balanced_weights(
        correctable_train_samples
    )
    residual_validation_weights = _group_balanced_weights(
        correctable_validation_samples
    )
    applicability_train_matrix = xgboost.DMatrix(
        train_scaled,
        label=correctable_train.astype(np.float64),
        weight=applicability_train_weights,
    )
    applicability_validation_matrix = xgboost.DMatrix(
        validation_scaled,
        label=correctable_validation.astype(np.float64),
        weight=applicability_validation_weights,
    )
    applicability_booster = xgboost.train(
        applicability_parameters,
        applicability_train_matrix,
        num_boost_round=max(1, int(num_boost_round)),
        evals=[(applicability_validation_matrix, "validation")],
        early_stopping_rounds=max(1, int(early_stopping_rounds)),
        verbose_eval=False,
    )
    train_matrix = xgboost.DMatrix(
        train_scaled[correctable_train],
        label=train_y[correctable_train],
        weight=residual_train_weights,
    )
    validation_matrix = xgboost.DMatrix(
        validation_scaled[correctable_validation],
        label=validation_y[correctable_validation],
        weight=residual_validation_weights,
    )
    booster = xgboost.train(
        parameters,
        train_matrix,
        num_boost_round=max(1, int(num_boost_round)),
        evals=[(validation_matrix, "validation")],
        early_stopping_rounds=max(1, int(early_stopping_rounds)),
        verbose_eval=False,
    )
    validation_prediction = np.asarray(
        booster.predict(xgboost.DMatrix(validation_scaled)),
        dtype=np.float64,
    )
    test_prediction = np.asarray(
        booster.predict(xgboost.DMatrix(test_scaled)),
        dtype=np.float64,
    )
    validation_applicability = np.asarray(
        applicability_booster.predict(
            xgboost.DMatrix(validation_scaled)
        ),
        dtype=np.float64,
    )
    test_applicability = np.asarray(
        applicability_booster.predict(xgboost.DMatrix(test_scaled)),
        dtype=np.float64,
    )
    validation_report, validation_accepted = _runtime_partition_report(
        validation_physics,
        validation_y,
        validation_prediction,
        validation_applicability,
        minimum_applicability_probability=float(
            minimum_applicability_probability
        ),
        maximum_absolute_residual_m=float(maximum_absolute_residual_m),
    )
    test_report, test_accepted = _runtime_partition_report(
        test_physics,
        test_y,
        test_prediction,
        test_applicability,
        minimum_applicability_probability=float(
            minimum_applicability_probability
        ),
        maximum_absolute_residual_m=float(maximum_absolute_residual_m),
    )
    validation_prediction_error = (
        validation_prediction[validation_accepted]
        - validation_y[validation_accepted]
    )
    residual_prediction_std_m = max(
        0.05,
        float(np.sqrt(np.mean(validation_prediction_error**2))),
    )

    output.mkdir(parents=True, exist_ok=False)
    scaler_path = output / "scaler.joblib"
    model_path = output / "model.json"
    applicability_model_path = output / "applicability_model.json"
    evaluation_path = output / "evaluation.json"
    joblib.dump(scaler, scaler_path)
    booster.save_model(str(model_path))
    applicability_booster.save_model(str(applicability_model_path))
    evaluation = {
        "validation": validation_report,
        "test": test_report,
    }
    promotion_thresholds = {
        "minimum_applicability_precision": 0.95,
        "minimum_applicability_recall": 0.50,
        "maximum_uncorrectable_false_accept_rate": 0.05,
        "maximum_corrected_mae_m": 1.0,
        "maximum_residual_prediction_std_m": 1.0,
    }
    validation_group_reports = _applicability_group_reports(
        split.validation,
        validation_accepted,
        correctable_validation,
        minimum_recall=promotion_thresholds[
            "minimum_applicability_recall"
        ],
        maximum_false_accept_rate=promotion_thresholds[
            "maximum_uncorrectable_false_accept_rate"
        ],
    )
    test_group_reports = _applicability_group_reports(
        split.test,
        test_accepted,
        correctable_test,
        minimum_recall=promotion_thresholds[
            "minimum_applicability_recall"
        ],
        maximum_false_accept_rate=promotion_thresholds[
            "maximum_uncorrectable_false_accept_rate"
        ],
    )
    evaluation["validation_groups"] = validation_group_reports
    evaluation["test_groups"] = test_group_reports

    def partition_passed(report: Mapping[str, Any]) -> bool:
        return bool(
            report["applicability_precision"]
            >= promotion_thresholds["minimum_applicability_precision"]
            and report["applicability_recall"]
            >= promotion_thresholds["minimum_applicability_recall"]
            and report["uncorrectable_false_accept_rate"]
            <= promotion_thresholds[
                "maximum_uncorrectable_false_accept_rate"
            ]
            and report["residual_corrected"]["mae_m"]
            <= promotion_thresholds["maximum_corrected_mae_m"]
            and report["mae_improvement_m"] > 0.0
            and report["rmse_improvement_m"] > 0.0
        )

    promotion_gate = {
        "thresholds": promotion_thresholds,
        "validation_passed": partition_passed(validation_report),
        "test_passed": partition_passed(test_report),
        "validation_groups_passed": all(
            report["passed"]
            for report in validation_group_reports.values()
        ),
        "test_groups_passed": all(
            report["passed"]
            for report in test_group_reports.values()
        ),
        "residual_prediction_std_passed": bool(
            residual_prediction_std_m
            <= promotion_thresholds[
                "maximum_residual_prediction_std_m"
            ]
        ),
    }
    promotion_gate["passed"] = bool(
        promotion_gate["validation_passed"]
        and promotion_gate["test_passed"]
        and promotion_gate["validation_groups_passed"]
        and promotion_gate["test_groups_passed"]
        and promotion_gate["residual_prediction_std_passed"]
    )
    evaluation_path.write_text(
        json.dumps(evaluation, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    training_manifest = {
        "dataset_provenance": provenance,
        "split_policy": (
            "deterministic_applicability_stratified_grouped_by_run_and_session"
        ),
        "seed": int(seed),
        "train_groups": list(split.train_groups),
        "validation_groups": list(split.validation_groups),
        "test_groups": list(split.test_groups),
        "sample_counts": {
            "train": len(split.train),
            "validation": len(split.validation),
            "test": len(split.test),
        },
        "xgboost_parameters": parameters,
        "xgboost_applicability_parameters": applicability_parameters,
        "sample_weight_policy": {
            "method": "equal_total_weight_per_run_session_group",
            "normalization": "mean_one_within_each_training_matrix",
            "applicability_train_group_totals": _group_weight_totals(
                split.train,
                applicability_train_weights,
            ),
            "applicability_validation_group_totals": _group_weight_totals(
                split.validation,
                applicability_validation_weights,
            ),
            "residual_train_group_totals": _group_weight_totals(
                correctable_train_samples,
                residual_train_weights,
            ),
            "residual_validation_group_totals": _group_weight_totals(
                correctable_validation_samples,
                residual_validation_weights,
            ),
        },
        "applicability_label": (
            "abs(ground_truth_distance_m - physics_distance_m) "
            "<= maximum_absolute_residual_m"
        ),
        "applicability_class_counts": {
            "train_correctable": int(np.count_nonzero(correctable_train)),
            "train_uncorrectable": int(
                correctable_train.size - np.count_nonzero(correctable_train)
            ),
            "validation_correctable": int(
                np.count_nonzero(correctable_validation)
            ),
            "validation_uncorrectable": int(
                correctable_validation.size
                - np.count_nonzero(correctable_validation)
            ),
            "test_correctable": int(np.count_nonzero(correctable_test)),
            "test_uncorrectable": int(
                correctable_test.size - np.count_nonzero(correctable_test)
            ),
        },
        "num_boost_round": int(num_boost_round),
        "early_stopping_rounds": int(early_stopping_rounds),
        "evaluation": evaluation,
        "promotion_gate": promotion_gate,
        "held_out_improvement_positive": bool(
            validation_report["mae_improvement_m"] > 0.0
            and validation_report["rmse_improvement_m"] > 0.0
            and test_report["mae_improvement_m"] > 0.0
            and test_report["rmse_improvement_m"] > 0.0
        ),
    }
    manifest = {
        "bundle_version": BUNDLE_VERSION,
        "feature_schema_version": FEATURE_SCHEMA_VERSION,
        "feature_names": list(FEATURE_NAMES),
        "feature_types": list(FEATURE_TYPES),
        "missing_value_policy": MISSING_VALUE_POLICY,
        "model_input": "scaled",
        "maximum_absolute_z_score": float(maximum_absolute_z_score),
        "maximum_absolute_residual_m": float(
            maximum_absolute_residual_m
        ),
        "residual_prediction_std_m": residual_prediction_std_m,
        "minimum_applicability_probability": float(
            minimum_applicability_probability
        ),
        "candidate_promotion_gate_passed": bool(
            promotion_gate["passed"]
        ),
        "ood": ood_contract,
        "training_manifest": training_manifest,
        "versions": {
            "python": platform.python_version(),
            "numpy": np.__version__,
            "scikit_learn": str(getattr(sklearn, "__version__", "unknown")),
            "xgboost": str(getattr(xgboost, "__version__", "unknown")),
            "joblib": str(getattr(joblib, "__version__", "unknown")),
        },
        "artifacts": {
            "scaler": {
                "filename": scaler_path.name,
                "sha256": file_sha256(scaler_path),
            },
            "model": {
                "filename": model_path.name,
                "sha256": file_sha256(model_path),
            },
            "applicability_model": {
                "filename": applicability_model_path.name,
                "sha256": file_sha256(applicability_model_path),
            },
        },
    }
    (output / "manifest.json").write_text(
        json.dumps(manifest, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    return manifest


def evaluate_range_residual_bundle(
    dataset_roots: Sequence[str | Path],
    bundle_dir: str | Path,
) -> dict[str, Any]:
    samples, provenance = load_training_samples(dataset_roots)
    if not samples:
        raise ValueError("no_labeled_samples")
    bundle = load_range_residual_bundle(bundle_dir)
    features, physics, residual_target = _matrix(samples)
    scaled = np.asarray(bundle.scaler.transform(features), dtype=np.float64)
    maximum_z = np.max(np.abs(scaled), axis=1)
    mahalanobis = _mahalanobis_distances(
        scaled,
        bundle.ood_mean,
        bundle.ood_inverse_covariance,
    )
    ood_accepted = (
        (maximum_z <= bundle.maximum_absolute_z_score)
        & (mahalanobis <= bundle.maximum_mahalanobis_distance)
    )
    if not np.any(ood_accepted):
        raise ValueError("all_evaluation_samples_ood")
    model_features = scaled if bundle.model_input == "scaled" else features
    applicability_probability = np.asarray(
        bundle.applicability_predictor.predict(
            model_features[ood_accepted]
        ),
        dtype=np.float64,
    ).reshape(-1)
    classifier_accepted = (
        np.isfinite(applicability_probability)
        & (applicability_probability >= (
            bundle.minimum_applicability_probability
        ))
    )
    if not np.any(classifier_accepted):
        raise ValueError("all_evaluation_samples_not_applicable")
    residual_prediction = np.asarray(
        bundle.predictor.predict(
            model_features[ood_accepted][classifier_accepted]
        ),
        dtype=np.float64,
    ).reshape(-1)
    candidate_physics = physics[ood_accepted][classifier_accepted]
    candidate_distance = candidate_physics + residual_prediction
    inference_valid = (
        np.isfinite(residual_prediction)
        & (np.abs(residual_prediction) <= bundle.maximum_absolute_residual_m)
        & np.isfinite(candidate_distance)
        & (candidate_distance >= 0.5)
        & (candidate_distance <= 80.0)
    )
    if not np.any(inference_valid):
        raise ValueError("all_evaluation_predictions_rejected")
    report = _comparison(
        candidate_physics[inference_valid],
        residual_target[ood_accepted][classifier_accepted][inference_valid],
        residual_prediction[inference_valid],
    )
    report.update(
        {
            "bundle_manifest_sha256": bundle.manifest_sha256,
            "dataset_provenance": provenance,
            "total_labeled_samples": int(len(samples)),
            "accepted_samples": int(np.count_nonzero(inference_valid)),
            "ood_samples": int(np.count_nonzero(~ood_accepted)),
            "zscore_ood_samples": int(
                np.count_nonzero(
                    maximum_z > bundle.maximum_absolute_z_score
                )
            ),
            "multivariate_ood_samples": int(
                np.count_nonzero(
                    mahalanobis > bundle.maximum_mahalanobis_distance
                )
            ),
            "prediction_rejected_samples": int(
                np.count_nonzero(~inference_valid)
            ),
            "classifier_rejected_samples": int(
                np.count_nonzero(~classifier_accepted)
            ),
            "coverage_fraction": float(
                np.count_nonzero(inference_valid) / len(samples)
            ),
        }
    )
    return report


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Train or evaluate the M52+MiDaS XGBoost residual bundle"
    )
    subparsers = parser.add_subparsers(dest="command", required=True)
    train = subparsers.add_parser("train")
    train.add_argument("datasets", nargs="+")
    train.add_argument("--output", required=True)
    train.add_argument("--seed", type=int, default=52)
    evaluate = subparsers.add_parser("evaluate")
    evaluate.add_argument("datasets", nargs="+")
    evaluate.add_argument("--bundle", required=True)
    evaluate.add_argument("--json-output")
    return parser


def main() -> int:
    args = _parser().parse_args()
    if args.command == "train":
        result = train_range_residual_bundle(
            args.datasets,
            args.output,
            seed=args.seed,
        )
    else:
        result = evaluate_range_residual_bundle(
            args.datasets,
            args.bundle,
        )
        if args.json_output:
            Path(args.json_output).write_text(
                json.dumps(result, indent=2, sort_keys=True) + "\n",
                encoding="utf-8",
            )
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
