"""Dynamic-robust retrain for the core-range 3-12m model.

Combines the integrity-audited static (27 groups / 832 frames) and dynamic
(8 groups / 844 frames) corpora into one group-disjoint cross-validation,
benchmarks three feature variants (PHYSICAL_ONLY, PHYSICAL_TEMPORAL,
BBOX_AUGMENTED), and selects a candidate only if it clears the static,
dynamic, temporal and bbox-robustness development gates simultaneously.

This module has no runtime/controller import and never overwrites the frozen
baseline candidate (SHA-256 8c3f2bb4...); it is read-only reference here.
"""

from __future__ import annotations

import argparse
import copy
import csv
import hashlib
import json
import math
import os
from pathlib import Path
from statistics import mean, median
from typing import Any, Mapping, Sequence

import numpy as np

from core_range_logging_eval import sha256_file
from core_range_xgboost_benchmark import (
    BINS,
    FEATURE_NAMES as FULL_FEATURE_NAMES,
    HYPERPARAMETERS,
    bin_bounds,
    bin_contains,
    canonical_sha256,
    collect_verified_rows,
    metric_values,
)
from core_range_xgboost_benchmark import assign_folds as static_assign_folds
from core_range_direct_dynamic_replay import (
    DIRECTION_DEADBAND_M_S,
    STRESS_VARIANTS,
    _direction_row,
    _group_metric,
    _load_dynamic_rows,
    _stop_row,
    perturb_bbox_features,
    verify_frozen_candidate,
)

RETRAIN_ID = "core_range_dynamic_robust_retrain_20260805_v001"
SEED = 52
BASELINE_CANDIDATE_CHECKSUM = (
    "8c3f2bb450439e6141143ae63ca021ac1fbbc76f25cd62e655aa434c652671fc"
)
CLIP_BOUNDS = (3.0, 12.0)

BBOX_RELATED_FEATURES = (
    "bbox_center_x_fraction", "bbox_center_y_fraction",
    "bbox_width_fraction", "bbox_height_fraction",
    "bbox_area_fraction", "bbox_aspect_ratio", "bbox_tracking_score",
)
PHYSICAL_ONLY_FEATURES = tuple(
    name for name in FULL_FEATURE_NAMES if name not in BBOX_RELATED_FEATURES
)
TEMPORAL_EXTRA_FEATURES = (
    "delta_time_s",
    "previous_raw_physical_range_m", "delta_raw_physical_range_m",
    "previous_target_inverse_depth", "delta_target_inverse_depth",
    "causal_ema_raw_range_m", "causal_rolling_median_raw_range_m",
    "causal_raw_range_rate_m_s", "causal_inverse_depth_rate",
    "calibration_a_rate", "calibration_b_rate", "validity_streak",
)
PHYSICAL_TEMPORAL_FEATURES = PHYSICAL_ONLY_FEATURES + TEMPORAL_EXTRA_FEATURES
BBOX_AUGMENTED_FEATURES = FULL_FEATURE_NAMES

FEATURE_VARIANTS = {
    "PHYSICAL_ONLY": PHYSICAL_ONLY_FEATURES,
    "PHYSICAL_TEMPORAL": PHYSICAL_TEMPORAL_FEATURES,
    "BBOX_AUGMENTED": BBOX_AUGMENTED_FEATURES,
}
VARIANT_PRIORITY = ("PHYSICAL_TEMPORAL", "PHYSICAL_ONLY", "BBOX_AUGMENTED")

CAUSAL_TAU_S = 0.20
CAUSAL_ROLLING_WINDOW = 5
BBOX_AUGMENT_VARIANTS = (
    "scale_minus_2", "scale_plus_2", "scale_minus_5", "scale_plus_5",
    "scale_minus_10", "scale_plus_10",
    "center_x_minus_2", "center_x_plus_2", "center_x_minus_5", "center_x_plus_5",
    "center_y_minus_2", "center_y_plus_2", "center_y_minus_5", "center_y_plus_5",
)
assert set(BBOX_AUGMENT_VARIANTS) <= set(STRESS_VARIANTS)

STATIC_GATES = {
    "equal_group_mae_m_max": 1.0,
    "worst_bin_mae_m_max": 1.5,
    "p90_m_max": 2.0,
    "error_gt_3m_fraction_max": 0.01,
    "stationary_std_m_max": 0.30,
}
DYNAMIC_GATES = {
    "equal_group_mae_m_max": 1.0,
    "worst_session_mae_m_max": 1.5,
    "p90_m_max": 2.0,
    "error_gt_3m_fraction_max": 0.01,
}
TEMPORAL_GATES = {
    "median_absolute_lag_s_max": 0.30,
    "stop_settling_time_s_max": 1.0,
    "stop_stationary_std_m_max": 0.30,
}
BBOX_GATES = {
    "pm5_mae_degradation_m_max": 0.30,
    "pm5_median_prediction_shift_m_max": 0.50,
    "pm10_catastrophic_error_fraction_max": 0.0,
}


def _json(path: Path) -> Any:
    return json.loads(path.read_text(encoding="utf-8"))


def _write_csv(path: Path, rows: Sequence[Mapping[str, Any]]) -> None:
    rows = list(rows)
    names: list[str] = []
    seen = set()
    for row in rows:
        for key in row:
            if key not in seen:
                seen.add(key)
                names.append(key)
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=names)
        writer.writeheader()
        writer.writerows(rows)


# ---------------------------------------------------------------------------
# Data loading: unify static + dynamic rows into one schema.
# ---------------------------------------------------------------------------

def load_combined_rows(
    workspace: Path, dynamic_output: Path
) -> tuple[list[dict[str, Any]], dict[str, Any]]:
    static_raw, static_manifest = collect_verified_rows(workspace)
    if static_manifest["group_count"] != 27 or static_manifest["frame_count"] != 832:
        raise ValueError(
            f"static_corpus_shape_unexpected:{static_manifest['group_count']}:"
            f"{static_manifest['frame_count']}"
        )
    plan, _candidate = verify_frozen_candidate(workspace, dynamic_output)
    dynamic_raw, dynamic_session_manifest = _load_dynamic_rows(
        workspace, dynamic_output, plan
    )
    if dynamic_session_manifest["accepted_group_count"] != 8 or len(dynamic_raw) != 844:
        raise ValueError(
            f"dynamic_corpus_shape_unexpected:"
            f"{dynamic_session_manifest['accepted_group_count']}:{len(dynamic_raw)}"
        )

    combined: list[dict[str, Any]] = []
    for row in static_raw:
        features = {name: row[name] for name in FULL_FEATURE_NAMES}
        combined.append({
            "domain": "static",
            "group_id": row["logical_session_id"],
            "session_id": row["logical_session_id"],
            "scenario_type": "static",
            "context": row["distance_bin"],
            "distance_bin": row["distance_bin"],
            "measurement_timestamp_s": row["measurement_timestamp_s"],
            "ground_truth_range_m": row["ground_truth_range_m"],
            "raw_physical_range_m": row["raw_physical_range_m"],
            "features": dict(features),
        })
    for row in dynamic_raw:
        combined.append({
            "domain": "dynamic",
            "group_id": row["session_id"],
            "session_id": row["session_id"],
            "scenario_type": row["scenario_type"],
            "context": row["context"],
            "distance_bin": None,
            "measurement_timestamp_s": row["measurement_timestamp_s"],
            "ground_truth_range_m": row["ground_truth_range_m"],
            "raw_physical_range_m": row["raw_physical_range_m"],
            "features": dict(row["features"]),
        })

    manifest = {
        "static": {
            k: v for k, v in static_manifest.items() if k != "sources"
        },
        "static_source_count": len(static_manifest["sources"]),
        "dynamic_session_manifest_sha256": canonical_sha256(dynamic_session_manifest),
        "dynamic_accepted_group_count": dynamic_session_manifest["accepted_group_count"],
        "dynamic_accepted_frame_count": len(dynamic_raw),
        "total_group_count": len({row["group_id"] for row in combined}),
        "total_frame_count": len(combined),
        "baseline_candidate_checksum_unchanged": (
            plan["candidate_checksum"] == BASELINE_CANDIDATE_CHECKSUM
        ),
    }
    return combined, manifest


def add_temporal_features(rows: Sequence[dict[str, Any]]) -> None:
    """Mutates each row's ``features`` dict in place with causal features.

    Reset at every group (session) boundary; uses only the current and
    strictly-past frames of the same group, ordered by measurement timestamp.
    """
    by_group: dict[str, list[dict[str, Any]]] = {}
    for row in rows:
        by_group.setdefault(row["group_id"], []).append(row)
    for group_rows in by_group.values():
        ordered = sorted(group_rows, key=lambda r: float(r["measurement_timestamp_s"]))
        prev_raw = prev_inv = prev_a = prev_b = prev_t = None
        ema: float | None = None
        rolling: list[float] = []
        streak = 0.0
        for row in ordered:
            raw = float(row["raw_physical_range_m"])
            inv = float(row["features"]["target_inverse_depth"])
            a = float(row["features"]["calibration_a"])
            b = float(row["features"]["calibration_b"])
            t = float(row["measurement_timestamp_s"])
            accepted = float(row["features"]["calibration_measurement_accepted"])
            if prev_t is None:
                dt, d_raw, d_inv = 0.0, 0.0, 0.0
                rate_raw = rate_inv = a_rate = b_rate = 0.0
                ema = raw
                rolling = [raw]
                streak = accepted
                p_raw, p_inv = raw, inv
            else:
                dt = max(t - prev_t, 1e-6)
                d_raw = raw - prev_raw
                d_inv = inv - prev_inv
                rate_raw = d_raw / dt
                rate_inv = d_inv / dt
                a_rate = (a - prev_a) / dt
                b_rate = (b - prev_b) / dt
                alpha = 1.0 - math.exp(-dt / CAUSAL_TAU_S)
                ema = ema + alpha * (raw - ema)
                rolling.append(raw)
                if len(rolling) > CAUSAL_ROLLING_WINDOW:
                    rolling.pop(0)
                streak = streak + accepted if accepted > 0.5 else 0.0
                p_raw, p_inv = prev_raw, prev_inv
            row["features"]["delta_time_s"] = dt
            row["features"]["previous_raw_physical_range_m"] = p_raw
            row["features"]["delta_raw_physical_range_m"] = d_raw
            row["features"]["previous_target_inverse_depth"] = p_inv
            row["features"]["delta_target_inverse_depth"] = d_inv
            row["features"]["causal_ema_raw_range_m"] = float(ema)
            row["features"]["causal_rolling_median_raw_range_m"] = float(np.median(rolling))
            row["features"]["causal_raw_range_rate_m_s"] = rate_raw
            row["features"]["causal_inverse_depth_rate"] = rate_inv
            row["features"]["calibration_a_rate"] = a_rate
            row["features"]["calibration_b_rate"] = b_rate
            row["features"]["validity_streak"] = streak
            prev_raw, prev_inv, prev_a, prev_b, prev_t = raw, inv, a, b, t


# ---------------------------------------------------------------------------
# Group-disjoint combined fold assignment.
# ---------------------------------------------------------------------------

def assign_combined_folds(
    workspace: Path, combined_rows: Sequence[dict[str, Any]], *, seed: int = SEED
) -> dict[str, int]:
    static_raw, _ = collect_verified_rows(workspace)
    static_assignment = static_assign_folds(static_raw)
    fold_map = {row["logical_session_id"]: int(row["fold"]) for row in static_assignment}

    dynamic_groups_by_type: dict[str, set[str]] = {}
    for row in combined_rows:
        if row["domain"] != "dynamic":
            continue
        dynamic_groups_by_type.setdefault(row["scenario_type"], set()).add(row["group_id"])
    for scenario_type, groups in dynamic_groups_by_type.items():
        ordered = sorted(
            groups,
            key=lambda g: hashlib.sha256(f"{seed}:{scenario_type}:{g}".encode()).hexdigest(),
        )
        for index, group in enumerate(ordered):
            fold_map[group] = index % 3

    all_groups = {row["group_id"] for row in combined_rows}
    if set(fold_map) != all_groups:
        raise ValueError("combined_fold_group_mismatch")
    for fold in range(3):
        fold_groups = {group for group, assigned in fold_map.items() if assigned == fold}
        static_in_fold = any(
            row["group_id"] in fold_groups for row in combined_rows if row["domain"] == "static"
        )
        dynamic_in_fold = any(
            row["group_id"] in fold_groups for row in combined_rows if row["domain"] == "dynamic"
        )
        if not (static_in_fold and dynamic_in_fold):
            raise ValueError(f"fold_missing_domain:{fold}")
    return fold_map


# ---------------------------------------------------------------------------
# Preprocessing, augmentation, training.
# ---------------------------------------------------------------------------

class VariantPreprocessor:
    def __init__(self, feature_names: Sequence[str], medians: np.ndarray) -> None:
        self.feature_names = tuple(feature_names)
        self.medians = medians

    @classmethod
    def fit(cls, rows: Sequence[Mapping[str, Any]], feature_names: Sequence[str]) -> "VariantPreprocessor":
        matrix = np.asarray(
            [[_as_float_or_nan(row["features"].get(name)) for name in feature_names] for row in rows],
            dtype=np.float64,
        )
        medians = np.nanmedian(matrix, axis=0)
        if np.any(~np.isfinite(medians)):
            missing = [feature_names[i] for i in np.where(~np.isfinite(medians))[0]]
            raise ValueError(f"training_feature_all_missing:{missing}")
        return cls(feature_names, medians)

    def transform(self, rows: Sequence[Mapping[str, Any]]) -> np.ndarray:
        matrix = np.asarray(
            [[_as_float_or_nan(row["features"].get(name)) for name in self.feature_names] for row in rows],
            dtype=np.float64,
        )
        missing = np.isnan(matrix)
        filled = np.where(missing, self.medians.reshape(1, -1), matrix)
        result = np.hstack([filled, missing.astype(np.float64)])
        if not np.all(np.isfinite(result)):
            raise ValueError("preprocessed_features_nonfinite")
        return result

    def as_json(self, train_groups: Sequence[str]) -> dict[str, Any]:
        return {
            "fit_partition": "train_only",
            "train_groups": sorted(set(train_groups)),
            "feature_names": list(self.feature_names),
            "output_feature_names": list(self.feature_names) + [f"{n}__missing" for n in self.feature_names],
            "medians": {name: float(value) for name, value in zip(self.feature_names, self.medians, strict=True)},
        }


def _as_float_or_nan(value: Any) -> float:
    if value is None:
        return float("nan")
    return float(value)


def augment_bbox_rows(train_rows: Sequence[dict[str, Any]]) -> list[dict[str, Any]]:
    augmented = list(train_rows)
    for row in train_rows:
        for variant in BBOX_AUGMENT_VARIANTS:
            new_features = perturb_bbox_features(row["features"], variant)
            augmented.append({**row, "features": new_features, "augmented": True, "augment_variant": variant})
    return augmented


def unified_group_weights(rows: Sequence[Mapping[str, Any]]) -> np.ndarray:
    counts: dict[str, int] = {}
    for row in rows:
        counts[row["group_id"]] = counts.get(row["group_id"], 0) + 1
    target = len(rows) / len(counts)
    return np.asarray([target / counts[row["group_id"]] for row in rows], dtype=np.float64)


def _xgb_parameters(config: Mapping[str, Any]) -> tuple[dict[str, Any], int]:
    return ({
        "objective": "reg:squarederror", "eval_metric": "mae", "tree_method": "hist",
        "max_depth": int(config["max_depth"]), "eta": float(config["learning_rate"]),
        "min_child_weight": float(config["min_child_weight"]), "subsample": float(config["subsample"]),
        "colsample_bytree": float(config["colsample_bytree"]), "alpha": float(config["reg_alpha"]),
        "lambda": float(config["reg_lambda"]), "seed": SEED, "nthread": 1,
    }, int(config["n_estimators"]))


def per_group_rows(rows: Sequence[Mapping[str, Any]], prediction: np.ndarray, method: str, config: str = "N/A") -> list[dict[str, Any]]:
    results = []
    groups = sorted({row["group_id"] for row in rows})
    for group in groups:
        indices = [i for i, row in enumerate(rows) if row["group_id"] == group]
        selected = [rows[i] for i in indices]
        truth = np.asarray([float(r["ground_truth_range_m"]) for r in selected])
        pred = prediction[indices]
        base = metric_values(truth, pred)
        results.append({
            "method": method, "config": config, "group_id": group,
            "domain": selected[0]["domain"], "scenario_type": selected[0]["scenario_type"],
            "context": selected[0]["context"], "distance_bin": selected[0]["distance_bin"],
            "frame_count": len(indices),
            "prediction_std_m": float(np.std(pred)),
            **base,
        })
    return results


def equal_group_aggregate(group_rows: Sequence[Mapping[str, Any]]) -> dict[str, float]:
    keys = (
        "signed_bias_m", "median_absolute_error_m", "mae_m", "rmse_m",
        "p90_abs_error_m", "p95_abs_error_m", "mean_absolute_relative_error",
        "error_gt_1m_fraction", "error_gt_2m_fraction", "error_gt_3m_fraction",
    )
    return {key: mean(float(row[key]) for row in group_rows) for key in keys}


# ---------------------------------------------------------------------------
# bbox robustness stress (reuses the same variant math as bbox_stress in the
# dynamic replay evaluator; PHYSICAL_ONLY/PHYSICAL_TEMPORAL exclude bbox_*
# from feature_names entirely, so they are expected to show ~0 degradation).
# ---------------------------------------------------------------------------

def bbox_stress_metrics(
    xgb: Any, booster: Any, preprocessor: VariantPreprocessor,
    test_rows: Sequence[dict[str, Any]],
) -> list[dict[str, Any]]:
    original_x = preprocessor.transform(test_rows)
    original_pred = np.clip(np.asarray(booster.predict(xgb.DMatrix(original_x)), dtype=np.float64), *CLIP_BOUNDS)
    truth = np.asarray([float(row["ground_truth_range_m"]) for row in test_rows])
    orig_groups = per_group_rows(test_rows, original_pred, "original")
    orig_mae = equal_group_aggregate(orig_groups)["mae_m"]
    results = []
    for variant in BBOX_AUGMENT_VARIANTS:
        perturbed_rows = [
            {**row, "features": perturb_bbox_features(row["features"], variant)}
            for row in test_rows
        ]
        perturbed_x = preprocessor.transform(perturbed_rows)
        perturbed_pred = np.clip(np.asarray(booster.predict(xgb.DMatrix(perturbed_x)), dtype=np.float64), *CLIP_BOUNDS)
        shift = np.abs(perturbed_pred - original_pred)
        pert_groups = per_group_rows(test_rows, perturbed_pred, variant)
        pert_mae = equal_group_aggregate(pert_groups)["mae_m"]
        results.append({
            "variant": variant,
            "equal_group_mae_degradation_m": pert_mae - orig_mae,
            "median_absolute_prediction_shift_m": float(np.median(shift)),
            "maximum_absolute_prediction_shift_m": float(np.max(shift)),
            "catastrophic_error_gt_3m_fraction": float(np.mean(np.abs(perturbed_pred - truth) > 3.0)),
        })
    return results


def _bbox_gate(stress_rows: Sequence[Mapping[str, Any]]) -> dict[str, Any]:
    pm5 = [r for r in stress_rows if r["variant"].endswith("_5")]
    pm10 = [r for r in stress_rows if r["variant"].endswith("_10")]
    worst_pm5_degradation = max((float(r["equal_group_mae_degradation_m"]) for r in pm5), default=0.0)
    worst_pm5_shift = max((float(r["median_absolute_prediction_shift_m"]) for r in pm5), default=0.0)
    worst_pm10_catastrophic = max((float(r["catastrophic_error_gt_3m_fraction"]) for r in pm10), default=0.0)
    checks = {
        "pm5_mae_degradation": worst_pm5_degradation <= BBOX_GATES["pm5_mae_degradation_m_max"],
        "pm5_median_prediction_shift": worst_pm5_shift <= BBOX_GATES["pm5_median_prediction_shift_m_max"],
        "pm10_no_catastrophic_error": worst_pm10_catastrophic <= BBOX_GATES["pm10_catastrophic_error_fraction_max"],
    }
    return {
        "passed": all(checks.values()), "checks": checks,
        "worst_pm5_mae_degradation_m": worst_pm5_degradation,
        "worst_pm5_median_prediction_shift_m": worst_pm5_shift,
        "worst_pm10_catastrophic_error_fraction": worst_pm10_catastrophic,
    }


# ---------------------------------------------------------------------------
# Static / dynamic / temporal gates.
# ---------------------------------------------------------------------------

def _static_gate(static_group_rows: Sequence[Mapping[str, Any]], bin_rows: Sequence[Mapping[str, Any]]) -> dict[str, Any]:
    agg = equal_group_aggregate(static_group_rows)
    worst_bin = max((float(r["equal_group_mae_m"]) for r in bin_rows), default=float("nan"))
    stationary = median(float(r["prediction_std_m"]) for r in static_group_rows)
    checks = {
        "equal_group_mae": agg["mae_m"] <= STATIC_GATES["equal_group_mae_m_max"],
        "worst_bin_mae": worst_bin <= STATIC_GATES["worst_bin_mae_m_max"],
        "p90": agg["p90_abs_error_m"] <= STATIC_GATES["p90_m_max"],
        "error_gt_3m": agg["error_gt_3m_fraction"] <= STATIC_GATES["error_gt_3m_fraction_max"],
        "stationary_std": stationary <= STATIC_GATES["stationary_std_m_max"],
    }
    return {
        "passed": all(checks.values()), "checks": checks,
        "equal_group_mae_m": agg["mae_m"], "worst_bin_mae_m": worst_bin,
        "p90_m": agg["p90_abs_error_m"], "error_gt_3m_fraction": agg["error_gt_3m_fraction"],
        "stationary_std_m": stationary,
    }


def _dynamic_gate(dynamic_group_rows: Sequence[Mapping[str, Any]]) -> dict[str, Any]:
    agg = equal_group_aggregate(dynamic_group_rows)
    worst_session = max(float(r["mae_m"]) for r in dynamic_group_rows)
    checks = {
        "equal_group_mae": agg["mae_m"] <= DYNAMIC_GATES["equal_group_mae_m_max"],
        "worst_session_mae": worst_session <= DYNAMIC_GATES["worst_session_mae_m_max"],
        "p90": agg["p90_abs_error_m"] <= DYNAMIC_GATES["p90_m_max"],
        "error_gt_3m": agg["error_gt_3m_fraction"] <= DYNAMIC_GATES["error_gt_3m_fraction_max"],
    }
    return {
        "passed": all(checks.values()), "checks": checks,
        "equal_group_mae_m": agg["mae_m"], "worst_session_mae_m": worst_session,
        "p90_m": agg["p90_abs_error_m"], "error_gt_3m_fraction": agg["error_gt_3m_fraction"],
    }


def _temporal_gate(direction_rows: Sequence[Mapping[str, Any]], stop_rows: Sequence[Mapping[str, Any]]) -> dict[str, Any]:
    lags = [abs(float(r["alignment_lag_s"])) for r in direction_rows if r["alignment_lag_s"] is not None]
    median_lag = median(lags) if lags else float("inf")
    settled = [r for r in stop_rows if r is not None]
    settling_times = [r["settling_time_s"] for r in settled]
    worst_settling = max((float(v) for v in settling_times if v is not None), default=float("inf"))
    never_settled = any(v is None for v in settling_times)
    stationary_stds = [float(r["stationary_std_m"]) for r in settled if r["stationary_std_m"] is not None]
    worst_stationary = max(stationary_stds, default=float("inf"))
    checks = {
        "median_absolute_lag": median_lag <= TEMPORAL_GATES["median_absolute_lag_s_max"],
        "stop_settling_time": (not never_settled) and worst_settling <= TEMPORAL_GATES["stop_settling_time_s_max"],
        "stop_stationary_std": worst_stationary <= TEMPORAL_GATES["stop_stationary_std_m_max"],
    }
    return {
        "passed": all(checks.values()), "checks": checks,
        "median_absolute_lag_s": median_lag, "worst_stop_settling_time_s": worst_settling,
        "any_stop_never_settled": never_settled, "worst_stop_stationary_std_m": worst_stationary,
    }


# ---------------------------------------------------------------------------
# Per-variant training: strict out-of-fold (OOF) predictions only. No row is
# ever scored by a booster that saw it (or its group) during training.
# ---------------------------------------------------------------------------

def train_variant(
    xgb: Any, variant_name: str, feature_names: Sequence[str],
    combined_rows: Sequence[dict[str, Any]], fold_map: Mapping[str, int],
) -> dict[str, Any]:
    n = len(combined_rows)
    oof: dict[str, np.ndarray] = {}
    boosters: dict[tuple[str, int], Any] = {}
    preprocessors: dict[tuple[str, int], VariantPreprocessor] = {}
    fold_metrics: list[dict[str, Any]] = []
    for config_name, config in HYPERPARAMETERS.items():
        config_oof = np.full(n, np.nan, dtype=np.float64)
        for fold in range(3):
            train_indices = [i for i, row in enumerate(combined_rows) if fold_map[row["group_id"]] != fold]
            test_indices = [i for i, row in enumerate(combined_rows) if fold_map[row["group_id"]] == fold]
            train_rows = [combined_rows[i] for i in train_indices]
            test_rows = [combined_rows[i] for i in test_indices]
            train_rows_for_fit = augment_bbox_rows(train_rows) if variant_name == "BBOX_AUGMENTED" else train_rows
            preprocessor = VariantPreprocessor.fit(train_rows_for_fit, feature_names)
            train_x = preprocessor.transform(train_rows_for_fit)
            test_x = preprocessor.transform(test_rows)
            train_y = np.asarray([float(r["ground_truth_range_m"]) for r in train_rows_for_fit])
            weights = unified_group_weights(train_rows_for_fit)
            params, rounds = _xgb_parameters(config)
            booster = xgb.train(
                params, xgb.DMatrix(train_x, label=train_y, weight=weights),
                num_boost_round=rounds, verbose_eval=False,
            )
            test_pred = np.clip(np.asarray(booster.predict(xgb.DMatrix(test_x)), dtype=np.float64), *CLIP_BOUNDS)
            config_oof[test_indices] = test_pred
            boosters[(config_name, fold)] = booster
            preprocessors[(config_name, fold)] = preprocessor
            train_pred = np.clip(np.asarray(booster.predict(xgb.DMatrix(train_x)), dtype=np.float64), *CLIP_BOUNDS)
            train_groups = per_group_rows(train_rows_for_fit, train_pred, variant_name, config_name)
            test_groups = per_group_rows(test_rows, test_pred, variant_name, config_name)
            for partition, group_rows in (("train", train_groups), ("test", test_groups)):
                fold_metrics.append({
                    "variant": variant_name, "config": config_name, "fold": fold, "partition": partition,
                    "group_count": len(group_rows), "frame_count": sum(int(r["frame_count"]) for r in group_rows),
                    **equal_group_aggregate(group_rows),
                })
        if np.any(~np.isfinite(config_oof)):
            raise ValueError(f"oof_incomplete:{variant_name}:{config_name}")
        oof[config_name] = config_oof
    return {
        "oof": oof, "boosters": boosters, "preprocessors": preprocessors,
        "fold_metrics": fold_metrics,
    }


def oof_bbox_stress(
    xgb: Any, combined_rows: Sequence[dict[str, Any]], fold_map: Mapping[str, int],
    boosters: Mapping[tuple[str, int], Any], preprocessors: Mapping[tuple[str, int], VariantPreprocessor],
    config_name: str,
) -> list[dict[str, Any]]:
    """Perturb bbox features and re-predict, always using the OOF booster/
    preprocessor pair for each row's own held-out fold (no leakage)."""
    n = len(combined_rows)
    original = np.full(n, np.nan, dtype=np.float64)
    for fold in range(3):
        indices = [i for i, row in enumerate(combined_rows) if fold_map[row["group_id"]] == fold]
        rows = [combined_rows[i] for i in indices]
        preprocessor = preprocessors[(config_name, fold)]
        booster = boosters[(config_name, fold)]
        x = preprocessor.transform(rows)
        original[indices] = np.clip(np.asarray(booster.predict(xgb.DMatrix(x)), dtype=np.float64), *CLIP_BOUNDS)
    truth = np.asarray([float(row["ground_truth_range_m"]) for row in combined_rows])
    orig_groups = per_group_rows(combined_rows, original, "original")
    orig_mae = equal_group_aggregate(orig_groups)["mae_m"]
    results = []
    for variant in BBOX_AUGMENT_VARIANTS:
        perturbed = np.full(n, np.nan, dtype=np.float64)
        for fold in range(3):
            indices = [i for i, row in enumerate(combined_rows) if fold_map[row["group_id"]] == fold]
            rows = [
                {**combined_rows[i], "features": perturb_bbox_features(combined_rows[i]["features"], variant)}
                for i in indices
            ]
            preprocessor = preprocessors[(config_name, fold)]
            booster = boosters[(config_name, fold)]
            x = preprocessor.transform(rows)
            perturbed[indices] = np.clip(np.asarray(booster.predict(xgb.DMatrix(x)), dtype=np.float64), *CLIP_BOUNDS)
        shift = np.abs(perturbed - original)
        pert_groups = per_group_rows(combined_rows, perturbed, variant)
        pert_mae = equal_group_aggregate(pert_groups)["mae_m"]
        results.append({
            "variant": variant,
            "equal_group_mae_degradation_m": pert_mae - orig_mae,
            "median_absolute_prediction_shift_m": float(np.median(shift)),
            "maximum_absolute_prediction_shift_m": float(np.max(shift)),
            "catastrophic_error_gt_3m_fraction": float(np.mean(np.abs(perturbed - truth) > 3.0)),
        })
    return results


def evaluate_config(
    combined_rows: Sequence[dict[str, Any]], oof_prediction: np.ndarray,
) -> dict[str, Any]:
    static_rows = [row for row in combined_rows if row["domain"] == "static"]
    dynamic_rows = [row for row in combined_rows if row["domain"] == "dynamic"]
    static_indices = [i for i, row in enumerate(combined_rows) if row["domain"] == "static"]
    dynamic_indices = [i for i, row in enumerate(combined_rows) if row["domain"] == "dynamic"]
    static_pred = oof_prediction[static_indices]
    dynamic_pred = oof_prediction[dynamic_indices]

    static_groups = per_group_rows(static_rows, static_pred, "oof")
    bin_rows = []
    for label in BINS:
        chosen = [row for row in static_groups if row["distance_bin"] == label]
        if chosen:
            bin_rows.append({"distance_bin": label, **equal_group_aggregate(chosen), "equal_group_mae_m": equal_group_aggregate(chosen)["mae_m"]})
    static_gate = _static_gate(static_groups, bin_rows)

    dynamic_groups = per_group_rows(dynamic_rows, dynamic_pred, "oof")
    dynamic_gate = _dynamic_gate(dynamic_groups)
    dynamic_by_type = {}
    for scenario_type in ("approaching", "receding", "stop_and_hold"):
        chosen = [row for row in dynamic_groups if row["scenario_type"] == scenario_type]
        if chosen:
            dynamic_by_type[scenario_type] = equal_group_aggregate(chosen)

    direction_rows = []
    stop_rows = []
    by_session: dict[str, list[int]] = {}
    for i in dynamic_indices:
        by_session.setdefault(combined_rows[i]["group_id"], []).append(i)
    for session_id, idx in by_session.items():
        session_rows = sorted(
            [combined_rows[i] for i in idx], key=lambda r: float(r["measurement_timestamp_s"])
        )
        sorted_idx = sorted(idx, key=lambda i: float(combined_rows[i]["measurement_timestamp_s"]))
        session_pred = oof_prediction[sorted_idx]
        direction_rows.append(_direction_row(session_rows, session_pred, "oof"))
        stop_result = _stop_row(session_rows, session_pred, "oof")
        if stop_result is not None:
            stop_rows.append(stop_result)
    temporal_gate = _temporal_gate(direction_rows, stop_rows)

    return {
        "static_groups": static_groups, "static_bin_rows": bin_rows, "static_gate": static_gate,
        "dynamic_groups": dynamic_groups, "dynamic_gate": dynamic_gate, "dynamic_by_scenario_type": dynamic_by_type,
        "direction_rows": direction_rows, "stop_rows": stop_rows, "temporal_gate": temporal_gate,
    }


# ---------------------------------------------------------------------------
# prepare() / run() / main() -- same offline precommit-then-fit discipline as
# core_range_xgboost_benchmark.py.
# ---------------------------------------------------------------------------

def prepare(workspace: Path, output: Path, dynamic_output: Path) -> dict[str, Any]:
    if (output / "retrain_manifest.json").exists():
        raise ValueError("completed_retrain_already_exists")
    output.mkdir(parents=True, exist_ok=True)
    combined_rows, dataset_manifest = load_combined_rows(workspace, dynamic_output)
    if not dataset_manifest["baseline_candidate_checksum_unchanged"]:
        raise ValueError("baseline_candidate_checksum_changed")
    add_temporal_features(combined_rows)
    fold_map = assign_combined_folds(workspace, combined_rows)

    fold_rows = [
        {"group_id": group, "fold": fold, "domain": next(r["domain"] for r in combined_rows if r["group_id"] == group)}
        for group, fold in sorted(fold_map.items())
    ]
    dataset_path = output / "frozen_dataset_manifest.json"
    folds_path = output / "fold_assignments.csv"
    variants_path = output / "feature_variants.json"
    dataset_path.write_text(json.dumps(dataset_manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    _write_csv(folds_path, fold_rows)
    variants_payload = {
        name: {"feature_names": list(names), "feature_count": len(names)}
        for name, names in FEATURE_VARIANTS.items()
    }
    variants_payload["bbox_related_features_excluded_from_physical_variants"] = list(BBOX_RELATED_FEATURES)
    variants_payload["temporal_extra_features"] = list(TEMPORAL_EXTRA_FEATURES)
    variants_payload["bbox_augment_variants"] = list(BBOX_AUGMENT_VARIANTS)
    variants_path.write_text(json.dumps(variants_payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")

    plan = {
        "retrain_id": RETRAIN_ID, "created_before_fit": True, "seed": SEED,
        "baseline_candidate_checksum": BASELINE_CANDIDATE_CHECKSUM,
        "baseline_candidate_untouched": True,
        "no_additional_data_collection": True,
        "cross_validation": {
            "type": "3_fold_group_disjoint_static_plus_dynamic",
            "total_groups": dataset_manifest["total_group_count"],
            "total_frames": dataset_manifest["total_frame_count"],
            "every_fold_has_static_and_dynamic_groups": True,
            "fold_assignments_sha256": sha256_file(folds_path),
        },
        "feature_variants": list(FEATURE_VARIANTS),
        "hyperparameters": HYPERPARAMETERS,
        "xgboost_common": {"objective": "reg:squarederror", "eval_metric": "mae", "tree_method": "hist", "seed": SEED, "nthread": 1},
        "static_gates": STATIC_GATES, "dynamic_gates": DYNAMIC_GATES,
        "temporal_gates": TEMPORAL_GATES, "bbox_gates": BBOX_GATES,
        "variant_priority": list(VARIANT_PRIORITY),
        "model_selection": "best_dynamic_equal_group_mae_among_configs_passing_all_gates_in_first_passing_variant_by_priority",
        "source_contracts": {
            "frozen_dataset_manifest_sha256": sha256_file(dataset_path),
            "feature_variants_sha256": sha256_file(variants_path),
        },
        "scope_guards": {
            "additional_data_collection": False, "baseline_candidate_overwritten": False,
            "runtime_modified": False, "controller_effect": False, "shadow": False,
            "follow_target": False, "final_holdout": False,
        },
    }
    plan_path = output / "retrain_plan.json"
    plan_path.write_text(json.dumps(plan, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps({
        "phase": "PRECOMMIT_COMPLETE_NO_FIT", "retrain_plan_sha256": sha256_file(plan_path),
        "groups": dataset_manifest["total_group_count"], "frames": dataset_manifest["total_frame_count"],
    }, indent=2))
    return plan


def _validate_precommit(output: Path) -> dict[str, Any]:
    plan = _json(output / "retrain_plan.json")
    if plan.get("retrain_id") != RETRAIN_ID or plan.get("seed") != SEED or plan.get("created_before_fit") is not True:
        raise ValueError("retrain_plan_invalid")
    contracts = plan["source_contracts"]
    if sha256_file(output / "frozen_dataset_manifest.json") != contracts["frozen_dataset_manifest_sha256"]:
        raise ValueError("dataset_manifest_changed_after_precommit")
    if sha256_file(output / "feature_variants.json") != contracts["feature_variants_sha256"]:
        raise ValueError("feature_variants_changed_after_precommit")
    if sha256_file(output / "fold_assignments.csv") != plan["cross_validation"]["fold_assignments_sha256"]:
        raise ValueError("fold_assignments_changed_after_precommit")
    return plan


def run(workspace: Path, output: Path, dynamic_output: Path) -> dict[str, Any]:
    plan = _validate_precommit(output)
    combined_rows, dataset_manifest = load_combined_rows(workspace, dynamic_output)
    if canonical_sha256(dataset_manifest) != canonical_sha256(_json(output / "frozen_dataset_manifest.json")):
        raise ValueError("source_dataset_changed_after_precommit")
    add_temporal_features(combined_rows)
    fold_map: dict[str, int] = {}
    with (output / "fold_assignments.csv").open() as stream:
        for row in csv.DictReader(stream):
            fold_map[row["group_id"]] = int(row["fold"])
    if set(fold_map) != {row["group_id"] for row in combined_rows}:
        raise ValueError("fold_group_mapping_incomplete")

    try:
        import xgboost as xgb
    except ImportError as error:
        raise RuntimeError("xgboost_offline_dependency_missing") from error

    models_dir = output / "models"
    models_dir.mkdir(exist_ok=True)

    all_static_metric_rows: list[dict[str, Any]] = []
    all_bin_rows: list[dict[str, Any]] = []
    all_dynamic_metric_rows: list[dict[str, Any]] = []
    all_temporal_rows: list[dict[str, Any]] = []
    all_bbox_rows: list[dict[str, Any]] = []
    all_fold_metrics: list[dict[str, Any]] = []
    model_comparison: list[dict[str, Any]] = []
    per_config_state: dict[tuple[str, str], dict[str, Any]] = {}

    for variant_name, feature_names in FEATURE_VARIANTS.items():
        trained = train_variant(xgb, variant_name, feature_names, combined_rows, fold_map)
        all_fold_metrics.extend(trained["fold_metrics"])
        for config_name in HYPERPARAMETERS:
            oof_prediction = trained["oof"][config_name]
            evaluation = evaluate_config(combined_rows, oof_prediction)
            bbox_stress = oof_bbox_stress(
                xgb, combined_rows, fold_map, trained["boosters"], trained["preprocessors"], config_name
            )
            bbox_gate = _bbox_gate(bbox_stress)
            for row in bbox_stress:
                all_bbox_rows.append({"variant": variant_name, "config": config_name, **row})
            for row in evaluation["static_groups"]:
                all_static_metric_rows.append({"variant": variant_name, **row, "config": config_name})
            for row in evaluation["static_bin_rows"]:
                all_bin_rows.append({"variant": variant_name, "config": config_name, **row})
            for row in evaluation["dynamic_groups"]:
                all_dynamic_metric_rows.append({"variant": variant_name, **row, "config": config_name})
            for row in evaluation["direction_rows"]:
                all_temporal_rows.append({"variant": variant_name, "config": config_name, "kind": "direction", **row})
            for row in evaluation["stop_rows"]:
                all_temporal_rows.append({"variant": variant_name, "config": config_name, "kind": "stop", **row})
            passed_all = (
                evaluation["static_gate"]["passed"] and evaluation["dynamic_gate"]["passed"]
                and evaluation["temporal_gate"]["passed"] and bbox_gate["passed"]
            )
            per_config_state[(variant_name, config_name)] = {
                "trained": trained, "evaluation": evaluation, "bbox_gate": bbox_gate,
                "bbox_stress": bbox_stress, "passed_all": passed_all,
            }
            model_comparison.append({
                "variant": variant_name, "config": config_name, "passed_all_gates": passed_all,
                "static_mae_m": evaluation["static_gate"]["equal_group_mae_m"],
                "static_passed": evaluation["static_gate"]["passed"],
                "dynamic_mae_m": evaluation["dynamic_gate"]["equal_group_mae_m"],
                "dynamic_passed": evaluation["dynamic_gate"]["passed"],
                "median_absolute_lag_s": evaluation["temporal_gate"]["median_absolute_lag_s"],
                "temporal_passed": evaluation["temporal_gate"]["passed"],
                "bbox_pm10_catastrophic_fraction": bbox_gate["worst_pm10_catastrophic_error_fraction"],
                "bbox_passed": bbox_gate["passed"],
            })

    selected_variant = None
    selected_config = None
    for variant_name in VARIANT_PRIORITY:
        candidates = [
            (config_name, per_config_state[(variant_name, config_name)])
            for config_name in HYPERPARAMETERS
            if per_config_state[(variant_name, config_name)]["passed_all"]
        ]
        if candidates:
            candidates.sort(
                key=lambda item: (
                    item[1]["evaluation"]["dynamic_gate"]["equal_group_mae_m"],
                    item[1]["evaluation"]["static_gate"]["equal_group_mae_m"],
                    item[0],
                )
            )
            selected_variant, (selected_config, _state) = variant_name, candidates[0]
            break

    conclusion = "NO_DYNAMIC_ROBUST_MODEL_MEETS_GATE" if selected_variant is None else "DYNAMIC_ROBUST_CANDIDATE_SELECTED"

    frozen_model_checksum = None
    if selected_variant is not None:
        state = per_config_state[(selected_variant, selected_config)]
        trained = state["trained"]
        for fold in range(3):
            booster = trained["boosters"][(selected_config, fold)]
            preprocessor = trained["preprocessors"][(selected_config, fold)]
            model_path = models_dir / f"{selected_variant}_{selected_config}_fold_{fold}.json"
            booster.save_model(str(model_path))
            train_groups_for_fold = sorted({
                row["group_id"] for row in combined_rows if fold_map[row["group_id"]] != fold
            })
            preprocessing_path = models_dir / f"{selected_variant}_{selected_config}_fold_{fold}_preprocessing.json"
            preprocessing_path.write_text(
                json.dumps(preprocessor.as_json(train_groups_for_fold), indent=2, sort_keys=True) + "\n",
                encoding="utf-8",
            )
        model_files = sorted(models_dir.glob(f"{selected_variant}_{selected_config}_*"))
        frozen_model_checksum = canonical_sha256([sha256_file(p) for p in model_files])

    prediction_rows = []
    for variant_name in FEATURE_VARIANTS:
        for config_name in HYPERPARAMETERS:
            oof_prediction = per_config_state[(variant_name, config_name)]["trained"]["oof"][config_name]
            for index, row in enumerate(combined_rows):
                prediction_rows.append({
                    "variant": variant_name, "config": config_name,
                    "group_id": row["group_id"], "domain": row["domain"], "scenario_type": row["scenario_type"],
                    "fold": fold_map[row["group_id"]],
                    "ground_truth_range_m": row["ground_truth_range_m"],
                    "raw_physical_range_m": row["raw_physical_range_m"],
                    "oof_prediction_m": float(oof_prediction[index]),
                })

    _write_csv(output / "static_metrics.csv", all_static_metric_rows)
    _write_csv(output / "per_bin_metrics.csv", all_bin_rows)
    _write_csv(output / "dynamic_metrics.csv", all_dynamic_metric_rows)
    _write_csv(output / "temporal_metrics.csv", all_temporal_rows)
    _write_csv(output / "bbox_stress_metrics.csv", all_bbox_rows)
    _write_csv(output / "per_group_metrics.csv", all_static_metric_rows + all_dynamic_metric_rows)
    _write_csv(output / "model_comparison.csv", model_comparison)
    _write_csv(output / "prediction_rows.csv", prediction_rows)

    report_lines = [
        "# CORE RANGE DYNAMIC ROBUST RETRAIN REPORT", "",
        f"Conclusion: `{conclusion}`", "",
        f"Selected: `{selected_variant}` / `{selected_config}`" if selected_variant else "No configuration passed all gates.",
        "", "## Model comparison", "",
        "| Variant | Config | Static MAE | Dynamic MAE | Median lag (s) | BBox pm10 catastrophic | All gates |",
        "|---|---|---:|---:|---:|---:|---|",
    ]
    for row in model_comparison:
        report_lines.append(
            f"| {row['variant']} | {row['config']} | {row['static_mae_m']:.3f} | {row['dynamic_mae_m']:.3f} | "
            f"{row['median_absolute_lag_s']:.3f} | {100*row['bbox_pm10_catastrophic_fraction']:.2f}% | "
            f"{'PASS' if row['passed_all_gates'] else 'FAIL'} |"
        )
    report_lines += ["", "## Scope guards", "", "No retraining of the baseline candidate; no additional data collection; no runtime, shadow or Follow Target action.", ""]
    (output / "retrain_report.md").write_text("\n".join(report_lines), encoding="utf-8")

    output_files = [p for p in output.rglob("*") if p.is_file() and p.name != "retrain_manifest.json"]
    manifest = {
        "retrain_id": RETRAIN_ID, "conclusion": conclusion,
        "selected_variant": selected_variant, "selected_config": selected_config,
        "frozen_model_checksum": frozen_model_checksum,
        "baseline_candidate_checksum": BASELINE_CANDIDATE_CHECKSUM,
        "baseline_candidate_overwritten": False,
        "model_comparison": model_comparison,
        "counts": dataset_manifest,
        "scope_guards": {
            "additional_data_collection": False, "runtime_modified": False,
            "controller_effect": False, "shadow": False, "follow_target": False, "final_holdout": False,
        },
        "artifacts": {str(p.relative_to(output)): sha256_file(p) for p in sorted(output_files)},
    }
    (output / "retrain_manifest.json").write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps({"conclusion": conclusion, "selected_variant": selected_variant, "selected_config": selected_config}, indent=2))
    return manifest


def main() -> int:
    parser = argparse.ArgumentParser()
    sub = parser.add_subparsers(dest="command", required=True)
    for command in ("prepare", "run"):
        child = sub.add_parser(command)
        child.add_argument("--workspace", type=Path, required=True)
        child.add_argument("--output", type=Path, required=True)
        child.add_argument("--dynamic-output", type=Path, required=True)
    args = parser.parse_args()
    workspace, output, dynamic_output = args.workspace.resolve(), args.output.resolve(), args.dynamic_output.resolve()
    if args.command == "prepare":
        prepare(workspace, output, dynamic_output)
    else:
        run(workspace, output, dynamic_output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
