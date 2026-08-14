"""CORE_RANGE_STABLE_RELATIVE_TRACKING_3_12M, Phase 3-10.

Pipeline: runtime physical/depth/geometry/quality features -> Direct
XGBoost -> per-fold monotonic calibration (affine or isotonic) -> causal
alpha-beta filter -> clamp [3,12] m -> observation-only stable_range_m.

Optimizes for tracking stability (direction agreement, rank correlation,
stationary jitter, absence of catastrophic jumps, smooth per-bin error) --
not raw global MAE. Reuses the frozen 8/8 dynamic + 27-group static corpus,
read-only. No Gazebo/PX4/ROS2/backend process.
"""

from __future__ import annotations

import csv
import json
from pathlib import Path
from statistics import mean, median, pstdev
from typing import Any

import numpy as np
import xgboost as xgb
from sklearn.isotonic import IsotonicRegression

from core_range_5hz_headless_retrain import load_combined_rows
from core_range_dynamic_robust_retrain import (
    CLIP_BOUNDS,
    VariantPreprocessor,
    assign_combined_folds,
    unified_group_weights,
)

WORKSPACE = Path(__file__).resolve().parent
DYNAMIC_OUTPUT = WORKSPACE / "artifacts/core_range_3_12m/overnight_sim_time_train/frozen_corpus"
OUT = WORKSPACE / "artifacts/core_range_3_12m/stable_relative_tracking"
SEED = 52

# --------------------------------------------------------------------------
# Phase 5: feature contract -- only fields that exist at inference time.
# Deliberately narrower than PHYSICAL_ONLY_FEATURES: excludes bbox_* (per
# both tasks), and also excludes anchor_*/calibration_source_*/
# calibration_inlier_fraction, which are not named in this task's section 5
# allowed-feature list. runtime_quality_correlation.csv from the prior
# offline-analysis task was checked for image_ray_y (the one field with a
# notable static/dynamic distribution shift, SMD=0.62) before deciding to
# keep it: its correlation with abs_residual is weak (r=0.066), not enough
# standalone evidence to drop it per section 5's removal criteria.
# --------------------------------------------------------------------------
STABLE_FEATURES = (
    "raw_physical_range_m",
    "target_inverse_depth",
    "target_roi_q_min", "target_roi_q_p10", "target_roi_q_p25",
    "target_roi_q_median", "target_roi_q_p75", "target_roi_q_p90", "target_roi_q_max",
    "target_roi_q_std", "target_roi_q_iqr",
    "calibration_a", "calibration_b", "calibration_denominator",
    "calibration_fit_residual", "calibration_condition",
    "calibration_inlier_count",
    "calibration_q_span", "calibration_depth_span",
    "calibration_age",
    "image_ray_x", "image_ray_y", "ray_scale",
    "deterministic_gate_applicable", "deterministic_measurement_usable",
    "calibration_measurement_accepted", "calibration_stable",
)

DISTANCE_BINS = [
    ("3-4m", 3.0, 4.0), ("4-5m", 4.0, 5.0), ("5-6m", 5.0, 6.0), ("6-7m", 6.0, 7.0),
    ("7-8m", 7.0, 8.0), ("8-9m", 8.0, 9.0), ("9-10m", 9.0, 10.0), ("10-11m", 10.0, 11.0),
    ("11-12m", 11.0, 12.001),
]


def bin_of(distance: float) -> str:
    for label, lo, hi in DISTANCE_BINS:
        if lo <= distance < hi:
            return label
    return "out_of_range"


def write_csv(path: Path, rows: list[dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    if not rows:
        path.write_text("")
        return
    keys: list[str] = []
    for row in rows:
        for key in row:
            if key not in keys:
                keys.append(key)
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=keys)
        writer.writeheader()
        writer.writerows(rows)


def load_corpus() -> tuple[list[dict[str, Any]], dict[str, int]]:
    combined_rows, _manifest = load_combined_rows(WORKSPACE, DYNAMIC_OUTPUT)
    fold_map = assign_combined_folds(WORKSPACE, combined_rows, seed=SEED)
    for row in combined_rows:
        row["distance_bin_9"] = bin_of(float(row["ground_truth_range_m"]))
    return combined_rows, fold_map


# --------------------------------------------------------------------------
# Phase 4: sample weighting for candidates A/B
# --------------------------------------------------------------------------

def weights_group_balanced(rows: list[dict[str, Any]]) -> np.ndarray:
    """Candidate A: equal total weight per group (reused unmodified)."""
    return unified_group_weights(rows)


def weights_group_and_distance_balanced(
    rows: list[dict[str, Any]], far_boost_bins: tuple[str, ...] = ("10-11m", "11-12m"), far_boost_factor: float = 1.5
) -> np.ndarray:
    """Candidate B: equal group weight, additionally equalized across the
    9 distance bins within the training set, with a modest extra boost for
    the far (10-12m) bins where per_bin_failure.csv showed the worst MAE.
    distance_bin_9 is derived from ground_truth_range_m purely to build
    this training-time sample weight -- it is never passed to the model as
    a feature (STABLE_FEATURES has no distance_bin field)."""
    group_w = unified_group_weights(rows)
    bin_counts: dict[str, int] = {}
    for row in rows:
        bin_counts[row["distance_bin_9"]] = bin_counts.get(row["distance_bin_9"], 0) + 1
    n_bins = len(bin_counts)
    target_per_bin = len(rows) / n_bins
    weights = np.empty(len(rows), dtype=np.float64)
    for i, row in enumerate(rows):
        bin_label = row["distance_bin_9"]
        bin_w = target_per_bin / bin_counts[bin_label]
        boost = far_boost_factor if bin_label in far_boost_bins else 1.0
        weights[i] = group_w[i] * bin_w * boost
    # renormalize so mean weight stays 1.0 (keeps overall loss scale comparable to Candidate A)
    weights *= len(weights) / weights.sum()
    return weights


# --------------------------------------------------------------------------
# Phase 4c: monotonic feature audit for Candidate C
# --------------------------------------------------------------------------

def monotonic_feature_audit(rows: list[dict[str, Any]]) -> list[dict[str, Any]]:
    """Spearman correlation of each STABLE_FEATURES field against GT
    distance, computed on the whole corpus (label-vs-feature relationship,
    not a fold-specific fit) -- used only to decide which features get a
    monotonic constraint sign, never to select training data."""
    from scipy.stats import spearmanr

    gt = np.asarray([float(r["ground_truth_range_m"]) for r in rows])
    results = []
    for name in STABLE_FEATURES:
        values = np.asarray([r["features"].get(name, np.nan) for r in rows], dtype=np.float64)
        finite = np.isfinite(values) & np.isfinite(gt)
        if finite.sum() < 10 or np.std(values[finite]) < 1e-9:
            results.append({"feature": name, "spearman_r": None, "n": int(finite.sum())})
            continue
        rho, _p = spearmanr(values[finite], gt[finite])
        results.append({"feature": name, "spearman_r": round(float(rho), 4), "n": int(finite.sum())})
    results.sort(key=lambda r: -abs(r["spearman_r"] or 0.0))
    return results


# --------------------------------------------------------------------------
# Phase 4/6: direct XGBoost training, group-disjoint 3-fold, OOF
# --------------------------------------------------------------------------

def train_direct(
    rows: list[dict[str, Any]], fold_map: dict[str, int], feature_names: tuple[str, ...],
    hyperparameters: dict[str, Any], weight_fn, monotone_constraints: tuple[int, ...] | None, seed: int = SEED,
) -> dict[str, Any]:
    n = len(rows)
    oof = np.full(n, np.nan, dtype=np.float64)
    boosters: dict[int, Any] = {}
    preprocessors: dict[int, VariantPreprocessor] = {}
    for fold in range(3):
        train_idx = [i for i, r in enumerate(rows) if fold_map[r["group_id"]] != fold]
        test_idx = [i for i, r in enumerate(rows) if fold_map[r["group_id"]] == fold]
        train_rows = [rows[i] for i in train_idx]
        test_rows = [rows[i] for i in test_idx]
        preprocessor = VariantPreprocessor.fit(train_rows, feature_names)
        train_x = preprocessor.transform(train_rows)
        test_x = preprocessor.transform(test_rows)
        train_y = np.asarray([float(r["ground_truth_range_m"]) for r in train_rows])
        weights = weight_fn(train_rows)
        params = {
            "objective": "reg:squarederror", "eval_metric": "mae", "tree_method": "hist",
            "max_depth": int(hyperparameters["max_depth"]), "eta": float(hyperparameters["learning_rate"]),
            "min_child_weight": float(hyperparameters["min_child_weight"]),
            "subsample": float(hyperparameters["subsample"]), "colsample_bytree": float(hyperparameters["colsample_bytree"]),
            "alpha": float(hyperparameters["reg_alpha"]), "lambda": float(hyperparameters["reg_lambda"]),
            "seed": seed, "nthread": 1,
        }
        if monotone_constraints is not None:
            params["monotone_constraints"] = "(" + ",".join(str(v) for v in monotone_constraints) + ")"
        booster = xgb.train(
            params, xgb.DMatrix(train_x, label=train_y, weight=weights),
            num_boost_round=int(hyperparameters["n_estimators"]), verbose_eval=False,
        )
        test_pred = np.clip(np.asarray(booster.predict(xgb.DMatrix(test_x)), dtype=np.float64), *CLIP_BOUNDS)
        oof[test_idx] = test_pred
        boosters[fold] = booster
        preprocessors[fold] = preprocessor
    if np.any(~np.isfinite(oof)):
        raise ValueError("oof_incomplete")
    return {"oof": oof, "boosters": boosters, "preprocessors": preprocessors}


# --------------------------------------------------------------------------
# Phase 7: calibration layer -- fit per fold on training predictions only
# --------------------------------------------------------------------------

def fold_train_predictions(rows: list[dict[str, Any]], fold_map: dict[str, int], trained: dict[str, Any]) -> dict[int, tuple[np.ndarray, np.ndarray]]:
    """Recompute each fold's booster's predictions on its OWN training
    rows (in-fold, not OOF) -- needed so calibration is fit only on
    training-fold data, per fold, never on the held-out test predictions."""
    out = {}
    for fold in range(3):
        train_idx = [i for i, r in enumerate(rows) if fold_map[r["group_id"]] != fold]
        train_rows = [rows[i] for i in train_idx]
        preprocessor = trained["preprocessors"][fold]
        booster = trained["boosters"][fold]
        x = preprocessor.transform(train_rows)
        pred = np.clip(np.asarray(booster.predict(xgb.DMatrix(x)), dtype=np.float64), *CLIP_BOUNDS)
        gt = np.asarray([float(r["ground_truth_range_m"]) for r in train_rows])
        out[fold] = (pred, gt)
    return out


def apply_affine_calibration(
    rows: list[dict[str, Any]], fold_map: dict[str, int], oof: np.ndarray, train_preds: dict[int, tuple[np.ndarray, np.ndarray]]
) -> tuple[np.ndarray, dict[int, dict[str, float]]]:
    calibrated = np.empty_like(oof)
    params: dict[int, dict[str, float]] = {}
    for fold in range(3):
        pred, gt = train_preds[fold]
        scale, bias = np.polyfit(pred, gt, 1)
        params[fold] = {"scale": float(scale), "bias": float(bias)}
        test_idx = [i for i, r in enumerate(rows) if fold_map[r["group_id"]] == fold]
        calibrated[test_idx] = np.clip(scale * oof[test_idx] + bias, *CLIP_BOUNDS)
    return calibrated, params


def apply_isotonic_calibration(
    rows: list[dict[str, Any]], fold_map: dict[str, int], oof: np.ndarray, train_preds: dict[int, tuple[np.ndarray, np.ndarray]]
) -> tuple[np.ndarray, dict[int, dict[str, Any]]]:
    calibrated = np.empty_like(oof)
    params: dict[int, dict[str, Any]] = {}
    for fold in range(3):
        pred, gt = train_preds[fold]
        iso = IsotonicRegression(y_min=CLIP_BOUNDS[0], y_max=CLIP_BOUNDS[1], out_of_bounds="clip")
        iso.fit(pred, gt)
        params[fold] = {
            "x_knots": [round(float(v), 6) for v in iso.X_thresholds_.tolist()],
            "y_knots": [round(float(v), 6) for v in iso.y_thresholds_.tolist()],
        }
        test_idx = [i for i, r in enumerate(rows) if fold_map[r["group_id"]] == fold]
        calibrated[test_idx] = np.clip(iso.predict(oof[test_idx]), *CLIP_BOUNDS)
    return calibrated, params


if __name__ == "__main__":
    print("This module provides shared functions for the phase3-10 driver scripts.")
