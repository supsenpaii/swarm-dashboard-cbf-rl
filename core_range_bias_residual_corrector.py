"""Giai đoạn 1.2 -- train up to 3 residual bias-corrector configs on top of
the frozen, unchanged Candidate B backbone.

corrected_range_m = clip(candidate_b_pred_m + residual_corrector(features), 3, 12)

Group-disjoint 3-fold CV (same fold_map/seed=52 reused unchanged). No
session/scenario/geometry-context-categorical/GT-bin/future-frame input.
Writes, per config, a prediction_rows.csv-schema-compatible CSV so the
existing control-ready offline replay/metrics pipeline can be reused
unmodified in Phase 1.3.
"""

from __future__ import annotations

import csv
import json
from pathlib import Path

import numpy as np
import xgboost as xgb
from sklearn.linear_model import Ridge

import core_range_stable_relative_tracking as srt
from core_range_dynamic_robust_retrain import CLIP_BOUNDS, VariantPreprocessor, unified_group_weights

WORKSPACE = Path(__file__).resolve().parent
SRC = WORKSPACE / "artifacts/core_range_3_12m/stable_relative_tracking"
OUT = WORKSPACE / "artifacts/core_range_3_12m/residual_bias_correction"
CANDIDATE = "CANDIDATE_B_GROUP_AND_DISTANCE_BALANCED"
SEED = 52

STABLE_SUBSET_FEATURES = ("ray_scale", "target_inverse_depth", "target_roi_q_min", "target_roi_q_p10")

PRECOMMIT = json.loads((OUT / "residual_corrector_precommit.json").read_text())


def load_joined_rows() -> tuple[list[dict], dict[str, int]]:
    combined_rows, fold_map = srt.load_corpus()
    by_key = {(r["group_id"], round(float(r["measurement_timestamp_s"]), 6)): r for r in combined_rows}
    with (SRC / "prediction_rows.csv").open() as f:
        pred_rows = [r for r in csv.DictReader(f) if r["candidate"] == CANDIDATE]
    joined = []
    for r in pred_rows:
        key = (r["group_id"], round(float(r["measurement_timestamp_s"]), 6))
        crow = by_key[key]
        row = dict(crow)
        row["candidate_b_pred_m"] = float(r["pred_none_m"])
        joined.append(row)
    assert len(joined) == len(combined_rows) == len(pred_rows)
    return joined, fold_map


def build_matrix(rows: list[dict], feature_names: tuple[str, ...], medians: dict[str, float] | None = None) -> tuple[np.ndarray, dict[str, float]]:
    raw = np.asarray(
        [[float(r["features"].get(name)) if r["features"].get(name) is not None else np.nan for name in feature_names]
         + [r["candidate_b_pred_m"]] for r in rows],
        dtype=np.float64,
    )
    all_names = list(feature_names) + ["candidate_b_pred_m"]
    if medians is None:
        medians = {n: float(np.nanmedian(raw[:, i])) for i, n in enumerate(all_names)}
    missing = np.isnan(raw)
    filled = raw.copy()
    for i, n in enumerate(all_names):
        filled[np.isnan(filled[:, i]), i] = medians[n]
    result = np.hstack([filled, missing.astype(np.float64)])
    return result, medians


def train_config_ridge(rows: list[dict], fold_map: dict[str, int], feature_names: tuple[str, ...]) -> np.ndarray:
    n = len(rows)
    oof = np.full(n, np.nan, dtype=np.float64)
    for fold in range(3):
        train_idx = [i for i, r in enumerate(rows) if fold_map[r["group_id"]] != fold]
        test_idx = [i for i, r in enumerate(rows) if fold_map[r["group_id"]] == fold]
        train_rows = [rows[i] for i in train_idx]
        test_rows = [rows[i] for i in test_idx]
        train_x, medians = build_matrix(train_rows, feature_names)
        test_x, _ = build_matrix(test_rows, feature_names, medians=medians)
        mean = train_x.mean(axis=0)
        std = train_x.std(axis=0)
        std[std < 1e-9] = 1.0
        train_x_std = (train_x - mean) / std
        test_x_std = (test_x - mean) / std
        train_y = np.asarray([r["ground_truth_range_m"] - r["candidate_b_pred_m"] for r in train_rows])
        weights = unified_group_weights(train_rows)
        model = Ridge(alpha=1.0)
        model.fit(train_x_std, train_y, sample_weight=weights)
        oof[test_idx] = model.predict(test_x_std)
    return oof


def train_config_xgb(rows: list[dict], fold_map: dict[str, int], feature_names: tuple[str, ...], hyperparameters: dict, seed: int = SEED) -> np.ndarray:
    n = len(rows)
    oof = np.full(n, np.nan, dtype=np.float64)
    for fold in range(3):
        train_idx = [i for i, r in enumerate(rows) if fold_map[r["group_id"]] != fold]
        test_idx = [i for i, r in enumerate(rows) if fold_map[r["group_id"]] == fold]
        train_rows = [rows[i] for i in train_idx]
        test_rows = [rows[i] for i in test_idx]
        train_x, medians = build_matrix(train_rows, feature_names)
        test_x, _ = build_matrix(test_rows, feature_names, medians=medians)
        train_y = np.asarray([r["ground_truth_range_m"] - r["candidate_b_pred_m"] for r in train_rows])
        weights = unified_group_weights(train_rows)
        params = {
            "objective": "reg:squarederror", "eval_metric": "mae", "tree_method": "hist",
            "max_depth": int(hyperparameters["max_depth"]), "eta": float(hyperparameters["learning_rate"]),
            "min_child_weight": float(hyperparameters["min_child_weight"]),
            "subsample": float(hyperparameters["subsample"]), "colsample_bytree": float(hyperparameters["colsample_bytree"]),
            "alpha": float(hyperparameters["reg_alpha"]), "lambda": float(hyperparameters["reg_lambda"]),
            "seed": seed, "nthread": 1,
        }
        booster = xgb.train(params, xgb.DMatrix(train_x, label=train_y, weight=weights),
                             num_boost_round=int(hyperparameters["n_estimators"]), verbose_eval=False)
        oof[test_idx] = booster.predict(xgb.DMatrix(test_x))
    return oof


def write_csv(path: Path, rows: list[dict]) -> None:
    if not rows:
        path.write_text("")
        return
    keys: list[str] = []
    for row in rows:
        for k in row:
            if k not in keys:
                keys.append(k)
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=keys)
        writer.writeheader()
        writer.writerows(rows)


def main() -> None:
    rows, fold_map = load_joined_rows()
    metrics_rows = []
    all_prediction_rows: dict[str, list[dict]] = {}

    for config_name, oof_residual in (
        ("A_RIDGE_BASELINE", train_config_ridge(rows, fold_map, srt.STABLE_FEATURES)),
        ("B_SHALLOW_XGBOOST_STRONG_REG", train_config_xgb(rows, fold_map, srt.STABLE_FEATURES, {
            "n_estimators": 100, "max_depth": 2, "learning_rate": 0.03, "min_child_weight": 5.0,
            "subsample": 0.8, "colsample_bytree": 0.8, "reg_alpha": 0.5, "reg_lambda": 10.0,
        })),
        ("C_SHALLOW_XGBOOST_STABLE_FEATURE_SUBSET", train_config_xgb(rows, fold_map, STABLE_SUBSET_FEATURES, {
            "n_estimators": 150, "max_depth": 2, "learning_rate": 0.03, "min_child_weight": 5.0,
            "subsample": 0.8, "colsample_bytree": 0.8, "reg_alpha": 0.2, "reg_lambda": 5.0,
        })),
    ):
        if np.any(~np.isfinite(oof_residual)):
            raise SystemExit(f"RESIDUAL_CORRECTOR_TRAIN_FAILED:{config_name}:oof_incomplete")

        corrected = np.clip(
            np.asarray([r["candidate_b_pred_m"] for r in rows]) + oof_residual, *CLIP_BOUNDS
        )
        gt = np.asarray([r["ground_truth_range_m"] for r in rows])
        residual_after = gt - corrected
        collapsed = float(np.std(corrected)) < 0.05
        metrics_rows.append({
            "config": config_name, "collapsed": collapsed,
            "mae_m": round(float(np.mean(np.abs(residual_after))), 4),
            "bias_m": round(float(np.mean(residual_after)), 4),
            "std_m": round(float(np.std(residual_after)), 4),
            "raw_candidate_b_mae_m": round(float(np.mean(np.abs(gt - np.asarray([r['candidate_b_pred_m'] for r in rows])))), 4),
        })

        pred_rows = []
        for i, r in enumerate(rows):
            pred_rows.append({
                "candidate": f"CANDIDATE_B_PLUS_RESIDUAL_{config_name}", "group_id": r["group_id"],
                "domain": r["domain"], "scenario_type": r["scenario_type"], "fold": fold_map[r["group_id"]],
                "measurement_timestamp_s": r["measurement_timestamp_s"],
                "ground_truth_range_m": r["ground_truth_range_m"], "raw_physical_range_m": r["raw_physical_range_m"],
                "pred_none_m": float(corrected[i]), "pred_affine_m": float(corrected[i]), "pred_isotonic_m": float(corrected[i]),
            })
        all_prediction_rows[config_name] = pred_rows
        write_csv(OUT / f"corrected_prediction_rows_{config_name}.csv", pred_rows)

    write_csv(OUT / "raw_corrector_metrics.csv", metrics_rows)
    print(json.dumps(metrics_rows, indent=2))


if __name__ == "__main__":
    main()
