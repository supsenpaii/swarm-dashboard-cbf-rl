"""Fit and evaluate a variance-restoring calibration for the Stage 2A model.

The trained XGBoost regressor shrinks toward the corpus mean -- measured
slope of predicted-vs-true range was 0.24-0.39 on three of four live
validation captures, meaning a target that actually receded 5.5 m was
reported as receding 2.3 m. That is MSE-optimal behaviour for a model
fitted on twelve scenario groups, but it under-drives any controller that
acts on distance error.

The fix is an affine calibration `corrected = a * prediction + b` whose
coefficients come from regressing true range on **out-of-fold** predictions,
so the calibration is fitted on predictions the model never trained against.
Regressing truth on prediction (not the reverse) is what restores unit
slope.

This trades accuracy for responsiveness: expanding a shrunk prediction
necessarily raises MSE, because shrinkage was minimising it. Both metrics
are reported so the trade can be judged rather than assumed, and a
`blend` sweep shows intermediate options between raw and fully expanded.
"""

from __future__ import annotations

import json

import numpy as np
import xgboost as xgb
from sklearn.model_selection import GroupKFold

from core_range_stage2a_relative_range_model import (
    CORPUS,
    DISTANCE_BOUNDS,
    OUT,
    SEED,
    SIZE_AGNOSTIC_FEATURES,
    build_matrix,
    extract_rows,
    lateral_offset_weights,
)


def out_of_fold_predictions(
    rows: list[dict], feature_names: tuple[str, ...]
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    groups = np.asarray([r["group_id"] for r in rows])
    y = np.asarray([r["distance_m"] for r in rows], dtype=np.float64)
    oof = np.full(len(rows), np.nan, dtype=np.float64)
    splitter = GroupKFold(n_splits=min(4, len(set(groups))))
    for train_idx, test_idx in splitter.split(rows, y, groups):
        train_rows = [rows[i] for i in train_idx]
        test_rows = [rows[i] for i in test_idx]
        train_x, medians = build_matrix(train_rows, feature_names)
        test_x, _ = build_matrix(test_rows, feature_names, medians=medians)
        booster = xgb.XGBRegressor(
            n_estimators=200, max_depth=4, learning_rate=0.05,
            subsample=0.8, colsample_bytree=0.8, reg_lambda=2.0,
            random_state=SEED,
        )
        booster.fit(train_x, y[train_idx], sample_weight=lateral_offset_weights(train_rows))
        oof[test_idx] = booster.predict(test_x)
    return oof, y, groups


def fit_calibration(prediction: np.ndarray, truth: np.ndarray) -> tuple[float, float]:
    """Regress truth on prediction; the resulting line has unit slope by construction."""
    mean_p, mean_t = float(np.mean(prediction)), float(np.mean(truth))
    variance = float(np.sum((prediction - mean_p) ** 2))
    if variance <= 1e-12:
        return 1.0, 0.0
    scale = float(np.sum((prediction - mean_p) * (truth - mean_t))) / variance
    return scale, mean_t - scale * mean_p


def per_group_slope(pred: np.ndarray, truth: np.ndarray, groups: np.ndarray) -> dict:
    result = {}
    for gid in sorted(set(groups)):
        mask = groups == gid
        p, t = pred[mask], truth[mask]
        variance = float(np.sum((t - t.mean()) ** 2))
        if variance <= 1e-12:
            continue
        result[gid] = float(np.sum((t - t.mean()) * (p - p.mean())) / variance)
    return result


def metrics(pred: np.ndarray, truth: np.ndarray, groups: np.ndarray) -> dict:
    clipped = np.clip(pred, *DISTANCE_BOUNDS)
    err = clipped - truth
    slopes = per_group_slope(clipped, truth, groups)
    return {
        "mae_m": float(np.mean(np.abs(err))),
        "rmse_m": float(np.sqrt(np.mean(err ** 2))),
        "bias_m": float(np.median(err)),
        "median_within_group_slope": float(np.median(list(slopes.values()))),
        "per_group_slope": slopes,
    }


def main() -> None:
    rows = extract_rows(CORPUS)
    oof, truth, groups = out_of_fold_predictions(rows, SIZE_AGNOSTIC_FEATURES)
    scale, offset = fit_calibration(oof, truth)

    print(f"Out-of-fold calibration: corrected = {scale:.4f} * prediction + {offset:+.4f}")
    print(f"(scale > 1 expands the shrunk prediction back toward true spread)\n")

    print(f"{'blend':>6s} {'MAE':>8s} {'RMSE':>8s} {'bias':>8s} {'slope':>8s}")
    print("-" * 44)
    best = None
    sweep = {}
    for blend in (0.0, 0.25, 0.5, 0.75, 1.0):
        effective_scale = 1.0 + blend * (scale - 1.0)
        effective_offset = blend * offset + (1.0 - blend) * 0.0
        # Keep the blended line passing through the same fixed point as the
        # full calibration so partial blends stay unbiased, not just rescaled.
        adjusted = effective_scale * oof + (
            blend * offset + (1.0 - blend) * (1.0 - effective_scale) * float(np.mean(oof))
        )
        m = metrics(adjusted, truth, groups)
        sweep[str(blend)] = {k: v for k, v in m.items() if k != "per_group_slope"}
        marker = ""
        if best is None or abs(m["median_within_group_slope"] - 1.0) < abs(best[1] - 1.0):
            best = (blend, m["median_within_group_slope"])
        print(f"{blend:6.2f} {m['mae_m']:8.3f} {m['rmse_m']:8.3f} "
              f"{m['bias_m']:+8.3f} {m['median_within_group_slope']:8.3f}{marker}")

    raw = metrics(oof, truth, groups)
    full = metrics(scale * oof + offset, truth, groups)
    print("\nPer-group slope, raw -> fully calibrated:")
    for gid in sorted(raw["per_group_slope"]):
        print(f"  {gid:38s} {raw['per_group_slope'][gid]:6.3f} -> "
              f"{full['per_group_slope'][gid]:6.3f}")

    OUT.mkdir(parents=True, exist_ok=True)
    (OUT / "slope_calibration.json").write_text(
        json.dumps({
            "scale": scale, "offset": offset,
            "fitted_on": "out_of_fold_predictions",
            "n_rows": len(rows), "n_groups": len(set(groups)),
            "raw": {k: v for k, v in raw.items() if k != "per_group_slope"},
            "fully_calibrated": {k: v for k, v in full.items() if k != "per_group_slope"},
            "blend_sweep": sweep,
            "raw_per_group_slope": raw["per_group_slope"],
            "calibrated_per_group_slope": full["per_group_slope"],
        }, indent=2),
        encoding="utf-8",
    )
    print(f"\nSaved {OUT / 'slope_calibration.json'}")


if __name__ == "__main__":
    main()
