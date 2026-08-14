"""Train a ranking-objective model to fix inverted within-scenario trends.

The MSE-trained regressor (core_range_stage2a_relative_range_model.py) reaches
MAE 0.69-0.88 m out-of-fold, but three of twelve scenario groups have a
*negative* within-group slope: the model reports range decreasing while the
target is actually receding, or vice versa. Diagnosis found no clean feature
confound behind this (see chat) -- with 12 groups and highly correlated
features, MSE regression on so little group diversity is free to fit
whichever direction reduces average squared error, including the wrong sign
in a minority of groups.

The user's actual requirement is explicitly relative, not absolute: "chỉ cần
tăng giảm ổn định trong khoảng đó" (just needs to move up/down consistently
in range). That is exactly what a *ranking* objective optimises, and MSE
does not. XGBoost's `rank:pairwise` objective, with each scenario group as a
query group, directly penalises any pair of frames within the same group
whose predicted order disagrees with the true order -- unlike MSE, it cannot
"average away" a wrong-direction group by being close on every other group.

Ranking scores are not metres; a single global affine fit of true distance
on out-of-fold ranking scores turns the score into an approximate metres
value, exactly as before -- this fit does not change ordering within a
group, so it cannot re-introduce the sign problem the ranking objective
fixed.

At inference this is still a plain per-frame prediction (score, then one
affine transform) -- no access to other frames in the scenario is needed,
so it drops into stage2a_range_model.py's predict_range_m() unchanged.
"""

from __future__ import annotations

import json

import numpy as np
import xgboost as xgb
from sklearn.model_selection import GroupKFold

from core_range_stage2a_relative_range_model import (
    BBOX_FEATURES,
    CORPUS,
    DISTANCE_BOUNDS,
    OUT,
    SEED,
    SIZE_AGNOSTIC_FEATURES,
    build_matrix,
    extract_rows,
)

FEATURES = SIZE_AGNOSTIC_FEATURES + BBOX_FEATURES


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


def per_group_spearman(pred: np.ndarray, truth: np.ndarray, groups: np.ndarray) -> dict:
    def rank(v: np.ndarray) -> np.ndarray:
        order = np.argsort(v)
        r = np.empty(len(v))
        r[order] = np.arange(len(v))
        return r

    result = {}
    for gid in sorted(set(groups)):
        mask = groups == gid
        p, t = rank(pred[mask]), rank(truth[mask])
        p, t = p - p.mean(), t - t.mean()
        denom = np.sqrt((p ** 2).sum() * (t ** 2).sum())
        result[gid] = float((p * t).sum() / denom) if denom > 1e-9 else float("nan")
    return result


def out_of_fold_rank_scores(
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
        train_groups = groups[train_idx]
        # XGBRanker needs contiguous group blocks; sort by group within fold.
        order = np.argsort(train_groups, kind="stable")
        group_sizes = np.unique(train_groups[order], return_counts=True)[1]
        ranker = xgb.XGBRanker(
            objective="rank:pairwise",
            n_estimators=200, max_depth=4, learning_rate=0.05,
            subsample=0.8, colsample_bytree=0.8, reg_lambda=2.0,
            random_state=SEED,
        )
        ranker.fit(train_x[order], y[train_idx][order], group=group_sizes)
        oof[test_idx] = ranker.predict(test_x)
    return oof, y, groups


def fit_score_to_metres(score: np.ndarray, truth: np.ndarray) -> tuple[float, float]:
    mean_s, mean_t = float(np.mean(score)), float(np.mean(truth))
    variance = float(np.sum((score - mean_s) ** 2))
    if variance <= 1e-12:
        return 0.0, mean_t
    scale = float(np.sum((score - mean_s) * (truth - mean_t))) / variance
    return scale, mean_t - scale * mean_s


def main() -> None:
    rows = extract_rows(CORPUS)
    rank_oof, truth, groups = out_of_fold_rank_scores(rows, FEATURES)

    slopes = per_group_slope(rank_oof, truth, groups)
    spearmans = per_group_spearman(rank_oof, truth, groups)

    print("=== Ranking model (rank:pairwise), out-of-fold per group ===\n")
    print(f"{'group':38s} {'slope(score)':>12s} {'spearman':>10s}")
    print("-" * 62)
    n_negative = 0
    for gid in sorted(slopes):
        s, r = slopes[gid], spearmans[gid]
        flag = "  <<< NGƯỢC CHIỀU" if s < 0 else ""
        if s < 0:
            n_negative += 1
        print(f"{gid:38s} {s:12.3f} {r:10.3f}{flag}")

    scale, offset = fit_score_to_metres(rank_oof, truth)
    calibrated = scale * rank_oof + offset
    clipped = np.clip(calibrated, *DISTANCE_BOUNDS)
    err = clipped - truth
    metre_slopes = per_group_slope(clipped, truth, groups)

    print(f"\nSố nhóm slope ÂM (đi ngược chiều): {n_negative}/12")
    print(f"Median Spearman trong-nhóm: {np.median(list(spearmans.values())):.3f}")
    print(f"\nSau hiệu chuẩn tuyến tính score -> mét "
          f"(corrected = {scale:.4f}*score + {offset:+.4f}):")
    print(f"  MAE={np.mean(np.abs(err)):.3f}m  RMSE={np.sqrt(np.mean(err**2)):.3f}m")
    print(f"  median within-group slope (mét) = {np.median(list(metre_slopes.values())):.3f}")

    # Compare against the MSE regression model's known numbers for an
    # apples-to-apples verdict, not just this model's own metrics.
    print("\n=== So với model hồi quy MSE (with_bbox) đã có ===")
    print("  MSE regression : MAE=0.691m  median slope=0.275  3/12 nhóm slope âm")
    print(f"  Ranking model  : MAE={np.mean(np.abs(err)):.3f}m  "
          f"median slope={np.median(list(metre_slopes.values())):.3f}  "
          f"{n_negative}/12 nhóm slope âm")

    OUT.mkdir(parents=True, exist_ok=True)
    (OUT / "rank_model_evaluation.json").write_text(
        json.dumps({
            "feature_names": list(FEATURES),
            "score_slope_per_group": slopes,
            "spearman_per_group": spearmans,
            "n_groups_negative_slope": n_negative,
            "score_to_metres_scale": scale,
            "score_to_metres_offset": offset,
            "metres_mae": float(np.mean(np.abs(err))),
            "metres_rmse": float(np.sqrt(np.mean(err ** 2))),
            "metres_median_within_group_slope": float(np.median(list(metre_slopes.values()))),
        }, indent=2),
        encoding="utf-8",
    )

    # Final production ranker + calibration, trained on all rows.
    x, medians = build_matrix(rows, FEATURES)
    y = np.asarray([r["distance_m"] for r in rows], dtype=np.float64)
    order = np.argsort(groups, kind="stable")
    group_sizes = np.unique(groups[order], return_counts=True)[1]
    final_ranker = xgb.XGBRanker(
        objective="rank:pairwise",
        n_estimators=200, max_depth=4, learning_rate=0.05,
        subsample=0.8, colsample_bytree=0.8, reg_lambda=2.0,
        random_state=SEED,
    )
    final_ranker.fit(x[order], y[order], group=group_sizes)
    final_ranker.save_model(str(OUT / "rank_model.xgb.json"))
    (OUT / "rank_model.medians.json").write_text(
        json.dumps({
            "feature_names": list(FEATURES), "medians": medians,
            "score_to_metres_scale": scale, "score_to_metres_offset": offset,
        }, indent=2),
        encoding="utf-8",
    )
    print(f"\nSaved rank_model.xgb.json, rank_model.medians.json, "
          f"rank_model_evaluation.json to {OUT}")


if __name__ == "__main__":
    main()
