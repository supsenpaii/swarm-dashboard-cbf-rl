"""Giai đoạn 1.1 -- bias audit for Candidate B.

Maps residual e = GT - Candidate B raw prediction against distance bin,
runtime geometry/depth/calibration features, and a precommitted
center/lateral/yaw_oblique geometry-context label derived from group naming
(disclosed heuristic, not tuned after seeing results), for both static and
dynamic domains. Purely descriptive/diagnostic -- no model fit here.
"""

from __future__ import annotations

import csv
import json
from pathlib import Path
from statistics import mean, pstdev

import numpy as np
from scipy.stats import spearmanr

import core_range_stable_relative_tracking as srt

WORKSPACE = Path(__file__).resolve().parent
SRC = WORKSPACE / "artifacts/core_range_3_12m/stable_relative_tracking"
OUT = WORKSPACE / "artifacts/core_range_3_12m/residual_bias_correction"
OUT.mkdir(parents=True, exist_ok=True)
CANDIDATE = "CANDIDATE_B_GROUP_AND_DISTANCE_BALANCED"

AUDIT_FEATURES = (
    "image_ray_x", "image_ray_y", "ray_scale", "target_inverse_depth",
    "target_roi_q_min", "target_roi_q_p10", "target_roi_q_p25", "target_roi_q_median",
    "target_roi_q_p75", "target_roi_q_p90", "target_roi_q_max", "target_roi_q_std", "target_roi_q_iqr",
    "calibration_a", "calibration_b", "calibration_fit_residual", "calibration_condition",
)


def geometry_context(group_id: str) -> str:
    """Precommitted, disclosed heuristic from group naming (not tuned after
    seeing bias results): 'yaw'/'oblique' substrings -> yaw_oblique;
    'lateral'/'left'/'right' -> lateral; else -> center."""
    gid = group_id.lower()
    if "yaw" in gid or "oblique" in gid:
        return "yaw_oblique"
    if "lateral" in gid or "left" in gid or "right" in gid:
        return "lateral"
    return "center"


def main() -> None:
    combined_rows, _fold_map = srt.load_corpus()
    by_key = {(r["group_id"], round(float(r["measurement_timestamp_s"]), 6)): r for r in combined_rows}

    with (SRC / "prediction_rows.csv").open() as f:
        pred_rows = [r for r in csv.DictReader(f) if r["candidate"] == CANDIDATE]

    joined = []
    for r in pred_rows:
        key = (r["group_id"], round(float(r["measurement_timestamp_s"]), 6))
        crow = by_key.get(key)
        if crow is None:
            continue
        gt = float(r["ground_truth_range_m"])
        pred = float(r["pred_none_m"])
        joined.append({
            "group_id": r["group_id"], "domain": r["domain"], "scenario_type": r["scenario_type"],
            "geometry_context": geometry_context(r["group_id"]),
            "distance_bin": srt.bin_of(gt),
            "ground_truth_range_m": gt, "pred_none_m": pred,
            "residual_m": gt - pred, "abs_residual_m": abs(gt - pred),
            "features": crow["features"],
        })
    assert len(joined) == len(pred_rows), f"join_incomplete:{len(joined)}:{len(pred_rows)}"

    # --- bias map: distance_bin x geometry_context x domain ---
    bias_map = {}
    for row in joined:
        key = (row["domain"], row["geometry_context"], row["distance_bin"])
        bias_map.setdefault(key, []).append(row["residual_m"])
    bias_map_rows = []
    for (domain, geom, bin_label), residuals in sorted(bias_map.items()):
        bias_map_rows.append({
            "domain": domain, "geometry_context": geom, "distance_bin": bin_label,
            "n": len(residuals), "mean_bias_m": round(mean(residuals), 4),
            "mean_abs_bias_m": round(mean(abs(v) for v in residuals), 4),
            "std_m": round(pstdev(residuals), 4) if len(residuals) > 1 else 0.0,
            "persistent_bias": abs(mean(residuals)) >= 1.0,  # >=1m sustained mean bias in this cell
        })
    write_csv(OUT / "bias_map.csv", bias_map_rows)

    # --- per-group bias summary (to catch single-group-dominated cells) ---
    by_group = {}
    for row in joined:
        by_group.setdefault(row["group_id"], []).append(row)
    group_rows = []
    for group_id, grows in sorted(by_group.items()):
        residuals = [r["residual_m"] for r in grows]
        group_rows.append({
            "group_id": group_id, "domain": grows[0]["domain"], "scenario_type": grows[0]["scenario_type"],
            "geometry_context": grows[0]["geometry_context"], "n": len(grows),
            "mean_bias_m": round(mean(residuals), 4), "mean_abs_bias_m": round(mean(abs(v) for v in residuals), 4),
            "std_m": round(pstdev(residuals), 4) if len(residuals) > 1 else 0.0,
        })
    write_csv(OUT / "per_group_bias.csv", group_rows)

    # --- feature-vs-residual correlation, overall and per geometry_context ---
    corr_rows = []
    gt_all = np.asarray([r["ground_truth_range_m"] for r in joined])
    resid_all = np.asarray([r["residual_m"] for r in joined])
    abs_resid_all = np.abs(resid_all)
    for feat in AUDIT_FEATURES:
        vals = np.asarray([r["features"].get(feat, np.nan) for r in joined], dtype=np.float64)
        finite = np.isfinite(vals) & np.isfinite(resid_all)
        if finite.sum() < 10 or np.std(vals[finite]) < 1e-9:
            corr_rows.append({"feature": feat, "context": "all", "n": int(finite.sum()),
                               "spearman_vs_signed_residual": None, "spearman_vs_abs_residual": None})
            continue
        rho_signed, _ = spearmanr(vals[finite], resid_all[finite])
        rho_abs, _ = spearmanr(vals[finite], abs_resid_all[finite])
        corr_rows.append({"feature": feat, "context": "all", "n": int(finite.sum()),
                           "spearman_vs_signed_residual": round(float(rho_signed), 4),
                           "spearman_vs_abs_residual": round(float(rho_abs), 4)})
        for geom in ("center", "lateral", "yaw_oblique"):
            idx = [i for i, r in enumerate(joined) if r["geometry_context"] == geom]
            gvals = vals[idx]
            gresid = resid_all[idx]
            gfinite = np.isfinite(gvals) & np.isfinite(gresid)
            if gfinite.sum() < 10 or np.std(gvals[gfinite]) < 1e-9:
                corr_rows.append({"feature": feat, "context": geom, "n": int(gfinite.sum()),
                                   "spearman_vs_signed_residual": None, "spearman_vs_abs_residual": None})
                continue
            rho_g, _ = spearmanr(gvals[gfinite], gresid[gfinite])
            rho_ga, _ = spearmanr(gvals[gfinite], np.abs(gresid[gfinite]))
            corr_rows.append({"feature": feat, "context": geom, "n": int(gfinite.sum()),
                               "spearman_vs_signed_residual": round(float(rho_g), 4),
                               "spearman_vs_abs_residual": round(float(rho_ga), 4)})
    write_csv(OUT / "feature_bias_correlation.csv", corr_rows)

    # --- geometry-context definition, disclosed ---
    context_counts = {}
    for row in joined:
        key = (row["domain"], row["geometry_context"])
        context_counts[key] = context_counts.get(key, 0) + 1
    group_by_context = {}
    for group_id, grows in by_group.items():
        group_by_context.setdefault(grows[0]["geometry_context"], []).append(group_id)

    conclusion = {
        "method": "substring heuristic on group_id: 'yaw'/'oblique' -> yaw_oblique; 'lateral'/'left'/'right' -> lateral; else -> center. Precommitted before computing any bias numbers.",
        "context_frame_counts": {f"{d}/{g}": n for (d, g), n in sorted(context_counts.items())},
        "groups_by_context": {k: sorted(v) for k, v in group_by_context.items()},
        "worst_cells_by_mean_abs_bias": sorted(bias_map_rows, key=lambda r: -r["mean_abs_bias_m"])[:8],
        "persistent_bias_cell_count": sum(1 for r in bias_map_rows if r["persistent_bias"]),
        "note_small_group_count_per_context": "dynamic corpus has only 8 groups total; some (domain, geometry_context) cells are dominated by a single group (e.g. dynamic/yaw_oblique = only cdr_approach_right_yaw + cdr_recede_left_yaw). Any residual-corrector feature highly correlated with geometry_context risks learning group identity rather than a general geometric relationship -- flagged explicitly for the Phase 1.2 leakage review.",
    }
    (OUT / "geometry_context_definition.json").write_text(json.dumps(conclusion, indent=2) + "\n")

    print(json.dumps({
        "persistent_bias_cells": conclusion["persistent_bias_cell_count"],
        "worst_cells": [f"{r['domain']}/{r['geometry_context']}/{r['distance_bin']}: {r['mean_bias_m']}m (n={r['n']})"
                         for r in conclusion["worst_cells_by_mean_abs_bias"][:5]],
    }, indent=2))


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


if __name__ == "__main__":
    main()
