"""CORE_RANGE_OFFLINE_FAILURE_ANALYSIS_AND_TARGETED_RETRAIN, Phase 2-4.

Pure offline analysis over the already-frozen, already-verified-reproducible
8/8 dynamic + 27-group static corpus. Does not touch Gazebo/PX4/backend, does
not modify the frozen dataset, does not retrain anything the baseline
reproduction didn't already retrain. Reuses core_range_dynamic_robust_retrain
and core_range_5hz_headless_retrain's functions unmodified.
"""

from __future__ import annotations

import csv
import json
import math
from pathlib import Path
from statistics import mean, median, pstdev
from typing import Any

import numpy as np
import xgboost as xgb

from core_range_5hz_headless_retrain import load_combined_rows
from core_range_dynamic_robust_retrain import (
    BBOX_AUGMENT_VARIANTS,
    BBOX_RELATED_FEATURES,
    CLIP_BOUNDS,
    FEATURE_VARIANTS,
    HYPERPARAMETERS,
    PHYSICAL_ONLY_FEATURES,
    PHYSICAL_TEMPORAL_FEATURES,
    TEMPORAL_EXTRA_FEATURES,
    add_temporal_features,
    assign_combined_folds,
    equal_group_aggregate,
    per_group_rows,
    train_variant,
)
from core_range_direct_dynamic_replay import (
    DIRECTION_DEADBAND_M_S,
    _direction_row,
    _stop_row,
    alignment_lag_s,
    perturb_bbox_features,
)
from core_range_xgboost_benchmark import BINS, FEATURE_NAMES as FULL_FEATURE_NAMES

WORKSPACE = Path(__file__).resolve().parent
DYNAMIC_OUTPUT = WORKSPACE / "artifacts/core_range_3_12m/overnight_sim_time_train/frozen_corpus"
OUT = WORKSPACE / "artifacts/core_range_3_12m/offline_targeted_retrain"
VARIANTS = ("PHYSICAL_ONLY", "PHYSICAL_TEMPORAL")


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


def load_everything() -> tuple[list[dict[str, Any]], dict[str, int], dict[str, dict[str, Any]]]:
    combined_rows, _manifest = load_combined_rows(WORKSPACE, DYNAMIC_OUTPUT)
    add_temporal_features(combined_rows)
    fold_map = assign_combined_folds(WORKSPACE, combined_rows, seed=52)

    # Cross-check against the already-verified baseline fold_assignments.csv
    baseline_folds = {}
    with (OUT / "baseline_repro" / "fold_assignments.csv").open() as stream:
        for row in csv.DictReader(stream):
            baseline_folds[row["group_id"]] = int(row["fold"])
    assert fold_map == baseline_folds, "fold_map mismatch vs verified baseline"

    trained: dict[str, dict[str, Any]] = {}
    for variant_name in VARIANTS:
        trained[variant_name] = train_variant(
            xgb, variant_name, FEATURE_VARIANTS[variant_name], combined_rows, fold_map
        )
    return combined_rows, fold_map, trained


DISTANCE_REGIMES = {"near": (3.0, 6.0), "core": (6.0, 9.0), "far": (9.0, 12.001)}

# Numeric runtime-quality covariates to correlate residuals against.
# calibration_age doubles as the "measurement age" dimension the task asks
# for -- no separate age-since-calibration field exists in the feature
# contract beyond this one.
COVARIATES = [
    "raw_physical_range_m", "target_inverse_depth",
    "calibration_a", "calibration_b", "calibration_fit_residual",
    "calibration_condition", "calibration_inlier_count", "calibration_inlier_fraction",
    "calibration_q_span", "calibration_depth_span", "calibration_age",
    "target_roi_q_std", "target_roi_q_iqr", "target_roi_q_median",
    "image_ray_x", "image_ray_y", "ray_scale",
    "anchor_coverage", "anchor_q_mad", "anchor_depth_span",
    "target_valid_fraction",
    "delta_time_s", "causal_raw_range_rate_m_s", "causal_inverse_depth_rate",
    "validity_streak",
]


def regime_of(distance: float) -> str:
    for name, (lo, hi) in DISTANCE_REGIMES.items():
        if lo <= distance < hi:
            return name
    return "out_of_range"


def build_failure_matrix(
    combined_rows: list[dict[str, Any]], trained: dict[str, dict[str, Any]]
) -> list[dict[str, Any]]:
    matrix = []
    for i, row in enumerate(combined_rows):
        entry = {
            "row_index": i, "group_id": row["group_id"], "domain": row["domain"],
            "scenario_type": row["scenario_type"], "context": row.get("context"),
            "distance_bin": row.get("distance_bin"),
            "distance_regime": regime_of(float(row["ground_truth_range_m"])),
            "measurement_timestamp_s": row["measurement_timestamp_s"],
            "ground_truth_range_m": row["ground_truth_range_m"],
            "raw_physical_range_m": row["raw_physical_range_m"],
            "raw_physical_residual_m": row["raw_physical_range_m"] - row["ground_truth_range_m"],
        }
        for name in COVARIATES:
            entry[name] = row["features"].get(name)
        for variant_name, trained_variant in trained.items():
            for config_name, oof in trained_variant["oof"].items():
                pred = float(oof[i])
                entry[f"{variant_name}_{config_name}_prediction_m"] = pred
                entry[f"{variant_name}_{config_name}_residual_m"] = pred - row["ground_truth_range_m"]
                entry[f"{variant_name}_{config_name}_abs_residual_m"] = abs(pred - row["ground_truth_range_m"])
        matrix.append(entry)
    return matrix


def phase2_per_group_and_per_bin(matrix: list[dict[str, Any]]) -> tuple[list[dict], list[dict]]:
    best_key = "PHYSICAL_TEMPORAL_B_balanced"  # overnight-baseline best config
    by_group: dict[str, list[dict]] = {}
    for row in matrix:
        by_group.setdefault(row["group_id"], []).append(row)
    per_group = []
    for group_id, rows in sorted(by_group.items()):
        abs_res = [r[f"{best_key}_abs_residual_m"] for r in rows]
        signed = [r[f"{best_key}_residual_m"] for r in rows]
        raw_abs = [abs(r["raw_physical_residual_m"]) for r in rows]
        per_group.append({
            "group_id": group_id, "domain": rows[0]["domain"], "scenario_type": rows[0]["scenario_type"],
            "context": rows[0]["context"], "frame_count": len(rows),
            "best_config": best_key,
            "mae_m": mean(abs_res), "medae_m": median(abs_res),
            "bias_m": mean(signed), "std_m": pstdev(signed) if len(signed) > 1 else 0.0,
            "p90_abs_error_m": float(np.percentile(abs_res, 90)),
            "p95_abs_error_m": float(np.percentile(abs_res, 95)),
            "error_gt_3m_fraction": mean(1.0 if v > 3.0 else 0.0 for v in abs_res),
            "raw_physical_mae_m": mean(raw_abs),
            "mae_improvement_over_raw_m": mean(raw_abs) - mean(abs_res),
        })
    per_group.sort(key=lambda r: -r["mae_m"])

    static_rows = [r for r in matrix if r["domain"] == "static"]
    per_bin = []
    for label in BINS:
        chosen = [r for r in static_rows if r["distance_bin"] == label]
        if not chosen:
            continue
        abs_res = [r[f"{best_key}_abs_residual_m"] for r in chosen]
        signed = [r[f"{best_key}_residual_m"] for r in chosen]
        per_bin.append({
            "distance_bin": label, "frame_count": len(chosen),
            "mae_m": mean(abs_res), "medae_m": median(abs_res), "bias_m": mean(signed),
            "std_m": pstdev(signed) if len(signed) > 1 else 0.0,
            "p90_abs_error_m": float(np.percentile(abs_res, 90)),
            "p95_abs_error_m": float(np.percentile(abs_res, 95)),
            "error_gt_3m_fraction": mean(1.0 if v > 3.0 else 0.0 for v in abs_res),
        })
    return per_group, per_bin


def phase2_covariate_correlation(matrix: list[dict[str, Any]]) -> list[dict[str, Any]]:
    best_key = "PHYSICAL_TEMPORAL_B_balanced"
    rows = []
    abs_res_all = np.asarray([r[f"{best_key}_abs_residual_m"] for r in matrix])
    for name in COVARIATES:
        values = np.asarray([r[name] for r in matrix], dtype=np.float64)
        finite = np.isfinite(values) & np.isfinite(abs_res_all)
        if finite.sum() < 10 or np.std(values[finite]) < 1e-9:
            rows.append({"covariate": name, "pearson_r_vs_abs_residual": None, "n": int(finite.sum())})
            continue
        r = float(np.corrcoef(values[finite], abs_res_all[finite])[0, 1])
        # quartile-binned mean abs residual for a monotonic/nonlinear check
        q = np.percentile(values[finite], [25, 50, 75])
        bins = np.digitize(values[finite], q)
        quartile_means = [float(np.mean(abs_res_all[finite][bins == b])) for b in range(4)]
        rows.append({
            "covariate": name, "pearson_r_vs_abs_residual": round(r, 4), "n": int(finite.sum()),
            "quartile_1_mean_abs_residual_m": round(quartile_means[0], 4),
            "quartile_2_mean_abs_residual_m": round(quartile_means[1], 4),
            "quartile_3_mean_abs_residual_m": round(quartile_means[2], 4),
            "quartile_4_mean_abs_residual_m": round(quartile_means[3], 4),
        })
    rows.sort(key=lambda r: -abs(r["pearson_r_vs_abs_residual"] or 0.0))
    return rows


def phase2_dynamic_trajectory_phase(
    matrix: list[dict[str, Any]], collection_plan: dict[str, dict[str, Any]]
) -> list[dict[str, Any]]:
    best_key = "PHYSICAL_TEMPORAL_B_balanced"
    dynamic = [r for r in matrix if r["domain"] == "dynamic"]
    by_group: dict[str, list[dict]] = {}
    for row in dynamic:
        by_group.setdefault(row["group_id"], []).append(row)
    out = []
    for group_id, rows in by_group.items():
        rows = sorted(rows, key=lambda r: r["measurement_timestamp_s"])
        spec = collection_plan[group_id]
        t0 = rows[0]["measurement_timestamp_s"]
        t_end = rows[-1]["measurement_timestamp_s"]
        span = max(t_end - t0, 1e-6)
        hold_s = float(spec["hold_duration_s"])
        move_s = float(spec["movement_duration_s"])
        for row in rows:
            elapsed = row["measurement_timestamp_s"] - t0
            fraction = elapsed / span
            if fraction < 1.0 / 3.0:
                third = "start"
            elif fraction < 2.0 / 3.0:
                third = "mid"
            else:
                third = "end"
            if hold_s > 0:
                phase = "stationary_after_stop" if elapsed >= move_s else "moving"
            else:
                phase = "moving"
            out.append({
                "group_id": group_id, "scenario_type": rows[0]["scenario_type"],
                "trajectory_third": third, "motion_phase": phase,
                "abs_residual_m": row[f"{best_key}_abs_residual_m"],
                "raw_physical_abs_residual_m": abs(row["raw_physical_residual_m"]),
            })
    return out


def phase3_static_diagnosis(per_bin: list[dict], per_group: list[dict]) -> dict[str, Any]:
    static_groups = [g for g in per_group if g["domain"] == "static"]
    bias_values = [b["bias_m"] for b in per_bin]
    mae_values = [b["mae_m"] for b in per_bin]
    worst_bin = max(per_bin, key=lambda b: b["mae_m"])
    worst_group = max(static_groups, key=lambda g: g["mae_m"]) if static_groups else None
    systematic_bias = mean(abs(b) for b in bias_values) if bias_values else 0.0
    bin_mae_spread = (max(mae_values) - min(mae_values)) if mae_values else 0.0
    return {
        "worst_static_bin": worst_bin["distance_bin"] if worst_bin else None,
        "worst_static_bin_mae_m": worst_bin["mae_m"] if worst_bin else None,
        "worst_static_group": worst_group["group_id"] if worst_group else None,
        "worst_static_group_mae_m": worst_group["mae_m"] if worst_group else None,
        "mean_abs_bin_bias_m": round(systematic_bias, 4),
        "bin_mae_spread_m": round(bin_mae_spread, 4),
        "static_group_mae_values_m": [round(g["mae_m"], 4) for g in static_groups],
        "static_group_mae_stdev_m": round(pstdev([g["mae_m"] for g in static_groups]), 4) if len(static_groups) > 1 else 0.0,
    }


def phase3_dynamic_diagnosis(per_group: list[dict], phase_rows: list[dict]) -> dict[str, Any]:
    dynamic_groups = [g for g in per_group if g["domain"] == "dynamic"]
    by_scenario: dict[str, list[float]] = {}
    for g in dynamic_groups:
        by_scenario.setdefault(g["scenario_type"], []).append(g["mae_m"])
    approaching_mae = mean(by_scenario.get("approaching", [0.0])) if by_scenario.get("approaching") else None
    receding_mae = mean(by_scenario.get("receding", [0.0])) if by_scenario.get("receding") else None

    by_phase: dict[str, list[float]] = {}
    for row in phase_rows:
        by_phase.setdefault(row["motion_phase"], []).append(row["abs_residual_m"])
    by_third: dict[str, list[float]] = {}
    for row in phase_rows:
        by_third.setdefault(row["trajectory_third"], []).append(row["abs_residual_m"])

    static_mean_mae = mean(g["mae_m"] for g in per_group if g["domain"] == "static")
    dynamic_mean_mae = mean(g["mae_m"] for g in dynamic_groups)
    return {
        "approaching_equal_group_mae_m": round(approaching_mae, 4) if approaching_mae is not None else None,
        "receding_equal_group_mae_m": round(receding_mae, 4) if receding_mae is not None else None,
        "approaching_receding_asymmetry_m": (
            round(approaching_mae - receding_mae, 4) if approaching_mae is not None and receding_mae is not None else None
        ),
        "mae_by_motion_phase_m": {k: round(mean(v), 4) for k, v in by_phase.items()},
        "mae_by_trajectory_third_m": {k: round(mean(v), 4) for k, v in by_third.items()},
        "static_mean_group_mae_m": round(static_mean_mae, 4),
        "dynamic_mean_group_mae_m": round(dynamic_mean_mae, 4),
        "static_to_dynamic_mae_carryover_m": round(dynamic_mean_mae - static_mean_mae, 4),
        "worst_dynamic_group": max(dynamic_groups, key=lambda g: g["mae_m"])["group_id"],
        "worst_dynamic_group_mae_m": max(g["mae_m"] for g in dynamic_groups),
    }


def phase3_temporal_diagnosis(
    combined_rows: list[dict[str, Any]], trained: dict[str, dict[str, Any]]
) -> tuple[list[dict[str, Any]], dict[str, Any]]:
    dynamic_indices = [i for i, r in enumerate(combined_rows) if r["domain"] == "dynamic"]
    by_session: dict[str, list[int]] = {}
    for i in dynamic_indices:
        by_session.setdefault(combined_rows[i]["group_id"], []).append(i)

    rows_out = []
    lag_summary: dict[str, list[float]] = {"raw_physical_range": []}
    for variant_name in trained:
        lag_summary[f"{variant_name}_B_balanced"] = []

    for session_id, idx in by_session.items():
        idx = sorted(idx, key=lambda i: float(combined_rows[i]["measurement_timestamp_s"]))
        session_rows = [combined_rows[i] for i in idx]
        raw_pred = np.asarray([float(combined_rows[i]["raw_physical_range_m"]) for i in idx])
        row = _direction_row(session_rows, raw_pred, "raw_physical_range")
        rows_out.append(row)
        lag_summary["raw_physical_range"].append(abs(row["alignment_lag_s"]))
        for variant_name, trained_variant in trained.items():
            oof = trained_variant["oof"]["B_balanced"]
            pred = oof[idx]
            row = _direction_row(session_rows, pred, f"{variant_name}_B_balanced")
            rows_out.append(row)
            lag_summary[f"{variant_name}_B_balanced"].append(abs(row["alignment_lag_s"]))

    saturation = {
        method: {
            "fraction_at_boundary_2s": round(mean(1.0 if v >= 1.999 else 0.0 for v in lags), 3),
            "median_abs_lag_s": round(median(lags), 4),
        }
        for method, lags in lag_summary.items()
    }
    return rows_out, saturation


def phase3_bbox_feature_trace(combined_rows: list[dict[str, Any]]) -> list[dict[str, Any]]:
    physical_only_set = set(PHYSICAL_ONLY_FEATURES)
    physical_temporal_set = set(PHYSICAL_TEMPORAL_FEATURES)
    sample = combined_rows[:200]
    rows = []
    for variant in BBOX_AUGMENT_VARIANTS:
        deltas: dict[str, list[float]] = {}
        for row in sample:
            original = row["features"]
            perturbed = perturb_bbox_features(original, variant)
            for key in original:
                try:
                    a, b = float(original[key]), float(perturbed[key])
                except (TypeError, ValueError):
                    continue
                if a != b:
                    deltas.setdefault(key, []).append(abs(b - a))
        for key, values in deltas.items():
            rows.append({
                "bbox_perturbation_variant": variant, "feature": key,
                "mean_abs_delta": round(mean(values), 6), "n_changed": len(values),
                "used_by_PHYSICAL_ONLY": key in physical_only_set,
                "used_by_PHYSICAL_TEMPORAL": key in physical_temporal_set,
            })
    return rows


def phase3_bbox_prediction_sensitivity(
    combined_rows: list[dict[str, Any]], fold_map: dict[str, int], trained: dict[str, dict[str, Any]]
) -> list[dict[str, Any]]:
    rows = []
    for variant_name, trained_variant in trained.items():
        for config_name in ("B_balanced",):
            oof = trained_variant["oof"][config_name]
            for variant in BBOX_AUGMENT_VARIANTS:
                perturbed_pred = np.full(len(combined_rows), np.nan)
                for fold in range(3):
                    idx = [i for i, r in enumerate(combined_rows) if fold_map[r["group_id"]] == fold]
                    prep = trained_variant["preprocessors"][(config_name, fold)]
                    booster = trained_variant["boosters"][(config_name, fold)]
                    perturbed_rows = [
                        {**combined_rows[i], "features": perturb_bbox_features(combined_rows[i]["features"], variant)}
                        for i in idx
                    ]
                    x = prep.transform(perturbed_rows)
                    pred = np.clip(np.asarray(booster.predict(xgb.DMatrix(x))), *CLIP_BOUNDS)
                    for pos, i in enumerate(idx):
                        perturbed_pred[i] = pred[pos]
                shift = np.abs(perturbed_pred - oof)
                catastrophic = float(np.mean(shift > 3.0))
                by_domain: dict[str, float] = {}
                for domain in ("static", "dynamic"):
                    d_idx = [i for i, r in enumerate(combined_rows) if r["domain"] == domain]
                    by_domain[domain] = float(np.median(shift[d_idx])) if d_idx else 0.0
                rows.append({
                    "variant": variant_name, "config": config_name, "bbox_perturbation": variant,
                    "median_abs_prediction_shift_m": round(float(np.median(shift)), 6),
                    "max_abs_prediction_shift_m": round(float(np.max(shift)), 6),
                    "catastrophic_shift_gt_3m_fraction": catastrophic,
                    "median_shift_static_m": round(by_domain["static"], 6),
                    "median_shift_dynamic_m": round(by_domain["dynamic"], 6),
                })
    return rows


def phase4_nearest_neighbor_ambiguity(
    combined_rows: list[dict[str, Any]], fold_map: dict[str, int], feature_names: tuple[str, ...]
) -> list[dict[str, Any]]:
    from core_range_dynamic_robust_retrain import VariantPreprocessor

    rows_out = []
    for fold in range(3):
        train_idx = [i for i, r in enumerate(combined_rows) if fold_map[r["group_id"]] != fold]
        test_idx = [i for i, r in enumerate(combined_rows) if fold_map[r["group_id"]] == fold]
        if not test_idx:
            continue
        train_rows = [combined_rows[i] for i in train_idx]
        test_rows = [combined_rows[i] for i in test_idx]
        prep = VariantPreprocessor.fit(train_rows, feature_names)
        train_x = prep.transform(train_rows)
        test_x = prep.transform(test_rows)
        train_gt = np.asarray([float(r["ground_truth_range_m"]) for r in train_rows])
        # standardize for a fair nearest-neighbor distance
        std = np.std(train_x, axis=0)
        std[std < 1e-9] = 1.0
        mean_ = np.mean(train_x, axis=0)
        train_z = (train_x - mean_) / std
        test_z = (test_x - mean_) / std
        for pos, i in enumerate(test_idx):
            diffs = train_z - test_z[pos]
            dist = np.sqrt(np.sum(diffs * diffs, axis=1))
            nearest = int(np.argmin(dist))
            rows_out.append({
                "group_id": combined_rows[i]["group_id"], "domain": combined_rows[i]["domain"],
                "own_gt_m": float(combined_rows[i]["ground_truth_range_m"]),
                "nearest_train_group": train_rows[nearest]["group_id"],
                "nearest_train_gt_m": float(train_gt[nearest]),
                "gt_gap_m": abs(float(combined_rows[i]["ground_truth_range_m"]) - float(train_gt[nearest])),
                "feature_distance": float(dist[nearest]),
            })
    return rows_out


def phase4_distribution_shift(
    combined_rows: list[dict[str, Any]], feature_names: tuple[str, ...]
) -> list[dict[str, Any]]:
    static_rows = [r for r in combined_rows if r["domain"] == "static"]
    dynamic_rows = [r for r in combined_rows if r["domain"] == "dynamic"]
    rows_out = []
    for name in feature_names:
        s = np.asarray([float(r["features"].get(name, np.nan)) for r in static_rows])
        d = np.asarray([float(r["features"].get(name, np.nan)) for r in dynamic_rows])
        s = s[np.isfinite(s)]
        d = d[np.isfinite(d)]
        if len(s) < 5 or len(d) < 5:
            continue
        pooled_std = math.sqrt(0.5 * (np.var(s) + np.var(d))) or 1.0
        smd = (float(np.mean(d)) - float(np.mean(s))) / pooled_std
        rows_out.append({
            "feature": name, "static_mean": round(float(np.mean(s)), 4), "dynamic_mean": round(float(np.mean(d)), 4),
            "static_median": round(float(np.median(s)), 4), "dynamic_median": round(float(np.median(d)), 4),
            "standardized_mean_difference": round(smd, 4),
        })
    rows_out.sort(key=lambda r: -abs(r["standardized_mean_difference"]))
    return rows_out


def phase4_feature_ablation(
    combined_rows: list[dict[str, Any]], fold_map: dict[str, int]
) -> list[dict[str, Any]]:
    groups = {
        "physical_range_only": ("raw_physical_range_m",),
        "depth_calibration_only": (
            "target_inverse_depth", "calibration_a", "calibration_b", "calibration_denominator",
            "calibration_fit_residual", "calibration_condition", "calibration_inlier_count",
            "calibration_inlier_fraction", "calibration_q_span", "calibration_depth_span", "calibration_age",
        ),
        "geometry_only": ("image_ray_x", "image_ray_y", "ray_scale"),
        "quality_only": (
            "target_valid_fraction", "calibration_measurement_accepted", "calibration_stable",
            "deterministic_gate_applicable", "deterministic_measurement_usable",
            "anchor_accept_count", "anchor_reject_count", "anchor_coverage",
        ),
    }
    groups["physical_plus_depth"] = groups["physical_range_only"] + groups["depth_calibration_only"]
    groups["physical_plus_geometry"] = groups["physical_range_only"] + groups["geometry_only"]
    groups["complete_PHYSICAL_ONLY"] = PHYSICAL_ONLY_FEATURES
    groups["complete_PHYSICAL_TEMPORAL"] = PHYSICAL_TEMPORAL_FEATURES

    rows_out = []
    for label, feature_names in groups.items():
        trained_variant = train_variant(xgb, label, feature_names, combined_rows, fold_map)
        oof = trained_variant["oof"]["B_balanced"]
        groups_rows = per_group_rows(combined_rows, oof, "oof", "B_balanced")
        agg = equal_group_aggregate(groups_rows)
        static_g = [g for g in groups_rows if g["domain"] == "static"]
        dynamic_g = [g for g in groups_rows if g["domain"] == "dynamic"]
        rows_out.append({
            "feature_group": label, "feature_count": len(feature_names),
            "equal_group_mae_m": round(agg["mae_m"], 4),
            "static_equal_group_mae_m": round(mean(x["mae_m"] for x in static_g), 4) if static_g else None,
            "dynamic_equal_group_mae_m": round(mean(x["mae_m"] for x in dynamic_g), 4) if dynamic_g else None,
        })
    return rows_out


def main() -> None:
    OUT.mkdir(parents=True, exist_ok=True)
    print("loading combined rows and training baseline variants (B_balanced reused for deep-dive analyses)...")
    combined_rows, fold_map, trained = load_everything()

    print("Phase 2: failure matrix...")
    matrix = build_failure_matrix(combined_rows, trained)
    write_csv(OUT / "failure_matrix.csv", matrix)

    per_group, per_bin = phase2_per_group_and_per_bin(matrix)
    write_csv(OUT / "per_group_failure.csv", per_group)
    write_csv(OUT / "per_bin_failure.csv", per_bin)

    plan = json.loads((DYNAMIC_OUTPUT / "collection_plan.json").read_text())
    plan_by_id = {s["session_id"]: s for s in plan["sessions"]}
    phase_rows = phase2_dynamic_trajectory_phase(matrix, plan_by_id)

    covariate_corr = phase2_covariate_correlation(matrix)
    write_csv(OUT / "runtime_quality_correlation.csv", covariate_corr)

    print("Phase 3: root-cause diagnosis per gate family...")
    static_diag = phase3_static_diagnosis(per_bin, per_group)
    dynamic_diag = phase3_dynamic_diagnosis(per_group, phase_rows)

    direction_rows, lag_saturation = phase3_temporal_diagnosis(combined_rows, trained)
    write_csv(OUT / "direction_metrics.csv", direction_rows)
    write_csv(OUT / "temporal_failure.csv", [
        {"method": k, **v} for k, v in lag_saturation.items()
    ])

    bbox_feature_trace = phase3_bbox_feature_trace(combined_rows)
    write_csv(OUT / "bbox_feature_sensitivity.csv", bbox_feature_trace)
    bbox_pred_sensitivity = phase3_bbox_prediction_sensitivity(combined_rows, fold_map, trained)
    write_csv(OUT / "bbox_prediction_sensitivity.csv", bbox_pred_sensitivity)

    print("Phase 4: feature information audit...")
    nn_ambiguity = phase4_nearest_neighbor_ambiguity(combined_rows, fold_map, PHYSICAL_ONLY_FEATURES)
    write_csv(OUT / "nearest_neighbor_ambiguity.csv", nn_ambiguity)
    dist_shift = phase4_distribution_shift(combined_rows, PHYSICAL_ONLY_FEATURES)
    write_csv(OUT / "feature_distribution_shift.csv", dist_shift)
    ablation = phase4_feature_ablation(combined_rows, fold_map)
    write_csv(OUT / "feature_ablation.csv", ablation)

    summary = {
        "phase2_dynamic_trajectory_phase_summary": {
            k: round(mean(r["abs_residual_m"] for r in phase_rows if r["motion_phase"] == k), 4)
            for k in {r["motion_phase"] for r in phase_rows}
        },
        "phase3_static_diagnosis": static_diag,
        "phase3_dynamic_diagnosis": dynamic_diag,
        "phase3_temporal_lag_saturation": lag_saturation,
        "phase3_bbox_finding": {
            "perturb_bbox_features_only_touches": sorted({r["feature"] for r in bbox_feature_trace}),
            "PHYSICAL_ONLY_uses_any_perturbed_feature": any(r["used_by_PHYSICAL_ONLY"] for r in bbox_feature_trace),
            "PHYSICAL_TEMPORAL_uses_any_perturbed_feature": any(r["used_by_PHYSICAL_TEMPORAL"] for r in bbox_feature_trace),
            "interpretation": (
                "perturb_bbox_features() only mutates bbox_center_x/y_fraction, "
                "bbox_width/height_fraction, bbox_area_fraction, bbox_aspect_ratio -- "
                "it never recomputes image_ray_x/y, ray_scale, target_roi_q_*, or any "
                "anchor_* statistic. PHYSICAL_ONLY/PHYSICAL_TEMPORAL exclude every "
                "bbox_* field from their feature list, so this perturbation cannot "
                "move their predictions at all (median/max shift confirmed ~0.0 in "
                "bbox_prediction_sensitivity.csv). The bbox_stress catastrophic-error "
                "gate failure for these two variants is measuring baseline "
                "(unperturbed) accuracy, not bbox robustness."
            ),
        },
        "phase4_nearest_neighbor_ambiguity_summary": {
            "median_gt_gap_m_for_close_neighbors": round(
                median(r["gt_gap_m"] for r in nn_ambiguity if r["feature_distance"] < 0.5), 4
            ) if any(r["feature_distance"] < 0.5 for r in nn_ambiguity) else None,
            "fraction_close_neighbors_with_gt_gap_gt_1m": round(
                mean(1.0 if r["gt_gap_m"] > 1.0 else 0.0 for r in nn_ambiguity if r["feature_distance"] < 0.5), 4
            ) if any(r["feature_distance"] < 0.5 for r in nn_ambiguity) else None,
        },
    }
    (OUT / "phase2_4_summary.json").write_text(json.dumps(summary, indent=2, default=str))
    print(json.dumps(summary, indent=2, default=str))


if __name__ == "__main__":
    main()
