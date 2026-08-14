"""CORE_RANGE_OFFLINE_FAILURE_ANALYSIS_AND_TARGETED_RETRAIN, Phase 6-9.

Trains and evaluates the 3 precommitted candidates
(artifacts/core_range_3_12m/offline_targeted_retrain/candidate_precommit.json)
against the same frozen corpus, same group-disjoint evaluation machinery
(evaluate_config/oof_bbox_stress/gates) as the baseline six-config run,
reused unmodified. Only the objective function and an optional monotonic
constraint on raw_physical_range_m differ per candidate -- feature sets,
preprocessing, group weighting, and causal/temporal reset policy are
otherwise identical to the corresponding baseline variant.
"""

from __future__ import annotations

import csv
import hashlib
import json
from pathlib import Path
from statistics import mean, median, pstdev
from typing import Any

import numpy as np
import xgboost as xgb

from core_range_5hz_headless_retrain import load_combined_rows
from core_range_dynamic_robust_retrain import (
    BBOX_GATES,
    CLIP_BOUNDS,
    DYNAMIC_GATES,
    PHYSICAL_ONLY_FEATURES,
    PHYSICAL_TEMPORAL_FEATURES,
    STATIC_GATES,
    TEMPORAL_GATES,
    VariantPreprocessor,
    _bbox_gate,
    add_temporal_features,
    assign_combined_folds,
    equal_group_aggregate,
    evaluate_config,
    oof_bbox_stress,
    per_group_rows,
    unified_group_weights,
)

WORKSPACE = Path(__file__).resolve().parent
DYNAMIC_OUTPUT = WORKSPACE / "artifacts/core_range_3_12m/overnight_sim_time_train/frozen_corpus"
OUT = WORKSPACE / "artifacts/core_range_3_12m/offline_targeted_retrain"


def write_csv(path: Path, rows: list[dict[str, Any]]) -> None:
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


def train_candidate(
    combined_rows: list[dict[str, Any]],
    fold_map: dict[str, int],
    feature_names: tuple[str, ...],
    hyperparameters: dict[str, Any],
    objective: str,
    monotone_constraints: tuple[int, ...] | None,
    seed: int,
) -> dict[str, Any]:
    n = len(combined_rows)
    oof = np.full(n, np.nan, dtype=np.float64)
    boosters: dict[int, Any] = {}
    preprocessors: dict[int, VariantPreprocessor] = {}
    for fold in range(3):
        train_indices = [i for i, row in enumerate(combined_rows) if fold_map[row["group_id"]] != fold]
        test_indices = [i for i, row in enumerate(combined_rows) if fold_map[row["group_id"]] == fold]
        train_rows = [combined_rows[i] for i in train_indices]
        test_rows = [combined_rows[i] for i in test_indices]
        preprocessor = VariantPreprocessor.fit(train_rows, feature_names)
        train_x = preprocessor.transform(train_rows)
        test_x = preprocessor.transform(test_rows)
        train_y = np.asarray([float(r["ground_truth_range_m"]) for r in train_rows])
        weights = unified_group_weights(train_rows)
        params = {
            "objective": objective, "eval_metric": "mae", "tree_method": "hist",
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
        oof[test_indices] = test_pred
        boosters[fold] = booster
        preprocessors[fold] = preprocessor
    if np.any(~np.isfinite(oof)):
        raise ValueError("oof_incomplete")
    return {"oof": oof, "boosters": boosters, "preprocessors": preprocessors}


def monotone_vector(feature_names: tuple[str, ...], positive: tuple[str, ...]) -> tuple[int, ...]:
    return tuple(1 if name in positive else 0 for name in feature_names)


def gate_summary(evaluation: dict[str, Any], bbox_gate: dict[str, Any]) -> dict[str, Any]:
    return {
        "static_mae_m": evaluation["static_gate"]["equal_group_mae_m"],
        "static_worst_bin_mae_m": evaluation["static_gate"]["worst_bin_mae_m"],
        "static_p90_m": evaluation["static_gate"]["p90_m"],
        "static_error_gt_3m_fraction": evaluation["static_gate"]["error_gt_3m_fraction"],
        "static_stationary_std_m": evaluation["static_gate"]["stationary_std_m"],
        "static_passed": evaluation["static_gate"]["passed"],
        "dynamic_mae_m": evaluation["dynamic_gate"]["equal_group_mae_m"],
        "dynamic_worst_session_mae_m": evaluation["dynamic_gate"]["worst_session_mae_m"],
        "dynamic_p90_m": evaluation["dynamic_gate"]["p90_m"],
        "dynamic_error_gt_3m_fraction": evaluation["dynamic_gate"]["error_gt_3m_fraction"],
        "dynamic_passed": evaluation["dynamic_gate"]["passed"],
        "median_absolute_lag_s": evaluation["temporal_gate"]["median_absolute_lag_s"],
        "worst_settling_time_s": evaluation["temporal_gate"]["worst_stop_settling_time_s"],
        "temporal_passed": evaluation["temporal_gate"]["passed"],
        "bbox_pm5_mae_degradation_m": bbox_gate["worst_pm5_mae_degradation_m"],
        "bbox_pm10_catastrophic_fraction": bbox_gate["worst_pm10_catastrophic_error_fraction"],
        "bbox_passed": bbox_gate["passed"],
        "all_gates_passed": (
            evaluation["static_gate"]["passed"] and evaluation["dynamic_gate"]["passed"]
            and evaluation["temporal_gate"]["passed"] and bbox_gate["passed"]
        ),
    }


def group_bootstrap_ci(group_rows: list[dict[str, Any]], n_boot: int = 500, seed: int = 52) -> dict[str, Any]:
    """Group-level (not frame-level) bootstrap: resample whole groups with
    replacement, recompute equal-group MAE each draw."""
    rng = np.random.default_rng(seed)
    maes = [float(r["mae_m"]) for r in group_rows]
    if not maes:
        return {"n": 0}
    draws = []
    n = len(maes)
    for _ in range(n_boot):
        sample = rng.choice(maes, size=n, replace=True)
        draws.append(float(np.mean(sample)))
    draws.sort()
    lo = draws[int(0.025 * n_boot)]
    hi = draws[int(0.975 * n_boot) - 1]
    return {"n_groups": n, "point_estimate_mae_m": round(mean(maes), 4), "ci95_lo_m": round(lo, 4), "ci95_hi_m": round(hi, 4)}


def main() -> None:
    combined_rows, _manifest = load_combined_rows(WORKSPACE, DYNAMIC_OUTPUT)
    add_temporal_features(combined_rows)
    fold_map_52 = assign_combined_folds(WORKSPACE, combined_rows, seed=52)

    candidate_precommit = json.loads((OUT / "candidate_precommit.json").read_text())
    repeat_seeds = candidate_precommit["repeat_seeds"]

    baseline_key_static = 1.078731344506175  # PHYSICAL_TEMPORAL/B_balanced, from model_comparison.csv
    baseline_key_dynamic = 1.250843054145135

    model_comparison_rows = []
    candidate_metrics_rows = []
    group_bootstrap_rows = []
    prediction_rows: list[dict[str, Any]] = []
    static_metrics_rows: list[dict[str, Any]] = []
    dynamic_metrics_rows: list[dict[str, Any]] = []
    temporal_metrics_rows: list[dict[str, Any]] = []
    bbox_stress_rows: list[dict[str, Any]] = []
    frozen_candidate = None
    models_dir = OUT / "models"
    models_dir.mkdir(exist_ok=True)

    for spec in candidate_precommit["candidates"]:
        name = spec["name"]
        feature_names = tuple(spec["feature_order"])
        hp = spec["hyperparameters"]
        objective = spec["objective"]
        mc_spec = spec["monotone_constraints"]
        monotone = None
        if mc_spec and mc_spec != "None":
            positive = tuple(f.strip().split(":")[0] for f in mc_spec.split(";") if "+1" in f)
            monotone = monotone_vector(feature_names, positive)

        print(f"=== training {name} (primary seed=52) ===", flush=True)
        trained = train_candidate(combined_rows, fold_map_52, feature_names, hp, objective, monotone, seed=52)
        evaluation = evaluate_config(combined_rows, trained["oof"])
        bbox_stress = oof_bbox_stress(
            xgb, combined_rows, fold_map_52,
            {(name, f): b for f, b in trained["boosters"].items()},
            {(name, f): p for f, p in trained["preprocessors"].items()},
            name,
        )
        bbox_gate = _bbox_gate(bbox_stress)
        summary = gate_summary(evaluation, bbox_gate)
        summary["candidate"] = name
        summary["static_mae_improvement_vs_baseline_m"] = round(baseline_key_static - summary["static_mae_m"], 4)
        summary["dynamic_mae_improvement_vs_baseline_m"] = round(baseline_key_dynamic - summary["dynamic_mae_m"], 4)
        model_comparison_rows.append(summary)

        for row in evaluation["static_groups"]:
            static_metrics_rows.append({"candidate": name, **row})
        for row in evaluation["dynamic_groups"]:
            dynamic_metrics_rows.append({"candidate": name, **row})
        for row in evaluation["direction_rows"]:
            temporal_metrics_rows.append({"candidate": name, "kind": "direction", **row})
        for row in evaluation["stop_rows"]:
            temporal_metrics_rows.append({"candidate": name, "kind": "stop", **row})
        for row in bbox_stress:
            bbox_stress_rows.append({"candidate": name, **row})

        all_group_rows = evaluation["static_groups"] + evaluation["dynamic_groups"]
        for scope, rows in (("all", all_group_rows), ("static", evaluation["static_groups"]), ("dynamic", evaluation["dynamic_groups"])):
            ci = group_bootstrap_ci(rows, seed=52)
            group_bootstrap_rows.append({"candidate": name, "scope": scope, **ci})

        for index, row in enumerate(combined_rows):
            prediction_rows.append({
                "candidate": name, "group_id": row["group_id"], "domain": row["domain"],
                "scenario_type": row["scenario_type"], "fold": fold_map_52[row["group_id"]],
                "ground_truth_range_m": row["ground_truth_range_m"], "raw_physical_range_m": row["raw_physical_range_m"],
                "oof_prediction_m": float(trained["oof"][index]),
            })

        # repeat-seed evaluation for confidence-interval robustness (different fold partition each)
        repeat_summaries = []
        for rseed in repeat_seeds:
            if rseed == 52:
                repeat_summaries.append({"seed": rseed, "static_mae_m": summary["static_mae_m"], "dynamic_mae_m": summary["dynamic_mae_m"]})
                continue
            fold_map_r = assign_combined_folds(WORKSPACE, combined_rows, seed=rseed)
            trained_r = train_candidate(combined_rows, fold_map_r, feature_names, hp, objective, monotone, seed=rseed)
            evaluation_r = evaluate_config(combined_rows, trained_r["oof"])
            repeat_summaries.append({
                "seed": rseed,
                "static_mae_m": evaluation_r["static_gate"]["equal_group_mae_m"],
                "dynamic_mae_m": evaluation_r["dynamic_gate"]["equal_group_mae_m"],
            })
        candidate_metrics_rows.append({
            "candidate": name, "hypothesis": spec["hypothesis"][:150],
            "primary_seed_static_mae_m": summary["static_mae_m"], "primary_seed_dynamic_mae_m": summary["dynamic_mae_m"],
            "repeat_seed_static_mae_values": [round(r["static_mae_m"], 4) for r in repeat_summaries],
            "repeat_seed_dynamic_mae_values": [round(r["dynamic_mae_m"], 4) for r in repeat_summaries],
            "static_mae_stdev_across_seeds": round(pstdev([r["static_mae_m"] for r in repeat_summaries]), 4) if len(repeat_summaries) > 1 else 0.0,
            "dynamic_mae_stdev_across_seeds": round(pstdev([r["dynamic_mae_m"] for r in repeat_summaries]), 4) if len(repeat_summaries) > 1 else 0.0,
            "all_gates_passed_primary_seed": summary["all_gates_passed"],
            "meets_rejection_criteria_improvement": (
                summary["static_mae_improvement_vs_baseline_m"] >= 0.05 or summary["dynamic_mae_improvement_vs_baseline_m"] >= 0.05
            ),
        })

        print(json.dumps(summary, indent=2, default=str), flush=True)

        if summary["all_gates_passed"] and frozen_candidate is None:
            frozen_candidate = {"name": name, "spec": spec, "trained": trained, "summary": summary}
            for fold, booster in trained["boosters"].items():
                booster.save_model(str(models_dir / f"{name}_fold_{fold}.json"))
                train_groups = sorted({r["group_id"] for r in combined_rows if fold_map_52[r["group_id"]] != fold})
                (models_dir / f"{name}_fold_{fold}_preprocessing.json").write_text(
                    json.dumps(trained["preprocessors"][fold].as_json(train_groups), indent=2, sort_keys=True) + "\n"
                )

    write_csv(OUT / "model_comparison.csv", model_comparison_rows)
    write_csv(OUT / "candidate_metrics.csv", candidate_metrics_rows)
    write_csv(OUT / "group_bootstrap.csv", group_bootstrap_rows)
    write_csv(OUT / "prediction_rows.csv", prediction_rows)
    write_csv(OUT / "static_metrics.csv", static_metrics_rows)
    write_csv(OUT / "dynamic_metrics.csv", dynamic_metrics_rows)
    write_csv(OUT / "temporal_metrics.csv", temporal_metrics_rows)
    write_csv(OUT / "bbox_stress_metrics.csv", bbox_stress_rows)
    fold_rows = [{"group_id": g, "fold": f} for g, f in sorted(fold_map_52.items())]
    write_csv(OUT / "fold_assignments.csv", fold_rows)

    conclusion = "TARGETED_OFFLINE_CANDIDATE_MEETS_CURRENT_GATES" if frozen_candidate else "TARGETED_MODEL_IMPROVES_BUT_STILL_FAILS"
    any_improvement = any(r["static_mae_improvement_vs_baseline_m"] > 0 or r["dynamic_mae_improvement_vs_baseline_m"] > 0 for r in model_comparison_rows)
    if not frozen_candidate and not any_improvement:
        conclusion = "TARGETED_MODEL_IMPROVES_BUT_STILL_FAILS"  # still report metrics either way; final wording decided in report

    manifest = {
        "phase": "PHASE_9_CANDIDATE_HANDLING",
        "frozen_candidate": frozen_candidate["name"] if frozen_candidate else None,
        "frozen_candidate_status": "OFFLINE_DEVELOPMENT_CANDIDATE" if frozen_candidate else None,
        "model_comparison": model_comparison_rows,
        "seed": 52, "repeat_seeds": repeat_seeds,
    }
    (OUT / "targeted_retrain_manifest.json").write_text(json.dumps(manifest, indent=2, default=str))
    print(json.dumps({"frozen_candidate": manifest["frozen_candidate"]}, indent=2))


if __name__ == "__main__":
    main()
