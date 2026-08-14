"""CORE_RANGE_STABLE_RELATIVE_TRACKING_3_12M, Phase 8-10 sweep driver."""

from __future__ import annotations

import csv
import json
from pathlib import Path
from statistics import mean

import numpy as np

import core_range_stable_relative_tracking as s
import core_range_stable_tracking_filter_and_metrics as m

OUT = s.OUT


def evaluate_combo(rows: list[dict], pred: np.ndarray) -> dict:
    direction = m.direction_metrics(rows, pred)
    rank = m.rank_correlation_metrics(rows, pred)
    stationary = m.stationary_metrics(rows, pred)
    jump = m.frame_jump_metrics(rows, pred)
    per_bin = m.per_bin_metrics(rows, pred)
    dynamic = m.dynamic_quality_metrics(rows, pred)
    gt = np.asarray([r["ground_truth_range_m"] for r in rows])
    global_mae = float(np.mean(np.abs(pred - gt)))
    worst_bin_mae = max((b["mae_m"] for b in per_bin), default=None)
    worst_bin_label = max(per_bin, key=lambda b: b["mae_m"])["distance_bin"] if per_bin else None
    bin_bias_spread = (max(b["bias_m"] for b in per_bin) - min(b["bias_m"] for b in per_bin)) if per_bin else None
    any_bin_collapsed = any(b["prediction_std_within_bin"] < 0.02 and b["sample_count"] > 5 for b in per_bin)
    return {
        "direction": direction, "rank": rank, "stationary": stationary, "jump": jump,
        "per_bin": per_bin, "dynamic": dynamic,
        "global_mae_m": round(global_mae, 4), "worst_bin_mae_m": worst_bin_mae, "worst_bin_label": worst_bin_label,
        "bin_bias_spread_m": round(bin_bias_spread, 4) if bin_bias_spread is not None else None,
        "any_bin_collapsed": any_bin_collapsed, "collapsed": rank["collapsed_session_count"] > 0 or any_bin_collapsed,
    }


def stability_score(evaluation: dict) -> float:
    """Higher is better. Not global-MAE-dominated: MAE only contributes a
    small, saturating term; direction agreement, absence of catastrophic
    jumps, and low stationary jitter dominate, per Phase 10's explicit
    ordering."""
    if evaluation["collapsed"]:
        return -1000.0
    direction = evaluation["direction"]["overall_direction_agreement"] or 0.0
    spearman = evaluation["rank"]["overall_spearman"] or 0.0
    stationary_std = evaluation["stationary"]["stationary_std_m"] or 1.0
    catastrophic = evaluation["jump"]["catastrophic_jump_count"] or 0
    worst_bin = evaluation["worst_bin_mae_m"] or 3.0
    bias_spread = evaluation["bin_bias_spread_m"] or 3.0
    dynamic_mae = evaluation["dynamic"]["dynamic_equal_group_mae_m"] or 3.0

    score = 0.0
    score += 40.0 * direction  # 0..40
    score += 20.0 * max(0.0, spearman)  # 0..20
    score += 15.0 * max(0.0, 1.0 - stationary_std / 0.40)  # 0 at std>=0.40m, 15 at std=0
    score -= 10.0 * catastrophic  # heavy penalty, uncapped downside
    score += 10.0 * max(0.0, 1.0 - worst_bin / 2.0)  # 0..10
    score += 5.0 * max(0.0, 1.0 - bias_spread / 2.0)  # 0..5
    score += 5.0 * max(0.0, 1.0 - min(dynamic_mae, 3.0) / 3.0)  # 0..5, saturates, small weight
    return round(score, 4)


def passes_practical_gate(evaluation: dict) -> bool:
    d = evaluation["direction"]["overall_direction_agreement"]
    sp = evaluation["rank"]["overall_spearman"]
    stat = evaluation["stationary"]["stationary_std_m"]
    cat = evaluation["jump"]["catastrophic_jump_count"]
    mae = evaluation["global_mae_m"]
    worst = evaluation["worst_bin_mae_m"]
    approaching_ok = (evaluation["direction"]["approaching_direction_agreement"] or 0) >= 0.5
    receding_ok = (evaluation["direction"]["receding_direction_agreement"] or 0) >= 0.5
    return bool(
        not evaluation["collapsed"] and d is not None and d >= 0.90 and sp is not None and sp >= 0.90
        and stat is not None and stat <= 0.40 and cat == 0 and mae <= 1.5 and worst is not None and worst <= 2.0
        and approaching_ok and receding_ok
    )


def main() -> None:
    rows = m.load_prediction_rows()
    candidates = ["CANDIDATE_A_GROUP_BALANCED", "CANDIDATE_B_GROUP_AND_DISTANCE_BALANCED", "CANDIDATE_C_MONOTONIC_RAY_SCALE"]
    calibrations = {"none": "pred_none_m", "affine": "pred_affine_m", "isotonic": "pred_isotonic_m"}

    all_results = []
    calibration_comparison = []
    filter_comparison = []

    for cand in candidates:
        cand_rows = [r for r in rows if r["candidate"] == cand]
        for calib_name, key in calibrations.items():
            pred = np.asarray([r[key] for r in cand_rows])
            evaluation = evaluate_combo(cand_rows, pred)
            score = stability_score(evaluation)
            record = {
                "candidate": cand, "calibration": calib_name, "filter": "none",
                "alpha": None, "beta": None, "threshold_m": None,
                "stability_score": score, "practical_gate_pass": passes_practical_gate(evaluation),
                "direction_agreement": evaluation["direction"]["overall_direction_agreement"],
                "spearman": evaluation["rank"]["overall_spearman"],
                "stationary_std_m": evaluation["stationary"]["stationary_std_m"],
                "catastrophic_jumps": evaluation["jump"]["catastrophic_jump_count"],
                "global_mae_m": evaluation["global_mae_m"], "worst_bin_mae_m": evaluation["worst_bin_mae_m"],
                "worst_bin_label": evaluation["worst_bin_label"], "dynamic_mae_m": evaluation["dynamic"]["dynamic_equal_group_mae_m"],
                "collapsed": evaluation["collapsed"],
            }
            calibration_comparison.append(record)
            all_results.append((record, evaluation))

            for alpha in m.ALPHA_GRID:
                for beta in m.BETA_GRID:
                    for threshold in m.THRESHOLD_GRID:
                        filtered = m.apply_filter_all_sessions(cand_rows, key, alpha, beta, threshold)
                        f_eval = evaluate_combo(cand_rows, filtered)
                        f_score = stability_score(f_eval)
                        f_record = {
                            "candidate": cand, "calibration": calib_name, "filter": "alpha_beta",
                            "alpha": alpha, "beta": beta, "threshold_m": threshold,
                            "stability_score": f_score, "practical_gate_pass": passes_practical_gate(f_eval),
                            "direction_agreement": f_eval["direction"]["overall_direction_agreement"],
                            "spearman": f_eval["rank"]["overall_spearman"],
                            "stationary_std_m": f_eval["stationary"]["stationary_std_m"],
                            "catastrophic_jumps": f_eval["jump"]["catastrophic_jump_count"],
                            "global_mae_m": f_eval["global_mae_m"], "worst_bin_mae_m": f_eval["worst_bin_mae_m"],
                            "worst_bin_label": f_eval["worst_bin_label"], "dynamic_mae_m": f_eval["dynamic"]["dynamic_equal_group_mae_m"],
                            "collapsed": f_eval["collapsed"],
                        }
                        filter_comparison.append(f_record)
                        all_results.append((f_record, f_eval))
        print(f"=== {cand} done ===", flush=True)

    s.write_csv(OUT / "calibration_comparison.csv", calibration_comparison)
    s.write_csv(OUT / "filter_comparison.csv", filter_comparison)

    all_results.sort(key=lambda pair: -pair[0]["stability_score"])
    best_record, best_eval = all_results[0]

    s.write_csv(OUT / "stability_score.csv", [r for r, _ in all_results])
    s.write_csv(OUT / "direction_metrics.csv", [
        {"candidate": r["candidate"], "calibration": r["calibration"], "filter": r["filter"], "alpha": r["alpha"], "beta": r["beta"], "threshold_m": r["threshold_m"], **e["direction"]}
        for r, e in all_results
    ])
    s.write_csv(OUT / "rank_correlation.csv", [
        {"candidate": r["candidate"], "calibration": r["calibration"], "filter": r["filter"], "alpha": r["alpha"], "beta": r["beta"], "threshold_m": r["threshold_m"],
         "overall_spearman": e["rank"]["overall_spearman"], "approaching_spearman": e["rank"]["approaching_spearman"],
         "receding_spearman": e["rank"]["receding_spearman"], "collapsed_session_count": e["rank"]["collapsed_session_count"]}
        for r, e in all_results
    ])
    s.write_csv(OUT / "stationary_metrics.csv", [
        {"candidate": r["candidate"], "calibration": r["calibration"], "filter": r["filter"], "alpha": r["alpha"], "beta": r["beta"], "threshold_m": r["threshold_m"], **e["stationary"]}
        for r, e in all_results
    ])
    s.write_csv(OUT / "frame_jump_metrics.csv", [
        {"candidate": r["candidate"], "calibration": r["calibration"], "filter": r["filter"], "alpha": r["alpha"], "beta": r["beta"], "threshold_m": r["threshold_m"], **e["jump"]}
        for r, e in all_results
    ])
    per_bin_rows = []
    for r, e in all_results:
        for b in e["per_bin"]:
            per_bin_rows.append({"candidate": r["candidate"], "calibration": r["calibration"], "filter": r["filter"], "alpha": r["alpha"], "beta": r["beta"], "threshold_m": r["threshold_m"], **b})
    s.write_csv(OUT / "per_bin_metrics.csv", per_bin_rows)
    s.write_csv(OUT / "dynamic_metrics.csv", [
        {"candidate": r["candidate"], "calibration": r["calibration"], "filter": r["filter"], "alpha": r["alpha"], "beta": r["beta"], "threshold_m": r["threshold_m"],
         **{k: v for k, v in e["dynamic"].items() if k != "per_session_mae_m"}}
        for r, e in all_results
    ])

    print("=== BEST BY STABILITY SCORE ===")
    print(json.dumps(best_record, indent=2, default=str))
    print("practical_gate_pass:", best_record["practical_gate_pass"])

    any_pass = [r for r, e in all_results if r["practical_gate_pass"]]
    print(f"\ncombinations passing practical gate: {len(any_pass)} / {len(all_results)}")
    if any_pass:
        any_pass.sort(key=lambda r: -r["stability_score"])
        print(json.dumps(any_pass[0], indent=2, default=str))


if __name__ == "__main__":
    main()
