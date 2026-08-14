"""Giai đoạn 1.3 -- run each residual-corrector config's dynamic predictions
through the unchanged control-ready pipeline (age compensation OFF per the
roadmap's own precommit) and check the Giai đoạn 1 gate.

Reuses control_ready_range_estimator.py unmodified. Reimplements a compact
version of the Section 9 metrics needed for the gate (global/worst-bin MAE,
catastrophic jumps, stationary std, windowed direction agreement) rather than
importing core_range_control_ready_offline_metrics.py, which is wired to a
different (raw Candidate B) input path -- avoids touching working, already-
tested code from the prior task.
"""

from __future__ import annotations

import csv
import json
from pathlib import Path
from statistics import mean

import numpy as np

import core_range_stable_relative_tracking as srt
from control_ready_range_estimator import ControlReadyRangeEstimator, RangeTrendEstimator

WORKSPACE = Path(__file__).resolve().parent
OUT = WORKSPACE / "artifacts/core_range_3_12m/residual_bias_correction"
PRECOMMIT = json.loads((OUT / "residual_corrector_precommit.json").read_text())
GATE = PRECOMMIT["residual_corrector_gate_thresholds"]

TREND_WINDOWS_S = (0.6, 0.8, 1.0)
MOTION_THRESHOLD_M = 0.10
CATASTROPHIC_JUMP_PRED_DELTA_M = 1.5
FRAME_JUMP_SMALL_GT_DELTA_M = 0.5
HOLD_BAND_M = 0.15
CONFIGS = ("A_RIDGE_BASELINE", "B_SHALLOW_XGBOOST_STRONG_REG", "C_SHALLOW_XGBOOST_STABLE_FEATURE_SUBSET")


def load_dynamic_rows(config: str) -> list[dict]:
    path = OUT / f"corrected_prediction_rows_{config}.csv"
    with path.open() as f:
        rows = [r for r in csv.DictReader(f) if r["domain"] == "dynamic"]
    for r in rows:
        r["measurement_timestamp_s"] = float(r["measurement_timestamp_s"])
        r["ground_truth_range_m"] = float(r["ground_truth_range_m"])
        r["corrected_range_m"] = float(r["pred_none_m"])
    return rows


def replay(config: str) -> list[dict]:
    rows = load_dynamic_rows(config)
    by_group: dict[str, list[dict]] = {}
    for r in rows:
        by_group.setdefault(r["group_id"], []).append(r)

    out_rows = []
    for group_id, group_rows in sorted(by_group.items()):
        ordered = sorted(group_rows, key=lambda r: r["measurement_timestamp_s"])
        estimator = ControlReadyRangeEstimator()
        trend = RangeTrendEstimator(windows_s=TREND_WINDOWS_S)
        for row in ordered:
            t = row["measurement_timestamp_s"]
            # age compensation OFF per Giai đoạn 1 precommit: query "now" == measurement time
            est_out = estimator.update(
                raw_candidate_range_m=row["corrected_range_m"], measurement_timestamp_s=t,
                current_timestamp_s=t, measurement_valid=True, measurement_age_s=0.0,
                session_id=group_id, model_signature=config,
            )
            trend_out = trend.update(t, est_out.predicted_current_range_m, valid=est_out.estimator_valid)
            record = dict(row)
            record["filtered_range_m"] = est_out.filtered_range_m
            record["predicted_current_range_m"] = est_out.predicted_current_range_m
            record["estimator_valid"] = est_out.estimator_valid
            record["estimator_status"] = est_out.estimator_status
            for w in TREND_WINDOWS_S:
                to = trend_out[w]
                record[f"trend_state_{w}s"] = to.state
                record[f"trend_slope_{w}s"] = to.slope_mps
            out_rows.append(record)
    return out_rows


def group_ordered(rows: list[dict]) -> dict[str, list[dict]]:
    by_group: dict[str, list[dict]] = {}
    for r in rows:
        by_group.setdefault(r["group_id"], []).append(r)
    for g in by_group:
        by_group[g] = sorted(by_group[g], key=lambda r: r["measurement_timestamp_s"])
    return by_group


def hold_phase_rows(grows: list[dict]) -> list[dict]:
    gt = [r["ground_truth_range_m"] for r in grows]
    final = gt[-1]
    start = len(grows) - 1
    for i in range(len(grows) - 1, -1, -1):
        if abs(gt[i] - final) <= HOLD_BAND_M:
            start = i
        else:
            break
    return grows[start:]


def range_quality(rows: list[dict], by_group: dict[str, list[dict]]) -> dict:
    pred = np.asarray([r["predicted_current_range_m"] for r in rows])
    gt = np.asarray([r["ground_truth_range_m"] for r in rows])
    abs_err = np.abs(pred - gt)

    worst_bin_mae = 0.0
    for label, lo, hi in srt.DISTANCE_BINS:
        idx = [i for i, r in enumerate(rows) if lo <= r["ground_truth_range_m"] < hi]
        if not idx:
            continue
        worst_bin_mae = max(worst_bin_mae, float(np.mean(abs_err[idx])))

    stationary_stds = []
    for group_id, grows in by_group.items():
        if grows[0]["scenario_type"] != "stop_and_hold":
            continue
        hold = hold_phase_rows(grows)
        if len(hold) < 3:
            continue
        hp = np.asarray([r["predicted_current_range_m"] for r in hold])
        stationary_stds.append(float(np.std(hp)))

    catastrophic = 0
    for group_id, grows in by_group.items():
        g_gt = np.asarray([r["ground_truth_range_m"] for r in grows])
        g_pred = np.asarray([r["predicted_current_range_m"] for r in grows])
        gt_delta = np.abs(np.diff(g_gt))
        pr_delta = np.abs(np.diff(g_pred))
        for gd, pd in zip(gt_delta, pr_delta):
            if gd < FRAME_JUMP_SMALL_GT_DELTA_M and pd > CATASTROPHIC_JUMP_PRED_DELTA_M:
                catastrophic += 1

    return {
        "global_mae_m": round(float(np.mean(abs_err)), 4),
        "worst_bin_mae_m": round(worst_bin_mae, 4),
        "stationary_std_m": round(mean(stationary_stds), 4) if stationary_stds else None,
        "catastrophic_jump_count": catastrophic,
    }


def windowed_direction(by_group: dict[str, list[dict]]) -> dict[float, dict]:
    results = {}
    for w in TREND_WINDOWS_S:
        agreements, approach_agree, recede_agree = [], [], []
        for group_id, grows in by_group.items():
            ts = [r["measurement_timestamp_s"] for r in grows]
            gt = [r["ground_truth_range_m"] for r in grows]
            states = [r[f"trend_state_{w}s"] for r in grows]
            n = len(grows)
            for i in range(n):
                ref_idx = None
                for j in range(i - 1, -1, -1):
                    if ts[i] - ts[j] >= w:
                        ref_idx = j
                        break
                if ref_idx is None:
                    continue
                gt_delta = gt[i] - gt[ref_idx]
                if abs(gt_delta) < MOTION_THRESHOLD_M:
                    continue
                correct_state = "APPROACHING" if gt_delta < 0 else "RECEDING"
                is_correct = states[i] == correct_state
                agreements.append(is_correct)
                (approach_agree if correct_state == "APPROACHING" else recede_agree).append(is_correct)
        results[w] = {
            "window_s": w,
            "overall_direction_agreement": round(mean(agreements), 4) if agreements else None,
            "approaching_direction_agreement": round(mean(approach_agree), 4) if approach_agree else None,
            "receding_direction_agreement": round(mean(recede_agree), 4) if recede_agree else None,
        }
    return results


def bias_reduction(config: str, rows: list[dict]) -> dict:
    """Compare persistent-bias cells from Phase 1.1's bias_map.csv (dynamic
    rows only, since Phase 1.1 covered both domains but Phase 1.3 gate is
    dynamic-only) against the corrected residual in the same cells."""
    with (OUT / "bias_map.csv").open() as f:
        baseline_cells = [r for r in csv.DictReader(f) if r["domain"] == "dynamic" and r["persistent_bias"] == "True"]

    def geometry_context(group_id: str) -> str:
        gid = group_id.lower()
        if "yaw" in gid or "oblique" in gid:
            return "yaw_oblique"
        if "lateral" in gid or "left" in gid or "right" in gid:
            return "lateral"
        return "center"

    cell_after: dict[tuple[str, str], list[float]] = {}
    for r in rows:
        key = (geometry_context(r["group_id"]), srt.bin_of(r["ground_truth_range_m"]))
        cell_after.setdefault(key, []).append(r["ground_truth_range_m"] - r["corrected_range_m"])

    reductions = []
    for cell in baseline_cells:
        key = (cell["geometry_context"], cell["distance_bin"])
        before = abs(float(cell["mean_bias_m"]))
        after_vals = cell_after.get(key)
        if not after_vals or before < 1e-6:
            continue
        after = abs(mean(after_vals))
        reductions.append({"geometry_context": key[0], "distance_bin": key[1],
                            "bias_before_m": round(before, 4), "bias_after_m": round(after, 4),
                            "reduction_fraction": round(1.0 - after / before, 4)})
    mean_reduction = mean(r["reduction_fraction"] for r in reductions) if reductions else None
    return {"config": config, "cells_compared": len(reductions), "mean_reduction_fraction": round(mean_reduction, 4) if mean_reduction is not None else None,
            "cells": reductions}


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
    all_results = []
    bias_reduction_rows = []
    for config in CONFIGS:
        replay_rows = replay(config)
        by_group = group_ordered(replay_rows)
        rq = range_quality(replay_rows, by_group)
        direction_by_window = windowed_direction(by_group)
        best_window = max(direction_by_window.values(), key=lambda d: d["overall_direction_agreement"] or -1)
        br = bias_reduction(config, load_dynamic_rows(config))
        bias_reduction_rows.append({"config": config, "cells_compared": br["cells_compared"], "mean_reduction_fraction": br["mean_reduction_fraction"]})

        checks = {
            "persistent_bias_reduction": (br["mean_reduction_fraction"], (br["mean_reduction_fraction"] or -1) >= GATE["persistent_bias_reduction_min_fraction"]),
            "overall_windowed_direction_agreement": (best_window["overall_direction_agreement"], (best_window["overall_direction_agreement"] or 0) >= GATE["windowed_direction_agreement_min"]),
            "approaching_direction_agreement": (best_window["approaching_direction_agreement"], (best_window["approaching_direction_agreement"] or 0) >= GATE["approaching_direction_agreement_min"]),
            "receding_direction_agreement": (best_window["receding_direction_agreement"], (best_window["receding_direction_agreement"] or 0) >= GATE["receding_direction_agreement_min"]),
            "stationary_std_m": (rq["stationary_std_m"], (rq["stationary_std_m"] or 999) <= GATE["stationary_std_m_max"]),
            "global_mae_m": (rq["global_mae_m"], rq["global_mae_m"] <= GATE["global_mae_m_max"]),
            "worst_bin_mae_m": (rq["worst_bin_mae_m"], rq["worst_bin_mae_m"] <= GATE["worst_bin_mae_m_max"]),
            "catastrophic_jump_count": (rq["catastrophic_jump_count"], rq["catastrophic_jump_count"] <= GATE["catastrophic_jump_count_max"]),
        }
        all_pass = all(v[1] for v in checks.values())
        result = {
            "config": config, "selected_window_s": best_window["window_s"],
            "range_quality": rq, "direction_by_window": direction_by_window,
            "bias_reduction": br, "checks": {k: {"value": v[0], "passed": bool(v[1])} for k, v in checks.items()},
            "all_pass": all_pass,
        }
        all_results.append(result)
        (OUT / f"gate_result_{config}.json").write_text(json.dumps(result, indent=2) + "\n")

    write_csv(OUT / "bias_reduction_summary.csv", bias_reduction_rows)
    write_csv(OUT / "control_ready_metrics_by_config.csv", [
        {"config": r["config"], "selected_window_s": r["selected_window_s"], **r["range_quality"],
         "bias_reduction_fraction": r["bias_reduction"]["mean_reduction_fraction"],
         "overall_direction_agreement": r["direction_by_window"][r["selected_window_s"]]["overall_direction_agreement"],
         "approaching_direction_agreement": r["direction_by_window"][r["selected_window_s"]]["approaching_direction_agreement"],
         "receding_direction_agreement": r["direction_by_window"][r["selected_window_s"]]["receding_direction_agreement"],
         "all_pass": r["all_pass"]}
        for r in all_results
    ])

    passing = [r for r in all_results if r["all_pass"]]
    selection = {
        "candidates_evaluated": [r["config"] for r in all_results],
        "passing_configs": [r["config"] for r in passing],
        "selected_config": passing[0]["config"] if passing else None,
        "conclusion": "RESIDUAL_BIAS_CORRECTOR_PASS" if passing else "RESIDUAL_BIAS_CORRECTOR_GATE_FAILED",
    }
    (OUT / "residual_corrector_selection.json").write_text(json.dumps(selection, indent=2) + "\n")
    print(json.dumps(selection, indent=2))
    print(json.dumps([{"config": r["config"], "checks": {k: v["value"] for k, v in r["checks"].items()}, "all_pass": r["all_pass"]} for r in all_results], indent=2))


if __name__ == "__main__":
    main()
