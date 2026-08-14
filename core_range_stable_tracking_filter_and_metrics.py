"""CORE_RANGE_STABLE_RELATIVE_TRACKING_3_12M, Phase 8-10.

Applies the causal alpha-beta filter (with a soft innovation limiter) to
every (candidate x calibration) combination's out-of-fold predictions,
sweeping the precommitted small alpha/beta/threshold grid, then computes
the stability-focused Phase 9 metrics for raw / calibrated / filtered
outputs and the Phase 10 composite stability score + selection.
"""

from __future__ import annotations

import csv
import json
from pathlib import Path
from statistics import mean, median, pstdev

import numpy as np

import core_range_stable_relative_tracking as s

OUT = s.OUT
CLIP = s.CLIP_BOUNDS
MOTION_NOISE_THRESHOLD_M = 0.05  # GT deltas below this are not scored for direction agreement
FRAME_JUMP_SMALL_GT_DELTA_M = 0.5
CATASTROPHIC_JUMP_PRED_DELTA_M = 1.5

ALPHA_GRID = (0.35, 0.50, 0.65)
BETA_GRID = (0.03, 0.08, 0.15)
THRESHOLD_GRID = (1.5, 2.0)


def load_prediction_rows() -> list[dict]:
    with (OUT / "prediction_rows.csv").open() as f:
        rows = list(csv.DictReader(f))
    for r in rows:
        r["fold"] = int(r["fold"])
        r["measurement_timestamp_s"] = float(r["measurement_timestamp_s"])
        r["ground_truth_range_m"] = float(r["ground_truth_range_m"])
        r["raw_physical_range_m"] = float(r["raw_physical_range_m"])
        r["pred_none_m"] = float(r["pred_none_m"])
        r["pred_affine_m"] = float(r["pred_affine_m"])
        r["pred_isotonic_m"] = float(r["pred_isotonic_m"])
    return rows


def alpha_beta_filter(
    session_rows: list[dict], measurement_key: str, alpha: float, beta: float, threshold_m: float
) -> np.ndarray:
    """session_rows must already be sorted by measurement_timestamp_s.
    Causal only: uses only the current and strictly-past frames of this
    one session. Resets state at the start of every session (this
    function is called once per session)."""
    n = len(session_rows)
    out = np.empty(n, dtype=np.float64)
    distance = None
    rate = 0.0
    last_t = None
    for i, row in enumerate(session_rows):
        z = float(row[measurement_key])
        t = row["measurement_timestamp_s"]
        if distance is None:
            distance = z
            rate = 0.0
            last_t = t
            out[i] = float(np.clip(distance, *CLIP))
            continue
        dt = max(t - last_t, 1e-3)
        predicted = distance + rate * dt
        innovation = z - predicted
        limited_innovation = float(np.clip(innovation, -threshold_m, threshold_m))
        distance = predicted + alpha * limited_innovation
        rate = rate + beta * limited_innovation / dt
        distance = float(np.clip(distance, *CLIP))
        out[i] = distance
        last_t = t
    return out


def apply_filter_all_sessions(rows: list[dict], measurement_key: str, alpha: float, beta: float, threshold_m: float) -> np.ndarray:
    by_group: dict[str, list[int]] = {}
    for i, r in enumerate(rows):
        by_group.setdefault(r["group_id"], []).append(i)
    out = np.empty(len(rows), dtype=np.float64)
    for group_id, indices in by_group.items():
        ordered = sorted(indices, key=lambda i: rows[i]["measurement_timestamp_s"])
        session_rows = [rows[i] for i in ordered]
        filtered = alpha_beta_filter(session_rows, measurement_key, alpha, beta, threshold_m)
        for pos, i in enumerate(ordered):
            out[i] = filtered[pos]
    return out


# ---------------------------------------------------------------------------
# Phase 9 metrics
# ---------------------------------------------------------------------------

def direction_metrics(rows: list[dict], pred: np.ndarray) -> dict:
    by_group: dict[str, list[int]] = {}
    for i, r in enumerate(rows):
        if r["domain"] != "dynamic":
            continue
        by_group.setdefault(r["group_id"], []).append(i)
    agreements = []
    approaching_agreements = []
    receding_agreements = []
    wrong_streaks = []
    for group_id, indices in by_group.items():
        ordered = sorted(indices, key=lambda i: rows[i]["measurement_timestamp_s"])
        gt = np.asarray([rows[i]["ground_truth_range_m"] for i in ordered])
        pr = pred[ordered]
        gt_delta = np.diff(gt)
        pr_delta = np.diff(pr)
        mask = np.abs(gt_delta) >= MOTION_NOISE_THRESHOLD_M
        if not np.any(mask):
            continue
        gt_dir = np.sign(gt_delta[mask])
        pr_dir = np.sign(pr_delta[mask])
        agree = gt_dir == pr_dir
        agreements.extend(agree.tolist())
        approaching_agreements.extend(agree[gt_dir < 0].tolist())
        receding_agreements.extend(agree[gt_dir > 0].tolist())
        streak = 0
        longest = 0
        for a in agree:
            streak = 0 if a else streak + 1
            longest = max(longest, streak)
        wrong_streaks.append(longest)
    return {
        "overall_direction_agreement": round(mean(agreements), 4) if agreements else None,
        "approaching_direction_agreement": round(mean(approaching_agreements), 4) if approaching_agreements else None,
        "receding_direction_agreement": round(mean(receding_agreements), 4) if receding_agreements else None,
        "wrong_direction_segment_count": int(sum(1 for a in agreements if not a)),
        "longest_wrong_direction_streak": max(wrong_streaks) if wrong_streaks else 0,
        "total_segments_scored": len(agreements),
    }


def rank_correlation_metrics(rows: list[dict], pred: np.ndarray) -> dict:
    from scipy.stats import spearmanr

    dynamic_idx = [i for i, r in enumerate(rows) if r["domain"] == "dynamic"]
    gt_all = np.asarray([rows[i]["ground_truth_range_m"] for i in dynamic_idx])
    pr_all = pred[dynamic_idx]
    overall_rho, _ = spearmanr(pr_all, gt_all)

    by_scenario = {}
    for scenario in ("approaching", "receding"):
        idx = [i for i in dynamic_idx if rows[i]["scenario_type"] == scenario]
        if len(idx) >= 3:
            gt = np.asarray([rows[i]["ground_truth_range_m"] for i in idx])
            pr = pred[idx]
            rho, _ = spearmanr(pr, gt)
            by_scenario[scenario] = round(float(rho), 4)

    by_session = {}
    collapsed_sessions = []
    by_group: dict[str, list[int]] = {}
    for i in dynamic_idx:
        by_group.setdefault(rows[i]["group_id"], []).append(i)
    for group_id, idx in by_group.items():
        gt = np.asarray([rows[i]["ground_truth_range_m"] for i in idx])
        pr = pred[idx]
        if np.std(pr) < 0.02:
            collapsed_sessions.append(group_id)
            by_session[group_id] = None
            continue
        rho, _ = spearmanr(pr, gt)
        by_session[group_id] = round(float(rho), 4)

    return {
        "overall_spearman": round(float(overall_rho), 4),
        "approaching_spearman": by_scenario.get("approaching"),
        "receding_spearman": by_scenario.get("receding"),
        "per_session_spearman": by_session,
        "collapsed_session_count": len(collapsed_sessions),
        "collapsed_sessions": collapsed_sessions,
    }


def stationary_metrics(rows: list[dict], pred: np.ndarray) -> dict:
    """stop_and_hold post-stop segments + static groups (both genuinely
    stationary-target regimes)."""
    stds = []
    successive_diffs = []
    ptp_values = []
    by_group: dict[str, list[int]] = {}
    for i, r in enumerate(rows):
        if r["domain"] == "static" or r["scenario_type"] == "stop_and_hold":
            by_group.setdefault(r["group_id"], []).append(i)
    for group_id, indices in by_group.items():
        ordered = sorted(indices, key=lambda i: rows[i]["measurement_timestamp_s"])
        if len(ordered) < 3:
            continue
        pr = pred[ordered]
        stds.append(float(np.std(pr)))
        diffs = np.abs(np.diff(pr))
        successive_diffs.extend(diffs.tolist())
        ptp_values.append(float(np.ptp(pr)))
    return {
        "stationary_std_m": round(mean(stds), 4) if stds else None,
        "median_abs_successive_diff_m": round(median(successive_diffs), 4) if successive_diffs else None,
        "p95_abs_successive_diff_m": round(float(np.percentile(successive_diffs, 95)), 4) if successive_diffs else None,
        "mean_peak_to_peak_m": round(mean(ptp_values), 4) if ptp_values else None,
        "max_peak_to_peak_m": round(max(ptp_values), 4) if ptp_values else None,
        "groups_scored": len(stds),
    }


def frame_jump_metrics(rows: list[dict], pred: np.ndarray) -> dict:
    by_group: dict[str, list[int]] = {}
    for i, r in enumerate(rows):
        by_group.setdefault(r["group_id"], []).append(i)
    small_gt_jumps = []
    real_gt_jumps = []
    catastrophic = 0
    for group_id, indices in by_group.items():
        ordered = sorted(indices, key=lambda i: rows[i]["measurement_timestamp_s"])
        if len(ordered) < 2:
            continue
        gt = np.asarray([rows[i]["ground_truth_range_m"] for i in ordered])
        pr = pred[ordered]
        gt_delta = np.abs(np.diff(gt))
        pr_delta = np.abs(np.diff(pr))
        for gd, pd in zip(gt_delta, pr_delta):
            if gd < FRAME_JUMP_SMALL_GT_DELTA_M:
                small_gt_jumps.append(pd)
                if pd > CATASTROPHIC_JUMP_PRED_DELTA_M:
                    catastrophic += 1
            else:
                real_gt_jumps.append(pd)
    return {
        "p95_frame_jump_small_gt_delta_m": round(float(np.percentile(small_gt_jumps, 95)), 4) if small_gt_jumps else None,
        "p95_frame_jump_real_gt_delta_m": round(float(np.percentile(real_gt_jumps, 95)), 4) if real_gt_jumps else None,
        "catastrophic_jump_count": catastrophic,
        "small_gt_delta_frame_count": len(small_gt_jumps),
        "real_gt_delta_frame_count": len(real_gt_jumps),
    }


def per_bin_metrics(rows: list[dict], pred: np.ndarray) -> list[dict]:
    out = []
    for label, lo, hi in s.DISTANCE_BINS:
        idx = [i for i, r in enumerate(rows) if lo <= r["ground_truth_range_m"] < hi]
        if not idx:
            continue
        gt = np.asarray([rows[i]["ground_truth_range_m"] for i in idx])
        pr = pred[idx]
        residual = pr - gt
        groups = {rows[i]["group_id"] for i in idx}
        out.append({
            "distance_bin": label, "sample_count": len(idx), "group_count": len(groups),
            "mae_m": round(float(np.mean(np.abs(residual))), 4), "bias_m": round(float(np.mean(residual)), 4),
            "std_m": round(float(np.std(residual)), 4),
            "prediction_std_within_bin": round(float(np.std(pr)), 4),
        })
    return out


def dynamic_quality_metrics(rows: list[dict], pred: np.ndarray) -> dict:
    by_group: dict[str, list[int]] = {}
    for i, r in enumerate(rows):
        if r["domain"] == "dynamic":
            by_group.setdefault(r["group_id"], []).append(i)
    by_scenario: dict[str, list[float]] = {}
    session_maes = {}
    for group_id, idx in by_group.items():
        gt = np.asarray([rows[i]["ground_truth_range_m"] for i in idx])
        pr = pred[idx]
        mae = float(np.mean(np.abs(pr - gt)))
        session_maes[group_id] = round(mae, 4)
        scenario = rows[idx[0]]["scenario_type"]
        by_scenario.setdefault(scenario, []).append(mae)
    return {
        "dynamic_equal_group_mae_m": round(mean(session_maes.values()), 4) if session_maes else None,
        "approaching_mae_m": round(mean(by_scenario.get("approaching", [0])), 4) if by_scenario.get("approaching") else None,
        "receding_mae_m": round(mean(by_scenario.get("receding", [0])), 4) if by_scenario.get("receding") else None,
        "stop_and_hold_mae_m": round(mean(by_scenario.get("stop_and_hold", [0])), 4) if by_scenario.get("stop_and_hold") else None,
        "worst_session_mae_m": round(max(session_maes.values()), 4) if session_maes else None,
        "worst_session": max(session_maes, key=session_maes.get) if session_maes else None,
        "per_session_mae_m": session_maes,
    }


if __name__ == "__main__":
    print("shared module for the sweep driver")
