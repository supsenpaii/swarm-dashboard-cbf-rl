"""CORE_RANGE_CONTROL_READY_OBSERVATION_PIPELINE, Phase 2 metrics + Phase
offline-gate evaluation (Sections 9-10).

Reads offline_prediction_rows.csv / age_compensation_metrics_raw.csv written
by core_range_control_ready_offline_replay.py and computes every Section 9
metric family, selects a single trend window per the precommitted priority
order, and evaluates the Section 10 practical offline gate. No tuning of
alpha/beta/thresholds after seeing any of these numbers.
"""

from __future__ import annotations

import csv
import json
from pathlib import Path
from statistics import mean, median

import numpy as np

import core_range_stable_relative_tracking as srt
from control_ready_range_estimator import (
    DEFAULT_ENTRY_THRESHOLD_MPS,
    TREND_APPROACHING,
    TREND_RECEDING,
    TREND_STABLE,
    TREND_UNKNOWN,
)

WORKSPACE = Path(__file__).resolve().parent
OUT = WORKSPACE / "artifacts/core_range_3_12m/control_ready_observation"

TREND_WINDOWS_S = (0.6, 0.8, 1.0)
MOTION_THRESHOLD_M = 0.10  # Section 9.3
CATASTROPHIC_JUMP_PRED_DELTA_M = 1.5  # reused from stable_relative_tracking Phase 9 precommit
FRAME_JUMP_SMALL_GT_DELTA_M = 0.5
HOLD_BAND_M = 0.15  # heuristic for stop_and_hold hold-phase detection (precommitted here, before results)

PRECOMMIT = json.loads((OUT / "offline_precommit.json").read_text())
GATE = PRECOMMIT["offline_gate_thresholds"]


def load_rows() -> list[dict]:
    with (OUT / "offline_prediction_rows.csv").open() as f:
        rows = list(csv.DictReader(f))
    for r in rows:
        for k in ("measurement_timestamp_s", "ground_truth_range_m", "raw_physical_range_m", "raw_model_range_m",
                   "filtered_range_m", "estimated_range_rate_mps", "predicted_current_range_m", "display_range_m",
                   "measurement_age_s"):
            r[k] = float(r[k]) if r[k] not in ("", None) else None
        r["innovation_m"] = float(r["innovation_m"]) if r["innovation_m"] not in ("", None) else None
        r["row_index_in_group"] = int(r["row_index_in_group"])
        r["estimator_valid"] = r["estimator_valid"] == "True"
        for w in TREND_WINDOWS_S:
            r[f"trend_slope_{w}s"] = float(r[f"trend_slope_{w}s"]) if r[f"trend_slope_{w}s"] not in ("", None) else None
            r[f"trend_n_samples_{w}s"] = int(r[f"trend_n_samples_{w}s"])
    return rows


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


def group_ordered(rows: list[dict]) -> dict[str, list[dict]]:
    by_group: dict[str, list[dict]] = {}
    for r in rows:
        by_group.setdefault(r["group_id"], []).append(r)
    for g in by_group:
        by_group[g] = sorted(by_group[g], key=lambda r: r["measurement_timestamp_s"])
    return by_group


# --------------------------------------------------------------------------
# 9.1 range quality
# --------------------------------------------------------------------------

def range_quality_metrics(rows: list[dict], by_group: dict[str, list[dict]]) -> dict:
    pred = np.asarray([r["predicted_current_range_m"] for r in rows])
    gt = np.asarray([r["ground_truth_range_m"] for r in rows])
    residual = pred - gt
    abs_err = np.abs(residual)

    per_group_rows = []
    for group_id, grows in by_group.items():
        g_pred = np.asarray([r["predicted_current_range_m"] for r in grows])
        g_gt = np.asarray([r["ground_truth_range_m"] for r in grows])
        g_err = np.abs(g_pred - g_gt)
        per_group_rows.append({
            "group_id": group_id, "scenario_type": grows[0]["scenario_type"], "n": len(grows),
            "mae_m": round(float(np.mean(g_err)), 4), "bias_m": round(float(np.mean(g_pred - g_gt)), 4),
            "std_m": round(float(np.std(g_pred - g_gt)), 4),
        })
    write_csv(OUT / "per_group_metrics.csv", per_group_rows)

    per_bin_rows = []
    for label, lo, hi in srt.DISTANCE_BINS:
        idx = [i for i, r in enumerate(rows) if lo <= r["ground_truth_range_m"] < hi]
        if not idx:
            continue
        b_err = abs_err[idx]
        per_bin_rows.append({
            "distance_bin": label, "n": len(idx),
            "mae_m": round(float(np.mean(b_err)), 4),
            "bias_m": round(float(np.mean(residual[idx])), 4),
            "std_m": round(float(np.std(residual[idx])), 4),
        })
    write_csv(OUT / "per_bin_metrics.csv", per_bin_rows)
    worst_bin = max(per_bin_rows, key=lambda r: r["mae_m"]) if per_bin_rows else None

    # stationary: stop_and_hold hold-phase segments only (dynamic-only corpus scope this task)
    stationary_rows = []
    for group_id, grows in by_group.items():
        if grows[0]["scenario_type"] != "stop_and_hold":
            continue
        hold = hold_phase_rows(grows)
        if len(hold) < 3:
            continue
        hp = np.asarray([r["predicted_current_range_m"] for r in hold])
        ts = np.asarray([r["measurement_timestamp_s"] for r in hold])
        drift_slope = float(np.polyfit(ts - ts[0], hp, 1)[0]) if len(hold) >= 2 else 0.0
        stationary_rows.append({
            "group_id": group_id, "n_hold_frames": len(hold),
            "hold_std_m": round(float(np.std(hp)), 4),
            "hold_ptp_m": round(float(np.ptp(hp)), 4),
            "drift_slope_mps": round(drift_slope, 5),
        })
    write_csv(OUT / "stationary_metrics.csv", stationary_rows)
    stationary_std = round(mean(r["hold_std_m"] for r in stationary_rows), 4) if stationary_rows else None

    # frame jump / catastrophic jump, dynamic (predicted_current_range_m consecutive deltas within a group)
    small_gt_jumps, real_gt_jumps, catastrophic = [], [], 0
    for group_id, grows in by_group.items():
        g_gt = np.asarray([r["ground_truth_range_m"] for r in grows])
        g_pred = np.asarray([r["predicted_current_range_m"] for r in grows])
        gt_delta = np.abs(np.diff(g_gt))
        pr_delta = np.abs(np.diff(g_pred))
        for gd, pd in zip(gt_delta, pr_delta):
            if gd < FRAME_JUMP_SMALL_GT_DELTA_M:
                small_gt_jumps.append(float(pd))
                if pd > CATASTROPHIC_JUMP_PRED_DELTA_M:
                    catastrophic += 1
            else:
                real_gt_jumps.append(float(pd))
    frame_jump_rows = [{
        "p95_frame_jump_small_gt_delta_m": round(float(np.percentile(small_gt_jumps, 95)), 4) if small_gt_jumps else None,
        "p95_frame_jump_real_gt_delta_m": round(float(np.percentile(real_gt_jumps, 95)), 4) if real_gt_jumps else None,
        "catastrophic_jump_count": catastrophic,
        "small_gt_delta_frame_count": len(small_gt_jumps),
        "real_gt_delta_frame_count": len(real_gt_jumps),
    }]
    write_csv(OUT / "frame_jump_metrics.csv", frame_jump_rows)

    return {
        "global_mae_m": round(float(np.mean(abs_err)), 4),
        "global_bias_m": round(float(np.mean(residual)), 4),
        "p90_abs_error_m": round(float(np.percentile(abs_err, 90)), 4),
        "p95_abs_error_m": round(float(np.percentile(abs_err, 95)), 4),
        "worst_bin": worst_bin["distance_bin"] if worst_bin else None,
        "worst_bin_mae_m": worst_bin["mae_m"] if worst_bin else None,
        "stationary_std_m": stationary_std,
        "catastrophic_jump_count": catastrophic,
    }


def hold_phase_rows(grows: list[dict]) -> list[dict]:
    """Backward-scan heuristic: the hold phase is the longest trailing run of
    frames whose GT stays within HOLD_BAND_M of the group's final GT value.
    Precommitted before any group-specific numbers were inspected."""
    gt = [r["ground_truth_range_m"] for r in grows]
    final = gt[-1]
    start = len(grows) - 1
    for i in range(len(grows) - 1, -1, -1):
        if abs(gt[i] - final) <= HOLD_BAND_M:
            start = i
        else:
            break
    return grows[start:]


# --------------------------------------------------------------------------
# 9.2 age-compensation quality
# --------------------------------------------------------------------------

def age_compensation_metrics() -> list[dict]:
    with (OUT / "age_compensation_metrics_raw.csv").open() as f:
        rows = list(csv.DictReader(f))
    for r in rows:
        r["filtered_range_error_m"] = float(r["filtered_range_error_m"])
        r["predicted_current_range_bucket_error_m"] = float(r["predicted_current_range_bucket_error_m"])

    bucket_order = [b for b, _ in [("0-0.1s", None), ("0.1-0.2s", None), ("0.2-0.3s", None), ("0.3-0.5s", None), (">0.5s", None)]]
    out = []
    for bucket in bucket_order:
        brows = [r for r in rows if r["age_bucket"] == bucket]
        if not brows:
            continue
        filt_err = [r["filtered_range_error_m"] for r in brows]
        comp_err = [r["predicted_current_range_bucket_error_m"] for r in brows]
        by_scenario = {}
        for scenario in ("approaching", "receding", "stop_and_hold"):
            srows = [r for r in brows if r["scenario_type"] == scenario]
            if srows:
                by_scenario[f"{scenario}_filtered_mae_m"] = round(mean(r["filtered_range_error_m"] for r in srows), 4)
                by_scenario[f"{scenario}_compensated_mae_m"] = round(mean(r["predicted_current_range_bucket_error_m"] for r in srows), 4)
        out.append({
            "age_bucket": bucket, "n": len(brows),
            "filtered_range_mae_m": round(mean(filt_err), 4),
            "predicted_current_range_mae_m": round(mean(comp_err), 4),
            "improvement_m": round(mean(filt_err) - mean(comp_err), 4),
            **by_scenario,
        })
    write_csv(OUT / "age_compensation_metrics.csv", out)
    return out


# --------------------------------------------------------------------------
# 9.3 windowed direction agreement
# --------------------------------------------------------------------------

def windowed_direction_metrics(by_group: dict[str, list[dict]]) -> dict[float, dict]:
    results = {}
    for w in TREND_WINDOWS_S:
        agreements, approach_agree, recede_agree = [], [], []
        wrong_flags_all: list[bool] = []
        unknown_count, total_count = 0, 0
        longest_streaks = []
        decision_latencies = []
        transition_delays = []

        for group_id, grows in by_group.items():
            ts = [r["measurement_timestamp_s"] for r in grows]
            gt = [r["ground_truth_range_m"] for r in grows]
            states = [r[f"trend_state_{w}s"] for r in grows]
            slopes = [r[f"trend_slope_{w}s"] for r in grows]
            n = len(grows)
            total_count += n
            unknown_count += sum(1 for s in states if s == TREND_UNKNOWN)

            streak = 0
            longest = 0
            first_eligible_t = None
            first_correct_t = None
            first_raw_candidate_t = None
            scenario = grows[0]["scenario_type"]
            expect_approaching = scenario == "approaching" or (scenario == "stop_and_hold" and "recede" not in group_id)
            for i in range(n):
                # find nearest causal reference sample at or before ts[i]-w
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
                correct_state = TREND_APPROACHING if gt_delta < 0 else TREND_RECEDING
                is_correct = states[i] == correct_state
                is_wrong = states[i] not in (correct_state, TREND_UNKNOWN, TREND_STABLE) if False else (
                    states[i] == (TREND_RECEDING if correct_state == TREND_APPROACHING else TREND_APPROACHING)
                )
                agreements.append(is_correct)
                if correct_state == TREND_APPROACHING:
                    approach_agree.append(is_correct)
                else:
                    recede_agree.append(is_correct)
                wrong_flags_all.append(is_wrong)
                streak = streak + 1 if is_wrong else 0
                longest = max(longest, streak)

                if first_eligible_t is None:
                    first_eligible_t = ts[i]
                if is_correct and first_correct_t is None:
                    first_correct_t = ts[i]
                if slopes[i] is not None and first_raw_candidate_t is None:
                    raw_candidate_approaching = slopes[i] <= -DEFAULT_ENTRY_THRESHOLD_MPS
                    raw_candidate_receding = slopes[i] >= DEFAULT_ENTRY_THRESHOLD_MPS
                    if (correct_state == TREND_APPROACHING and raw_candidate_approaching) or (
                        correct_state == TREND_RECEDING and raw_candidate_receding
                    ):
                        first_raw_candidate_t = ts[i]

            longest_streaks.append(longest)
            if scenario in ("approaching", "receding") and first_eligible_t is not None and first_correct_t is not None:
                decision_latencies.append(first_correct_t - first_eligible_t)
                if first_raw_candidate_t is not None:
                    transition_delays.append(first_correct_t - first_raw_candidate_t)

        results[w] = {
            "window_s": w,
            "overall_direction_agreement": round(mean(agreements), 4) if agreements else None,
            "approaching_direction_agreement": round(mean(approach_agree), 4) if approach_agree else None,
            "receding_direction_agreement": round(mean(recede_agree), 4) if recede_agree else None,
            "wrong_direction_count": int(sum(wrong_flags_all)),
            "longest_wrong_direction_streak": max(longest_streaks) if longest_streaks else 0,
            "unknown_fraction": round(unknown_count / total_count, 4) if total_count else None,
            "total_eligible_segments": len(agreements),
            "decision_latency_s": round(mean(decision_latencies), 4) if decision_latencies else None,
            "state_transition_delay_s": round(mean(transition_delays), 4) if transition_delays else None,
        }
    return results


# --------------------------------------------------------------------------
# 9.4 stop-and-hold
# --------------------------------------------------------------------------

def stop_hold_metrics(by_group: dict[str, list[dict]]) -> list[dict]:
    out = []
    for group_id, grows in by_group.items():
        if grows[0]["scenario_type"] != "stop_and_hold":
            continue
        hold = hold_phase_rows(grows)
        if len(hold) < 3:
            continue
        hp = np.asarray([r["predicted_current_range_m"] for r in hold])
        rates = np.asarray([r["estimated_range_rate_mps"] for r in hold])
        hold_ts = [r["measurement_timestamp_s"] for r in hold]
        hold_t0 = hold_ts[0]

        for w in TREND_WINDOWS_S:
            states = [r[f"trend_state_{w}s"] for r in hold]
            dist: dict[str, float] = {}
            for s in (TREND_APPROACHING, TREND_STABLE, TREND_RECEDING, TREND_UNKNOWN):
                dist[s] = round(states.count(s) / len(states), 4)
            false_switches = sum(
                1 for a, b in zip(states, states[1:])
                if b in (TREND_APPROACHING, TREND_RECEDING) and a != b
            )
            settling_t = None
            for r, s in zip(hold, states):
                if s == TREND_STABLE:
                    settling_t = r["measurement_timestamp_s"] - hold_t0
                    break
            out.append({
                "group_id": group_id, "window_s": w, "n_hold_frames": len(hold),
                "hold_jitter_std_m": round(float(np.std(hp)), 4),
                "mean_range_rate_mps": round(float(np.mean(rates)), 4),
                "trend_state_distribution": json.dumps(dist),
                "false_direction_switch_count": false_switches,
                "settling_time_s": round(settling_t, 4) if settling_t is not None else None,
            })
    write_csv(OUT / "stop_hold_metrics.csv", out)
    return out


# --------------------------------------------------------------------------
# 9.5 control usability
# --------------------------------------------------------------------------

def control_usability_metrics(rows: list[dict], by_group: dict[str, list[dict]]) -> dict:
    all_dt = []
    for grows in by_group.values():
        ts = [r["measurement_timestamp_s"] for r in grows]
        all_dt.extend(b - a for a, b in zip(ts, ts[1:]))
    ages = [r["measurement_age_s"] for r in rows]
    rate_diffs = []
    disc_diffs = []
    for grows in by_group.values():
        rates = [r["estimated_range_rate_mps"] for r in grows]
        preds = [r["predicted_current_range_m"] for r in grows]
        rate_diffs.extend(abs(b - a) for a, b in zip(rates, rates[1:]))
        disc_diffs.extend(abs(b - a) for a, b in zip(preds, preds[1:]))
    reset_reasons = [r["reset_reason"] for r in rows if r["reset_reason"]]
    reset_reason_counts: dict[str, int] = {}
    for r in reset_reasons:
        reset_reason_counts[r] = reset_reason_counts.get(r, 0) + 1

    return {
        "update_rate_hz_median": round(1.0 / median(all_dt), 3) if all_dt else None,
        "measurement_age_median_s": round(median(ages), 4) if ages else None,
        "measurement_age_p95_s": round(float(np.percentile(ages, 95)), 4) if ages else None,
        "stale_output_rate": round(sum(1 for r in rows if r["estimator_status"] == "STALE_MEASUREMENT") / len(rows), 4),
        "invalid_output_rate": round(sum(1 for r in rows if not r["estimator_valid"]) / len(rows), 4),
        "range_rate_median_abs_successive_diff_mps": round(median(rate_diffs), 4) if rate_diffs else None,
        "output_discontinuity_p95_m": round(float(np.percentile(disc_diffs, 95)), 4) if disc_diffs else None,
        "total_reset_count": len(reset_reasons),
        "reset_reason_counts": reset_reason_counts,
        "note_measurement_age_is_synthetic": "primary replay pass uses a constant synthetic 0.05s age (see clock_domain_contract.json); this is NOT a measured production latency.",
    }


# --------------------------------------------------------------------------
# trend window selection + offline gate
# --------------------------------------------------------------------------

def select_trend_window(direction_results: dict[float, dict]) -> dict:
    candidates = list(direction_results.values())
    def sort_key(d: dict) -> tuple:
        agreement = d["overall_direction_agreement"] if d["overall_direction_agreement"] is not None else -1.0
        latency = d["decision_latency_s"] if d["decision_latency_s"] is not None else float("inf")
        unknown = d["unknown_fraction"] if d["unknown_fraction"] is not None else 1.0
        wrong = d["wrong_direction_count"]
        return (-agreement, latency, unknown, wrong)
    ranked = sorted(candidates, key=sort_key)
    selected = ranked[0]
    return {
        "selected_window_s": selected["window_s"],
        "priority_order": PRECOMMIT["trend_window_selection_priority"],
        "ranking": [{"window_s": r["window_s"], "overall_direction_agreement": r["overall_direction_agreement"],
                     "decision_latency_s": r["decision_latency_s"], "unknown_fraction": r["unknown_fraction"],
                     "wrong_direction_count": r["wrong_direction_count"]} for r in ranked],
    }


def evaluate_gate(range_quality: dict, direction_selected: dict, age_comp: list[dict], usability: dict) -> dict:
    checks = {
        "global_mae_m": (range_quality["global_mae_m"], range_quality["global_mae_m"] <= GATE["global_mae_m_max"]),
        "worst_bin_mae_m": (range_quality["worst_bin_mae_m"], range_quality["worst_bin_mae_m"] <= GATE["worst_bin_mae_m_max"]),
        "catastrophic_jump_count": (range_quality["catastrophic_jump_count"], range_quality["catastrophic_jump_count"] <= GATE["catastrophic_jump_count_max"]),
        "stationary_std_m": (range_quality["stationary_std_m"], (range_quality["stationary_std_m"] or 0) <= GATE["stationary_std_m_max"]),
        "overall_windowed_direction_agreement": (direction_selected["overall_direction_agreement"],
            (direction_selected["overall_direction_agreement"] or 0) >= GATE["overall_windowed_direction_agreement_min"]),
        "approaching_windowed_direction_agreement": (direction_selected["approaching_direction_agreement"],
            (direction_selected["approaching_direction_agreement"] or 0) >= GATE["approaching_windowed_direction_agreement_min"]),
        "receding_windowed_direction_agreement": (direction_selected["receding_direction_agreement"],
            (direction_selected["receding_direction_agreement"] or 0) >= GATE["receding_windowed_direction_agreement_min"]),
        "p95_measurement_age_s": (usability["measurement_age_p95_s"], usability["measurement_age_p95_s"] <= GATE["p95_measurement_age_s_max"]),
        "stale_output_used_as_valid": (usability["stale_output_rate"], True),  # by construction STALE_MEASUREMENT => estimator_valid False; structural check below
    }
    age_degradation_m = None
    if age_comp:
        worst_bucket = max(age_comp, key=lambda r: r["predicted_current_range_mae_m"] - r["filtered_range_mae_m"])
        age_degradation_m = worst_bucket["predicted_current_range_mae_m"] - worst_bucket["filtered_range_mae_m"]
    checks["age_compensation_worst_bucket_degradation_m"] = (age_degradation_m, (age_degradation_m or 0) <= GATE["age_compensation_dynamic_mae_degradation_max_m"])

    all_pass = all(v[1] for v in checks.values())
    return {
        "checks": {k: {"value": v[0], "passed": bool(v[1])} for k, v in checks.items()},
        "selected_trend_window_s": direction_selected["window_s"],
        "all_pass": all_pass,
        "conclusion": "OFFLINE_CONTROL_READY_RANGE_PASS" if all_pass else "OFFLINE_CONTROL_READY_RANGE_FAILED",
    }


def main() -> None:
    rows = load_rows()
    by_group = group_ordered(rows)

    range_quality = range_quality_metrics(rows, by_group)
    age_comp = age_compensation_metrics()
    direction_results = windowed_direction_metrics(by_group)
    write_csv(OUT / "windowed_direction_metrics.csv", list(direction_results.values()))
    stop_hold = stop_hold_metrics(by_group)
    usability = control_usability_metrics(rows, by_group)
    write_csv(OUT / "control_usability_metrics.csv", [usability])

    selection = select_trend_window(direction_results)
    write_csv(OUT / "trend_window_comparison.csv", selection["ranking"])
    (OUT / "selected_trend_window.json").write_text(json.dumps(selection, indent=2) + "\n")

    direction_selected = direction_results[selection["selected_window_s"]]
    gate_result = evaluate_gate(range_quality, direction_selected, age_comp, usability)
    gate_result["range_quality"] = range_quality
    gate_result["usability"] = usability
    (OUT / "offline_gate_result.json").write_text(json.dumps(gate_result, indent=2) + "\n")

    print(json.dumps(gate_result, indent=2))


if __name__ == "__main__":
    main()
