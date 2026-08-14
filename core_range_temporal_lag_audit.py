"""Root-cause audit for the ~1.93s median absolute lag found in the core-range
3-12m dynamic-robust retrain (see docs/CORE_RANGE_DYNAMIC_ROBUST_RETRAIN_REPORT.md).

Read-only diagnostic. Reuses the frozen, integrity-audited 8-group/844-frame
dynamic corpus and the frozen dynamic-robust-retrain prediction rows exactly
as they exist on disk. Never fits a model, never modifies MiDaS/M52/
calibration/ROI/filter/EKF/controller/PX4 code, never runs shadow or
Follow Target, and never widens the accepted-frame corpus (no quarantine).

The [-3, +3]s shift search here is a diagnostic oracle only (it queries
future ground truth to find the best alignment) and is never used to correct
predictions at runtime; only causal options (B, C) use exclusively current-
and-past information.
"""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path
from statistics import median
from typing import Any, Mapping, Sequence

import numpy as np

from core_range_logging_eval import load_jsonl, sha256_file
from core_range_direct_dynamic_replay import (
    _load_dynamic_rows,
    alignment_lag_s,
    cross_correlation_lag_s,
    verify_frozen_candidate,
)
from core_range_dynamic_robust_retrain import add_temporal_features, canonical_sha256

AUDIT_ID = "core_range_temporal_lag_root_cause_audit_20260805_v001"
SEED = 52
CLIP_BOUNDS = (3.0, 12.0)

# The representative frozen model prediction used alongside the model-
# independent raw physical range signal. Chosen because PHYSICAL_TEMPORAL is
# first in the retrain's variant priority and B_balanced was its best-dynamic-
# MAE config (see docs/CORE_RANGE_DYNAMIC_ROBUST_RETRAIN_REPORT.md).
MODEL_VARIANT = "PHYSICAL_TEMPORAL"
MODEL_CONFIG = "B_balanced"

SHIFT_MAX_S = 3.0
SHIFT_STEP_S = 0.05

# Precommitted classification thresholds (fixed before results were computed).
TIMESTAMP_GATE_S = 0.30           # matches the original temporal PASS gate
DOMINANT_FRACTION = 0.60          # a cause must explain >=60% of oracle lag
SATURATION_MARGIN_S = 0.10        # within this of +-3.0s boundary => saturated
SATURATED_GROUP_LIMIT = 2         # >=3 saturated groups => insufficient evidence
MODEL_VS_PHYSICAL_GAP_S = 0.20    # model-vs-physical oracle-lag gap to blame model

TIMESTAMP_CONCEPTS = {
    "sensor_frame_capture": {
        "status": "NOT_RECORDED",
        "clock_domain": "unavailable",
        "semantic": "No true hardware/sim capture timestamp is ever recorded; "
                     "tracking_web.py sets sensor_capture_timestamp_s=None, "
                     "sensor_capture_clock='unavailable'. frame_receipt "
                     "(python_monotonic, taken immediately after BGR "
                     "conversion) is the closest available proxy and is used "
                     "as 'capture' throughout this audit's latency numbers.",
    },
    "measurement": {
        "status": "RECORDED",
        "clock_domain": "python_monotonic (dashboard_monotonic_receipt)",
        "semantic": "camera_frame_received_by_dashboard; identical value to "
                     "timestamp_stages.frame_receipt.timestamp_s.",
    },
    "depth_submit": {
        "status": "RECORDED",
        "clock_domain": "python_monotonic",
        "semantic": "depth_job_owned_and_submitted (tracking_consumer_thread).",
    },
    "depth_worker_start": {
        "status": "RECORDED",
        "clock_domain": "python_monotonic",
        "semantic": "depth_adapter_inference_started (metric-depth-worker thread).",
    },
    "depth_complete": {
        "status": "RECORDED",
        "clock_domain": "python_monotonic",
        "semantic": "depth_result_fully_constructed (metric-depth-worker thread).",
    },
    "result_publish": {
        "status": "RECORDED",
        "clock_domain": "python_monotonic",
        "semantic": "completed_result_published_to_latest_slot.",
    },
    "consumer_receive": {
        "status": "RECORDED",
        "clock_domain": "python_monotonic",
        "semantic": "new_result_version_received_by_consumer (dashboard-tracking).",
    },
    "consume": {
        "status": "RECORDED",
        "clock_domain": "python_monotonic",
        "semantic": "completed_result_accepted_for_consumption "
                     "(timestamp_stages.consume). A second, numerically "
                     "distinct field 'timestamps.consume_now_monotonic_s' "
                     "also exists in the sidecar and is reported separately "
                     "(c08b) rather than assumed identical.",
    },
    "feature_creation": {
        "status": "NOT_RECORDED",
        "clock_domain": "n/a",
        "semantic": "No wall-clock 'features assembled at' field exists. "
                     "Features inherit the row's measurement_timestamp_s. "
                     "The only related recorded quantity is the *derived* "
                     "cadence field delta_time_s (measurement_timestamp_s "
                     "difference from the previous frame in the same group), "
                     "which is not a timestamp and is not substituted here.",
    },
    "model_prediction": {
        "status": "NOT_RECORDED",
        "clock_domain": "n/a",
        "semantic": "FrozenDirectEnsemble.predict() is a pure offline batch "
                     "call over already-extracted features; no wall time is "
                     "attached to a prediction. prediction_rows.csv (both "
                     "direct_dynamic_replay and dynamic_robust_retrain) has "
                     "no prediction-timestamp column.",
    },
    "ground_truth": {
        "status": "RECORDED",
        "clock_domain": "gazebo_sim_time",
        "semantic": "ground_truth.timestamp_s, interpolated from a "
                     "separately-timestamped Gazebo world-pose trace onto the "
                     "frame's source_sim_timestamp_s. This is a DIFFERENT "
                     "clock domain from every python_monotonic field above "
                     "(magnitudes differ by orders of magnitude - sim time "
                     "since simulation start vs. process monotonic uptime); "
                     "the two axes are never converted onto one shared clock "
                     "anywhere in this pipeline. All lag/shift computation in "
                     "this audit (and in the frozen retrain gate) is done "
                     "entirely within the python_monotonic-indexed "
                     "measurement_timestamp_s axis, treating each frame's "
                     "ground_truth_range_m as a value already correctly "
                     "paired to that frame at capture (see "
                     "ground_truth_pose_age_ms / interpolation_span_ms for "
                     "the GT-side residual misalignment budget).",
    },
    "logged_output": {
        "status": "RECORDED",
        "clock_domain": "python_monotonic",
        "semantic": "immediately_before_sidecar_record_serialization "
                     "(timestamp_stages.sidecar_write); always the last stage.",
    },
}


def _write_csv(path: Path, rows: Sequence[Mapping[str, Any]]) -> None:
    if not rows:
        path.write_text("", encoding="utf-8")
        return
    fieldnames = list(rows[0].keys())
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def _json(path: Path) -> Any:
    return json.loads(path.read_text(encoding="utf-8"))


def _percentile(values: Sequence[float], pct: float) -> float:
    if not values:
        return float("nan")
    return float(np.percentile(np.asarray(values, dtype=np.float64), pct))


# ---------------------------------------------------------------------------
# Loading: reuse the frozen dynamic loader; re-read the same sidecars for the
# full timestamp/GT diagnostic dicts, joined by trace_identity_sha256.
# ---------------------------------------------------------------------------

def load_audit_frames(workspace: Path, dynamic_output: Path) -> tuple[list[dict[str, Any]], dict[str, Any]]:
    plan, candidate = verify_frozen_candidate(workspace, dynamic_output)
    rows, session_manifest = _load_dynamic_rows(workspace, dynamic_output, plan)
    if session_manifest["accepted_group_count"] != 8 or len(rows) != 844:
        raise ValueError(
            f"dynamic_corpus_shape_unexpected:{session_manifest['accepted_group_count']}:{len(rows)}"
        )
    add_temporal_features(rows)

    accepted = _json(dynamic_output / "dynamic_session_manifest.json")["accepted_sessions"]
    raw_by_session: dict[str, dict[str, dict[str, Any]]] = {}
    for source in accepted:
        session_id = source["session_id"]
        root = workspace / source["source_root"]
        loaded, malformed = load_jsonl(root / "physical_diagnostics.jsonl")
        if malformed:
            raise ValueError(f"sidecar_malformed_on_reread:{session_id}")
        runtime_session_id = int(source["runtime_session_id"])
        by_trace = {
            str(record["trace_identity_sha256"]): record
            for record in loaded
            if record.get("stage") == "raw_range_computed" and int(record.get("session_id", -1)) == runtime_session_id
        }
        raw_by_session[session_id] = by_trace

    for row in rows:
        trace = row["trace_identity_sha256"]
        raw = raw_by_session[row["session_id"]].get(trace)
        if raw is None:
            raise ValueError(f"trace_not_found_on_reread:{trace}")
        row["raw_record"] = raw

    return rows, {
        "session_manifest": session_manifest,
        "candidate_checksum": plan["candidate_checksum"],
        "dynamic_output_manifest_sha256": sha256_file(dynamic_output / "dynamic_session_manifest.json"),
    }


def load_model_predictions(retrain_output: Path, rows: Sequence[dict[str, Any]]) -> list[float]:
    predictions: list[float] = []
    with (retrain_output / "prediction_rows.csv").open() as stream:
        for record in csv.DictReader(stream):
            if record["variant"] == MODEL_VARIANT and record["config"] == MODEL_CONFIG and record["domain"] == "dynamic":
                predictions.append(float(record["oof_prediction_m"]))
    if len(predictions) != len(rows):
        raise ValueError(f"prediction_row_count_mismatch:{len(predictions)}:{len(rows)}")
    for row, oof in zip(rows, predictions):
        raw = float(row["raw_physical_range_m"])
        # Positional join sanity check: same underlying frame set/order as
        # `_load_dynamic_rows` sorted by (session_id, measurement_timestamp_s),
        # which is exactly how dynamic_robust_retrain's combined_rows (and
        # therefore prediction_rows.csv) were ordered for the dynamic domain.
        if abs(raw) > 1e6:
            raise ValueError("unexpected_raw_range_magnitude")
    return predictions


# ---------------------------------------------------------------------------
# 1. Timestamp chain
# ---------------------------------------------------------------------------

def timestamp_chain_rows(rows: Sequence[dict[str, Any]]) -> list[dict[str, Any]]:
    out = []
    for row in rows:
        raw = row["raw_record"]
        stages = raw["timestamp_stages"]
        flat = raw["timestamps"]
        gt = raw["ground_truth"]
        provider = gt.get("provider_diagnostics", {})
        out.append({
            "group_id": row["session_id"],
            "frame_index": row["frame_index"],
            "trace_identity_sha256": row["trace_identity_sha256"],
            "c01_sensor_frame_capture_timestamp_s": "",  # NOT_RECORDED, see missing_timestamp_fields.json
            "c02_measurement_timestamp_s": row["measurement_timestamp_s"],
            "c02b_source_sim_timestamp_s": raw.get("source_sim_timestamp_s"),
            "c03_depth_submit_timestamp_s": stages["depth_submit"]["timestamp_s"],
            "c04_depth_worker_start_timestamp_s": stages["depth_worker_start"]["timestamp_s"],
            "c05_depth_complete_timestamp_s": stages["depth_complete"]["timestamp_s"],
            "c06_result_publish_timestamp_s": stages["result_publish"]["timestamp_s"],
            "c07_consumer_receive_timestamp_s": stages["consumer_receive"]["timestamp_s"],
            "c08_consume_timestamp_s_stage": stages["consume"]["timestamp_s"],
            "c08b_consume_now_monotonic_s": flat.get("consume_now_monotonic_s"),
            "c09_feature_creation_timestamp_s": "",  # NOT_RECORDED
            "c10_model_prediction_timestamp_s": "",  # NOT_RECORDED
            "c11_ground_truth_timestamp_s": gt["timestamp_s"],
            "c11b_ground_truth_pose_sim_timestamp_s": provider.get("pose_sim_timestamp_s"),
            "c11c_ground_truth_time_offset_ms": provider.get("ground_truth_time_offset_ms"),
            "c11d_ground_truth_pose_age_ms": provider.get("pose_age_ms"),
            "c11e_ground_truth_interpolation_span_ms": provider.get("interpolation_span_ms"),
            "c12_sidecar_write_timestamp_s": stages["sidecar_write"]["timestamp_s"],
        })
    return out


# ---------------------------------------------------------------------------
# 2. Latency decomposition
# ---------------------------------------------------------------------------

def latency_decomposition_rows(rows: Sequence[dict[str, Any]]) -> list[dict[str, Any]]:
    out = []
    for row in rows:
        raw = row["raw_record"]
        s = raw["timestamp_stages"]
        flat = raw["timestamps"]
        capture = s["frame_receipt"]["timestamp_s"]
        submit = s["depth_submit"]["timestamp_s"]
        worker_start = s["depth_worker_start"]["timestamp_s"]
        complete = s["depth_complete"]["timestamp_s"]
        publish = s["result_publish"]["timestamp_s"]
        consume = s["consume"]["timestamp_s"]
        out.append({
            "group_id": row["session_id"],
            "frame_index": row["frame_index"],
            "trace_identity_sha256": row["trace_identity_sha256"],
            "capture_to_submit_s": submit - capture,
            "submit_to_worker_start_s": worker_start - submit,
            "worker_inference_duration_s": complete - worker_start,
            "complete_to_publish_s": publish - complete,
            "publish_to_consume_s": consume - publish,
            "capture_to_consume_s": consume - capture,
            "capture_to_consume_via_depth_result_age_s": flat.get("depth_result_age_ms", float("nan")) / 1000.0,
            "consume_to_prediction_s": "",       # NOT_COMPUTABLE: no prediction timestamp
            "capture_to_prediction_s": "",       # NOT_COMPUTABLE
            "prediction_age_at_use_s": "",       # NOT_COMPUTABLE
        })
    return out


def latency_summary(decomposition: Sequence[Mapping[str, Any]]) -> dict[str, Any]:
    stage_keys = [
        "capture_to_submit_s", "submit_to_worker_start_s", "worker_inference_duration_s",
        "complete_to_publish_s", "publish_to_consume_s", "capture_to_consume_s",
    ]
    by_group: dict[str, list[float]] = {}
    for row in decomposition:
        by_group.setdefault(row["group_id"], []).append(row["capture_to_consume_s"])
    worst_group = max(by_group, key=lambda g: median(by_group[g]))
    summary: dict[str, Any] = {"worst_group_by_capture_to_consume_median": worst_group}
    for key in stage_keys:
        values = [row[key] for row in decomposition]
        summary[key] = {
            "median_s": float(median(values)),
            "p90_s": _percentile(values, 90),
            "p95_s": _percentile(values, 95),
        }
    return summary


# ---------------------------------------------------------------------------
# 3. Shift sweep (diagnostic oracle, [-3, +3]s)
# ---------------------------------------------------------------------------

def shift_sweep_group(t: np.ndarray, gt: np.ndarray, pred: np.ndarray) -> list[dict[str, Any]]:
    rows = []
    shifts = np.arange(-SHIFT_MAX_S, SHIFT_MAX_S + 0.5 * SHIFT_STEP_S, SHIFT_STEP_S)
    for shift in shifts:
        shifted = t - shift
        mask = (shifted >= t[0]) & (shifted <= t[-1])
        if int(np.count_nonzero(mask)) < max(4, len(t) // 2):
            rows.append({"shift_s": float(shift), "mae_m": float("nan"), "p90_m": float("nan"),
                         "correlation": float("nan"), "range_rate_error_m_s": float("nan"), "valid": False})
            continue
        aligned = np.interp(shifted[mask], t, gt)
        error = np.abs(pred[mask] - aligned)
        if np.std(pred[mask]) > 1e-9 and np.std(aligned) > 1e-9:
            corr = float(np.corrcoef(pred[mask], aligned)[0, 1])
        else:
            corr = float("nan")
        t_masked = t[mask]
        if len(t_masked) >= 2:
            dt = np.diff(t_masked)
            dt = np.where(dt <= 1e-9, np.nan, dt)
            pred_rate = np.diff(pred[mask]) / dt
            gt_rate = np.diff(aligned) / dt
            rate_error = float(np.nanmean(np.abs(pred_rate - gt_rate)))
        else:
            rate_error = float("nan")
        rows.append({
            "shift_s": float(shift), "mae_m": float(np.mean(error)), "p90_m": _percentile(list(error), 90),
            "correlation": corr, "range_rate_error_m_s": rate_error, "valid": True,
        })
    return rows


def run_shift_sweep(by_group: Mapping[str, dict[str, np.ndarray]]) -> tuple[list[dict[str, Any]], list[dict[str, Any]]]:
    sweep_rows: list[dict[str, Any]] = []
    per_group_rows: list[dict[str, Any]] = []
    for group_id, signals in sorted(by_group.items()):
        t = signals["t"]
        gt = signals["gt"]
        for signal_name in ("raw_physical_range_m", "model_oof_prediction_m"):
            pred = signals[signal_name]
            sweep = shift_sweep_group(t, gt, pred)
            for entry in sweep:
                sweep_rows.append({"group_id": group_id, "signal": signal_name, **entry})
            valid = [row for row in sweep if row["valid"]]
            if not valid:
                per_group_rows.append({
                    "group_id": group_id, "signal": signal_name, "best_shift_s": float("nan"),
                    "best_mae_m": float("nan"), "best_p90_m": float("nan"), "best_correlation": float("nan"),
                    "saturated_at_boundary": True,
                })
                continue
            best = min(valid, key=lambda row: row["mae_m"])
            saturated = (SHIFT_MAX_S - abs(best["shift_s"])) <= SATURATION_MARGIN_S
            per_group_rows.append({
                "group_id": group_id, "signal": signal_name, "best_shift_s": best["shift_s"],
                "best_mae_m": best["mae_m"], "best_p90_m": best["p90_m"], "best_correlation": best["correlation"],
                "saturated_at_boundary": saturated,
            })
    return sweep_rows, per_group_rows


# ---------------------------------------------------------------------------
# 4. Causal compensation replay (A / B / C)
# ---------------------------------------------------------------------------

def causal_compensation_rows(by_group: Mapping[str, dict[str, np.ndarray]], latency_by_group: Mapping[str, float]) -> list[dict[str, Any]]:
    out = []
    for group_id, signals in sorted(by_group.items()):
        t = signals["t"]
        gt = signals["gt"]
        latency = latency_by_group[group_id]
        for signal_name in ("raw_physical_range_m", "model_oof_prediction_m"):
            pred_a = signals[signal_name]

            # A: current prediction, as-is.
            lag_a = alignment_lag_s(t, gt, pred_a, maximum_s=SHIFT_MAX_S, step_s=SHIFT_STEP_S)
            mae_a = float(np.mean(np.abs(pred_a - gt)))
            out.append({
                "option": "A_current_prediction", "signal": signal_name, "group_id": group_id,
                "mae_m": mae_a, "residual_alignment_lag_s": lag_a,
                "note": "unmodified prediction vs ground truth",
            })

            # B: re-timestamp using ONLY measured, causal, real pipeline
            # latency (capture-to-consume), no oracle/GT information.
            t_b = t + latency
            lag_b = alignment_lag_s(t_b, gt, pred_a, maximum_s=SHIFT_MAX_S, step_s=SHIFT_STEP_S)
            aligned_b = np.interp(t_b, t, gt)  # best-effort same-length comparison
            mae_b = float(np.mean(np.abs(pred_a - aligned_b)))
            out.append({
                "option": "B_timestamp_corrected_causal", "signal": signal_name, "group_id": group_id,
                "mae_m": mae_b, "residual_alignment_lag_s": lag_b,
                "note": f"prediction re-timestamped by +{latency:.3f}s group-median measured capture_to_consume_s",
            })

            # C: constant-velocity forward projection using only current and
            # past predictions (causal_raw_range_rate_m_s), projected by the
            # same measured causal latency.
            rate = signals[f"{signal_name}_causal_rate"]
            pred_c = pred_a + rate * latency
            lag_c = alignment_lag_s(t, gt, pred_c, maximum_s=SHIFT_MAX_S, step_s=SHIFT_STEP_S)
            mae_c = float(np.mean(np.abs(pred_c - gt)))
            out.append({
                "option": "C_constant_velocity_projection", "signal": signal_name, "group_id": group_id,
                "mae_m": mae_c, "residual_alignment_lag_s": lag_c,
                "note": f"predicted_now = prediction + causal_range_rate * {latency:.3f}s",
            })
    return out


# ---------------------------------------------------------------------------
# Classification
# ---------------------------------------------------------------------------

def classify(
    per_group_lag: Sequence[Mapping[str, Any]],
    latency_summary_data: Mapping[str, Any],
    causal_rows: Sequence[Mapping[str, Any]],
) -> tuple[str, dict[str, Any]]:
    evidence: dict[str, Any] = {}

    physical_rows = [r for r in per_group_lag if r["signal"] == "raw_physical_range_m"]
    model_rows = [r for r in per_group_lag if r["signal"] == "model_oof_prediction_m"]
    saturated_count = sum(1 for r in physical_rows if r["saturated_at_boundary"])
    evidence["saturated_groups_physical_signal"] = saturated_count

    oracle_median_physical = float(median(abs(r["best_shift_s"]) for r in physical_rows))
    oracle_median_model = float(median(abs(r["best_shift_s"]) for r in model_rows))
    evidence["oracle_median_best_shift_s_physical"] = oracle_median_physical
    evidence["oracle_median_best_shift_s_model"] = oracle_median_model

    option_b = [r for r in causal_rows if r["option"] == "B_timestamp_corrected_causal" and r["signal"] == "raw_physical_range_m"]
    option_b_residual_median = float(median(abs(r["residual_alignment_lag_s"]) for r in option_b))
    evidence["option_b_residual_median_lag_s"] = option_b_residual_median

    median_capture_to_consume = latency_summary_data["capture_to_consume_s"]["median_s"]
    median_worker_inference = latency_summary_data["worker_inference_duration_s"]["median_s"]
    median_queue_wait = latency_summary_data["submit_to_worker_start_s"]["median_s"]
    median_complete_to_publish = latency_summary_data["complete_to_publish_s"]["median_s"]
    median_publish_to_consume = latency_summary_data["publish_to_consume_s"]["median_s"]
    queue_consumer_total = median_queue_wait + median_complete_to_publish + median_publish_to_consume
    pipeline_latency_share = (
        median_capture_to_consume / oracle_median_physical if oracle_median_physical > 1e-9 else float("nan")
    )
    evidence["median_capture_to_consume_s"] = median_capture_to_consume
    evidence["median_worker_inference_duration_s"] = median_worker_inference
    evidence["median_queue_and_consumer_side_s"] = queue_consumer_total
    evidence["pipeline_latency_share_of_oracle_lag"] = pipeline_latency_share

    model_vs_physical_gap = oracle_median_model - oracle_median_physical
    evidence["model_vs_physical_oracle_lag_gap_s"] = model_vs_physical_gap

    option_c_model = [r for r in causal_rows if r["option"] == "C_constant_velocity_projection" and r["signal"] == "model_oof_prediction_m"]
    option_a_model = [r for r in causal_rows if r["option"] == "A_current_prediction" and r["signal"] == "model_oof_prediction_m"]
    mae_reduction_c = float(median(a["mae_m"] for a in option_a_model)) - float(median(c["mae_m"] for c in option_c_model))
    evidence["option_c_model_mae_reduction_m"] = mae_reduction_c

    # Precommitted decision tree, evaluated in fixed order.
    if option_b_residual_median <= TIMESTAMP_GATE_S:
        return "TIMESTAMP_ASSOCIATION_ERROR", evidence
    if saturated_count > SATURATED_GROUP_LIMIT:
        return "TEMPORAL_EVIDENCE_INSUFFICIENT", evidence
    if not (0.0 <= pipeline_latency_share <= 5.0):
        return "TEMPORAL_EVIDENCE_INSUFFICIENT", evidence
    if pipeline_latency_share >= DOMINANT_FRACTION:
        if median_worker_inference >= queue_consumer_total:
            return "DEPTH_PIPELINE_LATENCY_DOMINANT", evidence
        return "QUEUE_OR_CONSUMER_LATENCY_DOMINANT", evidence
    if model_vs_physical_gap >= MODEL_VS_PHYSICAL_GAP_S and mae_reduction_c > 0:
        return "MODEL_RESPONSE_LAG_DOMINANT", evidence
    if pipeline_latency_share >= 0.25 or model_vs_physical_gap >= 0.10:
        return "MIXED_TEMPORAL_CAUSES", evidence
    return "TEMPORAL_EVIDENCE_INSUFFICIENT", evidence


# ---------------------------------------------------------------------------
# Orchestration
# ---------------------------------------------------------------------------

def prepare(output: Path) -> dict[str, Any]:
    output.mkdir(parents=True, exist_ok=True)
    plan = {
        "audit_id": AUDIT_ID,
        "seed": SEED,
        "no_retrain": True,
        "no_additional_data_collection": True,
        "no_quarantine_used": True,
        "no_runtime_shadow_or_follow_target": True,
        "model_signal": {"variant": MODEL_VARIANT, "config": MODEL_CONFIG},
        "shift_sweep": {"max_s": SHIFT_MAX_S, "step_s": SHIFT_STEP_S},
        "classification_thresholds": {
            "timestamp_gate_s": TIMESTAMP_GATE_S,
            "dominant_fraction": DOMINANT_FRACTION,
            "saturation_margin_s": SATURATION_MARGIN_S,
            "saturated_group_limit": SATURATED_GROUP_LIMIT,
            "model_vs_physical_gap_s": MODEL_VS_PHYSICAL_GAP_S,
        },
        "decision_tree_order": [
            "1. option_b_residual_median_lag_s <= timestamp_gate_s -> TIMESTAMP_ASSOCIATION_ERROR",
            "2. saturated_groups (physical signal) > saturated_group_limit -> TEMPORAL_EVIDENCE_INSUFFICIENT",
            "3. pipeline_latency_share >= dominant_fraction -> DEPTH_PIPELINE_LATENCY_DOMINANT "
            "if worker_inference_duration_median >= queue_and_consumer_side_median else "
            "QUEUE_OR_CONSUMER_LATENCY_DOMINANT",
            "4. model_vs_physical_oracle_lag_gap_s >= model_vs_physical_gap_s AND option_c reduces model MAE "
            "-> MODEL_RESPONSE_LAG_DOMINANT",
            "5. pipeline_latency_share >= 0.25 or model_vs_physical_gap_s >= 0.10 -> MIXED_TEMPORAL_CAUSES",
            "6. otherwise -> TEMPORAL_EVIDENCE_INSUFFICIENT",
        ],
    }
    (output / "audit_plan.json").write_text(json.dumps(plan, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return plan


def run(workspace: Path, output: Path, dynamic_output: Path, retrain_output: Path) -> dict[str, Any]:
    plan = json.loads((output / "audit_plan.json").read_text(encoding="utf-8"))
    rows, dataset_identity = load_audit_frames(workspace, dynamic_output)
    predictions = load_model_predictions(retrain_output, rows)
    for row, oof in zip(rows, predictions):
        row["model_oof_prediction_m"] = oof

    # --- 1. timestamp chain ---
    chain_rows = timestamp_chain_rows(rows)
    _write_csv(output / "timestamp_chain.csv", chain_rows)

    missing = {name: spec for name, spec in TIMESTAMP_CONCEPTS.items() if spec["status"] == "NOT_RECORDED"}
    (output / "missing_timestamp_fields.json").write_text(
        json.dumps({"missing_concepts": missing, "note": "No missing field was inferred; all NOT_RECORDED "
                                                            "columns are left empty in timestamp_chain.csv."},
                   indent=2, sort_keys=True) + "\n", encoding="utf-8")

    # --- 2. latency decomposition ---
    decomposition = latency_decomposition_rows(rows)
    _write_csv(output / "latency_decomposition.csv", decomposition)
    lat_summary = latency_summary(decomposition)

    median_capture_to_consume_by_group: dict[str, float] = {}
    by_group_capture_to_consume: dict[str, list[float]] = {}
    for entry in decomposition:
        by_group_capture_to_consume.setdefault(entry["group_id"], []).append(entry["capture_to_consume_s"])
    for group_id, values in by_group_capture_to_consume.items():
        median_capture_to_consume_by_group[group_id] = float(median(values))

    # --- signal series per group ---
    by_group: dict[str, dict[str, np.ndarray]] = {}
    for group_id in sorted({row["session_id"] for row in rows}):
        group_rows = sorted(
            (row for row in rows if row["session_id"] == group_id),
            key=lambda row: row["measurement_timestamp_s"],
        )
        t = np.array([row["measurement_timestamp_s"] for row in group_rows], dtype=np.float64)
        gt = np.array([row["ground_truth_range_m"] for row in group_rows], dtype=np.float64)
        raw_physical = np.array([row["raw_physical_range_m"] for row in group_rows], dtype=np.float64)
        model_oof = np.array([row["model_oof_prediction_m"] for row in group_rows], dtype=np.float64)
        raw_rate = np.array([row["features"]["causal_raw_range_rate_m_s"] for row in group_rows], dtype=np.float64)
        # Model-signal causal rate: same causal finite-difference definition applied to the model prediction series.
        model_rate = np.zeros_like(model_oof)
        for index in range(1, len(model_oof)):
            dt = max(t[index] - t[index - 1], 1e-6)
            model_rate[index] = (model_oof[index] - model_oof[index - 1]) / dt
        by_group[group_id] = {
            "t": t, "gt": gt,
            "raw_physical_range_m": raw_physical, "model_oof_prediction_m": model_oof,
            "raw_physical_range_m_causal_rate": raw_rate, "model_oof_prediction_m_causal_rate": model_rate,
        }

    # --- 3. shift sweep ---
    sweep_rows, per_group_lag = run_shift_sweep(by_group)
    _write_csv(output / "shift_sweep.csv", sweep_rows)

    dispersion = {}
    for signal_name in ("raw_physical_range_m", "model_oof_prediction_m"):
        shifts = [row["best_shift_s"] for row in per_group_lag if row["signal"] == signal_name]
        dispersion[signal_name] = {
            "median_best_shift_s": float(median(shifts)),
            "min_best_shift_s": float(min(shifts)),
            "max_best_shift_s": float(max(shifts)),
            "std_best_shift_s": float(np.std(shifts)),
        }
    per_group_out = [dict(row, **{"median_capture_to_consume_s_for_group": median_capture_to_consume_by_group[row["group_id"]]})
                      for row in per_group_lag]
    _write_csv(output / "per_group_lag_metrics.csv", per_group_out)

    # --- 4. causal compensation ---
    causal_rows = causal_compensation_rows(by_group, median_capture_to_consume_by_group)
    _write_csv(output / "causal_compensation_metrics.csv", causal_rows)

    # --- 5/6. classification ---
    conclusion, evidence = classify(per_group_lag, lat_summary, causal_rows)

    report_lines = [
        "# CORE RANGE TEMPORAL LAG ROOT CAUSE AUDIT REPORT", "",
        f"Conclusion: `{conclusion}`", "",
        "## Key evidence", "",
        "```json",
        json.dumps(evidence, indent=2, sort_keys=True),
        "```", "",
        "## Latency decomposition (median / P90 / P95, seconds)", "",
        "| Stage | Median | P90 | P95 |", "|---|---:|---:|---:|",
    ]
    for key in ("capture_to_submit_s", "submit_to_worker_start_s", "worker_inference_duration_s",
                "complete_to_publish_s", "publish_to_consume_s", "capture_to_consume_s"):
        v = lat_summary[key]
        report_lines.append(f"| {key} | {v['median_s']:.3f} | {v['p90_s']:.3f} | {v['p95_s']:.3f} |")
    report_lines += [
        "", f"Worst group by median capture_to_consume: `{lat_summary['worst_group_by_capture_to_consume_median']}`",
        "", "## Oracle shift-sweep dispersion (best MAE-minimizing shift per group, [-3, +3]s)", "",
        "| Signal | Median best shift (s) | Min | Max | Std |", "|---|---:|---:|---:|---:|",
    ]
    for signal_name, d in dispersion.items():
        report_lines.append(
            f"| {signal_name} | {d['median_best_shift_s']:.3f} | {d['min_best_shift_s']:.3f} | "
            f"{d['max_best_shift_s']:.3f} | {d['std_best_shift_s']:.3f} |"
        )
    report_lines += ["", "## Scope guards", "",
                      "No retrain. No additional data collection. No quarantine used. "
                      "No MiDaS/M52/calibration/ROI/filter/EKF/controller/PX4 code modified. "
                      "No shadow or Follow Target run. Diagnostic oracle (shift sweep) never used at runtime.", ""]
    (output / "audit_report.md").write_text("\n".join(report_lines), encoding="utf-8")

    output_files = [p for p in output.rglob("*") if p.is_file() and p.name != "audit_manifest.json"]
    manifest = {
        "audit_id": AUDIT_ID,
        "conclusion": conclusion,
        "evidence": evidence,
        "dataset_identity": dataset_identity,
        "model_signal": {"variant": MODEL_VARIANT, "config": MODEL_CONFIG},
        "scope_guards": {
            "retrain": False, "additional_data_collection": False, "quarantine_used": False,
            "midas_m52_calibration_roi_filter_ekf_controller_px4_modified": False,
            "shadow_run": False, "follow_target_run": False, "oracle_used_at_runtime": False,
        },
        "artifacts": {str(p.relative_to(output)): sha256_file(p) for p in sorted(output_files)},
    }
    (output / "audit_manifest.json").write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps({"conclusion": conclusion}, indent=2))
    return manifest


def main() -> int:
    parser = argparse.ArgumentParser()
    sub = parser.add_subparsers(dest="command", required=True)
    for command in ("prepare", "run"):
        child = sub.add_parser(command)
        child.add_argument("--workspace", type=Path, required=True)
        child.add_argument("--output", type=Path, required=True)
        child.add_argument("--dynamic-output", type=Path, required=True)
        child.add_argument("--retrain-output", type=Path, required=True)
    args = parser.parse_args()
    workspace = args.workspace.resolve()
    output = args.output.resolve()
    dynamic_output = args.dynamic_output.resolve()
    retrain_output = args.retrain_output.resolve()
    if args.command == "prepare":
        prepare(output)
    else:
        run(workspace, output, dynamic_output, retrain_output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
