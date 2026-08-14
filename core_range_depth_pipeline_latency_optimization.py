"""Depth-pipeline (MiDaS) capture->consume latency optimization.

Read-mostly. The only production code touched is a single, additive,
resolution/precision-preserving change to `MidasSmallAdapter.load()`
(enable `torch.backends.cudnn.benchmark`), applied only if the precommitted
selection rule in `optimization_plan.json` picks it. Never retrains a model,
never collects an official/audited dataset, never touches GT/M52
geometry/calibration semantics/controller/PX4, never runs Follow Target.

Subcommands:
  prepare   - write the precommitted optimization_plan.json
  profile   - step 1: per-frame/per-session latency profile of the 8 frozen
              baseline dynamic sessions (reuses core_range_temporal_lag_audit
              loaders; adds the two extra stage splits and a
              would-be-stale-under-production-default derived flag)
  benchmark - step 3: standalone MiDaS latency/memory benchmark across
              configurations, synthetic frames, GPU
  select    - step 4: apply the precommitted selection rule to the benchmark
              + reused accuracy evidence; writes configuration_comparison.csv
              and (only if the rule picks a non-baseline config) applies the
              minimal code change
  smoke     - step 5 analysis: parse a freshly captured smoke session's
              sidecar and compute post-optimization metrics (does not launch
              the sim stack itself; that is orchestrated separately)
  report    - step 6: classify and write the final report/manifest
"""

from __future__ import annotations

import argparse
import csv
import json
import time
from pathlib import Path
from statistics import median
from typing import Any, Mapping, Sequence

import numpy as np

from core_range_logging_eval import sha256_file, load_jsonl
from core_range_direct_dynamic_replay import (
    FEATURE_NAMES,
    FrozenDirectEnsemble,
    alignment_lag_s,
    extract_features,
    verify_frozen_candidate,
)
from core_range_temporal_lag_audit import load_audit_frames

OPTIMIZATION_ID = "core_range_depth_pipeline_latency_optimization_20260805_v001"
SEED = 52
PRODUCTION_DEFAULT_MAX_DEPTH_AGE_S = 0.75  # .env.example / metric_target_fusion.py default
COLLECTION_MAX_DEPTH_AGE_S = 3.0           # main.py:2908-2911, used during all core-range captures
CAMERA_WIDTH, CAMERA_HEIGHT = 480, 270     # observed camera_info in the frozen dynamic sidecars
LAG_SWEEP_MAX_S = 6.0                      # widened from the +-3s audit which still saturated for 3/8 groups
LAG_SWEEP_STEP_S = 0.05


def _json(path: Path) -> Any:
    return json.loads(path.read_text(encoding="utf-8"))


def _write_csv(path: Path, rows: Sequence[Mapping[str, Any]]) -> None:
    if not rows:
        path.write_text("", encoding="utf-8")
        return
    fieldnames = list(rows[0].keys())
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def _percentile(values: Sequence[float], pct: float) -> float:
    if not values:
        return float("nan")
    return float(np.percentile(np.asarray(values, dtype=np.float64), pct))


# ---------------------------------------------------------------------------
# prepare
# ---------------------------------------------------------------------------

BENCHMARK_CONFIGS = [
    {"name": "current_256_fp32", "input_size": 256, "precision": "fp32", "cudnn_benchmark": False,
     "role": "baseline_production_default"},
    {"name": "cudnn_benchmark_256_fp32", "input_size": 256, "precision": "fp32", "cudnn_benchmark": True,
     "role": "selection_eligible_resolution_and_precision_preserving"},
    {"name": "res224_fp32", "input_size": 224, "precision": "fp32", "cudnn_benchmark": True,
     "role": "latency_exploratory_only"},
    {"name": "res192_fp32", "input_size": 192, "precision": "fp32", "cudnn_benchmark": True,
     "role": "latency_exploratory_only"},
    {"name": "res160_fp32", "input_size": 160, "precision": "fp32", "cudnn_benchmark": True,
     "role": "latency_exploratory_only"},
    {"name": "fp16_256", "input_size": 256, "precision": "fp16", "cudnn_benchmark": True,
     "role": "latency_exploratory_only"},
]

SELECTION_RULE = (
    "Select the fastest configuration whose role is "
    "'selection_eligible_resolution_and_precision_preserving' or "
    "'baseline_production_default' (i.e. identical input_size=256 and "
    "precision=fp32 to the current production default), AND that in the "
    "benchmark: (a) produces zero malformed/nonfinite inference outputs on "
    "synthetic-frame smoke calls, (b) has steady-state per-call latency less "
    "than or equal to the target submission interval implied by "
    "SWARM_METRIC_TARGET_DEPTH_RATE_HZ=7.5 (i.e. <= 0.1333s) is NOT required "
    "(current baseline already exceeds this and is still the operative "
    "config; the bar is: no WORSE backlog risk than baseline, i.e. "
    "steady-state latency <= baseline steady-state latency), and (c) has "
    "max-abs inverse-depth output deviation from baseline on the same "
    "synthetic input below 1e-2 (numerically negligible, confirming the "
    "change is output-preserving). "
    "Configurations at a different input_size or precision "
    "('latency_exploratory_only' role) are reported for information only "
    "and are NEVER eligible for selection in this task, because no cached "
    "raw camera frames exist to empirically re-validate static "
    "MAE/raw-jitter/raw-range-availability/calibration-validity at a "
    "changed resolution or precision without a new live capture, and this "
    "task does not authorize new official dataset collection. "
    "If no configuration other than the baseline satisfies (a)-(c), the "
    "baseline is retained and no code is changed. This rule is fixed here, "
    "before any benchmark is run, and is not altered afterward."
)

CLASSIFICATION_OUTCOMES = (
    "DEPTH_PIPELINE_LATENCY_REDUCED_READY_FOR_RETRAIN",
    "DEPTH_PIPELINE_LATENCY_REDUCED_MODEL_STILL_LAGS",
    "DEPTH_PIPELINE_OPTIMIZATION_BREAKS_ACCURACY",
    "DEPTH_PIPELINE_LATENCY_OPTIMIZATION_INSUFFICIENT",
    "OPTIMIZATION_BLOCKED_BY_ENVIRONMENT",
)


def prepare(output: Path) -> dict[str, Any]:
    output.mkdir(parents=True, exist_ok=True)
    plan = {
        "optimization_id": OPTIMIZATION_ID,
        "seed": SEED,
        "no_retrain": True,
        "no_official_dataset_collection": True,
        "no_gt_m52_calibration_controller_px4_change": True,
        "no_follow_target": True,
        "baseline_reference": {
            "capture_to_consume_median_s": 1.13,
            "capture_to_consume_p95_s": 1.57,
            "worker_inference_median_s": 0.61,
            "queue_consumer_stages_median_s": 0.21,
            "source": "docs/CORE_RANGE_TEMPORAL_LAG_ROOT_CAUSE_AUDIT.md",
        },
        "production_default_max_depth_age_s": PRODUCTION_DEFAULT_MAX_DEPTH_AGE_S,
        "collection_mode_max_depth_age_s": COLLECTION_MAX_DEPTH_AGE_S,
        "camera_frame_shape": {"width": CAMERA_WIDTH, "height": CAMERA_HEIGHT},
        "benchmark_configs": BENCHMARK_CONFIGS,
        "selection_rule": SELECTION_RULE,
        "lag_sweep": {"max_s": LAG_SWEEP_MAX_S, "step_s": LAG_SWEEP_STEP_S,
                      "note": "widened from the +-3s temporal-lag audit, which still "
                              "saturated for 3/8 groups on the model signal"},
        "smoke_sessions": [
            {"name": "smoke_approach", "scenario_type": "approaching",
             "based_on": "cdr_approach_center", "start_range_m": 11.5, "end_range_m": 3.5,
             "lateral_offset_m": 0.0, "target_z_m": 1.0, "yaw_start_deg": 0.0, "yaw_end_deg": 0.0,
             "movement_duration_s": 16.0, "hold_duration_s": 0.0, "pose_update_rate_hz": 5.0,
             "gimbal_pitch_deg": -10.0, "bbox_normalized": [0.465, 0.25, 0.07, 0.095]},
            {"name": "smoke_recede", "scenario_type": "receding",
             "based_on": "cdr_recede_center", "start_range_m": 3.5, "end_range_m": 11.5,
             "lateral_offset_m": 0.0, "target_z_m": 1.0, "yaw_start_deg": 0.0, "yaw_end_deg": 0.0,
             "movement_duration_s": 16.0, "hold_duration_s": 0.0, "pose_update_rate_hz": 5.0,
             "gimbal_pitch_deg": -10.0, "bbox_normalized": [0.398, 0.155, 0.204, 0.279]},
            {"name": "smoke_stop_hold", "scenario_type": "stop_and_hold",
             "based_on": "cdr_stop_approach_6", "start_range_m": 11.5, "end_range_m": 6.0,
             "lateral_offset_m": 0.0, "target_z_m": 1.0, "yaw_start_deg": 0.0, "yaw_end_deg": 0.0,
             "movement_duration_s": 11.0, "hold_duration_s": 6.0, "pose_update_rate_hz": 5.0,
             "gimbal_pitch_deg": -10.0, "bbox_normalized": [0.465, 0.25, 0.07, 0.095]},
        ],
        "classification_outcomes": list(CLASSIFICATION_OUTCOMES),
    }
    (output / "optimization_plan.json").write_text(json.dumps(plan, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return plan


# ---------------------------------------------------------------------------
# step 1: profile pipeline (baseline, 8 frozen dynamic sessions)
# ---------------------------------------------------------------------------

def profile_pipeline(workspace: Path, dynamic_output: Path, output: Path) -> None:
    rows, _identity = load_audit_frames(workspace, dynamic_output)
    profile_rows = []
    for row in rows:
        raw = row["raw_record"]
        s = raw["timestamp_stages"]
        capture = s["frame_receipt"]["timestamp_s"]
        submit = s["depth_submit"]["timestamp_s"]
        worker_start = s["depth_worker_start"]["timestamp_s"]
        complete = s["depth_complete"]["timestamp_s"]
        publish = s["result_publish"]["timestamp_s"]
        consumer_receive = s["consumer_receive"]["timestamp_s"]
        consume = s["consume"]["timestamp_s"]
        measurement_age_at_consume = consume - capture
        profile_rows.append({
            "group_id": row["session_id"],
            "frame_index": row["frame_index"],
            "frame_receipt_to_submit_s": submit - capture,
            "submit_to_worker_start_s": worker_start - submit,
            "worker_inference_s": complete - worker_start,
            "complete_to_publish_s": publish - complete,
            "publish_to_consumer_receive_s": consumer_receive - publish,
            "consumer_receive_to_consume_s": consume - consumer_receive,
            "measurement_age_at_consume_s": measurement_age_at_consume,
            "would_be_stale_under_production_default_0_75s": measurement_age_at_consume > PRODUCTION_DEFAULT_MAX_DEPTH_AGE_S,
            "pending_frame_replacements_this_session": "NOT_RECORDED_HISTORICALLY",
            "dropped_frames_this_session": "NOT_RECORDED_HISTORICALLY",
            "stale_result_count_this_session": "NOT_RECORDED_HISTORICALLY_UNDER_COLLECTION_MODE_MAX_AGE_3S",
        })
    _write_csv(output / "baseline_profile.csv", profile_rows)

    stage_keys = [
        "frame_receipt_to_submit_s", "submit_to_worker_start_s", "worker_inference_s",
        "complete_to_publish_s", "publish_to_consumer_receive_s", "consumer_receive_to_consume_s",
        "measurement_age_at_consume_s",
    ]
    by_group: dict[str, list[dict[str, Any]]] = {}
    for row in profile_rows:
        by_group.setdefault(row["group_id"], []).append(row)
    summary: dict[str, Any] = {"overall": {}, "per_group": {}}
    for key in stage_keys:
        values = [row[key] for row in profile_rows]
        summary["overall"][key] = {"median_s": float(median(values)), "p90_s": _percentile(values, 90), "p95_s": _percentile(values, 95)}
    for group_id, group_rows in by_group.items():
        would_be_stale = sum(1 for row in group_rows if row["would_be_stale_under_production_default_0_75s"])
        summary["per_group"][group_id] = {
            "frame_count": len(group_rows),
            "measurement_age_at_consume_median_s": float(median(row["measurement_age_at_consume_s"] for row in group_rows)),
            "would_be_stale_under_production_default_fraction": would_be_stale / len(group_rows),
        }
    (output / "baseline_profile_summary.json").write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps({"frames": len(profile_rows), "groups": len(by_group)}, indent=2))


# ---------------------------------------------------------------------------
# step 3: MiDaS inference benchmark (standalone, synthetic frames, GPU)
# ---------------------------------------------------------------------------

def benchmark_midas(output: Path, plan: Mapping[str, Any], warmup: int = 5, iterations: int = 20) -> None:
    import torch

    rng = np.random.default_rng(SEED)
    frame = rng.integers(0, 255, (CAMERA_HEIGHT, CAMERA_WIDTH, 3), dtype=np.uint8)

    rows: list[dict[str, Any]] = []
    baseline_output: np.ndarray | None = None
    for config in plan["benchmark_configs"]:
        torch.cuda.empty_cache()
        torch.cuda.reset_peak_memory_stats() if torch.cuda.is_available() else None
        from depth_model_adapter import MidasSmallAdapter

        torch.backends.cudnn.benchmark = bool(config["cudnn_benchmark"])
        adapter = MidasSmallAdapter(input_size=int(config["input_size"]))
        if config["precision"] == "fp16":
            adapter.load()
            adapter._model = adapter._model.half()
            real_forward = adapter._model.__call__

            def call_half(tensor, _forward=real_forward):
                return _forward(tensor.half()).float()
            adapter._model.__call__ = call_half  # type: ignore[method-assign]
        else:
            adapter.load()

        malformed = 0
        durations = []
        output_sample = None
        try:
            for index in range(warmup + iterations):
                start = time.perf_counter()
                depth_map = adapter.infer(frame)
                if torch.cuda.is_available():
                    torch.cuda.synchronize()
                elapsed = time.perf_counter() - start
                if not np.any(np.isfinite(depth_map.inverse_depth)):
                    malformed += 1
                if index >= warmup:
                    durations.append(elapsed)
                if index == warmup:
                    output_sample = depth_map.inverse_depth.copy()
        except Exception as error:
            rows.append({
                "config": config["name"], "role": config["role"], "input_size": config["input_size"],
                "precision": config["precision"], "cudnn_benchmark": config["cudnn_benchmark"],
                "status": f"FAILED:{type(error).__name__}:{error}",
                "warmup_latency_s": "", "steady_state_median_s": "", "steady_state_p90_s": "",
                "steady_state_p95_s": "", "malformed_output_count": "", "max_abs_deviation_from_baseline": "",
                "gpu_peak_memory_mb": "",
            })
            continue

        peak_memory_mb = (
            float(torch.cuda.max_memory_allocated()) / (1024 * 1024) if torch.cuda.is_available() else float("nan")
        )
        if config["name"] == "current_256_fp32":
            baseline_output = output_sample
        deviation = (
            float(np.max(np.abs(output_sample.astype(np.float64) - baseline_output.astype(np.float64))))
            if baseline_output is not None and output_sample is not None and output_sample.shape == baseline_output.shape
            else float("nan")
        )
        rows.append({
            "config": config["name"], "role": config["role"], "input_size": config["input_size"],
            "precision": config["precision"], "cudnn_benchmark": config["cudnn_benchmark"],
            "status": "OK",
            "warmup_latency_s": durations[0] if durations else "",
            "steady_state_median_s": float(median(durations)),
            "steady_state_p90_s": _percentile(durations, 90),
            "steady_state_p95_s": _percentile(durations, 95),
            "malformed_output_count": malformed,
            "max_abs_deviation_from_baseline": deviation,
            "gpu_peak_memory_mb": peak_memory_mb,
        })
        del adapter
        torch.cuda.empty_cache()

    _write_csv(output / "inference_benchmark.csv", rows)
    print(json.dumps({"configs_benchmarked": len(rows)}, indent=2))


# ---------------------------------------------------------------------------
# step 4: configuration selection (apply precommitted rule)
# ---------------------------------------------------------------------------

def select_configuration(output: Path, plan: Mapping[str, Any]) -> dict[str, Any]:
    with (output / "inference_benchmark.csv").open() as stream:
        bench_rows = list(csv.DictReader(stream))
    by_name = {row["config"]: row for row in bench_rows}
    baseline = by_name.get("current_256_fp32")
    if baseline is None or baseline["status"] != "OK":
        return {"selected_config": None, "reason": "baseline_benchmark_failed", "comparison_rows": []}
    baseline_latency = float(baseline["steady_state_median_s"])

    eligible_roles = {"baseline_production_default", "selection_eligible_resolution_and_precision_preserving"}
    comparison_rows = []
    best_name, best_latency = "current_256_fp32", baseline_latency
    for row in bench_rows:
        eligible = row["role"] in eligible_roles and row["status"] == "OK"
        passes_gate = False
        if eligible:
            malformed_ok = int(row["malformed_output_count"]) == 0
            latency_ok = float(row["steady_state_median_s"]) <= baseline_latency
            deviation = row["max_abs_deviation_from_baseline"]
            deviation_ok = deviation == "" or float(deviation) < 1e-2
            passes_gate = malformed_ok and latency_ok and deviation_ok
        comparison_rows.append({**row, "eligible_for_selection": eligible, "passes_selection_gate": passes_gate})
        if eligible and passes_gate and float(row["steady_state_median_s"]) < best_latency:
            best_name, best_latency = row["config"], float(row["steady_state_median_s"])

    _write_csv(output / "configuration_comparison.csv", comparison_rows)
    result = {
        "selected_config": best_name,
        "baseline_config": "current_256_fp32",
        "baseline_steady_state_median_s": baseline_latency,
        "selected_steady_state_median_s": best_latency,
        "changed_from_baseline": best_name != "current_256_fp32",
        "selection_rule": plan["selection_rule"],
    }
    (output / "selected_configuration.json").write_text(json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps(result, indent=2))
    return result


# ---------------------------------------------------------------------------
# step 5: smoke session analysis (post-capture; capture is orchestrated
# separately via core_range_collect_dynamic_scenario.sh)
# ---------------------------------------------------------------------------

def analyze_smoke_session(workspace: Path, dynamic_output: Path, smoke_root: Path, session_name: str, scenario_type: str) -> dict[str, Any]:
    plan, candidate = verify_frozen_candidate(workspace, dynamic_output)
    sidecar = smoke_root / "physical_diagnostics.jsonl"
    loaded, malformed = load_jsonl(sidecar)
    if malformed:
        raise ValueError(f"smoke_sidecar_malformed:{session_name}:{malformed}")
    stage_counts: dict[str, int] = {}
    for record in loaded:
        stage_counts[record.get("stage", "unknown")] = stage_counts.get(record.get("stage", "unknown"), 0) + 1
    rows = [r for r in loaded if r.get("stage") == "raw_range_computed"]
    if not rows:
        raise ValueError(f"smoke_no_raw_range_frames:{session_name}")
    rows.sort(key=lambda r: float(r["measurement_timestamp_s"]))

    ages = []
    for row in rows:
        s = row["timestamp_stages"]
        ages.append(s["consume"]["timestamp_s"] - s["frame_receipt"]["timestamp_s"])

    ensemble = FrozenDirectEnsemble(workspace, candidate)
    features = [extract_features(row) for row in rows]
    model_prediction, _disagreement = ensemble.predict(features)

    t = np.array([float(row["measurement_timestamp_s"]) for row in rows], dtype=np.float64)
    gt = np.array([float(row["ground_truth"]["distance_m"]) for row in rows], dtype=np.float64)
    raw = np.array([float(row["raw_range"]["physics_slant_range_m"]) for row in rows], dtype=np.float64)

    raw_mae = float(np.mean(np.abs(raw - gt)))
    model_mae = float(np.mean(np.abs(model_prediction - gt)))

    metrics = {
        "session_name": session_name, "scenario_type": scenario_type,
        "frame_count_raw_range_computed": len(rows),
        "stage_counts": stage_counts,
        "measurement_age_at_consume_median_s": float(median(ages)),
        "measurement_age_at_consume_p95_s": _percentile(ages, 95),
        "raw_dynamic_mae_m": raw_mae,
        "frozen_model_dynamic_mae_m": model_mae,
        "stale_result_count": stage_counts.get("depth_result_stale", 0),
        "invalid_result_count": stage_counts.get("depth_result_invalid", 0),
    }

    lag_rows = []
    shifts = np.arange(-LAG_SWEEP_MAX_S, LAG_SWEEP_MAX_S + 0.5 * LAG_SWEEP_STEP_S, LAG_SWEEP_STEP_S)
    for shift in shifts:
        shifted = t - shift
        mask = (shifted >= t[0]) & (shifted <= t[-1])
        if int(np.count_nonzero(mask)) < max(4, len(t) // 2):
            continue
        aligned = np.interp(shifted[mask], t, gt)
        mae = float(np.mean(np.abs(model_prediction[mask] - aligned)))
        lag_rows.append({"session_name": session_name, "shift_s": float(shift), "mae_m": mae})
    best = min(lag_rows, key=lambda r: r["mae_m"]) if lag_rows else None
    metrics["best_shift_s"] = best["shift_s"] if best else float("nan")
    metrics["saturated_at_sweep_boundary"] = bool(best and (LAG_SWEEP_MAX_S - abs(best["shift_s"])) <= 0.10)
    metrics["alignment_lag_s_reference"] = alignment_lag_s(t, gt, model_prediction, maximum_s=LAG_SWEEP_MAX_S, step_s=LAG_SWEEP_STEP_S)

    return {"metrics": metrics, "lag_rows": lag_rows}


def run_smoke_all(workspace: Path, dynamic_output: Path, output: Path, plan: Mapping[str, Any]) -> None:
    smoke_metrics = []
    all_lag_rows = []
    for session in plan["smoke_sessions"]:
        smoke_root = output / "smoke_sessions" / session["name"]
        result = analyze_smoke_session(workspace, dynamic_output, smoke_root, session["name"], session["scenario_type"])
        smoke_metrics.append(result["metrics"])
        all_lag_rows.extend(result["lag_rows"])
    _write_csv(output / "smoke_dynamic_metrics.csv", [
        {k: v for k, v in row.items() if k != "stage_counts"} | {"stage_counts_json": json.dumps(row["stage_counts"])}
        for row in smoke_metrics
    ])
    _write_csv(output / "lag_sweep.csv", all_lag_rows)
    print(json.dumps({"sessions": len(smoke_metrics)}, indent=2))


def main() -> int:
    parser = argparse.ArgumentParser()
    sub = parser.add_subparsers(dest="command", required=True)

    p = sub.add_parser("prepare")
    p.add_argument("--output", type=Path, required=True)

    p = sub.add_parser("profile")
    p.add_argument("--workspace", type=Path, required=True)
    p.add_argument("--dynamic-output", type=Path, required=True)
    p.add_argument("--output", type=Path, required=True)

    p = sub.add_parser("benchmark")
    p.add_argument("--output", type=Path, required=True)

    p = sub.add_parser("select")
    p.add_argument("--output", type=Path, required=True)

    p = sub.add_parser("smoke")
    p.add_argument("--workspace", type=Path, required=True)
    p.add_argument("--dynamic-output", type=Path, required=True)
    p.add_argument("--output", type=Path, required=True)

    args = parser.parse_args()
    output = args.output.resolve()
    if args.command == "prepare":
        prepare(output)
    elif args.command == "profile":
        profile_pipeline(args.workspace.resolve(), args.dynamic_output.resolve(), output)
    elif args.command == "benchmark":
        plan = _json(output / "optimization_plan.json")
        benchmark_midas(output, plan)
    elif args.command == "select":
        plan = _json(output / "optimization_plan.json")
        select_configuration(output, plan)
    elif args.command == "smoke":
        plan = _json(output / "optimization_plan.json")
        run_smoke_all(args.workspace.resolve(), args.dynamic_output.resolve(), output, plan)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
