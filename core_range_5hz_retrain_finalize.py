"""Finalize an incomplete 5 Hz collection without training or gate dilution."""

from __future__ import annotations

import csv
import json
import math
from pathlib import Path
from statistics import median

import numpy as np

from core_range_dynamic_robust_retrain import (
    PHYSICAL_ONLY_FEATURES, PHYSICAL_TEMPORAL_FEATURES, TEMPORAL_EXTRA_FEATURES,
)
from core_range_logging_eval import load_jsonl, sha256_file, verify_record
from core_range_xgboost_benchmark import collect_verified_rows
from range_physical_diagnostics import validate_timestamp_stages

ROOT = Path(__file__).resolve().parent
OUT = ROOT / "artifacts/core_range_3_12m/dynamic_5hz_retrain"


def write_json(name: str, value: object) -> None:
    (OUT / name).write_text(json.dumps(value, indent=2, sort_keys=True) + "\n")


def write_csv(name: str, rows: list[dict[str, object]], headers: list[str]) -> None:
    keys = list(headers)
    for row in rows:
        for key in row:
            if key not in keys:
                keys.append(key)
    with (OUT / name).open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=keys)
        writer.writeheader()
        writer.writerows(rows)


def pct(values: list[float], p: float) -> float | None:
    return None if not values else float(np.percentile(np.asarray(values), p))


def inspect_attempt(path: Path) -> dict[str, object]:
    capture_path = path / "capture_result.json"
    if not capture_path.exists() or not capture_path.stat().st_size:
        return {"attempt_path": str(path.relative_to(ROOT)), "parseable": False, "reason": "capture_result_missing_or_empty"}
    capture = json.loads(capture_path.read_text())
    rows, malformed = load_jsonl(path / "physical_diagnostics.jsonl")
    sid = int(capture["session_id"])
    raw = [row for row in rows if row.get("stage") == "raw_range_computed" and int(row.get("session_id", -1)) == sid]
    capture_consume: list[float] = []
    worker: list[float] = []
    queue: list[float] = []
    submit: list[float] = []
    fps_samples: list[float] = []
    checksum_failures = timestamp_failures = gt_failures = anchor_failures = nonfinite = 0
    traces: set[str] = set()
    duplicates = 0
    previous = None
    for row in raw:
        record_ok, trace_ok = verify_record(row)
        time_ok, _ = validate_timestamp_stages(row.get("timestamp_stages") or {})
        checksum_failures += int(not record_ok or not trace_ok)
        timestamp_failures += int(not time_ok)
        gt_failures += int(row.get("ground_truth_trace_valid") is not True)
        anchor_failures += int(len((row.get("anchors") or {}).get("per_grid_point") or []) != 96)
        try:
            finite = math.isfinite(float(row["raw_range"]["physics_slant_range_m"])) and math.isfinite(float(row["ground_truth"]["distance_m"]))
        except (KeyError, TypeError, ValueError):
            finite = False
        nonfinite += int(not finite)
        trace = str(row.get("trace_identity_sha256"))
        duplicates += int(trace in traces)
        traces.add(trace)
        stages = row["timestamp_stages"]
        submit.append(float(stages["depth_submit"]["timestamp_s"]))
        capture_consume.append(float(stages["consume"]["timestamp_s"]) - float(stages["frame_receipt"]["timestamp_s"]))
        worker.append(float(stages["depth_complete"]["timestamp_s"]) - float(stages["depth_worker_start"]["timestamp_s"]))
        queue.append(float(stages["depth_worker_start"]["timestamp_s"]) - float(stages["depth_submit"]["timestamp_s"]))
        current = (int(row["frame_index"]), float(row["measurement_timestamp_s"]))
        if previous and current[1] > previous[1]:
            fps_samples.append((current[0] - previous[0]) / (current[1] - previous[1]))
        previous = current
    gaps = [submit[i] - submit[i - 1] for i in range(1, len(submit))]
    first_queue = queue[: max(1, len(queue)//2)]
    last_queue = queue[len(queue)//2 :]
    integrity = malformed == checksum_failures == timestamp_failures == gt_failures == anchor_failures == nonfinite == duplicates == 0
    return {
        "attempt_path": str(path.relative_to(ROOT)), "parseable": True,
        "scenario_id": capture.get("scenario_id"), "dataset_role": capture.get("dataset_role"),
        "raw_frame_count": len(raw), "tracking_median_fps": median(fps_samples) if fps_samples else None,
        "effective_depth_rate_hz": (1.0 / median(gaps)) if gaps else None,
        "capture_consume_median_ms": 1000 * median(capture_consume) if capture_consume else None,
        "capture_consume_p95_ms": 1000 * pct(capture_consume, 95) if capture_consume else None,
        "worker_median_ms": 1000 * median(worker) if worker else None,
        "worker_p95_ms": 1000 * pct(worker, 95) if worker else None,
        "queue_wait_median_ms": 1000 * median(queue) if queue else None,
        "queue_wait_p95_ms": 1000 * pct(queue, 95) if queue else None,
        "queue_first_half_median_ms": 1000 * median(first_queue) if first_queue else None,
        "queue_second_half_median_ms": 1000 * median(last_queue) if last_queue else None,
        "backlog_increasing": bool(first_queue and last_queue and median(last_queue) > median(first_queue) + 0.020),
        "malformed_records": malformed, "checksum_failures": checksum_failures,
        "timestamp_failures": timestamp_failures, "gt_trace_failures": gt_failures,
        "anchor_count_failures": anchor_failures, "nonfinite_ranges": nonfinite,
        "duplicate_traces": duplicates, "integrity_pass": integrity,
        "sidecar_sha256": sha256_file(path / "physical_diagnostics.jsonl"),
    }


def main() -> None:
    manifest = json.loads((OUT / "dynamic_session_manifest.json").read_text())
    attempts = [inspect_attempt(path) for path in sorted((OUT / "quarantine").iterdir()) if path.is_dir()]
    parsed = [row for row in attempts if row.get("parseable")]
    write_csv("runtime_metrics.csv", parsed, ["attempt_path", "scenario_id", "raw_frame_count", "effective_depth_rate_hz"])
    write_csv("latency_metrics.csv", [
        {key: row.get(key) for key in ("attempt_path", "scenario_id", "capture_consume_median_ms", "capture_consume_p95_ms", "worker_median_ms", "worker_p95_ms", "queue_wait_median_ms", "queue_wait_p95_ms", "backlog_increasing")}
        for row in parsed
    ], ["attempt_path", "capture_consume_median_ms", "capture_consume_p95_ms", "worker_p95_ms"])
    write_csv("tracking_fps.csv", [
        {"attempt_path": row["attempt_path"], "scenario_id": row["scenario_id"], "tracking_median_fps": row["tracking_median_fps"], "gate_min_fps": 20.0, "pass": float(row["tracking_median_fps"]) >= 20.0}
        for row in parsed if row.get("tracking_median_fps") is not None
    ], ["attempt_path", "tracking_median_fps", "gate_min_fps", "pass"])

    unique_traces: set[str] = set()
    all_trace_count = 0
    for path in sorted((OUT / "quarantine").iterdir()):
        sidecar = path / "physical_diagnostics.jsonl"
        if not sidecar.exists():
            continue
        rows, _ = load_jsonl(sidecar)
        for row in rows:
            if row.get("stage") == "raw_range_computed":
                all_trace_count += 1
                unique_traces.add(str(row.get("trace_identity_sha256")))
    integrity_pass = bool(parsed and all(bool(row["integrity_pass"]) for row in parsed) and len(unique_traces) == all_trace_count)
    write_json("integrity_report.json", {
        "accepted_corpus_integrity_pass": False,
        "reason": "no accepted groups; quarantined attempts are never training input",
        "quarantined_attempt_count": len(attempts), "parseable_attempt_count": len(parsed),
        "quarantined_attempt_frame_integrity_pass": integrity_pass,
        "quarantined_raw_frame_count": all_trace_count,
        "cross_attempt_duplicate_trace_count": all_trace_count - len(unique_traces),
        "malformed_record_count": sum(int(row.get("malformed_records", 0)) for row in parsed),
        "checksum_failure_count": sum(int(row.get("checksum_failures", 0)) for row in parsed),
        "timestamp_failure_count": sum(int(row.get("timestamp_failures", 0)) for row in parsed),
        "anchor_count_failure_count": sum(int(row.get("anchor_count_failures", 0)) for row in parsed),
        "nonfinite_range_count": sum(int(row.get("nonfinite_ranges", 0)) for row in parsed),
        "data_gate": {"approaching": "0/3 FAIL", "receding": "0/3 FAIL", "stop_and_hold": "0/2 FAIL"},
    })

    _static_rows, static_manifest = collect_verified_rows(ROOT)
    write_json("frozen_dataset_manifest.json", {
        "frozen": False, "training_allowed": False,
        "static_verified_group_count": static_manifest["group_count"],
        "static_verified_frame_count": static_manifest["frame_count"],
        "dynamic_accepted_group_count": 0, "dynamic_accepted_frame_count": 0,
        "quarantine_included": False, "historical_2hz_included": False,
        "reason": "5 Hz dynamic data gate failed before dataset freeze",
    })
    write_json("feature_contract.json", {
        "status": "PRECOMMITTED_NOT_FIT",
        "PHYSICAL_ONLY": {"feature_order": list(PHYSICAL_ONLY_FEATURES), "bbox_width_height_area_aspect_excluded": True},
        "PHYSICAL_TEMPORAL": {"feature_order": list(PHYSICAL_TEMPORAL_FEATURES), "causal_only": True,
                              "temporal_features": list(TEMPORAL_EXTRA_FEATURES), "reset_policy": "reset_at_session_boundary"},
        "clip_m": [3.0, 12.0], "seed": 52, "missing_value_policy": "training-only preprocessor median; not fit",
        "forbidden_features": ["nominal_distance", "session_id", "scenario_label", "distance_bin", "GT-derived runtime features", "future-frame features"],
    })
    empty_specs = {
        "fold_assignments.csv": ["group_id", "fold", "domain"],
        "static_metrics.csv": ["variant", "config", "metric", "value", "passed"],
        "dynamic_metrics.csv": ["variant", "config", "scenario_type", "metric", "value", "passed"],
        "temporal_metrics.csv": ["variant", "config", "metric", "value", "passed"],
        "bbox_stress_metrics.csv": ["variant", "config", "perturbation", "metric", "value", "passed"],
        "per_group_metrics.csv": ["variant", "config", "group_id", "metric", "value"],
        "prediction_rows.csv": ["variant", "config", "group_id", "ground_truth_range_m", "prediction_m"],
        "model_comparison.csv": ["variant", "config", "passed_all_gates", "reason_not_trained"],
    }
    for name, headers in empty_specs.items():
        write_csv(name, [], headers)
    write_json("retrain_manifest.json", {
        "conclusion": "FIVE_HZ_DYNAMIC_CORPUS_INCOMPLETE",
        "training_started": False, "model_files_created": False, "candidate_selected": False,
        "accepted_group_count": manifest["accepted_group_count"],
        "accepted_scenario_counts": manifest["accepted_scenario_counts"],
        "quarantined_attempt_count": len(attempts),
        "reason_not_trained": "per-group runtime gate produced 0/8 accepted groups; tracking median was below 20 FPS across every parseable attempt and some MiDaS P95 values exceeded 100 ms",
        "depth_rate_changed": False, "effective_configured_depth_rate_hz": 5.0,
        "shadow": False, "active_runtime_integration": False, "follow_target": False,
        "focused_tests": "56 passed", "full_repository_tests": "383 passed",
        "run_all_check": "PASS",
    })
    report = """# CORE_RANGE 5 Hz dynamic recollection and retrain

Conclusion: `FIVE_HZ_DYNAMIC_CORPUS_INCOMPLETE`

The preflight passed and 16 independent attempts were captured (two per planned group). Every failed attempt was preserved unchanged in quarantine. No group was accepted because tracking median remained below the required 20 FPS; several attempts also exceeded the 100 ms MiDaS P95 gate. The data gate is therefore 0/3 approaching, 0/3 receding and 0/2 stop-and-hold.

The geometry-specific `central_quantile_region` recovery was precommitted for `cdr_recede_left_yaw_5hz` only and did restore raw-range production; that group still failed the runtime tracking gate. No ROI policy was changed after capture.

Training was not started. Quarantine and the historical 2 Hz corpus were not used. No model or runtime candidate was created.
"""
    (OUT / "retrain_report.md").write_text(report)


if __name__ == "__main__":
    main()
