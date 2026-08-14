"""Compose the raw-collection evidence deliverables for
CORE_RANGE_5HZ_HEADLESS_DYNAMIC_RECOLLECTION_AND_RETRAIN from
dynamic_session_manifest.json (accepted + quarantined sessions) and the
quarantine/ directory of failed attempts. Read-only with respect to the
collection itself; writes only the summary CSV/JSON deliverables.
"""

from __future__ import annotations

import csv
import json
import math
from pathlib import Path
from statistics import median
from typing import Any

import numpy as np

from core_range_camera_source_fps_audit import _interval_metrics, _load_jsonl, _parse_gz_stats_log
from core_range_logging_eval import load_jsonl, sha256_file, verify_record
from range_physical_diagnostics import validate_timestamp_stages

ROOT = Path(__file__).resolve().parent
OUT = ROOT / "artifacts/core_range_3_12m/dynamic_5hz_headless_retrain"
PREWARM_SKIP_S = 5.0


def _write_json(path: Path, value: Any) -> None:
    path.write_text(json.dumps(value, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def _write_csv(path: Path, rows: list[dict[str, Any]], headers: list[str]) -> None:
    keys = list(headers)
    for row in rows:
        for key in row:
            if key not in keys:
                keys.append(key)
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=keys)
        writer.writeheader()
        writer.writerows(rows)


def _pct(values: list[float], p: float) -> float | None:
    return None if not values else float(np.percentile(np.asarray(values), p))


def inspect_attempt(path: Path) -> dict[str, Any]:
    """Re-derive runtime/latency/integrity metrics for one attempt directory
    (accepted or quarantined), independent of whether it passed the gate."""
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
        stages = row.get("timestamp_stages") or {}
        try:
            submit.append(float(stages["depth_submit"]["timestamp_s"]))
            capture_consume.append(float(stages["consume"]["timestamp_s"]) - float(stages["frame_receipt"]["timestamp_s"]))
            worker.append(float(stages["depth_complete"]["timestamp_s"]) - float(stages["depth_worker_start"]["timestamp_s"]))
            queue.append(float(stages["depth_worker_start"]["timestamp_s"]) - float(stages["depth_submit"]["timestamp_s"]))
        except (KeyError, TypeError):
            pass
        current = (int(row["frame_index"]), float(row["measurement_timestamp_s"]))
        if previous and current[1] > previous[1]:
            fps_samples.append((current[0] - previous[0]) / (current[1] - previous[1]))
        previous = current
    gaps = [submit[i] - submit[i - 1] for i in range(1, len(submit))]
    first_queue = queue[: max(1, len(queue)//2)]
    last_queue = queue[len(queue)//2 :]
    integrity = malformed == checksum_failures == timestamp_failures == gt_failures == anchor_failures == nonfinite == duplicates == 0

    camera_events = [row for row in _load_jsonl(path / "camera_source_trace.jsonl") if row.get("event") == "camera_source_receipt"]
    camera_fps_values = []
    for drone_id in {row.get("drone_id") for row in camera_events if row.get("drone_id")}:
        drone_rows = sorted([row for row in camera_events if row.get("drone_id") == drone_id], key=lambda r: r.get("trace_monotonic_s", 0.0))
        if not drone_rows:
            continue
        t0 = drone_rows[0].get("trace_monotonic_s", 0.0)
        post = [r for r in drone_rows if r.get("trace_monotonic_s", t0) - t0 >= PREWARM_SKIP_S]
        use = post if len(post) >= 5 else drone_rows
        timestamps = [r.get("monotonic_receipt_s") for r in use if isinstance(r.get("monotonic_receipt_s"), (int, float))]
        metrics = _interval_metrics(timestamps)
        if metrics["median_fps"]:
            camera_fps_values.append(metrics["median_fps"])
    camera_source_median_fps = round(sum(camera_fps_values) / len(camera_fps_values), 3) if camera_fps_values else None
    gz_stats = _parse_gz_stats_log(path / "gz_world_stats.log")

    return {
        "attempt_path": str(path.relative_to(ROOT)), "parseable": True,
        "scenario_id": capture.get("scenario_id"), "dataset_role": capture.get("dataset_role"),
        "raw_frame_count": len(raw), "tracking_median_fps": median(fps_samples) if fps_samples else None,
        "camera_source_median_fps": camera_source_median_fps,
        "gazebo_rtf_median": gz_stats.get("real_time_factor_median"), "gazebo_rtf_p5": gz_stats.get("real_time_factor_p5"),
        "effective_depth_rate_hz": (1.0 / median(gaps)) if gaps else None,
        "capture_consume_median_ms": 1000 * median(capture_consume) if capture_consume else None,
        "capture_consume_p95_ms": 1000 * (_pct(capture_consume, 95) or 0) if capture_consume else None,
        "worker_median_ms": 1000 * median(worker) if worker else None,
        "worker_p95_ms": 1000 * (_pct(worker, 95) or 0) if worker else None,
        "queue_wait_median_ms": 1000 * median(queue) if queue else None,
        "backlog_increasing": bool(first_queue and last_queue and median(last_queue) > median(first_queue) + 0.020),
        "malformed_records": malformed, "checksum_failures": checksum_failures,
        "timestamp_failures": timestamp_failures, "gt_trace_failures": gt_failures,
        "anchor_count_failures": anchor_failures, "nonfinite_ranges": nonfinite,
        "duplicate_traces": duplicates, "integrity_pass": integrity,
        "sidecar_sha256": sha256_file(path / "physical_diagnostics.jsonl") if (path / "physical_diagnostics.jsonl").exists() else None,
        "accepted": False,
    }


def main() -> None:
    manifest = json.loads((OUT / "dynamic_session_manifest.json").read_text())
    accepted = manifest["accepted_sessions"]
    accepted_paths = {row["source_root"] for row in accepted}

    all_attempts: list[dict[str, Any]] = []
    for row in accepted:
        record = dict(row)
        record["attempt_path"] = row["source_root"]
        record["parseable"] = True
        record["raw_frame_count"] = row["frame_count"]
        record["accepted"] = True
        record["integrity_pass"] = True
        record["capture_consume_median_ms"] = row["capture_consume_median_s"] * 1000
        record["capture_consume_p95_ms"] = row["capture_consume_p95_s"] * 1000
        record["worker_median_ms"] = row["worker_median_s"] * 1000
        record["worker_p95_ms"] = row["worker_p95_s"] * 1000
        all_attempts.append(record)

    quarantine_dir = OUT / "quarantine"
    if quarantine_dir.exists():
        for path in sorted(quarantine_dir.iterdir()):
            if not path.is_dir():
                continue
            if str(path.relative_to(ROOT)) in accepted_paths:
                continue
            all_attempts.append(inspect_attempt(path))

    parsed = [row for row in all_attempts if row.get("parseable")]

    _write_csv(OUT / "runtime_metrics.csv", parsed, [
        "attempt_path", "accepted", "session_id", "scenario_id", "scenario_type", "raw_frame_count",
        "effective_depth_rate_hz", "integrity_pass",
    ])
    _write_csv(OUT / "latency_metrics.csv", [
        {key: row.get(key) for key in (
            "attempt_path", "accepted", "capture_consume_median_ms", "capture_consume_p95_ms",
            "worker_median_ms", "worker_p95_ms", "queue_wait_median_ms", "backlog_increasing",
        )}
        for row in parsed
    ], ["attempt_path", "accepted", "capture_consume_median_ms", "capture_consume_p95_ms", "worker_p95_ms", "backlog_increasing"])
    _write_csv(OUT / "camera_fps.csv", [
        {"attempt_path": row["attempt_path"], "accepted": row.get("accepted"),
         "camera_source_median_fps": row.get("camera_source_median_fps"),
         "gate_min_fps": 25.0, "pass": (row.get("camera_source_median_fps") or 0) >= 25.0}
        for row in parsed if row.get("camera_source_median_fps") is not None
    ], ["attempt_path", "accepted", "camera_source_median_fps", "gate_min_fps", "pass"])
    _write_csv(OUT / "tracking_fps.csv", [
        {"attempt_path": row["attempt_path"], "accepted": row.get("accepted"),
         "tracking_median_fps": row.get("tracking_median_fps"),
         "gate_min_fps": 20.0, "pass": (row.get("tracking_median_fps") or 0) >= 20.0}
        for row in parsed if row.get("tracking_median_fps") is not None
    ], ["attempt_path", "accepted", "tracking_median_fps", "gate_min_fps", "pass"])
    _write_csv(OUT / "gazebo_rtf.csv", [
        {"attempt_path": row["attempt_path"], "accepted": row.get("accepted"),
         "gazebo_rtf_median": row.get("gazebo_rtf_median"), "gazebo_rtf_p5": row.get("gazebo_rtf_p5"),
         "gate_min_median": 0.8, "pass": (row.get("gazebo_rtf_median") or 0) >= 0.8}
        for row in parsed if row.get("gazebo_rtf_median") is not None
    ], ["attempt_path", "accepted", "gazebo_rtf_median", "gazebo_rtf_p5", "gate_min_median", "pass"])

    unique_traces: set[str] = set()
    all_trace_count = 0
    for source in accepted:
        sidecar = ROOT / source["source_root"] / "physical_diagnostics.jsonl"
        if not sidecar.exists():
            continue
        rows, _ = load_jsonl(sidecar)
        for row in rows:
            if row.get("stage") == "raw_range_computed" and int(row.get("session_id", -1)) == int(source["runtime_session_id"]):
                all_trace_count += 1
                unique_traces.add(str(row.get("trace_identity_sha256")))
    integrity_pass = bool(accepted) and len(unique_traces) == all_trace_count and all(bool(row["integrity_status"] == "PASS") for row in accepted)

    _write_json(OUT / "integrity_report.json", {
        "accepted_corpus_integrity_pass": integrity_pass,
        "accepted_group_count": len(accepted), "accepted_frame_count": all_trace_count,
        "accepted_scenario_counts": manifest["accepted_scenario_counts"],
        "collection_complete": manifest["collection_complete"],
        "cross_session_duplicate_trace_count": all_trace_count - len(unique_traces),
        "quarantined_attempt_count": len(all_attempts) - len(accepted),
        "malformed_record_count": sum(int(row.get("malformed_records", 0)) for row in parsed if not row.get("accepted")),
        "checksum_failure_count": sum(int(row.get("checksum_failures", 0)) for row in parsed if not row.get("accepted")),
        "timestamp_failure_count": sum(int(row.get("timestamp_failures", 0)) for row in parsed if not row.get("accepted")),
        "anchor_count_failure_count": sum(int(row.get("anchor_count_failures", 0)) for row in parsed if not row.get("accepted")),
        "nonfinite_range_count": sum(int(row.get("nonfinite_ranges", 0)) for row in parsed if not row.get("accepted")),
        "data_gate": {
            key: f"{manifest['accepted_scenario_counts'].get(key, 0)}/{required} {'PASS' if manifest['accepted_scenario_counts'].get(key, 0) >= required else 'FAIL'}"
            for key, required in (("approaching", 3), ("receding", 3), ("stop_and_hold", 2))
        },
    })
    print(json.dumps({"accepted": len(accepted), "attempts_total": len(all_attempts), "integrity_pass": integrity_pass}, indent=2))


if __name__ == "__main__":
    main()
