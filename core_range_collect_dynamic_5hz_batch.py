"""Precommit, collect, quarantine and gate the post-contention 5 Hz corpus."""

from __future__ import annotations

import argparse
import json
import math
import os
from datetime import datetime, timezone
from pathlib import Path
import subprocess
from statistics import median

import numpy as np

from core_range_collect_dynamic_2hz_batch import _sessions as old_sessions
from core_range_collect_static_balance import archive_runtime_logs, disk_preflight, quarantine, stale_processes
from core_range_logging_eval import load_jsonl, sha256_file, verify_record
from range_physical_diagnostics import validate_timestamp_stages

GATE_ID = "core_range_dynamic_5hz_post_contention_fix_20260805_v001"
DATASET_ROLE = "core_range_dynamic_5hz_post_contention_fix"
SEED = 52
MAX_ATTEMPTS = 2


def write_json(path: Path, value: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, indent=2, sort_keys=True) + "\n")


def sessions() -> list[dict[str, object]]:
    values = []
    for source in old_sessions():
        item = dict(source)
        item["session_id"] = str(item["session_id"]).removesuffix("_2hz") + "_5hz"
        item["group_id"] = item["session_id"]
        item["dataset_role"] = DATASET_ROLE
        item["roi_policy"] = (
            "central_quantile_region"
            if item["session_id"] == "cdr_recede_left_yaw_5hz"
            else "default"
        )
        item["roi_policy_precommit_basis"] = (
            "dynamic_repair_and_replay/recovery_policy.json: H achieved 100% availability; official r4 validated"
            if item["roi_policy"] == "central_quantile_region"
            else "production default foreground-half+MAD; geometry-specific recovery not applicable"
        )
        values.append(item)
    return values


def prepare(output: Path) -> dict[str, object]:
    if output.exists() and any(output.iterdir()):
        raise ValueError("dynamic_5hz_output_not_empty")
    output.mkdir(parents=True, exist_ok=True)
    plan = {
        "gate_id": GATE_ID,
        "dataset_role": DATASET_ROLE,
        "created_before_collection": True,
        "seed": SEED,
        "scheduler_depth_rate_hz": 5.0,
        "maximum_attempts_per_session": MAX_ATTEMPTS,
        "sessions": sessions(),
        "expected_group_count": 8,
        "expected_scenario_counts": {"approaching": 3, "receding": 3, "stop_and_hold": 2},
        "per_group_gates": {
            "minimum_raw_frames": 40,
            "tracking_median_fps_min": 20.0,
            "capture_consume_median_s_max": 0.200,
            "capture_consume_p95_s_max": 0.300,
            "worker_p95_s_max": 0.100,
            "anchors_per_frame": 96,
            "effective_rate_hz": 5.0,
            "timestamp_checksum_integrity": "100%",
            "backlog": "not_increasing",
        },
        "scope_guards": {
            "trajectory_source": "independent_gazebo_set_pose_script",
            "model_prediction_controls_trajectory": False,
            "follow_target": False,
            "offboard_arm_takeoff": False,
            "hardware": False,
            "residual_mode": "off",
            "roi_recovery_scope": "cdr_recede_left_yaw_5hz_only; default-off globally",
        },
    }
    write_json(output / "collection_plan.json", plan)
    return plan


def percentile(values: list[float], p: float) -> float:
    return float(np.percentile(np.asarray(values, dtype=np.float64), p))


def audit(root: Path, planned: dict[str, object], workspace: Path) -> dict[str, object]:
    summary = json.loads((root / "audit/smoke_summary.json").read_text())
    if summary.get("conclusion") != "CORE_LOGGING_READY" or not all(summary.get("gates", {}).values()):
        raise ValueError("core_logging_gate_failed")
    capture = summary["capture"]
    if capture.get("dataset_role") != DATASET_ROLE:
        raise ValueError("dataset_role_mismatch")
    runtime_session = int(capture["session_id"])
    rows, malformed = load_jsonl(root / "physical_diagnostics.jsonl")
    if malformed:
        raise ValueError("malformed_sidecar")
    raw = [r for r in rows if r.get("stage") == "raw_range_computed" and int(r.get("session_id", -1)) == runtime_session]
    if len(raw) < 40:
        raise ValueError(f"minimum_raw_frames:{len(raw)}")
    traces: set[str] = set()
    submit: list[float] = []
    capture_consume: list[float] = []
    worker: list[float] = []
    queue: list[float] = []
    tracking_interval_fps: list[float] = []
    previous_frame = previous_time = None
    for row in raw:
        record_ok, trace_ok = verify_record(row)
        timestamp_ok, _ = validate_timestamp_stages(row.get("timestamp_stages") or {})
        if not record_ok or not trace_ok or not timestamp_ok or row.get("ground_truth_trace_valid") is not True:
            raise ValueError("frame_integrity_failed")
        if len((row.get("anchors") or {}).get("per_grid_point") or []) != 96:
            raise ValueError("anchor_count_not_96")
        trace = str(row["trace_identity_sha256"])
        if trace in traces:
            raise ValueError("duplicate_trace")
        traces.add(trace)
        gt = float(row["ground_truth"]["distance_m"])
        raw_range = float(row["raw_range"]["physics_slant_range_m"])
        if not math.isfinite(gt) or not math.isfinite(raw_range):
            raise ValueError("nonfinite_range")
        stages = row["timestamp_stages"]
        submit.append(float(stages["depth_submit"]["timestamp_s"]))
        capture_consume.append(float(stages["consume"]["timestamp_s"]) - float(stages["frame_receipt"]["timestamp_s"]))
        worker.append(float(stages["depth_complete"]["timestamp_s"]) - float(stages["depth_worker_start"]["timestamp_s"]))
        queue.append(float(stages["depth_worker_start"]["timestamp_s"]) - float(stages["depth_submit"]["timestamp_s"]))
        current_time = float(row["measurement_timestamp_s"])
        current_frame = int(row["frame_index"])
        if previous_time is not None and current_time > previous_time:
            tracking_interval_fps.append((current_frame - previous_frame) / (current_time - previous_time))
        previous_frame, previous_time = current_frame, current_time

    gaps = [submit[i] - submit[i - 1] for i in range(1, len(submit))]
    effective_rate = 1.0 / median(gaps)
    tracking_fps = median(tracking_interval_fps)
    queue_first = median(queue[: max(1, len(queue)//2)])
    queue_last = median(queue[len(queue)//2 :])
    backlog_increasing = queue_last > queue_first + 0.020
    gt = [float(r["ground_truth"]["distance_m"]) for r in raw]
    start, end = float(planned["start_range_m"]), float(planned["end_range_m"])
    if abs(gt[0] - start) > 0.75 or abs(gt[-1] - end) > 0.75:
        raise ValueError(f"gt_coverage:{gt[0]}:{gt[-1]}")
    differences = [gt[i] - gt[i-1] for i in range(1, len(gt))]
    if end < start and sum(x < -0.01 for x in differences) < 8:
        raise ValueError("approaching_evidence_insufficient")
    if end > start and sum(x > 0.01 for x in differences) < 8:
        raise ValueError("receding_evidence_insufficient")
    if planned["scenario_type"] == "stop_and_hold" and sum(abs(x) <= 0.005 for x in differences[-10:]) < 6:
        raise ValueError("stop_hold_evidence_insufficient")
    gates = {
        "raw_frames": len(raw) >= 40,
        "tracking_fps": tracking_fps >= 20.0,
        "capture_consume_median": median(capture_consume) <= 0.200,
        "capture_consume_p95": percentile(capture_consume, 95) <= 0.300,
        "worker_p95": percentile(worker, 95) <= 0.100,
        "backlog": not backlog_increasing,
        "effective_rate": 4.0 <= effective_rate <= 6.0,
    }
    if not all(gates.values()):
        raise ValueError(f"runtime_gate_failed:{gates}")
    archive = archive_runtime_logs(root)
    return {
        "session_id": planned["session_id"], "group_id": planned["group_id"],
        "scenario_type": planned["scenario_type"], "context": planned["context"],
        "roi_policy": planned["roi_policy"], "run_id": raw[0]["run_id"],
        "runtime_session_id": runtime_session, "frame_count": len(raw),
        "gt_start_m": gt[0], "gt_end_m": gt[-1], "gt_min_m": min(gt), "gt_max_m": max(gt),
        "tracking_median_fps": tracking_fps, "effective_depth_rate_hz": effective_rate,
        "capture_consume_median_s": median(capture_consume), "capture_consume_p95_s": percentile(capture_consume, 95),
        "worker_median_s": median(worker), "worker_p95_s": percentile(worker, 95),
        "queue_wait_median_s": median(queue), "queue_wait_p95_s": percentile(queue, 95),
        "queue_first_half_median_s": queue_first, "queue_second_half_median_s": queue_last,
        "backlog_increasing": backlog_increasing, "timestamp_ordering": "PASS",
        "gt_trace_checksum": "PASS", "record_checksum": "PASS", "anchors_per_frame": 96,
        "duplicate_trace_count": 0, "malformed_json_lines": 0, "raw_range_finite": True,
        "source_root": str(root.relative_to(workspace)),
        "source_sidecar_sha256": sha256_file(root / "physical_diagnostics.jsonl"),
        "source_capture_sha256": sha256_file(root / "capture_events.jsonl"),
        "source_trajectory_sha256": sha256_file(root / "trajectory_events.jsonl"),
        "runtime_archive_sha256": archive["archive_sha256"], "gates": gates, "integrity_status": "PASS",
    }


def collect(workspace: Path, output: Path) -> dict[str, object]:
    plan = json.loads((output / "collection_plan.json").read_text())
    accepted: list[dict[str, object]] = []
    quarantined: list[dict[str, object]] = []
    sessions_root = output / "runtime_sessions"
    quarantine_root = output / "quarantine"
    sessions_root.mkdir(exist_ok=True)
    for index, spec in enumerate(plan["sessions"]):
        success = False
        for attempt in range(1, MAX_ATTEMPTS + 1):
            stale = stale_processes()
            if stale:
                raise RuntimeError(f"stale_process_preflight:{stale}")
            disk_preflight(output, 8 - index)
            root = sessions_root / f"{spec['session_id']}_attempt_{attempt}"
            bbox = spec["initial_bbox_normalized"]
            command = [
                str(workspace / "core_range_collect_dynamic_5hz_scenario.sh"), str(spec["roi_policy"]), str(spec["session_id"]),
                str(spec["start_range_m"]), str(spec["end_range_m"]), str(spec["lateral_offset_m"]), str(spec["target_z_m"]),
                str(spec["target_yaw_start_deg"]), str(spec["target_yaw_end_deg"]), str(spec["movement_duration_s"]), str(spec["hold_duration_s"]),
                str(spec["pose_update_rate_hz"]), str(spec["gimbal_pitch_deg"]), *(str(x) for x in bbox), str(root),
            ]
            print(f"COLLECTING {index+1}/8 {spec['session_id']} attempt {attempt}/{MAX_ATTEMPTS}", flush=True)
            try:
                subprocess.run(command, cwd=workspace, check=True)
                subprocess.run([
                    str(workspace / ".venv/bin/python"), str(workspace / "core_range_logging_eval.py"), str(root),
                    "--output", str(root / "audit"), "--minimum-frames", "40", "--require-single-raw-session",
                ], cwd=workspace, env={**os.environ, "PYTHONPATH": str(workspace / ".venv/lib/python3.12/site-packages")}, check=True)
                row = audit(root, spec, workspace)
                row["attempt"] = attempt
                accepted.append(row)
                print(f"ACCEPTED {spec['session_id']} frames={row['frame_count']}", flush=True)
                success = True
                break
            except Exception as error:
                destination = quarantine(root, quarantine_root, str(error))
                quarantined.append({"session_id": spec["session_id"], "attempt": attempt, "reason": str(error), "path": str(destination.relative_to(workspace))})
                print(f"QUARANTINED {spec['session_id']}: {error}", flush=True)
        if not success:
            print(f"FAILED {spec['session_id']}", flush=True)
    counts = {kind: sum(r["scenario_type"] == kind for r in accepted) for kind in ("approaching", "receding", "stop_and_hold")}
    complete = len(accepted) == 8 and counts == {"approaching": 3, "receding": 3, "stop_and_hold": 2}
    manifest = {
        "gate_id": GATE_ID, "dataset_role": DATASET_ROLE,
        "collection_plan_sha256": sha256_file(output / "collection_plan.json"),
        "accepted_sessions": accepted, "quarantined_sessions": quarantined,
        "accepted_group_count": len(accepted), "accepted_frame_count": sum(int(r["frame_count"]) for r in accepted),
        "accepted_scenario_counts": counts, "collection_complete": complete,
        "completed_utc": datetime.now(timezone.utc).isoformat(), "scope_guards": plan["scope_guards"],
    }
    write_json(output / "dynamic_session_manifest.json", manifest)
    write_json(output / "accepted_sessions.json", accepted)
    write_json(output / "quarantined_sessions.json", quarantined)
    return manifest


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("command", choices=("prepare", "collect"))
    parser.add_argument("--workspace", type=Path, default=Path.cwd())
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    output = args.output.resolve()
    result = prepare(output) if args.command == "prepare" else collect(args.workspace.resolve(), output)
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0 if args.command == "prepare" or result["collection_complete"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
