"""Collect the 8-group dynamic development corpus at the new 2.0Hz depth
scheduler default (CORE_RANGE_2HZ_DYNAMIC_RECOLLECTION_AND_RETRAIN).

Trajectories are numerically identical to the frozen 8-session plan in
core_range_direct_dynamic_replay.py (same start/end range, lateral offset,
gimbal pitch, yaw, bbox, speed) so the new corpus is comparable to the old
one. Everything else here is new: a distinct dataset_role
("core_range_dynamic_2hz_development"), a distinct gate_id, no coupling to
the old frozen-candidate replay/verification path (this task explicitly
permits retraining, unlike that one), and up to two capture attempts per
session (quarantining every failed attempt) before moving on.
"""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import subprocess

from core_range_direct_dynamic_replay import _sessions as _replay_sessions
from core_range_logging_eval import load_jsonl, sha256_file, verify_record
from core_range_collect_static_balance import (
    archive_runtime_logs,
    disk_preflight,
    quarantine,
    stale_processes,
)
from range_physical_diagnostics import validate_timestamp_stages

GATE_ID = "core_range_dynamic_2hz_recollection_20260805_v001"
DATASET_ROLE = "core_range_dynamic_2hz_development"
SEED = 52
MAXIMUM_ATTEMPTS_PER_SESSION = 2


def _write_json(path: Path, value: object) -> None:
    path.write_text(json.dumps(value, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def _sessions() -> list[dict[str, object]]:
    sessions = []
    for session in _replay_sessions():
        adapted = dict(session)
        adapted["session_id"] = f"{session['session_id']}_2hz"
        adapted["group_id"] = adapted["session_id"]
        adapted["dataset_role"] = DATASET_ROLE
        sessions.append(adapted)
    return sessions


def audit_session(root: Path, planned: dict[str, object], workspace: Path) -> dict[str, object]:
    summary = json.loads((root / "audit/smoke_summary.json").read_text())
    if summary.get("conclusion") != "CORE_LOGGING_READY" or not all(summary.get("gates", {}).values()):
        raise ValueError("core_logging_gate_failed")
    capture = summary["capture"]
    if capture.get("dataset_role") != DATASET_ROLE:
        raise ValueError("dynamic_dataset_role_mismatch")
    runtime_session_id = int(capture["session_id"])
    rows, malformed = load_jsonl(root / "physical_diagnostics.jsonl")
    if malformed:
        raise ValueError("dynamic_malformed_sidecar")
    raw = [
        row for row in rows
        if row.get("stage") == "raw_range_computed" and int(row.get("session_id", -1)) == runtime_session_id
    ]
    if len(raw) < int(planned["minimum_raw_frames"]):
        raise ValueError(f"dynamic_minimum_frames:{len(raw)}")
    identities = {(str(row.get("run_id")), int(row.get("session_id")), str(row.get("group_id"))) for row in raw}
    if len(identities) != 1:
        raise ValueError("dynamic_one_group_session_failed")
    traces: set[str] = set()
    effective_rate_gap_samples: list[float] = []
    capture_to_consume_samples: list[float] = []
    for row in raw:
        record_ok, trace_ok = verify_record(row)
        timestamp_ok, _ = validate_timestamp_stages(row.get("timestamp_stages") or {})
        if not record_ok or not trace_ok or not timestamp_ok or row.get("ground_truth_trace_valid") is not True:
            raise ValueError("dynamic_frame_integrity_failed")
        if len(((row.get("anchors") or {}).get("per_grid_point") or [])) != 96:
            raise ValueError("dynamic_anchor_completeness_failed")
        trace = str(row["trace_identity_sha256"])
        if trace in traces:
            raise ValueError("dynamic_duplicate_trace")
        traces.add(trace)
        stages = row.get("timestamp_stages") or {}
        submit_s = (stages.get("depth_submit") or {}).get("timestamp_s")
        if submit_s is not None:
            effective_rate_gap_samples.append(float(submit_s))
        capture_s = (stages.get("frame_receipt") or {}).get("timestamp_s")
        consume_s = (stages.get("consume") or {}).get("timestamp_s")
        if capture_s is not None and consume_s is not None:
            capture_to_consume_samples.append(float(consume_s) - float(capture_s))
    gt = [float(row["ground_truth"]["distance_m"]) for row in raw]
    start, end = float(planned["start_range_m"]), float(planned["end_range_m"])
    if abs(gt[0] - start) > 0.75 or abs(gt[-1] - end) > 0.75 or min(gt) < 2.9 or max(gt) > 12.1:
        raise ValueError(f"dynamic_gt_coverage_failed:{gt[0]}:{gt[-1]}:{min(gt)}:{max(gt)}")
    differences = [gt[index] - gt[index - 1] for index in range(1, len(gt))]
    if end < start and sum(value < -0.01 for value in differences) < 8:
        raise ValueError("approaching_evidence_insufficient")
    if end > start and sum(value > 0.01 for value in differences) < 8:
        raise ValueError("receding_evidence_insufficient")
    if planned["scenario_type"] == "stop_and_hold" and sum(abs(value) <= 0.005 for value in differences[-10:]) < 6:
        raise ValueError("stop_hold_evidence_insufficient")
    effective_rate_gap_samples.sort()
    gaps = [
        effective_rate_gap_samples[index] - effective_rate_gap_samples[index - 1]
        for index in range(1, len(effective_rate_gap_samples))
    ]
    median_gap_s = sorted(gaps)[len(gaps) // 2] if gaps else float("nan")
    capture_to_consume_samples.sort()
    median_ctc_s = (
        capture_to_consume_samples[len(capture_to_consume_samples) // 2]
        if capture_to_consume_samples else float("nan")
    )
    p95_ctc_s = (
        capture_to_consume_samples[min(len(capture_to_consume_samples) - 1, int(round(0.95 * (len(capture_to_consume_samples) - 1))))]
        if capture_to_consume_samples else float("nan")
    )
    run_id, _, runtime_group_id = next(iter(identities))
    return {
        "session_id": planned["session_id"], "group_id": planned["group_id"],
        "scenario_type": planned["scenario_type"], "context": planned["context"],
        "run_id": run_id, "runtime_session_id": runtime_session_id, "runtime_group_id": runtime_group_id,
        "frame_count": len(raw), "gt_start_m": gt[0], "gt_end_m": gt[-1], "gt_min_m": min(gt), "gt_max_m": max(gt),
        "timestamp_ordering": "PASS", "gt_trace_checksum": "PASS", "record_checksum": "PASS",
        "anchors_per_frame": 96, "duplicate_trace_count": 0, "malformed_json_lines": 0,
        "median_depth_submit_gap_s": median_gap_s,
        "median_capture_to_consume_s": median_ctc_s,
        "p95_capture_to_consume_s": p95_ctc_s,
        "source_root": str(root.relative_to(workspace)),
        "source_sidecar_sha256": sha256_file(root / "physical_diagnostics.jsonl"),
        "source_capture_sha256": sha256_file(root / "capture_events.jsonl"),
        "source_trajectory_sha256": sha256_file(root / "trajectory_events.jsonl"),
        "integrity_status": "PASS",
    }


def prepare(output: Path) -> dict[str, object]:
    if output.exists() and any(output.iterdir()):
        raise ValueError("dynamic_2hz_collection_output_not_empty")
    output.mkdir(parents=True, exist_ok=True)
    sessions = _sessions()
    plan = {
        "gate_id": GATE_ID,
        "dataset_role": DATASET_ROLE,
        "created_before_collection": True,
        "seed": SEED,
        "scheduler_depth_rate_hz": 2.0,
        "maximum_attempts_per_session": MAXIMUM_ATTEMPTS_PER_SESSION,
        "sessions": sessions,
        "expected_group_count": 8,
        "expected_scenario_counts": {"approaching": 3, "receding": 3, "stop_and_hold": 2},
        "latency_gate": {
            "median_capture_to_consume_s_max": 0.200,
            "p95_capture_to_consume_s_max": 0.300,
        },
        "scope_guards": {
            "retrain_model": True,
            "hyperparameter_or_feature_tuning": "up_to_3_precommitted_configs_per_variant",
            "runtime_backend_controller_px4_change": False,
            "follow_offboard_arm_takeoff_mode_closed_loop": False,
            "residual_runtime_default": "off",
            "shadow_or_active_runtime_integration": False,
            "target_motion_source": "independent_gazebo_set_pose_script_not_model_output",
        },
    }
    plan_path = output / "collection_plan.json"
    _write_json(plan_path, plan)
    print(json.dumps({
        "phase": "DYNAMIC_2HZ_COLLECTION_PRECOMMIT_COMPLETE_NO_COLLECTION_YET",
        "plan_sha256": sha256_file(plan_path),
        "sessions": len(sessions),
    }, indent=2))
    return plan


def collect(workspace: Path, output: Path) -> dict[str, object]:
    plan = json.loads((output / "collection_plan.json").read_text())
    sessions_root = output / "runtime_sessions"
    quarantine_root = output / "quarantine"
    sessions_root.mkdir(exist_ok=True)
    accepted: list[dict[str, object]] = []
    failed: list[dict[str, object]] = []
    for index, session in enumerate(plan["sessions"]):
        session_ok = False
        last_error: Exception | None = None
        for attempt in range(1, MAXIMUM_ATTEMPTS_PER_SESSION + 1):
            root = sessions_root / f"{session['session_id']}_attempt_{attempt}"
            if root.exists():
                raise RuntimeError(f"dynamic_2hz_session_root_exists:{root}")
            stale = stale_processes()
            if stale:
                raise RuntimeError(f"stale_process_preflight:{stale}")
            disk_preflight(output, len(plan["sessions"]) - index)
            bbox = session["initial_bbox_normalized"]
            command = [
                str(workspace / "core_range_collect_dynamic_2hz_scenario.sh"), session["session_id"],
                str(session["start_range_m"]), str(session["end_range_m"]), str(session["lateral_offset_m"]), str(session["target_z_m"]),
                str(session["target_yaw_start_deg"]), str(session["target_yaw_end_deg"]), str(session["movement_duration_s"]), str(session["hold_duration_s"]),
                str(session["pose_update_rate_hz"]), str(session["gimbal_pitch_deg"]),
                str(bbox[0]), str(bbox[1]), str(bbox[2]), str(bbox[3]), str(root),
            ]
            print(f"COLLECTING {index + 1}/8 {session['session_id']} attempt {attempt}/{MAXIMUM_ATTEMPTS_PER_SESSION}", flush=True)
            try:
                subprocess.run(command, cwd=workspace, check=True)
                subprocess.run([
                    "python3", str(workspace / "core_range_logging_eval.py"), str(root),
                    "--output", str(root / "audit"), "--minimum-frames", str(session["minimum_raw_frames"]), "--require-single-raw-session",
                ], cwd=workspace, env={**os.environ, "PYTHONPATH": str(workspace / ".venv/lib/python3.12/site-packages")}, check=True)
                row = audit_session(root, session, workspace)
                archive = archive_runtime_logs(root)
                row["runtime_archive_sha256"] = archive["archive_sha256"]
                row["attempt"] = attempt
                accepted.append(row)
                print(f"ACCEPTED {session['session_id']} frames={row['frame_count']}", flush=True)
                session_ok = True
                break
            except Exception as error:
                last_error = error
                destination = quarantine(root, quarantine_root, str(error))
                print(f"QUARANTINED {session['session_id']} attempt {attempt}: {error} -> {destination}", flush=True)
        if not session_ok:
            failed.append({"session_id": session["session_id"], "reason": str(last_error), "attempts": MAXIMUM_ATTEMPTS_PER_SESSION})
    manifest = {
        "gate_id": plan["gate_id"], "dataset_role": DATASET_ROLE,
        "collection_plan_sha256": sha256_file(output / "collection_plan.json"),
        "accepted_sessions": accepted, "failed_sessions": failed,
        "accepted_group_count": len(accepted), "accepted_frame_count": sum(int(row["frame_count"]) for row in accepted),
        "accepted_scenario_counts": {
            scenario: sum(1 for row in accepted if row["scenario_type"] == scenario)
            for scenario in ("approaching", "receding", "stop_and_hold")
        },
        "collection_complete": len(accepted) == 8 and not failed,
        "scope_guards": plan["scope_guards"],
        "completed_utc": datetime.now(timezone.utc).isoformat(),
    }
    (output / "dynamic_session_manifest.json").write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return manifest


def main() -> int:
    parser = argparse.ArgumentParser()
    sub = parser.add_subparsers(dest="command", required=True)
    p = sub.add_parser("prepare")
    p.add_argument("--output", required=True, type=Path)
    p = sub.add_parser("collect")
    p.add_argument("--workspace", required=True, type=Path)
    p.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    if args.command == "prepare":
        prepare(args.output.resolve())
        return 0
    manifest = collect(args.workspace.resolve(), args.output.resolve())
    print(json.dumps(manifest, indent=2, sort_keys=True))
    return 0 if manifest["collection_complete"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
