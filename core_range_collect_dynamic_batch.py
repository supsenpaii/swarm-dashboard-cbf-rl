"""Collect and integrity-audit the frozen eight-session dynamic plan."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import shutil
import subprocess

from core_range_direct_dynamic_replay import DATASET_ROLE, verify_frozen_candidate
from core_range_logging_eval import load_jsonl, sha256_file, verify_record
from core_range_collect_static_balance import archive_runtime_logs, disk_preflight, quarantine, stale_processes
from range_physical_diagnostics import validate_timestamp_stages


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
    raw = [row for row in rows if row.get("stage") == "raw_range_computed" and int(row.get("session_id", -1)) == runtime_session_id]
    if len(raw) < int(planned["minimum_raw_frames"]):
        raise ValueError(f"dynamic_minimum_frames:{len(raw)}")
    identities = {(str(row.get("run_id")), int(row.get("session_id")), str(row.get("group_id"))) for row in raw}
    if len(identities) != 1:
        raise ValueError("dynamic_one_group_session_failed")
    traces = set()
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
    run_id, _, runtime_group_id = next(iter(identities))
    return {
        "session_id": planned["session_id"], "group_id": planned["group_id"],
        "scenario_type": planned["scenario_type"], "context": planned["context"],
        "run_id": run_id, "runtime_session_id": runtime_session_id, "runtime_group_id": runtime_group_id,
        "frame_count": len(raw), "gt_start_m": gt[0], "gt_end_m": gt[-1], "gt_min_m": min(gt), "gt_max_m": max(gt),
        "timestamp_ordering": "PASS", "gt_trace_checksum": "PASS", "record_checksum": "PASS",
        "anchors_per_frame": 96, "duplicate_trace_count": 0, "malformed_json_lines": 0,
        "source_root": str(root.relative_to(workspace)),
        "source_sidecar_sha256": sha256_file(root / "physical_diagnostics.jsonl"),
        "source_capture_sha256": sha256_file(root / "capture_events.jsonl"),
        "source_trajectory_sha256": sha256_file(root / "trajectory_events.jsonl"),
        "integrity_status": "PASS",
    }


def verify_reusable_session(
    row: dict[str, object], planned: dict[str, object], workspace: Path
) -> dict[str, object]:
    if row.get("session_id") != planned.get("session_id"):
        raise ValueError("reused_session_plan_identity_mismatch")
    root = workspace / str(row["source_root"])
    audited = audit_session(root, planned, workspace)
    for field in (
        "source_sidecar_sha256",
        "source_capture_sha256",
        "source_trajectory_sha256",
        "frame_count",
    ):
        if audited.get(field) != row.get(field):
            raise ValueError(f"reused_session_changed:{field}")
    archive_meta = json.loads((root / "runtime_log_archive.json").read_text())
    archive = root / str(archive_meta["archive_name"])
    if (
        archive_meta.get("read_test") != "PASS"
        or sha256_file(archive) != row.get("runtime_archive_sha256")
        or archive_meta.get("archive_sha256") != row.get("runtime_archive_sha256")
    ):
        raise ValueError("reused_session_archive_integrity_failed")
    audited["runtime_archive_sha256"] = row["runtime_archive_sha256"]
    audited["reused_from_prior_integrity_pass"] = True
    return audited


def collect(workspace: Path, output: Path) -> dict[str, object]:
    plan, _ = verify_frozen_candidate(workspace, output)
    sessions_root = output / "runtime_sessions"
    quarantine_root = output / "quarantine"
    sessions_root.mkdir(exist_ok=True)
    accepted, failed = [], []
    prior_manifest_path = output / "dynamic_session_manifest.json"
    prior_rows = {}
    if prior_manifest_path.is_file():
        prior = json.loads(prior_manifest_path.read_text())
        prior_rows = {
            str(row["session_id"]): row
            for row in prior.get("accepted_sessions", [])
        }
    for index, session in enumerate(plan["sessions"]):
        root = sessions_root / session["session_id"]
        if session["session_id"] in prior_rows:
            row = verify_reusable_session(
                prior_rows[session["session_id"]], session, workspace
            )
            accepted.append(row)
            print(f"REUSE_ACCEPTED {session['session_id']} frames={row['frame_count']}", flush=True)
            continue
        if root.exists():
            raise RuntimeError(f"dynamic_session_root_exists:{session['session_id']}")
        stale = stale_processes()
        if stale:
            raise RuntimeError(f"stale_process_preflight:{stale}")
        disk_preflight(output, len(plan["sessions"]) - index)
        bbox = session["initial_bbox_normalized"]
        command = [
            str(workspace / "core_range_collect_dynamic_scenario.sh"), session["session_id"],
            str(session["start_range_m"]), str(session["end_range_m"]), str(session["lateral_offset_m"]), str(session["target_z_m"]),
            str(session["target_yaw_start_deg"]), str(session["target_yaw_end_deg"]), str(session["movement_duration_s"]), str(session["hold_duration_s"]),
            str(session["pose_update_rate_hz"]), str(session["gimbal_pitch_deg"]),
            str(bbox[0]), str(bbox[1]), str(bbox[2]), str(bbox[3]), str(root),
        ]
        print(f"COLLECTING {index + 1}/8 {session['session_id']}", flush=True)
        try:
            subprocess.run(command, cwd=workspace, check=True)
            subprocess.run([
                "python3", str(workspace / "core_range_logging_eval.py"), str(root),
                "--output", str(root / "audit"), "--minimum-frames", str(session["minimum_raw_frames"]), "--require-single-raw-session",
            ], cwd=workspace, env={**os.environ, "PYTHONPATH": str(workspace / ".venv/lib/python3.12/site-packages")}, check=True)
            row = audit_session(root, session, workspace)
            archive = archive_runtime_logs(root)
            row["runtime_archive_sha256"] = archive["archive_sha256"]
            accepted.append(row)
            print(f"ACCEPTED {session['session_id']} frames={row['frame_count']}", flush=True)
        except Exception as error:
            destination = quarantine(root, quarantine_root, str(error))
            failed.append({"session_id": session["session_id"], "reason": str(error), "quarantine": None if destination is None else str(destination.relative_to(workspace))})
            print(f"QUARANTINED {session['session_id']}: {error}", flush=True)
            break
    manifest = {
        "gate_id": plan["gate_id"], "dataset_role": DATASET_ROLE,
        "dynamic_replay_plan_sha256": sha256_file(output / "dynamic_replay_plan.json"),
        "effective_amendments": {
            key: plan[key]
            for key in (
                "effective_amendment",
                "effective_amendment_v002",
                "effective_amendment_v003",
                "effective_amendment_v004",
                "effective_amendment_v005",
                "effective_amendment_v006",
            )
            if key in plan
        },
        "accepted_sessions": accepted, "failed_sessions": failed,
        "prior_quarantined_attempts": sorted(
            str(path.relative_to(workspace))
            for path in quarantine_root.glob("*_attempt_*")
            if path.is_dir()
        ),
        "accepted_group_count": len(accepted), "accepted_frame_count": sum(int(row["frame_count"]) for row in accepted),
        "collection_complete": len(accepted) == 8 and not failed,
        "scope_guards": plan["scope_guards"],
        "completed_utc": datetime.now(timezone.utc).isoformat(),
    }
    (output / "dynamic_session_manifest.json").write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return manifest


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--workspace", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    manifest = collect(args.workspace.resolve(), args.output.resolve())
    print(json.dumps(manifest, indent=2, sort_keys=True))
    return 0 if manifest["collection_complete"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
