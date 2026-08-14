"""CORE_RANGE_TARGETED_DYNAMIC_PILOT_COLLECTION_STAGE_2A collector.

Adaptation of core_range_collect_dynamic_5hz_headless_batch.py (left
unmodified -- it belongs to the accepted 8-group dynamic corpus and its own
manifest references it by name) for the 14-scenario targeted pilot defined
in artifacts/core_range_3_12m/targeted_dynamic_pilot/pilot_precommit.json.

Reuses the same integrity/runtime-gate logic, helper functions, and
per-session shell driver pattern (core_range_collect_targeted_pilot_scenario.sh,
itself a dataset-role-only variant of the accepted headless scenario
script). The only behavioral differences, all driven by the task's own
Section 6-9 spec:
  - sessions are loaded from the precommitted pilot plan, not a hardcoded
    8-group list;
  - MAX_ATTEMPTS = 3 (task Section 8), not 8;
  - GT endpoint tolerance is tightened to 0.30m (vs. the original 0.75m)
    because several bands here span only 0.8-1.8m, where 0.75m tolerance
    would not meaningfully distinguish an in-band trajectory from one that
    drifted out of it; 0.30m still comfortably exceeds the ~0.15m endpoint
    deviation observed in the accepted 8-group corpus;
  - an additional band-compliance gate enforces the task's own hard floor
    (near band: GT never < 3.0m) / hard ceiling (far band: GT never >
    12.0m) and a soft +/-0.15m margin on the declared band edges elsewhere,
    directly implementing Section 6's "trajectory phai nam chu yeu trong
    vung X" / "khong di xuong duoi 3m" / "khong vuot qua 12m" constraints;
  - stop-and-hold sessions are additionally checked for a stable hold-phase
    GT within +/-0.15m of the declared hold target.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import subprocess
import time
from datetime import datetime, timezone
from pathlib import Path
from statistics import median

from core_range_collect_dynamic_5hz_headless_batch import (
    GAZEBO_RTF_MEDIAN_MIN,
    _camera_source_median_fps,
    _gazebo_rtf_quality,
    percentile,
)
from core_range_collect_static_balance import archive_runtime_logs, disk_preflight, quarantine, stale_processes
from core_range_logging_eval import load_jsonl, sha256_file, verify_record
from range_physical_diagnostics import validate_timestamp_stages

GATE_ID = "targeted_dynamic_pilot_stage_2a_20260806_v001"
DATASET_ROLE = "targeted_dynamic_pilot_stage_2a"
MAX_ATTEMPTS = 3
RETRY_COOLDOWN_S = 10.0
GT_ENDPOINT_TOLERANCE_M = 0.30
BAND_SOFT_MARGIN_M = 0.15
HOLD_STABILITY_TOLERANCE_M = 0.15
WORKSPACE = Path(__file__).resolve().parent
PRECOMMIT_PATH = WORKSPACE / "artifacts/core_range_3_12m/targeted_dynamic_pilot/pilot_precommit.json"

BAND_BOUNDS = {
    "near_3_4m": {"lo": 3.0, "hi": 4.0, "hard_lo": 3.0, "hard_hi": None},
    "mid_4_5_8m": {"lo": 4.5, "hi": 8.0, "hard_lo": None, "hard_hi": None},
    "far_10_12m": {"lo": 10.0, "hi": 12.0, "hard_lo": None, "hard_hi": 12.0},
    "stop_and_hold": {"lo": None, "hi": None, "hard_lo": None, "hard_hi": None},
}


def write_json(path: Path, value: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, indent=2, sort_keys=True) + "\n")


def sessions() -> list[dict[str, object]]:
    precommit = json.loads(PRECOMMIT_PATH.read_text())
    return list(precommit["sessions"])


def prepare(output: Path) -> dict[str, object]:
    if output.exists() and any(p.name not in {"plots"} for p in output.iterdir()):
        pass  # targeted_dynamic_pilot/ already holds Phase 0/1 deliverables; do not require empty
    output.mkdir(parents=True, exist_ok=True)
    precommit = json.loads(PRECOMMIT_PATH.read_text())
    plan = {
        "gate_id": GATE_ID,
        "dataset_role": DATASET_ROLE,
        "created_before_collection": True,
        "seed": precommit["seed"],
        "maximum_attempts_per_session": MAX_ATTEMPTS,
        "sessions": precommit["sessions"],
        "expected_group_count": 14,
        "expected_scenario_counts": {"approaching": 6, "receding": 6, "stop_and_hold": 2},
    }
    write_json(output / "collection_plan_pilot.json", plan)
    return plan


def _band_gate(spec: dict[str, object], gt: list[float]) -> dict[str, object]:
    band = str(spec["band"])
    bounds = BAND_BOUNDS[band]
    ok = True
    reason = None
    if band != "stop_and_hold":
        lo, hi = bounds["lo"], bounds["hi"]
        soft_lo = lo - BAND_SOFT_MARGIN_M
        soft_hi = hi + BAND_SOFT_MARGIN_M
        hard_lo = bounds["hard_lo"]
        hard_hi = bounds["hard_hi"]
        gt_min, gt_max = min(gt), max(gt)
        if gt_min < soft_lo or gt_max > soft_hi:
            ok, reason = False, f"band_soft_bounds_violated:{gt_min}:{gt_max}:[{soft_lo},{soft_hi}]"
        if hard_lo is not None and gt_min < hard_lo:
            ok, reason = False, f"band_hard_floor_violated:{gt_min}<{hard_lo}"
        if hard_hi is not None and gt_max > hard_hi:
            ok, reason = False, f"band_hard_ceiling_violated:{gt_max}>{hard_hi}"
    else:
        hold_target = float(spec["end_range_m"])
        hold_n = max(1, int(round(float(spec["hold_duration_s"]) * float(spec["pose_update_rate_hz"]))))
        hold_window = gt[-hold_n:]
        if not hold_window:
            ok, reason = False, "hold_window_empty"
        else:
            max_dev = max(abs(v - hold_target) for v in hold_window)
            if max_dev > HOLD_STABILITY_TOLERANCE_M:
                ok, reason = False, f"hold_phase_unstable:max_dev={max_dev}>{HOLD_STABILITY_TOLERANCE_M}"
    return {"pass": ok, "reason": reason}


def audit(root: Path, spec: dict[str, object], workspace: Path) -> dict[str, object]:
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
    if len(raw) < int(spec["minimum_raw_frames"]):
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
        gt_v = float(row["ground_truth"]["distance_m"])
        raw_range = float(row["raw_range"]["physics_slant_range_m"])
        if not math.isfinite(gt_v) or not math.isfinite(raw_range):
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
    queue_first = median(queue[: max(1, len(queue) // 2)])
    queue_last = median(queue[len(queue) // 2:])
    backlog_increasing = queue_last > queue_first + 0.020
    gt = [float(r["ground_truth"]["distance_m"]) for r in raw]
    start, end = float(spec["start_range_m"]), float(spec["end_range_m"])
    if abs(gt[0] - start) > GT_ENDPOINT_TOLERANCE_M or abs(gt[-1] - end) > GT_ENDPOINT_TOLERANCE_M:
        raise ValueError(f"gt_coverage:{gt[0]}:{gt[-1]}:tolerance={GT_ENDPOINT_TOLERANCE_M}")
    differences = [gt[i] - gt[i - 1] for i in range(1, len(gt))]
    scenario_type = spec["scenario_type"]
    # The near-band plan intentionally moves at 0.04 m/s and samples at
    # 5 Hz, so its expected per-sample displacement is only 0.008 m.  A
    # fixed 0.01 m direction threshold made a valid near trajectory
    # mathematically unable to pass.  Keep the established 0.01 m ceiling
    # for faster scenarios, but derive a conservative half-step threshold
    # for slow precommitted trajectories.
    expected_step_m = abs(float(spec["speed_m_s"])) / float(spec["pose_update_rate_hz"])
    direction_step_min_m = min(0.01, max(0.001, 0.5 * expected_step_m))
    if scenario_type == "approaching" and sum(x < -direction_step_min_m for x in differences) < 8:
        raise ValueError("approaching_evidence_insufficient")
    if scenario_type == "receding" and sum(x > direction_step_min_m for x in differences) < 8:
        raise ValueError("receding_evidence_insufficient")
    if scenario_type == "stop_and_hold" and sum(abs(x) <= 0.005 for x in differences[-10:]) < 6:
        raise ValueError("stop_hold_evidence_insufficient")

    band_gate = _band_gate(spec, gt)
    if not band_gate["pass"]:
        raise ValueError(f"band_compliance_failed:{band_gate['reason']}")

    camera_source_median_fps = _camera_source_median_fps(root)
    gazebo_rtf_quality = _gazebo_rtf_quality(root / "gz_world_stats.log")
    gazebo_rtf_median = gazebo_rtf_quality["median"]
    gui_client_processes = subprocess.run(
        ["pgrep", "-f", "gz sim -g"], check=False, capture_output=True, text=True,
    ).stdout.strip()

    gates = {
        "raw_frames": len(raw) >= int(spec["minimum_raw_frames"]),
        "tracking_fps": tracking_fps >= 20.0,
        "camera_source_fps": camera_source_median_fps is not None and camera_source_median_fps >= 25.0,
        "gazebo_rtf": gazebo_rtf_median is not None and gazebo_rtf_median >= GAZEBO_RTF_MEDIAN_MIN,
        "no_long_gazebo_stutter": bool(gazebo_rtf_quality["long_stutter_pass"]),
        "no_gazebo_gui_process": gui_client_processes == "",
        "capture_consume_median": median(capture_consume) <= 0.200,
        "capture_consume_p95": percentile(capture_consume, 95) <= 0.300,
        "worker_p95": percentile(worker, 95) <= 0.100,
        "backlog": not backlog_increasing,
        "effective_rate": 4.0 <= effective_rate <= 6.0,
        "band_compliance": band_gate["pass"],
    }
    if not all(gates.values()):
        raise ValueError(f"runtime_gate_failed:{gates}")
    archive = archive_runtime_logs(root)
    return {
        "group_id": spec["group_id"], "session_id": spec["session_id"], "band": spec["band"],
        "scenario_type": spec["scenario_type"], "context": spec["context"], "roi_policy": spec["roi_policy"],
        "run_id": raw[0]["run_id"], "runtime_session_id": runtime_session, "frame_count": len(raw),
        "gt_start_m": gt[0], "gt_end_m": gt[-1], "gt_min_m": min(gt), "gt_max_m": max(gt),
        "tracking_median_fps": tracking_fps, "camera_source_median_fps": camera_source_median_fps,
        "gazebo_rtf_median": gazebo_rtf_median, "effective_depth_rate_hz": effective_rate,
        "gazebo_rtf_sample_count": gazebo_rtf_quality["sample_count"],
        "gazebo_longest_stutter_samples": gazebo_rtf_quality["longest_consecutive_rtf_le_0_3_samples"],
        "capture_consume_median_s": median(capture_consume), "capture_consume_p95_s": percentile(capture_consume, 95),
        "worker_median_s": median(worker), "worker_p95_s": percentile(worker, 95),
        "queue_wait_median_s": median(queue), "queue_wait_p95_s": percentile(queue, 95),
        "queue_first_half_median_s": queue_first, "queue_second_half_median_s": queue_last,
        "backlog_increasing": backlog_increasing, "timestamp_ordering": "PASS",
        "gt_trace_checksum": "PASS", "record_checksum": "PASS", "anchors_per_frame": 96,
        "duplicate_trace_count": 0, "malformed_json_lines": 0, "raw_range_finite": True,
        "band_compliance_reason": band_gate["reason"],
        "direction_step_min_m": direction_step_min_m,
        "source_root": str(root.relative_to(workspace)),
        "source_sidecar_sha256": sha256_file(root / "physical_diagnostics.jsonl"),
        "source_capture_sha256": sha256_file(root / "capture_events.jsonl"),
        "source_trajectory_sha256": sha256_file(root / "trajectory_events.jsonl"),
        "runtime_archive_sha256": archive["archive_sha256"], "gates": gates, "integrity_status": "PASS",
    }


def collect(workspace: Path, output: Path) -> dict[str, object]:
    plan = json.loads((output / "collection_plan_pilot.json").read_text())
    attempts_log: list[dict[str, object]] = []
    accepted: list[dict[str, object]] = []
    quarantined: list[dict[str, object]] = []
    sessions_root = output / "raw_sessions"
    quarantine_root = output / "quarantine"
    sessions_root.mkdir(exist_ok=True)
    for index, spec in enumerate(plan["sessions"]):
        success = False
        for attempt in range(1, MAX_ATTEMPTS + 1):
            stale = stale_processes()
            if stale:
                raise RuntimeError(f"stale_process_preflight:{stale}")
            disk_preflight(output, 14 - index)
            root = sessions_root / f"{spec['session_id']}_attempt_{attempt}"
            bbox = spec["initial_bbox_normalized"]
            command = [
                str(workspace / "core_range_collect_targeted_pilot_scenario.sh"), str(spec["roi_policy"]), str(spec["session_id"]),
                str(spec["start_range_m"]), str(spec["end_range_m"]), str(spec["lateral_offset_m"]), str(spec["target_z_m"]),
                str(spec["target_yaw_start_deg"]), str(spec["target_yaw_end_deg"]), str(spec["movement_duration_s"]), str(spec["hold_duration_s"]),
                str(spec["pose_update_rate_hz"]), str(spec["gimbal_pitch_deg"]), *(str(x) for x in bbox), str(root), "sim_time_plugin",
            ]
            print(f"COLLECTING {index + 1}/14 {spec['session_id']} attempt {attempt}/{MAX_ATTEMPTS}", flush=True)
            attempt_record = {"session_id": spec["session_id"], "attempt": attempt, "started_utc": datetime.now(timezone.utc).isoformat()}
            try:
                subprocess.run(command, cwd=workspace, check=True)
                subprocess.run([
                    str(workspace / ".venv/bin/python"), str(workspace / "core_range_logging_eval.py"), str(root),
                    "--output", str(root / "audit"), "--minimum-frames", str(spec["minimum_raw_frames"]), "--require-single-raw-session",
                ], cwd=workspace, env={**os.environ, "PYTHONPATH": str(workspace / ".venv/lib/python3.12/site-packages")}, check=True)
                row = audit(root, spec, workspace)
                row["attempt"] = attempt
                accepted.append(row)
                attempt_record["result"] = "ACCEPTED"
                print(f"ACCEPTED {spec['session_id']} frames={row['frame_count']} camera_fps={row['camera_source_median_fps']} rtf={row['gazebo_rtf_median']}", flush=True)
                success = True
            except Exception as error:
                destination = quarantine(root, quarantine_root, str(error))
                quarantined.append({"session_id": spec["session_id"], "attempt": attempt, "reason": str(error), "path": str(destination.relative_to(workspace))})
                attempt_record["result"] = "QUARANTINED"
                attempt_record["reason"] = str(error)
                print(f"QUARANTINED {spec['session_id']}: {error}", flush=True)
            attempt_record["ended_utc"] = datetime.now(timezone.utc).isoformat()
            attempts_log.append(attempt_record)
            if success:
                break
            time.sleep(RETRY_COOLDOWN_S)
        if not success:
            print(f"FAILED {spec['session_id']}", flush=True)
    counts = {kind: sum(r["scenario_type"] == kind for r in accepted) for kind in ("approaching", "receding", "stop_and_hold")}
    complete = len(accepted) == 14 and counts == {"approaching": 6, "receding": 6, "stop_and_hold": 2}
    manifest = {
        "gate_id": GATE_ID, "dataset_role": DATASET_ROLE,
        "collection_plan_sha256": sha256_file(output / "collection_plan_pilot.json"),
        "accepted_sessions": accepted, "quarantined_sessions": quarantined,
        "accepted_group_count": len(accepted), "accepted_frame_count": sum(int(r["frame_count"]) for r in accepted),
        "accepted_scenario_counts": counts, "collection_complete": complete,
        "completed_utc": datetime.now(timezone.utc).isoformat(),
    }
    write_json(output / "accepted_group_manifest.json", manifest)
    write_json(output / "quarantine_manifest.json", quarantined)
    write_json(output / "attempt_manifest.json", attempts_log)
    return manifest


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("command", choices=("prepare", "collect"))
    parser.add_argument("--workspace", type=Path, default=Path.cwd())
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    output = args.output.resolve()
    if args.command == "prepare":
        result = prepare(output)
    else:
        result = collect(args.workspace.resolve(), output)
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0 if args.command == "prepare" or result["collection_complete"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
