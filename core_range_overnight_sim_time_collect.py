"""CORE_RANGE_OVERNIGHT_SIM_TIME_TRAIN collection driver.

Collects the 3 missing dynamic groups (2 receding + 1 stop_and_hold) using
the sim_time_plugin driver and production bbox/geometry from
collection_plan.json, reusing -- unmodified -- the same integrity/runtime
gate logic already used by the accepted 5/8 headless corpus
(core_range_collect_dynamic_5hz_headless_batch.audit(), which itself
requires core_range_logging_eval.py's CORE_LOGGING_READY gate, which
already enforces 0 GT_BRACKET_TIMEOUT / 100% ground_truth_trace_valid).

Does not touch the 5 already-accepted groups. Each attempt gets its own
directory; failures are quarantined unmodified. One record per attempt is
appended to attempt_ledger.csv.
"""

from __future__ import annotations

import argparse
import json
import os
import subprocess
import time
from datetime import datetime, timezone
from pathlib import Path

from core_range_collect_dynamic_5hz_headless_batch import audit
from core_range_collect_static_balance import quarantine, stale_processes

WORKSPACE = Path(__file__).resolve().parent
SCENARIO_SCRIPT = WORKSPACE / "core_range_collect_dynamic_5hz_headless_scenario.sh"
PLAN_PATH = WORKSPACE / "artifacts/core_range_3_12m/dynamic_5hz_headless_retrain/collection_plan.json"

LEDGER_COLUMNS = [
    "attempt_id", "group_role", "scenario", "seed", "start_time", "end_time",
    "result", "reason_code", "rtf_median", "rtf_long_stutter_pass",
    "camera_fps", "tracking_fps", "capture_consume_median_s",
    "capture_consume_p95_s", "midas_worker_p95_s", "raw_count",
    "gt_timeout_count", "checksum_status", "source_path", "quarantine_path",
    "accepted_manifest_path",
]


def _load_missing_specs(group_ids: list[str]) -> dict[str, dict]:
    plan = json.loads(PLAN_PATH.read_text())
    specs = {s["session_id"]: s for s in plan["sessions"]}
    return {gid: specs[gid] for gid in group_ids}


def _gt_timeout_count(root: Path) -> int:
    path = root / "physical_diagnostics.jsonl"
    if not path.exists():
        return 0
    count = 0
    for line in path.read_text(encoding="utf-8").splitlines():
        if not line.strip():
            continue
        try:
            row = json.loads(line)
        except json.JSONDecodeError:
            continue
        error = ((row.get("ground_truth") or {}).get("provider_diagnostics") or {}).get("error")
        if error == "GT_BRACKET_TIMEOUT":
            count += 1
    return count


def _append_ledger(ledger_path: Path, row: dict) -> None:
    is_new = not ledger_path.exists()
    with ledger_path.open("a", encoding="utf-8") as stream:
        if is_new:
            stream.write(",".join(LEDGER_COLUMNS) + "\n")
        values = [str(row.get(col, "")).replace(",", ";") for col in LEDGER_COLUMNS]
        stream.write(",".join(values) + "\n")


def run_attempt(
    spec: dict,
    attempt: int,
    output_root: Path,
    quarantine_root: Path,
    ledger_path: Path,
) -> dict | None:
    stale = stale_processes()
    if stale:
        raise RuntimeError(f"stale_process_preflight:{stale}")

    group_id = spec["session_id"]
    root = output_root / f"{group_id}_attempt_{attempt}"
    bbox = spec["initial_bbox_normalized"]
    command = [
        str(SCENARIO_SCRIPT), str(spec["roi_policy"]), str(group_id),
        str(spec["start_range_m"]), str(spec["end_range_m"]), str(spec["lateral_offset_m"]),
        str(spec["target_z_m"]), str(spec["target_yaw_start_deg"]), str(spec["target_yaw_end_deg"]),
        str(spec["movement_duration_s"]), str(spec["hold_duration_s"]), str(spec["pose_update_rate_hz"]),
        str(spec["gimbal_pitch_deg"]), *(str(x) for x in bbox), str(root), "sim_time_plugin",
    ]
    start_time = datetime.now(timezone.utc).isoformat()
    start_monotonic = time.monotonic()
    print(f"ATTEMPT {group_id} #{attempt}: {' '.join(command)}", flush=True)

    ledger_row = {
        "attempt_id": f"{group_id}_attempt_{attempt}", "group_role": group_id,
        "scenario": spec["scenario_type"], "seed": spec["seed"], "start_time": start_time,
    }

    try:
        subprocess.run(command, cwd=WORKSPACE, check=True, timeout=280)
    except subprocess.CalledProcessError as error:
        ledger_row.update({
            "end_time": datetime.now(timezone.utc).isoformat(), "result": "QUARANTINED",
            "reason_code": "TRAJECTORY_COMMAND_FAIL_OR_ENVIRONMENT_FAIL",
            "source_path": str(root),
        })
        destination = quarantine(root, quarantine_root, f"scenario_script_failed:{error}")
        ledger_row["quarantine_path"] = str(destination) if destination else ""
        _append_ledger(ledger_path, ledger_row)
        return None
    except subprocess.TimeoutExpired:
        ledger_row.update({
            "end_time": datetime.now(timezone.utc).isoformat(), "result": "QUARANTINED",
            "reason_code": "ENVIRONMENT_FAIL", "source_path": str(root),
        })
        destination = quarantine(root, quarantine_root, "scenario_script_timeout_280s")
        ledger_row["quarantine_path"] = str(destination) if destination else ""
        _append_ledger(ledger_path, ledger_row)
        return None

    gt_timeouts = _gt_timeout_count(root)

    try:
        subprocess.run(
            [
                str(WORKSPACE / ".venv/bin/python"), str(WORKSPACE / "core_range_logging_eval.py"),
                str(root), "--output", str(root / "audit"), "--minimum-frames", "40",
                "--require-single-raw-session",
            ],
            cwd=WORKSPACE,
            env={**os.environ, "PYTHONPATH": str(WORKSPACE / ".venv/lib/python3.12/site-packages")},
            check=True,
        )
        row = audit(root, spec, WORKSPACE)
    except Exception as error:
        reason = str(error)
        reason_code = "INTEGRITY_FAIL"
        if "core_logging_gate_failed" in reason:
            reason_code = "GT_BRACKET_TIMEOUT" if gt_timeouts else "INTEGRITY_FAIL"
        elif "'gazebo_rtf': False" in reason:
            reason_code = "RTF_HARD_GATE_FAIL"
        elif "'no_long_gazebo_stutter': False" in reason:
            reason_code = "RTF_LONG_STUTTER"
        elif "'tracking_fps': False" in reason:
            reason_code = "TRACKING_GATE_FAIL"
        elif "'camera_source_fps': False" in reason:
            reason_code = "CAMERA_GATE_FAIL"
        elif "'capture_consume_median': False" in reason or "'capture_consume_p95': False" in reason or "'worker_p95': False" in reason:
            reason_code = "LATENCY_GATE_FAIL"
        elif "'raw_frames': False" in reason:
            reason_code = "RAW_RANGE_INSUFFICIENT"
        elif "gt_coverage" in reason or "evidence_insufficient" in reason:
            reason_code = "TRAJECTORY_COMMAND_FAIL"
        elif "runtime_gate_failed" in reason:
            reason_code = "INTEGRITY_FAIL"
        ledger_row.update({
            "end_time": datetime.now(timezone.utc).isoformat(), "result": "QUARANTINED",
            "reason_code": reason_code, "gt_timeout_count": gt_timeouts,
            "source_path": str(root),
        })
        destination = quarantine(root, quarantine_root, reason)
        ledger_row["quarantine_path"] = str(destination) if destination else ""
        _append_ledger(ledger_path, ledger_row)
        return None

    if gt_timeouts:
        # Should be unreachable: CORE_LOGGING_READY already requires 100%
        # ground_truth_trace_valid, which GT_BRACKET_TIMEOUT rows fail.
        # Defense in depth only.
        ledger_row.update({
            "end_time": datetime.now(timezone.utc).isoformat(), "result": "QUARANTINED",
            "reason_code": "GT_BRACKET_TIMEOUT", "gt_timeout_count": gt_timeouts,
            "source_path": str(root),
        })
        destination = quarantine(root, quarantine_root, f"gt_bracket_timeout_count:{gt_timeouts}")
        ledger_row["quarantine_path"] = str(destination) if destination else ""
        _append_ledger(ledger_path, ledger_row)
        return None

    ledger_row.update({
        "end_time": datetime.now(timezone.utc).isoformat(), "result": "ACCEPTED",
        "reason_code": "ACCEPTED", "rtf_median": row["gazebo_rtf_median"],
        "camera_fps": row["camera_source_median_fps"], "tracking_fps": row["tracking_median_fps"],
        "capture_consume_median_s": row["capture_consume_median_s"],
        "capture_consume_p95_s": row["capture_consume_p95_s"],
        "midas_worker_p95_s": row["worker_p95_s"], "raw_count": row["frame_count"],
        "gt_timeout_count": 0, "checksum_status": row["integrity_status"],
        "source_path": row["source_root"],
    })
    _append_ledger(ledger_path, ledger_row)
    print(f"ACCEPTED {group_id} attempt {attempt}: frames={row['frame_count']} rtf={row['gazebo_rtf_median']}", flush=True)
    return row


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--groups", nargs="+", required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--max-attempts", type=int, default=12)
    parser.add_argument("--cooldown-s", type=float, default=75.0)
    args = parser.parse_args()

    args.output.mkdir(parents=True, exist_ok=True)
    quarantine_root = args.output / "quarantine"
    ledger_path = args.output / "attempt_ledger.csv"

    specs = _load_missing_specs(args.groups)
    accepted: dict[str, dict] = {}
    attempt_counts = {group_id: 0 for group_id in args.groups}
    remaining = list(args.groups)  # round-robin order, preserved as given (cycle-1 priority)
    cycle = 0
    while remaining:
        cycle += 1
        print(f"=== CYCLE {cycle}: {remaining} ===", flush=True)
        for group_id in list(remaining):
            attempt_counts[group_id] += 1
            attempt = attempt_counts[group_id]
            row = run_attempt(specs[group_id], attempt, args.output, quarantine_root, ledger_path)
            if row is not None:
                accepted[group_id] = row
                remaining.remove(group_id)
                print(f"GROUP DONE, stopping attempts: {group_id}", flush=True)
                continue
            if attempt >= args.max_attempts:
                remaining.remove(group_id)
                print(f"MISSING BUDGET EXHAUSTED: {group_id}", flush=True)
                continue
            print(f"cooldown {args.cooldown_s}s before next attempt", flush=True)
            time.sleep(args.cooldown_s)

    (args.output / "newly_accepted_sessions.json").write_text(
        json.dumps(list(accepted.values()), indent=2, sort_keys=True) + "\n"
    )
    print(json.dumps({"accepted_groups": list(accepted.keys())}, indent=2))
    return 0 if len(accepted) == len(args.groups) else 1


if __name__ == "__main__":
    raise SystemExit(main())
