"""Collect extra independent repeats of specific Stage 2A groups for training.

`core_range_targeted_pilot_topup.py` fills in groups the 2026-08-07 batch run
never accepted. This script does something different: it adds *more*
successful attempts for groups that already have an accepted capture, so
`core_range_stage2a_relative_range_model.py` sees multiple independent
observations of the same scenario instead of exactly one.

Why this might help (and why it might not, honestly): four attempts to fix
the model's low within-scenario slope in three of twelve groups all failed
without new data (see docs/CORE_RANGE_STAGE_2A_FAILURE_AUDIT_20260806.md
section 10). More repeats give the regressor multiple independent noisy
draws of the same feature-to-distance relationship for exactly the geometry
that's weakest, which is the standard way to reduce bias/variance when a
relationship is only weakly identifiable from a single frame -- but it does
not add new sub-ranges within a trajectory, so it may reduce noise without
fixing the responsiveness gap. This collects the data; it does not by
itself claim the fix will work.

Extra attempts land in the same `raw_sessions/` directory as existing ones,
using the next free attempt index, so `extract_rows()`'s existing
`scenario_group` derivation (strip `_attempt_N`) folds them into the same
training group automatically -- no changes needed elsewhere.
"""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import subprocess
import time
from datetime import datetime, timezone

from core_range_collect_targeted_pilot_batch import (
    RETRY_COOLDOWN_S,
    audit,
    disk_preflight,
    quarantine,
    stale_processes,
)
from core_range_targeted_pilot_topup import next_attempt_index

DEFAULT_TARGET_GROUPS = (
    "pilot_far_lateral_yaw_recede",
    "pilot_mid_lateral_yaw_approach",
    "pilot_near_lateral_recede",
    "pilot_near_lateral_approach",
    "pilot_mid_lateral_yaw_recede",
)


def load_specs(output: Path, group_ids: tuple[str, ...]) -> list[dict]:
    plan = json.loads((output / "collection_plan_pilot.json").read_text())
    by_id = {s["group_id"]: s for s in plan["sessions"]}
    missing = [g for g in group_ids if g not in by_id]
    if missing:
        raise ValueError(f"unknown group_id(s): {missing}")
    return [by_id[g] for g in group_ids]


def collect_repeats(
    workspace: Path,
    output: Path,
    group_ids: tuple[str, ...],
    repeats_per_group: int,
    max_attempts_per_repeat: int,
) -> dict:
    specs = load_specs(output, group_ids)
    sessions_root = output / "raw_sessions"
    quarantine_root = output / "quarantine"
    sessions_root.mkdir(exist_ok=True)

    log_path = output / "repeat_collection_log.json"
    log: list[dict] = json.loads(log_path.read_text()) if log_path.exists() else []
    accepted_count: dict[str, int] = {}

    for spec in specs:
        group_id = spec["group_id"]
        accepted_count[group_id] = 0
        for repeat in range(repeats_per_group):
            success = False
            for _ in range(max_attempts_per_repeat):
                stale = stale_processes()
                if stale:
                    raise RuntimeError(f"stale_process_preflight:{stale}")
                disk_preflight(output, 1)
                attempt = next_attempt_index(sessions_root, group_id)
                root = sessions_root / f"{group_id}_attempt_{attempt}"
                bbox = spec["initial_bbox_normalized"]
                command = [
                    str(workspace / "core_range_collect_targeted_pilot_scenario.sh"),
                    str(spec["roi_policy"]), str(group_id),
                    str(spec["start_range_m"]), str(spec["end_range_m"]),
                    str(spec["lateral_offset_m"]), str(spec["target_z_m"]),
                    str(spec["target_yaw_start_deg"]), str(spec["target_yaw_end_deg"]),
                    str(spec["movement_duration_s"]), str(spec["hold_duration_s"]),
                    str(spec["pose_update_rate_hz"]), str(spec["gimbal_pitch_deg"]),
                    *(str(v) for v in bbox), str(root), "sim_time_plugin",
                ]
                print(f"COLLECTING {group_id} repeat {repeat + 1}/{repeats_per_group} "
                      f"attempt {attempt}", flush=True)
                record = {
                    "group_id": group_id, "repeat": repeat + 1, "attempt": attempt,
                    "started_utc": datetime.now(timezone.utc).isoformat(),
                }
                try:
                    subprocess.run(command, cwd=workspace, check=True)
                    subprocess.run([
                        str(workspace / ".venv/bin/python"),
                        str(workspace / "core_range_logging_eval.py"), str(root),
                        "--output", str(root / "audit"),
                        "--minimum-frames", str(spec["minimum_raw_frames"]),
                        "--require-single-raw-session",
                    ], cwd=workspace, env={
                        **os.environ,
                        "PYTHONPATH": str(workspace / ".venv/lib/python3.12/site-packages"),
                    }, check=True)
                    row = audit(root, spec, workspace)
                    record["result"] = "ACCEPTED"
                    record["frame_count"] = row["frame_count"]
                    print(f"ACCEPTED {group_id} repeat {repeat + 1} "
                          f"frames={row['frame_count']}", flush=True)
                    success = True
                    accepted_count[group_id] += 1
                except Exception as error:
                    destination = quarantine(root, quarantine_root, str(error))
                    record["result"] = "QUARANTINED"
                    record["reason"] = str(error)
                    record["path"] = str(destination.relative_to(workspace))
                    print(f"QUARANTINED {group_id} repeat {repeat + 1}: {error}", flush=True)
                record["ended_utc"] = datetime.now(timezone.utc).isoformat()
                log.append(record)
                log_path.write_text(json.dumps(log, indent=2), encoding="utf-8")
                if success:
                    break
                time.sleep(RETRY_COOLDOWN_S)
            if not success:
                print(f"FAILED {group_id} repeat {repeat + 1} "
                      f"(exhausted {max_attempts_per_repeat} attempts)", flush=True)

    print("\nREPEAT_COLLECTION_DONE " + json.dumps(accepted_count))
    return accepted_count


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--workspace", type=Path, default=Path.cwd())
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--groups", nargs="*", default=list(DEFAULT_TARGET_GROUPS))
    parser.add_argument("--repeats-per-group", type=int, default=2)
    parser.add_argument("--max-attempts-per-repeat", type=int, default=3)
    args = parser.parse_args()
    collect_repeats(
        args.workspace.resolve(), args.output.resolve(), tuple(args.groups),
        args.repeats_per_group, args.max_attempts_per_repeat,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
