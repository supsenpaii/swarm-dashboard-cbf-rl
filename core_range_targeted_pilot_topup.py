"""Collect only the Stage 2A groups a previous batch run failed to accept.

`core_range_collect_targeted_pilot_batch.collect` walks all fourteen sessions
unconditionally and the scenario script refuses to reuse an existing attempt
directory, so re-running it cannot fill in the gaps left by a partial run. The
2026-08-07 run accepted twelve of fourteen; the two it lost had failed the same
way on 2026-08-06 and then passed on retry, so the failures are probabilistic
and more attempts are the right response.

This tops up only the missing groups, using the same scenario script, the same
`core_range_logging_eval` invocation and the same `audit()` acceptance gate as
the batch, then rewrites the manifests with the merged result. It never edits or
deletes an already-accepted session, and it keeps a timestamped copy of the
manifest it replaces.
"""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import shutil
import subprocess
import time
from datetime import datetime, timezone

from core_range_collect_targeted_pilot_batch import (
    DATASET_ROLE,
    GATE_ID,
    RETRY_COOLDOWN_S,
    audit,
    disk_preflight,
    quarantine,
    sha256_file,
    stale_processes,
    write_json,
)


def missing_groups(output: Path) -> list[dict[str, object]]:
    plan = json.loads((output / "collection_plan_pilot.json").read_text())
    manifest = json.loads((output / "accepted_group_manifest.json").read_text())
    accepted = {row["group_id"] for row in manifest["accepted_sessions"]}
    return [spec for spec in plan["sessions"] if spec["group_id"] not in accepted]


def next_attempt_index(sessions_root: Path, session_id: str) -> int:
    """Smallest unused attempt number, not a count of existing directories.

    Counting existing directories collides whenever an earlier attempt was
    quarantined (moved out of raw_sessions/): e.g. attempt_1 quarantined,
    attempt_2 accepted -> only 1 directory remains, so a naive count+1
    recomputes "2" again and the scenario script refuses to overwrite the
    accepted capture that's already there.
    """
    existing_numbers = []
    prefix = f"{session_id}_attempt_"
    for path in sessions_root.glob(f"{prefix}*"):
        suffix = path.name[len(prefix):]
        if suffix.isdigit():
            existing_numbers.append(int(suffix))
    return max(existing_numbers, default=0) + 1


def topup(workspace: Path, output: Path, attempts: int) -> dict[str, object]:
    manifest = json.loads((output / "accepted_group_manifest.json").read_text())
    accepted: list[dict[str, object]] = list(manifest["accepted_sessions"])
    quarantined: list[dict[str, object]] = json.loads(
        (output / "quarantine_manifest.json").read_text()
    )
    attempts_log: list[dict[str, object]] = json.loads(
        (output / "attempt_manifest.json").read_text()
    )
    sessions_root = output / "raw_sessions"
    quarantine_root = output / "quarantine"

    pending = missing_groups(output)
    if not pending:
        print("nothing missing; manifests unchanged", flush=True)
        return manifest

    print(f"TOPPING UP {len(pending)} group(s): "
          f"{', '.join(str(spec['group_id']) for spec in pending)}", flush=True)

    for spec in pending:
        success = False
        first = next_attempt_index(sessions_root, str(spec["session_id"]))
        for offset in range(attempts):
            stale = stale_processes()
            if stale:
                raise RuntimeError(f"stale_process_preflight:{stale}")
            disk_preflight(output, len(pending))
            attempt = first + offset
            root = sessions_root / f"{spec['session_id']}_attempt_{attempt}"
            bbox = spec["initial_bbox_normalized"]
            command = [
                str(workspace / "core_range_collect_targeted_pilot_scenario.sh"),
                str(spec["roi_policy"]), str(spec["session_id"]),
                str(spec["start_range_m"]), str(spec["end_range_m"]),
                str(spec["lateral_offset_m"]), str(spec["target_z_m"]),
                str(spec["target_yaw_start_deg"]), str(spec["target_yaw_end_deg"]),
                str(spec["movement_duration_s"]), str(spec["hold_duration_s"]),
                str(spec["pose_update_rate_hz"]), str(spec["gimbal_pitch_deg"]),
                *(str(value) for value in bbox), str(root), "sim_time_plugin",
            ]
            print(f"COLLECTING {spec['session_id']} attempt {attempt} "
                  f"({offset + 1}/{attempts})", flush=True)
            record = {
                "session_id": spec["session_id"],
                "attempt": attempt,
                "topup": True,
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
                row["attempt"] = attempt
                accepted.append(row)
                record["result"] = "ACCEPTED"
                print(f"ACCEPTED {spec['session_id']} frames={row['frame_count']} "
                      f"camera_fps={row['camera_source_median_fps']} "
                      f"rtf={row['gazebo_rtf_median']}", flush=True)
                success = True
            except Exception as error:
                destination = quarantine(root, quarantine_root, str(error))
                quarantined.append({
                    "session_id": spec["session_id"], "attempt": attempt,
                    "reason": str(error),
                    "path": str(destination.relative_to(workspace)),
                })
                record["result"] = "QUARANTINED"
                record["reason"] = str(error)
                print(f"QUARANTINED {spec['session_id']}: {error}", flush=True)
            record["ended_utc"] = datetime.now(timezone.utc).isoformat()
            attempts_log.append(record)
            if success:
                break
            time.sleep(RETRY_COOLDOWN_S)
        if not success:
            print(f"FAILED {spec['session_id']}", flush=True)

    counts = {
        kind: sum(row["scenario_type"] == kind for row in accepted)
        for kind in ("approaching", "receding", "stop_and_hold")
    }
    complete = len(accepted) == 14 and counts == {
        "approaching": 6, "receding": 6, "stop_and_hold": 2
    }
    merged = {
        "gate_id": GATE_ID, "dataset_role": DATASET_ROLE,
        "collection_plan_sha256": sha256_file(output / "collection_plan_pilot.json"),
        "accepted_sessions": accepted, "quarantined_sessions": quarantined,
        "accepted_group_count": len(accepted),
        "accepted_frame_count": sum(int(row["frame_count"]) for row in accepted),
        "accepted_scenario_counts": counts, "collection_complete": complete,
        "topup_applied_utc": datetime.now(timezone.utc).isoformat(),
        "completed_utc": manifest.get("completed_utc"),
    }

    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    for name in ("accepted_group_manifest", "quarantine_manifest", "attempt_manifest"):
        source = output / f"{name}.json"
        if source.exists():
            shutil.copy2(source, output / f"{name}.pre_topup_{stamp}.json")

    write_json(output / "accepted_group_manifest.json", merged)
    write_json(output / "quarantine_manifest.json", quarantined)
    write_json(output / "attempt_manifest.json", attempts_log)
    return merged


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--workspace", type=Path, default=Path.cwd())
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--attempts", type=int, default=3)
    arguments = parser.parse_args()
    manifest = topup(
        arguments.workspace.resolve(), arguments.output.resolve(), arguments.attempts
    )
    print(json.dumps({
        "accepted_group_count": manifest["accepted_group_count"],
        "accepted_scenario_counts": manifest["accepted_scenario_counts"],
        "collection_complete": manifest["collection_complete"],
    }, indent=2), flush=True)
    return 0 if manifest["collection_complete"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
