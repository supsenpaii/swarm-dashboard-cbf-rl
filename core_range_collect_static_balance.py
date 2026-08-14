"""Run the frozen 18-session static balance plan with fail-closed audits."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess

from core_range_logging_eval import sha256_file
from core_range_static_balance_audit import DATASET_ROLE, load_effective_plan


MINIMUM_FREE_BYTES = 5 * 1024**3
PROJECTED_SESSION_BYTES = 512 * 1024**2


def stale_processes() -> list[str]:
    result = subprocess.run(["ps", "-eo", "pid=,args="], check=True, text=True, capture_output=True)
    needles = ("/px4 ", "gz sim", "gzserver", "mavlink_manual_bridge.py", "MicroXRCEAgent", "python3 main.py")
    return [line.strip() for line in result.stdout.splitlines() if any(needle in line for needle in needles)]


def disk_preflight(path: Path, remaining_sessions: int) -> dict[str, int]:
    usage = shutil.disk_usage(path)
    projected = max(1, remaining_sessions) * PROJECTED_SESSION_BYTES
    if usage.free < MINIMUM_FREE_BYTES or usage.free < projected:
        raise RuntimeError(f"disk_preflight_failed:free={usage.free}:projected={projected}")
    return {"free_bytes": usage.free, "minimum_free_bytes": MINIMUM_FREE_BYTES, "projected_remaining_bytes": projected}


def archive_runtime_logs(root: Path) -> dict[str, object]:
    logs = (root / "runtime_logs").resolve()
    if logs.parent != root.resolve() or not logs.is_dir():
        raise RuntimeError(f"runtime_logs_missing_or_unsafe:{root}")
    archive = root / "runtime_logs.tar.zst"
    if archive.exists():
        raise RuntimeError(f"archive_already_exists:{archive}")
    subprocess.run(["tar", "--zstd", "-cf", str(archive), "-C", str(root), "runtime_logs"], check=True)
    listing = subprocess.run(["tar", "--zstd", "-tf", str(archive)], check=True, text=True, capture_output=True)
    members = [line for line in listing.stdout.splitlines() if line]
    if not members or not any(line.startswith("runtime_logs/") for line in members):
        raise RuntimeError(f"archive_read_test_empty:{archive}")
    checksum = sha256_file(archive)
    shutil.rmtree(logs)
    if logs.exists():
        raise RuntimeError(f"raw_runtime_log_removal_failed:{logs}")
    payload = {
        "archive_name": archive.name,
        "archive_sha256": checksum,
        "archive_size_bytes": archive.stat().st_size,
        "member_count": len(members),
        "read_test": "PASS",
        "raw_runtime_logs_removed_after_verification": True,
        "created_utc": datetime.now(timezone.utc).isoformat(),
    }
    (root / "runtime_log_archive.json").write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return payload


def quarantine(root: Path, quarantine_root: Path, reason: str) -> Path | None:
    if not root.exists():
        return None
    quarantine_root.mkdir(parents=True, exist_ok=True)
    suffix = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    destination = quarantine_root / f"{root.name}_attempt_{suffix}"
    shutil.move(str(root), str(destination))
    note = destination / "QUARANTINE.md"
    note.write_text(f"# Quarantined static balance attempt\n\n- Reason: `{reason}`\n- Preserved unchanged from capture root; excluded from audit/training.\n", encoding="utf-8")
    return destination


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("plan", type=Path)
    parser.add_argument("--workspace", type=Path, default=Path(__file__).parent)
    parser.add_argument("--continue-after-failure", action="store_true")
    parser.add_argument("--amendment", type=Path)
    args = parser.parse_args()
    workspace = args.workspace.resolve()
    plan_path = args.plan.resolve()
    amendment_path = None if args.amendment is None else args.amendment.resolve()
    plan, amendment = load_effective_plan(plan_path, amendment_path)
    expected_plan_sha = plan.get("precommit_sha256_external")
    if expected_plan_sha and expected_plan_sha != sha256_file(plan_path):
        raise RuntimeError("precommit_checksum_mismatch")
    batch_root = plan_path.parent
    sessions_root = batch_root / "runtime_sessions"
    quarantine_root = batch_root / "quarantine"
    sessions_root.mkdir(parents=True, exist_ok=True)
    failures: list[dict[str, str]] = []
    for index, session in enumerate(plan["sessions"]):
        session_name = session["session_id"]
        root = sessions_root / session_name
        if root.exists():
            print(f"SKIP_EXISTING {session_name}", flush=True)
            continue
        stale = stale_processes()
        if stale:
            raise RuntimeError(f"stale_process_preflight:{stale}")
        disk = disk_preflight(batch_root, len(plan["sessions"]) - index)
        target = session["target_pose"]
        bbox = session["bbox_normalized"]
        command = [
            str(workspace / "core_range_collect_scenario.sh"), session_name,
            str(target["x_m"]), str(target["y_m"]), str(session["nominal_distance_m"]),
            str(session["camera_gimbal_pitch_deg"]), str(bbox[0]), str(bbox[1]),
            str(bbox[2]), str(bbox[3]), str(root),
            str(session.get("prewarm_timeout_s", 90)),
            str(session.get("capture_timeout_s", 90)),
        ]
        env = os.environ.copy()
        env["CORE_RANGE_DATASET_ROLE"] = DATASET_ROLE
        env["CORE_RANGE_RUN_ID"] = f"core_static_balance_{session_name}_20260804"
        print(f"COLLECTING {index + 1}/18 {session_name} free={disk['free_bytes']}", flush=True)
        try:
            subprocess.run(command, cwd=workspace, env=env, check=True)
            subprocess.run([
                "python3", str(workspace / "core_range_logging_eval.py"), str(root),
                "--output", str(root / "audit"), "--minimum-frames", "30",
                "--require-single-raw-session",
            ], cwd=workspace, env={**env, "PYTHONPATH": str(workspace / ".venv/lib/python3.12/site-packages")}, check=True)
            summary = json.loads((root / "audit" / "smoke_summary.json").read_text(encoding="utf-8"))
            if summary.get("capture", {}).get("dataset_role") != DATASET_ROLE:
                raise RuntimeError("dataset_role_post_capture_mismatch")
            archive_runtime_logs(root)
            print(f"ACCEPTED {session_name}", flush=True)
        except Exception as error:
            destination = quarantine(root, quarantine_root, str(error))
            failures.append({"session_id": session_name, "reason": str(error), "quarantine": str(destination) if destination else "none"})
            print(f"QUARANTINED {session_name}: {error}", flush=True)
            if not args.continue_after_failure:
                break
    result = {
        "plan_sha256": sha256_file(plan_path),
        "amendment_id": None if amendment is None else amendment.get("amendment_id"),
        "amendment_sha256": None if amendment_path is None else sha256_file(amendment_path),
        "failures": failures,
        "completed_utc": datetime.now(timezone.utc).isoformat(),
    }
    execution_name = (
        "collection_execution.json"
        if amendment is None
        else f"collection_execution_{amendment['amendment_id']}.json"
    )
    (batch_root / execution_name).write_text(json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return 0 if not failures else 1


if __name__ == "__main__":
    raise SystemExit(main())
