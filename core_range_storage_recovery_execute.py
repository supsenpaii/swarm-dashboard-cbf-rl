"""CORE_RANGE_SAFE_STORAGE_RECOVERY -- execute approved actions B1-B4.

Sequentially, one directory at a time (no parallelism):
  1. snapshot per-file SHA-256 of every file in the source directory
  2. stream-compress (tar | gzip) the whole directory straight to
     /mnt/px4ssd/swarm_dashboard_archive/<relative path>.tar.gz (never
     materializing two large copies at once)
  3. SHA-256 the archive
  4. list-test (tar -tzf) and full read-test (stream-decompress each member
     through sha256sum, compare against step 1 -- no extracted duplicate
     ever touches disk)
  5. fsync the archive file and its parent directory
  6. only if every check passed: delete the source directory
  7. re-verify the archive is still readable after deletion
  8. append one row/entry to executed_actions.csv / move_manifest.json

Fail-closed: any failure in steps 2-5 stops that action, leaves the source
untouched, and reports the exact reason. Checks /mnt/px4ssd free space
(>=3GB) before every action; stops before starting the next one if not.
"""

from __future__ import annotations

import csv
import hashlib
import json
import os
import shutil
import subprocess
import time
from pathlib import Path

WORKSPACE = Path("/home/sup/swarm_dashboard")
ARTIFACTS = WORKSPACE / "artifacts"
ARCHIVE_ROOT = Path("/mnt/px4ssd/swarm_dashboard_archive")
OUT = WORKSPACE / "artifacts/storage_recovery"
MIN_PX4SSD_FREE_BYTES = 3 * 1024**3

TARGET_DIRS = [
    "run_20260803_094502",
    "run_20260803_110431",
    "run_20260803_104238",
    "run_20260803_104512",
]


def sha256_file(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as f:
        for chunk in iter(lambda: f.read(4 * 1024 * 1024), b""):
            h.update(chunk)
    return h.hexdigest()


def free_bytes(path: str) -> int:
    st = os.statvfs(path)
    return st.f_bavail * st.f_frsize


def dir_size_bytes(path: Path) -> int:
    return sum(f.stat().st_size for f in path.rglob("*") if f.is_file())


def snapshot_checksums(src_dir: Path) -> dict[str, str]:
    checksums = {}
    for f in sorted(src_dir.rglob("*")):
        if f.is_file():
            rel = str(f.relative_to(src_dir))
            checksums[rel] = sha256_file(f)
    return checksums


def run_action(dirname: str) -> dict:
    src = ARTIFACTS / dirname
    result = {"action_id": None, "path": f"artifacts/{dirname}", "started_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())}

    if not src.is_dir():
        result.update(status="SKIPPED", reason="source_directory_missing")
        return result

    pre_free = free_bytes("/mnt/px4ssd")
    if pre_free < MIN_PX4SSD_FREE_BYTES:
        result.update(status="STOPPED_BEFORE_START", reason=f"px4ssd_free_below_3GB:{pre_free}")
        return result

    source_size = dir_size_bytes(src)
    result["source_size_bytes"] = source_size

    # step 1: per-file checksum snapshot BEFORE any compression
    try:
        pre_checksums = snapshot_checksums(src)
    except Exception as exc:
        result.update(status="FAILED", reason=f"pre_checksum_failed:{exc}")
        return result
    result["source_file_count"] = len(pre_checksums)

    dest_dir = ARCHIVE_ROOT / "artifacts"
    dest_dir.mkdir(parents=True, exist_ok=True)
    archive_path = dest_dir / f"{dirname}.tar.gz"

    if archive_path.exists():
        result.update(status="FAILED", reason=f"archive_already_exists_not_overwriting:{archive_path}")
        return result

    # step 2: stream compress directly to destination (single pass, tar|gzip)
    tmp_archive = archive_path.with_suffix(".tar.gz.partial")
    try:
        with tmp_archive.open("wb") as out_f:
            tar = subprocess.Popen(["tar", "-cf", "-", "-C", str(ARTIFACTS), dirname], stdout=subprocess.PIPE)
            gz = subprocess.Popen(["gzip", "-1"], stdin=tar.stdout, stdout=out_f)
            tar.stdout.close()
            gz.wait()
            tar.wait()
        if tar.returncode != 0 or gz.returncode != 0:
            raise RuntimeError(f"tar_rc={tar.returncode}_gzip_rc={gz.returncode}")
        out_fd = os.open(str(tmp_archive), os.O_RDONLY)
        os.fsync(out_fd)
        os.close(out_fd)
        tmp_archive.rename(archive_path)
        dir_fd = os.open(str(dest_dir), os.O_RDONLY)
        os.fsync(dir_fd)
        os.close(dir_fd)
    except Exception as exc:
        if tmp_archive.exists():
            tmp_archive.unlink()
        result.update(status="FAILED", reason=f"compression_failed:{exc}")
        return result

    archive_size = archive_path.stat().st_size
    result["archive_size_bytes"] = archive_size
    result["compression_ratio"] = round(source_size / archive_size, 2) if archive_size else None

    # step 3: archive checksum
    archive_sha256 = sha256_file(archive_path)
    result["archive_sha256"] = archive_sha256

    # step 4a: list-test
    try:
        listing = subprocess.run(["tar", "-tzf", str(archive_path)], capture_output=True, text=True, timeout=300)
        if listing.returncode != 0:
            raise RuntimeError(f"tar_list_rc={listing.returncode}:{listing.stderr[:500]}")
        listed_members = {line[len(dirname) + 1:] for line in listing.stdout.splitlines() if line.strip() and line != f"{dirname}/" and line.rstrip("/") != dirname}
    except Exception as exc:
        result.update(status="FAILED", reason=f"list_test_failed:{exc}")
        return result

    expected_members = set(pre_checksums.keys())
    if listed_members != expected_members:
        missing = expected_members - listed_members
        extra = listed_members - expected_members
        result.update(status="FAILED", reason=f"member_list_mismatch:missing={list(missing)[:5]}:extra={list(extra)[:5]}")
        return result

    # step 4b: full read-test -- stream-decompress each member, hash, compare
    mismatches = []
    try:
        for rel, expected_hash in pre_checksums.items():
            proc = subprocess.run(
                ["tar", "-xOzf", str(archive_path), f"{dirname}/{rel}"],
                capture_output=True, timeout=600,
            )
            if proc.returncode != 0:
                mismatches.append(f"{rel}:extract_rc={proc.returncode}")
                continue
            actual_hash = hashlib.sha256(proc.stdout).hexdigest()
            if actual_hash != expected_hash:
                mismatches.append(f"{rel}:hash_mismatch")
    except Exception as exc:
        result.update(status="FAILED", reason=f"read_test_failed:{exc}")
        return result

    if mismatches:
        result.update(status="FAILED", reason=f"read_test_mismatches:{mismatches[:10]}")
        return result

    result["list_test"] = "PASS"
    result["read_test"] = "PASS"
    result["member_count_verified"] = len(pre_checksums)

    # step 6: delete source only now that everything passed
    try:
        shutil.rmtree(src)
    except Exception as exc:
        result.update(status="FAILED_AFTER_VERIFICATION_SOURCE_NOT_DELETED", reason=f"delete_failed:{exc}")
        return result

    # step 7: re-verify archive still readable after source deletion
    try:
        relist = subprocess.run(["tar", "-tzf", str(archive_path)], capture_output=True, text=True, timeout=300)
        if relist.returncode != 0:
            result.update(status="SOURCE_DELETED_BUT_ARCHIVE_UNREADABLE_AFTER", reason=relist.stderr[:500])
            return result
        post_delete_sha256 = sha256_file(archive_path)
        if post_delete_sha256 != archive_sha256:
            result.update(status="SOURCE_DELETED_BUT_ARCHIVE_CHECKSUM_CHANGED_AFTER", reason="checksum_drift")
            return result
    except Exception as exc:
        result.update(status="SOURCE_DELETED_BUT_POST_VERIFY_FAILED", reason=str(exc))
        return result

    result.update(
        status="SUCCESS",
        archive_path=str(archive_path),
        source_deleted=True,
        finished_utc=time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        px4ssd_free_before_bytes=pre_free,
        px4ssd_free_after_bytes=free_bytes("/mnt/px4ssd"),
        home_free_after_bytes=free_bytes("/home/sup"),
    )
    return result


def append_csv(path: Path, row: dict) -> None:
    exists = path.exists() and path.stat().st_size > 0
    keys = ["action_id", "path", "action", "result", "freed_bytes", "timestamp_utc"]
    with path.open("a", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=keys)
        if not exists:
            writer.writeheader()
        writer.writerow(row)


def main() -> None:
    action_ids = ["B1", "B2", "B3", "B4"]
    all_results = []
    for action_id, dirname in zip(action_ids, TARGET_DIRS):
        print(f"=== {action_id}: {dirname} ===", flush=True)
        res = run_action(dirname)
        res["action_id"] = action_id
        all_results.append(res)
        print(json.dumps(res, indent=2, default=str), flush=True)

        freed = res.get("source_size_bytes", 0) if res.get("status") == "SUCCESS" else 0
        append_csv(OUT / "executed_actions.csv", {
            "action_id": action_id, "path": f"artifacts/{dirname}",
            "action": "stream-compress to /mnt/px4ssd/swarm_dashboard_archive + verify + delete source",
            "result": res.get("status"), "freed_bytes": freed,
            "timestamp_utc": res.get("finished_utc", res.get("started_utc")),
        })

        # update move_manifest.json incrementally
        manifest_path = OUT / "move_manifest.json"
        manifest = json.loads(manifest_path.read_text()) if manifest_path.exists() else {"moves_executed": []}
        manifest.setdefault("moves_executed", []).append(res)
        manifest_path.write_text(json.dumps(manifest, indent=2, default=str) + "\n")

        if res.get("status") != "SUCCESS":
            print(f"STOPPING: {action_id} did not succeed ({res.get('status')}: {res.get('reason')})", flush=True)
            break

        if free_bytes("/mnt/px4ssd") < MIN_PX4SSD_FREE_BYTES:
            print("STOPPING: px4ssd free space below 3GB after this action", flush=True)
            break

    print("\n=== FINAL DISK STATE ===")
    print("home free:", free_bytes("/home/sup"))
    print("px4ssd free:", free_bytes("/mnt/px4ssd"))


if __name__ == "__main__":
    main()
