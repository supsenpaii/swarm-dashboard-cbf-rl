"""Run the frozen priority static collection manifest sequentially."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import subprocess


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("manifest", type=Path)
    parser.add_argument("--workspace", type=Path, default=Path(__file__).parent)
    parser.add_argument("--skip-verified-existing", action="store_true")
    parser.add_argument("--amendment", type=Path)
    args = parser.parse_args()
    workspace = args.workspace.resolve()
    manifest = json.loads(args.manifest.resolve().read_text(encoding="utf-8"))
    if args.amendment is not None:
        amendment = json.loads(
            args.amendment.resolve().read_text(encoding="utf-8")
        )
        if amendment.get("parent_collection_id") != manifest.get("collection_id"):
            raise ValueError("collection_amendment_parent_mismatch")
        changes = amendment.get("changes") or {}
        for scenario in manifest["scenarios"]:
            override = changes.get(scenario["scenario_id"])
            if override:
                scenario.update(override)
    parent = args.manifest.resolve().parent / "runtime_sessions"
    for scenario in manifest["scenarios"]:
        scenario_id = scenario["scenario_id"]
        x, y, _ = scenario["target_pose_xyz_m"]
        distance = (x * x + y * y) ** 0.5
        bbox_x, bbox_y, bbox_w, bbox_h = scenario["bbox_normalized"]
        root = parent / scenario_id
        if root.exists() and args.skip_verified_existing:
            summary_path = root / "audit" / "smoke_summary.json"
            if not summary_path.is_file():
                raise RuntimeError(f"existing_scenario_not_verified:{scenario_id}")
            summary = json.loads(summary_path.read_text(encoding="utf-8"))
            if summary.get("conclusion") != "CORE_LOGGING_READY":
                raise RuntimeError(f"existing_scenario_not_ready:{scenario_id}")
            print(f"SKIPPED VERIFIED {scenario_id}", flush=True)
            continue
        command = [
            str(workspace / "core_range_collect_scenario.sh"),
            scenario_id,
            str(x),
            str(y),
            str(distance),
            str(scenario["gimbal_pitch_deg"]),
            str(bbox_x),
            str(bbox_y),
            str(bbox_w),
            str(bbox_h),
            str(root),
            str(scenario.get("prewarm_timeout_s", 90)),
        ]
        print(f"COLLECTING {scenario_id}", flush=True)
        subprocess.run(command, cwd=workspace, check=True)
        subprocess.run(
            [
                "python3",
                str(workspace / "core_range_logging_eval.py"),
                str(root),
                "--output",
                str(root / "audit"),
                "--minimum-frames",
                "30",
                "--require-single-raw-session",
            ],
            cwd=workspace,
            check=True,
        )
        print(f"COLLECTED {scenario_id}", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
