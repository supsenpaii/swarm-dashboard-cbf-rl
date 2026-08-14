#!/usr/bin/env python3
"""Reduce the interleaved on-demand/eager long-soak validation."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

from core_range_subscription_ablation_analyze import summarize


CONFIGS = ("S6_on_demand", "S5")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", type=Path, required=True)
    args = parser.parse_args()

    rows = []
    for config_id in CONFIGS:
        for repeat in range(1, 4):
            run_dir = args.root / "runs" / f"{config_id}_run{repeat}"
            if not run_dir.is_dir():
                rows.append({"run_id": run_dir.name, "config_id": config_id,
                             "repeat": repeat, "complete": False,
                             "rtf_gate_pass": False, "exit_code": "missing"})
                continue
            row, _trace, _callbacks = summarize(run_dir)
            rows.append(row)

    fields = sorted({key for row in rows for key in row})
    with (args.root / "validation_runs.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)

    selected = {name: [row for row in rows if row["config_id"] == name] for name in CONFIGS}
    lazy_passes = sum(row.get("rtf_gate_pass") is True for row in selected["S6_on_demand"])
    eager_failures = sum(
        row.get("complete") is True and row.get("rtf_gate_pass") is not True
        for row in selected["S5"]
    )
    complete = all(row.get("complete") is True for row in rows)
    decision = {
        "schema": "camera_on_demand_fix_validation/v1",
        "matrix_complete": complete,
        "on_demand_pass_count": lazy_passes,
        "eager_fail_count": eager_failures,
        "fix_runtime_gate_pass": complete and lazy_passes == 3,
        "camera_eager_cause_confirmed": complete and lazy_passes == 3 and eager_failures == 3,
        "conclusion": (
            "ON_DEMAND_FIX_VALIDATED_CAUSE_CONFIRMED"
            if complete and lazy_passes == 3 and eager_failures == 3
            else "ON_DEMAND_FIX_VALIDATED_CAUSE_INCONCLUSIVE"
            if complete and lazy_passes == 3
            else "ON_DEMAND_FIX_FAILED"
            if complete
            else "VALIDATION_INCOMPLETE"
        ),
        "gate": {"median_rtf_min": 0.8, "rtf_low_max": 0.3,
                 "maximum_consecutive_low_samples": 2, "required_repeats": 3},
    }
    (args.root / "validation_decision.json").write_text(
        json.dumps(decision, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    print(json.dumps(decision, indent=2, sort_keys=True))
    return 0 if decision["fix_runtime_gate_pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
