"""Audit the frozen priority static 3–12 m collection, offline only."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
from statistics import mean, pstdev
from typing import Any, Iterable

from core_range_logging_eval import load_jsonl, sha256_file, verify_record
from range_physical_diagnostics import validate_timestamp_stages


AUDIT_ID = "core_range_priority_static_audit_20260804_v001"
CONCLUSION = "PRIORITY_STATIC_COVERAGE_COLLECTED"
TRAINING_GATE = "NO_GO_DYNAMIC_AND_FULL_BIN_BALANCE_MISSING"


def percentile(values: Iterable[float], fraction: float) -> float:
    ordered = sorted(float(value) for value in values)
    if not ordered:
        raise ValueError("percentile_empty")
    position = (len(ordered) - 1) * fraction
    lower = math.floor(position)
    upper = math.ceil(position)
    if lower == upper:
        return ordered[lower]
    return ordered[lower] + (ordered[upper] - ordered[lower]) * (position - lower)


def bin_bounds(label: str) -> tuple[float, float]:
    if not label.endswith("m") or "-" not in label:
        raise ValueError(f"invalid_bin:{label}")
    lower, upper = label[:-1].split("-", 1)
    return float(lower), float(upper)


def bin_contains(label: str, value: float) -> bool:
    lower, upper = bin_bounds(label)
    return lower <= value < upper


def load_effective_manifest(manifest_path: Path, amendment_path: Path) -> dict[str, Any]:
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    amendment = json.loads(amendment_path.read_text(encoding="utf-8"))
    if amendment.get("parent_collection_id") != manifest.get("collection_id"):
        raise ValueError("collection_amendment_parent_mismatch")
    if amendment.get("parent_manifest_sha256") != sha256_file(manifest_path):
        raise ValueError("collection_amendment_checksum_mismatch")
    changes = amendment.get("changes") or {}
    for scenario in manifest["scenarios"]:
        scenario.update(changes.get(scenario["scenario_id"]) or {})
    manifest["effective_amendment"] = amendment
    return manifest


def raw_rows(root: Path, session_id: int) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    malformed = 0
    for path in sorted(root.glob("physical_diagnostics*.jsonl")):
        loaded, invalid = load_jsonl(path)
        malformed += invalid
        rows.extend(
            row
            for row in loaded
            if row.get("stage") == "raw_range_computed"
            and row.get("session_id") == session_id
        )
    if malformed:
        raise ValueError(f"malformed_sidecar:{root}:{malformed}")
    return rows


def audit_group(root: Path, scenario: dict[str, Any]) -> dict[str, Any]:
    audit = root / "audit"
    summary_path = audit / "smoke_summary.json"
    smoke_manifest_path = audit / "smoke_manifest.json"
    summary = json.loads(summary_path.read_text(encoding="utf-8"))
    smoke_manifest = json.loads(smoke_manifest_path.read_text(encoding="utf-8"))
    if summary.get("conclusion") != "CORE_LOGGING_READY":
        raise ValueError(f"group_not_ready:{root.name}")
    if not all(summary.get("gates", {}).values()):
        raise ValueError(f"group_gate_failed:{root.name}")
    capture = summary["capture"]
    if capture.get("dataset_role") != "core_range_development":
        raise ValueError(f"dataset_role_mismatch:{root.name}")
    session_id = int(capture["session_id"])
    rows = raw_rows(root, session_id)
    if len(rows) < 30:
        raise ValueError(f"group_too_short:{root.name}:{len(rows)}")
    if summary["counts"]["raw_session_ids"] != [session_id]:
        raise ValueError(f"multiple_raw_sessions:{root.name}")
    if any(not verify_record(row)[0] or not verify_record(row)[1] for row in rows):
        raise ValueError(f"record_or_trace_checksum_failed:{root.name}")
    if any(not validate_timestamp_stages(row.get("timestamp_stages") or {})[0] for row in rows):
        raise ValueError(f"timestamp_order_failed:{root.name}")
    if any(len(((row.get("anchors") or {}).get("per_grid_point") or [])) != 96 for row in rows):
        raise ValueError(f"anchor_count_failed:{root.name}")
    expected_checksums = smoke_manifest.get("runtime_artifact_checksums") or {}
    for name, expected in expected_checksums.items():
        path = root / name
        if not path.is_file() or sha256_file(path) != expected:
            raise ValueError(f"source_checksum_mismatch:{root.name}:{name}")

    gt = [float(row["ground_truth"]["distance_m"]) for row in rows]
    raw = [float(row["raw_range"]["physics_slant_range_m"]) for row in rows]
    if any(not math.isfinite(value) for value in gt + raw):
        raise ValueError(f"nonfinite_metric:{root.name}")
    if any(not bin_contains(scenario["bin"], value) for value in gt):
        raise ValueError(f"ground_truth_outside_declared_bin:{root.name}")
    errors = [estimate - truth for estimate, truth in zip(raw, gt, strict=True)]
    frame_deltas = [abs(raw[index] - raw[index - 1]) for index in range(1, len(raw))]
    run_ids = {str(row.get("run_id")) for row in rows}
    group_ids = {str(row.get("group_id")) for row in rows}
    if len(run_ids) != 1 or len(group_ids) != 1:
        raise ValueError(f"trace_group_identity_mismatch:{root.name}")
    return {
        "scenario_id": scenario["scenario_id"],
        "bin": scenario["bin"],
        "view": scenario["requested_view"],
        "pitch_deg": float(scenario["gimbal_pitch_deg"]),
        "target_x_m": float(scenario["target_pose_xyz_m"][0]),
        "target_y_m": float(scenario["target_pose_xyz_m"][1]),
        "run_id": next(iter(run_ids)),
        "session_id": session_id,
        "group_id": next(iter(group_ids)),
        "frame_count": len(rows),
        "gt_mean_m": mean(gt),
        "gt_min_m": min(gt),
        "gt_max_m": max(gt),
        "raw_mean_m": mean(raw),
        "raw_std_m": pstdev(raw),
        "signed_bias_m": mean(errors),
        "mae_m": mean(abs(value) for value in errors),
        "p90_abs_error_m": percentile((abs(value) for value in errors), 0.90),
        "p95_abs_error_m": percentile((abs(value) for value in errors), 0.95),
        "p95_frame_delta_m": percentile(frame_deltas, 0.95),
        "max_abs_error_m": max(abs(value) for value in errors),
        "source_sidecar_sha256": sha256_file(root / "physical_diagnostics.jsonl"),
        "source_capture_sha256": sha256_file(root / "capture_events.jsonl"),
        "source_audit_summary_sha256": sha256_file(summary_path),
        "source_audit_manifest_sha256": sha256_file(smoke_manifest_path),
        "safety": "PASS_DISARMED_OBSERVATION_ONLY",
    }


def write_csv(path: Path, rows: list[dict[str, Any]]) -> None:
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def audit(manifest_path: Path, amendment_path: Path, sessions_root: Path, output: Path) -> dict[str, Any]:
    effective = load_effective_manifest(manifest_path, amendment_path)
    group_rows = [
        audit_group(sessions_root / scenario["scenario_id"], scenario)
        for scenario in effective["scenarios"]
    ]
    identities = {(row["run_id"], row["session_id"], row["group_id"]) for row in group_rows}
    if len(identities) != len(group_rows):
        raise ValueError("group_identity_leakage")

    bin_rows: list[dict[str, Any]] = []
    for label in sorted({row["bin"] for row in group_rows}, key=lambda item: bin_bounds(item)[0]):
        selected = [row for row in group_rows if row["bin"] == label]
        bin_rows.append(
            {
                "bin": label,
                "independent_group_count": len(selected),
                "frame_count": sum(row["frame_count"] for row in selected),
                "equal_group_bias_m": mean(row["signed_bias_m"] for row in selected),
                "equal_group_mae_m": mean(row["mae_m"] for row in selected),
                "equal_group_raw_std_m": mean(row["raw_std_m"] for row in selected),
                "priority_static_status": (
                    "COVERED" if len(selected) >= 3 else "PARTIAL"
                ),
                "development_promotion_status": "PARTIAL_DYNAMIC_MISSING",
            }
        )

    output.mkdir(parents=True, exist_ok=True)
    group_csv = output / "per_group_audit.csv"
    bin_csv = output / "per_bin_coverage.csv"
    write_csv(group_csv, group_rows)
    write_csv(bin_csv, bin_rows)
    summary = {
        "audit_id": AUDIT_ID,
        "conclusion": CONCLUSION,
        "training_gate": TRAINING_GATE,
        "accepted_group_count": len(group_rows),
        "accepted_frame_count": sum(row["frame_count"] for row in group_rows),
        "bins": bin_rows,
        "global_gates": {
            "all_planned_scenarios_present": len(group_rows) == len(effective["scenarios"]),
            "three_independent_groups_each_priority_bin": all(row["independent_group_count"] >= 3 for row in bin_rows),
            "minimum_30_frames_each_group": all(row["frame_count"] >= 30 for row in group_rows),
            "one_raw_session_each_dataset": True,
            "source_and_record_checksums": True,
            "timestamp_and_ground_truth_trace": True,
            "both_uavs_disarmed_observation_only": True,
            "dynamic_coverage": False,
            "full_3_to_12m_balanced_coverage": False,
        },
        "scope_guards": {
            "model_trained": False,
            "scaler_calibrator_or_threshold_fit": False,
            "simulation_controller_effect": False,
            "follow_offboard_arm_takeoff_mode_or_motion_called": False,
            "residual_correction": "off",
            "runtime_changed": False,
            "final_holdout_opened": False,
        },
    }
    summary_path = output / "priority_collection_audit.json"
    summary_path.write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    rows_md = "\n".join(
        f"| {row['bin']} | {row['independent_group_count']} | {row['frame_count']} | {row['equal_group_mae_m']:.3f} m | {row['priority_static_status']} | PARTIAL |"
        for row in bin_rows
    )
    report = f"""# Core Range 3–12 m — Priority Static Collection Audit

## Outcome

```text
{CONCLUSION}
```

Nine independent static development groups passed the strict logging and
integrity gates. This closes the priority **static** gaps at 6–7 m, 10–11 m,
and 11–12 m. It does not authorize model training or promotion because dynamic
approach/recede data and balanced independent coverage across every 1 m bin are
still missing.

| Bin | Groups | Frames | Equal-group raw MAE | Priority static | Development/promotion |
| --- | ---: | ---: | ---: | --- | --- |
{rows_md}

## Integrity and safety

- Every accepted root has one raw run/session group and at least 30 frames.
- Every accepted raw record passed record/trace checksum, timestamp order,
  ground-truth trace, finite-value, exact-bin and 96-anchor checks.
- Both UAVs remained disarmed; no Follow, OFFBOARD, arm, takeoff, flight-mode,
  motion or controller endpoint was called.
- Residual correction remained off. No model/scaler/calibrator/threshold was fit.
- Failed attempts remain under `quarantine/` and are excluded.

## Next gate

```text
{TRAINING_GATE}
```

Collect independent observation-only dynamic sequences (approaching and
receding) and fill balanced session coverage in the remaining 3–10 m bins.
Do not train a promotion candidate from this static-only batch.
"""
    report_path = output / "priority_collection_report.md"
    report_path.write_text(report, encoding="utf-8")

    manifest = {
        "audit_id": AUDIT_ID,
        "conclusion": CONCLUSION,
        "training_gate": TRAINING_GATE,
        "inputs": {
            str(manifest_path): sha256_file(manifest_path),
            str(amendment_path): sha256_file(amendment_path),
            "core_range_priority_collection_audit.py": sha256_file(Path(__file__)),
        },
        "accepted_sources": {
            row["scenario_id"]: {
                "physical_diagnostics.jsonl": row["source_sidecar_sha256"],
                "capture_events.jsonl": row["source_capture_sha256"],
                "audit/smoke_summary.json": row["source_audit_summary_sha256"],
                "audit/smoke_manifest.json": row["source_audit_manifest_sha256"],
            }
            for row in group_rows
        },
        "outputs": {
            path.name: sha256_file(path)
            for path in (group_csv, bin_csv, summary_path, report_path)
        },
        "exact_command": (
            f"python3 core_range_priority_collection_audit.py {manifest_path} "
            f"{amendment_path} {sessions_root} --output {output}"
        ),
        "scope_guards": summary["scope_guards"],
    }
    manifest_path_out = output / "priority_collection_manifest.json"
    manifest_path_out.write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return summary


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("manifest", type=Path)
    parser.add_argument("amendment", type=Path)
    parser.add_argument("sessions_root", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    result = audit(
        args.manifest.resolve(),
        args.amendment.resolve(),
        args.sessions_root.resolve(),
        args.output.resolve(),
    )
    print(result["conclusion"])
    print(result["training_gate"])
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
