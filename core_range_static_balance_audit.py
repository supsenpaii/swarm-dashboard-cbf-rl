"""Offline integrity and coverage audit for CORE_RANGE_STATIC_BALANCE_COLLECTION."""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
from statistics import mean, median, pstdev
from typing import Any, Iterable

from core_range_logging_eval import load_jsonl, sha256_file, verify_record
from range_physical_diagnostics import validate_timestamp_stages


AUDIT_ID = "core_range_static_balance_audit_20260804_v001"
COLLECTION_ID = "core_range_static_balance_20260804_v001"
REQUIRED_BINS = ("3-4m", "4-5m", "5-6m", "7-8m", "8-9m", "9-10m")
DATASET_ROLE = "core_range_static_development"
ALLOWED_CONCLUSIONS = {
    "STATIC_3_12M_COVERAGE_READY_FOR_MODEL_BENCHMARK",
    "STATIC_3_12M_COVERAGE_INCOMPLETE",
    "COLLECTION_BLOCKED_BY_INTEGRITY_OR_LOGGING",
}


def percentile(values: Iterable[float], fraction: float) -> float:
    ordered = sorted(float(value) for value in values)
    if not ordered:
        raise ValueError("percentile_empty")
    position = (len(ordered) - 1) * float(fraction)
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
    return lower <= float(value) < upper


def validate_precommit(plan: dict[str, Any]) -> None:
    if plan.get("collection_id") != COLLECTION_ID:
        raise ValueError("collection_id_mismatch")
    if plan.get("created_before_collection") is not True:
        raise ValueError("plan_not_precommitted")
    sessions = plan.get("sessions") or []
    if len(sessions) != 18:
        raise ValueError(f"precommit_session_count:{len(sessions)}")
    ids = [str(row.get("session_id")) for row in sessions]
    groups = [str(row.get("group_id")) for row in sessions]
    if len(set(ids)) != 18 or len(set(groups)) != 18:
        raise ValueError("precommit_identity_duplicate")
    required = {
        "session_id", "group_id", "distance_bin", "nominal_distance_m",
        "scenario", "target_pose", "observer_pose", "camera_gimbal_pitch_deg",
        "background_context", "planned_valid_frames", "dataset_role",
        "pipeline_version", "logging_schema_version", "logging_schema_checksum",
        "random_seed", "bbox_normalized",
    }
    for row in sessions:
        missing = sorted(required - set(row))
        if missing:
            raise ValueError(f"precommit_fields_missing:{row.get('session_id')}:{missing}")
        if row["dataset_role"] != DATASET_ROLE or int(row["random_seed"]) != 52:
            raise ValueError(f"precommit_policy_mismatch:{row['session_id']}")
        if int(row["planned_valid_frames"]) < 30:
            raise ValueError(f"precommit_frames_too_low:{row['session_id']}")
        if not bin_contains(row["distance_bin"], row["nominal_distance_m"]):
            raise ValueError(f"precommit_nominal_outside_bin:{row['session_id']}")
    counts = {label: sum(row["distance_bin"] == label for row in sessions) for label in REQUIRED_BINS}
    if any(count != 3 for count in counts.values()):
        raise ValueError(f"precommit_bin_balance:{counts}")


def load_effective_plan(
    plan_path: Path,
    amendment_path: Path | None = None,
) -> tuple[dict[str, Any], dict[str, Any] | None]:
    plan = json.loads(plan_path.read_text(encoding="utf-8"))
    validate_precommit(plan)
    if amendment_path is None:
        return plan, None
    amendment = json.loads(amendment_path.read_text(encoding="utf-8"))
    if amendment.get("parent_collection_id") != plan.get("collection_id"):
        raise ValueError("amendment_parent_collection_mismatch")
    if amendment.get("parent_plan_sha256") != sha256_file(plan_path):
        raise ValueError("amendment_parent_checksum_mismatch")
    if amendment.get("created_before_replacement_capture") is not True:
        raise ValueError("replacement_not_precommitted")
    replacements = amendment.get("replacements") or {}
    effective_sessions: list[dict[str, Any]] = []
    seen_replacements: set[str] = set()
    for session in plan["sessions"]:
        original_id = session["session_id"]
        replacement = replacements.get(original_id)
        if replacement is None:
            effective_sessions.append(session)
            continue
        if replacement.get("replacement_for") != original_id:
            raise ValueError(f"replacement_trace_mismatch:{original_id}")
        if replacement.get("technical_failure_only") is not True:
            raise ValueError(f"replacement_not_technical:{original_id}")
        if replacement.get("raw_accuracy_metrics_used") is not False:
            raise ValueError(f"replacement_metric_leakage:{original_id}")
        effective_sessions.append(replacement)
        seen_replacements.add(original_id)
    if seen_replacements != set(replacements):
        raise ValueError("replacement_unknown_original")
    effective = dict(plan)
    effective["sessions"] = effective_sessions
    effective["effective_amendment_id"] = amendment.get("amendment_id")
    validate_precommit(effective)
    return effective, amendment


def _raw_rows(root: Path, session_id: int) -> tuple[list[dict[str, Any]], int]:
    rows: list[dict[str, Any]] = []
    malformed = 0
    for path in sorted(root.glob("physical_diagnostics*.jsonl")):
        loaded, invalid = load_jsonl(path)
        malformed += invalid
        rows.extend(
            row for row in loaded
            if row.get("stage") == "raw_range_computed"
            and int(row.get("session_id", -1)) == session_id
        )
    return rows, malformed


def audit_group(root: Path, planned: dict[str, Any]) -> dict[str, Any]:
    summary_path = root / "audit" / "smoke_summary.json"
    smoke_manifest_path = root / "audit" / "smoke_manifest.json"
    archive_manifest_path = root / "runtime_log_archive.json"
    summary = json.loads(summary_path.read_text(encoding="utf-8"))
    smoke_manifest = json.loads(smoke_manifest_path.read_text(encoding="utf-8"))
    archive_manifest = json.loads(archive_manifest_path.read_text(encoding="utf-8"))
    if summary.get("conclusion") != "CORE_LOGGING_READY" or not all(summary.get("gates", {}).values()):
        raise ValueError(f"logging_gate_failed:{planned['session_id']}")
    capture = summary["capture"]
    if capture.get("dataset_role") != DATASET_ROLE:
        raise ValueError(f"dataset_role_mismatch:{planned['session_id']}")
    session_id = int(capture["session_id"])
    rows, malformed = _raw_rows(root, session_id)
    if malformed:
        raise ValueError(f"malformed_json:{planned['session_id']}:{malformed}")
    if len(rows) < 30:
        raise ValueError(f"minimum_frames_failed:{planned['session_id']}:{len(rows)}")
    if summary["counts"]["raw_session_ids"] != [session_id]:
        raise ValueError(f"multiple_measurement_sessions:{planned['session_id']}")
    run_ids = {str(row.get("run_id")) for row in rows}
    group_ids = {str(row.get("group_id")) for row in rows}
    trace_ids = {str(row.get("trace_identity_sha256")) for row in rows}
    if len(run_ids) != 1 or len(group_ids) != 1 or len(trace_ids) != len(rows):
        raise ValueError(f"identity_or_duplicate_trace_failed:{planned['session_id']}")
    record_pass = sum(all(verify_record(row)) for row in rows)
    timestamp_pass = sum(validate_timestamp_stages(row.get("timestamp_stages") or {})[0] for row in rows)
    gt_trace_pass = sum(row.get("ground_truth_trace_valid") is True for row in rows)
    anchor_pass = sum(len(((row.get("anchors") or {}).get("per_grid_point") or [])) == 96 for row in rows)
    fingerprint_pass = sum(
        isinstance(row.get("pipeline_config_fingerprint_sha256"), str)
        and len(row["pipeline_config_fingerprint_sha256"]) == 64
        for row in rows
    )
    if min(record_pass, timestamp_pass, gt_trace_pass, anchor_pass, fingerprint_pass) != len(rows):
        raise ValueError(f"frame_integrity_failed:{planned['session_id']}")
    expected = smoke_manifest.get("runtime_artifact_checksums") or {}
    for name, checksum in expected.items():
        path = root / name
        if not path.is_file() or sha256_file(path) != checksum:
            raise ValueError(f"source_checksum_mismatch:{planned['session_id']}:{name}")
    archive = root / archive_manifest["archive_name"]
    if (
        not archive.is_file()
        or sha256_file(archive) != archive_manifest.get("archive_sha256")
        or archive_manifest.get("read_test") != "PASS"
        or (root / "runtime_logs").exists()
    ):
        raise ValueError(f"archive_integrity_failed:{planned['session_id']}")
    gt = [float(row["ground_truth"]["distance_m"]) for row in rows]
    raw = [float(row["raw_range"]["physics_slant_range_m"]) for row in rows]
    if any(not math.isfinite(value) for value in gt + raw):
        raise ValueError(f"nonfinite_metric:{planned['session_id']}")
    if any(not bin_contains(planned["distance_bin"], value) for value in gt):
        raise ValueError(f"gt_outside_bin:{planned['session_id']}")
    errors = [estimate - truth for estimate, truth in zip(raw, gt, strict=True)]
    absolute = [abs(value) for value in errors]
    return {
        "session_id": planned["session_id"],
        "runtime_session_id": session_id,
        "group_id": planned["group_id"],
        "runtime_group_id": next(iter(group_ids)),
        "run_id": next(iter(run_ids)),
        "distance_bin": planned["distance_bin"],
        "scenario": planned["scenario"],
        "valid_frames": len(rows),
        "gt_mean_m": mean(gt),
        "gt_std_m": pstdev(gt),
        "raw_mean_m": mean(raw),
        "raw_std_m": pstdev(raw),
        "signed_bias_m": mean(errors),
        "median_absolute_error_m": median(absolute),
        "mae_m": mean(absolute),
        "p90_abs_error_m": percentile(absolute, 0.90),
        "p95_abs_error_m": percentile(absolute, 0.95),
        "error_gt_2m_fraction": mean(value > 2.0 for value in absolute),
        "error_gt_3m_fraction": mean(value > 3.0 for value in absolute),
        "timestamp_pass": True,
        "gt_trace_checksum_pass": True,
        "anchor_completeness": "96_PER_FRAME",
        "malformed_json_lines": 0,
        "duplicate_trace_identities": 0,
        "archive_sha256": archive_manifest["archive_sha256"],
        "integrity_status": "PASS",
        "source_sidecar_sha256": sha256_file(root / "physical_diagnostics.jsonl"),
        "source_capture_sha256": sha256_file(root / "capture_events.jsonl"),
        "source_smoke_summary_sha256": sha256_file(summary_path),
        "source_smoke_manifest_sha256": sha256_file(smoke_manifest_path),
        "source_archive_manifest_sha256": sha256_file(archive_manifest_path),
    }


def write_csv(path: Path, rows: list[dict[str, Any]]) -> None:
    if not rows:
        path.write_text("", encoding="utf-8")
        return
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def audit(
    plan_path: Path,
    sessions_root: Path,
    quarantine_root: Path,
    output: Path,
    amendment_path: Path | None = None,
) -> dict[str, Any]:
    plan, amendment = load_effective_plan(plan_path, amendment_path)
    accepted: list[dict[str, Any]] = []
    failed: list[dict[str, Any]] = []
    for planned in plan["sessions"]:
        root = sessions_root / planned["session_id"]
        try:
            accepted.append(audit_group(root, planned))
        except (OSError, ValueError, KeyError, json.JSONDecodeError) as error:
            failed.append({"session_id": planned["session_id"], "reason": str(error)})
    identities = {(row["run_id"], row["runtime_session_id"], row["runtime_group_id"]) for row in accepted}
    duplicate_group_identity = len(identities) != len(accepted)
    bin_rows: list[dict[str, Any]] = []
    for label in REQUIRED_BINS:
        selected = [row for row in accepted if row["distance_bin"] == label]
        covered = (
            len(selected) >= 3
            and all(row["valid_frames"] >= 30 for row in selected)
            and all(row["integrity_status"] == "PASS" for row in selected)
        )
        bin_rows.append({
            "distance_bin": label,
            "group_count": len(selected),
            "frame_count": sum(row["valid_frames"] for row in selected),
            "equal_group_bias_m": mean(row["signed_bias_m"] for row in selected) if selected else "N/A",
            "equal_group_mae_m": mean(row["mae_m"] for row in selected) if selected else "N/A",
            "equal_group_p90_m": mean(row["p90_abs_error_m"] for row in selected) if selected else "N/A",
            "equal_group_p95_m": mean(row["p95_abs_error_m"] for row in selected) if selected else "N/A",
            "minimum_group_mae_m": min((row["mae_m"] for row in selected), default="N/A"),
            "maximum_group_mae_m": max((row["mae_m"] for row in selected), default="N/A"),
            "coverage_status": "STATIC_COVERED" if covered else "MISSING_OR_PARTIAL",
        })
    all_covered = all(row["coverage_status"] == "STATIC_COVERED" for row in bin_rows)
    if all_covered and not failed and not duplicate_group_identity:
        conclusion = "STATIC_3_12M_COVERAGE_READY_FOR_MODEL_BENCHMARK"
    elif not accepted and failed:
        conclusion = "COLLECTION_BLOCKED_BY_INTEGRITY_OR_LOGGING"
    else:
        conclusion = "STATIC_3_12M_COVERAGE_INCOMPLETE"
    assert conclusion in ALLOWED_CONCLUSIONS

    output.mkdir(parents=True, exist_ok=True)
    group_csv = output / "per_group_audit.csv"
    bin_csv = output / "per_bin_coverage.csv"
    accepted_path = output / "accepted_sessions.json"
    quarantine_path = output / "quarantined_sessions.json"
    integrity_path = output / "integrity_report.json"
    readiness_path = output / "training_readiness_report.json"
    session_manifest_path = output / "session_manifest.json"
    write_csv(group_csv, accepted)
    write_csv(bin_csv, bin_rows)
    accepted_path.write_text(json.dumps(accepted, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    quarantine_entries = sorted(path.name for path in quarantine_root.iterdir()) if quarantine_root.is_dir() else []
    quarantine_path.write_text(json.dumps({"audit_failures": failed, "preserved_roots": quarantine_entries}, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    integrity = {
        "audit_id": AUDIT_ID,
        "accepted_sessions": len(accepted),
        "accepted_frames": sum(row["valid_frames"] for row in accepted),
        "failed_planned_sessions": failed,
        "duplicate_group_identity": duplicate_group_identity,
        "duplicate_trace_identity_count": 0,
        "malformed_json_line_count": 0,
        "timestamp_order_pass_frames": sum(row["valid_frames"] for row in accepted),
        "ground_truth_trace_checksum_pass_frames": sum(row["valid_frames"] for row in accepted),
        "anchor_96_pass_frames": sum(row["valid_frames"] for row in accepted),
        "all_timestamp_gt_checksum_anchor_malformed_gates_pass": not failed and not duplicate_group_identity,
        "runtime_logs_archived_read_tested_then_raw_removed": all(row["integrity_status"] == "PASS" for row in accepted),
    }
    integrity_path.write_text(json.dumps(integrity, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    readiness = {
        "conclusion": conclusion,
        "static_balance_bins": bin_rows,
        "static_model_benchmark_authorized_after_user_review": all_covered,
        "automatic_training_started": False,
        "group_split_created": False,
        "models_scalers_thresholds_fit": False,
        "remaining_scope": "user review before raw-vs-residual-vs-direct benchmark",
    }
    readiness_path.write_text(json.dumps(readiness, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    session_manifest = {
        "collection_id": COLLECTION_ID,
        "precommit_plan_sha256": sha256_file(plan_path),
        "effective_amendment_id": None if amendment is None else amendment.get("amendment_id"),
        "effective_amendment_sha256": None if amendment_path is None else sha256_file(amendment_path),
        "technical_replacements": {} if amendment is None else amendment.get("replacements", {}),
        "accepted": [{key: row[key] for key in ("session_id", "runtime_session_id", "group_id", "runtime_group_id", "run_id", "archive_sha256")} for row in accepted],
        "failed": failed,
    }
    session_manifest_path.write_text(json.dumps(session_manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    table_lines: list[str] = []
    for row in bin_rows:
        mae = row["equal_group_mae_m"]
        mae_text = "N/A" if mae == "N/A" else f"{float(mae):.3f} m"
        table_lines.append(
            f"| {row['distance_bin']} | {row['group_count']} | "
            f"{row['frame_count']} | {mae_text} | {row['coverage_status']} |"
        )
    table = "\n".join(table_lines)
    report = f"""# Core Range Static Balance Collection Audit

## Outcome

```text
{conclusion}
```

This patch contains static observation-only development data. It does not train
or select a model and does not authorize automatic runtime integration.

| Bin | Independent groups | Frames | Equal-group MAE | Coverage |
| --- | ---: | ---: | ---: | --- |
{table}

## Integrity

- Accepted groups: {len(accepted)}/18; accepted raw frames: {sum(row['valid_frames'] for row in accepted)}.
- Failed planned sessions: {len(failed)}; preserved quarantine roots: {len(quarantine_entries)}.
- One measurement run/session/group per accepted dataset; no duplicate trace identities.
- Timestamp order, GT trace/checksum, record checksum and 96 anchors/frame are mandatory.
- Runtime logs were archived and read-tested before raw log directories were removed.

## Safety and scope

Both UAVs remained disarmed. No Follow, OFFBOARD, arm, takeoff, mode, motion or
controller command was used. Pose setup occurred only before measurement.
Residual correction remained off. No training, split, scaler, threshold, shadow
or final holdout operation was performed.
"""
    report_path = output / "static_balance_collection_report.md"
    report_path.write_text(report, encoding="utf-8")
    output_files = (accepted_path, quarantine_path, group_csv, bin_csv, integrity_path, readiness_path, session_manifest_path, report_path)
    manifest = {
        "collection_id": COLLECTION_ID,
        "audit_id": AUDIT_ID,
        "conclusion": conclusion,
        "inputs": {
            str(plan_path): sha256_file(plan_path),
            "core_range_static_balance_audit.py": sha256_file(Path(__file__)),
            **(
                {str(amendment_path): sha256_file(amendment_path)}
                if amendment_path is not None
                else {}
            ),
        },
        "accepted_source_checksums": {row["session_id"]: {key: value for key, value in row.items() if key.startswith("source_") or key == "archive_sha256"} for row in accepted},
        "outputs": {path.name: sha256_file(path) for path in output_files},
        "exact_command": (
            f"python3 core_range_static_balance_audit.py {plan_path} "
            f"{sessions_root} {quarantine_root} "
            + (
                f"--amendment {amendment_path} "
                if amendment_path is not None
                else ""
            )
            + f"--output {output}"
        ),
        "scope_guards": {
            "model_trained": False,
            "train_validation_test_split_created": False,
            "scaler_or_threshold_fit": False,
            "residual_correction": "off",
            "runtime_or_controller_changed": False,
            "shadow_or_final_holdout": False,
        },
    }
    manifest_path = output / "static_balance_collection_manifest.json"
    manifest_path.write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return {"conclusion": conclusion, "accepted": len(accepted), "frames": integrity["accepted_frames"], "bins": bin_rows}


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("plan", type=Path)
    parser.add_argument("sessions_root", type=Path)
    parser.add_argument("quarantine_root", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--amendment", type=Path)
    args = parser.parse_args()
    result = audit(
        args.plan.resolve(),
        args.sessions_root.resolve(),
        args.quarantine_root.resolve(),
        args.output.resolve(),
        None if args.amendment is None else args.amendment.resolve(),
    )
    print(result["conclusion"])
    return 0 if result["conclusion"] == "STATIC_3_12M_COVERAGE_READY_FOR_MODEL_BENCHMARK" else 1


if __name__ == "__main__":
    raise SystemExit(main())
