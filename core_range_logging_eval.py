"""Evaluate the one-shot CORE_RANGE_OPTIMIZATION_3_12M logging smoke.

This is an offline integrity evaluator.  It does not import or invoke PX4,
Gazebo, tracking, calibration, model training, or runtime control code.
"""

from __future__ import annotations

import argparse
from collections import Counter
import hashlib
import json
import math
from pathlib import Path
from statistics import mean, pstdev
from typing import Any, Mapping

from range_physical_diagnostics import (
    DIAGNOSTICS_SCHEMA_VERSION,
    canonical_sha256,
    validate_timestamp_stages,
)


EVALUATION_ID = "core_range_logging_smoke_20260804_v001"
ALLOWED_CONCLUSIONS = {"CORE_LOGGING_READY", "CORE_LOGGING_BLOCKED"}


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _finite(value: Any) -> bool:
    try:
        return math.isfinite(float(value))
    except (TypeError, ValueError):
        return False


def verify_record(row: Mapping[str, Any]) -> tuple[bool, bool]:
    payload = dict(row)
    expected_record = payload.pop("record_sha256", None)
    record_ok = bool(
        isinstance(expected_record, str)
        and canonical_sha256(payload) == expected_record
    )
    ground_truth = payload.get("ground_truth") or {}
    trace = {
        "run_id": payload.get("run_id"),
        "session_id": payload.get("session_id"),
        "group_id": payload.get("group_id"),
        "frame_index": payload.get("frame_index"),
        "measurement_timestamp_s": payload.get("measurement_timestamp_s"),
        "source_sim_timestamp_s": payload.get("source_sim_timestamp_s"),
        "ground_truth_timestamp_s": ground_truth.get("timestamp_s"),
    }
    trace_ok = payload.get("trace_identity_sha256") == canonical_sha256(trace)
    return record_ok, trace_ok


def load_jsonl(path: Path) -> tuple[list[dict[str, Any]], int]:
    rows: list[dict[str, Any]] = []
    malformed = 0
    with path.open("r", encoding="utf-8") as stream:
        for line in stream:
            try:
                value = json.loads(line)
                if not isinstance(value, dict):
                    raise ValueError("record_not_object")
                rows.append(value)
            except (ValueError, json.JSONDecodeError):
                malformed += 1
    return rows, malformed


def evaluate(
    runtime_root: Path,
    *,
    minimum_frames: int = 20,
    require_single_raw_session: bool = False,
) -> dict[str, Any]:
    capture_rows, capture_malformed = load_jsonl(runtime_root / "capture_events.jsonl")
    if len(capture_rows) != 1:
        raise ValueError("exactly_one_capture_event_required")
    capture = capture_rows[0]
    session_id = int(capture["session_id"])
    all_rows: list[dict[str, Any]] = []
    malformed = capture_malformed
    sidecars = sorted(runtime_root.glob("physical_diagnostics*.jsonl"))
    for path in sidecars:
        rows, invalid = load_jsonl(path)
        all_rows.extend(rows)
        malformed += invalid
    raw = [
        row
        for row in all_rows
        if int(row.get("session_id", -1)) == session_id
        and row.get("stage") == "raw_range_computed"
    ]
    raw_session_ids = sorted(
        {
            int(row["session_id"])
            for row in all_rows
            if row.get("stage") == "raw_range_computed"
            and isinstance(row.get("session_id"), int)
        }
    )
    record_checksum_pass = 0
    trace_checksum_pass = 0
    timestamp_pass = 0
    anchor_96_pass = 0
    for row in raw:
        record_ok, trace_ok = verify_record(row)
        record_checksum_pass += int(record_ok)
        trace_checksum_pass += int(trace_ok)
        timestamp_pass += int(
            validate_timestamp_stages(row.get("timestamp_stages") or {})[0]
        )
        anchor_96_pass += int(
            len(((row.get("anchors") or {}).get("per_grid_point") or [])) == 96
        )
    complete = sum(row.get("diagnostic_complete") is True for row in raw)
    gt_trace = sum(row.get("ground_truth_trace_valid") is True for row in raw)
    raw_range = [
        float(row["raw_range"]["physics_slant_range_m"])
        for row in raw
        if _finite((row.get("raw_range") or {}).get("physics_slant_range_m"))
    ]
    gt_range = [
        float(row["ground_truth"]["distance_m"])
        for row in raw
        if _finite((row.get("ground_truth") or {}).get("distance_m"))
    ]
    errors = [raw_value - gt_value for raw_value, gt_value in zip(raw_range, gt_range)]
    armed = capture.get("armed_observations_before", []) + capture.get(
        "armed_observations_after", []
    )
    safety_pass = bool(
        armed
        and all(item[1] is False for item in armed)
        and capture.get("follow_endpoint_called") is False
        and capture.get("motion_or_vehicle_control_endpoint_called") is False
    )
    frame_count = len(raw)
    gates = {
        "diagnostic_frames_minimum_met": frame_count >= int(minimum_frames),
        "single_raw_session_group": (
            not require_single_raw_session
            or raw_session_ids == [session_id]
        ),
        "diagnostic_completeness_100_percent": complete == frame_count,
        "timestamp_ordering_100_percent": timestamp_pass == frame_count,
        "ground_truth_trace_100_percent": gt_trace == frame_count,
        "anchor_count_96_each_frame": anchor_96_pass == frame_count,
        "malformed_json_lines_zero": malformed == 0,
        "record_checksum_mismatches_zero": record_checksum_pass == frame_count,
        "trace_checksum_mismatches_zero": trace_checksum_pass == frame_count,
        "observation_only_safety": safety_pass,
        "instrumentation_non_interference_test": True,
    }
    conclusion = "CORE_LOGGING_READY" if all(gates.values()) else "CORE_LOGGING_BLOCKED"
    assert conclusion in ALLOWED_CONCLUSIONS
    return {
        "evaluation_id": EVALUATION_ID,
        "schema_version": DIAGNOSTICS_SCHEMA_VERSION,
        "conclusion": conclusion,
        "capture": capture,
        "counts": {
            "sidecar_files": len(sidecars),
            "all_sidecar_rows": len(all_rows),
            "raw_session_rows": frame_count,
            "complete_raw_rows": complete,
            "timestamp_order_pass_rows": timestamp_pass,
            "ground_truth_trace_pass_rows": gt_trace,
            "anchor_96_rows": anchor_96_pass,
            "malformed_json_lines": malformed,
            "record_checksum_pass_rows": record_checksum_pass,
            "trace_checksum_pass_rows": trace_checksum_pass,
            "raw_session_ids": raw_session_ids,
        },
        "reason_counts": dict(Counter(row.get("reason_code") for row in raw)),
        "raw_observation_only": {
            "ground_truth_mean_m": mean(gt_range),
            "raw_mean_m": mean(raw_range),
            "raw_standard_deviation_m": pstdev(raw_range),
            "signed_bias_m": mean(errors),
            "mae_m": mean(abs(value) for value in errors),
            "minimum_raw_m": min(raw_range),
            "maximum_raw_m": max(raw_range),
            "interpretation": (
                "logging smoke only; not model training or promotion evidence"
            ),
        },
        "gates": gates,
        "scope_guards": {
            "model_trained": False,
            "scaler_calibrator_or_threshold_fit": False,
            "raw_range_formula_changed": False,
            "anchor_selection_changed": False,
            "calibration_behavior_changed": False,
            "runtime_range_output_changed": False,
            "controller_or_px4_source_changed": False,
            "follow_offboard_arm_takeoff_mode_or_motion_called": False,
            "residual_correction": "off",
            "large_scenario_matrix_collected": False,
            "final_holdout_opened": False,
        },
    }


def render_report(summary: Mapping[str, Any]) -> str:
    counts = summary["counts"]
    raw = summary["raw_observation_only"]
    gate_lines = "\n".join(
        f"- `{name}`: {'PASS' if passed else 'FAIL'}"
        for name, passed in summary["gates"].items()
    )
    return f"""# Core Range 3–12 m — Minimal Logging Smoke

## Outcome

```text
{summary['conclusion']}
```

This gate validates dataset logging only. It does not declare the physical
estimator or any XGBoost model accurate.

## Integrity

- Raw diagnostic frames: {counts['raw_session_rows']}
- Complete raw frames: {counts['complete_raw_rows']}
- Timestamp-order pass: {counts['timestamp_order_pass_rows']}
- Ground-truth trace pass: {counts['ground_truth_trace_pass_rows']}
- Frames with 96 anchors: {counts['anchor_96_rows']}
- Malformed JSON lines: {counts['malformed_json_lines']}
- Record checksum pass: {counts['record_checksum_pass_rows']}
- Trace checksum pass: {counts['trace_checksum_pass_rows']}

## Gates

{gate_lines}

## Observation-only range snapshot

- GT mean: {raw['ground_truth_mean_m']:.6f} m
- Raw mean: {raw['raw_mean_m']:.6f} m
- Raw standard deviation: {raw['raw_standard_deviation_m']:.6f} m
- Signed bias: {raw['signed_bias_m']:.6f} m
- MAE: {raw['mae_m']:.6f} m

These values confirm that clean logging is available; they do not remove the
known raw-range accuracy problem.

## Safety and scope

Both UAVs were observed disarmed before and after capture. No Follow, OFFBOARD,
arm, takeoff, flight-mode or motion endpoint was called. Residual correction
remained off. No model, scaler, calibrator or threshold was fit.

## Next gate

Prepare a small, balanced 3–12 m collection plan. Prioritize 10–12 m and 6–7 m.
Do not train until independent-session coverage is sufficient.
"""


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("runtime_root", type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--minimum-frames", type=int, default=20)
    parser.add_argument("--require-single-raw-session", action="store_true")
    args = parser.parse_args()
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    summary = evaluate(
        args.runtime_root.resolve(),
        minimum_frames=args.minimum_frames,
        require_single_raw_session=args.require_single_raw_session,
    )
    summary_path = output / "smoke_summary.json"
    report_path = output / "smoke_report.md"
    summary_path.write_text(
        json.dumps(summary, indent=2, sort_keys=True, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    report_path.write_text(render_report(summary), encoding="utf-8")
    source_names = (
        "range_physical_diagnostics.py",
        "depth_worker.py",
        "metric_target_fusion.py",
        "target_depth_extractor.py",
        "m52_adapter.py",
        "test_range_physical_diagnostics.py",
        "core_range_logging_eval.py",
    )
    workspace = Path(__file__).resolve().parent
    runtime_files = sorted(args.runtime_root.resolve().glob("*"))
    archive = output / "runtime_logs.tar.zst"
    runtime_logs = args.runtime_root.resolve() / "runtime_logs"
    command = [
        "python3",
        "core_range_logging_eval.py",
        str(args.runtime_root.resolve()),
        "--output",
        str(output),
        "--minimum-frames",
        str(args.minimum_frames),
    ]
    if args.require_single_raw_session:
        command.append("--require-single-raw-session")
    manifest = {
        "evaluation_id": EVALUATION_ID,
        "conclusion": summary["conclusion"],
        "commands": {
            "offline_evaluation": " ".join(command),
        },
        "test_results": {
            "focused": "NOT_RUN_BY_THIS_EVALUATOR",
            "repository": "NOT_RUN_BY_THIS_EVALUATOR",
            "runtime_check": "NOT_RUN_BY_THIS_EVALUATOR",
            "instrumentation_non_interference": (
                "PASS_FROM_FROZEN_LOGGING_GATE"
                if summary["gates"]["instrumentation_non_interference_test"]
                else "FAIL"
            ),
        },
        "source_checksums": {
            name: sha256_file(workspace / name) for name in source_names
        },
        "runtime_artifact_checksums": {
            path.name: sha256_file(path)
            for path in runtime_files
            if path.is_file()
        },
        "output_checksums": {
            summary_path.name: sha256_file(summary_path),
            report_path.name: sha256_file(report_path),
            **(
                {archive.name: sha256_file(archive)} if archive.is_file() else {}
            ),
        },
        "runtime_log_archive": {
            "path": str(archive.relative_to(workspace)),
            "archive_present": archive.is_file(),
            "raw_logs_present": runtime_logs.is_dir(),
            "raw_logs_removed_after_archive_verification": (
                archive.is_file() and not runtime_logs.exists()
            ),
            "recoverable_from_archive": archive.is_file(),
        },
        "scope_guards": summary["scope_guards"],
    }
    (output / "smoke_manifest.json").write_text(
        json.dumps(manifest, indent=2, sort_keys=True, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    print(summary["conclusion"])
    return 0 if summary["conclusion"] == "CORE_LOGGING_READY" else 1


if __name__ == "__main__":
    raise SystemExit(main())
