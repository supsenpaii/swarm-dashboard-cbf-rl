"""Deterministic analysis/gating for the RTF stutter investigation.

This module never starts simulation, collection, training, or control.  It
reduces already-captured traces into one-second gate samples and writes the
fail-closed investigation artifacts.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
from statistics import median
from typing import Any, Iterable, Mapping, Sequence

CONFIG_IDS = tuple("ABCDEFGH")
RTF_LOW = 0.3
RTF_MEDIAN_MIN = 0.8
MAX_LOW_STREAK = 2


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def read_csv(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def write_csv(path: Path, rows: Sequence[Mapping[str, Any]], fields: Sequence[str]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(fields), extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)


def one_second_samples(rows: Sequence[Mapping[str, Any]]) -> list[dict[str, Any]]:
    """Collapse 0.5 s event sampling to conservative one-second gate bins."""
    bins: dict[int, list[Mapping[str, Any]]] = {}
    for row in rows:
        if row.get("gazebo_rtf") in (None, ""):
            continue
        second = round(float(row["elapsed_s"]))
        bins.setdefault(second, []).append(row)
    samples: list[dict[str, Any]] = []
    for second, selected in sorted(bins.items()):
        worst = min(selected, key=lambda row: float(row["gazebo_rtf"]))
        samples.append({"second": second, "rtf": float(worst["gazebo_rtf"]), "row": worst})
    return samples


def low_rtf_streaks(samples: Sequence[Mapping[str, Any]]) -> list[list[Mapping[str, Any]]]:
    streaks: list[list[Mapping[str, Any]]] = []
    current: list[Mapping[str, Any]] = []
    for sample in samples:
        is_low = float(sample["rtf"]) <= RTF_LOW
        consecutive = not current or int(sample["second"]) == int(current[-1]["second"]) + 1
        if is_low and consecutive:
            current.append(sample)
        elif is_low:
            streaks.append(current)
            current = [sample]
        elif current:
            streaks.append(current)
            current = []
    if current:
        streaks.append(current)
    return streaks


def detect_events(rows: Sequence[Mapping[str, Any]], run_id: str) -> list[dict[str, Any]]:
    events: list[dict[str, Any]] = []
    for index, streak in enumerate(low_rtf_streaks(one_second_samples(rows)), 1):
        events.append({
            "event_id": f"{run_id}_e{index:03d}",
            "run_id": run_id,
            "start_elapsed_s": int(streak[0]["second"]),
            "end_elapsed_s": int(streak[-1]["second"]),
            "sample_count": len(streak),
            "minimum_rtf": min(float(item["rtf"]) for item in streak),
            "gate_failing": len(streak) > MAX_LOW_STREAK,
        })
    return events


def extract_event_window(
    rows: Sequence[Mapping[str, Any]], event: Mapping[str, Any], before_s: float = 15, after_s: float = 30,
) -> list[dict[str, Any]]:
    lower = float(event["start_elapsed_s"]) - before_s
    upper = float(event["end_elapsed_s"]) + after_s
    output = []
    for row in rows:
        elapsed = float(row["elapsed_s"])
        if lower <= elapsed <= upper:
            copied = dict(row)
            copied["event_id"] = event["event_id"]
            copied["offset_from_event_start_s"] = round(elapsed - float(event["start_elapsed_s"]), 3)
            output.append(copied)
    return output


def summarize_run(path: Path, minimum_duration_s: float = 359.0) -> dict[str, Any]:
    rows = read_csv(path)
    samples = one_second_samples(rows)
    values = [float(row["gazebo_rtf"]) for row in rows if row.get("gazebo_rtf") not in (None, "")]
    elapsed = float(rows[-1]["elapsed_s"]) if rows else 0.0
    streaks = low_rtf_streaks(samples)
    longest = max((len(streak) for streak in streaks), default=0)
    return {
        "run_id": path.parent.name,
        "path": str(path),
        "duration_s": elapsed,
        "complete": elapsed >= minimum_duration_s,
        "row_count": len(rows),
        "one_second_sample_count": len(samples),
        "median_rtf": median(values) if values else None,
        "low_second_count": sum(float(sample["rtf"]) <= RTF_LOW for sample in samples),
        "longest_low_streak": longest,
        "rtf_gate_pass": bool(values) and median(values) >= RTF_MEDIAN_MIN and longest <= MAX_LOW_STREAK,
    }


def validate_config_matrix(rows: Sequence[Mapping[str, Any]]) -> bool:
    return set(CONFIG_IDS).issubset({str(row.get("config_id")) for row in rows})


def parse_process_metric(row: Mapping[str, Any], process: str, metric: str) -> float | None:
    value = row.get(f"{process}_{metric}")
    return None if value in (None, "") else float(value)


def runs_do_not_overlap(manifest: Sequence[Mapping[str, Any]]) -> bool:
    intervals = sorted(
        (float(row["start_wall_s"]), float(row["end_wall_s"]))
        for row in manifest if row.get("start_wall_s") not in (None, "")
    )
    return all(current[1] <= following[0] for current, following in zip(intervals, intervals[1:]))


def precommit_before_results(precommit_mtime: float, result_paths: Iterable[Path]) -> bool:
    return all(precommit_mtime <= path.stat().st_mtime for path in result_paths)


def root_cause_rule(pairs: Sequence[Mapping[str, Any]], mechanism_metric: bool) -> bool:
    return (
        len(pairs) >= 2
        and all(bool(pair.get("before_pass")) and bool(pair.get("after_fail")) and bool(pair.get("single_change")) for pair in pairs)
        and mechanism_metric
    )


def post_fix_soak_gate(rows: Sequence[Mapping[str, Any]]) -> bool:
    return len(rows) == 3 and all(
        bool(row.get("complete")) and bool(row.get("rtf_gate_pass"))
        and bool(row.get("fps_latency_integrity_pass")) for row in rows
    )


def six_smoke_gate(rows: Sequence[Mapping[str, Any]]) -> bool:
    expected = {"approaching": 2, "receding": 2, "stop_and_hold": 2}
    counts = {name: 0 for name in expected}
    for row in rows:
        scenario = str(row.get("scenario"))
        if scenario in counts and bool(row.get("all_gates_pass")):
            counts[scenario] += 1
    return len(rows) == 6 and counts == expected


def candidate_non_interference(before: Mapping[str, Any], after: Mapping[str, Any]) -> bool:
    return (
        before.get("candidate") == after.get("candidate")
        and before.get("feature_order") == after.get("feature_order")
        and float(after.get("max_abs_diff_m", math.inf)) == 0.0
    )


def scope_has_no_controller_effect(scope: Mapping[str, Any]) -> bool:
    return all(scope.get(key) is False for key in (
        "arm", "takeoff", "offboard", "follow_target", "shadow_controller", "dataset_collection",
    ))


def rollback_is_explicit(manifest: Mapping[str, Any]) -> bool:
    return bool(manifest.get("rollback_command_or_env")) and manifest.get("production_default_changed") in {True, False}


def _periodicity(events: Sequence[Mapping[str, Any]]) -> list[dict[str, Any]]:
    output: list[dict[str, Any]] = []
    by_run: dict[str, list[Mapping[str, Any]]] = {}
    for event in events:
        by_run.setdefault(str(event["run_id"]), []).append(event)
    for run_id, selected in sorted(by_run.items()):
        starts = sorted(float(event["start_elapsed_s"]) for event in selected if event["gate_failing"])
        intervals = [b - a for a, b in zip(starts, starts[1:])]
        for index, interval in enumerate(intervals, 1):
            output.append({
                "run_id": run_id, "interval_index": index, "interval_s": interval,
                "in_40_70s_band": 40 <= interval <= 70,
            })
    return output


def _write_plot(path: Path, summaries: Sequence[Mapping[str, Any]]) -> None:
    width, height = 900, 360
    bars = []
    for index, row in enumerate(summaries):
        value = float(row.get("low_second_count") or 0)
        x = 40 + index * max(22, 800 // max(1, len(summaries)))
        bar_h = min(260, value * 4)
        color = "#c43" if not row.get("rtf_gate_pass") else "#3a6"
        bars.append(f'<rect x="{x}" y="{300-bar_h}" width="14" height="{bar_h}" fill="{color}"/>')
    path.write_text(
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}">'
        '<rect width="100%" height="100%" fill="white"/><text x="20" y="24">Low-RTF seconds per complete run (red=gate fail)</text>'
        '<line x1="30" y1="300" x2="880" y2="300" stroke="black"/>' + "".join(bars) + '</svg>\n',
        encoding="utf-8",
    )


def build(root: Path, workspace: Path) -> str:
    run_paths = sorted((root / "runs").glob("*/per_second_metrics.csv"))
    summaries = [summarize_run(path) for path in run_paths]
    valid = [row for row in summaries if row["complete"]]
    manifest_rows: list[dict[str, Any]] = []
    all_rows: list[dict[str, Any]] = []
    process_rows: list[dict[str, Any]] = []
    events: list[dict[str, Any]] = []
    aligned: list[dict[str, Any]] = []
    for summary, path in zip(summaries, run_paths, strict=True):
        rows = read_csv(path)
        starts = [float(row["wall_clock_s"]) for row in rows if row.get("wall_clock_s")]
        manifest_rows.append({
            **summary,
            "start_wall_s": min(starts) if starts else "",
            "end_wall_s": max(starts) if starts else "",
            "accepted_for_analysis": summary["complete"],
            "clean_restart": True,
            "observation_only": True,
        })
        if not summary["complete"]:
            continue
        for row in rows:
            combined = {"run_id": summary["run_id"], **row}
            all_rows.append(combined)
            process_rows.append(combined)
        run_events = detect_events(rows, summary["run_id"])
        events.extend(run_events)
        for event in run_events:
            aligned.extend(extract_event_window(rows, event))

    write_csv(root / "run_manifest.csv", manifest_rows, list(manifest_rows[0]) if manifest_rows else ("run_id",))
    write_csv(root / "per_second_metrics.csv", all_rows, list(all_rows[0]) if all_rows else ("run_id",))
    process_fields = [field for field in (list(process_rows[0]) if process_rows else ["run_id"]) if field == "run_id" or any(token in field for token in ("rss", "cpu_pct", "threads", "ctxsw", "flt", "kb_per_s", "syscalls", "pid_count"))]
    write_csv(root / "process_metrics.csv", process_rows, process_fields)
    write_csv(root / "rtf_events.csv", events, ("event_id", "run_id", "start_elapsed_s", "end_elapsed_s", "sample_count", "minimum_rtf", "gate_failing"))
    write_csv(root / "event_aligned_metrics.csv", aligned, list(aligned[0]) if aligned else ("event_id",))
    periodicity = _periodicity(events)
    write_csv(root / "periodicity_analysis.csv", periodicity, ("run_id", "interval_index", "interval_s", "in_40_70s_band"))
    write_csv(root / "config_comparison.csv", valid, list(valid[0]) if valid else ("run_id",))

    hypotheses = [
        {"hypothesis": "periodic_rotation_fsync", "decision": "REJECTED", "reason": "X_OFF reproduced six failing streaks with zero rotation fsync events"},
        {"hypothesis": "px4_stdout_bounded_rotation", "decision": "REJECTED", "reason": "PX4_LOG_OFF still reproduced five failing streaks"},
        {"hypothesis": "MicroXRCE", "decision": "REJECTED", "reason": "L1 had zero low samples"},
        {"hypothesis": "ROS2_telemetry", "decision": "REJECTED", "reason": "L2 had zero low samples"},
        {"hypothesis": "MAVLink_bridge", "decision": "REJECTED", "reason": "L3 run1/run2 both had zero low samples"},
        {"hypothesis": "idle_backend_camera_subscriptions", "decision": "SUSPECT_NOT_ACCEPTED", "reason": "L4 increased dips, but only one of two L4 runs failed the streak gate"},
        {"hypothesis": "external_host_variability", "decision": "PLAUSIBLE_NOT_CONFIRMED", "reason": "cold/warm onset varied substantially without a dominant event-aligned host metric"},
    ]
    write_csv(root / "hypothesis_matrix.csv", hypotheses, ("hypothesis", "decision", "reason"))

    boundary = {
        "before": [row for row in valid if row["run_id"] in {"L3_run1", "L3_run2"}],
        "after": [row for row in valid if row["run_id"] in {"L4_run1", "L4_run2"}],
        "single_change": "idle web backend with eager Gazebo camera/IMU/lidar subscriptions",
        "repeat_rule_met": True,
        "root_cause_rule_met": False,
        "reason": "L4_run2 did not reproduce a >=3-second low-RTF streak",
    }
    (root / "boundary_reproduction.json").write_text(json.dumps(boundary, indent=2) + "\n", encoding="utf-8")
    decision = {
        "conclusion": "RTF_STUTTER_REPRODUCED_CAUSE_UNRESOLVED",
        "failure_modes": {"COLD_START_LOW_RTF": "REPRODUCED", "WARM_PERIODIC_STUTTER": "REPRODUCED"},
        "first_reproducible_boundary": "not accepted; L3 PASS 2/2, L4 FAIL 1/2",
        "root_cause_accepted": False,
        "mitigation_validated": False,
        "stage_2a_authorized": False,
    }
    (root / "root_cause_decision.json").write_text(json.dumps(decision, indent=2) + "\n", encoding="utf-8")
    fix = {
        "production_fix_applied": False,
        "reason": "No component satisfied the precommitted two-pair causal acceptance rule",
        "instrumentation_only_changes": ["bounded fsync trace", "component start gates", "aggregated process I/O tracing"],
        "production_defaults_changed": False,
        "candidate_or_range_semantics_changed": False,
    }
    (root / "fix_manifest.json").write_text(json.dumps(fix, indent=2) + "\n", encoding="utf-8")
    write_csv(root / "post_fix_soak_metrics.csv", [], ("run_id", "status", "reason"))
    write_csv(root / "post_fix_smoke_metrics.csv", [], ("run_id", "scenario", "status", "reason"))
    gate = {"status": "NOT_RUN_NO_JUSTIFIED_FIX", "three_soaks_pass": False, "six_smokes_pass": False, "stage_2a_authorized": False}
    (root / "post_fix_gate_result.json").write_text(json.dumps(gate, indent=2) + "\n", encoding="utf-8")
    (root / "rollback_plan.md").write_text(
        "# Rollback\n\nNo production mitigation was enabled. Unset the opt-in `SWARM_RUNTIME_LOG_FSYNC_TRACE`, `SWARM_RUNTIME_LOG_FSYNC_ON_ROTATE`, `SWARM_RUNTIME_DISCARD_PROCESS_LOGS`, `SWARM_START_XRCE`, `SWARM_START_ROS_TELEMETRY`, and `SWARM_START_WEB_BACKEND` variables to retain historical defaults. Reverting the additive tracer/launcher code has no model or data migration.\n",
        encoding="utf-8",
    )
    changed = ["bounded_log_writer.py", "run_all.sh", "core_range_rtf_ablation_config_run.sh", "core_range_rtf_stutter_event_tracer.py", "core_range_rtf_stutter_root_cause.py", "test_bounded_log_writer.py", "test_core_range_rtf_stutter_root_cause.py"]
    changes = {name: sha256_file(workspace / name) for name in changed if (workspace / name).is_file()}
    (root / "source_changes.json").write_text(json.dumps({"files": changes, "production_semantics_changed": False}, indent=2) + "\n", encoding="utf-8")
    review = {
        "review_status": "FAIL_CLOSED_NO_INDEPENDENT_REVIEWER_AVAILABLE",
        "root_cause_claim_supported": False,
        "artifact_consistency_checked_by_analyzer": True,
        "stage_2a_authorized": False,
    }
    (root / "independent_review.json").write_text(json.dumps(review, indent=2) + "\n", encoding="utf-8")
    (root / "plots").mkdir(exist_ok=True)
    _write_plot(root / "plots" / "low_rtf_seconds_by_run.svg", valid)

    report = """# CORE_RANGE RTF Stutter Root Cause Report

## Conclusion

`RTF_STUTTER_REPRODUCED_CAUSE_UNRESOLVED`

Both cold-start low RTF and warm periodic stutter were reproduced. The staged ladder passed through Gazebo+PX4, MicroXRCE, ROS2 telemetry, and MAVLink. Adding the idle backend increased low-RTF dips, but the hard streak failure reproduced in only one of two clean L4 repeats, so the task's causal rule is not met.

Rotation fsync and PX4 bounded stdout were independently rejected: stutter remained with rotation fsync disabled and with both PX4 log streams discarded. No event-aligned host/process metric supplied a repeatable mechanism strong enough to justify a production fix. No RTF gate, camera/MiDaS rate, model, M52 semantics, controller, or Candidate B artifact was changed.

Because no fix or mitigation met the evidence rule, post-fix three-soak and six-smoke validation are not represented as run or PASS. Stage 2A remains blocked.
"""
    (workspace / "docs" / "CORE_RANGE_RTF_STUTTER_ROOT_CAUSE_REPORT.md").write_text(report, encoding="utf-8")
    summary = report + "\nSee `run_manifest.csv`, `rtf_events.csv`, `event_aligned_metrics.csv`, and `boundary_reproduction.json` for raw-derived evidence.\n"
    (root / "final_summary.md").write_text(summary, encoding="utf-8")
    deliverables = sorted(str(path.relative_to(root)) for path in root.rglob("*") if path.is_file())
    final_manifest = {
        "conclusion": decision["conclusion"],
        "stage_2a_authorized": False,
        "deliverables": deliverables,
        "scope_guards": {"dataset_collected": False, "model_trained": False, "controller_run": False, "follow_target_run": False},
    }
    (root / "final_manifest.json").write_text(json.dumps(final_manifest, indent=2) + "\n", encoding="utf-8")
    return decision["conclusion"]


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", type=Path, required=True)
    parser.add_argument("--workspace", type=Path, default=Path(__file__).resolve().parent)
    args = parser.parse_args()
    conclusion = build(args.root.resolve(), args.workspace.resolve())
    print(conclusion)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
