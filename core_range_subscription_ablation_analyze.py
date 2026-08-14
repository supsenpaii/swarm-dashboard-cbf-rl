#!/usr/bin/env python3
"""Fail-closed reduction of the eager Gazebo subscription ablation matrix."""

from __future__ import annotations

import argparse
import csv
import json
import math
from collections import defaultdict
from pathlib import Path
from statistics import median
from typing import Any, Iterable, Mapping, Sequence

RTF_LOW = 0.3
RTF_MEDIAN_MIN = 0.8
MAX_LOW_STREAK = 2
MIN_TRACE_ELAPSED_S = 599.0
CONFIGS = (
    "S0", "S1", "S2_camera_uav01", "S2_camera_uav02",
    "S2_body_imu_uav01", "S2_camera_imu_uav01",
    "S2_body_imu_uav02", "S2_camera_imu_uav02",
    "S2_front_lidar", "S3", "S4", "S5",
)


def read_csv(path: Path) -> list[dict[str, str]]:
    if not path.is_file():
        return []
    with path.open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def write_csv(path: Path, rows: Sequence[Mapping[str, Any]], fields: Sequence[str]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(fields), extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)


def number(value: Any) -> float | None:
    try:
        parsed = float(value)
    except (TypeError, ValueError):
        return None
    return parsed if math.isfinite(parsed) else None


def one_second(rows: Sequence[Mapping[str, Any]]) -> list[dict[str, Any]]:
    bins: dict[int, list[Mapping[str, Any]]] = defaultdict(list)
    for row in rows:
        elapsed = number(row.get("elapsed_s"))
        rtf = number(row.get("gazebo_rtf"))
        if elapsed is not None and rtf is not None:
            bins[round(elapsed)].append(row)
    output = []
    for second, selected in sorted(bins.items()):
        worst = min(selected, key=lambda row: float(row["gazebo_rtf"]))
        output.append({"second": second, "rtf": float(worst["gazebo_rtf"]), "row": worst})
    return output


def events(rows: Sequence[Mapping[str, Any]], run_id: str) -> list[dict[str, Any]]:
    found: list[dict[str, Any]] = []
    current: list[Mapping[str, Any]] = []
    for sample in one_second(rows):
        low = sample["rtf"] <= RTF_LOW
        consecutive = not current or sample["second"] == current[-1]["second"] + 1
        if low and consecutive:
            current.append(sample)
        else:
            if current:
                found.append(_event(run_id, len(found) + 1, current))
            current = [sample] if low else []
    if current:
        found.append(_event(run_id, len(found) + 1, current))
    return found


def _event(run_id: str, index: int, streak: Sequence[Mapping[str, Any]]) -> dict[str, Any]:
    return {
        "event_id": f"{run_id}_e{index:03d}",
        "run_id": run_id,
        "start_elapsed_s": streak[0]["second"],
        "end_elapsed_s": streak[-1]["second"],
        "low_second_count": len(streak),
        "minimum_rtf": min(float(item["rtf"]) for item in streak),
        "gate_failing": len(streak) > MAX_LOW_STREAK,
    }


def summarize(run_dir: Path) -> tuple[dict[str, Any], list[dict[str, str]], list[dict[str, str]]]:
    run_id = run_dir.name
    config_id, _, repeat_text = run_id.rpartition("_run")
    trace = read_csv(run_dir / "per_second_metrics.csv")
    callbacks = read_csv(run_dir / "callback_phase_raw.csv")
    elapsed = max((number(row.get("elapsed_s")) or 0.0 for row in trace), default=0.0)
    samples = one_second(trace)
    values = [float(item["rtf"]) for item in samples]
    run_events = events(trace, run_id)
    longest = max((int(item["low_second_count"]) for item in run_events), default=0)
    exit_code_rows = (run_dir / "exit_code.txt").read_text().strip() if (run_dir / "exit_code.txt").is_file() else ""
    complete = elapsed >= MIN_TRACE_ELAPSED_S and exit_code_rows == "0"
    gate_pass = complete and bool(values) and median(values) >= RTF_MEDIAN_MIN and longest <= MAX_LOW_STREAK
    return ({
        "run_id": run_id, "config_id": config_id, "repeat": repeat_text,
        "requested_duration_s": 600, "trace_elapsed_s": round(elapsed, 3),
        "complete": complete, "exit_code": exit_code_rows,
        "rtf_samples": len(values), "median_rtf": median(values) if values else "",
        "low_rtf_seconds": sum(value <= RTF_LOW for value in values),
        "longest_low_rtf_streak_s": longest, "rtf_gate_pass": gate_pass,
        "clean_restart_verified": exit_code_rows == "0",
        "callback_rows": len(callbacks),
        "trace_path": str(run_dir / "per_second_metrics.csv"),
        "callback_path": str(run_dir / "callback_phase_raw.csv"),
    }, trace, callbacks)


def callback_second(row: Mapping[str, Any]) -> int | None:
    value = number(row.get("monotonic_second"))
    return int(value) if value is not None else None


def trace_second(row: Mapping[str, Any]) -> int | None:
    value = number(row.get("monotonic_ns"))
    return int(value // 1_000_000_000) if value is not None else None


def _phase_rows(run_id: str, config_id: str, repeat: str, rows: Iterable[Mapping[str, Any]]) -> list[dict[str, Any]]:
    output = []
    for row in rows:
        output.append({
            "run_id": run_id, "config_id": config_id, "repeat": repeat,
            **row,
            "deserialize_ms_p50": "", "deserialize_ms_p95": "",
            "deserialize_ms_p99": "", "deserialize_ms_max": "",
            "deserialize_phase_observable": False,
            "deserialize_note": "gz.transport Python callback receives an already-deserialized protobuf",
            "queue_depth_note": "binding queue depth unavailable; max_active_callbacks and late/drop estimates are proxies",
        })
    return output


def _consistent_mechanism(
    failing_events: Sequence[Mapping[str, Any]],
    aligned: Sequence[Mapping[str, Any]],
) -> dict[str, Any] | None:
    # Precommitted effect-size floors prevent noise from satisfying the rule.
    candidates = {
        "callback_ms_p99": (1.25, 0.25),
        "callback_ms_max": (1.25, 0.50),
        "processing_ms_p99": (1.25, 0.25),
        "copy_ms_p99": (1.25, 0.25),
        "max_active_callbacks": (1.25, 1.0),
        "estimated_dropped_or_coalesced_count": (1.25, 1.0),
        "web_backend_cpu_pct": (1.25, 5.0),
        "web_backend_thread_cpu_max_pct": (1.25, 5.0),
        "gz_sim_cpu_pct": (1.15, 5.0),
        "context_switches_per_s": (1.20, 100.0),
        "web_backend_nonvol_ctxsw_per_s": (1.25, 2.0),
        "web_backend_majflt_per_s": (1.25, 1.0),
        "gc_pause_ms": (1.25, 0.5),
    }
    by_event: dict[str, list[Mapping[str, Any]]] = defaultdict(list)
    for row in aligned:
        by_event[str(row.get("event_id"))].append(row)
    for metric, (ratio_floor, delta_floor) in candidates.items():
        evidence = []
        accepted = True
        for event in failing_events:
            selected = by_event.get(str(event["event_id"]), [])
            baseline = [number(row.get(metric)) for row in selected if -15 <= float(row["offset_from_event_start_s"]) <= -6]
            mechanism = [number(row.get(metric)) for row in selected if -5 <= float(row["offset_from_event_start_s"]) <= float(event["end_elapsed_s"]) - float(event["start_elapsed_s"])]
            baseline_values = [value for value in baseline if value is not None]
            mechanism_values = [value for value in mechanism if value is not None]
            if not baseline_values or not mechanism_values:
                accepted = False
                break
            before = median(baseline_values)
            during = max(mechanism_values)
            ratio = math.inf if before == 0 and during > 0 else (during / before if before else 1.0)
            changed = ratio >= ratio_floor and during - before >= delta_floor
            evidence.append({"event_id": event["event_id"], "before_median": before, "lead_or_event_max": during, "ratio": ratio, "changed": changed})
            if not changed:
                accepted = False
                break
        if accepted and len(evidence) == len(failing_events) and evidence:
            return {"metric": metric, "direction": "increase", "event_evidence": evidence}
    return None


def build(root: Path) -> str:
    runs_root = root / "runs"
    summaries: list[dict[str, Any]] = []
    phase_rows: list[dict[str, Any]] = []
    event_rows: list[dict[str, Any]] = []
    aligned_rows: list[dict[str, Any]] = []

    for config_id in CONFIGS:
        for repeat in range(1, 4):
            run_dir = runs_root / f"{config_id}_run{repeat}"
            if not run_dir.is_dir():
                summaries.append({
                    "run_id": run_dir.name, "config_id": config_id, "repeat": repeat,
                    "requested_duration_s": 600, "trace_elapsed_s": 0,
                    "complete": False, "exit_code": "missing", "rtf_samples": 0,
                    "median_rtf": "", "low_rtf_seconds": 0,
                    "longest_low_rtf_streak_s": 0, "rtf_gate_pass": False,
                    "clean_restart_verified": False, "callback_rows": 0,
                    "trace_path": str(run_dir / "per_second_metrics.csv"),
                    "callback_path": str(run_dir / "callback_phase_raw.csv"),
                })
                continue
            summary, trace, callbacks = summarize(run_dir)
            summaries.append(summary)
            current_phase = _phase_rows(summary["run_id"], config_id, str(repeat), callbacks)
            phase_rows.extend(current_phase)
            run_events = events(trace, summary["run_id"])
            event_rows.extend(run_events)
            callback_by_second: dict[int, list[Mapping[str, Any]]] = defaultdict(list)
            for row in current_phase:
                second = callback_second(row)
                if second is not None:
                    callback_by_second[second].append(row)
            for event in run_events:
                for trace_row in trace:
                    elapsed = number(trace_row.get("elapsed_s"))
                    monotonic_second = trace_second(trace_row)
                    if elapsed is None or monotonic_second is None:
                        continue
                    offset = elapsed - float(event["start_elapsed_s"])
                    event_duration = float(event["end_elapsed_s"]) - float(event["start_elapsed_s"])
                    if not (-15 <= offset <= event_duration + 30):
                        continue
                    matches = callback_by_second.get(monotonic_second) or [{}]
                    for callback in matches:
                        aligned_rows.append({
                            "event_id": event["event_id"], "run_id": summary["run_id"],
                            "config_id": config_id, "repeat": repeat,
                            "offset_from_event_start_s": round(offset, 3),
                            "event_gate_failing": event["gate_failing"],
                            **trace_row, **callback,
                        })

    manifest_fields = list(summaries[0]) if summaries else ["run_id"]
    write_csv(root / "subscription_ablation_manifest.csv", summaries, manifest_fields)
    phase_fields = list(phase_rows[0]) if phase_rows else [
        "run_id", "config_id", "repeat", "monotonic_second", "topic_id",
        "message_count", "payload_bytes", "callback_ms_p50", "callback_ms_p95",
        "callback_ms_p99", "callback_ms_max", "deserialize_phase_observable",
    ]
    write_csv(root / "callback_phase_metrics.csv", phase_rows, phase_fields)
    aligned_fields = list(aligned_rows[0]) if aligned_rows else [
        "event_id", "run_id", "config_id", "repeat", "offset_from_event_start_s",
        "event_gate_failing", "monotonic_ns", "gazebo_rtf", "topic_id",
    ]
    write_csv(root / "event_aligned_subscription_metrics.csv", aligned_rows, aligned_fields)
    write_csv(root / "rtf_events.csv", event_rows, (
        "event_id", "run_id", "start_elapsed_s", "end_elapsed_s",
        "low_second_count", "minimum_rtf", "gate_failing",
    ))

    by_config = defaultdict(list)
    for row in summaries:
        by_config[row["config_id"]].append(row)
    complete_matrix = all(len(by_config[name]) == 3 and all(row["complete"] for row in by_config[name]) for name in CONFIGS)
    pass_counts = {name: sum(bool(row["rtf_gate_pass"]) for row in by_config[name]) for name in CONFIGS}
    fail_counts = {name: sum(bool(row["complete"]) and not bool(row["rtf_gate_pass"]) for row in by_config[name]) for name in CONFIGS}
    controls_pass = pass_counts.get("S0") == 3 and pass_counts.get("S1") == 3
    suspects = [name for name in CONFIGS[2:] if fail_counts.get(name) == 3]
    failing_events = [row for row in event_rows if row["gate_failing"] and any(row["run_id"].startswith(f"{suspect}_run") for suspect in suspects)]
    mechanism = _consistent_mechanism(failing_events, aligned_rows) if suspects and failing_events else None
    accepted = complete_matrix and controls_pass and bool(suspects) and mechanism is not None
    conclusion = "ROOT_CAUSE_CONFIRMED" if accepted else ("INCONCLUSIVE" if complete_matrix else "COLLECTION_INCOMPLETE")
    decision = {
        "schema": "gazebo_subscription_causal_decision/v1",
        "conclusion": conclusion,
        "root_cause_accepted": accepted,
        "production_mitigation_implemented": False,
        "matrix_complete": complete_matrix,
        "controls": {"S0_pass_count": pass_counts.get("S0", 0), "S1_pass_count": pass_counts.get("S1", 0), "required": "3/3 each", "pass": controls_pass},
        "suspect_enabled_fail_3_of_3": suspects,
        "suspect_disabled_configuration": "S1",
        "mechanism_metric": mechanism,
        "rule": {
            "control_passes": "3/3", "suspect_enabled_fails": "3/3",
            "suspect_disabled_passes": "3/3",
            "mechanism_changes_before_or_during_every_event": True,
            "anything_weaker": "INCONCLUSIVE",
        },
        "per_config_pass_counts": pass_counts,
        "per_config_fail_counts": fail_counts,
        "scope_guards": {
            "candidate_b_changed": False, "range_semantics_changed": False,
            "production_defaults_changed": False, "datasets_changed": False,
            "stage_2a_behavior_changed": False, "mitigation_attempted": False,
        },
        "measurement_limits": {
            "binding_deserialization_duration": "unobservable in gz.transport Python API",
            "transport_queue_depth": "unobservable; max active callbacks and late/drop estimates used as proxies",
            "message_age": "unavailable without a validated simulation-to-host clock mapping; never reported as zero",
        },
    }
    (root / "causal_decision.json").write_text(json.dumps(decision, indent=2, sort_keys=True) + "\n", encoding="utf-8")

    completed = sum(bool(row["complete"]) for row in summaries)
    summary = f"""# Eager Gazebo subscription causal isolation

Conclusion: `{conclusion}`.

- Completed clean 600-second repeats: {completed}/36.
- S0 control passes: {pass_counts.get('S0', 0)}/3.
- S1 zero-subscription control passes: {pass_counts.get('S1', 0)}/3.
- Configurations failing 3/3: {', '.join(suspects) if suspects else 'none'}.
- Consistent pre/during-event mechanism metric: {mechanism['metric'] if mechanism else 'none accepted'}.
- Production mitigation: not implemented.
- Candidate B, range semantics, production defaults, datasets, and Stage 2A behavior: unchanged.

The Gazebo Python binding invokes callbacks with protobuf objects already deserialized and does not expose its internal receive queue. Deserialization duration and true queue depth are therefore explicitly unavailable; the captured proxies are callback concurrency, interarrival lateness, and estimated dropped/coalesced messages. Message age is also left unavailable unless a validated simulation-time to host-monotonic mapping exists.
"""
    (root / "final_summary.md").write_text(summary, encoding="utf-8")
    return conclusion


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", type=Path, required=True)
    args = parser.parse_args()
    print(build(args.root.resolve()))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
