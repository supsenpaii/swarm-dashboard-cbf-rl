"""Analysis tooling for CORE_RANGE_GAZEBO_RTF_STUTTER_AND_GATE_VALIDATION
sections 2 and 3: transition detection + resource correlation from
long_run_resource_trace.csv, and stable-vs-stutter window comparison using
real raw_range_computed rows (sim-time and wall-clock normalized) plus the
camera trace.

Read-only with respect to captured data.
"""

from __future__ import annotations

import csv
import json
import math
from pathlib import Path
from statistics import median, pstdev
from typing import Any

import numpy as np

from core_range_camera_source_fps_audit import _interval_metrics, _load_jsonl
from core_range_logging_eval import load_jsonl, verify_record
from range_physical_diagnostics import validate_timestamp_stages

STABLE_RTF_MIN = 0.8
STUTTER_RTF_MAX = 0.3
MIN_RUN_LENGTH_S = 3


def _read_trace_csv(path: Path) -> list[dict[str, Any]]:
    with path.open() as stream:
        rows = list(csv.DictReader(stream))
    for row in rows:
        for key, value in list(row.items()):
            if value in ("", None):
                row[key] = None
                continue
            try:
                row[key] = float(value)
            except ValueError:
                pass
    return rows


def _percentile(values: list[float], p: float) -> float | None:
    return None if not values else float(np.percentile(np.asarray(values, dtype=np.float64), p))


def _stats(values: list[float]) -> dict[str, Any]:
    values = [v for v in values if v is not None and math.isfinite(v)]
    if not values:
        return {"n": 0, "median": None, "mean": None, "p5": None, "p95": None, "stdev": None}
    return {
        "n": len(values), "median": round(median(values), 4), "mean": round(sum(values) / len(values), 4),
        "p5": round(_percentile(values, 5), 4), "p95": round(_percentile(values, 95), 4),
        "stdev": round(pstdev(values), 4) if len(values) > 1 else 0.0,
    }


# ---------------------------------------------------------------------------
# transition detection + resource correlation
# ---------------------------------------------------------------------------

def detect_transitions(trace_rows: list[dict[str, Any]]) -> list[dict[str, Any]]:
    """A transition is a run of >=MIN_RUN_LENGTH_S consecutive samples above
    STABLE_RTF_MIN immediately followed by a run of >=MIN_RUN_LENGTH_S
    consecutive samples below STUTTER_RTF_MAX (a stable->stutter onset), or
    the reverse (a stutter->stable recovery). Runs of intermediate/missing
    RTF values break a streak without themselves being labeled."""
    labels = []
    for row in trace_rows:
        rtf = row.get("gazebo_rtf")
        if rtf is None:
            labels.append(None)
        elif rtf >= STABLE_RTF_MIN:
            labels.append("stable")
        elif rtf <= STUTTER_RTF_MAX:
            labels.append("stutter")
        else:
            labels.append("mid")

    runs: list[tuple[str, int, int]] = []
    index = 0
    while index < len(labels):
        label = labels[index]
        if label is None:
            index += 1
            continue
        start = index
        while index < len(labels) and labels[index] == label:
            index += 1
        runs.append((label, start, index - 1))

    qualifying = [r for r in runs if r[0] in ("stable", "stutter") and (r[2] - r[1] + 1) >= MIN_RUN_LENGTH_S]

    transitions = []
    for previous, current in zip(qualifying, qualifying[1:]):
        if previous[0] == current[0]:
            continue
        transition_elapsed_s = trace_rows[current[1]]["elapsed_s"]
        window_s = 15
        before = trace_rows[max(previous[1], previous[2] - window_s + 1):previous[2] + 1]
        after = trace_rows[current[1]:current[1] + window_s]
        def _win_stats(rows: list[dict[str, Any]], key: str) -> dict[str, Any]:
            return _stats([r[key] for r in rows if r.get(key) is not None])
        metrics = ["gazebo_rtf", "cpu_freq_mean_mhz", "thermal_zone_max_c", "mem_available_mb",
                   "disk_write_kb_per_s", "context_switches_per_s", "gpu_util_pct", "gpu_clock_sm_mhz"]
        transitions.append({
            "kind": f"{previous[0]}_to_{current[0]}",
            "transition_elapsed_s": transition_elapsed_s,
            "before_run_length_s": previous[2] - previous[1] + 1,
            "after_run_length_s": current[2] - current[1] + 1,
            "before_window": {m: _win_stats(before, m) for m in metrics},
            "after_window": {m: _win_stats(after, m) for m in metrics},
        })
    return transitions


def resource_correlation(trace_rows: list[dict[str, Any]]) -> dict[str, Any]:
    """Pearson correlation of each resource metric against gazebo_rtf across
    the whole trace (not just at transitions) -- a complementary, coarser
    signal to the windowed transition comparison above."""
    metrics = ["cpu_freq_mean_mhz", "thermal_zone_max_c", "mem_available_mb",
               "disk_write_kb_per_s", "context_switches_per_s", "gpu_util_pct",
               "gpu_clock_sm_mhz", "process_thread_count", "elapsed_s"]
    rtf = np.array([r["gazebo_rtf"] for r in trace_rows if r.get("gazebo_rtf") is not None])
    result = {}
    for metric in metrics:
        paired = [(r["gazebo_rtf"], r[metric]) for r in trace_rows if r.get("gazebo_rtf") is not None and r.get(metric) is not None]
        if len(paired) < 10:
            result[metric] = None
            continue
        a = np.array([p[0] for p in paired])
        b = np.array([p[1] for p in paired])
        if np.std(a) < 1e-9 or np.std(b) < 1e-9:
            result[metric] = 0.0
        else:
            result[metric] = round(float(np.corrcoef(a, b)[0, 1]), 4)
    return result


# ---------------------------------------------------------------------------
# section 2: stable vs stutter window comparison using real raw_range rows
# ---------------------------------------------------------------------------

def _load_raw_rows(sidecar_path: Path) -> list[dict[str, Any]]:
    rows, _ = load_jsonl(sidecar_path)
    return [r for r in rows if r.get("stage") == "raw_range_computed"]


def _monotonic_to_wall_offset() -> float:
    """time.time() - time.monotonic(), sampled once "now". Stable across a
    session absent an NTP step correction; used to convert the diagnostic
    sidecar's python_monotonic timestamps onto the same axis as this
    monitor trace's wall_clock_s (itself time.time()-based), since Linux
    CLOCK_MONOTONIC is a system-wide clock comparable across processes but
    NOT directly comparable to CLOCK_REALTIME (time.time()) without this
    offset -- the two clocks have different epochs."""
    import time as _time
    return _time.time() - _time.monotonic()


def _nearest_rtf(trace_rows: list[dict[str, Any]], target_wall_clock_s: float, max_gap_s: float = 3.0) -> float | None:
    best = None
    best_gap = max_gap_s
    for row in trace_rows:
        if row.get("gazebo_rtf") is None:
            continue
        gap = abs(row["wall_clock_s"] - target_wall_clock_s)
        if gap < best_gap:
            best_gap = gap
            best = row["gazebo_rtf"]
    return best


def rtf_context_comparison(
    trace_rows: list[dict[str, Any]], sidecar_path: Path, camera_trace_path: Path,
    monotonic_to_wall_offset: float | None = None,
) -> dict[str, Any]:
    """Labels every real raw_range_computed row (and every camera_source_
    receipt event) by the RTF value nearest its own capture wall-clock
    moment (converted from python_monotonic via monotonic_to_wall_offset),
    then aggregates stats per label (stable RTF>=0.8 vs stutter RTF<=0.3).
    Chosen over a single contiguous stable/stutter window because the real
    RTF trace is tightly interleaved (switches every few seconds, not in
    long uniform blocks -- see transition_events.json), so a per-row
    label join uses far more of the available real data than picking one
    window each would."""
    offset = monotonic_to_wall_offset if monotonic_to_wall_offset is not None else _monotonic_to_wall_offset()
    raw_rows = _load_raw_rows(sidecar_path)
    camera_events = [r for r in _load_jsonl(camera_trace_path) if r.get("event") == "camera_source_receipt"]

    def _row_wall_clock(row: dict[str, Any]) -> float | None:
        try:
            return float(row["timestamp_stages"]["frame_receipt"]["timestamp_s"]) + offset
        except (KeyError, TypeError):
            return None

    labeled_rows: dict[str, list[dict[str, Any]]] = {"stable": [], "stutter": []}
    for row in raw_rows:
        wall = _row_wall_clock(row)
        if wall is None:
            continue
        rtf = _nearest_rtf(trace_rows, wall)
        if rtf is None:
            continue
        if rtf >= STABLE_RTF_MIN:
            labeled_rows["stable"].append(row)
        elif rtf <= STUTTER_RTF_MAX:
            labeled_rows["stutter"].append(row)

    labeled_camera: dict[str, list[dict[str, Any]]] = {"stable": [], "stutter": []}
    for event in camera_events:
        wall = event.get("monotonic_receipt_s")
        if wall is None:
            continue
        wall = float(wall) + offset
        rtf = _nearest_rtf(trace_rows, wall)
        if rtf is None:
            continue
        if rtf >= STABLE_RTF_MIN:
            labeled_camera["stable"].append(event)
        elif rtf <= STUTTER_RTF_MAX:
            labeled_camera["stutter"].append(event)

    def _label_report(label: str) -> dict[str, Any]:
        selected = sorted(labeled_rows[label], key=_row_wall_clock)
        gt_errors, delta_times, range_rates, capture_consume = [], [], [], []
        checksum_ok = timestamp_ok = anchor_ok = finite_ok = 0
        previous_raw = previous_wall = None
        for row in selected:
            record_ok, trace_ok = verify_record(row)
            checksum_ok += int(record_ok and trace_ok)
            t_ok, _ = validate_timestamp_stages(row.get("timestamp_stages") or {})
            timestamp_ok += int(t_ok)
            anchor_ok += int(len((row.get("anchors") or {}).get("per_grid_point") or []) == 96)
            try:
                gt = float(row["ground_truth"]["distance_m"])
                raw = float(row["raw_range"]["physics_slant_range_m"])
                finite = math.isfinite(gt) and math.isfinite(raw)
            except (KeyError, TypeError, ValueError):
                gt = raw = None
                finite = False
            finite_ok += int(finite)
            if finite:
                gt_errors.append(abs(raw - gt))
            stages = row.get("timestamp_stages") or {}
            try:
                capture_consume.append(float(stages["consume"]["timestamp_s"]) - float(stages["frame_receipt"]["timestamp_s"]))
            except (KeyError, TypeError):
                pass
            wall = _row_wall_clock(row)
            if previous_wall is not None and wall is not None and wall > previous_wall and raw is not None and previous_raw is not None:
                dt = wall - previous_wall
                delta_times.append(dt)
                range_rates.append((raw - previous_raw) / dt)
            if raw is not None:
                previous_raw = raw
            if wall is not None:
                previous_wall = wall

        n = len(selected)
        return {
            "label": label, "raw_range_row_count": n,
            "raw_range_vs_gt_abs_error_m": _stats(gt_errors),
            "delta_time_s_between_consecutive_accepted_frames": _stats(delta_times),
            "causal_range_rate_m_per_s": _stats(range_rates),
            "capture_consume_s": _stats(capture_consume),
            "checksum_pass_fraction": round(checksum_ok / n, 4) if n else None,
            "timestamp_pass_fraction": round(timestamp_ok / n, 4) if n else None,
            "anchor_pass_fraction": round(anchor_ok / n, 4) if n else None,
            "finite_range_fraction": round(finite_ok / n, 4) if n else None,
            "camera_frame_count_same_label": len(labeled_camera[label]),
        }

    return {"stable": _label_report("stable"), "stutter": _label_report("stutter"),
            "monotonic_to_wall_offset_used": offset}


def main() -> None:
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument("--long-run-root", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()

    root = args.long_run_root.resolve()
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)

    trace_rows = _read_trace_csv(root / "long_run_resource_trace.csv")
    transitions = detect_transitions(trace_rows)
    correlation = resource_correlation(trace_rows)
    (output / "transition_events.json").write_text(json.dumps({
        "transitions": transitions, "whole_run_correlation_with_rtf": correlation,
        "rtf_samples": len([r for r in trace_rows if r.get("gazebo_rtf") is not None]),
        "stable_threshold": STABLE_RTF_MIN, "stutter_threshold": STUTTER_RTF_MAX,
    }, indent=2, sort_keys=True) + "\n", encoding="utf-8")

    comparison = rtf_context_comparison(trace_rows, root / "shared" / "physical_diagnostics.jsonl", root / "camera_source_trace.jsonl")
    rows = [comparison["stable"], comparison["stutter"]]
    keys: list[str] = []
    for row in rows:
        for key in row:
            if key not in keys:
                keys.append(key)
    with (output / "rtf_window_comparison.csv").open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=keys)
        writer.writeheader()
        writer.writerows(rows)
    print(json.dumps(comparison, indent=2, default=str))


if __name__ == "__main__":
    main()
