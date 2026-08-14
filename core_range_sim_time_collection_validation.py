"""Accuracy/runtime validation for the deterministic sim-time collection
driver (CORE_RANGE_DETERMINISTIC_SIM_TIME_COLLECTION_AND_GT_VALIDATION).

Read-only analysis over a scenario root already produced by
core_range_collect_dynamic_5hz_headless_scenario.sh <...> sim_time_plugin.
Does not run Gazebo/PX4 itself and does not train or fit anything -- it only
measures. Reuses the same integrity/runtime-metric helpers already used by
the accepted 5/8 headless corpus (core_range_logging_eval.py,
core_range_collect_dynamic_5hz_headless_batch.py) plus the RTF-window MAE
comparison from core_range_gazebo_rtf_stutter_analysis.py, adapted to the
per-session gz_world_stats.log sampler format instead of the long-run
resource-trace CSV.
"""

from __future__ import annotations

import json
import math
import re
from pathlib import Path
from statistics import mean, median
from typing import Any

from core_range_collect_dynamic_5hz_headless_batch import (
    _camera_source_median_fps,
    _gazebo_rtf_quality,
    percentile,
)
from core_range_gazebo_rtf_stutter_analysis import (
    STABLE_RTF_MIN,
    STUTTER_RTF_MAX,
    _stats,
)
from core_range_logging_eval import evaluate, load_jsonl, verify_record
from range_physical_diagnostics import validate_timestamp_stages

GATES = {
    "raw_ranges_minimum_40": 40,
    "camera_fps_minimum": 25.0,
    "tracking_fps_minimum": 20.0,
    "capture_consume_median_s_maximum": 0.200,
    "capture_consume_p95_s_maximum": 0.300,
    "midas_worker_p95_s_maximum": 0.100,
    "raw_error_low_rtf_vs_high_rtf_max_increase_fraction": 0.20,
    "pose_determinism_max_abs_diff_m": 0.01,
}


def _parse_gz_world_stats_trace(path: Path) -> list[dict[str, Any]]:
    """Parse the per-second `--- <epoch> ---` + protobuf-text sampler log
    written by core_range_collect_dynamic_5hz_headless_scenario.sh's
    start_rtf_sampler into [{"wall_clock_s": float, "gazebo_rtf": float}]."""
    if not path.exists():
        return []
    text = path.read_text(encoding="utf-8", errors="replace")
    chunks = re.split(r"--- ([\d.]+) ---", text)
    rows: list[dict[str, Any]] = []
    # re.split with a capturing group yields [pre, ts1, chunk1, ts2, chunk2, ...]
    for index in range(1, len(chunks), 2):
        timestamp = chunks[index]
        body = chunks[index + 1] if index + 1 < len(chunks) else ""
        match = re.search(r"real_time_factor:\s*([\d.eE+-]+)", body)
        if match is None:
            continue
        try:
            rows.append({"wall_clock_s": float(timestamp), "gazebo_rtf": float(match.group(1))})
        except ValueError:
            continue
    return rows


def _monotonic_to_wall_offset(capture_result: dict[str, Any], raw_rows: list[dict[str, Any]]) -> float | None:
    """Best-effort offset between the sidecar's time.monotonic() axis and
    the RTF sampler's time.time() axis, anchored on the gimbal command's
    server_timestamp_ms (wall clock) versus the first raw row's frame
    receipt monotonic timestamp captured moments later in the same run."""
    gimbal = capture_result.get("gimbal") or {}
    server_ms = gimbal.get("server_timestamp_ms")
    if not raw_rows or not isinstance(server_ms, (int, float)):
        return None
    try:
        first_monotonic = float(raw_rows[0]["timestamp_stages"]["frame_receipt"]["timestamp_s"])
    except (KeyError, TypeError, ValueError):
        return None
    # Anchor is approximate (gimbal command precedes the first raw frame by
    # seconds, not the same instant) -- acceptable given _nearest_rtf's
    # multi-second matching tolerance and the goal of a coarse stable/
    # stutter label, not sub-second alignment.
    return (server_ms / 1000.0) - first_monotonic


def _nearest_rtf(trace_rows: list[dict[str, Any]], target_wall_clock_s: float, max_gap_s: float = 3.0) -> float | None:
    best = None
    best_gap = max_gap_s
    for row in trace_rows:
        gap = abs(row["wall_clock_s"] - target_wall_clock_s)
        if gap < best_gap:
            best_gap = gap
            best = row["gazebo_rtf"]
    return best


def _rtf_window_mae(root: Path, raw_rows: list[dict[str, Any]], capture_result: dict[str, Any]) -> dict[str, Any]:
    trace_rows = _parse_gz_world_stats_trace(root / "gz_world_stats.log")
    offset = _monotonic_to_wall_offset(capture_result, raw_rows)
    labeled: dict[str, list[float]] = {"stable": [], "stutter": []}
    if offset is not None and trace_rows:
        for row in raw_rows:
            try:
                wall = float(row["timestamp_stages"]["frame_receipt"]["timestamp_s"]) + offset
                gt = float(row["ground_truth"]["distance_m"])
                raw = float(row["raw_range"]["physics_slant_range_m"])
            except (KeyError, TypeError, ValueError):
                continue
            if not (math.isfinite(gt) and math.isfinite(raw)):
                continue
            rtf = _nearest_rtf(trace_rows, wall)
            if rtf is None:
                continue
            if rtf >= STABLE_RTF_MIN:
                labeled["stable"].append(abs(raw - gt))
            elif rtf <= STUTTER_RTF_MAX:
                labeled["stutter"].append(abs(raw - gt))
    stable_stats = _stats(labeled["stable"])
    stutter_stats = _stats(labeled["stutter"])
    increase_fraction = None
    gate_pass = None
    if stable_stats.get("mean") and stutter_stats.get("mean") is not None and labeled["stable"] and labeled["stutter"]:
        increase_fraction = (stutter_stats["mean"] - stable_stats["mean"]) / stable_stats["mean"]
        gate_pass = increase_fraction <= GATES["raw_error_low_rtf_vs_high_rtf_max_increase_fraction"]
    return {
        "trace_sample_count": len(trace_rows),
        "monotonic_to_wall_offset_used": offset,
        "stable_rtf_ge_0_8": {"row_count": len(labeled["stable"]), "abs_error_m": stable_stats},
        "stutter_rtf_le_0_3": {"row_count": len(labeled["stutter"]), "abs_error_m": stutter_stats},
        "increase_fraction": increase_fraction,
        "gate_pass": gate_pass,
        "insufficient_evidence": not labeled["stable"] or not labeled["stutter"],
    }


def _trajectory_kinematics(root: Path) -> dict[str, Any]:
    path = root / "trajectory_events.jsonl"
    if not path.exists():
        return {"event_count": 0}
    steps = []
    for line in path.read_text(encoding="utf-8").splitlines():
        if not line.strip():
            continue
        row = json.loads(line)
        if row.get("kind") in ("trajectory_step", "trajectory_activated"):
            steps.append(row)
    steps.sort(key=lambda r: r.get("sim_timestamp_s", 0.0))
    velocities = [row.get("x_velocity_m_s") for row in steps if isinstance(row.get("x_velocity_m_s"), (int, float))]
    ranges = [row.get("range_m") for row in steps if isinstance(row.get("range_m"), (int, float))]
    return {
        "event_count": len(steps),
        "sim_timestamp_span_s": (
            [steps[0]["sim_timestamp_s"], steps[-1]["sim_timestamp_s"]] if steps else None
        ),
        "range_start_m": ranges[0] if ranges else None,
        "range_end_m": ranges[-1] if ranges else None,
        "x_velocity_m_s": _stats(velocities) if velocities else None,
        "clock_domain_all_gazebo_sim_time": all(row.get("clock_domain") == "gazebo_sim_time" for row in steps),
    }


def measure(root: Path, spec: dict[str, Any]) -> dict[str, Any]:
    root = root.resolve()
    capture_rows, _ = load_jsonl(root / "capture_events.jsonl")
    if len(capture_rows) != 1:
        raise ValueError("exactly_one_capture_event_required")
    capture = capture_rows[0]
    session_id = int(capture["session_id"])

    integrity = evaluate(root, minimum_frames=40, require_single_raw_session=True)

    all_rows: list[dict[str, Any]] = []
    for path in sorted(root.glob("physical_diagnostics*.jsonl")):
        rows, _ = load_jsonl(path)
        all_rows.extend(rows)
    raw = [
        row for row in all_rows
        if int(row.get("session_id", -1)) == session_id and row.get("stage") == "raw_range_computed"
    ]

    submit, capture_consume, worker, queue = [], [], [], []
    tracking_interval_fps = []
    previous_frame = previous_time = None
    for row in raw:
        stages = row.get("timestamp_stages") or {}
        try:
            submit.append(float(stages["depth_submit"]["timestamp_s"]))
            capture_consume.append(float(stages["consume"]["timestamp_s"]) - float(stages["frame_receipt"]["timestamp_s"]))
            worker.append(float(stages["depth_complete"]["timestamp_s"]) - float(stages["depth_worker_start"]["timestamp_s"]))
            queue.append(float(stages["depth_worker_start"]["timestamp_s"]) - float(stages["depth_submit"]["timestamp_s"]))
        except (KeyError, TypeError, ValueError):
            continue
        current_time = row.get("measurement_timestamp_s")
        current_frame = row.get("frame_index")
        if (
            previous_time is not None
            and isinstance(current_time, (int, float))
            and current_time > previous_time
        ):
            tracking_interval_fps.append((current_frame - previous_frame) / (current_time - previous_time))
        previous_frame, previous_time = current_frame, current_time

    camera_fps = _camera_source_median_fps(root)
    gazebo_rtf_quality = _gazebo_rtf_quality(root / "gz_world_stats.log")
    tracking_fps_median = median(tracking_interval_fps) if tracking_interval_fps else None
    capture_consume_median = median(capture_consume) if capture_consume else None
    capture_consume_p95 = percentile(capture_consume, 95) if capture_consume else None
    worker_p95 = percentile(worker, 95) if worker else None

    rtf_window = _rtf_window_mae(root, raw, capture)
    trajectory = _trajectory_kinematics(root)
    trajectory_result = capture.get("trajectory") or {}

    gates = {
        "integrity_all_pass": all(integrity["gates"].values()),
        "raw_ranges_minimum_40": len(raw) >= GATES["raw_ranges_minimum_40"],
        "camera_fps_minimum_25": camera_fps is not None and camera_fps >= GATES["camera_fps_minimum"],
        "tracking_fps_minimum_20": tracking_fps_median is not None and tracking_fps_median >= GATES["tracking_fps_minimum"],
        "capture_consume_median_le_200ms": (
            capture_consume_median is not None and capture_consume_median <= GATES["capture_consume_median_s_maximum"]
        ),
        "capture_consume_p95_le_300ms": (
            capture_consume_p95 is not None and capture_consume_p95 <= GATES["capture_consume_p95_s_maximum"]
        ),
        "midas_worker_p95_le_100ms": worker_p95 is not None and worker_p95 <= GATES["midas_worker_p95_s_maximum"],
        "no_sim_time_driver_timeout": (
            trajectory_result.get("driver") == "gazebo_preupdate_sim_time_plugin"
            and int(trajectory_result.get("synchronous_set_pose_command_count", -1)) == 0
            and trajectory["event_count"] > 0
        ),
        "raw_error_rtf_window_within_20_percent": bool(rtf_window["gate_pass"]),
    }

    return {
        "group_id": spec["session_id"],
        "scenario_type": spec["scenario_type"],
        "root": str(root),
        "session_id": session_id,
        "raw_range_row_count": len(raw),
        "camera_source_median_fps": camera_fps,
        "gazebo_rtf": gazebo_rtf_quality,
        "tracking_median_fps": tracking_fps_median,
        "capture_consume_median_s": capture_consume_median,
        "capture_consume_p95_s": capture_consume_p95,
        "midas_worker_p95_s": worker_p95,
        "rtf_window_raw_error_mae": rtf_window,
        "trajectory_kinematics": trajectory,
        "trajectory_result": trajectory_result,
        "integrity": {"conclusion": integrity["conclusion"], "gates": integrity["gates"], "counts": integrity["counts"]},
        "gates": gates,
        "all_gates_pass": all(gates.values()),
    }


def pose_determinism(root_a: Path, root_b: Path) -> dict[str, Any]:
    """Compare target pose at matching *elapsed* trajectory time (t_sim
    minus that run's own trajectory_activated timestamp) between two runs
    of the identical scenario spec. Absolute sim_timestamp_s is NOT
    comparable across runs: each run's Gazebo world starts sim_time at 0,
    but prewarm/bbox-selection wall-clock timing varies run to run, so the
    trajectory activates at a different absolute sim_timestamp_s each time
    (observed: 19.62s vs 21.276s start for two otherwise-identical
    approaching runs, both spanning exactly 31.8s / 160 steps). The
    trajectory itself is a pure function of *elapsed* time since
    activation (sim_time_trajectory.py), so matching-elapsed-time poses
    from two independent runs should agree to floating-point precision
    regardless of any RTF variation during either run."""

    def _steps(root: Path) -> list[dict[str, Any]]:
        path = root / "trajectory_events.jsonl"
        rows = [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines() if line.strip()]
        rows = [row for row in rows if row.get("kind") in ("trajectory_activated", "trajectory_step")]
        rows.sort(key=lambda r: r["sim_timestamp_s"])
        activated = next((r for r in rows if r["kind"] == "trajectory_activated"), rows[0])
        origin = activated["sim_timestamp_s"]
        steps = [r for r in rows if r["kind"] == "trajectory_step"]
        for row in steps:
            row["elapsed_s"] = row["sim_timestamp_s"] - origin
        return steps

    steps_a = _steps(root_a)
    steps_b = _steps(root_b)
    diffs = []
    b_index = 0
    for row_a in steps_a:
        t_a = row_a["elapsed_s"]
        best_gap = math.inf
        best_row = None
        while b_index < len(steps_b) and steps_b[b_index]["elapsed_s"] < t_a - 0.5:
            b_index += 1
        for row_b in steps_b[b_index:]:
            gap = abs(row_b["elapsed_s"] - t_a)
            if gap < best_gap:
                best_gap = gap
                best_row = row_b
            elif row_b["elapsed_s"] > t_a + 0.5:
                break
        if best_row is not None and best_gap <= 0.05:
            diffs.append(abs(best_row["x_m"] - row_a["x_m"]))
    return {
        "run_a_event_count": len(steps_a),
        "run_b_event_count": len(steps_b),
        "matched_pairs": len(diffs),
        "max_abs_diff_m": max(diffs) if diffs else None,
        "mean_abs_diff_m": mean(diffs) if diffs else None,
        "gate_pass": bool(diffs) and max(diffs) <= GATES["pose_determinism_max_abs_diff_m"],
    }
