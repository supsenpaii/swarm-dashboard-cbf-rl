"""Observation-only paired capture for memory/no-write collection ablations."""

from __future__ import annotations

import argparse
import asyncio
import json
from pathlib import Path
import statistics
import threading
import time

from core_range_dynamic_capture import run_trajectory
from range_v2_r3_6a_capture import _request, assert_observation_only, set_gimbal_pitch, status, wait_for_observation_only_ready


def metric_status(value: dict) -> dict:
    return ((value.get("tracking") or {}).get("metric_target_fusion") or {})


def wait_calibration(timeout_s: float) -> None:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        value = status()
        assert_observation_only(value)
        calibration = metric_status(value).get("calibration") or {}
        if calibration.get("stable") is True:
            return
        time.sleep(0.25)
    raise RuntimeError("memory_capture_calibration_not_stable")


def capture(args: argparse.Namespace) -> dict:
    before = wait_for_observation_only_ready()
    _request("POST", "/api/tracking/start", {"drone_id": "UAV-01", "desired_distance_m": args.start_range_m})
    asyncio.run(set_gimbal_pitch(args.pitch_deg))
    wait_calibration(90.0)
    selected = _request("POST", "/api/tracking/bbox", {
        "x": args.bbox_x, "y": args.bbox_y, "width": args.bbox_width, "height": args.bbox_height,
    })
    samples: list[dict] = []
    stop_event = threading.Event()

    def monitor() -> None:
        while not stop_event.wait(0.20):
            try:
                value = status()
                assert_observation_only(value)
                tracking = value.get("tracking") or {}
                fusion = tracking.get("metric_target_fusion") or {}
                samples.append({
                    "monotonic_s": time.monotonic(), "tracking_fps": tracking.get("tracking_fps"),
                    "camera_fps": tracking.get("source_fps"), "tracking_ms": tracking.get("tracking_ms"),
                    "controller_ms": tracking.get("controller_ms"), "main_loop_ms": tracking.get("main_loop_ms"),
                    "timing": tracking.get("timing") or {},
                    "source_frames_dropped": tracking.get("source_frames_dropped"),
                    "dataset": fusion.get("range_residual_dataset") or {},
                    "diagnostics": fusion.get("physical_range_diagnostics") or {},
                    "worker": fusion.get("worker") or {},
                })
            except Exception:
                pass

    thread = threading.Thread(target=monitor, name="paired-status-monitor", daemon=True)
    thread.start()
    try:
        trajectory = run_trajectory(args, args.root / "trajectory_events.jsonl")
        time.sleep(2.0)
    finally:
        stop_event.set()
        thread.join(timeout=2.0)
    final = status()
    assert_observation_only(final)
    stopped = _request("POST", "/api/tracking/stop")
    active = [row for row in samples if isinstance(row.get("tracking_fps"), (int, float)) and row["tracking_fps"] > 0]
    dataset_counts = [int((row.get("dataset") or {}).get("record_count", 0)) for row in samples]
    diagnostics_counts = [int((row.get("diagnostics") or {}).get("record_count", 0)) for row in samples]
    payload = {
        "scenario_id": args.scenario_id, "dataset_role": "paired_memory_ablation",
        "selected_tracking_state": (selected.get("tracking") or {}).get("state"),
        "stopped_tracking_state": (stopped.get("tracking") or {}).get("state"),
        "duration_s": args.movement_duration_s + args.hold_duration_s,
        "trajectory": trajectory, "status_sample_count": len(active),
        "tracking_fps_median": statistics.median(row["tracking_fps"] for row in active),
        "camera_fps_median": statistics.median(row["camera_fps"] for row in active if isinstance(row.get("camera_fps"), (int, float))),
        "raw_record_count": max(dataset_counts, default=0) - min(dataset_counts, default=0),
        "diagnostic_record_count": max(diagnostics_counts, default=0) - min(diagnostics_counts, default=0),
        "maximum_writer_queue_depth": max((int((row.get("diagnostics") or {}).get("memory_buffer_depth", 0)) for row in samples), default=0),
        "dropped_logging_records": max((int((row.get("diagnostics") or {}).get("dropped_record_count", 0)) for row in samples), default=0),
        "samples": samples,
        "model_output_used_for_motion": False, "follow_endpoint_called": False,
    }
    args.root.mkdir(parents=True, exist_ok=True)
    (args.root / "memory_capture_result.json").write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n")
    return payload


def parser() -> argparse.ArgumentParser:
    value = argparse.ArgumentParser()
    value.add_argument("--root", required=True, type=Path); value.add_argument("--scenario-id", required=True)
    for name in ("start_range_m", "end_range_m", "lateral_m", "target_z_m", "yaw_start_deg", "yaw_end_deg", "movement_duration_s", "hold_duration_s", "pose_update_rate_hz", "pitch_deg", "bbox_x", "bbox_y", "bbox_width", "bbox_height"):
        value.add_argument("--" + name.replace("_", "-"), required=True, type=float)
    return value


if __name__ == "__main__":
    print(json.dumps(capture(parser().parse_args()), indent=2, sort_keys=True))
