"""Ablation C: Gazebo (headless) + camera + MiDaS, no ROS2/dashboard/bridge.

Standalone process. Subscribes directly to the Gazebo camera topic over
gz-transport (the same mechanism `main.py`'s GazeboDashboardBridge uses --
`_image_to_bgr`/`_message_timestamp_s` below are read-only copies of its
static methods, not an import of main.py, to avoid pulling in the FastAPI
app/module-level side effects of the full dashboard), and feeds frames
directly into the real production LatestDepthWorker + MidasSmallAdapter.
No ROS2 telemetry launch, no mavlink_manual_bridge.py, no uvicorn/main.py
dashboard is required or started by this script -- only a Gazebo server
(headless, no GUI) and one PX4 SITL instance (to spawn the camera-bearing
vehicle model) need to already be running, started separately by
core_range_contention_launch_config_c.sh.

See docs/CORE_RANGE_LIVE_STACK_CONTENTION_REPORT.md.
"""

from __future__ import annotations

import argparse
import json
import math
import signal
import sys
import time
from pathlib import Path
from typing import Any

SYSTEM_DIST_PACKAGES = Path("/usr/lib/python3/dist-packages")
if SYSTEM_DIST_PACKAGES.exists() and str(SYSTEM_DIST_PACKAGES) not in sys.path:
    sys.path.append(str(SYSTEM_DIST_PACKAGES))

import cv2
import numpy as np
from gz.msgs10.image_pb2 import Image as GzImage
from gz.transport13 import Node as GzNode

from core_range_live_stack_contention_isolation import (
    CAMERA_HEIGHT, CAMERA_WIDTH, FINE_STAGE_KEYS, PRODUCTION_DEPTH_RATE_HZ,
    _BackgroundDmon, _nvidia_smi_snapshot, _stats, _write_csv, _write_json,
)
from depth_model_adapter import MidasSmallAdapter
from depth_worker import DepthJob, LatestDepthWorker

DRONE_MODELS = {"UAV-01": "x500_custom_0", "UAV-02": "x500_custom_1"}


def _message_timestamp_s(message: Any) -> float | None:
    try:
        stamp = message.header.stamp
        timestamp = float(stamp.sec) + float(stamp.nsec) * 1e-9
        return timestamp if math.isfinite(timestamp) else None
    except (AttributeError, TypeError, ValueError):
        return None


def _image_to_bgr(message: Any) -> np.ndarray:
    width = int(message.width)
    height = int(message.height)
    step = int(message.step)
    pixel_format = int(message.pixel_format_type)
    raw_data = np.frombuffer(message.data, dtype=np.uint8)

    if width <= 0 or height <= 0 or raw_data.size <= 0:
        raise ValueError("empty Gazebo image")

    if pixel_format == 1:
        channels, conversion = 1, cv2.COLOR_GRAY2BGR
    elif pixel_format == 3:
        channels, conversion = 3, cv2.COLOR_RGB2BGR
    elif pixel_format == 4:
        channels, conversion = 4, cv2.COLOR_RGBA2BGR
    elif pixel_format == 5:
        channels, conversion = 4, cv2.COLOR_BGRA2BGR
    elif pixel_format == 8:
        channels, conversion = 3, None
    else:
        raise ValueError(f"unsupported Gazebo pixel format {pixel_format}")

    row_bytes = width * channels
    if step < row_bytes or raw_data.size < step * height:
        raise ValueError(f"invalid Gazebo image stride {step} for {width}x{height}")

    rows = raw_data[: step * height].reshape(height, step)
    image = rows[:, :row_bytes].reshape(height, width, channels)
    image = cv2.cvtColor(image, conversion) if conversion is not None else image.copy()
    return np.ascontiguousarray(image)


def run(output: Path, duration_s: float, model_name: str, rate_hz: float, profile_stages: bool) -> dict[str, Any]:
    import os
    os.environ["SWARM_DEPTH_PROFILE_STAGES"] = "1" if profile_stages else "0"

    adapter = MidasSmallAdapter()
    worker = LatestDepthWorker(adapter)
    worker.start()

    camera_topic = f"/world/default/model/{model_name}/link/camera_link/sensor/camera/image"
    node = GzNode()

    state: dict[str, Any] = {
        "frame_count": 0, "arrival_timestamps": [], "decode_errors": 0,
        "last_submit_monotonic_s": 0.0,
    }
    frame_index_holder = {"value": 0}
    submit_interval_s = 1.0 / rate_hz

    def camera_callback(message: Any) -> None:
        now = time.monotonic()
        state["frame_count"] += 1
        state["arrival_timestamps"].append(now)
        # Mirrors metric_target_fusion.py's SWARM_METRIC_TARGET_DEPTH_RATE_HZ
        # throttle: production submits a depth job at most once per
        # 1/rate_hz, not on every camera frame (camera itself streams much
        # faster than that).
        if now - state["last_submit_monotonic_s"] < submit_interval_s:
            return
        try:
            frame_bgr = _image_to_bgr(message)
        except Exception:
            state["decode_errors"] += 1
            return
        state["last_submit_monotonic_s"] = now
        worker.submit(DepthJob(frame_bgr, now, frame_index_holder["value"]))
        frame_index_holder["value"] += 1

    subscribed = node.subscribe(GzImage, camera_topic, camera_callback)
    if not subscribed:
        raise RuntimeError(f"could not subscribe to {camera_topic}")

    gpu_before = _nvidia_smi_snapshot()
    seen_versions: set[int] = set()
    latency_rows: list[dict[str, Any]] = []
    stop = {"flag": False}

    def _handle_signal(_signum: int, _frame: Any) -> None:
        stop["flag"] = True

    signal.signal(signal.SIGINT, _handle_signal)
    signal.signal(signal.SIGTERM, _handle_signal)

    with _BackgroundDmon(output, "config_c"):
        deadline = time.monotonic() + duration_s
        while time.monotonic() < deadline and not stop["flag"]:
            version, result = worker.latest()
            if result is not None and version not in seen_versions:
                seen_versions.add(version)
                latency_rows.append({
                    "frame_index": result.frame_index,
                    "worker_inference_s": (
                        (result.completed_timestamp_s - result.inference_started_timestamp_s)
                        if result.inference_started_timestamp_s is not None else None
                    ),
                    "queue_wait_s": (
                        (result.inference_started_timestamp_s - result.submitted_timestamp_s)
                        if result.inference_started_timestamp_s is not None and result.submitted_timestamp_s is not None else None
                    ),
                    "valid": result.valid,
                    "reason": result.reason,
                    **{key: (result.profiling_stages or {}).get(key) for key in FINE_STAGE_KEYS},
                })
            time.sleep(0.03)
    gpu_after = _nvidia_smi_snapshot()

    status = worker.status()
    worker.stop()

    arrivals = state["arrival_timestamps"]
    intervals = [b - a for a, b in zip(arrivals, arrivals[1:])]
    frame_arrival_rate_hz = (len(arrivals) - 1) / (arrivals[-1] - arrivals[0]) if len(arrivals) > 1 and arrivals[-1] > arrivals[0] else 0.0

    _write_csv(output / f"config_c_stage_latency{'_profiled' if profile_stages else ''}.csv", latency_rows)
    durations = [row["worker_inference_s"] for row in latency_rows if row["worker_inference_s"] is not None]
    result = {
        "config": "C_gazebo_headless_camera_only",
        "profile_stages": profile_stages,
        "duration_s": duration_s,
        "model_name": model_name,
        "camera_topic": camera_topic,
        "frame_count_received": state["frame_count"],
        "decode_errors": state["decode_errors"],
        "frame_arrival_rate_hz": frame_arrival_rate_hz,
        "submitted": status["submitted"],
        "dropped": status["dropped"],
        "processed": status["processed"],
        "failed": status["failed"],
        "worker_inference_s": _stats(durations),
        "gpu_before": gpu_before,
        "gpu_after": gpu_after,
    }
    _write_json(output / f"config_c_result{'_profiled' if profile_stages else ''}.json", result)
    print(json.dumps({
        "frame_count": state["frame_count"], "processed": status["processed"], "dropped": status["dropped"],
        "median_s": result["worker_inference_s"]["median_s"], "frame_arrival_rate_hz": frame_arrival_rate_hz,
    }, indent=2))
    return result


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--duration-s", type=float, default=20.0)
    parser.add_argument("--model-name", type=str, default=DRONE_MODELS["UAV-01"])
    parser.add_argument("--rate-hz", type=float, default=PRODUCTION_DEPTH_RATE_HZ)
    parser.add_argument("--profile-stages", action="store_true")
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    run(args.output, args.duration_s, args.model_name, args.rate_hz, args.profile_stages)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
