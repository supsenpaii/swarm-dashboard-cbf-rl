#!/usr/bin/env python3
"""Read-only Phase 0B-A runtime evidence collector.

The collector observes Gazebo Transport topics and the dashboard GET endpoint.
It never advertises or publishes a Gazebo topic and never sends MAVLink or HTTP
control requests. Image callbacks retain timestamps and dimensions only; image
payload bytes are deliberately not copied.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import platform
import statistics
import threading
import time
import urllib.error
import urllib.request
from collections import deque
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Callable, Iterable


DRONE_MODELS = {
    "UAV-01": os.environ.get("SWARM_GAZEBO_MODEL_UAV_01", "x500_custom_0"),
    "UAV-02": os.environ.get("SWARM_GAZEBO_MODEL_UAV_02", "x500_custom_1"),
}
EXPECTED_CAMERA_WIDTH = int(os.environ.get("SWARM_CAMERA_WIDTH", "640"))
EXPECTED_CAMERA_HEIGHT = int(os.environ.get("SWARM_CAMERA_HEIGHT", "360"))
EXPECTED_CAMERA_HORIZONTAL_FOV_RAD = float(
    os.environ.get("SWARM_CAMERA_HORIZONTAL_FOV_RAD", "2.0")
)
EXPECTED_CAMERA_RATE_HZ = float(
    os.environ.get("SWARM_GAZEBO_CAMERA_EXPECTED_RATE_HZ", "30")
)
EXPECTED_CAMERA_IMU_RATE_HZ = float(
    os.environ.get("SWARM_GAZEBO_CAMERA_IMU_EXPECTED_RATE_HZ", "250")
)
DEFAULT_API_URL = "http://127.0.0.1:8000/api/drones"
DEFAULT_DURATION_S = 12.0
DEFAULT_API_INTERVAL_S = 0.5
DEFAULT_MAX_METADATA_SAMPLES = 20_000


def percentile(values: Iterable[float], fraction: float) -> float | None:
    ordered = sorted(float(value) for value in values)
    if not ordered:
        return None
    if len(ordered) == 1:
        return ordered[0]
    position = max(0.0, min(1.0, fraction)) * (len(ordered) - 1)
    lower = math.floor(position)
    upper = math.ceil(position)
    if lower == upper:
        return ordered[lower]
    weight = position - lower
    return ordered[lower] * (1.0 - weight) + ordered[upper] * weight


def rounded(value: float | None, digits: int = 6) -> float | None:
    if value is None or not math.isfinite(value):
        return None
    return round(value, digits)


def protobuf_time_s(value: Any) -> float | None:
    try:
        result = float(value.sec) + float(value.nsec) * 1e-9
    except (AttributeError, TypeError, ValueError):
        return None
    return result if math.isfinite(result) else None


def message_timestamp_s(message: Any) -> float | None:
    try:
        return protobuf_time_s(message.header.stamp)
    except AttributeError:
        return None


@dataclass(frozen=True)
class TimingSample:
    source_timestamp_s: float | None
    receipt_monotonic_s: float
    callback_duration_s: float


@dataclass(frozen=True)
class ClockMapping:
    first_sim_s: float
    last_sim_s: float
    first_host_s: float
    last_host_s: float

    def host_for_sim(self, sim_s: float) -> float | None:
        sim_span = self.last_sim_s - self.first_sim_s
        if sim_span <= 0.0:
            return None
        host_span = self.last_host_s - self.first_host_s
        return self.first_host_s + (
            (sim_s - self.first_sim_s) * host_span / sim_span
        )


class StreamRecorder:
    """Bounded metadata-only timing recorder suitable for transport callbacks."""

    def __init__(
        self,
        name: str,
        nominal_source_rate_hz: float,
        *,
        maximum_samples: int = DEFAULT_MAX_METADATA_SAMPLES,
    ) -> None:
        self.name = name
        self.nominal_source_rate_hz = float(nominal_source_rate_hz)
        self._samples: deque[TimingSample] = deque(maxlen=maximum_samples)
        self._lock = threading.Lock()
        self.total_callbacks = 0
        self.metadata_samples_overwritten = 0
        self.missing_source_timestamps = 0
        self.duplicates = 0
        self.out_of_order = 0
        self._last_source_timestamp_s: float | None = None

    def record(
        self,
        source_timestamp_s: float | None,
        receipt_monotonic_s: float,
        callback_duration_s: float,
    ) -> None:
        sample = TimingSample(
            source_timestamp_s,
            float(receipt_monotonic_s),
            max(0.0, float(callback_duration_s)),
        )
        with self._lock:
            self.total_callbacks += 1
            if len(self._samples) == self._samples.maxlen:
                self.metadata_samples_overwritten += 1
            if source_timestamp_s is None:
                self.missing_source_timestamps += 1
            elif self._last_source_timestamp_s is not None:
                if source_timestamp_s == self._last_source_timestamp_s:
                    self.duplicates += 1
                elif source_timestamp_s < self._last_source_timestamp_s:
                    self.out_of_order += 1
            if source_timestamp_s is not None:
                self._last_source_timestamp_s = source_timestamp_s
            self._samples.append(sample)

    def summary(
        self,
        *,
        mean_rtf: float | None,
        clock_mapping: ClockMapping | None,
    ) -> dict[str, Any]:
        with self._lock:
            samples = list(self._samples)
            counters = {
                "callbacks": self.total_callbacks,
                "retained_metadata_samples": len(samples),
                "metadata_samples_overwritten": self.metadata_samples_overwritten,
                "missing_source_timestamps": self.missing_source_timestamps,
                "duplicates": self.duplicates,
                "out_of_order": self.out_of_order,
            }

        timestamped = [
            sample for sample in samples if sample.source_timestamp_s is not None
        ]
        source_deltas = [
            float(current.source_timestamp_s) - float(previous.source_timestamp_s)
            for previous, current in zip(timestamped, timestamped[1:])
            if float(current.source_timestamp_s) > float(previous.source_timestamp_s)
        ]
        receipt_deltas = [
            current.receipt_monotonic_s - previous.receipt_monotonic_s
            for previous, current in zip(samples, samples[1:])
            if current.receipt_monotonic_s > previous.receipt_monotonic_s
        ]
        source_span_s = (
            float(timestamped[-1].source_timestamp_s)
            - float(timestamped[0].source_timestamp_s)
            if len(timestamped) >= 2
            else None
        )
        receipt_span_s = (
            samples[-1].receipt_monotonic_s - samples[0].receipt_monotonic_s
            if len(samples) >= 2
            else None
        )
        source_rate_hz = (
            (len(timestamped) - 1) / source_span_s
            if source_span_s is not None and source_span_s > 0.0
            else None
        )
        receipt_rate_hz = (
            (len(samples) - 1) / receipt_span_s
            if receipt_span_s is not None and receipt_span_s > 0.0
            else None
        )
        expected_count = (
            int(round(source_span_s * self.nominal_source_rate_hz)) + 1
            if source_span_s is not None
            else None
        )
        expected_missing = (
            max(0, expected_count - len(timestamped))
            if expected_count is not None
            else None
        )
        rtf_expected_receipt_rate_hz = (
            self.nominal_source_rate_hz * mean_rtf
            if mean_rtf is not None and mean_rtf > 0.0
            else None
        )
        rtf_receipt_rate_ratio = (
            receipt_rate_hz / rtf_expected_receipt_rate_hz
            if receipt_rate_hz is not None
            and rtf_expected_receipt_rate_hz is not None
            and rtf_expected_receipt_rate_hz > 0.0
            else None
        )
        mapped_residuals_ms: list[float] = []
        if clock_mapping is not None:
            for sample in timestamped:
                mapped_host = clock_mapping.host_for_sim(
                    float(sample.source_timestamp_s)
                )
                if mapped_host is not None:
                    mapped_residuals_ms.append(
                        (sample.receipt_monotonic_s - mapped_host) * 1000.0
                    )

        rate_shortfall = bool(
            rtf_receipt_rate_ratio is not None
            and rtf_receipt_rate_ratio < 0.90
        )
        continuity_failure = bool(
            (expected_missing or 0) > 0
            or counters["duplicates"] > 0
            or counters["out_of_order"] > 0
            or counters["metadata_samples_overwritten"] > 0
        )
        callback_ms = [sample.callback_duration_s * 1000.0 for sample in samples]
        return {
            "name": self.name,
            "nominal_source_rate_hz": self.nominal_source_rate_hz,
            **counters,
            "source_span_s": rounded(source_span_s),
            "receipt_span_s": rounded(receipt_span_s),
            "source_rate_hz": rounded(source_rate_hz, 3),
            "receipt_rate_hz": rounded(receipt_rate_hz, 3),
            "rtf_expected_receipt_rate_hz": rounded(
                rtf_expected_receipt_rate_hz, 3
            ),
            "rtf_receipt_rate_ratio": rounded(rtf_receipt_rate_ratio, 4),
            "expected_inclusive_count": expected_count,
            "estimated_missing_samples": expected_missing,
            "source_interval_ms": {
                "median": rounded(
                    percentile((value * 1000.0 for value in source_deltas), 0.5),
                    3,
                ),
                "p95": rounded(
                    percentile((value * 1000.0 for value in source_deltas), 0.95),
                    3,
                ),
                "max": rounded(
                    max((value * 1000.0 for value in source_deltas), default=None),
                    3,
                ),
            },
            "receipt_interval_ms": {
                "median": rounded(
                    percentile((value * 1000.0 for value in receipt_deltas), 0.5),
                    3,
                ),
                "p95": rounded(
                    percentile((value * 1000.0 for value in receipt_deltas), 0.95),
                    3,
                ),
                "max": rounded(
                    max((value * 1000.0 for value in receipt_deltas), default=None),
                    3,
                ),
            },
            "callback_duration_ms": {
                "median": rounded(percentile(callback_ms, 0.5), 4),
                "p95": rounded(percentile(callback_ms, 0.95), 4),
                "max": rounded(max(callback_ms, default=None), 4),
            },
            "mapped_receipt_residual_ms": {
                "status": (
                    "sim_to_host_affine_estimate"
                    if mapped_residuals_ms
                    else "unavailable"
                ),
                "median": rounded(percentile(mapped_residuals_ms, 0.5), 3),
                "p95": rounded(percentile(mapped_residuals_ms, 0.95), 3),
                "min": rounded(min(mapped_residuals_ms, default=None), 3),
                "max": rounded(max(mapped_residuals_ms, default=None), 3),
            },
            "collector_policy": {
                "queue": "bounded_metadata_only",
                "image_payload_bytes_copied": 0,
                "transport_queue_age": "not_exposed_by_gz_python_callback",
            },
            "rtf_normalized_backlog_suspected": bool(
                rate_shortfall or continuity_failure
            ),
        }


class StatsRecorder:
    def __init__(self, maximum_samples: int = DEFAULT_MAX_METADATA_SAMPLES) -> None:
        self._samples: deque[tuple[float, float, float, float | None]] = deque(
            maxlen=maximum_samples
        )
        self._lock = threading.Lock()

    def record(
        self,
        sim_s: float | None,
        real_s: float | None,
        receipt_monotonic_s: float,
        reported_rtf: float | None,
    ) -> None:
        if sim_s is None or real_s is None:
            return
        with self._lock:
            self._samples.append(
                (sim_s, real_s, receipt_monotonic_s, reported_rtf)
            )

    def summary(self) -> tuple[dict[str, Any], ClockMapping | None]:
        with self._lock:
            samples = list(self._samples)
        if len(samples) < 2:
            return {
                "samples": len(samples),
                "mean_rtf_from_real_deltas": None,
                "mean_rtf_from_host_deltas": None,
                "normalization_rtf": None,
                "normalization_reason": (
                    "stream receipt rates use host monotonic time"
                ),
                "reported_rtf_median": None,
                "clock_mapping": "unavailable",
            }, None
        first = samples[0]
        last = samples[-1]
        delta_sim_s = last[0] - first[0]
        delta_real_s = last[1] - first[1]
        delta_host_s = last[2] - first[2]
        mean_rtf_from_real = (
            delta_sim_s / delta_real_s if delta_real_s > 0.0 else None
        )
        mean_rtf_from_host = (
            delta_sim_s / delta_host_s if delta_host_s > 0.0 else None
        )
        reported = [
            float(sample[3])
            for sample in samples
            if sample[3] is not None and math.isfinite(float(sample[3]))
        ]
        mapping = (
            ClockMapping(first[0], last[0], first[2], last[2])
            if delta_sim_s > 0.0 and delta_host_s > 0.0
            else None
        )
        return {
            "samples": len(samples),
            "first_sim_s": rounded(first[0]),
            "last_sim_s": rounded(last[0]),
            "delta_sim_s": rounded(delta_sim_s),
            "delta_real_s": rounded(delta_real_s),
            "delta_host_s": rounded(delta_host_s),
            "mean_rtf_from_real_deltas": rounded(mean_rtf_from_real, 9),
            "mean_rtf_from_host_deltas": rounded(mean_rtf_from_host, 9),
            "normalization_rtf": rounded(mean_rtf_from_host, 9),
            "normalization_reason": (
                "stream receipt rates use host monotonic time"
            ),
            "reported_rtf_median": rounded(
                statistics.median(reported) if reported else None, 9
            ),
            "clock_mapping": (
                "affine_sim_to_host_from_stats_endpoints"
                if mapping is not None
                else "unavailable"
            ),
        }, mapping


def camera_info_snapshot(message: Any) -> dict[str, Any]:
    intrinsics = [float(value) for value in message.intrinsics.k]
    distortion = [float(value) for value in message.distortion.k]
    projection = [float(value) for value in message.projection.p]
    return {
        "source_timestamp_s": message_timestamp_s(message),
        "width": int(message.width),
        "height": int(message.height),
        "intrinsics_k": intrinsics,
        "projection_p": projection,
        "distortion_model": int(message.distortion.model),
        "distortion_k": distortion,
    }


def validate_camera_info(
    snapshot: dict[str, Any] | None,
    *,
    expected_width: int = EXPECTED_CAMERA_WIDTH,
    expected_height: int = EXPECTED_CAMERA_HEIGHT,
    expected_horizontal_fov_rad: float = EXPECTED_CAMERA_HORIZONTAL_FOV_RAD,
    focal_tolerance_px: float = 0.75,
    principal_point_tolerance_px: float = 0.75,
) -> dict[str, Any]:
    expected_fx = expected_width / (
        2.0 * math.tan(expected_horizontal_fov_rad / 2.0)
    )
    if snapshot is None:
        return {
            "pass": False,
            "authority": "gazebo_camera_info",
            "reason": "camera_info_not_received",
            "expected_fx_px": rounded(expected_fx, 6),
        }
    intrinsics = snapshot.get("intrinsics_k", [])
    if len(intrinsics) < 9:
        return {
            "pass": False,
            "authority": "gazebo_camera_info",
            "reason": "camera_info_intrinsics_incomplete",
            "snapshot": snapshot,
            "expected_fx_px": rounded(expected_fx, 6),
        }
    fx, fy, cx, cy = (
        float(intrinsics[0]),
        float(intrinsics[4]),
        float(intrinsics[2]),
        float(intrinsics[5]),
    )
    checks = {
        "width": int(snapshot.get("width", -1)) == expected_width,
        "height": int(snapshot.get("height", -1)) == expected_height,
        "fx_from_horizontal_fov": abs(fx - expected_fx) <= focal_tolerance_px,
        "square_pixels": abs(fy - fx) <= focal_tolerance_px,
        "cx_pixel_center": abs(cx - expected_width / 2.0)
        <= principal_point_tolerance_px,
        "cy_pixel_center": abs(cy - expected_height / 2.0)
        <= principal_point_tolerance_px,
        "zero_distortion": all(
            abs(float(value)) <= 1e-9
            for value in snapshot.get("distortion_k", [])
        ),
    }
    return {
        "pass": all(checks.values()),
        "authority": "gazebo_camera_info",
        "reason": "match" if all(checks.values()) else "contract_mismatch",
        "checks": checks,
        "expected": {
            "width": expected_width,
            "height": expected_height,
            "horizontal_fov_rad": expected_horizontal_fov_rad,
            "fx_px": rounded(expected_fx, 6),
            "cx_px": expected_width / 2.0,
            "cy_px": expected_height / 2.0,
        },
        "observed": snapshot,
    }


def _required_bool(
    mapping: dict[str, Any], key: str, path: str, violations: list[str]
) -> bool:
    if key not in mapping:
        violations.append(f"missing:{path}.{key}")
        return False
    value = bool(mapping[key])
    if value:
        violations.append(f"active:{path}.{key}")
    return value


def _required_zero(
    mapping: dict[str, Any], key: str, path: str, violations: list[str]
) -> int | float | None:
    if key not in mapping:
        violations.append(f"missing:{path}.{key}")
        return None
    value = mapping[key]
    try:
        numeric = float(value)
    except (TypeError, ValueError):
        violations.append(f"invalid:{path}.{key}")
        return None
    if numeric != 0.0:
        violations.append(f"nonzero:{path}.{key}={value}")
    return value


def evaluate_api_safety(
    payload: dict[str, Any],
    expected_drones: Iterable[str] = DRONE_MODELS,
) -> dict[str, Any]:
    violations: list[str] = []
    observations: dict[str, Any] = {"drones": {}, "tracking": {}}
    drones = payload.get("drones")
    if not isinstance(drones, dict):
        return {
            "pass": False,
            "violations": ["missing:drones"],
            "observations": observations,
        }
    for drone_id in expected_drones:
        drone = drones.get(drone_id)
        if not isinstance(drone, dict):
            violations.append(f"missing:drones.{drone_id}")
            continue
        status = drone.get("status")
        if not isinstance(status, dict):
            violations.append(f"missing:drones.{drone_id}.status")
            continue
        armed = _required_bool(
            status, "armed", f"drones.{drone_id}.status", violations
        )
        observations["drones"][drone_id] = {
            "armed": armed,
            "failsafe": bool(status.get("failsafe", False)),
            "nav_state": status.get("nav_state"),
            "online": drone.get("online"),
        }

    tracking = payload.get("tracking")
    if not isinstance(tracking, dict):
        violations.append("missing:tracking")
        tracking = {}
    for key in (
        "active",
        "visual_follow_requested",
        "visual_follow_active",
        "visual_target_stream_active",
    ):
        observations["tracking"][key] = _required_bool(
            tracking, key, "tracking", violations
        )
    observations["tracking"]["follow_mode_request_attempts"] = _required_zero(
        tracking, "follow_mode_request_attempts", "tracking", violations
    )

    pose_streams = payload.get("tracking_pose_streams")
    if not isinstance(pose_streams, dict):
        violations.append("missing:tracking_pose_streams")
        pose_streams = {}
    bridge_observations: dict[str, Any] = {}
    for drone_id in expected_drones:
        stream = pose_streams.get(drone_id)
        bridge = (
            stream.get("visual_follow_bridge")
            if isinstance(stream, dict)
            else None
        )
        path = f"tracking_pose_streams.{drone_id}.visual_follow_bridge"
        if not isinstance(bridge, dict):
            violations.append(f"missing:{path}")
            continue
        bridge_observations[drone_id] = {
            "enabled": _required_bool(bridge, "enabled", path, violations),
            "publish_count": _required_zero(
                bridge, "publish_count", path, violations
            ),
            "mode_request_attempts": _required_zero(
                bridge, "mode_request_attempts", path, violations
            ),
            "px4_nav_state": bridge.get("px4_nav_state"),
        }
    observations["visual_follow_bridge"] = bridge_observations
    return {
        "pass": not violations,
        "violations": violations,
        "observations": observations,
    }


def compact_api_snapshot(payload: dict[str, Any]) -> dict[str, Any]:
    safety = evaluate_api_safety(payload)
    return {
        "server_timestamp_ms": payload.get("server_timestamp_ms"),
        "mqtt_connected": payload.get("mqtt_connected"),
        "safety": safety,
    }


def endpoint_description(endpoint: Any) -> dict[str, Any]:
    message_type = getattr(endpoint, "msg_type_name", None)
    if callable(message_type):
        message_type = message_type()
    return {"message_type": str(message_type or "unknown")}


def topic_info_snapshot(node: Any, topic: str) -> dict[str, Any]:
    try:
        publishers, subscribers = node.topic_info(topic)
    except Exception as error:
        return {"topic": topic, "error": str(error), "publisher_count": None}
    return {
        "topic": topic,
        "publisher_count": len(publishers),
        "subscriber_count": len(subscribers),
        "publishers": [endpoint_description(item) for item in publishers],
        "subscribers": [endpoint_description(item) for item in subscribers],
    }


def evaluate_gate(
    *,
    stream_summaries: dict[str, dict[str, Any]],
    camera_contracts: dict[str, dict[str, Any]],
    api_safety_samples: list[dict[str, Any]],
    gimbal_topics: dict[str, dict[str, Any]],
) -> dict[str, Any]:
    blockers: list[str] = []
    if not stream_summaries:
        blockers.append("runtime_stream_evidence_missing")
    for name, summary in stream_summaries.items():
        if summary.get("callbacks", 0) < 2:
            blockers.append(f"insufficient_stream_samples:{name}")
        if summary.get("rtf_normalized_backlog_suspected"):
            blockers.append(f"stream_continuity_or_rate:{name}")
    for drone_id, contract in camera_contracts.items():
        if not contract.get("pass"):
            blockers.append(f"camera_contract:{drone_id}")
    if not api_safety_samples:
        blockers.append("api_safety_evidence_missing")
    elif any(not sample.get("pass") for sample in api_safety_samples):
        blockers.append("api_safety_invariant_failed")
    for topic, info in gimbal_topics.items():
        publisher_count = info.get("publisher_count")
        if publisher_count is None:
            blockers.append(f"gimbal_authority_unknown:{topic}")
        elif int(publisher_count) != 1:
            blockers.append(
                f"gimbal_publisher_count:{topic}={publisher_count}"
            )

    out_of_scope_open_gates = [
        "vehicle_camera_gimbal_clock_alignment_not_closed",
        "frame_origin_altitude_contract_not_closed",
        "measured_target_shadow_not_run_requires_separate_approval",
        "follow_mode_jerk_fixture_not_run_requires_exact_command_approval",
    ]
    return {
        "p0b_a_result": "PASS" if not blockers else "FAIL",
        "phase0b_decision": "NO-GO",
        "p1b_authorized": False,
        "follow_mode_authorized": False,
        "blockers": blockers + out_of_scope_open_gates,
    }


def fetch_json_get(url: str, timeout_s: float = 2.0) -> dict[str, Any]:
    request = urllib.request.Request(
        url,
        headers={"Accept": "application/json"},
        method="GET",
    )
    with urllib.request.urlopen(request, timeout=timeout_s) as response:
        payload = json.loads(response.read().decode("utf-8"))
    if not isinstance(payload, dict):
        raise ValueError("GET response is not a JSON object")
    return payload


@dataclass(frozen=True)
class GazeboBindings:
    node_type: Any
    camera_info_type: Any
    image_type: Any
    imu_type: Any
    pose_v_type: Any
    world_statistics_type: Any


def load_gazebo_bindings() -> GazeboBindings:
    from gz.msgs10.camera_info_pb2 import CameraInfo
    from gz.msgs10.image_pb2 import Image
    from gz.msgs10.imu_pb2 import IMU
    from gz.msgs10.pose_v_pb2 import Pose_V
    from gz.msgs10.world_stats_pb2 import WorldStatistics
    from gz.transport13 import Node

    return GazeboBindings(Node, CameraInfo, Image, IMU, Pose_V, WorldStatistics)


class Phase0BRuntimeCollector:
    def __init__(
        self,
        *,
        api_url: str,
        duration_s: float,
        api_interval_s: float,
        bindings: GazeboBindings | None = None,
    ) -> None:
        self.api_url = api_url
        self.duration_s = max(1.0, float(duration_s))
        self.api_interval_s = max(0.1, float(api_interval_s))
        self.bindings = bindings or load_gazebo_bindings()
        self.node = self.bindings.node_type()
        self.stats = StatsRecorder()
        self.streams: dict[str, StreamRecorder] = {}
        self.camera_info: dict[str, dict[str, Any]] = {}
        self.camera_info_counts: dict[str, int] = {}
        self.camera_metadata: dict[str, dict[str, Any]] = {}
        self._metadata_lock = threading.Lock()
        self._subscribed_topics: list[str] = []

    @staticmethod
    def _camera_topics(model_name: str) -> dict[str, str]:
        prefix = f"/world/default/model/{model_name}/link/camera_link/sensor"
        return {
            "image": f"{prefix}/camera/image",
            "camera_info": f"{prefix}/camera/camera_info",
            "camera_imu": f"{prefix}/camera_imu/imu",
        }

    @staticmethod
    def _gimbal_topics(model_name: str) -> list[str]:
        return [
            f"/model/{model_name}/command/gimbal_{axis}"
            for axis in ("roll", "pitch", "yaw")
        ]

    def _subscribe(
        self, message_type: Any, topic: str, callback: Callable[[Any], None]
    ) -> None:
        if not self.node.subscribe(message_type, topic, callback):
            raise RuntimeError(f"Could not subscribe to {topic}")
        self._subscribed_topics.append(topic)

    def _timing_callback(
        self,
        recorder: StreamRecorder,
        metadata_callback: Callable[[Any], None] | None = None,
    ) -> Callable[[Any], None]:
        def callback(message: Any) -> None:
            callback_started_ns = time.perf_counter_ns()
            receipt_s = time.monotonic()
            source_s = message_timestamp_s(message)
            if metadata_callback is not None:
                metadata_callback(message)
            callback_duration_s = (
                time.perf_counter_ns() - callback_started_ns
            ) * 1e-9
            recorder.record(source_s, receipt_s, callback_duration_s)

        return callback

    def _record_image_metadata(self, drone_id: str, message: Any) -> None:
        with self._metadata_lock:
            self.camera_metadata[drone_id] = {
                "width": int(message.width),
                "height": int(message.height),
                "step": int(message.step),
                "pixel_format_type": int(message.pixel_format_type),
                "payload_bytes_observed_not_copied": len(message.data),
                "payload_bytes_copied": 0,
            }

    def _record_camera_info(self, drone_id: str, message: Any) -> None:
        snapshot = camera_info_snapshot(message)
        with self._metadata_lock:
            self.camera_info[drone_id] = snapshot
            self.camera_info_counts[drone_id] = (
                self.camera_info_counts.get(drone_id, 0) + 1
            )

    def _record_stats(self, message: Any) -> None:
        reported_rtf = getattr(message, "real_time_factor", None)
        try:
            reported_rtf = float(reported_rtf)
        except (TypeError, ValueError):
            reported_rtf = None
        self.stats.record(
            protobuf_time_s(message.sim_time),
            protobuf_time_s(message.real_time),
            time.monotonic(),
            reported_rtf,
        )

    def subscribe(self) -> None:
        for drone_id, model_name in DRONE_MODELS.items():
            topics = self._camera_topics(model_name)
            image_recorder = StreamRecorder(
                f"{drone_id}.camera_image", EXPECTED_CAMERA_RATE_HZ
            )
            imu_recorder = StreamRecorder(
                f"{drone_id}.camera_imu", EXPECTED_CAMERA_IMU_RATE_HZ
            )
            self.streams[image_recorder.name] = image_recorder
            self.streams[imu_recorder.name] = imu_recorder
            self._subscribe(
                self.bindings.image_type,
                topics["image"],
                self._timing_callback(
                    image_recorder,
                    lambda message, selected=drone_id: self._record_image_metadata(
                        selected, message
                    ),
                ),
            )
            self._subscribe(
                self.bindings.imu_type,
                topics["camera_imu"],
                self._timing_callback(imu_recorder),
            )
            self._subscribe(
                self.bindings.camera_info_type,
                topics["camera_info"],
                lambda message, selected=drone_id: self._record_camera_info(
                    selected, message
                ),
            )

        pose_recorder = StreamRecorder("world.pose_info", 50.0)
        self.streams[pose_recorder.name] = pose_recorder
        self._subscribe(
            self.bindings.pose_v_type,
            "/world/default/pose/info",
            self._timing_callback(pose_recorder),
        )
        self._subscribe(
            self.bindings.world_statistics_type,
            "/stats",
            self._record_stats,
        )

    def unsubscribe(self) -> None:
        for topic in self._subscribed_topics:
            try:
                self.node.unsubscribe(topic)
            except Exception:
                pass
        self._subscribed_topics.clear()

    def gimbal_topic_evidence(self) -> dict[str, dict[str, Any]]:
        evidence: dict[str, dict[str, Any]] = {}
        for model_name in DRONE_MODELS.values():
            for topic in self._gimbal_topics(model_name):
                evidence[topic] = topic_info_snapshot(self.node, topic)
        return evidence

    def run(self) -> dict[str, Any]:
        started_utc = datetime.now(timezone.utc)
        preflight_payload = fetch_json_get(self.api_url)
        preflight_safety = evaluate_api_safety(preflight_payload)
        api_samples = [
            {
                "receipt_monotonic_s": time.monotonic(),
                **compact_api_snapshot(preflight_payload),
            }
        ]
        if not preflight_safety["pass"]:
            gate = evaluate_gate(
                stream_summaries={},
                camera_contracts={},
                api_safety_samples=[preflight_safety],
                gimbal_topics={},
            )
            return {
                "schema": "phase0b_runtime_evidence/v1",
                "collection_aborted": True,
                "abort_reason": "unsafe_or_incomplete_read_only_api_preflight",
                "started_utc": started_utc.isoformat(),
                "api_samples": api_samples,
                "gate": gate,
            }

        self.subscribe()
        gimbal_before = self.gimbal_topic_evidence()
        deadline = time.monotonic() + self.duration_s
        next_api_poll = time.monotonic() + self.api_interval_s
        try:
            while time.monotonic() < deadline:
                now = time.monotonic()
                if now >= next_api_poll:
                    try:
                        payload = fetch_json_get(self.api_url)
                        api_samples.append(
                            {
                                "receipt_monotonic_s": now,
                                **compact_api_snapshot(payload),
                            }
                        )
                    except (OSError, ValueError, urllib.error.URLError) as error:
                        api_samples.append(
                            {
                                "receipt_monotonic_s": now,
                                "error": str(error),
                                "safety": {
                                    "pass": False,
                                    "violations": ["api_get_failed"],
                                },
                            }
                        )
                    next_api_poll += self.api_interval_s
                time.sleep(min(0.02, max(0.0, deadline - time.monotonic())))
        finally:
            gimbal_after = self.gimbal_topic_evidence()
            self.unsubscribe()

        stats_summary, clock_mapping = self.stats.summary()
        mean_rtf = stats_summary.get("normalization_rtf")
        stream_summaries = {
            name: recorder.summary(
                mean_rtf=mean_rtf,
                clock_mapping=clock_mapping,
            )
            for name, recorder in self.streams.items()
        }
        with self._metadata_lock:
            camera_info = dict(self.camera_info)
            camera_info_counts = dict(self.camera_info_counts)
            camera_metadata = dict(self.camera_metadata)
        camera_contracts = {
            drone_id: validate_camera_info(camera_info.get(drone_id))
            for drone_id in DRONE_MODELS
        }
        api_safety_samples = [
            sample.get(
                "safety",
                {"pass": False, "violations": ["api_safety_missing"]},
            )
            for sample in api_samples
        ]
        gate = evaluate_gate(
            stream_summaries=stream_summaries,
            camera_contracts=camera_contracts,
            api_safety_samples=api_safety_samples,
            gimbal_topics=gimbal_after,
        )
        return {
            "schema": "phase0b_runtime_evidence/v1",
            "collection_aborted": False,
            "started_utc": started_utc.isoformat(),
            "finished_utc": datetime.now(timezone.utc).isoformat(),
            "requested_duration_s": self.duration_s,
            "safety_contract": {
                "gazebo_operations": ["subscribe", "topic_info", "unsubscribe"],
                "http_operations": ["GET /api/drones"],
                "gazebo_publish": False,
                "mavlink_send": False,
                "http_control_post": False,
                "follow_mode_requested": False,
                "image_payload_copy": False,
            },
            "environment": {
                "hostname": platform.node(),
                "python": platform.python_version(),
                "gz_partition": os.environ.get("GZ_PARTITION", "default"),
                "collector_sha256": hashlib.sha256(
                    Path(__file__).read_bytes()
                ).hexdigest(),
            },
            "gazebo_stats": stats_summary,
            "streams": stream_summaries,
            "camera_image_metadata": camera_metadata,
            "camera_info_message_counts": camera_info_counts,
            "camera_contracts": camera_contracts,
            "api_samples": api_samples,
            "gimbal_topics_before": gimbal_before,
            "gimbal_topics_after": gimbal_after,
            "gate": gate,
        }


def default_output_path(output_dir: Path) -> Path:
    timestamp = datetime.now().astimezone().strftime("%Y%m%d_%H%M%S")
    return output_dir / f"phase0b_runtime_evidence_{timestamp}.json"


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--api-url", default=DEFAULT_API_URL)
    parser.add_argument("--duration", type=float, default=DEFAULT_DURATION_S)
    parser.add_argument(
        "--api-interval", type=float, default=DEFAULT_API_INTERVAL_S
    )
    parser.add_argument("--output", type=Path)
    parser.add_argument("--output-dir", type=Path, default=Path("artifacts"))
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    output_path = args.output or default_output_path(args.output_dir)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    collector = Phase0BRuntimeCollector(
        api_url=args.api_url,
        duration_s=args.duration,
        api_interval_s=args.api_interval,
    )
    evidence = collector.run()
    output_path.write_text(
        json.dumps(evidence, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    print(f"Evidence: {output_path.resolve()}")
    print(json.dumps(evidence["gate"], indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
