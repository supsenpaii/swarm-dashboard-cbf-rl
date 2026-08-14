#!/usr/bin/env python3
"""Opt-in, per-second Gazebo subscription callback phase instrumentation.

The Gazebo Python binding invokes application callbacks after protobuf
deserialization.  Consequently binding-internal receive/deserialize time and
transport queue depth are not observable here.  This recorder reports those
limits explicitly and measures only application-visible phases.
"""

from __future__ import annotations

import csv
import gc
import math
import os
import threading
import time
from collections import defaultdict
from pathlib import Path
from typing import Any, Callable


def _percentile(values: list[float], percentile: float) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    position = (len(ordered) - 1) * percentile
    lower = math.floor(position)
    upper = math.ceil(position)
    if lower == upper:
        return ordered[lower]
    return ordered[lower] + (ordered[upper] - ordered[lower]) * (position - lower)


class SubscriptionPhaseRecorder:
    FIELDNAMES = [
        "monotonic_second", "wall_clock_s", "topic_id", "expected_rate_hz",
        "message_count", "payload_bytes", "message_rate_hz", "bytes_per_s",
        "callback_ms_p50", "callback_ms_p95", "callback_ms_p99", "callback_ms_max",
        "receive_wait_ms_p50", "receive_wait_ms_p95", "receive_wait_ms_p99",
        "receive_wait_ms_max", "copy_ms_p50", "copy_ms_p95", "copy_ms_p99",
        "copy_ms_max", "processing_ms_p50", "processing_ms_p95",
        "processing_ms_p99", "processing_ms_max", "publish_ms_p50",
        "publish_ms_p95", "publish_ms_p99", "publish_ms_max",
        "callback_thread_cpu_ms", "callback_thread_count", "max_active_callbacks",
        "late_interarrival_count", "estimated_dropped_or_coalesced_count",
        "message_age_ms_p50", "message_age_ms_p95", "message_age_ms_p99",
        "message_age_ms_max", "gc_pause_count", "gc_pause_ms",
        "binding_receive_time_observable", "binding_deserialize_time_observable",
        "transport_queue_depth_observable", "copy_phase_observable",
        "message_age_observable", "publish_phase_invoked",
    ]

    def __init__(self, output: Path | None) -> None:
        self.output = output
        self.enabled = output is not None
        self._lock = threading.RLock()
        self._buckets: dict[tuple[int, str], dict[str, Any]] = {}
        self._last_receipt_ns: dict[str, int] = {}
        self._active_callbacks = 0
        self._gc_started_ns: int | None = None
        self._gc_by_second: dict[int, list[float]] = defaultdict(list)
        self._file = None
        self._writer = None
        self._closed = False
        if not self.enabled:
            return
        assert output is not None
        output.parent.mkdir(parents=True, exist_ok=True)
        self._file = output.open("w", newline="")
        self._writer = csv.DictWriter(self._file, fieldnames=self.FIELDNAMES)
        self._writer.writeheader()
        self._file.flush()
        gc.callbacks.append(self._gc_callback)

    @classmethod
    def from_environment(cls) -> "SubscriptionPhaseRecorder":
        raw = os.environ.get("SWARM_GAZEBO_SUBSCRIPTION_TRACE_CSV", "").strip()
        return cls(Path(raw).resolve() if raw else None)

    def _gc_callback(self, phase: str, _info: dict[str, Any]) -> None:
        now_ns = time.monotonic_ns()
        with self._lock:
            if phase == "start":
                self._gc_started_ns = now_ns
            elif phase == "stop" and self._gc_started_ns is not None:
                second = self._gc_started_ns // 1_000_000_000
                self._gc_by_second[second].append((now_ns - self._gc_started_ns) / 1e6)
                self._gc_started_ns = None

    @staticmethod
    def _payload_size(message: Any) -> int:
        # Images dominate this path. Reading the existing bytes field length is
        # constant-time and avoids a protobuf-wide ByteSize traversal in the
        # count-only configuration.
        try:
            data = message.data
        except Exception:
            data = None
        if data is not None:
            try:
                return len(data)
            except Exception:
                pass
        try:
            return int(message.ByteSize())
        except Exception:
            return 0

    @staticmethod
    def _source_age_ms(message: Any) -> float | None:
        # A Gazebo header stamp is simulation time, while receipt is monotonic
        # host time. Without an explicit clock mapping their difference is not
        # a meaningful message age, so it remains unobservable.
        del message
        return None

    def execute(
        self,
        *,
        topic_id: str,
        expected_rate_hz: float,
        message: Any,
        copy_action: Callable[[], Any] | None = None,
        processing_action: Callable[[], Any] | None = None,
        publish_action: Callable[[], Any] | None = None,
        copy_observable: bool = False,
    ) -> None:
        if not self.enabled:
            if copy_action is not None:
                copy_action()
            if processing_action is not None:
                processing_action()
            if publish_action is not None:
                publish_action()
            return

        receipt_ns = time.monotonic_ns()
        thread_cpu_started_ns = time.thread_time_ns()
        thread_id = threading.get_native_id()
        payload_bytes = self._payload_size(message)
        period_ns = 1e9 / expected_rate_hz if expected_rate_hz > 0 else 0.0
        with self._lock:
            previous_ns = self._last_receipt_ns.get(topic_id)
            self._last_receipt_ns[topic_id] = receipt_ns
            self._active_callbacks += 1
            active = self._active_callbacks

        receive_wait_ms = None
        late_count = 0
        drop_estimate = 0
        if previous_ns is not None and period_ns > 0:
            interval_ns = receipt_ns - previous_ns
            receive_wait_ms = max(0.0, (interval_ns - period_ns) / 1e6)
            late_count = int(interval_ns > period_ns * 1.5)
            drop_estimate = max(0, int(round(interval_ns / period_ns)) - 1)

        copy_ms = None
        processing_ms = None
        publish_ms = None
        try:
            if copy_action is not None:
                phase_started = time.perf_counter_ns()
                copy_action()
                copy_ms = (time.perf_counter_ns() - phase_started) / 1e6
            if processing_action is not None:
                phase_started = time.perf_counter_ns()
                processing_action()
                processing_ms = (time.perf_counter_ns() - phase_started) / 1e6
            if publish_action is not None:
                phase_started = time.perf_counter_ns()
                publish_action()
                publish_ms = (time.perf_counter_ns() - phase_started) / 1e6
        finally:
            ended_ns = time.monotonic_ns()
            thread_cpu_ms = (time.thread_time_ns() - thread_cpu_started_ns) / 1e6
            callback_ms = (ended_ns - receipt_ns) / 1e6
            second = receipt_ns // 1_000_000_000
            key = (second, topic_id)
            with self._lock:
                bucket = self._buckets.setdefault(key, {
                    "expected_rate_hz": expected_rate_hz,
                    "count": 0, "bytes": 0, "callback": [], "receive_wait": [],
                    "copy": [], "processing": [], "publish": [], "thread_cpu": 0.0,
                    "threads": set(), "max_active": 0, "late": 0, "drops": 0,
                    "age": [], "copy_observable": copy_observable,
                    "publish_invoked": publish_action is not None,
                    "wall_clock_s": time.time(),
                })
                bucket["count"] += 1
                bucket["bytes"] += payload_bytes
                bucket["callback"].append(callback_ms)
                if receive_wait_ms is not None:
                    bucket["receive_wait"].append(receive_wait_ms)
                if copy_ms is not None:
                    bucket["copy"].append(copy_ms)
                if processing_ms is not None:
                    bucket["processing"].append(processing_ms)
                if publish_ms is not None:
                    bucket["publish"].append(publish_ms)
                age_ms = self._source_age_ms(message)
                if age_ms is not None:
                    bucket["age"].append(age_ms)
                bucket["thread_cpu"] += thread_cpu_ms
                bucket["threads"].add(thread_id)
                bucket["max_active"] = max(bucket["max_active"], active)
                bucket["late"] += late_count
                bucket["drops"] += drop_estimate
                self._active_callbacks -= 1
                self._flush_before(second - 1)

    @staticmethod
    def _stats(row: dict[str, Any], prefix: str, values: list[float]) -> None:
        row[f"{prefix}_p50"] = _percentile(values, 0.50)
        row[f"{prefix}_p95"] = _percentile(values, 0.95)
        row[f"{prefix}_p99"] = _percentile(values, 0.99)
        row[f"{prefix}_max"] = max(values) if values else None

    def _flush_before(self, maximum_second: int) -> None:
        if self._writer is None or self._file is None:
            return
        keys = sorted(key for key in self._buckets if key[0] <= maximum_second)
        for key in keys:
            second, topic_id = key
            bucket = self._buckets.pop(key)
            row: dict[str, Any] = {
                "monotonic_second": second,
                "wall_clock_s": round(bucket["wall_clock_s"], 6),
                "topic_id": topic_id,
                "expected_rate_hz": bucket["expected_rate_hz"],
                "message_count": bucket["count"],
                "payload_bytes": bucket["bytes"],
                "message_rate_hz": bucket["count"],
                "bytes_per_s": bucket["bytes"],
                "callback_thread_cpu_ms": round(bucket["thread_cpu"], 6),
                "callback_thread_count": len(bucket["threads"]),
                "max_active_callbacks": bucket["max_active"],
                "late_interarrival_count": bucket["late"],
                "estimated_dropped_or_coalesced_count": bucket["drops"],
                "gc_pause_count": len(self._gc_by_second.get(second, [])),
                "gc_pause_ms": round(sum(self._gc_by_second.pop(second, [])), 6),
                "binding_receive_time_observable": False,
                "binding_deserialize_time_observable": False,
                "transport_queue_depth_observable": False,
                "copy_phase_observable": bucket["copy_observable"],
                "message_age_observable": bool(bucket["age"]),
                "publish_phase_invoked": bucket["publish_invoked"],
            }
            for prefix, values in (
                ("callback_ms", bucket["callback"]),
                ("receive_wait_ms", bucket["receive_wait"]),
                ("copy_ms", bucket["copy"]),
                ("processing_ms", bucket["processing"]),
                ("publish_ms", bucket["publish"]),
                ("message_age_ms", bucket["age"]),
            ):
                self._stats(row, prefix, values)
            self._writer.writerow(row)
        if keys:
            self._file.flush()

    def close(self) -> None:
        with self._lock:
            if self._closed:
                return
            self._closed = True
            if self.enabled:
                self._flush_before(2**63 - 1)
                try:
                    gc.callbacks.remove(self._gc_callback)
                except ValueError:
                    pass
            if self._file is not None:
                self._file.close()


_RECORDER: SubscriptionPhaseRecorder | None = None


def get_recorder() -> SubscriptionPhaseRecorder:
    global _RECORDER
    if _RECORDER is None:
        _RECORDER = SubscriptionPhaseRecorder.from_environment()
    return _RECORDER
