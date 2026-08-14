from __future__ import annotations

import threading
import time
from dataclasses import dataclass, replace
from typing import Any

import numpy as np

from depth_model_adapter import DepthMap, DepthModelAdapter


@dataclass(frozen=True)
class DepthJob:
    frame_bgr: np.ndarray
    measurement_timestamp_s: float
    frame_index: int
    context: Any = None
    generation: int = 0
    submitted_timestamp_s: float | None = None


@dataclass(frozen=True)
class DepthResult:
    valid: bool
    reason: str
    measurement_timestamp_s: float
    completed_timestamp_s: float
    frame_index: int
    inference_ms: float
    depth_map: DepthMap | None
    context: Any = None
    generation: int = 0
    submitted_timestamp_s: float | None = None
    inference_started_timestamp_s: float | None = None
    result_publish_timestamp_s: float | None = None
    worker_thread_name: str | None = None
    publisher_thread_name: str | None = None
    profiling_stages: dict[str, float] | None = None

    @property
    def age_s(self) -> float:
        return max(0.0, time.monotonic() - self.measurement_timestamp_s)


class LatestDepthWorker:
    """Single-slot asynchronous inference worker.

    `submit` replaces an unprocessed frame, keeping camera/tracker latency
    bounded even when inference is slower than the source stream.
    """

    def __init__(self, adapter: DepthModelAdapter) -> None:
        self.adapter = adapter
        self._condition = threading.Condition()
        self._pending: DepthJob | None = None
        self._latest: DepthResult | None = None
        self._latest_version = 0
        self._generation = 0
        self._stop = False
        self._thread: threading.Thread | None = None
        self._pending_submitted_s: float | None = None
        self._active_started_s: float | None = None
        self.submitted = 0
        self.dropped = 0
        self.processed = 0
        self.failed = 0
        self.discarded_generation_results = 0
        self.load_error = ""
        self.last_exception = ""

    def start(self) -> None:
        with self._condition:
            if self._thread is not None and self._thread.is_alive():
                return
            self._stop = False
            self._thread = threading.Thread(
                target=self._run,
                name="metric-depth-worker",
                daemon=True,
            )
            self._thread.start()

    def stop(self, timeout_s: float = 2.0) -> None:
        with self._condition:
            self._stop = True
            self._pending = None
            self._pending_submitted_s = None
            self._condition.notify_all()
            thread = self._thread
        if thread is not None:
            thread.join(timeout=max(0.0, timeout_s))

    def reset(self) -> None:
        with self._condition:
            self._pending = None
            self._pending_submitted_s = None
            self._latest = None
            self._latest_version = 0
            self._generation += 1
            self.submitted = 0
            self.dropped = 0
            self.processed = 0
            self.failed = 0
            self.last_exception = ""

    def submit(self, job: DepthJob) -> None:
        self.start()
        submitted_timestamp_s = time.monotonic()
        owned = DepthJob(
            frame_bgr=np.ascontiguousarray(job.frame_bgr).copy(),
            measurement_timestamp_s=float(job.measurement_timestamp_s),
            frame_index=int(job.frame_index),
            context=job.context,
            generation=self._generation,
            submitted_timestamp_s=submitted_timestamp_s,
        )
        with self._condition:
            if self._pending is not None:
                self.dropped += 1
            self._pending = owned
            self._pending_submitted_s = time.monotonic()
            self.submitted += 1
            self._condition.notify()

    def latest(self) -> tuple[int, DepthResult | None]:
        with self._condition:
            return self._latest_version, self._latest

    def status(self) -> dict[str, Any]:
        with self._condition:
            thread_alive = bool(
                self._thread is not None and self._thread.is_alive()
            )
            latest = self._latest
            now_s = time.monotonic()
            return {
                "thread_alive": thread_alive,
                "generation": self._generation,
                "submitted": self.submitted,
                "dropped": self.dropped,
                "processed": self.processed,
                "failed": self.failed,
                "discarded_generation_results": (
                    self.discarded_generation_results
                ),
                "pending": self._pending is not None,
                "pending_age_ms": (
                    None
                    if self._pending_submitted_s is None
                    else max(0.0, now_s - self._pending_submitted_s) * 1000.0
                ),
                "active": self._active_started_s is not None,
                "active_age_ms": (
                    None
                    if self._active_started_s is None
                    else max(0.0, now_s - self._active_started_s) * 1000.0
                ),
                "latest_valid": bool(latest and latest.valid),
                "latest_reason": latest.reason if latest else "no_result",
                "latest_frame_index": latest.frame_index if latest else 0,
                "latest_inference_ms": latest.inference_ms if latest else None,
                "latest_age_ms": (
                    latest.age_s * 1000.0 if latest is not None else None
                ),
                "load_error": self.load_error,
                "last_exception": self.last_exception,
            }

    def _run(self) -> None:
        try:
            self.adapter.load()
            load_error = ""
        except Exception as error:
            load_error = f"model_load_failed:{error}"
        with self._condition:
            self.load_error = load_error
        while True:
            with self._condition:
                self._condition.wait_for(
                    lambda: self._stop or self._pending is not None
                )
                if self._stop:
                    return
                job = self._pending
                self._pending = None
                self._pending_submitted_s = None
                self._active_started_s = time.monotonic()
            assert job is not None
            started = time.perf_counter()
            inference_started_timestamp_s = time.monotonic()
            try:
                if load_error:
                    raise RuntimeError(load_error)
                depth_map = self.adapter.infer(job.frame_bgr)
                result = DepthResult(
                    valid=True,
                    reason="ok",
                    measurement_timestamp_s=job.measurement_timestamp_s,
                    completed_timestamp_s=time.monotonic(),
                    frame_index=job.frame_index,
                    inference_ms=(time.perf_counter() - started) * 1000.0,
                    depth_map=depth_map,
                    context=job.context,
                    generation=job.generation,
                    submitted_timestamp_s=job.submitted_timestamp_s,
                    inference_started_timestamp_s=(
                        inference_started_timestamp_s
                    ),
                    worker_thread_name=threading.current_thread().name,
                    profiling_stages=getattr(
                        self.adapter, "last_profiling_stages", None
                    ),
                )
            except Exception as error:
                failure_reason = f"inference_failed:{error}"
                result = DepthResult(
                    valid=False,
                    reason=failure_reason,
                    measurement_timestamp_s=job.measurement_timestamp_s,
                    completed_timestamp_s=time.monotonic(),
                    frame_index=job.frame_index,
                    inference_ms=(time.perf_counter() - started) * 1000.0,
                    depth_map=None,
                    context=job.context,
                    generation=job.generation,
                    submitted_timestamp_s=job.submitted_timestamp_s,
                    inference_started_timestamp_s=(
                        inference_started_timestamp_s
                    ),
                    worker_thread_name=threading.current_thread().name,
                    profiling_stages=getattr(
                        self.adapter, "last_profiling_stages", None
                    ),
                )
            with self._condition:
                self._active_started_s = None
                if job.generation != self._generation:
                    self.discarded_generation_results += 1
                    continue
                result = replace(
                    result,
                    result_publish_timestamp_s=time.monotonic(),
                    publisher_thread_name=threading.current_thread().name,
                )
                self._latest = result
                self._latest_version += 1
                self.processed += 1
                if result.valid:
                    self.last_exception = ""
                else:
                    self.failed += 1
                    self.last_exception = result.reason
