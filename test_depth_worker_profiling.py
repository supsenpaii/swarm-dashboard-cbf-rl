"""DepthResult.profiling_stages plumbing from adapter to worker (opt-in).

See docs/CORE_RANGE_LIVE_STACK_CONTENTION_REPORT.md.
"""

from __future__ import annotations

import time

import numpy as np

from depth_model_adapter import CallableDepthAdapter, DepthModelAdapter, DepthMap
from depth_worker import DepthJob, LatestDepthWorker


def _wait_for_result(worker: LatestDepthWorker, timeout_s: float = 2.0):
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        _, result = worker.latest()
        if result is not None:
            return result
        time.sleep(0.01)
    raise AssertionError("timed out waiting for a depth result")


class _ProfilingStubAdapter(DepthModelAdapter):
    """Stand-in adapter that mimics MidasSmallAdapter's opt-in attribute."""

    def __init__(self, stage_dict: dict[str, float] | None) -> None:
        self._stage_dict = stage_dict
        self.last_profiling_stages: dict[str, float] | None = None

    def load(self) -> None:
        return

    def infer(self, frame_bgr: np.ndarray) -> DepthMap:
        self.last_profiling_stages = self._stage_dict
        return DepthMap(
            inverse_depth=np.ones(frame_bgr.shape[:2], dtype=np.float32),
            source="stub",
        )


def test_worker_attaches_adapter_profiling_stages_to_result() -> None:
    stage_dict = {"model_forward_s": 0.01, "numpy_postprocess_s": 0.001}
    worker = LatestDepthWorker(_ProfilingStubAdapter(stage_dict))
    frame = np.ones((10, 12, 3), dtype=np.uint8)
    worker.submit(DepthJob(frame, 1.0, 1))
    result = _wait_for_result(worker)
    worker.stop()

    assert result.valid
    assert result.profiling_stages == stage_dict


def test_worker_leaves_profiling_stages_none_when_adapter_has_no_such_attribute() -> None:
    def infer(frame: np.ndarray) -> np.ndarray:
        return np.ones(frame.shape[:2], dtype=np.float32)

    worker = LatestDepthWorker(CallableDepthAdapter(infer))
    frame = np.ones((10, 12, 3), dtype=np.uint8)
    worker.submit(DepthJob(frame, 1.0, 1))
    result = _wait_for_result(worker)
    worker.stop()

    assert result.valid
    assert result.profiling_stages is None
