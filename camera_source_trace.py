"""Opt-in, additive hop-level camera-source tracing.

Disabled unless ``SWARM_CAMERA_TRACE_JSONL`` names an output file path; the
env var is re-read on every call (a cheap dict lookup) so tracing turns on
and off within a single process without restarting it. When unset, ``record``
is a no-op that touches no file. Used to isolate a camera-source FPS
regression (CORE_RANGE_CAMERA_SOURCE_FPS_AUDIT_AND_FIX); it does not change
default runtime behavior, calibration, control, or PX4 code paths.
"""

from __future__ import annotations

import json
import os
import threading
import time
from pathlib import Path
from typing import Any

_TRACE_PATH_ENV = "SWARM_CAMERA_TRACE_JSONL"
_write_lock = threading.Lock()
_dir_ready: set[str] = set()


def trace_enabled() -> bool:
    return bool(os.environ.get(_TRACE_PATH_ENV, "").strip())


def record(event: str, **fields: Any) -> None:
    path = os.environ.get(_TRACE_PATH_ENV, "").strip()
    if not path:
        return
    row = {"event": event, "trace_monotonic_s": time.monotonic(), **fields}
    line = json.dumps(row, default=str)
    try:
        with _write_lock:
            if path not in _dir_ready:
                Path(path).parent.mkdir(parents=True, exist_ok=True)
                _dir_ready.add(path)
            with open(path, "a") as handle:
                handle.write(line + "\n")
    except OSError:
        # Best-effort diagnostic tracing must never break the camera
        # callback or tracking thread it is instrumenting.
        pass
