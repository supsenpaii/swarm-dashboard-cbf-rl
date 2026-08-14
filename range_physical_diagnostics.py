"""Append-only, observation-only physical range diagnostics sidecar.

The sidecar is disabled unless an explicit diagnostics directory or the
existing range-dataset directory is configured.  It never changes a range,
calibration, gate, estimator, runtime output, or controller decision.
"""

from __future__ import annotations

import hashlib
import json
import math
import os
from pathlib import Path
import re
import shutil
import threading
import time
from collections import deque
from typing import Any, Mapping


DIAGNOSTICS_SCHEMA_VERSION = "core_range_logging_3_12m_v001"
DIAGNOSTICS_FILENAME = "physical_diagnostics.jsonl"
DIAGNOSTICS_MANIFEST_FILENAME = "physical_diagnostics_manifest.json"
_IDENTIFIER = re.compile(r"^[A-Za-z0-9_.-]{1,96}$")
TIMESTAMP_STAGE_ORDER = (
    "frame_receipt",
    "depth_submit",
    "depth_worker_start",
    "depth_complete",
    "result_publish",
    "consumer_receive",
    "consume",
    "sidecar_write",
)


def timestamp_stage(
    timestamp_s: float | None,
    *,
    component: str,
    execution_context: str,
    semantic: str,
) -> dict[str, Any]:
    return {
        "timestamp_s": timestamp_s,
        "clock_id": "python_monotonic",
        "component": component,
        "execution_context": execution_context,
        "semantic": semantic,
    }


def validate_timestamp_stages(stages: Mapping[str, Any]) -> tuple[bool, str]:
    values: list[float] = []
    for name in TIMESTAMP_STAGE_ORDER:
        stage = stages.get(name)
        if not isinstance(stage, Mapping):
            return False, f"timestamp_stage_missing:{name}"
        try:
            value = float(stage.get("timestamp_s"))
        except (TypeError, ValueError):
            return False, f"timestamp_value_invalid:{name}"
        if not math.isfinite(value):
            return False, f"timestamp_value_invalid:{name}"
        if stage.get("clock_id") != "python_monotonic":
            return False, f"timestamp_clock_invalid:{name}"
        for field in ("component", "execution_context", "semantic"):
            if not isinstance(stage.get(field), str) or not stage.get(field):
                return False, f"timestamp_metadata_invalid:{name}:{field}"
        values.append(value)
    if any(left > right for left, right in zip(values, values[1:])):
        return False, "timestamp_order_invalid"
    return True, "ok"


def canonical_sha256(value: Any) -> str:
    encoded = json.dumps(
        value,
        sort_keys=True,
        separators=(",", ":"),
        allow_nan=False,
    ).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def _identifier(value: Any, name: str) -> str:
    text = str(value).strip()
    if not _IDENTIFIER.fullmatch(text):
        raise ValueError(f"{name}_invalid")
    return text


def _json_safe(value: Any, path: str = "root") -> Any:
    if value is None or isinstance(value, (str, bool, int)):
        return value
    if isinstance(value, float):
        if not math.isfinite(value):
            raise ValueError(f"diagnostic_nonfinite:{path}")
        return value
    if isinstance(value, Mapping):
        return {
            str(key): _json_safe(item, f"{path}.{key}")
            for key, item in value.items()
        }
    if isinstance(value, (list, tuple)):
        return [
            _json_safe(item, f"{path}[{index}]")
            for index, item in enumerate(value)
        ]
    if hasattr(value, "item"):
        return _json_safe(value.item(), path)
    raise ValueError(f"diagnostic_type_invalid:{path}:{type(value).__name__}")


def diagnostics_manifest(run_id: str, target_id: str) -> dict[str, Any]:
    return {
        "diagnostics_schema_version": DIAGNOSTICS_SCHEMA_VERSION,
        "artifact_role": "instrumentation_only_sidecar",
        "run_id": _identifier(run_id, "run_id"),
        "target_id": _identifier(target_id, "target_id"),
        "group_key": "run_id/session_id",
        "records_file": DIAGNOSTICS_FILENAME,
        "scope": "minimal_core_range_logging_3_to_12m",
        "runtime_semantics": {
            "changes_raw_range": False,
            "changes_calibration": False,
            "changes_runtime_output": False,
            "changes_controller": False,
        },
        "field_contract": {
            "reference_centers": "raw geometry, GT link/model centers, optical-center availability",
            "camera_info": "resolution, K convention, distortion/rectification availability and fingerprint",
            "extrinsics": "vehicle/camera poses and configured lever arms with frame labels",
            "anchors": "one record per configured grid point including accept/reject evidence",
            "calibration": "fit raw and filtered parameters, applied/cache source, recovery/reseed state and epoch",
            "timestamps": "measurement, source simulation, depth completion/age and synchronized pose/GT diagnostics",
        },
        "timestamp_stage_order": list(TIMESTAMP_STAGE_ORDER),
    }


class RangePhysicalDiagnosticsCollector:
    """Fail-soft append-only sidecar collector."""

    def __init__(
        self,
        root: str | os.PathLike[str] | None = None,
        *,
        run_id: str = "",
        target_id: str = "",
        load_error: str = "",
    ) -> None:
        self.root = None if root is None else Path(root)
        self.run_id = str(run_id).strip()
        self.target_id = str(target_id).strip()
        self.load_error = str(load_error)
        self._lock = threading.Lock()
        self.record_count = 0
        self.rejected_count = 0
        self.last_reason = "disabled" if self.root is None else "ready"
        self.write_mode = os.getenv(
            "SWARM_RANGE_DIAGNOSTICS_WRITE_MODE", "disk"
        ).strip().lower()
        if self.write_mode not in {"disk", "memory"}:
            self.load_error = "physical_diagnostics_write_mode_invalid"
        self.memory_max_records = max(
            1, int(os.getenv("SWARM_RANGE_DIAGNOSTICS_MEMORY_MAX_RECORDS", "512"))
        )
        self._memory_records: deque[bytes] = deque()
        self.dropped_record_count = 0
        self._timings_ms: dict[str, deque[float]] = {
            name: deque(maxlen=512)
            for name in ("record_construction", "json_serialization", "file_write", "fsync", "record_total")
        }
        self.minimum_free_bytes = int(
            os.getenv("SWARM_RANGE_DIAGNOSTICS_MIN_FREE_BYTES", str(256 * 1024 * 1024))
        )
        self.maximum_file_bytes = int(
            os.getenv("SWARM_RANGE_DIAGNOSTICS_MAX_FILE_BYTES", str(64 * 1024 * 1024))
        )
        self.fsync_every_records = max(
            1, int(os.getenv("SWARM_RANGE_DIAGNOSTICS_FSYNC_EVERY", "10"))
        )
        self.projected_record_bytes = int(
            os.getenv("SWARM_RANGE_DIAGNOSTICS_PROJECTED_RECORD_BYTES", str(128 * 1024))
        )
        self.projected_record_count = int(
            os.getenv("SWARM_RANGE_DIAGNOSTICS_PROJECTED_RECORD_COUNT", "100")
        )
        self._segment_index = 0
        self._records_since_fsync = 0
        self._file_descriptor: int | None = None

    @classmethod
    def from_environment(cls) -> "RangePhysicalDiagnosticsCollector":
        if os.getenv(
            "SWARM_RANGE_PHYSICAL_DIAGNOSTICS_ENABLED", "true"
        ).strip().lower() in {"0", "false", "no", "off"}:
            return cls()
        path = os.getenv("SWARM_RANGE_PHYSICAL_DIAGNOSTICS_DIR", "").strip()
        if not path:
            path = os.getenv("SWARM_RANGE_DATASET_DIR", "").strip()
        if not path:
            return cls()
        collector = cls(
            path,
            run_id=os.getenv("SWARM_RANGE_DATASET_RUN_ID", "").strip(),
            target_id=os.getenv("SWARM_RANGE_DATASET_TARGET_ID", "").strip(),
        )
        try:
            collector._prepare()
        except Exception as error:
            collector.load_error = str(error)
            collector.last_reason = collector.load_error
        return collector

    @property
    def enabled(self) -> bool:
        return bool(self.root is not None and not self.load_error)

    def _prepare(self) -> None:
        if self.root is None:
            raise ValueError("diagnostics_path_missing")
        manifest = diagnostics_manifest(self.run_id, self.target_id)
        if self.write_mode == "memory":
            return
        self.root.mkdir(parents=True, exist_ok=True)
        available = shutil.disk_usage(self.root).free
        projected = self.projected_record_bytes * self.projected_record_count
        if available - projected < self.minimum_free_bytes:
            raise ValueError("physical_diagnostics_disk_preflight_failed")
        path = self.root / DIAGNOSTICS_MANIFEST_FILENAME
        if path.exists():
            current = json.loads(path.read_text(encoding="utf-8"))
            if current != manifest:
                raise ValueError("physical_diagnostics_manifest_mismatch")
        else:
            path.write_text(
                json.dumps(manifest, indent=2, sort_keys=True) + "\n",
                encoding="utf-8",
            )

    def _segment_path(self) -> Path:
        assert self.root is not None
        if self._segment_index == 0:
            return self.root / DIAGNOSTICS_FILENAME
        return self.root / f"physical_diagnostics.{self._segment_index:06d}.jsonl"

    def _open_segment(self, encoded_size: int) -> int:
        path = self._segment_path()
        current_size = path.stat().st_size if path.exists() else 0
        if current_size and current_size + encoded_size > self.maximum_file_bytes:
            if self._file_descriptor is not None:
                os.fsync(self._file_descriptor)
                os.close(self._file_descriptor)
                self._file_descriptor = None
            self._segment_index += 1
            path = self._segment_path()
        if self._file_descriptor is None:
            self._file_descriptor = os.open(
                path,
                os.O_WRONLY | os.O_CREAT | os.O_APPEND,
                0o644,
            )
        return self._file_descriptor

    @staticmethod
    def _write_bytes(fd: int, encoded: bytes) -> None:
        offset = 0
        while offset < len(encoded):
            written = os.write(fd, encoded[offset:])
            if written <= 0:
                raise OSError("physical_diagnostics_short_write")
            offset += written

    def _transactional_append(self, encoded: bytes) -> tuple[float, float]:
        assert self.root is not None
        available = shutil.disk_usage(self.root).free
        if available - len(encoded) < self.minimum_free_bytes:
            raise OSError("physical_diagnostics_disk_guard_stop")
        fd = self._open_segment(len(encoded))
        original_size = os.lseek(fd, 0, os.SEEK_END)
        write_started = time.perf_counter()
        try:
            self._write_bytes(fd, encoded)
        except Exception:
            os.ftruncate(fd, original_size)
            os.fsync(fd)
            raise
        write_ms = (time.perf_counter() - write_started) * 1000.0
        self._records_since_fsync += 1
        fsync_ms = 0.0
        if self._records_since_fsync >= self.fsync_every_records:
            fsync_started = time.perf_counter()
            os.fsync(fd)
            fsync_ms = (time.perf_counter() - fsync_started) * 1000.0
            self._records_since_fsync = 0
        return write_ms, fsync_ms

    def record(self, payload: Mapping[str, Any]) -> bool:
        if not self.enabled or self.root is None:
            self.last_reason = self.load_error or "disabled"
            return False
        try:
            record_started = time.perf_counter()
            value = _json_safe(dict(payload))
            if value.get("diagnostics_schema_version") != DIAGNOSTICS_SCHEMA_VERSION:
                raise ValueError("physical_diagnostics_schema_mismatch")
            if value.get("run_id") != self.run_id:
                raise ValueError("physical_diagnostics_run_mismatch")
            _identifier(value.get("group_id"), "group_id")
            session = int(value.get("session_id"))
            frame = int(value.get("frame_index"))
            timestamp = float(value.get("measurement_timestamp_s"))
            if session <= 0 or frame < 0 or not math.isfinite(timestamp):
                raise ValueError("physical_diagnostics_identity_invalid")
            if value.get("instrumentation_only") is not True:
                raise ValueError("physical_diagnostics_semantics_invalid")
            stages = value.get("timestamp_stages")
            if not isinstance(stages, Mapping):
                stages = {}
            stages = dict(stages)
            stages["sidecar_write"] = timestamp_stage(
                time.monotonic(),
                component="RangePhysicalDiagnosticsCollector",
                execution_context=threading.current_thread().name,
                semantic="immediately_before_sidecar_record_serialization",
            )
            value["timestamp_stages"] = stages
            timestamp_ok, timestamp_reason = validate_timestamp_stages(stages)
            value["timestamp_order_valid"] = timestamp_ok
            ground_truth = value.get("ground_truth")
            ground_truth_trace_valid = False
            if isinstance(ground_truth, Mapping):
                try:
                    ground_truth_trace_valid = bool(
                        ground_truth.get("valid") is True
                        and math.isfinite(float(ground_truth.get("distance_m")))
                        and math.isfinite(float(ground_truth.get("timestamp_s")))
                        and ground_truth.get("timestamp_clock_id")
                        == "gazebo_sim_time"
                    )
                except (TypeError, ValueError):
                    ground_truth_trace_valid = False
            is_training_row = value.get("stage") == "raw_range_computed"
            value["ground_truth_trace_valid"] = ground_truth_trace_valid
            value["diagnostic_complete"] = bool(
                timestamp_ok and ground_truth_trace_valid and is_training_row
            )
            if not timestamp_ok:
                reason_code = timestamp_reason
            elif not is_training_row:
                reason_code = "stage_not_core_training_row"
            elif not ground_truth_trace_valid:
                reason_code = "ground_truth_trace_invalid"
            else:
                reason_code = "ok"
            value["reason_code"] = reason_code
            trace_identity = {
                "run_id": value.get("run_id"),
                "session_id": session,
                "group_id": value.get("group_id"),
                "frame_index": frame,
                "measurement_timestamp_s": timestamp,
                "source_sim_timestamp_s": value.get("source_sim_timestamp_s"),
                "ground_truth_timestamp_s": (
                    (value.get("ground_truth") or {}).get("timestamp_s")
                    if isinstance(value.get("ground_truth"), Mapping)
                    else None
                ),
            }
            value["trace_identity_sha256"] = canonical_sha256(trace_identity)
            value["record_sha256"] = canonical_sha256(value)
            construction_ms = (time.perf_counter() - record_started) * 1000.0
            serialization_started = time.perf_counter()
            line = json.dumps(
                value,
                sort_keys=True,
                separators=(",", ":"),
                allow_nan=False,
            )
            serialization_ms = (time.perf_counter() - serialization_started) * 1000.0
            write_ms = fsync_ms = 0.0
            with self._lock:
                encoded = (line + "\n").encode("utf-8")
                if self.write_mode == "memory":
                    if len(self._memory_records) >= self.memory_max_records:
                        self.dropped_record_count += 1
                        raise BufferError("physical_diagnostics_memory_buffer_full")
                    self._memory_records.append(encoded)
                else:
                    write_ms, fsync_ms = self._transactional_append(encoded)
                self.record_count += 1
                self._timings_ms["record_construction"].append(construction_ms)
                self._timings_ms["json_serialization"].append(serialization_ms)
                self._timings_ms["file_write"].append(write_ms)
                self._timings_ms["fsync"].append(fsync_ms)
                self._timings_ms["record_total"].append(
                    (time.perf_counter() - record_started) * 1000.0
                )
            self.last_reason = "recorded"
            return True
        except Exception as error:
            self.rejected_count += 1
            self.last_reason = str(error)
            return False

    def finalize_integrity(self) -> dict[str, Any]:
        with self._lock:
            if self._file_descriptor is not None:
                os.fsync(self._file_descriptor)
                os.close(self._file_descriptor)
                self._file_descriptor = None
            paths = [] if self.root is None else sorted(
                self.root.glob("physical_diagnostics*.jsonl")
            )
            parsed = 0
            malformed = 0
            checksum_mismatches = 0
            for path in paths:
                with path.open("r", encoding="utf-8") as stream:
                    for line in stream:
                        try:
                            row = json.loads(line)
                            expected = row.pop("record_sha256")
                            if canonical_sha256(row) != expected:
                                checksum_mismatches += 1
                            parsed += 1
                        except (KeyError, TypeError, ValueError, json.JSONDecodeError):
                            malformed += 1
            return {
                "segment_count": len(paths),
                "parsed_record_count": parsed,
                "collector_record_count": self.record_count,
                "malformed_json_lines": malformed,
                "checksum_mismatches": checksum_mismatches,
                "counts_reconciled": parsed == self.record_count,
                "pass": bool(
                    malformed == 0
                    and checksum_mismatches == 0
                    and parsed == self.record_count
                ),
            }

    def status(self) -> dict[str, Any]:
        timing = {}
        for name, samples in self._timings_ms.items():
            ordered = sorted(samples)
            timing[name] = {
                "count": len(ordered),
                "median_ms": (ordered[len(ordered) // 2] if ordered else None),
                "p90_ms": (ordered[min(len(ordered) - 1, int(0.90 * len(ordered)))] if ordered else None),
                "p95_ms": (ordered[min(len(ordered) - 1, int(0.95 * len(ordered)))] if ordered else None),
                "thread": "dashboard-tracking",
            }
        return {
            "enabled": self.enabled,
            "schema_version": DIAGNOSTICS_SCHEMA_VERSION,
            "root": None if self.root is None else str(self.root),
            "run_id": self.run_id,
            "target_id": self.target_id,
            "record_count": self.record_count,
            "rejected_count": self.rejected_count,
            "last_reason": self.last_reason,
            "load_error": self.load_error,
            "instrumentation_only": True,
            "write_mode": self.write_mode,
            "memory_buffer_depth": len(self._memory_records),
            "memory_buffer_capacity": self.memory_max_records,
            "dropped_record_count": self.dropped_record_count,
            "timing": timing,
            "minimum_free_bytes": self.minimum_free_bytes,
            "maximum_file_bytes": self.maximum_file_bytes,
            "segment_index": self._segment_index,
        }
