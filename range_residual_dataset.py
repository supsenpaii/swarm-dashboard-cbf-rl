from __future__ import annotations

import hashlib
import json
import math
import os
import re
import threading
import time
from collections import deque
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Iterable, Mapping

from range_ground_truth import configured_ground_truth_source
from range_ground_truth import (
    RTK_GROUND_TRUTH_SOURCE,
    SIMULATION_GROUND_TRUTH_SOURCE,
)
from range_residual_correction import (
    FEATURE_NAMES,
    FEATURE_SCHEMA_VERSION,
    MISSING_VALUE_POLICY,
    RangeCorrectionResult,
    RangeResidualFeatures,
    feature_schema,
)


DATASET_SCHEMA_VERSION = "m52_midas_range_residual_dataset_v2"
SAMPLES_FILENAME = "samples.jsonl"
MANIFEST_FILENAME = "manifest.json"
_IDENTIFIER = re.compile(r"^[A-Za-z0-9_.-]{1,96}$")
TRAINING_GROUND_TRUTH_QUALITIES = {
    "simulation_exact",
    "rtk_fixed",
    "total_station",
    "uwb_calibrated",
}
LOCKED_SOURCE_QUALITIES = {
    SIMULATION_GROUND_TRUTH_SOURCE: "simulation_exact",
    RTK_GROUND_TRUTH_SOURCE: "rtk_fixed",
}
MAXIMUM_TRAINING_GROUND_TRUTH_UNCERTAINTY_M = 0.20
MAXIMUM_TRAINING_GROUND_TRUTH_TIME_OFFSET_MS = 100.0


def file_sha256(path: str | os.PathLike[str]) -> str:
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _finite_positive(value: Any, name: str) -> float:
    number = float(value)
    if not math.isfinite(number) or number <= 0.0:
        raise ValueError(f"{name}_invalid")
    return number


def _identifier(value: Any, name: str) -> str:
    text = str(value).strip()
    if not _IDENTIFIER.fullmatch(text):
        raise ValueError(f"{name}_invalid")
    return text


def dataset_manifest(
    run_id: str,
    target_id: str,
    ground_truth_source: str,
) -> dict[str, Any]:
    return {
        "dataset_schema_version": DATASET_SCHEMA_VERSION,
        "feature_schema": feature_schema(),
        "label": {
            "name": "range_residual_m",
            "definition": "ground_truth_distance_m - physics_distance_m",
            "ground_truth_source": _identifier(
                ground_truth_source,
                "ground_truth_source",
            ),
            "ground_truth_target_id": _identifier(target_id, "target_id"),
            "distance_semantics": "camera_center_to_target_center_slant_range",
            "quality_metadata_required": True,
        },
        "grouping": {
            "run_id": _identifier(run_id, "run_id"),
            "group_key": "run_id/session_id",
            "group_ids_are_features": False,
        },
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "samples_file": SAMPLES_FILENAME,
    }


@dataclass(frozen=True)
class RangeResidualDatasetSample:
    run_id: str
    target_id: str
    group_id: str
    session_id: int
    frame_index: int
    measurement_timestamp_s: float
    source_sim_timestamp_s: float | None
    features: RangeResidualFeatures
    physics_distance_m: float
    ground_truth_distance_m: float | None
    ground_truth_valid: bool
    ground_truth_reason: str
    ground_truth_source: str
    ground_truth_uncertainty_m: float | None
    ground_truth_time_offset_ms: float | None
    ground_truth_quality: str
    ground_truth_lever_arm_corrected: bool
    correction_mode: str
    candidate_distance_m: float | None

    @property
    def residual_target_m(self) -> float | None:
        if not self.ground_truth_valid or self.ground_truth_distance_m is None:
            return None
        return self.ground_truth_distance_m - self.physics_distance_m

    def as_record(self) -> dict[str, Any]:
        return {
            "dataset_schema_version": DATASET_SCHEMA_VERSION,
            "run_id": self.run_id,
            "target_id": self.target_id,
            "group_id": self.group_id,
            "session_id": self.session_id,
            "frame_index": self.frame_index,
            "measurement_timestamp_s": self.measurement_timestamp_s,
            "source_sim_timestamp_s": self.source_sim_timestamp_s,
            "features": self.features.as_dict(),
            "physics_distance_m": self.physics_distance_m,
            "ground_truth_distance_m": self.ground_truth_distance_m,
            "ground_truth_valid": self.ground_truth_valid,
            "ground_truth_reason": self.ground_truth_reason,
            "ground_truth_source": self.ground_truth_source,
            "ground_truth_uncertainty_m": self.ground_truth_uncertainty_m,
            "ground_truth_time_offset_ms": self.ground_truth_time_offset_ms,
            "ground_truth_quality": self.ground_truth_quality,
            "ground_truth_lever_arm_corrected": (
                self.ground_truth_lever_arm_corrected
            ),
            "range_residual_m": self.residual_target_m,
            "correction_mode": self.correction_mode,
            "candidate_distance_m": self.candidate_distance_m,
        }


class RangeResidualDatasetCollector:
    """Append-only evaluation collector; never changes fusion output."""

    def __init__(
        self,
        root: str | os.PathLike[str] | None = None,
        *,
        run_id: str = "",
        target_id: str = "",
        ground_truth_source: str = "gazebo_camera_to_target_center",
        load_error: str = "",
    ) -> None:
        self.root = None if root is None else Path(root)
        self.run_id = str(run_id).strip()
        self.target_id = str(target_id).strip()
        self.ground_truth_source = str(ground_truth_source).strip()
        self.load_error = str(load_error)
        self._lock = threading.Lock()
        self.record_count = 0
        self.labeled_count = 0
        self.rejected_count = 0
        self.last_reason = "disabled" if self.root is None else "ready"
        self.write_mode = os.getenv(
            "SWARM_RANGE_DATASET_WRITE_MODE", "disk"
        ).strip().lower()
        if self.write_mode not in {"disk", "memory"}:
            self.load_error = "range_dataset_write_mode_invalid"
        self._serialize_ms: deque[float] = deque(maxlen=512)
        self._write_ms: deque[float] = deque(maxlen=512)

    @classmethod
    def from_environment(cls) -> "RangeResidualDatasetCollector":
        path = os.getenv("SWARM_RANGE_DATASET_DIR", "").strip()
        if not path:
            return cls()
        run_id = os.getenv("SWARM_RANGE_DATASET_RUN_ID", "").strip()
        target_id = os.getenv("SWARM_RANGE_DATASET_TARGET_ID", "").strip()
        try:
            ground_truth_source = configured_ground_truth_source()
            collector = cls(
                path,
                run_id=run_id,
                target_id=target_id,
                ground_truth_source=ground_truth_source,
            )
            collector._prepare()
            return collector
        except Exception as error:
            return cls(load_error=str(error))

    @property
    def enabled(self) -> bool:
        return bool(self.root is not None and not self.load_error)

    def _prepare(self) -> None:
        if self.root is None:
            raise ValueError("dataset_path_missing")
        _identifier(self.run_id, "run_id")
        _identifier(self.target_id, "target_id")
        _identifier(self.ground_truth_source, "ground_truth_source")
        if self.write_mode == "memory":
            return
        self.root.mkdir(parents=True, exist_ok=True)
        manifest_path = self.root / MANIFEST_FILENAME
        if manifest_path.exists():
            current = json.loads(manifest_path.read_text(encoding="utf-8"))
            if not isinstance(current, dict):
                raise ValueError("dataset_manifest_invalid")
            if current.get("dataset_schema_version") != DATASET_SCHEMA_VERSION:
                raise ValueError("dataset_schema_mismatch")
            feature = current.get("feature_schema", {})
            if (
                not isinstance(feature, dict)
                or feature.get("version") != FEATURE_SCHEMA_VERSION
                or feature.get("names") != list(FEATURE_NAMES)
                or feature.get("missing_value_policy")
                != MISSING_VALUE_POLICY
            ):
                raise ValueError("dataset_feature_schema_mismatch")
            grouping = current.get("grouping", {})
            if not isinstance(grouping, dict) or grouping.get("run_id") != self.run_id:
                raise ValueError("dataset_run_id_mismatch")
            label = current.get("label", {})
            if not isinstance(label, dict) or label.get(
                "ground_truth_target_id"
            ) != self.target_id:
                raise ValueError("dataset_target_id_mismatch")
            if label.get("ground_truth_source") != self.ground_truth_source:
                raise ValueError("dataset_ground_truth_source_mismatch")
        else:
            manifest_path.write_text(
                json.dumps(
                    dataset_manifest(
                        self.run_id,
                        self.target_id,
                        self.ground_truth_source,
                    ),
                    indent=2,
                    sort_keys=True,
                )
                + "\n",
                encoding="utf-8",
            )

    def record(
        self,
        *,
        features: RangeResidualFeatures,
        physics_distance_m: float,
        correction: RangeCorrectionResult,
        group_id: str,
        session_id: int,
        frame_index: int,
        measurement_timestamp_s: float,
        source_sim_timestamp_s: float | None,
        ground_truth_distance_m: float | None,
        ground_truth_valid: bool,
        ground_truth_reason: str,
        ground_truth_uncertainty_m: float | None = 0.0,
        ground_truth_time_offset_ms: float | None = 0.0,
        ground_truth_quality: str = "simulation_exact",
        ground_truth_lever_arm_corrected: bool = True,
    ) -> bool:
        if not self.enabled or self.root is None:
            self.last_reason = self.load_error or "disabled"
            return False
        try:
            features.as_array()
            physics = _finite_positive(physics_distance_m, "physics_distance")
            group = _identifier(group_id, "group_id")
            session = int(session_id)
            frame = int(frame_index)
            timestamp = float(measurement_timestamp_s)
            if session <= 0 or frame < 0 or not math.isfinite(timestamp):
                raise ValueError("sample_identity_invalid")
            sim_timestamp = (
                None
                if source_sim_timestamp_s is None
                else float(source_sim_timestamp_s)
            )
            if sim_timestamp is not None and not math.isfinite(sim_timestamp):
                raise ValueError("source_sim_timestamp_invalid")
            truth = None
            truth_valid = bool(ground_truth_valid)
            if truth_valid:
                truth = _finite_positive(
                    ground_truth_distance_m,
                    "ground_truth_distance",
                )
            uncertainty = (
                None
                if ground_truth_uncertainty_m is None
                else float(ground_truth_uncertainty_m)
            )
            if (
                uncertainty is not None
                and (not math.isfinite(uncertainty) or uncertainty < 0.0)
            ):
                raise ValueError("ground_truth_uncertainty_invalid")
            time_offset_ms = (
                None
                if ground_truth_time_offset_ms is None
                else float(ground_truth_time_offset_ms)
            )
            if time_offset_ms is not None and not math.isfinite(time_offset_ms):
                raise ValueError("ground_truth_time_offset_invalid")
            quality = _identifier(
                ground_truth_quality,
                "ground_truth_quality",
            )
            if truth_valid:
                if (
                    uncertainty is None
                    or uncertainty
                    > MAXIMUM_TRAINING_GROUND_TRUTH_UNCERTAINTY_M
                ):
                    raise ValueError("ground_truth_uncertainty_too_large")
                if (
                    time_offset_ms is None
                    or abs(time_offset_ms)
                    > MAXIMUM_TRAINING_GROUND_TRUTH_TIME_OFFSET_MS
                ):
                    raise ValueError("ground_truth_time_offset_too_large")
                if quality not in TRAINING_GROUND_TRUTH_QUALITIES:
                    raise ValueError("ground_truth_quality_invalid")
                expected_quality = LOCKED_SOURCE_QUALITIES.get(
                    self.ground_truth_source
                )
                if expected_quality is not None and quality != expected_quality:
                    raise ValueError("ground_truth_quality_source_mismatch")
                if not ground_truth_lever_arm_corrected:
                    raise ValueError("ground_truth_lever_arm_uncorrected")
            sample = RangeResidualDatasetSample(
                run_id=self.run_id,
                target_id=self.target_id,
                group_id=group,
                session_id=session,
                frame_index=frame,
                measurement_timestamp_s=timestamp,
                source_sim_timestamp_s=sim_timestamp,
                features=features,
                physics_distance_m=physics,
                ground_truth_distance_m=truth,
                ground_truth_valid=truth_valid,
                ground_truth_reason=str(ground_truth_reason),
                ground_truth_source=self.ground_truth_source,
                ground_truth_uncertainty_m=uncertainty,
                ground_truth_time_offset_ms=time_offset_ms,
                ground_truth_quality=quality,
                ground_truth_lever_arm_corrected=bool(
                    ground_truth_lever_arm_corrected
                ),
                correction_mode=str(correction.reason),
                candidate_distance_m=correction.candidate_distance_m,
            )
            serialize_started = time.perf_counter()
            line = json.dumps(
                sample.as_record(),
                sort_keys=True,
                separators=(",", ":"),
                allow_nan=False,
            )
            serialize_ms = (time.perf_counter() - serialize_started) * 1000.0
            with self._lock:
                write_started = time.perf_counter()
                if self.write_mode == "disk":
                    with (self.root / SAMPLES_FILENAME).open(
                        "a", encoding="utf-8"
                    ) as stream:
                        stream.write(line + "\n")
                write_ms = (time.perf_counter() - write_started) * 1000.0
                self._serialize_ms.append(serialize_ms)
                self._write_ms.append(write_ms)
                self.record_count += 1
                if truth_valid:
                    self.labeled_count += 1
            self.last_reason = "recorded"
            return True
        except Exception as error:
            self.rejected_count += 1
            self.last_reason = str(error)
            return False

    def status(self) -> dict[str, Any]:
        def timing(samples: deque[float]) -> dict[str, Any]:
            ordered = sorted(samples)
            return {
                "count": len(ordered),
                "median_ms": ordered[len(ordered)//2] if ordered else None,
                "p90_ms": ordered[min(len(ordered)-1, int(.90*len(ordered)))] if ordered else None,
                "p95_ms": ordered[min(len(ordered)-1, int(.95*len(ordered)))] if ordered else None,
                "thread": "dashboard-tracking",
            }
        return {
            "enabled": self.enabled,
            "path": None if self.root is None else str(self.root),
            "run_id": self.run_id,
            "target_id": self.target_id,
            "ground_truth_source": self.ground_truth_source,
            "load_error": self.load_error,
            "record_count": self.record_count,
            "labeled_count": self.labeled_count,
            "rejected_count": self.rejected_count,
            "last_reason": self.last_reason,
            "write_mode": self.write_mode,
            "timing": {"json_serialization": timing(self._serialize_ms), "file_write": timing(self._write_ms)},
        }


def load_training_samples(
    dataset_roots: Iterable[str | os.PathLike[str]],
) -> tuple[list[RangeResidualDatasetSample], dict[str, Any]]:
    samples: list[RangeResidualDatasetSample] = []
    manifests: list[dict[str, Any]] = []
    sample_digests: dict[str, str] = {}
    seen_identities: set[tuple[str, str, int]] = set()
    for root_value in dataset_roots:
        root = Path(root_value)
        manifest_path = root / MANIFEST_FILENAME
        samples_path = root / SAMPLES_FILENAME
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
        if manifest.get("dataset_schema_version") != DATASET_SCHEMA_VERSION:
            raise ValueError("dataset_schema_mismatch")
        feature = manifest.get("feature_schema", {})
        if (
            feature.get("version") != FEATURE_SCHEMA_VERSION
            or feature.get("names") != list(FEATURE_NAMES)
        ):
            raise ValueError("dataset_feature_schema_mismatch")
        run_id = _identifier(manifest["grouping"]["run_id"], "run_id")
        target_id = _identifier(
            manifest["label"]["ground_truth_target_id"],
            "target_id",
        )
        ground_truth_source = _identifier(
            manifest["label"]["ground_truth_source"],
            "ground_truth_source",
        )
        manifests.append(manifest)
        sample_digests[str(root)] = file_sha256(samples_path)
        with samples_path.open("r", encoding="utf-8") as stream:
            for line_number, line in enumerate(stream, start=1):
                if not line.strip():
                    continue
                record = json.loads(line)
                if record.get("dataset_schema_version") != DATASET_SCHEMA_VERSION:
                    raise ValueError(f"sample_schema_mismatch:{line_number}")
                if record.get("run_id") != run_id:
                    raise ValueError(f"sample_run_id_mismatch:{line_number}")
                if record.get("target_id") != target_id:
                    raise ValueError(f"sample_target_id_mismatch:{line_number}")
                feature_values = record.get("features")
                if not isinstance(feature_values, Mapping) or set(
                    feature_values.keys()
                ) != set(FEATURE_NAMES):
                    raise ValueError(f"sample_feature_schema_mismatch:{line_number}")
                features = RangeResidualFeatures(
                    **{name: float(feature_values[name]) for name in FEATURE_NAMES}
                )
                features.as_array()
                session_id = int(record["session_id"])
                frame_index = int(record["frame_index"])
                identity = (run_id, str(record["group_id"]), frame_index)
                if identity in seen_identities:
                    raise ValueError(f"duplicate_sample_identity:{line_number}")
                seen_identities.add(identity)
                truth_valid = bool(record.get("ground_truth_valid", False))
                truth = record.get("ground_truth_distance_m")
                sample = RangeResidualDatasetSample(
                    run_id=run_id,
                    target_id=target_id,
                    group_id=_identifier(record["group_id"], "group_id"),
                    session_id=session_id,
                    frame_index=frame_index,
                    measurement_timestamp_s=float(
                        record["measurement_timestamp_s"]
                    ),
                    source_sim_timestamp_s=(
                        None
                        if record.get("source_sim_timestamp_s") is None
                        else float(record["source_sim_timestamp_s"])
                    ),
                    features=features,
                    physics_distance_m=_finite_positive(
                        record["physics_distance_m"], "physics_distance"
                    ),
                    ground_truth_distance_m=(
                        _finite_positive(truth, "ground_truth_distance")
                        if truth_valid
                        else None
                    ),
                    ground_truth_valid=truth_valid,
                    ground_truth_reason=str(
                        record.get("ground_truth_reason", "")
                    ),
                    ground_truth_source=_identifier(
                        record.get("ground_truth_source", ""),
                        "ground_truth_source",
                    ),
                    ground_truth_uncertainty_m=(
                        None
                        if record.get("ground_truth_uncertainty_m") is None
                        else float(record["ground_truth_uncertainty_m"])
                    ),
                    ground_truth_time_offset_ms=(
                        None
                        if record.get("ground_truth_time_offset_ms") is None
                        else float(record["ground_truth_time_offset_ms"])
                    ),
                    ground_truth_quality=_identifier(
                        record.get("ground_truth_quality", ""),
                        "ground_truth_quality",
                    ),
                    ground_truth_lever_arm_corrected=bool(
                        record.get("ground_truth_lever_arm_corrected", False)
                    ),
                    correction_mode=str(record.get("correction_mode", "")),
                    candidate_distance_m=(
                        None
                        if record.get("candidate_distance_m") is None
                        else float(record["candidate_distance_m"])
                    ),
                )
                if sample.ground_truth_source != ground_truth_source:
                    raise ValueError(
                        f"sample_ground_truth_source_mismatch:{line_number}"
                    )
                if sample.ground_truth_valid:
                    uncertainty = sample.ground_truth_uncertainty_m
                    if (
                        uncertainty is None
                        or not math.isfinite(uncertainty)
                        or uncertainty < 0.0
                    ):
                        raise ValueError(
                            f"sample_ground_truth_uncertainty_invalid:{line_number}"
                        )
                    if (
                        uncertainty
                        > MAXIMUM_TRAINING_GROUND_TRUTH_UNCERTAINTY_M
                    ):
                        raise ValueError(
                            f"sample_ground_truth_uncertainty_too_large:{line_number}"
                        )
                    if (
                        sample.ground_truth_quality
                        not in TRAINING_GROUND_TRUTH_QUALITIES
                    ):
                        raise ValueError(
                            f"sample_ground_truth_quality_invalid:{line_number}"
                        )
                    expected_quality = LOCKED_SOURCE_QUALITIES.get(
                        sample.ground_truth_source
                    )
                    if (
                        expected_quality is not None
                        and sample.ground_truth_quality != expected_quality
                    ):
                        raise ValueError(
                            "sample_ground_truth_quality_source_mismatch:"
                            f"{line_number}"
                        )
                    time_offset = sample.ground_truth_time_offset_ms
                    if (
                        time_offset is None
                        or not math.isfinite(time_offset)
                        or abs(time_offset)
                        > MAXIMUM_TRAINING_GROUND_TRUTH_TIME_OFFSET_MS
                    ):
                        raise ValueError(
                            f"sample_ground_truth_time_offset_invalid:{line_number}"
                        )
                    if not sample.ground_truth_lever_arm_corrected:
                        raise ValueError(
                            f"sample_ground_truth_lever_arm_uncorrected:{line_number}"
                        )
                if sample.ground_truth_valid:
                    samples.append(sample)
    provenance = {
        "dataset_schema_version": DATASET_SCHEMA_VERSION,
        "dataset_count": len(manifests),
        "manifests": manifests,
        "sample_sha256": sample_digests,
        "labeled_sample_count": len(samples),
    }
    return samples, provenance
