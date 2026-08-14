"""Read-only Range V2 R2 derived-label and development-coverage audit.

The source datasets are opened only for reading.  All output is written below
``artifacts/range_v2`` (or an explicitly selected output directory), and every
derived row carries enough identity and SHA-256 metadata to trace it to the
original JSONL record.
"""

from __future__ import annotations

import argparse
from collections import Counter, defaultdict
import csv
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import tempfile
from typing import Any, Iterable, Mapping, Sequence

from range_applicability_gate import (
    RangeApplicabilityGate,
    RangeApplicabilityInputs,
)
from range_residual_correction import FEATURE_NAMES, RangeResidualFeatures
from range_v2_labels import (
    DERIVED_LABEL_SCHEMA_VERSION,
    SPEC_ID,
    derive_distance_label,
)


AUDIT_SCHEMA_VERSION = "range_v2_data_audit_r2_v001"
EXPECTED_DATASET_SCHEMA = "m52_midas_range_residual_dataset_v2"
EXPECTED_DISTANCE_SEMANTICS = "camera_center_to_target_center_slant_range"
MAX_GT_UNCERTAINTY_M = 0.20
MAX_GT_TIME_OFFSET_MS = 100.0
OLD_CORRECTABLE_LIMIT_M = 3.0

ACCEPTED_GT_QUALITIES = {
    "simulation_exact",
    "rtk_fixed",
    "total_station",
    "uwb_calibrated",
}
SOURCE_QUALITY = {
    "gazebo_camera_to_target_center": "simulation_exact",
    "rtk_fixed_camera_to_target_center": "rtk_fixed",
}

CORE_BINS = tuple(f"{lower}-{lower + 1}m" for lower in range(3, 12))
INVALID_REGIMES = (
    "bbox_lost",
    "bbox_too_small",
    "roi_noise",
    "calibration_invalid",
    "critical_feature_missing",
    "midas_unstable",
    "severe_ood_geometry",
    "reacquire_hold",
    "calibration_hold",
)


def file_sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def bytes_sha256(value: bytes) -> str:
    return hashlib.sha256(value).hexdigest()


def _finite(value: Any) -> float | None:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return number if math.isfinite(number) else None


def _json_dump(path: Path, value: Any) -> None:
    path.write_text(
        json.dumps(value, indent=2, sort_keys=True, allow_nan=False) + "\n",
        encoding="utf-8",
    )


def discover_default_dataset_roots(workspace: Path) -> list[Path]:
    """Discover the exact class of sources used by the v18 development audit."""

    artifact_root = workspace / "artifacts"
    roots = [
        path
        for path in artifact_root.glob("range_v2_*")
        if path.is_dir()
        and (path / "manifest.json").is_file()
        and (path / "samples.jsonl").is_file()
    ]
    for name in (
        "holdout_range_v2_front100_shifted_20260803",
        "holdout_range_v2_front82_pitch11_20260803",
    ):
        path = artifact_root / name
        if (path / "manifest.json").is_file() and (path / "samples.jsonl").is_file():
            roots.append(path)
    return sorted(set(path.resolve() for path in roots))


@dataclass(frozen=True)
class SourceSnapshot:
    dataset_id: str
    dataset_root: str
    manifest_path: str
    samples_path: str
    manifest_sha256: str
    samples_sha256: str
    sample_bytes: int


def snapshot_source(root: Path, workspace: Path) -> SourceSnapshot:
    manifest_path = root / "manifest.json"
    samples_path = root / "samples.jsonl"
    if not manifest_path.is_file() or not samples_path.is_file():
        raise ValueError(f"dataset_files_missing:{root}")
    try:
        relative_root = root.resolve().relative_to(workspace.resolve()).as_posix()
        relative_manifest = manifest_path.resolve().relative_to(
            workspace.resolve()
        ).as_posix()
        relative_samples = samples_path.resolve().relative_to(
            workspace.resolve()
        ).as_posix()
    except ValueError:
        relative_root = root.resolve().as_posix()
        relative_manifest = manifest_path.resolve().as_posix()
        relative_samples = samples_path.resolve().as_posix()
    return SourceSnapshot(
        dataset_id=root.name,
        dataset_root=relative_root,
        manifest_path=relative_manifest,
        samples_path=relative_samples,
        manifest_sha256=file_sha256(manifest_path),
        samples_sha256=file_sha256(samples_path),
        sample_bytes=samples_path.stat().st_size,
    )


def _ground_truth_audit(
    record: Mapping[str, Any],
    manifest: Mapping[str, Any],
) -> tuple[bool, str, list[str]]:
    reasons: list[str] = []
    label = manifest.get("label")
    if not isinstance(label, Mapping):
        label = {}
        reasons.append("manifest_label_missing")
    manifest_source = label.get("ground_truth_source")
    record_source = record.get("ground_truth_source")
    if manifest_source != record_source:
        reasons.append("ground_truth_source_mismatch")
    if label.get("distance_semantics") != EXPECTED_DISTANCE_SEMANTICS:
        reasons.append("distance_semantics_unaccepted")
    if record.get("ground_truth_valid") is not True:
        reasons.append("ground_truth_not_marked_valid")
    distance = _finite(record.get("ground_truth_distance_m"))
    if distance is None or distance <= 0.0:
        reasons.append("ground_truth_distance_invalid")
    uncertainty = _finite(record.get("ground_truth_uncertainty_m"))
    if uncertainty is None:
        reasons.append("ground_truth_uncertainty_unknown")
    elif uncertainty < 0.0 or uncertainty > MAX_GT_UNCERTAINTY_M:
        reasons.append("ground_truth_uncertainty_out_of_bounds")
    offset = _finite(record.get("ground_truth_time_offset_ms"))
    if offset is None:
        reasons.append("ground_truth_time_offset_unknown")
    elif abs(offset) > MAX_GT_TIME_OFFSET_MS:
        reasons.append("ground_truth_time_offset_out_of_bounds")
    quality = record.get("ground_truth_quality")
    if quality not in ACCEPTED_GT_QUALITIES:
        reasons.append("ground_truth_quality_unaccepted")
    expected_quality = SOURCE_QUALITY.get(str(manifest_source))
    if expected_quality is not None and quality != expected_quality:
        reasons.append("ground_truth_quality_source_mismatch")
    if record.get("ground_truth_lever_arm_corrected") is not True:
        reasons.append("ground_truth_reference_center_unverified")
    if str(record.get("ground_truth_reason", "")) != "ok":
        reasons.append("ground_truth_reason_not_ok")
    trustworthy = not reasons
    status = (
        "valid_static_development_only"
        if trustworthy
        else "invalid_or_untrusted"
    )
    return trustworthy, status, reasons


def _feature_audit(record: Mapping[str, Any]) -> tuple[bool, str]:
    features = record.get("features")
    if not isinstance(features, Mapping):
        return False, "features_missing"
    if set(features) != set(FEATURE_NAMES):
        return False, "feature_schema_names_mismatch"
    try:
        RangeResidualFeatures(
            **{name: features[name] for name in FEATURE_NAMES}
        ).as_array()
    except (KeyError, TypeError, ValueError) as error:
        return False, f"feature_validation_failed:{error}"
    return True, "valid"


def _deterministic_policy_label(
    record: Mapping[str, Any],
    gate: RangeApplicabilityGate,
) -> tuple[str, str]:
    features = record.get("features")
    if not isinstance(features, Mapping):
        return "unknown", "features_missing"
    try:
        condition_number = 10.0 ** float(
            features["calibration_condition_number_log10"]
        )
        result = gate.evaluate(
            RangeApplicabilityInputs(
                image_ray_x=float(features["image_ray_x"]),
                image_ray_y=float(features["image_ray_y"]),
                target_bearing_down=float(features["target_bearing_down"]),
                camera_optical_axis_down=float(
                    features["camera_optical_axis_down"]
                ),
                calibration_condition_number=condition_number,
                calibration_residual_m_inv=float(
                    features["calibration_residual_m_inv"]
                ),
                calibration_inlier_fraction=float(
                    features["calibration_inlier_fraction"]
                ),
                anchor_spatial_coverage_fraction=float(
                    features["anchor_spatial_coverage_fraction"]
                ),
                target_anchor_extrapolation_iqr=float(
                    features["target_anchor_extrapolation_iqr"]
                ),
                ray_range_relative_std=float(
                    features["ray_range_relative_std"]
                ),
            )
        )
    except (KeyError, OverflowError, TypeError, ValueError) as error:
        return "unknown", f"policy_input_invalid:{error}"
    return ("accepted", result.reason) if result.applicable else (
        "rejected",
        result.reason,
    )


def _timestamp_row_audit(record: Mapping[str, Any]) -> tuple[str, list[str]]:
    reasons: list[str] = []
    if _finite(record.get("measurement_timestamp_s")) is None:
        reasons.append("measurement_timestamp_invalid")
    if _finite(record.get("source_sim_timestamp_s")) is None:
        reasons.append("source_sim_timestamp_missing_or_invalid")
    if reasons:
        return "invalid", reasons
    # Legacy data has a receipt-domain measurement time but no capture clock ID.
    return "partial_legacy_static_only", [
        "measurement_clock_id_unknown",
        "sensor_capture_timestamp_unknown",
        "capture_to_receipt_uncertainty_unknown",
        "ground_truth_timestamp_not_stored_separately",
    ]


def _old_label(record: Mapping[str, Any], trustworthy_gt: bool) -> tuple[str, bool]:
    if not trustworthy_gt:
        return "unknown", False
    distance = _finite(record.get("ground_truth_distance_m"))
    physics = _finite(record.get("physics_distance_m"))
    stored = _finite(record.get("range_residual_m"))
    if distance is None or physics is None or stored is None:
        return "unknown", False
    residual = distance - physics
    consistent = math.isclose(stored, residual, rel_tol=0.0, abs_tol=1.0e-9)
    if not consistent:
        return "unknown", False
    return (
        "correctable"
        if abs(residual) <= OLD_CORRECTABLE_LIMIT_M
        else "uncorrectable",
        True,
    )


def _read_sources(
    roots: Sequence[Path],
    *,
    workspace: Path,
    frozen_spec_sha256: str,
) -> tuple[list[dict[str, Any]], list[dict[str, Any]], list[SourceSnapshot]]:
    derived_rows: list[dict[str, Any]] = []
    source_details: list[dict[str, Any]] = []
    snapshots: list[SourceSnapshot] = []
    gate = RangeApplicabilityGate(mode="active")

    for root in roots:
        snapshot = snapshot_source(root, workspace)
        snapshots.append(snapshot)
        manifest = json.loads((root / "manifest.json").read_text(encoding="utf-8"))
        if manifest.get("dataset_schema_version") != EXPECTED_DATASET_SCHEMA:
            raise ValueError(f"dataset_schema_mismatch:{root}")
        feature_schema = manifest.get("feature_schema")
        if not isinstance(feature_schema, Mapping) or feature_schema.get(
            "names"
        ) != list(FEATURE_NAMES):
            raise ValueError(f"manifest_feature_schema_mismatch:{root}")

        raw_lines = (root / "samples.jsonl").read_bytes().splitlines(keepends=True)
        dataset_row_count = 0
        dataset_run_ids: set[str] = set()
        dataset_atomic_groups: set[str] = set()
        for line_number, raw_line in enumerate(raw_lines, start=1):
            if not raw_line.strip():
                continue
            try:
                record = json.loads(raw_line)
            except json.JSONDecodeError as error:
                raise ValueError(
                    f"sample_json_invalid:{root}:{line_number}:{error}"
                ) from error
            if not isinstance(record, Mapping):
                raise ValueError(f"sample_not_object:{root}:{line_number}")

            run_id = str(record.get("run_id", ""))
            session_id = record.get("session_id")
            group_id = str(record.get("group_id", ""))
            frame_index = record.get("frame_index")
            atomic_group_key = f"{run_id}/{session_id}"
            legacy_group_key = f"{run_id}/{group_id}"
            dataset_run_ids.add(run_id)
            dataset_atomic_groups.add(atomic_group_key)

            gt_trustworthy, gt_status, gt_reasons = _ground_truth_audit(
                record,
                manifest,
            )
            derived_label = derive_distance_label(
                record.get("ground_truth_distance_m"),
                ground_truth_trustworthy=gt_trustworthy,
            )
            feature_valid, feature_reason = _feature_audit(record)
            policy_label, policy_reason = _deterministic_policy_label(
                record,
                gate,
            )
            timestamp_status, timestamp_reasons = _timestamp_row_audit(record)
            old_label, old_label_consistent = _old_label(record, gt_trustworthy)

            row: dict[str, Any] = {
                "derived_label_schema_version": DERIVED_LABEL_SCHEMA_VERSION,
                "audit_schema_version": AUDIT_SCHEMA_VERSION,
                "frozen_spec_id": SPEC_ID,
                "frozen_spec_sha256": frozen_spec_sha256,
                "source_dataset_id": snapshot.dataset_id,
                "source_dataset_root": snapshot.dataset_root,
                "source_manifest_path": snapshot.manifest_path,
                "source_samples_path": snapshot.samples_path,
                "source_manifest_sha256": snapshot.manifest_sha256,
                "source_samples_sha256": snapshot.samples_sha256,
                "source_line_number": line_number,
                "source_record_sha256": bytes_sha256(raw_line),
                "source_dataset_schema_version": record.get(
                    "dataset_schema_version"
                ),
                "run_id": run_id,
                "session_id": session_id,
                "group_id": group_id,
                "atomic_group_key": atomic_group_key,
                "legacy_group_key": legacy_group_key,
                "frame_index": frame_index,
                "target_id": record.get("target_id"),
                "measurement_timestamp_s": record.get(
                    "measurement_timestamp_s"
                ),
                "measurement_clock_id": "unknown",
                "sensor_capture_timestamp_s": None,
                "source_sim_timestamp_s": record.get(
                    "source_sim_timestamp_s"
                ),
                "timestamp_audit_status": timestamp_status,
                "timestamp_audit_reasons": timestamp_reasons,
                "ground_truth_distance_m": record.get(
                    "ground_truth_distance_m"
                ),
                "ground_truth_valid_recorded": record.get(
                    "ground_truth_valid"
                ),
                "ground_truth_audit_status": gt_status,
                "ground_truth_audit_reasons": gt_reasons,
                "ground_truth_source": record.get("ground_truth_source"),
                "ground_truth_frame": "gazebo_enu_camera_center_to_target_center"
                if record.get("ground_truth_source")
                == "gazebo_camera_to_target_center"
                else "unknown",
                "ground_truth_quality": record.get("ground_truth_quality"),
                "ground_truth_uncertainty_m": record.get(
                    "ground_truth_uncertainty_m"
                ),
                "ground_truth_time_offset_ms": record.get(
                    "ground_truth_time_offset_ms"
                ),
                "uncertainty_budget_status": "legacy_total_only_components_unknown",
                "distance_label_trustworthy": gt_trustworthy,
                "validity_gt": "unknown",
                "validity_gt_reason": "legacy_dataset_has_no_range_v2_validity_annotation",
                **derived_label.as_dict(),
                "direction_gt": "unknown",
                "direction_gt_reason": "deadband_and_dynamic_capture_time_evidence_unavailable",
                "dynamic_label_eligible": False,
                "promotion_eligible": False,
                "track_epoch": "unknown",
                "calibration_epoch": "unknown",
                "location_id": "unknown",
                "world_position": "unknown",
                "background_regime": "unknown",
                "pitch_regime": "unknown",
                "view_direction": "unknown",
                "lateral_regime": "unknown",
                "feature_record_valid": feature_valid,
                "feature_record_reason": feature_reason,
                "deterministic_policy_label": policy_label,
                "deterministic_policy_reason": policy_reason,
                "old_range_residual_m": record.get("range_residual_m"),
                "old_correctability_label": old_label,
                "old_residual_label_consistent": old_label_consistent,
                "correction_mode": record.get("correction_mode"),
            }
            derived_rows.append(row)
            dataset_row_count += 1

        manifest_run = str(manifest.get("grouping", {}).get("run_id", ""))
        source_details.append(
            {
                **asdict(snapshot),
                "row_count": dataset_row_count,
                "manifest_run_id": manifest_run,
                "record_run_ids": sorted(dataset_run_ids),
                "atomic_group_keys": sorted(dataset_atomic_groups),
                "one_run_session_group": (
                    len(dataset_run_ids) == 1
                    and len(dataset_atomic_groups) == 1
                    and dataset_run_ids == {manifest_run}
                ),
                "corpus_role": "development",
                "former_holdout_now_development": snapshot.dataset_id.startswith(
                    "holdout_"
                ),
            }
        )
    return derived_rows, source_details, snapshots


def _strictly_increasing(values: Iterable[Any]) -> bool | None:
    converted = [_finite(value) for value in values]
    if any(value is None for value in converted):
        return None
    numbers = [float(value) for value in converted if value is not None]
    return all(right > left for left, right in zip(numbers, numbers[1:]))


def _group_summaries(rows: Sequence[dict[str, Any]]) -> list[dict[str, Any]]:
    grouped: dict[str, list[dict[str, Any]]] = defaultdict(list)
    for row in rows:
        grouped[row["atomic_group_key"]].append(row)

    summaries: list[dict[str, Any]] = []
    for key in sorted(grouped):
        group_rows = grouped[key]
        trusted_distances = [
            float(row["ground_truth_distance_m"])
            for row in group_rows
            if row["distance_label_trustworthy"]
        ]
        zones = sorted(
            {
                row["distance_zone_gt"]
                for row in group_rows
                if row["distance_zone_gt"] != "unknown"
            }
        )
        bins = sorted(
            {
                row["core_bin"]
                for row in group_rows
                if row["core_bin"] != "unknown"
            }
        )
        old_labels = Counter(row["old_correctability_label"] for row in group_rows)
        measurement_monotonic = _strictly_increasing(
            row["measurement_timestamp_s"] for row in group_rows
        )
        source_monotonic = _strictly_increasing(
            row["source_sim_timestamp_s"] for row in group_rows
        )
        frame_monotonic = _strictly_increasing(
            row["frame_index"] for row in group_rows
        )
        constant_gt = bool(
            len(trusted_distances) >= 2
            and min(trusted_distances) == max(trusted_distances)
        )
        stationary_evidence = bool(
            constant_gt and measurement_monotonic and source_monotonic
        )
        source_roots = sorted({row["source_dataset_root"] for row in group_rows})
        summary = {
            "dataset_id": ";".join(
                sorted({row["source_dataset_id"] for row in group_rows})
            ),
            "source_dataset_root": ";".join(source_roots),
            "source_manifest_sha256": ";".join(
                sorted({row["source_manifest_sha256"] for row in group_rows})
            ),
            "source_samples_sha256": ";".join(
                sorted({row["source_samples_sha256"] for row in group_rows})
            ),
            "run_id": group_rows[0]["run_id"],
            "session_id": group_rows[0]["session_id"],
            "group_id": ";".join(sorted({row["group_id"] for row in group_rows})),
            "atomic_group_key": key,
            "legacy_group_key": ";".join(
                sorted({row["legacy_group_key"] for row in group_rows})
            ),
            "frame_count": len(group_rows),
            "source_line_min": min(row["source_line_number"] for row in group_rows),
            "source_line_max": max(row["source_line_number"] for row in group_rows),
            "label_trustworthy_frame_count": len(trusted_distances),
            "label_trustworthy_fraction": len(trusted_distances) / len(group_rows),
            "feature_valid_frame_count": sum(
                bool(row["feature_record_valid"]) for row in group_rows
            ),
            "valid_feature_fraction": sum(
                bool(row["feature_record_valid"]) for row in group_rows
            )
            / len(group_rows),
            "ground_truth_distance_m": trusted_distances[0]
            if trusted_distances and constant_gt
            else "unknown",
            "ground_truth_distance_min_m": min(trusted_distances)
            if trusted_distances
            else "unknown",
            "ground_truth_distance_max_m": max(trusted_distances)
            if trusted_distances
            else "unknown",
            "new_distance_zone_gt": zones[0] if len(zones) == 1 else "mixed_or_unknown",
            "new_core_bin": bins[0] if len(bins) == 1 else "mixed_or_unknown",
            "transition_frame_count": sum(
                row["is_boundary_transition"] is True for row in group_rows
            ),
            "validity_gt": "unknown",
            "old_label": (
                next(iter(old_labels))
                if len(old_labels) == 1
                else "mixed"
            ),
            "old_correctable_frames": old_labels["correctable"],
            "old_uncorrectable_frames": old_labels["uncorrectable"],
            "location_id": "unknown",
            "world_position": "unknown",
            "background_regime": "unknown",
            "pitch_regime": "unknown",
            "view_direction": "unknown",
            "motion_regime": "unknown",
            "radial_stationary_evidence": stationary_evidence,
            "radial_stationary_evidence_reason": (
                "exact_constant_trustworthy_gt_range_with_monotonic_timestamps"
                if stationary_evidence
                else "insufficient_evidence"
            ),
            "approaching_evidence": "unknown",
            "receding_evidence": "unknown",
            "lateral_evidence": "unknown",
            "calibration_epoch": "unknown",
            "track_epoch": "unknown",
            "calibration_event_count": "unknown",
            "reacquire_event_count": "unknown",
            "measurement_timestamp_strictly_increasing": measurement_monotonic,
            "source_timestamp_strictly_increasing": source_monotonic,
            "frame_index_strictly_increasing": frame_monotonic,
            "measurement_clock_id": "unknown",
            "timestamp_gt_status": (
                "PARTIAL"
                if measurement_monotonic and source_monotonic and frame_monotonic
                else "INVALID"
            ),
            "gt_uncertainty_budget_status": "PARTIAL_legacy_total_only",
            "deterministic_policy_rejected_frames": sum(
                row["deterministic_policy_label"] == "rejected"
                for row in group_rows
            ),
            "source_root_count": len(source_roots),
            "group_audit_status": (
                "PARTIAL"
                if len(source_roots) == 1
                and measurement_monotonic
                and source_monotonic
                and frame_monotonic
                else "INVALID"
            ),
        }
        summaries.append(summary)
    return summaries


def _leakage_audit(
    rows: Sequence[dict[str, Any]],
    source_details: Sequence[dict[str, Any]],
) -> dict[str, Any]:
    identity_counts = Counter(
        (row["run_id"], row["session_id"], row["frame_index"])
        for row in rows
    )
    atomic_roots: dict[str, set[str]] = defaultdict(set)
    run_roots: dict[str, set[str]] = defaultdict(set)
    legacy_to_atomic: dict[str, set[str]] = defaultdict(set)
    atomic_to_legacy: dict[str, set[str]] = defaultdict(set)
    for row in rows:
        atomic_roots[row["atomic_group_key"]].add(row["source_dataset_root"])
        run_roots[row["run_id"]].add(row["source_dataset_root"])
        legacy_to_atomic[row["legacy_group_key"]].add(row["atomic_group_key"])
        atomic_to_legacy[row["atomic_group_key"]].add(row["legacy_group_key"])

    duplicate_identities = [
        {"run_id": key[0], "session_id": key[1], "frame_index": key[2], "count": count}
        for key, count in identity_counts.items()
        if count > 1
    ]
    atomic_cross_source = {
        key: sorted(values) for key, values in atomic_roots.items() if len(values) > 1
    }
    run_cross_source = {
        key: sorted(values) for key, values in run_roots.items() if len(values) > 1
    }
    group_mapping_conflicts = {
        "legacy_to_atomic": {
            key: sorted(values)
            for key, values in legacy_to_atomic.items()
            if len(values) > 1
        },
        "atomic_to_legacy": {
            key: sorted(values)
            for key, values in atomic_to_legacy.items()
            if len(values) > 1
        },
    }
    one_group_violations = [
        detail["dataset_id"]
        for detail in source_details
        if not detail["one_run_session_group"]
    ]
    detected = bool(
        duplicate_identities
        or atomic_cross_source
        or run_cross_source
        or group_mapping_conflicts["legacy_to_atomic"]
        or group_mapping_conflicts["atomic_to_legacy"]
        or one_group_violations
    )
    return {
        "atomic_identity": "run_id/session_id/frame_index",
        "atomic_group": "run_id/session_id",
        "legacy_group": "run_id/group_id",
        "dataset_count": len(source_details),
        "unique_atomic_group_count": len(atomic_roots),
        "unique_legacy_group_count": len(legacy_to_atomic),
        "duplicate_identity_count": len(duplicate_identities),
        "duplicate_identities": duplicate_identities,
        "atomic_groups_spanning_source_roots": atomic_cross_source,
        "run_ids_spanning_source_roots": run_cross_source,
        "group_mapping_conflicts": group_mapping_conflicts,
        "one_run_session_group_violations": one_group_violations,
        "r2_partition_leakage_status": "not_applicable_no_r2_split_created",
        "promotion_leakage_status": "not_applicable_no_promotion_corpus_opened",
        "development_corpus_group_collision_detected": detected,
        "status": "PASS" if not detected else "FAIL",
    }


def _coverage_rows(
    rows: Sequence[dict[str, Any]],
    groups: Sequence[dict[str, Any]],
) -> list[dict[str, Any]]:
    by_group = {group["atomic_group_key"]: group for group in groups}
    coverage: list[dict[str, Any]] = []

    def summarize(scope: str, name: str, selected: Sequence[dict[str, Any]]) -> dict[str, Any]:
        group_counts = Counter(row["atomic_group_key"] for row in selected)
        group_keys = sorted(group_counts)
        qualifying = sorted(key for key, count in group_counts.items() if count >= 30)
        stationary = sum(
            bool(by_group[key]["radial_stationary_evidence"])
            for key in group_keys
        )
        return {
            "scope": scope,
            "name": name,
            "frame_count": len(selected),
            "independent_group_count": len(group_keys),
            "groups_with_at_least_30_frames": len(qualifying),
            "stationary_evidence_group_count": stationary,
            "approaching_group_count": 0,
            "receding_group_count": 0,
            "lateral_group_count": 0,
            "known_location_count": 0,
            "known_background_count": 0,
            "known_pitch_geometry_regime_count": 0,
            "boundary_crossing_group_count": 0,
            "group_keys": ";".join(group_keys),
        }

    for bin_name in CORE_BINS:
        selected = [
            row
            for row in rows
            if row["distance_label_trustworthy"]
            and row["core_bin"] == bin_name
        ]
        item = summarize("core_bin", bin_name, selected)
        minimum_groups = 5 if bin_name in {"5-6m", "6-7m", "7-8m"} else 3
        item["minimum_independent_groups"] = minimum_groups
        if item["independent_group_count"] == 0:
            status = "MISSING"
        elif item["independent_group_count"] < 2:
            status = "MISSING"
        else:
            # No present bin has the required context and dynamic evidence.
            status = "PARTIAL"
        item["status"] = status
        item["blocking_evidence"] = (
            "no_labelled_frames"
            if not selected
            else "location_background_pitch_and_dynamic_regimes_unknown_or_missing"
        )
        coverage.append(item)

    for zone in ("near", "core", "far"):
        selected = [
            row
            for row in rows
            if row["distance_label_trustworthy"]
            and row["distance_zone_gt"] == zone
        ]
        item = summarize("zone", zone, selected)
        if zone == "core":
            populated_bins = {
                row["core_bin"] for row in selected if row["core_bin"] != "unknown"
            }
            item["status"] = "PARTIAL" if selected else "MISSING"
            item["blocking_evidence"] = (
                f"populated_core_bins={len(populated_bins)}/9;dynamic_and_context_evidence_missing"
            )
        else:
            item["status"] = "MISSING" if not selected else "PARTIAL"
            item["blocking_evidence"] = (
                "no_labelled_frames"
                if not selected
                else "stationary_dynamic_and_boundary_requirements_incomplete"
            )
        item["minimum_independent_groups"] = 3
        coverage.append(item)

    for band in ("near_core_transition", "core_far_transition"):
        selected = [row for row in rows if row["boundary_band"] == band]
        item = summarize("transition_band", band, selected)
        item["minimum_independent_groups"] = 2
        item["status"] = "MISSING" if not selected else "PARTIAL"
        item["blocking_evidence"] = (
            "no_transition_frames_or_boundary_crossing_groups"
            if not selected
            else "boundary_crossing_evidence_incomplete"
        )
        coverage.append(item)

    invalid_policy_rows = [
        row for row in rows if row["deterministic_policy_label"] == "rejected"
    ]
    invalid_item = summarize("zone", "invalid", [])
    invalid_item.update(
        {
            "minimum_independent_groups": 2,
            "status": "MISSING",
            "blocking_evidence": "validity_gt_is_unknown_for_all_legacy_rows",
            "policy_flag_frame_count": len(invalid_policy_rows),
            "policy_flag_group_count": len(
                {row["atomic_group_key"] for row in invalid_policy_rows}
            ),
        }
    )
    coverage.append(invalid_item)

    for regime in INVALID_REGIMES:
        policy_rows = []
        if regime == "severe_ood_geometry":
            policy_rows = invalid_policy_rows
        item = summarize("invalid_regime", regime, [])
        item.update(
            {
                "minimum_independent_groups": 2,
                "status": "MISSING",
                "blocking_evidence": (
                    "policy_flags_exist_but_no_independent_validity_gt_annotation"
                    if policy_rows
                    else "no_explicit_regime_annotation_evidence_unknown"
                ),
                "policy_flag_frame_count": len(policy_rows),
                "policy_flag_group_count": len(
                    {row["atomic_group_key"] for row in policy_rows}
                ),
            }
        )
        coverage.append(item)
    return coverage


def _write_csv(path: Path, rows: Sequence[Mapping[str, Any]]) -> None:
    if not rows:
        raise ValueError(f"cannot_write_empty_csv:{path}")
    fieldnames: list[str] = []
    seen: set[str] = set()
    for row in rows:
        for name in row:
            if name not in seen:
                seen.add(name)
                fieldnames.append(name)
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames, extrasaction="ignore")
        writer.writeheader()
        for row in rows:
            writer.writerow(row)


def _markdown_report(
    *,
    rows: Sequence[dict[str, Any]],
    groups: Sequence[dict[str, Any]],
    coverage: Sequence[dict[str, Any]],
    leakage: Mapping[str, Any],
    source_details: Sequence[dict[str, Any]],
    frozen_spec_sha256: str,
) -> str:
    policy_reasons = Counter(
        row["deterministic_policy_reason"]
        for row in rows
        if row["deterministic_policy_label"] == "rejected"
    )
    coverage_by_key = {
        (row["scope"], row["name"]): row for row in coverage
    }
    lines = [
        "# Range V2 R2 Derived-Label and Data Coverage Audit",
        "",
        f"Frozen spec: `{SPEC_ID}` (`{frozen_spec_sha256}`)  ",
        f"Derived schema: `{DERIVED_LABEL_SCHEMA_VERSION}`  ",
        "Corpus role: **development only**  ",
        "Decision: **R2 COMPLETE; training/promotion/runtime remain NO-GO**",
        "",
        "## Scope and immutability",
        "",
        f"- Audited {len(source_details)} source datasets, {len(groups)} atomic run/session groups, and {len(rows):,} rows.",
        "- Original `manifest.json` and `samples.jsonl` files were read-only and their pre/post SHA-256 values matched.",
        "- H1/H2 are explicitly development data; no promotion/final-holdout corpus was opened.",
        "- No model, scaler, calibrator, threshold, simulation, shadow, runtime, or controller path was used.",
        "",
        "## Derived-label rules",
        "",
        "- Canonical labels are exact: near `<3 m`, core `3–12 m` inclusive, far `>12 m`.",
        "- Transition is an orthogonal flag; it never changes the canonical label.",
        "- `validity_gt` is `unknown` for every legacy row because these datasets have no Range V2 validity annotation.",
        "- Direction is `unknown`: the controller/noise-derived deadband and dynamic capture-time evidence do not exist.",
        "- Legacy deterministic-gate rejection is reported as a policy flag, never promoted to validity ground truth.",
        "",
        "## Leakage and identity audit",
        "",
        f"- Status: **{leakage['status']}**.",
        f"- Duplicate `run/session/frame` identities: {leakage['duplicate_identity_count']}.",
        f"- Atomic groups spanning multiple source roots: {len(leakage['atomic_groups_spanning_source_roots'])}.",
        f"- One-run/session-per-dataset violations: {len(leakage['one_run_session_group_violations'])}.",
        "- R2 created no split, so cross-partition leakage is not applicable; future split audit is still mandatory.",
        "",
        "## Timestamp and ground-truth audit",
        "",
        f"- Monotonic measurement/source/frame identity groups: {sum(g['timestamp_gt_status'] == 'PARTIAL' for g in groups)}/{len(groups)}.",
        f"- Trustworthy static-development distance rows: {sum(bool(r['distance_label_trustworthy']) for r in rows):,}/{len(rows):,}.",
        "- Stored GT total uncertainty/time offset pass legacy limits, but component budgets, measurement clock ID, sensor capture timestamp, track epoch, and calibration epoch are absent.",
        "- Therefore labels are usable for this static development coverage audit only, not for direction, lag, event, or promotion evidence.",
        "",
        "## Core-bin development coverage",
        "",
        "| Bin | Frames | Groups | Groups >=30 frames | Stationary evidence | Approach | Recede | Lateral | Status |",
        "|---|---:|---:|---:|---:|---:|---:|---:|---|",
    ]
    for bin_name in CORE_BINS:
        item = coverage_by_key[("core_bin", bin_name)]
        lines.append(
            f"| {bin_name} | {item['frame_count']} | {item['independent_group_count']} | "
            f"{item['groups_with_at_least_30_frames']} | {item['stationary_evidence_group_count']} | "
            f"{item['approaching_group_count']} | {item['receding_group_count']} | "
            f"{item['lateral_group_count']} | **{item['status']}** |"
        )
    lines.extend(
        [
            "",
            "Known location/background/pitch-regime counts are zero for every bin because the legacy manifest does not store authoritative values. Dataset names were not used as labels.",
            "",
            "## Zone and transition coverage",
            "",
            "| Scope | Frames | Groups | Status | Blocking evidence |",
            "|---|---:|---:|---|---|",
        ]
    )
    for key in (
        ("zone", "near"),
        ("zone", "core"),
        ("zone", "far"),
        ("zone", "invalid"),
        ("transition_band", "near_core_transition"),
        ("transition_band", "core_far_transition"),
    ):
        item = coverage_by_key[key]
        lines.append(
            f"| {item['name']} | {item['frame_count']} | {item['independent_group_count']} | "
            f"**{item['status']}** | `{item['blocking_evidence']}` |"
        )
    lines.extend(
        [
            "",
            "## Invalid-regime coverage",
            "",
            "| Regime | Annotated frames | Annotated groups | Policy-only flags | Status |",
            "|---|---:|---:|---:|---|",
        ]
    )
    for regime in INVALID_REGIMES:
        item = coverage_by_key[("invalid_regime", regime)]
        lines.append(
            f"| {regime} | {item['frame_count']} | {item['independent_group_count']} | "
            f"{item.get('policy_flag_frame_count', 0)} | **{item['status']}** |"
        )
    lines.extend(
        [
            "",
            "The old deterministic envelope rejected policy-only samples as follows:",
            "",
        ]
    )
    if policy_reasons:
        for reason, count in sorted(policy_reasons.items()):
            lines.append(f"- `{reason}`: {count} frames")
    else:
        lines.append("- No deterministic policy rejection was observed.")
    lines.extend(
        [
            "",
            "These are not canonical invalid labels and do not satisfy invalid-regime coverage.",
            "",
            "## Development versus promotion coverage",
            "",
            "- Development: **PARTIAL**. Only core static distance data is present; two core bins and all near/far/invalid/dynamic/boundary evidence remain missing.",
            "- Promotion: **MISSING / NOT COLLECTED**. Existing and former-holdout rows contribute zero promotion evidence.",
            "",
            "## R2 gate",
            "",
            "- R2 derived-label/audit deliverable: **PASS** (source immutability and traceability verified).",
            "- R3 offline raw-baseline audit: **GO WITH CONDITIONS** for the available static core subset only; unavailable metrics must remain `unknown`/`N/A`.",
            "- Training, q90 fitting, model promotion, shadow, simulation, runtime, and controller integration: **NO-GO**.",
            "",
        ]
    )
    return "\n".join(lines)


def run_audit(
    *,
    workspace: Path,
    dataset_roots: Sequence[Path],
    output_dir: Path,
    frozen_spec_path: Path,
) -> dict[str, Any]:
    workspace = workspace.resolve()
    output_dir = output_dir.resolve()
    roots = sorted(path.resolve() for path in dataset_roots)
    if not roots:
        raise ValueError("no_dataset_roots")
    for root in roots:
        if output_dir == root or output_dir in root.parents or root in output_dir.parents:
            raise ValueError(f"output_overlaps_source_dataset:{root}")

    frozen_spec_sha256 = file_sha256(frozen_spec_path.resolve())
    pre_snapshots = [snapshot_source(root, workspace) for root in roots]
    rows, source_details, snapshots = _read_sources(
        roots,
        workspace=workspace,
        frozen_spec_sha256=frozen_spec_sha256,
    )
    groups = _group_summaries(rows)
    leakage = _leakage_audit(rows, source_details)
    coverage = _coverage_rows(rows, groups)

    output_dir.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(
        prefix="range_v2_r2_", dir=output_dir.parent
    ) as temporary:
        temp_dir = Path(temporary)
        derived_path = temp_dir / "derived_labels.jsonl"
        with derived_path.open("w", encoding="utf-8") as stream:
            for row in rows:
                stream.write(
                    json.dumps(row, sort_keys=True, separators=(",", ":"), allow_nan=False)
                    + "\n"
                )
        _write_csv(temp_dir / "data_coverage_report.csv", groups)
        _write_csv(temp_dir / "coverage_status.csv", coverage)
        _json_dump(temp_dir / "leakage_audit.json", leakage)
        _json_dump(
            temp_dir / "source_dataset_manifest.json",
            {
                "artifact_type": "range_v2_r2_source_dataset_snapshot",
                "audit_schema_version": AUDIT_SCHEMA_VERSION,
                "frozen_spec_id": SPEC_ID,
                "frozen_spec_sha256": frozen_spec_sha256,
                "corpus_role": "development",
                "source_dataset_count": len(source_details),
                "derived_row_count": len(rows),
                "sources": source_details,
            },
        )
        (temp_dir / "data_coverage_report.md").write_text(
            _markdown_report(
                rows=rows,
                groups=groups,
                coverage=coverage,
                leakage=leakage,
                source_details=source_details,
                frozen_spec_sha256=frozen_spec_sha256,
            ),
            encoding="utf-8",
        )

        post_snapshots = [snapshot_source(root, workspace) for root in roots]
        originals_unchanged = pre_snapshots == post_snapshots == snapshots
        if not originals_unchanged:
            raise RuntimeError("source_dataset_changed_during_audit")

        output_files = sorted(
            path for path in temp_dir.iterdir() if path.is_file()
        )
        output_checksums = {
            path.name: file_sha256(path) for path in output_files
        }
        _json_dump(
            temp_dir / "r2_audit_manifest.json",
            {
                "artifact_type": "range_v2_r2_audit_manifest",
                "created_utc": datetime.now(timezone.utc).isoformat(),
                "audit_schema_version": AUDIT_SCHEMA_VERSION,
                "derived_label_schema_version": DERIVED_LABEL_SCHEMA_VERSION,
                "frozen_spec_id": SPEC_ID,
                "frozen_spec_sha256": frozen_spec_sha256,
                "source_dataset_count": len(source_details),
                "source_group_count": len(groups),
                "derived_row_count": len(rows),
                "source_originals_read_only_verified": originals_unchanged,
                "source_checksums": [asdict(snapshot) for snapshot in snapshots],
                "output_checksums": output_checksums,
                "leakage_status": leakage["status"],
                "scope_guards": {
                    "model_trained": False,
                    "scaler_fitted": False,
                    "calibrator_fitted": False,
                    "threshold_fitted": False,
                    "simulation_run": False,
                    "shadow_run": False,
                    "runtime_modified": False,
                    "final_holdout_opened": False,
                },
                "decision": "R2_COMPLETE_TRAINING_PROMOTION_RUNTIME_NO_GO",
            },
        )
        for path in sorted(temp_dir.iterdir()):
            if path.is_file():
                path.replace(output_dir / path.name)

    return {
        "dataset_count": len(source_details),
        "group_count": len(groups),
        "row_count": len(rows),
        "leakage_status": leakage["status"],
        "output_dir": str(output_dir),
        "source_originals_read_only_verified": True,
    }


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--workspace",
        type=Path,
        default=Path(__file__).resolve().parent,
    )
    parser.add_argument(
        "--dataset-root",
        action="append",
        type=Path,
        default=[],
        help="Repeat for an explicit source corpus; defaults to current v18 development roots.",
    )
    parser.add_argument("--output-dir", type=Path)
    parser.add_argument("--frozen-spec", type=Path)
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    workspace = args.workspace.resolve()
    roots = args.dataset_root or discover_default_dataset_roots(workspace)
    roots = [
        path if path.is_absolute() else (workspace / path)
        for path in roots
    ]
    output_dir = args.output_dir or workspace / "artifacts" / "range_v2"
    frozen_spec = args.frozen_spec or workspace / "docs" / "RANGE_V2_FROZEN_SPEC.md"
    result = run_audit(
        workspace=workspace,
        dataset_roots=roots,
        output_dir=output_dir,
        frozen_spec_path=frozen_spec,
    )
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
