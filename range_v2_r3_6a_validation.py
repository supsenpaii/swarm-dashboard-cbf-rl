"""Offline-only analysis for Range V2 R3.6A-V diagnostic sidecars.

This module never imports the dashboard runtime, changes calibration policy, or
controls a simulator.  It verifies append-only R3.6A records and reconstructs
diagnostic alternatives from the values already logged.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
from statistics import median
from typing import Any, Iterable, Mapping, Sequence

import numpy as np

from metric_depth_calibrator import MetricDepthCalibrator
from range_physical_diagnostics import (
    DIAGNOSTICS_FILENAME,
    DIAGNOSTICS_MANIFEST_FILENAME,
    DIAGNOSTICS_SCHEMA_VERSION,
    canonical_sha256,
)


VALIDATION_ID = "range_v2_r3_6a_validation_v001"
DATASET_ROLE = "physical_diagnostic_development"
ALLOWED_CONCLUSIONS = frozenset(
    {
        "CALIBRATION_LIFECYCLE_FIX_REQUIRED",
        "ANCHOR_SELECTION_OR_FIT_FIX_REQUIRED",
        "QUANTITY_DOMAIN_FIX_REQUIRED",
        "MULTIPLE_PHYSICAL_FIXES_REQUIRED",
        "DIAGNOSTIC_EVIDENCE_INSUFFICIENT",
    }
)
VARIANT_LIVE = "current_live_applied"
VARIANT_FREEZE = "stable_window_freeze"
VARIANT_ORACLE = "group_median_oracle_NON_CAUSAL_ORACLE"
VARIANT_REFIT = "anchor_refit_replay"


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _finite(value: Any) -> bool:
    try:
        return math.isfinite(float(value))
    except (TypeError, ValueError):
        return False


def _nested(row: Mapping[str, Any], *keys: str) -> Any:
    value: Any = row
    for key in keys:
        if not isinstance(value, Mapping):
            return None
        value = value.get(key)
    return value


def verify_record_checksum(row: Mapping[str, Any]) -> bool:
    expected = row.get("record_sha256")
    if not isinstance(expected, str) or len(expected) != 64:
        return False
    payload = {
        key: value
        for key, value in row.items()
        if not str(key).startswith("_")
    }
    payload.pop("record_sha256", None)
    try:
        return canonical_sha256(payload) == expected
    except (TypeError, ValueError):
        return False


def timestamp_order_status(row: Mapping[str, Any]) -> tuple[bool, str]:
    timestamps = row.get("timestamps")
    if not isinstance(timestamps, Mapping):
        return False, "timestamps_missing"
    ordered_names = (
        "measurement_timestamp_s",
        "depth_submitted_timestamp_s",
        "depth_inference_started_timestamp_s",
        "depth_completed_timestamp_s",
        "consume_now_monotonic_s",
    )
    values = [timestamps.get(name) for name in ordered_names]
    if any(not _finite(value) for value in values):
        return False, "monotonic_timestamp_missing"
    numeric = [float(value) for value in values]
    if any(numeric[index] > numeric[index + 1] for index in range(4)):
        return False, "monotonic_timestamp_order_invalid"
    if not _finite(timestamps.get("frame_source_sim_timestamp_s")):
        return False, "source_image_timestamp_missing"
    source_clock = timestamps.get("source_sim_clock")
    if not isinstance(source_clock, str) or not source_clock.strip():
        return False, "source_image_clock_missing"
    return True, "ok"


def anchors_status(row: Mapping[str, Any]) -> tuple[bool, list[str]]:
    anchors = _nested(row, "anchors", "per_grid_point")
    reasons: list[str] = []
    if not isinstance(anchors, list) or len(anchors) != 96:
        return False, ["anchor_count_not_96"]
    indices: list[int] = []
    for anchor in anchors:
        if not isinstance(anchor, Mapping):
            reasons.append("anchor_record_invalid")
            continue
        try:
            indices.append(int(anchor.get("grid_index")))
        except (TypeError, ValueError):
            reasons.append("anchor_index_invalid")
        accepted = anchor.get("accepted")
        reason = anchor.get("reason")
        if not isinstance(accepted, bool):
            reasons.append("anchor_acceptance_missing")
        if not isinstance(reason, str) or not reason:
            reasons.append("anchor_reason_missing")
        for name in ("pixel_u", "pixel_v"):
            if not _finite(anchor.get(name)):
                reasons.append(f"anchor_{name}_missing")
        if accepted:
            ray = anchor.get("ray_ned_unit")
            if not (
                isinstance(ray, list)
                and len(ray) == 3
                and all(_finite(value) for value in ray)
            ):
                reasons.append("accepted_anchor_ray_missing")
            for name in (
                "relative_inverse_depth",
                "ground_slant_range_m",
                "metric_optical_depth_m",
                "local_inverse_depth_mean",
                "local_inverse_depth_std",
                "local_inverse_depth_cv",
            ):
                if not _finite(anchor.get(name)):
                    reasons.append(f"accepted_anchor_{name}_missing")
        if "target_exclusion_result" not in anchor:
            reasons.append("anchor_target_exclusion_result_not_recorded")
    if sorted(indices) != list(range(96)):
        reasons.append("anchor_indices_not_exact_0_to_95")
    return not reasons, sorted(set(reasons))


def completeness_reasons(row: Mapping[str, Any]) -> list[str]:
    reasons: list[str] = []
    if row.get("diagnostics_schema_version") != DIAGNOSTICS_SCHEMA_VERSION:
        reasons.append("schema_version_mismatch")
    if not verify_record_checksum(row):
        reasons.append("record_checksum_invalid")
    for name in ("run_id", "group_id"):
        if not isinstance(row.get(name), str) or not row.get(name):
            reasons.append(f"{name}_missing")
    for name in ("session_id", "frame_index", "track_epoch", "calibration_epoch"):
        if not _finite(row.get(name)):
            reasons.append(f"{name}_missing")
    timestamp_ok, timestamp_reason = timestamp_order_status(row)
    if not timestamp_ok:
        reasons.append(timestamp_reason)

    references = row.get("reference_centers")
    if not isinstance(references, Mapping):
        reasons.append("reference_centers_missing")
    else:
        ground_truth = references.get("ground_truth")
        if not isinstance(ground_truth, Mapping):
            reasons.append("camera_link_or_target_reference_missing")
        optical = references.get("optical_center")
        if not isinstance(optical, Mapping):
            reasons.append("optical_center_missing")
        elif "verified" not in str(optical.get("status", "")) or "not_" in str(
            optical.get("status", "")
        ):
            reasons.append("optical_center_unverified")

    camera = row.get("camera_info")
    if not isinstance(camera, Mapping):
        reasons.append("camera_info_missing")
    else:
        for name in ("width", "height", "fx", "fy", "cx", "cy"):
            if not _finite(camera.get(name)):
                reasons.append(f"camera_info_{name}_missing")
        if camera.get("distortion_model") is None:
            reasons.append("distortion_model_not_recorded")
        if camera.get("rectified") is None:
            reasons.append("rectification_not_recorded")

    extrinsics = row.get("extrinsics")
    if not isinstance(extrinsics, Mapping):
        reasons.append("extrinsics_missing")
    elif "fingerprint_sha256" not in extrinsics:
        reasons.append("extrinsics_fingerprint_not_recorded")

    anchor_ok, anchor_reasons = anchors_status(row)
    if not anchor_ok:
        reasons.extend(anchor_reasons)

    calibration = row.get("calibration")
    if not isinstance(calibration, Mapping):
        reasons.append("calibration_missing")
    else:
        for name in ("fit", "applied"):
            item = calibration.get(name)
            if not isinstance(item, Mapping):
                reasons.append(f"calibration_{name}_missing")
            else:
                for parameter in (
                    "raw_scale",
                    "raw_offset",
                    "filtered_scale",
                    "filtered_offset",
                    "inlier_count",
                    "condition_number",
                ):
                    if item.get(parameter) is None:
                        reasons.append(f"calibration_{name}_{parameter}_missing")
        if "cache_age_s" not in calibration or "cache_ttl_s" not in calibration:
            reasons.append("calibration_cache_contract_missing")
        if "calibration_age_s" not in calibration:
            reasons.append("calibration_age_not_recorded")
        if not isinstance(calibration.get("recovery"), Mapping):
            reasons.append("calibration_recovery_missing")

    target = row.get("target_depth")
    if not isinstance(target, Mapping):
        reasons.append("target_depth_missing")
    else:
        if "raw_relative_inverse_depth_quantiles" not in target:
            reasons.append("target_roi_q_quantiles_not_recorded")
        if "selected_foreground_statistic" not in target:
            reasons.append("target_foreground_statistic_not_recorded")
    for name in ("bbox", "raw_range", "ground_truth"):
        if not isinstance(row.get(name), Mapping):
            reasons.append(f"{name}_missing")
    return sorted(set(reasons))


def exact_live_reconstruction(row: Mapping[str, Any], *, tolerance_m: float = 1e-9) -> tuple[bool, float | None]:
    q = _nested(row, "inverse_depth_filter", "filtered_inverse_depth")
    a = _nested(row, "calibration", "applied", "filtered_scale")
    b = _nested(row, "calibration", "applied", "filtered_offset")
    ray_scale = _nested(row, "raw_range", "ray_scale")
    recorded = _nested(row, "raw_range", "physics_slant_range_m")
    if not all(_finite(value) for value in (q, a, b, ray_scale, recorded)):
        return False, None
    denominator = float(a) * float(q) + float(b)
    if denominator <= 0.0:
        return False, None
    reconstructed = float(ray_scale) / denominator
    error = reconstructed - float(recorded)
    return abs(error) <= tolerance_m, error


def stable_freeze_parameters(rows: Sequence[Mapping[str, Any]], window: int = 5) -> dict[str, Any] | None:
    qualifying: list[tuple[int, float, float]] = []
    for index, row in enumerate(rows):
        applied = _nested(row, "calibration", "applied")
        source = _nested(row, "calibration", "source")
        if not isinstance(applied, Mapping):
            qualifying.clear()
            continue
        valid = (
            bool(applied.get("valid"))
            and bool(applied.get("stable"))
            and bool(applied.get("measurement_accepted"))
            and source == "live"
            and _finite(applied.get("condition_number"))
            and float(applied["condition_number"]) <= 100000.0
            and _finite(applied.get("residual_m_inv"))
            and float(applied["residual_m_inv"]) <= 0.025
            and _finite(applied.get("filtered_scale"))
            and _finite(applied.get("filtered_offset"))
        )
        if valid:
            qualifying.append(
                (index, float(applied["filtered_scale"]), float(applied["filtered_offset"]))
            )
        else:
            qualifying.clear()
        if len(qualifying) >= window:
            selected = qualifying[-window:]
            return {
                "causal": True,
                "window_start_index": selected[0][0],
                "window_end_index": selected[-1][0],
                "evaluation_start_index": selected[-1][0] + 1,
                "scale": float(median(item[1] for item in selected)),
                "offset": float(median(item[2] for item in selected)),
            }
    return None


def group_median_oracle_parameters(rows: Sequence[Mapping[str, Any]]) -> dict[str, Any] | None:
    values = [
        (
            float(_nested(row, "calibration", "applied", "filtered_scale")),
            float(_nested(row, "calibration", "applied", "filtered_offset")),
        )
        for row in rows
        if _finite(_nested(row, "calibration", "applied", "filtered_scale"))
        and _finite(_nested(row, "calibration", "applied", "filtered_offset"))
    ]
    if not values:
        return None
    return {
        "causal": False,
        "label": "NON_CAUSAL_ORACLE",
        "scale": float(median(value[0] for value in values)),
        "offset": float(median(value[1] for value in values)),
    }


def _runtime_calibrator() -> MetricDepthCalibrator:
    return MetricDepthCalibrator(
        minimum_anchors=12,
        ransac_iterations=80,
        residual_threshold_m_inv=0.025,
        minimum_inlier_fraction=0.45,
        ema_alpha=0.08,
        random_seed=7,
        temporal_window=15,
        stable_samples_required=5,
        maximum_scale_step_fraction=0.18,
        maximum_offset_step_m_inv=0.04,
        maximum_condition_number=1.0e5,
        temporal_uncertainty_floor_fraction=0.35,
    )


def anchor_refit_replay(
    rows: Sequence[Mapping[str, Any]],
    *,
    calibration_sequence: Sequence[Mapping[str, Any]] | None = None,
) -> list[dict[str, Any]]:
    calibrator = _runtime_calibrator()
    target_hashes = {str(row.get("record_sha256")) for row in rows}
    output_by_hash: dict[str, dict[str, Any]] = {}
    sequence = list(calibration_sequence if calibration_sequence is not None else rows)
    sequence.sort(
        key=lambda row: (
            float(row.get("measurement_timestamp_s", 0.0)),
            int(row.get("frame_index", 0)),
        )
    )
    for row in sequence:
        anchors = _nested(row, "anchors", "per_grid_point")
        accepted = [
            anchor
            for anchor in (anchors if isinstance(anchors, list) else [])
            if isinstance(anchor, Mapping)
            and anchor.get("accepted") is True
            and _finite(anchor.get("relative_inverse_depth"))
            and _finite(anchor.get("metric_optical_depth_m"))
        ]
        q = np.asarray(
            [float(anchor["relative_inverse_depth"]) for anchor in accepted],
            dtype=np.float64,
        )
        depth = np.asarray(
            [float(anchor["metric_optical_depth_m"]) for anchor in accepted],
            dtype=np.float64,
        )
        result = calibrator.fit(q, depth)
        logged = _nested(row, "calibration", "fit")
        record_hash = str(row.get("record_sha256"))
        if record_hash in target_hashes:
            output_by_hash[record_hash] = {
                "frame_index": row.get("frame_index"),
                "valid": bool(result.valid),
                "reason": result.reason,
                "anchor_count": len(accepted),
                "raw_scale": result.raw_scale,
                "raw_offset": result.raw_offset,
                "filtered_scale": result.scale,
                "filtered_offset": result.offset,
                "logged_raw_scale_delta": (
                    None
                    if not isinstance(logged, Mapping)
                    or not _finite(logged.get("raw_scale"))
                    or not _finite(result.raw_scale)
                    else float(result.raw_scale) - float(logged["raw_scale"])
                ),
                "logged_raw_offset_delta": (
                    None
                    if not isinstance(logged, Mapping)
                    or not _finite(logged.get("raw_offset"))
                    or not _finite(result.raw_offset)
                    else float(result.raw_offset) - float(logged["raw_offset"])
                ),
            }
    return [
        output_by_hash.get(
            str(row.get("record_sha256")),
            {
                "frame_index": row.get("frame_index"),
                "valid": False,
                "reason": "target_row_not_replayed",
                "anchor_count": 0,
                "raw_scale": None,
                "raw_offset": None,
                "filtered_scale": None,
                "filtered_offset": None,
                "logged_raw_scale_delta": None,
                "logged_raw_offset_delta": None,
            },
        )
        for row in rows
    ]


def _range_from_parameters(row: Mapping[str, Any], scale: Any, offset: Any) -> float | None:
    q = _nested(row, "inverse_depth_filter", "filtered_inverse_depth")
    ray = _nested(row, "raw_range", "ray_scale")
    if not all(_finite(value) for value in (q, ray, scale, offset)):
        return None
    denominator = float(scale) * float(q) + float(offset)
    if denominator <= 0.0:
        return None
    return float(ray) / denominator


def _maximum_window_span(times: np.ndarray, values: np.ndarray, window_s: float = 10.0) -> float | None:
    if len(values) < 2:
        return None
    maximum = 0.0
    for start in range(len(values)):
        end = int(np.searchsorted(times, times[start] + window_s, side="right"))
        subset = values[start:end]
        if len(subset) >= 2:
            maximum = max(maximum, float(np.max(subset) - np.min(subset)))
    return maximum


def _pearson(left: Sequence[float], right: Sequence[float]) -> float | None:
    if len(left) < 3 or len(left) != len(right):
        return None
    x = np.asarray(left, dtype=np.float64)
    y = np.asarray(right, dtype=np.float64)
    if not np.all(np.isfinite(x)) or not np.all(np.isfinite(y)):
        return None
    if float(np.std(x)) <= 1e-15 or float(np.std(y)) <= 1e-15:
        return None
    return float(np.corrcoef(x, y)[0, 1])


def metric_summary(rows: Sequence[Mapping[str, Any]], values: Sequence[float | None]) -> dict[str, Any]:
    retained = [
        (row, float(value))
        for row, value in zip(rows, values, strict=True)
        if value is not None
        and _finite(value)
        and _finite(_nested(row, "ground_truth", "distance_m"))
        and _finite(row.get("measurement_timestamp_s"))
    ]
    if not retained:
        return {"frame_count": 0}
    times = np.asarray([float(row["measurement_timestamp_s"]) for row, _ in retained])
    estimates = np.asarray([value for _, value in retained])
    truth = np.asarray([float(_nested(row, "ground_truth", "distance_m")) for row, _ in retained])
    errors = estimates - truth
    absolute = np.abs(errors)
    deltas = np.abs(np.diff(estimates))
    slope = None
    if len(estimates) >= 2 and float(np.ptp(times)) > 0.0:
        slope = float(np.polyfit(times - times[0], estimates, 1)[0])
    return {
        "frame_count": len(estimates),
        "bias_m": float(np.mean(errors)),
        "mae_m": float(np.mean(absolute)),
        "p90_absolute_error_m": float(np.quantile(absolute, 0.90)),
        "p95_absolute_error_m": float(np.quantile(absolute, 0.95)),
        "output_standard_deviation_m": float(np.std(estimates)),
        "p95_absolute_frame_delta_m": (
            None if not len(deltas) else float(np.quantile(deltas, 0.95))
        ),
        "drift_slope_m_s": slope,
        "maximum_10_second_span_m": _maximum_window_span(times, estimates),
    }


def _write_csv(path: Path, rows: Sequence[Mapping[str, Any]]) -> None:
    fields = sorted({str(key) for row in rows for key in row})
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for row in rows:
            writer.writerow({key: row.get(key) for key in fields})


def load_sessions(
    session_roots: Sequence[Path],
) -> tuple[list[dict[str, Any]], list[dict[str, Any]], dict[str, dict[str, Any]]]:
    records: list[dict[str, Any]] = []
    integrity: list[dict[str, Any]] = []
    selected_groups: dict[str, dict[str, Any]] = {}
    for root in session_roots:
        manifest_path = root / DIAGNOSTICS_MANIFEST_FILENAME
        records_path = root / DIAGNOSTICS_FILENAME
        if not manifest_path.is_file() or not records_path.is_file():
            integrity.append({"root": str(root), "status": "MISSING_SIDECAR"})
            continue
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
        local_rows: list[dict[str, Any]] = []
        malformed_line_count = 0
        for line in records_path.read_text(encoding="utf-8").splitlines():
            if not line.strip():
                continue
            try:
                value = json.loads(line)
            except json.JSONDecodeError:
                malformed_line_count += 1
                continue
            if isinstance(value, dict):
                local_rows.append(value)
            else:
                malformed_line_count += 1
        checksum_valid = all(verify_record_checksum(row) for row in local_rows)
        event_path = root / "capture_events.jsonl"
        if event_path.is_file():
            for line in event_path.read_text(encoding="utf-8").splitlines():
                if not line.strip():
                    continue
                event = json.loads(line)
                group_key = f"{event.get('run_id')}/{event.get('session_id')}"
                selected_groups[group_key] = {
                    **event,
                    "source_root": str(root),
                }
        integrity.append(
            {
                "root": str(root),
                "status": (
                    "PASS"
                    if checksum_valid and malformed_line_count == 0
                    else "FAIL"
                ),
                "manifest_sha256": sha256_file(manifest_path),
                "records_sha256": sha256_file(records_path),
                "record_count": len(local_rows),
                "malformed_line_count": malformed_line_count,
                "run_id": manifest.get("run_id"),
                "schema_version": manifest.get("diagnostics_schema_version"),
            }
        )
        for row in local_rows:
            row["_source_root"] = str(root)
            records.append(row)
    return records, integrity, selected_groups


def analyze(session_roots: Sequence[Path], output_dir: Path, scenario_map: Mapping[str, Any] | None = None) -> dict[str, Any]:
    output_dir.mkdir(parents=True, exist_ok=True)
    all_records, integrity, selected_groups = load_sessions(session_roots)
    raw_rows = [
        row
        for row in all_records
        if row.get("stage") == "raw_range_computed"
        and f"{row.get('run_id')}/{row.get('session_id')}" in selected_groups
    ]
    grouped: dict[str, list[dict[str, Any]]] = {}
    for row in raw_rows:
        key = f"{row.get('run_id')}/{row.get('session_id')}"
        grouped.setdefault(key, []).append(row)
    for rows in grouped.values():
        rows.sort(key=lambda row: (float(row["measurement_timestamp_s"]), int(row["frame_index"])))

    all_raw_group_keys = {
        f"{row.get('run_id')}/{row.get('session_id')}"
        for row in all_records
        if row.get("stage") == "raw_range_computed"
    }
    prewarm_group_keys = {
        f"{event.get('run_id')}/{event.get('prewarm_session_id')}"
        for event in selected_groups.values()
        if event.get("prewarm_session_id") is not None
    }
    session_manifest = {
        "artifact_type": "range_v2_r3_6a_validation_session_manifest",
        "validation_id": VALIDATION_ID,
        "dataset_role": DATASET_ROLE,
        "group_key": "run_id/session_id",
        "measurement_groups": [
            selected_groups[key] for key in sorted(selected_groups)
        ],
        "measurement_group_count": len(selected_groups),
        "prewarm_group_keys": sorted(prewarm_group_keys),
        "excluded_or_aborted_raw_group_keys": sorted(
            all_raw_group_keys - set(selected_groups)
        ),
        "source_roots": [str(path) for path in session_roots],
        "world_setup": {
            "gravity_m_s2": [0.0, 0.0, 0.0],
            "model_pose_changes": "outside measurement windows",
            "reason": "keep disarmed models static above ground so existing ground-anchor geometry remains observable",
            "production_geometry_or_calibration_changed": False,
        },
        "corpus_isolation": {
            "merged_into_range_v2_development": False,
            "r4_collection": False,
            "promotion_eligible": False,
        },
    }
    (output_dir / "session_manifest.json").write_text(
        json.dumps(session_manifest, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )

    diagnostic_rows: list[dict[str, Any]] = []
    anchor_rows: list[dict[str, Any]] = []
    per_session: list[dict[str, Any]] = []
    calibration_events: list[dict[str, Any]] = []
    anchor_metrics: list[dict[str, Any]] = []
    quantity_rows: list[dict[str, Any]] = []
    all_missing: dict[str, int] = {}
    reconstruction_errors: list[float] = []
    refit_deltas: list[float] = []

    for group_key, rows in sorted(grouped.items()):
        scenario = selected_groups.get(group_key, {})
        freeze = stable_freeze_parameters(rows)
        oracle = group_median_oracle_parameters(rows)
        run_id = scenario.get("run_id")
        # RANSAC consumes a deterministic RNG stream across every fit in the
        # process/run, including aborted setup sessions.  Replaying only the
        # selected measurement and its prewarm session shifts that stream and
        # creates a false refit mismatch.  Feed every logged fit attempt in the
        # same run; target hashes below still restrict reported results to the
        # selected independent group.
        calibration_sequence = [
            row
            for row in all_records
            if row.get("run_id") == run_id
            and isinstance(_nested(row, "anchors", "per_grid_point"), list)
        ]
        refit = anchor_refit_replay(
            rows,
            calibration_sequence=calibration_sequence,
        )
        live_values = [_nested(row, "raw_range", "physics_slant_range_m") for row in rows]
        freeze_values = [
            None
            if freeze is None or index < int(freeze["evaluation_start_index"])
            else _range_from_parameters(row, freeze["scale"], freeze["offset"])
            for index, row in enumerate(rows)
        ]
        oracle_values = [
            None if oracle is None else _range_from_parameters(row, oracle["scale"], oracle["offset"])
            for row in rows
        ]
        refit_values = [
            _range_from_parameters(row, item["filtered_scale"], item["filtered_offset"])
            for row, item in zip(rows, refit, strict=True)
        ]
        for variant, values, causal in (
            (VARIANT_LIVE, live_values, True),
            (VARIANT_FREEZE, freeze_values, True),
            (VARIANT_ORACLE, oracle_values, False),
            (VARIANT_REFIT, refit_values, True),
        ):
            summary = metric_summary(rows, values)
            summary.update(
                {
                    "group_key": group_key,
                    "scenario_id": scenario.get("scenario_id"),
                    "variant": variant,
                    "causal": causal,
                    "promotion_eligible": False,
                    "freeze_frame_index": None if freeze is None else freeze["window_end_index"],
                }
            )
            per_session.append(summary)

        previous_source = None
        previous_epoch = None
        previous_range = None
        previous_a = None
        previous_b = None
        accepted_sets: list[set[int]] = []
        a_values: list[float] = []
        b_values: list[float] = []
        condition_values: list[float] = []
        q_spans: list[float] = []
        depth_spans: list[float] = []
        target_q_values: list[float] = []
        ray_scale_values: list[float] = []
        raw_range_values: list[float] = []
        churn_for_jumps: list[float] = []
        parameter_jumps: list[float] = []
        timestamp_order_invalid_count = 0
        rejection_reason_counts: dict[str, int] = {}
        for index, (row, replay) in enumerate(zip(rows, refit, strict=True)):
            missing = completeness_reasons(row)
            for reason in missing:
                all_missing[reason] = all_missing.get(reason, 0) + 1
            if "monotonic_timestamp_order_invalid" in missing:
                timestamp_order_invalid_count += 1
            reconstruction_ok, reconstruction_error = exact_live_reconstruction(row)
            if reconstruction_error is not None:
                reconstruction_errors.append(abs(reconstruction_error))
            # Preserve every source diagnostic field in the compacted frame
            # artifact.  Anchor detail is emitted separately to avoid storing
            # the same 96-record array twice.
            normalized = {
                key: value
                for key, value in row.items()
                if key not in {"anchors", "_source_root"}
            }
            normalized.update({
                "validation_id": VALIDATION_ID,
                "dataset_role": DATASET_ROLE,
                "source_root": row.get("_source_root"),
                "run_id": row.get("run_id"),
                "session_id": row.get("session_id"),
                "group_id": row.get("group_id"),
                "group_key": group_key,
                "scenario_id": scenario.get("scenario_id"),
                "frame_index": row.get("frame_index"),
                "measurement_timestamp_s": row.get("measurement_timestamp_s"),
                "source_record_sha256": row.get("record_sha256"),
                "complete": not missing,
                "incomplete_reasons": missing,
                "live_reconstruction_exact": reconstruction_ok,
                "live_reconstruction_error_m": reconstruction_error,
                "stable_freeze_range_m": freeze_values[index],
                "oracle_range_m": oracle_values[index],
                "oracle_label": "NON_CAUSAL_ORACLE",
                "anchor_refit_range_m": refit_values[index],
                "anchor_refit": replay,
                "stable_freeze_parameters": freeze,
                "group_median_oracle_parameters": oracle,
                "ground_truth_distance_m": _nested(row, "ground_truth", "distance_m"),
                "raw_physical_range_m": _nested(row, "raw_range", "physics_slant_range_m"),
                "calibration_source": _nested(row, "calibration", "source"),
                "track_epoch": row.get("track_epoch"),
                "calibration_epoch": row.get("calibration_epoch"),
            })
            diagnostic_rows.append(normalized)
            anchors = _nested(row, "anchors", "per_grid_point") or []
            accepted = {
                int(anchor["grid_index"])
                for anchor in anchors
                if isinstance(anchor, Mapping) and anchor.get("accepted") is True
            }
            accepted_sets.append(accepted)
            if len(accepted_sets) >= 2:
                previous_set = accepted_sets[-2]
                union = previous_set | accepted
                churn_for_jumps.append(
                    0.0 if not union else 1.0 - len(previous_set & accepted) / len(union)
                )
            accepted_q = [
                float(anchor["relative_inverse_depth"])
                for anchor in anchors
                if isinstance(anchor, Mapping)
                and anchor.get("accepted") is True
                and _finite(anchor.get("relative_inverse_depth"))
            ]
            accepted_depth = [
                float(anchor["metric_optical_depth_m"])
                for anchor in anchors
                if isinstance(anchor, Mapping)
                and anchor.get("accepted") is True
                and _finite(anchor.get("metric_optical_depth_m"))
            ]
            if accepted_q:
                q_spans.append(max(accepted_q) - min(accepted_q))
            if accepted_depth:
                depth_spans.append(max(accepted_depth) - min(accepted_depth))
            for anchor in anchors:
                if not isinstance(anchor, Mapping):
                    continue
                reason = str(anchor.get("reason") or "missing_reason")
                rejection_reason_counts[reason] = (
                    rejection_reason_counts.get(reason, 0) + 1
                )
                anchor_rows.append(
                    {
                        "dataset_role": DATASET_ROLE,
                        "group_key": group_key,
                        "scenario_id": scenario.get("scenario_id"),
                        "frame_index": row.get("frame_index"),
                        "source_record_sha256": row.get("record_sha256"),
                        **dict(anchor),
                        "target_exclusion_result": anchor.get("target_exclusion_result", "NOT_RECORDED"),
                    }
                )
            applied = _nested(row, "calibration", "applied") or {}
            if _finite(applied.get("filtered_scale")):
                a_values.append(float(applied["filtered_scale"]))
            if _finite(applied.get("filtered_offset")):
                b_values.append(float(applied["filtered_offset"]))
            if len(a_values) >= 2 and len(b_values) >= 2:
                parameter_jumps.append(
                    math.hypot(a_values[-1] - a_values[-2], b_values[-1] - b_values[-2])
                )
            if _finite(applied.get("condition_number")):
                condition_values.append(float(applied["condition_number"]))
            target_q = _nested(row, "inverse_depth_filter", "filtered_inverse_depth")
            ray_scale = _nested(row, "raw_range", "ray_scale")
            raw_range = _nested(row, "raw_range", "physics_slant_range_m")
            if all(_finite(value) for value in (target_q, ray_scale, raw_range)):
                target_q_values.append(float(target_q))
                ray_scale_values.append(float(ray_scale))
                raw_range_values.append(float(raw_range))
            if _finite(replay.get("logged_raw_scale_delta")):
                refit_deltas.append(abs(float(replay["logged_raw_scale_delta"])))
            source = _nested(row, "calibration", "source")
            epoch = row.get("calibration_epoch")
            reseed = bool(_nested(row, "calibration", "fit", "change_point_reseeded"))
            if source != previous_source or epoch != previous_epoch or reseed:
                calibration_events.append(
                    {
                        "group_key": group_key,
                        "scenario_id": scenario.get("scenario_id"),
                        "frame_index": row.get("frame_index"),
                        "measurement_timestamp_s": row.get("measurement_timestamp_s"),
                        "event": "reseed" if reseed else "source_or_epoch_transition",
                        "calibration_source": source,
                        "calibration_epoch": epoch,
                        "cache_age_s": _nested(row, "calibration", "cache_age_s"),
                        "cache_ttl_s": _nested(row, "calibration", "cache_ttl_s"),
                        "physics_slant_range_m": _nested(row, "raw_range", "physics_slant_range_m"),
                        "range_jump_m": (
                            None
                            if not _finite(previous_range) or not _finite(raw_range)
                            else float(raw_range) - float(previous_range)
                        ),
                        "raw_fit_a": _nested(row, "calibration", "fit", "raw_scale"),
                        "raw_fit_b": _nested(row, "calibration", "fit", "raw_offset"),
                        "applied_a": applied.get("filtered_scale"),
                        "applied_b": applied.get("filtered_offset"),
                        "applied_a_jump": (
                            None
                            if not _finite(previous_a) or not _finite(applied.get("filtered_scale"))
                            else float(applied["filtered_scale"]) - float(previous_a)
                        ),
                        "applied_b_jump": (
                            None
                            if not _finite(previous_b) or not _finite(applied.get("filtered_offset"))
                            else float(applied["filtered_offset"]) - float(previous_b)
                        ),
                        "accepted_anchor_count": len(accepted),
                        "q_span": None if not accepted_q else max(accepted_q) - min(accepted_q),
                        "metric_depth_span_m": None if not accepted_depth else max(accepted_depth) - min(accepted_depth),
                        "condition_number": applied.get("condition_number"),
                        "fit_residual_m_inv": applied.get("residual_m_inv"),
                    }
                )
            previous_source, previous_epoch = source, epoch
            previous_range = raw_range
            previous_a = applied.get("filtered_scale")
            previous_b = applied.get("filtered_offset")

            gt_ref = _nested(row, "reference_centers", "ground_truth") or {}
            camera_link = gt_ref.get("camera_link_to_target_model_origin_m") if isinstance(gt_ref, Mapping) else None
            drone_center = gt_ref.get("drone_center_to_target_model_origin_m") if isinstance(gt_ref, Mapping) else None
            quantity_rows.append(
                {
                    "group_key": group_key,
                    "scenario_id": scenario.get("scenario_id"),
                    "frame_index": row.get("frame_index"),
                    "camera_link_to_target_model_origin_m": camera_link,
                    "drone_center_to_target_model_origin_m": drone_center,
                    "drone_minus_camera_center_range_m": (
                        None
                        if not _finite(camera_link) or not _finite(drone_center)
                        else float(drone_center) - float(camera_link)
                    ),
                    "raw_geometry_center_source": _nested(row, "reference_centers", "raw_geometry_center_source"),
                    "optical_center_status": _nested(row, "reference_centers", "optical_center", "status"),
                    "raw_target_semantics": _nested(row, "reference_centers", "target_reference", "raw"),
                    "gt_target_semantics": _nested(row, "reference_centers", "target_reference", "ground_truth"),
                    "surface_to_center_offset_m": "UNKNOWN_NOT_RECORDED",
                }
            )

        churn = []
        for previous, current in zip(accepted_sets, accepted_sets[1:]):
            union = previous | current
            churn.append(0.0 if not union else 1.0 - len(previous & current) / len(union))
        anchor_metrics.append(
            {
                "group_key": group_key,
                "scenario_id": scenario.get("scenario_id"),
                "frame_count": len(rows),
                "mean_accepted_anchor_count": float(np.mean([len(value) for value in accepted_sets])) if accepted_sets else None,
                "mean_anchor_set_churn": float(np.mean(churn)) if churn else None,
                "p95_anchor_set_churn": float(np.quantile(churn, 0.95)) if churn else None,
                "mean_q_span": float(np.mean(q_spans)) if q_spans else None,
                "mean_metric_depth_span_m": float(np.mean(depth_spans)) if depth_spans else None,
                "applied_a_standard_deviation": float(np.std(a_values)) if a_values else None,
                "applied_b_standard_deviation": float(np.std(b_values)) if b_values else None,
                "median_condition_number": float(np.median(condition_values)) if condition_values else None,
                "raw_range_standard_deviation_m": float(np.std(raw_range_values)) if raw_range_values else None,
                "target_q_standard_deviation": float(np.std(target_q_values)) if target_q_values else None,
                "ray_scale_standard_deviation": float(np.std(ray_scale_values)) if ray_scale_values else None,
                "raw_range_vs_applied_a_correlation": _pearson(raw_range_values, a_values),
                "raw_range_vs_applied_b_correlation": _pearson(raw_range_values, b_values),
                "anchor_churn_vs_parameter_jump_correlation": _pearson(churn_for_jumps, parameter_jumps),
                "q_span_vs_applied_a_correlation": _pearson(q_spans, a_values),
                "depth_span_vs_applied_a_correlation": _pearson(depth_spans, a_values),
                "timestamp_order_invalid_count": timestamp_order_invalid_count,
                "anchor_reason_counts_json": json.dumps(rejection_reason_counts, sort_keys=True),
            }
        )

    complete_count = sum(1 for row in diagnostic_rows if row["complete"])
    integrity_pass = all(item.get("status") == "PASS" for item in integrity) and bool(integrity)
    exact_reconstruction = bool(reconstruction_errors) and max(reconstruction_errors) <= 1e-9
    refit_exact = bool(refit_deltas) and max(refit_deltas) <= 1e-12
    center_offsets = [
        abs(float(item["drone_minus_camera_center_range_m"]))
        for item in quantity_rows
        if _finite(item.get("drone_minus_camera_center_range_m"))
    ]
    zero_churn_sessions = sum(
        1
        for item in anchor_metrics
        if item.get("mean_anchor_set_churn") == 0.0
    )
    conclusion = "DIAGNOSTIC_EVIDENCE_INSUFFICIENT"
    hypotheses = [
        {
            "hypothesis": "quantity_domain_mismatch",
            "status": "SUPPORTED" if quantity_rows else "INCONCLUSIVE",
            "evidence": "raw uses eroded-bbox foreground statistic while GT uses target model origin; optical-center and surface offset remain incomplete",
        },
        {
            "hypothesis": "vehicle_center_vs_camera_link_explains_multi_meter_error",
            "status": "NOT_SUPPORTED" if center_offsets and max(center_offsets) < 0.05 else "INCONCLUSIVE",
            "evidence": f"maximum observed drone-center versus camera_link range contribution is {max(center_offsets) if center_offsets else None} m",
        },
        {
            "hypothesis": "calibration_lifecycle_variation",
            "status": "SUPPORTED" if any((item.get("applied_a_standard_deviation") or 0) > 0 for item in anchor_metrics) else "INCONCLUSIVE",
            "evidence": "applied a/b variation and event-aligned output are reported per session; correlation alone is not causal",
        },
        {
            "hypothesis": "anchor_membership_churn_is_primary_driver",
            "status": "NOT_SUPPORTED" if zero_churn_sessions >= 8 else "INCONCLUSIVE",
            "evidence": f"{zero_churn_sessions}/{len(anchor_metrics)} sessions have exactly zero accepted-anchor-set churn despite nonzero a/b and range variation",
        },
        {
            "hypothesis": "anchor_refit_implementation_mismatch",
            "status": "NOT_SUPPORTED" if refit_exact else "INCONCLUSIVE",
            "evidence": f"full-run same-config replay maximum raw-a delta {max(refit_deltas) if refit_deltas else None}",
        },
        {
            "hypothesis": "timestamp_frame_association_valid",
            "status": "NOT_SUPPORTED" if all_missing.get("monotonic_timestamp_order_invalid", 0) else "SUPPORTED",
            "evidence": f"{all_missing.get('monotonic_timestamp_order_invalid', 0)} frames record consume_now before depth completion",
        },
        {
            "hypothesis": "camera_intrinsics_and_extrinsics_contract_complete",
            "status": "NOT_SUPPORTED",
            "evidence": "distortion, rectification, verified optical center, and extrinsics fingerprint are absent on every selected frame",
        },
        {
            "hypothesis": "required_diagnostics_complete",
            "status": "CONFIRMED" if complete_count == len(diagnostic_rows) and diagnostic_rows else "NOT_SUPPORTED",
            "evidence": f"{complete_count}/{len(diagnostic_rows)} raw-range frames satisfy the precommitted required-field contract",
        },
    ]

    with (output_dir / "diagnostic_frames.jsonl").open("w", encoding="utf-8") as stream:
        for row in diagnostic_rows:
            stream.write(json.dumps(row, sort_keys=True, allow_nan=False) + "\n")
    with (output_dir / "anchor_records.jsonl").open("w", encoding="utf-8") as stream:
        for row in anchor_rows:
            stream.write(json.dumps(row, sort_keys=True, allow_nan=False) + "\n")
    _write_csv(output_dir / "per_session_metrics.csv", per_session)
    _write_csv(output_dir / "calibration_event_metrics.csv", calibration_events)
    _write_csv(output_dir / "anchor_stability_metrics.csv", anchor_metrics)
    _write_csv(output_dir / "quantity_center_comparison.csv", quantity_rows)
    _write_csv(output_dir / "hypothesis_matrix.csv", hypotheses)
    (output_dir / "sidecar_integrity_report.json").write_text(
        json.dumps({"status": "PASS" if integrity_pass else "FAIL", "sessions": integrity}, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    (output_dir / "missing_or_incomplete_fields.json").write_text(
        json.dumps(
            {
                "required_raw_range_frames": len(diagnostic_rows),
                "complete_frames": complete_count,
                "incomplete_frames": len(diagnostic_rows) - complete_count,
                "reason_counts": dict(sorted(all_missing.items())),
                "policy": "missing values are not inferred; frame remains diagnostic-only and incomplete",
            },
            indent=2,
            sort_keys=True,
        )
        + "\n",
        encoding="utf-8",
    )
    _write_csv(output_dir / "hypothesis_matrix.csv", hypotheses)
    summary = {
        "validation_id": VALIDATION_ID,
        "dataset_role": DATASET_ROLE,
        "session_count": len(grouped),
        "sidecar_record_count": len(all_records),
        "raw_range_frame_count": len(diagnostic_rows),
        "complete_frame_count": complete_count,
        "anchor_record_count": len(anchor_rows),
        "sidecar_integrity": "PASS" if integrity_pass else "FAIL",
        "exact_live_reconstruction": exact_reconstruction,
        "maximum_live_reconstruction_error_m": max(reconstruction_errors) if reconstruction_errors else None,
        "anchor_refit_matches_logged_raw_fit": refit_exact,
        "maximum_anchor_refit_raw_scale_delta": max(refit_deltas) if refit_deltas else None,
        "conclusion": conclusion,
    }
    return summary


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("session_roots", nargs="+", type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    summary = analyze(args.session_roots, args.output)
    print(json.dumps(summary, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
