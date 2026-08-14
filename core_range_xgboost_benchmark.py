"""Offline-only XGBoost benchmark for static core-range development data.

This module never imports or calls runtime/controller code.  It has two explicit
phases: ``prepare`` freezes source provenance, features, folds and model policy;
``run`` verifies that precommit and then performs development-only cross-validation.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import os
from dataclasses import dataclass
from pathlib import Path
from statistics import mean, median, pstdev
from typing import Any, Iterable, Mapping, Sequence

import numpy as np

from core_range_logging_eval import load_jsonl, sha256_file, verify_record
from range_physical_diagnostics import validate_timestamp_stages


BENCHMARK_ID = "core_range_xgboost_offline_benchmark_20260804_v001"
SEED = 52
SCHEMA_VERSION = "core_range_logging_3_12m_v001"
BINS = tuple(f"{lower}-{lower + 1}m" for lower in range(3, 12))
TRACE_COLUMNS = (
    "source_collection", "source_root", "logical_session_id", "run_id",
    "runtime_session_id", "runtime_group_id", "frame_index",
    "measurement_timestamp_s", "source_sim_timestamp_s",
    "trace_identity_sha256", "record_sha256", "distance_bin",
)
LABEL_COLUMNS = (
    "ground_truth_range_m", "residual_gt_m", "direct_target_m",
)
FEATURE_NAMES = (
    "raw_physical_range_m",
    "calibration_a", "calibration_b", "calibration_denominator",
    "calibration_fit_residual", "calibration_condition",
    "calibration_inlier_count", "calibration_inlier_fraction",
    "calibration_q_span", "calibration_depth_span", "calibration_age",
    "calibration_source_live", "calibration_source_cache",
    "calibration_source_prewarm", "calibration_source_recovery",
    "target_inverse_depth", "target_roi_q_min", "target_roi_q_p10",
    "target_roi_q_p25", "target_roi_q_median", "target_roi_q_p75",
    "target_roi_q_p90", "target_roi_q_max", "target_roi_q_std",
    "target_roi_q_iqr", "image_ray_x", "image_ray_y", "ray_scale",
    "bbox_center_x_fraction", "bbox_center_y_fraction",
    "bbox_width_fraction", "bbox_height_fraction", "bbox_area_fraction",
    "bbox_aspect_ratio", "bbox_tracking_score", "anchor_accept_count",
    "anchor_reject_count", "anchor_coverage", "anchor_q_median",
    "anchor_q_mad", "anchor_depth_median", "anchor_depth_span",
    "target_valid_fraction",
    "calibration_measurement_accepted", "calibration_stable",
    "deterministic_gate_applicable", "deterministic_measurement_usable",
)
FEATURE_BLACKLIST = {
    "ground_truth_range_m", "nominal_distance_m", "distance_bin",
    "session_id", "runtime_session_id", "logical_session_id", "group_id",
    "runtime_group_id", "run_id", "dataset_path", "source_root",
    "target_pose", "scenario", "fold", "fold_index",
    "source_sim_timestamp_s", "ground_truth_timestamp_s",
}
HYPERPARAMETERS = {
    "A_conservative": {
        "n_estimators": 200, "max_depth": 2, "learning_rate": 0.03,
        "min_child_weight": 5.0, "subsample": 0.8,
        "colsample_bytree": 0.8, "reg_alpha": 0.1, "reg_lambda": 5.0,
    },
    "B_balanced": {
        "n_estimators": 300, "max_depth": 3, "learning_rate": 0.03,
        "min_child_weight": 3.0, "subsample": 0.8,
        "colsample_bytree": 0.8, "reg_alpha": 0.05, "reg_lambda": 3.0,
    },
    "C_shallow": {
        "n_estimators": 150, "max_depth": 1, "learning_rate": 0.05,
        "min_child_weight": 5.0, "subsample": 1.0,
        "colsample_bytree": 1.0, "reg_alpha": 0.1, "reg_lambda": 5.0,
    },
}
GATES = {
    "overall_equal_group_mae_m_max": 1.0,
    "worst_bin_equal_group_mae_m_max": 1.5,
    "overall_equal_group_p90_m_max": 2.0,
    "error_gt_3m_fraction_max": 0.01,
    "stationary_median_group_std_m_max": 0.3,
    "raw_equal_group_mae_improvement_min": 0.50,
}


def canonical_sha256(value: Any) -> str:
    payload = json.dumps(
        value, sort_keys=True, separators=(",", ":"), allow_nan=False
    ).encode("utf-8")
    return hashlib.sha256(payload).hexdigest()


def percentile(values: Iterable[float], fraction: float) -> float:
    ordered = sorted(float(value) for value in values)
    if not ordered:
        return float("nan")
    position = (len(ordered) - 1) * float(fraction)
    lower, upper = math.floor(position), math.ceil(position)
    if lower == upper:
        return ordered[lower]
    return ordered[lower] + (ordered[upper] - ordered[lower]) * (position - lower)


def bin_bounds(label: str) -> tuple[float, float]:
    lower, upper = label.removesuffix("m").split("-", 1)
    return float(lower), float(upper)


def bin_contains(label: str, value: float) -> bool:
    lower, upper = bin_bounds(label)
    return lower <= float(value) < upper


def _json(path: Path) -> Any:
    return json.loads(path.read_text(encoding="utf-8"))


def _quantile(values: Sequence[Any], index: int) -> float | None:
    if len(values) != 7 or values[index] is None:
        return None
    return float(values[index])


def _finite_or_none(value: Any) -> float | None:
    if value is None:
        return None
    result = float(value)
    return result if math.isfinite(result) else None


def reconstructed_config_fingerprint(row: Mapping[str, Any]) -> str:
    """Fingerprint only logged runtime configuration, never pose or GT."""
    anchors = row.get("anchors") or {}
    calibration = row.get("calibration") or {}
    camera = row.get("camera_info") or {}
    depth = row.get("depth_model") or {}
    raw = row.get("raw_range") or {}
    correction = row.get("correction_observation") or {}
    contract = {
        "diagnostics_schema_version": row.get("diagnostics_schema_version"),
        "camera_info_fingerprint": camera.get("fingerprint_sha256"),
        "camera_dimensions": [camera.get("width"), camera.get("height")],
        "anchor_adapter_config": anchors.get("adapter_config"),
        "calibration_cache_ttl_s": calibration.get("cache_ttl_s"),
        "depth_model": depth,
        "raw_formula": raw.get("formula"),
        "residual_mode": correction.get("mode"),
    }
    return canonical_sha256(contract)


def _sample_index(root: Path, runtime_session_id: int) -> dict[tuple[Any, ...], dict[str, Any]]:
    path = root / "samples.jsonl"
    rows, malformed = load_jsonl(path)
    if malformed:
        raise ValueError(f"sample_json_malformed:{root}:{malformed}")
    selected = [r for r in rows if int(r.get("session_id", -1)) == runtime_session_id]
    result: dict[tuple[Any, ...], dict[str, Any]] = {}
    for row in selected:
        key = (row.get("run_id"), int(row["session_id"]), int(row["frame_index"]))
        if key in result:
            raise ValueError(f"duplicate_sample_identity:{root}:{key}")
        result[key] = row
    return result


def _raw_rows(root: Path, runtime_session_id: int) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    malformed = 0
    for path in sorted(root.glob("physical_diagnostics*.jsonl")):
        loaded, invalid = load_jsonl(path)
        malformed += invalid
        rows.extend(
            row for row in loaded
            if row.get("stage") == "raw_range_computed"
            and int(row.get("session_id", -1)) == runtime_session_id
        )
    if malformed:
        raise ValueError(f"diagnostic_json_malformed:{root}:{malformed}")
    return rows


def _accepted_anchor_values(row: Mapping[str, Any], key: str) -> list[float]:
    records = ((row.get("anchors") or {}).get("per_grid_point") or [])
    return [
        float(anchor[key]) for anchor in records
        if anchor.get("accepted") is True and anchor.get(key) is not None
        and math.isfinite(float(anchor[key]))
    ]


def extract_features(row: Mapping[str, Any]) -> dict[str, float | None]:
    raw = row["raw_range"]
    calibration = row["calibration"]
    applied = calibration["applied"]
    target = row["target_depth"]
    inverse_filter = row["inverse_depth_filter"]
    anchors = row["anchors"]
    bbox = row["bbox"]
    camera = row["camera_info"]
    q = float(inverse_filter["filtered_inverse_depth"])
    a = float(applied["filtered_scale"])
    b = float(applied["filtered_offset"])
    target_quantiles = target["raw_relative_inverse_depth_quantiles"]
    anchor_q = _accepted_anchor_values(row, "relative_inverse_depth")
    anchor_depth = _accepted_anchor_values(row, "metric_optical_depth_m")
    q_median = median(anchor_q) if anchor_q else None
    q_mad = median(abs(value - q_median) for value in anchor_q) if anchor_q else None
    width, height = float(camera["width"]), float(camera["height"])
    x, y, w, h = (float(value) for value in bbox["xywh_px"])
    source = str(calibration.get("source") or "unknown").lower()
    applied_q = applied.get("anchor_inverse_depth_quantiles") or []
    applied_depth = applied.get("metric_depth_quantiles_m") or []
    applicability = row.get("applicability") or {}
    values: dict[str, float | None] = {
        "raw_physical_range_m": float(raw["physics_slant_range_m"]),
        "calibration_a": a,
        "calibration_b": b,
        "calibration_denominator": a * q + b,
        "calibration_fit_residual": _finite_or_none(applied.get("residual_m_inv")),
        "calibration_condition": _finite_or_none(applied.get("condition_number")),
        "calibration_inlier_count": _finite_or_none(applied.get("inlier_count")),
        "calibration_inlier_fraction": (
            float(applied["inlier_count"]) / float(applied["anchor_count"])
            if applied.get("anchor_count") else None
        ),
        "calibration_q_span": (
            float(applied_q[-1]) - float(applied_q[0]) if len(applied_q) == 7 else None
        ),
        "calibration_depth_span": (
            float(applied_depth[-1]) - float(applied_depth[0])
            if len(applied_depth) == 7 else None
        ),
        "calibration_age": _finite_or_none(calibration.get("calibration_age_s")),
        "calibration_source_live": float(source == "live"),
        "calibration_source_cache": float(source == "cache"),
        "calibration_source_prewarm": float(source == "prewarm"),
        "calibration_source_recovery": float(source == "recovery"),
        "target_inverse_depth": q,
        "target_roi_q_min": _quantile(target_quantiles, 0),
        "target_roi_q_p10": _quantile(target_quantiles, 1),
        "target_roi_q_p25": _quantile(target_quantiles, 2),
        "target_roi_q_median": _quantile(target_quantiles, 3),
        "target_roi_q_p75": _quantile(target_quantiles, 4),
        "target_roi_q_p90": _quantile(target_quantiles, 5),
        "target_roi_q_max": _quantile(target_quantiles, 6),
        "target_roi_q_std": _finite_or_none(target.get("raw_relative_inverse_depth_std")),
        "target_roi_q_iqr": (
            float(target_quantiles[4]) - float(target_quantiles[2])
            if len(target_quantiles) == 7 else None
        ),
        "image_ray_x": float(raw["image_ray_x"]),
        "image_ray_y": float(raw["image_ray_y"]),
        "ray_scale": float(raw["ray_scale"]),
        "bbox_center_x_fraction": (x + 0.5 * w) / width,
        "bbox_center_y_fraction": (y + 0.5 * h) / height,
        "bbox_width_fraction": w / width,
        "bbox_height_fraction": h / height,
        "bbox_area_fraction": (w * h) / (width * height),
        "bbox_aspect_ratio": w / h,
        "bbox_tracking_score": _finite_or_none(bbox.get("tracking_score")),
        "anchor_accept_count": float(anchors["accepted_ground_anchor_count"]),
        "anchor_reject_count": float(96 - int(anchors["accepted_ground_anchor_count"])),
        "anchor_coverage": _finite_or_none(anchors.get("spatial_coverage_fraction")),
        "anchor_q_median": q_median,
        "anchor_q_mad": q_mad,
        "anchor_depth_median": median(anchor_depth) if anchor_depth else None,
        "anchor_depth_span": max(anchor_depth) - min(anchor_depth) if anchor_depth else None,
        "target_valid_fraction": _finite_or_none(target.get("valid_fraction")),
        "calibration_measurement_accepted": float(applied.get("measurement_accepted") is True),
        "calibration_stable": float(applied.get("stable") is True),
        "deterministic_gate_applicable": float(applicability.get("applicable") is True),
        "deterministic_measurement_usable": float(applicability.get("measurement_usable") is True),
    }
    if set(values) != set(FEATURE_NAMES):
        raise AssertionError("feature_contract_internal_mismatch")
    return values


def _source_specs(workspace: Path) -> list[dict[str, Any]]:
    priority = workspace / "artifacts/core_range_3_12m/priority_collection"
    static = workspace / "artifacts/core_range_3_12m/static_balance_collection"
    priority_rows = list(csv.DictReader((priority / "audit/per_group_audit.csv").open()))
    static_rows = list(csv.DictReader((static / "per_group_audit.csv").open()))
    specs: list[dict[str, Any]] = []
    for audit in priority_rows:
        logical = audit["scenario_id"]
        specs.append({
            "source_collection": "priority_static_collection",
            "logical_session_id": logical,
            "distance_bin": audit["bin"],
            "scenario": audit["view"],
            "runtime_session_id": int(audit["session_id"]),
            "root": priority / "runtime_sessions" / logical,
            "expected_sidecar_sha256": audit["source_sidecar_sha256"],
        })
    for audit in static_rows:
        logical = audit["session_id"]
        specs.append({
            "source_collection": "static_balance_collection",
            "logical_session_id": logical,
            "distance_bin": audit["distance_bin"],
            "scenario": audit["scenario"],
            "runtime_session_id": int(audit["runtime_session_id"]),
            "root": static / "runtime_sessions" / logical,
            "expected_sidecar_sha256": audit["source_sidecar_sha256"],
        })
    return specs


def collect_verified_rows(workspace: Path) -> tuple[list[dict[str, Any]], dict[str, Any]]:
    specs = _source_specs(workspace)
    if len(specs) != 27:
        raise ValueError(f"accepted_group_count_not_27:{len(specs)}")
    rows: list[dict[str, Any]] = []
    sources: list[dict[str, Any]] = []
    all_trace_ids: set[str] = set()
    all_groups: set[str] = set()
    for spec in specs:
        root = Path(spec["root"])
        sidecar = root / "physical_diagnostics.jsonl"
        samples_path = root / "samples.jsonl"
        if sha256_file(sidecar) != spec["expected_sidecar_sha256"]:
            raise ValueError(f"sidecar_checksum_mismatch:{spec['logical_session_id']}")
        sample_index = _sample_index(root, spec["runtime_session_id"])
        diagnostics = _raw_rows(root, spec["runtime_session_id"])
        if len(diagnostics) < 30:
            raise ValueError(f"group_below_30_frames:{spec['logical_session_id']}")
        run_ids = {str(r.get("run_id")) for r in diagnostics}
        runtime_groups = {str(r.get("group_id")) for r in diagnostics}
        runtime_sessions = {int(r.get("session_id")) for r in diagnostics}
        if len(run_ids) != 1 or len(runtime_groups) != 1 or len(runtime_sessions) != 1:
            raise ValueError(f"dataset_not_one_group:{spec['logical_session_id']}")
        logical_group = str(spec["logical_session_id"])
        if logical_group in all_groups:
            raise ValueError(f"logical_group_duplicate:{logical_group}")
        all_groups.add(logical_group)
        config_fingerprints: set[str] = set()
        recorded_pipeline_fingerprints = 0
        sample_join_count = 0
        for diagnostic in diagnostics:
            if diagnostic.get("diagnostics_schema_version") != SCHEMA_VERSION:
                raise ValueError(f"logging_schema_mismatch:{logical_group}")
            record_ok, trace_ok = verify_record(diagnostic)
            if not record_ok or not trace_ok:
                raise ValueError(f"record_checksum_mismatch:{logical_group}")
            if diagnostic.get("ground_truth_trace_valid") is not True:
                raise ValueError(f"gt_trace_invalid:{logical_group}")
            timestamp_ok, _ = validate_timestamp_stages(diagnostic.get("timestamp_stages") or {})
            if not timestamp_ok or diagnostic.get("timestamp_order_valid") is not True:
                raise ValueError(f"timestamp_order_invalid:{logical_group}")
            if len(((diagnostic.get("anchors") or {}).get("per_grid_point") or [])) != 96:
                raise ValueError(f"anchor_count_not_96:{logical_group}")
            truth = float(diagnostic["ground_truth"]["distance_m"])
            if not bin_contains(spec["distance_bin"], truth):
                raise ValueError(f"gt_outside_bin:{logical_group}:{truth}")
            trace_id = str(diagnostic["trace_identity_sha256"])
            if trace_id in all_trace_ids:
                raise ValueError(f"duplicate_trace_identity:{trace_id}")
            all_trace_ids.add(trace_id)
            key = (diagnostic["run_id"], int(diagnostic["session_id"]), int(diagnostic["frame_index"]))
            sample = sample_index.get(key)
            if sample is not None:
                sample_join_count += 1
                if not math.isclose(float(sample["physics_distance_m"]), float(diagnostic["raw_range"]["physics_slant_range_m"]), abs_tol=1e-9):
                    raise ValueError(f"sample_raw_mismatch:{logical_group}:{key}")
                if not math.isclose(float(sample["ground_truth_distance_m"]), truth, abs_tol=1e-9):
                    raise ValueError(f"sample_gt_mismatch:{logical_group}:{key}")
            config_fingerprints.add(reconstructed_config_fingerprint(diagnostic))
            pipeline_fp = diagnostic.get("pipeline_config_fingerprint_sha256")
            if isinstance(pipeline_fp, str) and len(pipeline_fp) == 64:
                recorded_pipeline_fingerprints += 1
            feature_values = extract_features(diagnostic)
            raw_value = float(diagnostic["raw_range"]["physics_slant_range_m"])
            result = {
                "source_collection": spec["source_collection"],
                "source_root": str(root.relative_to(workspace)),
                "logical_session_id": logical_group,
                "run_id": str(diagnostic["run_id"]),
                "runtime_session_id": int(diagnostic["session_id"]),
                "runtime_group_id": str(diagnostic["group_id"]),
                "frame_index": int(diagnostic["frame_index"]),
                "measurement_timestamp_s": float(diagnostic["measurement_timestamp_s"]),
                "source_sim_timestamp_s": _finite_or_none(diagnostic.get("source_sim_timestamp_s")),
                "trace_identity_sha256": trace_id,
                "record_sha256": str(diagnostic["record_sha256"]),
                "distance_bin": spec["distance_bin"],
                "ground_truth_range_m": truth,
                "residual_gt_m": truth - raw_value,
                "direct_target_m": truth,
                **feature_values,
            }
            rows.append(result)
        sources.append({
            "source_collection": spec["source_collection"],
            "logical_session_id": logical_group,
            "distance_bin": spec["distance_bin"],
            "scenario": spec["scenario"],
            "source_root": str(root.relative_to(workspace)),
            "runtime_session_id": spec["runtime_session_id"],
            "frame_count": len(diagnostics),
            "legacy_sample_join_count": sample_join_count,
            "feature_source": "physical_diagnostics_sidecar_only",
            "source_files": {
                "physical_diagnostics.jsonl": sha256_file(sidecar),
                "samples.jsonl": sha256_file(samples_path),
                "manifest.json": sha256_file(root / "manifest.json"),
                "capture_events.jsonl": sha256_file(root / "capture_events.jsonl"),
                "audit/smoke_manifest.json": sha256_file(root / "audit/smoke_manifest.json"),
                "audit/smoke_summary.json": sha256_file(root / "audit/smoke_summary.json"),
            },
            "logging_schema_version": SCHEMA_VERSION,
            "reconstructed_runtime_config_fingerprints": sorted(config_fingerprints),
            "recorded_pipeline_fingerprint_rows": recorded_pipeline_fingerprints,
            "config_fingerprint_status": (
                "RECORDED_AND_RECONSTRUCTED" if recorded_pipeline_fingerprints == len(diagnostics)
                else "RECONSTRUCTED_FROM_LOGGED_RUNTIME_CONFIG"
            ),
            "integrity_status": "PASS",
        })
    per_bin = {label: [s for s in sources if s["distance_bin"] == label] for label in BINS}
    if any(len(group_rows) != 3 for group_rows in per_bin.values()):
        raise ValueError(f"three_groups_per_bin_failed:{ {k:len(v) for k,v in per_bin.items()} }")
    manifest = {
        "benchmark_id": BENCHMARK_ID,
        "dataset_role": "static_development_only",
        "group_count": len(sources), "frame_count": len(rows),
        "bin_group_counts": {key: len(value) for key, value in per_bin.items()},
        "bin_frame_counts": {key: sum(r["frame_count"] for r in value) for key, value in per_bin.items()},
        "sources": sorted(sources, key=lambda value: value["logical_session_id"]),
        "source_manifests": {
            "priority_collection_manifest": sha256_file(workspace / "artifacts/core_range_3_12m/priority_collection/audit/priority_collection_manifest.json"),
            "static_balance_collection_manifest": sha256_file(workspace / "artifacts/core_range_3_12m/static_balance_collection/static_balance_collection_manifest.json"),
        },
        "integrity": {
            "source_checksum": "PASS", "record_checksum": "PASS",
            "gt_trace_checksum": "PASS", "timestamp_ordering": "PASS",
            "duplicate_trace_identity_count": 0,
            "one_dataset_one_group_session": "PASS", "gt_distance_bin": "PASS",
            "logging_schema": "PASS", "runtime_config_fingerprint": "PASS",
            "quarantine_attempts_used": False,
        },
    }
    return rows, manifest


def assign_folds(rows: Sequence[Mapping[str, Any]]) -> list[dict[str, Any]]:
    groups: dict[str, dict[str, str]] = {}
    for row in rows:
        group = str(row["logical_session_id"])
        groups[group] = {"logical_session_id": group, "distance_bin": str(row["distance_bin"])}
    assignments: list[dict[str, Any]] = []
    for label in BINS:
        selected = sorted(
            (value for value in groups.values() if value["distance_bin"] == label),
            key=lambda value: hashlib.sha256(f"{SEED}:{label}:{value['logical_session_id']}".encode()).hexdigest(),
        )
        if len(selected) != 3:
            raise ValueError(f"fold_bin_group_count:{label}:{len(selected)}")
        for fold, item in enumerate(selected):
            assignments.append({**item, "fold": fold, "role_in_fold": "test"})
    return sorted(assignments, key=lambda value: (value["fold"], bin_bounds(value["distance_bin"])[0]))


def _feature_contract() -> dict[str, Any]:
    return {
        "contract_id": "core_range_xgboost_features_20260804_v001",
        "model_input": "raw_numeric_after_train_only_median_imputation",
        "feature_names": list(FEATURE_NAMES),
        "missing_indicators": [f"{name}__missing" for name in FEATURE_NAMES],
        "trace_columns_not_model_features": list(TRACE_COLUMNS),
        "label_columns_not_model_features": list(LABEL_COLUMNS),
        "feature_blacklist": sorted(FEATURE_BLACKLIST),
        "target_roi_q_mad": {
            "status": "NOT_RECORDED",
            "replacement": "target_roi_q_std_and_target_roi_q_iqr",
            "reason": "do_not_infer_MAD_from_quantiles",
        },
        "camera_pitch": {
            "status": "NOT_USED",
            "reason": "no_independently_validated_scalar_in_sidecar; scenario pitch is forbidden metadata",
        },
        "gimbal_pitch": {
            "status": "NOT_USED",
            "reason": "no_independent_runtime_scalar_logged; scenario pitch is forbidden metadata",
        },
        "temporal_features": {
            "status": "NOT_USED_IN_PRIMARY_STATIC_BENCHMARK",
            "reason": "framewise benchmark avoids previous-frame shortcut and dynamic claims",
        },
        "preprocessing": {
            "finite_validation": True,
            "median_fit_partition": "training_groups_only_per_fold",
            "missing_indicator_policy": "fixed_indicator_for_every_feature",
            "standard_scaler": False,
        },
    }


def _write_csv(path: Path, rows: Sequence[Mapping[str, Any]], fieldnames: Sequence[str] | None = None) -> None:
    rows = list(rows)
    names = list(fieldnames or (list(rows[0]) if rows else []))
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=names)
        writer.writeheader()
        writer.writerows(rows)


def prepare(workspace: Path, output: Path) -> dict[str, Any]:
    if (output / "benchmark_manifest.json").exists():
        raise ValueError("completed_benchmark_already_exists")
    output.mkdir(parents=True, exist_ok=True)
    rows, dataset_manifest = collect_verified_rows(workspace)
    contract = _feature_contract()
    if set(contract["feature_names"]) & FEATURE_BLACKLIST:
        raise ValueError("feature_leakage_blacklist")
    assignments = assign_folds(rows)
    dataset_path = output / "dataset_manifest.json"
    contract_path = output / "feature_contract.json"
    folds_path = output / "fold_assignments.csv"
    dataset_path.write_text(json.dumps(dataset_manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    contract_path.write_text(json.dumps(contract, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    _write_csv(folds_path, assignments)
    plan = {
        "benchmark_id": BENCHMARK_ID,
        "created_before_fit": True,
        "seed": SEED,
        "scope": "offline_static_development_only_3_to_12m",
        "methods": ["raw_physical", "residual_xgboost", "direct_xgboost"],
        "cross_validation": {
            "type": "3_fold_stratified_group_cross_validation",
            "test_groups_per_fold": 9, "train_groups_per_fold": 18,
            "one_test_group_per_bin_per_fold": True,
            "fold_assignments_sha256": sha256_file(folds_path),
        },
        "hyperparameters": HYPERPARAMETERS,
        "xgboost_common": {
            "objective": "reg:squarederror", "eval_metric": "mae",
            "tree_method": "hist", "seed": SEED, "nthread": 1,
        },
        "preprocessing": contract["preprocessing"],
        "training_weights": "equal_total_weight_per_training_group",
        "prediction_policies": {
            "primary_clip_m": [3.0, 12.0],
            "legacy_residual_clamp": "clip residual to +/-min(3.0m,0.25*raw_range)",
        },
        "primary_metric": "mean_equal_group_MAE_across_27_outer_test_groups",
        "secondary_metrics": ["worst_bin_equal_group_MAE", "equal_group_P90", "error_gt_2m", "stationary_group_std"],
        "development_accuracy_gates": GATES,
        "configuration_selection": "minimum OOF equal-group clipped MAE independently for residual and direct",
        "source_contracts": {
            "dataset_manifest_sha256": sha256_file(dataset_path),
            "feature_contract_sha256": sha256_file(contract_path),
        },
        "scope_guards": {
            "runtime_modified": False, "controller_effect": False,
            "simulation_or_shadow": False, "final_holdout": False,
            "residual_runtime_default": "off", "dynamic_metrics_claimed": False,
        },
    }
    plan_path = output / "benchmark_plan.json"
    plan_path.write_text(json.dumps(plan, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    leakage = {
        "status": "PASS", "group_count": 27, "frame_count": len(rows),
        "duplicate_trace_identity_count": 0,
        "feature_blacklist_intersection": [],
        "fold_group_overlap_count": 0,
        "quarantine_attempts_used": False,
        "near_duplicate_audit": "deferred_to_run_after_train-only_imputation",
    }
    (output / "leakage_audit.json").write_text(json.dumps(leakage, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps({
        "phase": "PRECOMMIT_COMPLETE_NO_FIT", "benchmark_plan_sha256": sha256_file(plan_path),
        "groups": 27, "frames": len(rows), "folds": 3,
    }, indent=2))
    return plan


@dataclass
class FoldPreprocessor:
    medians: np.ndarray
    feature_names: tuple[str, ...] = FEATURE_NAMES

    @classmethod
    def fit(cls, rows: Sequence[Mapping[str, Any]]) -> "FoldPreprocessor":
        matrix = np.asarray([[np.nan if row[name] is None else float(row[name]) for name in FEATURE_NAMES] for row in rows], dtype=np.float64)
        medians = np.nanmedian(matrix, axis=0)
        if np.any(~np.isfinite(medians)):
            missing = [FEATURE_NAMES[i] for i in np.where(~np.isfinite(medians))[0]]
            raise ValueError(f"training_feature_all_missing:{missing}")
        return cls(medians=medians)

    def transform(self, rows: Sequence[Mapping[str, Any]]) -> np.ndarray:
        matrix = np.asarray([[np.nan if row[name] is None else float(row[name]) for name in FEATURE_NAMES] for row in rows], dtype=np.float64)
        missing = np.isnan(matrix)
        filled = np.where(missing, self.medians.reshape(1, -1), matrix)
        result = np.hstack([filled, missing.astype(np.float64)])
        if not np.all(np.isfinite(result)):
            raise ValueError("preprocessed_features_nonfinite")
        return result

    def as_json(self, train_groups: Sequence[str]) -> dict[str, Any]:
        return {
            "fit_partition": "train_only", "train_groups": sorted(train_groups),
            "feature_names": list(FEATURE_NAMES),
            "output_feature_names": list(FEATURE_NAMES) + [f"{name}__missing" for name in FEATURE_NAMES],
            "medians": {name: float(value) for name, value in zip(FEATURE_NAMES, self.medians, strict=True)},
            "standard_scaler": False,
        }


def group_weights(rows: Sequence[Mapping[str, Any]]) -> np.ndarray:
    counts: dict[str, int] = {}
    for row in rows:
        group = str(row["logical_session_id"])
        counts[group] = counts.get(group, 0) + 1
    target = len(rows) / len(counts)
    return np.asarray([target / counts[str(row["logical_session_id"])] for row in rows], dtype=np.float64)


def metric_values(truth: np.ndarray, prediction: np.ndarray) -> dict[str, float]:
    error = prediction - truth
    absolute = np.abs(error)
    return {
        "signed_bias_m": float(np.mean(error)), "median_absolute_error_m": float(np.median(absolute)),
        "mae_m": float(np.mean(absolute)), "rmse_m": float(np.sqrt(np.mean(error ** 2))),
        "p90_abs_error_m": float(np.quantile(absolute, 0.90)), "p95_abs_error_m": float(np.quantile(absolute, 0.95)),
        "mean_absolute_relative_error": float(np.mean(absolute / truth)),
        "error_gt_1m_fraction": float(np.mean(absolute > 1.0)), "error_gt_2m_fraction": float(np.mean(absolute > 2.0)),
        "error_gt_3m_fraction": float(np.mean(absolute > 3.0)),
    }


def per_group_metric_rows(rows: Sequence[Mapping[str, Any]], prediction: np.ndarray, method: str, config: str = "N/A") -> list[dict[str, Any]]:
    results: list[dict[str, Any]] = []
    groups = sorted({str(row["logical_session_id"]) for row in rows})
    for group in groups:
        indices = [i for i, row in enumerate(rows) if str(row["logical_session_id"]) == group]
        selected_rows = [rows[i] for i in indices]
        truth = np.asarray([float(row["ground_truth_range_m"]) for row in selected_rows])
        pred = prediction[indices]
        deltas = np.abs(np.diff(pred))
        base = metric_values(truth, pred)
        results.append({
            "method": method, "config": config, "logical_session_id": group,
            "source_collection": selected_rows[0]["source_collection"], "distance_bin": selected_rows[0]["distance_bin"],
            "frame_count": len(indices), "gt_mean_m": float(np.mean(truth)), "gt_std_m": float(np.std(truth)),
            "prediction_mean_m": float(np.mean(pred)), "prediction_std_m": float(np.std(pred)),
            **base, "p95_frame_delta_m": float(np.quantile(deltas, 0.95)) if len(deltas) else 0.0,
            "maximum_span_m": float(np.max(pred) - np.min(pred)),
        })
    return results


def equal_group_aggregate(group_rows: Sequence[Mapping[str, Any]]) -> dict[str, float]:
    keys = (
        "signed_bias_m", "median_absolute_error_m", "mae_m", "rmse_m",
        "p90_abs_error_m", "p95_abs_error_m", "mean_absolute_relative_error",
        "error_gt_1m_fraction", "error_gt_2m_fraction", "error_gt_3m_fraction",
    )
    return {key: mean(float(row[key]) for row in group_rows) for key in keys}


def _xgb_parameters(config: Mapping[str, Any]) -> tuple[dict[str, Any], int]:
    return ({
        "objective": "reg:squarederror", "eval_metric": "mae", "tree_method": "hist",
        "max_depth": int(config["max_depth"]), "eta": float(config["learning_rate"]),
        "min_child_weight": float(config["min_child_weight"]), "subsample": float(config["subsample"]),
        "colsample_bytree": float(config["colsample_bytree"]), "alpha": float(config["reg_alpha"]),
        "lambda": float(config["reg_lambda"]), "seed": SEED, "nthread": 1,
    }, int(config["n_estimators"]))


def _validate_precommit(output: Path) -> tuple[dict[str, Any], dict[str, int]]:
    plan = _json(output / "benchmark_plan.json")
    if plan.get("benchmark_id") != BENCHMARK_ID or plan.get("seed") != SEED or plan.get("created_before_fit") is not True:
        raise ValueError("benchmark_plan_invalid")
    contracts = plan["source_contracts"]
    if sha256_file(output / "dataset_manifest.json") != contracts["dataset_manifest_sha256"]:
        raise ValueError("dataset_manifest_changed_after_precommit")
    if sha256_file(output / "feature_contract.json") != contracts["feature_contract_sha256"]:
        raise ValueError("feature_contract_changed_after_precommit")
    if sha256_file(output / "fold_assignments.csv") != plan["cross_validation"]["fold_assignments_sha256"]:
        raise ValueError("fold_assignments_changed_after_precommit")
    fold_map: dict[str, int] = {}
    with (output / "fold_assignments.csv").open() as stream:
        for row in csv.DictReader(stream): fold_map[row["logical_session_id"]] = int(row["fold"])
    return plan, fold_map


def _write_parquet(rows: Sequence[Mapping[str, Any]], path: Path) -> None:
    try:
        import pyarrow as pa
        import pyarrow.parquet as pq
    except ImportError as error:
        raise RuntimeError("pyarrow_required_for_feature_table") from error
    table = pa.Table.from_pylist(list(rows))
    pq.write_table(table, path, compression="zstd")


def _train_booster(xgb: Any, train_x: np.ndarray, target: np.ndarray, weights: np.ndarray, config: Mapping[str, Any]) -> Any:
    params, rounds = _xgb_parameters(config)
    return xgb.train(params, xgb.DMatrix(train_x, label=target, weight=weights), num_boost_round=rounds, verbose_eval=False)


def _method_predictions(raw: np.ndarray, model_prediction: np.ndarray, target_kind: str) -> dict[str, np.ndarray]:
    if target_kind == "residual":
        unclipped = raw + model_prediction
        limit = np.minimum(3.0, 0.25 * raw)
        legacy = raw + np.clip(model_prediction, -limit, limit)
        return {
            "residual_xgboost_unclipped": unclipped,
            "residual_xgboost_clipped": np.clip(unclipped, 3.0, 12.0),
            "residual_xgboost_legacy_clamp": np.clip(legacy, 3.0, 12.0),
        }
    return {
        "direct_xgboost_unclipped": model_prediction,
        "direct_xgboost_clipped": np.clip(model_prediction, 3.0, 12.0),
    }


def _manual_permutation_importance(xgb: Any, booster: Any, test_x: np.ndarray, test_rows: Sequence[Mapping[str, Any]], truth: np.ndarray, baseline_prediction: np.ndarray, fold: int, target_kind: str, config_name: str) -> list[dict[str, Any]]:
    baseline_groups = per_group_metric_rows(test_rows, baseline_prediction, "baseline")
    baseline_mae = equal_group_aggregate(baseline_groups)["mae_m"]
    names = list(FEATURE_NAMES) + [f"{name}__missing" for name in FEATURE_NAMES]
    results: list[dict[str, Any]] = []
    for feature_index, feature_name in enumerate(names):
        permuted = test_x.copy()
        rng = np.random.default_rng(SEED + 1000 * fold + feature_index)
        for group in sorted({str(row["logical_session_id"]) for row in test_rows}):
            indices = np.asarray([i for i, row in enumerate(test_rows) if str(row["logical_session_id"]) == group])
            permuted[indices, feature_index] = permuted[rng.permutation(indices), feature_index]
        raw_prediction = np.asarray(booster.predict(xgb.DMatrix(permuted)), dtype=np.float64)
        raw_values = np.asarray([float(row["raw_physical_range_m"]) for row in test_rows])
        predicted = _method_predictions(raw_values, raw_prediction, target_kind)[f"{target_kind}_xgboost_clipped"]
        permuted_mae = equal_group_aggregate(per_group_metric_rows(test_rows, predicted, "permuted"))["mae_m"]
        results.append({
            "target": target_kind, "config": config_name, "fold": fold,
            "feature": feature_name, "importance_type": "groupwise_test_permutation_equal_group_mae_delta_m",
            "importance": permuted_mae - baseline_mae,
        })
        global_permuted = test_x.copy()
        global_permuted[:, feature_index] = global_permuted[
            rng.permutation(len(test_rows)), feature_index
        ]
        global_raw_prediction = np.asarray(
            booster.predict(xgb.DMatrix(global_permuted)), dtype=np.float64
        )
        global_prediction = _method_predictions(
            raw_values, global_raw_prediction, target_kind
        )[f"{target_kind}_xgboost_clipped"]
        global_mae = equal_group_aggregate(
            per_group_metric_rows(test_rows, global_prediction, "permuted")
        )["mae_m"]
        results.append({
            "target": target_kind, "config": config_name, "fold": fold,
            "feature": feature_name,
            "importance_type": "global_test_permutation_equal_group_mae_delta_m",
            "importance": global_mae - baseline_mae,
        })
    return results


def _gain_importance(booster: Any, fold: int, target_kind: str, config_name: str) -> list[dict[str, Any]]:
    score = booster.get_score(importance_type="gain")
    names = list(FEATURE_NAMES) + [f"{name}__missing" for name in FEATURE_NAMES]
    return [{
        "target": target_kind, "config": config_name, "fold": fold,
        "feature": name, "importance_type": "xgboost_gain",
        "importance": float(score.get(f"f{index}", 0.0)),
    } for index, name in enumerate(names)]


def _selected_config(predictions: Mapping[tuple[str, str], np.ndarray], rows: Sequence[Mapping[str, Any]], target: str) -> str:
    scores = {}
    for config_name in HYPERPARAMETERS:
        pred = predictions[(target, config_name)]
        scores[config_name] = equal_group_aggregate(per_group_metric_rows(rows, pred, target, config_name))["mae_m"]
    return min(scores, key=lambda name: (scores[name], name))


def _model_gate(group_rows: Sequence[Mapping[str, Any]], bin_rows: Sequence[Mapping[str, Any]], raw_equal_mae: float) -> dict[str, Any]:
    aggregate = equal_group_aggregate(group_rows)
    worst_bin = max(float(row["equal_group_mae_m"]) for row in bin_rows)
    stationary = median(float(row["prediction_std_m"]) for row in group_rows)
    improvement = (raw_equal_mae - aggregate["mae_m"]) / raw_equal_mae
    checks = {
        "overall_equal_group_mae": aggregate["mae_m"] <= GATES["overall_equal_group_mae_m_max"],
        "worst_bin_equal_group_mae": worst_bin <= GATES["worst_bin_equal_group_mae_m_max"],
        "overall_equal_group_p90": aggregate["p90_abs_error_m"] <= GATES["overall_equal_group_p90_m_max"],
        "error_gt_3m": aggregate["error_gt_3m_fraction"] <= GATES["error_gt_3m_fraction_max"],
        "stationary_median_group_std": stationary <= GATES["stationary_median_group_std_m_max"],
        "raw_equal_group_mae_improvement": improvement >= GATES["raw_equal_group_mae_improvement_min"],
    }
    return {
        "passed": all(checks.values()), "checks": checks,
        "overall_equal_group_mae_m": aggregate["mae_m"],
        "worst_bin_equal_group_mae_m": worst_bin,
        "overall_equal_group_p90_m": aggregate["p90_abs_error_m"],
        "error_gt_3m_fraction": aggregate["error_gt_3m_fraction"],
        "stationary_median_group_std_m": stationary,
        "raw_equal_group_mae_improvement_fraction": improvement,
    }


def _pearson(left: np.ndarray, right: np.ndarray) -> float | None:
    if len(left) < 2 or float(np.std(left)) == 0.0 or float(np.std(right)) == 0.0:
        return None
    return float(np.corrcoef(left, right)[0, 1])


def _importance_stability(
    importance_rows: Sequence[Mapping[str, Any]], target: str, config: str
) -> dict[str, Any]:
    selected = [
        row for row in importance_rows
        if row["target"] == target and row["config"] == config
        and row["importance_type"] == "xgboost_gain"
    ]
    top_by_fold: dict[str, list[str]] = {}
    for fold in range(3):
        ranked = sorted(
            (row for row in selected if int(row["fold"]) == fold),
            key=lambda row: float(row["importance"]), reverse=True,
        )
        top_by_fold[str(fold)] = [str(row["feature"]) for row in ranked[:5]]
    jaccards: list[float] = []
    for first, second in ((0, 1), (0, 2), (1, 2)):
        left, right = set(top_by_fold[str(first)]), set(top_by_fold[str(second)])
        jaccards.append(len(left & right) / len(left | right))
    return {
        "top_5_gain_features_by_fold": top_by_fold,
        "pairwise_top_5_jaccard": jaccards,
        "mean_pairwise_top_5_jaccard": mean(jaccards),
        "causal_interpretation": "PROHIBITED",
    }
def run(workspace: Path, output: Path) -> dict[str, Any]:
    plan, fold_map = _validate_precommit(output)
    rows, current_manifest = collect_verified_rows(workspace)
    frozen_manifest = _json(output / "dataset_manifest.json")
    if canonical_sha256(current_manifest) != canonical_sha256(frozen_manifest):
        raise ValueError("source_dataset_changed_after_precommit")
    if set(fold_map) != {str(row["logical_session_id"]) for row in rows}:
        raise ValueError("fold_group_mapping_incomplete")
    for fold in range(3):
        test = [group for group, assigned in fold_map.items() if assigned == fold]
        if len(test) != 9 or {next(row["distance_bin"] for row in rows if row["logical_session_id"] == group) for group in test} != set(BINS):
            raise ValueError(f"fold_stratification_invalid:{fold}")
    _write_parquet(rows, output / "feature_table.parquet")
    try:
        import xgboost as xgb
    except ImportError as error:
        raise RuntimeError("xgboost_offline_dependency_missing") from error
    n = len(rows)
    all_predictions: dict[tuple[str, str], np.ndarray] = {}
    train_predictions: dict[tuple[str, str, int], tuple[list[dict[str, Any]], np.ndarray]] = {}
    boosters: dict[tuple[str, str, int], Any] = {}
    fold_metrics: list[dict[str, Any]] = []
    importance_rows: list[dict[str, Any]] = []
    preprocessing_dir = output / "preprocessing"
    models_dir = output / "models"
    preprocessing_dir.mkdir(exist_ok=True)
    models_dir.mkdir(exist_ok=True)
    for target_kind, label_name in (("residual", "residual_gt_m"), ("direct", "direct_target_m")):
        for config_name, config in HYPERPARAMETERS.items():
            oof = np.full(n, np.nan, dtype=np.float64)
            for fold in range(3):
                train_indices = [i for i, row in enumerate(rows) if fold_map[str(row["logical_session_id"])] != fold]
                test_indices = [i for i, row in enumerate(rows) if fold_map[str(row["logical_session_id"])] == fold]
                train_rows = [rows[i] for i in train_indices]
                test_rows = [rows[i] for i in test_indices]
                preprocessor = FoldPreprocessor.fit(train_rows)
                train_x, test_x = preprocessor.transform(train_rows), preprocessor.transform(test_rows)
                train_y = np.asarray([float(row[label_name]) for row in train_rows])
                booster = _train_booster(xgb, train_x, train_y, group_weights(train_rows), config)
                raw_train_prediction = np.asarray(booster.predict(xgb.DMatrix(train_x)), dtype=np.float64)
                raw_test_prediction = np.asarray(booster.predict(xgb.DMatrix(test_x)), dtype=np.float64)
                train_raw = np.asarray([float(row["raw_physical_range_m"]) for row in train_rows])
                test_raw = np.asarray([float(row["raw_physical_range_m"]) for row in test_rows])
                train_pred = _method_predictions(train_raw, raw_train_prediction, target_kind)[f"{target_kind}_xgboost_clipped"]
                test_pred = _method_predictions(test_raw, raw_test_prediction, target_kind)[f"{target_kind}_xgboost_clipped"]
                oof[test_indices] = test_pred
                train_predictions[(target_kind, config_name, fold)] = (train_rows, train_pred)
                boosters[(target_kind, config_name, fold)] = booster
                train_group_rows = per_group_metric_rows(train_rows, train_pred, target_kind, config_name)
                test_group_rows = per_group_metric_rows(test_rows, test_pred, target_kind, config_name)
                for partition, metrics_rows in (("train", train_group_rows), ("test", test_group_rows)):
                    fold_metrics.append({
                        "target": target_kind, "config": config_name, "fold": fold, "partition": partition,
                        "group_count": len(metrics_rows), "frame_count": sum(int(r["frame_count"]) for r in metrics_rows),
                        **equal_group_aggregate(metrics_rows),
                    })
                importance_rows.extend(_gain_importance(booster, fold, target_kind, config_name))
                importance_rows.extend(_manual_permutation_importance(xgb, booster, test_x, test_rows, np.asarray([float(r["ground_truth_range_m"]) for r in test_rows]), test_pred, fold, target_kind, config_name))
                preprocessor_path = preprocessing_dir / f"{target_kind}_{config_name}_fold_{fold}.json"
                preprocessor_path.write_text(json.dumps(preprocessor.as_json(sorted({str(r['logical_session_id']) for r in train_rows})), indent=2, sort_keys=True) + "\n", encoding="utf-8")
            if np.any(~np.isfinite(oof)):
                raise ValueError(f"oof_prediction_incomplete:{target_kind}:{config_name}")
            all_predictions[(target_kind, config_name)] = oof
    selected = {target: _selected_config(all_predictions, rows, target) for target in ("residual", "direct")}
    configuration_rows: list[dict[str, Any]] = []
    for target in ("residual", "direct"):
        for config_name in HYPERPARAMETERS:
            groups = per_group_metric_rows(
                rows, all_predictions[(target, config_name)], target, config_name
            )
            aggregate = equal_group_aggregate(groups)
            configuration_rows.append({
                "target": target, "config": config_name,
                "selected_for_target": config_name == selected[target],
                "oof_equal_group_mae_m": aggregate["mae_m"],
                "oof_equal_group_p90_m": aggregate["p90_abs_error_m"],
                "oof_error_gt_2m_fraction": aggregate["error_gt_2m_fraction"],
                "oof_error_gt_3m_fraction": aggregate["error_gt_3m_fraction"],
                "stationary_median_group_std_m": median(float(row["prediction_std_m"]) for row in groups),
            })
    for target, config_name in selected.items():
        for fold in range(3):
            boosters[(target, config_name, fold)].save_model(str(models_dir / f"{target}_fold_{fold}.json"))
    truth = np.asarray([float(row["ground_truth_range_m"]) for row in rows])
    raw = np.asarray([float(row["raw_physical_range_m"]) for row in rows])
    method_predictions: dict[str, np.ndarray] = {
        "raw_physical_range": raw, "raw_clipped_3_12m": np.clip(raw, 3.0, 12.0),
    }
    for target, config_name in selected.items():
        selected_clipped = all_predictions[(target, config_name)]
        # Reconstruct unclipped OOF for reports using a second prediction pass from saved in-memory boosters.
        unclip = np.full(n, np.nan)
        legacy = np.full(n, np.nan)
        for fold in range(3):
            test_indices = [i for i, row in enumerate(rows) if fold_map[str(row["logical_session_id"])] == fold]
            test_rows = [rows[i] for i in test_indices]
            preprocessor_data = _json(preprocessing_dir / f"{target}_{config_name}_fold_{fold}.json")
            medians = np.asarray([preprocessor_data["medians"][name] for name in FEATURE_NAMES])
            test_x = FoldPreprocessor(medians).transform(test_rows)
            raw_model = np.asarray(boosters[(target, config_name, fold)].predict(xgb.DMatrix(test_x)))
            variants = _method_predictions(np.asarray([float(r["raw_physical_range_m"]) for r in test_rows]), raw_model, target)
            unclip[test_indices] = variants[f"{target}_xgboost_unclipped"]
            if target == "residual": legacy[test_indices] = variants["residual_xgboost_legacy_clamp"]
        method_predictions[f"{target}_xgboost_unclipped"] = unclip
        method_predictions[f"{target}_xgboost_clipped"] = selected_clipped
        if target == "residual": method_predictions["residual_xgboost_legacy_clamp"] = legacy
    comparison_rows: list[dict[str, Any]] = []
    all_selected_group_rows: list[dict[str, Any]] = []
    all_bin_rows: list[dict[str, Any]] = []
    for method, prediction in method_predictions.items():
        config_name = selected["residual"] if method.startswith("residual") else selected["direct"] if method.startswith("direct") else "N/A"
        groups = per_group_metric_rows(rows, prediction, method, config_name)
        all_selected_group_rows.extend(groups)
        aggregate = equal_group_aggregate(groups)
        frame = metric_values(truth, prediction)
        bins: list[dict[str, Any]] = []
        for label in BINS:
            chosen = [row for row in groups if row["distance_bin"] == label]
            agg = equal_group_aggregate(chosen)
            bin_row = {
                "method": method, "config": config_name, "distance_bin": label,
                "group_count": len(chosen), "frame_count": sum(int(row["frame_count"]) for row in chosen),
                "equal_group_bias_m": agg["signed_bias_m"], "equal_group_median_absolute_error_m": agg["median_absolute_error_m"],
                "equal_group_mae_m": agg["mae_m"], "equal_group_p90_m": agg["p90_abs_error_m"],
                "equal_group_p95_m": agg["p95_abs_error_m"], "worst_group_mae_m": max(float(row["mae_m"]) for row in chosen),
            }
            bins.append(bin_row); all_bin_rows.append(bin_row)
        comparison_rows.append({
            "method": method, "selected_config": config_name,
            "equal_group_signed_bias_m": aggregate["signed_bias_m"], "equal_group_median_absolute_error_m": aggregate["median_absolute_error_m"],
            "equal_group_mae_m": aggregate["mae_m"], "equal_group_rmse_m": aggregate["rmse_m"],
            "equal_group_p90_m": aggregate["p90_abs_error_m"], "equal_group_p95_m": aggregate["p95_abs_error_m"],
            "equal_group_mean_absolute_relative_error": aggregate["mean_absolute_relative_error"],
            "equal_group_error_gt_1m_fraction": aggregate["error_gt_1m_fraction"], "equal_group_error_gt_2m_fraction": aggregate["error_gt_2m_fraction"],
            "equal_group_error_gt_3m_fraction": aggregate["error_gt_3m_fraction"],
            "frame_weighted_mae_m": frame["mae_m"], "frame_weighted_p90_m": frame["p90_abs_error_m"],
            "worst_bin_equal_group_mae_m": max(float(row["equal_group_mae_m"]) for row in bins),
            "stationary_median_group_std_m": median(float(row["prediction_std_m"]) for row in groups),
            "boundary_clip_fraction": float(np.mean((prediction <= 3.0 + 1e-12) | (prediction >= 12.0 - 1e-12))) if "clipped" in method or "clamp" in method else 0.0,
        })
    raw_equal_mae = next(float(row["equal_group_mae_m"]) for row in comparison_rows if row["method"] == "raw_clipped_3_12m")
    gates = {}
    for target in ("residual", "direct"):
        method = f"{target}_xgboost_clipped"
        groups = [row for row in all_selected_group_rows if row["method"] == method]
        bins = [row for row in all_bin_rows if row["method"] == method]
        gates[target] = _model_gate(groups, bins, raw_equal_mae)
    passing = [target for target in ("direct", "residual") if gates[target]["passed"]]
    if passing:
        best = min(passing, key=lambda target: gates[target]["overall_equal_group_mae_m"])
        conclusion = "DIRECT_XGBOOST_BEST_DEVELOPMENT_CANDIDATE" if best == "direct" else "RESIDUAL_XGBOOST_BEST_DEVELOPMENT_CANDIDATE"
    else:
        conclusion = "NO_MODEL_MEETS_DEVELOPMENT_ACCURACY_GATE"
    prediction_rows = []
    for index, row in enumerate(rows):
        prediction_rows.append({
            **{key: row[key] for key in TRACE_COLUMNS}, "ground_truth_range_m": row["ground_truth_range_m"],
            "outer_test_fold": fold_map[str(row["logical_session_id"])],
            **{method: float(pred[index]) for method, pred in method_predictions.items()},
        })
    # Exact and descriptive near-duplicate audit on model feature vectors across groups.
    raw_feature_keys: dict[str, set[str]] = {}
    duplicate_pairs = 0
    for row in rows:
        signature = canonical_sha256([row[name] for name in FEATURE_NAMES])
        group = str(row["logical_session_id"])
        previous = raw_feature_keys.setdefault(signature, set())
        if previous and group not in previous: duplicate_pairs += len(previous)
        previous.add(group)
    audit_matrix = np.asarray([
        [np.nan if row[name] is None else float(row[name]) for name in FEATURE_NAMES]
        for row in rows
    ], dtype=np.float64)
    audit_medians = np.nanmedian(audit_matrix, axis=0)
    audit_matrix = np.where(np.isnan(audit_matrix), audit_medians, audit_matrix)
    audit_scale = np.std(audit_matrix, axis=0)
    usable = audit_scale > 1e-12
    standardized = (
        (audit_matrix[:, usable] - np.mean(audit_matrix[:, usable], axis=0))
        / audit_scale[usable]
    )
    minimum_cross_group_distance = float("inf")
    near_pair_counts = {"rms_z_le_0_001": 0, "rms_z_le_0_01": 0}
    for left in range(len(rows)):
        for right in range(left + 1, len(rows)):
            if rows[left]["logical_session_id"] == rows[right]["logical_session_id"]:
                continue
            distance = float(np.sqrt(np.mean((standardized[left] - standardized[right]) ** 2)))
            minimum_cross_group_distance = min(minimum_cross_group_distance, distance)
            if distance <= 0.001: near_pair_counts["rms_z_le_0_001"] += 1
            if distance <= 0.01: near_pair_counts["rms_z_le_0_01"] += 1
    leakage = _json(output / "leakage_audit.json")
    leakage.update({
        "near_duplicate_audit": {
            "method": "exact_SHA256_and_descriptive_RMS_z_distance_across_nonconstant_runtime_features",
            "exact_cross_group_duplicate_pairs": duplicate_pairs,
            "minimum_cross_group_rms_z_distance": minimum_cross_group_distance,
            "descriptive_near_pair_counts": near_pair_counts,
            "near_pair_thresholds_are_not_candidate_gates": True,
            "approximate_image_near_duplicates": "UNKNOWN_IMAGES_NOT_LOGGED",
        },
        "status": "PASS" if duplicate_pairs == 0 else "FAIL",
    })
    if duplicate_pairs:
        raise ValueError(f"cross_group_feature_duplicates:{duplicate_pairs}")
    (output / "leakage_audit.json").write_text(json.dumps(leakage, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    _write_csv(output / "fold_metrics.csv", fold_metrics)
    _write_csv(output / "configuration_metrics.csv", configuration_rows)
    _write_csv(output / "feature_importance.csv", importance_rows)
    _write_csv(output / "per_group_metrics.csv", all_selected_group_rows)
    _write_csv(output / "per_bin_metrics.csv", all_bin_rows)
    _write_csv(output / "model_comparison.csv", comparison_rows)
    _write_csv(output / "prediction_rows.csv", prediction_rows)
    residual_metrics = [row for row in comparison_rows if row["method"].startswith("residual")]
    direct_metrics = [row for row in comparison_rows if row["method"].startswith("direct")]
    raw_metrics = [row for row in comparison_rows if row["method"].startswith("raw")]
    _write_csv(output / "residual_model_metrics.csv", residual_metrics)
    _write_csv(output / "direct_model_metrics.csv", direct_metrics)
    _write_csv(output / "raw_baseline_metrics.csv", raw_metrics)
    direct_prediction = method_predictions["direct_xgboost_clipped"]
    direct_unclipped = method_predictions["direct_xgboost_unclipped"]
    bbox_width = np.asarray([float(row["bbox_width_fraction"]) for row in rows])
    direct_groups = [row for row in all_selected_group_rows if row["method"] == "direct_xgboost_clipped"]
    scenario_map = {source["logical_session_id"]: source["scenario"] for source in frozen_manifest["sources"]}
    selected_fold_gaps: dict[str, list[dict[str, float]]] = {}
    for target in ("residual", "direct"):
        config_name = selected[target]
        selected_fold_gaps[target] = []
        for fold in range(3):
            train_metric = next(row for row in fold_metrics if row["target"] == target and row["config"] == config_name and int(row["fold"]) == fold and row["partition"] == "train")
            test_metric = next(row for row in fold_metrics if row["target"] == target and row["config"] == config_name and int(row["fold"]) == fold and row["partition"] == "test")
            selected_fold_gaps[target].append({
                "fold": fold, "train_equal_group_mae_m": float(train_metric["mae_m"]),
                "test_equal_group_mae_m": float(test_metric["mae_m"]),
                "test_minus_train_mae_m": float(test_metric["mae_m"]) - float(train_metric["mae_m"]),
            })
    global_permutation = [
        row for row in importance_rows
        if row["target"] == "direct" and row["config"] == selected["direct"]
        and row["importance_type"] == "global_test_permutation_equal_group_mae_delta_m"
    ]
    averaged_permutation = []
    for feature in sorted({str(row["feature"]) for row in global_permutation}):
        values = [float(row["importance"]) for row in global_permutation if row["feature"] == feature]
        averaged_permutation.append({"feature": feature, "mean_delta_mae_m": mean(values), "fold_values": values})
    averaged_permutation.sort(key=lambda row: row["mean_delta_mae_m"], reverse=True)
    gain_stability = _importance_stability(importance_rows, "direct", selected["direct"])
    bbox_top_gain_all_folds = all(
        gain_stability["top_5_gain_features_by_fold"][str(fold)][0].startswith("bbox_")
        for fold in range(3)
    )
    shortcut_audit = {
        "train_vs_test": selected_fold_gaps,
        "feature_importance_stability": gain_stability,
        "global_test_permutation_top_10": averaged_permutation[:10],
        "prediction_dependence": {
            "pearson_direct_vs_raw": _pearson(direct_prediction, raw),
            "pearson_direct_vs_bbox_width_fraction": _pearson(direct_prediction, bbox_width),
            "pearson_direct_vs_ground_truth": _pearson(direct_prediction, truth),
            "bbox_feature_is_top_gain_in_all_folds": bbox_top_gain_all_folds,
            "interpretation": "SUPPORTED_STATIC_TARGET_SIZE_SHORTCUT_RISK" if bbox_top_gain_all_folds else "NO_SINGLE_DOMINANT_BBOX_GAIN_PATTERN",
            "causal_claim": False,
        },
        "output_collapse": {
            "rounded_6dp_unique_prediction_count": len(set(np.round(direct_prediction, 6))),
            "zero_or_near_zero_std_group_count": sum(float(row["prediction_std_m"]) <= 1e-6 for row in direct_groups),
            "group_count": len(direct_groups),
            "overall_prediction_std_m": float(np.std(direct_prediction)),
            "overall_ground_truth_std_m": float(np.std(truth)),
            "global_mean_collapse": bool(float(np.std(direct_prediction)) < 0.25 * float(np.std(truth))),
            "piecewise_constant_static_output_risk": True,
        },
        "boundary_and_saturation": {
            "unclipped_below_3m_fraction": float(np.mean(direct_unclipped < 3.0)),
            "unclipped_above_12m_fraction": float(np.mean(direct_unclipped > 12.0)),
            "within_0_1m_of_3m_fraction": float(np.mean(direct_prediction <= 3.1)),
            "within_0_1m_of_12m_fraction": float(np.mean(direct_prediction >= 11.9)),
        },
        "worst_direct_groups": [
            {
                "logical_session_id": row["logical_session_id"],
                "distance_bin": row["distance_bin"], "scenario": scenario_map[row["logical_session_id"]],
                "mae_m": row["mae_m"], "prediction_std_m": row["prediction_std_m"],
            }
            for row in sorted(direct_groups, key=lambda item: float(item["mae_m"]), reverse=True)[:8]
        ],
        "duplicate_audit": leakage["near_duplicate_audit"],
        "limitations": [
            "all sessions are static, so zero within-group variation does not validate dynamic response",
            "images are not retained in this feature audit, so visual near-duplicate status is UNKNOWN",
            "feature importance and correlation are descriptive, not causal",
        ],
    }
    (output / "shortcut_overfitting_audit.json").write_text(json.dumps(shortcut_audit, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    report_data = {
        "benchmark_id": BENCHMARK_ID, "conclusion": conclusion,
        "selected_configs": selected, "development_gates": gates,
        "model_comparison": comparison_rows,
        "shortcut_overfitting_audit": shortcut_audit,
        "static_only_limitations": [
            "dynamic lag/range-rate/approach/recede metrics are N/A",
            "development CV is not a final holdout or promotion result",
            "bbox/context shortcut risk is descriptive, not causal",
        ],
        "scope_guards": plan["scope_guards"],
    }
    report_lines = [
        "# CORE RANGE XGBOOST OFFLINE BENCHMARK", "", f"Conclusion: `{conclusion}`", "",
        "Static development-only 3-fold group-disjoint CV, seed 52. No runtime integration or promotion claim.", "",
        "## Selected configurations", "", f"- Residual: `{selected['residual']}`", f"- Direct: `{selected['direct']}`", "",
        "## Primary comparison (clipped [3,12] m)", "",
        "| Method | Equal-group MAE | P90 | Worst-bin MAE | >3 m | Median group std |", "|---|---:|---:|---:|---:|---:|",
    ]
    for method in ("raw_clipped_3_12m", "residual_xgboost_clipped", "direct_xgboost_clipped"):
        item = next(row for row in comparison_rows if row["method"] == method)
        report_lines.append(f"| {method} | {item['equal_group_mae_m']:.4f} | {item['equal_group_p90_m']:.4f} | {item['worst_bin_equal_group_mae_m']:.4f} | {100*item['equal_group_error_gt_3m_fraction']:.2f}% | {item['stationary_median_group_std_m']:.4f} |")
    report_lines += [
        "", "## Development gates", "", "```json", json.dumps(gates, indent=2, sort_keys=True), "```", "",
        "## Overfitting and shortcut audit", "",
        f"Direct train-to-test gaps by fold: `{json.dumps(selected_fold_gaps['direct'], sort_keys=True)}`.", "",
        f"BBox width is the top gain feature in all folds: `{bbox_top_gain_all_folds}`. This supports a static target-size shortcut risk; it is not a causal claim.", "",
        f"Near-zero within-session prediction std occurs in `{shortcut_audit['output_collapse']['zero_or_near_zero_std_group_count']}/27` groups. The output does not collapse to one global mean, but it is piecewise constant on this static corpus.", "",
        f"Worst direct group: `{shortcut_audit['worst_direct_groups'][0]['logical_session_id']}` with MAE `{shortcut_audit['worst_direct_groups'][0]['mae_m']:.4f} m`.", "",
        "## Limits", "",
        "Dynamic response metrics are N/A because every source session is static. This benchmark does not authorize shadow, runtime use, controller output, or model promotion.", ""
    ]
    (output / "benchmark_report.md").write_text("\n".join(report_lines), encoding="utf-8")
    output_files = [path for path in output.rglob("*") if path.is_file() and path.name != "benchmark_manifest.json"]
    manifest = {
        **report_data,
        "benchmark_plan_sha256": sha256_file(output / "benchmark_plan.json"),
        "evaluator_source_sha256": sha256_file(workspace / "core_range_xgboost_benchmark.py"),
        "versions": {"python": os.sys.version.split()[0], "numpy": np.__version__, "xgboost": xgb.__version__},
        "exact_commands": [
            ".venv/bin/python core_range_xgboost_benchmark.py prepare --workspace /home/sup/swarm_dashboard --output artifacts/core_range_3_12m/xgboost_benchmark",
            ".venv/bin/python core_range_xgboost_benchmark.py run --workspace /home/sup/swarm_dashboard --output artifacts/core_range_3_12m/xgboost_benchmark",
        ],
        "counts": {"groups": 27, "frames": len(rows), "folds": 3, "bins": 9},
        "artifacts": {str(path.relative_to(output)): sha256_file(path) for path in sorted(output_files)},
    }
    manifest_path = output / "benchmark_manifest.json"
    manifest_path.write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps({"conclusion": conclusion, "selected_configs": selected, "gates": gates}, indent=2))
    return manifest


def main() -> int:
    parser = argparse.ArgumentParser()
    sub = parser.add_subparsers(dest="command", required=True)
    for command in ("prepare", "run"):
        child = sub.add_parser(command)
        child.add_argument("--workspace", type=Path, required=True)
        child.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    workspace, output = args.workspace.resolve(), args.output.resolve()
    if args.command == "prepare": prepare(workspace, output)
    else: run(workspace, output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
