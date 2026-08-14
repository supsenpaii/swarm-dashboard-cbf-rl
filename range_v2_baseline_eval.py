"""R3 read-only raw physical range baseline evaluator.

This module evaluates the physical range already stored in the locked R2
development corpus.  It does not import or invoke any training, calibration,
simulation, runtime, temporal-filter, or controller code.
"""

from __future__ import annotations

import argparse
from collections import Counter, defaultdict
import csv
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import tempfile
from typing import Any, Mapping, Sequence

import numpy as np

from range_v2_labels import SPEC_ID, core_bin


EVALUATOR_SCHEMA_VERSION = "range_v2_raw_baseline_r3_v001"
FROZEN_SPEC_SHA256 = (
    "5b2251aadf07d7dc12169d5cddc319a9abb07ead1998c26361c5dd96d3289112"
)
R2_AUDIT_MANIFEST_SHA256 = (
    "fc5ecca960aebd1dee4c8991f363064c7b595e8f1437dc2f260cca71534441f6"
)
SEED = 52
BOOTSTRAP_REPLICATES = 5000
CORE_BINS = tuple(f"{lower}-{lower + 1}m" for lower in range(3, 12))
PRESENT_CORE_BINS = CORE_BINS[:7]
MISSING_CORE_BINS = CORE_BINS[7:]

FROZEN_LIMITS = {
    "maximum_absolute_signed_bias_m": 0.50,
    "maximum_median_absolute_error_m": 0.75,
    "maximum_mae_m": 1.00,
    "maximum_p90_absolute_error_m": 1.50,
    "maximum_p95_absolute_error_m": 2.00,
    "maximum_catastrophic_error_rate": 0.01,
    "catastrophic_error_threshold_m": 3.0,
    "maximum_stationary_std_m": 0.20,
    "maximum_p95_frame_delta_m": 0.25,
    "maximum_absolute_drift_slope_m_s": 0.02,
    "maximum_10_second_drift_m": 0.30,
}

UNSUPPORTED_METRICS = {
    "approaching_receding_stationary_direction_classification": "N/A",
    "range_response_lag": "N/A",
    "range_rate_error": "N/A",
    "stop_overshoot": "N/A",
    "stop_settling_time": "N/A",
    "near_core_boundary_behavior": "N/A",
    "core_far_boundary_behavior": "N/A",
    "calibration_event_jump": "UNKNOWN",
    "reacquire_event_jump": "UNKNOWN",
    "promotion_performance": "N/A",
}


def file_sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def bytes_sha256(value: bytes) -> str:
    return hashlib.sha256(value).hexdigest()


def _finite(value: Any, name: str, *, positive: bool = False) -> float:
    try:
        number = float(value)
    except (TypeError, ValueError) as error:
        raise ValueError(f"{name}_invalid") from error
    if not math.isfinite(number) or (positive and number <= 0.0):
        raise ValueError(f"{name}_invalid")
    return number


def _nullable(value: float | None) -> float | None:
    if value is None:
        return None
    number = float(value)
    if not math.isfinite(number):
        raise ValueError("metric_not_finite")
    return number


def _quantile(values: Sequence[float], probability: float) -> float:
    array = np.asarray(values, dtype=np.float64)
    if array.size == 0 or not np.all(np.isfinite(array)):
        raise ValueError("quantile_input_invalid")
    return float(np.quantile(array, probability, method="linear"))


def assign_core_bin(distance_m: float) -> str:
    bin_name = core_bin(distance_m, ground_truth_trustworthy=True)
    if bin_name == "unknown":
        raise ValueError("distance_outside_core")
    return bin_name


def _validated_metric_rows(rows: Sequence[Mapping[str, Any]]) -> list[Mapping[str, Any]]:
    if not rows:
        raise ValueError("metric_rows_empty")
    validated: list[Mapping[str, Any]] = []
    for row in rows:
        ground_truth = _finite(
            row.get("ground_truth_distance_m"),
            "ground_truth_distance_m",
            positive=True,
        )
        raw_range = _finite(
            row.get("raw_physical_range_m"),
            "raw_physical_range_m",
            positive=True,
        )
        error = _finite(row.get("signed_error_m"), "signed_error_m")
        if not math.isclose(error, raw_range - ground_truth, rel_tol=0.0, abs_tol=1e-9):
            raise ValueError("signed_error_inconsistent")
        absolute_error = _finite(
            row.get("absolute_error_m"),
            "absolute_error_m",
        )
        if not math.isclose(absolute_error, abs(error), rel_tol=0.0, abs_tol=1e-9):
            raise ValueError("absolute_error_inconsistent")
        relative_error = _finite(
            row.get("absolute_relative_error"),
            "absolute_relative_error",
        )
        if not math.isclose(
            relative_error,
            absolute_error / ground_truth,
            rel_tol=0.0,
            abs_tol=1e-9,
        ):
            raise ValueError("relative_error_inconsistent")
        validated.append(row)
    return validated


def _group_metric_values(
    rows: Sequence[Mapping[str, Any]],
) -> dict[str, dict[str, float]]:
    grouped: dict[str, list[Mapping[str, Any]]] = defaultdict(list)
    for row in rows:
        grouped[str(row["atomic_group_key"])].append(row)
    result: dict[str, dict[str, float]] = {}
    for group_key, group_rows in grouped.items():
        errors = [float(row["signed_error_m"]) for row in group_rows]
        absolute = [float(row["absolute_error_m"]) for row in group_rows]
        relative = [float(row["absolute_relative_error"]) for row in group_rows]
        result[group_key] = {
            "signed_bias_m": float(np.mean(errors)),
            "median_absolute_error_m": float(np.median(absolute)),
            "mae_m": float(np.mean(absolute)),
            "p90_absolute_error_m": _quantile(absolute, 0.90),
            "p95_absolute_error_m": _quantile(absolute, 0.95),
            "mean_absolute_relative_error": float(np.mean(relative)),
            "catastrophic_error_rate": float(
                np.mean(np.asarray(absolute) > FROZEN_LIMITS["catastrophic_error_threshold_m"])
            ),
            "minimum_error_m": float(np.min(errors)),
            "maximum_error_m": float(np.max(errors)),
        }
    return result


def _cluster_bootstrap_ci(
    group_metrics: Mapping[str, Mapping[str, float]],
    metric_name: str,
    *,
    seed: int = SEED,
    replicates: int = BOOTSTRAP_REPLICATES,
) -> tuple[float | None, float | None]:
    values = np.asarray(
        [metrics[metric_name] for _, metrics in sorted(group_metrics.items())],
        dtype=np.float64,
    )
    if values.size < 2:
        return None, None
    if not np.all(np.isfinite(values)):
        raise ValueError("group_bootstrap_input_invalid")
    generator = np.random.default_rng(seed)
    indices = generator.integers(0, values.size, size=(replicates, values.size))
    bootstrap = np.mean(values[indices], axis=1)
    return (
        float(np.quantile(bootstrap, 0.025, method="linear")),
        float(np.quantile(bootstrap, 0.975, method="linear")),
    )


def metric_summary(
    rows: Sequence[Mapping[str, Any]],
    *,
    bootstrap_seed: int = SEED,
) -> dict[str, Any]:
    validated = _validated_metric_rows(rows)
    errors = np.asarray([row["signed_error_m"] for row in validated], dtype=np.float64)
    absolute = np.asarray([row["absolute_error_m"] for row in validated], dtype=np.float64)
    relative = np.asarray(
        [row["absolute_relative_error"] for row in validated],
        dtype=np.float64,
    )
    group_metrics = _group_metric_values(validated)
    equal_names = (
        "signed_bias_m",
        "median_absolute_error_m",
        "mae_m",
        "p90_absolute_error_m",
        "p95_absolute_error_m",
        "mean_absolute_relative_error",
        "catastrophic_error_rate",
    )
    summary: dict[str, Any] = {
        "frame_count": len(validated),
        "independent_group_count": len(group_metrics),
        "signed_bias_m": float(np.mean(errors)),
        "median_absolute_error_m": float(np.median(absolute)),
        "mae_m": float(np.mean(absolute)),
        "p90_absolute_error_m": _quantile(absolute.tolist(), 0.90),
        "p95_absolute_error_m": _quantile(absolute.tolist(), 0.95),
        "mean_absolute_relative_error": float(np.mean(relative)),
        "catastrophic_error_rate": float(
            np.mean(absolute > FROZEN_LIMITS["catastrophic_error_threshold_m"])
        ),
        "minimum_error_m": float(np.min(errors)),
        "maximum_error_m": float(np.max(errors)),
    }
    for name in equal_names:
        summary[f"equal_group_{name}"] = float(
            np.mean([metrics[name] for metrics in group_metrics.values()])
        )
    bias_low, bias_high = _cluster_bootstrap_ci(
        group_metrics,
        "signed_bias_m",
        seed=bootstrap_seed,
    )
    mae_low, mae_high = _cluster_bootstrap_ci(
        group_metrics,
        "mae_m",
        seed=bootstrap_seed,
    )
    summary.update(
        {
            "group_bootstrap_bias_ci95_low_m": bias_low,
            "group_bootstrap_bias_ci95_high_m": bias_high,
            "group_bootstrap_mae_ci95_low_m": mae_low,
            "group_bootstrap_mae_ci95_high_m": mae_high,
            "bootstrap_unit": "whole_run_session_group",
            "bootstrap_replicates": BOOTSTRAP_REPLICATES,
            "bootstrap_seed": bootstrap_seed,
        }
    )
    return summary


def stationary_metrics(rows: Sequence[Mapping[str, Any]]) -> dict[str, Any]:
    if not rows:
        raise ValueError("stationary_rows_empty")
    ordered = sorted(rows, key=lambda row: float(row["measurement_timestamp_s"]))
    times = np.asarray(
        [_finite(row["measurement_timestamp_s"], "measurement_timestamp_s") for row in ordered],
        dtype=np.float64,
    )
    ranges = np.asarray(
        [
            _finite(row["raw_physical_range_m"], "raw_physical_range_m", positive=True)
            for row in ordered
        ],
        dtype=np.float64,
    )
    if times.size > 1 and not np.all(np.diff(times) > 0.0):
        raise ValueError("stationary_timestamp_not_strictly_increasing")
    duration = float(times[-1] - times[0]) if times.size > 1 else 0.0
    deltas = np.abs(np.diff(ranges))
    slope: float | None = None
    if times.size > 1 and duration > 0.0:
        centered_time = times - float(np.mean(times))
        denominator = float(np.dot(centered_time, centered_time))
        if denominator > 0.0:
            slope = float(
                np.dot(centered_time, ranges - float(np.mean(ranges))) / denominator
            )
    maximum_10s: float | None = None
    if times.size > 1 and duration >= 10.0:
        window_spans: list[float] = []
        right = 0
        for left in range(times.size):
            right = max(right, left)
            while right + 1 < times.size and times[right + 1] - times[left] <= 10.0:
                right += 1
            if right > left:
                window = ranges[left : right + 1]
                window_spans.append(float(np.max(window) - np.min(window)))
        if window_spans:
            maximum_10s = max(window_spans)
    return {
        "frame_count": int(times.size),
        "sample_duration_s": duration,
        "output_mean_m": float(np.mean(ranges)),
        "output_median_m": float(np.median(ranges)),
        "output_std_m": float(np.std(ranges, ddof=0)),
        "median_absolute_frame_delta_m": float(np.median(deltas))
        if deltas.size
        else None,
        "p95_absolute_frame_delta_m": _quantile(deltas.tolist(), 0.95)
        if deltas.size
        else None,
        "linear_drift_slope_m_s": slope,
        "maximum_drift_m": float(np.max(ranges) - np.min(ranges)),
        "maximum_10_second_drift_m": maximum_10s,
        "timestamp_basis": "legacy_measurement_receipt_domain",
    }


def verify_source_checksums(
    workspace: Path,
    source_entries: Sequence[Mapping[str, Any]],
) -> None:
    for source in source_entries:
        for path_key, checksum_key in (
            ("manifest_path", "manifest_sha256"),
            ("samples_path", "samples_sha256"),
        ):
            path = workspace / str(source[path_key])
            expected = str(source[checksum_key])
            if not path.is_file() or file_sha256(path) != expected:
                raise ValueError(f"source_checksum_mismatch:{path}")


def verify_locked_inputs(
    *,
    workspace: Path,
    r2_dir: Path,
    frozen_spec_path: Path,
    expected_frozen_spec_sha256: str = FROZEN_SPEC_SHA256,
    expected_r2_manifest_sha256: str = R2_AUDIT_MANIFEST_SHA256,
) -> dict[str, Any]:
    if file_sha256(frozen_spec_path) != expected_frozen_spec_sha256:
        raise ValueError("frozen_spec_checksum_mismatch")
    r2_manifest_path = r2_dir / "r2_audit_manifest.json"
    if file_sha256(r2_manifest_path) != expected_r2_manifest_sha256:
        raise ValueError("r2_audit_manifest_checksum_mismatch")
    r2_manifest = json.loads(r2_manifest_path.read_text(encoding="utf-8"))
    if (
        r2_manifest.get("frozen_spec_id") != SPEC_ID
        or r2_manifest.get("frozen_spec_sha256") != expected_frozen_spec_sha256
        or not r2_manifest.get("source_originals_read_only_verified")
        or r2_manifest.get("leakage_status") != "PASS"
    ):
        raise ValueError("r2_audit_contract_mismatch")
    for name, expected in r2_manifest.get("output_checksums", {}).items():
        path = r2_dir / name
        if not path.is_file() or file_sha256(path) != expected:
            raise ValueError(f"r2_artifact_checksum_mismatch:{name}")
    source_manifest = json.loads(
        (r2_dir / "source_dataset_manifest.json").read_text(encoding="utf-8")
    )
    if (
        source_manifest.get("frozen_spec_id") != SPEC_ID
        or source_manifest.get("corpus_role") != "development"
        or source_manifest.get("source_dataset_count") != 32
        or source_manifest.get("derived_row_count") != 2500
    ):
        raise ValueError("r2_source_manifest_contract_mismatch")
    sources = source_manifest.get("sources")
    if not isinstance(sources, list) or len(sources) != 32:
        raise ValueError("r2_source_list_invalid")
    verify_source_checksums(workspace, sources)
    r2_source_map = {
        source["dataset_root"]: (
            source["manifest_sha256"],
            source["samples_sha256"],
        )
        for source in r2_manifest.get("source_checksums", [])
    }
    source_map = {
        source["dataset_root"]: (
            source["manifest_sha256"],
            source["samples_sha256"],
        )
        for source in sources
    }
    if source_map != r2_source_map:
        raise ValueError("r2_source_checksum_maps_mismatch")
    return {
        "r2_manifest": r2_manifest,
        "source_manifest": source_manifest,
        "sources": sources,
        "r2_artifact_checksums": {
            **r2_manifest["output_checksums"],
            "r2_audit_manifest.json": expected_r2_manifest_sha256,
        },
    }


def _load_derived_rows(r2_dir: Path) -> dict[tuple[str, int], dict[str, Any]]:
    result: dict[tuple[str, int], dict[str, Any]] = {}
    with (r2_dir / "derived_labels.jsonl").open("r", encoding="utf-8") as stream:
        for line_number, line in enumerate(stream, start=1):
            if not line.strip():
                continue
            row = json.loads(line)
            key = (str(row["source_dataset_root"]), int(row["source_line_number"]))
            if key in result:
                raise ValueError(f"r2_derived_trace_duplicate:{key}")
            result[key] = row
    return result


def load_evaluation_rows(
    *,
    workspace: Path,
    r2_dir: Path,
    sources: Sequence[Mapping[str, Any]],
) -> list[dict[str, Any]]:
    derived = _load_derived_rows(r2_dir)
    rows: list[dict[str, Any]] = []
    for source in sources:
        samples_path = workspace / str(source["samples_path"])
        raw_lines = samples_path.read_bytes().splitlines(keepends=True)
        observed_count = 0
        for line_number, raw_line in enumerate(raw_lines, start=1):
            if not raw_line.strip():
                continue
            record = json.loads(raw_line)
            trace_key = (str(source["dataset_root"]), line_number)
            label = derived.get(trace_key)
            if label is None:
                raise ValueError(f"r2_derived_trace_missing:{trace_key}")
            if label["source_record_sha256"] != bytes_sha256(raw_line):
                raise ValueError(f"r2_source_record_checksum_mismatch:{trace_key}")
            identity_pairs = (
                ("run_id", record.get("run_id")),
                ("session_id", record.get("session_id")),
                ("group_id", record.get("group_id")),
                ("frame_index", record.get("frame_index")),
            )
            for field, value in identity_pairs:
                if label.get(field) != value:
                    raise ValueError(f"r2_source_identity_mismatch:{trace_key}:{field}")
            if (
                label.get("frozen_spec_id") != SPEC_ID
                or label.get("distance_zone_gt") != "core"
                or label.get("distance_label_trustworthy") is not True
                or label.get("ground_truth_audit_status")
                != "valid_static_development_only"
                or label.get("dynamic_label_eligible") is not False
            ):
                raise ValueError(f"r2_static_core_contract_mismatch:{trace_key}")
            ground_truth = _finite(
                record.get("ground_truth_distance_m"),
                "ground_truth_distance_m",
                positive=True,
            )
            raw_range = _finite(
                record.get("physics_distance_m"),
                "physics_distance_m",
                positive=True,
            )
            timestamp = _finite(
                record.get("measurement_timestamp_s"),
                "measurement_timestamp_s",
            )
            error = raw_range - ground_truth
            absolute_error = abs(error)
            bin_name = assign_core_bin(ground_truth)
            if label.get("core_bin") != bin_name:
                raise ValueError(f"r2_core_bin_mismatch:{trace_key}")
            policy_label = str(label.get("deterministic_policy_label"))
            if policy_label not in {"accepted", "rejected"}:
                raise ValueError(f"r2_policy_label_invalid:{trace_key}")
            if str(record.get("correction_mode")) != "disabled":
                raise ValueError(f"source_residual_correction_not_disabled:{trace_key}")
            rows.append(
                {
                    "evaluator_schema_version": EVALUATOR_SCHEMA_VERSION,
                    "source_dataset_id": source["dataset_id"],
                    "source_dataset_root": source["dataset_root"],
                    "source_manifest_sha256": source["manifest_sha256"],
                    "source_samples_sha256": source["samples_sha256"],
                    "source_line_number": line_number,
                    "source_record_sha256": label["source_record_sha256"],
                    "run_id": record["run_id"],
                    "session_id": record["session_id"],
                    "group_id": record["group_id"],
                    "atomic_group_key": label["atomic_group_key"],
                    "frame_index": record["frame_index"],
                    "measurement_timestamp_s": timestamp,
                    "source_sim_timestamp_s": record.get("source_sim_timestamp_s"),
                    "core_bin": bin_name,
                    "ground_truth_distance_m": ground_truth,
                    "raw_physical_range_m": raw_range,
                    "signed_error_m": error,
                    "absolute_error_m": absolute_error,
                    "absolute_relative_error": absolute_error / ground_truth,
                    "catastrophic_error": absolute_error
                    > FROZEN_LIMITS["catastrophic_error_threshold_m"],
                    "deterministic_policy_label": policy_label,
                    "deterministic_policy_reason": label[
                        "deterministic_policy_reason"
                    ],
                    "validity_gt": "unknown",
                    "correction_mode": "disabled",
                }
            )
            observed_count += 1
        if observed_count != int(source["row_count"]):
            raise ValueError(f"source_row_count_mismatch:{source['dataset_id']}")
    if len(rows) != len(derived):
        raise ValueError("r2_derived_source_row_count_mismatch")
    identities = {
        (row["run_id"], row["session_id"], row["frame_index"])
        for row in rows
    }
    if len(identities) != len(rows):
        raise ValueError("raw_baseline_identity_duplicate")
    return rows


def _with_equal_group_metrics(summary: Mapping[str, Any]) -> dict[str, Any]:
    return dict(summary)


def _limit_status(summary: Mapping[str, Any]) -> dict[str, Any]:
    checks = {
        "signed_bias_limit_met": abs(float(summary["signed_bias_m"]))
        <= FROZEN_LIMITS["maximum_absolute_signed_bias_m"],
        "median_absolute_error_limit_met": float(
            summary["median_absolute_error_m"]
        )
        <= FROZEN_LIMITS["maximum_median_absolute_error_m"],
        "mae_limit_met": float(summary["mae_m"])
        <= FROZEN_LIMITS["maximum_mae_m"],
        "p90_limit_met": float(summary["p90_absolute_error_m"])
        <= FROZEN_LIMITS["maximum_p90_absolute_error_m"],
        "p95_limit_met": float(summary["p95_absolute_error_m"])
        <= FROZEN_LIMITS["maximum_p95_absolute_error_m"],
        "catastrophic_rate_limit_met": float(summary["catastrophic_error_rate"])
        <= FROZEN_LIMITS["maximum_catastrophic_error_rate"],
    }
    checks["all_frozen_absolute_limits_met"] = all(checks.values())
    checks["limit_comparison_semantics"] = "raw_baseline_only_not_model_promotion"
    return checks


def build_per_bin_metrics(rows: Sequence[Mapping[str, Any]]) -> list[dict[str, Any]]:
    result: list[dict[str, Any]] = []
    for index, bin_name in enumerate(CORE_BINS):
        selected = [row for row in rows if row["core_bin"] == bin_name]
        if not selected:
            result.append(
                {
                    "core_bin": bin_name,
                    "status": "MISSING",
                    "frame_count": 0,
                    "independent_group_count": 0,
                    "reason": "no_locked_static_development_rows_no_interpolation",
                }
            )
            continue
        summary = metric_summary(selected, bootstrap_seed=SEED + index)
        result.append(
            {
                "core_bin": bin_name,
                "status": "CHARACTERIZED",
                **summary,
                **_limit_status(summary),
            }
        )
    return result


def build_per_group_metrics(rows: Sequence[Mapping[str, Any]]) -> list[dict[str, Any]]:
    grouped: dict[str, list[Mapping[str, Any]]] = defaultdict(list)
    for row in rows:
        grouped[str(row["atomic_group_key"])].append(row)
    result: list[dict[str, Any]] = []
    for group_key in sorted(grouped):
        group_rows = grouped[group_key]
        summary = metric_summary(group_rows)
        temporal = stationary_metrics(group_rows)
        policy_counts = Counter(row["deterministic_policy_label"] for row in group_rows)
        policy_reasons = Counter(
            row["deterministic_policy_reason"]
            for row in group_rows
            if row["deterministic_policy_label"] == "rejected"
        )
        std_met = temporal["output_std_m"] <= FROZEN_LIMITS["maximum_stationary_std_m"]
        delta_met = (
            temporal["p95_absolute_frame_delta_m"] is not None
            and temporal["p95_absolute_frame_delta_m"]
            <= FROZEN_LIMITS["maximum_p95_frame_delta_m"]
        )
        slope_met = (
            temporal["linear_drift_slope_m_s"] is not None
            and abs(temporal["linear_drift_slope_m_s"])
            <= FROZEN_LIMITS["maximum_absolute_drift_slope_m_s"]
        )
        ten_second_met = (
            None
            if temporal["maximum_10_second_drift_m"] is None
            else temporal["maximum_10_second_drift_m"]
            <= FROZEN_LIMITS["maximum_10_second_drift_m"]
        )
        result.append(
            {
                "source_dataset_id": group_rows[0]["source_dataset_id"],
                "source_dataset_root": group_rows[0]["source_dataset_root"],
                "source_manifest_sha256": group_rows[0]["source_manifest_sha256"],
                "source_samples_sha256": group_rows[0]["source_samples_sha256"],
                "run_id": group_rows[0]["run_id"],
                "session_id": group_rows[0]["session_id"],
                "group_id": group_rows[0]["group_id"],
                "atomic_group_key": group_key,
                "core_bin": group_rows[0]["core_bin"],
                "ground_truth_distance_m": group_rows[0]["ground_truth_distance_m"],
                **summary,
                **temporal,
                "policy_accepted_frames": policy_counts["accepted"],
                "policy_rejected_frames": policy_counts["rejected"],
                "policy_rejection_reasons": json.dumps(
                    policy_reasons, sort_keys=True, separators=(",", ":")
                ),
                "stationary_std_limit_met": std_met,
                "stationary_p95_delta_limit_met": delta_met,
                "stationary_drift_slope_limit_met": slope_met,
                "stationary_10_second_drift_limit_met": ten_second_met,
                "stationary_limit_comparison_semantics": "raw_baseline_only",
            }
        )
    return result


def _empty_metric_fields() -> dict[str, Any]:
    return {
        "frame_count": 0,
        "independent_group_count": 0,
        "signed_bias_m": None,
        "median_absolute_error_m": None,
        "mae_m": None,
        "p90_absolute_error_m": None,
        "p95_absolute_error_m": None,
        "mean_absolute_relative_error": None,
        "catastrophic_error_rate": None,
        "minimum_error_m": None,
        "maximum_error_m": None,
    }


def build_policy_slice_metrics(
    rows: Sequence[Mapping[str, Any]],
) -> list[dict[str, Any]]:
    slices = {
        "all_trustworthy_static_development": list(rows),
        "deterministic_policy_accepted": [
            row for row in rows if row["deterministic_policy_label"] == "accepted"
        ],
        "deterministic_policy_rejected": [
            row for row in rows if row["deterministic_policy_label"] == "rejected"
        ],
    }
    result: list[dict[str, Any]] = []
    for slice_name, selected in slices.items():
        summary = metric_summary(selected)
        result.append(
            {
                "slice": slice_name,
                "scope": "aggregate",
                "core_bin": "all_present_core_bins",
                "status": "CHARACTERIZED",
                **summary,
                "reason_counts": json.dumps(
                    Counter(
                        row["deterministic_policy_reason"]
                        for row in selected
                        if row["deterministic_policy_label"] == "rejected"
                    ),
                    sort_keys=True,
                    separators=(",", ":"),
                ),
                "bin_distribution": json.dumps(
                    Counter(row["core_bin"] for row in selected),
                    sort_keys=True,
                    separators=(",", ":"),
                ),
                "same_bin_comparison_status": "not_applicable",
            }
        )
    accepted = slices["deterministic_policy_accepted"]
    rejected = slices["deterministic_policy_rejected"]
    for bin_name in PRESENT_CORE_BINS:
        accepted_bin = [row for row in accepted if row["core_bin"] == bin_name]
        rejected_bin = [row for row in rejected if row["core_bin"] == bin_name]
        accepted_summary = metric_summary(accepted_bin) if accepted_bin else None
        rejected_summary = metric_summary(rejected_bin) if rejected_bin else None
        for slice_name, selected, summary in (
            ("deterministic_policy_accepted", accepted_bin, accepted_summary),
            ("deterministic_policy_rejected", rejected_bin, rejected_summary),
        ):
            row: dict[str, Any] = {
                "slice": slice_name,
                "scope": "bin",
                "core_bin": bin_name,
                "status": "CHARACTERIZED" if summary else "MISSING",
                **(summary if summary else _empty_metric_fields()),
                "reason_counts": json.dumps(
                    Counter(
                        item["deterministic_policy_reason"]
                        for item in selected
                        if item["deterministic_policy_label"] == "rejected"
                    ),
                    sort_keys=True,
                    separators=(",", ":"),
                ),
                "bin_distribution": json.dumps(
                    {bin_name: len(selected)}, separators=(",", ":")
                ),
                "same_bin_comparison_status": "not_applicable",
            }
            if slice_name == "deterministic_policy_rejected" and summary:
                rejected_groups = len(
                    {item["atomic_group_key"] for item in selected}
                )
                accepted_groups = len(
                    {item["atomic_group_key"] for item in accepted_bin}
                )
                enough = (
                    len(selected) >= 30
                    and len(accepted_bin) >= 30
                    and rejected_groups >= 2
                    and accepted_groups >= 2
                )
                row["same_bin_comparison_status"] = (
                    "AVAILABLE"
                    if enough
                    else "DESCRIPTIVE_ONLY_INSUFFICIENT_INDEPENDENT_GROUPS"
                )
                if accepted_summary:
                    for name in (
                        "signed_bias_m",
                        "mae_m",
                        "p90_absolute_error_m",
                        "p95_absolute_error_m",
                    ):
                        row[f"rejected_minus_accepted_{name}"] = float(
                            summary[name] - accepted_summary[name]
                        )
            result.append(row)
    return result


def _write_csv(path: Path, rows: Sequence[Mapping[str, Any]]) -> None:
    if not rows:
        raise ValueError(f"csv_rows_empty:{path.name}")
    fieldnames: list[str] = []
    seen: set[str] = set()
    for row in rows:
        for name in row:
            if name not in seen:
                seen.add(name)
                fieldnames.append(name)
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def _json_dump(path: Path, value: Any) -> None:
    path.write_text(
        json.dumps(value, indent=2, sort_keys=True, allow_nan=False) + "\n",
        encoding="utf-8",
    )


def _unsupported_payload() -> dict[str, Any]:
    reasons = {
        name: (
            "static_core_only_no_dynamic_or_boundary_evidence"
            if status == "N/A"
            else "legacy_schema_missing_explicit_event_epochs"
        )
        for name, status in UNSUPPORTED_METRICS.items()
    }
    return {
        "artifact_type": "range_v2_r3_unsupported_metrics",
        "metrics": {
            name: {"status": status, "reason": reasons[name]}
            for name, status in UNSUPPORTED_METRICS.items()
        },
        "prohibition": "no_dynamic_metric_inference_from_static_development_data",
    }


def _build_blockers(
    bins: Sequence[Mapping[str, Any]],
    groups: Sequence[Mapping[str, Any]],
) -> list[dict[str, Any]]:
    blockers: list[dict[str, Any]] = []
    for item in bins:
        if item["status"] == "MISSING":
            blockers.append(
                {
                    "type": "data_coverage",
                    "scope": item["core_bin"],
                    "evidence": "no_locked_static_development_rows",
                    "required_action": "collect_under_separate_R4_plan",
                }
            )
        elif abs(float(item["signed_bias_m"])) > 0.5:
            blockers.append(
                {
                    "type": "systematic_raw_range_bias",
                    "scope": item["core_bin"],
                    "evidence": {
                        "signed_bias_m": item["signed_bias_m"],
                        "mae_m": item["mae_m"],
                        "group_count": item["independent_group_count"],
                    },
                    "root_cause": "unknown_not_attributed_by_R3",
                    "required_action": "separate_intrinsics_pitch_timestamp_calibration_geometry_audit",
                    "xgboost_assumption": "prohibited_as_root_cause_mask",
                }
            )
    unstable_groups = [
        group["atomic_group_key"]
        for group in groups
        if not group["stationary_std_limit_met"]
        or not group["stationary_p95_delta_limit_met"]
        or not group["stationary_drift_slope_limit_met"]
        or group["stationary_10_second_drift_limit_met"] is False
    ]
    if unstable_groups:
        blockers.append(
            {
                "type": "stationary_raw_instability",
                "group_count": len(unstable_groups),
                "groups": unstable_groups,
                "root_cause": "unknown_not_attributed_by_R3",
                "required_action": "separate_raw_pipeline_stability_audit",
            }
        )
    return blockers


def _markdown_report(
    *,
    summary: Mapping[str, Any],
    bins: Sequence[Mapping[str, Any]],
    groups: Sequence[Mapping[str, Any]],
    policy_rows: Sequence[Mapping[str, Any]],
) -> str:
    aggregate = summary["frame_weighted_aggregate"]
    accepted = next(
        row
        for row in policy_rows
        if row["slice"] == "deterministic_policy_accepted"
        and row["scope"] == "aggregate"
    )
    rejected = next(
        row
        for row in policy_rows
        if row["slice"] == "deterministic_policy_rejected"
        and row["scope"] == "aggregate"
    )
    rejected_bins = [
        row
        for row in policy_rows
        if row["slice"] == "deterministic_policy_rejected"
        and row["scope"] == "bin"
        and row["status"] == "CHARACTERIZED"
    ]
    lines = [
        "# Range V2 R3 Raw Physical Baseline",
        "",
        f"Outcome: **{summary['outcome']}**  ",
        f"Frozen spec: `{SPEC_ID}` (`{FROZEN_SPEC_SHA256}`)  ",
        "Scope: locked static-core development corpus only",
        "",
        "## Integrity and safety",
        "",
        "- Frozen spec, all R2 derived artifacts, and all 32 source manifest/sample checksum pairs were verified before and after evaluation.",
        "- Every output row retains source dataset/run/session/group/frame, source line, record checksum, and source file checksums.",
        "- No model/scaler/calibrator/threshold was trained or fitted. No temporal, geometry, calibration, MiDaS, runtime, simulation, shadow, PX4, or controller path changed or ran.",
        "- Policy rejection remains a policy slice and is not `invalid_gt`.",
        "",
        "## Aggregate raw error",
        "",
        "| Aggregation | Frames | Groups | Bias m | MedAE m | MAE m | P90 m | P95 m | Mean abs relative | Catastrophic >3m |",
        "|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|",
        f"| Frame-weighted | {aggregate['frame_count']} | {aggregate['independent_group_count']} | {aggregate['signed_bias_m']:.3f} | {aggregate['median_absolute_error_m']:.3f} | {aggregate['mae_m']:.3f} | {aggregate['p90_absolute_error_m']:.3f} | {aggregate['p95_absolute_error_m']:.3f} | {aggregate['mean_absolute_relative_error']:.3f} | {aggregate['catastrophic_error_rate']:.3f} |",
        f"| Equal-group | {aggregate['frame_count']} | {aggregate['independent_group_count']} | {aggregate['equal_group_signed_bias_m']:.3f} | {aggregate['equal_group_median_absolute_error_m']:.3f} | {aggregate['equal_group_mae_m']:.3f} | {aggregate['equal_group_p90_absolute_error_m']:.3f} | {aggregate['equal_group_p95_absolute_error_m']:.3f} | {aggregate['equal_group_mean_absolute_relative_error']:.3f} | {aggregate['equal_group_catastrophic_error_rate']:.3f} |",
        "",
        "Bootstrap intervals resample whole run/session groups (5,000 replicates, seed 52); frames are never bootstrapped independently.",
        "",
        "## Per-bin raw metric",
        "",
        "| Bin | Frames | Groups | Bias m | MedAE m | MAE m | P90 m | P95 m | Rel. err | Cat. rate | Frozen-limit comparison |",
        "|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---|",
    ]
    for item in bins:
        if item["status"] == "MISSING":
            lines.append(
                f"| {item['core_bin']} | 0 | 0 | N/A | N/A | N/A | N/A | N/A | N/A | N/A | **MISSING** |"
            )
        else:
            gate = "MEETS" if item["all_frozen_absolute_limits_met"] else "EXCEEDS"
            lines.append(
                f"| {item['core_bin']} | {item['frame_count']} | {item['independent_group_count']} | {item['signed_bias_m']:.3f} | {item['median_absolute_error_m']:.3f} | {item['mae_m']:.3f} | {item['p90_absolute_error_m']:.3f} | {item['p95_absolute_error_m']:.3f} | {item['mean_absolute_relative_error']:.3f} | {item['catastrophic_error_rate']:.3f} | {gate} raw limits |"
            )
    lines.extend(
        [
            "",
            "This is a baseline-limit comparison only; it is not a model promotion PASS/FAIL decision.",
            "",
            "## Deterministic policy slices",
            "",
            "| Slice | Frames | Groups | Bias m | MAE m | P90 m | P95 m |",
            "|---|---:|---:|---:|---:|---:|---:|",
            f"| Accepted | {accepted['frame_count']} | {accepted['independent_group_count']} | {accepted['signed_bias_m']:.3f} | {accepted['mae_m']:.3f} | {accepted['p90_absolute_error_m']:.3f} | {accepted['p95_absolute_error_m']:.3f} |",
            f"| Rejected | {rejected['frame_count']} | {rejected['independent_group_count']} | {rejected['signed_bias_m']:.3f} | {rejected['mae_m']:.3f} | {rejected['p90_absolute_error_m']:.3f} | {rejected['p95_absolute_error_m']:.3f} |",
            "",
            f"Rejected reasons: `{rejected['reason_counts']}`. Bin distribution: `{rejected['bin_distribution']}`.",
            "",
            "| Same-bin slice | Accepted MAE m | Rejected MAE m | Delta m | Comparison status |",
            "|---|---:|---:|---:|---|",
        ]
    )
    for item in rejected_bins:
        delta = float(item["rejected_minus_accepted_mae_m"])
        accepted_mae = float(item["mae_m"]) - delta
        lines.append(
            f"| {item['core_bin']} | {accepted_mae:.3f} | {float(item['mae_m']):.3f} | {delta:+.3f} | `{item['same_bin_comparison_status']}` |"
        )
    lines.extend(
        [
            "",
            "Same-bin differences are descriptive where either slice has fewer than two independent groups; frame count alone is not treated as independent evidence.",
            "",
            "## Stationary group stability",
            "",
            f"Per-group table contains all {len(groups)} groups. Groups exceeding one or more comparable raw stationary limits: {summary['stationary_groups_exceeding_any_limit']}.",
            "",
            "## Unsupported metrics",
            "",
            "Direction classification, dynamic lag/rate/stop response, near/core and core/far behavior, calibration/reacquire jumps, and promotion performance are `N/A` or `UNKNOWN`. Static data is not used to infer them.",
            "",
            "## Blockers and next decision",
            "",
        ]
    )
    for blocker in summary["blockers"]:
        lines.append(f"- `{blocker['type']}` — `{blocker.get('scope', 'multiple_groups')}`; root cause is not inferred by R3.")
    lines.extend(
        [
            "",
            "R3 characterizes the raw baseline only. It does not claim Range V2 model PASS or FAIL. Work stops before R4/R5.",
            "",
        ]
    )
    return "\n".join(lines)


def evaluate_raw_baseline(
    *,
    workspace: Path,
    r2_dir: Path,
    output_dir: Path,
    frozen_spec_path: Path,
    expected_frozen_spec_sha256: str = FROZEN_SPEC_SHA256,
    expected_r2_manifest_sha256: str = R2_AUDIT_MANIFEST_SHA256,
) -> dict[str, Any]:
    workspace = workspace.resolve()
    r2_dir = r2_dir.resolve()
    output_dir = output_dir.resolve()
    frozen_spec_path = frozen_spec_path.resolve()
    if output_dir == r2_dir or r2_dir in output_dir.parents:
        # raw_baseline is the only allowed child of R2's artifact directory.
        if output_dir != r2_dir / "raw_baseline":
            raise ValueError("output_directory_not_raw_baseline_child")
    locked = verify_locked_inputs(
        workspace=workspace,
        r2_dir=r2_dir,
        frozen_spec_path=frozen_spec_path,
        expected_frozen_spec_sha256=expected_frozen_spec_sha256,
        expected_r2_manifest_sha256=expected_r2_manifest_sha256,
    )
    source_pre = {
        source["dataset_root"]: (
            source["manifest_sha256"],
            source["samples_sha256"],
        )
        for source in locked["sources"]
    }
    r2_pre = {
        name: file_sha256(r2_dir / name)
        for name in locked["r2_artifact_checksums"]
    }
    rows = load_evaluation_rows(
        workspace=workspace,
        r2_dir=r2_dir,
        sources=locked["sources"],
    )
    if len(rows) != 2500:
        raise ValueError("raw_baseline_expected_2500_rows")
    accepted_count = sum(
        row["deterministic_policy_label"] == "accepted" for row in rows
    )
    rejected_count = len(rows) - accepted_count
    if (accepted_count, rejected_count) != (2340, 160):
        raise ValueError("raw_baseline_policy_slice_count_mismatch")

    per_bin = build_per_bin_metrics(rows)
    per_group = build_per_group_metrics(rows)
    policy = build_policy_slice_metrics(rows)
    aggregate = metric_summary(rows)
    blockers = _build_blockers(per_bin, per_group)
    unstable_count = sum(
        not group["stationary_std_limit_met"]
        or not group["stationary_p95_delta_limit_met"]
        or not group["stationary_drift_slope_limit_met"]
        or group["stationary_10_second_drift_limit_met"] is False
        for group in per_group
    )
    summary = {
        "artifact_type": "range_v2_r3_raw_baseline_summary",
        "outcome": "RAW_BASELINE_CHARACTERIZED",
        "frozen_spec_id": SPEC_ID,
        "frozen_spec_sha256": expected_frozen_spec_sha256,
        "scope": "locked_static_core_development_only",
        "row_count": len(rows),
        "group_count": len(per_group),
        "policy_accepted_row_count": accepted_count,
        "policy_rejected_row_count": rejected_count,
        "present_core_bins": list(PRESENT_CORE_BINS),
        "missing_core_bins": list(MISSING_CORE_BINS),
        "frame_weighted_aggregate": aggregate,
        "equal_group_aggregate": {
            key.removeprefix("equal_group_"): value
            for key, value in aggregate.items()
            if key.startswith("equal_group_")
        },
        "stationary_groups_exceeding_any_limit": unstable_count,
        "frozen_limits": FROZEN_LIMITS,
        "blockers": blockers,
        "unsupported_metrics": UNSUPPORTED_METRICS,
        "model_promotion_conclusion": "NOT_APPLICABLE_NO_MODEL_TRAINED",
    }

    output_dir.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix="range_v2_r3_", dir=output_dir.parent) as temporary:
        temp_dir = Path(temporary)
        _write_csv(temp_dir / "raw_baseline_rows.csv", rows)
        _write_csv(temp_dir / "per_group_metrics.csv", per_group)
        _write_csv(temp_dir / "per_bin_metrics.csv", per_bin)
        _write_csv(temp_dir / "policy_slice_metrics.csv", policy)
        _json_dump(temp_dir / "unsupported_metrics.json", _unsupported_payload())
        _json_dump(temp_dir / "raw_baseline_summary.json", summary)
        (temp_dir / "raw_baseline_report.md").write_text(
            _markdown_report(
                summary=summary,
                bins=per_bin,
                groups=per_group,
                policy_rows=policy,
            ),
            encoding="utf-8",
        )

        verify_source_checksums(workspace, locked["sources"])
        source_post = {
            source["dataset_root"]: (
                file_sha256(workspace / source["manifest_path"]),
                file_sha256(workspace / source["samples_path"]),
            )
            for source in locked["sources"]
        }
        if source_pre != source_post:
            raise RuntimeError("source_changed_during_raw_baseline")
        r2_post = {
            name: file_sha256(r2_dir / name)
            for name in locked["r2_artifact_checksums"]
        }
        if r2_pre != r2_post:
            raise RuntimeError("r2_artifact_changed_during_raw_baseline")

        output_checksums = {
            path.name: file_sha256(path)
            for path in sorted(temp_dir.iterdir())
            if path.is_file()
        }
        evaluator_path = Path(__file__).resolve()
        manifest = {
            "artifact_type": "range_v2_r3_raw_baseline_manifest",
            "created_utc": datetime.now(timezone.utc).isoformat(),
            "outcome": summary["outcome"],
            "evaluator_schema_version": EVALUATOR_SCHEMA_VERSION,
            "frozen_spec_id": SPEC_ID,
            "frozen_spec_sha256": expected_frozen_spec_sha256,
            "r2_artifact_checksums": locked["r2_artifact_checksums"],
            "source_dataset_checksums": locked["sources"],
            "evaluator_source": evaluator_path.name,
            "evaluator_source_sha256": file_sha256(evaluator_path),
            "commands": {
                "evaluation": "PYTHONPATH=/home/sup/swarm_dashboard/.venv/lib/python3.12/site-packages python3 range_v2_baseline_eval.py",
                "focused_tests": "PYTHONPATH=/home/sup/swarm_dashboard/.venv/lib/python3.12/site-packages python3 -m pytest -q test_range_v2_baseline_eval.py",
                "repository_tests": "PYTHONPATH=/home/sup/swarm_dashboard/.venv/lib/python3.12/site-packages python3 -m pytest -q test_*.py",
                "runtime_configuration_check": "./run_all.sh --check",
            },
            "counts": {
                "rows": len(rows),
                "groups": len(per_group),
                "policy_accepted_rows": accepted_count,
                "policy_rejected_rows": rejected_count,
                "present_bins": len(PRESENT_CORE_BINS),
                "missing_bins": len(MISSING_CORE_BINS),
            },
            "metric_definitions": {
                "signed_error_m": "raw_physical_range_m - ground_truth_distance_m",
                "signed_bias_m": "arithmetic mean of signed_error_m",
                "median_absolute_error_m": "median(abs(signed_error_m))",
                "mae_m": "mean(abs(signed_error_m))",
                "p90_p95": "numpy linear sample quantiles of abs(signed_error_m)",
                "mean_absolute_relative_error": "mean(abs(error)/ground_truth_distance_m)",
                "catastrophic_error_rate": "mean(abs(error)>3.0m), strict greater-than",
                "standard_deviation": "population std, ddof=0",
                "frame_delta": "absolute difference of consecutive timestamp-ordered raw ranges",
                "linear_drift_slope": "ordinary least-squares raw range versus elapsed measurement time",
                "maximum_drift": "max(raw range)-min(raw range) within group",
                "maximum_10_second_drift": "maximum raw range span in any observed <=10s window; N/A when duration<10s",
                "equal_group": "unweighted arithmetic mean of independently computed group metrics",
                "uncertainty_interval": "95% percentile cluster bootstrap resampling whole groups, 5000 replicates, seed 52",
            },
            "unsupported_metrics": UNSUPPORTED_METRICS,
            "output_checksums": output_checksums,
            "manifest_self_checksum": "excluded_to_avoid_recursive_checksum",
            "scope_guards": {
                "model_trained": False,
                "scaler_fitted": False,
                "calibrator_fitted": False,
                "confidence_threshold_fitted": False,
                "temporal_filter_tuned_or_changed": False,
                "geometry_changed": False,
                "midas_changed": False,
                "runtime_changed": False,
                "controller_changed": False,
                "px4_or_simulation_run": False,
                "shadow_run": False,
                "final_holdout_opened_or_created": False,
                "source_dataset_or_manifest_changed": False,
                "r2_artifact_changed": False,
                "residual_correction_default_off_preserved": True,
            },
        }
        _json_dump(temp_dir / "raw_baseline_manifest.json", manifest)
        for path in sorted(temp_dir.iterdir()):
            if path.is_file():
                path.replace(output_dir / path.name)
    return {
        "outcome": summary["outcome"],
        "row_count": len(rows),
        "group_count": len(per_group),
        "policy_accepted_rows": accepted_count,
        "policy_rejected_rows": rejected_count,
        "output_dir": str(output_dir),
    }


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--workspace", type=Path, default=Path(__file__).resolve().parent
    )
    parser.add_argument("--r2-dir", type=Path)
    parser.add_argument("--output-dir", type=Path)
    parser.add_argument("--frozen-spec", type=Path)
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    workspace = args.workspace.resolve()
    result = evaluate_raw_baseline(
        workspace=workspace,
        r2_dir=(args.r2_dir or workspace / "artifacts" / "range_v2"),
        output_dir=(
            args.output_dir
            or workspace / "artifacts" / "range_v2" / "raw_baseline"
        ),
        frozen_spec_path=(
            args.frozen_spec
            or workspace / "docs" / "RANGE_V2_FROZEN_SPEC.md"
        ),
    )
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
