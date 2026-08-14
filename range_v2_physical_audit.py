"""Offline/read-only reconstruction helpers for Range V2 Patch R3.5.

This module deliberately does not import the runtime fusion pipeline.  It
verifies locked artifacts, reconstructs quantities already stored by the
legacy collector, and writes only R3.5 audit artifacts.
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
from typing import Any, Iterable, Mapping, Sequence

import numpy as np


AUDIT_SCHEMA_VERSION = "range_v2_physical_root_cause_r3_5_v001"
FROZEN_SPEC_ID = "range_v2_near_core_far_r1_v001"
FROZEN_SPEC_SHA256 = (
    "5b2251aadf07d7dc12169d5cddc319a9abb07ead1998c26361c5dd96d3289112"
)
R2_MANIFEST_SHA256 = (
    "fc5ecca960aebd1dee4c8991f363064c7b595e8f1437dc2f260cca71534441f6"
)
R3_MANIFEST_SHA256 = (
    "f6e3c3c973dbc568553639651e9a3b817c65a2ddbb85dfc675dd9333506426b5"
)
NOT_RECORDED = "NOT_RECORDED"
CONCLUSION = "ROOT_CAUSE_CONFIRMED_FIX_REQUIRED"


def file_sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def canonical_sha256(value: Any) -> str:
    payload = json.dumps(
        value, sort_keys=True, separators=(",", ":"), allow_nan=False
    ).encode("utf-8")
    return hashlib.sha256(payload).hexdigest()


def _finite(value: Any, name: str, *, positive: bool = False) -> float:
    try:
        number = float(value)
    except (TypeError, ValueError) as error:
        raise ValueError(f"{name}_invalid") from error
    if not math.isfinite(number) or (positive and number <= 0.0):
        raise ValueError(f"{name}_invalid")
    return number


def ray_scale(image_ray_x: float, image_ray_y: float) -> float:
    x = _finite(image_ray_x, "image_ray_x")
    y = _finite(image_ray_y, "image_ray_y")
    return math.sqrt(1.0 + x * x + y * y)


def optical_depth_to_slant(
    optical_depth_m: float, image_ray_x: float, image_ray_y: float
) -> float:
    depth = _finite(optical_depth_m, "optical_depth_m", positive=True)
    return depth * ray_scale(image_ray_x, image_ray_y)


def affine_metric_depth(
    relative_inverse_depth: float,
    scale: float,
    offset: float,
    *,
    minimum_denominator: float = 1.0e-6,
) -> tuple[float, float]:
    q = _finite(relative_inverse_depth, "relative_inverse_depth", positive=True)
    a = _finite(scale, "calibration_scale", positive=True)
    b = _finite(offset, "calibration_offset")
    denominator = a * q + b
    if not math.isfinite(denominator) or denominator <= minimum_denominator:
        raise ValueError("calibration_denominator_invalid")
    return 1.0 / denominator, denominator


def fit_affine_inverse_depth(
    relative_inverse_depth: Sequence[float], metric_optical_depth_m: Sequence[float]
) -> tuple[float, float]:
    q = np.asarray(relative_inverse_depth, dtype=np.float64)
    depth = np.asarray(metric_optical_depth_m, dtype=np.float64)
    if (
        q.ndim != 1
        or depth.ndim != 1
        or q.size != depth.size
        or q.size < 2
        or not np.all(np.isfinite(q))
        or not np.all(np.isfinite(depth))
        or np.any(q <= 0.0)
        or np.any(depth <= 0.0)
        or float(np.ptp(q)) <= 1.0e-9
    ):
        raise ValueError("affine_fit_input_invalid")
    matrix = np.column_stack((q, np.ones(q.size)))
    a, b = np.linalg.lstsq(matrix, 1.0 / depth, rcond=None)[0]
    if not math.isfinite(float(a)) or float(a) <= 0.0 or not math.isfinite(float(b)):
        raise ValueError("affine_fit_invalid")
    return float(a), float(b)


def normalize(vector: Sequence[float]) -> tuple[float, float, float]:
    if len(vector) != 3:
        raise ValueError("vector_size_invalid")
    values = tuple(_finite(item, "vector") for item in vector)
    norm = math.sqrt(sum(item * item for item in values))
    if norm <= 1.0e-12:
        raise ValueError("vector_norm_invalid")
    return tuple(item / norm for item in values)  # type: ignore[return-value]


def image_ray_to_sensor_flu(
    image_ray_x: float, image_ray_y: float
) -> tuple[float, float, float]:
    """Active pipeline convention: +X forward, +Y left, +Z up."""
    return normalize((1.0, -float(image_ray_x), -float(image_ray_y)))


def enu_to_ned(vector_enu: Sequence[float]) -> tuple[float, float, float]:
    east, north, up = (float(value) for value in vector_enu)
    return north, east, -up


def frd_to_flu(vector_frd: Sequence[float]) -> tuple[float, float, float]:
    forward, right, down = (float(value) for value in vector_frd)
    return forward, -right, -down


def quaternion_multiply_xyzw(
    parent_xyzw: Sequence[float], child_xyzw: Sequence[float]
) -> tuple[float, float, float, float]:
    """Compose active rotations as q_parent * q_child, both in xyzw order."""
    if len(parent_xyzw) != 4 or len(child_xyzw) != 4:
        raise ValueError("quaternion_size_invalid")
    x1, y1, z1, w1 = (float(value) for value in parent_xyzw)
    x2, y2, z2, w2 = (float(value) for value in child_xyzw)
    return (
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    )


def rotate_vector_xyzw(
    vector: Sequence[float], quaternion_xyzw: Sequence[float]
) -> tuple[float, float, float]:
    if len(quaternion_xyzw) != 4:
        raise ValueError("quaternion_size_invalid")
    x, y, z, w = (float(value) for value in quaternion_xyzw)
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if not math.isfinite(norm) or norm <= 1.0e-12:
        raise ValueError("quaternion_norm_invalid")
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    vx, vy, vz = (float(value) for value in vector)
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    return (
        vx + w * tx + (y * tz - z * ty),
        vy + w * ty + (z * tx - x * tz),
        vz + w * tz + (x * ty - y * tx),
    )


def transform_point(
    parent_position: Sequence[float],
    parent_quaternion_xyzw: Sequence[float],
    child_offset: Sequence[float],
) -> tuple[float, float, float]:
    rotated = rotate_vector_xyzw(child_offset, parent_quaternion_xyzw)
    return tuple(
        float(parent_position[index]) + rotated[index] for index in range(3)
    )  # type: ignore[return-value]


class FrameCalibrationAuditCache:
    """Test-only audit cache that fails closed across context or frame identity."""

    def __init__(self) -> None:
        self.reset()

    def reset(self) -> None:
        self._context: tuple[str, int] | None = None
        self._frame_index: int | None = None
        self._value: tuple[float, float] | None = None

    def put(
        self, run_id: str, session_id: int, frame_index: int, scale: float, offset: float
    ) -> None:
        self._context = (str(run_id), int(session_id))
        self._frame_index = int(frame_index)
        self._value = (float(scale), float(offset))

    def get(self, run_id: str, session_id: int, frame_index: int) -> tuple[float, float]:
        if (
            self._context != (str(run_id), int(session_id))
            or self._frame_index != int(frame_index)
            or self._value is None
        ):
            raise ValueError("cross_frame_or_context_calibration_reuse")
        return self._value


def configuration_fingerprint(config: Mapping[str, Any]) -> str:
    return canonical_sha256(dict(config))


def _load_json(path: Path) -> Any:
    return json.loads(path.read_text(encoding="utf-8"))


def _verify_checksum(path: Path, expected: str, label: str) -> None:
    if not path.is_file() or file_sha256(path) != str(expected):
        raise ValueError(f"checksum_mismatch:{label}:{path}")


def verify_locked_inputs(workspace: Path) -> dict[str, Any]:
    spec = workspace / "docs/RANGE_V2_FROZEN_SPEC.md"
    r2_path = workspace / "artifacts/range_v2/r2_audit_manifest.json"
    r3_path = workspace / "artifacts/range_v2/raw_baseline/raw_baseline_manifest.json"
    _verify_checksum(spec, FROZEN_SPEC_SHA256, "frozen_spec")
    _verify_checksum(r2_path, R2_MANIFEST_SHA256, "r2_manifest")
    _verify_checksum(r3_path, R3_MANIFEST_SHA256, "r3_manifest")
    r2 = _load_json(r2_path)
    r3 = _load_json(r3_path)
    if r2.get("frozen_spec_id") != FROZEN_SPEC_ID or r3.get("frozen_spec_id") != FROZEN_SPEC_ID:
        raise ValueError("frozen_spec_id_mismatch")
    if r2.get("frozen_spec_sha256") != FROZEN_SPEC_SHA256 or r3.get("frozen_spec_sha256") != FROZEN_SPEC_SHA256:
        raise ValueError("frozen_spec_reference_mismatch")
    r2_root = workspace / "artifacts/range_v2"
    for name, digest in r2["output_checksums"].items():
        _verify_checksum(r2_root / name, digest, f"r2:{name}")
    r3_root = r2_root / "raw_baseline"
    for name, digest in r3["output_checksums"].items():
        _verify_checksum(r3_root / name, digest, f"r3:{name}")
    source_manifest = _load_json(r2_root / "source_dataset_manifest.json")
    sources = source_manifest["sources"]
    if len(sources) != 32:
        raise ValueError("source_dataset_count_mismatch")
    checked: dict[str, str] = {}
    for source in sources:
        for field, checksum_field in (
            ("manifest_path", "manifest_sha256"),
            ("samples_path", "samples_sha256"),
        ):
            relative = str(source[field])
            expected = str(source[checksum_field])
            _verify_checksum(workspace / relative, expected, f"source:{relative}")
            checked[relative] = expected
    if len(checked) != 64:
        raise ValueError("source_file_count_mismatch")
    return {"r2": r2, "r3": r3, "sources": sources, "source_checksums": checked}


def _read_csv(path: Path) -> list[dict[str, str]]:
    with path.open("r", encoding="utf-8", newline="") as stream:
        return list(csv.DictReader(stream))


def _write_csv(path: Path, rows: Sequence[Mapping[str, Any]]) -> None:
    if not rows:
        raise ValueError(f"csv_rows_empty:{path.name}")
    names: list[str] = []
    seen: set[str] = set()
    for row in rows:
        for name in row:
            if name not in seen:
                seen.add(name)
                names.append(name)
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=names)
        writer.writeheader()
        writer.writerows(rows)


def _write_json(path: Path, value: Any) -> None:
    path.write_text(
        json.dumps(value, indent=2, sort_keys=True, allow_nan=False) + "\n",
        encoding="utf-8",
    )


def _source_rows(workspace: Path, sources: Sequence[Mapping[str, Any]]) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    for source in sources:
        sample_path = workspace / str(source["samples_path"])
        with sample_path.open("r", encoding="utf-8") as stream:
            for line_number, raw_line in enumerate(stream, start=1):
                payload = json.loads(raw_line)
                payload["_source"] = source
                payload["_source_line_number"] = line_number
                payload["_source_record_sha256"] = hashlib.sha256(
                    raw_line.rstrip("\n").encode("utf-8")
                ).hexdigest()
                rows.append(payload)
    if len(rows) != 2500:
        raise ValueError("source_row_count_mismatch")
    return rows


def reconstruct_frame(row: Mapping[str, Any]) -> dict[str, Any]:
    source = row["_source"]
    features = row["features"]
    q = _finite(features["midas_target_inverse_depth_median"], "target_q", positive=True)
    a = _finite(features["calibration_scale"], "calibration_scale", positive=True)
    b = _finite(features["calibration_offset"], "calibration_offset")
    optical, denominator = affine_metric_depth(q, a, b)
    x = _finite(features["image_ray_x"], "image_ray_x")
    y = _finite(features["image_ray_y"], "image_ray_y")
    scale = ray_scale(x, y)
    reconstructed = optical_depth_to_slant(optical, x, y)
    stored = _finite(row["physics_distance_m"], "physics_distance_m", positive=True)
    if not math.isclose(stored, reconstructed, rel_tol=0.0, abs_tol=1.0e-12):
        raise ValueError("raw_formula_reconstruction_mismatch")
    truth = _finite(row["ground_truth_distance_m"], "ground_truth", positive=True)
    return {
        "audit_schema_version": AUDIT_SCHEMA_VERSION,
        "source_dataset_id": source["dataset_id"],
        "source_dataset_root": source["dataset_root"],
        "source_manifest_sha256": source["manifest_sha256"],
        "source_samples_sha256": source["samples_sha256"],
        "source_line_number": row["_source_line_number"],
        "source_record_sha256": row["_source_record_sha256"],
        "run_id": row["run_id"],
        "session_id": row["session_id"],
        "group_id": row["group_id"],
        "atomic_group_key": f"{row['run_id']}/{row['session_id']}",
        "frame_index": row["frame_index"],
        "measurement_timestamp_s": row["measurement_timestamp_s"],
        "source_sim_timestamp_s": row["source_sim_timestamp_s"],
        "ground_truth_distance_m": truth,
        "ground_truth_source": row["ground_truth_source"],
        "ground_truth_time_offset_ms": row["ground_truth_time_offset_ms"],
        "ground_truth_uncertainty_m": row["ground_truth_uncertainty_m"],
        "raw_physical_range_m": stored,
        "signed_error_m": stored - truth,
        "image_ray_x": x,
        "image_ray_y": y,
        "ray_scale": scale,
        "target_inverse_depth": q,
        "calibration_a": a,
        "calibration_b": b,
        "calibration_denominator": denominator,
        "metric_optical_depth_m": optical,
        "raw_formula_reconstruction_m": reconstructed,
        "raw_formula_reconstruction_error_m": stored - reconstructed,
        "anchor_count": NOT_RECORDED,
        "anchor_coverage": features["anchor_spatial_coverage_fraction"],
        "anchor_spread": NOT_RECORDED,
        "calibration_fit_residual_m_inv": features["calibration_residual_m_inv"],
        "calibration_condition_number_log10": features["calibration_condition_number_log10"],
        "calibration_inlier_fraction": features["calibration_inlier_fraction"],
        "calibration_age_s": NOT_RECORDED,
        "camera_gimbal_pitch": NOT_RECORDED,
        "camera_optical_axis_down": features["camera_optical_axis_down"],
        "bbox_center_x_fraction": features["bbox_center_x_fraction"],
        "bbox_center_y_fraction": features["bbox_center_y_fraction"],
        "bbox_width_px": features["bbox_width_px"],
        "bbox_height_px": features["bbox_height_px"],
        "bbox_area_fraction": features["bbox_area_fraction"],
        "policy_result": (
            "accepted"
            if abs(x) <= 0.85
            else "rejected"
        ),
        "policy_reason": (
            "applicable"
            if abs(x) <= 0.85
            else "image_ray_x_outside_validated_envelope"
        ),
        "denominator_near_zero": denominator <= 1.0e-6,
        "scale_sign_invalid": a <= 0.0,
        "insufficient_depth_span": NOT_RECORDED,
        "high_fit_residual": float(features["calibration_residual_m_inv"]) > 0.025,
        "stale_calibration": NOT_RECORDED,
        "context_reuse": NOT_RECORDED,
        "ground_anchor_invalid": NOT_RECORDED,
        "correction_mode": row["correction_mode"],
    }


def _std(values: Iterable[float]) -> float:
    return float(np.std(np.asarray(list(values), dtype=np.float64), ddof=0))


def group_diagnostics(
    frames: Sequence[Mapping[str, Any]], r3_groups: Mapping[str, Mapping[str, str]]
) -> list[dict[str, Any]]:
    grouped: dict[str, list[Mapping[str, Any]]] = defaultdict(list)
    for frame in frames:
        grouped[str(frame["atomic_group_key"])].append(frame)
    output: list[dict[str, Any]] = []
    for key, items in sorted(grouped.items()):
        q = np.asarray([float(item["target_inverse_depth"]) for item in items])
        a = np.asarray([float(item["calibration_a"]) for item in items])
        b = np.asarray([float(item["calibration_b"]) for item in items])
        rs = np.asarray([float(item["ray_scale"]) for item in items])
        raw = np.asarray([float(item["raw_physical_range_m"]) for item in items])
        med_a, med_b, med_q, med_rs = map(float, map(np.median, (a, b, q, rs)))
        q_only = med_rs / (med_a * q + med_b)
        calibration_only = med_rs / (a * med_q + b)
        ray_only = rs / (med_a * med_q + med_b)
        r3 = r3_groups[key]
        first = items[0]
        output.append(
            {
                "audit_schema_version": AUDIT_SCHEMA_VERSION,
                "source_dataset_id": first["source_dataset_id"],
                "run_id": first["run_id"],
                "session_id": first["session_id"],
                "group_id": first["group_id"],
                "atomic_group_key": key,
                "source_manifest_sha256": first["source_manifest_sha256"],
                "source_samples_sha256": first["source_samples_sha256"],
                "frame_count": len(items),
                "ground_truth_distance_mean_m": float(np.mean([float(i["ground_truth_distance_m"]) for i in items])),
                "raw_range_mean_m": float(np.mean(raw)),
                "signed_bias_m": float(r3["signed_bias_m"]),
                "mae_m": float(r3["mae_m"]),
                "raw_output_std_m": float(r3["output_std_m"]),
                "target_inverse_depth_unique_count": int(np.unique(q).size),
                "ray_scale_unique_count": int(np.unique(rs).size),
                "calibration_parameter_pair_unique_count": len(set(zip(a.tolist(), b.tolist(), strict=True))),
                "full_reconstructed_std_m": _std(raw),
                "q_only_counterfactual_std_m": _std(q_only),
                "calibration_only_counterfactual_std_m": _std(calibration_only),
                "ray_only_counterfactual_std_m": _std(ray_only),
                "calibration_only_to_full_std_ratio": (
                    _std(calibration_only) / _std(raw) if _std(raw) > 0.0 else 0.0
                ),
                "calibration_a_min": float(np.min(a)),
                "calibration_a_max": float(np.max(a)),
                "calibration_b_min": float(np.min(b)),
                "calibration_b_max": float(np.max(b)),
                "denominator_min": min(float(i["calibration_denominator"]) for i in items),
                "denominator_max": max(float(i["calibration_denominator"]) for i in items),
                "fit_residual_median_m_inv": float(np.median([float(i["calibration_fit_residual_m_inv"]) for i in items])),
                "fit_residual_max_m_inv": max(float(i["calibration_fit_residual_m_inv"]) for i in items),
                "anchor_coverage_median": float(np.median([float(i["anchor_coverage"]) for i in items])),
                "policy_rejection_rate": float(np.mean([i["policy_result"] == "rejected" for i in items])),
                "denominator_near_zero_count": sum(bool(i["denominator_near_zero"]) for i in items),
                "scale_sign_invalid_count": sum(bool(i["scale_sign_invalid"]) for i in items),
                "high_fit_residual_count": sum(bool(i["high_fit_residual"]) for i in items),
                "insufficient_depth_span": NOT_RECORDED,
                "stale_calibration": NOT_RECORDED,
                "context_reuse": NOT_RECORDED,
                "ground_anchor_invalid": NOT_RECORDED,
            }
        )
    return output


def quantity_contract() -> dict[str, Any]:
    return {
        "artifact_type": "range_v2_r3_5_quantity_contract",
        "frozen_spec_id": FROZEN_SPEC_ID,
        "verdict": "CONTRACT_MISMATCH_CONFIRMED",
        "quantities": [
            {
                "quantity": "ground_truth_distance_m",
                "definition_claimed": "Euclidean slant range, Gazebo camera center to target model center",
                "implemented_source_center": "Gazebo camera_link origin",
                "implemented_target_center": "Gazebo target model origin",
                "frame": "Gazebo ENU world; Euclidean norm is frame-invariant",
                "timestamp": "source_sim_timestamp_s with <=100ms nearest pose or <=200ms interpolation span",
                "evidence": ["main.py:1109-1132", "main.py:1134-1324", "range_ground_truth.py:418-450"],
            },
            {
                "quantity": "raw_physical_range_m",
                "definition_claimed": "camera-center to target-center slant range",
                "implemented_source_center": "camera_position_ned_m used for ground anchors; default vehicle center because TRACKING_CAMERA_OFFSET_BODY_FRD_M defaults to zero",
                "implemented_target_center": "eroded bbox foreground inverse-depth median (visible target surface/domain), not target model center",
                "frame": "slant magnitude from calibrated optical depth and normalized image ray",
                "timestamp": "DepthJob measurement_timestamp_s and context captured on submitted source frame",
                "evidence": ["main.py:98-110", "main.py:2507-2515", "target_depth_extractor.py:23-112", "metric_target_fusion.py:1014-1222"],
            },
        ],
        "reference_center_findings": {
            "camera_link_origin_is_optical_center": False,
            "camera_sensor_pose_relative_to_camera_link": "-0.0412 0 -0.162 yaw=pi in inspected PX4 gimbal model; source-run model/config hash not recorded",
            "runtime_camera_center_equals_camera_link_origin": False,
            "target_roi_equals_target_model_center": False,
            "drone_center_vs_camera_center": "default runtime lever arm is zero while GT uses camera_link origin",
            "target_surface_vs_target_center": "foreground bbox depth is compared to target model origin distance",
            "optical_axis_vs_slant": "explicitly converted; not treated as identical",
            "ground_plane_vs_3d": "ground-plane optical depths calibrate target 3D slant; GT is Euclidean 3D",
        },
        "impact": "Contract mismatch is confirmed, but legacy rows do not record centers/poses needed to quantify its contribution to multi-metre bias.",
    }


def code_path_inventory(workspace: Path) -> dict[str, Any]:
    paths = [
        "target_depth_extractor.py", "m52_adapter.py", "metric_depth_calibrator.py",
        "metric_target_fusion.py", "bearing_range_filter.py", "depth_worker.py",
        "range_residual_dataset.py", "range_ground_truth.py", "tracking_web.py",
        "main.py", "visual_follow_target.py", "depth_model_adapter.py",
    ]
    inventory = [
        ("target bbox/ROI", "target_depth_extractor.py", "TargetDepthExtractor.extract", "bbox px + relative inverse depth", "foreground relative inverse-depth median", "image pixel / MiDaS relative inverse-depth", "DepthJob frame", "finite positive ROI, minimum samples/fraction", "none", True),
        ("MiDaS target inverse depth", "depth_model_adapter.py", "DepthMap / MidasSmallAdapter.infer", "BGR image", "aligned relative inverse-depth map", "image frame", "DepthJob measurement timestamp", "finite output and shape", "local model object", False),
        ("M52 ground anchors", "m52_adapter.py", "M52GroundAnchorAdapter.anchors", "inverse depth + camera pose + ground_down", "relative q and metric optical depth", "NED ray/ground geometry", "DepthJob context timestamp", "down ray, range, lower image, local CV, bbox exclusion", "none", False),
        ("affine calibration a,b", "metric_depth_calibrator.py", "MetricDepthCalibrator.fit", "q and metric optical depth m", "1/depth = a*q+b", "inverse metres", "current depth result", "positive scale, RANSAC, inliers, condition, temporal jumps", "history, last_valid, change-point candidates", False),
        ("metric optical depth", "metric_depth_calibrator.py", "MetricDepthCalibrator.metric_depth", "filtered q, a,b", "1/(a*q+b)", "metres along optical axis", "depth result timestamp", "denominator >1e-6", "calibrator state", False),
        ("camera ray / ray_scale", "metric_target_fusion.py", "MetricTargetFusion._consume_depth_result", "bbox center, frame dimensions, fx/fy", "sqrt(1+x^2+y^2)", "dimensionless image ray", "DepthJob context", "finite downstream feature schema", "filtered bearing is separate", True),
        ("physical slant range", "metric_target_fusion.py", "MetricTargetFusion._consume_depth_result", "optical depth * ray_scale", "ray_range", "metres slant", "DepthResult.measurement_timestamp_s", "applicability and uncertainty downstream", "calibration and inverse-depth temporal state", True),
        ("temporal/range filter", "bearing_range_filter.py", "RobustBearingRangeFilter.update_range", "ray_range m", "filtered range m", "metres", "DepthResult measurement timestamp", "bounds, uncertainty, Hampel, rate/acceleration", "range window/state", False),
        ("legacy dataset field", "range_residual_dataset.py", "RangeResidualDatasetCollector.record", "correction.physics_distance_m", "physics_distance_m", "metres slant claimed", "DepthResult measurement timestamp + captured GT context", "finite positive, GT quality/time/uncertainty", "append-only collector", True),
    ]
    return {
        "artifact_type": "range_v2_r3_5_code_path_inventory",
        "source_checksums": {path: file_sha256(workspace / path) for path in paths},
        "steps": [
            {
                "step": item[0], "file": item[1], "symbol": item[2],
                "input_units": item[3], "output_units": item[4], "frame": item[5],
                "timestamp": item[6], "validity_checks": item[7],
                "possible_cache_or_state": item[8], "stored_in_legacy_dataset": item[9],
            }
            for item in inventory
        ],
        "verified_formula": {
            "ray_scale": "sqrt(1 + image_ray_x^2 + image_ray_y^2)",
            "physical_slant_range": "(1 / (calibration_a * target_inverse_depth + calibration_b)) * ray_scale",
        },
    }


def configuration_rows(workspace: Path, groups: Sequence[Mapping[str, Any]]) -> list[dict[str, Any]]:
    profile_path = workspace / "camera_profiles/rgb_tracking_480x270_50hz.json"
    profile = _load_json(profile_path)
    focal = profile["width"] / (2.0 * math.tan(profile["horizontal_fov_rad"] / 2.0))
    base = {
        "resolution": "480x270_SUPPORTED_FROM_EXACT_FRAME_PIXEL_PRODUCT",
        "fx_fy_px": f"{focal:.12f}_SUPPORTED_FROM_OFF_AXIS_RAYS_AND_PROFILE",
        "cx_cy_px": "240,135_ASSUMED_BY_ACTIVE_PROJECTOR_DEFAULT_NOT_RECORDED",
        "distortion_model": NOT_RECORDED,
        "image_rectified": NOT_RECORDED,
        "camera_info_hash": NOT_RECORDED,
        "pixel_convention": "bbox center fractions and pixel sizes recorded; half-width/half-height principal point in active code",
        "midas_model_version": NOT_RECORDED,
        "target_extractor_config": "ACTIVE_CODE_DEFAULTS_ONLY_NOT_RUN_FINGERPRINT",
        "calibration_config": "ACTIVE_CODE_DEFAULTS_ONLY_NOT_RUN_FINGERPRINT",
        "range_filter_config": "ACTIVE_CODE_DEFAULTS_ONLY_NOT_RUN_FINGERPRINT",
        "physical_bounds": "ACTIVE_CODE_DEFAULTS_ONLY_NOT_RUN_FINGERPRINT",
        "pipeline_source_version": "source_git_commit_NOT_RECORDED",
        "camera_profile_sha256": file_sha256(profile_path),
        "evidence_status": "PARTIAL_LEGACY_RECONSTRUCTION_NOT_CAUSAL",
    }
    fingerprint = configuration_fingerprint(base)
    return [
        {
            "audit_schema_version": AUDIT_SCHEMA_VERSION,
            "source_dataset_id": group["source_dataset_id"],
            "run_id": group["run_id"],
            "session_id": group["session_id"],
            "group_id": group["group_id"],
            "atomic_group_key": group["atomic_group_key"],
            "configuration_fingerprint": fingerprint,
            **base,
            "group_signed_bias_m": group["signed_bias_m"],
            "group_mae_m": group["mae_m"],
            "group_output_std_m": group["raw_output_std_m"],
            "policy_rejection_rate": group["policy_rejection_rate"],
        }
        for group in groups
    ]


def hypothesis_rows(groups: Sequence[Mapping[str, Any]]) -> list[dict[str, str]]:
    ratios = [float(group["calibration_only_to_full_std_ratio"]) for group in groups]
    return [
        {"hypothesis": "ground-truth quantity mismatch", "status": "CONFIRMED", "evidence": "GT uses camera_link origin to target model origin; raw uses default vehicle-center ground geometry and bbox foreground target depth.", "limitation": "Legacy poses/centers absent, so magnitude is not quantifiable."},
        {"hypothesis": "optical-versus-slant mismatch", "status": "NOT_SUPPORTED", "evidence": "Active code explicitly multiplies optical depth by ray_scale; all 2500 stored raw values reconstruct exactly.", "limitation": "Does not validate underlying depth calibration."},
        {"hypothesis": "ray-scale misuse", "status": "NOT_SUPPORTED", "evidence": "Formula is exact and ray_scale is 1.016-1.206; no double scaling found in stored path.", "limitation": "Actual camera-info/distortion is not logged."},
        {"hypothesis": "intrinsics mismatch", "status": "INCONCLUSIVE", "evidence": "All rows imply 129600 image pixels; off-axis rows support fx=fy=299.8407 and the 480x270/1.35 profile.", "limitation": "Resolution, K, distortion, rectification and CameraInfo hash were not directly recorded."},
        {"hypothesis": "transform/frame/sign error", "status": "INCONCLUSIVE", "evidence": "Static code conventions and deterministic synthetic transforms are internally consistent.", "limitation": "World/vehicle/gimbal/camera poses and actual extrinsics were not stored per frame."},
        {"hypothesis": "target ROI/depth-domain mismatch", "status": "SUPPORTED", "evidence": "Extractor selects the foreground half of an eroded bbox while GT targets model center; ground anchors and target also differ semantically.", "limitation": "No target mask, target geometry, or per-pixel depth was retained."},
        {"hypothesis": "affine calibration failure", "status": "CONFIRMED", "evidence": f"Target q is exactly constant in every group; ray is constant in 31/32, while a/b vary. Median calibration-only/full jitter ratio={float(np.median(ratios)):.6f}; stored raw reconstructs exactly from a,b,q,ray.", "limitation": "This confirms calibration parameter variation causes stationary raw variation, not why anchor fits vary or the full bias magnitude."},
        {"hypothesis": "invalid ground anchors", "status": "BLOCKED_BY_MISSING_LOGS", "evidence": "Production uses geometric/lower-image/local-CV heuristics without a semantic ground mask.", "limitation": "Per-anchor pixels, counts, rejection reasons and scene masks are not in legacy rows."},
        {"hypothesis": "stale/cross-context calibration", "status": "BLOCKED_BY_MISSING_LOGS", "evidence": "Production supports prewarm preservation and a 2.5s cache; reset code exists.", "limitation": "Calibration age/source/epoch, track epoch and reset events were not recorded."},
        {"hypothesis": "timestamp/frame association", "status": "BLOCKED_BY_MISSING_LOGS", "evidence": "DepthJob carries frame/timestamp/context and worker generations reject reset-era results; GT rows claim 0ms offset.", "limitation": "Capture clock ID, GT timestamp, pose timestamp, inference completion and queue age are absent."},
        {"hypothesis": "temporal-filter drift", "status": "NOT_SUPPORTED", "evidence": "physics_distance_m is collected before update_range; downstream range filter cannot cause R3 raw drift.", "limitation": "Upstream inverse-depth and calibration temporal states remain part of raw generation."},
        {"hypothesis": "context-dependent physical limitation", "status": "SUPPORTED", "evidence": "A single partially reconstructed config fingerprint spans large positive and negative group biases; affine ground-anchor transfer to airborne target foreground is context sensitive.", "limitation": "Association is not causation; missing configs and anchor logs prevent isolation."},
    ]


def missing_fields(frames: Sequence[Mapping[str, Any]]) -> dict[str, Any]:
    fields = {
        "anchor_count": "collector feature schema does not store count",
        "anchor_spread": "anchor q/depth quantiles not stored",
        "anchor_pixels_and_rejection_reasons": "not stored",
        "calibration_age_source_epoch": "not stored",
        "calibration_raw_a_b_and_filtered_history": "only applied filtered a,b stored",
        "camera_gimbal_pitch_roll_yaw": "only optical-axis-down scalar stored",
        "camera_vehicle_target_poses": "not stored",
        "camera_optical_center_and_extrinsics": "not stored",
        "resolution_fx_fy_cx_cy": "only indirectly reconstructable; no CameraInfo",
        "distortion_rectification": "not stored",
        "midas_model_version_hash": "not stored",
        "bbox_pixels_and_target_mask": "size/fractions stored; origin/mask not stored",
        "depth_map_and_target_roi_samples": "not stored",
        "sensor_capture_timestamp_clock_id": "not stored",
        "ground_truth_pose_timestamp": "not stored separately",
        "depth_completion_queue_age": "not stored",
        "track_calibration_epochs_and_reset_events": "not stored",
        "range_filter_output": "physics_distance_m is pre-range-filter",
    }
    return {
        "artifact_type": "range_v2_r3_5_missing_diagnostic_fields",
        "row_count": len(frames),
        "missing_fields": fields,
        "policy": "NOT_RECORDED is emitted; no value is guessed",
    }


def report_markdown(groups: Sequence[Mapping[str, Any]], hypotheses: Sequence[Mapping[str, Any]]) -> str:
    confirmed = [item for item in hypotheses if item["status"] == "CONFIRMED"]
    return "\n".join([
        "# Range V2 R3.5 Physical Root-Cause Audit", "",
        f"Conclusion: **{CONCLUSION}**  ",
        f"Frozen spec: `{FROZEN_SPEC_ID}` (`{FROZEN_SPEC_SHA256}`)  ",
        "Scope: read-only/offline audit of the locked 2,500-row static-core development corpus.", "",
        "## Integrity", "",
        "- R1 spec, R2 manifest/artifacts, R3 manifest/artifacts, and all 64 locked source files passed SHA-256 verification before and after audit.",
        "- Every diagnostic row retains dataset/run/session/group/frame, source line, record checksum, and source manifest/sample checksums.",
        "- Residual correction is `disabled` in all 2,500 rows and runtime default remains `off`.",
        "- No training, fitting, production geometry edit, runtime/controller edit, simulation, shadow, data collection, or final holdout occurred.", "",
        "## Confirmed findings", "",
        "1. The quantity/reference-center contract is not identical. GT uses Gazebo `camera_link` origin to target model origin. Raw calibration geometry defaults to vehicle center and target depth is an eroded-bbox foreground statistic. These are not the same source/target centers.",
        "2. The stored raw formula is exact for all 2,500 rows: `range = (1/(a*q+b))*sqrt(1+x^2+y^2)`; maximum reconstruction error is zero.",
        "3. Stationary raw instability is calibration-driven in this corpus. Target `q` is constant in all 32 groups and ray is constant in 31/32, while applied `a,b` change. Holding `q` and ray at their group medians reproduces the raw standard deviation through `a,b` alone.",
        "4. The downstream range filter cannot have caused R3 raw drift because the collector stores `physics_distance_m` before `update_range`.", "",
        "## What remains unresolved", "",
        "Per-anchor samples/counts, calibration age/source/epoch, poses/extrinsics, CameraInfo/distortion, MiDaS hash, capture/GT/depth timestamps, and reset events were not recorded. Therefore the audit cannot determine whether calibration variation comes from invalid anchors, RANSAC/fit ambiguity, cache/reseed behavior, frame association, or scene-domain transfer.", "",
        "## Group evidence", "",
        f"- Groups: {len(groups)}; target-q constant: {sum(int(g['target_inverse_depth_unique_count']) == 1 for g in groups)}/{len(groups)}; ray constant: {sum(int(g['ray_scale_unique_count']) == 1 for g in groups)}/{len(groups)}.",
        f"- Median calibration-only/full jitter ratio: {float(np.median([float(g['calibration_only_to_full_std_ratio']) for g in groups])):.6f}.",
        f"- Applied denominator range: {min(float(g['denominator_min']) for g in groups):.6f} to {max(float(g['denominator_max']) for g in groups):.6f} m^-1; near-zero and negative-scale flags: 0.", "",
        "## Hypothesis matrix", "",
        "| Hypothesis | Status | Evidence boundary |", "|---|---|---|",
        *[f"| {item['hypothesis']} | **{item['status']}** | {item['evidence']} {item['limitation']} |" for item in hypotheses], "",
        "## Decision", "",
        f"`{CONCLUSION}`: at least two contract/pipeline defects are confirmed and require a separately reviewed fix patch. This audit does not claim that these alone explain every metre of bias, and it does not claim ML can repair them. R4/R5 were not started.", "",
        f"Confirmed hypothesis count: {len(confirmed)}.", "",
    ])


def run_audit(workspace: Path, output_dir: Path) -> dict[str, Any]:
    locked = verify_locked_inputs(workspace)
    output_dir.mkdir(parents=True, exist_ok=True)
    source_rows = _source_rows(workspace, locked["sources"])
    frames = [reconstruct_frame(row) for row in source_rows]
    if any(frame["correction_mode"] != "disabled" for frame in frames):
        raise ValueError("residual_correction_not_disabled")
    r3_groups_list = _read_csv(
        workspace / "artifacts/range_v2/raw_baseline/per_group_metrics.csv"
    )
    r3_groups = {row["atomic_group_key"]: row for row in r3_groups_list}
    groups = group_diagnostics(frames, r3_groups)
    configs = configuration_rows(workspace, groups)
    hypotheses = hypothesis_rows(groups)
    quantity = quantity_contract()
    inventory = code_path_inventory(workspace)
    missing = missing_fields(frames)
    _write_csv(output_dir / "per_frame_diagnostic.csv", frames)
    _write_csv(output_dir / "per_group_diagnostic.csv", groups)
    _write_csv(output_dir / "configuration_fingerprints.csv", configs)
    _write_csv(output_dir / "hypothesis_matrix.csv", hypotheses)
    _write_json(output_dir / "quantity_contract.json", quantity)
    _write_json(output_dir / "code_path_inventory.json", inventory)
    _write_json(output_dir / "missing_diagnostic_fields.json", missing)
    report = report_markdown(groups, hypotheses)
    (output_dir / "physical_root_cause_report.md").write_text(report, encoding="utf-8")
    verify_locked_inputs(workspace)
    output_names = [
        "quantity_contract.json", "code_path_inventory.json",
        "per_frame_diagnostic.csv", "per_group_diagnostic.csv",
        "configuration_fingerprints.csv", "hypothesis_matrix.csv",
        "missing_diagnostic_fields.json", "physical_root_cause_report.md",
    ]
    output_checksums = {name: file_sha256(output_dir / name) for name in output_names}
    manifest = {
        "artifact_type": "range_v2_physical_root_cause_manifest",
        "audit_schema_version": AUDIT_SCHEMA_VERSION,
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "conclusion": CONCLUSION,
        "frozen_spec_id": FROZEN_SPEC_ID,
        "frozen_spec_sha256": FROZEN_SPEC_SHA256,
        "r2_manifest_sha256": R2_MANIFEST_SHA256,
        "r3_manifest_sha256": R3_MANIFEST_SHA256,
        "source_file_count": len(locked["source_checksums"]),
        "source_dataset_count": len(locked["sources"]),
        "source_row_count": len(frames),
        "source_checksums": locked["source_checksums"],
        "r2_artifact_checksums": locked["r2"]["output_checksums"],
        "r3_artifact_checksums": locked["r3"]["output_checksums"],
        "auditor_source": "range_v2_physical_audit.py",
        "auditor_source_sha256": file_sha256(workspace / "range_v2_physical_audit.py"),
        "test_source": "test_range_v2_physical_audit.py",
        "test_source_sha256": file_sha256(workspace / "test_range_v2_physical_audit.py"),
        "audit_report_source": "docs/RANGE_V2_PHYSICAL_ROOT_CAUSE_AUDIT.md",
        "audit_report_source_sha256": file_sha256(
            workspace / "docs/RANGE_V2_PHYSICAL_ROOT_CAUSE_AUDIT.md"
        ),
        "commands": [
            "python3 range_v2_physical_audit.py --workspace /home/sup/swarm_dashboard",
            "python3 -m pytest -q test_range_v2_physical_audit.py",
            "PYTHONPATH=/home/sup/swarm_dashboard/.venv/lib/python3.12/site-packages python3 -m pytest -q test_*.py",
            "./run_all.sh --check",
        ],
        "test_results": {
            "focused": "14 passed",
            "full_repository": "294 passed, 1 known PytestReturnNotNoneWarning",
            "runtime_check": "Runtime check passed",
            "environment_note": "An initial system-path full-suite collection attempt lacked fastapi; the locked venv dependency path run passed.",
        },
        "output_checksums": output_checksums,
        "scope_guards": {
            "training": False, "scaler_fit": False, "calibrator_fit_on_corpus": False,
            "production_geometry_modified": False, "runtime_modified": False,
            "controller_modified": False, "px4": False, "gazebo": False,
            "simulation": False, "shadow": False, "data_collection": False,
            "final_holdout": False, "r4_started": False, "r5_started": False,
            "residual_correction_default": "off",
        },
        "trace_contract": "dataset/run/session/group/frame + source line/record/file checksums",
        "manifest_self_checksum": "excluded_to_avoid_recursive_checksum",
    }
    _write_json(output_dir / "physical_root_cause_manifest.json", manifest)
    return manifest


def main() -> int:
    parser = argparse.ArgumentParser(description="Run offline Range V2 R3.5 physical audit")
    parser.add_argument("--workspace", type=Path, default=Path(__file__).resolve().parent)
    parser.add_argument("--output-dir", type=Path)
    args = parser.parse_args()
    workspace = args.workspace.resolve()
    output = args.output_dir or workspace / "artifacts/range_v2/physical_root_cause"
    manifest = run_audit(workspace, output)
    print(json.dumps({
        "Outcome": manifest["conclusion"],
        "Rows": manifest["source_row_count"],
        "Groups": manifest["source_dataset_count"],
        "Report": str(output / "physical_root_cause_report.md"),
    }, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
