import copy

import pytest

from range_physical_diagnostics import DIAGNOSTICS_SCHEMA_VERSION, canonical_sha256
from range_v2_r3_6a_validation import (
    anchor_refit_replay,
    anchors_status,
    completeness_reasons,
    exact_live_reconstruction,
    group_median_oracle_parameters,
    stable_freeze_parameters,
    timestamp_order_status,
    verify_record_checksum,
)


def anchor(index: int) -> dict:
    return {
        "grid_index": index,
        "pixel_u": float(index % 12),
        "pixel_v": float(index // 12),
        "accepted": True,
        "reason": "accepted",
        "target_exclusion_result": "outside_excluded_bbox",
        "ray_ned_unit": [0.8, 0.0, 0.6],
        "ground_slant_range_m": 10.0 + index * 0.05,
        "metric_optical_depth_m": 8.0 + index * 0.04,
        "relative_inverse_depth": 0.2 + index * 0.01,
        "normalized_image_x": 0.0,
        "normalized_image_y": 0.0,
        "sensor_forward_component": 0.8,
        "local_sample_count": 9,
        "local_inverse_depth_mean": 0.2 + index * 0.01,
        "local_inverse_depth_std": 0.005,
        "local_inverse_depth_cv": 0.02,
    }


def row(frame: int = 1, *, scale: float = 0.1, offset: float = 0.02) -> dict:
    q = 0.5
    ray_scale = 1.1
    physics = ray_scale / (scale * q + offset)
    calibration = {
        "valid": True,
        "reason": "ok",
        "raw_scale": scale + 0.001,
        "raw_offset": offset + 0.001,
        "filtered_scale": scale,
        "filtered_offset": offset,
        "inlier_count": 90,
        "anchor_count": 96,
        "residual_m_inv": 0.005,
        "residual_std_m_inv": 0.005,
        "condition_number": 100.0,
        "parameter_uncertainty": 0.01,
        "stable_samples": 6,
        "stable": True,
        "measurement_accepted": True,
        "parameter_covariance_ab": [[0.0, 0.0], [0.0, 0.0]],
        "anchor_inverse_depth_quantiles": [0.2, 0.5, 1.0],
        "metric_depth_quantiles_m": [5.0, 8.0, 12.0],
        "change_point_reseeded": False,
    }
    result = {
        "diagnostics_schema_version": DIAGNOSTICS_SCHEMA_VERSION,
        "instrumentation_only": True,
        "run_id": "run-a",
        "target_id": "UAV-02",
        "group_id": "UAV-01.1",
        "session_id": 1,
        "frame_index": frame,
        "measurement_timestamp_s": 1.0 + frame,
        "source_sim_timestamp_s": 100.0 + frame,
        "track_epoch": 2,
        "calibration_epoch": 3,
        "stage": "raw_range_computed",
        "reason": "ok",
        "reference_centers": {
            "raw_geometry_center_ned_m": [0.0, 0.0, 0.0],
            "raw_geometry_center_source": "camera_optical_center",
            "ground_truth": {
                "camera_link_to_target_model_origin_m": 8.0,
                "drone_center_to_target_model_origin_m": 8.1,
            },
            "optical_center": {"status": "verified", "position_ned_m": [0, 0, 0]},
            "target_reference": {"raw": "surface", "ground_truth": "center"},
            "unavailable_fields": [],
        },
        "camera_info": {
            "width": 480,
            "height": 270,
            "fx": 300.0,
            "fy": 300.0,
            "cx": 240.0,
            "cy": 135.0,
            "distortion_model": "none",
            "distortion_coefficients": [],
            "rectified": True,
            "fingerprint_sha256": "a" * 64,
        },
        "extrinsics": {
            "fingerprint_sha256": "b" * 64,
            "camera_position_ned_m": [0, 0, 0],
            "camera_quaternion_xyzw": [0, 0, 0, 1],
        },
        "timestamps": {
            "measurement_timestamp_s": 1.0 + frame,
            "depth_submitted_timestamp_s": 1.01 + frame,
            "depth_inference_started_timestamp_s": 1.02 + frame,
            "depth_completed_timestamp_s": 1.03 + frame,
            "consume_now_monotonic_s": 1.04 + frame,
            "frame_source_sim_timestamp_s": 100.0 + frame,
            "source_sim_clock": "gazebo_sim_time",
        },
        "bbox": {"xywh_px": [200, 90, 40, 30], "center_px": [220, 105]},
        "anchors": {"per_grid_point": [anchor(index) for index in range(96)]},
        "calibration": {
            "fit": {**calibration, "filtered_scale": scale - 0.001},
            "applied": calibration,
            "source": "live",
            "fit_reason": "ok",
            "cache_age_s": None,
            "cache_ttl_s": 2.0,
            "calibration_age_s": 0.0,
            "recovery": {"state": "stable", "reseed_count": 0},
        },
        "target_depth": {
            "raw_relative_inverse_depth_quantiles": [0.45, 0.5, 0.55],
            "selected_foreground_statistic": "foreground_median",
        },
        "inverse_depth_filter": {"filtered_inverse_depth": q},
        "raw_range": {
            "ray_scale": ray_scale,
            "filtered_optical_depth_m": 1.0 / (scale * q + offset),
            "physics_slant_range_m": physics,
        },
        "ground_truth": {"distance_m": 8.0},
    }
    result["record_sha256"] = canonical_sha256(result)
    return result


def resign(value: dict) -> dict:
    value.pop("record_sha256", None)
    value["record_sha256"] = canonical_sha256(value)
    return value


def test_sidecar_schema_version_and_checksum() -> None:
    value = row()
    assert verify_record_checksum(value)
    value["frame_index"] = 99
    assert not verify_record_checksum(value)


def test_all_96_anchors_and_reason_integrity() -> None:
    valid, reasons = anchors_status(row())
    assert valid and not reasons
    value = row()
    value["anchors"]["per_grid_point"][4]["accepted"] = False
    value["anchors"]["per_grid_point"][4]["reason"] = ""
    valid, reasons = anchors_status(value)
    assert not valid
    assert "anchor_reason_missing" in reasons


def test_timestamp_order_and_epoch_contract() -> None:
    value = row()
    assert timestamp_order_status(value) == (True, "ok")
    assert not completeness_reasons(value)
    value["timestamps"]["depth_completed_timestamp_s"] = 0.0
    resign(value)
    assert timestamp_order_status(value)[1] == "monotonic_timestamp_order_invalid"


def test_raw_filtered_applied_are_separate_and_live_reconstructs_exactly() -> None:
    value = row()
    assert value["calibration"]["fit"]["raw_scale"] != value["calibration"]["fit"]["filtered_scale"]
    assert value["calibration"]["fit"]["filtered_scale"] != value["calibration"]["applied"]["filtered_scale"]
    exact, error = exact_live_reconstruction(value)
    assert exact
    assert error == pytest.approx(0.0, abs=1e-12)


def test_cache_source_age_ttl_contract() -> None:
    value = row()
    value["calibration"]["source"] = "cached"
    value["calibration"]["cache_age_s"] = 0.4
    resign(value)
    assert not completeness_reasons(value)
    assert value["calibration"]["cache_age_s"] < value["calibration"]["cache_ttl_s"]


def test_stable_freeze_is_causal_but_oracle_is_explicitly_noncausal() -> None:
    rows = [row(index, scale=0.10 + index * 0.001) for index in range(1, 8)]
    first = stable_freeze_parameters(rows)
    assert first is not None and first["causal"]
    changed = copy.deepcopy(rows)
    changed[-2] = row(6, scale=0.001)
    changed[-1] = row(7, scale=0.001)
    second = stable_freeze_parameters(changed)
    assert second == first
    oracle_first = group_median_oracle_parameters(rows)
    oracle_second = group_median_oracle_parameters(changed)
    assert oracle_first is not None and not oracle_first["causal"]
    assert oracle_first["label"] == "NON_CAUSAL_ORACLE"
    assert oracle_second != oracle_first


def test_anchor_refit_is_deterministic() -> None:
    rows = [row(index) for index in range(1, 9)]
    assert anchor_refit_replay(rows) == anchor_refit_replay(rows)


def test_incomplete_diagnostic_is_fail_closed() -> None:
    value = row()
    value["camera_info"]["rectified"] = None
    value["reference_centers"]["optical_center"]["status"] = (
        "configured_runtime_reference_not_optical_center_verified"
    )
    value["target_depth"].pop("raw_relative_inverse_depth_quantiles")
    value["anchors"]["per_grid_point"][0].pop("target_exclusion_result")
    resign(value)
    reasons = completeness_reasons(value)
    assert "rectification_not_recorded" in reasons
    assert "optical_center_unverified" in reasons
    assert "target_roi_q_quantiles_not_recorded" in reasons
    assert "anchor_target_exclusion_result_not_recorded" in reasons
