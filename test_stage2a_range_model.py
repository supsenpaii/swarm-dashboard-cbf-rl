import json
from pathlib import Path

import pytest

import stage2a_range_model as srm
from core_range_stage2a_relative_range_model import SIZE_AGNOSTIC_FEATURES
from metric_depth_calibrator import MetricCalibration
from bearing_range_filter import InverseDepthFilterResult
from target_depth_extractor import TargetDepth
from metric_target_fusion import stage2a_model_features


MODEL_PRESENT = (srm.MODEL_DIR / "size_agnostic.xgb.json").exists()


def _sample_objects():
    target_depth = TargetDepth(
        True, "ok",
        optical_depth_m=7.2, optical_depth_std_m=0.4,
        relative_inverse_depth=260.0, relative_inverse_depth_std=6.0,
        valid_fraction=0.9, sample_count=900,
    )
    inverse_result = InverseDepthFilterResult(
        True, "ok",
        raw_inverse_depth=258.0, filtered_inverse_depth=259.0,
        inverse_depth_std=5.0, robust_center=259.0, robust_sigma=4.0,
        sample_count=5, measurement_accepted=True, outlier=False,
    )
    calibration = MetricCalibration(
        valid=True, reason="ok", scale=0.0013, offset=0.02,
        inlier_count=45, anchor_count=60, residual_m_inv=0.012,
        raw_scale=0.0013, raw_offset=0.02, condition_number=1500.0,
        parameter_uncertainty=0.05, stable_samples=5, stable=True,
        measurement_accepted=True,
        anchor_inverse_depth_quantiles=(200.0, 230.0, 300.0, 400.0, 500.0, 600.0, 650.0),
    )
    return target_depth, inverse_result, calibration


def test_feature_schema_matches_training_script() -> None:
    target_depth, inverse_result, calibration = _sample_objects()
    features = stage2a_model_features(
        target_depth=target_depth, inverse_result=inverse_result,
        calibration=calibration, normalized_x=0.1, normalized_y=-0.05,
        camera_position_ned_m=(0.0, 0.0, -1.2), physics_slant_range_m=7.3,
    )
    assert set(features.keys()) == set(SIZE_AGNOSTIC_FEATURES)


def test_camera_altitude_sign_matches_training_convention() -> None:
    target_depth, inverse_result, calibration = _sample_objects()
    features = stage2a_model_features(
        target_depth=target_depth, inverse_result=inverse_result,
        calibration=calibration, normalized_x=0.0, normalized_y=0.0,
        camera_position_ned_m=(0.0, 0.0, -1.2), physics_slant_range_m=7.3,
    )
    # NED down is negative above ground; altitude is -position[2], matching
    # core_range_stage2a_relative_range_model.extract_rows exactly.
    assert features["camera_altitude_px4_m"] == pytest.approx(1.2)


def test_missing_anchor_quantiles_yields_none_span() -> None:
    target_depth, inverse_result, calibration = _sample_objects()
    calibration = MetricCalibration(
        valid=False, reason="insufficient_anchors",
    )
    features = stage2a_model_features(
        target_depth=target_depth, inverse_result=inverse_result,
        calibration=calibration, normalized_x=0.0, normalized_y=0.0,
        camera_position_ned_m=(0.0, 0.0, -1.0), physics_slant_range_m=5.0,
    )
    assert features["anchor_inverse_depth_span"] is None
    assert features["calibration_scale"] is None


@pytest.mark.skipif(not MODEL_PRESENT, reason="trained model artifact not present")
def test_predict_range_m_stays_within_bounds() -> None:
    target_depth, inverse_result, calibration = _sample_objects()
    features = stage2a_model_features(
        target_depth=target_depth, inverse_result=inverse_result,
        calibration=calibration, normalized_x=0.1, normalized_y=-0.05,
        camera_position_ned_m=(0.0, 0.0, -1.2), physics_slant_range_m=7.3,
    )
    prediction = srm.predict_range_m(features)
    assert prediction is not None
    assert 3.0 <= prediction <= 12.0


@pytest.mark.skipif(not MODEL_PRESENT, reason="trained model artifact not present")
def test_predict_range_m_handles_all_missing_features() -> None:
    prediction = srm.predict_range_m({name: None for name in srm.feature_names()})
    assert prediction is not None
    assert 3.0 <= prediction <= 12.0


@pytest.mark.skipif(not MODEL_PRESENT, reason="trained model artifact not present")
def test_feature_names_match_training_script_order_independent() -> None:
    assert set(srm.feature_names()) == set(SIZE_AGNOSTIC_FEATURES)


def test_known_risk_geometry_flags_only_large_horizontal_offset() -> None:
    assert srm.is_known_risk_geometry({"image_ray_x": 0.15}) is True
    assert srm.is_known_risk_geometry({"image_ray_x": -0.15}) is True
    assert srm.is_known_risk_geometry({"image_ray_x": 0.05}) is False
    assert srm.is_known_risk_geometry({"image_ray_x": None}) is False
    assert srm.is_known_risk_geometry({}) is False


def test_known_risk_geometry_ignores_image_ray_y() -> None:
    # image_ray_y sits around 0.18-0.20 in every Stage 2A scenario regardless
    # of risk (fixed downward gimbal pitch); it must not be able to trigger
    # the flag on its own, or every frame in the corpus fires it.
    assert srm.is_known_risk_geometry({"image_ray_x": 0.0, "image_ray_y": 0.9}) is False


@pytest.mark.skipif(not MODEL_PRESENT, reason="trained model artifact not present")
def test_known_risk_geometry_separates_centered_from_lateral_scenarios() -> None:
    """Ground the threshold in the actual corpus, not just unit examples.

    The flag was first calibrated against three specific groups from the
    2026-08-06 12-group corpus that happened to have negative within-scenario
    slope. Once 2026-08-07 added two more |lateral_offset_m|=0.75 groups
    (pilot_near_lateral_approach, pilot_mid_lateral_yaw_recede) that turned
    out to have *positive* slope, a hardcoded group-name assertion here went
    stale immediately. What the flag actually measures -- confirmed by
    checking every group's flagged fraction against its scenario spec -- is
    "camera-centred vs. large lateral offset" via image_ray_x, not a specific
    list of geometries known to be unsafe. `far_lateral_yaw_approach` is the
    one lateral group that sits just under the threshold (median |ray_x|
    0.125 vs. 0.13) and is excluded deliberately, not a bug.
    """
    from core_range_stage2a_relative_range_model import CORPUS, extract_rows

    plan = json.loads(
        (CORPUS / "collection_plan_pilot.json").read_text()
    )
    lateral_offset = {s["group_id"]: abs(s["lateral_offset_m"]) for s in plan["sessions"]}
    known_edge_case = "pilot_far_lateral_yaw_approach"

    rows = extract_rows(CORPUS)
    by_group: dict[str, list[bool]] = {}
    for row in rows:
        by_group.setdefault(row["group_id"], []).append(
            srm.is_known_risk_geometry(row["features"])
        )
    for group_id, flags in by_group.items():
        flagged_fraction = sum(flags) / len(flags)
        if lateral_offset.get(group_id, 0.0) < 1e-6:
            assert flagged_fraction == 0.0, (
                f"{group_id} (camera-centred) should not be flagged, got {flagged_fraction:.1%}"
            )
        elif group_id == known_edge_case:
            assert flagged_fraction == 0.0, (
                f"{group_id} is the documented sub-threshold edge case; "
                f"got {flagged_fraction:.1%}, update the docstring if this changed"
            )
        else:
            assert flagged_fraction > 0.8, (
                f"{group_id} (lateral offset {lateral_offset[group_id]}m) "
                f"should be flagged, got {flagged_fraction:.1%}"
            )


def test_predict_range_m_returns_none_without_model_artifacts(monkeypatch) -> None:
    monkeypatch.setattr(srm, "_model", None)
    monkeypatch.setattr(srm, "MODEL_DIR", Path("/nonexistent/path"))
    result = srm.predict_range_m({name: 1.0 for name in SIZE_AGNOSTIC_FEATURES})
    assert result is None
