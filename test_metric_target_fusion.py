from __future__ import annotations

import threading
import time

import numpy as np

from bearing_range_filter import RobustBearingRangeFilter
from bearing_target_estimator import TargetEstimate
from depth_model_adapter import CallableDepthAdapter
from depth_worker import DepthJob, LatestDepthWorker
from follow_target_quality_gate import FollowTargetQualityGate
from metric_depth_calibrator import MetricDepthCalibrator
from metric_target_fusion import MetricTargetFusion
from target_depth_extractor import TargetDepthExtractor
from target_fusion_ekf import TargetFusionEKF
from visual_follow_target import CameraRayProjector


def test_robust_range_filter_suppresses_large_single_spikes() -> None:
    filter_ = RobustBearingRangeFilter(
        range_window=7,
        range_minimum_samples=5,
        range_tau_s=0.5,
    )
    measurements = [20.0, 20.2, 19.9, 20.1, 20.0, 75.0, 19.8, 20.2]
    results = [
        filter_.update_range(value, 0.2, index * 0.15)
        for index, value in enumerate(measurements)
    ]

    assert results[5].outlier
    assert results[5].valid
    assert not results[5].measurement_accepted
    assert results[5].range_m is not None
    assert abs(results[5].range_m - 20.0) < 0.4
    assert results[-1].range_m is not None
    assert abs(results[-1].range_m - 20.0) < 0.4


def test_robust_range_filter_tracks_sustained_motion_with_bounded_rate() -> None:
    filter_ = RobustBearingRangeFilter(
        range_window=5,
        range_minimum_samples=3,
        range_tau_s=0.25,
        maximum_range_rate_m_s=3.0,
        maximum_range_acceleration_m_s2=20.0,
    )
    result = None
    for index in range(30):
        timestamp = index * 0.1
        result = filter_.update_range(10.0 + timestamp, 0.15, timestamp)

    assert result is not None and result.valid
    assert result.range_m is not None
    assert 11.5 < result.range_m < 12.9
    assert result.range_rate_m_s is not None
    assert abs(result.range_rate_m_s) <= 3.0
    assert result.measurement_accepted


def test_range_filter_reacquires_a_stable_change_point_without_a_jump() -> None:
    filter_ = RobustBearingRangeFilter(
        range_window=9,
        range_minimum_samples=3,
        range_tau_s=1.0,
        range_absolute_gate_m=0.5,
        range_relative_gate=0.05,
        maximum_range_rate_m_s=2.0,
        maximum_range_acceleration_m_s2=3.0,
        range_reacquire_samples=7,
        range_reacquire_max_sigma_m=0.5,
        range_reacquire_min_duration_s=0.6,
    )
    for index, value in enumerate((20.0, 20.1, 19.9, 20.0, 20.1)):
        filter_.update_range(value, 0.2, index * 0.15)

    quarantined = []
    for offset, value in enumerate(
        (10.0, 10.1, 9.9, 10.0, 10.1, 9.9),
        start=5,
    ):
        quarantined.append(
            filter_.update_range(value, 0.2, offset * 0.15)
        )
    assert all(not result.measurement_accepted for result in quarantined)
    assert all(result.sample_count == 5 for result in quarantined)

    change_point = filter_.update_range(10.0, 0.2, 11 * 0.15)
    assert not change_point.measurement_accepted
    assert change_point.reason == "range_change_point_warming_up"
    # Only one consensus value is promoted, not seven rejected samples.
    assert change_point.sample_count == 1

    recovered = change_point
    for offset, value in enumerate(
        (10.1, 10.0, 9.9, 10.0, 10.1, 10.0),
        start=12,
    ):
        recovered = filter_.update_range(value, 0.2, offset * 0.15)

    assert recovered.valid
    assert recovered.measurement_accepted
    assert recovered.range_m is not None
    assert 10.0 < recovered.range_m < 20.0
    assert abs(recovered.range_rate_m_s) <= 2.0
    status = filter_.status()["range"]
    assert status["change_point_reacquire_count"] == 1
    assert status["change_point_quarantine_count"] == 0


def test_robust_bearing_filter_suppresses_single_angular_spike() -> None:
    filter_ = RobustBearingRangeFilter(
        bearing_window=5,
        bearing_tau_s=0.1,
        maximum_bearing_rate_deg_s=90.0,
        bearing_outlier_deg=5.0,
    )
    normal = np.asarray((1.0, 0.0, 0.0))
    for index in range(5):
        filter_.update_bearing(normal, index * 0.05)
    result = filter_.update_bearing((0.0, 1.0, 0.0), 0.25)

    assert result.valid
    assert result.outlier
    assert result.bearing_ned_unit is not None
    assert result.bearing_ned_unit[0] > 0.99


def test_robust_range_filter_never_initializes_from_implausible_depth() -> None:
    filter_ = RobustBearingRangeFilter(
        range_window=7,
        range_minimum_samples=5,
        maximum_range_m=80.0,
        maximum_measurement_std_m=3.0,
    )
    results = [
        filter_.update_range(value, 12.0, index * 0.15)
        for index, value in enumerate((120.0, 300.0, 75.0, 500.0, 70.0))
    ]

    assert all(not result.valid for result in results)
    assert all(not result.measurement_accepted for result in results)
    assert results[-1].sample_count == 0


def test_metric_depth_calibrator_recovers_affine_inverse_depth() -> None:
    rng = np.random.default_rng(11)
    relative = np.linspace(0.1, 1.2, 80)
    inverse_metric = 0.18 * relative + 0.035
    metric = 1.0 / inverse_metric
    metric += rng.normal(0.0, 0.02, metric.shape)
    metric[::13] *= 1.8

    calibrator = MetricDepthCalibrator(
        residual_threshold_m_inv=0.01,
        minimum_anchors=12,
    )
    result = calibrator.fit(relative, metric)

    assert result.valid
    assert result.inlier_count >= 60
    assert result.scale is not None
    assert result.offset is not None
    assert abs(result.scale - 0.18) < 0.02
    assert abs(result.offset - 0.035) < 0.02
    covariance = np.asarray(result.parameter_covariance_ab)
    assert covariance.shape == (2, 2)
    assert np.all(np.isfinite(covariance))
    assert np.all(np.linalg.eigvalsh(covariance) >= -1e-15)
    assert result.residual_std_m_inv is not None


def test_calibration_change_point_requires_consensus_before_reseed() -> None:
    relative = np.linspace(0.1, 1.2, 100)
    old_metric = 1.0 / (0.18 * relative + 0.035)
    new_metric = 1.0 / (0.32 * relative + 0.090)
    calibrator = MetricDepthCalibrator(
        stable_samples_required=3,
        change_point_samples_required=5,
    )
    for _ in range(3):
        assert calibrator.fit(relative, old_metric).valid
    original_scale = calibrator.scale

    quarantined = [
        calibrator.fit(relative, new_metric)
        for _ in range(4)
    ]

    assert all(not result.valid for result in quarantined)
    assert all(
        result.reason == "calibration_temporal_jump"
        for result in quarantined
    )
    assert calibrator.scale == original_scale
    assert calibrator.recovery_status()["quarantine_sample_count"] == 4

    reseeded = calibrator.fit(relative, new_metric)

    assert reseeded.valid
    assert reseeded.measurement_accepted
    assert reseeded.change_point_reseeded
    assert not reseeded.stable
    assert reseeded.reason == "calibration_change_point_reseeded"
    assert reseeded.scale is not None
    assert abs(reseeded.scale - 0.32) < 0.02
    assert calibrator.recovery_status()["reseed_count"] == 1

    for _ in range(2):
        recovered = calibrator.fit(relative, new_metric)
    assert recovered.stable
    assert recovered.reason == "ok"


def test_target_inverse_depth_outside_anchor_support_is_rejected() -> None:
    relative = np.linspace(0.2, 1.0, 100)
    metric = 1.0 / (0.18 * relative + 0.035)
    calibrator = MetricDepthCalibrator()
    assert calibrator.fit(relative, metric).valid

    supported = calibrator.target_inverse_depth_support(0.60)
    extrapolated = calibrator.target_inverse_depth_support(2.50)

    assert supported["supported"]
    assert supported["reason"] == "ok"
    assert not extrapolated["supported"]
    assert (
        extrapolated["reason"]
        == "target_inverse_depth_outside_anchor_support"
    )
    assert float(extrapolated["extrapolation_iqr"]) > 1.0


def test_calibration_uncertainty_uses_full_scale_offset_covariance() -> None:
    rng = np.random.default_rng(21)
    relative = np.linspace(180.0, 420.0, 120)
    inverse_metric = 0.00015 * relative + 0.020
    noisy_inverse_metric = inverse_metric + rng.normal(
        0.0,
        0.0008,
        relative.shape,
    )
    metric = 1.0 / noisy_inverse_metric
    calibrator = MetricDepthCalibrator(
        residual_threshold_m_inv=0.005,
        minimum_anchors=20,
        temporal_window=5,
    )
    result = calibrator.fit(relative, metric)

    assert result.valid
    components = calibrator.inverse_metric_uncertainty(300.0, 12.0)
    covariance = np.asarray(result.parameter_covariance_ab)
    jacobian = np.asarray((300.0, 1.0))
    expected_parameter_std = float(
        np.sqrt(max(0.0, jacobian @ covariance @ jacobian))
    )
    expected_inverse_depth_std = abs(float(result.scale)) * 12.0
    expected_total = float(
        np.sqrt(
            expected_parameter_std**2
            + expected_inverse_depth_std**2
            + float(result.residual_std_m_inv) ** 2
        )
    )

    assert np.isclose(
        components["parameter_std_m_inv"],
        expected_parameter_std,
    )
    assert np.isclose(
        components["inverse_depth_std_m_inv"],
        expected_inverse_depth_std,
    )
    assert np.isclose(components["total_std_m_inv"], expected_total)
    legacy_overestimate = float(result.parameter_uncertainty) * 301.0
    assert components["parameter_std_m_inv"] < legacy_overestimate * 0.1


def test_calibration_uncertainty_increases_with_fit_residual() -> None:
    relative = np.linspace(100.0, 400.0, 100)
    base_inverse_metric = 0.0002 * relative + 0.025
    low_noise = MetricDepthCalibrator(residual_threshold_m_inv=0.02)
    high_noise = MetricDepthCalibrator(residual_threshold_m_inv=0.02)
    phase = np.linspace(0.0, 8.0 * np.pi, relative.size)
    assert low_noise.fit(
        relative,
        1.0 / (base_inverse_metric + 0.0002 * np.sin(phase)),
    ).valid
    assert high_noise.fit(
        relative,
        1.0 / (base_inverse_metric + 0.004 * np.sin(phase)),
    ).valid

    low = low_noise.inverse_metric_uncertainty(250.0, 5.0)
    high = high_noise.inverse_metric_uncertainty(250.0, 5.0)
    assert high["residual_std_m_inv"] > low["residual_std_m_inv"]
    assert high["total_std_m_inv"] > low["total_std_m_inv"]


def test_cached_calibration_inflates_covariance_with_age() -> None:
    relative = np.linspace(0.1, 1.2, 80)
    metric = 1.0 / (0.18 * relative + 0.035)
    calibrator = MetricDepthCalibrator(stable_samples_required=3)
    for _ in range(calibrator.stable_samples_required):
        assert calibrator.fit(relative, metric).valid
    assert calibrator.last.stable

    recent = calibrator.use_cached(
        0.2,
        scale_drift_fraction_per_s=0.08,
        offset_drift_m_inv_per_s=0.003,
    )
    older = calibrator.use_cached(
        1.2,
        scale_drift_fraction_per_s=0.08,
        offset_drift_m_inv_per_s=0.003,
    )

    assert recent is not None and older is not None
    assert recent.reason == "cached_calibration"
    assert not recent.measurement_accepted
    assert np.max(np.diag(older.parameter_covariance_ab)) > np.max(
        np.diag(recent.parameter_covariance_ab)
    )
    assert older.parameter_uncertainty > recent.parameter_uncertainty


def test_metric_fusion_reset_can_preserve_only_stable_calibration() -> None:
    fusion = MetricTargetFusion(
        CameraRayProjector(200.0, 200.0),
        enabled=False,
    )
    relative = np.linspace(0.1, 1.2, 80)
    metric = 1.0 / (0.18 * relative + 0.035)
    for _ in range(fusion.calibrator.stable_samples_required):
        assert fusion.calibrator.fit(relative, metric).valid
    fusion._last_valid_calibration_timestamp_s = 10.0
    fusion.ekf.update_position(
        (8.0, 0.0, 0.0),
        np.eye(3) * 0.1,
        10.0,
    )

    fusion.reset(preserve_calibration=True)

    assert fusion.calibrator.last.valid
    assert fusion.calibrator.last.stable
    assert fusion.calibrator.last.reason == "ok"
    assert fusion.ekf.status()["update_count"] == 0
    assert fusion.status()["calibration_cache"]["source"] == "prewarmed"
    assert fusion.status()["reason"] == "calibration_preloaded"


def test_target_depth_extractor_rejects_bbox_background() -> None:
    calibrator = MetricDepthCalibrator()
    relative = np.linspace(0.2, 1.0, 60)
    metric = 1.0 / (0.2 * relative + 0.04)
    assert calibrator.fit(relative, metric).valid

    depth_map = np.full((100, 120), 0.25, dtype=np.float32)
    depth_map[30:70, 40:80] = 0.80
    depth_map[44:56, 54:66] = 0.85
    extractor = TargetDepthExtractor(minimum_samples=20)
    result = extractor.extract(
        depth_map,
        (35.0, 25.0, 50.0, 50.0),
        calibrator,
    )

    assert result.valid
    assert result.relative_inverse_depth is not None
    assert result.relative_inverse_depth > 0.7
    expected = 1.0 / (0.2 * 0.8 + 0.04)
    assert result.optical_depth_m is not None
    assert abs(result.optical_depth_m - expected) < 0.6


def test_target_fusion_ekf_estimates_target_velocity() -> None:
    ekf = TargetFusionEKF(
        acceleration_noise_m_s2=0.3,
        velocity_ready_updates=4,
    )
    camera = (0.0, 0.0, -10.0)
    covariance = np.eye(3) * 0.04
    for index in range(12):
        timestamp = index * 0.1
        position = (12.0 + timestamp, 3.0, 0.0)
        ekf.update_position(position, covariance, timestamp)
        relative = np.asarray(position) - np.asarray(camera)
        ekf.update_bearing(
            camera,
            relative / np.linalg.norm(relative),
            angular_std_rad=0.005,
            timestamp_s=timestamp,
        )

    estimate = ekf.estimate(1.1)
    assert estimate.valid
    assert estimate.velocity_valid
    assert estimate.position_ned_m is not None
    assert estimate.velocity_ned_m_s is not None
    assert abs(estimate.position_ned_m[0] - 13.1) < 0.3
    assert abs(estimate.velocity_ned_m_s[0] - 1.0) < 0.35
    assert abs(estimate.velocity_ned_m_s[1]) < 0.2


def test_stationary_target_converges_to_zero_velocity_and_fixed_position() -> None:
    ekf = TargetFusionEKF(
        acceleration_noise_m_s2=0.25,
        velocity_ready_updates=4,
    )
    covariance = np.eye(3) * 0.03
    target = np.asarray((14.0, -3.0, 0.0))
    for index in range(14):
        ekf.update_position(target, covariance, index * 0.1)

    estimate = ekf.estimate(1.3)
    assert estimate.valid
    assert estimate.velocity_valid
    assert estimate.estimator_mode == "STATIONARY"
    assert estimate.stationary_probability > 0.9
    assert estimate.position_ned_m is not None
    assert estimate.velocity_ned_m_s == (0.0, 0.0, 0.0)
    assert np.linalg.norm(np.asarray(estimate.position_ned_m) - target) < 0.05


def test_stationary_global_target_is_not_confused_with_camera_ego_motion() -> None:
    ekf = TargetFusionEKF(
        acceleration_noise_m_s2=0.25,
        velocity_ready_updates=4,
    )
    covariance = np.eye(3) * 0.04
    target = np.asarray((18.0, 4.0, 0.0))
    for index in range(16):
        timestamp = index * 0.1
        camera = np.asarray((0.4 * index, -0.1 * index, -10.0))
        relative = target - camera
        bearing = relative / np.linalg.norm(relative)
        measured_target = camera + bearing * np.linalg.norm(relative)
        ekf.update_position(measured_target, covariance, timestamp)
        ekf.update_bearing(camera, bearing, 0.004, timestamp)

    estimate = ekf.estimate(1.5)
    assert estimate.valid
    assert estimate.estimator_mode == "STATIONARY"
    assert estimate.position_ned_m is not None
    assert np.linalg.norm(np.asarray(estimate.position_ned_m) - target) < 0.12
    assert estimate.velocity_ned_m_s == (0.0, 0.0, 0.0)


def test_bearing_only_updates_do_not_hide_stale_metric_range() -> None:
    ekf = TargetFusionEKF(velocity_ready_updates=2)
    covariance = np.eye(3) * 0.05
    ekf.update_position((10.0, 0.0, 0.0), covariance, 0.0)
    ekf.update_position((10.0, 0.0, 0.0), covariance, 0.1)
    for timestamp in (0.5, 1.0, 1.5, 2.0):
        ekf.update_bearing(
            (0.0, 0.0, -10.0),
            (2.0**-0.5, 0.0, 2.0**-0.5),
            0.005,
            timestamp,
        )

    estimate = ekf.estimate(2.0, stale_timeout_s=1.5)
    assert not estimate.valid
    assert estimate.reason == "stale"
    assert estimate.measurement_age_s is not None
    assert estimate.measurement_age_s >= 1.9


def test_inverse_depth_filter_rejects_spike_before_history() -> None:
    filter_ = RobustBearingRangeFilter(
        inverse_depth_window=5,
        inverse_depth_minimum_samples=3,
    )
    results = [
        filter_.update_inverse_depth(value, 0.01, index * 0.1)
        for index, value in enumerate((0.50, 0.51, 0.49, 2.0, 0.50))
    ]
    assert results[2].measurement_accepted
    assert results[3].outlier
    assert not results[3].measurement_accepted
    assert results[3].sample_count == 3
    assert results[-1].filtered_inverse_depth is not None
    assert abs(results[-1].filtered_inverse_depth - 0.50) < 0.03


def test_strong_inverse_depth_consensus_reduces_random_uncertainty() -> None:
    filter_ = RobustBearingRangeFilter(
        inverse_depth_window=15,
        inverse_depth_minimum_samples=7,
        inverse_depth_tau_s=1.5,
        inverse_depth_uncertainty_floor_fraction=0.35,
    )
    result = None
    for index, value in enumerate(
        (400.0, 403.0, 398.0, 402.0, 399.0, 401.0, 400.5)
    ):
        result = filter_.update_inverse_depth(
            value,
            30.0,
            index * 0.15,
        )

    assert result is not None and result.valid
    assert result.measurement_accepted
    assert result.inverse_depth_std is not None
    assert 0.0 < result.inverse_depth_std <= 30.0 / np.sqrt(7.0) + 1e-9
    accepted_count = result.sample_count

    spike = filter_.update_inverse_depth(650.0, 30.0, 1.2)

    assert spike.outlier
    assert not spike.measurement_accepted
    assert spike.sample_count == accepted_count


def test_temporal_metric_uncertainty_keeps_systematic_floor() -> None:
    relative = np.linspace(0.1, 1.2, 80)
    metric = 1.0 / (0.18 * relative + 0.035)
    calibrator = MetricDepthCalibrator(
        stable_samples_required=3,
        temporal_uncertainty_floor_fraction=0.35,
    )
    for _ in range(5):
        assert calibrator.fit(relative, metric).valid

    single = calibrator.inverse_metric_uncertainty(0.6, 0.02, 1)
    temporal = calibrator.inverse_metric_uncertainty(0.6, 0.02, 9)

    assert temporal["residual_std_m_inv"] < single["residual_std_m_inv"]
    assert np.isclose(
        temporal["residual_std_m_inv"],
        temporal["raw_residual_std_m_inv"] * 0.35,
    )
    assert temporal["temporal_sample_count"] == 9.0


def test_quality_gate_requires_consecutive_stable_metric_samples() -> None:
    ekf = TargetFusionEKF(velocity_ready_updates=2)
    gate = FollowTargetQualityGate(stable_samples_required=3)
    covariance = np.eye(3) * 0.1
    outputs = []
    for index in range(3):
        ekf.update_position(
            (10.0 + 0.1 * index, 0.0, 0.0),
            covariance,
            index * 0.1,
        )
        outputs.append(
            gate.update(
                ekf.estimate(index * 0.1),
                calibration_valid=True,
                target_depth_valid=True,
            )
        )
    assert not outputs[0].ready
    assert not outputs[1].ready
    assert outputs[2].ready


def test_quality_gate_preserves_specific_upstream_rejection_reason() -> None:
    gate = FollowTargetQualityGate()
    estimate = TargetFusionEKF().estimate(1.0)

    calibration = gate.update(
        estimate,
        calibration_valid=False,
        target_depth_valid=False,
        calibration_reason="calibration_temporal_jump",
    )
    target_support = gate.update(
        estimate,
        calibration_valid=True,
        target_depth_valid=False,
        target_depth_reason="target_inverse_depth_outside_anchor_support",
    )

    assert calibration.reason == "calibration_temporal_jump"
    assert (
        target_support.reason
        == "target_inverse_depth_outside_anchor_support"
    )


def test_latest_depth_worker_drops_stale_pending_frames() -> None:
    release = threading.Event()

    def infer(frame: np.ndarray) -> np.ndarray:
        release.wait(timeout=1.0)
        return np.full(frame.shape[:2], float(frame[0, 0, 0]))

    worker = LatestDepthWorker(CallableDepthAdapter(infer))
    frame = np.ones((12, 16, 3), dtype=np.uint8)
    worker.submit(DepthJob(frame, 1.0, 1))
    worker.submit(DepthJob(frame * 2, 2.0, 2))
    worker.submit(DepthJob(frame * 3, 3.0, 3))
    release.set()
    deadline = time.monotonic() + 1.0
    result = None
    while time.monotonic() < deadline:
        _, result = worker.latest()
        if result is not None:
            break
        time.sleep(0.01)
    worker.stop()

    assert worker.dropped >= 1
    assert result is not None
    assert result.valid
    assert result.frame_index in {1, 3}
    status = worker.status()
    assert status["processed"] >= 1
    assert not status["active"]
    assert status["last_exception"] == ""


def test_metric_target_fusion_reaches_ready_without_lateral_bootstrap() -> None:
    projector = CameraRayProjector(200.0, 200.0)
    height, width = 120, 160
    camera = (0.0, 0.0, -10.0)
    pitch = np.deg2rad(45.0)
    quaternion = (0.0, np.sin(pitch / 2.0), 0.0, np.cos(pitch / 2.0))
    inverse = np.full((height, width), np.nan, dtype=np.float32)
    for v in range(height):
        for u in range(width):
            ray = projector.pixel_to_ned_ray(
                u,
                v,
                width,
                height,
                quaternion,
            )
            if ray[2] <= 1e-4:
                continue
            ray_range = (0.0 - camera[2]) / ray[2]
            sensor_forward = 1.0 / np.sqrt(
                1.0
                + ((u - width / 2.0) / projector.fx) ** 2
                + ((v - height / 2.0) / projector.fy) ** 2
            )
            optical_depth = ray_range * sensor_forward
            inverse[v, u] = (1.0 / optical_depth - 0.03) / 0.2
    bbox = (62.0, 42.0, 36.0, 40.0)
    inverse[48:76, 68:92] = (1.0 / 12.0 - 0.03) / 0.2
    adapter = CallableDepthAdapter(
        lambda _frame: inverse.copy(),
        source="synthetic_midas",
    )
    fusion = MetricTargetFusion(
        projector,
        adapter=adapter,
        enabled=True,
    )
    # Test drives measurement_timestamp_s at a fixed 0.2s cadence; pin the
    # submission gate to that cadence explicitly so this test's convergence
    # behavior doesn't depend on whatever depth_rate_hz production defaults
    # to (see metric_target_fusion.py SWARM_METRIC_TARGET_DEPTH_RATE_HZ).
    fusion.depth_rate_hz = 10.0
    frame = np.zeros((height, width, 3), dtype=np.uint8)
    estimate = None
    for index in range(20):
        timestamp = index * 0.2
        estimate = fusion.update(
            frame_bgr=frame,
            bbox_xywh=bbox,
            bbox_center_px=(80.0, 62.0),
            tracking_score=0.99,
            tracking_valid=True,
            measurement_timestamp_s=timestamp,
            now_s=timestamp,
            frame_index=index,
            camera_position_ned_m=camera,
            camera_quaternion_xyzw=quaternion,
        )
        if estimate.valid:
            break
        time.sleep(0.015)
    fusion.shutdown()

    assert estimate is not None
    assert estimate.valid, fusion.status()
    assert estimate.position_ned_m is not None
    assert estimate.range_m is not None
    assert 10.5 < estimate.range_m < 14.0
    status = fusion.status()
    assert status["calibration"]["valid"]
    assert len(status["calibration"]["parameter_covariance_ab"]) == 2
    assert status["calibration"]["residual_std_m_inv"] is not None
    assert len(status["calibration"]["anchor_inverse_depth_quantiles"]) == 7
    assert len(status["m52_anchors"]["inverse_depth_quantiles"]) == 7
    assert status["m52_anchors"]["accepted_ground_anchor_count"] >= 12
    assert status["m52_anchors"]["spatial_bin_count"] >= 2
    assert status["target_inverse_depth_support"]["supported"]
    uncertainty = status["range_uncertainty_components"]
    assert uncertainty["ray_range_std_m"] >= 0.15
    assert uncertainty["total_std_m_inv"] > 0.0
    correction = status["range_residual_correction"]
    assert correction["mode"] == "off"
    assert correction["request_count"] >= 1
    assert correction["applied_count"] == 0
    applicability = status["range_applicability_gate"]
    assert applicability["mode"] == "active"
    assert applicability["request_count"] >= 1
    assert applicability["applicable_count"] >= 1
    assert applicability["enforced_rejection_count"] == 0
    assert not status["range_residual_dataset"]["enabled"]


def test_metric_target_fusion_prewarms_without_initializing_target() -> None:
    projector = CameraRayProjector(200.0, 200.0)
    height, width = 120, 160
    camera = (0.0, 0.0, -10.0)
    pitch = np.deg2rad(45.0)
    quaternion = (0.0, np.sin(pitch / 2.0), 0.0, np.cos(pitch / 2.0))
    inverse = np.full((height, width), np.nan, dtype=np.float32)
    for v in range(height):
        for u in range(width):
            ray = projector.pixel_to_ned_ray(
                u,
                v,
                width,
                height,
                quaternion,
            )
            if ray[2] <= 1e-4:
                continue
            ray_range = -camera[2] / ray[2]
            sensor_forward = 1.0 / np.sqrt(
                1.0
                + ((u - width / 2.0) / projector.fx) ** 2
                + ((v - height / 2.0) / projector.fy) ** 2
            )
            optical_depth = ray_range * sensor_forward
            inverse[v, u] = (1.0 / optical_depth - 0.03) / 0.2
    fusion = MetricTargetFusion(
        projector,
        adapter=CallableDepthAdapter(
            lambda _frame: inverse.copy(),
            source="synthetic_midas",
        ),
        enabled=True,
    )
    # See depth_rate_hz note in
    # test_metric_target_fusion_reaches_ready_without_lateral_bootstrap.
    fusion.depth_rate_hz = 10.0
    frame = np.zeros((height, width, 3), dtype=np.uint8)
    for index in range(12):
        timestamp = index * 0.2
        fusion.prewarm(
            frame_bgr=frame,
            measurement_timestamp_s=timestamp,
            now_s=timestamp,
            frame_index=index,
            camera_position_ned_m=camera,
            camera_quaternion_xyzw=quaternion,
        )
        if fusion.calibrator.last.stable:
            break
        time.sleep(0.015)
    status = fusion.status()
    fusion.shutdown()

    assert status["calibration"]["stable"], status
    assert status["calibration_cache"]["source"] == "prewarm"
    assert status["calibration_cache"]["prewarm_result_count"] >= 3
    assert status["calibration_cache"]["prewarm_accepted_count"] >= 1
    assert (
        status["calibration_cache"]["prewarm_result_count"]
        == status["calibration_cache"]["prewarm_accepted_count"]
        + status["calibration_cache"]["prewarm_rejected_count"]
    )
    assert status["estimator"]["update_count"] == 0
    assert not status["quality_gate"]["initialized"]


def test_prewarm_reports_failed_worker_results_instead_of_appearing_stuck() -> None:
    def fail_inference(_frame: np.ndarray) -> np.ndarray:
        raise RuntimeError("synthetic failure")

    fusion = MetricTargetFusion(
        CameraRayProjector(200.0, 200.0),
        adapter=CallableDepthAdapter(fail_inference, source="failing"),
        enabled=True,
    )
    frame = np.zeros((32, 48, 3), dtype=np.uint8)
    for index in range(20):
        timestamp = index * 0.2
        fusion.prewarm(
            frame_bgr=frame,
            measurement_timestamp_s=timestamp,
            now_s=timestamp,
            frame_index=index,
            camera_position_ned_m=(0.0, 0.0, -10.0),
            camera_quaternion_xyzw=(0.0, 0.0, 0.0, 1.0),
        )
        if fusion.status()["calibration_cache"]["prewarm_result_count"]:
            break
        time.sleep(0.01)
    status = fusion.status()
    fusion.shutdown()

    cache = status["calibration_cache"]
    assert cache["prewarm_result_count"] >= 1
    assert cache["prewarm_accepted_count"] == 0
    assert cache["prewarm_rejected_count"] >= 1
    assert str(cache["last_prewarm_reason"]).startswith("inference_failed:")
    assert status["worker"]["processed"] >= 1
    assert status["worker"]["failed"] >= 1
    assert str(status["worker"]["last_exception"]).startswith(
        "inference_failed:"
    )


def test_passive_multiview_can_initialize_after_independent_quality_warmup() -> None:
    fusion = MetricTargetFusion(
        CameraRayProjector(200.0, 200.0),
        enabled=False,
    )
    covariance = np.eye(3) * 0.1
    first = None
    for index in range(10):
        timestamp = 0.1 + index * 0.1
        triangulated = TargetEstimate(
            timestamp_s=timestamp,
            state="VALID",
            valid=True,
            reason="ok",
            position_ned_m=(10.0 + 0.01 * index, 2.0, 0.0),
            covariance=tuple(tuple(row) for row in covariance),
            range_m=10.2 + 0.01 * index,
            range_std_m=0.4,
            reprojection_error_px=1.0,
            baseline_m=0.8,
            intersection_angle_deg=5.0,
            condition_number=500.0,
            estimate_age_ms=0.0,
        )
        accepted = fusion.fuse_multiview(triangulated, timestamp)
        if index == 0:
            first = accepted

    assert first is False
    assert fusion.ready
    assert fusion.ready_for_safe_distance_lock
    assert fusion.active_metric_source == "multiview"
    prediction = fusion.predict(1.05)
    assert prediction.valid
    assert prediction.position_ned_m is not None
    assert prediction.range_m is not None
    status = fusion.status()["multiview_crosscheck"]
    assert status["accepted"]
    assert status["reason"] == "accepted"
    assert status["quality_gate"]["initialized"]
    assert status["range_filter"]["sample_count"] >= 7
    update_count = fusion.ekf.status()["update_count"]
    spike = TargetEstimate(
        timestamp_s=1.2,
        state="VALID",
        valid=True,
        reason="ok",
        position_ned_m=(30.0, 2.0, 0.0),
        covariance=tuple(tuple(row) for row in covariance),
        range_m=30.0,
        range_std_m=0.4,
        reprojection_error_px=1.0,
        baseline_m=0.8,
        intersection_angle_deg=5.0,
        condition_number=500.0,
        estimate_age_ms=0.0,
    )
    assert not fusion.fuse_multiview(spike, 1.2)
    assert fusion.ekf.status()["update_count"] == update_count
    assert (
        fusion.status()["multiview_crosscheck"]["reason"]
        == "range_change_point_quarantine"
    )


def test_passive_multiview_rejects_degenerate_geometry_without_warmup() -> None:
    fusion = MetricTargetFusion(
        CameraRayProjector(200.0, 200.0),
        enabled=False,
    )
    covariance = tuple(tuple(row) for row in np.eye(3) * 0.1)
    degenerate = TargetEstimate(
        timestamp_s=0.1,
        state="VALID",
        valid=True,
        reason="ok",
        position_ned_m=(10.0, 2.0, 0.0),
        covariance=covariance,
        range_m=10.2,
        range_std_m=0.4,
        reprojection_error_px=8.0,
        baseline_m=0.2,
        intersection_angle_deg=1.0,
        condition_number=2.0e7,
        estimate_age_ms=0.0,
    )

    assert not fusion.fuse_multiview(degenerate, 0.1)
    status = fusion.status()["multiview_crosscheck"]
    assert not status["accepted"]
    assert status["reason"] == "insufficient_baseline"
    assert status["range_filter"]["sample_count"] == 0
    assert fusion.ekf.status()["update_count"] == 0
    assert not fusion.ready
