import math
import unittest

import numpy as np

from bearing_target_estimator import (
    BearingObservation,
    BearingTargetEstimator,
    BearingTargetEstimatorConfig,
)
from visual_follow_target import CameraRayProjector


def observation(
    camera,
    target,
    timestamp,
    *,
    frame_index=0,
    score=0.95,
    pose_age=0.01,
    gimbal_age=0.01,
    bearing_offset=None,
):
    camera_array = np.asarray(camera, dtype=float)
    direction = np.asarray(target, dtype=float) - camera_array
    direction /= np.linalg.norm(direction)
    if bearing_offset is not None:
        direction = direction + np.asarray(bearing_offset, dtype=float)
        direction /= np.linalg.norm(direction)
    return BearingObservation(
        timestamp_s=timestamp,
        camera_position_ned_m=tuple(camera_array),
        bearing_ned_unit=tuple(direction),
        bbox_center_px=(320.0, 180.0),
        bbox_size_px=(80.0, 60.0),
        tracking_score=score,
        focal_length_px=300.0,
        pose_age_s=pose_age,
        gimbal_age_s=gimbal_age,
        frame_index=frame_index,
    )


def estimator_config(**overrides):
    values = dict(
        minimum_observations=5,
        maximum_observations=30,
        observation_window_s=4.0,
        minimum_tracking_score=0.7,
        maximum_pose_age_s=0.5,
        maximum_gimbal_age_s=0.5,
        minimum_baseline_m=0.5,
        minimum_intersection_angle_deg=2.0,
        maximum_reprojection_error_px=4.0,
        minimum_range_m=1.0,
        maximum_range_m=80.0,
        maximum_range_std_m=2.0,
        maximum_range_relative_std=0.25,
        stale_timeout_s=0.5,
        maximum_condition_number=1e7,
        bearing_noise_px=1.5,
        acceleration_noise_m_s2=1.0,
        innovation_gate_sigma=5.0,
    )
    values.update(overrides)
    return BearingTargetEstimatorConfig(**values)


class CameraBearingTests(unittest.TestCase):
    def test_center_pixel_points_north_for_identity_gazebo_camera(self):
        projector = CameraRayProjector(300.0, 300.0)
        ray = projector.pixel_to_ned_ray(
            320.0, 180.0, 640, 360, (0.0, 0.0, 0.0, 1.0)
        )
        self.assertTrue(np.allclose(ray, (0.0, 1.0, 0.0), atol=1e-9))

    def test_image_right_has_negative_east_component_for_identity_camera(self):
        projector = CameraRayProjector(300.0, 300.0)
        ray = projector.pixel_to_ned_ray(
            420.0, 180.0, 640, 360, (0.0, 0.0, 0.0, 1.0)
        )
        self.assertLess(ray[1], 1.0)
        self.assertLess(ray[0], 0.0)


class BearingTriangulationTests(unittest.TestCase):
    def test_default_window_is_physically_compatible_with_rgb_bootstrap(self):
        config = BearingTargetEstimatorConfig()
        self.assertGreaterEqual(config.maximum_observations, 60)
        retained_seconds_at_30_fps = config.maximum_observations / 30.0
        self.assertGreaterEqual(
            retained_seconds_at_30_fps
            * config.bootstrap_lateral_speed_m_s,
            config.minimum_baseline_m,
        )

    def run_track(self, cameras, target, **config_overrides):
        estimator = BearingTargetEstimator(estimator_config(**config_overrides))
        estimate = None
        for index, camera in enumerate(cameras):
            estimate = estimator.update(
                observation(camera, target, index * 0.1, frame_index=index)
            )
        return estimator, estimate

    def test_lateral_baseline_recovers_static_target(self):
        target = (10.0, 4.0, -2.0)
        cameras = [(0.0, y, -2.0) for y in np.linspace(-0.5, 0.5, 8)]
        _, estimate = self.run_track(cameras, target)
        self.assertIsNotNone(estimate)
        self.assertTrue(estimate.valid, estimate.reason)
        self.assertTrue(np.allclose(estimate.position_ned_m, target, atol=0.15))
        self.assertGreaterEqual(estimate.baseline_m, 1.0)

    def test_rotation_only_is_not_a_metric_baseline(self):
        target = (10.0, 0.0, -2.0)
        cameras = [(0.0, 0.0, -2.0)] * 8
        estimator, estimate = self.run_track(cameras, target)
        self.assertFalse(estimate.valid)
        self.assertEqual(estimate.reason, "insufficient_baseline")
        guidance = estimator.bootstrap_guidance()
        self.assertTrue(guidance["required"])
        self.assertFalse(guidance["command_authorized"])

    def test_small_baseline_is_rejected(self):
        target = (12.0, 2.0, -1.0)
        cameras = [(0.0, y, -1.0) for y in np.linspace(0.0, 0.1, 8)]
        _, estimate = self.run_track(cameras, target)
        self.assertFalse(estimate.valid)
        self.assertEqual(estimate.reason, "insufficient_baseline")

    def test_straight_closing_geometry_is_rejected_as_weak(self):
        target = (20.0, 0.0, -2.0)
        cameras = [(x, 0.0, -2.0) for x in np.linspace(0.0, 1.0, 8)]
        _, estimate = self.run_track(cameras, target)
        self.assertFalse(estimate.valid)
        self.assertIn(
            estimate.reason,
            {"weak_intersection_angle", "degenerate_geometry"},
        )

    def test_noisy_bearings_remain_close(self):
        rng = np.random.default_rng(3)
        target = np.asarray((9.0, 3.0, -1.5))
        estimator = BearingTargetEstimator(
            estimator_config(maximum_reprojection_error_px=6.0)
        )
        estimate = None
        for index, y in enumerate(np.linspace(-0.7, 0.7, 12)):
            noise = rng.normal(0.0, 0.0015, 3)
            estimate = estimator.update(
                observation(
                    (0.0, y, -1.5),
                    target,
                    index * 0.1,
                    frame_index=index,
                    bearing_offset=noise,
                )
            )
        self.assertTrue(estimate.valid, estimate.reason)
        self.assertLess(np.linalg.norm(np.asarray(estimate.position_ned_m) - target), 0.5)

    def test_outlier_is_robustly_rejected_during_bootstrap(self):
        target = (10.0, 3.0, -2.0)
        estimator = BearingTargetEstimator(
            estimator_config(minimum_observations=6, maximum_reprojection_error_px=5.0)
        )
        estimate = None
        for index, y in enumerate(np.linspace(-0.8, 0.8, 10)):
            offset = (0.0, 0.25, 0.0) if index == 4 else None
            estimate = estimator.update(
                observation(
                    (0.0, y, -2.0),
                    target,
                    index * 0.1,
                    frame_index=index,
                    bearing_offset=offset,
                )
            )
        self.assertIsNotNone(estimate.position_ned_m)
        self.assertLess(np.linalg.norm(np.asarray(estimate.position_ned_m) - target), 0.8)

    def test_pixel_domain_consensus_rejects_multiple_bbox_outliers(self):
        target = np.asarray((10.0, 3.0, -2.0))
        estimator = BearingTargetEstimator(
            estimator_config(
                minimum_observations=8,
                maximum_observations=40,
                maximum_reprojection_error_px=3.0,
            )
        )
        observation_count = 30
        observations = []
        for index, y in enumerate(np.linspace(-0.8, 0.8, observation_count)):
            # Six moderate bbox-centre errors are small enough to survive the
            # metre-domain consensus but must not weaken the 3 px final gate.
            offset = (0.0, 0.018, 0.012) if index % 5 == 2 else None
            observations.append(
                observation(
                    (0.0, y, -2.0),
                    target,
                    index * 0.1,
                    frame_index=index,
                    bearing_offset=offset,
                )
            )

        geometry = estimator._triangulate(observations)

        self.assertTrue(geometry.valid, geometry.reason)
        self.assertLessEqual(geometry.reprojection_error_px, 3.0)
        self.assertGreaterEqual(
            geometry.observation_count,
            math.ceil(0.65 * observation_count),
        )
        self.assertLess(geometry.observation_count, observation_count)
        self.assertLess(
            np.linalg.norm(np.asarray(geometry.position) - target),
            0.5,
        )

    def test_target_behind_bearings_is_rejected(self):
        estimator = BearingTargetEstimator(estimator_config())
        estimate = None
        target = (10.0, 2.0, -2.0)
        for index, y in enumerate(np.linspace(-0.5, 0.5, 8)):
            obs = observation((0.0, y, -2.0), target, index * 0.1)
            backwards = tuple(-value for value in obs.bearing_ned_unit)
            estimate = estimator.update(
                BearingObservation(
                    **{
                        **obs.__dict__,
                        "bearing_ned_unit": backwards,
                        "frame_index": index,
                    }
                )
            )
        self.assertFalse(estimate.valid)
        self.assertIn(
            estimate.reason,
            {"target_behind_camera", "range_out_of_bounds"},
        )

    def test_stale_pose_is_rejected(self):
        estimator = BearingTargetEstimator(estimator_config())
        estimate = estimator.update(
            observation((0, 0, 0), (10, 2, 0), 1.0, pose_age=0.8)
        )
        self.assertFalse(estimate.valid)
        self.assertEqual(estimate.reason, "stale_pose")

    def test_out_of_order_observation_is_rejected(self):
        estimator = BearingTargetEstimator(estimator_config())
        estimator.update(observation((0, 0, 0), (10, 2, 0), 1.0))
        estimate = estimator.update(observation((0, 0.1, 0), (10, 2, 0), 0.9))
        self.assertFalse(estimate.valid)
        self.assertEqual(estimate.reason, "out_of_order_observation")

    def test_moving_target_filter_outputs_velocity_after_bootstrap(self):
        estimator = BearingTargetEstimator(
            estimator_config(maximum_reprojection_error_px=8.0)
        )
        estimate = None
        for index in range(18):
            timestamp = index * 0.1
            target = (10.0 + 0.25 * timestamp, 3.0, -2.0)
            camera = (0.0, -0.9 + 0.12 * index, -2.0)
            estimate = estimator.update(
                observation(camera, target, timestamp, frame_index=index)
            )
        self.assertIsNotNone(estimate.position_ned_m)
        self.assertTrue(estimate.velocity_valid)
        self.assertTrue(all(math.isfinite(value) for value in estimate.velocity_ned_m_s))

    def test_initialized_filter_remains_valid_while_camera_pauses(self):
        target = (10.0, 3.0, -2.0)
        estimator = BearingTargetEstimator(
            estimator_config(
                minimum_observations=5,
                maximum_observations=6,
            )
        )
        estimate = None
        timestamp = 0.0
        for index, y in enumerate(np.linspace(-0.6, 0.6, 6)):
            estimate = estimator.update(
                observation(
                    (0.0, y, -2.0),
                    target,
                    timestamp,
                    frame_index=index,
                )
            )
            timestamp += 0.1
        self.assertTrue(estimate.valid, estimate.reason)

        for index in range(6, 14):
            estimate = estimator.update(
                observation(
                    (0.0, 0.6, -2.0),
                    target,
                    timestamp,
                    frame_index=index,
                )
            )
            timestamp += 0.1

        self.assertTrue(estimate.valid, estimate.reason)
        self.assertLess(estimate.baseline_m, 0.01)
        self.assertLess(estimate.intersection_angle_deg, 0.01)

    def test_long_frame_gap_reinitializes_from_valid_retained_geometry(self):
        target = (10.0, 3.0, -2.0)
        estimator = BearingTargetEstimator(estimator_config())
        estimate = None
        for index, y in enumerate(np.linspace(-0.6, 0.6, 7)):
            estimate = estimator.update(
                observation(
                    (0.0, y, -2.0),
                    target,
                    index * 0.1,
                    frame_index=index,
                )
            )
        self.assertTrue(estimate.valid, estimate.reason)

        estimate = estimator.update(
            observation(
                (0.0, 0.8, -2.0),
                target,
                1.8,
                frame_index=8,
            )
        )

        self.assertTrue(estimate.valid, estimate.reason)
        self.assertTrue(
            np.allclose(estimate.position_ned_m, target, atol=0.15)
        )


if __name__ == "__main__":
    unittest.main()
