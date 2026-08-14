import math
import unittest

from tracking_web import (
    apparent_size_error,
    braking_speed_limit_m_s,
    cv2,
    hold_visual_follow_radius,
    native_precenter_yaw_rate_deg_s,
    np,
    object_pixel_scale_px,
    safe_follow_band_error,
    soft_handover_range_m,
    visual_follow_center_error_deg,
)
from visual_follow_target import (
    CameraRayProjector,
    DEFAULT_VISUAL_RANGE_PROFILE,
    TargetStateFilter,
    VisualBBoxFilter,
    VisualBBoxRangeEstimator,
    ned_target_to_wgs84,
)


class ApparentSizeFollowTests(unittest.TestCase):
    def test_smaller_bbox_requests_forward_error(self):
        self.assertGreater(apparent_size_error(100.0, 80.0), 0.0)

    def test_larger_bbox_requests_backward_error(self):
        self.assertLess(apparent_size_error(100.0, 125.0), 0.0)

    def test_matching_bbox_holds_zero_error(self):
        self.assertAlmostEqual(apparent_size_error(100.0, 100.0), 0.0)

    @unittest.skipIf(cv2 is None or np is None, "OpenCV is unavailable")
    def test_object_pixel_scale_tracks_foreground_not_bbox_area(self):
        class Box:
            x = 0.0
            y = 0.0
            w = 100.0
            h = 100.0

        small = np.zeros((100, 100, 3), dtype=np.uint8)
        large = np.zeros((100, 100, 3), dtype=np.uint8)
        cv2.rectangle(small, (40, 40), (59, 59), (255, 255, 255), -1)
        cv2.rectangle(large, (30, 30), (69, 69), (255, 255, 255), -1)

        small_scale, small_source = object_pixel_scale_px(small, Box())
        large_scale, large_source = object_pixel_scale_px(large, Box())

        self.assertGreater(large_scale, small_scale)
        self.assertNotEqual(small_source, "bbox_fallback")
        self.assertNotEqual(large_source, "bbox_fallback")


class VisualRangeTests(unittest.TestCase):
    def test_safe_follow_band_holds_between_eight_and_twelve_metres(self):
        self.assertEqual(safe_follow_band_error(8.0, 8.0, 12.0), 0.0)
        self.assertEqual(safe_follow_band_error(10.0, 8.0, 12.0), 0.0)
        self.assertEqual(safe_follow_band_error(12.0, 8.0, 12.0), 0.0)

    def test_safe_follow_band_moves_toward_nearest_boundary(self):
        self.assertAlmostEqual(safe_follow_band_error(14.5, 8.0, 12.0), 2.5)
        self.assertAlmostEqual(safe_follow_band_error(6.5, 8.0, 12.0), -1.5)

    def test_braking_speed_reaches_zero_at_safe_boundary(self):
        self.assertEqual(braking_speed_limit_m_s(0.0, 1.5), 0.0)
        self.assertAlmostEqual(
            braking_speed_limit_m_s(3.0, 1.5),
            3.0,
        )

    def test_soft_handover_starts_with_zero_radial_error(self):
        distance, blend = soft_handover_range_m(10.0, 16.0, 0.0, 2.0)
        self.assertEqual(distance, 10.0)
        self.assertEqual(blend, 0.0)

    def test_soft_handover_ramps_to_measured_range(self):
        midpoint, blend = soft_handover_range_m(10.0, 16.0, 1.0, 2.0)
        complete, complete_blend = soft_handover_range_m(
            10.0, 16.0, 3.0, 2.0
        )
        self.assertEqual(midpoint, 13.0)
        self.assertEqual(blend, 0.5)
        self.assertEqual(complete, 16.0)
        self.assertEqual(complete_blend, 1.0)

    def test_native_precenter_yaws_when_gimbal_hits_limit(self):
        self.assertGreater(
            native_precenter_yaw_rate_deg_s(15.0, 20.0),
            20.0,
        )

    def test_native_precenter_stops_near_center(self):
        self.assertEqual(
            native_precenter_yaw_rate_deg_s(2.0, 1.0),
            0.0,
        )

    def test_center_error_is_zero_at_frame_center(self):
        self.assertAlmostEqual(
            visual_follow_center_error_deg(320, 180, 640, 360, 205.5, 205.5),
            0.0,
        )

    def test_center_error_increases_for_off_center_bbox(self):
        self.assertGreater(
            visual_follow_center_error_deg(400, 240, 640, 360, 205.5, 205.5),
            15.0,
        )

    def test_follow_radius_deadband_holds_radial_motion(self):
        target, velocity, held = hold_visual_follow_radius(
            (0.0, 0.0, 0.0),
            (9.5, 2.0, -1.0),
            (1.0, 0.5, 0.2),
            10.0,
            9.8,
            1.0,
        )
        self.assertTrue(held)
        self.assertAlmostEqual((target[0] ** 2 + target[1] ** 2) ** 0.5, 10.0)
        self.assertAlmostEqual(velocity[0] * target[0] + velocity[1] * target[1], 0.0, places=5)

    def test_follow_radius_deadband_releases_outside_band(self):
        target, velocity, held = hold_visual_follow_radius(
            (0.0, 0.0, 0.0),
            (12.0, 0.0, -1.0),
            (0.0, 0.0, 0.0),
            10.0,
            12.0,
            1.0,
        )
        self.assertFalse(held)
        self.assertEqual(target, (12.0, 0.0, -1.0))
        self.assertEqual(velocity, (0.0, 0.0, 0.0))

    def test_lut_hits_calibration_samples(self):
        for sample in DEFAULT_VISUAL_RANGE_PROFILE.samples:
            estimator = VisualBBoxRangeEstimator()
            result = None
            for index in range(5):
                result = estimator.update(
                    width_px=sample.width_px,
                    height_px=sample.height_px,
                    frame_width=640,
                    frame_height=360,
                    timestamp_s=index * 0.05,
                    tracking_score=1.0,
                )
            self.assertIsNotNone(result)
            self.assertAlmostEqual(result.distance_raw_m, sample.distance_m, places=6)
            self.assertTrue(result.ready)

    def test_resolution_normalization(self):
        estimator = VisualBBoxRangeEstimator()
        result = estimator.update(
            width_px=38,
            height_px=23,
            frame_width=320,
            frame_height=180,
            timestamp_s=0.0,
            tracking_score=1.0,
        )
        self.assertAlmostEqual(result.distance_raw_m, 10.0, places=6)

    def test_distant_nose_on_uav_bbox_is_accepted(self):
        estimator = VisualBBoxRangeEstimator()
        result = estimator.update(
            width_px=30,
            height_px=31,
            frame_width=640,
            frame_height=360,
            timestamp_s=0.0,
            tracking_score=0.97,
        )
        self.assertTrue(result.valid)
        self.assertNotEqual(result.reason, "aspect_ratio_outlier")

    def test_scale_jump_is_rejected(self):
        estimator = VisualBBoxRangeEstimator()
        estimator.update(
            width_px=76, height_px=46, frame_width=640, frame_height=360,
            timestamp_s=0.0, tracking_score=1.0,
        )
        result = estimator.update(
            width_px=150, height_px=90, frame_width=640, frame_height=360,
            timestamp_s=0.05, tracking_score=1.0,
        )
        self.assertFalse(result.valid)
        self.assertEqual(result.reason, "bbox_scale_jump")

    def test_expanding_bbox_produces_ttc_block(self):
        estimator = VisualBBoxRangeEstimator()
        estimator.update(
            width_px=70, height_px=42, frame_width=640, frame_height=360,
            timestamp_s=0.0, tracking_score=1.0,
        )
        result = estimator.update(
            width_px=82, height_px=49, frame_width=640, frame_height=360,
            timestamp_s=0.1, tracking_score=1.0,
        )
        self.assertIsNotNone(result.ttc_s)
        self.assertEqual(result.reason, "ttc_block")


class ProjectionTests(unittest.TestCase):
    def test_identity_camera_center_points_east_in_ned(self):
        projector = CameraRayProjector(205.5, 205.5)
        ray = projector.pixel_to_ned_ray(320, 180, 640, 360, (0, 0, 0, 1))
        self.assertAlmostEqual(ray[0], 0.0, places=7)
        self.assertAlmostEqual(ray[1], 1.0, places=7)
        self.assertAlmostEqual(ray[2], 0.0, places=7)

    def test_image_right_rotates_toward_sensor_negative_y(self):
        projector = CameraRayProjector(205.5, 205.5)
        ray = projector.pixel_to_ned_ray(420, 180, 640, 360, (0, 0, 0, 1))
        self.assertLess(ray[0], 0.0)
        self.assertGreater(ray[1], 0.0)

    def test_target_distance_is_preserved(self):
        projector = CameraRayProjector(205.5, 205.5)
        target = projector.target_ned(
            u=320, v=180, frame_width=640, frame_height=360,
            camera_quaternion_xyzw=(0, 0, 0, 1),
            camera_position_ned=(2, 3, -5), distance_m=10,
        )
        delta = (target[0] - 2, target[1] - 3, target[2] + 5)
        self.assertAlmostEqual(math.sqrt(sum(v * v for v in delta)), 10.0)


class TargetFilterTests(unittest.TestCase):
    def test_filter_rejects_large_jump(self):
        target_filter = TargetStateFilter(max_jump_m=2.0)
        self.assertTrue(target_filter.update((0, 0, 0), 0.0).valid)
        result = target_filter.update((10, 0, 0), 0.1)
        self.assertEqual(result.reason, "position_jump")
        self.assertEqual(result.position_ned, (0.0, 0.0, 0.0))

    def test_filter_reinitializes_after_long_retry_gap(self):
        target_filter = TargetStateFilter()
        self.assertTrue(target_filter.update((0, 0, 0), 0.0).valid)
        result = target_filter.update((2, 3, -4), 1.1)
        self.assertTrue(result.valid)
        self.assertEqual(result.reason, "reinitialized")
        self.assertEqual(result.position_ned, (2.0, 3.0, -4.0))
        self.assertEqual(result.velocity_ned, (0.0, 0.0, 0.0))

    def test_small_target_jitter_does_not_create_velocity(self):
        target_filter = TargetStateFilter()
        target_filter.update((10.0, 0.0, -5.0), 0.0)
        result = target_filter.update((10.05, 0.02, -5.0), 0.1)
        self.assertEqual(result.position_ned, (10.0, 0.0, -5.0))
        self.assertEqual(result.velocity_ned, (0.0, 0.0, 0.0))

    def test_target_position_rate_is_limited(self):
        target_filter = TargetStateFilter(
            max_jump_m=5.0,
            max_position_rate_m_s=2.0,
        )
        target_filter.update((0.0, 0.0, 0.0), 0.0)
        result = target_filter.update((4.0, 0.0, 0.0), 0.1)
        self.assertAlmostEqual(result.position_ned[0], 0.2, places=6)

    def test_ned_to_wgs84(self):
        lat, lon, alt = ned_target_to_wgs84(
            follower_lat_deg=0.0,
            follower_lon_deg=0.0,
            follower_alt_msl_m=100.0,
            follower_position_ned=(0, 0, -10),
            target_position_ned=(10, 10, -12),
        )
        self.assertAlmostEqual(lat, math.degrees(10 / 6_378_137.0))
        self.assertAlmostEqual(lon, math.degrees(10 / 6_378_137.0))
        self.assertEqual(alt, 102.0)


class BBoxFilterTests(unittest.TestCase):
    def test_requires_stable_frames_and_smooths_jitter(self):
        bbox_filter = VisualBBoxFilter(stable_frames_required=4)
        outputs = []
        for index, offset in enumerate((0.0, 2.0, -2.0, 1.0)):
            outputs.append(bbox_filter.update(
                cx=320.0 + offset,
                cy=180.0,
                width=76.0,
                height=46.0,
                timestamp_s=index * 0.05,
            ))
        self.assertTrue(outputs[-1].ready)
        self.assertLess(abs(outputs[-1].cx - 320.0), 1.0)

    def test_bbox_jump_is_held_then_rejected(self):
        bbox_filter = VisualBBoxFilter(hold_rejected_frames=2)
        bbox_filter.update(
            cx=100.0,
            cy=100.0,
            width=60.0,
            height=40.0,
            timestamp_s=0.0,
        )
        outputs = [
            bbox_filter.update(
                cx=300.0,
                cy=100.0,
                width=60.0,
                height=40.0,
                timestamp_s=timestamp,
            )
            for timestamp in (0.05, 0.10, 0.15)
        ]
        self.assertEqual(outputs[0].reason, "holding_outlier")
        self.assertEqual(outputs[1].reason, "holding_outlier")
        self.assertFalse(outputs[2].valid)


if __name__ == "__main__":
    unittest.main()
