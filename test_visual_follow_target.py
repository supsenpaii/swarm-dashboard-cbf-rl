import math
import unittest

from tracking_web import (
    apparent_size_error,
    braking_speed_limit_m_s,
    cv2,
    metric_follow_command_target,
    np,
    object_pixel_scale_px,
    safe_follow_band_error,
    slew_toward,
    visual_follow_center_error_deg,
)
from visual_follow_target import (
    BBoxMotionSafetyEstimator,
    CameraRayProjector,
    TargetStateFilter,
    VisualBBoxFilter,
    ned_target_to_wgs84,
    selection_anchored_target_ned,
)


class ApparentSizeSafetyTests(unittest.TestCase):
    def test_size_error_sign(self):
        self.assertGreater(apparent_size_error(100.0, 80.0), 0.0)
        self.assertLess(apparent_size_error(100.0, 125.0), 0.0)
        self.assertEqual(apparent_size_error(100.0, 100.0), 0.0)

    @unittest.skipIf(cv2 is None or np is None, "OpenCV is unavailable")
    def test_object_scale_uses_foreground(self):
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

    def test_apparent_size_estimator_provides_ttc_only_not_metric_authority(self):
        estimator = BBoxMotionSafetyEstimator()
        estimator.update(
            width_px=70,
            height_px=42,
            frame_width=640,
            frame_height=360,
            timestamp_s=0.0,
            tracking_score=1.0,
        )
        result = estimator.update(
            width_px=82,
            height_px=49,
            frame_width=640,
            frame_height=360,
            timestamp_s=0.1,
            tracking_score=1.0,
        )
        self.assertIsNotNone(result.ttc_s)
        self.assertEqual(result.reason, "ttc_block")


class SafeFollowBandTests(unittest.TestCase):
    def command(self, distance_m):
        return metric_follow_command_target(
            distance_m,
            8.0,
            12.0,
            proportional_gain=0.12,
            maximum_command=0.30,
            maximum_velocity_m_s=3.0,
            brake_deceleration_m_s2=1.5,
        )

    def test_band_error_and_command_direction(self):
        self.assertLess(safe_follow_band_error(7.9, 8.0, 12.0), 0.0)
        self.assertEqual(safe_follow_band_error(10.0, 8.0, 12.0), 0.0)
        self.assertGreater(safe_follow_band_error(12.1, 8.0, 12.0), 0.0)
        self.assertLess(self.command(7.9)[1], 0.0)
        self.assertEqual(self.command(10.0)[1], 0.0)
        self.assertGreater(self.command(12.1)[1], 0.0)

    def test_slew_and_braking_limit(self):
        command = slew_toward(0.2, 0.0, 0.15, 0.1)
        self.assertGreaterEqual(command, 0.0)
        self.assertLess(command, 0.2)
        self.assertEqual(braking_speed_limit_m_s(0.0, 1.5), 0.0)
        self.assertAlmostEqual(braking_speed_limit_m_s(3.0, 1.5), 3.0)


class ProjectionTests(unittest.TestCase):
    def test_identity_camera_axes(self):
        projector = CameraRayProjector(205.5, 205.5)
        center = projector.pixel_to_ned_ray(
            320, 180, 640, 360, (0, 0, 0, 1)
        )
        right = projector.pixel_to_ned_ray(
            420, 180, 640, 360, (0, 0, 0, 1)
        )
        up = projector.pixel_to_ned_ray(
            320, 80, 640, 360, (0, 0, 0, 1)
        )
        down = projector.pixel_to_ned_ray(
            320, 280, 640, 360, (0, 0, 0, 1)
        )
        self.assertAlmostEqual(center[0], 0.0)
        self.assertAlmostEqual(center[1], 1.0)
        self.assertAlmostEqual(center[2], 0.0)
        self.assertLess(right[0], 0.0)
        self.assertLess(up[2], 0.0)
        self.assertGreater(down[2], 0.0)

    def test_target_distance_is_preserved(self):
        projector = CameraRayProjector(205.5, 205.5)
        target = projector.target_ned(
            u=320,
            v=180,
            frame_width=640,
            frame_height=360,
            camera_quaternion_xyzw=(0, 0, 0, 1),
            camera_position_ned=(2, 3, -5),
            distance_m=10,
        )
        delta = (target[0] - 2, target[1] - 3, target[2] + 5)
        self.assertAlmostEqual(math.sqrt(sum(value * value for value in delta)), 10)

    def test_selection_anchor_locks_exact_horizontal_safe_distance(self):
        target, slant = selection_anchored_target_ned(
            camera_position_ned=(0.20, -0.10, -8.20),
            vehicle_position_ned=(0.0, 0.0, -8.0),
            bearing_ned=(0.8, 0.6, 0.1),
            safe_horizontal_distance_m=10.0,
        )

        self.assertGreater(slant, 0.0)
        self.assertAlmostEqual(math.hypot(target[0], target[1]), 10.0)
        ray_delta = (
            target[0] - 0.20,
            target[1] + 0.10,
            target[2] + 8.20,
        )
        cross = (
            ray_delta[1] * 0.1 - ray_delta[2] * 0.6,
            ray_delta[2] * 0.8 - ray_delta[0] * 0.1,
            ray_delta[0] * 0.6 - ray_delta[1] * 0.8,
        )
        self.assertLess(math.sqrt(sum(value * value for value in cross)), 1e-8)

    def test_selection_anchor_rejects_near_vertical_ray(self):
        with self.assertRaisesRegex(ValueError, "horizontal component"):
            selection_anchored_target_ned(
                camera_position_ned=(0.0, 0.0, -8.0),
                vehicle_position_ned=(0.0, 0.0, -8.0),
                bearing_ned=(0.0, 0.0, 1.0),
                safe_horizontal_distance_m=10.0,
            )

    def test_center_error(self):
        self.assertAlmostEqual(
            visual_follow_center_error_deg(320, 180, 640, 360, 205.5, 205.5),
            0.0,
        )
        self.assertGreater(
            visual_follow_center_error_deg(400, 240, 640, 360, 205.5, 205.5),
            15.0,
        )


class TargetFilterAndConversionTests(unittest.TestCase):
    def test_filter_rejects_large_jump(self):
        target_filter = TargetStateFilter(max_jump_m=2.0)
        self.assertTrue(target_filter.update((0, 0, 0), 0.0).valid)
        result = target_filter.update((10, 0, 0), 0.1)
        self.assertEqual(result.reason, "position_jump")
        self.assertEqual(result.position_ned, (0.0, 0.0, 0.0))

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
        outputs = [
            bbox_filter.update(
                cx=320.0 + offset,
                cy=180.0,
                width=76.0,
                height=46.0,
                timestamp_s=index * 0.05,
            )
            for index, offset in enumerate((0.0, 2.0, -2.0, 1.0))
        ]
        self.assertTrue(outputs[-1].ready)
        self.assertLess(abs(outputs[-1].cx - 320.0), 1.0)

    def test_accepted_tracker_corrections_can_reach_ready(self):
        """Periodic KCF corrections inside the jump gate must not deadlock pointing."""
        bbox_filter = VisualBBoxFilter(stable_frames_required=4)
        outputs = [
            bbox_filter.update(
                cx=320.0 + offset,
                cy=180.0,
                width=76.0,
                height=46.0,
                timestamp_s=index * 0.025,
            )
            for index, offset in enumerate((0.0, 14.0, 1.0, 15.0))
        ]
        self.assertTrue(outputs[-1].ready)
        self.assertTrue(all(output.accepted for output in outputs))

    def test_bbox_jump_is_eventually_rejected(self):
        bbox_filter = VisualBBoxFilter(hold_rejected_frames=2)
        bbox_filter.update(
            cx=100,
            cy=100,
            width=60,
            height=40,
            timestamp_s=0.0,
        )
        outputs = [
            bbox_filter.update(
                cx=300,
                cy=100,
                width=60,
                height=40,
                timestamp_s=timestamp,
            )
            for timestamp in (0.05, 0.10, 0.15)
        ]
        self.assertEqual(outputs[0].reason, "holding_outlier")
        self.assertFalse(outputs[-1].valid)


if __name__ == "__main__":
    unittest.main()
