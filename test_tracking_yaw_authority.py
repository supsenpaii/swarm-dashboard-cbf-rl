import math
import time
import unittest
from unittest.mock import patch

import main
from body_yaw_recenter import BodyYawRecenterConfig, BodyYawRecenterController


class GimbalInnerLoopTests(unittest.TestCase):
    def setUp(self):
        main.tracking_gimbal_rates_deg_s["UAV-02"] = {
            "roll": 0.0,
            "pitch": 0.0,
            "yaw": 0.0,
        }
        main.tracking_gimbal_last_command_monotonic["UAV-02"] = 0.0

    def test_rate_limiter_caps_first_step_and_maximum(self):
        first = main.limit_tracking_gimbal_rate_deg_s(
            256.0,
            0.0,
            maximum_rate_deg_s=30.0,
            maximum_acceleration_deg_s2=90.0,
            dt_s=0.1,
        )
        self.assertAlmostEqual(first, 9.0)
        rate = first
        for _ in range(10):
            rate = main.limit_tracking_gimbal_rate_deg_s(
                256.0,
                rate,
                maximum_rate_deg_s=30.0,
                maximum_acceleration_deg_s2=90.0,
                dt_s=0.1,
            )
        self.assertEqual(rate, 30.0)

    def test_gimbal_reversal_brakes_to_zero(self):
        first = main.limit_tracking_gimbal_rate_deg_s(
            -30.0,
            12.0,
            maximum_rate_deg_s=15.0,
            maximum_acceleration_deg_s2=60.0,
            maximum_braking_acceleration_deg_s2=180.0,
            dt_s=0.1,
        )
        second = main.limit_tracking_gimbal_rate_deg_s(
            -30.0,
            first,
            maximum_rate_deg_s=15.0,
            maximum_acceleration_deg_s2=60.0,
            maximum_braking_acceleration_deg_s2=180.0,
            dt_s=0.1,
        )
        self.assertEqual(first, 0.0)
        self.assertLess(second, 0.0)

    def test_non_finite_rate_is_neutralized(self):
        self.assertEqual(
            main.limit_tracking_gimbal_rate_deg_s(
                math.nan,
                math.nan,
                maximum_rate_deg_s=30.0,
                maximum_acceleration_deg_s2=90.0,
                dt_s=0.1,
            ),
            0.0,
        )

    def test_gimbal_yaw_is_clamped(self):
        self.assertEqual(
            main.clamp_tracking_gimbal_yaw_deg(14.0, 10.0),
            (10.0, True),
        )
        self.assertEqual(
            main.clamp_tracking_gimbal_yaw_deg(-14.0, 10.0),
            (-10.0, True),
        )

    def test_tracking_pitch_is_clamped_away_from_singularity(self):
        self.assertEqual(
            main.clamp_tracking_gimbal_pitch_deg(-100.0),
            (-45.0, True),
        )
        self.assertEqual(
            main.clamp_tracking_gimbal_pitch_deg(30.0),
            (30.0, False),
        )

    def test_command_never_publishes_outside_limit(self):
        published = {}

        def publish(_drone_id, axis, target):
            published[axis] = target
            return True, ""

        with (
            patch.object(
                main,
                "synced_gimbal_angles_deg",
                return_value={"roll": 0.0, "pitch": 0.0, "yaw": 9.8},
            ),
            patch.object(main.gazebo_bridge, "publish_gimbal", publish),
        ):
            result = main.command_tracking_gimbal(
                "UAV-02",
                math.radians(120.0),
                0.0,
                0.1,
            )
        self.assertTrue(result["yaw_saturated_outward"])
        self.assertLessEqual(abs(published["yaw"]), 10.0)

    def test_command_target_integrates_while_feedback_lags(self):
        published_yaw = []

        def publish(_drone_id, axis, target):
            if axis == "yaw":
                published_yaw.append(target)
            return True, ""

        original_angles = dict(main.gimbal_angles_deg["UAV-02"])
        original_rates = dict(main.tracking_gimbal_rates_deg_s["UAV-02"])
        original_time = main.tracking_gimbal_last_command_monotonic["UAV-02"]
        try:
            main.gimbal_angles_deg["UAV-02"] = {
                "roll": 0.0,
                "pitch": 0.0,
                "yaw": 0.0,
            }
            main.tracking_gimbal_rates_deg_s["UAV-02"] = {
                "roll": 0.0,
                "pitch": 0.0,
                "yaw": 0.0,
            }
            main.tracking_gimbal_last_command_monotonic["UAV-02"] = (
                time.monotonic()
            )
            with (
                patch.object(
                    main,
                    "synced_gimbal_angles_deg",
                    return_value={"roll": 0.0, "pitch": 0.0, "yaw": 0.0},
                ),
                patch.object(main.gazebo_bridge, "publish_gimbal", publish),
            ):
                main.command_tracking_gimbal(
                    "UAV-02",
                    math.radians(10.0),
                    0.0,
                    0.1,
                )
                main.command_tracking_gimbal(
                    "UAV-02",
                    math.radians(10.0),
                    0.0,
                    0.1,
                )
            self.assertGreater(published_yaw[1], published_yaw[0])
        finally:
            main.gimbal_angles_deg["UAV-02"] = original_angles
            main.tracking_gimbal_rates_deg_s["UAV-02"] = original_rates
            main.tracking_gimbal_last_command_monotonic["UAV-02"] = original_time


class SingleBodyYawAuthorityTests(unittest.TestCase):
    def controller(self):
        return BodyYawRecenterController(
            BodyYawRecenterConfig(
                enter_deg=6.0,
                exit_deg=2.5,
                enter_hold_s=0.25,
                filter_time_constant_s=0.01,
                proportional_gain=1.5,
                bbox_feedforward_gain=0.0,
                maximum_rate_deg_s=10.0,
                slew_rate_deg_s2=30.0,
            )
        )

    def test_bbox_does_not_drive_body_when_feedforward_is_zero(self):
        controller = self.controller()
        output = controller.update(
            gimbal_yaw_deg=0.0,
            bbox_horizontal_error_deg=40.0,
            tracking_valid=True,
            gimbal_fresh=True,
            dt_s=0.1,
        )
        self.assertEqual(output.limited_rate_deg_s, 0.0)
        self.assertFalse(output.active)

    def test_outer_loop_is_slower_than_gimbal_limit(self):
        controller = self.controller()
        output = None
        for _ in range(5):
            output = controller.update(
                gimbal_yaw_deg=12.0,
                bbox_horizontal_error_deg=0.0,
                tracking_valid=True,
                gimbal_fresh=True,
                dt_s=0.1,
            )
        self.assertTrue(output.active)
        self.assertLessEqual(abs(output.limited_rate_deg_s), 10.0)

    def test_stale_gimbal_neutralizes_body_yaw(self):
        controller = self.controller()
        for _ in range(5):
            controller.update(
                gimbal_yaw_deg=12.0,
                bbox_horizontal_error_deg=0.0,
                tracking_valid=True,
                gimbal_fresh=True,
                dt_s=0.1,
            )
        for _ in range(3):
            output = controller.update(
                gimbal_yaw_deg=12.0,
                bbox_horizontal_error_deg=0.0,
                tracking_valid=True,
                gimbal_fresh=False,
                dt_s=0.2,
            )
        self.assertEqual(output.limited_rate_deg_s, 0.0)
        self.assertEqual(output.reason, "gimbal_stale")


if __name__ == "__main__":
    unittest.main()
