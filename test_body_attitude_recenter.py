from __future__ import annotations

import unittest
from dataclasses import replace

from body_attitude_recenter import (
    BodyAttitudeRecenterConfig,
    BodyAttitudeRecenterController,
)


class BodyAttitudeRecenterControllerTest(unittest.TestCase):
    def setUp(self) -> None:
        config = replace(
            BodyAttitudeRecenterConfig.from_environment(),
            enabled=True,
            rate_slew_deg_s2=1000.0,
            horizontal_accel_jerk_m_s3=1000.0,
        )
        self.controller = BodyAttitudeRecenterController(config)

    def update(self, **overrides):
        values = {
            "enabled": True,
            "gimbal_angles_deg": {"roll": 0.0, "pitch": 0.0, "yaw": 0.0},
            "body_attitude_deg": {"roll": 0.0, "pitch": 0.0},
            "altitude_m": 5.0,
            "vertical_velocity_down_m_s": 0.0,
            "dt": 0.05,
        }
        values.update(overrides)
        return self.controller.update(**values)

    def test_gimbal_error_drives_only_body_yaw(self) -> None:
        result = self.update(
            gimbal_angles_deg={"roll": 10.0, "pitch": -10.0, "yaw": 20.0}
        )
        rates = result["body_rates_deg_s"]
        self.assertEqual(rates["roll"], 0.0)
        self.assertEqual(rates["pitch"], 0.0)
        self.assertGreater(rates["yaw"], 0.0)

    def test_deadband_produces_zero_rates_when_level(self) -> None:
        result = self.update(
            gimbal_angles_deg={"roll": 1.0, "pitch": -1.0, "yaw": 1.0}
        )
        self.assertEqual(
            result["body_rates_deg_s"],
            {"roll": 0.0, "pitch": 0.0, "yaw": 0.0},
        )

    def test_hysteresis_does_not_start_between_exit_and_enter(self) -> None:
        result = self.update(
            gimbal_angles_deg={"roll": 2.0, "pitch": -2.0, "yaw": 2.0}
        )
        self.assertEqual(
            result["recenter_active_axes"],
            {"roll": False, "pitch": False, "yaw": False},
        )

    def test_hysteresis_stays_active_until_exit_threshold(self) -> None:
        self.update(
            gimbal_angles_deg={"roll": 0.0, "pitch": 0.0, "yaw": 10.0}
        )
        result = None
        for _ in range(30):
            result = self.update(
                gimbal_angles_deg={"roll": 0.0, "pitch": 0.0, "yaw": 4.0}
            )
        self.assertIsNotNone(result)
        self.assertTrue(result["recenter_active_axes"]["yaw"])

        for _ in range(30):
            result = self.update(
                gimbal_angles_deg={"roll": 0.0, "pitch": 0.0, "yaw": 0.0}
            )
        self.assertFalse(result["recenter_active_axes"]["yaw"])

    def test_low_pass_rejects_single_frame_gimbal_jump(self) -> None:
        self.update()
        result = self.update(
            gimbal_angles_deg={"roll": 20.0, "pitch": 0.0, "yaw": 0.0}
        )
        self.assertLess(result["gimbal_filtered_deg"]["roll"], 5.0)
        self.assertFalse(result["recenter_active_axes"]["roll"])

    def test_bbox_error_does_not_directly_drive_body_yaw(self) -> None:
        result = self.update(
            bbox_image_error_deg={"yaw": 10.0, "pitch": -8.0}
        )
        rates = result["body_rates_deg_s"]
        acceleration = result["body_acceleration_m_s2"]
        self.assertEqual(rates["roll"], 0.0)
        self.assertEqual(rates["pitch"], 0.0)
        self.assertEqual(rates["yaw"], 0.0)
        self.assertEqual(acceleration, {"forward": 0.0, "right": 0.0})

    def test_bbox_deadband_does_not_move_body(self) -> None:
        result = self.update(
            bbox_image_error_deg={"yaw": 0.2, "pitch": -0.2}
        )
        self.assertEqual(
            result["body_rates_deg_s"],
            {"roll": 0.0, "pitch": 0.0, "yaw": 0.0},
        )

    def test_large_gimbal_yaw_uses_fast_body_takeover(self) -> None:
        result = self.update(
            gimbal_angles_deg={"roll": 0.0, "pitch": 0.0, "yaw": 6.0}
        )
        self.assertGreaterEqual(
            result["body_rates_deg_s"]["yaw"],
            self.controller.config.yaw_fast_min_rate_deg_s * 0.8,
        )

    def test_existing_body_tilt_does_not_add_roll_or_pitch_tracking_command(self) -> None:
        result = self.update(
            gimbal_angles_deg={"roll": 30.0, "pitch": 30.0, "yaw": 0.0},
            body_attitude_deg={"roll": 6.0, "pitch": 6.0},
        )
        rates = result["body_rates_deg_s"]
        self.assertEqual(rates["roll"], 0.0)
        self.assertEqual(rates["pitch"], 0.0)

    def test_descent_and_tilt_increase_thrust(self) -> None:
        hover = self.update()["thrust"]
        result = self.update(
            body_attitude_deg={"roll": 5.0, "pitch": 5.0},
            altitude_m=4.8,
            vertical_velocity_down_m_s=0.3,
        )
        self.assertGreater(result["thrust"], hover)

    def test_disable_resets_altitude_reference(self) -> None:
        self.update()
        self.update(enabled=False)
        self.assertIsNone(self.controller.altitude_target_m)

    def test_follow_command_does_not_add_nose_down_rate(self) -> None:
        result = self.update(normalized_forward_command=1.0)
        self.assertEqual(result["body_rates_deg_s"]["pitch"], 0.0)

    def test_flight_output_uses_zero_horizontal_acceleration_with_absolute_z(self) -> None:
        result = self.update(
            gimbal_angles_deg={"roll": 10.0, "pitch": -10.0, "yaw": 20.0}
        )
        acceleration = result["body_acceleration_m_s2"]
        self.assertEqual(result["vertical_control"], "px4_absolute_z")
        self.assertEqual(acceleration, {"forward": 0.0, "right": 0.0})
        self.assertEqual(result["altitude_target_m"], 5.0)

    def test_follow_command_does_not_create_horizontal_acceleration(self) -> None:
        result = self.update(normalized_forward_command=1.0)
        acceleration = result["body_acceleration_m_s2"]
        self.assertEqual(acceleration, {"forward": 0.0, "right": 0.0})

    def test_adaptive_yaw_gain_increases_for_large_error(self) -> None:
        near = self.update(
            gimbal_angles_deg={"roll": 0.0, "pitch": 0.0, "yaw": 4.0}
        )
        far = self.update(
            gimbal_angles_deg={"roll": 0.0, "pitch": 0.0, "yaw": 15.0}
        )
        for _ in range(10):
            far = self.update(
                gimbal_angles_deg={"roll": 0.0, "pitch": 0.0, "yaw": 15.0}
            )
        self.assertGreater(far["adaptive_yaw_kp"], near["adaptive_yaw_kp"])

    def test_altitude_guard_pauses_acceleration_and_then_exits(self) -> None:
        self.update()
        paused = self.update(
            gimbal_angles_deg={"roll": 20.0, "pitch": 20.0, "yaw": 0.0},
            altitude_m=(
                5.0 - self.controller.config.altitude_pause_error_m - 0.01
            ),
        )
        self.assertTrue(paused["active"])
        self.assertTrue(paused["altitude_guard_active"])
        exited = self.update(
            altitude_m=(
                5.0 - self.controller.config.altitude_exit_error_m - 0.01
            )
        )
        self.assertFalse(exited["active"])
        self.assertEqual(exited["state"], "altitude_exit")

    def test_horizontal_speed_does_not_create_tracking_acceleration(self) -> None:
        result = self.update(
            gimbal_angles_deg={"roll": 0.0, "pitch": -20.0, "yaw": 0.0},
            horizontal_velocity_body_m_s={"forward": 0.8, "right": 0.0},
        )
        self.assertEqual(
            result["body_acceleration_m_s2"],
            {"forward": 0.0, "right": 0.0},
        )


if __name__ == "__main__":
    unittest.main()
