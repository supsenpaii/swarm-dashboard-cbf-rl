import unittest

from body_yaw_recenter import (
    BodyYawRecenterConfig,
    BodyYawRecenterController,
)


def controller():
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


class BodyYawRecenterControllerTests(unittest.TestCase):
    def test_does_not_recenter_inside_deadband(self):
        output = controller().update(
            gimbal_yaw_deg=2.0,
            bbox_horizontal_error_deg=10.0,
            tracking_valid=True,
            dt_s=0.1,
        )
        self.assertFalse(output.active)
        self.assertEqual(output.limited_rate_deg_s, 0.0)

    def test_requires_enter_hold_time(self):
        subject = controller()
        first = subject.update(
            gimbal_yaw_deg=8.0,
            bbox_horizontal_error_deg=0.0,
            tracking_valid=True,
            dt_s=0.1,
        )
        second = subject.update(
            gimbal_yaw_deg=8.0,
            bbox_horizontal_error_deg=0.0,
            tracking_valid=True,
            dt_s=0.1,
        )
        third = subject.update(
            gimbal_yaw_deg=8.0,
            bbox_horizontal_error_deg=0.0,
            tracking_valid=True,
            dt_s=0.1,
        )
        self.assertFalse(first.active)
        self.assertFalse(second.active)
        self.assertTrue(third.active)

    def test_hysteresis_stays_active_between_exit_and_enter(self):
        subject = controller()
        for _ in range(3):
            output = subject.update(
                gimbal_yaw_deg=8.0,
                bbox_horizontal_error_deg=0.0,
                tracking_valid=True,
                dt_s=0.1,
            )
        self.assertTrue(output.active)
        output = subject.update(
            gimbal_yaw_deg=4.0,
            bbox_horizontal_error_deg=0.0,
            tracking_valid=True,
            dt_s=0.1,
        )
        self.assertTrue(output.active)
        for _ in range(3):
            output = subject.update(
                gimbal_yaw_deg=1.0,
                bbox_horizontal_error_deg=0.0,
                tracking_valid=True,
                dt_s=0.1,
            )
        self.assertFalse(output.active)

    def test_rate_is_slew_limited(self):
        subject = controller()
        for _ in range(3):
            output = subject.update(
                gimbal_yaw_deg=20.0,
                bbox_horizontal_error_deg=0.0,
                tracking_valid=True,
                dt_s=0.1,
            )
        self.assertAlmostEqual(output.limited_rate_deg_s, 3.0)
        output = subject.update(
            gimbal_yaw_deg=20.0,
            bbox_horizontal_error_deg=0.0,
            tracking_valid=True,
            dt_s=0.1,
        )
        self.assertAlmostEqual(output.limited_rate_deg_s, 6.0)

    def test_reversal_brakes_to_zero_first(self):
        subject = controller()
        for _ in range(6):
            positive = subject.update(
                gimbal_yaw_deg=20.0,
                bbox_horizontal_error_deg=0.0,
                tracking_valid=True,
                dt_s=0.1,
            )
        self.assertGreater(positive.limited_rate_deg_s, 0.0)
        outputs = []
        for _ in range(8):
            outputs.append(
                subject.update(
                    gimbal_yaw_deg=-20.0,
                    bbox_horizontal_error_deg=0.0,
                    tracking_valid=True,
                    dt_s=0.1,
                )
            )
        rates = [output.limited_rate_deg_s for output in outputs]
        first_negative = next(index for index, rate in enumerate(rates) if rate < 0.0)
        self.assertIn(0.0, rates[: first_negative + 1])

    def test_invalid_tracking_ramps_command_to_zero(self):
        subject = controller()
        for _ in range(5):
            active = subject.update(
                gimbal_yaw_deg=15.0,
                bbox_horizontal_error_deg=0.0,
                tracking_valid=True,
                dt_s=0.1,
            )
        self.assertGreater(active.limited_rate_deg_s, 0.0)
        blocked = subject.update(
            gimbal_yaw_deg=15.0,
            bbox_horizontal_error_deg=0.0,
            tracking_valid=False,
            dt_s=0.1,
        )
        self.assertFalse(blocked.active)
        self.assertLess(blocked.limited_rate_deg_s, active.limited_rate_deg_s)


if __name__ == "__main__":
    unittest.main()
