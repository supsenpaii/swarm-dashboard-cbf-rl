import unittest
from unittest.mock import patch

import main


class TrackingMotionAltitudeHoldTest(unittest.TestCase):
    def setUp(self):
        main.tracking_motion_hold_z_down_m["UAV-01"] = None
        main.latest_drones["UAV-01"] = {
            "local_position": {
                "z_down_m": -5.0,
                "heading_rad": 0.0,
            },
            "status": {
                "armed": True,
                "failsafe": False,
            },
        }
        self.payloads = []

    def publish(self, payload):
        self.payloads.append(payload)
        return True, ""

    def command(self, forward, distance, state="tracking"):
        return main.command_tracking_motion(
            "UAV-01",
            True,
            forward,
            0.0,
            0.0,
            distance,
            9.0,
            "bearing_triangulation",
            state,
        )

    def test_altitude_target_does_not_change_with_depth(self):
        with (
            patch.object(main, "TRACKING_OFFBOARD_ENABLED", True),
            patch.object(main, "publish_control_message", self.publish),
            patch.object(
                main,
                "tracking_body_yaw_safety_error",
                return_value="",
            ),
        ):
            first = self.command(0.5, 10.1)
            main.latest_drones["UAV-01"]["local_position"]["z_down_m"] = -7.0
            second = self.command(-0.5, 7.9)

        self.assertEqual(first["hold_z_down_m"], -5.0)
        self.assertEqual(second["hold_z_down_m"], -5.0)
        self.assertEqual(self.payloads[0]["hold_z_down_m"], -5.0)
        self.assertEqual(self.payloads[1]["hold_z_down_m"], -5.0)
        self.assertEqual(self.payloads[0]["down_velocity_m_s"], 0.0)
        self.assertEqual(self.payloads[1]["down_velocity_m_s"], 0.0)

    def test_stale_depth_neutral_command_keeps_altitude_hold(self):
        with (
            patch.object(main, "TRACKING_OFFBOARD_ENABLED", True),
            patch.object(main, "publish_control_message", self.publish),
            patch.object(
                main,
                "tracking_body_yaw_safety_error",
                return_value="",
            ),
        ):
            self.command(0.5, 10.1)
            neutral = self.command(0.0, 10.1, "stale_depth")

        self.assertTrue(neutral["enabled"])
        self.assertEqual(neutral["forward_velocity_m_s"], 0.0)
        self.assertEqual(neutral["hold_z_down_m"], -5.0)
        self.assertEqual(self.payloads[-1]["north_velocity_m_s"], 0.0)
        self.assertEqual(self.payloads[-1]["east_velocity_m_s"], 0.0)


class VisualFollowHandoverSafetyTest(unittest.TestCase):
    def test_native_target_prestream_is_allowed_from_offboard(self):
        drone_id = "UAV-02"
        telemetry = {
            "online": True,
            "status": {
                "armed": True,
                "failsafe": False,
                "nav_state": 14,
            },
            "failsafe_flags": {
                "local_position_invalid": False,
                "global_position_invalid": False,
            },
        }
        pose = {
            "available": True,
            "local_position": {"z_down_m": -10.0},
        }
        with (
            patch.dict(main.latest_drones, {drone_id: telemetry}),
            patch.dict(main.manual_control_deadline, {drone_id: 0.0}),
            patch.object(
                main,
                "tracking_visual_pose",
                return_value=pose,
            ),
        ):
            self.assertEqual(
                main.tracking_visual_follow_safety_error(drone_id),
                "",
            )

    def test_manual_loss_has_bounded_grace_during_native_handover(self):
        drone_id = "UAV-02"
        telemetry = {
            "online": True,
            "status": {
                "armed": True,
                "failsafe": True,
                "nav_state": 19,
            },
            "failsafe_flags": {
                "manual_control_signal_lost": True,
                "offboard_control_signal_lost": True,
                "local_position_invalid": False,
                "global_position_invalid": False,
            },
        }
        bridge_status = {
            "enabled": True,
            "valid": True,
            "timeout_state": "FRESH",
            "prestream_count": 10,
            "prestream_required": 10,
        }
        pose = {
            "available": True,
            "local_position": {"z_down_m": -10.0},
        }
        with (
            patch.dict(main.latest_drones, {drone_id: telemetry}),
            patch.dict(main.manual_control_deadline, {drone_id: 0.0}),
            patch.dict(
                main.tracking_visual_follow_bridge_status,
                {drone_id: bridge_status},
            ),
            patch.dict(
                main.tracking_visual_follow_manual_loss_started_monotonic,
                {drone_id: None},
            ),
            patch.object(
                main,
                "tracking_visual_pose",
                return_value=pose,
            ),
            patch.object(main.time, "monotonic", return_value=100.0),
        ):
            self.assertEqual(
                main.tracking_visual_follow_safety_error(drone_id),
                "",
            )
            main.tracking_visual_follow_manual_loss_started_monotonic[
                drone_id
            ] = 99.0
            self.assertIn(
                "persisted beyond handover grace",
                main.tracking_visual_follow_safety_error(drone_id),
            )

    def test_manual_loss_grace_never_masks_another_failsafe(self):
        drone_id = "UAV-02"
        telemetry = {
            "online": True,
            "status": {
                "armed": True,
                "failsafe": True,
                "nav_state": 19,
            },
            "failsafe_flags": {
                "manual_control_signal_lost": True,
                "battery_warning": True,
                "local_position_invalid": False,
                "global_position_invalid": False,
            },
        }
        bridge_status = {
            "enabled": True,
            "valid": True,
            "timeout_state": "FRESH",
            "prestream_count": 10,
            "prestream_required": 10,
        }
        with (
            patch.dict(main.latest_drones, {drone_id: telemetry}),
            patch.dict(main.manual_control_deadline, {drone_id: 0.0}),
            patch.dict(
                main.tracking_visual_follow_bridge_status,
                {drone_id: bridge_status},
            ),
        ):
            self.assertEqual(
                main.tracking_visual_follow_safety_error(drone_id),
                "Visual Follow blocked: PX4 failsafe is active",
            )


if __name__ == "__main__":
    unittest.main()
