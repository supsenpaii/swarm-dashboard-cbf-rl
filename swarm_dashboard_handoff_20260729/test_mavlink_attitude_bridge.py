from __future__ import annotations

import json
import time
import unittest

from mavlink_manual_bridge import (
    OFFBOARD_ATTITUDE_TIMEOUT_S,
    OFFBOARD_LOCAL_ALTITUDE_HOLD_TYPE_MASK,
    OFFBOARD_LOCAL_ACCEL_ALTITUDE_HOLD_TYPE_MASK,
    MavlinkWorker,
    OffboardFollowState,
    OffboardAttitudeState,
    VISUAL_FOLLOW_TIMEOUT_S,
    VisualFollowTargetState,
    offboard_follow_states,
    on_mqtt_message,
    visual_follow_target_states,
)


class OffboardAttitudeStateTest(unittest.TestCase):
    def test_active_message_is_clamped(self) -> None:
        state = OffboardAttitudeState()
        state.update(
            {
                "enabled": True,
                "roll_rate_deg_s": 99.0,
                "pitch_rate_deg_s": -99.0,
                "yaw_rate_deg_s": 99.0,
                "thrust": 0.5,
            }
        )
        roll, pitch, yaw, thrust, active, releasing = state.output()
        self.assertTrue(active)
        self.assertFalse(releasing)
        self.assertEqual((roll, pitch, yaw), (10.0, -10.0, 30.0))
        self.assertEqual(thrust, 0.5)

    def test_disable_streams_neutral_release_before_stopping(self) -> None:
        state = OffboardAttitudeState()
        state.update({"enabled": True, "thrust": 0.52})
        state.update({"enabled": False, "thrust": 0.0})
        roll, pitch, yaw, thrust, active, releasing = state.output()
        self.assertTrue(active)
        self.assertTrue(releasing)
        self.assertEqual((roll, pitch, yaw), (0.0, 0.0, 0.0))
        self.assertEqual(thrust, 0.52)
        state.finish_release()
        self.assertFalse(state.output()[4])

    def test_watchdog_enters_release_instead_of_dropping_setpoint(self) -> None:
        state = OffboardAttitudeState()
        state.update({"enabled": True, "thrust": 0.5})
        state.last_command_time = time.monotonic() - (
            OFFBOARD_ATTITUDE_TIMEOUT_S + 0.1
        )
        output = state.output()
        self.assertTrue(output[4])
        self.assertTrue(output[5])
        self.assertEqual(output[:3], (0.0, 0.0, 0.0))


class OffboardAltitudeHoldStateTest(unittest.TestCase):
    def test_local_ned_altitude_hold_survives_state_output(self) -> None:
        state = OffboardFollowState()
        state.update(
            {
                "enabled": True,
                "velocity_frame": "local_ned_altitude_hold",
                "north_velocity_m_s": 0.2,
                "east_velocity_m_s": -0.1,
                "hold_z_down_m": -3.25,
                "yaw_rate_deg_s": 4.0,
            }
        )
        output = state.output()
        self.assertTrue(output[4])
        self.assertEqual(output[5:8], (0.2, -0.1, -3.25))
        self.assertTrue(output[8])

    def test_disable_removes_absolute_altitude_hold(self) -> None:
        state = OffboardFollowState()
        state.update(
            {
                "enabled": True,
                "velocity_frame": "local_ned_altitude_hold",
                "hold_z_down_m": -3.0,
            }
        )
        state.disable()
        self.assertFalse(state.output()[4])
        self.assertFalse(state.local_altitude_hold)

    def test_acceleration_altitude_hold_survives_state_output(self) -> None:
        state = OffboardFollowState()
        state.update(
            {
                "enabled": True,
                "velocity_frame": "local_ned_acceleration_altitude_hold",
                "north_acceleration_m_s2": 0.3,
                "east_acceleration_m_s2": -0.2,
                "hold_z_down_m": -3.0,
            }
        )
        output = state.output()
        self.assertTrue(output[4])
        self.assertTrue(output[8])
        self.assertEqual(output[9:11], (0.3, -0.2))
        self.assertTrue(output[11])

    def test_mavlink_packet_uses_absolute_z_and_horizontal_velocity(self) -> None:
        calls = []

        class FakeMav:
            def set_position_target_local_ned_send(self, *args):
                calls.append(args)

        class FakeConnection:
            mav = FakeMav()

        class FakeWorker:
            connection = FakeConnection()
            expected_system_id = 2

        MavlinkWorker.send_offboard_setpoint(
            FakeWorker(),
            0.0,
            0.0,
            0.0,
            5.0,
            0.2,
            -0.1,
            -3.25,
            True,
        )
        packet = calls[0]
        self.assertEqual(packet[4], OFFBOARD_LOCAL_ALTITUDE_HOLD_TYPE_MASK)
        self.assertEqual(packet[7], -3.25)
        self.assertEqual(packet[8:11], (0.2, -0.1, 0.0))

    def test_mavlink_packet_uses_absolute_z_and_horizontal_acceleration(self) -> None:
        calls = []

        class FakeMav:
            def set_position_target_local_ned_send(self, *args):
                calls.append(args)

        class FakeConnection:
            mav = FakeMav()

        class FakeWorker:
            connection = FakeConnection()
            expected_system_id = 2

        MavlinkWorker.send_offboard_setpoint(
            FakeWorker(),
            0.0,
            0.0,
            0.0,
            8.0,
            0.0,
            0.0,
            -3.0,
            True,
            0.3,
            -0.2,
            True,
        )
        packet = calls[0]
        self.assertEqual(
            packet[4], OFFBOARD_LOCAL_ACCEL_ALTITUDE_HOLD_TYPE_MASK
        )
        self.assertEqual(packet[7], -3.0)
        self.assertEqual(packet[8:11], (0.0, 0.0, 0.0))
        self.assertEqual(packet[11:14], (0.3, -0.2, 0.0))


class VisualFollowTargetStateTest(unittest.TestCase):
    def test_valid_target_is_active_and_watchdog_expires(self) -> None:
        state = VisualFollowTargetState()
        state.update(
            {
                "enabled": True,
                "latitude_deg": 21.0,
                "longitude_deg": 105.0,
                "altitude_msl_m": 25.0,
                "velocity_north_m_s": 1.0,
                "velocity_east_m_s": -0.5,
                "velocity_down_m_s": 0.1,
                "quality": 0.8,
            }
        )
        output = state.output()
        self.assertTrue(output[-1])
        self.assertEqual(output[:3], (21.0, 105.0, 25.0))
        state.last_command_time = time.monotonic() - VISUAL_FOLLOW_TIMEOUT_S - 0.1
        self.assertFalse(state.output()[-1])

    def test_disabled_target_does_not_require_position_fields(self) -> None:
        state = VisualFollowTargetState()
        state.update({"enabled": False})
        self.assertFalse(state.output()[-1])

    def test_follow_target_packet_contains_position_and_velocity(self) -> None:
        calls = []

        class FakeMav:
            def follow_target_send(self, *args):
                calls.append(args)

        class FakeConnection:
            mav = FakeMav()

        class FakeWorker:
            connection = FakeConnection()

        MavlinkWorker.send_visual_follow_target(
            FakeWorker(),
            21.0,
            105.0,
            25.0,
            1.0,
            -0.5,
            0.1,
            0.8,
        )
        packet = calls[0]
        self.assertEqual(packet[1], 3)
        self.assertEqual(packet[2:5], (210000000, 1050000000, 25.0))
        self.assertEqual(packet[5], [1.0, -0.5, 0.1])

    def test_follow_parameters_use_fixed_altitude_and_safe_height(self) -> None:
        calls = []

        class FakeMav:
            def param_set_send(self, *args):
                calls.append(args)

        class FakeConnection:
            mav = FakeMav()

        class FakeWorker:
            connection = FakeConnection()
            expected_system_id = 1

        MavlinkWorker.configure_visual_follow_parameters(FakeWorker(), 10.0, 3.0)
        values = {call[2]: call[3] for call in calls}
        self.assertEqual(values[b"FLW_TGT_ALT_M"], 0.0)
        self.assertEqual(values[b"FLW_TGT_DST"], 10.0)
        self.assertEqual(values[b"FLW_TGT_HT"], 8.0)
        self.assertEqual(values[b"FLW_TGT_RS"], 0.75)
        self.assertEqual(values[b"FLW_TGT_MAX_VEL"], 0.4)

    def test_native_follow_and_offboard_are_mutually_exclusive(self) -> None:
        class Message:
            def __init__(self, payload):
                self.payload = json.dumps(payload).encode("utf-8")

        drone_id = "UAV-01"
        offboard = offboard_follow_states[drone_id]
        native = visual_follow_target_states[drone_id]
        try:
            offboard.update({"enabled": True})
            on_mqtt_message(
                None,
                None,
                Message(
                    {
                        "type": "visual_follow_target",
                        "drone_id": drone_id,
                        "enabled": True,
                        "latitude_deg": 21.0,
                        "longitude_deg": 105.0,
                        "altitude_msl_m": 25.0,
                        "quality": 0.9,
                        "follow_distance_m": 10.0,
                        "follow_height_m": 8.0,
                    }
                ),
            )
            self.assertFalse(offboard.output()[4])
            self.assertTrue(native.output()[-1])

            on_mqtt_message(
                None,
                None,
                Message(
                    {
                        "type": "manual_control",
                        "drone_id": drone_id,
                        "enabled": True,
                        "forward": 0.1,
                    }
                ),
            )
            self.assertFalse(native.output()[-1])
        finally:
            offboard.disable()
            native.disable()


if __name__ == "__main__":
    unittest.main()
