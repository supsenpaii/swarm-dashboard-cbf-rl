from __future__ import annotations

import json
import inspect
import time
import unittest
from unittest import mock
from types import SimpleNamespace
from unittest.mock import patch

from pymavlink import mavutil

import mavlink_manual_bridge as bridge_module
from mavlink_manual_bridge import (
    OFFBOARD_ATTITUDE_TIMEOUT_S,
    OFFBOARD_FOLLOW_TIMEOUT_S,
    OFFBOARD_LOCAL_ALTITUDE_HOLD_TYPE_MASK,
    OFFBOARD_LOCAL_ACCEL_ALTITUDE_HOLD_TYPE_MASK,
    PX4_MAIN_MODE_AUTO,
    PX4_SUB_MODE_AUTO_FOLLOW_TARGET,
    MavlinkWorker,
    FastPoseCache,
    OffboardFollowState,
    OffboardAttitudeState,
    VISUAL_FOLLOW_TIMEOUT_S,
    VISUAL_FOLLOW_HOLD_TIMEOUT_S,
    VISUAL_FOLLOW_MAX_TARGET_SPEED_M_S,
    VISUAL_FOLLOW_MAX_TARGET_VERTICAL_SPEED_M_S,
    VISUAL_FOLLOW_PRESTREAM_FRAMES,
    VISUAL_FOLLOW_SETTLE_FRAMES,
    VISUAL_FOLLOW_VELOCITY_RAMP_S,
    VisualFollowTargetState,
    offboard_follow_states,
    on_mqtt_message,
    visual_follow_target_states,
)
from swarm_state import GeodeticOrigin



def legacy_authority():
    """Authority context permitting the legacy OFFBOARD writer.

    The single-writer interlock in mavlink_manual_bridge refuses transmit
    unless exactly one writer is authorized. Tests of the legacy writer's
    wire format must therefore state which writer they are exercising.
    """
    return mock.patch.dict(
        "os.environ",
        {
            "SWARM_OFFBOARD_AUTHORITY": "legacy_tracking",
            "SWARM_TRACKING_OFFBOARD_ENABLED": "true",
        },
        clear=False,
    )


class FastPosePeerStateTests(unittest.TestCase):
    class Message:
        def __init__(self, message_type, **values):
            self.message_type = message_type
            for key, value in values.items():
                setattr(self, key, value)

        def get_type(self):
            return self.message_type

    def test_global_and_local_pose_build_common_enu_peer_packet(self):
        cache = FastPoseCache()
        cache.update(
            self.Message(
                "LOCAL_POSITION_NED",
                x=30.0,
                y=40.0,
                z=-5.0,
                vx=1.0,
                vy=2.0,
                vz=3.0,
            ),
            10.0,
        )
        cache.update(
            self.Message(
                "GLOBAL_POSITION_INT",
                lat=100000100,
                lon=1060000200,
                alt=20000,
            ),
            10.0,
        )
        packet = cache.peer_payload(
            "UAV-01",
            10.1,
            GeodeticOrigin(10.0, 106.0, 15.0),
            sequence=7,
            healthy=True,
        )
        self.assertIsNotNone(packet)
        assert packet is not None
        self.assertEqual(packet["frame"], "ENU")
        self.assertEqual(packet["sequence"], 7)
        self.assertEqual(packet["velocity_enu_m_s"], [2.0, 1.0, -3.0])
        self.assertGreater(packet["position_enu_m"][0], 2.0)
        self.assertGreater(packet["position_enu_m"][1], 1.0)

    def test_peer_packet_is_suppressed_when_global_position_is_stale(self):
        cache = FastPoseCache()
        cache.update(
            self.Message(
                "LOCAL_POSITION_NED", x=0.0, y=0.0, z=0.0,
                vx=0.0, vy=0.0, vz=0.0,
            ),
            10.0,
        )
        cache.update(
            self.Message(
                "GLOBAL_POSITION_INT", lat=100000000, lon=1060000000, alt=15000,
            ),
            10.0,
        )
        self.assertIsNone(
            cache.peer_payload(
                "UAV-01",
                10.6,
                GeodeticOrigin(10.0, 106.0, 15.0),
                sequence=0,
                healthy=True,
            )
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
    def test_watchdog_neutralizes_stale_offboard_command(self) -> None:
        state = OffboardFollowState()
        state.update(
            {
                "enabled": True,
                "velocity_frame": "local_ned_altitude_hold",
                "north_velocity_m_s": 0.8,
                "hold_z_down_m": -3.25,
            }
        )
        state.last_command_time = (
            time.monotonic() - OFFBOARD_FOLLOW_TIMEOUT_S - 0.1
        )
        output = state.output()
        self.assertFalse(output[4])
        self.assertEqual(output[0:4], (0.0, 0.0, 0.0, 0.0))
        self.assertEqual(output[5:8], (0.0, 0.0, 0.0))
        self.assertFalse(output[8])

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
            legacy_offboard_refused_count = 0

        # This exercises the LEGACY offboard writer's packet format, so it
        # must declare legacy authority. Without it the single-writer
        # interlock correctly refuses and no packet is produced.
        with legacy_authority():
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
            legacy_offboard_refused_count = 0

        with legacy_authority():
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
    @staticmethod
    def measured_payload(**overrides):
        payload = {
            "enabled": True,
            "session_id": 1,
            "source_kind": "midas_metric",
            "target_semantics": "measured_target",
            "measurement_timestamp_s": time.monotonic(),
            "latitude_deg": 21.0,
            "longitude_deg": 105.0,
            "altitude_msl_m": 25.0,
            "quality": 0.8,
            "follow_distance_m": 10.0,
            "follow_height_m": 8.0,
        }
        payload.update(overrides)
        return payload

    def test_valid_target_holds_neutral_before_watchdog_expires(self) -> None:
        state = VisualFollowTargetState()
        state.update(
            self.measured_payload(
                velocity_north_m_s=1.0,
                velocity_east_m_s=-0.5,
                velocity_down_m_s=0.1,
            )
        )
        state.activated_monotonic_s = (
            time.monotonic() - VISUAL_FOLLOW_VELOCITY_RAMP_S
        )
        output = state.output()
        self.assertTrue(output[-1])
        self.assertEqual(output[:3], (21.0, 105.0, 25.0))
        state.last_command_time = time.monotonic() - VISUAL_FOLLOW_TIMEOUT_S - 0.1
        self.assertFalse(state.output()[-1])
        self.assertEqual(state.status()["timeout_state"], "STALE")
        self.assertFalse(state.output()[-1])

    def test_target_velocity_is_ramped_and_clamped(self) -> None:
        state = VisualFollowTargetState()
        state.update(
            self.measured_payload(
                velocity_north_m_s=3.0,
                velocity_east_m_s=4.0,
                velocity_down_m_s=2.0,
            )
        )
        initial = state.output()
        self.assertLess(
            (initial[3] ** 2 + initial[4] ** 2) ** 0.5,
            VISUAL_FOLLOW_MAX_TARGET_SPEED_M_S,
        )
        state.activated_monotonic_s = (
            time.monotonic() - VISUAL_FOLLOW_VELOCITY_RAMP_S
        )
        ramped = state.output()
        self.assertAlmostEqual(
            (ramped[3] ** 2 + ramped[4] ** 2) ** 0.5,
            VISUAL_FOLLOW_MAX_TARGET_SPEED_M_S,
            places=5,
        )
        self.assertEqual(
            ramped[5],
            VISUAL_FOLLOW_MAX_TARGET_VERTICAL_SPEED_M_S,
        )

    def test_disabled_target_does_not_require_position_fields(self) -> None:
        state = VisualFollowTargetState()
        state.update({"enabled": False})
        self.assertFalse(state.output()[-1])

    def test_short_stale_measurement_is_rejected(self) -> None:
        state = VisualFollowTargetState()
        state.update(
            self.measured_payload(
                measurement_timestamp_s=(
                    time.monotonic() - VISUAL_FOLLOW_TIMEOUT_S - 0.1
                )
            )
        )
        output = state.output()
        self.assertFalse(output[-1])
        self.assertEqual(state.status()["timeout_state"], "INACTIVE")

    def test_measurement_older_than_hold_timeout_is_rejected(self) -> None:
        state = VisualFollowTargetState()
        state.update(
            self.measured_payload(
                measurement_timestamp_s=(
                    time.monotonic() - VISUAL_FOLLOW_HOLD_TIMEOUT_S - 0.1
                )
            )
        )
        self.assertFalse(state.output()[-1])
        self.assertEqual(state.status()["timeout_state"], "INACTIVE")

    def test_zero_or_provisional_target_is_rejected(self) -> None:
        zero_state = VisualFollowTargetState()
        zero_state.update(
            self.measured_payload(latitude_deg=0.0, longitude_deg=0.0)
        )
        self.assertFalse(zero_state.output()[-1])

        provisional_state = VisualFollowTargetState()
        provisional_state.update(
            self.measured_payload(source_kind="selection_anchor_provisional")
        )
        self.assertFalse(provisional_state.output()[-1])

    def test_duplicate_out_of_order_and_previous_session_are_rejected(
        self,
    ) -> None:
        state = VisualFollowTargetState()
        first_timestamp = time.monotonic()
        state.update(
            self.measured_payload(
                session_id=7,
                measurement_timestamp_s=first_timestamp,
            )
        )
        self.assertTrue(state.output()[-1])

        state.update(
            self.measured_payload(
                session_id=7,
                measurement_timestamp_s=first_timestamp,
            )
        )
        self.assertFalse(state.output()[-1])

        state.update(self.measured_payload(session_id=8))
        self.assertTrue(state.output()[-1])
        state.update(self.measured_payload(session_id=7))
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

    def test_follow_target_packet_uses_fusion_covariance_and_capabilities(
        self,
    ) -> None:
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
            0.0,
            0.0,
            0.0,
            0.7,
            (0.2, 0.3, 0.4),
            1,
        )
        packet = calls[0]
        self.assertEqual(packet[1], 1)
        self.assertEqual(packet[9], [0.2, 0.3, 0.4])

    def test_follow_target_sender_rejects_invalid_packet(self) -> None:
        calls = []

        class FakeMav:
            def follow_target_send(self, *args):
                calls.append(args)

        worker = SimpleNamespace(
            connection=SimpleNamespace(mav=FakeMav()),
        )
        self.assertFalse(MavlinkWorker.send_visual_follow_target(
            worker,
            0.0,
            0.0,
            25.0,
            0.0,
            0.0,
            0.0,
            0.8,
        ))
        self.assertFalse(MavlinkWorker.send_visual_follow_target(
            worker,
            21.0,
            105.0,
            float("nan"),
            0.0,
            0.0,
            0.0,
            0.8,
        ))
        self.assertEqual(calls, [])

    def test_stale_target_is_not_available_for_beacon(self) -> None:
        state = VisualFollowTargetState()
        state.update(
            self.measured_payload(
                est_capabilities=3,
                position_covariance=[0.2, 0.3, 0.4],
            )
        )
        state.last_command_time = (
            time.monotonic() - VISUAL_FOLLOW_TIMEOUT_S - 0.1
        )
        output = state.output()
        self.assertEqual(output[-3], (0.2, 0.3, 0.4))
        self.assertFalse(output[-1])

    def test_mode_request_default_off_and_no_auto_reentry(self) -> None:
        worker = MavlinkWorker.__new__(MavlinkWorker)
        worker.visual_follow_stream_frames = VISUAL_FOLLOW_PRESTREAM_FRAMES
        worker.visual_follow_settle_frames = VISUAL_FOLLOW_SETTLE_FRAMES
        worker.visual_follow_request_attempts = 0
        worker.visual_follow_entry_blocked = False
        worker.visual_follow_mode_requested = False
        worker.visual_follow_was_observed = False
        worker.visual_follow_mode_ack = "not_requested"
        worker.last_mode_request_monotonic = 0.0

        self.assertFalse(
            worker.should_request_visual_follow_mode(10.0, False)
        )
        with patch(
            "mavlink_manual_bridge.SAFE_NATIVE_FOLLOW_MODE_REQUEST_ENABLED",
            True,
        ):
            self.assertTrue(
                worker.should_request_visual_follow_mode(10.0, False)
            )
            self.assertFalse(
                worker.should_request_visual_follow_mode(11.0, True)
            )
            self.assertFalse(
                worker.should_request_visual_follow_mode(12.0, False)
            )
        self.assertTrue(worker.visual_follow_entry_blocked)
        self.assertEqual(worker.visual_follow_mode_ack, "reacquire_required")

    def test_follow_mode_request_is_pending_until_command_ack(self) -> None:
        calls = []

        class FakeMav:
            def command_long_send(self, *args):
                calls.append(args)

        worker = MavlinkWorker.__new__(MavlinkWorker)
        worker.connection = SimpleNamespace(mav=FakeMav())
        worker.expected_system_id = 2
        worker.drone_id = "UAV-02"
        worker.visual_follow_mode_ack = "not_requested"
        worker.visual_follow_mode_ack_result = None
        worker.visual_follow_mode_ack_monotonic = 0.0
        worker.visual_follow_exit_ack = "not_requested"
        with patch(
            "mavlink_manual_bridge.SAFE_NATIVE_FOLLOW_MODE_REQUEST_ENABLED",
            True,
        ):
            sent = worker.request_px4_main_mode(
                PX4_MAIN_MODE_AUTO,
                "AUTO FOLLOW TARGET",
                PX4_SUB_MODE_AUTO_FOLLOW_TARGET,
            )
        self.assertTrue(sent)
        self.assertEqual(worker.visual_follow_mode_ack, "pending")
        self.assertIsNone(worker.visual_follow_mode_ack_result)
        self.assertEqual(len(calls), 1)

        ack = SimpleNamespace(
            get_type=lambda: "COMMAND_ACK",
            get_srcSystem=lambda: 2,
            command=mavutil.mavlink.MAV_CMD_DO_SET_MODE,
            result=mavutil.mavlink.MAV_RESULT_ACCEPTED,
        )
        self.assertTrue(worker.handle_visual_follow_command_ack(ack))
        self.assertEqual(worker.visual_follow_mode_ack, "accepted")

    def test_follow_mode_request_feature_flag_blocks_command(self) -> None:
        calls = []

        class FakeMav:
            def command_long_send(self, *args):
                calls.append(args)

        worker = MavlinkWorker.__new__(MavlinkWorker)
        worker.connection = SimpleNamespace(mav=FakeMav())
        worker.expected_system_id = 2
        worker.drone_id = "UAV-02"
        worker.visual_follow_mode_ack = "not_requested"
        worker.visual_follow_mode_ack_result = None
        worker.visual_follow_mode_ack_monotonic = 0.0
        worker.visual_follow_entry_blocked = False
        sent = worker.request_px4_main_mode(
            PX4_MAIN_MODE_AUTO,
            "AUTO FOLLOW TARGET",
            PX4_SUB_MODE_AUTO_FOLLOW_TARGET,
        )
        self.assertFalse(sent)
        self.assertEqual(calls, [])
        self.assertEqual(
            worker.visual_follow_mode_ack,
            "blocked_by_feature_flag",
        )

    def test_follow_parameters_are_requested_and_validated_read_only(self) -> None:
        calls = []

        class FakeMav:
            def param_request_read_send(self, *args):
                calls.append(args)

        class FakeConnection:
            mav = FakeMav()

        worker = SimpleNamespace(
            connection=FakeConnection(),
            expected_system_id=1,
            drone_id="UAV-02",
            visual_follow_parameter_values={},
            visual_follow_parameter_last_request_monotonic=0.0,
            visual_follow_parameter_ack="not_requested",
            visual_follow_parameter_ack_reason="",
            visual_follow_parameter_ack_monotonic=0.0,
        )
        configured = MavlinkWorker.configure_visual_follow_parameters(
            worker,
            10.0,
            8.0,
        )
        self.assertFalse(configured)
        self.assertEqual(len(calls), 6)
        requested_names = {call[2] for call in calls}
        self.assertEqual(
            requested_names,
            {name.encode("ascii") for name in bridge_module.VISUAL_FOLLOW_PARAMETER_NAMES},
        )

        worker.visual_follow_parameter_values.update(
            {
                "FLW_TGT_ALT_M": 0.0,
                "FLW_TGT_DST": 10.0,
                "FLW_TGT_HT": 8.0,
                "FLW_TGT_FA": 180.0,
                "FLW_TGT_RS": 0.75,
                "FLW_TGT_MAX_VEL": 0.4,
            }
        )
        self.assertTrue(MavlinkWorker.configure_visual_follow_parameters(
            worker,
            10.0,
            8.0,
        ))
        self.assertEqual(worker.visual_follow_parameter_ack, "accepted")

    def test_read_only_follow_parameter_mismatch_fails_closed(self) -> None:
        worker = SimpleNamespace(
            connection=SimpleNamespace(mav=SimpleNamespace()),
            expected_system_id=1,
            drone_id="UAV-02",
            visual_follow_parameter_values={
                "FLW_TGT_ALT_M": 2.0,
                "FLW_TGT_DST": 10.0,
                "FLW_TGT_HT": 8.0,
                "FLW_TGT_FA": 180.0,
                "FLW_TGT_RS": 0.75,
                "FLW_TGT_MAX_VEL": 0.4,
            },
            visual_follow_parameter_last_request_monotonic=0.0,
            visual_follow_parameter_ack="not_requested",
            visual_follow_parameter_ack_reason="",
            visual_follow_parameter_ack_monotonic=0.0,
        )
        self.assertFalse(MavlinkWorker.configure_visual_follow_parameters(
            worker,
            10.0,
            8.0,
        ))
        self.assertEqual(worker.visual_follow_parameter_ack, "rejected")

    def test_production_bridge_contains_no_parameter_write(self) -> None:
        self.assertNotIn(
            "param_set_send",
            inspect.getsource(bridge_module),
        )

    def test_safe_native_path_has_no_manual_heartbeat_sender(self) -> None:
        source = inspect.getsource(bridge_module.MavlinkWorker)
        self.assertNotIn("send_visual_follow_manual_heartbeat", source)

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
                        "session_id": 1,
                        "source_kind": "midas_metric",
                        "target_semantics": "measured_target",
                        "measurement_timestamp_s": time.monotonic(),
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

    def test_neutral_offboard_packet_does_not_cancel_native_handover(self) -> None:
        class Message:
            def __init__(self, payload):
                self.payload = json.dumps(payload).encode("utf-8")

        drone_id = "UAV-01"
        offboard = offboard_follow_states[drone_id]
        native = visual_follow_target_states[drone_id]
        try:
            native.update(
                self.measured_payload(quality=0.9)
            )
            on_mqtt_message(
                None,
                None,
                Message(
                    {
                        "type": "offboard_follow",
                        "drone_id": drone_id,
                        "enabled": False,
                    }
                ),
            )
            self.assertTrue(native.output()[-1])
            self.assertFalse(offboard.output()[4])

            on_mqtt_message(
                None,
                None,
                Message(
                    {
                        "type": "offboard_follow",
                        "drone_id": drone_id,
                        "enabled": True,
                        "forward_velocity_m_s": 0.1,
                    }
                ),
            )
            self.assertTrue(native.output()[-1])
            self.assertFalse(offboard.output()[4])
        finally:
            offboard.disable()
            native.disable()


if __name__ == "__main__":
    unittest.main()
