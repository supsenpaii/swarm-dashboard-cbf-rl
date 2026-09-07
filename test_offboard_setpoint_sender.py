from __future__ import annotations

import math
import unittest

from cbf_command_gate import CbfCommand
from companion_safety import CompanionSafetyStatus
from emergency_supervisor import EmergencyDecision, EmergencyStage
from offboard_setpoint_sender import (
    MAV_COMP_ID_AUTOPILOT1,
    MAV_FRAME_LOCAL_NED,
    OFFBOARD_VELOCITY_TYPE_MASK,
    OffboardSetpointPreview,
    ShadowOffboardSetpointSender,
    enu_to_ned_velocity,
)


PX4_MAIN_MODE_OFFBOARD = 6
PX4_MAIN_MODE_POSCTL = 3


def decision(
    stage: EmergencyStage = EmergencyStage.NORMAL,
    velocity=(0.0, 0.0, 0.0),
    recommended_px4_mode: str | None = None,
) -> EmergencyDecision:
    return EmergencyDecision(
        stage=stage,
        velocity_enu_m_s=velocity,
        active=stage is not EmergencyStage.NORMAL,
        reason="test",
        recommended_px4_mode=recommended_px4_mode,
        time_in_stage_s=0.0,
    )


def status(
    velocity=(0.0, 0.0, 0.0),
    output_valid: bool = True,
    stage: EmergencyStage = EmergencyStage.NORMAL,
    recommended_px4_mode: str | None = None,
) -> CompanionSafetyStatus:
    return CompanionSafetyStatus(
        drone_id="UAV-01",
        nominal_velocity_enu_m_s=(9.0, 9.0, 9.0),  # deliberately different
        nominal_active=True,
        nominal_reason="tracking_slot",
        nominal_position_error_m=None,
        command=CbfCommand("UAV-01", (7.0, 7.0, 7.0), True, "cbf_filtered", 1.0, 0.0),
        emergency=decision(stage, velocity, recommended_px4_mode),
        output_velocity_enu_m_s=velocity,
        output_valid=output_valid,
        peer_ids_used=("UAV-02",),
        peer_ids_missing=(),
    )


def sender(**overrides) -> ShadowOffboardSetpointSender:
    defaults = dict(
        drone_id="UAV-01",
        px4_system_id=1,
        maximum_velocity_m_s=2.0,
        maximum_command_age_s=0.5,
    )
    defaults.update(overrides)
    return ShadowOffboardSetpointSender(**defaults)


def preview(
    send: ShadowOffboardSetpointSender,
    st: CompanionSafetyStatus,
    *,
    now: float = 100.0,
    command_at: float | None = 100.0,
    mode: int | None = PX4_MAIN_MODE_OFFBOARD,
    armed: bool | None = True,
) -> OffboardSetpointPreview:
    return send.preview(
        st,
        now_monotonic_s=now,
        command_monotonic_s=command_at,
        px4_main_mode=mode,
        px4_armed=armed,
        px4_offboard_main_mode=PX4_MAIN_MODE_OFFBOARD,
    )


class FrameConversionTests(unittest.TestCase):
    """ENU (east, north, up) -> NED (north, east, down)."""

    def test_zero_vector(self) -> None:
        self.assertEqual(enu_to_ned_velocity((0.0, 0.0, 0.0)), (0.0, 0.0, 0.0))

    def test_plus_east(self) -> None:
        # ENU +east -> NED east component (index 1), nothing else.
        self.assertEqual(enu_to_ned_velocity((1.0, 0.0, 0.0)), (0.0, 1.0, 0.0))

    def test_minus_east(self) -> None:
        self.assertEqual(enu_to_ned_velocity((-1.0, 0.0, 0.0)), (0.0, -1.0, 0.0))

    def test_plus_north(self) -> None:
        # ENU +north -> NED north component (index 0).
        self.assertEqual(enu_to_ned_velocity((0.0, 1.0, 0.0)), (1.0, 0.0, 0.0))

    def test_minus_north(self) -> None:
        self.assertEqual(enu_to_ned_velocity((0.0, -1.0, 0.0)), (-1.0, 0.0, 0.0))

    def test_plus_up_becomes_negative_down(self) -> None:
        # Climbing in ENU is NEGATIVE down in NED. Sign error here would
        # invert every altitude command.
        self.assertEqual(enu_to_ned_velocity((0.0, 0.0, 1.0)), (0.0, 0.0, -1.0))

    def test_minus_up_becomes_positive_down(self) -> None:
        self.assertEqual(enu_to_ned_velocity((0.0, 0.0, -1.0)), (0.0, 0.0, 1.0))

    def test_diagonal_vector_maps_every_axis_at_once(self) -> None:
        self.assertEqual(
            enu_to_ned_velocity((1.0, 2.0, 3.0)), (2.0, 1.0, -3.0)
        )

    def test_is_exact_inverse_of_swarm_state_ned_to_enu(self) -> None:
        """The project already has ned_to_enu; this must invert it exactly,
        or the two halves of the system would disagree about the frame."""
        from swarm_state import ned_to_enu

        for ned in ((1.0, 2.0, 3.0), (-4.0, 5.0, -6.0), (0.0, 0.0, 0.0)):
            with self.subTest(ned=ned):
                self.assertEqual(enu_to_ned_velocity(ned_to_enu(ned)), ned)

    def test_magnitude_is_preserved(self) -> None:
        enu = (0.6, -0.8, 1.5)
        ned = enu_to_ned_velocity(enu)
        self.assertAlmostEqual(
            math.sqrt(sum(v * v for v in enu)),
            math.sqrt(sum(v * v for v in ned)),
            places=12,
        )


class MirroredConstantTests(unittest.TestCase):
    """This module must not import pymavlink, so its constants are mirrors.
    Pin them to the real values so the mirror cannot drift silently."""

    def test_constants_match_pymavlink(self) -> None:
        from pymavlink import mavutil

        self.assertEqual(MAV_FRAME_LOCAL_NED, mavutil.mavlink.MAV_FRAME_LOCAL_NED)
        self.assertEqual(
            MAV_COMP_ID_AUTOPILOT1, mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1
        )

    def test_type_mask_matches_the_bridge_constant(self) -> None:
        import mavlink_manual_bridge as bridge

        self.assertEqual(
            OFFBOARD_VELOCITY_TYPE_MASK, bridge.OFFBOARD_VELOCITY_TYPE_MASK
        )

    def test_type_mask_actually_enables_all_three_velocity_axes(self) -> None:
        # Bits set = IGNORED. vx=3, vy=4, vz=5 must all be clear, or the
        # vertical command from ALTITUDE_SEPARATION would be silently dropped.
        for bit, name in ((3, "vx"), (4, "vy"), (5, "vz")):
            with self.subTest(axis=name):
                self.assertEqual(OFFBOARD_VELOCITY_TYPE_MASK >> bit & 1, 0)


class HardTransmitLockTests(unittest.TestCase):
    def test_preview_flags_are_constant_false(self) -> None:
        result = preview(sender(), status((1.0, 0.0, 0.0)))

        self.assertFalse(result.transmit_allowed)
        self.assertFalse(result.transmit_attempted)
        self.assertFalse(result.as_dict()["transmit_allowed"])
        self.assertFalse(result.as_dict()["transmit_attempted"])

    def test_transmit_flags_cannot_be_set_via_constructor(self) -> None:
        """They are properties, not dataclass fields -- there is no keyword
        to pass, so no config or test harness can flip them."""
        with self.assertRaises(TypeError):
            OffboardSetpointPreview(  # type: ignore[call-arg]
                drone_id="UAV-01",
                source_authority="shadow_companion_only",
                input_velocity_enu_m_s=(0.0, 0.0, 0.0),
                output_velocity_ned_m_s=(0.0, 0.0, 0.0),
                px4_target_system=1,
                px4_target_component=1,
                px4_frame=1,
                px4_type_mask=1479,
                intended_message_type="SET_POSITION_TARGET_LOCAL_NED",
                yaw_rate_rad_s=0.0,
                boot_time_ms=0,
                evaluated_monotonic_s=0.0,
                command_age_ms=0.0,
                emergency_stage="normal",
                recommended_px4_mode=None,
                valid=True,
                inhibited=False,
                inhibit_reason="",
                transmit_allowed=True,
            )

    def test_module_imports_nothing_that_could_transmit(self) -> None:
        """Structural lock, checked against the parsed import graph rather
        than the raw text (the module's own docstring discusses pymavlink):
        the module must not import any transport library, so it physically
        cannot construct or send a MAVLink/MQTT/socket message."""
        import ast
        from pathlib import Path

        tree = ast.parse(
            Path("offboard_setpoint_sender.py").read_text(encoding="utf-8")
        )
        imported: set[str] = set()
        for node in ast.walk(tree):
            if isinstance(node, ast.Import):
                imported.update(alias.name.split(".")[0] for alias in node.names)
            elif isinstance(node, ast.ImportFrom) and node.module:
                imported.add(node.module.split(".")[0])

        for forbidden in ("pymavlink", "socket", "paho", "serial", "requests"):
            with self.subTest(module=forbidden):
                self.assertNotIn(forbidden, imported)
        # Whatever it does import must be inert stdlib/typing only.
        self.assertTrue(imported <= {"__future__", "dataclasses", "math", "typing"})

    def test_module_contains_no_mavlink_send_call(self) -> None:
        """No attribute call ending in _send anywhere in the module."""
        import ast
        from pathlib import Path

        tree = ast.parse(
            Path("offboard_setpoint_sender.py").read_text(encoding="utf-8")
        )
        called = [
            node.func.attr
            for node in ast.walk(tree)
            if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute)
        ]
        self.assertEqual([name for name in called if name.endswith("_send")], [])

    def test_counters_stay_zero(self) -> None:
        send = sender()
        for _ in range(5):
            preview(send, status((1.0, 0.0, 0.0)))

        self.assertEqual(send.transmit_attempt_count, 0)
        self.assertEqual(send.arm_command_count, 0)
        self.assertEqual(send.mode_change_count, 0)
        self.assertFalse(send.status()["transmit_allowed"])


class ValidPreviewTests(unittest.TestCase):
    def test_normal_command_is_previewed_with_correct_px4_fields(self) -> None:
        # Magnitude 1.375 m/s, inside the 2.0 m/s limit.
        result = preview(sender(), status((0.6, 1.2, 0.3)))

        self.assertTrue(result.valid)
        self.assertFalse(result.inhibited)
        self.assertEqual(result.inhibit_reason, "")
        self.assertEqual(result.input_velocity_enu_m_s, (0.6, 1.2, 0.3))
        self.assertEqual(result.output_velocity_ned_m_s, (1.2, 0.6, -0.3))
        self.assertEqual(result.px4_frame, MAV_FRAME_LOCAL_NED)
        self.assertEqual(result.px4_type_mask, OFFBOARD_VELOCITY_TYPE_MASK)
        self.assertEqual(result.px4_target_component, MAV_COMP_ID_AUTOPILOT1)
        self.assertEqual(result.intended_message_type, "SET_POSITION_TARGET_LOCAL_NED")

    def test_yaw_rate_is_radians_and_zero(self) -> None:
        """The pipeline produces no yaw authority; the preview must not
        invent one. Units are rad/s to match the bridge's transmit boundary."""
        result = preview(sender(), status((1.0, 0.0, 0.0)))

        self.assertEqual(result.yaw_rate_rad_s, 0.0)
        self.assertIn("yaw_rate_rad_s", result.as_dict())
        self.assertNotIn("yaw_rate_deg_s", result.as_dict())

    def test_command_age_is_reported_in_milliseconds(self) -> None:
        result = preview(sender(), status((1.0, 0.0, 0.0)), now=100.2, command_at=100.0)

        self.assertAlmostEqual(result.command_age_ms, 200.0, places=6)


class FreshnessAndValidityGateTests(unittest.TestCase):
    def test_stale_command_is_inhibited(self) -> None:
        result = preview(
            sender(maximum_command_age_s=0.5),
            status((1.0, 0.0, 0.0)),
            now=101.0,
            command_at=100.0,
        )

        self.assertTrue(result.inhibited)
        self.assertEqual(result.inhibit_reason, "command_stale")

    def test_stale_command_previews_zero_not_the_last_good_value(self) -> None:
        """Replaying a stale command is the specific failure this gate
        exists to prevent -- inhibited must mean zero, not 'previous'."""
        send = sender()
        preview(send, status((1.5, 0.0, 0.0)))  # good command first
        result = preview(
            send, status((1.5, 0.0, 0.0)), now=101.0, command_at=100.0
        )

        self.assertTrue(result.inhibited)
        self.assertEqual(result.inhibit_category, "command_integrity")
        self.assertEqual(result.input_velocity_enu_m_s, (0.0, 0.0, 0.0))
        self.assertEqual(result.output_velocity_ned_m_s, (0.0, 0.0, 0.0))

    def test_missing_timestamp_is_inhibited(self) -> None:
        result = preview(sender(), status((1.0, 0.0, 0.0)), command_at=None)

        self.assertTrue(result.inhibited)
        self.assertEqual(result.inhibit_reason, "missing_command_timestamp")

    def test_regressed_timestamp_is_inhibited(self) -> None:
        result = preview(
            sender(), status((1.0, 0.0, 0.0)), now=100.0, command_at=101.0
        )

        self.assertTrue(result.inhibited)
        self.assertEqual(result.inhibit_reason, "command_timestamp_regressed")

    def test_validator_rejection_is_inhibited(self) -> None:
        result = preview(sender(), status((1.0, 0.0, 0.0), output_valid=False))

        self.assertTrue(result.inhibited)
        self.assertEqual(result.inhibit_reason, "command_validator_rejected")

    def test_nan_command_is_inhibited(self) -> None:
        result = preview(sender(), status((float("nan"), 0.0, 0.0)))

        self.assertTrue(result.inhibited)
        self.assertEqual(result.inhibit_reason, "malformed_command")

    def test_inf_command_is_inhibited(self) -> None:
        result = preview(sender(), status((float("inf"), 0.0, 0.0)))

        self.assertTrue(result.inhibited)
        self.assertEqual(result.inhibit_reason, "malformed_command")

    def test_over_limit_command_is_inhibited(self) -> None:
        result = preview(sender(maximum_velocity_m_s=2.0), status((5.0, 0.0, 0.0)))

        self.assertTrue(result.inhibited)
        self.assertEqual(result.inhibit_reason, "command_exceeds_velocity_limit")

    def test_unauthorized_source_is_inhibited(self) -> None:
        class ForeignStatus:
            output_valid = True
            output_velocity_enu_m_s = (1.0, 0.0, 0.0)
            emergency = decision()

            def as_dict(self):
                return {"authority": "shadow_safety_gate_only"}

        result = preview(sender(), ForeignStatus())

        self.assertTrue(result.inhibited)
        self.assertEqual(result.inhibit_reason, "unauthorized_source")

    def test_disarmed_px4_is_inhibited_but_preserves_the_would_be_setpoint(self) -> None:
        """Vehicle-readiness inhibit: the command is trustworthy, so the
        converted setpoint stays inspectable. valid is still False, so
        nothing may treat it as transmittable."""
        result = preview(sender(), status((1.0, 0.0, 0.0)), armed=False)

        self.assertTrue(result.inhibited)
        self.assertFalse(result.valid)
        self.assertEqual(result.inhibit_reason, "px4_not_armed")
        self.assertEqual(result.inhibit_category, "vehicle_not_ready")
        self.assertEqual(result.input_velocity_enu_m_s, (1.0, 0.0, 0.0))
        self.assertEqual(result.output_velocity_ned_m_s, (0.0, 1.0, -0.0))

    def test_unknown_arm_state_fails_closed(self) -> None:
        result = preview(sender(), status((1.0, 0.0, 0.0)), armed=None)

        self.assertTrue(result.inhibited)
        self.assertFalse(result.valid)
        self.assertEqual(result.inhibit_reason, "px4_not_armed")

    def test_px4_not_in_offboard_is_inhibited(self) -> None:
        result = preview(
            sender(), status((1.0, 0.0, 0.0)), mode=PX4_MAIN_MODE_POSCTL
        )

        self.assertTrue(result.inhibited)
        self.assertFalse(result.valid)
        self.assertEqual(result.inhibit_reason, "px4_not_in_offboard")
        self.assertEqual(result.inhibit_category, "vehicle_not_ready")

    def test_unknown_px4_mode_fails_closed(self) -> None:
        result = preview(sender(), status((1.0, 0.0, 0.0)), mode=None)

        self.assertTrue(result.inhibited)
        self.assertFalse(result.valid)
        self.assertEqual(result.inhibit_reason, "px4_mode_unknown")


class EmergencyStageMappingTests(unittest.TestCase):
    """Each §20 stage maps to a defined preview behaviour."""

    def test_normal_previews_the_supervisor_command(self) -> None:
        result = preview(sender(), status((1.0, 0.5, 0.0), stage=EmergencyStage.NORMAL))

        self.assertFalse(result.inhibited)
        self.assertEqual(result.emergency_stage, "normal")
        self.assertEqual(result.output_velocity_ned_m_s, (0.5, 1.0, -0.0))

    def test_stop_horizontal_previews_zero(self) -> None:
        result = preview(
            sender(), status((0.0, 0.0, 0.0), stage=EmergencyStage.STOP_HORIZONTAL)
        )

        self.assertEqual(result.emergency_stage, "stop_horizontal")
        self.assertEqual(result.output_velocity_ned_m_s, (0.0, 0.0, 0.0))

    def test_reduce_velocity_previews_supervisor_output(self) -> None:
        result = preview(
            sender(), status((0.0, 0.0, 0.0), stage=EmergencyStage.REDUCE_VELOCITY)
        )

        self.assertEqual(result.emergency_stage, "reduce_velocity")
        self.assertEqual(result.output_velocity_ned_m_s, (0.0, 0.0, 0.0))

    def test_altitude_separation_preserves_climb_sign_after_conversion(self) -> None:
        """A climb (+up in ENU) must become NEGATIVE down in NED. Getting
        this backwards would command a descent during deconfliction."""
        result = preview(
            sender(),
            status((0.0, 0.0, 2.0), stage=EmergencyStage.ALTITUDE_SEPARATION),
        )

        self.assertEqual(result.emergency_stage, "altitude_separation")
        self.assertEqual(result.input_velocity_enu_m_s[2], 2.0)
        self.assertEqual(result.output_velocity_ned_m_s[2], -2.0)
        self.assertLess(result.output_velocity_ned_m_s[2], 0.0)

    def test_altitude_separation_preserves_descent_sign(self) -> None:
        result = preview(
            sender(),
            status((0.0, 0.0, -1.5), stage=EmergencyStage.ALTITUDE_SEPARATION),
        )

        self.assertEqual(result.output_velocity_ned_m_s[2], 1.5)

    def test_hold_previews_inert_zero_command(self) -> None:
        result = preview(sender(), status((0.0, 0.0, 0.0), stage=EmergencyStage.HOLD))

        self.assertEqual(result.emergency_stage, "hold")
        self.assertEqual(result.output_velocity_ned_m_s, (0.0, 0.0, 0.0))
        self.assertFalse(result.inhibited)

    def test_recommend_position_hold_is_annotation_only_and_inhibits(self) -> None:
        result = preview(
            sender(),
            status(
                (0.0, 0.0, 0.0),
                stage=EmergencyStage.RECOMMEND_POSITION_HOLD,
                recommended_px4_mode="POSITION_HOLD",
            ),
        )

        self.assertEqual(result.recommended_px4_mode, "POSITION_HOLD")
        self.assertTrue(result.inhibited)
        self.assertEqual(result.inhibit_reason, "emergency_recommends_mode_change")
        # Annotation only: no mode change performed or counted.
        self.assertEqual(result.output_velocity_ned_m_s, (0.0, 0.0, 0.0))

    def test_recommend_rtl_or_land_is_annotation_only_and_inhibits(self) -> None:
        send = sender()
        result = preview(
            send,
            status(
                (0.0, 0.0, 0.0),
                stage=EmergencyStage.RECOMMEND_RTL_OR_LAND,
                recommended_px4_mode="RTL_OR_LAND",
            ),
        )

        self.assertEqual(result.recommended_px4_mode, "RTL_OR_LAND")
        self.assertTrue(result.inhibited)
        self.assertEqual(result.inhibit_reason, "emergency_recommends_mode_change")
        self.assertEqual(send.mode_change_count, 0)


class AuthorityBoundaryTests(unittest.TestCase):
    def test_sender_uses_validated_output_not_nominal_or_raw_cbf(self) -> None:
        """The sender must not reach behind the validator. status() sets the
        nominal to (9,9,9) and the raw CBF command to (7,7,7); only the
        validated output (1,0,0) may appear."""
        st = status((1.0, 0.0, 0.0))
        result = preview(sender(), st)

        self.assertEqual(result.input_velocity_enu_m_s, (1.0, 0.0, 0.0))
        self.assertNotEqual(result.input_velocity_enu_m_s, st.nominal_velocity_enu_m_s)
        self.assertNotEqual(
            result.input_velocity_enu_m_s, st.command.velocity_enu_m_s
        )

    def test_authority_string_is_carried_through(self) -> None:
        result = preview(sender(), status((1.0, 0.0, 0.0)))

        self.assertEqual(result.source_authority, "shadow_companion_only")
        self.assertEqual(
            result.as_dict()["authority"], "shadow_offboard_setpoint_preview_only"
        )


class ConstructorValidationTests(unittest.TestCase):
    def test_rejects_invalid_configuration(self) -> None:
        for kwargs in (
            {"drone_id": "  "},
            {"px4_system_id": 0},
            {"px4_system_id": 999},
            {"maximum_velocity_m_s": 0.0},
            {"maximum_velocity_m_s": float("nan")},
            {"maximum_command_age_s": -1.0},
        ):
            with self.subTest(kwargs=kwargs):
                with self.assertRaises(ValueError):
                    sender(**kwargs)


if __name__ == "__main__":
    unittest.main()
