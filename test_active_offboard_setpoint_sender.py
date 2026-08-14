from __future__ import annotations

import ast
import dataclasses
import unittest
from pathlib import Path
from unittest import mock

from active_offboard_setpoint_sender import (
    CALLER_REPORTED_CONDITIONS,
    SELF_DERIVED_CONDITIONS,
    ActiveOffboardSetpointSender,
    TransmitDecision,
)
from emergency_supervisor import EmergencyStage
from offboard_authority import AUTHORITY_ENV_VAR
from offboard_setpoint_sender import (
    MAV_COMP_ID_AUTOPILOT1,
    MAV_FRAME_LOCAL_NED,
    OFFBOARD_VELOCITY_TYPE_MASK,
)
from one_uav_active_readiness import ABORT_MATRIX, AbortAction, WarmupContract

# Previews are built by the real ShadowOffboardSetpointSender through the
# helpers the shadow suite already uses, never hand-fabricated: the active
# sender's contract is "consumes OffboardSetpointPreview unchanged", and a
# test that invented its own preview would not be testing that contract.
from test_offboard_setpoint_sender import preview, sender as shadow_sender, status


COMPANION = {AUTHORITY_ENV_VAR: "companion_safety"}


def good_preview(**overrides):
    """A preview that every gate should accept: valid, armed, in OFFBOARD."""
    return preview(shadow_sender(), status(velocity=(0.6, 1.2, 0.3)), **overrides)


def active(**overrides) -> tuple[ActiveOffboardSetpointSender, list[tuple]]:
    """Returns (sender, recorded wire calls). The sink records instead of
    transmitting, which is the only way to observe the wire without one."""
    calls: list[tuple] = []
    defaults = dict(
        drone_id="UAV-01",
        px4_system_id=1,
        explicit_opt_in=True,
        transmit=lambda *args: calls.append(args),
        environment=dict(COMPANION),
    )
    defaults.update(overrides)
    return ActiveOffboardSetpointSender(**defaults), calls


class AbortMatrixCoverageTests(unittest.TestCase):
    """Every rule in the matrix must be reachable, or the matrix is fiction."""

    def test_every_abort_condition_is_handled_exactly_once(self) -> None:
        handled = set(SELF_DERIVED_CONDITIONS) | set(CALLER_REPORTED_CONDITIONS)
        matrix = {rule.condition for rule in ABORT_MATRIX}

        self.assertEqual(handled, matrix)
        self.assertEqual(
            len(SELF_DERIVED_CONDITIONS) + len(CALLER_REPORTED_CONDITIONS),
            len(matrix),
            "a condition is claimed as both self-derived and caller-reported",
        )

    def test_every_caller_reported_condition_produces_its_matrix_action(self) -> None:
        for rule in ABORT_MATRIX:
            if rule.condition not in CALLER_REPORTED_CONDITIONS:
                continue
            with self.subTest(condition=rule.condition):
                send, calls = active()
                frame = send.step(
                    good_preview(), reported_conditions=[rule.condition]
                )

                self.assertEqual(frame.abort_condition, rule.condition)
                self.assertEqual(frame.abort_action, rule.immediate_action.value)
                if rule.immediate_action is AbortAction.SEND_ZERO_VELOCITY:
                    self.assertIs(frame.decision, TransmitDecision.TRANSMIT_ZERO)
                    self.assertEqual(frame.velocity_ned_m_s, (0.0, 0.0, 0.0))
                    self.assertEqual(len(calls), 1)
                elif rule.immediate_action is AbortAction.REQUEST_POSITION_MODE:
                    self.assertIs(
                        frame.decision, TransmitDecision.REQUEST_POSITION_MODE
                    )
                    self.assertEqual(calls, [])
                else:
                    self.assertIs(frame.decision, TransmitDecision.WITHHOLD)
                    self.assertEqual(calls, [])

    def test_matrix_order_is_the_severity_order_used(self) -> None:
        """Two conditions at once must resolve to the more severe rule, and
        'more severe' is the matrix's own ordering, not a second ranking."""
        send, _ = active()
        frame = send.step(
            good_preview(),
            reported_conditions=["cbf_invalid_or_infeasible", "self_telemetry_stale"],
        )

        # self_telemetry_stale is rule 0; cbf_invalid_or_infeasible is later.
        self.assertEqual(frame.abort_condition, "self_telemetry_stale")

    def test_unknown_reported_condition_cannot_authorize_anything(self) -> None:
        send, calls = active()
        frame = send.step(good_preview(), reported_conditions=["not_a_real_condition"])

        self.assertIsNone(frame.abort_condition)
        self.assertIs(frame.decision, TransmitDecision.TRANSMIT)
        self.assertEqual(len(calls), 1)


class SelfDerivedConditionTests(unittest.TestCase):
    def test_stale_command_aborts(self) -> None:
        send, calls = active()
        stale = preview(shadow_sender(), status(), now=101.0, command_at=100.0)

        frame = send.step(stale)

        self.assertEqual(frame.abort_condition, "command_stale")
        self.assertEqual(calls, [])

    def test_malformed_command_maps_to_nan_or_inf_rule(self) -> None:
        send, calls = active()
        bad = preview(shadow_sender(), status(velocity=(float("nan"), 0.0, 0.0)))

        frame = send.step(bad)

        self.assertEqual(frame.abort_condition, "nan_or_inf_command")
        self.assertEqual(calls, [])

    def test_emergency_hold_sends_zero_velocity(self) -> None:
        send, calls = active()
        held = preview(
            shadow_sender(),
            status(velocity=(0.0, 0.0, 0.0), stage=EmergencyStage.HOLD),
        )

        frame = send.step(held)

        self.assertEqual(frame.abort_condition, "emergency_supervisor_hold")
        self.assertIs(frame.decision, TransmitDecision.TRANSMIT_ZERO)
        self.assertEqual(len(calls), 1)

    def test_position_hold_recommendation_requests_position_mode(self) -> None:
        requested = []
        send, calls = active(request_position_mode=lambda: requested.append(1))
        recommended = preview(
            shadow_sender(),
            status(
                stage=EmergencyStage.RECOMMEND_POSITION_HOLD,
                recommended_px4_mode="POSITION_HOLD",
            ),
        )

        frame = send.step(recommended)

        self.assertIs(frame.decision, TransmitDecision.REQUEST_POSITION_MODE)
        self.assertEqual(requested, [1])
        self.assertEqual(calls, [], "no setpoint may accompany a mode handover")

    def test_rtl_recommendation_also_only_requests_position_mode(self) -> None:
        """No autonomous RTL/Land exists in this project; the sender must not
        pretend otherwise."""
        requested = []
        send, _ = active(request_position_mode=lambda: requested.append(1))
        recommended = preview(
            shadow_sender(),
            status(
                stage=EmergencyStage.RECOMMEND_RTL_OR_LAND,
                recommended_px4_mode="RTL_OR_LAND",
            ),
        )

        frame = send.step(recommended)

        self.assertEqual(frame.abort_condition, "emergency_recommends_rtl_or_land")
        self.assertEqual(frame.abort_action, AbortAction.REQUEST_POSITION_MODE.value)
        self.assertEqual(requested, [1])

    def test_missing_position_mode_sink_still_latches(self) -> None:
        """A sender with no mode-change sink must not carry on streaming as
        if the handover had happened."""
        send, calls = active(request_position_mode=None)
        send.step(
            preview(
                shadow_sender(),
                status(
                    stage=EmergencyStage.RECOMMEND_POSITION_HOLD,
                    recommended_px4_mode="POSITION_HOLD",
                ),
            ),
            now_monotonic_s=100.0,
        )
        frame = send.step(good_preview(now=100.05))

        self.assertTrue(frame.latched)
        self.assertEqual(calls, [])


class StreamContinuityTests(unittest.TestCase):
    def test_first_frame_has_no_gap(self) -> None:
        send, calls = active()
        frame = send.step(good_preview())

        self.assertIsNone(frame.interval_s)
        self.assertIs(frame.decision, TransmitDecision.TRANSMIT)
        self.assertEqual(len(calls), 1)

    def test_gap_larger_than_the_warmup_contract_aborts(self) -> None:
        send, calls = active()
        send.step(good_preview(), now_monotonic_s=100.0)
        gap = WarmupContract().maximum_gap_s + 0.01

        frame = send.step(good_preview(), now_monotonic_s=100.0 + gap)

        self.assertEqual(frame.abort_condition, "setpoint_stream_gap")
        self.assertEqual(len(calls), 1, "only the first frame went out")

    def test_gap_at_the_limit_is_accepted(self) -> None:
        send, _ = active()
        send.step(good_preview(), now_monotonic_s=100.0)

        frame = send.step(
            good_preview(), now_monotonic_s=100.0 + WarmupContract().maximum_gap_s
        )

        self.assertIsNone(frame.abort_condition)

    def test_regressed_clock_is_treated_as_a_broken_stream(self) -> None:
        send, _ = active()
        send.step(good_preview(), now_monotonic_s=100.0)

        frame = send.step(good_preview(), now_monotonic_s=99.9)

        self.assertEqual(frame.abort_condition, "setpoint_stream_gap")

    def test_nominal_cadence_streams_continuously(self) -> None:
        send, calls = active()
        now = 100.0
        for _ in range(40):  # 2 s at the companion's 20 Hz
            send.step(good_preview(), now_monotonic_s=now)
            now += 0.05

        self.assertEqual(len(calls), 40)
        self.assertEqual(send.withheld_count, 0)
        self.assertIsNone(send.latched_abort)


class LatchingTests(unittest.TestCase):
    def test_non_recoverable_abort_never_resumes(self) -> None:
        for rule in ABORT_MATRIX:
            if rule.recovery_allowed or rule.condition not in CALLER_REPORTED_CONDITIONS:
                continue
            with self.subTest(condition=rule.condition):
                send, calls = active()
                send.step(
                    good_preview(),
                    now_monotonic_s=99.95,
                    reported_conditions=[rule.condition],
                )
                before = len(calls)
                now = 100.0
                for _ in range(100):  # 100 perfectly healthy frames
                    frame = send.step(good_preview(), now_monotonic_s=now)
                    now += 0.05

                self.assertEqual(send.latched_abort, rule.condition)
                self.assertIs(frame.decision, TransmitDecision.WITHHOLD)
                self.assertTrue(frame.reason.startswith("abort_latched:"))
                self.assertEqual(len(calls), before)

    def test_recoverable_abort_does_not_latch(self) -> None:
        send, calls = active()
        send.step(
            good_preview(),
            now_monotonic_s=100.0,
            reported_conditions=["cbf_invalid_or_infeasible"],
        )

        frame = send.step(good_preview(), now_monotonic_s=100.05)

        self.assertIsNone(send.latched_abort)
        self.assertIs(frame.decision, TransmitDecision.TRANSMIT)
        self.assertEqual(len(calls), 2)  # the zero-velocity hold, then the real one

    def test_latch_survives_a_gap_that_would_otherwise_be_the_reason(self) -> None:
        """The recorded condition must stay the original cause, not be
        overwritten by a downstream symptom."""
        send, _ = active()
        send.step(
            good_preview(),
            now_monotonic_s=100.0,
            reported_conditions=["mavlink_connection_loss"],
        )

        frame = send.step(good_preview(), now_monotonic_s=200.0)

        self.assertEqual(send.latched_abort, "mavlink_connection_loss")
        self.assertEqual(frame.abort_condition, "mavlink_connection_loss")


class AuthorizationTests(unittest.TestCase):
    def test_transmits_only_when_every_gate_holds(self) -> None:
        send, calls = active()
        frame = send.step(good_preview())

        self.assertIs(frame.decision, TransmitDecision.TRANSMIT)
        self.assertTrue(frame.transmitted)
        self.assertEqual(len(calls), 1)

    def test_no_opt_in_means_no_transmission(self) -> None:
        send, calls = active(explicit_opt_in=False)
        frame = send.step(good_preview())

        self.assertIs(frame.decision, TransmitDecision.WITHHOLD)
        self.assertIn("explicit_opt_in_absent", frame.reason)
        self.assertEqual(calls, [])

    def test_opt_in_must_be_a_bool(self) -> None:
        with self.assertRaises(ValueError):
            ActiveOffboardSetpointSender("UAV-01", 1, explicit_opt_in="yes")

    def test_authority_disabled_blocks_every_frame(self) -> None:
        for value in ({}, {AUTHORITY_ENV_VAR: "disabled"}, {AUTHORITY_ENV_VAR: "junk"}):
            with self.subTest(environment=value):
                send, calls = active(environment=dict(value))
                frame = send.step(good_preview())

                self.assertIs(frame.decision, TransmitDecision.WITHHOLD)
                self.assertTrue(frame.reason.startswith("authority_refused:"))
                self.assertEqual(calls, [])

    def test_legacy_authority_blocks_the_companion_sender(self) -> None:
        send, calls = active(
            environment={
                AUTHORITY_ENV_VAR: "legacy_tracking",
                "SWARM_TRACKING_OFFBOARD_ENABLED": "true",
            }
        )
        frame = send.step(good_preview())

        self.assertEqual(calls, [])
        self.assertIn("legacy_tracking", frame.reason)

    def test_uav02_at_wrong_system_id_is_refused(self) -> None:
        """UAV-02 at its own system id (2) IS authorized -- see
        test_uav02_at_its_own_system_id_is_authorized -- so this covers only
        the system ids that must still be refused."""
        for system_id in (1, 99):
            with self.subTest(system_id=system_id):
                send, calls = active(drone_id="UAV-02", px4_system_id=system_id)
                frame = send.step(good_preview())

                self.assertEqual(calls, [])
                self.assertIs(frame.decision, TransmitDecision.WITHHOLD)

    def test_uav02_at_its_own_system_id_is_authorized(self) -> None:
        """Added for TWO_UAV_ACTIVE_SITL_FLIGHT. The preview must be built
        for UAV-02 too -- good_preview() is UAV-01's -- or this would pass
        for the wrong reason (preview_drone_mismatch masking the
        authorization question this test exists to answer)."""
        send, calls = active(drone_id="UAV-02", px4_system_id=2)
        matching_preview = preview(
            shadow_sender(drone_id="UAV-02", px4_system_id=2),
            status(velocity=(0.6, 1.2, 0.3)),
        )

        frame = send.step(matching_preview)

        self.assertIs(frame.decision, TransmitDecision.TRANSMIT)
        self.assertEqual(len(calls), 1)

    def test_broadcast_system_id_is_refused(self) -> None:
        send, calls = active(px4_system_id=0)
        frame = send.step(good_preview())

        self.assertEqual(calls, [])
        self.assertIn("broadcast_system_id_forbidden", frame.reason)

    def test_preview_built_for_another_drone_is_refused(self) -> None:
        """Previews are plain dataclasses; one drone's must not be flown by
        another's sender, or a peer's CBF solution commands this airframe."""
        send, calls = active()
        foreign = preview(
            shadow_sender(drone_id="UAV-02", px4_system_id=2),
            status(velocity=(0.6, 1.2, 0.3)),
        )

        frame = send.step(foreign)

        self.assertEqual(calls, [])
        self.assertIn("preview_drone_mismatch", frame.reason)

    def test_authority_is_re_resolved_every_frame(self) -> None:
        environment = dict(COMPANION)
        send, calls = active(environment=environment)
        send.step(good_preview(), now_monotonic_s=100.0)

        environment[AUTHORITY_ENV_VAR] = "disabled"
        frame = send.step(good_preview(), now_monotonic_s=100.05)

        self.assertEqual(len(calls), 1)
        self.assertIs(frame.decision, TransmitDecision.WITHHOLD)


class PreviewGatingTests(unittest.TestCase):
    def test_preview_rejected_by_the_validator_is_never_transmitted(self) -> None:
        send, calls = active()
        rejected = preview(shadow_sender(), status(output_valid=False))

        send.step(rejected)

        self.assertEqual(calls, [])

    def test_every_command_integrity_inhibit_withholds_entirely(self) -> None:
        """Warmup relaxes VEHICLE_NOT_READY only. An untrustworthy command is
        not made trustworthy by needing to bootstrap OFFBOARD."""
        cases = {
            "command_stale": preview(
                shadow_sender(), status(), now=101.0, command_at=100.0
            ),
            "malformed_command": preview(
                shadow_sender(), status(velocity=(float("inf"), 0.0, 0.0))
            ),
            "command_validator_rejected": preview(
                shadow_sender(), status(output_valid=False)
            ),
        }
        for reason, source in cases.items():
            with self.subTest(reason=reason):
                send, calls = active()
                send.step(source, now_monotonic_s=200.0)

                self.assertEqual(calls, [])


class WarmupTests(unittest.TestCase):
    """PX4 refuses OFFBOARD unless it is already receiving setpoints, so the
    sender must stream before the vehicle is armed or in OFFBOARD."""

    def test_disarmed_vehicle_receives_a_warmup_setpoint(self) -> None:
        send, calls = active()
        not_armed = preview(shadow_sender(), status(velocity=(0.6, 1.2, 0.3)), armed=False)

        frame = send.step(not_armed)

        self.assertFalse(not_armed.valid)
        self.assertIs(frame.decision, TransmitDecision.TRANSMIT_WARMUP)
        self.assertEqual(frame.reason, "px4_not_armed")
        self.assertEqual(len(calls), 1)

    def test_not_in_offboard_receives_a_warmup_setpoint(self) -> None:
        send, calls = active()
        wrong_mode = preview(shadow_sender(), status(velocity=(0.6, 1.2, 0.3)), mode=3)

        frame = send.step(wrong_mode)

        self.assertIs(frame.decision, TransmitDecision.TRANSMIT_WARMUP)
        self.assertEqual(frame.reason, "px4_not_in_offboard")
        self.assertEqual(len(calls), 1)

    def test_warmup_never_carries_the_real_velocity(self) -> None:
        """A real velocity streamed at a vehicle not yet in OFFBOARD would
        take effect the instant the mode engaged."""
        send, calls = active()
        send.step(
            preview(shadow_sender(), status(velocity=(0.6, 1.2, 0.3)), armed=False)
        )

        _, _, _, _, _, *rest = calls[0]
        self.assertEqual(rest, [0.0] * 11)

    def test_warmup_still_requires_full_authorization(self) -> None:
        for overrides in (
            {"explicit_opt_in": False},
            {"environment": {AUTHORITY_ENV_VAR: "disabled"}},
            # UAV-02 at its own system id (2) is authorized (see
            # test_uav02_at_its_own_system_id_is_authorized); system id 1 is
            # not, and stays a genuine example of "unauthorized" here.
            {"drone_id": "UAV-02", "px4_system_id": 1},
        ):
            with self.subTest(**overrides):
                send, calls = active(**overrides)
                frame = send.step(preview(shadow_sender(), status(), armed=False))

                self.assertIs(frame.decision, TransmitDecision.WITHHOLD)
                self.assertEqual(calls, [])

    def test_warmup_stops_at_an_abort(self) -> None:
        send, calls = active()
        frame = send.step(
            preview(shadow_sender(), status(), armed=False),
            now_monotonic_s=100.0,
            reported_conditions=["mavlink_connection_loss"],
        )

        self.assertIs(frame.decision, TransmitDecision.WITHHOLD)
        self.assertEqual(calls, [])

    def test_stream_metrics_measure_the_wire_not_the_loop(self) -> None:
        """A withheld frame is not part of the stream PX4 sees."""
        send, calls = active()
        now = 100.0
        for _ in range(40):
            send.step(
                preview(shadow_sender(), status(), armed=False), now_monotonic_s=now
            )
            now += 0.05
        state = send.status()

        self.assertEqual(len(calls), 40)
        self.assertEqual(state["warmup_transmit_count"], 40)
        self.assertAlmostEqual(state["stream_duration_s"], 1.95, places=2)
        self.assertLessEqual(state["max_transmit_gap_s"], 0.051)


class WireConsistencyTests(unittest.TestCase):
    def test_mutated_frame_is_refused(self) -> None:
        send, calls = active()
        tampered = dataclasses.replace(good_preview(), px4_frame=MAV_FRAME_LOCAL_NED + 1)

        frame = send.step(tampered)

        self.assertEqual(calls, [])
        self.assertIn("unexpected_frame", frame.reason)

    def test_mutated_type_mask_is_refused(self) -> None:
        send, calls = active()
        tampered = dataclasses.replace(good_preview(), px4_type_mask=1507)

        frame = send.step(tampered)

        self.assertEqual(calls, [])
        self.assertIn("unexpected_type_mask", frame.reason)

    def test_inconsistent_conversion_is_refused(self) -> None:
        send, calls = active()
        tampered = dataclasses.replace(
            good_preview(), output_velocity_ned_m_s=(9.0, 9.0, -0.3)
        )

        frame = send.step(tampered)

        self.assertEqual(calls, [])
        self.assertIn("frame_conversion_inconsistent", frame.reason)

    def test_inverted_vertical_axis_aborts_before_any_other_gate(self) -> None:
        send, calls = active()
        # ENU up = +0.3 must give NED down = -0.3; +0.3 is a descent command.
        inverted = dataclasses.replace(
            good_preview(), output_velocity_ned_m_s=(1.2, 0.6, 0.3)
        )

        frame = send.step(inverted)

        self.assertEqual(frame.abort_condition, "vertical_velocity_sign_mismatch")
        self.assertEqual(calls, [])

    def test_zero_vertical_paired_with_non_zero_down_is_a_mismatch(self) -> None:
        send, calls = active()
        level = preview(shadow_sender(), status(velocity=(0.6, 1.2, 0.0)))
        inverted = dataclasses.replace(level, output_velocity_ned_m_s=(1.2, 0.6, 0.5))

        frame = send.step(inverted)

        self.assertEqual(frame.abort_condition, "vertical_velocity_sign_mismatch")
        self.assertEqual(calls, [])


class WirePayloadTests(unittest.TestCase):
    def test_payload_matches_set_position_target_local_ned_send(self) -> None:
        send, calls = active()
        source = good_preview()
        send.step(source)

        (
            time_boot_ms,
            target_system,
            target_component,
            frame_id,
            type_mask,
            x,
            y,
            z,
            vx,
            vy,
            vz,
            afx,
            afy,
            afz,
            yaw,
            yaw_rate,
        ) = calls[0]

        self.assertEqual(time_boot_ms, source.boot_time_ms)
        self.assertEqual(target_system, 1)
        self.assertEqual(target_component, MAV_COMP_ID_AUTOPILOT1)
        self.assertEqual(frame_id, MAV_FRAME_LOCAL_NED)
        self.assertEqual(type_mask, OFFBOARD_VELOCITY_TYPE_MASK)
        self.assertEqual((x, y, z), (0.0, 0.0, 0.0))
        self.assertEqual((afx, afy, afz), (0.0, 0.0, 0.0))
        self.assertEqual(yaw, 0.0)
        self.assertEqual(yaw_rate, source.yaw_rate_rad_s)
        # ENU (east=0.6, north=1.2, up=0.3) -> NED (north, east, down).
        self.assertAlmostEqual(vx, 1.2)
        self.assertAlmostEqual(vy, 0.6)
        self.assertAlmostEqual(vz, -0.3)

    def test_zero_hold_payload_carries_no_residual_velocity(self) -> None:
        """The preview that triggered the abort may be untrustworthy, so the
        hold setpoint must not be built from its fields."""
        send, calls = active()
        send.step(
            good_preview(), reported_conditions=["geofence_violation_or_risk"]
        )

        _, target_system, _, frame_id, type_mask, *rest = calls[0]
        self.assertEqual(target_system, 1)
        self.assertEqual(frame_id, MAV_FRAME_LOCAL_NED)
        self.assertEqual(type_mask, OFFBOARD_VELOCITY_TYPE_MASK)
        self.assertEqual(rest, [0.0] * 11)


class InertWithoutSinkTests(unittest.TestCase):
    def test_a_sender_without_a_sink_evaluates_but_cannot_transmit(self) -> None:
        send = ActiveOffboardSetpointSender(
            "UAV-01", 1, explicit_opt_in=True, environment=dict(COMPANION)
        )
        frame = send.step(good_preview())

        self.assertIs(frame.decision, TransmitDecision.TRANSMIT)
        self.assertFalse(frame.transmitted)
        self.assertEqual(frame.reason, "no_transmit_sink")
        self.assertEqual(send.transmit_count, 0)

    def test_default_construction_has_no_sink_and_no_opt_in(self) -> None:
        send = ActiveOffboardSetpointSender("UAV-01", 1)
        state = send.status()

        self.assertFalse(state["transmit_sink_attached"])
        self.assertFalse(state["explicit_opt_in"])
        self.assertEqual(state["arm_command_count"], 0)


class StructuralTests(unittest.TestCase):
    MODULE = Path("active_offboard_setpoint_sender.py")

    def imports(self) -> set[str]:
        found: set[str] = set()
        for node in ast.walk(ast.parse(self.MODULE.read_text(encoding="utf-8"))):
            if isinstance(node, ast.Import):
                found.update(alias.name.split(".")[0] for alias in node.names)
            elif isinstance(node, ast.ImportFrom) and node.module:
                found.add(node.module.split(".")[0])
        return found

    def test_imports_no_transport_library(self) -> None:
        forbidden = {"pymavlink", "socket", "paho", "requests", "serial", "asyncio"}

        self.assertEqual(self.imports() & forbidden, set())

    def test_cannot_reach_the_safety_stack_it_must_not_bypass(self) -> None:
        """Structural proof of the plan's must_not list: the module has no
        reference to CBF, the supervisor, or the nominal controller, so no
        expression in it can produce a pre-CBF velocity."""
        forbidden = {"cbf_command_gate", "emergency_supervisor", "companion_safety",
                     "formation_controller"}

        self.assertEqual(self.imports() & forbidden, set())

    def test_no_arm_or_mode_command_is_constructed(self) -> None:
        source = self.MODULE.read_text(encoding="utf-8")

        for token in (
            "command_long_send",
            "MAV_CMD_COMPONENT_ARM_DISARM",
            "MAV_CMD_NAV_TAKEOFF",
            "MAV_CMD_NAV_LAND",
            "set_mode_send",
        ):
            with self.subTest(token=token):
                self.assertNotIn(token, source)

    def test_opt_in_is_not_readable_from_the_environment(self) -> None:
        """An env var would make first-flight authorization a deploy-time
        flag flip rather than a reviewed code change."""
        source = self.MODULE.read_text(encoding="utf-8")

        self.assertNotIn("os.environ", source)
        self.assertNotIn("getenv", source)


class BridgeIntegrationTests(unittest.TestCase):
    """The sender runs in the live bridge loop with no sink."""

    def worker(self):
        from mavlink_manual_bridge import (
            MavlinkWorker,
            build_active_offboard_setpoint_sender,
        )

        class FakeWorker:
            drone_id = "UAV-01"
            expected_system_id = 1
            last_px4_armed = False
            active_sender_reset_count = 0

            def transmit_active_offboard_setpoint(self, *_args):
                raise AssertionError("no connection in this fixture")

            def request_position_mode(self):
                raise AssertionError("no connection in this fixture")

            active_offboard_sender = build_active_offboard_setpoint_sender("UAV-01", 1)

        return FakeWorker(), MavlinkWorker

    def test_the_bridge_builds_a_sender_with_no_sink_and_no_opt_in(self) -> None:
        from mavlink_manual_bridge import build_active_offboard_setpoint_sender

        send = build_active_offboard_setpoint_sender("UAV-01", 1)
        state = send.status()

        self.assertFalse(state["transmit_sink_attached"])
        self.assertFalse(state["explicit_opt_in"])
        self.assertEqual(state["transmit_count"], 0)

    def test_a_latched_sender_is_replaced_while_disarmed(self) -> None:
        """A latch is scoped to one flight. Disarmed means there is no flight,
        so leaving it frozen would silence the shadow output for the session."""
        worker, MavlinkWorker = self.worker()
        worker.active_offboard_sender.step(
            good_preview(),
            now_monotonic_s=100.0,
            reported_conditions=["mavlink_connection_loss"],
        )
        original = worker.active_offboard_sender
        self.assertIsNotNone(original.latched_abort)

        MavlinkWorker.reset_latched_active_sender(worker, ())

        self.assertIsNot(worker.active_offboard_sender, original)
        self.assertIsNone(worker.active_offboard_sender.latched_abort)
        self.assertEqual(worker.active_sender_reset_count, 1)

    def test_a_latched_sender_is_never_replaced_while_armed(self) -> None:
        worker, MavlinkWorker = self.worker()
        worker.last_px4_armed = True
        worker.active_offboard_sender.step(
            good_preview(),
            now_monotonic_s=100.0,
            reported_conditions=["mavlink_connection_loss"],
        )
        original = worker.active_offboard_sender

        MavlinkWorker.reset_latched_active_sender(worker, ())

        self.assertIs(worker.active_offboard_sender, original)
        self.assertEqual(worker.active_sender_reset_count, 0)

    def test_unknown_armed_state_does_not_clear_a_latch(self) -> None:
        """None means 'not yet known', which is not a safe basis for clearing
        an abort."""
        worker, MavlinkWorker = self.worker()
        worker.last_px4_armed = None
        worker.active_offboard_sender.step(
            good_preview(),
            now_monotonic_s=100.0,
            reported_conditions=["mavlink_connection_loss"],
        )
        original = worker.active_offboard_sender

        MavlinkWorker.reset_latched_active_sender(worker, ())

        self.assertIs(worker.active_offboard_sender, original)

    def test_a_persistent_fault_does_not_churn_the_sender(self) -> None:
        """Resetting while the cause is still active would rebuild the sender
        at 20 Hz and destroy the frame history that explains the abort."""
        worker, MavlinkWorker = self.worker()
        worker.active_offboard_sender.step(
            good_preview(),
            now_monotonic_s=100.0,
            reported_conditions=["mavlink_connection_loss"],
        )
        original = worker.active_offboard_sender

        for _ in range(20):
            MavlinkWorker.reset_latched_active_sender(
                worker, ("mavlink_connection_loss",)
            )

        self.assertIs(worker.active_offboard_sender, original)
        self.assertEqual(worker.active_sender_reset_count, 0)

    def test_an_unlatched_sender_is_never_replaced(self) -> None:
        worker, MavlinkWorker = self.worker()
        worker.active_offboard_sender.step(good_preview(), now_monotonic_s=100.0)
        original = worker.active_offboard_sender

        MavlinkWorker.reset_latched_active_sender(worker, ())

        self.assertIs(worker.active_offboard_sender, original)
        self.assertEqual(worker.active_sender_reset_count, 0)


class LegacyModeConflictTests(unittest.TestCase):
    """The first armed flight held OFFBOARD for 0.5 s before the bridge's own
    idle loop pulled it back to POSCTL. These pin the resolution."""

    def worker(self, **overrides):
        from mavlink_manual_bridge import (
            PX4_MAIN_MODE_OFFBOARD,
            MavlinkWorker,
            build_active_offboard_setpoint_sender,
        )

        class FakeWorker:
            drone_id = "UAV-01"
            expected_system_id = 1
            last_px4_main_mode = PX4_MAIN_MODE_OFFBOARD
            last_px4_sub_mode = None
            offboard_mode_requested = False
            visual_follow_mode_requested = False
            last_mode_request_monotonic = 0.0
            requested: list[str] = []

            def request_px4_main_mode(self, main_mode, label):
                type(self).requested.append(label)
                return True

            # The real implementation, so the test exercises the shipped
            # predicate rather than a copy of it.
            companion_holds_offboard = MavlinkWorker.companion_holds_offboard

        worker = FakeWorker()
        FakeWorker.requested = []
        with mock.patch.dict(
            "os.environ", {AUTHORITY_ENV_VAR: "companion_safety"}, clear=True
        ):
            worker.active_offboard_sender = build_active_offboard_setpoint_sender(
                "UAV-01", 1, lambda *a: None
            )
        for key, value in overrides.items():
            setattr(worker, key, value)
        return worker, MavlinkWorker

    def test_companion_driven_offboard_is_not_pulled_back(self) -> None:
        worker, MavlinkWorker = self.worker()

        self.assertTrue(MavlinkWorker.companion_holds_offboard(worker))
        MavlinkWorker.request_position_mode(worker)

        self.assertEqual(worker.requested, [])

    def test_a_latched_abort_hands_offboard_back_to_the_legacy_recovery(self) -> None:
        """The backstop: an aborted companion must lose OFFBOARD, not keep it."""
        worker, MavlinkWorker = self.worker()
        worker.active_offboard_sender.step(
            good_preview(),
            now_monotonic_s=100.0,
            reported_conditions=["mavlink_connection_loss"],
        )

        self.assertFalse(MavlinkWorker.companion_holds_offboard(worker))
        MavlinkWorker.request_position_mode(worker)

        self.assertEqual(worker.requested, ["POSITION"])

    def test_offboard_without_a_companion_sender_still_recovers(self) -> None:
        """Unchanged legacy behaviour when no companion sender exists."""
        worker, MavlinkWorker = self.worker(active_offboard_sender=None)

        self.assertFalse(MavlinkWorker.companion_holds_offboard(worker))
        MavlinkWorker.request_position_mode(worker)

        self.assertEqual(worker.requested, ["POSITION"])

    def test_an_unauthorized_sender_does_not_hold_offboard(self) -> None:
        """UAV-02 at system id 2 is authorized (TWO_UAV_ACTIVE_SITL_FLIGHT),
        so system id 1 -- its wrong id -- is what proves the identity gate
        still refuses a genuinely unauthorized pairing."""
        from mavlink_manual_bridge import (
            MavlinkWorker,
            build_active_offboard_setpoint_sender,
        )

        worker, _ = self.worker()
        with mock.patch.dict(
            "os.environ", {AUTHORITY_ENV_VAR: "companion_safety"}, clear=True
        ):
            worker.active_offboard_sender = build_active_offboard_setpoint_sender(
                "UAV-02", 1, lambda *a: None
            )

        self.assertFalse(MavlinkWorker.companion_holds_offboard(worker))
        MavlinkWorker.request_position_mode(worker)

        self.assertEqual(worker.requested, ["POSITION"])

    def test_the_position_mode_abort_action_is_wired_in_the_bridge(self) -> None:
        """It was a no-op in the first flight: the sender's callback was
        never passed, so REQUEST_POSITION_MODE did nothing."""
        from mavlink_manual_bridge import build_active_offboard_setpoint_sender

        called = []
        with mock.patch.dict(
            "os.environ", {AUTHORITY_ENV_VAR: "companion_safety"}, clear=True
        ):
            send = build_active_offboard_setpoint_sender(
                "UAV-01", 1, lambda *a: None, lambda: called.append(1)
            )
        send.step(
            preview(
                shadow_sender(),
                status(
                    stage=EmergencyStage.RECOMMEND_POSITION_HOLD,
                    recommended_px4_mode="POSITION_HOLD",
                ),
            ),
            now_monotonic_s=100.0,
        )

        self.assertEqual(called, [1])


class FrameHistoryTests(unittest.TestCase):
    def test_history_is_bounded(self) -> None:
        """The sender runs at 20 Hz in a process that stays up for days; an
        unbounded list would be a slow leak."""
        from active_offboard_setpoint_sender import MAX_RETAINED_FRAMES

        send, _ = active()
        now = 100.0
        for _ in range(MAX_RETAINED_FRAMES + 50):
            send.step(good_preview(), now_monotonic_s=now)
            now += 0.05

        self.assertEqual(len(send.frames), MAX_RETAINED_FRAMES)
        # Cumulative counters must not be truncated with the history.
        self.assertEqual(send.sequence, MAX_RETAINED_FRAMES + 50)
        self.assertEqual(send.transmit_count, MAX_RETAINED_FRAMES + 50)


class FrameRecordTests(unittest.TestCase):
    def test_withheld_frames_are_recorded_too(self) -> None:
        """A log that only records transmissions cannot tell 'healthy and
        quiet' from 'aborted and silent'."""
        send, _ = active(explicit_opt_in=False)
        now = 100.0
        for _ in range(5):
            send.step(good_preview(), now_monotonic_s=now)
            now += 0.05

        self.assertEqual(len(send.frames), 5)
        self.assertEqual(send.withheld_count, 5)
        for frame in send.frames:
            self.assertIn("decision", frame.as_dict())
            self.assertFalse(frame.as_dict()["transmitted"])


if __name__ == "__main__":
    unittest.main()
