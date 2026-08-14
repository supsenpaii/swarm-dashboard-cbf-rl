from __future__ import annotations

import math
import unittest
from unittest import mock

from cbf_command_gate import CbfCommand
from companion_safety import CompanionSafetyStatus
from emergency_supervisor import EmergencyDecision, EmergencyStage
from offboard_setpoint_sender import ShadowOffboardSetpointSender
from one_uav_active_readiness import (
    ABORT_MATRIX,
    FIRST_ACTIVE_FLIGHT_DRONE_ID,
    PX4_FACTS,
    STATE_SEQUENCE,
    AbortAction,
    ActiveOffboardSetpointSenderPlan,
    FlightEnvelope,
    ReadinessState,
    WarmupContract,
    abort_rule_for,
    evaluate_velocity_readiness,
    is_authorized_for_first_active_flight,
    transition_guard,
)


PX4_MAIN_MODE_OFFBOARD = 6


def status(velocity=(0.0, 0.0, 0.0), stage=EmergencyStage.NORMAL) -> CompanionSafetyStatus:
    return CompanionSafetyStatus(
        drone_id="UAV-01",
        nominal_velocity_enu_m_s=velocity,
        nominal_active=True,
        nominal_reason="tracking_slot",
        nominal_position_error_m=None,
        command=CbfCommand("UAV-01", velocity, True, "cbf_filtered", 1.0, 0.0),
        emergency=EmergencyDecision(
            stage=stage,
            velocity_enu_m_s=velocity,
            active=stage is not EmergencyStage.NORMAL,
            reason="cbf_feasible",
            recommended_px4_mode=None,
            time_in_stage_s=0.0,
        ),
        output_velocity_enu_m_s=velocity,
        output_valid=True,
        peer_ids_used=("UAV-02",),
        peer_ids_missing=(),
    )


class PositiveValidPathTests(unittest.TestCase):
    """Phase 2: prove that the ONLY thing blocking a live valid preview is the
    deliberate disarmed state -- not a defect, and not something that needs a
    validity rule weakened to reach."""

    def sender(self) -> ShadowOffboardSetpointSender:
        return ShadowOffboardSetpointSender(
            drone_id="UAV-01",
            px4_system_id=1,
            maximum_velocity_m_s=2.0,
            maximum_command_age_s=0.5,
        )

    def test_disarmed_is_the_only_blocker_in_the_live_configuration(self) -> None:
        """Same command, same freshness, same authority as live -- only the
        armed/mode inputs differ."""
        send = self.sender()
        command = status((0.5, 1.0, 0.25))

        live_like = send.preview(
            command,
            now_monotonic_s=100.0,
            command_monotonic_s=100.0,
            px4_main_mode=2,  # POSCTL, as observed live
            px4_armed=False,  # as observed live
            px4_offboard_main_mode=PX4_MAIN_MODE_OFFBOARD,
        )
        self.assertFalse(live_like.valid)
        self.assertEqual(live_like.inhibit_reason, "px4_not_armed")
        self.assertEqual(live_like.inhibit_category, "vehicle_not_ready")

        future_ready = send.preview(
            command,
            now_monotonic_s=100.0,
            command_monotonic_s=100.0,
            px4_main_mode=PX4_MAIN_MODE_OFFBOARD,
            px4_armed=True,
            px4_offboard_main_mode=PX4_MAIN_MODE_OFFBOARD,
        )
        self.assertTrue(future_ready.valid)
        self.assertFalse(future_ready.inhibited)
        self.assertEqual(future_ready.inhibit_reason, "")

    def test_valid_path_produces_the_correct_ned_setpoint(self) -> None:
        result = self.sender().preview(
            status((0.5, 1.0, 0.25)),
            now_monotonic_s=100.0,
            command_monotonic_s=100.0,
            px4_main_mode=PX4_MAIN_MODE_OFFBOARD,
            px4_armed=True,
            px4_offboard_main_mode=PX4_MAIN_MODE_OFFBOARD,
        )

        self.assertTrue(result.valid)
        # ENU (0.5 east, 1.0 north, 0.25 up) -> NED (1.0 north, 0.5 east, -0.25 down)
        self.assertEqual(result.output_velocity_ned_m_s, (1.0, 0.5, -0.25))
        self.assertEqual(result.px4_frame, PX4_FACTS["nav_state_posctl"]["value"] - 1)  # MAV_FRAME_LOCAL_NED == 1
        self.assertEqual(result.px4_type_mask, 1479)
        self.assertEqual(result.px4_target_system, 1)

    def test_valid_path_still_never_transmits(self) -> None:
        """Reaching valid=True must not make the shadow sender transmittable."""
        send = self.sender()
        result = send.preview(
            status((0.5, 1.0, 0.25)),
            now_monotonic_s=100.0,
            command_monotonic_s=100.0,
            px4_main_mode=PX4_MAIN_MODE_OFFBOARD,
            px4_armed=True,
            px4_offboard_main_mode=PX4_MAIN_MODE_OFFBOARD,
        )

        self.assertTrue(result.valid)
        self.assertFalse(result.transmit_allowed)
        self.assertFalse(result.transmit_attempted)
        self.assertEqual(send.transmit_attempt_count, 0)

    def test_validity_rules_were_not_weakened(self) -> None:
        """Even with PX4 ready, an unsafe command is still refused."""
        send = self.sender()
        for velocity, expected in (
            ((float("nan"), 0.0, 0.0), "malformed_command"),
            ((99.0, 0.0, 0.0), "command_exceeds_velocity_limit"),
        ):
            with self.subTest(velocity=velocity):
                result = send.preview(
                    status(velocity),
                    now_monotonic_s=100.0,
                    command_monotonic_s=100.0,
                    px4_main_mode=PX4_MAIN_MODE_OFFBOARD,
                    px4_armed=True,
                    px4_offboard_main_mode=PX4_MAIN_MODE_OFFBOARD,
                )
                self.assertFalse(result.valid)
                self.assertEqual(result.inhibit_reason, expected)


class AuthorizationTests(unittest.TestCase):
    def test_only_uav01_with_system_id_1_and_opt_in_is_authorized(self) -> None:
        allowed, reason = is_authorized_for_first_active_flight("UAV-01", 1, True)
        self.assertTrue(allowed)
        self.assertEqual(reason, "")

    def test_uav02_can_never_be_authorized(self) -> None:
        for system_id in (1, 2, 0, 99):
            with self.subTest(system_id=system_id):
                allowed, reason = is_authorized_for_first_active_flight(
                    "UAV-02", system_id, True
                )
                self.assertFalse(allowed)
                self.assertTrue(reason.startswith("drone_not_authorized"))

    def test_broadcast_system_id_zero_is_refused(self) -> None:
        """MAVLink target_system 0 is a broadcast accepted by EVERY PX4
        instance (mavlink_receiver.cpp), so it must never be authorized."""
        allowed, reason = is_authorized_for_first_active_flight("UAV-01", 0, True)
        self.assertFalse(allowed)
        self.assertIn("system_id", reason)

    def test_opt_in_is_required(self) -> None:
        allowed, reason = is_authorized_for_first_active_flight("UAV-01", 1, False)
        self.assertFalse(allowed)
        self.assertEqual(reason, "explicit_opt_in_absent")

    def test_authorized_drone_is_hardcoded_not_configurable(self) -> None:
        """A config typo must not be able to authorize a different vehicle."""
        with mock.patch.dict(
            "os.environ", {"SWARM_FIRST_ACTIVE_DRONE_ID": "UAV-02"}, clear=False
        ):
            allowed, _ = is_authorized_for_first_active_flight("UAV-02", 2, True)
            self.assertFalse(allowed)
        self.assertEqual(FIRST_ACTIVE_FLIGHT_DRONE_ID, "UAV-01")


class EnvelopeTests(unittest.TestCase):
    def test_envelope_never_exceeds_the_cbf_limit(self) -> None:
        with mock.patch.dict(
            "os.environ", {"SWARM_CBF_MAXIMUM_VELOCITY_M_S": "2.0"}, clear=False
        ):
            envelope = FlightEnvelope.from_environment()

        self.assertLessEqual(envelope.maximum_horizontal_velocity_m_s, 2.0)
        self.assertLessEqual(envelope.maximum_vertical_velocity_m_s, 2.0)
        # Vertical is deliberately tighter than horizontal for a first flight.
        self.assertLess(
            envelope.maximum_vertical_velocity_m_s,
            envelope.maximum_horizontal_velocity_m_s,
        )

    def test_hover_altitude_is_inside_the_geofence(self) -> None:
        envelope = FlightEnvelope.from_environment()

        self.assertGreaterEqual(envelope.hover_altitude_m, envelope.geofence_min_enu_m[2])
        self.assertLessEqual(envelope.hover_altitude_m, envelope.geofence_max_enu_m[2])

    def test_ages_match_the_existing_freshness_contract(self) -> None:
        with mock.patch.dict(
            "os.environ", {"SWARM_PEER_STATE_MAX_AGE_MS": "500"}, clear=False
        ):
            envelope = FlightEnvelope.from_environment()

        self.assertEqual(envelope.maximum_state_age_s, 0.5)
        self.assertEqual(envelope.maximum_command_age_s, 0.5)


class WarmupTests(unittest.TestCase):
    def test_project_warmup_is_stricter_than_px4_requirement(self) -> None:
        warmup = WarmupContract()

        self.assertEqual(warmup.px4_setpoint_timeout_s, PX4_FACTS["offboard_setpoint_timeout_s"]["value"])
        self.assertGreater(warmup.minimum_duration_s, warmup.px4_setpoint_timeout_s)
        self.assertLess(warmup.maximum_gap_s, warmup.px4_setpoint_timeout_s)

    def test_warmup_guard_rejects_a_stream_gap(self) -> None:
        envelope = FlightEnvelope.from_environment()
        warmup = WarmupContract()

        ok, reason = transition_guard(
            ReadinessState.SETPOINT_WARMUP,
            {
                "warmup_duration_s": 10.0,
                "valid_sample_count": 200,
                "max_gap_s": 1.5,
                "all_previews_valid": True,
            },
            envelope,
            warmup,
        )

        self.assertFalse(ok)
        self.assertEqual(reason, "warmup_stream_gap_exceeded")

    def test_warmup_guard_rejects_any_invalid_preview(self) -> None:
        ok, reason = transition_guard(
            ReadinessState.SETPOINT_WARMUP,
            {
                "warmup_duration_s": 10.0,
                "valid_sample_count": 200,
                "max_gap_s": 0.05,
                "all_previews_valid": False,
            },
            FlightEnvelope.from_environment(),
            WarmupContract(),
        )

        self.assertFalse(ok)
        self.assertEqual(reason, "warmup_contains_invalid_preview")


class TransitionGuardTests(unittest.TestCase):
    def test_precheck_requires_disarmed(self) -> None:
        ok, reason = transition_guard(
            ReadinessState.PRECHECK,
            {
                "preflight_checks_pass": True,
                "local_position_invalid_false": True,
                "companion_safety_running": True,
                "peer_state_fresh": True,
                "armed": True,
            },
            FlightEnvelope.from_environment(),
            WarmupContract(),
        )

        self.assertFalse(ok)
        self.assertEqual(reason, "precheck_requires_disarmed")

    def test_ascent_guard_rejects_velocity_above_envelope(self) -> None:
        envelope = FlightEnvelope.from_environment()
        ok, reason = transition_guard(
            ReadinessState.CONTROLLED_ASCENT,
            {
                "altitude_m": envelope.hover_altitude_m,
                "vertical_velocity_m_s": envelope.maximum_vertical_velocity_m_s + 1.0,
            },
            envelope,
            WarmupContract(),
        )

        self.assertFalse(ok)
        self.assertEqual(reason, "ascent_velocity_exceeds_envelope")

    def test_exit_offboard_requires_actually_leaving_offboard(self) -> None:
        ok, reason = transition_guard(
            ReadinessState.EXIT_OFFBOARD,
            {"nav_state": PX4_FACTS["nav_state_offboard"]["value"]},
            FlightEnvelope.from_environment(),
            WarmupContract(),
        )

        self.assertFalse(ok)
        self.assertEqual(reason, "still_in_offboard")

    def test_every_state_in_the_sequence_has_a_guard(self) -> None:
        envelope = FlightEnvelope.from_environment()
        for state in STATE_SEQUENCE:
            with self.subTest(state=state):
                _, reason = transition_guard(state, {}, envelope, WarmupContract())
                self.assertNotEqual(reason, f"unknown_state:{state}")


class AbortMatrixTests(unittest.TestCase):
    REQUIRED = (
        "self_telemetry_stale",
        "command_stale",
        "setpoint_stream_gap",
        "px4_rejects_offboard",
        "px4_exits_offboard_unexpectedly",
        "cbf_invalid_or_infeasible",
        "emergency_supervisor_hold",
        "emergency_recommends_position_hold",
        "emergency_recommends_rtl_or_land",
        "geofence_violation_or_risk",
        "vertical_velocity_sign_mismatch",
        "nan_or_inf_command",
        "mavlink_connection_loss",
        "companion_safety_loop_stops",
    )

    def test_every_required_condition_is_covered(self) -> None:
        for condition in self.REQUIRED:
            with self.subTest(condition=condition):
                self.assertIsNotNone(abort_rule_for(condition))

    def test_no_rule_claims_unimplemented_autonomous_rtl_or_land(self) -> None:
        """The project has no autonomous RTL/Land. No abort rule may promise
        one -- claiming behaviour that does not exist is the dangerous
        failure mode here."""
        for rule in ABORT_MATRIX:
            with self.subTest(condition=rule.condition):
                self.assertNotIn("RTL", rule.px4_policy.replace("RTL/Land NOT implemented", ""))
                self.assertIn(
                    rule.immediate_action,
                    (
                        AbortAction.REFUSE_TO_PROCEED,
                        AbortAction.STOP_SENDING_SETPOINTS,
                        AbortAction.SEND_ZERO_VELOCITY,
                        AbortAction.REQUEST_POSITION_MODE,
                    ),
                )

    def test_terminal_emergency_requires_manual_intervention(self) -> None:
        rule = abort_rule_for("emergency_recommends_rtl_or_land")
        assert rule is not None

        self.assertTrue(rule.manual_intervention_required)
        self.assertFalse(rule.recovery_allowed)
        self.assertIn("NOT implemented", rule.px4_policy)

    def test_stream_loss_relies_on_the_real_px4_failsafe(self) -> None:
        rule = abort_rule_for("setpoint_stream_gap")
        assert rule is not None

        self.assertEqual(rule.immediate_action, AbortAction.STOP_SENDING_SETPOINTS)
        # COM_OBL_RC_ACT=0 -> Position mode. Read from PX4 source, not assumed.
        self.assertEqual(PX4_FACTS["offboard_loss_action"]["value"], 0)
        self.assertEqual(PX4_FACTS["offboard_loss_action"]["meaning"], "Position mode")


class ActiveSenderPlanTests(unittest.TestCase):
    def test_plan_names_the_module_that_implements_it(self) -> None:
        plan = ActiveOffboardSetpointSenderPlan()

        self.assertTrue(plan.implemented)
        self.assertIn("active_offboard_setpoint_sender.py", " ".join(plan.layering))
        self.assertNotIn("NOT IMPLEMENTED", " ".join(plan.layering))

    def test_plan_forbids_bypassing_the_safety_stack(self) -> None:
        plan = ActiveOffboardSetpointSenderPlan()

        for forbidden in ("bypass CBF", "bypass EmergencySupervisor"):
            self.assertIn(forbidden, plan.must_not)

    def test_only_the_authorized_vehicle_ever_receives_a_transmit_sink(self) -> None:
        """The sink is the capability; the builder is where it is granted.

        Tested behaviourally rather than by scanning source, because the
        question is no longer "does anyone pass a sink" -- the flight gate
        does -- but "can an unauthorized vehicle end up holding one".

        `build_active_offboard_setpoint_sender` now gates on
        two_uav_active_readiness rather than this module's own narrower
        gate (see active_offboard_setpoint_sender._authorization), so UAV-02
        at its own system id is a legitimate grant here, not a leak. This
        test stays in this file because it exercises the real bridge
        builder end to end, which one_uav_active_dry_run.py's own isolation
        proof (using this module's is_authorized_for_first_active_flight
        directly) deliberately does not.
        """
        from unittest import mock

        from mavlink_manual_bridge import build_active_offboard_setpoint_sender
        from offboard_authority import AUTHORITY_ENV_VAR, LEGACY_WRITER_FLAGS

        def sink(*_args):  # pragma: no cover - must never be called
            raise AssertionError("unauthorized transmit")

        cases = [
            ({AUTHORITY_ENV_VAR: "companion_safety"}, "UAV-01", 1, True),
            ({AUTHORITY_ENV_VAR: "companion_safety"}, "UAV-02", 2, True),
            ({AUTHORITY_ENV_VAR: "companion_safety"}, "UAV-02", 1, False),
            ({AUTHORITY_ENV_VAR: "companion_safety"}, "UAV-01", 2, False),
            ({AUTHORITY_ENV_VAR: "companion_safety"}, "UAV-01", 0, False),
            ({AUTHORITY_ENV_VAR: "disabled"}, "UAV-01", 1, False),
            ({}, "UAV-01", 1, False),
            (
                {
                    AUTHORITY_ENV_VAR: "companion_safety",
                    LEGACY_WRITER_FLAGS[0]: "true",
                },
                "UAV-01",
                1,
                False,
            ),
        ]
        for environment, drone_id, system_id, expected in cases:
            with self.subTest(drone=drone_id, system_id=system_id, env=environment):
                with mock.patch.dict("os.environ", environment, clear=True):
                    sender = build_active_offboard_setpoint_sender(
                        drone_id, system_id, sink
                    )

                self.assertEqual(sender.status()["transmit_sink_attached"], expected)
                self.assertEqual(sender.status()["explicit_opt_in"], expected)


class DryRunModuleTests(unittest.TestCase):
    def test_dry_run_imports_no_transport_library(self) -> None:
        import ast
        from pathlib import Path

        for name in ("one_uav_active_dry_run.py", "one_uav_active_readiness.py"):
            with self.subTest(module=name):
                tree = ast.parse(Path(name).read_text(encoding="utf-8"))
                imported: set[str] = set()
                for node in ast.walk(tree):
                    if isinstance(node, ast.Import):
                        imported.update(a.name.split(".")[0] for a in node.names)
                    elif isinstance(node, ast.ImportFrom) and node.module:
                        imported.add(node.module.split(".")[0])
                for forbidden in ("pymavlink", "socket", "paho", "serial"):
                    self.assertNotIn(forbidden, imported)



class VelocityReadinessTests(unittest.TestCase):
    """BLOCKER 1: PX4's OFFBOARD velocity precondition, fail-closed."""

    def flags(self, **overrides):
        base = {
            "local_velocity_invalid": False,
            "received_monotonic_s": 100.0,
            "timestamp_us": 1_700_000_000_000,
        }
        base.update(overrides)
        return base

    def test_false_allows_readiness(self) -> None:
        result = evaluate_velocity_readiness(self.flags(), 100.1, 3.0)

        self.assertTrue(result.ready)
        self.assertEqual(result.reason, "")
        self.assertIs(result.observed_value, False)

    def test_true_blocks_readiness(self) -> None:
        result = evaluate_velocity_readiness(
            self.flags(local_velocity_invalid=True), 100.1, 3.0
        )

        self.assertFalse(result.ready)
        self.assertEqual(result.reason, "local_velocity_invalid")

    def test_missing_key_blocks_readiness(self) -> None:
        flags = self.flags()
        del flags["local_velocity_invalid"]

        result = evaluate_velocity_readiness(flags, 100.1, 3.0)

        self.assertFalse(result.ready)
        self.assertEqual(result.reason, "local_velocity_invalid_not_reported")

    def test_none_value_blocks_readiness(self) -> None:
        """The telemetry node emits None when PX4's message lacks the field.
        None must never be read as 'valid'."""
        result = evaluate_velocity_readiness(
            self.flags(local_velocity_invalid=None), 100.1, 3.0
        )

        self.assertFalse(result.ready)
        self.assertEqual(result.reason, "local_velocity_invalid_not_reported")

    def test_stale_sample_blocks_readiness(self) -> None:
        result = evaluate_velocity_readiness(self.flags(), 200.0, 3.0)

        self.assertFalse(result.ready)
        self.assertEqual(result.reason, "failsafe_flags_stale")

    def test_regressed_timestamp_blocks_readiness(self) -> None:
        result = evaluate_velocity_readiness(self.flags(), 99.0, 3.0)

        self.assertFalse(result.ready)
        self.assertEqual(result.reason, "failsafe_flags_timestamp_regressed")

    def test_missing_timestamp_blocks_readiness(self) -> None:
        flags = self.flags()
        del flags["received_monotonic_s"]

        result = evaluate_velocity_readiness(flags, 100.1, 3.0)

        self.assertFalse(result.ready)
        self.assertEqual(result.reason, "failsafe_flags_timestamp_missing")

    def test_malformed_value_blocks_readiness(self) -> None:
        """A non-bool must not be coerced: the string 'false' is truthy."""
        for value in ("false", 0, 1, [], {}):
            with self.subTest(value=value):
                result = evaluate_velocity_readiness(
                    self.flags(local_velocity_invalid=value), 100.1, 3.0
                )
                self.assertFalse(result.ready)
                self.assertEqual(result.reason, "local_velocity_invalid_malformed")

    def test_absent_failsafe_flags_block_readiness(self) -> None:
        for payload in (None, "", [], 42):
            with self.subTest(payload=payload):
                result = evaluate_velocity_readiness(payload, 100.1, 3.0)
                self.assertFalse(result.ready)
                self.assertEqual(result.reason, "failsafe_flags_missing")

    def test_readiness_is_never_inferred_from_velocity_values(self) -> None:
        """A perfectly plausible velocity must not make readiness pass when
        the reported flag says invalid."""
        flags = self.flags(local_velocity_invalid=True)
        flags["velocity_ned_m_s"] = [0.0, 0.0, 0.0]

        result = evaluate_velocity_readiness(flags, 100.1, 3.0)

        self.assertFalse(result.ready)


class PrecheckIntegrationTests(unittest.TestCase):
    """The PRECHECK guard must enforce both blockers."""

    def evidence(self, **overrides):
        base = {
            "preflight_checks_pass": True,
            "local_position_invalid_false": True,
            "companion_safety_running": True,
            "peer_state_fresh": True,
            "armed": False,
            "velocity_readiness": evaluate_velocity_readiness(
                {"local_velocity_invalid": False, "received_monotonic_s": 100.0},
                100.1,
                3.0,
            ),
            "offboard_authority": {
                "active_offboard_writer_count": 1,
                "companion_writer_permitted": True,
                "reason": "",
            },
        }
        base.update(overrides)
        return base

    def guard(self, evidence):
        return transition_guard(
            ReadinessState.PRECHECK,
            evidence,
            FlightEnvelope.from_environment(),
            WarmupContract(),
        )

    def test_healthy_evidence_passes(self) -> None:
        ok, reason = self.guard(self.evidence())

        self.assertTrue(ok)
        self.assertEqual(reason, "")

    def test_velocity_invalid_blocks_precheck(self) -> None:
        ok, reason = self.guard(
            self.evidence(
                velocity_readiness=evaluate_velocity_readiness(
                    {"local_velocity_invalid": True, "received_monotonic_s": 100.0},
                    100.1,
                    3.0,
                )
            )
        )

        self.assertFalse(ok)
        self.assertEqual(reason, "velocity_not_ready:local_velocity_invalid")

    def test_absent_velocity_evidence_blocks_precheck(self) -> None:
        evidence = self.evidence()
        del evidence["velocity_readiness"]

        ok, reason = self.guard(evidence)

        self.assertFalse(ok)
        self.assertEqual(reason, "precheck_missing:velocity_readiness")

    def test_two_writers_block_precheck(self) -> None:
        ok, reason = self.guard(
            self.evidence(
                offboard_authority={
                    "active_offboard_writer_count": 2,
                    "companion_writer_permitted": True,
                    "reason": "",
                }
            )
        )

        self.assertFalse(ok)
        self.assertEqual(reason, "multiple_offboard_writers_permitted")

    def test_companion_not_permitted_blocks_precheck(self) -> None:
        ok, reason = self.guard(
            self.evidence(
                offboard_authority={
                    "active_offboard_writer_count": 0,
                    "companion_writer_permitted": False,
                    "reason": "authority_disabled",
                }
            )
        )

        self.assertFalse(ok)
        self.assertIn("companion_writer_not_permitted", reason)

    def test_absent_authority_evidence_blocks_precheck(self) -> None:
        evidence = self.evidence()
        del evidence["offboard_authority"]

        ok, reason = self.guard(evidence)

        self.assertFalse(ok)
        self.assertEqual(reason, "precheck_missing:offboard_authority")

if __name__ == "__main__":
    unittest.main()
