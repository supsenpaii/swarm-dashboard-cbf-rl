from __future__ import annotations

import unittest

from active_offboard_setpoint_sender import CALLER_REPORTED_CONDITIONS
from offboard_abort_conditions import (
    DEFAULT_ACCEPTED_ACK_RESULTS,
    conditions_from_status,
    derive_reported_conditions,
)


HEALTHY = dict(
    now_monotonic_s=100.0,
    self_state_valid=True,
    cbf_active=True,
    cbf_reason="cbf_filtered",
    last_heartbeat_monotonic_s=99.8,
    heartbeat_timeout_s=3.0,
    px4_main_mode=3,
    px4_offboard_main_mode=6,
    offboard_expected=False,
    offboard_mode_ack_result=None,
    previous_evaluation_monotonic_s=99.95,
    maximum_command_age_s=0.5,
)


def derive(**overrides) -> tuple[str, ...]:
    return derive_reported_conditions(**{**HEALTHY, **overrides})


class HealthyBaselineTests(unittest.TestCase):
    def test_a_healthy_frame_reports_nothing(self) -> None:
        self.assertEqual(derive(), ())

    def test_every_produced_condition_is_one_the_sender_accepts(self) -> None:
        """A condition string with no matching rule would be silently
        ignored by the sender, which is a failure that looks like success."""
        produced = set()
        for override in (
            {"self_state_valid": False},
            {"cbf_active": False},
            {"cbf_reason": "outside_geofence", "cbf_active": False},
            {"last_heartbeat_monotonic_s": None},
            {"offboard_expected": True},
            {"offboard_mode_ack_result": 4},
            {"previous_evaluation_monotonic_s": 90.0},
        ):
            produced.update(derive(**override))

        self.assertEqual(produced, set(CALLER_REPORTED_CONDITIONS))


def real_status_payload(
    *, cbf_active: bool = True, cbf_reason: str = "cbf_filtered", self_reason="ok"
) -> dict:
    """A payload produced by the REAL CompanionSafetyStatus serializer.

    Hand-written samples are what let the first version of this module ship
    reading `payload["command"]` when the serializer emits `payload["cbf"]`:
    the test agreed with the bug, and only the live run disagreed. Every
    adapter test below goes through the real dataclass so the key names are
    pinned by construction.
    """
    from cbf_command_gate import CbfCommand
    from companion_safety import CompanionSafetyStatus
    from emergency_supervisor import EmergencyDecision, EmergencyStage

    payload = CompanionSafetyStatus(
        drone_id="UAV-01",
        nominal_velocity_enu_m_s=(0.0, 0.0, 0.0),
        nominal_active=True,
        nominal_reason="test",
        nominal_position_error_m=None,
        command=CbfCommand("UAV-01", (0.0, 0.0, 0.0), cbf_active, cbf_reason, 1.0, 0.0),
        emergency=EmergencyDecision(
            stage=EmergencyStage.NORMAL,
            velocity_enu_m_s=(0.0, 0.0, 0.0),
            active=False,
            reason="test",
            recommended_px4_mode=None,
            time_in_stage_s=0.0,
        ),
        output_velocity_enu_m_s=(0.0, 0.0, 0.0),
        output_valid=True,
        peer_ids_used=("UAV-02",),
        peer_ids_missing=(),
    ).as_dict()
    if self_reason is not None:
        payload["self_state_reason"] = self_reason
    return payload


def adapt(payload: dict, **overrides) -> tuple[str, ...]:
    base = dict(
        now_monotonic_s=100.0,
        last_heartbeat_monotonic_s=99.8,
        heartbeat_timeout_s=3.0,
        px4_main_mode=3,
        px4_offboard_main_mode=6,
        offboard_expected=False,
        offboard_mode_ack_result=None,
        previous_evaluation_monotonic_s=99.95,
        maximum_command_age_s=0.5,
    )
    base.update(overrides)
    return conditions_from_status(payload, **base)


class SelfStateTests(unittest.TestCase):
    def test_invalid_own_state_aborts_regardless_of_reason(self) -> None:
        self.assertIn("self_telemetry_stale", derive(self_state_valid=False))

    def test_status_adapter_treats_a_missing_reason_as_unhealthy(self) -> None:
        payload = real_status_payload(self_reason=None)

        self.assertIn("self_telemetry_stale", adapt(payload))

    def test_status_adapter_accepts_a_healthy_real_payload(self) -> None:
        """The regression test for the wrong-key bug: a genuinely healthy
        status must produce no conditions at all."""
        self.assertEqual(adapt(real_status_payload()), ())

    def test_status_adapter_reads_a_real_inactive_cbf(self) -> None:
        conditions = adapt(
            real_status_payload(cbf_active=False, cbf_reason="cbf_constraints_infeasible")
        )

        self.assertIn("cbf_invalid_or_infeasible", conditions)

    def test_status_adapter_reads_a_real_geofence_hold(self) -> None:
        conditions = adapt(
            real_status_payload(cbf_active=False, cbf_reason="outside_geofence")
        )

        self.assertIn("geofence_violation_or_risk", conditions)

    def test_status_adapter_fails_closed_on_a_missing_cbf_block(self) -> None:
        payload = real_status_payload()
        payload.pop("cbf")

        self.assertIn("cbf_invalid_or_infeasible", adapt(payload))


class CbfTests(unittest.TestCase):
    def test_inactive_cbf_aborts(self) -> None:
        self.assertIn("cbf_invalid_or_infeasible", derive(cbf_active=False))

    def test_geofence_hold_reports_both_conditions(self) -> None:
        """The gate sets active=False for a geofence hold too. Reporting both
        lets the sender's matrix ordering pick the severity; choosing one
        here would pre-empt that decision with a less informed one."""
        conditions = derive(cbf_active=False, cbf_reason="outside_geofence")

        self.assertIn("geofence_violation_or_risk", conditions)
        self.assertIn("cbf_invalid_or_infeasible", conditions)

    def test_infeasible_constraints_are_not_mistaken_for_geofence(self) -> None:
        conditions = derive(cbf_active=False, cbf_reason="cbf_constraints_infeasible")

        self.assertNotIn("geofence_violation_or_risk", conditions)


class HeartbeatTests(unittest.TestCase):
    def test_never_heard_from_is_a_connection_loss(self) -> None:
        self.assertIn(
            "mavlink_connection_loss", derive(last_heartbeat_monotonic_s=None)
        )

    def test_timeout_exceeded_is_a_connection_loss(self) -> None:
        self.assertIn(
            "mavlink_connection_loss", derive(last_heartbeat_monotonic_s=96.9)
        )

    def test_at_the_timeout_is_still_alive(self) -> None:
        self.assertNotIn(
            "mavlink_connection_loss", derive(last_heartbeat_monotonic_s=97.0)
        )


class OffboardModeTests(unittest.TestCase):
    def test_posctl_is_not_an_unexpected_exit_while_offboard_is_not_expected(
        self,
    ) -> None:
        self.assertNotIn("px4_exits_offboard_unexpectedly", derive(px4_main_mode=3))

    def test_leaving_offboard_when_it_is_expected_aborts(self) -> None:
        conditions = derive(offboard_expected=True, px4_main_mode=3)

        self.assertIn("px4_exits_offboard_unexpectedly", conditions)

    def test_unknown_mode_while_offboard_is_expected_fails_closed(self) -> None:
        conditions = derive(offboard_expected=True, px4_main_mode=None)

        self.assertIn("px4_exits_offboard_unexpectedly", conditions)

    def test_staying_in_offboard_is_fine(self) -> None:
        conditions = derive(offboard_expected=True, px4_main_mode=6)

        self.assertNotIn("px4_exits_offboard_unexpectedly", conditions)

    def test_no_ack_is_not_a_rejection(self) -> None:
        self.assertNotIn("px4_rejects_offboard", derive(offboard_mode_ack_result=None))

    def test_accepted_results_are_not_rejections(self) -> None:
        for result in DEFAULT_ACCEPTED_ACK_RESULTS:
            with self.subTest(result=result):
                self.assertNotIn(
                    "px4_rejects_offboard", derive(offboard_mode_ack_result=result)
                )

    def test_any_other_result_is_a_rejection(self) -> None:
        for result in (1, 2, 3, 4, 6):  # TEMPORARILY_REJECTED, DENIED, ...
            with self.subTest(result=result):
                self.assertIn(
                    "px4_rejects_offboard", derive(offboard_mode_ack_result=result)
                )

    def test_bridge_and_module_agree_on_accepted_results(self) -> None:
        """The bridge derives its set from pymavlink; this module mirrors it
        as plain ints so it stays testable. Drift would make an accepted mode
        change look like a rejection."""
        from mavlink_manual_bridge import OFFBOARD_ACCEPTED_ACK_RESULTS

        self.assertEqual(OFFBOARD_ACCEPTED_ACK_RESULTS, DEFAULT_ACCEPTED_ACK_RESULTS)


class LoopLivenessTests(unittest.TestCase):
    def test_first_frame_is_not_a_stall(self) -> None:
        self.assertNotIn(
            "companion_safety_loop_stops",
            derive(previous_evaluation_monotonic_s=None),
        )

    def test_a_gap_beyond_the_command_age_budget_is_a_stall(self) -> None:
        self.assertIn(
            "companion_safety_loop_stops",
            derive(previous_evaluation_monotonic_s=99.4),
        )

    def test_nominal_cadence_is_not_a_stall(self) -> None:
        self.assertNotIn(
            "companion_safety_loop_stops",
            derive(previous_evaluation_monotonic_s=99.95),  # 50 ms
        )


if __name__ == "__main__":
    unittest.main()
