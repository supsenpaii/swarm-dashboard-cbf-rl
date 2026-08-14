from __future__ import annotations

import unittest

from offboard_authority import ACTIVE_FLIGHT_AUTHORIZED_VEHICLES
from two_uav_active_readiness import (
    ABORT_MATRIX,
    STATE_SEQUENCE,
    FlightEnvelope,
    ReadinessState,
    WarmupContract,
    abort_rule_for,
    evaluate_velocity_readiness,
    is_authorized_for_two_uav_active_flight,
    transition_guard,
)


class TwoUavAuthorizationTests(unittest.TestCase):
    def test_uav01_is_authorized_with_opt_in(self) -> None:
        allowed, reason = is_authorized_for_two_uav_active_flight("UAV-01", 1, True)

        self.assertTrue(allowed)
        self.assertEqual(reason, "")

    def test_uav02_is_authorized_with_opt_in(self) -> None:
        """The whole point of this module: UAV-02 is no longer impossible
        here, unlike in one_uav_active_readiness."""
        allowed, reason = is_authorized_for_two_uav_active_flight("UAV-02", 2, True)

        self.assertTrue(allowed)
        self.assertEqual(reason, "")

    def test_opt_in_is_still_required_for_both_vehicles(self) -> None:
        for drone_id, system_id in (("UAV-01", 1), ("UAV-02", 2)):
            with self.subTest(drone_id=drone_id):
                allowed, reason = is_authorized_for_two_uav_active_flight(
                    drone_id, system_id, False
                )
                self.assertFalse(allowed)
                self.assertEqual(reason, "explicit_opt_in_absent")

    def test_wrong_system_id_is_refused_for_both_vehicles(self) -> None:
        for drone_id, wrong_system_id in (("UAV-01", 2), ("UAV-02", 1)):
            with self.subTest(drone_id=drone_id):
                allowed, reason = is_authorized_for_two_uav_active_flight(
                    drone_id, wrong_system_id, True
                )
                self.assertFalse(allowed)
                self.assertIn("vehicle_not_authorized", reason)

    def test_unlisted_drone_id_is_refused(self) -> None:
        allowed, reason = is_authorized_for_two_uav_active_flight("UAV-03", 3, True)

        self.assertFalse(allowed)
        self.assertIn("vehicle_not_authorized", reason)

    def test_broadcast_system_id_zero_is_refused(self) -> None:
        allowed, reason = is_authorized_for_two_uav_active_flight("UAV-01", 0, True)

        self.assertFalse(allowed)
        self.assertIn("vehicle_not_authorized", reason)

    def test_authorized_set_matches_offboard_authority(self) -> None:
        """Drift between the two would mean the sink-attachment gate and this
        readiness gate disagree about who may fly."""
        for drone_id, system_id in ACTIVE_FLIGHT_AUTHORIZED_VEHICLES:
            with self.subTest(drone_id=drone_id, system_id=system_id):
                allowed, _ = is_authorized_for_two_uav_active_flight(
                    drone_id, system_id, True
                )
                self.assertTrue(allowed)


class ReexportIdentityTests(unittest.TestCase):
    """Vehicle-agnostic contract pieces must be the SAME objects as
    one_uav_active_readiness's, not copies that could drift."""

    def test_abort_matrix_is_the_same_object(self) -> None:
        from one_uav_active_readiness import ABORT_MATRIX as one_uav_matrix

        self.assertIs(ABORT_MATRIX, one_uav_matrix)

    def test_state_sequence_is_the_same_object(self) -> None:
        from one_uav_active_readiness import STATE_SEQUENCE as one_uav_sequence

        self.assertIs(STATE_SEQUENCE, one_uav_sequence)

    def test_transition_guard_is_the_same_function(self) -> None:
        from one_uav_active_readiness import transition_guard as one_uav_guard

        self.assertIs(transition_guard, one_uav_guard)

    def test_abort_rule_for_is_the_same_function(self) -> None:
        from one_uav_active_readiness import abort_rule_for as one_uav_abort_rule_for

        self.assertIs(abort_rule_for, one_uav_abort_rule_for)

    def test_evaluate_velocity_readiness_is_the_same_function(self) -> None:
        from one_uav_active_readiness import (
            evaluate_velocity_readiness as one_uav_velocity_readiness,
        )

        self.assertIs(evaluate_velocity_readiness, one_uav_velocity_readiness)


class SanityUsableTests(unittest.TestCase):
    """The re-exported pieces must still work end to end through this
    module's names, not just be importable."""

    def test_transition_guard_works_through_this_module(self) -> None:
        envelope = FlightEnvelope.from_environment()
        warmup = WarmupContract()
        allowed, reason = transition_guard(
            ReadinessState.PRECHECK, {}, envelope, warmup
        )
        self.assertFalse(allowed)
        self.assertTrue(reason.startswith("precheck_missing"))

    def test_abort_rule_for_works_through_this_module(self) -> None:
        rule = abort_rule_for("cbf_invalid_or_infeasible")
        self.assertIsNotNone(rule)
        self.assertEqual(rule.condition, "cbf_invalid_or_infeasible")

    def test_evaluate_velocity_readiness_works_through_this_module(self) -> None:
        readiness = evaluate_velocity_readiness(None, 100.0, 0.5)
        self.assertFalse(readiness.ready)
        self.assertEqual(readiness.reason, "failsafe_flags_missing")


if __name__ == "__main__":
    unittest.main()
