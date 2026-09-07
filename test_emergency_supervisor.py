from __future__ import annotations

import unittest

from cbf_command_gate import CbfCommand
from emergency_supervisor import (
    EmergencyConfig,
    EmergencyStage,
    EmergencySupervisor,
    STAGE_ORDER,
)


FEASIBLE = CbfCommand("UAV-01", (1.0, 0.0, 0.0), True, "cbf_filtered", 5.0, 0.1)
INFEASIBLE = CbfCommand("UAV-01", (0.0, 0.0, 0.0), False, "cbf_constraints_infeasible", -1.0, 0.0)


def supervisor(**overrides) -> EmergencySupervisor:
    config = EmergencyConfig(
        stage_dwell_s=2.0,
        recovery_confirmation_s=0.5,
        altitude_base_m=10.0,
        altitude_step_m=3.0,
    )
    defaults = dict(
        drone_id="UAV-01",
        peer_ids=("UAV-02",),
        geofence_min_enu_m=(-100.0, -100.0, 0.0),
        geofence_max_enu_m=(100.0, 100.0, 50.0),
        maximum_velocity_m_s=2.0,
        altitude_gain_s_inv=0.6,
        altitude_arrival_radius_m=0.25,
        config=config,
    )
    defaults.update(overrides)
    return EmergencySupervisor(**defaults)


class NormalAndFirstEscalationTests(unittest.TestCase):
    def test_feasible_command_stays_normal_and_passes_through_velocity(self) -> None:
        sup = supervisor()
        decision = sup.evaluate(FEASIBLE, 0.0, own_state_valid=True, own_altitude_m=10.0)

        self.assertEqual(decision.stage, EmergencyStage.NORMAL)
        self.assertFalse(decision.active)
        self.assertEqual(decision.velocity_enu_m_s, FEASIBLE.velocity_enu_m_s)
        self.assertIsNone(decision.recommended_px4_mode)

    def test_first_infeasible_frame_enters_stop_horizontal_immediately(self) -> None:
        """§20 gives no arming delay before rung 1 -- the very first infeasible
        frame must already be inside the fallback ladder, not still NORMAL."""
        sup = supervisor()
        decision = sup.evaluate(INFEASIBLE, 0.0, own_state_valid=True, own_altitude_m=10.0)

        self.assertEqual(decision.stage, EmergencyStage.STOP_HORIZONTAL)
        self.assertTrue(decision.active)
        self.assertEqual(decision.velocity_enu_m_s, (0.0, 0.0, 0.0))


class EscalationLadderTests(unittest.TestCase):
    def test_escalates_one_rung_per_dwell_period_never_skipping(self) -> None:
        sup = supervisor()
        t = 0.0
        seen_stages = []
        # Drive well past every stage's dwell and record the stage sequence.
        for _ in range(40):
            decision = sup.evaluate(INFEASIBLE, t, own_state_valid=True, own_altitude_m=10.0)
            if not seen_stages or seen_stages[-1] != decision.stage:
                seen_stages.append(decision.stage)
            t += 0.5

        # Altitude separation is reachable here (own state valid, target 10m
        # is inside the geofence), so the full ladder must appear in order.
        self.assertEqual(list(seen_stages), list(STAGE_ORDER[1:]))

    def test_does_not_advance_before_dwell_elapses(self) -> None:
        sup = supervisor()
        sup.evaluate(INFEASIBLE, 0.0, own_state_valid=True, own_altitude_m=10.0)
        decision = sup.evaluate(INFEASIBLE, 1.0, own_state_valid=True, own_altitude_m=10.0)

        self.assertEqual(decision.stage, EmergencyStage.STOP_HORIZONTAL)

    def test_reaches_terminal_stage_and_holds_there(self) -> None:
        sup = supervisor()
        t = 0.0
        decision = None
        for _ in range(20):
            decision = sup.evaluate(INFEASIBLE, t, own_state_valid=True, own_altitude_m=10.0)
            t += 3.0

        assert decision is not None
        self.assertEqual(decision.stage, EmergencyStage.RECOMMEND_RTL_OR_LAND)
        self.assertEqual(decision.recommended_px4_mode, "RTL_OR_LAND")
        self.assertEqual(decision.velocity_enu_m_s, (0.0, 0.0, 0.0))

        # One more infeasible frame must not move past the terminal stage.
        further = sup.evaluate(INFEASIBLE, t + 100.0, own_state_valid=True, own_altitude_m=10.0)
        self.assertEqual(further.stage, EmergencyStage.RECOMMEND_RTL_OR_LAND)

    def test_every_stage_produces_a_finite_bounded_command(self) -> None:
        sup = supervisor()
        t = 0.0
        for _ in range(20):
            decision = sup.evaluate(INFEASIBLE, t, own_state_valid=True, own_altitude_m=10.0)
            for component in decision.velocity_enu_m_s:
                self.assertTrue(component == component)  # not NaN
            magnitude = sum(v * v for v in decision.velocity_enu_m_s) ** 0.5
            self.assertLessEqual(magnitude, 2.0 + 1e-9)
            t += 3.0


class AltitudeSeparationPreconditionTests(unittest.TestCase):
    def test_altitude_separation_commands_climb_toward_drone_id_target(self) -> None:
        sup = supervisor()
        t = 0.0
        # Drive to ALTITUDE_SEPARATION (third rung).
        for _ in range(3):
            decision = sup.evaluate(INFEASIBLE, t, own_state_valid=True, own_altitude_m=6.0)
            t += 2.0

        self.assertEqual(decision.stage, EmergencyStage.ALTITUDE_SEPARATION)
        # UAV-01 sorts first among {UAV-01, UAV-02} -> index 0 -> target 10 m.
        # Currently at 6 m, so the command must climb (positive vz).
        self.assertGreater(decision.velocity_enu_m_s[2], 0.0)
        self.assertEqual(decision.velocity_enu_m_s[:2], (0.0, 0.0))

    def test_second_drone_id_gets_a_higher_deconflicted_altitude(self) -> None:
        sup = supervisor(drone_id="UAV-02", peer_ids=("UAV-01",))
        t = 0.0
        for _ in range(3):
            decision = sup.evaluate(
                CbfCommand("UAV-02", (0.0, 0.0, 0.0), False, "cbf_constraints_infeasible", -1.0, 0.0),
                t,
                own_state_valid=True,
                own_altitude_m=10.0,
            )
            t += 2.0

        self.assertEqual(decision.stage, EmergencyStage.ALTITUDE_SEPARATION)
        # UAV-02 sorts second -> target 13 m; currently at 10 m -> climb.
        self.assertGreater(decision.velocity_enu_m_s[2], 0.0)

    def test_altitude_separation_is_skipped_when_own_state_is_invalid(self) -> None:
        """A drone that does not know its own altitude cannot safely execute a
        position-dependent altitude change; the rung must be skipped, not
        stall the ladder or fabricate a value."""
        sup = supervisor()
        t = 0.0
        stages = []
        # 3 dwell periods: STOP_HORIZONTAL -> REDUCE_VELOCITY -> (skip) -> HOLD.
        for _ in range(3):
            decision = sup.evaluate(INFEASIBLE, t, own_state_valid=False, own_altitude_m=None)
            stages.append(decision.stage)
            t += 2.0

        self.assertNotIn(EmergencyStage.ALTITUDE_SEPARATION, stages)
        # Ladder must still progress toward HOLD rather than getting stuck.
        self.assertEqual(stages[-1], EmergencyStage.HOLD)

    def test_altitude_separation_is_skipped_when_target_violates_geofence(self) -> None:
        sup = supervisor(geofence_max_enu_m=(100.0, 100.0, 11.0))  # target 10m ok, but UAV-02's 13m is not
        sup2 = supervisor(
            drone_id="UAV-02",
            peer_ids=("UAV-01",),
            geofence_max_enu_m=(100.0, 100.0, 11.0),
        )
        t = 0.0
        stages = []
        for _ in range(3):
            decision = sup2.evaluate(
                CbfCommand("UAV-02", (0.0, 0.0, 0.0), False, "cbf_constraints_infeasible", -1.0, 0.0),
                t,
                own_state_valid=True,
                own_altitude_m=10.0,
            )
            stages.append(decision.stage)
            t += 2.0

        self.assertNotIn(EmergencyStage.ALTITUDE_SEPARATION, stages)
        self.assertEqual(stages[-1], EmergencyStage.HOLD)

    def test_altitude_separation_zeroes_vertical_once_within_arrival_radius(self) -> None:
        sup = supervisor()
        t = 0.0
        for _ in range(3):
            decision = sup.evaluate(INFEASIBLE, t, own_state_valid=True, own_altitude_m=10.0)
            t += 2.0

        self.assertEqual(decision.stage, EmergencyStage.ALTITUDE_SEPARATION)
        self.assertEqual(decision.velocity_enu_m_s, (0.0, 0.0, 0.0))


class RecoveryTests(unittest.TestCase):
    def test_single_feasible_frame_does_not_immediately_recover(self) -> None:
        """Anti-chatter: recovery requires sustained feasibility, not one frame."""
        sup = supervisor()
        sup.evaluate(INFEASIBLE, 0.0, own_state_valid=True, own_altitude_m=10.0)
        decision = sup.evaluate(FEASIBLE, 0.1, own_state_valid=True, own_altitude_m=10.0)

        self.assertNotEqual(decision.stage, EmergencyStage.NORMAL)
        self.assertEqual(decision.stage, EmergencyStage.STOP_HORIZONTAL)

    def test_recovers_after_confirmation_window_of_continuous_feasibility(self) -> None:
        sup = supervisor()
        sup.evaluate(INFEASIBLE, 0.0, own_state_valid=True, own_altitude_m=10.0)
        sup.evaluate(FEASIBLE, 0.1, own_state_valid=True, own_altitude_m=10.0)
        decision = sup.evaluate(FEASIBLE, 0.7, own_state_valid=True, own_altitude_m=10.0)

        self.assertEqual(decision.stage, EmergencyStage.NORMAL)
        self.assertEqual(decision.velocity_enu_m_s, FEASIBLE.velocity_enu_m_s)

    def test_recovery_streak_resets_if_infeasibility_returns_before_confirmed(self) -> None:
        """No recovery while the triggering fault is still intermittently present."""
        sup = supervisor()
        sup.evaluate(INFEASIBLE, 0.0, own_state_valid=True, own_altitude_m=10.0)
        sup.evaluate(FEASIBLE, 0.1, own_state_valid=True, own_altitude_m=10.0)
        sup.evaluate(INFEASIBLE, 0.4, own_state_valid=True, own_altitude_m=10.0)  # breaks the streak
        decision = sup.evaluate(FEASIBLE, 0.5, own_state_valid=True, own_altitude_m=10.0)

        # Only 0.1s into a fresh feasible streak -- must not have recovered.
        self.assertNotEqual(decision.stage, EmergencyStage.NORMAL)

    def test_no_oscillation_from_rapid_sub_threshold_flips(self) -> None:
        """Chatter faster than the confirmation window must never recover to
        NORMAL (no blip sustains long enough) and must never regress to an
        earlier rung -- only monotonic forward progress (or holding) is
        allowed, exactly as if the blips were not happening at all."""
        sup = supervisor()
        t = 0.0
        sup.evaluate(INFEASIBLE, t, own_state_valid=True, own_altitude_m=10.0)
        indices_seen = []
        for _ in range(12):
            t += 0.05
            decision = sup.evaluate(FEASIBLE, t, own_state_valid=True, own_altitude_m=10.0)
            indices_seen.append(STAGE_ORDER.index(decision.stage))
            t += 0.05
            decision = sup.evaluate(INFEASIBLE, t, own_state_valid=True, own_altitude_m=10.0)
            indices_seen.append(STAGE_ORDER.index(decision.stage))

        self.assertNotIn(EmergencyStage.NORMAL, {STAGE_ORDER[i] for i in indices_seen})
        self.assertEqual(indices_seen, sorted(indices_seen))

    def test_escalation_dwell_clock_is_not_disturbed_by_a_feasible_blip(self) -> None:
        """A blip must not reset (or advance) the stage-entry clock: dwell
        progress made before the blip must still count afterward."""
        sup = supervisor()
        sup.evaluate(INFEASIBLE, 0.0, own_state_valid=True, own_altitude_m=10.0)
        sup.evaluate(INFEASIBLE, 1.9, own_state_valid=True, own_altitude_m=10.0)  # 1.9s dwell so far
        sup.evaluate(FEASIBLE, 1.95, own_state_valid=True, own_altitude_m=10.0)  # blip, < confirmation
        decision = sup.evaluate(INFEASIBLE, 2.0, own_state_valid=True, own_altitude_m=10.0)

        # Total dwell since original entry is 2.0s >= stage_dwell_s -> escalate.
        self.assertEqual(decision.stage, EmergencyStage.REDUCE_VELOCITY)

    def test_full_cycle_infeasible_to_hold_to_recovered(self) -> None:
        sup = supervisor()
        t = 0.0
        for _ in range(8):
            sup.evaluate(INFEASIBLE, t, own_state_valid=True, own_altitude_m=10.0)
            t += 2.0
        held = sup.evaluate(INFEASIBLE, t, own_state_valid=True, own_altitude_m=10.0)
        self.assertNotEqual(held.stage, EmergencyStage.NORMAL)

        t += 0.1
        sup.evaluate(FEASIBLE, t, own_state_valid=True, own_altitude_m=10.0)
        t += 0.6
        recovered = sup.evaluate(FEASIBLE, t, own_state_valid=True, own_altitude_m=10.0)

        self.assertEqual(recovered.stage, EmergencyStage.NORMAL)
        self.assertEqual(recovered.velocity_enu_m_s, FEASIBLE.velocity_enu_m_s)


class ReasonAgnosticismTests(unittest.TestCase):
    """§19's control loop branches only on `result.feasible`, never on why."""

    def test_every_non_active_reason_enters_the_same_first_rung(self) -> None:
        reasons = (
            "nominal_or_self_state_invalid",
            "peer_state_invalid",
            "outside_geofence",
            "peer_position_overlap",
            "cbf_constraints_infeasible",
        )
        for reason in reasons:
            with self.subTest(reason=reason):
                sup = supervisor()
                command = CbfCommand("UAV-01", (0.0, 0.0, 0.0), False, reason, None, 0.0)
                decision = sup.evaluate(command, 0.0, own_state_valid=True, own_altitude_m=10.0)
                self.assertEqual(decision.stage, EmergencyStage.STOP_HORIZONTAL)


if __name__ == "__main__":
    unittest.main()
