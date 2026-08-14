from __future__ import annotations

import unittest

from companion_safety import CompanionSafetyMonitor
from formation_controller import (
    AltitudeHoldConfig,
    AltitudeHoldController,
    FormationSlot,
)


def state(altitude_m: float, valid: bool = True, east: float = 0.0, north: float = 0.0):
    return {
        "valid": valid,
        "position_enu_m": [east, north, altitude_m],
        "velocity_enu_m_s": [0.0, 0.0, 0.0],
    }


class ReferenceCaptureTests(unittest.TestCase):
    """The reference must be captured on the transition into station-keeping.

    Capturing it whenever state is valid would latch the ground altitude, and
    the companion would command a dive the instant OFFBOARD engaged after an
    AUTO takeoff. This is the failure these tests exist to prevent.
    """

    def test_inactive_captures_nothing(self) -> None:
        hold = AltitudeHoldController()

        command = hold.command("UAV-01", state(0.6), station_keeping=False)

        self.assertIsNone(hold.reference_altitude_m)
        self.assertFalse(command.active)
        self.assertEqual(command.reason, "station_keeping_inactive")

    def test_ground_altitude_is_not_latched_before_takeoff(self) -> None:
        hold = AltitudeHoldController()
        for _ in range(50):  # sitting on the ground, disarmed
            hold.command("UAV-01", state(0.6), station_keeping=False)
        # AUTO takeoff happens, THEN OFFBOARD engages at altitude.
        command = hold.command("UAV-01", state(9.2), station_keeping=True)

        self.assertAlmostEqual(hold.reference_altitude_m, 9.2)
        self.assertEqual(command.velocity_enu_m_s, (0.0, 0.0, 0.0))
        self.assertEqual(command.reason, "altitude_reference_captured")

    def test_first_active_frame_commands_nothing(self) -> None:
        """Capture and correct in the same frame would act on a zero error."""
        hold = AltitudeHoldController()

        command = hold.command("UAV-01", state(9.2), station_keeping=True)

        self.assertEqual(command.velocity_enu_m_s, (0.0, 0.0, 0.0))

    def test_leaving_station_keeping_clears_the_reference(self) -> None:
        hold = AltitudeHoldController()
        hold.command("UAV-01", state(9.2), station_keeping=True)

        hold.command("UAV-01", state(9.2), station_keeping=False)

        self.assertIsNone(hold.reference_altitude_m)

    def test_reference_is_recaptured_at_the_new_altitude(self) -> None:
        hold = AltitudeHoldController()
        hold.command("UAV-01", state(9.2), station_keeping=True)
        hold.command("UAV-01", state(9.2), station_keeping=False)

        hold.command("UAV-01", state(20.0), station_keeping=True)

        self.assertAlmostEqual(hold.reference_altitude_m, 20.0)

    def test_invalid_state_drops_the_reference(self) -> None:
        """Holding against a stale reference is how a small gap becomes a
        large correction."""
        hold = AltitudeHoldController()
        hold.command("UAV-01", state(9.2), station_keeping=True)

        command = hold.command("UAV-01", state(9.2, valid=False), station_keeping=True)

        self.assertIsNone(hold.reference_altitude_m)
        self.assertFalse(command.active)
        self.assertEqual(command.reason, "own_state_invalid")


class CorrectionTests(unittest.TestCase):
    def setUp(self) -> None:
        self.hold = AltitudeHoldController(
            AltitudeHoldConfig(gain_s_inv=0.6, maximum_velocity_m_s=1.0,
                               arrival_radius_m=0.25)
        )
        self.hold.command("UAV-01", state(9.2), station_keeping=True)

    def test_drifting_up_commands_a_descent(self) -> None:
        """The measured failure: +1.36 m of upward drift went uncorrected."""
        command = self.hold.command("UAV-01", state(10.56), station_keeping=True)

        self.assertLess(command.velocity_enu_m_s[2], 0.0)
        self.assertAlmostEqual(command.velocity_enu_m_s[2], 0.6 * -1.36, places=6)
        self.assertEqual(command.reason, "altitude_hold")

    def test_drifting_down_commands_a_climb(self) -> None:
        command = self.hold.command("UAV-01", state(8.0), station_keeping=True)

        self.assertGreater(command.velocity_enu_m_s[2], 0.0)

    def test_correction_is_vertical_only(self) -> None:
        """Horizontal station-keeping measured 0.028 m/s unaided; adding
        horizontal feedback would be fixing something that is not broken."""
        command = self.hold.command(
            "UAV-01", state(10.5, east=5.0, north=-3.0), station_keeping=True
        )

        self.assertEqual(command.velocity_enu_m_s[0], 0.0)
        self.assertEqual(command.velocity_enu_m_s[1], 0.0)

    def test_inside_the_arrival_radius_commands_nothing(self) -> None:
        command = self.hold.command("UAV-01", state(9.35), station_keeping=True)

        self.assertEqual(command.velocity_enu_m_s, (0.0, 0.0, 0.0))
        self.assertEqual(command.reason, "altitude_held")

    def test_correction_is_bounded_by_the_flight_envelope(self) -> None:
        for altitude in (0.0, 100.0, -50.0):
            with self.subTest(altitude=altitude):
                command = self.hold.command(
                    "UAV-01", state(altitude), station_keeping=True
                )

                self.assertLessEqual(abs(command.velocity_enu_m_s[2]), 1.0)

    def test_a_step_disturbance_converges(self) -> None:
        """Closed loop at 20 Hz, integrating the commanded velocity."""
        altitude = 9.2 + 1.36
        for _ in range(400):  # 20 s
            command = self.hold.command("UAV-01", state(altitude), station_keeping=True)
            altitude += command.velocity_enu_m_s[2] * 0.05

        self.assertLess(abs(altitude - 9.2), 0.25)


class MonitorIntegrationTests(unittest.TestCase):
    """Altitude hold must apply to the leader and never displace a slot."""

    def monitor(self, drone_id: str) -> CompanionSafetyMonitor:
        return CompanionSafetyMonitor(
            drone_id=drone_id,
            peer_ids=("UAV-02",) if drone_id == "UAV-01" else ("UAV-01",),
            leader_id="UAV-01",
            slots=(FormationSlot("UAV-02", (-10.0, 0.0, 0.0)),),
        )

    def swarm(self, leader_altitude: float = 10.56):
        return {
            "UAV-01": state(leader_altitude),
            "UAV-02": state(10.0, east=-40.0),
        }

    def test_leader_holds_altitude_while_station_keeping(self) -> None:
        monitor = self.monitor("UAV-01")
        monitor.evaluate(self.swarm(9.2), 100.0, station_keeping=True)

        status = monitor.evaluate(self.swarm(10.56), 100.05, station_keeping=True)

        self.assertTrue(status.nominal_active)
        self.assertEqual(status.nominal_reason, "altitude_hold")
        self.assertLess(status.nominal_velocity_enu_m_s[2], 0.0)

    def test_leader_commands_nothing_when_not_station_keeping(self) -> None:
        """Unchanged behaviour on the ground, reason string included: masking
        it would turn two different diagnoses into one."""
        monitor = self.monitor("UAV-01")

        status = monitor.evaluate(self.swarm(0.6), 100.0, station_keeping=False)

        self.assertFalse(status.nominal_active)
        self.assertEqual(status.nominal_reason, "formation_slot_unassigned")
        self.assertEqual(status.nominal_velocity_enu_m_s, (0.0, 0.0, 0.0))

    def test_altitude_hold_failure_does_not_mask_the_formation_reason(self) -> None:
        monitor = self.monitor("UAV-01")
        swarm = self.swarm()
        swarm["UAV-01"] = state(9.2, valid=False)

        status = monitor.evaluate(swarm, 100.0, station_keeping=True)

        self.assertFalse(status.nominal_active)
        self.assertEqual(status.nominal_reason, "formation_slot_unassigned")

    def test_default_call_does_not_enable_station_keeping(self) -> None:
        """Every existing caller omits the flag; none may gain altitude hold
        by accident."""
        monitor = self.monitor("UAV-01")

        status = monitor.evaluate(self.swarm(9.2), 100.0)

        self.assertFalse(status.nominal_active)

    def test_a_follower_with_a_slot_is_untouched(self) -> None:
        monitor = self.monitor("UAV-02")

        status = monitor.evaluate(self.swarm(), 100.0, station_keeping=True)

        self.assertTrue(status.nominal_active)
        self.assertEqual(status.nominal_reason, "tracking_slot")
        self.assertIsNone(monitor.altitude_hold.reference_altitude_m)

    def test_the_correction_still_passes_through_cbf(self) -> None:
        """Altitude hold is a nominal, not a bypass: it is filtered like any
        other command."""
        monitor = self.monitor("UAV-01")
        monitor.evaluate(self.swarm(9.2), 100.0, station_keeping=True)

        status = monitor.evaluate(self.swarm(10.56), 100.05, station_keeping=True)

        self.assertIsNotNone(status.command)
        self.assertTrue(status.output_valid)

    def test_configuration_is_inherited_not_reinvented(self) -> None:
        monitor = self.monitor("UAV-01")

        self.assertEqual(
            monitor.altitude_hold.config.gain_s_inv,
            monitor.formation.config.position_gain_s_inv,
        )
        self.assertEqual(
            monitor.altitude_hold.config.arrival_radius_m,
            monitor.formation.config.arrival_radius_m,
        )
        self.assertEqual(
            monitor.altitude_hold.config.maximum_velocity_m_s,
            monitor.formation.config.maximum_velocity_m_s / 2.0,
        )


if __name__ == "__main__":
    unittest.main()
