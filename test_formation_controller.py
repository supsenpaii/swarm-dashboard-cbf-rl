import math
import unittest

from formation_controller import (
    DeterministicFormationController,
    FormationConfig,
    FormationSlot,
)


def state(position, velocity=(0.0, 0.0, 0.0), valid=True):
    return {
        "valid": valid,
        "position_enu_m": position,
        "velocity_enu_m_s": velocity,
    }


class FormationControllerTests(unittest.TestCase):
    def setUp(self):
        self.controller = DeterministicFormationController(
            "UAV-01",
            (FormationSlot("UAV-02", (-10.0, 0.0, 0.0)),),
            FormationConfig(position_gain_s_inv=0.5, maximum_velocity_m_s=2.0, arrival_radius_m=0.2),
        )

    def test_follower_moves_toward_its_enu_slot(self):
        command = self.controller.command(
            "UAV-02",
            {"UAV-01": state((0.0, 0.0, 5.0)), "UAV-02": state((0.0, 0.0, 5.0))},
        )
        self.assertTrue(command.active)
        self.assertEqual(command.target_enu_m, (-10.0, 0.0, 5.0))
        self.assertLess(command.velocity_enu_m_s[0], 0.0)

    def test_leader_velocity_is_feedforward(self):
        command = self.controller.command(
            "UAV-02",
            {
                "UAV-01": state((0.0, 0.0, 0.0), (1.0, 0.0, 0.0)),
                "UAV-02": state((-10.0, -1.0, 0.0)),
            },
        )
        self.assertGreater(command.velocity_enu_m_s[0], 0.9)
        self.assertGreater(command.velocity_enu_m_s[1], 0.0)

    def test_command_is_velocity_limited(self):
        command = self.controller.command(
            "UAV-02",
            {"UAV-01": state((0.0, 0.0, 0.0)), "UAV-02": state((100.0, 0.0, 0.0))},
        )
        self.assertLessEqual(math.sqrt(sum(value * value for value in command.velocity_enu_m_s)), 2.0)

    def test_arrival_radius_holds_without_chasing_noise(self):
        command = self.controller.command(
            "UAV-02",
            {"UAV-01": state((0.0, 0.0, 0.0)), "UAV-02": state((-9.9, 0.0, 0.0))},
        )
        self.assertTrue(command.active)
        self.assertEqual(command.reason, "slot_reached")
        self.assertEqual(command.velocity_enu_m_s, (0.0, 0.0, 0.0))

    def test_invalid_or_stale_leader_holds_follower(self):
        command = self.controller.command(
            "UAV-02",
            {"UAV-01": state((0.0, 0.0, 0.0), valid=False), "UAV-02": state((-10.0, 0.0, 0.0))},
        )
        self.assertFalse(command.active)
        self.assertEqual(command.reason, "leader_state_invalid")

    def test_unassigned_vehicle_has_no_formation_authority(self):
        command = self.controller.command("UAV-03", {})
        self.assertFalse(command.active)
        self.assertEqual(command.reason, "formation_slot_unassigned")


if __name__ == "__main__":
    unittest.main()
