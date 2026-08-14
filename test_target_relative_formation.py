from __future__ import annotations

import unittest

from formation_controller import (
    FormationConfig,
    FormationSlot,
    TargetRelativeFormationController,
)


def follower(position, valid: bool = True) -> dict:
    return {"position_enu_m": list(position), "valid": valid}


def target(position, velocity=None, valid: bool = True) -> dict:
    state = {"position_enu_m": list(position), "valid": valid}
    if velocity is not None:
        state["velocity_enu_m_s"] = list(velocity)
    return state


def controller(**overrides) -> TargetRelativeFormationController:
    slots = overrides.pop(
        "slots", (FormationSlot("UAV-02", (-5.0, 0.0, 2.0)),)
    )
    config = overrides.pop("config", FormationConfig(arrival_radius_m=0.25))
    return TargetRelativeFormationController(slots, config)


class BasicTrackingTests(unittest.TestCase):
    def test_tracks_slot_relative_to_target_with_velocity_feedforward(self) -> None:
        result = controller().command(
            "UAV-02",
            follower((0.0, 0.0, 0.0)),
            target((10.0, 0.0, 0.0), velocity=(1.0, 0.0, 0.0)),
        )

        self.assertTrue(result.active)
        self.assertEqual(result.reason, "tracking_target_slot")
        # target = (10,0,0) + (-5,0,2) = (5,0,2)
        self.assertEqual(result.target_enu_m, (5.0, 0.0, 2.0))
        # velocity feed-forward means vx includes the target's 1.0 m/s.
        self.assertGreater(result.velocity_enu_m_s[0], 1.0)

    def test_missing_velocity_degrades_to_zero_feedforward_not_invalid(self) -> None:
        result = controller().command(
            "UAV-02",
            follower((0.0, 0.0, 0.0)),
            target((10.0, 0.0, 0.0)),  # no velocity key at all
        )

        self.assertTrue(result.active)
        self.assertEqual(result.reason, "tracking_target_slot")

    def test_slot_reached_within_arrival_radius(self) -> None:
        result = controller().command(
            "UAV-02",
            follower((5.0, 0.0, 2.0)),
            target((10.0, 0.0, 0.0), velocity=(0.0, 0.0, 0.0)),
        )

        self.assertEqual(result.reason, "target_slot_reached")
        self.assertEqual(result.velocity_enu_m_s, (0.0, 0.0, 0.0))

    def test_velocity_is_bounded_by_maximum_velocity(self) -> None:
        result = controller(config=FormationConfig(position_gain_s_inv=5.0, maximum_velocity_m_s=1.0)).command(
            "UAV-02",
            follower((0.0, 0.0, 0.0)),
            target((100.0, 0.0, 0.0), velocity=(0.0, 0.0, 0.0)),
        )

        magnitude = sum(v * v for v in result.velocity_enu_m_s) ** 0.5
        self.assertLessEqual(magnitude, 1.0 + 1e-9)


class FailClosedTests(unittest.TestCase):
    def test_unassigned_drone_holds(self) -> None:
        result = controller().command(
            "UAV-03",
            follower((0.0, 0.0, 0.0)),
            target((10.0, 0.0, 0.0), velocity=(0.0, 0.0, 0.0)),
        )

        self.assertFalse(result.active)
        self.assertEqual(result.reason, "formation_slot_unassigned")
        self.assertEqual(result.velocity_enu_m_s, (0.0, 0.0, 0.0))

    def test_invalid_follower_state_holds(self) -> None:
        result = controller().command(
            "UAV-02",
            follower((0.0, 0.0, 0.0), valid=False),
            target((10.0, 0.0, 0.0), velocity=(0.0, 0.0, 0.0)),
        )

        self.assertFalse(result.active)
        self.assertEqual(result.reason, "follower_state_invalid")

    def test_invalid_target_state_holds(self) -> None:
        result = controller().command(
            "UAV-02",
            follower((0.0, 0.0, 0.0)),
            target((10.0, 0.0, 0.0), valid=False),
        )

        self.assertFalse(result.active)
        self.assertEqual(result.reason, "target_state_invalid")

    def test_target_never_treated_as_a_drone_id_lookup(self) -> None:
        """The target is passed explicitly, never looked up from swarm_state
        by id -- confirm the signature enforces this (no drone_id field is
        read from the target dict at all)."""
        result = controller().command(
            "UAV-02",
            follower((0.0, 0.0, 0.0)),
            {"position_enu_m": [10.0, 0.0, 0.0], "valid": True},  # no drone_id key
        )
        self.assertTrue(result.active)


class UniqueSlotTests(unittest.TestCase):
    def test_duplicate_slot_ids_are_rejected(self) -> None:
        with self.assertRaises(ValueError):
            TargetRelativeFormationController(
                (
                    FormationSlot("UAV-02", (-5.0, 0.0, 0.0)),
                    FormationSlot("UAV-02", (5.0, 0.0, 0.0)),
                )
            )


if __name__ == "__main__":
    unittest.main()
