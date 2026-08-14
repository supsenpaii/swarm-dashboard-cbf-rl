from __future__ import annotations

import unittest

from swarm_state import GeodeticOrigin, geodetic_to_enu, target_wgs84_to_state


ORIGIN = GeodeticOrigin(47.397971057728974, 8.546163739800146, 0.0)


class TargetWgs84BridgeTests(unittest.TestCase):
    def test_valid_target_converts_to_the_same_enu_as_geodetic_to_enu(self) -> None:
        lat, lon, alt = 47.398416, 8.546163739800146, 5.0
        expected = geodetic_to_enu(lat, lon, alt, ORIGIN)

        state = target_wgs84_to_state(
            lat, lon, alt, (1.0, 0.0, 0.0), ORIGIN, valid=True, message_age_ms=10.0
        )

        self.assertTrue(state["valid"])
        self.assertEqual(state["reason"], "ok")
        self.assertAlmostEqual(state["position_enu_m"][0], expected[0], places=6)
        self.assertAlmostEqual(state["position_enu_m"][1], expected[1], places=6)
        self.assertAlmostEqual(state["position_enu_m"][2], expected[2], places=6)
        self.assertEqual(state["velocity_enu_m_s"], [1.0, 0.0, 0.0])

    def test_missing_velocity_is_carried_as_none_not_fabricated_zero(self) -> None:
        """The bridge itself must not invent a velocity; degrading a missing
        velocity to zero feed-forward is TargetRelativeFormationController's
        job, not this pure conversion function's."""
        state = target_wgs84_to_state(
            47.398, 8.546, 5.0, None, ORIGIN, valid=True, message_age_ms=10.0
        )

        self.assertTrue(state["valid"])
        self.assertIsNone(state["velocity_enu_m_s"])

    def test_upstream_invalid_flag_is_honoured_without_touching_math(self) -> None:
        state = target_wgs84_to_state(
            47.398, 8.546, 5.0, (0.0, 0.0, 0.0), ORIGIN, valid=False, message_age_ms=2000.0
        )

        self.assertFalse(state["valid"])
        self.assertEqual(state["reason"], "target_invalid")
        self.assertIsNone(state["position_enu_m"])

    def test_missing_origin_fails_closed_with_a_distinct_reason(self) -> None:
        state = target_wgs84_to_state(
            47.398, 8.546, 5.0, None, None, valid=True, message_age_ms=10.0
        )

        self.assertFalse(state["valid"])
        self.assertEqual(state["reason"], "enu_origin_not_configured")

    def test_missing_coordinate_fails_closed(self) -> None:
        state = target_wgs84_to_state(
            None, 8.546, 5.0, None, ORIGIN, valid=True, message_age_ms=10.0
        )

        self.assertFalse(state["valid"])

    def test_out_of_range_coordinate_fails_closed_via_geodetic_to_enu(self) -> None:
        state = target_wgs84_to_state(
            999.0, 8.546, 5.0, None, ORIGIN, valid=True, message_age_ms=10.0
        )

        self.assertFalse(state["valid"])
        self.assertEqual(state["reason"], "target_enu_transform_failed")

    def test_result_never_carries_a_drone_id(self) -> None:
        """A target state must never be mistakable for a drone/peer entry."""
        state = target_wgs84_to_state(
            47.398, 8.546, 5.0, (0.0, 0.0, 0.0), ORIGIN, valid=True, message_age_ms=10.0
        )
        self.assertNotIn("drone_id", state)


if __name__ == "__main__":
    unittest.main()
