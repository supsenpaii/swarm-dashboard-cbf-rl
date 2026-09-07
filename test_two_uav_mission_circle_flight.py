"""The scenario geometry, which has to survive a change of envelope.

The five scenarios are drawn against two spawns 5.39 m apart -- a picture
from the 4 m separation envelope. The high-speed profile asks for 20 m and its coordinator
does not engage until 120 m, so the picture has to be fitted onto the real
spawns before it means anything. These tests pin that the fit preserves the
shape and refuses when it cannot.
"""

from __future__ import annotations

import math
import unittest

from two_uav_mission_circle_flight import (
    DRONE_IDS,
    SPAWN_ENU_M,
    FlightAbort,
    fit_to_spawns,
    mission_waypoints_enu,
)

CROSSING_SCENARIOS = (
    "diagonal_cross",
    "head_on_swap",
    "opposite_orbit",
    "repeated_bow_tie",
)


def spread(points):
    return max(math.dist(a[:2], b[:2]) for a in points for b in points)


class FitToSpawnsTests(unittest.TestCase):
    def test_it_scales_the_picture_by_the_spawn_separation(self) -> None:
        nominal = mission_waypoints_enu("diagonal_cross")
        # 300 m apart on the same axis: pure scale, no rotation.
        fitted = fit_to_spawns(nominal, (0.0, 0.0, 9.0), (-300.0, 120.0, 9.0))

        factor = math.dist((0.0, 0.0), (-300.0, 120.0)) / math.dist(
            SPAWN_ENU_M[DRONE_IDS[0]][:2], SPAWN_ENU_M[DRONE_IDS[1]][:2]
        )
        for drone_id in DRONE_IDS:
            self.assertAlmostEqual(
                spread(fitted[drone_id]), spread(nominal[drone_id]) * factor, places=3
            )

    def test_it_preserves_every_angle(self) -> None:
        """A similarity transform is the whole point: the crossing geometry
        each scenario was designed around must survive being made real."""

        def turn_angles(points):
            out = []
            for i in range(1, len(points) - 1):
                a = (points[i][0] - points[i - 1][0], points[i][1] - points[i - 1][1])
                b = (points[i + 1][0] - points[i][0], points[i + 1][1] - points[i][1])
                if math.hypot(*a) < 1e-9 or math.hypot(*b) < 1e-9:
                    continue
                out.append(
                    math.atan2(a[0] * b[1] - a[1] * b[0], a[0] * b[0] + a[1] * b[1])
                )
            return out

        for scenario in CROSSING_SCENARIOS:
            with self.subTest(scenario=scenario):
                nominal = mission_waypoints_enu(scenario)
                fitted = fit_to_spawns(nominal, (10.0, -4.0, 9.0), (250.0, 190.0, 9.0))
                for drone_id in DRONE_IDS:
                    for before, after in zip(
                        turn_angles(nominal[drone_id]), turn_angles(fitted[drone_id])
                    ):
                        self.assertAlmostEqual(before, after, places=6)

    def test_the_two_paths_still_cross(self) -> None:
        """Scaled apart, a crossing scenario must still stage a conflict."""
        for scenario in CROSSING_SCENARIOS:
            with self.subTest(scenario=scenario):
                fitted = fit_to_spawns(
                    mission_waypoints_enu(scenario),
                    (0.0, 0.0, 9.0),
                    (-260.0, 104.0, 9.0),
                )
                closest = min(
                    math.dist(a[:2], b[:2])
                    for a in fitted[DRONE_IDS[0]]
                    for b in fitted[DRONE_IDS[1]]
                )
                self.assertLess(closest, 20.0, f"{scenario} no longer converges")

    def test_overlapping_spawns_fail_closed(self) -> None:
        """Unscaled is worse than refused: right angles, wrong distances."""
        with self.assertRaises(FlightAbort):
            fit_to_spawns(
                mission_waypoints_enu("diagonal_cross"),
                (5.0, 5.0, 9.0),
                (5.0, 5.0, 9.0),
            )


if __name__ == "__main__":
    unittest.main()
