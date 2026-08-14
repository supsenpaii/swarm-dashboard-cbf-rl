import math

from mission_plan import MINIMUM_SEGMENT_M
from two_uav_mission_circle_flight import (
    DRONE_IDS,
    SPAWN_ENU_M,
    enu_waypoints_to_geodetic,
    mission_waypoints_enu,
)


def test_crossing_scenarios_start_at_spawn_and_have_valid_segments() -> None:
    for scenario in (
        "diagonal_cross",
        "head_on_swap",
        "opposite_orbit",
        "repeated_bow_tie",
    ):
        paths = mission_waypoints_enu(scenario)
        assert set(paths) == set(DRONE_IDS)
        for drone_id, points in paths.items():
            assert points[0] == SPAWN_ENU_M[drone_id]
            assert all(
                math.dist(points[index], points[(index + 1) % len(points)])
                >= MINIMUM_SEGMENT_M
                for index in range(len(points))
            )


def test_custom_circle_geometry_and_conversion() -> None:
    paths = mission_waypoints_enu(
        "circle", center_enu_m=(-1.5, 3.5), radius_m=5.0
    )
    converted = enu_waypoints_to_geodetic(paths)
    assert all(len(converted[drone_id]) == 24 for drone_id in DRONE_IDS)
