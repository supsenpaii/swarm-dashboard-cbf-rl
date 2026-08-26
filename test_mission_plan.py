"""A drawn mission must become a flyable loop, or be refused with a reason."""

from __future__ import annotations

import math

import pytest

from mission_plan import (
    MissionLimits,
    MissionRejected,
    closest_approach_m,
    review_missions,
    unreachable_destinations,
    validate_mission,
)
from trajectory_controller import ClosedPolylineTrajectory, LinearTrajectory
from swarm_state import GeodeticOrigin

ORIGIN = GeodeticOrigin(latitude_deg=47.3977508, longitude_deg=8.5456073, altitude_msl_m=488.0)
LIMITS = MissionLimits(
    geofence_min_enu_m=(-100.0, -100.0, 0.0),
    geofence_max_enu_m=(100.0, 100.0, 50.0),
    maximum_speed_m_s=1.5,
    minimum_separation_m=4.0,
)
# Roughly 1 m of latitude / longitude at the origin, for drawing test squares.
METRE_LAT = 1.0 / 111_320.0
METRE_LON = METRE_LAT / math.cos(math.radians(ORIGIN.latitude_deg))


def square(side_m: float, *, east_offset_m: float = 0.0, north_offset_m: float = 0.0):
    corners = ((0.0, 0.0), (side_m, 0.0), (side_m, side_m), (0.0, side_m))
    return [
        {
            "latitude_deg": ORIGIN.latitude_deg + (north + north_offset_m) * METRE_LAT,
            "longitude_deg": ORIGIN.longitude_deg + (east + east_offset_m) * METRE_LON,
        }
        for east, north in corners
    ]


def mission(**overrides):
    payload = {"waypoints": square(20.0), "altitude_m": 9.0, "speed_m_s": 1.5}
    payload.update(overrides)
    return payload


def test_a_drawn_square_becomes_a_flyable_lap():
    trajectory = validate_mission(mission(), ORIGIN, LIMITS)
    assert trajectory.perimeter_m() == pytest.approx(80.0, abs=0.5)
    assert trajectory.lap_duration_s() == pytest.approx(80.0 / 1.5, abs=0.5)
    assert not trajectory.is_finished(10_000.0)
    assert all(point[2] == 9.0 for point in trajectory.waypoints_enu_m)


def test_speed_above_the_tracker_headroom_is_refused():
    with pytest.raises(MissionRejected, match="speed"):
        validate_mission(mission(speed_m_s=2.0), ORIGIN, LIMITS)


def test_default_mission_speed_keeps_half_the_cbf_limit(monkeypatch):
    monkeypatch.setenv("SWARM_CBF_MAXIMUM_VELOCITY_M_S", "2.0")
    monkeypatch.delenv("SWARM_MISSION_MAXIMUM_VELOCITY_M_S", raising=False)
    assert MissionLimits.from_environment().maximum_speed_m_s == 1.0


def test_validated_profile_can_raise_mission_speed_to_cbf_limit(monkeypatch):
    monkeypatch.setenv("SWARM_CBF_MAXIMUM_VELOCITY_M_S", "10.0")
    monkeypatch.setenv("SWARM_MISSION_MAXIMUM_VELOCITY_M_S", "10.0")
    assert MissionLimits.from_environment().maximum_speed_m_s == 10.0


def test_mission_speed_override_cannot_exceed_cbf_limit(monkeypatch):
    monkeypatch.setenv("SWARM_CBF_MAXIMUM_VELOCITY_M_S", "10.0")
    monkeypatch.setenv("SWARM_MISSION_MAXIMUM_VELOCITY_M_S", "30.0")
    assert MissionLimits.from_environment().maximum_speed_m_s == 10.0


def test_altitude_outside_the_geofence_is_refused():
    with pytest.raises(MissionRejected, match="altitude"):
        validate_mission(mission(altitude_m=80.0), ORIGIN, LIMITS)


def test_waypoint_outside_the_geofence_is_refused():
    with pytest.raises(MissionRejected, match="geofence"):
        validate_mission(mission(waypoints=square(20.0, east_offset_m=250.0)), ORIGIN, LIMITS)


def test_two_waypoints_draw_an_open_line_not_a_loop():
    """A survey leg is a mission too, and it is not a lap.

    This used to demand three waypoints. Two now give a LinearTrajectory,
    flown once and held at the far end, which is also the trajectory kind the
    controller follows from measured progress rather than a clock.
    """
    trajectory = validate_mission(
        mission(waypoints=square(20.0)[:2]), ORIGIN, LIMITS
    )

    assert isinstance(trajectory, LinearTrajectory)
    assert not isinstance(trajectory, ClosedPolylineTrajectory)
    # The leg, not a perimeter: no closing edge is counted.
    assert trajectory.perimeter_m() == pytest.approx(20.0, abs=0.5)
    assert trajectory.lap_duration_s() == pytest.approx(
        trajectory.perimeter_m() / trajectory.speed_m_s
    )


def test_one_waypoint_is_still_not_a_mission():
    with pytest.raises(MissionRejected, match="2 to"):
        validate_mission(mission(waypoints=square(20.0)[:1]), ORIGIN, LIMITS)


def test_an_open_line_still_has_to_be_long_enough_to_fly():
    close_together = [
        {"latitude_deg": 47.397971, "longitude_deg": 8.546163},
        {"latitude_deg": 47.397972, "longitude_deg": 8.546163},
    ]
    with pytest.raises(MissionRejected, match="closer than"):
        validate_mission(mission(waypoints=close_together), ORIGIN, LIMITS)


def test_malformed_waypoints_are_refused_rather_than_coerced():
    for broken in ("not-a-list", [{"latitude_deg": "x", "longitude_deg": 8.5}] * 3):
        with pytest.raises(MissionRejected):
            validate_mission(mission(waypoints=broken), ORIGIN, LIMITS)
    with pytest.raises(MissionRejected):
        validate_mission(mission(waypoints=[{"latitude_deg": 47.4, "longitude_deg": 8.5}] * 3), ORIGIN, LIMITS)


def test_separated_paths_need_no_cbf_intervention():
    near = validate_mission(mission(), ORIGIN, LIMITS)
    far = validate_mission(
        mission(waypoints=square(20.0, north_offset_m=40.0)), ORIGIN, LIMITS
    )
    assert closest_approach_m(near, far) >= 4.0
    report = review_missions({"UAV-01": near, "UAV-02": far}, LIMITS)
    assert report["geometrically_separated"] is True
    assert report["conflicts"] == []


def test_overlapping_paths_are_reported_as_relying_on_cbf():
    first = validate_mission(mission(), ORIGIN, LIMITS)
    second = validate_mission(
        mission(waypoints=square(20.0, north_offset_m=2.0)), ORIGIN, LIMITS
    )
    report = review_missions({"UAV-01": first, "UAV-02": second}, LIMITS)
    assert report["geometrically_separated"] is False
    assert report["conflicts"][0]["drones"] == ["UAV-01", "UAV-02"]
    assert report["conflicts"][0]["closest_approach_m"] < 4.0


def test_closest_approach_never_overstates_the_gap():
    """The bound must err low: it is what a go/no-go decision rests on."""
    first = validate_mission(mission(), ORIGIN, LIMITS)
    second = validate_mission(
        mission(waypoints=square(20.0, north_offset_m=30.0)), ORIGIN, LIMITS
    )
    coarse = closest_approach_m(first, second, sample_step_m=2.0)
    fine = closest_approach_m(first, second, sample_step_m=0.1)
    assert coarse <= fine + 1e-9


def bow_tie(side_m: float):
    """A square with two corners swapped: the classic self-crossing loop."""
    corners = ((0.0, 0.0), (side_m, 0.0), (0.0, side_m), (side_m, side_m))
    return [
        {
            "latitude_deg": ORIGIN.latitude_deg + north * METRE_LAT,
            "longitude_deg": ORIGIN.longitude_deg + east * METRE_LON,
        }
        for east, north in corners
    ]


def test_a_self_crossing_mission_is_refused():
    """Nearest-point progress cannot tell the branches apart at a crossing."""
    with pytest.raises(MissionRejected, match="cross"):
        validate_mission(mission(waypoints=bow_tie(20.0)), ORIGIN, LIMITS)


def test_convex_and_star_shaped_loops_still_pass():
    from mission_plan import self_intersecting_pair

    square_enu = [(0.0, 0.0, 9.0), (10.0, 0.0, 9.0), (10.0, 10.0, 9.0), (0.0, 10.0, 9.0)]
    triangle = [(0.0, 0.0, 9.0), (10.0, 0.0, 9.0), (5.0, 9.0, 9.0)]
    # Non-convex but simple: an L. Adjacent segments touch and must not count.
    ell = [
        (0.0, 0.0, 9.0), (20.0, 0.0, 9.0), (20.0, 6.0, 9.0),
        (6.0, 6.0, 9.0), (6.0, 20.0, 9.0), (0.0, 20.0, 9.0),
    ]
    for loop in (square_enu, triangle, ell):
        assert self_intersecting_pair(loop) is None


# --- destination conflicts -------------------------------------------------
# `review_missions` above asks whether the PATHS pass close, and is advisory
# because CBF resolves a crossing. These ask whether the ENDPOINTS can both be
# occupied, which CBF cannot resolve at any gain: the pair hovers at the
# barrier short of both points until someone cancels.

HOLD_LIMITS = MissionLimits(
    geofence_min_enu_m=(-100.0, -100.0, 0.0),
    geofence_max_enu_m=(100.0, 100.0, 50.0),
    maximum_speed_m_s=1.5,
    minimum_separation_m=4.0,
    station_keeping_separation_m=22.0,
)


def leg(east_m: float, north_m: float, *, altitude_m: float = 9.0):
    """An open two-waypoint leg from the origin to one ENU point."""
    return validate_mission(
        {
            "waypoints": [
                {"latitude_deg": ORIGIN.latitude_deg, "longitude_deg": ORIGIN.longitude_deg},
                {
                    "latitude_deg": ORIGIN.latitude_deg + north_m * METRE_LAT,
                    "longitude_deg": ORIGIN.longitude_deg + east_m * METRE_LON,
                },
            ],
            "altitude_m": altitude_m,
            "speed_m_s": 1.5,
        },
        ORIGIN,
        HOLD_LIMITS,
    )


def test_two_points_closer_than_the_hold_floor_are_refused():
    conflicts = unreachable_destinations(
        {"UAV-01": leg(40.0, 0.0), "UAV-02": leg(45.0, 0.0)}, HOLD_LIMITS
    )
    assert len(conflicts) == 1
    assert conflicts[0]["drones"] == ["UAV-01", "UAV-02"]
    assert conflicts[0]["destination_separation_m"] == pytest.approx(5.0, abs=0.2)
    assert conflicts[0]["station_keeping_separation_m"] == 22.0


def test_two_points_beyond_the_hold_floor_are_allowed():
    assert not unreachable_destinations(
        {"UAV-01": leg(-30.0, 0.0), "UAV-02": leg(30.0, 0.0)}, HOLD_LIMITS
    )


def test_altitude_counts_because_the_barrier_is_three_dimensional():
    """The remedy the rejection message offers has to actually work: the same
    two points 5 m apart horizontally become legal once the altitudes split."""
    assert not unreachable_destinations(
        {
            "UAV-01": leg(40.0, 0.0, altitude_m=8.0),
            "UAV-02": leg(45.0, 0.0, altitude_m=30.0),
        },
        HOLD_LIMITS,
    )


def test_a_drawn_lap_has_no_destination_to_conflict_over():
    """A closed polyline never holds anywhere, so overlapping loops are not a
    destination conflict -- they are a crossing, which is review_missions' job
    and which four certified scenarios show CBF resolves."""
    assert not unreachable_destinations(
        {
            "UAV-01": validate_mission(mission(), ORIGIN, HOLD_LIMITS),
            "UAV-02": validate_mission(mission(), ORIGIN, HOLD_LIMITS),
        },
        HOLD_LIMITS,
    )


def test_the_hold_floor_is_separation_plus_the_tracking_reserve(monkeypatch):
    """At rest every speed-derived term of the CBF's required separation is
    zero; these two are what is left, and the map draws its disc at exactly
    this radius."""
    monkeypatch.setenv("SWARM_CBF_MINIMUM_SEPARATION_M", "20.0")
    monkeypatch.setenv("SWARM_CBF_TRACKING_RESERVE_M", "2.0")
    assert MissionLimits.from_environment().station_keeping_separation_m == 22.0
