"""A drawn mission must become a flyable loop, or be refused with a reason."""

from __future__ import annotations

import math

import pytest

from mission_plan import (
    MissionLimits,
    MissionRejected,
    closest_approach_m,
    review_missions,
    validate_mission,
)
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


def test_a_loop_needs_at_least_three_waypoints():
    with pytest.raises(MissionRejected, match="3 to"):
        validate_mission(mission(waypoints=square(20.0)[:2]), ORIGIN, LIMITS)


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
