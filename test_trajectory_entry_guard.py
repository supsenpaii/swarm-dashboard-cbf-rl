"""A vehicle must reach the path before the lap clock starts.

Regression for 2026-08-12: the lap clock latched the instant the companion
took authority, so the reference ran away at mission speed from a point the
vehicle was 19 m from. The tracker answered with a permanently saturated
chase and the vehicle ended up on the ground.
"""

from __future__ import annotations

import math

import pytest

from companion_safety import CompanionSafetyMonitor
from conflict_coordinator import ConflictCoordinator, x500_20m_conflict_config
from formation_controller import FormationConfig, FormationSlot
from trajectory_controller import ClosedPolylineTrajectory, LinearTrajectory

SQUARE = ClosedPolylineTrajectory(
    waypoints_enu_m=((0.0, 0.0, 9.0), (20.0, 0.0, 9.0), (20.0, 20.0, 9.0), (0.0, 20.0, 9.0)),
    speed_m_s=1.5,
)


def monitor(maximum_velocity_m_s: float = 2.0) -> CompanionSafetyMonitor:
    """The ceiling is a parameter because the tracker and the entry controller
    share it, so a test that installs a faster leg has to raise it too -- a
    mission above the ceiling is refused now, being the exact runaway this
    file's docstring is about."""
    subject = CompanionSafetyMonitor(
        drone_id="UAV-01",
        peer_ids=("UAV-02",),
        leader_id="UAV-01",
        slots=(FormationSlot("UAV-02", (-10.0, 0.0, 0.0)),),
        formation_config=FormationConfig(maximum_velocity_m_s=maximum_velocity_m_s),
    )
    subject.set_trajectory(SQUARE)
    return subject


def state(position, peer=(0.0, 60.0, 9.0)):
    return {
        "UAV-01": {
            "valid": True,
            "position_enu_m": position,
            "velocity_enu_m_s": (0.0, 0.0, 0.0),
            "position_covariance_m2": (0.04, 0.04, 0.07),
            "message_age_ms": 10.0,
        },
        "UAV-02": {
            "valid": True,
            "position_enu_m": peer,
            "velocity_enu_m_s": (0.0, 0.0, 0.0),
            "position_covariance_m2": (0.04, 0.04, 0.07),
            "message_age_ms": 10.0,
        },
    }


def test_far_from_the_path_it_approaches_instead_of_starting_the_lap():
    subject = monitor()
    # The 2026-08-12 geometry: hovering low near the origin, path at 9 m.
    status = subject.evaluate(state((0.0, 0.0, 5.0)), 1000.0, station_keeping=True)
    assert status.nominal_reason == "trajectory_entering"
    assert subject.trajectory_start_monotonic_s is None
    # Straight up toward the path, not a saturated chase after a moving point.
    velocity = status.nominal_velocity_enu_m_s
    assert velocity[2] > 0.0
    assert math.sqrt(sum(v * v for v in velocity)) <= SQUARE.speed_m_s + 1e-9


def test_the_lap_starts_only_once_the_vehicle_is_on_the_path():
    subject = monitor()
    subject.evaluate(state((0.0, 0.0, 5.0)), 1000.0, station_keeping=True)
    assert subject.trajectory_start_monotonic_s is None
    status = subject.evaluate(state((0.0, 0.5, 9.0)), 1001.0, station_keeping=True)
    assert status.nominal_reason == "tracking_trajectory"
    assert subject.trajectory_start_monotonic_s == 1001.0


def test_the_lap_begins_at_the_nearest_point_not_at_waypoint_zero():
    subject = monitor()
    # Standing on the far edge, three quarters of the way round the loop.
    standing = SQUARE.reference(SQUARE.lap_duration_s() * 0.75).position_enu_m
    status = subject.evaluate(state(standing), 1000.0, station_keeping=True)
    assert status.nominal_reason == "tracking_trajectory"
    assert subject.trajectory_entry_time_s == pytest.approx(
        SQUARE.lap_duration_s() * 0.75, abs=0.1
    )
    # It picks up the lap from there, so the tracking error stays small.
    assert status.nominal_position_error_m < 0.5


def test_an_unreachable_entry_point_hovers_rather_than_dashing():
    """Fail closed: never latch, never exceed mission speed, never give up."""
    subject = monitor()
    for step in range(50):
        status = subject.evaluate(
            state((0.0, 0.0, 5.0)), 1000.0 + step, station_keeping=True
        )
        assert status.nominal_reason == "trajectory_entering"
        assert math.sqrt(
            sum(v * v for v in status.nominal_velocity_enu_m_s)
        ) <= SQUARE.speed_m_s + 1e-9
    assert subject.trajectory_start_monotonic_s is None


def test_losing_authority_forgets_the_entry_point():
    subject = monitor()
    subject.evaluate(state((0.0, 0.0, 5.0)), 1000.0, station_keeping=True)
    assert subject.trajectory_entry_time_s is not None
    subject.evaluate(state((0.0, 0.0, 5.0)), 1001.0, station_keeping=False)
    assert subject.trajectory_entry_time_s is None
    assert subject.trajectory_start_monotonic_s is None


def test_an_invalid_own_state_holds_instead_of_entering():
    subject = monitor()
    broken = state((0.0, 0.0, 5.0))
    broken["UAV-01"]["valid"] = False
    status = subject.evaluate(broken, 1000.0, station_keeping=True)
    assert status.nominal_reason == "trajectory_entry_state_invalid"
    assert status.nominal_velocity_enu_m_s == (0.0, 0.0, 0.0)
    assert subject.trajectory_start_monotonic_s is None


def test_elapsed_time_cannot_leave_a_stationary_vehicle_behind():
    """Polygon progress comes from position, so wall time cannot run away."""
    subject = monitor()
    on_path = SQUARE.reference(0.0).position_enu_m
    subject.evaluate(state(on_path), 1000.0, station_keeping=True)
    first_entry = subject.trajectory_entry_time_s

    status = subject.evaluate(state(on_path), 1030.0, station_keeping=True)
    assert status.nominal_reason == "tracking_trajectory"
    assert subject.trajectory_start_monotonic_s == 1000.0
    assert subject.trajectory_entry_time_s == first_entry
    assert status.nominal_position_error_m < 0.5


def test_falling_far_off_the_path_returns_to_the_bounded_approach():
    """The real 2026-08-12 failure: 9.5 m median error, command saturated,
    vehicle on the ground a quarter of the time, nothing pulling it back."""
    subject = monitor()
    on_path = SQUARE.reference(0.0).position_enu_m
    subject.evaluate(state(on_path), 1000.0, station_keeping=True)
    assert subject.trajectory_start_monotonic_s is not None

    # Sagging to the ground under a path at 9 m, as it actually did.
    status = subject.evaluate(state((0.0, 0.0, 0.5)), 1005.0, station_keeping=True)
    assert status.nominal_reason == "trajectory_entering"
    assert subject.trajectory_start_monotonic_s is None
    # Bounded by mission speed again, not by the higher tracking ceiling.
    assert math.sqrt(
        sum(v * v for v in status.nominal_velocity_enu_m_s)
    ) <= SQUARE.speed_m_s + 1e-9
    # And pointing back up at the path rather than chasing along it.
    assert status.nominal_velocity_enu_m_s[2] > 0.0


def test_normal_tracking_lag_does_not_trip_the_re_entry():
    """The tracker's own steady-state lag must stay well inside the band."""
    subject = monitor()
    on_path = SQUARE.reference(0.0).position_enu_m
    subject.evaluate(state(on_path), 1000.0, station_keeping=True)
    # 2 s later, 3 m of reference travel: inside the 5 m re-entry error.
    status = subject.evaluate(state(on_path), 1002.0, station_keeping=True)
    assert status.nominal_reason == "tracking_trajectory"
    assert subject.trajectory_start_monotonic_s is not None


def test_linear_reentry_joins_at_current_progress_instead_of_flying_back_to_start():
    subject = monitor(10.0)
    leg = LinearTrajectory((-70.0, 0.0, 9.0), (70.0, 0.0, 9.0), 10.0)
    subject.set_trajectory(leg)
    subject.evaluate(state((-70.0, 0.0, 9.0)), 1000.0, station_keeping=True)

    status = subject.evaluate(
        state((0.0, 20.0, 9.0)), 1005.0, station_keeping=True
    )

    assert status.nominal_reason == "trajectory_entering"
    assert subject.trajectory_entry_time_s == pytest.approx(7.0)
    entry = leg.reference(subject.trajectory_entry_time_s)
    assert entry.position_enu_m == pytest.approx((0.0, 0.0, 9.0))
    assert status.nominal_velocity_enu_m_s[0] == pytest.approx(0.0)


def test_linear_reentry_never_regresses_behind_scheduled_progress():
    subject = monitor(10.0)
    leg = LinearTrajectory((70.0, 0.0, 9.0), (-70.0, 0.0, 9.0), 10.0)
    subject.set_trajectory(leg)
    subject.evaluate(state((70.0, 0.0, 9.0)), 1000.0, station_keeping=True)

    status = subject.evaluate(
        state((75.0, -34.0, 9.0)), 1015.0, station_keeping=True
    )

    assert status.nominal_reason == "trajectory_entering"
    assert subject.trajectory_entry_time_s == pytest.approx(15.0)
    assert leg.reference(subject.trajectory_entry_time_s).position_enu_m == (
        -70.0,
        0.0,
        9.0,
    )
    assert status.nominal_velocity_enu_m_s[0] < 0.0


def test_the_two_radii_cannot_chatter():
    subject = monitor()
    assert subject.trajectory_reentry_error_m > subject.trajectory_entry_radius_m


def latched_coordinator() -> ConflictCoordinator:
    """A coordinator holding a confirmed head-on encounter."""
    coordinators = ConflictCoordinator.pair(
        ("UAV-01", "UAV-02"), x500_20m_conflict_config()
    )
    closing = {
        "UAV-01": {
            "valid": True,
            "position_enu_m": (-28.0, 0.0, 9.0),
            "velocity_enu_m_s": (10.0, 0.0, 0.0),
        },
        "UAV-02": {
            "valid": True,
            "position_enu_m": (28.0, 0.0, 9.0),
            "velocity_enu_m_s": (-10.0, 0.0, 0.0),
        },
    }
    subject = coordinators["UAV-01"]
    subject.filter((10.0, 0.0, 0.0), (10.0, 0.0, 0.0), closing)
    assert subject.active
    return subject


def test_a_yield_on_a_linear_leg_is_not_treated_as_lost_tracking():
    """A one-sided yield in a shared corridor REQUIRES leaving the path by the
    whole separation floor -- ~22.6 m at 20 m -- against a 5 m re-entry band.
    Re-entering mid-encounter hands the coordinator an entry command pointing
    back at the path instead of the corridor direction. Measured on the 1 m/s
    head-on replay: dynamic margin 0.000 -> 5.585 m, CBF interventions 15.3%
    -> 0%, and the peer's own cross-track 19.18 -> 0.54 m."""
    subject = monitor(10.0)
    leg = LinearTrajectory((-70.0, 0.0, 9.0), (70.0, 0.0, 9.0), 10.0)
    subject.set_trajectory(leg)
    subject.evaluate(state((-70.0, 0.0, 9.0)), 1000.0, station_keeping=True)
    subject.cbf_rl_shadow.coordinator = latched_coordinator()

    status = subject.evaluate(
        state((0.0, 20.0, 9.0)), 1005.0, station_keeping=True
    )

    assert status.nominal_reason == "tracking_trajectory"
    assert subject.trajectory_start_monotonic_s is not None


def test_a_yield_on_a_closed_polygon_still_re_enters():
    """A closed path has nowhere to yield to -- it comes back. Suppressing
    re-entry there cost the 240 m counter-rotating square 2.399 m of dynamic
    margin, down to 0.174 m."""
    subject = monitor()
    on_path = SQUARE.reference(0.0).position_enu_m
    subject.evaluate(state(on_path), 1000.0, station_keeping=True)
    subject.cbf_rl_shadow.coordinator = latched_coordinator()

    status = subject.evaluate(state((0.0, 0.0, 0.5)), 1005.0, station_keeping=True)

    assert status.nominal_reason == "trajectory_entering"
    assert subject.trajectory_start_monotonic_s is None
