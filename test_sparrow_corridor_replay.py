"""The corridor swap: the geometry a drawn mission actually produces.

Two vehicles sent down one lane in opposite directions cannot resolve by
geometry -- somebody has to give way. The safety matrix does not ask this
question: it spawns each pair on 160 m of open runway, so at 1 m/s the vehicles
never get within 38 m of each other and every layer looks equally competent.
Here the corridor is 30 seconds of closing time wide at every rung, so the
question stays the same shape as speed rises.
"""

from __future__ import annotations

import pytest

from sparrow_corridor_replay import replay


RUNGS = {
    10.0: ("sparrow_active_10ms_corridor.env", 90.0),
    15.0: ("sparrow_active_15ms_corridor.env", 120.0),
    20.0: ("sparrow_active_20ms_corridor.env", 150.0),
}


def test_shadow_mode_cannot_resolve_a_head_on_and_says_so():
    """The result that sent the whole active rung forward.

    In shadow the policy's action is discarded and the coordinator is reset
    every frame, so the only layer left is the barrier -- and a barrier brakes
    along the line of approach, it does not steer around anything. The pair
    stops nose to nose at the required distance and neither ever arrives. That
    is the correct behaviour for a shield and a useless one for a mission,
    which is exactly why shadow is not a flight mode.
    """
    report = replay("sparrow_shadow_1ms_headon.env", duration_s=120.0)

    assert report["mode"] == "shadow"
    assert report["verdict"] == "REPLAY_FAIL_DEADLOCK"
    assert report["first_breach"] is None
    assert report["rl_applied_rate"] == 0.0
    # Stopped at the requirement, not through it.
    assert report["minimum_dynamic_margin_m"] >= 0.0
    assert not report["swapped_ends"]


def test_active_mode_resolves_the_same_head_on():
    report = replay("sparrow_active_1ms_headon.env", duration_s=200.0)

    assert report["mode"] == "active"
    assert report["verdict"] == "REPLAY_PASS"
    assert report["swapped_ends"]
    assert report["first_breach"] is None
    assert report["rl_applied_rate"] == 1.0
    # The layers that actually avoid carry the encounter, so the shield stops
    # being the thing holding the pair apart.
    assert report["cbf_intervention_rate"] < 0.5


@pytest.mark.parametrize("speed_m_s", sorted(RUNGS))
def test_every_rung_swaps_ends_without_breaching(speed_m_s):
    profile, duration_s = RUNGS[speed_m_s]
    report = replay(profile, duration_s=duration_s)

    assert report["verdict"] == "REPLAY_PASS"
    assert report["swapped_ends"]
    assert report["first_breach"] is None
    assert report["minimum_distance_m"] >= 20.0
    assert report["minimum_dynamic_margin_m"] >= 0.0
    assert report["rl_rejected_frames"] == 0


def test_a_profile_the_entry_controller_could_never_fly_is_refused():
    """A trajectory faster than the shared entry ceiling degrades to a crawl.

    It does not crash and it does not stop -- the reference outruns the
    vehicle, the error crosses the re-entry threshold, and the vehicle
    approaches at the ceiling forever. On a dashboard that reads as "flying,
    slowly", which is why it has to be refused at construction instead.
    """
    import os
    from pathlib import Path

    from companion_safety import CompanionSafetyMonitor
    from sparrow_corridor_replay import DRONE_IDS, load_env_profile

    saved = dict(os.environ)
    try:
        load_env_profile("sparrow_active_20ms_corridor.env")
        os.environ["SWARM_FORMATION_MAXIMUM_VELOCITY_M_S"] = "2.0"
        with pytest.raises(ValueError, match="SWARM_FORMATION_MAXIMUM_VELOCITY_M_S"):
            CompanionSafetyMonitor.from_environment("UAV-01", DRONE_IDS)
    finally:
        os.environ.clear()
        os.environ.update(saved)


def test_corner_slowdown_and_avoidance_hold_on_the_same_frames():
    """One square, flown both ways: the product's two halves at once.

    Ninety degree corners force the tracker to brake hard for accuracy, and
    counter-rotation guarantees head-on encounters ON the drawn path. Proving
    corner accuracy on a quiet lap and avoidance on a straight line would leave
    the interesting frames -- a corner entered while yielding -- untested.
    """
    from sparrow_corridor_replay import counter_rotating_square

    report = replay(
        "sparrow_active_15ms_corridor.env",
        duration_s=300.0,
        missions=counter_rotating_square(240.0, 15.0),
    )

    assert report["verdict"] == "REPLAY_PASS"
    assert report["first_breach"] is None
    assert report["minimum_distance_m"] >= 20.0
    assert all(
        visited == "4/4" for visited in report["waypoints_visited"].values()
    )
    # The requested speed is actually reached somewhere on the straights, so
    # the corners are being slowed FOR rather than never reached.
    assert all(
        value == pytest.approx(15.0, abs=0.2)
        for value in report["maximum_speed_m_s"].values()
    )
    # Off-path excursions belong to avoidance; once rejoined the vehicle is
    # back inside the re-entry band rather than trailing the reference. The
    # this read 3.99/4.91 against a plant that could pull 25 m/s^2, then 0.95
    # and 5.13 once the plant was honest -- and now both sit near 4, because a
    # 2.7 s engagement lead brings the vehicle holding its line into the
    # resolution too instead of leaving all of it to the one that yields.
    assert all(
        value < 6.0
        for value in report["maximum_cross_track_clear_of_conflict_m"].values()
    )
    # 0.0 m of margin here is NOT a near miss, and reading it as one sent this
    # comment the wrong way once already. At the worst frame the pair is
    # 22.31 m apart with ZERO closing speed, against a required separation of
    # 22.31 m -- which at zero closing speed is just the static floor:
    # minimum_separation 20 + tracking_reserve 2 + 0.31 of uncertainty. They
    # are 2.3 m clear of the hard minimum and the barrier is holding them
    # exactly where a minimally-invasive filter is supposed to, while both
    # crawl through a corner. What the engagement lead bought is how often it
    # has to act: 51.5% of frames at the old fixed trigger, 21.6% now.
    assert report["minimum_dynamic_margin_m"] == pytest.approx(0.0, abs=0.05)
    assert report["minimum_distance_m"] > 22.0
    assert 0.15 < report["cbf_intervention_rate"] < 0.30
    assert report["coordinated_rate"] > 0.10


def test_corner_accuracy_without_a_conflict_is_sub_metre():
    """The corner claim on its own, with nothing to avoid.

    Separated deliberately: the conflict run's cross-track is dominated by
    deliberate detours, so it can neither confirm nor refute how tightly the
    tracker holds a 90 degree corner at speed.

    It is also the control for how SLOW the square is. Measured 2026-08-18,
    one vehicle with nothing whatsoever to avoid spends 29% of its flight
    below 5 m/s; the conflicting pair spends 36-39%. So most of that square's
    slowness is its own corners -- 90 degrees, 1 m of tracking tolerance,
    4 m/s^2 -- and only about eight points of it belong to the encounter.
    Attributing the whole thing to avoidance would send the next investigation
    into the coordinator, which is not where it is.
    """
    import math
    import os

    from conflict_coordinator import reset_shared_conflict_state
    from sparrow_corridor_replay import (
        DRONE_IDS,
        SPARROW_TAU_S,
        _Vehicle,
        _distance,
        counter_rotating_square,
        load_env_profile,
    )

    reset_shared_conflict_state(DRONE_IDS)
    saved = dict(os.environ)
    try:
        load_env_profile("sparrow_active_15ms_corridor.env")
        from companion_safety import CompanionSafetyMonitor

        subject = CompanionSafetyMonitor.from_environment("UAV-01", DRONE_IDS)
        square = counter_rotating_square(240.0, 15.0)["UAV-01"]
        subject.set_trajectory(square)
    finally:
        os.environ.clear()
        os.environ.update(saved)

    vehicle = _Vehicle([0.0, 0.0, 30.0])
    parked = {
        "position_enu_m": [2000.0, 2000.0, 30.0],
        "velocity_enu_m_s": [0.0, 0.0, 0.0],
        "position_covariance_m2": [0.81, 0.81, 3.17],
        "message_age_ms": 20.0,
        "valid": True,
    }
    worst_cross_track_m = 0.0
    slowest_m_s = math.inf
    fastest_m_s = 0.0
    for step in range(int(140.0 / 0.05)):
        now_s = step * 0.05
        status = subject.evaluate(
            {
                "UAV-01": {
                    "position_enu_m": list(vehicle.position_enu_m),
                    "velocity_enu_m_s": list(vehicle.velocity_enu_m_s),
                    "position_covariance_m2": [0.81, 0.81, 3.17],
                    "message_age_ms": 20.0,
                    "valid": True,
                },
                "UAV-02": parked,
            },
            now_s,
            station_keeping=True,
        )
        vehicle.step(
            status.output_velocity_enu_m_s,
            0.05,
            SPARROW_TAU_S,
            subject.mission_maximum_acceleration_m_s2,
        )
        if now_s < 5.0 or status.nominal_reason != "tracking_trajectory":
            continue
        on_path = square.reference(
            float(square.nearest_time_s(tuple(vehicle.position_enu_m)))
        ).position_enu_m
        worst_cross_track_m = max(
            worst_cross_track_m, _distance(vehicle.position_enu_m, list(on_path))
        )
        speed_m_s = math.sqrt(sum(v * v for v in vehicle.velocity_enu_m_s))
        slowest_m_s = min(slowest_m_s, speed_m_s)
        fastest_m_s = max(fastest_m_s, speed_m_s)

    # 3.489 m before the response lag was modelled, 0.145 m after.
    assert worst_cross_track_m < 0.5
    # It slows for the corner rather than cutting it, and recovers afterwards.
    assert slowest_m_s < 1.0
    assert fastest_m_s > 14.0
