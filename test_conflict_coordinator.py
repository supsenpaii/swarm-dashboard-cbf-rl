from dataclasses import replace
import math

import pytest

from conflict_coordinator import (
    ConflictCoordinator,
    ConflictCoordinatorConfig,
    reset_shared_conflict_state,
    x500_20m_conflict_config,
)


def states(distance: float = 10.0) -> dict[str, dict[str, object]]:
    return {
        "UAV-01": {
            "valid": True,
            "position_enu_m": (-distance / 2.0, 0.0, 9.0),
            "velocity_enu_m_s": (1.0, 0.0, 0.0),
        },
        "UAV-02": {
            "valid": True,
            "position_enu_m": (distance / 2.0, 0.0, 9.0),
            "velocity_enu_m_s": (-1.0, 0.0, 0.0),
        },
    }


def test_head_on_encounter_has_one_priority_and_one_yield_role() -> None:
    config = ConflictCoordinatorConfig(
        reserve_separation_m=7.0,
        release_distance_m=8.0,
        encounter_reset_distance_m=9.0,
    )
    first = ConflictCoordinator("UAV-01", "UAV-02", config)
    second = ConflictCoordinator("UAV-02", "UAV-01", config)
    state = states()

    first_candidate = (0.0, 1.0, 0.0)
    second_candidate = (0.0, -1.0, 0.0)
    first_velocity, first_status = first.filter(
        first_candidate, (1.0, 0.0, 0.0), state
    )
    second_velocity, second_status = second.filter(
        second_candidate, (-1.0, 0.0, 0.0), state
    )

    assert first_status["role"] == "priority"
    assert second_status["role"] == "yield"
    assert first_status["priority_drone_id"] == second_status["priority_drone_id"]
    assert first_velocity == (1.0, 0.0, 0.0)
    assert second_velocity == second_candidate


def test_hysteresis_does_not_release_on_one_separating_frame() -> None:
    config = ConflictCoordinatorConfig(release_frames=2)
    coordinator = ConflictCoordinator("UAV-01", "UAV-02", config)
    coordinator.filter((1.0, 0.0, 0.0), (1.0, 0.0, 0.0), states())
    # Regression for the SITL head-on mission: the vehicles had safely passed
    # one another but never reached the old 8 m release distance, so the yield
    # role remained latched and starved mission progress.
    separating = states(6.5)
    separating["UAV-01"]["velocity_enu_m_s"] = (-1.0, 0.0, 0.0)
    separating["UAV-02"]["velocity_enu_m_s"] = (1.0, 0.0, 0.0)

    _, first = coordinator.filter(
        (1.0, 0.0, 0.0), (1.0, 0.0, 0.0), separating
    )
    _, second = coordinator.filter(
        (1.0, 0.0, 0.0), (1.0, 0.0, 0.0), separating
    )

    assert first["active"]
    assert not second["active"]


def test_released_conflict_keeps_priority_until_pair_is_fully_clear() -> None:
    coordinator = ConflictCoordinator(
        "UAV-01", "UAV-02", ConflictCoordinatorConfig(release_frames=1)
    )
    _, first = coordinator.filter(
        (1.0, 0.0, 0.0), (1.0, 0.0, 0.0), states()
    )
    separating = states(6.5)
    separating["UAV-01"]["velocity_enu_m_s"] = (-1.0, 0.0, 0.0)
    separating["UAV-02"]["velocity_enu_m_s"] = (1.0, 0.0, 0.0)
    coordinator.filter((1.0, 0.0, 0.0), (1.0, 0.0, 0.0), separating)

    _, resumed = coordinator.filter(
        (1.0, 0.0, 0.0), (1.0, 0.0, 0.0), states()
    )

    assert resumed["active"]
    assert resumed["priority_drone_id"] == first["priority_drone_id"]
    assert resumed["encounter"] == first["encounter"]


def test_priority_alternates_between_encounters() -> None:
    coordinator = ConflictCoordinator(
        "UAV-01", "UAV-02", ConflictCoordinatorConfig(release_frames=1)
    )
    _, first = coordinator.filter(
        (1.0, 0.0, 0.0), (1.0, 0.0, 0.0), states()
    )
    separating = states(9.0)
    separating["UAV-01"]["velocity_enu_m_s"] = (-1.0, 0.0, 0.0)
    separating["UAV-02"]["velocity_enu_m_s"] = (1.0, 0.0, 0.0)
    coordinator.filter((1.0, 0.0, 0.0), (1.0, 0.0, 0.0), separating)
    _, second = coordinator.filter(
        (1.0, 0.0, 0.0), (1.0, 0.0, 0.0), states()
    )

    assert first["priority_drone_id"] == "UAV-01"
    assert second["priority_drone_id"] == "UAV-02"


def test_x500_clear_path_uses_mission_velocity_not_rl_candidate() -> None:
    coordinator = ConflictCoordinator(
        "UAV-01", "UAV-02", x500_20m_conflict_config()
    )
    # Genuinely clear: outside the engagement floor AND outside the horizon.
    # This used to read states(20.0), which is a head-on pair closing inside
    # the config's own 20 m reserve -- "clear" only because 2 m/s closing puts
    # the closest approach 10 s away, past the 8 s horizon. That was the bug
    # the floor exists to close, not a property worth pinning.
    state = states(80.0)
    candidate = (0.2, 0.3, 0.0)
    mission = (1.0, 0.0, 0.0)

    output, status = coordinator.filter(candidate, mission, state)

    assert output == mission
    assert status["role"] == "clear"
    assert not status["active"]
    assert math.isfinite(status["predicted_miss_distance_m"])


def test_legacy_clear_path_keeps_the_existing_candidate_behavior() -> None:
    coordinator = ConflictCoordinator("UAV-01", "UAV-02")
    candidate = (0.2, 0.3, 0.0)

    output, status = coordinator.filter(
        candidate, (1.0, 0.0, 0.0), states(20.0)
    )

    assert output == candidate
    assert status["role"] == "clear"


def test_x500_20m_profile_yields_before_the_dynamic_braking_boundary() -> None:
    coordinators = ConflictCoordinator.pair(
        ("UAV-01", "UAV-02"), x500_20m_conflict_config()
    )
    state = states(100.0)
    state["UAV-01"]["velocity_enu_m_s"] = (10.0, 0.0, 0.0)
    state["UAV-02"]["velocity_enu_m_s"] = (-10.0, 0.0, 0.0)

    outputs = {
        drone: coordinators[drone].filter(
            state[drone]["velocity_enu_m_s"],
            state[drone]["velocity_enu_m_s"],
            state,
        )
        for drone in coordinators
    }

    statuses = [status for _, status in outputs.values()]
    assert {status["role"] for status in statuses} == {"priority", "yield"}
    yielding_velocity = next(
        velocity
        for velocity, status in outputs.values()
        if status["role"] == "yield"
    )
    assert abs(yielding_velocity[0]) < 10.0


def test_x500_yield_keeps_progress_and_altitude_while_latching_a_side_step() -> None:
    coordinators = ConflictCoordinator.pair(
        ("UAV-01", "UAV-02"), x500_20m_conflict_config()
    )
    state = states(100.0)
    state["UAV-01"]["velocity_enu_m_s"] = (10.0, 0.0, 0.0)
    state["UAV-02"]["velocity_enu_m_s"] = (-10.0, 0.0, 0.0)
    mission = (-9.4, 0.0, 0.25)

    output, status = coordinators["UAV-02"].filter(
        (9.0, -0.1, 8.0), mission, state
    )

    assert status["role"] == "yield"
    assert output[0] < 0.0
    assert math.isclose(abs(output[1]), 3.0)
    assert output[2] == mission[2]
    assert math.isclose(math.hypot(output[0], output[1]), 9.4)


def test_x500_12ms_profile_uses_a_wider_lane_change() -> None:
    assert x500_20m_conflict_config().yield_lateral_speed_m_s == 3.0
    assert x500_20m_conflict_config(12.0).yield_lateral_speed_m_s == 8.0


def test_x500_15ms_profile_triggers_before_its_dynamic_boundary() -> None:
    config = x500_20m_conflict_config(15.0)

    assert config.yield_lateral_speed_m_s == 9.5
    assert config.trigger_distance_m > 150.0


def test_x500_release_does_not_reactivate_the_same_encounter() -> None:
    config = replace(x500_20m_conflict_config(), release_frames=1)
    coordinator = ConflictCoordinator("UAV-02", "UAV-01", config)
    closing = states(100.0)
    closing["UAV-01"]["velocity_enu_m_s"] = (10.0, 0.0, 0.0)
    closing["UAV-02"]["velocity_enu_m_s"] = (-10.0, 0.0, 0.0)
    coordinator.filter((-10.0, 0.0, 0.0), (-10.0, 0.0, 0.0), closing)
    stopped = states(70.0)
    stopped["UAV-01"]["velocity_enu_m_s"] = (0.0, 0.0, 0.0)
    stopped["UAV-02"]["velocity_enu_m_s"] = (0.0, 0.0, 0.0)

    _, released = coordinator.filter(
        (-10.0, 0.0, 0.0), (-10.0, 0.0, 0.0), stopped
    )
    _, resumed = coordinator.filter(
        (-10.0, 0.0, 0.0), (-10.0, 0.0, 0.0), closing
    )

    assert released["released"]
    assert not released["active"]
    assert resumed["role"] == "clear"
    assert not resumed["active"]


def test_shared_runtime_pair_agrees_on_priority_and_can_be_reset() -> None:
    reset_shared_conflict_state(("UAV-01", "UAV-02"))
    first = ConflictCoordinator.shared("UAV-01", "UAV-02")
    second = ConflictCoordinator.shared("UAV-02", "UAV-01")

    _, first_status = first.filter(
        (1.0, 0.0, 0.0), (1.0, 0.0, 0.0), states()
    )
    _, second_status = second.filter(
        (-1.0, 0.0, 0.0), (-1.0, 0.0, 0.0), states()
    )

    assert first_status["priority_drone_id"] == second_status["priority_drone_id"]
    assert first_status["encounter"] == second_status["encounter"] == 1
    reset_shared_conflict_state(("UAV-01", "UAV-02"))
    fresh = ConflictCoordinator.shared("UAV-01", "UAV-02")
    assert fresh.encounter == 0


def test_slow_head_on_engages_before_the_cbf_reserve() -> None:
    """A fixed horizon is a distance that shrinks with closing speed.

    At 20 m/s closing, 8 s of lead time is 160 m. At 2 m/s it is 16 m -- inside
    the margin the CBF gate will demand of the same pair, so the coordinator
    would only ever latch after the shield had already started braking. The
    engagement floor makes range, not time, the binding gate down there.
    """
    config = x500_20m_conflict_config(10.0)
    assert config.minimum_engagement_distance_m > config.reserve_separation_m

    pair = ConflictCoordinator.pair(("UAV-01", "UAV-02"), config)
    gap = config.minimum_engagement_distance_m - 1.0
    state = {
        "UAV-01": {
            "valid": True,
            "position_enu_m": (0.0, 0.0, 10.0),
            "velocity_enu_m_s": (1.0, 0.0, 0.0),
        },
        "UAV-02": {
            "valid": True,
            "position_enu_m": (gap, 0.0, 10.0),
            "velocity_enu_m_s": (-1.0, 0.0, 0.0),
        },
    }
    _, priority = pair["UAV-01"].filter((1.0, 0.0, 0.0), (1.0, 0.0, 0.0), state)
    yielded, yielding = pair["UAV-02"].filter(
        (-1.0, 0.0, 0.0), (-1.0, 0.0, 0.0), state
    )

    assert priority["active"] and yielding["active"]
    assert {priority["role"], yielding["role"]} == {"priority", "yield"}
    # The yielding vehicle is actually pushed off the collision line.
    assert abs(yielded[1]) > 0.5


def test_engagement_floor_does_not_move_the_certified_rungs() -> None:
    """The floor must be slack wherever the rungs were signed off.

    Engagement happens at min(trigger, horizon x closing speed); at every
    certified ceiling the trigger is the tighter of the two, so adding a floor
    below it changes nothing those matrices measured.
    """
    for ceiling_m_s in (10.0, 15.0, 20.0):
        config = x500_20m_conflict_config(ceiling_m_s)
        closing_m_s = 2.0 * ceiling_m_s
        assert config.minimum_engagement_distance_m < config.trigger_distance_m
        assert config.trigger_distance_m < config.prediction_horizon_s * closing_m_s


def test_yield_lane_change_does_not_saturate_against_tracker_feedback() -> None:
    """The 2026-08-16 flight failure, closed loop.

    `mission` is the tracker's corrected command, so once the vehicle is off
    its line it carries a pull-back term -- and that term used to re-enter the
    yield output weighted by forward_speed.  The excursion then stalled at
    yield_lateral_speed_m_s / position_gain_s_inv (3.0 / 0.6 = 5 m measured)
    while the barrier wanted ~52 m.
    """
    coordinator = ConflictCoordinator("UAV-02", "UAV-01", x500_20m_conflict_config())
    state = states(100.0)
    state["UAV-01"]["velocity_enu_m_s"] = (10.0, 0.0, 0.0)
    state["UAV-02"]["velocity_enu_m_s"] = (-10.0, 0.0, 0.0)
    position_gain_s_inv = 0.6
    dt_s = 0.1

    offset_m = 0.0
    for _ in range(100):
        mission = (-9.4, -position_gain_s_inv * offset_m, 0.25)
        output, status = coordinator.filter((0.0, 0.0, 0.0), mission, state)
        assert status["role"] == "yield"
        offset_m += output[1] * dt_s

    assert abs(offset_m) > 25.0, offset_m


def _vertical_pair(gap_m: float = 80.0, speed: float = 20.0):
    return {
        "UAV-01": {
            "valid": True,
            "position_enu_m": (0.0, 0.0, 100.0 - gap_m / 2.0),
            "velocity_enu_m_s": (0.0, 0.0, speed),
        },
        "UAV-02": {
            "valid": True,
            "position_enu_m": (0.0, 0.0, 100.0 + gap_m / 2.0),
            "velocity_enu_m_s": (0.0, 0.0, -speed),
        },
    }


def test_a_vertical_head_on_gets_a_sideways_yield() -> None:
    """It used to get no maneuver at all.

    The lane change needs a horizontal normal to take its tangent from and a
    horizontal mission to preserve; a vertical head-on has neither, so the
    whole block was skipped and only the maximum_toward_peer clamp acted --
    output (0, 0, -16) against a mission of (0, 0, -20). Nothing broke the
    symmetry, and the pair held separation without ever passing.
    """
    reset_shared_conflict_state(("UAV-01", "UAV-02"))
    coordinator = ConflictCoordinator(
        "UAV-02", "UAV-01", x500_20m_conflict_config(20.0)
    )

    output, status = coordinator.filter(
        (0.0, 0.0, -20.0), (0.0, 0.0, -20.0), _vertical_pair()
    )

    assert status["role"] == "yield"
    assert math.hypot(output[0], output[1]) > 1.0, output
    # The sideways speed is drawn from the vertical mission, exactly as the
    # horizontal branch draws forward speed from the horizontal one.
    assert math.isclose(
        math.hypot(*output), 20.0, rel_tol=1e-6
    ), output


def test_a_vertical_yield_does_not_flip_to_the_horizontal_branch() -> None:
    """The displacement it creates would otherwise switch branches.

    One frame after the sideways step starts working, the vehicle is off the
    axis and `mission` grows a horizontal component pointing back at it.
    Re-deciding per frame handed the horizontal branch that pull-back as its
    along-path heading and flew the vehicle back into the standoff.
    """
    reset_shared_conflict_state(("UAV-01", "UAV-02"))
    coordinator = ConflictCoordinator(
        "UAV-02", "UAV-01", x500_20m_conflict_config(20.0)
    )
    coordinator.filter((0.0, 0.0, -20.0), (0.0, 0.0, -20.0), _vertical_pair())

    state = _vertical_pair()
    state["UAV-02"]["position_enu_m"] = (30.0, 0.0, 140.0)
    # The mission now points back at the axis, which is the trap.
    output, status = coordinator.filter(
        (0.0, 0.0, -20.0), (-12.0, 0.0, -16.0), state
    )

    assert status["role"] == "yield"
    # Displaced to +x, the yield must not carry the vehicle back toward the
    # axis it just left. It may step tangentially -- once off-axis the normal
    # has a horizontal part again and the step rotates with it -- but a
    # negative x component here is the standoff reasserting itself.
    assert output[0] >= 0.0, f"yield reversed into the standoff: {output}"
    # It keeps the mission's vertical progress and discards the horizontal
    # component outright -- which is the pull-back, and the whole point.
    assert math.isclose(math.hypot(*output), 16.0, rel_tol=1e-6), output


def test_a_horizontal_encounter_is_untouched_by_the_vertical_branch() -> None:
    reset_shared_conflict_state(("UAV-01", "UAV-02"))
    coordinator = ConflictCoordinator("UAV-02", "UAV-01", x500_20m_conflict_config())
    state = states(100.0)
    state["UAV-01"]["velocity_enu_m_s"] = (10.0, 0.0, 0.0)
    state["UAV-02"]["velocity_enu_m_s"] = (-10.0, 0.0, 0.0)

    output, status = coordinator.filter((9.0, -0.1, 8.0), (-9.4, 0.0, 0.25), state)

    assert status["role"] == "yield"
    assert math.isclose(abs(output[1]), 3.0)
    assert math.isclose(math.hypot(output[0], output[1]), 9.4)
    assert output[2] == 0.25


def test_a_released_encounter_can_latch_again_once_the_pair_is_clear() -> None:
    """Otherwise coordination stops for the rest of a crossing mission.

    `released_until_clear` exists to stop a released encounter re-latching
    immediately, on geometry that has not changed yet. Clearing it only at
    encounter_reset_distance_m WHILE separating is a condition two vehicles
    orbiting a shared waypoint never meet: measured on diagonal_cross, the
    pair latched at 90.1 m, yielded 9.7 s, released at 25.6 m and then held
    role "clear" through a convergence with a 3.4 m predicted miss distance,
    to a CBF margin of -0.840 m.
    """
    config = replace(x500_20m_conflict_config(10.0), release_frames=3)
    coordinator = ConflictCoordinator("UAV-02", "UAV-01", config)
    closing = states(100.0)
    closing["UAV-01"]["velocity_enu_m_s"] = (10.0, 0.0, 0.0)
    closing["UAV-02"]["velocity_enu_m_s"] = (-10.0, 0.0, 0.0)
    _, latched = coordinator.filter((-10.0, 0.0, 0.0), (-10.0, 0.0, 0.0), closing)
    assert latched["active"], latched

    # Released at a distance a crossing mission never leaves.
    parted = states(60.0)
    for drone_id in ("UAV-01", "UAV-02"):
        parted[drone_id]["velocity_enu_m_s"] = (0.0, 0.0, 0.0)
    for _ in range(8):
        _, released = coordinator.filter((0.0, 0.0, 0.0), (0.0, 0.0, 0.0), parted)
    assert not released["active"]

    _, again = coordinator.filter((-10.0, 0.0, 0.0), (-10.0, 0.0, 0.0), closing)

    assert again["active"], "a fresh convergence was left uncoordinated"
    assert again["encounter"] == 2


def test_a_release_is_still_not_undone_on_unchanged_geometry() -> None:
    """The suppression the previous test relaxes must still do its job."""
    config = replace(x500_20m_conflict_config(10.0), release_frames=3)
    coordinator = ConflictCoordinator("UAV-02", "UAV-01", config)
    closing = states(100.0)
    closing["UAV-01"]["velocity_enu_m_s"] = (10.0, 0.0, 0.0)
    closing["UAV-02"]["velocity_enu_m_s"] = (-10.0, 0.0, 0.0)
    coordinator.filter((-10.0, 0.0, 0.0), (-10.0, 0.0, 0.0), closing)
    stopped = states(70.0)
    for drone_id in ("UAV-01", "UAV-02"):
        stopped[drone_id]["velocity_enu_m_s"] = (0.0, 0.0, 0.0)
    # Only as far as the release itself: stop the moment it happens, so the
    # non-threat counter has had no frame of its own to run.
    for _ in range(12):
        _, status = coordinator.filter((0.0, 0.0, 0.0), (0.0, 0.0, 0.0), stopped)
        if status["released"]:
            break
    assert status["released"], status

    # Straight back onto a collision course in the very next frame. The
    # geometry has not changed since the release, which is exactly what this
    # suppression is for.
    _, immediate = coordinator.filter(
        (-10.0, 0.0, 0.0), (-10.0, 0.0, 0.0), closing
    )
    assert immediate["encounter"] == 1, immediate
    assert not immediate["active"], immediate


def test_x500_keeps_the_fixed_distance_it_was_certified_with() -> None:
    relative = 30.0
    boundary = (
        20.0 + 2.0 + relative * (0.65 + 0.15) + relative * relative / 12.0 + 5.0
    )

    assert x500_20m_conflict_config(15.0).trigger_distance_m == pytest.approx(
        max(120.0, boundary + 25.0), abs=0.5
    )
