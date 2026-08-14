from dataclasses import replace
import math

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
    state = states(20.0)
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
