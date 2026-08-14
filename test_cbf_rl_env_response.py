"""The offline environment must be able to lag like the real vehicle.

Training against an instantaneous plant is why a policy could pass the
offline gate in exactly 700 steps and still fail in flight (2026-08-12).
"""

from __future__ import annotations

import math

import pytest

from cbf_rl_env import DRONE_IDS, CbfRlEnvConfig, CbfRlEnvironment

GOALS = {"UAV-01": (60.0, 0.0, 9.0), "UAV-02": (60.0, 40.0, 9.0)}
SPAWN = {"UAV-01": (0.0, 0.0, 9.0), "UAV-02": (0.0, 40.0, 9.0)}
FULL_EAST = {drone: (1.0, 0.0, 0.0) for drone in DRONE_IDS}


def environment(**config):
    subject = CbfRlEnvironment(GOALS, CbfRlEnvConfig(**config))
    subject.reset(SPAWN)
    return subject


def test_zero_lag_keeps_the_frozen_phase_zero_contract():
    """Default must stay bit-identical: the vehicle adopts the safe velocity."""
    subject = environment()
    _, _, _, _, info = subject.step(FULL_EAST)
    for drone in DRONE_IDS:
        assert subject.velocities[drone] == info[drone]["safe_velocity_enu_m_s"]
    assert subject.velocities["UAV-01"][0] == pytest.approx(2.0)


def test_a_lagged_vehicle_reaches_63_percent_after_one_time_constant():
    tau = 0.5
    subject = environment(response_time_constant_s=tau)
    for _ in range(int(round(tau / subject.config.dt_s))):
        subject.step(FULL_EAST)
    # Requested is the CBF-safe 2.0 m/s; first order gives 1 - 1/e of it.
    assert subject.velocities["UAV-01"][0] == pytest.approx(
        2.0 * (1.0 - math.exp(-1.0)), abs=0.02
    )


def test_the_lagged_velocity_is_what_moves_the_vehicle():
    """Not just reported: position, observation and barrier must all see it."""
    subject = environment(response_time_constant_s=0.5)
    before = subject.positions["UAV-01"][0]
    subject.step(FULL_EAST)
    achieved = subject.velocities["UAV-01"][0]
    assert achieved < 2.0
    moved = subject.positions["UAV-01"][0] - before
    assert moved == pytest.approx(achieved * subject.config.dt_s)
    assert subject.observations()["UAV-01"][3] == pytest.approx(achieved)


def test_lag_costs_real_distance_over_a_leg():
    """The whole point: the same policy travels less under a real plant."""
    instant = environment()
    lagged = environment(response_time_constant_s=0.6)
    for _ in range(200):
        instant.step(FULL_EAST)
        lagged.step(FULL_EAST)
    assert lagged.positions["UAV-01"][0] < instant.positions["UAV-01"][0] - 0.5


def test_the_range_randomises_per_episode_and_stays_seeded():
    config = dict(response_time_constant_range_s=(0.4, 0.8), response_seed=7)
    first = CbfRlEnvironment(GOALS, CbfRlEnvConfig(**config))
    drawn = []
    for _ in range(5):
        first.reset(SPAWN)
        drawn.append(first.response_time_constant_s)
    assert all(0.4 <= value <= 0.8 for value in drawn)
    assert len(set(drawn)) > 1, "a fixed lag is a simulator, not a vehicle"

    repeat = CbfRlEnvironment(GOALS, CbfRlEnvConfig(**config))
    again = []
    for _ in range(5):
        repeat.reset(SPAWN)
        again.append(repeat.response_time_constant_s)
    assert again == drawn


def test_an_invalid_response_configuration_is_refused():
    with pytest.raises(ValueError):
        CbfRlEnvConfig(response_time_constant_s=-0.1)
    with pytest.raises(ValueError):
        CbfRlEnvConfig(response_time_constant_range_s=(0.8, 0.4))
    with pytest.raises(ValueError):
        CbfRlEnvConfig(response_time_constant_range_s=(-1.0, 0.4))
