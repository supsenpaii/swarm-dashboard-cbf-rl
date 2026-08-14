from __future__ import annotations

import json

import pytest

from cbf_rl_policy import ProximityCbfRlPolicy
from cbf_rl_train import OrbitScenario, TrainingScenario, rollout, train


SIMPLE = TrainingScenario(
    "simple",
    {"UAV-01": (0.0, 0.0, 9.0), "UAV-02": (0.0, 8.0, 9.0)},
    {"UAV-01": (1.0, 0.0, 9.0), "UAV-02": (1.0, 8.0, 9.0)},
)


def test_policy_round_trip_and_checksum(tmp_path) -> None:
    policy, report = train(
        seed=3,
        generations=1,
        population=3,
        elite_count=1,
        scenarios=(SIMPLE,),
        maximum_steps=20,
    )
    destination = tmp_path / "policy.json"
    policy.save(destination, training=report)
    loaded = ProximityCbfRlPolicy.load(destination)
    observation = (0.1,) * 20

    assert loaded.act(observation) == policy.act(observation)
    value = json.loads(destination.read_text())
    value["parameters"]["goal_gain"] += 0.1
    with pytest.raises(ValueError, match="checksum"):
        ProximityCbfRlPolicy.from_dict(value)


def test_seeded_training_is_reproducible() -> None:
    arguments = dict(
        seed=5,
        generations=2,
        population=4,
        elite_count=2,
        scenarios=(SIMPLE,),
        maximum_steps=20,
    )
    first_policy, first_report = train(**arguments)
    second_policy, second_report = train(**arguments)

    assert first_policy == second_policy
    assert first_report == second_report


def test_loader_rejects_observation_contract_drift() -> None:
    policy, _ = train(
        seed=1,
        generations=1,
        population=3,
        elite_count=1,
        scenarios=(SIMPLE,),
        maximum_steps=20,
    )
    value = policy.as_dict()
    value["observation_fields"].reverse()

    with pytest.raises(ValueError, match="observation contract"):
        ProximityCbfRlPolicy.from_dict(value)


def test_orbit_rollout_uses_the_runtime_tracker_and_reports_progress() -> None:
    result = rollout(
        ProximityCbfRlPolicy.load("cbf_rl_policy_v2.json"),
        OrbitScenario("short_orbit", phase_waypoints=4, maximum_steps=20),
        maximum_steps=20,
    )

    assert result["scenario"] == "short_orbit"
    assert set(result["progress_m"]) == {"UAV-01", "UAV-02"}
    assert result["minimum_distance_m"] >= 4.0
