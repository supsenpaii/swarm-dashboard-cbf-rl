from cbf_rl_conflict_evaluate import rollout
from cbf_rl_policy import ProximityCbfRlPolicy


def test_hybrid_crossing_rollout_keeps_positive_reserve_and_progress() -> None:
    result = rollout(
        ProximityCbfRlPolicy.load("cbf_rl_policy_mission_v1.json"),
        "diagonal_cross",
        0.8,
        0.70,
        maximum_steps=800,
    )

    assert result["minimum_cbf_margin_m"] >= 0.30
    assert result["hold_frames"] == 0
    assert result["final_cbf_intervention_frames"] == 0
    assert result["minimum_progress_fraction"] >= 0.45
