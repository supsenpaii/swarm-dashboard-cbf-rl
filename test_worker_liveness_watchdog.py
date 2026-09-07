import pytest

from worker_liveness_watchdog import (
    WorkerLivenessSample,
    WorkerLivenessWatchdog,
    check_worker_liveness,
)


def _sample(
    drone_id: str = "UAV-01",
    companion_safety_enabled: bool = True,
    last_evaluation_monotonic_s: float | None = 0.0,
) -> WorkerLivenessSample:
    return WorkerLivenessSample(
        drone_id=drone_id,
        companion_safety_enabled=companion_safety_enabled,
        last_evaluation_monotonic_s=last_evaluation_monotonic_s,
    )


def test_fresh_evaluation_is_not_stalled() -> None:
    (result,) = check_worker_liveness(
        (_sample(last_evaluation_monotonic_s=10.0),),
        now_monotonic_s=10.05,
        stall_after_s=1.5,
    )
    assert result.stalled is False
    assert result.reason == "ok"
    assert result.stalled_for_s == pytest.approx(0.05)


def test_stale_evaluation_beyond_threshold_is_stalled() -> None:
    (result,) = check_worker_liveness(
        (_sample(last_evaluation_monotonic_s=10.0),),
        now_monotonic_s=12.0,
        stall_after_s=1.5,
    )
    assert result.stalled is True
    assert result.reason == "evaluation_loop_stalled"
    assert result.stalled_for_s == 2.0


def test_exactly_at_threshold_is_not_yet_stalled() -> None:
    (result,) = check_worker_liveness(
        (_sample(last_evaluation_monotonic_s=10.0),),
        now_monotonic_s=11.5,
        stall_after_s=1.5,
    )
    assert result.stalled is False


def test_companion_safety_disabled_never_flagged_regardless_of_age() -> None:
    (result,) = check_worker_liveness(
        (
            _sample(
                companion_safety_enabled=False,
                last_evaluation_monotonic_s=None,
            ),
        ),
        now_monotonic_s=99999.0,
        stall_after_s=1.5,
    )
    assert result.stalled is False
    assert result.reason == "companion_safety_disabled"
    assert result.stalled_for_s is None


def test_enabled_but_never_evaluated_yet_is_not_a_stall() -> None:
    (result,) = check_worker_liveness(
        (
            _sample(
                companion_safety_enabled=True,
                last_evaluation_monotonic_s=None,
            ),
        ),
        now_monotonic_s=5.0,
        stall_after_s=1.5,
    )
    assert result.stalled is False
    assert result.reason == "awaiting_first_evaluation"


def test_independent_per_drone() -> None:
    results = check_worker_liveness(
        (
            _sample(drone_id="UAV-01", last_evaluation_monotonic_s=10.0),
            _sample(drone_id="UAV-02", last_evaluation_monotonic_s=0.0),
        ),
        now_monotonic_s=10.1,
        stall_after_s=1.5,
    )
    by_drone = {result.drone_id: result for result in results}
    assert by_drone["UAV-01"].stalled is False
    assert by_drone["UAV-02"].stalled is True


def test_watchdog_reports_initial_state_as_a_transition() -> None:
    watchdog = WorkerLivenessWatchdog(stall_after_s=1.5)
    _results, transitions = watchdog.poll(
        (_sample(last_evaluation_monotonic_s=10.0),),
        now_monotonic_s=10.05,
    )
    assert len(transitions) == 1
    assert transitions[0].stalled is False


def test_watchdog_does_not_repeat_transitions_while_stalled() -> None:
    watchdog = WorkerLivenessWatchdog(stall_after_s=1.5)
    watchdog.poll((_sample(last_evaluation_monotonic_s=10.0),), now_monotonic_s=10.05)

    _results, first_stall_transitions = watchdog.poll(
        (_sample(last_evaluation_monotonic_s=10.0),), now_monotonic_s=12.0
    )
    assert len(first_stall_transitions) == 1
    assert first_stall_transitions[0].stalled is True

    _results, still_stalled_transitions = watchdog.poll(
        (_sample(last_evaluation_monotonic_s=10.0),), now_monotonic_s=13.0
    )
    assert still_stalled_transitions == ()


def test_watchdog_reports_recovery_transition() -> None:
    watchdog = WorkerLivenessWatchdog(stall_after_s=1.5)
    watchdog.poll((_sample(last_evaluation_monotonic_s=10.0),), now_monotonic_s=10.05)
    watchdog.poll((_sample(last_evaluation_monotonic_s=10.0),), now_monotonic_s=12.0)

    _results, recovery_transitions = watchdog.poll(
        (_sample(last_evaluation_monotonic_s=20.0),), now_monotonic_s=20.05
    )
    assert len(recovery_transitions) == 1
    assert recovery_transitions[0].stalled is False


def test_watchdog_tracks_multiple_drones_independently() -> None:
    watchdog = WorkerLivenessWatchdog(stall_after_s=1.5)
    watchdog.poll(
        (
            _sample(drone_id="UAV-01", last_evaluation_monotonic_s=10.0),
            _sample(drone_id="UAV-02", last_evaluation_monotonic_s=10.0),
        ),
        now_monotonic_s=10.05,
    )

    # Only UAV-02 stalls; UAV-01 keeps advancing normally.
    _results, transitions = watchdog.poll(
        (
            _sample(drone_id="UAV-01", last_evaluation_monotonic_s=12.0),
            _sample(drone_id="UAV-02", last_evaluation_monotonic_s=10.0),
        ),
        now_monotonic_s=12.0,
    )
    assert len(transitions) == 1
    assert transitions[0].drone_id == "UAV-02"
    assert transitions[0].stalled is True
