from __future__ import annotations

import copy
import threading
import time
from collections import deque
from unittest.mock import patch

import pytest

import main
from simulation_ground_truth import optical_center_distance


def _snapshot(timestamp_s: float, target_x: float) -> dict:
    identity = (0.0, 0.0, 0.0, 1.0)
    observer = main.DRONE_MODELS["UAV-01"]
    target = main.DRONE_MODELS["UAV-02"]
    return {
        "sim_timestamp_s": timestamp_s,
        "poses": {
            observer: {
                "position_xyz": (0.0, 0.0, 0.0),
                "quaternion_xyzw": identity,
            },
            f"{observer}::camera_link": {
                "position_xyz": (0.0, 0.0, 0.0),
                "quaternion_xyzw": identity,
            },
            target: {
                "position_xyz": (target_x, 0.0, 0.0),
                "quaternion_xyzw": identity,
            },
        },
    }


def _new_bridge(initial_snapshots=()) -> main.GazeboDashboardBridge:
    bridge = main.GazeboDashboardBridge.__new__(main.GazeboDashboardBridge)
    bridge.pose_lock = threading.Lock()
    bridge.pose_history = deque(initial_snapshots, maxlen=600)
    bridge.pose_history_condition = threading.Condition(bridge.pose_lock)
    bridge._gt_bracket_generation = 0
    bridge._gt_bracket_waiting_count = 0
    return bridge


def test_simulation_ground_truth_copies_only_bracketing_pose_records() -> None:
    bridge = _new_bridge(
        _snapshot(index * 0.01, -float(index)) for index in range(600)
    )

    with patch.object(main.copy, "deepcopy", wraps=copy.deepcopy) as deepcopy:
        result = bridge.simulation_target_ground_truth(
            "UAV-01", 3.005, "UAV-02"
        )

    assert result["available"]
    expected = optical_center_distance(
        _snapshot(3.005, -300.5)["poses"][f'{main.DRONE_MODELS["UAV-01"]}::camera_link'],
        _snapshot(3.005, -300.5)["poses"][main.DRONE_MODELS["UAV-02"]],
    )["distance_m"]
    assert result["camera_to_target_center_m"] == pytest.approx(expected)
    assert result["interpolation_span_ms"] == pytest.approx(10.0)
    assert deepcopy.call_count == 2


def test_simulation_ground_truth_lower_missing_fails_immediately_no_wait() -> None:
    """A camera timestamp older than the oldest buffered sample can never
    be resolved by waiting -- time only moves forward -- so it must fail
    immediately, not retry until GT_BRACKET_WAIT_TIMEOUT_S elapses."""
    bridge = _new_bridge([_snapshot(10.0, -8.0)])

    started = time.monotonic()
    result = bridge.simulation_target_ground_truth("UAV-01", 5.0, "UAV-02")
    elapsed = time.monotonic() - started

    assert not result["available"]
    assert result["error"] == "Gazebo pose bracket unavailable"
    assert elapsed < 0.05


def test_simulation_ground_truth_upper_missing_times_out_with_no_fabricated_gt() -> None:
    """upper_source missing (camera newer than every buffered sample) is
    retried up to a bounded timeout; if nothing ever arrives to complete
    the bracket, it must fail with GT_BRACKET_TIMEOUT and no distance
    value -- never fall back to the nearest sample or extrapolate."""
    bridge = _new_bridge([_snapshot(10.0, -8.0)])

    with patch.object(main, "GT_BRACKET_WAIT_TIMEOUT_S", 0.03):
        started = time.monotonic()
        result = bridge.simulation_target_ground_truth(
            "UAV-01", 10.11, "UAV-02"
        )
        elapsed = time.monotonic() - started

    assert not result["available"]
    assert result["error"] == "GT_BRACKET_TIMEOUT"
    assert "camera_to_target_center_m" not in result
    assert elapsed >= 0.03
    assert elapsed < 0.5


def test_simulation_ground_truth_resolves_after_late_pose_arrives() -> None:
    """A frame that arrives before its bracket is complete must be
    resolved -- not dropped -- once the missing pose sample is appended
    from another thread, using the same interpolation as the no-wait
    path (never nearest-pose, never extrapolated)."""
    bridge = _new_bridge([_snapshot(10.0, -8.0)])

    def _append_later() -> None:
        time.sleep(0.03)
        with bridge.pose_lock:
            bridge.pose_history.append(_snapshot(10.2, -8.2))
            bridge.pose_history_condition.notify_all()

    appender = threading.Thread(target=_append_later)
    with patch.object(main, "GT_BRACKET_WAIT_TIMEOUT_S", 2.0):
        appender.start()
        result = bridge.simulation_target_ground_truth(
            "UAV-01", 10.11, "UAV-02"
        )
    appender.join()

    assert result["available"], result
    expected = optical_center_distance(
        _snapshot(10.11, -8.11)["poses"][f'{main.DRONE_MODELS["UAV-01"]}::camera_link'],
        _snapshot(10.11, -8.11)["poses"][main.DRONE_MODELS["UAV-02"]],
    )["distance_m"]
    assert result["camera_to_target_center_m"] == pytest.approx(expected)
    assert result["bracket_retry_count"] >= 1


def test_simulation_ground_truth_ignores_insufficient_intermediate_pose() -> None:
    """While waiting, a pose sample that arrives but still doesn't reach
    the requested timestamp must not be accepted as the bracket -- the
    wait must continue until a genuinely sufficient sample arrives (or
    time out), never accepting a bracket where t_after < t_camera."""
    bridge = _new_bridge([_snapshot(10.0, -8.0)])

    def _append_two_waves() -> None:
        time.sleep(0.02)
        with bridge.pose_lock:
            # Still short of 10.11 -- must not be accepted as upper_source.
            bridge.pose_history.append(_snapshot(10.05, -8.05))
            bridge.pose_history_condition.notify_all()
        time.sleep(0.02)
        with bridge.pose_lock:
            bridge.pose_history.append(_snapshot(10.2, -8.2))
            bridge.pose_history_condition.notify_all()

    appender = threading.Thread(target=_append_two_waves)
    with patch.object(main, "GT_BRACKET_WAIT_TIMEOUT_S", 2.0):
        appender.start()
        result = bridge.simulation_target_ground_truth(
            "UAV-01", 10.11, "UAV-02"
        )
    appender.join()

    assert result["available"], result
    # Bracket must be [10.05, 10.2] (the still-open-below sample from the
    # first wave, and the first sample that actually reaches 10.11), not
    # a premature/incorrect pairing.
    assert result["pose_bracket_lower_sim_timestamp_s"] == pytest.approx(10.05)
    assert result["pose_bracket_upper_sim_timestamp_s"] == pytest.approx(10.2)
    assert result["bracket_retry_count"] >= 2


def test_reset_ground_truth_pending_abandons_wait_without_writing_gt() -> None:
    """A session boundary (start/stop/bbox reselect) must abandon any
    in-flight wait immediately rather than let it either time out slowly
    or resolve against a pose sample that arrives after the reset -- no
    GT for a frame from the abandoned wait may be produced from a
    post-reset pose sample."""
    bridge = _new_bridge([_snapshot(10.0, -8.0)])

    def _reset_soon() -> None:
        time.sleep(0.02)
        bridge.reset_ground_truth_pending()

    resetter = threading.Thread(target=_reset_soon)
    with patch.object(main, "GT_BRACKET_WAIT_TIMEOUT_S", 5.0):
        started = time.monotonic()
        resetter.start()
        result = bridge.simulation_target_ground_truth(
            "UAV-01", 10.11, "UAV-02"
        )
        elapsed = time.monotonic() - started
    resetter.join()

    assert not result["available"]
    assert result["error"] == "GT_BRACKET_TIMEOUT"
    assert "camera_to_target_center_m" not in result
    # Abandoned well before the 5s configured timeout.
    assert elapsed < 1.0


def test_simulation_ground_truth_pending_limit_fails_fast() -> None:
    """A bounded number of concurrent waiters is enforced -- once at the
    cap, further concurrent callers fail immediately instead of queuing
    without bound."""
    bridge = _new_bridge([_snapshot(10.0, -8.0)])
    bridge._gt_bracket_waiting_count = main.GT_BRACKET_MAX_PENDING_WAITERS

    result = bridge.simulation_target_ground_truth("UAV-01", 10.11, "UAV-02")

    assert not result["available"]
    assert result["error"] == "GT_BRACKET_PENDING_LIMIT_EXCEEDED"


def _snapshot_missing_entity(timestamp_s: float, *, drop: str) -> dict:
    snapshot = _snapshot(timestamp_s, -8.0)
    del snapshot["poses"][drop]
    return snapshot


def test_simulation_ground_truth_target_entity_missing_from_time_complete_bracket() -> None:
    """A time-complete bracket (upper_source found) whose snapshot is
    still missing the *target*'s pose entry (that model's own broadcast
    lagging behind, a different race than the timestamp bracket this fix
    targets) must not fabricate a distance -- it fails with the existing,
    unchanged 'target unavailable' error rather than being silently
    retried into a wrong answer."""
    bridge = _new_bridge(
        [
            _snapshot(10.0, -8.0),
            _snapshot_missing_entity(10.2, drop=main.DRONE_MODELS["UAV-02"]),
        ]
    )

    result = bridge.simulation_target_ground_truth("UAV-01", 10.11, "UAV-02")

    assert not result["available"]
    assert result["error"] == "camera link or target model pose unavailable"


def test_simulation_ground_truth_camera_entity_missing_from_time_complete_bracket() -> None:
    """Same as above but the *camera*'s own pose entry is the one missing
    from the time-complete snapshot."""
    bridge = _new_bridge(
        [
            _snapshot(10.0, -8.0),
            _snapshot_missing_entity(
                10.2, drop=f'{main.DRONE_MODELS["UAV-01"]}::camera_link'
            ),
        ]
    )

    result = bridge.simulation_target_ground_truth("UAV-01", 10.11, "UAV-02")

    assert not result["available"]
    assert result["error"] == "camera link or target model pose unavailable"


def test_simulation_ground_truth_called_exactly_once_per_frame(monkeypatch) -> None:
    """The retry/wait loop lives entirely inside one call -- a caller must
    never need to invoke simulation_target_ground_truth more than once for
    the same frame, so exactly one accepted record is ever produced."""
    bridge = _new_bridge([_snapshot(10.0, -8.0)])
    calls = []
    original = bridge.simulation_target_ground_truth

    def _counting(*args, **kwargs):
        calls.append(1)
        return original(*args, **kwargs)

    def _append_later() -> None:
        time.sleep(0.02)
        with bridge.pose_lock:
            bridge.pose_history.append(_snapshot(10.2, -8.2))
            bridge.pose_history_condition.notify_all()

    appender = threading.Thread(target=_append_later)
    with patch.object(main, "GT_BRACKET_WAIT_TIMEOUT_S", 2.0):
        appender.start()
        result = _counting("UAV-01", 10.11, "UAV-02")
    appender.join()

    assert result["available"]
    assert len(calls) == 1
