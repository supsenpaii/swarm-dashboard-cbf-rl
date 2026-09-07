"""Installing an operator mission at runtime, and refusing to do it mid-flight."""

from __future__ import annotations

import pytest

from companion_safety import CompanionSafetyMonitor
from formation_controller import FormationSlot
from trajectory_controller import ClosedPolylineTrajectory

SQUARE = ClosedPolylineTrajectory(
    waypoints_enu_m=((0.0, 0.0, 9.0), (20.0, 0.0, 9.0), (20.0, 20.0, 9.0), (0.0, 20.0, 9.0)),
    speed_m_s=1.5,
)


def monitor() -> CompanionSafetyMonitor:
    return CompanionSafetyMonitor(
        drone_id="UAV-01",
        peer_ids=("UAV-02",),
        leader_id="UAV-01",
        slots=(FormationSlot("UAV-02", (-10.0, 0.0, 0.0)),),
    )


def test_installs_a_mission_when_no_mission_is_running():
    subject = monitor()
    assert subject.trajectory_reference_start_enu_m is None
    subject.set_trajectory(SQUARE)
    assert subject.trajectory_reference_start_enu_m == (0.0, 0.0, 9.0)
    assert subject.trajectory_tracking.maximum_acceleration_m_s2 == 0.5


def test_operator_mission_acceleration_limit_is_configurable(monkeypatch):
    monkeypatch.setenv("SWARM_MISSION_MAXIMUM_ACCELERATION_M_S2", "0.25")
    subject = monitor()
    subject.set_trajectory(SQUARE)
    assert subject.trajectory_tracking.maximum_acceleration_m_s2 == 0.25


def test_operator_mission_corner_tolerance_is_configurable(monkeypatch):
    monkeypatch.setenv("SWARM_MISSION_CORNER_TRACKING_TOLERANCE_M", "0.75")
    subject = monitor()
    subject.set_trajectory(SQUARE)
    assert subject.trajectory_tracking.corner_tracking_tolerance_m == 0.75


def test_clearing_a_mission_restores_the_formation_nominal():
    subject = monitor()
    subject.set_trajectory(SQUARE)
    subject.set_trajectory(None)
    assert subject.trajectory_tracking is None
    assert subject.trajectory_reference_start_enu_m is None


def test_refuses_to_swap_the_path_under_a_flying_vehicle():
    subject = monitor()
    subject.set_trajectory(SQUARE)
    # The mission clock latches when the companion becomes the authority.
    subject.trajectory_start_monotonic_s = 1234.0
    with pytest.raises(RuntimeError, match="already running"):
        subject.set_trajectory(SQUARE)
    with pytest.raises(RuntimeError, match="already running"):
        subject.set_trajectory(None)
