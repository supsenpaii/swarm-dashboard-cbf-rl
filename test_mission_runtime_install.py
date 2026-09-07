"""The bridge's mission install path: default-off, authoritative, fail-closed."""

from __future__ import annotations

import math
from types import SimpleNamespace

import pytest
from pymavlink import mavutil

import mavlink_manual_bridge as bridge
from companion_safety import CompanionSafetyMonitor
from formation_controller import FormationSlot
from one_uav_active_readiness import WarmupContract
from swarm_state import GeodeticOrigin
from trajectory_controller import ClosedPolylineTrajectory

ORIGIN = GeodeticOrigin(latitude_deg=47.3977508, longitude_deg=8.5456073, altitude_msl_m=488.0)
METRE_LAT = 1.0 / 111_320.0
METRE_LON = METRE_LAT / math.cos(math.radians(ORIGIN.latitude_deg))
EXISTING = ClosedPolylineTrajectory(
    waypoints_enu_m=((0.0, 0.0, 9.0), (10.0, 0.0, 9.0), (10.0, 10.0, 9.0)),
    speed_m_s=1.0,
)


def payload(**overrides):
    corners = ((0.0, 0.0), (20.0, 0.0), (20.0, 20.0), (0.0, 20.0))
    body = {
        "type": "mission_path",
        "drone_id": "UAV-01",
        "altitude_m": 9.0,
        "speed_m_s": 1.0,
        "waypoints": [
            {
                "latitude_deg": ORIGIN.latitude_deg + north * METRE_LAT,
                "longitude_deg": ORIGIN.longitude_deg + east * METRE_LON,
            }
            for east, north in corners
        ],
    }
    body.update(overrides)
    return body


@pytest.fixture
def worker(monkeypatch):
    """A MavlinkWorker reduced to exactly what the install path touches."""
    subject = object.__new__(bridge.MavlinkWorker)
    subject.drone_id = "UAV-01"
    subject.companion_safety = CompanionSafetyMonitor(
        drone_id="UAV-01",
        peer_ids=("UAV-02",),
        leader_id="UAV-01",
        slots=(FormationSlot("UAV-02", (-10.0, 0.0, 0.0)),),
        trajectory=EXISTING,
    )
    subject.mission_status = {"runtime_enabled": False, "state": "none", "reason": ""}
    monkeypatch.setattr(bridge, "PEER_STATE_ORIGIN", ORIGIN)
    monkeypatch.setattr(bridge, "mission_inbox", bridge.MissionInbox())
    return subject


def test_nothing_happens_without_a_pending_mission(worker, monkeypatch):
    monkeypatch.setattr(bridge, "MISSION_RUNTIME_ENABLED", True)
    worker.install_pending_mission()
    assert worker.mission_status["state"] == "none"
    assert worker.companion_safety.trajectory_tracking.trajectory is EXISTING


def test_default_off_refuses_and_leaves_the_flight_path_alone(worker, monkeypatch):
    monkeypatch.setattr(bridge, "MISSION_RUNTIME_ENABLED", False)
    bridge.mission_inbox.submit("UAV-01", payload())
    worker.install_pending_mission()
    assert worker.mission_status["state"] == "refused"
    assert "SWARM_MISSION_RUNTIME_ENABLED" in worker.mission_status["reason"]
    assert worker.companion_safety.trajectory_tracking.trajectory is EXISTING


def test_a_valid_mission_replaces_the_flight_path(worker, monkeypatch):
    monkeypatch.setattr(bridge, "MISSION_RUNTIME_ENABLED", True)
    bridge.mission_inbox.submit("UAV-01", payload())
    worker.install_pending_mission()
    assert worker.mission_status["state"] == "installed"
    assert worker.mission_status["waypoints"] == 4
    assert worker.mission_status["perimeter_m"] == pytest.approx(80.0, abs=0.5)
    installed = worker.companion_safety.trajectory_tracking.trajectory
    assert installed is not EXISTING
    assert not installed.is_finished(10_000.0)


def test_the_bridge_repeats_validation_rather_than_trusting_mqtt(worker, monkeypatch):
    """A payload the dashboard would have refused must still be refused here."""
    monkeypatch.setattr(bridge, "MISSION_RUNTIME_ENABLED", True)
    bridge.mission_inbox.submit("UAV-01", payload(speed_m_s=9.0))
    worker.install_pending_mission()
    assert worker.mission_status["state"] == "refused"
    assert "speed" in worker.mission_status["reason"]
    assert worker.companion_safety.trajectory_tracking.trajectory is EXISTING


def test_a_running_mission_is_not_swapped_underneath_the_vehicle(worker, monkeypatch):
    monkeypatch.setattr(bridge, "MISSION_RUNTIME_ENABLED", True)
    worker.mission_status = {
        "runtime_enabled": True,
        "state": "installed",
        "reason": "",
        "execution_state": "running",
        "execution_reason": "",
    }
    previous_status = dict(worker.mission_status)
    worker.companion_safety.trajectory_start_monotonic_s = 100.0
    bridge.mission_inbox.submit("UAV-01", payload())
    worker.install_pending_mission()
    assert worker.mission_status == previous_status
    assert worker.companion_safety.trajectory_tracking.trajectory is EXISTING


def test_a_mission_entering_offboard_is_not_swapped_underneath_the_vehicle(
    worker, monkeypatch
):
    monkeypatch.setattr(bridge, "MISSION_RUNTIME_ENABLED", True)
    worker.mission_status = {
        "runtime_enabled": True,
        "state": "installed",
        "reason": "",
        "execution_state": "entering",
        "execution_reason": "",
    }
    bridge.mission_inbox.submit("UAV-01", payload())
    worker.install_pending_mission()
    assert worker.mission_status["execution_state"] == "entering"
    assert worker.companion_safety.trajectory_tracking.trajectory is EXISTING


def test_the_inbox_keeps_only_the_newest_drawing(worker):
    bridge.mission_inbox.submit("UAV-01", payload(speed_m_s=0.5))
    bridge.mission_inbox.submit("UAV-01", payload(speed_m_s=1.2))
    assert bridge.mission_inbox.take("UAV-01")["speed_m_s"] == 1.2
    assert bridge.mission_inbox.take("UAV-01") is None


class ReadySender:
    warmup = WarmupContract()

    def __init__(self):
        self.latched_abort = None
        self.stream_started_monotonic_s = 10.0
        self.last_transmit_monotonic_s = 13.1
        self.transmit_count = self.warmup.minimum_valid_samples + 1
        self.max_transmit_gap_s = 0.05

    def status(self):
        return {
            "transmit_sink_attached": True,
            "explicit_opt_in": True,
            "latched_abort": self.latched_abort,
            "stream_duration_s": (
                self.last_transmit_monotonic_s
                - self.stream_started_monotonic_s
            ),
            "transmit_count": self.transmit_count,
            "max_transmit_gap_s": self.max_transmit_gap_s,
        }


@pytest.fixture
def execution_worker(monkeypatch):
    subject = object.__new__(bridge.MavlinkWorker)
    subject.drone_id = "UAV-01"
    subject.expected_system_id = 1
    subject.mission_status = {
        "runtime_enabled": True,
        "state": "installed",
        "reason": "",
        "execution_state": "idle",
        "execution_reason": "",
    }
    subject.active_offboard_sender = ReadySender()
    subject.last_px4_armed = True
    subject.last_px4_main_mode = bridge.PX4_MAIN_MODE_POSCTL
    subject.last_px4_sub_mode = 0
    subject.mission_start_pending = False
    subject.mission_offboard_expected = False
    subject.mission_mode_ack_pending = False
    subject.mission_start_attempts = 0
    subject.mission_start_requested_monotonic = 0.0
    subject.last_offboard_mode_ack_result = None
    subject.mode_requests = []

    def request_mode(main_mode, label, sub_mode=0, *, companion_authorized=False):
        subject.mode_requests.append(
            (main_mode, label, sub_mode, companion_authorized)
        )
        return True

    subject.request_px4_main_mode = request_mode
    monkeypatch.setattr(bridge, "MISSION_RUNTIME_ENABLED", True)
    monkeypatch.setattr(bridge, "mission_inbox", bridge.MissionInbox())
    return subject


def healthy_sender_payload(**frame_overrides):
    frame = {
        "transmitted": True,
        "reason": "",
        "abort_condition": None,
    }
    frame.update(frame_overrides)
    return {
        "active_offboard_conditions": [],
        "active_offboard_frame": frame,
    }


def test_start_requires_the_current_measured_warmup(execution_worker):
    execution_worker.active_offboard_sender.last_transmit_monotonic_s = 10.5
    reason = execution_worker.mission_start_block_reason(
        healthy_sender_payload()
    )
    assert reason == "setpoint warmup is not complete"


def test_start_enters_only_through_the_companion_mode_request(execution_worker):
    bridge.mission_inbox.submit_action("UAV-01", "mission_start")
    execution_worker.handle_pending_mission_action(
        healthy_sender_payload(),
        10.0,
    )
    assert execution_worker.mission_status["execution_state"] == "entering"
    assert execution_worker.mission_start_pending is True
    assert execution_worker.mode_requests == [
        (
            bridge.PX4_MAIN_MODE_OFFBOARD,
            "MISSION OFFBOARD",
            0,
            True,
        )
    ]


def test_offboard_is_expected_only_after_px4_is_observed_there(execution_worker):
    bridge.mission_inbox.submit_action("UAV-01", "mission_start")
    execution_worker.handle_pending_mission_action(
        healthy_sender_payload(),
        10.0,
    )
    assert execution_worker.mission_offboard_expected is False

    execution_worker.last_px4_main_mode = bridge.PX4_MAIN_MODE_OFFBOARD
    execution_worker.prepare_mission_execution_state()
    assert execution_worker.mission_status["execution_state"] == "running"
    assert execution_worker.mission_offboard_expected is True


def test_duplicate_start_preserves_the_running_mission(execution_worker):
    execution_worker.last_px4_main_mode = bridge.PX4_MAIN_MODE_OFFBOARD
    execution_worker.mission_status["execution_state"] = "running"
    execution_worker.mission_offboard_expected = True
    bridge.mission_inbox.submit_action("UAV-01", "mission_start")
    execution_worker.handle_pending_mission_action(
        healthy_sender_payload(),
        20.0,
    )
    assert execution_worker.mission_status["execution_state"] == "running"
    assert execution_worker.mission_status["execution_reason"] == ""
    assert execution_worker.mission_offboard_expected is True
    assert execution_worker.mode_requests == []


def test_stop_clears_expected_before_requesting_position(execution_worker):
    execution_worker.last_px4_main_mode = bridge.PX4_MAIN_MODE_OFFBOARD
    execution_worker.mission_status["execution_state"] = "running"
    execution_worker.mission_offboard_expected = True
    bridge.mission_inbox.submit_action("UAV-01", "mission_stop")
    execution_worker.handle_pending_mission_action(
        healthy_sender_payload(),
        20.0,
    )
    assert execution_worker.mission_offboard_expected is False
    assert execution_worker.mission_status["execution_state"] == "stopping"
    assert execution_worker.mode_requests[-1][:2] == (
        bridge.PX4_MAIN_MODE_POSCTL,
        "POSITION (MISSION STOP)",
    )


def test_start_is_blocked_while_disarmed(execution_worker):
    execution_worker.last_px4_armed = False
    bridge.mission_inbox.submit_action("UAV-01", "mission_start")
    execution_worker.handle_pending_mission_action(
        healthy_sender_payload(),
        10.0,
    )
    assert execution_worker.mission_status["execution_state"] == "blocked"
    assert execution_worker.mission_status["execution_reason"] == (
        "vehicle is not armed"
    )
    assert execution_worker.mode_requests == []


@pytest.mark.parametrize("execution_state", ["blocked", "aborted"])
def test_disarm_preserves_terminal_mission_diagnostics(
    execution_worker, execution_state
):
    execution_worker.last_px4_armed = False
    execution_worker.mission_status["execution_state"] = execution_state
    execution_worker.mission_status["execution_reason"] = "diagnostic"
    execution_worker.prepare_mission_execution_state()
    assert execution_worker.mission_status["execution_state"] == execution_state
    assert execution_worker.mission_status["execution_reason"] == "diagnostic"


def test_disarm_still_ends_an_active_mission(execution_worker):
    execution_worker.last_px4_armed = False
    execution_worker.mission_status["execution_state"] = "running"
    execution_worker.mission_offboard_expected = True
    execution_worker.prepare_mission_execution_state()
    assert execution_worker.mission_status["execution_state"] == "idle"
    assert execution_worker.mission_status["execution_reason"] == "vehicle is disarmed"
    assert execution_worker.mission_offboard_expected is False


def test_mission_mode_ack_is_scoped_to_a_pending_request(execution_worker):
    ack = SimpleNamespace(
        get_type=lambda: "COMMAND_ACK",
        get_srcSystem=lambda: 1,
        command=mavutil.mavlink.MAV_CMD_DO_SET_MODE,
        result=mavutil.mavlink.MAV_RESULT_ACCEPTED,
    )
    assert execution_worker.handle_mission_mode_command_ack(ack) is False
    execution_worker.mission_mode_ack_pending = True
    assert execution_worker.handle_mission_mode_command_ack(ack) is True
    assert execution_worker.last_offboard_mode_ack_result == (
        mavutil.mavlink.MAV_RESULT_ACCEPTED
    )


def test_offboard_entry_retries_then_reports_timeout(execution_worker):
    bridge.mission_inbox.submit_action("UAV-01", "mission_start")
    execution_worker.handle_pending_mission_action(
        healthy_sender_payload(),
        10.0,
    )
    execution_worker.handle_pending_mission_action(
        healthy_sender_payload(),
        11.1,
    )
    execution_worker.handle_pending_mission_action(
        healthy_sender_payload(),
        12.2,
    )
    assert len(execution_worker.mode_requests) == 3

    execution_worker.handle_pending_mission_action(
        healthy_sender_payload(),
        13.3,
    )
    assert execution_worker.mission_start_pending is False
    assert execution_worker.mission_status["execution_state"] == "blocked"
    assert "3 attempts" in execution_worker.mission_status["execution_reason"]


def test_abort_frame_ends_the_execution_expectation(execution_worker):
    execution_worker.mission_start_pending = True
    execution_worker.mission_offboard_expected = True
    execution_worker.mission_status["execution_state"] = "running"
    execution_worker.handle_pending_mission_action(
        healthy_sender_payload(
            abort_condition="px4_exits_offboard_unexpectedly"
        ),
        20.0,
    )
    assert execution_worker.mission_offboard_expected is False
    assert execution_worker.mission_status["execution_state"] == "aborted"
    assert execution_worker.mission_status["execution_reason"] == (
        "px4_exits_offboard_unexpectedly"
    )


def test_the_inbox_keeps_only_the_newest_execution_action(worker):
    bridge.mission_inbox.submit_action("UAV-01", "mission_start")
    bridge.mission_inbox.submit_action("UAV-01", "mission_stop")
    assert bridge.mission_inbox.take_action("UAV-01") == "mission_stop"
    assert bridge.mission_inbox.take_action("UAV-01") is None


def test_mqtt_routes_mission_start_to_the_worker_inbox(worker):
    message = SimpleNamespace(
        payload=b'{"type":"mission_start","drone_id":"UAV-01"}'
    )
    bridge.on_mqtt_message(None, None, message)
    assert bridge.mission_inbox.take_action("UAV-01") == "mission_start"
