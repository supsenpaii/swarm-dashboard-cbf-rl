import pytest

import range_v2_r3_6a_capture as capture


def _status(armed_present: bool, armed: bool = False) -> dict:
    drone = {"status": {}}
    if armed_present:
        drone["status"]["armed"] = armed
    return {"drones": {"UAV-01": drone, "UAV-02": drone}, "tracking": {}}


def test_observation_ready_waits_for_telemetry(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    values = iter([_status(False), _status(False), _status(True)])
    monkeypatch.setattr(capture, "status", lambda: next(values))
    monkeypatch.setattr(capture.time, "sleep", lambda _seconds: None)
    result = capture.wait_for_observation_only_ready(timeout_s=5.0)
    assert result["drones"]["UAV-01"]["status"]["armed"] is False


def test_observation_ready_fails_immediately_if_armed(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setattr(capture, "status", lambda: _status(True, armed=True))
    with pytest.raises(RuntimeError, match="armed_vehicle_detected"):
        capture.wait_for_observation_only_ready(timeout_s=5.0)
