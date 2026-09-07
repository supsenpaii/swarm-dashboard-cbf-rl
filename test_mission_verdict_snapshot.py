"""The bridge's mission verdict must reach the dashboard snapshot.

Regression: the first version read the verdict out of `companion_safety`,
which the websocket snapshot does not carry, so an operator saw "sent" while
the bridge refused every mission.
"""

from __future__ import annotations

import asyncio

import main


def test_verdict_covers_every_drone_even_before_any_mission():
    verdicts = main.mission_verdicts()
    assert set(verdicts) == set(main.ALLOWED_DRONES)
    assert all(value is None for value in verdicts.values())


def test_a_refusal_reaches_the_snapshot_with_its_reason(monkeypatch):
    refusal = {
        "runtime_enabled": False,
        "state": "refused",
        "reason": "mission runtime is disabled (SWARM_MISSION_RUNTIME_ENABLED)",
    }
    monkeypatch.setitem(
        main.tracking_companion_safety, "UAV-01", {"mission": refusal}
    )
    verdict = main.mission_verdicts()["UAV-01"]
    assert verdict["state"] == "refused"
    assert "SWARM_MISSION_RUNTIME_ENABLED" in verdict["reason"]


def test_the_snapshot_copy_cannot_be_mutated_through(monkeypatch):
    live = {"mission": {"state": "installed", "waypoints": 4}}
    monkeypatch.setitem(main.tracking_companion_safety, "UAV-02", live)
    verdict = main.mission_verdicts()["UAV-02"]
    verdict["waypoints"] = 999
    assert live["mission"]["waypoints"] == 4


def test_mission_start_is_published_as_a_companion_command(monkeypatch):
    published = []

    def publish(payload):
        published.append(payload)
        return True, ""

    class WebSocket:
        def __init__(self):
            self.messages = []

        async def send_json(self, payload):
            self.messages.append(payload)

    monkeypatch.setattr(main, "publish_control_message", publish)
    websocket = WebSocket()
    asyncio.run(
        main.handle_mission_action(
            websocket,
            asyncio.Lock(),
            "mission_start",
            "UAV-01",
        )
    )
    assert published == [{"type": "mission_start", "drone_id": "UAV-01"}]
    assert websocket.messages[0]["action"] == "mission_start"
    assert websocket.messages[0]["ok"] is True
