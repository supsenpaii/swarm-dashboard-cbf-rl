from types import SimpleNamespace

import core_range_dynamic_capture as capture


class Contract:
    total_duration_s = 0.1

    def command_payload(self, session):
        return f"start|{session}"


def test_sim_time_start_retries_until_activation_ack(monkeypatch, tmp_path):
    clock = [0.0]
    publishes = []

    monkeypatch.setattr(capture, "contract_from_args", lambda _args: Contract())
    monkeypatch.setattr(capture.time, "monotonic", lambda: clock[0])
    monkeypatch.setattr(capture.time, "sleep", lambda seconds: clock.__setitem__(0, clock[0] + seconds))
    monkeypatch.setattr(capture, "assert_observation_only", lambda _status: None)
    monkeypatch.setattr(capture, "status", lambda: {})

    def publish(payload, *, wait_for_subscriber=False):
        publishes.append((payload, wait_for_subscriber))

    monkeypatch.setattr(capture, "_publish_sim_time_command", publish)

    def events(_path, _session):
        start_count = sum(payload.startswith("start|") for payload, _ in publishes)
        if start_count < 2:
            return []
        return [
            {"kind": "trajectory_activated", "sim_timestamp_s": 10.0},
            {"kind": "trajectory_completed", "sim_timestamp_s": 10.1},
        ]

    monkeypatch.setattr(capture, "_trajectory_events", events)
    args = SimpleNamespace(scenario_id="s1", capture_timeout_s=1.0)
    result = capture.run_sim_time_trajectory(args, tmp_path / "events.jsonl")

    assert result["start_publish_attempts"] == 2
    assert result["transport_publish_count"] == 3
    assert publishes[-1][0] == "stop|s1"
