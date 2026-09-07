"""The response fit must recover a known lag and refuse a lying channel."""

from __future__ import annotations

import json
import math

from companion_response_id import analyse


def _write_trace(path, *, vertical_velocity_is_real: bool, warmup: bool = False) -> None:
    """Synthesise a trace: E is a real first-order plant, U reports a response
    the position channel never makes when `vertical_velocity_is_real` is False.
    """
    step_s = 0.05
    tau_s = 0.5
    pole = math.exp(-step_s / tau_s)
    position = [0.0, 0.0, 9.0]
    velocity = [0.0, 0.0, 0.0]
    with open(path, "w", encoding="utf-8") as handle:
        for index in range(400):
            # A square wave, the input `latency_measurement_flight.py` flies:
            # a constant command cannot separate plant gain from plant bias.
            command = [1.0 if (index // 40) % 2 == 0 else 0.2, 0.0, -0.2]
            # Record state BEFORE applying this frame's command: a measured
            # velocity can never already contain the command sent alongside it.
            handle.write(
                json.dumps(
                    {
                        "wall_clock_s": index * step_s,
                        "drone_id": "UAV-01",
                        "nominal_active": True,
                        "output_valid": True,
                        "own_position_enu_m": list(position),
                        "output_velocity_enu_m_s": command,
                        "own_velocity_enu_m_s": list(velocity),
                        # A warmup frame streams zeros while the line above
                        # still shows the live command.
                        "active_offboard_frame": {
                            "decision": "transmit_warmup" if warmup else "transmit",
                            "transmitted": True,
                            "velocity_ned_m_s": (
                                [0.0, 0.0, 0.0]
                                if warmup
                                else [command[1], command[0], -command[2]]
                            ),
                        },
                    }
                )
                + "\n"
            )
            # The plant only ever sees what was transmitted, so a warmup frame
            # leaves it still while the recorded command says otherwise.
            received = [0.0, 0.0, 0.0] if warmup else command
            for axis in range(3):
                velocity[axis] = pole * velocity[axis] + (1.0 - pole) * received[axis]
            position[0] += velocity[0] * step_s
            position[1] += velocity[1] * step_s
            # The failure mode: altitude drifts up while vz claims a descent.
            position[2] += (velocity[2] if vertical_velocity_is_real else 0.012) * step_s


def test_recovers_the_injected_time_constant_and_unit_gain(tmp_path):
    trace = tmp_path / "honest.jsonl"
    _write_trace(trace, vertical_velocity_is_real=True)
    report = analyse(trace)
    east = report["per_drone"]["UAV-01"]["response"]["E"]
    assert abs(east["time_constant_s"] - 0.5) < 0.01
    assert abs(east["steady_state_gain"] - 1.0) < 0.01
    assert abs(east["bias_m_s"]) < 0.01
    assert report["verdict"] == "CONSISTENT"
    assert report["inconsistent_axes"] == []


def test_flags_a_velocity_channel_the_position_channel_contradicts(tmp_path):
    trace = tmp_path / "lying.jsonl"
    _write_trace(trace, vertical_velocity_is_real=False)
    report = analyse(trace)
    consistency = report["per_drone"]["UAV-01"]["channel_consistency"]
    assert consistency["E"]["consistent"] is True
    assert consistency["U"]["consistent"] is False
    assert report["inconsistent_axes"] == ["UAV-01:U"]
    assert report["verdict"] == "CHANNEL_MISMATCH"


def test_warmup_frames_never_reach_the_fit(tmp_path):
    """The 2026-08-12 trap: a live command recorded beside a transmitted zero.

    Fitting those makes any plant look dead, which is how the vertical channel
    was written up as a blocker. The loader must drop them instead.
    """
    trace = tmp_path / "warmup.jsonl"
    _write_trace(trace, vertical_velocity_is_real=True, warmup=True)
    report = analyse(trace)

    assert report["sample_provenance"]["kept"] == 0
    assert report["sample_provenance"]["not_transmitted"] == 400
    assert report["per_drone"] == {}


def test_transmitted_frames_are_kept_and_counted(tmp_path):
    trace = tmp_path / "honest.jsonl"
    _write_trace(trace, vertical_velocity_is_real=True)
    provenance = analyse(trace)["sample_provenance"]

    assert provenance["kept"] == 400
    assert provenance["not_transmitted"] == 0
    assert provenance["unverifiable"] == 0
