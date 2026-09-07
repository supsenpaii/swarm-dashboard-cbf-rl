#!/usr/bin/env python3
"""Re-run the CBF gate against a flight's own 20 Hz record.

WHY THIS EXISTS
---------------
`sparrow_corridor_replay.py` closes the loop around a modelled vehicle, so
every question it answers is really a question about the model. Twice now the
model has been wrong in a way that hid a real failure -- the command
acceleration limit (38c4f4a) and the plant's own acceleration (5d5ce58) -- and
even with both fixed it reports +9.68 m of margin on the 20 m/s corridor where
the flight's log says -1.33 m.

This tool has no vehicle model at all. It reads what the aircraft actually
measured, frame by frame, and feeds those exact states back through the real
`CbfCommandGate`. That makes it useless for asking "what would happen if the
gate behaved differently" -- the vehicle would then have flown somewhere else
-- and it makes it the only honest way to ask the question that matters here:
GIVEN what the aircraft saw, was the gate's response the right one, and does a
proposed change alter that response on the frames that broke?

The two drones log independently at 20 Hz, so a frame pair is matched by
nearest timestamp. The peer state the gate consumed was itself already aged
(`peer_message_age_ms`), which is recorded and passed through unchanged, so
the pairing error is smaller than the staleness the gate was designed around.

Usage:
    python3 cbf_gate_flight_bench.py artifacts/companion_safety_<profile>.jsonl
"""

from __future__ import annotations

import argparse
import json
import math
import statistics
from pathlib import Path
from typing import Any

DRONE_IDS = ("UAV-01", "UAV-02")


def load_last_run(path: str | Path) -> list[dict[str, Any]]:
    """The last flight in the file.

    One log accumulates every run against a profile, and a stack restart
    resets the monotonic clock, so runs are split where time goes backwards
    or jumps by more than a minute.
    """
    rows = [json.loads(line) for line in Path(path).read_text().splitlines() if line.strip()]
    if not rows:
        raise ValueError(f"{path} is empty")
    runs: list[list[dict[str, Any]]] = [[rows[0]]]
    for previous, current in zip(rows, rows[1:]):
        gap = current["evaluated_monotonic_s"] - previous["evaluated_monotonic_s"]
        if gap < 0.0 or gap > 60.0:
            runs.append([])
        runs[-1].append(current)
    return runs[-1]


def paired_frames(run: list[dict[str, Any]]) -> list[dict[str, dict[str, Any]]]:
    """Frames where both vehicles were station-keeping, matched by time."""
    by_drone = {
        drone_id: [
            row
            for row in run
            if row["drone_id"] == drone_id
            and row["station_keeping"]
            and row.get("own_position_enu_m")
            and row.get("own_velocity_enu_m_s")
        ]
        for drone_id in DRONE_IDS
    }
    if not all(by_drone.values()):
        return []
    other = by_drone[DRONE_IDS[1]]
    times = [row["evaluated_monotonic_s"] for row in other]
    pairs = []
    index = 0
    for row in by_drone[DRONE_IDS[0]]:
        t = row["evaluated_monotonic_s"]
        while index + 1 < len(times) and abs(times[index + 1] - t) <= abs(times[index] - t):
            index += 1
        if abs(times[index] - t) > 0.05:
            continue
        pairs.append({DRONE_IDS[0]: row, DRONE_IDS[1]: other[index]})
    return pairs


def swarm_state(pair: dict[str, dict[str, Any]]) -> dict[str, dict[str, Any]]:
    """The state dict the gate consumed, rebuilt from what each side logged.

    Covariance and message age come from the row of the vehicle that was
    ASKING, because those are the numbers that vehicle's gate actually used --
    the peer's own view of its age is a different quantity.
    """
    asker = pair[DRONE_IDS[0]]
    covariance = asker.get("position_covariance_m2_by_drone") or {}
    ages = dict(asker.get("peer_message_age_ms_by_drone") or {})
    ages[DRONE_IDS[0]] = asker.get("self_message_age_ms")
    state = {}
    for drone_id, row in pair.items():
        state[drone_id] = {
            "position_enu_m": list(row["own_position_enu_m"]),
            "velocity_enu_m_s": list(row["own_velocity_enu_m_s"]),
            "position_covariance_m2": covariance.get(drone_id),
            "message_age_ms": ages.get(drone_id, 20.0),
            "valid": True,
        }
    return state


def _norm(vector: Any) -> float:
    return math.sqrt(sum(float(value) ** 2 for value in vector))


def bench(path: str | Path) -> dict[str, Any]:
    from cbf_command_gate import CbfCommandGate
    from companion_safety import CompanionSafetyMonitor

    run = load_last_run(path)
    pairs = paired_frames(run)
    if not pairs:
        raise ValueError("no station-keeping frames with both vehicles present")

    gates = {
        drone_id: CbfCommandGate(
            drone_id,
            tuple(peer for peer in DRONE_IDS if peer != drone_id),
            CompanionSafetyMonitor.from_environment(drone_id, DRONE_IDS).gate.config,
        )
        for drone_id in DRONE_IDS
    }

    frames: list[dict[str, Any]] = []
    for pair in pairs:
        state = swarm_state(pair)
        row = {"t_s": pair[DRONE_IDS[0]]["evaluated_monotonic_s"]}
        for drone_id in DRONE_IDS:
            recorded = pair[drone_id]
            command = gates[drone_id].filter(
                recorded["nominal_velocity_enu_m_s"], state
            )
            reported = command.as_dict()
            row[drone_id] = {
                "recorded_margin_m": recorded["cbf"].get("minimum_margin_m"),
                "bench_margin_m": reported["minimum_margin_m"],
                "recorded_output_m_s": _norm(recorded["output_velocity_enu_m_s"][:2]),
                "bench_output_m_s": _norm(command.velocity_enu_m_s[:2]),
                "recorded_required_m": recorded["cbf"].get(
                    "critical_required_separation_m"
                ),
                "bench_required_m": reported["critical_required_separation_m"],
                "nominal_m_s": _norm(recorded["nominal_velocity_enu_m_s"][:2]),
            }
        frames.append(row)

    report: dict[str, Any] = {
        "log": str(path),
        "frames": len(frames),
        "duration_s": round(frames[-1]["t_s"] - frames[0]["t_s"], 2),
    }
    for drone_id in DRONE_IDS:
        margins = [f[drone_id]["bench_margin_m"] for f in frames
                   if f[drone_id]["bench_margin_m"] is not None]
        recorded = [f[drone_id]["recorded_margin_m"] for f in frames
                    if f[drone_id]["recorded_margin_m"] is not None]
        # How closely the bench tracks the flight is the bench's own
        # credential: a large error here means the rebuilt state is wrong and
        # nothing else in this report can be trusted.
        errors = [
            abs(f[drone_id]["bench_margin_m"] - f[drone_id]["recorded_margin_m"])
            for f in frames
            if f[drone_id]["bench_margin_m"] is not None
            and f[drone_id]["recorded_margin_m"] is not None
        ]
        swings = []
        for first, second in zip(frames, frames[1:]):
            dt = second["t_s"] - first["t_s"]
            if 0.01 < dt < 0.5:
                swings.append(
                    abs(second[drone_id]["bench_output_m_s"]
                        - first[drone_id]["bench_output_m_s"]) / dt
                )
        report[drone_id] = {
            "recorded_minimum_margin_m": round(min(recorded), 3) if recorded else None,
            "bench_minimum_margin_m": round(min(margins), 3) if margins else None,
            "bench_breach_frames": sum(1 for value in margins if value < 0.0),
            "reproduction_error_m": {
                "median": round(statistics.median(errors), 4) if errors else None,
                "p95": round(sorted(errors)[int(0.95 * len(errors))], 4) if errors else None,
                "max": round(max(errors), 4) if errors else None,
            },
            # The gate's own output slew. A minimally-invasive filter riding
            # its constraint boundary chatters by construction; this is how
            # hard, in units the airframe can be compared against.
            "output_slew_m_s2": {
                "median": round(statistics.median(swings), 2) if swings else None,
                "p90": round(sorted(swings)[int(0.90 * len(swings))], 2) if swings else None,
                "max": round(max(swings), 2) if swings else None,
            },
        }
    report["frames_detail"] = frames
    return report


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("log", help="companion_safety_*.jsonl from a flight")
    parser.add_argument("--profile", help="env profile to load the gate config from")
    parser.add_argument("--window", nargs=2, type=float, metavar=("START_S", "END_S"))
    arguments = parser.parse_args()

    import os

    saved = dict(os.environ)
    try:
        if arguments.profile:
            from sparrow_corridor_replay import load_env_profile

            load_env_profile(arguments.profile)
        report = bench(arguments.log)
    finally:
        os.environ.clear()
        os.environ.update(saved)

    frames = report.pop("frames_detail")
    print(json.dumps(report, indent=2))
    if arguments.window:
        start, end = arguments.window
        base = frames[0]["t_s"]
        print(f"\n{'t':>6} " + " ".join(
            f"{d}: {'req':>7} {'margin':>8} {'out':>6} {'nom':>6}" for d in DRONE_IDS
        ))
        for frame in frames:
            t = frame["t_s"] - base
            if not start <= t <= end:
                continue
            cells = []
            for drone_id in DRONE_IDS:
                cell = frame[drone_id]
                cells.append(
                    f"{(cell['bench_required_m'] or 0):7.2f} "
                    f"{(cell['bench_margin_m'] if cell['bench_margin_m'] is not None else 0):8.2f} "
                    f"{cell['bench_output_m_s']:6.2f} {cell['nominal_m_s']:6.2f}"
                )
            print(f"{t:6.2f} " + "      ".join(cells))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
