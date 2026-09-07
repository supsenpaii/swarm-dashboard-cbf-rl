#!/usr/bin/env python3
"""Offline plant system-ID over a companion-safety JSONL trace.

`latency_measurement_flight.py` flies and records; this is the separate offline
fit it defers to. Per drone and per ENU axis it answers two questions:

  1. How does the vehicle respond to a commanded velocity?  A first-order fit
     ``v[k+1] = a*v[k] + b*u[k] + c`` gives the response time constant, the
     steady-state command->response gain, and a bias.
  2. Do the position and velocity channels agree?  They come from different
     PX4 messages (position from GLOBAL_POSITION_INT via `geodetic_to_enu`,
     velocity from LOCAL_POSITION_NED via `ned_to_enu`), so an axis where the
     integrated reported velocity does not match the travelled distance is
     reporting a response the vehicle never made -- and no position controller
     closed over that axis can converge, whatever its gains.

Read-only: no flight authority, no simulator, no network.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Any

import numpy as np

# A sample pair is only usable if it sits within one nominal frame of its
# neighbour; the trace interleaves drones and can drop frames.
MINIMUM_STEP_S = 0.025
MAXIMUM_STEP_S = 0.100
# Consistency tolerance: the horizontal axes of every healthy flight so far
# land inside 3%, so 20% is loose enough to absorb estimator noise on a short
# leg and still refuse a channel that is off by metres.
CONSISTENCY_SLOPE_RANGE = (0.8, 1.2)
CONSISTENCY_ABSOLUTE_M = 0.5
CONSISTENCY_RELATIVE = 0.20
# An axis the flight never moved carries no evidence either way. Judging one
# anyway turns estimator noise into a verdict, so both the fit and the
# consistency check report "undetermined" below this measured-motion floor.
MINIMUM_EXCITATION_M_S = 0.05
AXES = ("E", "N", "U")


def reached_the_vehicle(record: dict[str, Any]) -> bool:
    """Did the command recorded here actually go out to PX4?

    `output_velocity_enu_m_s` is what the companion decided, which is not what
    the vehicle received: a warmup frame streams zeros while that field still
    carries the live command, and a withheld frame sends nothing at all.
    Fitting those against the vehicle's response fits a command the vehicle
    never got, and the plant looks dead on whichever axis the companion wanted
    most. In `companion_safety_trace_20260812_preguard.jsonl` 25,253 of the
    35,472 samples this function now rejects were warmup, and 47% of them
    wanted over 0.3 m/s of climb while a zero went out -- which is the whole
    reason the vertical channel was written up as a blocker.

    Traces recorded before per-frame accounting carry no frame at all. They
    cannot be checked, so they are kept and counted separately rather than
    silently dropped.
    """
    frame = record.get("active_offboard_frame")
    if frame is None:
        return True
    return bool(frame.get("transmitted")) and frame.get("decision") == "transmit"


def load_samples(
    path: str | Path,
) -> tuple[dict[str, list[tuple[float, list[float], list[float], list[float]]]], dict[str, int]]:
    """Return per-drone (time, position, command, measured velocity) samples."""
    samples: dict[str, list[Any]] = {}
    provenance = {"kept": 0, "not_transmitted": 0, "unverifiable": 0}
    with open(path, encoding="utf-8") as handle:
        for line in handle:
            record = json.loads(line)
            if not record.get("nominal_active") or not record.get("output_valid"):
                continue
            position = record.get("own_position_enu_m")
            command = record.get("output_velocity_enu_m_s")
            measured = record.get("own_velocity_enu_m_s")
            if not position or not command or not measured:
                continue
            if not reached_the_vehicle(record):
                provenance["not_transmitted"] += 1
                continue
            if record.get("active_offboard_frame") is None:
                provenance["unverifiable"] += 1
            provenance["kept"] += 1
            samples.setdefault(record["drone_id"], []).append(
                (float(record["wall_clock_s"]), position, command, measured)
            )
    for series in samples.values():
        series.sort()
    return samples, provenance


def _pairs(series: list[Any]) -> list[tuple[Any, Any, float]]:
    return [
        (first, second, second[0] - first[0])
        for first, second in zip(series, series[1:])
        if MINIMUM_STEP_S < second[0] - first[0] < MAXIMUM_STEP_S
    ]


def _excitation_m_s(pairs: list[tuple[Any, Any, float]], axis: int) -> float:
    measured = np.array([first[3][axis] for first, _, _ in pairs])
    return float(np.sqrt((measured**2).mean()))


def fit_axis(series: list[Any], axis: int) -> dict[str, Any]:
    """First-order command->response fit for one axis."""
    pairs = _pairs(series)
    if len(pairs) < 10:
        return {"samples": len(pairs), "fit": None}
    if _excitation_m_s(pairs, axis) < MINIMUM_EXCITATION_M_S:
        return {"samples": len(pairs), "fit": None, "reason": "insufficient_excitation"}
    commands = np.array([first[2][axis] for first, _, _ in pairs])
    # A command that never varies is collinear with the bias column, and the
    # least-squares split between the two is then arbitrary. Drop the bias
    # rather than report a gain that is half its true value.
    separable = float(commands.std()) >= MINIMUM_EXCITATION_M_S
    columns = [[first[3][axis] for first, _, _ in pairs], list(commands)]
    if separable:
        columns.append([1.0] * len(pairs))
    design = np.array(columns).T
    target = np.array([second[3][axis] for _, second, _ in pairs])
    solution, *_ = np.linalg.lstsq(design, target, rcond=None)
    pole, gain = solution[0], solution[1]
    offset = solution[2] if separable else 0.0
    step_s = float(np.median([step for _, _, step in pairs]))
    residual = float(np.sqrt(((design @ solution - target) ** 2).mean()))
    settled = pole < 1.0
    return {
        "samples": len(pairs),
        "step_s": round(step_s, 4),
        "pole": round(float(pole), 6),
        # A pole at or above 1 means the fit found no decay: the response is
        # not a stable first-order lag and the time constant is meaningless.
        "time_constant_s": round(-step_s / math.log(pole), 4) if 0.0 < pole < 1.0 else None,
        "steady_state_gain": round(float(gain / (1.0 - pole)), 4) if settled else None,
        "bias_m_s": (
            round(float(offset / (1.0 - pole)), 5) if settled and separable else None
        ),
        "residual_rms_m_s": round(residual, 5),
    }


def consistency_axis(series: list[Any], axis: int) -> dict[str, Any]:
    """Does the reported velocity on this axis explain the travelled distance?"""
    pairs = _pairs(series)
    if len(pairs) < 10:
        return {"samples": len(pairs), "consistent": None}
    if _excitation_m_s(pairs, axis) < MINIMUM_EXCITATION_M_S:
        return {
            "samples": len(pairs),
            "consistent": None,
            "reason": "insufficient_excitation",
        }
    integrated = sum(
        0.5 * (first[3][axis] + second[3][axis]) * step for first, second, step in pairs
    )
    # Both sides must cover the same intervals. Taking the endpoints of the
    # whole series instead charges the velocity channel for every gap between
    # OFFBOARD episodes, which flags all three axes on any multi-episode trace.
    travelled = sum(second[1][axis] - first[1][axis] for first, second, _ in pairs)
    measured = np.array([0.5 * (first[3][axis] + second[3][axis]) for first, second, _ in pairs])
    differentiated = np.array(
        [(second[1][axis] - first[1][axis]) / step for first, second, step in pairs
    ])
    denominator = float(measured @ measured)
    slope = float(measured @ differentiated / denominator) if denominator > 1e-9 else None
    tolerance = CONSISTENCY_ABSOLUTE_M + CONSISTENCY_RELATIVE * abs(travelled)
    return {
        "samples": len(pairs),
        "integrated_velocity_m": round(integrated, 3),
        "travelled_m": round(travelled, 3),
        "position_over_velocity_slope": None if slope is None else round(slope, 3),
        "consistent": bool(
            abs(integrated - travelled) <= tolerance
            and slope is not None
            and CONSISTENCY_SLOPE_RANGE[0] <= slope <= CONSISTENCY_SLOPE_RANGE[1]
        ),
    }


def analyse(path: str | Path) -> dict[str, Any]:
    per_drone = {}
    samples, provenance = load_samples(path)
    for drone, series in sorted(samples.items()):
        per_drone[drone] = {
            "samples": len(series),
            "duration_s": round(series[-1][0] - series[0][0], 2) if series else 0.0,
            "response": {name: fit_axis(series, axis) for axis, name in enumerate(AXES)},
            "channel_consistency": {
                name: consistency_axis(series, axis) for axis, name in enumerate(AXES)
            },
        }
    inconsistent = sorted(
        f"{drone}:{name}"
        for drone, report in per_drone.items()
        for name, axis in report["channel_consistency"].items()
        if axis.get("consistent") is False
    )
    return {
        "trace": str(path),
        "sample_provenance": provenance,
        "per_drone": per_drone,
        "inconsistent_axes": inconsistent,
        "verdict": "CONSISTENT" if not inconsistent else "CHANNEL_MISMATCH",
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("trace", help="companion-safety JSONL trace")
    parser.add_argument("--output")
    arguments = parser.parse_args()
    report = analyse(arguments.trace)
    encoded = json.dumps(report, indent=2, sort_keys=True) + "\n"
    if arguments.output:
        Path(arguments.output).write_text(encoded, encoding="utf-8")
    print(encoded, end="")
    return 0 if report["verdict"] == "CONSISTENT" else 1


if __name__ == "__main__":
    raise SystemExit(main())
