#!/usr/bin/env python3
"""Deterministic 20 m Sparrow safety gate over speed, angle and lag bounds."""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
from typing import Any

from cbf_rl_x500_safety_matrix import (
    SafetyCase,
    _horizontal_case as _profile_horizontal_case,
    _vertical_case as _profile_vertical_case,
    evaluate_case,
    run as _run,
)
from cbf_rl_x500_safety_matrix import cases as _cases


def _horizontal_case(speed: float, angle: float, tau: float, age_ms: float) -> SafetyCase:
    return _profile_horizontal_case(speed, angle, tau, age_ms, "sparrow")


def _vertical_case(speed: float, tau: float, age_ms: float) -> SafetyCase:
    return _profile_vertical_case(speed, tau, age_ms, "sparrow")


def cases(maximum_speed_m_s: int = 10) -> tuple[SafetyCase, ...]:
    return _cases(maximum_speed_m_s, "sparrow")


def run(
    model: str | Path,
    *,
    workers: int = 1,
    maximum_speed_m_s: int | None = None,
) -> dict[str, Any]:
    return _run(
        model,
        workers=workers,
        maximum_speed_m_s=maximum_speed_m_s,
        expected_vehicle_profile="sparrow",
    )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("model")
    parser.add_argument(
        "--workers", type=int, default=max(1, min(8, (os.cpu_count() or 2) - 2))
    )
    parser.add_argument("--maximum-speed-m-s", type=int)
    parser.add_argument(
        "--output", default="artifacts/cbf_rl_sparrow_20m_safety_matrix.json"
    )
    arguments = parser.parse_args()
    report = run(
        arguments.model,
        workers=arguments.workers,
        maximum_speed_m_s=arguments.maximum_speed_m_s,
    )
    destination = Path(arguments.output)
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_text(
        json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    print(
        json.dumps(
            {key: value for key, value in report.items() if key != "results"},
            indent=2,
        )
    )
    print(f"wrote {destination}")
    return 0 if report["verdict"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
