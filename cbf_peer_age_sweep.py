#!/usr/bin/env python3
"""CBF_PEER_AGE_OFFLINE_SWEEP: check the continuous peer-age safe region.

Offline only.  This imports the same kinematic replay and genuine validated
crossing geometry as ``cbf_uncertainty_sigma_sweep.py``; it makes no network,
PX4, MQTT, socket, arming, mode, or environment-file change.

The uncertainty feature is deliberately held off (``sigma=0`` and covariance
not published).  Therefore the result isolates the existing peer-state-age
contract from covariance tuning.  Ages are scanned at one-millisecond
resolution from fresh state upward until the first hard infeasibility.  The
largest preceding age is the continuous geometry-safe lower bound when the
scan ends without a failure. It is evidence only: a less binding geometry is
not permission to loosen the already flight-validated 100 ms production cap.

Run: python3 cbf_peer_age_sweep.py [--json PATH]
"""

from __future__ import annotations

import argparse
import json
from dataclasses import replace
from pathlib import Path
from typing import Any

from cbf_uncertainty_sigma_sweep import (
    ENV,
    _expected_completers,
    _feasible,
    scenarios,
    simulate,
)


POLICY_ENV = "SWARM_PEER_STATE_MAX_AGE_MS"
DEFAULT_POLICY_MAX_AGE_MS = 100.0
MINIMUM_SCAN_MAX_AGE_MS = 500.0
SCAN_STEP_MS = 1.0


def _crossing_scenario():
    for scenario in scenarios():
        if scenario.name == "crossing_validated":
            return scenario
    raise ValueError("crossing_validated_scenario_missing")


def _configured_policy_max_age_ms() -> float:
    try:
        value = float(ENV.get(POLICY_ENV, DEFAULT_POLICY_MAX_AGE_MS))
    except (TypeError, ValueError):
        return DEFAULT_POLICY_MAX_AGE_MS
    if value < 0.0:
        raise ValueError(f"{POLICY_ENV}_invalid")
    return value


def _run_at_age(peer_age_ms: float) -> dict[str, Any]:
    scenario = replace(_crossing_scenario(), peer_age_ms=float(peer_age_ms))
    row = simulate(scenario, 0.0, with_covariance=False).as_dict()
    hard, strict = _feasible(row, _expected_completers(scenario))
    return {
        "peer_age_ms": round(float(peer_age_ms), 3),
        "hard_feasible": hard,
        "strict_feasible": strict,
        **row,
    }


def run_sweep() -> dict[str, Any]:
    """Scan the fresh-to-stale contiguous feasible region at 1 ms steps."""
    policy_max_age_ms = _configured_policy_max_age_ms()
    scan_max_age_ms = max(policy_max_age_ms, MINIMUM_SCAN_MAX_AGE_MS)
    age_ms = 0.0
    rows: list[dict[str, Any]] = []
    last_hard_feasible_ms: float | None = None
    first_hard_failure_ms: float | None = None

    while age_ms <= scan_max_age_ms:
        row = _run_at_age(age_ms)
        rows.append(row)
        if not row["hard_feasible"]:
            first_hard_failure_ms = age_ms
            break
        last_hard_feasible_ms = age_ms
        age_ms = round(age_ms + SCAN_STEP_MS, 3)

    policy_row = next(
        (row for row in rows if row["peer_age_ms"] == policy_max_age_ms),
        _run_at_age(policy_max_age_ms),
    )
    comparison_450ms_row = _run_at_age(450.0)

    return {
        "milestone": "CBF_PEER_AGE_OFFLINE_SWEEP",
        "scenario": "crossing_validated",
        "uncertainty_feature": "off",
        "covariance_sigma": 0.0,
        "position_covariance_published": False,
        "policy_environment": POLICY_ENV,
        "configured_policy_max_age_ms": policy_max_age_ms,
        "scan_step_ms": SCAN_STEP_MS,
        "continuous_hard_feasible_up_to_peer_age_ms": last_hard_feasible_ms,
        "first_hard_failure_peer_age_ms": first_hard_failure_ms,
        "policy_age_hard_feasible": policy_row["hard_feasible"],
        "policy_age_strict_feasible": policy_row["strict_feasible"],
        "policy_age_result": policy_row,
        "comparison_450ms_result": comparison_450ms_row,
        "runs": rows,
        "interpretation": (
            "The continuous limit is the largest age whose every smaller "
            "integer-millisecond age was hard-feasible. If no failure was "
            "found it is a lower bound, not a new policy recommendation."
        ),
    }


def _print(report: dict[str, Any]) -> None:
    result = report["policy_age_result"]
    print("CBF peer-age offline sweep (validated crossing, sigma=0, covariance off)")
    print(f"configured policy maximum: {report['configured_policy_max_age_ms']:.0f} ms")
    print(
        "continuous hard-feasible limit: "
        f"{report['continuous_hard_feasible_up_to_peer_age_ms']} ms"
    )
    print(f"first hard failure: {report['first_hard_failure_peer_age_ms']} ms")
    print(
        f"policy age hard/strict feasible: {result['hard_feasible']}/"
        f"{result['strict_feasible']}"
    )
    print(
        "policy-age result: "
        f"margin={result['min_margin_reported_m']:.3f} m, "
        f"infeasible_frames={result['infeasible_frames']}, "
        f"min_distance={result['min_distance_m']:.3f} m, "
        f"stage={result['max_emergency_stage']}"
    )


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--json",
        default="artifacts/cbf_peer_age_sweep/peer_age_sweep.json",
        help="where to write the full report",
    )
    arguments = parser.parse_args()
    report = run_sweep()
    _print(report)
    destination = Path(arguments.json)
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(f"wrote {destination}")


if __name__ == "__main__":
    main()
