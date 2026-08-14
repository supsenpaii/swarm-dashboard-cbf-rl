#!/usr/bin/env python3
"""Prepare, or explicitly execute, the non-conflict sigma-candidate flight."""

from __future__ import annotations

import argparse
import hashlib
import json
import re
import subprocess
import sys
from pathlib import Path
from typing import Any

from two_uav_trajectory_flight import DEFAULT_TRAJECTORY_ENV


REPO = Path(__file__).resolve().parent
RESTING_ENV = {
    "SWARM_OFFBOARD_AUTHORITY": "disabled",
    "SWARM_PEER_STATE_MAX_AGE_MS": "100",
    "SWARM_CBF_UNCERTAINTY_SOURCE": "odometry",
    "SWARM_CBF_COVARIANCE_SIGMA": "0.0",
    "SWARM_CBF_REQUIRE_POSITION_COVARIANCE": "false",
}


def env_file(path: Path = REPO / ".env") -> dict[str, str]:
    values: dict[str, str] = {}
    for line in path.read_text(encoding="utf-8").splitlines():
        match = re.match(r"^([A-Z0-9_]+)=(.*)$", line.strip())
        if match:
            values[match.group(1)] = match.group(2).strip()
    return values


def configuration_failures(
    actual: dict[str, str], required: dict[str, str]
) -> list[str]:
    return [
        f"{key}:expected={expected}:actual={actual.get(key)}"
        for key, expected in required.items()
        if actual.get(key) != expected
    ]


def build_plan(selection_path: Path, output_dir: Path) -> dict[str, Any]:
    selection = json.loads(selection_path.read_text(encoding="utf-8"))
    if selection.get("verdict") != "CBF_UNCERTAINTY_SIGMA_CANDIDATE_SELECTED":
        raise ValueError("sigma candidate selection is not validated")
    sigma = selection.get("selected_sigma_for_flight_validation")
    if not isinstance(sigma, (int, float)) or sigma <= 0.0:
        raise ValueError("selected flight-validation sigma is invalid")
    selected = next(
        row for row in selection["candidates"] if row["sigma"] == sigma
    )
    if not selected["eligible_for_flight_validation"]:
        raise ValueError("selected sigma is not eligible for flight validation")

    required_flight_env = {
        "SWARM_OFFBOARD_AUTHORITY": "companion_safety",
        "SWARM_PEER_STATE_MAX_AGE_MS": "100",
        "SWARM_CBF_UNCERTAINTY_SOURCE": "odometry",
        "SWARM_CBF_COVARIANCE_SIGMA": str(sigma),
        "SWARM_CBF_REQUIRE_POSITION_COVARIANCE": "true",
        **DEFAULT_TRAJECTORY_ENV,
    }
    collector_command = [
        sys.executable,
        "flight_covariance_measurement.py",
        "--hold-s",
        "25",
        "--sample-hz",
        "10",
        "--prearm-s",
        "5",
        "--expected-sigma",
        str(sigma),
        "--expected-require-covariance",
        "--require-clean-safety-events",
        "--output-dir",
        str(output_dir),
    ]
    return {
        "milestone": "CBF_UNCERTAINTY_FLIGHT_VALIDATION_PREP",
        "scenario": "parallel_trajectory_baseline_non_conflict",
        "selected_sigma_for_flight_validation": sigma,
        "production_sigma_change_authorized": False,
        "crossing_flight_authorized": False,
        "required_temporary_flight_env": required_flight_env,
        "collector_command": collector_command,
        "prearm_gates": [
            "both UAVs disarmed",
            "both covariance vectors present, finite, nonnegative, and <=100 ms old",
            "position_covariance_m2_by_drone complete on both UAVs",
            f"live covariance_sigma equals {sigma}",
            "live require_position_covariance is true",
        ],
        "postflight_gates": [
            "flight driver verdict FLIGHT_PASS",
            "zero missing covariance samples",
            "zero uncertainty-configuration mismatch samples",
            "zero negative CBF margin or infeasible samples",
            "supervisor normal; no watchdog condition or sender latch",
            "both UAVs land and disarm",
        ],
        "cleanup": [
            "stop stack",
            "restore .env to resting values",
            "verify no T-state process",
            "run full regression",
        ],
        "selection_artifact": str(selection_path),
        "selection_sha256": hashlib.sha256(selection_path.read_bytes()).hexdigest(),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--execute", action="store_true", help="explicitly run the flight")
    parser.add_argument(
        "--selection",
        type=Path,
        default=Path(
            "artifacts/cbf_uncertainty_sigma_candidate_selection/selection.json"
        ),
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path("artifacts/cbf_uncertainty_flight_validation"),
    )
    parser.add_argument(
        "--prep-json",
        type=Path,
        default=Path("artifacts/cbf_uncertainty_flight_validation_prep/prep.json"),
    )
    arguments = parser.parse_args()
    plan = build_plan(arguments.selection, arguments.output_dir)
    actual = env_file()
    required = (
        plan["required_temporary_flight_env"] if arguments.execute else RESTING_ENV
    )
    failures = configuration_failures(actual, required)
    plan["mode"] = "execute" if arguments.execute else "prepare_only"
    plan["environment_validation"] = "PASS" if not failures else "FAIL"
    plan["environment_failures"] = failures
    plan["verdict"] = (
        "CBF_UNCERTAINTY_FLIGHT_VALIDATION_PREP_READY"
        if not arguments.execute and not failures
        else "CBF_UNCERTAINTY_FLIGHT_VALIDATION_PREP_NOT_READY"
        if failures
        else "CBF_UNCERTAINTY_FLIGHT_VALIDATION_EXECUTING"
    )
    arguments.prep_json.parent.mkdir(parents=True, exist_ok=True)
    arguments.prep_json.write_text(json.dumps(plan, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(plan, indent=2))
    if failures:
        return 1
    if not arguments.execute:
        return 0
    return subprocess.run(plan["collector_command"], cwd=REPO, check=False).returncode


if __name__ == "__main__":
    raise SystemExit(main())
