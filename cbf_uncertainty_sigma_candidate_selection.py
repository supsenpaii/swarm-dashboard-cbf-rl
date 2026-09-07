#!/usr/bin/env python3
"""Select an uncertainty sigma for flight validation, never production."""

from __future__ import annotations

import argparse
import hashlib
import json
from dataclasses import replace
from pathlib import Path
from typing import Any

from cbf_uncertainty_offline_sweep_v2 import covariance_profiles, load_trace
from cbf_uncertainty_sigma_sweep import (
    ENV,
    _expected_completers,
    _feasible,
    scenarios,
    simulate,
)


# Existing crossing-flight design criterion from test_crossing_geometry_buffer.py.
# This is a selection gate, not a new CBF constant.
MINIMUM_FLIGHT_VALIDATION_MARGIN_M = 0.30


def choose_candidate(rows: list[dict[str, Any]]) -> float | None:
    eligible = [row["sigma"] for row in rows if row["eligible_for_flight_validation"]]
    return max(eligible) if eligible else None


def run_selection(sweep_path: Path, trace_path: Path) -> dict[str, Any]:
    sweep = json.loads(sweep_path.read_text(encoding="utf-8"))
    if sweep.get("verdict") != "CBF_UNCERTAINTY_OFFLINE_SWEEP_V2_VALIDATED":
        raise ValueError("offline sweep V2 is not validated")
    trace_hash = hashlib.sha256(trace_path.read_bytes()).hexdigest()
    if sweep.get("source", {}).get("sha256") != trace_hash:
        raise ValueError("flight covariance trace does not match sweep V2")

    runtime_sigma = float(ENV.get("SWARM_CBF_COVARIANCE_SIGMA", 0.0))
    runtime_require = ENV.get("SWARM_CBF_REQUIRE_POSITION_COVARIANCE", "false").lower()
    if runtime_sigma != 0.0 or runtime_require in {"1", "true", "yes", "on"}:
        raise ValueError("production uncertainty behavior must remain disabled during selection")

    trace = load_trace(trace_path)
    profiles = covariance_profiles(trace)
    observed_peer_age_max_ms = max(
        float(row["drones"][drone]["peer_age_ms"])
        for row in trace["rows"]
        for drone in ("UAV-01", "UAV-02")
    )
    crossing = replace(
        next(scenario for scenario in scenarios() if scenario.name == "crossing_validated"),
        peer_age_ms=observed_peer_age_max_ms,
    )
    completers = _expected_completers(crossing)
    contract_summary = {
        row["sigma"]: row for row in sweep["candidate_grid_summary"]
    }

    candidates: list[dict[str, Any]] = []
    for sigma in sweep["offline_strict_candidate_sigmas"]:
        profile_runs = []
        for profile_name, profile in profiles.items():
            run = simulate(
                crossing,
                sigma,
                covariance_frames=profile["frames"],
                covariance_sample_period_s=trace["sample_period_s"],
            )
            row = run.as_dict()
            hard, strict = _feasible(row, completers)
            profile_runs.append(
                {
                    "covariance_profile": profile_name,
                    "hard_feasible": hard,
                    "strict_feasible": strict,
                    "min_margin_reported_m_raw": run.min_margin_reported_m,
                    **row,
                }
            )
        worst = min(profile_runs, key=lambda row: row["min_margin_reported_m_raw"])
        contract = contract_summary[sigma]
        comfortable = (
            worst["min_margin_reported_m_raw"]
            >= MINIMUM_FLIGHT_VALIDATION_MARGIN_M
        )
        eligible = (
            contract["all_strict_feasible"]
            and all(row["hard_feasible"] and row["strict_feasible"] for row in profile_runs)
            and comfortable
        )
        candidates.append(
            {
                "sigma": sigma,
                "uncertainty_margin_at_measured_max_m": (
                    sigma * trace["u_base_distribution_m"]["max"]
                ),
                "strict_feasible_at_peer_age_contract_100ms": contract[
                    "all_strict_feasible"
                ],
                "worst_margin_at_observed_peer_age_max_m": worst[
                    "min_margin_reported_m_raw"
                ],
                "worst_covariance_profile": worst["covariance_profile"],
                "comfortable_margin_pass": comfortable,
                "eligible_for_flight_validation": eligible,
                "profile_runs": profile_runs,
            }
        )

    selected = choose_candidate(candidates)
    verdict = (
        "CBF_UNCERTAINTY_SIGMA_CANDIDATE_SELECTED"
        if selected is not None
        else "CBF_UNCERTAINTY_SIGMA_CANDIDATE_NOT_SELECTED"
    )
    return {
        "milestone": "CBF_UNCERTAINTY_SIGMA_CANDIDATE_SELECTION",
        "verdict": verdict,
        "selected_sigma_for_flight_validation": selected,
        "production_sigma_selected": False,
        "production_configuration_changed": False,
        "selection_rule": (
            "Require strict feasibility across sweep V2 at the 100 ms peer-age "
            "contract, require the existing 0.30 m comfortable-margin criterion "
            "at the maximum peer age observed in the covariance flight, then "
            "select the largest remaining sigma to represent the most measured uncertainty."
        ),
        "minimum_flight_validation_margin_m": MINIMUM_FLIGHT_VALIDATION_MARGIN_M,
        "observed_peer_age_max_ms": observed_peer_age_max_ms,
        "measured_u_base_max_m": trace["u_base_distribution_m"]["max"],
        "candidates": candidates,
        "inputs": {
            "sweep_v2": str(sweep_path),
            "flight_covariance_trace": str(trace_path),
            "flight_covariance_trace_sha256": trace_hash,
        },
        "runtime_rest_state": {
            "covariance_sigma": runtime_sigma,
            "require_position_covariance": False,
        },
    }


def print_report(report: dict[str, Any]) -> None:
    print(report["verdict"])
    print("sigma  unc@MAX  margin@observed-peer-MAX  comfortable  eligible")
    for row in report["candidates"]:
        print(
            f"{row['sigma']:>4.2f}  "
            f"{row['uncertainty_margin_at_measured_max_m']:>7.4f}  "
            f"{row['worst_margin_at_observed_peer_age_max_m']:>24.3f}  "
            f"{str(row['comfortable_margin_pass']):>11}  "
            f"{str(row['eligible_for_flight_validation']):>8}"
        )
    print(
        "selected for flight validation: "
        f"sigma={report['selected_sigma_for_flight_validation']}"
    )
    print("production sigma remains 0.0; require_position_covariance remains false")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--sweep",
        type=Path,
        default=Path("artifacts/cbf_uncertainty_sweep_v2/sweep.json"),
    )
    parser.add_argument(
        "--trace",
        type=Path,
        default=Path("artifacts/flight_covariance_measurement/samples.jsonl"),
    )
    parser.add_argument(
        "--json",
        type=Path,
        default=Path(
            "artifacts/cbf_uncertainty_sigma_candidate_selection/selection.json"
        ),
    )
    arguments = parser.parse_args()
    report = run_selection(arguments.sweep, arguments.trace)
    print_report(report)
    arguments.json.parent.mkdir(parents=True, exist_ok=True)
    arguments.json.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(f"wrote {arguments.json}")


if __name__ == "__main__":
    main()
