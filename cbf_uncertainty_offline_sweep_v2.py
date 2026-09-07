#!/usr/bin/env python3
"""Replay measured SITL-flight covariance through the existing CBF harness.

Offline only: no stack, network, arming, environment edit, or production
configuration change.  Geometry, controllers, CBF, and supervisor come from
``cbf_uncertainty_sigma_sweep``; only covariance_sigma, peer age inside the
100 ms contract, and the measured covariance input profile vary.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import statistics
from dataclasses import replace
from pathlib import Path
from typing import Any

from cbf_uncertainty_sigma_sweep import (
    ENV,
    UAV_01,
    UAV_02,
    _expected_completers,
    _feasible,
    scenarios,
    simulate,
)
from flight_covariance_measurement import distribution, finite_covariance, reset_analysis


DRONES = (UAV_01, UAV_02)
SCENARIOS = ("parallel_trajectory_baseline", "crossing_validated")
PEER_AGES_MS = (0.0, 50.0, 100.0)
SIGMAS = (0.0, 0.05, 0.10, 0.15, 0.20, 0.25, 0.30, 0.40)
EDGE_SCAN_STEP = 0.05
EDGE_SCAN_MAX = 2.0


def load_trace(path: Path) -> dict[str, Any]:
    rows: list[dict[str, Any]] = []
    frames: list[dict[str, tuple[float, float, float]]] = []
    u_base: list[float] = []
    for line_number, line in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
        try:
            row = json.loads(line)
            frame: dict[str, tuple[float, float, float]] = {}
            for drone in DRONES:
                covariance = finite_covariance(
                    row["drones"][drone]["position_covariance_enu_m2"]
                )
                if covariance is None:
                    raise ValueError(f"{drone}:invalid_covariance")
                frame[drone] = tuple(covariance)  # type: ignore[assignment]
        except (KeyError, TypeError, ValueError, json.JSONDecodeError) as error:
            raise ValueError(f"invalid covariance trace line {line_number}: {error}") from error
        rows.append(row)
        frames.append(frame)
        u_base.append(math.sqrt(sum(sum(frame[drone]) for drone in DRONES)))

    if len(rows) < 2:
        raise ValueError("covariance trace needs at least two samples")
    deltas = [
        float(rows[index]["monotonic_s"]) - float(rows[index - 1]["monotonic_s"])
        for index in range(1, len(rows))
    ]
    if not all(math.isfinite(delta) and delta > 0.0 for delta in deltas):
        raise ValueError("covariance trace time is not strictly increasing")
    return {
        "rows": rows,
        "frames": tuple(frames),
        "sample_period_s": statistics.median(deltas),
        "u_base": u_base,
        "u_base_distribution_m": distribution(u_base),
        "phase_counts": {
            phase: sum(row.get("phase") == phase for row in rows)
            for phase in sorted({str(row.get("phase")) for row in rows})
        },
        "reset_counter": {drone: reset_analysis(rows, drone) for drone in DRONES},
    }


def covariance_profiles(trace: dict[str, Any]) -> dict[str, dict[str, Any]]:
    frames = trace["frames"]
    u_base = trace["u_base"]
    ordered = sorted(range(len(frames)), key=u_base.__getitem__)
    profiles = {
        "measured_trace": {
            "frames": frames,
            "u_base_m": trace["u_base_distribution_m"],
        }
    }
    for name, fraction in (("p95_constant", 0.95), ("p99_constant", 0.99), ("max_constant", 1.0)):
        index = ordered[math.ceil((len(ordered) - 1) * fraction)]
        profiles[name] = {
            "frames": (frames[index],),
            "source_sample_index": index,
            "u_base_m": u_base[index],
        }
    return profiles


def run_case(
    scenario: Any,
    sigma: float,
    profile_name: str,
    profile: dict[str, Any],
    sample_period_s: float,
) -> dict[str, Any]:
    row = simulate(
        scenario,
        sigma,
        covariance_frames=profile["frames"],
        covariance_sample_period_s=sample_period_s,
    ).as_dict()
    hard, strict = _feasible(row, _expected_completers(scenario))
    return {
        "peer_age_ms": scenario.peer_age_ms,
        "covariance_profile": profile_name,
        "hard_feasible": hard,
        "strict_feasible": strict,
        **row,
    }


def summarize_sigma(rows: list[dict[str, Any]], sigma: float) -> dict[str, Any]:
    selected = [row for row in rows if row["covariance_sigma"] == sigma]
    infeasible_times = [
        row["first_infeasible_s"]
        for row in selected
        if row["first_infeasible_s"] is not None
    ]
    return {
        "sigma": sigma,
        "all_hard_feasible": all(row["hard_feasible"] for row in selected),
        "all_strict_feasible": all(row["strict_feasible"] for row in selected),
        "minimum_reported_margin_m": min(row["min_margin_reported_m"] for row in selected),
        "minimum_distance_m": min(row["min_distance_m"] for row in selected),
        "maximum_intervention_rate": max(row["intervention_rate"] for row in selected),
        "maximum_correction_m_s": max(row["max_correction_m_s"] for row in selected),
        "total_infeasible_frames": sum(row["infeasible_frames"] for row in selected),
        "earliest_infeasible_s": min(infeasible_times) if infeasible_times else None,
        "failing_cases": [
            {
                "scenario": row["scenario"],
                "peer_age_ms": row["peer_age_ms"],
                "covariance_profile": row["covariance_profile"],
            }
            for row in selected
            if not row["strict_feasible"]
        ],
    }


def characterize_edge(rows: list[dict[str, Any]]) -> dict[str, Any]:
    hard_edge = strict_edge = None
    first_hard_failure = first_strict_failure = None
    feasible_islands: list[float] = []
    for row in rows:
        sigma = row["covariance_sigma"]
        if first_hard_failure is None:
            if row["hard_feasible"]:
                hard_edge = sigma
            else:
                first_hard_failure = sigma
        elif row["hard_feasible"]:
            feasible_islands.append(sigma)
        if first_strict_failure is None:
            if row["strict_feasible"]:
                strict_edge = sigma
            else:
                first_strict_failure = sigma
    return {
        "hard_feasible_up_to_sigma": hard_edge,
        "strict_feasible_up_to_sigma": strict_edge,
        "first_hard_failure_sigma": first_hard_failure,
        "first_strict_failure_sigma": first_strict_failure,
        "feasible_islands_above_first_hard_failure": feasible_islands,
        "runs": rows,
    }


def run_sweep(trace_path: Path) -> dict[str, Any]:
    trace = load_trace(trace_path)
    profiles = covariance_profiles(trace)
    selected_scenarios = {
        scenario.name: scenario for scenario in scenarios() if scenario.name in SCENARIOS
    }
    if set(selected_scenarios) != set(SCENARIOS):
        raise ValueError("validated baseline or crossing scenario is missing")

    rows: list[dict[str, Any]] = []
    sigma_zero_reproduces_baseline = True
    for scenario_name in SCENARIOS:
        for peer_age_ms in PEER_AGES_MS:
            scenario = replace(selected_scenarios[scenario_name], peer_age_ms=peer_age_ms)
            baseline = simulate(scenario, 0.0, with_covariance=False).as_dict()
            for profile_name, profile in profiles.items():
                for sigma in SIGMAS:
                    row = run_case(
                        scenario,
                        sigma,
                        profile_name,
                        profile,
                        trace["sample_period_s"],
                    )
                    rows.append(row)
                    if sigma == 0.0:
                        comparable = {
                            key: value
                            for key, value in row.items()
                            if key not in {"peer_age_ms", "covariance_profile", "hard_feasible", "strict_feasible"}
                        }
                        sigma_zero_reproduces_baseline &= comparable == baseline

    by_sigma = [summarize_sigma(rows, sigma) for sigma in SIGMAS]
    offline_candidates = [
        item["sigma"]
        for item in by_sigma
        if item["sigma"] > 0.0 and item["all_strict_feasible"]
    ]

    binding_scenario = replace(selected_scenarios["crossing_validated"], peer_age_ms=100.0)
    binding_profile = profiles["max_constant"]
    existing = {
        row["covariance_sigma"]: row
        for row in rows
        if row["scenario"] == "crossing_validated"
        and row["peer_age_ms"] == 100.0
        and row["covariance_profile"] == "max_constant"
    }
    edge_rows = []
    for index in range(int(round(EDGE_SCAN_MAX / EDGE_SCAN_STEP)) + 1):
        sigma = round(index * EDGE_SCAN_STEP, 2)
        edge_rows.append(
            existing.get(sigma)
            or run_case(
                binding_scenario,
                sigma,
                "max_constant",
                binding_profile,
                trace["sample_period_s"],
            )
        )

    reset_counter = trace["reset_counter"]
    reset_covariance_continuous = all(
        event["covariance_missing_at_observation"] is False
        for drone in DRONES
        for event in reset_counter[drone]["events"]
    )
    validated = sigma_zero_reproduces_baseline
    return {
        "milestone": "CBF_UNCERTAINTY_OFFLINE_SWEEP_V2",
        "verdict": (
            "CBF_UNCERTAINTY_OFFLINE_SWEEP_V2_VALIDATED"
            if validated
            else "CBF_UNCERTAINTY_OFFLINE_SWEEP_V2_NOT_VALIDATED"
        ),
        "source": {
            "path": str(trace_path),
            "sha256": hashlib.sha256(trace_path.read_bytes()).hexdigest(),
            "joint_samples": len(trace["frames"]),
            "sample_period_median_s": trace["sample_period_s"],
            "u_base_distribution_m": trace["u_base_distribution_m"],
            "phase_counts": trace["phase_counts"],
        },
        "covariance_profiles": {
            name: {key: value for key, value in profile.items() if key != "frames"}
            for name, profile in profiles.items()
        },
        "reset_counter": reset_counter,
        "reset_replay": {
            "covariance_continuous_at_collector_cadence": reset_covariance_continuous,
            "synthetic_missing_covariance_injected": False,
            "reason": (
                "reset_counter is not a CBF input and covariance was present at every "
                "10 Hz observation; inventing a missing interval would exceed the measurement."
            ),
        },
        "held_fixed": {
            "minimum_separation_m": float(ENV.get("SWARM_CBF_MINIMUM_SEPARATION_M", 4.0)),
            "command_latency_s": 0.65,
            "peer_age_contract_ms": float(ENV.get("SWARM_PEER_STATE_MAX_AGE_MS", 100.0)),
            "peer_ages_swept_ms": list(PEER_AGES_MS),
            "scenarios": list(SCENARIOS),
            "production_runtime_sigma": float(ENV.get("SWARM_CBF_COVARIANCE_SIGMA", 0.0)),
            "production_require_position_covariance": ENV.get(
                "SWARM_CBF_REQUIRE_POSITION_COVARIANCE", "false"
            ),
        },
        "sigma_zero_reproduces_feature_off_baseline": sigma_zero_reproduces_baseline,
        "candidate_grid_summary": by_sigma,
        "offline_strict_candidate_sigmas": offline_candidates,
        "candidate_range_is_not_a_production_selection": True,
        "binding_edge_scan": {
            "scenario": "crossing_validated",
            "peer_age_ms": 100.0,
            "covariance_profile": "max_constant",
            "scan_step": EDGE_SCAN_STEP,
            **characterize_edge(edge_rows),
        },
        "runs": rows,
    }


def print_report(report: dict[str, Any]) -> None:
    source = report["source"]
    print(report["verdict"])
    print(
        f"trace: {source['joint_samples']} joint samples, "
        f"U_base P95/P99/MAX={source['u_base_distribution_m']['p95']:.4f}/"
        f"{source['u_base_distribution_m']['p99']:.4f}/"
        f"{source['u_base_distribution_m']['max']:.4f} m"
    )
    print("sigma  hard  strict  min_margin  min_distance  intervention  infeasible")
    for row in report["candidate_grid_summary"]:
        print(
            f"{row['sigma']:>4.2f}  {str(row['all_hard_feasible']):>5}  "
            f"{str(row['all_strict_feasible']):>6}  "
            f"{row['minimum_reported_margin_m']:>10.3f}  "
            f"{row['minimum_distance_m']:>12.3f}  "
            f"{row['maximum_intervention_rate']:>12.4f}  "
            f"{row['total_infeasible_frames']:>10}"
        )
    edge = report["binding_edge_scan"]
    print(
        "binding edge (crossing, peer age 100 ms, measured MAX constant): "
        f"hard<={edge['hard_feasible_up_to_sigma']}, "
        f"strict<={edge['strict_feasible_up_to_sigma']}, "
        f"first hard failure={edge['first_hard_failure_sigma']}"
    )
    print(f"offline strict candidates: {report['offline_strict_candidate_sigmas']}")
    print("production sigma/require unchanged")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--trace",
        type=Path,
        default=Path("artifacts/flight_covariance_measurement/samples.jsonl"),
    )
    parser.add_argument(
        "--json",
        type=Path,
        default=Path("artifacts/cbf_uncertainty_sweep_v2/sweep.json"),
    )
    arguments = parser.parse_args()
    report = run_sweep(arguments.trace)
    print_report(report)
    arguments.json.parent.mkdir(parents=True, exist_ok=True)
    arguments.json.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(f"wrote {arguments.json}")


if __name__ == "__main__":
    main()
