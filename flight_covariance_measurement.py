#!/usr/bin/env python3
"""Observe two-UAV covariance around the validated parallel SITL flight."""

from __future__ import annotations

import argparse
import json
import math
import subprocess
import sys
import time
import urllib.request
from pathlib import Path
from typing import Any

API_URL = "http://127.0.0.1:8000/api/drones"
DRONES = ("UAV-01", "UAV-02")
PHASES = ("GROUND", "TAKEOFF/CLIMB", "HOVER", "TRAJECTORY", "LANDING")
SIGMAS = (0.05, 0.10, 0.15, 0.20, 0.25, 0.30, 0.40)


def percentile(values: list[float], fraction: float) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    position = (len(ordered) - 1) * fraction
    lower = int(math.floor(position))
    upper = int(math.ceil(position))
    if lower == upper:
        return ordered[lower]
    return ordered[lower] + (ordered[upper] - ordered[lower]) * (position - lower)


def distribution(values: list[float]) -> dict[str, float | int | None]:
    return {
        "samples": len(values),
        "p50": percentile(values, 0.50),
        "p90": percentile(values, 0.90),
        "p95": percentile(values, 0.95),
        "p99": percentile(values, 0.99),
        "max": max(values) if values else None,
    }


def finite_covariance(value: Any) -> list[float] | None:
    if not isinstance(value, (list, tuple)) or len(value) != 3:
        return None
    try:
        covariance = [float(component) for component in value]
    except (TypeError, ValueError):
        return None
    if not all(math.isfinite(component) and component >= 0.0 for component in covariance):
        return None
    return covariance


def uncertainty_configuration_matches(
    state: dict[str, Any], expected_sigma: float, expected_require_covariance: bool
) -> bool:
    sigma = state.get("cbf_covariance_sigma")
    return (
        isinstance(sigma, (int, float))
        and math.isclose(float(sigma), expected_sigma, rel_tol=0.0, abs_tol=1.0e-9)
        and state.get("cbf_require_position_covariance")
        is expected_require_covariance
    )


def fetch() -> dict[str, Any]:
    with urllib.request.urlopen(API_URL, timeout=2.0) as response:
        return json.load(response)


def phase_for(states: dict[str, dict[str, Any]], trajectory_seen: bool) -> tuple[str, bool]:
    if states and all(state.get("armed") is False for state in states.values()):
        return "GROUND", trajectory_seen
    if any(state.get("station_keeping") is True for state in states.values()):
        return "TRAJECTORY", True
    if trajectory_seen:
        return "LANDING", True
    hovering = all(
        state.get("altitude_m") is not None
        and float(state["altitude_m"]) > 7.0
        and abs(float(state.get("vertical_velocity_m_s") or 0.0)) < 0.3
        for state in states.values()
    )
    return ("HOVER" if hovering else "TAKEOFF/CLIMB"), trajectory_seen


def parse_snapshot(payload: dict[str, Any], trajectory_seen: bool) -> tuple[dict[str, Any], bool]:
    states: dict[str, dict[str, Any]] = {}
    for drone_id in DRONES:
        drone = payload["drones"][drone_id]
        safety = payload["tracking_pose_streams"][drone_id].get("companion_safety") or {}
        local = drone.get("local_position") or {}
        cbf = safety.get("cbf") or {}
        covariance_map = safety.get("position_covariance_m2_by_drone") or {}
        own_covariance = finite_covariance(covariance_map.get(drone_id))
        states[drone_id] = {
            "armed": drone["status"].get("armed"),
            "nav_state": drone["status"].get("nav_state"),
            "altitude_m": None if local.get("z_down_m") is None else -float(local["z_down_m"]),
            "vertical_velocity_m_s": None if local.get("vz_m_s") is None else -float(local["vz_m_s"]),
            "position_covariance_enu_m2": own_covariance,
            "position_covariance_map_complete": all(
                finite_covariance(covariance_map.get(candidate)) is not None
                for candidate in DRONES
            ),
            "covariance_age_ms": safety.get("position_covariance_age_ms"),
            "odometry_reset_counter": safety.get("odometry_reset_counter"),
            "odometry_sample_count": safety.get("odometry_sample_count"),
            "odometry_covariance_sample_count": safety.get(
                "odometry_covariance_sample_count"
            ),
            "peer_age_ms": (safety.get("peer_message_age_ms_by_drone") or {}).get(
                DRONES[1] if drone_id == DRONES[0] else DRONES[0]
            ),
            "own_state_age_ms": safety.get("self_message_age_ms"),
            "velocity_enu_m_s": safety.get("own_velocity_enu_m_s"),
            "cbf_margin_m": cbf.get("minimum_margin_m"),
            "cbf_active": cbf.get("active"),
            "cbf_reason": cbf.get("reason"),
            "cbf_intervened": safety.get("intervened"),
            "supervisor_stage": (safety.get("emergency") or {}).get("stage"),
            "station_keeping": safety.get("station_keeping"),
            "trajectory_state": safety.get("nominal_reason"),
            "watchdog_conditions": safety.get("active_offboard_conditions") or [],
            "sender_latched_abort": (safety.get("active_offboard_sender") or {}).get(
                "latched_abort"
            ),
            "cbf_covariance_sigma": safety.get("cbf_covariance_sigma"),
            "cbf_require_position_covariance": safety.get(
                "cbf_require_position_covariance"
            ),
        }
    phase, trajectory_seen = phase_for(states, trajectory_seen)
    return {
        "wall_clock_s": time.time(),
        "monotonic_s": time.monotonic(),
        "phase": phase,
        "drones": states,
    }, trajectory_seen


def prearm_ok(
    rows: list[dict[str, Any]],
    maximum_age_ms: float = 100.0,
    *,
    expected_sigma: float = 0.0,
    expected_require_covariance: bool = False,
) -> tuple[bool, list[str]]:
    failures: list[str] = []
    if not rows:
        return False, ["no_prearm_samples"]
    for drone_id in DRONES:
        states = [row["drones"][drone_id] for row in rows]
        latest = states[-1]
        covariance = latest["position_covariance_enu_m2"]
        age = latest["covariance_age_ms"]
        if latest["armed"] is not False:
            failures.append(f"{drone_id}:not_disarmed")
        if covariance is None:
            failures.append(f"{drone_id}:covariance_missing_or_invalid")
        if latest["position_covariance_map_complete"] is not True:
            failures.append(f"{drone_id}:covariance_map_incomplete")
        if not isinstance(age, (int, float)) or not 0.0 <= float(age) <= maximum_age_ms:
            failures.append(f"{drone_id}:covariance_stale:{age}")
        if not isinstance(latest["odometry_reset_counter"], int):
            failures.append(f"{drone_id}:reset_counter_unobservable")
        if not uncertainty_configuration_matches(
            latest, expected_sigma, expected_require_covariance
        ):
            failures.append(
                f"{drone_id}:uncertainty_configuration_mismatch:"
                f"sigma={latest['cbf_covariance_sigma']}:"
                f"require={latest['cbf_require_position_covariance']}"
            )
    return not failures, failures


def reset_analysis(rows: list[dict[str, Any]], drone_id: str) -> dict[str, Any]:
    events: list[dict[str, Any]] = []
    previous: dict[str, Any] | None = None
    for row in rows:
        state = row["drones"][drone_id]
        counter = state["odometry_reset_counter"]
        if (
            previous is not None
            and row["phase"] != "GROUND"
            and isinstance(counter, int)
            and isinstance(previous["counter"], int)
            and counter != previous["counter"]
        ):
            raw_delta = int(state["odometry_sample_count"] or 0) - int(
                previous["sample_count"] or 0
            )
            valid_delta = int(state["odometry_covariance_sample_count"] or 0) - int(
                previous["valid_count"] or 0
            )
            events.append(
                {
                    "monotonic_s": row["monotonic_s"],
                    "phase": row["phase"],
                    "old_counter": previous["counter"],
                    "new_counter": counter,
                    "invalid_odometry_samples_in_interval": max(0, raw_delta - valid_delta),
                    "covariance_missing_at_observation": state[
                        "position_covariance_enu_m2"
                    ]
                    is None,
                    "recovery_time_upper_bound_s": row["monotonic_s"]
                    - previous["monotonic_s"],
                }
            )
        previous = {
            "counter": counter,
            "sample_count": state["odometry_sample_count"],
            "valid_count": state["odometry_covariance_sample_count"],
            "monotonic_s": row["monotonic_s"],
        }
    return {
        "verdict": (
            "IN_FLIGHT_ODOMETRY_RESET_OBSERVED"
            if events
            else "NO_IN_FLIGHT_ODOMETRY_RESET_OBSERVED"
        ),
        "events": events,
    }


def analyze(
    rows: list[dict[str, Any]],
    flight_result: dict[str, Any],
    *,
    expected_sigma: float = 0.0,
    expected_require_covariance: bool = False,
    require_clean_safety_events: bool = False,
) -> dict[str, Any]:
    per_drone: dict[str, Any] = {}
    for drone_id in DRONES:
        valid_states = [
            row["drones"][drone_id]
            for row in rows
            if row["drones"][drone_id]["position_covariance_enu_m2"] is not None
        ]
        sqrt_traces = [
            math.sqrt(sum(state["position_covariance_enu_m2"]))
            for state in valid_states
        ]
        ages = [
            float(state["covariance_age_ms"])
            for state in valid_states
            if isinstance(state["covariance_age_ms"], (int, float))
        ]
        axes = tuple(zip(*(state["position_covariance_enu_m2"] for state in valid_states)))
        per_drone[drone_id] = {
            "valid_covariance_samples": len(valid_states),
            "missing_or_invalid_samples": len(rows) - len(valid_states),
            "uncertainty_configuration_mismatch_samples": sum(
                not uncertainty_configuration_matches(
                    row["drones"][drone_id],
                    expected_sigma,
                    expected_require_covariance,
                )
                for row in rows
            ),
            "covariance_age_ms": distribution(ages),
            "sqrt_trace_cov_m": distribution(sqrt_traces),
            "per_axis_variance_range_m2": {
                name: {"min": min(values), "max": max(values)}
                for name, values in zip(("var_e", "var_n", "var_u"), axes)
            }
            if axes
            else {},
            "reset_counter": reset_analysis(rows, drone_id),
        }

    pair_rows: list[tuple[str, float]] = []
    for row in rows:
        covariance = [
            row["drones"][drone_id]["position_covariance_enu_m2"]
            for drone_id in DRONES
        ]
        if all(value is not None for value in covariance):
            pair_rows.append(
                (row["phase"], math.sqrt(sum(sum(value) for value in covariance)))
            )
    u_base = [value for _, value in pair_rows]
    overall = distribution(u_base)
    phase_statistics = {
        phase: distribution([value for label, value in pair_rows if label == phase])
        for phase in PHASES
    }
    sigma_table = [
        {
            "sigma": sigma,
            "margin_at_p95_m": sigma * float(overall["p95"]),
            "margin_at_p99_m": sigma * float(overall["p99"]),
            "margin_at_max_m": sigma * float(overall["max"]),
        }
        for sigma in SIGMAS
        if overall["p95"] is not None
    ]
    events = {
        "negative_cbf_margin_samples": sum(
            1
            for row in rows
            for state in row["drones"].values()
            if isinstance(state["cbf_margin_m"], (int, float))
            and state["cbf_margin_m"] < 0.0
        ),
        "cbf_infeasible_samples": sum(
            1
            for row in rows
            for state in row["drones"].values()
            if state["cbf_reason"] == "cbf_constraints_infeasible"
        ),
        "cbf_intervention_samples": sum(
            1
            for row in rows
            for state in row["drones"].values()
            if state["cbf_intervened"] is True
        ),
        "supervisor_stages": sorted(
            {
                str(state["supervisor_stage"])
                for row in rows
                for state in row["drones"].values()
                if state["supervisor_stage"] is not None
            }
        ),
        "watchdog_conditions": sorted(
            {
                str(condition)
                for row in rows
                for state in row["drones"].values()
                for condition in state["watchdog_conditions"]
            }
        ),
        "sender_latched_aborts": sorted(
            {
                str(state["sender_latched_abort"])
                for row in rows
                for state in row["drones"].values()
                if state["sender_latched_abort"]
            }
        ),
    }
    enough_phase_data = all(phase_statistics[phase]["samples"] for phase in PHASES)
    configuration_valid = all(
        per_drone[drone]["uncertainty_configuration_mismatch_samples"] == 0
        for drone in DRONES
    )
    safety_events_clean = bool(
        events["negative_cbf_margin_samples"] == 0
        and events["cbf_infeasible_samples"] == 0
        and set(events["supervisor_stages"]) <= {"normal"}
        and not events["watchdog_conditions"]
        and not events["sender_latched_aborts"]
    )
    return {
        "measurement_data_valid": bool(
            flight_result.get("verdict") == "FLIGHT_PASS"
            and u_base
            and enough_phase_data
            and all(per_drone[drone]["valid_covariance_samples"] for drone in DRONES)
            and configuration_valid
            and (safety_events_clean or not require_clean_safety_events)
        ),
        "expected_uncertainty_configuration": {
            "covariance_sigma": expected_sigma,
            "require_position_covariance": expected_require_covariance,
        },
        "require_clean_safety_events": require_clean_safety_events,
        "safety_events_clean": safety_events_clean,
        "flight_verdict": flight_result.get("verdict"),
        "raw_joint_samples": len(rows),
        "per_drone": per_drone,
        "u_base_m": overall,
        "phase_statistics_u_base_m": phase_statistics,
        "sigma_analysis_only": sigma_table,
        "events": events,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--hold-s", type=float, default=25.0)
    parser.add_argument("--sample-hz", type=float, default=10.0)
    parser.add_argument("--prearm-s", type=float, default=5.0)
    parser.add_argument("--expected-sigma", type=float, default=0.0)
    parser.add_argument("--expected-require-covariance", action="store_true")
    parser.add_argument("--require-clean-safety-events", action="store_true")
    parser.add_argument(
        "--output-dir", type=Path, default=Path("artifacts/flight_covariance_measurement")
    )
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    raw_path = args.output_dir / "samples.jsonl"
    flight_path = args.output_dir / "flight.json"
    flight_log_path = args.output_dir / "flight.log"
    summary_path = args.output_dir / "summary.json"
    raw_path.write_text("", encoding="utf-8")
    rows: list[dict[str, Any]] = []
    trajectory_seen = False
    period_s = 1.0 / max(1.0, args.sample_hz)

    deadline = time.monotonic() + args.prearm_s
    while time.monotonic() < deadline:
        row, trajectory_seen = parse_snapshot(fetch(), trajectory_seen)
        rows.append(row)
        with raw_path.open("a", encoding="utf-8") as handle:
            handle.write(json.dumps(row, separators=(",", ":")) + "\n")
        time.sleep(period_s)
    valid, failures = prearm_ok(
        rows,
        expected_sigma=args.expected_sigma,
        expected_require_covariance=args.expected_require_covariance,
    )
    if not valid:
        summary_path.write_text(
            json.dumps({"measurement_data_valid": False, "prearm_failures": failures}, indent=2)
            + "\n",
            encoding="utf-8",
        )
        print(json.dumps({"prearm": "FAIL", "failures": failures}, indent=2))
        return 1
    print(json.dumps({"prearm": "PASS", "samples": len(rows)}, indent=2), flush=True)

    command = [
        sys.executable,
        "two_uav_trajectory_flight.py",
        "--hold-s",
        str(args.hold_s),
        "--output",
        str(flight_path),
    ]
    with flight_log_path.open("w", encoding="utf-8") as flight_log:
        process = subprocess.Popen(command, stdout=flight_log, stderr=subprocess.STDOUT)
        while process.poll() is None:
            try:
                row, trajectory_seen = parse_snapshot(fetch(), trajectory_seen)
                rows.append(row)
                with raw_path.open("a", encoding="utf-8") as handle:
                    handle.write(json.dumps(row, separators=(",", ":")) + "\n")
            except Exception as error:  # collection failure is data, never zero covariance
                with raw_path.open("a", encoding="utf-8") as handle:
                    handle.write(
                        json.dumps(
                            {
                                "wall_clock_s": time.time(),
                                "monotonic_s": time.monotonic(),
                                "error": str(error),
                            },
                            separators=(",", ":"),
                        )
                        + "\n"
                    )
            time.sleep(period_s)
        returncode = process.wait()

    for _ in range(max(1, int(3.0 / period_s))):
        try:
            row, trajectory_seen = parse_snapshot(fetch(), trajectory_seen)
            rows.append(row)
            with raw_path.open("a", encoding="utf-8") as handle:
                handle.write(json.dumps(row, separators=(",", ":")) + "\n")
        except Exception:
            pass
        time.sleep(period_s)
    flight_result = json.loads(flight_path.read_text(encoding="utf-8"))
    summary = analyze(
        rows,
        flight_result,
        expected_sigma=args.expected_sigma,
        expected_require_covariance=args.expected_require_covariance,
        require_clean_safety_events=args.require_clean_safety_events,
    )
    summary["flight_process_returncode"] = returncode
    summary["prearm_validation"] = "PASS"
    summary_path.write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(summary, indent=2))
    return 0 if summary["measurement_data_valid"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
