#!/usr/bin/env python3
"""Evaluate a CBF-RL shadow or active trace against a completed SITL flight."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Any, Sequence


DRONES = ("UAV-01", "UAV-02")
TRAJECTORY_REASONS = {"tracking_trajectory", "trajectory_reached"}


def _percentile(values: Sequence[float], fraction: float) -> float:
    ordered = sorted(values)
    position = (len(ordered) - 1) * fraction
    lower = int(math.floor(position))
    upper = int(math.ceil(position))
    if lower == upper:
        return ordered[lower]
    return ordered[lower] + (ordered[upper] - ordered[lower]) * (position - lower)


def _distribution(values: Sequence[float]) -> dict[str, float]:
    return {
        "p50": _percentile(values, 0.50),
        "p95": _percentile(values, 0.95),
        "max": max(values),
    }


def evaluate(
    rows: Sequence[dict[str, Any]],
    flight: dict[str, Any],
    expected_model_sha256: str,
    *,
    minimum_trajectory_samples: int = 100,
    expected_mode: str = "shadow",
) -> dict[str, Any]:
    if expected_mode not in {"shadow", "active"}:
        raise ValueError("expected_mode must be shadow or active")
    trajectory_flight = bool(
        flight.get("expected_trajectory_env") or flight.get("circle")
    )
    final_state = flight.get("final_state", {})
    if not final_state:
        final_safe = next(
            (
                record
                for record in reversed(flight.get("records", ()))
                if record.get("event") == "FINAL_SAFE_STATE"
            ),
            {},
        )
        armed = final_safe.get("armed") or {}
        final_state = {
            drone: {"armed": armed.get(drone)} for drone in DRONES
        }
    trajectory_summaries = {
        record.get("drone_id"): record
        for record in flight.get("records", ())
        if record.get("step") == "TRAJECTORY_HOLD_COMPLETE"
    }
    per_drone: dict[str, Any] = {}
    checks: dict[str, bool] = {
        "flight_pass": flight.get("verdict") == "FLIGHT_PASS",
        "landed_and_disarmed": all(
            (final_state.get(drone) or {}).get("armed") is False
            for drone in DRONES
        ),
    }
    for drone in DRONES:
        trajectory = [
            row
            for row in rows
            if row.get("drone_id") == drone
            and row.get("station_keeping") is True
            and (
                (expected_mode == "active" and not trajectory_flight)
                or row.get("nominal_reason") in TRAJECTORY_REASONS
            )
        ]
        shadows = [row.get("cbf_rl_shadow") or {} for row in trajectory]
        valid = [shadow for shadow in shadows if shadow.get("valid") is True]
        nominal_deltas = [float(shadow["nominal_delta_norm_m_s"]) for shadow in valid]
        shielded_deltas = [
            float(shadow["shielded_delta_norm_m_s"]) for shadow in valid
        ]
        shadow_margins = [
            float(shadow["cbf"]["minimum_margin_m"])
            for shadow in valid
            if isinstance((shadow.get("cbf") or {}).get("minimum_margin_m"), (int, float))
        ]
        actual_margins = [
            float(row["cbf"]["minimum_margin_m"])
            for row in trajectory
            if isinstance((row.get("cbf") or {}).get("minimum_margin_m"), (int, float))
        ]
        action_values = [
            float(value)
            for shadow in valid
            for value in shadow.get("normalized_action", ())
        ]
        policy_cbf_clean = bool(valid) and all(
            (shadow.get("cbf") or {}).get("active") is True
            and (shadow.get("cbf") or {}).get("reason")
            != "cbf_constraints_infeasible"
            for shadow in valid
        ) and bool(shadow_margins) and min(shadow_margins) >= 0.0
        drone_checks = {
            "enough_samples": len(trajectory) >= minimum_trajectory_samples,
            "every_sample_valid": len(valid) == len(trajectory) and bool(valid),
            "runtime_mode": bool(valid)
            and all(shadow.get("mode") == expected_mode for shadow in valid),
            "model_authenticated": bool(valid)
            and all(shadow.get("model_sha256") == expected_model_sha256 for shadow in valid),
            "actions_finite_and_bounded": len(action_values) == len(valid) * 3
            and all(math.isfinite(value) and abs(value) <= 1.0 for value in action_values),
            f"{expected_mode}_cbf_clean": policy_cbf_clean,
            "actual_safety_clean": bool(trajectory)
            and all(
                (row.get("cbf") or {}).get("reason") != "cbf_constraints_infeasible"
                and (row.get("emergency") or {}).get("stage") == "normal"
                and not row.get("active_offboard_conditions")
                and not (row.get("active_offboard_sender") or {}).get("latched_abort")
                for row in trajectory
            )
            and bool(actual_margins)
            and min(actual_margins) >= 0.0,
        }
        if expected_mode == "shadow":
            drone_checks["never_applied_or_authorized"] = bool(valid) and all(
                shadow.get("applied") is False
                and shadow.get("transmit_authority") is False
                for shadow in valid
            )
        else:
            drone_checks.update(
                {
                    "every_sample_applied_and_authorized": bool(valid)
                    and all(
                        shadow.get("applied") is True
                        and shadow.get("transmit_authority") is True
                        for shadow in valid
                    ),
                    "routed_through_companion_safety": bool(trajectory)
                    and all(
                        row.get("authority") == "shadow_companion_only"
                        and row.get("nominal_source") == "cbf_rl_active"
                        and row.get("output_velocity_enu_m_s")
                        == (row.get("cbf_rl_shadow") or {}).get(
                            "shielded_velocity_enu_m_s"
                        )
                        for row in trajectory
                    ),
                    "every_frame_transmitted": bool(trajectory)
                    and all(
                        (row.get("active_offboard_frame") or {}).get("decision")
                        == "transmit"
                        and (row.get("active_offboard_frame") or {}).get(
                            "transmitted"
                        )
                        is True
                        for row in trajectory
                    ),
                }
            )
        if flight.get("expected_trajectory_env"):
            drone_checks["trajectory_completed"] = (
                trajectory_summaries.get(drone, {}).get("reached_trajectory_end")
                is True
            )
        checks.update({f"{drone}:{name}": value for name, value in drone_checks.items()})
        per_drone[drone] = {
            "trajectory_samples": len(trajectory),
            f"valid_{expected_mode}_samples": len(valid),
            "checks": drone_checks,
            "maximum_absolute_action": max((abs(value) for value in action_values), default=None),
            f"minimum_{expected_mode}_cbf_margin_m": min(
                shadow_margins, default=None
            ),
            "minimum_actual_cbf_margin_m": min(actual_margins, default=None),
            "nominal_delta_norm_m_s": (
                _distribution(nominal_deltas) if nominal_deltas else None
            ),
            "shielded_delta_norm_m_s": (
                _distribution(shielded_deltas) if shielded_deltas else None
            ),
        }
    passed = all(checks.values())
    return {
        "milestone": (
            "CBF_RL_ACTIVE_TRAJECTORY"
            if expected_mode == "active" and trajectory_flight
            else "CBF_RL_PRODUCTION_ENABLEMENT"
            if expected_mode == "active"
            else "CBF_RL_SITL_SHADOW"
        ),
        "mode": expected_mode,
        "verdict": "PASS" if passed else "FAIL",
        "expected_model_sha256": expected_model_sha256,
        "checks": checks,
        "per_drone": per_drone,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--trace", required=True, type=Path)
    parser.add_argument("--flight", required=True, type=Path)
    parser.add_argument("--model-sha256", required=True)
    parser.add_argument("--minimum-trajectory-samples", type=int, default=100)
    parser.add_argument("--mode", choices=("shadow", "active"), default="shadow")
    parser.add_argument("--output", type=Path)
    arguments = parser.parse_args()
    rows = [
        json.loads(line)
        for line in arguments.trace.read_text(encoding="utf-8").splitlines()
        if line.strip()
    ]
    report = evaluate(
        rows,
        json.loads(arguments.flight.read_text(encoding="utf-8")),
        arguments.model_sha256,
        minimum_trajectory_samples=arguments.minimum_trajectory_samples,
        expected_mode=arguments.mode,
    )
    encoded = json.dumps(report, indent=2, sort_keys=True) + "\n"
    if arguments.output:
        arguments.output.parent.mkdir(parents=True, exist_ok=True)
        arguments.output.write_text(encoded, encoding="utf-8")
    print(encoded, end="")
    return 0 if report["verdict"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
