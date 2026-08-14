#!/usr/bin/env python3
"""Read-only SITL runtime gate for formation and CBF shadow outputs.

It makes HTTP GET requests only.  It never launches SITL, sends MQTT, arms a
vehicle, changes flight mode, or routes CBF output to PX4.
"""

from __future__ import annotations

import argparse
from collections import Counter
from datetime import datetime, timezone
import json
from pathlib import Path
import time
from typing import Any
from urllib.request import urlopen


def fetch_json(url: str, timeout_s: float = 2.0) -> dict[str, Any]:
    with urlopen(url, timeout=timeout_s) as response:  # nosec B310: user-selected local SITL URL
        payload = json.loads(response.read().decode("utf-8"))
    if not isinstance(payload, dict):
        raise ValueError("API payload is not an object")
    return payload


def evaluate_snapshots(
    snapshots: list[dict[str, Any]],
    *,
    require_intervention: bool = False,
    required_hold_reasons: tuple[str, ...] = (),
) -> dict[str, Any]:
    violations: list[str] = []
    hold_reasons: Counter[str] = Counter()
    valid_swarm_samples = 0
    cbf_active_samples = 0
    interventions = 0
    for index, payload in enumerate(snapshots):
        prefix = f"sample[{index}]"
        if not payload.get("swarm_state_origin_configured", False):
            violations.append(f"{prefix}: common_enu_origin_unconfigured")
        formation = payload.get("formation")
        cbf = payload.get("cbf")
        if not isinstance(formation, dict) or not formation.get("enabled", False):
            violations.append(f"{prefix}: formation_shadow_disabled")
            continue
        if not isinstance(cbf, dict) or not cbf.get("enabled", False):
            violations.append(f"{prefix}: cbf_shadow_disabled")
            continue
        swarm_state = payload.get("swarm_state")
        if isinstance(swarm_state, dict) and swarm_state and all(
            isinstance(state, dict) and state.get("valid", False)
            for state in swarm_state.values()
        ):
            valid_swarm_samples += 1
        nominal_commands = formation.get("commands", {})
        filtered_commands = cbf.get("commands", {})
        if not isinstance(nominal_commands, dict) or not isinstance(filtered_commands, dict):
            violations.append(f"{prefix}: command_contract_missing")
            continue
        for drone_id, nominal in nominal_commands.items():
            if not isinstance(nominal, dict) or not nominal.get("active", False):
                continue
            filtered = filtered_commands.get(drone_id)
            if not isinstance(filtered, dict):
                violations.append(f"{prefix}: {drone_id}: cbf_command_missing")
                continue
            if filtered.get("active", False):
                velocity = filtered.get("velocity_enu_m_s")
                if not isinstance(velocity, list) or len(velocity) != 3:
                    violations.append(f"{prefix}: {drone_id}: cbf_velocity_invalid")
                    continue
                cbf_active_samples += 1
                try:
                    intervention = float(filtered.get("intervention_norm_m_s", 0.0))
                except (TypeError, ValueError):
                    violations.append(f"{prefix}: {drone_id}: intervention_invalid")
                    continue
                if intervention > 1e-6:
                    interventions += 1
            else:
                reason = filtered.get("reason")
                if not isinstance(reason, str) or not reason:
                    violations.append(f"{prefix}: {drone_id}: hold_reason_missing")
                else:
                    hold_reasons[reason] += 1
    if not snapshots:
        violations.append("no_api_samples")
    if valid_swarm_samples == 0:
        violations.append("no_all_valid_swarm_state_sample")
    if require_intervention and interventions == 0:
        violations.append("cbf_intervention_not_observed")
    for reason in required_hold_reasons:
        if hold_reasons[reason] == 0:
            violations.append(f"required_hold_not_observed:{reason}")
    return {
        "pass": not violations,
        "sample_count": len(snapshots),
        "valid_swarm_samples": valid_swarm_samples,
        "cbf_active_samples": cbf_active_samples,
        "cbf_interventions": interventions,
        "hold_reasons": dict(sorted(hold_reasons.items())),
        "violations": violations,
    }


def collect(
    api_url: str,
    duration_s: float,
    interval_s: float,
    **evaluation_kwargs: Any,
) -> dict[str, Any]:
    started = datetime.now(timezone.utc)
    snapshots = []
    errors = []
    deadline = time.monotonic() + max(1.0, duration_s)
    while time.monotonic() < deadline:
        try:
            snapshots.append(fetch_json(api_url))
        except (OSError, ValueError, json.JSONDecodeError) as error:
            errors.append(str(error))
        time.sleep(max(0.05, interval_s))
    gate = evaluate_snapshots(snapshots, **evaluation_kwargs)
    if errors:
        gate["pass"] = False
        gate["violations"].append("api_get_failed")
    return {
        "schema": "cbf_sitl_shadow_gate/v1",
        "started_utc": started.isoformat(),
        "finished_utc": datetime.now(timezone.utc).isoformat(),
        "requested_duration_s": duration_s,
        "api_url": api_url,
        "safety_contract": {
            "http_operations": ["GET /api/drones"],
            "mqtt_publish": False,
            "mavlink_send": False,
            "px4_mode_change": False,
            "px4_arm": False,
        },
        "api_error_count": len(errors),
        "api_errors": errors,
        "gate": gate,
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--api-url", default="http://127.0.0.1:8000/api/drones")
    parser.add_argument("--duration-s", type=float, default=30.0)
    parser.add_argument("--interval-s", type=float, default=0.2)
    parser.add_argument("--require-intervention", action="store_true")
    parser.add_argument("--require-hold-reason", action="append", default=[])
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    report = collect(
        args.api_url,
        args.duration_s,
        args.interval_s,
        require_intervention=args.require_intervention,
        required_hold_reasons=tuple(args.require_hold_reason),
    )
    rendered = json.dumps(report, indent=2, sort_keys=True)
    print(rendered)
    if args.output:
        args.output.write_text(rendered + "\n", encoding="utf-8")
    return 0 if report["gate"]["pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
