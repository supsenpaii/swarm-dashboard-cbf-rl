#!/usr/bin/env python3
"""Repeatable offline fault-injection checks for formation/CBF shadow logic."""

from __future__ import annotations

import argparse
import json
from typing import Any

from cbf_command_gate import CbfCommandGate, CbfConfig
from peer_state import PeerStateRegistry, make_peer_state


def _state(position, velocity=(0.0, 0.0, 0.0), *, valid=True, age_ms=0.0):
    return {
        "valid": valid,
        "position_enu_m": position,
        "velocity_enu_m_s": velocity,
        "position_covariance_m2": (0.0, 0.0, 0.0),
        "message_age_ms": age_ms,
    }


def _gate() -> CbfCommandGate:
    return CbfCommandGate(
        "UAV-02",
        ("UAV-01",),
        CbfConfig(
            minimum_separation_m=4.0,
            maximum_velocity_m_s=2.0,
            geofence_min_enu_m=(-20.0, -20.0, 0.0),
            geofence_max_enu_m=(20.0, 20.0, 20.0),
        ),
    )


def run_fault_injection() -> dict[str, Any]:
    gate = _gate()
    scenarios: dict[str, dict[str, Any]] = {}

    head_on = gate.filter(
        (2.0, 0.0, 0.0),
        {
            "UAV-02": _state((0.0, 0.0, 5.0)),
            "UAV-01": _state((4.5, 0.0, 5.0), (-1.0, 0.0, 0.0)),
        },
    )
    scenarios["head_on"] = {
        "pass": head_on.active and head_on.intervention_norm_m_s > 0.0,
        "command": head_on.as_dict(),
    }

    crossing = gate.filter(
        (2.0, 0.0, 0.0),
        {
            "UAV-02": _state((0.0, 0.0, 5.0)),
            "UAV-01": _state((2.0, 2.0, 5.0), (0.0, -1.5, 0.0)),
        },
    )
    scenarios["crossing"] = {
        "pass": crossing.active or crossing.reason == "cbf_constraints_infeasible",
        "command": crossing.as_dict(),
    }

    collapse = gate.filter(
        (2.0, 0.0, 0.0),
        {
            "UAV-02": _state((0.0, 0.0, 5.0)),
            "UAV-01": _state((3.0, 0.0, 5.0)),
        },
    )
    scenarios["formation_collapse"] = {
        "pass": not collapse.active and collapse.reason == "cbf_constraints_infeasible",
        "command": collapse.as_dict(),
    }

    stale = gate.filter(
        (1.0, 0.0, 0.0),
        {
            "UAV-02": _state((0.0, 0.0, 5.0)),
            "UAV-01": _state((10.0, 0.0, 5.0), valid=False),
        },
    )
    scenarios["peer_stale"] = {
        "pass": not stale.active and stale.reason == "peer_state_invalid",
        "command": stale.as_dict(),
    }

    geofence = gate.filter(
        (2.0, 0.0, 0.0),
        {
            "UAV-02": _state((19.9, 0.0, 5.0)),
            "UAV-01": _state((0.0, 0.0, 5.0)),
        },
    )
    scenarios["geofence"] = {
        "pass": geofence.active and geofence.velocity_enu_m_s[0] <= 0.10001,
        "command": geofence.as_dict(),
    }

    registry = PeerStateRegistry(max_age_s=0.5)
    for sequence in (1, 4):
        registry.ingest(make_peer_state(
            drone_id="UAV-01", sequence=sequence,
            position_enu_m=(10.0, 0.0, 5.0),
            velocity_enu_m_s=(0.0, 0.0, 0.0), healthy=True,
            timestamp_ms=1_000,
        ), received_monotonic_s=10.0 + sequence * 0.01)
    packet_loss = registry.snapshot(now_monotonic_s=10.2)["peers"]["UAV-01"]
    scenarios["packet_loss"] = {
        "pass": packet_loss["valid"] and packet_loss["lost_packets"] == 2,
        "peer": packet_loss,
    }

    return {
        "suite": "cbf_fault_injection_offline",
        "pass": all(result["pass"] for result in scenarios.values()),
        "scenarios": scenarios,
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", help="optional JSON report path")
    args = parser.parse_args()
    report = run_fault_injection()
    rendered = json.dumps(report, indent=2, sort_keys=True)
    print(rendered)
    if args.output:
        with open(args.output, "w", encoding="utf-8") as output:
            output.write(rendered + "\n")
    return 0 if report["pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
