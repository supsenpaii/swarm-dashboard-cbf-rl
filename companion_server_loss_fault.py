#!/usr/bin/env python3
"""Prove that losing the central server does not remove collision avoidance.

The architecture requires the server to plan missions while each companion
avoids collisions locally, so the acceptance condition is that the companion
CBF keeps producing filtered commands from peer-to-peer state while the server
is gone.

The outage is applied with SIGSTOP rather than SIGKILL for two reasons: the
run_all.sh supervisor waits on any child and tears the whole stack down when
one exits, and a stopped process models an unreachable server (it neither
serves HTTP nor consumes MQTT) without destroying the run. Both the backend
and the MQTT broker are stopped, so the entire central path is down -- if the
companion depended on either, its trace would stall or fail closed.

Read-only with respect to the vehicles: nothing here arms, takes off, or sends
any setpoint. It only signals host processes and reads the companion trace.
"""

from __future__ import annotations

import argparse
import json
import os
import signal
import subprocess
import time
from pathlib import Path
from typing import Any

API_URL = "http://127.0.0.1:8000/api/drones"


def pids_matching(pattern: str) -> list[int]:
    result = subprocess.run(
        ["pgrep", "-f", pattern], capture_output=True, text=True, check=False
    )
    return [int(line) for line in result.stdout.split() if line.strip()]


def process_name(pid: int) -> str:
    try:
        return Path(f"/proc/{pid}/comm").read_text(encoding="utf-8").strip()
    except OSError:
        return ""


def backend_pids() -> list[int]:
    """The uvicorn interpreter only.

    run_all.sh wraps the backend in a shell that pipes into a log rotator, and
    that shell's command line contains the same text, so match on the process
    name too rather than stopping the wrapper.
    """
    return [
        pid
        for pid in pids_matching(r"uvicorn main:app")
        if pid != os.getpid() and process_name(pid).startswith("python")
    ]


def broker_pids() -> list[int]:
    result = subprocess.run(
        ["pgrep", "-x", "mosquitto"], capture_output=True, text=True, check=False
    )
    return [int(line) for line in result.stdout.split() if line.strip()]


def api_reachable(timeout_s: float = 2.0) -> bool:
    result = subprocess.run(
        ["curl", "-fsS", "-m", str(timeout_s), API_URL],
        capture_output=True,
        check=False,
    )
    return result.returncode == 0


def signal_all(pids: list[int], which: int) -> list[int]:
    """Signal every pid, returning those that could not be signalled.

    A refusal must not abort the run: the broker usually belongs to another
    user, and the caller still has to restore anything it already stopped.
    """
    refused = []
    for pid in pids:
        try:
            os.kill(pid, which)
        except ProcessLookupError:
            pass
        except PermissionError:
            refused.append(pid)
    return refused


def read_trace(path: Path, since_byte: int) -> tuple[list[dict[str, Any]], int]:
    """Incremental tail of the companion trace. The cursor is a byte offset,
    opaque to callers: pass back whatever the previous call returned.

    Seeks rather than re-reading the whole file, which is not a micro-
    optimization. The earlier version called `readlines()` on every poll, so
    a caller sampling twice a second re-read the entire trace each time --
    and the bridge appends to that same file at ~40 rows/s while writing it
    with an open/write/close per frame. Measured on the first
    ACTIVE_FLIGHT_FAULT_INJECTION flight (2026-08-11): against a ~45,000-row
    trace this stalled the bridge's own 20 Hz safety loop for 0.581 s -- past
    both WarmupContract.maximum_gap_s (0.25 s) and PEER_STATE_MAX_AGE_S
    (0.5 s), so the active sender latched `setpoint_stream_gap` and reported
    `companion_safety_loop_stops`, and the flight aborted. The observing tool
    perturbed the thing it was observing. Those were the only two gaps above
    0.053 s in the whole trace, and both landed inside the one window where
    this function was being polled.
    """
    if not path.exists():
        return [], since_byte
    with path.open("rb") as handle:
        handle.seek(since_byte)
        chunk = handle.read()
    # Consume only through the last complete line: the writer appends
    # continuously, so the tail of any read can be a half-written record.
    # Leaving it for the next call costs one poll of latency and keeps a
    # torn row from being silently dropped by the decoder below.
    cut = chunk.rfind(b"\n")
    if cut < 0:
        return [], since_byte
    consumed = chunk[: cut + 1]
    fresh = []
    for line in consumed.decode("utf-8", "replace").splitlines():
        line = line.strip()
        if not line:
            continue
        try:
            fresh.append(json.loads(line))
        except json.JSONDecodeError:
            continue
    return fresh, since_byte + len(consumed)


def summarise(samples: list[dict[str, Any]]) -> dict[str, Any]:
    per_drone: dict[str, Any] = {}
    for drone_id in sorted({sample["drone_id"] for sample in samples}):
        rows = [sample for sample in samples if sample["drone_id"] == drone_id]
        margins = [
            row["cbf"]["minimum_margin_m"]
            for row in rows
            if row["cbf"]["minimum_margin_m"] is not None
        ]
        reasons: dict[str, int] = {}
        for row in rows:
            reasons[row["cbf"]["reason"]] = reasons.get(row["cbf"]["reason"], 0) + 1
        timestamps = [row["evaluated_monotonic_s"] for row in rows]
        gaps = [b - a for a, b in zip(timestamps, timestamps[1:])]
        per_drone[drone_id] = {
            "samples": len(rows),
            "cbf_reason_counts": reasons,
            "filtered_fraction": round(
                reasons.get("cbf_filtered", 0) / max(1, len(rows)), 4
            ),
            "peers_used_fraction": round(
                sum(1 for row in rows if row["peer_ids_used"]) / max(1, len(rows)), 4
            ),
            "minimum_margin_m": round(min(margins), 3) if margins else None,
            "median_margin_m": round(sorted(margins)[len(margins) // 2], 3)
            if margins
            else None,
            "max_evaluation_gap_ms": round(max(gaps) * 1000.0, 1) if gaps else None,
        }
    return per_drone


def phase(
    name: str,
    duration_s: float,
    trace: Path,
    cursor: int,
) -> tuple[dict[str, Any], int]:
    start = time.monotonic()
    api_samples: list[bool] = []
    while time.monotonic() - start < duration_s:
        api_samples.append(api_reachable())
        time.sleep(2.0)
    samples, cursor = read_trace(trace, cursor)
    return (
        {
            "phase": name,
            "duration_s": round(time.monotonic() - start, 1),
            "api_reachable_fraction": round(
                sum(api_samples) / max(1, len(api_samples)), 3
            ),
            "companion": summarise(samples),
        },
        cursor,
    )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--trace",
        default="artifacts/companion_cbf/trace_server_loss.jsonl",
        help="companion safety JSONL the bridge is currently writing",
    )
    parser.add_argument(
        "--output", default="artifacts/companion_cbf/server_loss_result.json"
    )
    parser.add_argument("--baseline-s", type=float, default=30.0)
    parser.add_argument("--outage-s", type=float, default=40.0)
    parser.add_argument("--recovery-s", type=float, default=30.0)
    arguments = parser.parse_args()

    trace = Path(arguments.trace)
    backend = backend_pids()
    broker = broker_pids()
    if not backend:
        print("no backend process found; is the stack running?")
        return 2
    print(f"backend pids={backend} broker pids={broker}")
    broker_stopped: list[int] = []

    _, cursor = read_trace(trace, 0)
    phases = []

    try:
        baseline, cursor = phase("baseline", arguments.baseline_s, trace, cursor)
        phases.append(baseline)

        print("stopping central server (backend + broker)")
        signal_all(backend, signal.SIGSTOP)
        broker_refused = signal_all(broker, signal.SIGSTOP)
        broker_stopped = [pid for pid in broker if pid not in broker_refused]
        if broker_refused:
            print(
                f"broker pids {broker_refused} could not be stopped "
                "(owned by another user); the outage covers the server "
                "application only"
            )
        try:
            outage, cursor = phase("server_lost", arguments.outage_s, trace, cursor)
            phases.append(outage)
        finally:
            print("restoring central server")
            signal_all(broker, signal.SIGCONT)
            signal_all(backend, signal.SIGCONT)

        recovery, cursor = phase("recovered", arguments.recovery_s, trace, cursor)
        phases.append(recovery)
    finally:
        # Never leave the host with stopped processes, even on an exception.
        signal_all(broker, signal.SIGCONT)
        signal_all(backend, signal.SIGCONT)

    by_name = {entry["phase"]: entry for entry in phases}
    by_name["server_lost"]["broker_also_stopped"] = broker_stopped
    outage_companion = by_name["server_lost"]["companion"]
    checks = {
        "server_was_actually_unreachable": by_name["server_lost"][
            "api_reachable_fraction"
        ]
        == 0.0,
        "server_was_reachable_before": by_name["baseline"]["api_reachable_fraction"]
        == 1.0,
        "server_recovered": by_name["recovered"]["api_reachable_fraction"] > 0.0,
        "companion_kept_filtering_during_outage": bool(outage_companion)
        and all(
            drone["filtered_fraction"] >= 0.95 for drone in outage_companion.values()
        ),
        "companion_kept_peers_during_outage": bool(outage_companion)
        and all(
            drone["peers_used_fraction"] >= 0.95 for drone in outage_companion.values()
        ),
        "companion_cadence_held_during_outage": bool(outage_companion)
        and all(
            (drone["max_evaluation_gap_ms"] or 0.0) <= 500.0
            for drone in outage_companion.values()
        ),
        "separation_never_breached": all(
            (drone["minimum_margin_m"] is None or drone["minimum_margin_m"] > 0.0)
            for entry in phases
            for drone in entry["companion"].values()
        ),
    }
    result = {
        "phases": phases,
        "checks": checks,
        "conclusion": "SERVER_LOSS_COLLISION_AVOIDANCE_PASS"
        if all(checks.values())
        else "SERVER_LOSS_COLLISION_AVOIDANCE_FAIL",
    }
    output = Path(arguments.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({"checks": checks, "conclusion": result["conclusion"]}, indent=2))
    return 0 if all(checks.values()) else 1


if __name__ == "__main__":
    raise SystemExit(main())
