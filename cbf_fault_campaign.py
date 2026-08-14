#!/usr/bin/env python3
"""CBF_FAULT_INJECTION_EXPANDED harness.

Drives the SITL/test-only SWARM_TEST_TELEMETRY_BLOCK_FILE hook through a
scripted timeline and captures /api/drones throughout. Read-only against the
API (GET), never arms/commands anything.

Usage:
  fault_campaign.py <case> <block_file> <out_path>

Cases:
  self_stale_uav02   fresh 15s -> block UAV-02 30s -> recover 20s
  self_stale_uav01   fresh 15s -> block UAV-01 30s -> recover 20s
  packet_loss        fresh 10s -> 40s of 300ms-on/300ms-off blocking -> 15s recover
  bounded_delay      fresh 10s -> 40s of 300ms-block/900ms-clear (each block
                     shorter than the 500ms stale threshold) -> 15s recover
"""
import json
import sys
import time
import urllib.request

CASE = sys.argv[1]
BLOCK_FILE = sys.argv[2]
OUT_PATH = sys.argv[3]

API_URL = "http://127.0.0.1:8000/api/drones"
POLL_S = 0.1


def fetch():
    try:
        with urllib.request.urlopen(API_URL, timeout=1.0) as resp:
            return json.loads(resp.read().decode("utf-8"))
    except Exception as exc:  # noqa: BLE001
        return {"_fetch_error": str(exc)}


def set_block(drone_id):
    with open(BLOCK_FILE, "w") as f:
        f.write(drone_id)


def plan(case):
    """Return (total_s, fn(elapsed) -> drone_id_to_block_or_empty)."""
    if case == "self_stale_uav02":
        return 65.0, lambda t: "UAV-02" if 15.0 <= t < 45.0 else ""
    if case == "self_stale_uav01":
        return 65.0, lambda t: "UAV-01" if 15.0 <= t < 45.0 else ""
    if case == "packet_loss":
        def f(t):
            if not (10.0 <= t < 50.0):
                return ""
            # 300ms blocked / 300ms open, repeating
            return "UAV-02" if int((t - 10.0) / 0.3) % 2 == 0 else ""
        return 65.0, f
    if case == "bounded_delay":
        def f(t):
            if not (10.0 <= t < 50.0):
                return ""
            # Native telemetry period is ~200-227ms, so a 150ms block drops at
            # most one message and keeps the resulting gap under the 500ms
            # stale threshold. Longer blocks swallow consecutive messages and
            # legitimately exceed it.
            return "UAV-02" if ((t - 10.0) % 1.2) < 0.15 else ""
        return 65.0, f
    raise SystemExit(f"unknown case: {case}")


def main():
    total_s, block_for = plan(CASE)
    t_start = time.monotonic()
    transitions = []
    current = None
    set_block("")
    with open(OUT_PATH, "w") as f:
        while True:
            now = time.monotonic()
            elapsed = now - t_start
            if elapsed >= total_s:
                break
            want = block_for(elapsed)
            if want != current:
                set_block(want)
                transitions.append({"wall_ts": time.time(), "elapsed": elapsed, "block": want})
                current = want
            payload = fetch()
            f.write(json.dumps({
                "poll_wall_ts": time.time(),
                "poll_mono_elapsed": elapsed,
                "block_active": want,
                "resp": payload,
            }) + "\n")
            time.sleep(POLL_S)
    set_block("")
    with open(OUT_PATH + ".transitions.json", "w") as f:
        json.dump(transitions, f, indent=2)
    print(f"done case={CASE} transitions={len(transitions)}")


if __name__ == "__main__":
    main()
