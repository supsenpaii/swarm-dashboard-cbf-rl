#!/usr/bin/env python3
"""Timed telemetry-age-stale fault injection + capture, using the new
SITL/test-only SWARM_TEST_TELEMETRY_BLOCK_FILE hook in main.py.

FAULT_TARGET: UAV-01's SwarmState ingestion path only.
FAULT_METHOD: write "UAV-01" into the block file main.py polls per MQTT
              message; main.py then skips swarm_state_store.ingest() for
              UAV-01 while still recording its raw telemetry in
              latest_drones (so /api/drones "drones"/"gazebo" sections keep
              showing UAV-01 as online -- only the common-ENU SwarmState
              layer goes stale). Clearing the file resumes ingestion.
              No process is paused, no field is edited directly, nothing is
              mocked.
"""
import json
import sys
import time
import urllib.request

BLOCK_FILE = sys.argv[1]
OUT_PATH = sys.argv[2]

FRESH_S = 15.0
STALE_S = 30.0
RECOVERY_S = 20.0
POLL_S = 0.1

API_URL = "http://127.0.0.1:8000/api/drones"


def fetch():
    try:
        with urllib.request.urlopen(API_URL, timeout=1.0) as resp:
            return json.loads(resp.read().decode("utf-8"))
    except Exception as exc:  # noqa: BLE001
        return {"_fetch_error": str(exc)}


def main():
    t_start = time.monotonic()
    events = []
    with open(OUT_PATH, "w") as f:
        fault_sent = False
        resume_sent = False
        end = t_start + FRESH_S + STALE_S + RECOVERY_S
        while True:
            now = time.monotonic()
            if now >= end:
                break
            elapsed = now - t_start
            if not fault_sent and elapsed >= FRESH_S:
                with open(BLOCK_FILE, "w") as bf:
                    bf.write("UAV-01")
                t1 = time.time()
                events.append(("T1_fault_block_written", t1, elapsed))
                fault_sent = True
            if not resume_sent and elapsed >= FRESH_S + STALE_S:
                with open(BLOCK_FILE, "w") as bf:
                    bf.write("")
                t5 = time.time()
                events.append(("T5_resume_block_cleared", t5, elapsed))
                resume_sent = True
            payload = fetch()
            record = {"poll_wall_ts": time.time(), "poll_mono_elapsed": elapsed, "resp": payload}
            f.write(json.dumps(record) + "\n")
            time.sleep(POLL_S)
    with open(OUT_PATH + ".events.json", "w") as f:
        json.dump(events, f, indent=2)
    print("done", events)


if __name__ == "__main__":
    main()
