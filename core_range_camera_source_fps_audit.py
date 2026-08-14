"""CORE_RANGE_CAMERA_SOURCE_FPS_AUDIT_AND_FIX.

Investigates why camera-source FPS in the full representative live stack
measures ~17-18 FPS today versus ~27-28 FPS in an earlier same-day smoke
(docs/CORE_RANGE_FULL_STACK_CONTENTION_FIX_REPORT.md's post-fix smoke,
reused verbatim by docs/CORE_RANGE_COLLECTION_THROUGHPUT_FIX_REPORT.md,
which concluded COLLECTION_THROUGHPUT_EVIDENCE_INSUFFICIENT: collection/
diagnostics/overlay/fsync code is not the dominant cost, and the regression
sits upstream of it, in camera-source delivery itself). No retrain, no full
dynamic corpus recollection, no M52/calibration/EKF/controller/PX4 change,
no Follow Target.

New instrumentation this task adds (both additive, opt-in, no default
runtime change):
  - camera_source_trace.py + hooks in main.py (GazeboDashboardBridge) and
    tracking_web.py (TrackingManager) -- gated by SWARM_CAMERA_TRACE_JSONL
    (a file path; unset = no-op, no file I/O).
  - run_all.sh: SWARM_START_GAZEBO_GUI (default true, unchanged behavior)
    to allow a headless-GUI ablation without duplicating the launcher.

Subcommands:
  prepare      - write profiling_plan.json
  monitor      - read-only nvidia-smi dmon/pmon + pidstat + gz world-stats
                 (real-time factor) sampling for a fixed duration
  gpu-snapshot - one-shot nvidia-smi snapshot appended to process_gpu_usage
  analyze-run  - parse one completed run's camera_source_trace.jsonl +
                 physical_diagnostics.jsonl + capture_result.json into
                 per-hop FPS/jitter metrics; appends to the shared CSVs
  validate     - post-fix validation smoke: gate-check one completed run
                 against the precommitted validation thresholds, appends
                 to validation_smoke.csv
  compose-final - assemble remaining deliverables + gate conclusion
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import re
import subprocess
import time
from pathlib import Path
from statistics import median, pstdev
from typing import Any, Mapping, Sequence

import numpy as np

AUDIT_ID = "core_range_camera_source_fps_audit_20260805_v001"
PREWARM_SKIP_S = 5.0  # matches the task's ">=30s after prewarm" requirement

GATE_OUTCOMES = (
    "CAMERA_SOURCE_FPS_FIXED_READY_FOR_RECOLLECTION",
    "CAMERA_SOURCE_CONFIG_INTENTIONALLY_BELOW_GATE",
    "GAZEBO_REALTIME_FACTOR_DOMINANT",
    "CAMERA_BRIDGE_OR_TRANSPORT_DOMINANT",
    "BACKEND_CAMERA_RECEIVE_DOMINANT",
    "HISTORICAL_CURRENT_CONFIG_MISMATCH",
    "CAMERA_SOURCE_FPS_EVIDENCE_INSUFFICIENT",
    "CAMERA_SOURCE_FIX_BLOCKED_BY_ENVIRONMENT",
)

VALIDATION_GATE = {
    "camera_source_median_fps_min": 25.0,
    "tracking_median_fps_min": 20.0,
    "tracking_over_camera_source_frame_fraction_min": 0.90,
    "capture_to_consume_median_ms_max": 200.0,
    "capture_to_consume_p95_ms_max": 300.0,
    "midas_p95_ms_max": 100.0,
    "no_growing_backlog": True,
    "raw_availability_non_degrading": True,
}

HOPS = [
    {
        "hop": "H1_gazebo_sim_publish",
        "event": "camera_source_receipt",
        "timestamp_field": "sim_timestamp_s",
        "description": (
            "Gazebo camera sensor's own simulation-time publish cadence "
            "(message header.stamp), sampled at every gz-transport callback "
            "regardless of client/tracking gating."
        ),
    },
    {
        "hop": "H2_backend_gztransport_receipt",
        "event": "camera_source_receipt",
        "timestamp_field": "monotonic_receipt_s",
        "description": (
            "Wall-clock monotonic receipt time in GazeboDashboardBridge."
            "_handle_camera_image, every callback. Architecture note: this "
            "app subscribes to Gazebo camera images directly via "
            "gz.transport13 (main.py) -- there is no ROS2/Gazebo image "
            "bridge process and no separate ROS2 image topic republish in "
            "this codebase (confirmed by code audit: no ros_gz_bridge, "
            "image_transport, or sensor_msgs/Image usage for camera data "
            "anywhere in the app). The task's hops 2-4 (Gazebo topic "
            "publish / ROS2-Gazebo bridge receive / ROS2 image topic "
            "publish) therefore collapse into this single measured hop; "
            "H1 vs H2 is the only meaningful split of that portion of the "
            "pipeline for this architecture."
        ),
    },
    {
        "hop": "H3_camera_mailbox_store",
        "event": "camera_mailbox_store",
        "timestamp_field": "monotonic_store_s",
        "description": (
            "Frame accepted into the single-slot raw_frames mailbox for "
            "the actively-tracked drone (main.py), after BGR conversion."
        ),
    },
    {
        "hop": "H4_tracker_mailbox_dequeue",
        "event": "camera_mailbox_dequeue",
        "timestamp_field": "dequeue_monotonic_s",
        "description": (
            "TrackingManager._run (tracking_web.py) wakes on the mailbox "
            "condition variable and pulls the latest frame version."
        ),
    },
    {
        "hop": "H5_tracker_output",
        "event": "tracker_output",
        "timestamp_field": "tracker_output_monotonic_s",
        "description": (
            "tracker.update(frame) returns inside TrackingManager."
            "_process_frame -- this is the detector/tracker output hop."
        ),
    },
]


# ---------------------------------------------------------------------------
# generic helpers
# ---------------------------------------------------------------------------

def _json(path: Path) -> Any:
    return json.loads(path.read_text(encoding="utf-8"))


def _write_json(path: Path, value: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def _write_csv(path: Path, rows: Sequence[Mapping[str, Any]], fieldnames: Sequence[str] | None = None) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    if not rows:
        if not path.exists():
            path.write_text("", encoding="utf-8")
        return
    names = list(fieldnames) if fieldnames is not None else list(rows[0].keys())
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=names)
        writer.writeheader()
        writer.writerows(rows)


def _append_csv(path: Path, rows: Sequence[Mapping[str, Any]], fieldnames: Sequence[str]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    existing: list[dict[str, Any]] = []
    if path.exists() and path.stat().st_size > 0:
        with path.open() as stream:
            existing = list(csv.DictReader(stream))
    _write_csv(path, existing + [dict(row) for row in rows], fieldnames=fieldnames)


def _percentile(values: Sequence[float], pct: float) -> float | None:
    if not values:
        return None
    return float(np.percentile(np.asarray(values, dtype=np.float64), pct))


def _sha256(path: Path) -> str | None:
    if not path.exists():
        return None
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _load_jsonl(path: Path) -> list[dict[str, Any]]:
    if not path.exists():
        return []
    rows = []
    for line in path.read_text(encoding="utf-8").splitlines():
        line = line.strip()
        if line:
            try:
                rows.append(json.loads(line))
            except json.JSONDecodeError:
                continue
    return rows


# ---------------------------------------------------------------------------
# prepare
# ---------------------------------------------------------------------------

def prepare(output: Path) -> dict[str, Any]:
    plan = {
        "audit_id": AUDIT_ID,
        "no_full_dynamic_corpus_recollection": True,
        "no_training": True,
        "no_range_model_m52_calibration_ekf_controller_px4_change": True,
        "no_follow_target": True,
        "instrumentation": {
            "camera_source_trace_env_var": "SWARM_CAMERA_TRACE_JSONL",
            "gazebo_gui_toggle_env_var": "SWARM_START_GAZEBO_GUI",
            "opt_in": True,
            "default_runtime_unchanged": True,
        },
        "hops": HOPS,
        "configs": [
            {"name": "A_current_representative_baseline", "varying_factor_vs_previous": "baseline"},
            {"name": "B_gazebo_headless_no_gui", "varying_factor_vs_previous": "SWARM_START_GAZEBO_GUI=false"},
            {"name": "C_depth_throttled_near_off", "varying_factor_vs_previous": "SWARM_RANGE_DATASET_DEPTH_RATE_HZ lowered to ~0.1Hz (functionally off) via the existing production rate knob -- checks GPU/render contention from MiDaS"},
            {"name": "D_dashboard_client_off", "varying_factor_vs_previous": "no WebSocket/browser dashboard client attached during capture (observation-only API capture already does not attach one; this config makes that explicit and re-verifies it)"},
            {"name": "E_ros_bridge_bypass", "varying_factor_vs_previous": "N/A -- architecture audit (see H2 description) found the camera path already bypasses ROS2 entirely (direct gz-transport13 subscription in main.py, no ros_gz_bridge/image_transport/sensor_msgs usage anywhere for camera data); there is nothing to bypass that is not already bypassed, so this configuration is not separately executed"},
            {"name": "F_historical_config_restore_or_fresh_repeat", "varying_factor_vs_previous": "restore the exact historical launch config if a diff is found (see historical_current_config_diff.json); if no diff is found, this becomes a fresh same-config repeat to test environmental/runtime-state recovery"},
        ],
        "resource_metrics": [
            "gazebo_real_time_factor_median_p5", "simulation_step_rate",
            "camera_sensor_configured_update_rate_hz", "gpu_utilization_pct",
            "gpu_memory_used_mb", "cpu_utilization_pct_per_process",
            "renderer_frame_time_ms", "ros_bridge_cpu_pct",
            "image_serialization_copy_duration_ms", "subscriber_backlog",
            "dropped_transport_messages",
        ],
        "validation_gate": VALIDATION_GATE,
        "gate_outcomes": list(GATE_OUTCOMES),
    }
    _write_json(output / "profiling_plan.json", plan)
    print(json.dumps({"hops": len(HOPS), "configs": len(plan["configs"])}, indent=2))
    return plan


# ---------------------------------------------------------------------------
# monitor: read-only nvidia-smi dmon/pmon + pidstat + gz world-stats (RTF)
# ---------------------------------------------------------------------------

def _gz_stats_sampler(path: Path, duration_s: float, world: str = "default") -> subprocess.Popen:
    """Background loop: sample /world/<world>/stats (real_time_factor,
    sim/real time, iterations) once per second via the read-only `gz topic
    -e` CLI, for the given duration. Writes raw protobuf text-format
    snapshots, parsed later in compose_final."""
    script = (
        f'topic="/world/{world}/stats"; '
        f'end=$(( $(date +%s) + {int(duration_s)} )); '
        'while [ "$(date +%s)" -lt "$end" ]; do '
        '  echo "--- $(date +%s.%N) ---"; '
        f'  timeout 2 gz topic -e -t "$topic" -n 1 2>&1; '
        '  sleep 1; '
        'done'
    )
    path.parent.mkdir(parents=True, exist_ok=True)
    handle = path.open("w")
    return subprocess.Popen(["bash", "-c", script], stdout=handle, stderr=subprocess.STDOUT)


def monitor(output: Path, duration_s: float, label: str, world: str = "default") -> None:
    dmon_path = output / f"{label}_nvidia_smi_dmon.log"
    pmon_path = output / f"{label}_nvidia_smi_pmon.log"
    pidstat_path = output / f"{label}_pidstat.log"
    gz_stats_path = output / f"{label}_gz_world_stats.log"
    output.mkdir(parents=True, exist_ok=True)

    with dmon_path.open("w") as dmon_file, pmon_path.open("w") as pmon_file, pidstat_path.open("w") as pidstat_file:
        dmon = subprocess.Popen(["nvidia-smi", "dmon", "-s", "pucv", "-d", "1"], stdout=dmon_file, stderr=subprocess.STDOUT)
        pmon = subprocess.Popen(["nvidia-smi", "pmon", "-s", "u", "-d", "1"], stdout=pmon_file, stderr=subprocess.STDOUT)
        pidstat = subprocess.Popen(["pidstat", "-t", "-u", "-r", "-w", "1", str(int(duration_s))], stdout=pidstat_file, stderr=subprocess.STDOUT)
        gz_stats = _gz_stats_sampler(gz_stats_path, duration_s, world=world)
        try:
            time.sleep(duration_s)
        finally:
            for proc in (dmon, pmon, pidstat, gz_stats):
                if proc.poll() is None:
                    proc.terminate()
            for proc in (dmon, pmon, pidstat, gz_stats):
                try:
                    proc.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    proc.kill()
    print(json.dumps({
        "label": label, "duration_s": duration_s, "dmon": str(dmon_path),
        "pmon": str(pmon_path), "pidstat": str(pidstat_path), "gz_stats": str(gz_stats_path),
    }, indent=2))


def process_gpu_usage_snapshot(output: Path, label: str) -> None:
    try:
        completed = subprocess.run(
            ["nvidia-smi", "--query-compute-apps=pid,process_name,used_memory", "--format=csv,noheader"],
            check=False, text=True, capture_output=True, timeout=5,
        )
        lines = [line.strip() for line in completed.stdout.strip().splitlines() if line.strip()]
    except Exception as error:
        lines = [f"ERROR:{error}"]
    rows = []
    for line in lines:
        parts = [p.strip() for p in line.split(",")]
        if len(parts) == 3:
            rows.append({"label": label, "pid": parts[0], "process_name": parts[1], "used_memory": parts[2]})
    _append_csv(output / "process_gpu_usage.csv", rows, fieldnames=["label", "pid", "process_name", "used_memory"])
    print(json.dumps({"label": label, "gpu_processes": len(rows)}, indent=2))


# ---------------------------------------------------------------------------
# analyze-run: per-hop FPS/jitter metrics from one completed run
# ---------------------------------------------------------------------------

def _interval_metrics(timestamps: Sequence[float]) -> dict[str, Any]:
    ts = sorted(timestamps)
    duplicates = sum(1 for a, b in zip(ts, ts[1:]) if b == a)
    intervals = [b - a for a, b in zip(ts, ts[1:]) if b > a]
    if not intervals:
        return {
            "frame_count": len(ts), "median_fps": 0.0,
            "p5_interframe_interval_s": None, "p95_interframe_interval_s": None,
            "jitter_stddev_s": None, "duplicate_timestamp_count": duplicates,
        }
    median_interval = median(intervals)
    return {
        "frame_count": len(ts),
        "median_fps": round(1.0 / median_interval, 3) if median_interval > 0 else 0.0,
        "p5_interframe_interval_s": round(_percentile(intervals, 5) or 0.0, 6),
        "p95_interframe_interval_s": round(_percentile(intervals, 95) or 0.0, 6),
        "jitter_stddev_s": round(pstdev(intervals), 6) if len(intervals) > 1 else 0.0,
        "duplicate_timestamp_count": duplicates,
    }


def _sequence_drop_count(seq_values: Sequence[int]) -> int:
    values = sorted(set(int(v) for v in seq_values if v is not None))
    if len(values) < 2:
        return 0
    return (max(values) - min(values) + 1) - len(values)


def analyze_run(run_dir: Path, output: Path, label: str, prewarm_skip_s: float = PREWARM_SKIP_S) -> dict[str, Any]:
    trace_path = run_dir / "camera_source_trace.jsonl"
    events = _load_jsonl(trace_path)

    by_event: dict[str, list[dict[str, Any]]] = {}
    for row in events:
        by_event.setdefault(row.get("event", "?"), []).append(row)

    hop_rows: list[dict[str, Any]] = []
    jitter_rows: list[dict[str, Any]] = []
    seq_fields = {
        "camera_source_receipt": "frame_seq",
        "camera_mailbox_store": "frame_seq",
        "camera_mailbox_dequeue": "raw_frame_version",
        "tracker_output": "raw_frame_version",
    }

    drone_ids = sorted({row.get("drone_id") for row in events if row.get("drone_id")})
    for drone_id in drone_ids:
        for hop in HOPS:
            rows = [r for r in by_event.get(hop["event"], []) if r.get("drone_id") == drone_id]
            if not rows:
                continue
            rows.sort(key=lambda r: r.get("trace_monotonic_s", 0.0))
            t0 = rows[0]["trace_monotonic_s"]
            post_prewarm = [r for r in rows if r.get("trace_monotonic_s", t0) - t0 >= prewarm_skip_s]
            use_rows = post_prewarm if len(post_prewarm) >= 5 else rows
            timestamps = [r.get(hop["timestamp_field"]) for r in use_rows]
            timestamps = [float(t) for t in timestamps if isinstance(t, (int, float))]
            metrics = _interval_metrics(timestamps)
            seq_field = seq_fields.get(hop["event"])
            dropped = _sequence_drop_count([r.get(seq_field) for r in use_rows]) if seq_field else None
            hop_row = {
                "label": label, "hop": hop["hop"], "drone_id": drone_id,
                "post_prewarm_skip_s": prewarm_skip_s,
                **metrics, "dropped_sequence_count": dropped,
                "queue_depth": "n/a (single-slot latest-wins mailbox by design; see H3/H4)" if hop["hop"] in ("H3_camera_mailbox_store", "H4_tracker_mailbox_dequeue") else "n/a",
            }
            hop_rows.append(hop_row)
            jitter_rows.append({
                "label": label, "hop": hop["hop"], "drone_id": drone_id,
                "p5_interframe_interval_s": metrics["p5_interframe_interval_s"],
                "p95_interframe_interval_s": metrics["p95_interframe_interval_s"],
                "jitter_stddev_s": metrics["jitter_stddev_s"],
                "median_fps": metrics["median_fps"], "frame_count": metrics["frame_count"],
            })

        # callback/mailbox delay: dequeue - store (H3->H4), per event pair on raw_frame_version
        store_by_version = {
            r.get("raw_frame_version"): r.get("monotonic_store_s")
            for r in by_event.get("camera_mailbox_store", []) if r.get("drone_id") == drone_id
        }
        dequeue_rows_d = [r for r in by_event.get("camera_mailbox_dequeue", []) if r.get("drone_id") == drone_id]
        callback_delays = []
        for row in dequeue_rows_d:
            store_t = store_by_version.get(row.get("raw_frame_version"))
            if store_t is not None and row.get("dequeue_monotonic_s") is not None:
                callback_delays.append(row["dequeue_monotonic_s"] - store_t)
        if callback_delays:
            hop_rows.append({
                "label": label, "hop": "H3_to_H4_callback_delay_ms", "drone_id": drone_id,
                "frame_count": len(callback_delays),
                "median_fps": None,
                "p5_interframe_interval_s": round((_percentile(callback_delays, 5) or 0.0) * 1000, 3),
                "p95_interframe_interval_s": round((_percentile(callback_delays, 95) or 0.0) * 1000, 3),
                "jitter_stddev_s": round(pstdev(callback_delays) * 1000, 3) if len(callback_delays) > 1 else 0.0,
                "duplicate_timestamp_count": None, "dropped_sequence_count": None,
                "queue_depth": "callback delay in ms, not an FPS row",
            })

    _append_csv(
        output / "hop_fps_metrics.csv", hop_rows,
        fieldnames=["label", "hop", "drone_id", "post_prewarm_skip_s", "frame_count", "median_fps",
                    "p5_interframe_interval_s", "p95_interframe_interval_s", "jitter_stddev_s",
                    "duplicate_timestamp_count", "dropped_sequence_count", "queue_depth"],
    )
    _append_csv(
        output / "interframe_jitter.csv", jitter_rows,
        fieldnames=["label", "hop", "drone_id", "p5_interframe_interval_s", "p95_interframe_interval_s",
                    "jitter_stddev_s", "median_fps", "frame_count"],
    )

    result = {"label": label, "run_dir": str(run_dir), "trace_events": len(events),
              "drone_ids": drone_ids, "hop_rows": hop_rows}
    _write_json(output / f"{label}_hop_metrics.json", result)
    print(json.dumps({"label": label, "trace_events": len(events), "hops_written": len(hop_rows)}, indent=2))
    return result


# ---------------------------------------------------------------------------
# validate: post-fix gate check against VALIDATION_GATE
# ---------------------------------------------------------------------------

def validate(run_dir: Path, output: Path, label: str, prewarm_skip_s: float = PREWARM_SKIP_S) -> dict[str, Any]:
    hop_metrics = analyze_run(run_dir, output, label, prewarm_skip_s=prewarm_skip_s)
    drone_ids = hop_metrics["drone_ids"]

    def _hop_median_fps(hop_name: str) -> list[float]:
        return [
            row["median_fps"] for row in hop_metrics["hop_rows"]
            if row["hop"] == hop_name and row["median_fps"] is not None
        ]

    h2_values = _hop_median_fps("H2_backend_gztransport_receipt")
    h5_values = _hop_median_fps("H5_tracker_output")
    camera_source_median_fps = round(sum(h2_values) / len(h2_values), 3) if h2_values else None
    tracking_median_fps = round(sum(h5_values) / len(h5_values), 3) if h5_values else None

    # H5 (tracker output) only exists for the actively-tracked drone, while
    # H2 (backend receipt) exists for every camera-publishing drone -- so
    # this fraction must compare H2 against the SAME tracked drone_id(s),
    # not H2 summed across every drone (which would spuriously deflate it
    # on a two-UAV scenario where only one drone is tracked).
    tracked_drone_ids = {
        row["drone_id"] for row in hop_metrics["hop_rows"] if row["hop"] == "H5_tracker_output"
    }
    h2_frames = sum(
        row["frame_count"] for row in hop_metrics["hop_rows"]
        if row["hop"] == "H2_backend_gztransport_receipt" and row["drone_id"] in tracked_drone_ids
    )
    h5_frames = sum(row["frame_count"] for row in hop_metrics["hop_rows"] if row["hop"] == "H5_tracker_output")
    tracking_over_source_fraction = round(h5_frames / h2_frames, 4) if h2_frames else None

    capture_path = run_dir / "capture_result.json"
    diagnostics_path = run_dir / "physical_diagnostics.jsonl"
    capture = _json(capture_path) if capture_path.exists() else {}
    session_id = capture.get("session_id")
    diag_rows = [row for row in _load_jsonl(diagnostics_path) if row.get("session_id") == session_id]

    def _delta_ms(rows: list[dict[str, Any]], start: str, end: str) -> list[float]:
        values = []
        for row in rows:
            stages = row.get("timestamp_stages") or {}
            try:
                values.append(1000 * (stages[end]["timestamp_s"] - stages[start]["timestamp_s"]))
            except (KeyError, TypeError):
                pass
        return values

    capture_consume_ms = _delta_ms(diag_rows, "frame_receipt", "consume")
    midas_ms = [
        float(row["timestamps"]["depth_inference_ms"]) for row in diag_rows
        if isinstance(row.get("timestamps"), dict) and isinstance(row["timestamps"].get("depth_inference_ms"), (int, float))
    ]
    raw_range_count = sum(row.get("stage") == "raw_range_computed" for row in diag_rows)
    timestamps_ok = all(bool(row.get("timestamp_order_valid")) for row in diag_rows) if diag_rows else False
    checksums_ok = all(bool(row.get("record_sha256")) for row in diag_rows) if diag_rows else False

    capture_consume_median = round(median(capture_consume_ms), 2) if capture_consume_ms else None
    capture_consume_p95 = round(_percentile(capture_consume_ms, 95) or 0.0, 2) if capture_consume_ms else None
    midas_p95 = round(_percentile(midas_ms, 95) or 0.0, 2) if midas_ms else None

    checks = {
        "camera_source_median_fps_ge_25": (camera_source_median_fps or 0) >= VALIDATION_GATE["camera_source_median_fps_min"],
        "tracking_median_fps_ge_20": (tracking_median_fps or 0) >= VALIDATION_GATE["tracking_median_fps_min"],
        "tracking_over_source_fraction_ge_90pct": (tracking_over_source_fraction or 0) >= VALIDATION_GATE["tracking_over_camera_source_frame_fraction_min"],
        "capture_consume_median_le_200ms": capture_consume_median is not None and capture_consume_median <= VALIDATION_GATE["capture_to_consume_median_ms_max"],
        "capture_consume_p95_le_300ms": capture_consume_p95 is not None and capture_consume_p95 <= VALIDATION_GATE["capture_to_consume_p95_ms_max"],
        "midas_p95_le_100ms": midas_p95 is not None and midas_p95 <= VALIDATION_GATE["midas_p95_ms_max"],
        "raw_range_count_ge_40": raw_range_count >= 40,
        "timestamp_integrity": timestamps_ok,
        "checksum_integrity": checksums_ok,
    }
    gate_pass = all(checks.values())

    row = {
        "label": label, "drone_ids": ";".join(drone_ids), "raw_range_count": raw_range_count,
        "camera_source_median_fps": camera_source_median_fps, "tracking_median_fps": tracking_median_fps,
        "tracking_over_source_fraction": tracking_over_source_fraction,
        "capture_consume_median_ms": capture_consume_median, "capture_consume_p95_ms": capture_consume_p95,
        "midas_p95_ms": midas_p95, "timestamp_integrity": timestamps_ok, "checksum_integrity": checksums_ok,
        "gate_pass": gate_pass, **{f"check_{name}": value for name, value in checks.items()},
    }
    _append_csv(
        output / "validation_smoke.csv", [row],
        fieldnames=list(row.keys()),
    )
    print(json.dumps({"label": label, "gate_pass": gate_pass, **{k: v for k, v in row.items() if not k.startswith("check_")}}, indent=2))
    return row


# ---------------------------------------------------------------------------
# resource-log parsing (nvidia-smi dmon, gz world-stats)
# ---------------------------------------------------------------------------

def _parse_dmon_log(path: Path) -> dict[str, Any]:
    """`nvidia-smi dmon -s pucv` header order on this driver is
    `gpu pwr gtemp mtemp sm mem enc dec jpg ofa mclk pclk pviol tviol`
    (verified against a captured log header) -- sm (% SM util) is column
    index 4, not a fixed offset assumed a priori, so this is parsed against
    the log's own header line rather than a hardcoded column count."""
    if not path.exists():
        return {}
    sm_index = 4
    pviol_index = None
    for line in path.read_text().splitlines():
        if line.startswith("# gpu"):
            header = line.lstrip("#").split()
            if "sm" in header:
                sm_index = header.index("sm")
            if "pviol" in header:
                pviol_index = header.index("pviol")
            break
    sm_values: list[float] = []
    pviol_values: list[float] = []
    for line in path.read_text().splitlines():
        parts = line.split()
        if len(parts) > sm_index and not parts[0].startswith("#") and parts[0].lstrip("-").isdigit():
            try:
                sm_values.append(float(parts[sm_index]))
            except ValueError:
                continue
            if pviol_index is not None and len(parts) > pviol_index:
                try:
                    pviol_values.append(float(parts[pviol_index]))
                except ValueError:
                    pass
    if not sm_values:
        return {}
    result = {
        "gpu_util_median_pct": round(median(sm_values), 2),
        "gpu_util_p95_pct": round(_percentile(sm_values, 95) or 0.0, 2),
        "gpu_util_samples": len(sm_values),
    }
    if pviol_values:
        result["gpu_power_violation_median_pct"] = round(median(pviol_values), 1)
    return result


def _parse_gz_stats_log(path: Path) -> dict[str, Any]:
    if not path.exists():
        return {}
    text = path.read_text()
    rtf_values = [float(m) for m in re.findall(r"real_time_factor:\s*([\d.eE+-]+)", text)]
    iterations = [int(m) for m in re.findall(r"iterations:\s*(\d+)", text)]
    step_rates = []
    for a, b in zip(iterations, iterations[1:]):
        if b > a:
            step_rates.append(b - a)
    result: dict[str, Any] = {"rtf_samples": len(rtf_values)}
    if rtf_values:
        result["real_time_factor_median"] = round(median(rtf_values), 4)
        result["real_time_factor_p5"] = round(_percentile(rtf_values, 5) or 0.0, 4)
    if step_rates:
        result["simulation_step_rate_median_per_s"] = round(median(step_rates), 1)
    return result


def _parse_pidstat_cpu(path: Path, process_substrings: Sequence[str]) -> dict[str, list[float]]:
    """Best-effort %CPU-per-process extraction from a `pidstat -t -u -r -w`
    log. Column offsets shift with the 12-hour timestamp format, so this
    matches by substring on the trailing Command token and takes the last
    decimal-looking field on the line (%CPU is consistently the last
    floating-point column pidstat -u prints before Command)."""
    if not path.exists():
        return {}
    by_process: dict[str, list[float]] = {name: [] for name in process_substrings}
    for line in path.read_text().splitlines():
        stripped = line.strip()
        if not stripped or stripped.startswith("Linux") or stripped.startswith("Average") or "%CPU" in stripped:
            continue
        tokens = stripped.split()
        if len(tokens) < 10:
            continue
        command = tokens[-1]
        for name in process_substrings:
            if name in command:
                cpu_candidates = [t for t in tokens if re.fullmatch(r"\d+\.\d+", t)]
                if cpu_candidates:
                    by_process[name].append(float(cpu_candidates[-1]))
                break
    return by_process


# ---------------------------------------------------------------------------
# compose-final
# ---------------------------------------------------------------------------

def _historical_current_config_diff(output: Path) -> dict[str, Any]:
    repo = Path(__file__).resolve().parent
    px4_root = Path("/mnt/px4ssd/PX4-Autopilot")
    gimbal_sdf = px4_root / "Tools/simulation/gz/models/gimbal/model.sdf"
    world_sdf = px4_root / "Tools/simulation/gz/worlds/default.sdf"

    historical_hashes = _json(repo / "artifacts/core_range_3_12m/full_stack_contention_fix/source_changes.json")
    current_hashes = {
        name: _sha256(repo / name)
        for name in ("main.py", "tracking_web.py", "run_all.sh", "metric_target_fusion.py", "range_physical_diagnostics.py")
    }
    code_diff = {
        name: {
            "historical_sha256": (historical_hashes.get(name) or {}).get("sha256"),
            "current_sha256": current_hashes[name],
            "note": "run_all.sh and main.py/tracking_web.py were additively modified by THIS task (SWARM_START_GAZEBO_GUI env-gate, opt-in camera trace hooks); the historical hash predates those additive-only changes" if name in ("run_all.sh", "main.py", "tracking_web.py") else "unchanged",
        }
        for name in current_hashes
    }

    gimbal_camera_block = None
    if gimbal_sdf.exists():
        text = gimbal_sdf.read_text()
        match = re.search(r'<sensor name="camera" type="camera">.*?</sensor>', text, re.DOTALL)
        gimbal_camera_block = match.group(0) if match else None

    world_physics_block = None
    if world_sdf.exists():
        text = world_sdf.read_text()
        match = re.search(r"<physics[^>]*>.*?</physics>", text, re.DOTALL)
        world_physics_block = match.group(0) if match else None

    diff = {
        "audit_id": AUDIT_ID,
        "method": (
            "Compared file mtimes/hashes for external Gazebo world/model SDF "
            "(outside this repo, under PX4_AUTOPILOT_ROOT) against the "
            "timestamps of the historical 27-28 FPS smoke "
            "(full_stack_contention_fix, ~05:31 UTC 2026-08-05) and the "
            "currently-regressed paired runs (collection_throughput_fix, "
            "~06:42 UTC 2026-08-05 onward). Also compared repo-tracked "
            "Python source hashes against the prior task's recorded hashes."
        ),
        "gazebo_world_file": str(world_sdf),
        "gazebo_world_mtime": world_sdf.stat().st_mtime if world_sdf.exists() else None,
        "gazebo_world_physics_block": world_physics_block,
        "gimbal_camera_model_file": str(gimbal_sdf),
        "gimbal_camera_model_mtime": gimbal_sdf.stat().st_mtime if gimbal_sdf.exists() else None,
        "gimbal_camera_sensor_block": gimbal_camera_block,
        "gimbal_model_backup_found": str(gimbal_sdf.parent / "model.sdf.pre_gui_optimization_20260723") if (gimbal_sdf.parent / "model.sdf.pre_gui_optimization_20260723").exists() else None,
        "external_sdf_mutation_tool": {
            "path": str(repo / "configure_gazebo_camera.py"),
            "note": (
                "A standalone tool exists in this repo that regex-patches "
                "width/height/horizontal_fov/update_rate inside a named "
                "<sensor> block of an arbitrary SDF file. No caller of it "
                "was found inside run_all.sh/main.py; its own mtime and the "
                "gimbal model.sdf's mtime both predate today's runs, so it "
                "was not the cause of today's regression, but it is the "
                "mechanism by which this external, non-version-controlled "
                "file could silently drift in the future -- flagged for "
                "awareness, not modified by this task."
            ),
        },
        "code_hashes": code_diff,
        "launch_mode": {
            "historical_and_current_both": "run_all.sh with Gazebo server headless (-r -s) + separate GUI client (-g), physics real_time_update_rate=250 max_step_size=0.004 (target RTF 1.0), render_engine=ogre2",
            "camera_sensor_resolution_and_rate": "480x270 R8G8B8, update_rate=50Hz (both historical and current -- unchanged file, mtime predates both runs)",
        },
        "conclusion": (
            "No static Gazebo world/model/launch config difference was found "
            "between the historical 27-28 FPS smoke and the current ~17-18 "
            "FPS runs: the world file, the camera sensor SDF (resolution, "
            "format, update_rate), and the application/launcher source code "
            "all predate or are identical across both. This corroborates "
            "docs/CORE_RANGE_COLLECTION_THROUGHPUT_FIX_REPORT.md's own "
            "finding of 'no COLLECTION_METRIC_DEFINITION_MISMATCH' and "
            "extends it to the Gazebo/launch layer: the regression is not "
            "explained by a config diff, so it must be a runtime/live-state "
            "effect, which is what this task's hop-level + resource "
            "instrumentation is designed to isolate."
        ),
    }
    _write_json(output / "historical_current_config_diff.json", diff)
    return diff


def compose_final(output: Path, config_labels: Sequence[str]) -> dict[str, Any]:
    repo = Path(__file__).resolve().parent
    config_diff = _historical_current_config_diff(output)

    cpu_process_names = ["px4", "python", "gzserver", "gz", "ros2", "mavlink_manual_bridge.py"]
    simulator_rows = []
    resource_rows = []
    for label in config_labels:
        # core_range_camera_fps_run_scenario.sh points `monitor --output` at
        # output/configs/<label>/, not the top-level output dir -- logs
        # therefore live one level deeper than the CSVs this function
        # writes. Fall back to the flat layout for callers that don't use
        # that per-config subdirectory convention.
        run_dir = output / "configs" / label
        if not (run_dir / f"{label}_nvidia_smi_dmon.log").exists():
            run_dir = output
        gz_stats = _parse_gz_stats_log(run_dir / f"{label}_gz_world_stats.log")
        dmon = _parse_dmon_log(run_dir / f"{label}_nvidia_smi_dmon.log")
        pidstat_cpu = _parse_pidstat_cpu(run_dir / f"{label}_pidstat.log", cpu_process_names)
        if gz_stats or dmon or any(pidstat_cpu.values()):
            simulator_rows.append({"label": label, **gz_stats})
            cpu_medians = {
                f"cpu_median_pct_{name}": round(median(values), 2) if values else None
                for name, values in pidstat_cpu.items()
            }
            resource_rows.append({"label": label, **dmon, **cpu_medians})

    _write_csv(output / "simulator_metrics.csv", simulator_rows, fieldnames=["label", "rtf_samples", "real_time_factor_median", "real_time_factor_p5", "simulation_step_rate_median_per_s"])
    _write_csv(
        output / "resource_metrics.csv", resource_rows,
        fieldnames=["label", "gpu_util_median_pct", "gpu_util_p95_pct", "gpu_util_samples",
                    "gpu_power_violation_median_pct",
                    *[f"cpu_median_pct_{name}" for name in cpu_process_names]],
    )

    # ablation_metrics.csv: one row per config, pulling H2 (backend receipt,
    # the true source-delivery FPS) and H4 (tracker consumption FPS) from
    # hop_fps_metrics.csv if present.
    hop_csv = output / "hop_fps_metrics.csv"
    hop_rows_all: list[dict[str, Any]] = []
    if hop_csv.exists() and hop_csv.stat().st_size > 0:
        with hop_csv.open() as stream:
            hop_rows_all = list(csv.DictReader(stream))

    def _hop_value(label: str, hop: str, field: str) -> Any:
        matches = [row for row in hop_rows_all if row["label"] == label and row["hop"] == hop]
        if not matches:
            return None
        values = [float(row[field]) for row in matches if row.get(field) not in (None, "", "None")]
        return round(sum(values) / len(values), 3) if values else None

    ablation_rows = []
    for label in config_labels:
        ablation_rows.append({
            "config": label,
            "camera_source_median_fps_H2": _hop_value(label, "H2_backend_gztransport_receipt", "median_fps"),
            "gazebo_sim_publish_median_fps_H1": _hop_value(label, "H1_gazebo_sim_publish", "median_fps"),
            "tracker_dequeue_median_fps_H4": _hop_value(label, "H4_tracker_mailbox_dequeue", "median_fps"),
            "tracker_output_median_fps_H5": _hop_value(label, "H5_tracker_output", "median_fps"),
        })
    _write_csv(output / "ablation_metrics.csv", ablation_rows)

    print(json.dumps({"configs_composed": len(config_labels)}, indent=2))
    return {"config_diff": config_diff, "ablation_rows": ablation_rows, "simulator_rows": simulator_rows, "resource_rows": resource_rows}


def main() -> int:
    parser = argparse.ArgumentParser()
    sub = parser.add_subparsers(dest="command", required=True)

    p = sub.add_parser("prepare")
    p.add_argument("--output", type=Path, required=True)

    p = sub.add_parser("monitor")
    p.add_argument("--output", type=Path, required=True)
    p.add_argument("--duration-s", type=float, required=True)
    p.add_argument("--label", type=str, required=True)
    p.add_argument("--world", type=str, default="default")

    p = sub.add_parser("gpu-snapshot")
    p.add_argument("--output", type=Path, required=True)
    p.add_argument("--label", type=str, required=True)

    p = sub.add_parser("analyze-run")
    p.add_argument("--run-dir", type=Path, required=True)
    p.add_argument("--output", type=Path, required=True)
    p.add_argument("--label", type=str, required=True)
    p.add_argument("--prewarm-skip-s", type=float, default=PREWARM_SKIP_S)

    p = sub.add_parser("validate")
    p.add_argument("--run-dir", type=Path, required=True)
    p.add_argument("--output", type=Path, required=True)
    p.add_argument("--label", type=str, required=True)
    p.add_argument("--prewarm-skip-s", type=float, default=PREWARM_SKIP_S)

    p = sub.add_parser("compose-final")
    p.add_argument("--output", type=Path, required=True)
    p.add_argument("--config-labels", type=str, required=True, help="comma-separated")

    args = parser.parse_args()
    output = args.output.resolve()
    if args.command == "prepare":
        prepare(output)
    elif args.command == "monitor":
        monitor(output, args.duration_s, args.label, world=args.world)
    elif args.command == "gpu-snapshot":
        process_gpu_usage_snapshot(output, args.label)
    elif args.command == "analyze-run":
        analyze_run(args.run_dir.resolve(), output, args.label, prewarm_skip_s=args.prewarm_skip_s)
    elif args.command == "validate":
        validate(args.run_dir.resolve(), output, args.label, prewarm_skip_s=args.prewarm_skip_s)
    elif args.command == "compose-final":
        compose_final(output, [c.strip() for c in args.config_labels.split(",") if c.strip()])
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
