"""CORE_RANGE_RTF_STUTTER_ROOT_CAUSE_AND_MITIGATION event-aligned tracer.

Read-only observation sampler: one row per second (baseline cadence) of
Gazebo RTF (fresh-process `gz topic -e`, the same method already validated
across the accepted 8-group corpus -- NOT the persistent gz-transport
subscription used by core_range_gazebo_rtf_long_run_monitor.py, which an
earlier task flagged as showing volatility not confirmed by this method),
system resource metrics (reused unchanged from that module), and named
per-process metrics (RSS, CPU%, thread count, voluntary/involuntary
context switches, minor/major page faults). When RTF <= 0.3, the sampling
cadence for the surrounding event window (15s before .. 30s after) is
raised to 500ms, matching the task's own 200-500ms allowance.

Never starts/stops any process itself -- pure observation of whatever
stack the caller already launched.
"""

from __future__ import annotations

import argparse
import csv
import re
import signal
import subprocess
import sys
import time
from pathlib import Path
from typing import Any

from core_range_gazebo_rtf_long_run_monitor import (
    _read_cpu_freqs_mhz,
    _read_diskstats_sectors_written,
    _read_gpu_snapshot,
    _read_meminfo_mb,
    _read_proc_stat_ctxt,
    _read_thermal_zones_c,
)

_shutdown_requested = False


def _handle_shutdown(_signum: int, _frame: Any) -> None:
    global _shutdown_requested
    _shutdown_requested = True


TRACKED_PROCESS_PATTERNS = {
    "gz_sim": r"gz sim --verbose",
    "px4_uav_01": r"px4_sitl_default/bin/px4 -i 0",
    "px4_uav_02": r"px4_sitl_default/bin/px4 -i 1",
    "web_backend": r"uvicorn main:app",
    "microxrce": r"MicroXRCEAgent",
    "ros_telemetry": r"ros2 (launch|run)",
    "mavlink_bridge": r"mavlink_manual_bridge\.py",
    "logging_compression": r"bounded_log_writer\.py",
}

_CLOCK_TICKS = 100  # standard Linux USER_HZ; validated via os.sysconf if available
try:
    import os as _os

    _CLOCK_TICKS = _os.sysconf("SC_CLK_TCK")
except Exception:
    pass


def _find_pids(pattern: str) -> list[int]:
    completed = subprocess.run(["pgrep", "-f", pattern], capture_output=True, text=True, timeout=2)
    return [
        int(value) for value in completed.stdout.split()
        if value.strip() and int(value) != _os.getpid()
    ]


def _read_proc_stat_fields(pid: int) -> dict[str, float] | None:
    try:
        text = (Path(f"/proc/{pid}/stat")).read_text()
    except (FileNotFoundError, ProcessLookupError):
        return None
    # comm field may contain spaces/parens; split after the last ')'
    rparen = text.rfind(")")
    rest = text[rparen + 2:].split()
    utime, stime = float(rest[11]), float(rest[12])
    num_threads = float(rest[17])
    minflt, majflt = float(rest[7]), float(rest[9])
    return {"utime": utime, "stime": stime, "num_threads": num_threads, "minflt": minflt, "majflt": majflt}


def _read_proc_status_fields(pid: int) -> dict[str, float] | None:
    try:
        text = (Path(f"/proc/{pid}/status")).read_text()
    except (FileNotFoundError, ProcessLookupError):
        return None
    values: dict[str, float] = {}
    for line in text.splitlines():
        if line.startswith("VmRSS:"):
            values["rss_kb"] = float(line.split()[1])
        elif line.startswith("voluntary_ctxt_switches:"):
            values["vol_ctxsw"] = float(line.split()[1])
        elif line.startswith("nonvoluntary_ctxt_switches:"):
            values["nonvol_ctxsw"] = float(line.split()[1])
    return values


def _read_proc_io_fields(pid: int) -> dict[str, float] | None:
    try:
        text = Path(f"/proc/{pid}/io").read_text()
    except (FileNotFoundError, ProcessLookupError, PermissionError):
        return None
    values: dict[str, float] = {}
    for line in text.splitlines():
        key, _, raw = line.partition(":")
        if key in {"read_bytes", "write_bytes", "syscr", "syscw"}:
            values[key] = float(raw.strip())
    return values


def _sample_rtf(world: str) -> tuple[float | None, int | None, int | None]:
    try:
        completed = subprocess.run(
            ["timeout", "1", "gz", "topic", "-e", "-t", f"/world/{world}/stats", "-n", "1"],
            capture_output=True, text=True, timeout=2,
        )
    except Exception:
        return None, None, None
    text = completed.stdout
    rtf_match = re.search(r"real_time_factor:\s*([\d.eE+-]+)", text)
    iter_match = re.search(r"iterations:\s*(\d+)", text)
    model_match = re.search(r"model_count:\s*(\d+)", text)
    return (
        float(rtf_match.group(1)) if rtf_match else None,
        int(iter_match.group(1)) if iter_match else None,
        int(model_match.group(1)) if model_match else None,
    )


def _read_thread_cpu_ticks(pid: int) -> dict[int, float]:
    result: dict[int, float] = {}
    task_root = Path(f"/proc/{pid}/task")
    try:
        tids = list(task_root.iterdir())
    except (FileNotFoundError, ProcessLookupError, PermissionError):
        return result
    for entry in tids:
        try:
            text = (entry / "stat").read_text()
            rest = text[text.rfind(")") + 2:].split()
            result[int(entry.name)] = float(rest[11]) + float(rest[12])
        except (FileNotFoundError, ProcessLookupError, PermissionError, ValueError, IndexError):
            continue
    return result


def _percentile(values: list[float], fraction: float) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    index = min(len(ordered) - 1, max(0, int(round((len(ordered) - 1) * fraction))))
    return ordered[index]


def trace(output: Path, duration_s: float, world: str = "default", config_label: str = "") -> None:
    output.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = [
        "monotonic_ns", "wall_clock_s", "elapsed_s", "config_label", "cadence_s",
        "gazebo_rtf", "gazebo_iterations", "gazebo_iteration_delta",
        "gazebo_wall_ms_per_iteration", "gazebo_model_count",
        "cpu_freq_mean_mhz", "cpu_freq_max_mhz", "thermal_zone_max_c",
        "mem_available_mb", "mem_used_mb", "swap_used_mb",
        "disk_write_kb_per_s", "context_switches_per_s",
        "gpu_util_pct", "gpu_mem_used_mb", "gpu_clock_sm_mhz", "gpu_pstate", "gpu_power_w",
    ]
    for name in TRACKED_PROCESS_PATTERNS:
        fieldnames += [f"{name}_rss_mb", f"{name}_cpu_pct", f"{name}_threads",
                       f"{name}_thread_cpu_p95_pct", f"{name}_thread_cpu_max_pct",
                       f"{name}_vol_ctxsw_per_s", f"{name}_nonvol_ctxsw_per_s",
                       f"{name}_minflt_per_s", f"{name}_majflt_per_s",
                       f"{name}_read_kb_per_s", f"{name}_write_kb_per_s",
                       f"{name}_read_syscalls_per_s", f"{name}_write_syscalls_per_s",
                       f"{name}_pid_count"]

    trace_file = output.open("w", newline="")
    writer = csv.DictWriter(trace_file, fieldnames=fieldnames)
    writer.writeheader()
    trace_file.flush()

    signal.signal(signal.SIGTERM, _handle_shutdown)

    pids: dict[str, list[int]] = {name: [] for name in TRACKED_PROCESS_PATTERNS}
    previous: dict[tuple[str, int], dict[str, Any]] = {}
    previous_thread_ticks: dict[tuple[str, int, int], tuple[float, float]] = {}

    start = time.monotonic()
    start_wall = time.time()
    previous_ctxt = _read_proc_stat_ctxt()
    previous_sectors = _read_diskstats_sectors_written()
    previous_t = start
    last_dip_elapsed: float | None = None
    refresh_interval = 5.0
    last_pid_refresh = -999.0
    previous_iterations: int | None = None
    previous_iteration_time: float | None = None

    while time.monotonic() - start < duration_s and not _shutdown_requested:
        now = time.monotonic()
        elapsed = now - start

        in_event_window = last_dip_elapsed is not None and (elapsed - last_dip_elapsed) <= 30.0
        cadence = 0.5 if in_event_window else 1.0

        if elapsed - last_pid_refresh >= refresh_interval:
            for name, pattern in TRACKED_PROCESS_PATTERNS.items():
                pids[name] = _find_pids(pattern)
            last_pid_refresh = elapsed

        rtf, iterations, model_count = _sample_rtf(world)
        if rtf is not None and rtf <= 0.3:
            last_dip_elapsed = elapsed

        freqs = _read_cpu_freqs_mhz()
        thermal = _read_thermal_zones_c()
        mem = _read_meminfo_mb()
        ctxt = _read_proc_stat_ctxt()
        sectors = _read_diskstats_sectors_written()
        dt = max(now - previous_t, 1e-6)
        ctxt_rate = ((ctxt - previous_ctxt) / dt) if (ctxt is not None and previous_ctxt is not None) else None
        disk_rate = ((sectors - previous_sectors) * 0.5 / dt) if sectors is not None and previous_sectors is not None else None
        previous_ctxt, previous_sectors, previous_t = ctxt, sectors, now
        gpu = _read_gpu_snapshot()

        row: dict[str, Any] = {
            "monotonic_ns": time.monotonic_ns(),
            "wall_clock_s": round(start_wall + elapsed, 3), "elapsed_s": round(elapsed, 3),
            "config_label": config_label, "cadence_s": cadence,
            "gazebo_rtf": rtf, "gazebo_iterations": iterations,
            "gazebo_iteration_delta": (
                iterations - previous_iterations
                if iterations is not None and previous_iterations is not None
                else None
            ),
            "gazebo_wall_ms_per_iteration": (
                1000.0 * (now - previous_iteration_time) / (iterations - previous_iterations)
                if iterations is not None and previous_iterations is not None
                and previous_iteration_time is not None and iterations > previous_iterations
                else None
            ),
            "gazebo_model_count": model_count,
            "cpu_freq_mean_mhz": round(sum(freqs) / len(freqs), 1) if freqs else None,
            "cpu_freq_max_mhz": round(max(freqs), 1) if freqs else None,
            "thermal_zone_max_c": round(max(thermal), 1) if thermal else None,
            "mem_available_mb": round(mem.get("MemAvailable", 0.0), 1),
            "mem_used_mb": round(mem.get("MemTotal", 0.0) - mem.get("MemAvailable", 0.0), 1),
            "swap_used_mb": round(mem.get("SwapTotal", 0.0) - mem.get("SwapFree", 0.0), 1),
            "disk_write_kb_per_s": round(disk_rate, 1) if disk_rate is not None else None,
            "context_switches_per_s": round(ctxt_rate, 1) if ctxt_rate is not None else None,
            **{k: gpu.get(k) for k in ("gpu_util_pct", "gpu_mem_used_mb", "gpu_clock_sm_mhz", "gpu_pstate", "gpu_power_w")},
        }

        for name, process_ids in pids.items():
            totals = {key: 0.0 for key in (
                "rss_mb", "cpu_pct", "threads", "vol_ctxsw_per_s",
                "nonvol_ctxsw_per_s", "minflt_per_s", "majflt_per_s",
                "read_kb_per_s", "write_kb_per_s", "read_syscalls_per_s",
                "write_syscalls_per_s",
            )}
            rate_samples = 0
            valid_pids = 0
            thread_cpu_values: list[float] = []
            for pid in process_ids:
                stat = _read_proc_stat_fields(pid)
                status = _read_proc_status_fields(pid)
                proc_io = _read_proc_io_fields(pid)
                if stat is None or status is None:
                    continue
                valid_pids += 1
                for tid, ticks in _read_thread_cpu_ticks(pid).items():
                    thread_key = (name, pid, tid)
                    prior_thread = previous_thread_ticks.get(thread_key)
                    if prior_thread is not None:
                        prior_ticks, prior_time = prior_thread
                        thread_dt = max(now - prior_time, 1e-6)
                        thread_cpu_values.append(
                            max(0.0, 100.0 * (ticks - prior_ticks) / _CLOCK_TICKS / thread_dt)
                        )
                    previous_thread_ticks[thread_key] = (ticks, now)
                totals["rss_mb"] += status.get("rss_kb", 0.0) / 1024.0
                totals["threads"] += stat["num_threads"]
                key = (name, pid)
                prior = previous.get(key)
                if prior is not None:
                    proc_dt = max(now - float(prior["time"]), 1e-6)
                    prior_stat = prior["stat"]
                    prior_status = prior["status"]
                    totals["cpu_pct"] += 100.0 * (
                        (stat["utime"] + stat["stime"])
                        - (prior_stat["utime"] + prior_stat["stime"])
                    ) / _CLOCK_TICKS / proc_dt
                    for output_key, source_key, source in (
                        ("minflt_per_s", "minflt", stat),
                        ("majflt_per_s", "majflt", stat),
                        ("vol_ctxsw_per_s", "vol_ctxsw", status),
                        ("nonvol_ctxsw_per_s", "nonvol_ctxsw", status),
                    ):
                        prior_source = prior_stat if source is stat else prior_status
                        totals[output_key] += (
                            source.get(source_key, 0.0) - prior_source.get(source_key, 0.0)
                        ) / proc_dt
                    prior_io = prior.get("io")
                    if proc_io is not None and prior_io is not None:
                        for output_key, source_key, divisor in (
                            ("read_kb_per_s", "read_bytes", 1024.0),
                            ("write_kb_per_s", "write_bytes", 1024.0),
                            ("read_syscalls_per_s", "syscr", 1.0),
                            ("write_syscalls_per_s", "syscw", 1.0),
                        ):
                            totals[output_key] += (
                                proc_io.get(source_key, 0.0) - prior_io.get(source_key, 0.0)
                            ) / divisor / proc_dt
                    rate_samples += 1
                previous[key] = {
                    "time": now, "stat": stat, "status": status, "io": proc_io,
                }
            row[f"{name}_pid_count"] = valid_pids
            row[f"{name}_thread_cpu_p95_pct"] = (
                round(_percentile(thread_cpu_values, 0.95), 2)
                if thread_cpu_values else None
            )
            row[f"{name}_thread_cpu_max_pct"] = (
                round(max(thread_cpu_values), 2) if thread_cpu_values else None
            )
            for metric, value in totals.items():
                if metric in {"rss_mb", "threads"}:
                    row[f"{name}_{metric}"] = round(value, 2) if valid_pids else None
                else:
                    row[f"{name}_{metric}"] = round(value, 2) if rate_samples else None

        writer.writerow(row)
        trace_file.flush()
        if iterations is not None:
            previous_iterations = iterations
            previous_iteration_time = now
        time.sleep(max(0.0, cadence - (time.monotonic() - now)))

    trace_file.close()
    print(f"wrote trace to {output}")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--duration-s", type=float, required=True)
    parser.add_argument("--world", default="default")
    parser.add_argument("--config-label", default="")
    args = parser.parse_args()
    trace(args.output.resolve(), args.duration_s, world=args.world, config_label=args.config_label)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
