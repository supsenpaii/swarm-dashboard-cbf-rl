"""Per-second resource/RTF sampler for CORE_RANGE_GAZEBO_RTF_STUTTER_AND_
GATE_VALIDATION section 3 (long-run stutter root-cause profiling).

Read-only. Samples, once per second for --duration-s seconds:
  - Gazebo real_time_factor + iterations (sim step rate), via a persistent
    gz-transport subscription (not a fresh `gz topic -e` process per
    sample -- avoids the per-sample subscribe/discovery overhead noted as
    a risk in the camera-source-FPS task, though that task's burst test
    already showed the bimodal pattern is not a sampling artifact).
  - CPU: aggregate + per-core frequency (scaling_cur_freq, kHz) and
    thermal zone temperatures (no sudo required).
  - RAM/swap: /proc/meminfo.
  - Disk write throughput: /proc/diskstats delta (sectors written * 512).
  - Process/thread count: `ps -eL` row count.
  - Context switches: /proc/stat `ctxt` delta.
  - GPU utilization/memory/clocks: nvidia-smi (subprocess, 1 Hz).
  - Per-process CPU: pidstat (subprocess, 1 Hz), written to its own log
    and merged in post-processing rather than every-sample subprocess
    spawns.
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

_shutdown_requested = False


def _handle_shutdown_signal(_signum: int, _frame: Any) -> None:
    global _shutdown_requested
    _shutdown_requested = True

SYSTEM_DIST_PACKAGES = Path("/usr/lib/python3/dist-packages")
if SYSTEM_DIST_PACKAGES.exists() and str(SYSTEM_DIST_PACKAGES) not in sys.path:
    sys.path.append(str(SYSTEM_DIST_PACKAGES))

try:
    from gz.msgs10.world_stats_pb2 import WorldStatistics
    from gz.transport13 import Node as GzNode
except ImportError:
    WorldStatistics = None
    GzNode = None


def _read_cpu_freqs_mhz() -> list[float]:
    values = []
    for path in sorted(Path("/sys/devices/system/cpu").glob("cpu[0-9]*/cpufreq/scaling_cur_freq")):
        try:
            values.append(int(path.read_text().strip()) / 1000.0)
        except (OSError, ValueError):
            continue
    return values


def _read_thermal_zones_c() -> list[float]:
    values = []
    for path in sorted(Path("/sys/class/thermal").glob("thermal_zone*/temp")):
        try:
            values.append(int(path.read_text().strip()) / 1000.0)
        except (OSError, ValueError):
            continue
    return values


def _read_meminfo_mb() -> dict[str, float]:
    result: dict[str, float] = {}
    for line in Path("/proc/meminfo").read_text().splitlines():
        match = re.match(r"^(\w+):\s+(\d+)\s*kB", line)
        if match:
            result[match.group(1)] = int(match.group(2)) / 1024.0
    return result


def _read_proc_stat_ctxt() -> int | None:
    for line in Path("/proc/stat").read_text().splitlines():
        if line.startswith("ctxt"):
            return int(line.split()[1])
    return None


def _read_diskstats_sectors_written() -> int:
    total = 0
    for line in Path("/proc/diskstats").read_text().splitlines():
        parts = line.split()
        if len(parts) < 10:
            continue
        name = parts[2]
        if re.fullmatch(r"(nvme\d+n\d+|sd[a-z]+|loop\d+)", name):
            total += int(parts[9])
    return total


def _read_process_thread_count() -> int:
    try:
        completed = subprocess.run(["ps", "-eL", "--no-headers"], capture_output=True, text=True, timeout=5)
        return len(completed.stdout.splitlines())
    except Exception:
        return -1


def _read_gpu_snapshot() -> dict[str, Any]:
    try:
        completed = subprocess.run(
            ["nvidia-smi", "--query-gpu=utilization.gpu,memory.used,clocks.sm,pstate,power.draw",
             "--format=csv,noheader,nounits"],
            capture_output=True, text=True, timeout=5,
        )
        parts = [p.strip() for p in completed.stdout.strip().splitlines()[0].split(",")]
        return {
            "gpu_util_pct": float(parts[0]), "gpu_mem_used_mb": float(parts[1]),
            "gpu_clock_sm_mhz": float(parts[2]), "gpu_pstate": parts[3],
            "gpu_power_w": float(parts[4]) if parts[4] not in ("", "[N/A]") else None,
        }
    except Exception:
        return {}


class _RtfSubscriber:
    def __init__(self, world: str) -> None:
        self.latest_rtf: float | None = None
        self.latest_iterations: int | None = None
        self.latest_model_count: int | None = None
        self.samples: int = 0
        self._node = None
        self._world = world

    def _callback(self, message: Any) -> None:
        self.latest_rtf = float(message.real_time_factor)
        self.latest_iterations = int(message.iterations)
        self.latest_model_count = int(message.model_count)
        self.samples += 1

    def start(self) -> bool:
        if GzNode is None or WorldStatistics is None:
            return False
        self._node = GzNode()
        return bool(self._node.subscribe(WorldStatistics, f"/world/{self._world}/stats", self._callback))


def monitor(output: Path, duration_s: float, world: str = "default") -> None:
    output.mkdir(parents=True, exist_ok=True)
    trace_path = output / "long_run_resource_trace.csv"

    rtf_sub = _RtfSubscriber(world)
    rtf_ok = rtf_sub.start()
    if not rtf_ok:
        print("WARNING: persistent gz-transport RTF subscription unavailable; falling back to per-sample gz topic -e", file=sys.stderr)

    gpu_dmon_log = output / "long_run_nvidia_smi_dmon.log"
    pidstat_log = output / "long_run_pidstat.log"
    dmon_proc = subprocess.Popen(["nvidia-smi", "dmon", "-s", "pucv", "-d", "1"], stdout=gpu_dmon_log.open("w"), stderr=subprocess.STDOUT)
    pidstat_proc = subprocess.Popen(["pidstat", "-t", "-u", "-r", "-w", "1", str(int(duration_s) + 5)], stdout=pidstat_log.open("w"), stderr=subprocess.STDOUT)

    fieldnames = [
        "wall_clock_s", "elapsed_s", "gazebo_rtf", "gazebo_iterations", "gazebo_model_count",
        "cpu_freq_mean_mhz", "cpu_freq_max_mhz", "thermal_zone_max_c",
        "mem_available_mb", "mem_used_mb", "swap_used_mb",
        "disk_write_kb_per_s", "context_switches_per_s",
        "process_thread_count", "gpu_util_pct", "gpu_mem_used_mb",
        "gpu_clock_sm_mhz", "gpu_pstate", "gpu_power_w",
    ]
    # Written incrementally (one row + flush per sample), not buffered to
    # the end, because this process is killed with SIGTERM by the caller's
    # cleanup trap once the capture loop finishes -- an unhandled SIGTERM
    # does not run Python `finally` blocks, so a buffer-then-write-at-end
    # design would silently lose the entire trace on every normal shutdown.
    trace_file = trace_path.open("w", newline="")
    writer = csv.DictWriter(trace_file, fieldnames=fieldnames)
    writer.writeheader()
    trace_file.flush()
    row_count = 0
    start = time.monotonic()
    start_wall = time.time()
    previous_ctxt = _read_proc_stat_ctxt()
    previous_sectors = _read_diskstats_sectors_written()
    previous_t = start

    signal.signal(signal.SIGTERM, _handle_shutdown_signal)
    try:
        while time.monotonic() - start < duration_s and not _shutdown_requested:
            now = time.monotonic()
            elapsed = now - start
            if not rtf_ok:
                try:
                    completed = subprocess.run(
                        ["timeout", "1", "gz", "topic", "-e", "-t", f"/world/{world}/stats", "-n", "1"],
                        capture_output=True, text=True, timeout=2,
                    )
                    match = re.search(r"real_time_factor:\s*([\d.eE+-]+)", completed.stdout)
                    rtf_value = float(match.group(1)) if match else None
                    iter_match = re.search(r"iterations:\s*(\d+)", completed.stdout)
                    iterations = int(iter_match.group(1)) if iter_match else None
                    model_match = re.search(r"model_count:\s*(\d+)", completed.stdout)
                    model_count = int(model_match.group(1)) if model_match else None
                except Exception:
                    rtf_value, iterations, model_count = None, None, None
            else:
                rtf_value, iterations, model_count = rtf_sub.latest_rtf, rtf_sub.latest_iterations, rtf_sub.latest_model_count

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
            row = {
                "wall_clock_s": round(start_wall + elapsed, 3), "elapsed_s": round(elapsed, 3),
                "gazebo_rtf": rtf_value, "gazebo_iterations": iterations, "gazebo_model_count": model_count,
                "cpu_freq_mean_mhz": round(sum(freqs) / len(freqs), 1) if freqs else None,
                "cpu_freq_max_mhz": round(max(freqs), 1) if freqs else None,
                "thermal_zone_max_c": round(max(thermal), 1) if thermal else None,
                "mem_available_mb": round(mem.get("MemAvailable", 0.0), 1),
                "mem_used_mb": round(mem.get("MemTotal", 0.0) - mem.get("MemAvailable", 0.0), 1),
                "swap_used_mb": round(mem.get("SwapTotal", 0.0) - mem.get("SwapFree", 0.0), 1),
                "disk_write_kb_per_s": round(disk_rate, 1) if disk_rate is not None else None,
                "context_switches_per_s": round(ctxt_rate, 1) if ctxt_rate is not None else None,
                "process_thread_count": _read_process_thread_count() if int(elapsed) % 10 == 0 else None,
                **{k: gpu.get(k) for k in ("gpu_util_pct", "gpu_mem_used_mb", "gpu_clock_sm_mhz", "gpu_pstate", "gpu_power_w")},
            }
            writer.writerow(row)
            trace_file.flush()
            row_count += 1
            time.sleep(max(0.0, 1.0 - (time.monotonic() - now)))
    finally:
        trace_file.close()
        for proc in (dmon_proc, pidstat_proc):
            if proc.poll() is None:
                proc.terminate()
        for proc in (dmon_proc, pidstat_proc):
            try:
                proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                proc.kill()

    print(f"wrote {row_count} rows to {trace_path}")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--duration-s", type=float, required=True)
    parser.add_argument("--world", type=str, default="default")
    args = parser.parse_args()
    monitor(args.output.resolve(), args.duration_s, world=args.world)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
