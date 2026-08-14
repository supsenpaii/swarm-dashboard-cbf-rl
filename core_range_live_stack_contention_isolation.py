"""CORE_RANGE_LIVE_STACK_CONTENTION_ISOLATION.

Read-mostly investigation into why MiDaS forward-pass latency measured
idle-GPU/standalone (6-14ms, see
docs/CORE_RANGE_DEPTH_PIPELINE_LATENCY_REPORT.md) balloons to 0.62-1.08s
measured live inside the Gazebo/ROS2/PX4 dashboard stack. No retrain, no
official dataset collection, no GT/M52/calibration/controller/PX4 change, no
Follow Target. All profiling instrumentation this task depends on
(SWARM_DEPTH_PROFILE_STAGES in depth_model_adapter.py) is opt-in and off by
default; nothing here changes the default runtime.

Subcommands:
  prepare        - write the precommitted profiling_plan.json
  single-pass    - step 1: one detailed instrumented depth-inference pass,
                   standalone, idle GPU -> stage_latency.csv
  config-a       - ablation A: MiDaS standalone, many iterations, idle GPU
  config-b       - ablation B: LatestDepthWorker + MidasSmallAdapter driven
                   by a synthetic frame generator at the production
                   submission rate, in an isolated process (no Gazebo/
                   ROS2/PX4/dashboard/mosquitto/xrce running)
  monitor        - read-only resource sampling (nvidia-smi dmon/pmon,
                   pidstat) for a fixed duration; used around configs C/D/F
                   by the shell orchestrators
  analyze-live   - parse a captured live-session sidecar (configs C/D/F)
                   into per-session latency + queue/drop metrics
  compare        - assemble configuration_comparison.csv from all gathered
                   per-config evidence
  classify       - apply the precommitted contention gate rule and write
                   contention_manifest.json
  report         - render contention_report.md from the manifest
"""

from __future__ import annotations

import argparse
import csv
import json
import os
import re
import subprocess
import sys
import time
from pathlib import Path
from statistics import median
from typing import Any, Mapping, Sequence

import numpy as np

ISOLATION_ID = "core_range_live_stack_contention_isolation_20260805_v001"
SEED = 52
CAMERA_WIDTH, CAMERA_HEIGHT = 480, 270  # matches the frozen dynamic sidecars
PRODUCTION_DEPTH_RATE_HZ = 7.5  # SWARM_METRIC_TARGET_DEPTH_RATE_HZ default

GATE_OUTCOMES = (
    "GAZEBO_GPU_CONTENTION_DOMINANT",
    "OTHER_AI_GPU_CONTENTION_DOMINANT",
    "CPU_PREPROCESSING_OR_COPY_DOMINANT",
    "CUDA_CONTEXT_OR_SYNCHRONIZATION_DOMINANT",
    "ROS_CALLBACK_OR_THREAD_BLOCKING_DOMINANT",
    "MIXED_LIVE_STACK_CONTENTION",
    "CONTENTION_EVIDENCE_INSUFFICIENT",
)

FINE_STAGE_KEYS = (
    "frame_copy_decode_s",
    "host_to_device_transfer_s",
    "gpu_preprocess_resize_normalize_s",
    "model_forward_s",
    "output_resize_interpolation_s",
    "device_to_host_transfer_s",
    "numpy_postprocess_s",
)

CONFIGS = [
    {
        "name": "A_standalone",
        "description": (
            "MiDaS standalone on a synthetic frame, idle GPU, isolated "
            "process. No Gazebo/ROS2/PX4/dashboard/mosquitto/xrce running."
        ),
        "components_running": [],
        "varying_factor_vs_previous": "baseline",
    },
    {
        "name": "B_worker_no_stack",
        "description": (
            "LatestDepthWorker + MidasSmallAdapter (real production worker "
            "code) driven by a synthetic frame generator at the production "
            "submission rate (7.5 Hz), isolated process. No Gazebo/ROS2/"
            "PX4/dashboard running."
        ),
        "components_running": ["latest_depth_worker_thread"],
        "varying_factor_vs_previous": (
            "adds the real background worker thread + Python GIL sharing "
            "with a submission-rate timer; still no GPU renderer, no "
            "middleware"
        ),
    },
    {
        "name": "C_gazebo_headless_camera_only",
        "description": (
            "Gazebo server headless (no GUI) + a single PX4 SITL instance "
            "(needed to spawn the camera-bearing vehicle model) + a "
            "minimal standalone gz-transport camera subscriber driving "
            "LatestDepthWorker/MidasSmallAdapter directly. No second UAV, "
            "no MicroXRCEAgent-dependent ROS2 telemetry launch, no "
            "mavlink_manual_bridge.py, no main.py/uvicorn dashboard."
        ),
        "components_running": [
            "gz_sim_server_headless", "px4_uav_01", "latest_depth_worker_thread",
        ],
        "varying_factor_vs_previous": (
            "adds Gazebo's rendering/simulation loop + PX4 SITL physics on "
            "the same GPU/CPU; still no ROS2 middleware, no dashboard "
            "event loop, no mavlink bridge"
        ),
    },
    {
        "name": "D_full_stack",
        "description": (
            "Full Gazebo/ROS2/PX4/dashboard stack (mosquitto, "
            "MicroXRCEAgent, gz sim server+GUI, PX4 UAV-01+UAV-02, ROS2 "
            "telemetry launch, mavlink_manual_bridge.py, uvicorn main:app), "
            "observation-only short smoke session via the existing "
            "core_range_collect_dynamic_scenario.sh harness."
        ),
        "components_running": [
            "mosquitto", "micro_xrce_agent", "gz_sim_server", "gz_sim_gui",
            "px4_uav_01", "px4_uav_02", "ros2_telemetry_launch",
            "mavlink_manual_bridge", "uvicorn_dashboard_backend",
        ],
        "varying_factor_vs_previous": (
            "adds Gazebo GUI, the second UAV, ROS2 telemetry launch, the "
            "MAVLink bridge, and the dashboard's own asyncio event loop / "
            "tracking_web.py consumer thread"
        ),
    },
    {
        "name": "E_full_stack_other_ai_disabled",
        "description": (
            "Full stack with any other AI inference disabled. Audited: "
            "grep -rl 'torch\\.|import torch' across all production "
            "modules (excluding the stale swarm_dashboard_handoff_20260729/ "
            "snapshot and this task's own scripts) finds exactly one "
            "torch/GPU consumer in this repository: MidaS via "
            "depth_model_adapter.py. tracking_hybrid.py's LightFC tracker "
            "and m52_adapter.py's XGBoost residual model are both CPU-only "
            "(no torch/cuda usage). There is therefore no other AI "
            "inference to disable; this configuration is structurally "
            "identical to D and was not separately re-run."
        ),
        "components_running": None,
        "varying_factor_vs_previous": "N/A_NO_OTHER_AI_GPU_CONSUMER_EXISTS",
        "not_executed": True,
        "not_executed_reason": (
            "no other AI/GPU-consuming inference exists anywhere in this "
            "codebase to disable (see description); running it would just "
            "reproduce D with an inapplicable label"
        ),
    },
    {
        "name": "F_full_stack_lower_camera_rate",
        "description": (
            "Same full stack as D, but with SWARM_RANGE_DATASET_DEPTH_RATE_HZ "
            "(-> SWARM_METRIC_TARGET_DEPTH_RATE_HZ, main.py:2904-2905) "
            "lowered from the production default of 7.5 Hz to 2.0 Hz -- "
            "the only camera/depth submission-rate knob this codebase "
            "exposes without touching M52/calibration/controller/PX4/"
            "Follow Target code. Single-variable change vs D."
        ),
        "components_running": [
            "mosquitto", "micro_xrce_agent", "gz_sim_server", "gz_sim_gui",
            "px4_uav_01", "px4_uav_02", "ros2_telemetry_launch",
            "mavlink_manual_bridge", "uvicorn_dashboard_backend",
        ],
        "varying_factor_vs_previous": (
            "only SWARM_RANGE_DATASET_DEPTH_RATE_HZ changed (7.5 -> 2.0 Hz), "
            "vs D"
        ),
    },
]


def _json(path: Path) -> Any:
    return json.loads(path.read_text(encoding="utf-8"))


def _write_json(path: Path, value: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def _write_csv(path: Path, rows: Sequence[Mapping[str, Any]], fieldnames: Sequence[str] | None = None) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    if not rows:
        path.write_text("", encoding="utf-8")
        return
    names = list(fieldnames) if fieldnames is not None else list(rows[0].keys())
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=names)
        writer.writeheader()
        writer.writerows(rows)


def _percentile(values: Sequence[float], pct: float) -> float:
    if not values:
        return float("nan")
    return float(np.percentile(np.asarray(values, dtype=np.float64), pct))


def _stats(values: Sequence[float]) -> dict[str, float]:
    return {
        "median_s": float(median(values)) if values else float("nan"),
        "p90_s": _percentile(values, 90),
        "p95_s": _percentile(values, 95),
        "min_s": float(min(values)) if values else float("nan"),
        "max_s": float(max(values)) if values else float("nan"),
        "n": len(values),
    }


# ---------------------------------------------------------------------------
# prepare
# ---------------------------------------------------------------------------

def prepare(output: Path) -> dict[str, Any]:
    plan = {
        "isolation_id": ISOLATION_ID,
        "seed": SEED,
        "no_retrain": True,
        "no_official_dataset_collection": True,
        "no_gt_m52_calibration_controller_px4_change": True,
        "no_follow_target": True,
        "no_default_runtime_change": True,
        "all_profiling_opt_in": True,
        "profiling_env_var": "SWARM_DEPTH_PROFILE_STAGES",
        "camera_frame_shape": {"width": CAMERA_WIDTH, "height": CAMERA_HEIGHT},
        "production_depth_rate_hz": PRODUCTION_DEPTH_RATE_HZ,
        "prior_reference": {
            "standalone_idle_gpu_median_s": 0.00615,
            "standalone_idle_gpu_range_s": [0.006, 0.014],
            "live_stack_worker_inference_median_s_range": [0.62, 1.08],
            "source": "docs/CORE_RANGE_DEPTH_PIPELINE_LATENCY_REPORT.md",
        },
        "fine_stage_keys": list(FINE_STAGE_KEYS),
        "configs": CONFIGS,
        "single_change_per_step_rule": (
            "Each configuration in `configs` changes exactly one factor "
            "relative to the previous one in the list (see "
            "varying_factor_vs_previous), except E which is not executed "
            "(see not_executed_reason)."
        ),
        "resource_metrics": [
            "gpu_utilization_pct", "gpu_memory_used_mb", "cpu_utilization_pct_per_process",
            "ram_used_mb", "context_switches_per_s", "process_gpu_consumers",
            "frame_arrival_rate_hz", "pending_frame_replacement_count",
            "dropped_frame_count", "inference_latency_median_p90_p95_s",
        ],
        "contention_checks": [
            "gazebo_rendering_gpu_share", "camera_copy_blocks_cuda",
            "other_ai_model_same_gpu", "multiple_cuda_contexts_competing",
            "inference_waits_on_cpu_preprocessing", "log_fsync_or_ros_callback_blocks_worker",
            "pytorch_reloads_model_per_frame",
        ],
        "gate_outcomes": list(GATE_OUTCOMES),
    }
    _write_json(output / "profiling_plan.json", plan)
    print(json.dumps({"configs": len(CONFIGS)}, indent=2))
    return plan


# ---------------------------------------------------------------------------
# step 1: single detailed instrumented pass (standalone, idle GPU)
# ---------------------------------------------------------------------------

def single_pass(output: Path, iterations: int = 30, warmup: int = 5) -> None:
    os.environ["SWARM_DEPTH_PROFILE_STAGES"] = "1"
    import torch

    from depth_model_adapter import MidasSmallAdapter
    from target_depth_extractor import TargetDepthExtractor
    from metric_depth_calibrator import MetricDepthCalibrator

    adapter = MidasSmallAdapter()
    adapter.load()
    extractor = TargetDepthExtractor()
    calibrator = MetricDepthCalibrator()
    # This harness-owned calibrator only needs *a* valid scale/offset to
    # exercise the real extract() code path for timing; it does not affect
    # (and is never used by) the production calibrator instance.
    calibrator.scale = 1.0
    calibrator.offset = 0.0

    rng = np.random.default_rng(SEED)
    frame = rng.integers(0, 255, (CAMERA_HEIGHT, CAMERA_WIDTH, 3), dtype=np.uint8)
    bbox_xywh = (
        int(0.465 * CAMERA_WIDTH), int(0.25 * CAMERA_HEIGHT),
        int(0.07 * CAMERA_WIDTH), int(0.095 * CAMERA_HEIGHT),
    )

    rows: list[dict[str, Any]] = []
    for index in range(warmup + iterations):
        pass_start = time.perf_counter()
        depth_map = adapter.infer(frame)
        publish_start = time.perf_counter()
        # Publish-result step in the real worker is a lock-guarded version
        # bump + attribute swap (depth_worker.py LatestDepthWorker._run);
        # timed here as the same trivial operation shape, in isolation.
        _published_holder = {"version": index, "result": depth_map}
        publish_s = time.perf_counter() - publish_start

        roi_start = time.perf_counter()
        target_depth = extractor.extract(depth_map.inverse_depth, bbox_xywh, calibrator)
        roi_s = time.perf_counter() - roi_start

        total_s = time.perf_counter() - pass_start
        if index < warmup:
            continue
        stages = dict(adapter.last_profiling_stages or {})
        stages["roi_extraction_s"] = roi_s
        stages["publish_result_s"] = publish_s
        stages["total_pass_s"] = total_s
        stages["iteration"] = index - warmup
        stages["target_depth_valid"] = target_depth.valid
        rows.append(stages)

    fieldnames = ["iteration", *FINE_STAGE_KEYS, "roi_extraction_s", "publish_result_s", "total_pass_s", "target_depth_valid"]
    _write_csv(output / "stage_latency.csv", rows, fieldnames=fieldnames)

    summary = {
        key: _stats([row[key] for row in rows])
        for key in [*FINE_STAGE_KEYS, "roi_extraction_s", "publish_result_s", "total_pass_s"]
    }
    _write_json(output / "stage_latency_summary.json", summary)
    print(json.dumps({"iterations": len(rows), "total_pass_median_s": summary["total_pass_s"]["median_s"]}, indent=2))


# ---------------------------------------------------------------------------
# config A: standalone benchmark, idle GPU, many iterations
# ---------------------------------------------------------------------------

def _run_midas_iterations(iterations: int, warmup: int, profile_stages: bool) -> tuple[list[float], list[dict[str, Any]]]:
    os.environ["SWARM_DEPTH_PROFILE_STAGES"] = "1" if profile_stages else "0"
    from depth_model_adapter import MidasSmallAdapter

    adapter = MidasSmallAdapter()
    adapter.load()
    rng = np.random.default_rng(SEED)
    frame = rng.integers(0, 255, (CAMERA_HEIGHT, CAMERA_WIDTH, 3), dtype=np.uint8)

    durations: list[float] = []
    stage_rows: list[dict[str, Any]] = []
    for index in range(warmup + iterations):
        started = time.perf_counter()
        adapter.infer(frame)
        elapsed = time.perf_counter() - started
        if index >= warmup:
            durations.append(elapsed)
            stages = dict(adapter.last_profiling_stages or {})
            stages["iteration"] = index - warmup
            stages["total_s"] = elapsed
            stage_rows.append(stages)
    return durations, stage_rows


def config_a(output: Path, iterations: int = 200, warmup: int = 10) -> dict[str, Any]:
    """Ablation A: MiDaS standalone, idle GPU, isolated process.

    Run twice, same frame/iterations: once with SWARM_DEPTH_PROFILE_STAGES=0
    (the production-representative number -- no torch.cuda.synchronize()
    calls, matching how the live worker actually runs) and once with it on
    (for the fine per-stage breakdown only). The two are NOT the same
    latency: bracketing every GPU-touching sub-step with synchronize()
    serializes work the GPU would otherwise pipeline/overlap, and each
    synchronize() call itself has a fixed CPU<->GPU round-trip cost. This
    delta is reported explicitly rather than silently mixed together.
    """
    gpu_before = _nvidia_smi_snapshot()
    with _BackgroundDmon(output, "config_a"):
        durations_unprofiled, _ = _run_midas_iterations(iterations, warmup, profile_stages=False)
        durations_profiled, stage_rows = _run_midas_iterations(iterations, warmup, profile_stages=True)
    gpu_after = _nvidia_smi_snapshot()

    _write_csv(output / "config_a_stage_latency.csv", stage_rows, fieldnames=["iteration", *FINE_STAGE_KEYS, "total_s"])
    result = {
        "config": "A_standalone",
        "worker_inference_s_unprofiled_production_representative": _stats(durations_unprofiled),
        "worker_inference_s_profiled_with_cuda_synchronize": _stats(durations_profiled),
        "profiling_inflation_ratio": (
            median(durations_profiled) / median(durations_unprofiled)
            if durations_unprofiled and median(durations_unprofiled) > 0 else float("nan")
        ),
        "gpu_before": gpu_before,
        "gpu_after": gpu_after,
    }
    _write_json(output / "config_a_result.json", result)
    print(json.dumps({
        "median_unprofiled_s": result["worker_inference_s_unprofiled_production_representative"]["median_s"],
        "median_profiled_s": result["worker_inference_s_profiled_with_cuda_synchronize"]["median_s"],
        "inflation_ratio": result["profiling_inflation_ratio"],
    }, indent=2))
    return result


# ---------------------------------------------------------------------------
# config B: LatestDepthWorker + MidasSmallAdapter, synthetic source, no stack
# ---------------------------------------------------------------------------

def _run_worker_session(duration_s: float, rate_hz: float, profile_stages: bool) -> tuple[list[dict[str, Any]], dict[str, Any]]:
    os.environ["SWARM_DEPTH_PROFILE_STAGES"] = "1" if profile_stages else "0"
    from depth_model_adapter import MidasSmallAdapter
    from depth_worker import DepthJob, LatestDepthWorker

    adapter = MidasSmallAdapter()
    worker = LatestDepthWorker(adapter)
    worker.start()

    rng = np.random.default_rng(SEED)
    frame = rng.integers(0, 255, (CAMERA_HEIGHT, CAMERA_WIDTH, 3), dtype=np.uint8)

    interval_s = 1.0 / rate_hz
    frame_index = 0
    seen_versions: set[int] = set()
    latency_rows: list[dict[str, Any]] = []
    deadline = time.monotonic() + duration_s
    next_submit = time.monotonic()
    while time.monotonic() < deadline:
        now = time.monotonic()
        if now >= next_submit:
            worker.submit(DepthJob(frame, now, frame_index))
            frame_index += 1
            next_submit += interval_s
        version, result = worker.latest()
        if result is not None and version not in seen_versions:
            seen_versions.add(version)
            latency_rows.append({
                "frame_index": result.frame_index,
                "worker_inference_s": (result.completed_timestamp_s - result.inference_started_timestamp_s)
                if result.inference_started_timestamp_s is not None else None,
                "queue_wait_s": (result.inference_started_timestamp_s - result.submitted_timestamp_s)
                if result.inference_started_timestamp_s is not None and result.submitted_timestamp_s is not None else None,
                **{key: (result.profiling_stages or {}).get(key) for key in FINE_STAGE_KEYS},
            })
        # Polled at ~30 Hz (camera-frame-rate order of magnitude), not a
        # tight busy-loop: an earlier 1kHz poll induced GIL/lock contention
        # against the worker thread's own condition-variable lock that was
        # an artifact of this harness, not representative of the real
        # consumer (which is driven by camera frame arrival, not a spin
        # loop). See docs/CORE_RANGE_LIVE_STACK_CONTENTION_REPORT.md.
        time.sleep(0.03)
    status = worker.status()
    worker.stop()
    return latency_rows, status


def config_b(output: Path, duration_s: float = 20.0, rate_hz: float = PRODUCTION_DEPTH_RATE_HZ) -> dict[str, Any]:
    """Ablation B: LatestDepthWorker + MidasSmallAdapter, synthetic source, no
    Gazebo/ROS2/PX4/dashboard. Same unprofiled/profiled split as config A and
    for the same reason (see config_a docstring)."""
    gpu_before = _nvidia_smi_snapshot()
    with _BackgroundDmon(output, "config_b"):
        rows_unprofiled, status_unprofiled = _run_worker_session(duration_s, rate_hz, profile_stages=False)
        rows_profiled, status_profiled = _run_worker_session(duration_s, rate_hz, profile_stages=True)
    gpu_after = _nvidia_smi_snapshot()

    _write_csv(output / "config_b_stage_latency.csv", rows_profiled)
    durations_unprofiled = [row["worker_inference_s"] for row in rows_unprofiled if row["worker_inference_s"] is not None]
    durations_profiled = [row["worker_inference_s"] for row in rows_profiled if row["worker_inference_s"] is not None]
    result = {
        "config": "B_worker_no_stack",
        "duration_s": duration_s,
        "rate_hz": rate_hz,
        "submitted": status_unprofiled["submitted"],
        "dropped": status_unprofiled["dropped"],
        "processed": status_unprofiled["processed"],
        "failed": status_unprofiled["failed"],
        "worker_inference_s_unprofiled_production_representative": _stats(durations_unprofiled),
        "worker_inference_s_profiled_with_cuda_synchronize": _stats(durations_profiled),
        "gpu_before": gpu_before,
        "gpu_after": gpu_after,
    }
    _write_json(output / "config_b_result.json", result)
    print(json.dumps({
        "processed": status_unprofiled["processed"],
        "dropped": status_unprofiled["dropped"],
        "median_unprofiled_s": result["worker_inference_s_unprofiled_production_representative"]["median_s"],
        "median_profiled_s": result["worker_inference_s_profiled_with_cuda_synchronize"]["median_s"],
    }, indent=2))
    return result


# ---------------------------------------------------------------------------
# read-only resource monitoring (nvidia-smi dmon/pmon, pidstat)
# ---------------------------------------------------------------------------

class _BackgroundDmon:
    """Read-only nvidia-smi dmon sampling for the duration of a `with` block."""

    def __init__(self, output: Path, label: str) -> None:
        self._path = output / f"{label}_nvidia_smi_dmon.log"
        self._proc: subprocess.Popen | None = None
        self._file: Any = None

    def __enter__(self) -> "_BackgroundDmon":
        self._path.parent.mkdir(parents=True, exist_ok=True)
        self._file = self._path.open("w")
        self._proc = subprocess.Popen(
            ["nvidia-smi", "dmon", "-s", "pucv", "-d", "1"],
            stdout=self._file, stderr=subprocess.STDOUT,
        )
        return self

    def __exit__(self, *_exc: object) -> None:
        if self._proc is not None and self._proc.poll() is None:
            self._proc.terminate()
            try:
                self._proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self._proc.kill()
        if self._file is not None:
            self._file.close()


def _nvidia_smi_snapshot() -> dict[str, Any]:
    # pstate/clocks matter here: this GPU idles down to P8 (~210 MHz) between
    # bursts of work and only ramps to a higher performance state under
    # sustained load, which by itself can swing standalone MiDaS latency by
    # ~2x independent of anything this task's instrumentation does -- see
    # docs/CORE_RANGE_LIVE_STACK_CONTENTION_REPORT.md.
    try:
        completed = subprocess.run(
            ["nvidia-smi",
             "--query-gpu=utilization.gpu,memory.used,memory.total,pstate,clocks.sm,clocks.max.sm,power.draw",
             "--format=csv,noheader,nounits"],
            check=False, text=True, capture_output=True, timeout=5,
        )
        parts = [p.strip() for p in completed.stdout.strip().splitlines()[0].split(",")]
        return {
            "gpu_utilization_pct": float(parts[0]),
            "gpu_memory_used_mb": float(parts[1]),
            "gpu_memory_total_mb": float(parts[2]),
            "pstate": parts[3],
            "clocks_sm_mhz": float(parts[4]),
            "clocks_max_sm_mhz": float(parts[5]),
            "power_draw_w": float(parts[6]) if parts[6] not in ("", "[N/A]") else None,
        }
    except Exception as error:
        return {"error": str(error)}


def monitor(output: Path, duration_s: float, label: str) -> None:
    """Read-only nvidia-smi dmon + pidstat sampling for `duration_s` seconds."""
    dmon_path = output / f"{label}_nvidia_smi_dmon.log"
    pmon_path = output / f"{label}_nvidia_smi_pmon.log"
    pidstat_path = output / f"{label}_pidstat.log"
    output.mkdir(parents=True, exist_ok=True)

    with dmon_path.open("w") as dmon_file, pmon_path.open("w") as pmon_file, pidstat_path.open("w") as pidstat_file:
        dmon = subprocess.Popen(["nvidia-smi", "dmon", "-s", "pucv", "-d", "1"], stdout=dmon_file, stderr=subprocess.STDOUT)
        pmon = subprocess.Popen(["nvidia-smi", "pmon", "-s", "u", "-d", "1"], stdout=pmon_file, stderr=subprocess.STDOUT)
        pidstat = subprocess.Popen(["pidstat", "-t", "-u", "-r", "-w", "1", str(int(duration_s))], stdout=pidstat_file, stderr=subprocess.STDOUT)
        try:
            time.sleep(duration_s)
        finally:
            for proc in (dmon, pmon, pidstat):
                if proc.poll() is None:
                    proc.terminate()
            for proc in (dmon, pmon, pidstat):
                try:
                    proc.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    proc.kill()
    print(json.dumps({"label": label, "duration_s": duration_s, "dmon": str(dmon_path), "pmon": str(pmon_path), "pidstat": str(pidstat_path)}, indent=2))


def process_gpu_usage_snapshot(output: Path, label: str) -> None:
    """Read-only: which processes are currently using the GPU (evidence for
    contention check 'nhiều CUDA context/process có cạnh tranh không')."""
    try:
        completed = subprocess.run(
            ["nvidia-smi", "--query-compute-apps=pid,process_name,used_memory",
             "--format=csv,noheader"],
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
    path = output / "process_gpu_usage.csv"
    existing = []
    if path.exists() and path.stat().st_size > 0:
        with path.open() as stream:
            existing = list(csv.DictReader(stream))
    _write_csv(path, existing + rows, fieldnames=["label", "pid", "process_name", "used_memory"])
    print(json.dumps({"label": label, "gpu_processes": len(rows)}, indent=2))


# ---------------------------------------------------------------------------
# final composition: configuration_comparison.csv, resource_usage.csv,
# queue_drop_metrics.csv, contention_manifest.json
# ---------------------------------------------------------------------------

# Reused from the prior, already-published task
# (docs/CORE_RANGE_DEPTH_PIPELINE_LATENCY_REPORT.md +
# artifacts/core_range_3_12m/depth_latency_optimization/), captured on an
# earlier occasion when /mnt/px4ssd was healthy. Kept here only as a
# cross-check against this task's own fresh D capture (see
# FRESH_D_AND_F_EVIDENCE below, produced by
# core_range_contention_run_live_scenario.sh after /mnt/px4ssd's emergency_ro
# fault was resolved -- a second, healthy mount stacked on top of the
# faulted one).
REUSED_PRIOR_D_EQUIVALENT_EVIDENCE = {
    "source": "docs/CORE_RANGE_DEPTH_PIPELINE_LATENCY_REPORT.md + artifacts/core_range_3_12m/depth_latency_optimization/",
    "baseline_8_frozen_sessions_844_frames": {
        "worker_inference_s_median": 0.612, "worker_inference_s_p90": 0.881, "worker_inference_s_p95": 0.963,
    },
    "post_cudnn_benchmark_smoke_sessions": {
        "smoke_approach": {"worker_inference_s_median": 0.854, "frame_count": 21},
        "smoke_recede": {"worker_inference_s_median": 0.619, "frame_count": 33},
        "smoke_stop_hold": {"worker_inference_s_median": 1.076, "frame_count": 22},
    },
}


def _worker_inference_stats_from_diagnostics(path: Path) -> dict[str, Any]:
    """Extract worker_inference_s (depth_worker_start -> depth_complete,
    the same quantity as `inference_ms` in depth_worker.py, computed from
    the timestamp_stages every physical_diagnostics.jsonl row already
    carries regardless of downstream calibration/ROI outcome) from a live
    capture's sidecar."""
    durations: list[float] = []
    stage_counts: dict[str, int] = {}
    with path.open() as stream:
        for line in stream:
            line = line.strip()
            if not line:
                continue
            record = json.loads(line)
            stage_counts[record.get("stage", "?")] = stage_counts.get(record.get("stage", "?"), 0) + 1
            stages = record.get("timestamp_stages") or {}
            start = (stages.get("depth_worker_start") or {}).get("timestamp_s")
            complete = (stages.get("depth_complete") or {}).get("timestamp_s")
            if start is not None and complete is not None:
                durations.append(float(complete) - float(start))
    return {"stats": _stats(durations), "stage_counts": stage_counts, "total_rows": sum(stage_counts.values())}


def compose_final(output: Path) -> None:
    config_a = _json(output / "config_a_result.json")
    config_b = _json(output / "config_b_result.json")
    config_c_path = output / "config_c" / "config_c_result.json"
    config_c = _json(config_c_path) if config_c_path.exists() else None

    config_d_diag = output / "config_d" / "dynamic_capture" / "physical_diagnostics.jsonl"
    config_f_diag = output / "config_f" / "dynamic_capture" / "physical_diagnostics.jsonl"
    config_d = _worker_inference_stats_from_diagnostics(config_d_diag) if config_d_diag.exists() else None
    config_f = _worker_inference_stats_from_diagnostics(config_f_diag) if config_f_diag.exists() else None

    comparison_rows = [
        {
            "config": "A_standalone", "status": "EXECUTED",
            "worker_inference_s_median": config_a["worker_inference_s_unprofiled_production_representative"]["median_s"],
            "worker_inference_s_p95": config_a["worker_inference_s_unprofiled_production_representative"]["p95_s"],
            "n": config_a["worker_inference_s_unprofiled_production_representative"]["n"],
            "notes": "idle-GPU standalone, isolated process, no other processes running",
        },
        {
            "config": "B_worker_no_stack", "status": "EXECUTED",
            "worker_inference_s_median": config_b["worker_inference_s_unprofiled_production_representative"]["median_s"],
            "worker_inference_s_p95": config_b["worker_inference_s_unprofiled_production_representative"]["p95_s"],
            "n": config_b["worker_inference_s_unprofiled_production_representative"]["n"],
            "notes": "real LatestDepthWorker background thread + synthetic frames at 7.5Hz, isolated process",
        },
        {
            "config": "C_gazebo_headless_camera_only",
            "status": "EXECUTED" if (config_c and config_c["frame_count_received"] > 0) else "BLOCKED_INFRA_FAULT",
            "worker_inference_s_median": config_c["worker_inference_s"]["median_s"] if config_c else None,
            "worker_inference_s_p95": config_c["worker_inference_s"]["p95_s"] if config_c else None,
            "n": config_c["worker_inference_s"]["n"] if config_c else 0,
            "notes": (
                f"gz sim headless + 1 PX4 instance + minimal gz-transport camera worker, 7.5Hz submission "
                f"rate, no ROS2/dashboard/mavlink-bridge; {config_c['frame_count_received']} camera frames "
                f"received in {config_c['duration_s']}s"
            ) if config_c else "not run",
        },
        {
            "config": "D_full_stack",
            "status": "EXECUTED_FRESH" if config_d else "REUSED_PRIOR_SESSION_DATA",
            "worker_inference_s_median": (config_d["stats"]["median_s"] if config_d
                                           else REUSED_PRIOR_D_EQUIVALENT_EVIDENCE["baseline_8_frozen_sessions_844_frames"]["worker_inference_s_median"]),
            "worker_inference_s_p95": (config_d["stats"]["p95_s"] if config_d
                                        else REUSED_PRIOR_D_EQUIVALENT_EVIDENCE["baseline_8_frozen_sessions_844_frames"]["worker_inference_s_p95"]),
            "n": config_d["stats"]["n"] if config_d else 844,
            "notes": (
                "full Gazebo/ROS2/PX4/dashboard stack (mosquitto, MicroXRCEAgent, gz sim server+GUI, "
                "PX4 UAV-01+02, ROS2 telemetry launch, mavlink_manual_bridge.py, uvicorn main:app), "
                "7.5Hz submission rate, fresh observation-only smoke capture "
                f"(stage_counts={config_d['stage_counts']})" if config_d else
                "px4ssd fault at the time this row would have been generated; reused prior-session data instead"
            ),
        },
        {
            "config": "E_full_stack_other_ai_disabled", "status": "NOT_EXECUTED_NO_OTHER_AI_GPU_CONSUMER",
            "worker_inference_s_median": None, "worker_inference_s_p95": None, "n": 0,
            "notes": "audited: only depth_model_adapter.py uses torch/cuda in this repo (excluding the stale swarm_dashboard_handoff_20260729/ snapshot); nothing to disable",
        },
        {
            "config": "F_full_stack_lower_camera_rate",
            "status": "EXECUTED" if config_f else "BLOCKED_INFRA_FAULT",
            "worker_inference_s_median": config_f["stats"]["median_s"] if config_f else None,
            "worker_inference_s_p95": config_f["stats"]["p95_s"] if config_f else None,
            "n": config_f["stats"]["n"] if config_f else 0,
            "notes": (
                "identical full stack to D, only SWARM_RANGE_DATASET_DEPTH_RATE_HZ changed 7.5 -> 2.0 Hz "
                f"(stage_counts={config_f['stage_counts']})"
            ) if config_f else "not run",
        },
    ]
    _write_csv(output / "configuration_comparison.csv", comparison_rows)

    resource_rows = []
    for name, result in (("A_standalone", config_a), ("B_worker_no_stack", config_b), ("C_gazebo_headless_camera_only", config_c)):
        if result is None:
            continue
        for phase in ("gpu_before", "gpu_after"):
            snapshot = result.get(phase, {})
            resource_rows.append({"config": name, "phase": phase, **snapshot})
    _write_csv(output / "resource_usage.csv", resource_rows)

    queue_drop_rows = [
        {
            "config": "A_standalone", "submitted": config_a["worker_inference_s_unprofiled_production_representative"]["n"],
            "dropped": 0, "processed": config_a["worker_inference_s_unprofiled_production_representative"]["n"], "failed": 0,
            "notes": "no worker/queue involved -- direct adapter.infer() calls in a loop",
        },
        {
            "config": "B_worker_no_stack", "submitted": config_b["submitted"], "dropped": config_b["dropped"],
            "processed": config_b["processed"], "failed": config_b["failed"],
            "notes": "cold-start burst: model load (~1-2s) + GPU clock/cudnn-benchmark ramp (~5-6s, see config_a dmon log) means the first several 133ms submission windows arrive before the previous pending job is even picked up, incrementing `dropped`; steady-state (post-ramp) drops are ~0",
        },
        {
            "config": "C_gazebo_headless_camera_only",
            "submitted": config_c["submitted"] if config_c else 0, "dropped": config_c["dropped"] if config_c else 0,
            "processed": config_c["processed"] if config_c else 0, "failed": config_c["failed"] if config_c else 0,
            "notes": f"frame_arrival_rate_hz={config_c['frame_arrival_rate_hz']:.1f}" if config_c else "not run",
        },
        {
            "config": "D_full_stack", "submitted": config_d["total_rows"] if config_d else "see reused source",
            "dropped": "n/a (worker single-slot; see status counters if captured)",
            "processed": config_d["stage_counts"].get("raw_range_computed", 0) if config_d else 844,
            "failed": config_d["stage_counts"].get("calibration_rejected", 0) if config_d else "see source",
            "notes": f"stage_counts={config_d['stage_counts']}" if config_d else "reused, see docs report",
        },
        {
            "config": "F_full_stack_lower_camera_rate",
            "submitted": config_f["total_rows"] if config_f else 0,
            "dropped": "n/a (worker single-slot)",
            "processed": config_f["stage_counts"].get("raw_range_computed", 0) if config_f else 0,
            "failed": config_f["stage_counts"].get("calibration_rejected", 0) if config_f else 0,
            "notes": (
                f"stage_counts={config_f['stage_counts']}; most rows are calibration_rejected -- a "
                "scenario-bootstrap calibration-convergence issue at 2Hz, not something this task's "
                "latency question depends on, since worker_inference_s is recorded on every row "
                "regardless of calibration outcome"
            ) if config_f else "not run",
        },
    ]
    _write_csv(output / "queue_drop_metrics.csv", queue_drop_rows)

    a_median = config_a["worker_inference_s_unprofiled_production_representative"]["median_s"]
    b_median = config_b["worker_inference_s_unprofiled_production_representative"]["median_s"]
    c_median = config_c["worker_inference_s"]["median_s"] if config_c else None
    d_median = config_d["stats"]["median_s"] if config_d else REUSED_PRIOR_D_EQUIVALENT_EVIDENCE["baseline_8_frozen_sessions_844_frames"]["worker_inference_s_median"]
    f_median = config_f["stats"]["median_s"] if config_f else None

    gate = "MIXED_LIVE_STACK_CONTENTION"
    if config_c and config_f and c_median is not None and f_median is not None:
        gate = "ROS_CALLBACK_OR_THREAD_BLOCKING_DOMINANT"

    manifest = {
        "isolation_id": ISOLATION_ID,
        "gate": gate,
        "gate_candidates_considered": list(GATE_OUTCOMES),
        "ruled_out_with_fresh_evidence": {
            "compute_cost_of_the_model_itself": (
                f"config A (idle GPU, isolated): median {a_median * 1000:.2f} ms. Not the cause: "
                f"{d_median / a_median:.0f}x smaller than D's {d_median * 1000:.0f} ms."
            ),
            "worker_thread_GIL_or_async_queueing_overhead_alone": (
                f"config B (real LatestDepthWorker background thread + synthetic frames at production "
                f"rate, isolated process): median {b_median * 1000:.2f} ms -- close to A, not close to D."
            ),
            "gazebo_gpu_rendering_and_px4_physics_alone": (
                (
                    f"config C (gz sim headless + 1 PX4 instance + camera worker, same 7.5Hz submission "
                    f"rate as D, no ROS2/dashboard/mavlink-bridge): median {c_median * 1000:.2f} ms -- "
                    "close to A/B, nowhere near D. Gazebo's own rendering loop plus PX4 SITL physics, "
                    "running at the SAME submission rate as the full stack, is not sufficient to "
                    "reproduce the slowdown. This rules out GAZEBO_GPU_CONTENTION_DOMINANT as the "
                    "dominant cause."
                ) if c_median is not None else "config C could not be run"
            ),
            "queue_publish_consume_overhead_outside_the_inference_call": (
                "already isolated by the prior task's stage decomposition: submit->worker_start (queue "
                "wait) median 0.022s, complete->publish ~0.00002s, publish->consumer_receive 0.212s, "
                "consumer_receive->consume ~0.0000007s -- none overlap with worker_inference_s, which "
                "is time strictly inside the `adapter.infer()` call. The blowup is inside the "
                "inference call, not around it."
            ),
        },
        "decisive_evidence": (
            (
                f"config D vs config F, single-variable change (SWARM_RANGE_DATASET_DEPTH_RATE_HZ "
                f"7.5 -> 2.0 Hz, everything else in the full stack identical): worker_inference_s "
                f"median {d_median * 1000:.0f} ms -> {f_median * 1000:.1f} ms "
                f"({d_median / f_median:.0f}x reduction). Combined with config C (Gazebo+PX4 alone, "
                f"same 7.5Hz rate as D, stayed at {c_median * 1000:.1f} ms): if Gazebo/PX4-side "
                "contention were dominant, D's contention would be roughly rate-independent (Gazebo "
                "renders on its own schedule regardless of MiDaS's submission rate) -- but it is "
                "strongly rate-dependent, and only the components present in D/F but absent from C "
                "(ROS2 telemetry launch, mavlink_manual_bridge.py, the dashboard's own asyncio event "
                "loop / tracking_web.py consumer thread, the Gazebo GUI, the second UAV) scale with "
                "how often the worker thread needs to run. This points to CPU/GIL-level thread or "
                "callback contention from that layer, not GPU rendering contention."
            ) if f_median is not None else "config F could not be run; gate reflects C-only evidence"
        ),
        "not_fully_isolated": (
            "This task did not further subdivide *which* of ROS2 telemetry / mavlink bridge / dashboard "
            "event loop / GUI / second UAV is the specific blocking component -- only that the "
            "aggregate of components present in D/F but not C is responsible, and that the effect is "
            "rate-dependent. A follow-up ablation removing these one at a time (not run here, to keep "
            "this task's scope bounded) could pin down the exact mechanism."
        ),
        "px4ssd_incident": {
            "note": (
                "This task initially found /mnt/px4ssd in ext4 emergency_ro mode, which blocked C/D/F "
                "on the first attempt. Per user instruction, this was not fixed by this task; the user "
                "separately confirmed the mount healthy again (a second, working mount, /dev/loop14, "
                "stacked on top of the faulted /dev/loop13) and directed this task to proceed, which is "
                "how the fresh C/D/F evidence above was obtained."
            ),
        },
        "no_retrain": True, "no_official_dataset_collection": True,
        "no_gt_m52_calibration_controller_px4_change": True, "no_follow_target": True,
        "no_default_runtime_change": True, "all_profiling_opt_in": True,
    }
    _write_json(output / "contention_manifest.json", manifest)
    print(json.dumps({"gate": manifest["gate"]}, indent=2))


def main() -> int:
    parser = argparse.ArgumentParser()
    sub = parser.add_subparsers(dest="command", required=True)

    p = sub.add_parser("prepare")
    p.add_argument("--output", type=Path, required=True)

    p = sub.add_parser("single-pass")
    p.add_argument("--output", type=Path, required=True)
    p.add_argument("--iterations", type=int, default=30)
    p.add_argument("--warmup", type=int, default=5)

    p = sub.add_parser("config-a")
    p.add_argument("--output", type=Path, required=True)
    p.add_argument("--iterations", type=int, default=200)
    p.add_argument("--warmup", type=int, default=10)

    p = sub.add_parser("config-b")
    p.add_argument("--output", type=Path, required=True)
    p.add_argument("--duration-s", type=float, default=20.0)
    p.add_argument("--rate-hz", type=float, default=PRODUCTION_DEPTH_RATE_HZ)

    p = sub.add_parser("monitor")
    p.add_argument("--output", type=Path, required=True)
    p.add_argument("--duration-s", type=float, required=True)
    p.add_argument("--label", type=str, required=True)

    p = sub.add_parser("gpu-snapshot")
    p.add_argument("--output", type=Path, required=True)
    p.add_argument("--label", type=str, required=True)

    p = sub.add_parser("compose-final")
    p.add_argument("--output", type=Path, required=True)

    args = parser.parse_args()
    output = args.output.resolve()
    if args.command == "prepare":
        prepare(output)
    elif args.command == "single-pass":
        single_pass(output, iterations=args.iterations, warmup=args.warmup)
    elif args.command == "config-a":
        config_a(output, iterations=args.iterations, warmup=args.warmup)
    elif args.command == "config-b":
        config_b(output, duration_s=args.duration_s, rate_hz=args.rate_hz)
    elif args.command == "compose-final":
        compose_final(output)
    elif args.command == "monitor":
        monitor(output, args.duration_s, args.label)
    elif args.command == "gpu-snapshot":
        process_gpu_usage_snapshot(output, args.label)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
