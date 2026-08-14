# CORE RANGE LIVE STACK CONTENTION ISOLATION REPORT

Isolation ID: `core_range_live_stack_contention_isolation_20260805_v001`

## Conclusion

```
ROS_CALLBACK_OR_THREAD_BLOCKING_DOMINANT
```

All six planned configurations were resolved: A, B, C, D, and F ran with
fresh, real evidence; E was audited and correctly not executed (no other
AI/GPU consumer exists in this repository to disable). No retrain, no
official dataset collection, no GT/M52/calibration/controller/PX4 change,
no Follow Target. All profiling instrumentation this task added is opt-in
(`SWARM_DEPTH_PROFILE_STAGES`) and off by default; nothing about the
default runtime changed.

**A mid-task infrastructure incident, now resolved**: this task initially
found `/mnt/px4ssd` (the ext4 image backing PX4-Autopilot) stuck in kernel
`emergency_ro` mode, which blocked PX4 SITL from booting and, with it,
configs C/D/F. Per instruction, this was not repaired by this task
(fsck/remount on an already-faulted filesystem is a real data-loss-risk
decision that belongs to the user). The user separately investigated and
confirmed a second, healthy mount (`/dev/loop14`, from
`/media/sup/New Volume/px4-linux.img`) is now stacked on top of the
faulted one (`/dev/loop13`) at the same mountpoint, verified by directory
listing, `stat`, a write test, and PX4 booting past its previous failure
point -- and directed this task to proceed. All results below for C, D,
and F were captured after that.

## Instrumentation added (opt-in only)

`depth_model_adapter.py`'s `MidasSmallAdapter.infer()` times seven
sub-stages with wall-clock monotonic (`time.perf_counter()`), bracketing
every GPU-touching boundary with `torch.cuda.synchronize()` -- but
**only** when `SWARM_DEPTH_PROFILE_STAGES=1` is set; unset (the default),
there is zero behavior or timing change (verified in
`test_depth_model_adapter_profiling.py`, including a CUDA test that counts
`torch.cuda.synchronize()` calls and confirms zero when the flag is off).
`depth_worker.py`'s `DepthResult` gained one new optional field,
`profiling_stages` (`test_depth_worker_profiling.py`).

**Methodological findings from building this instrumentation**:
bracketing every stage with `cuda.synchronize()` costs ~2-3% once GPU
clocks are fully ramped (config A: 5.85ms profiled vs 5.71ms unprofiled).
Separately, this GPU (RTX 4060 Laptop) idles down to power state P8
(~210MHz) between bursts of work and only ramps to a performance state
after several seconds of sustained load (`config_a_nvidia_smi_dmon.log`:
P8/210MHz -> P3/2490-2505MHz within ~5-6s) -- any fresh process pays this
ramp tax on its first several calls, independent of anything in this
task.

## 1. Single detailed instrumented pass (standalone, idle GPU)

One real pass -- `adapter.infer()` (fully profiled) -> `TargetDepthExtractor.
extract()` (timed) -> a trivial publish-equivalent op (timed) -- repeated
20 times after 5 warmup iterations (`stage_latency.csv`,
`stage_latency_summary.json`):

| Stage | Median (ms) | Share of total |
|---|---:|---:|
| frame_copy_decode | 0.81 | 5.5% |
| host_to_device_transfer | 0.76 | 5.1% |
| gpu_preprocess_resize_normalize | 0.22 | 1.5% |
| **model_forward** | **10.71** | **72.2%** |
| output_resize_interpolation | 0.10 | 0.7% |
| device_to_host_transfer | 0.18 | 1.2% |
| numpy_postprocess | 1.46 | 9.9% |
| roi_extraction | 0.54 | 3.6% |
| publish_result | ~0.000001 | ~0.0% |
| **total_pass** | **14.84** | 100% |

(This run captured during partial clock ramp, so absolute values run a
little high vs. the fully-converged config A numbers below; relative
proportions are what matters here.) Every CPU-side step this task was
asked to instrument -- frame copy/decode, transfers, ROI extraction,
publish -- is individually under 10% of the total. This matters for
section 2: whatever inflates live-stack latency to hundreds of
milliseconds cannot be one of these small steps growing on its own; it
has to be something new that doesn't exist in this isolated pass at all.

## 2. Controlled ablation -- final results

Only one factor changes between adjacent rows. Full data:
`configuration_comparison.csv`.

| Config | `worker_inference_s` median | P95 | n |
|---|---:|---:|---:|
| A: standalone, idle GPU | 5.71 ms | 6.31 ms | 150 |
| B: real worker thread + synthetic frames, no stack | 13.69 ms | 15.56 ms | 78 |
| **C: Gazebo headless + camera + MiDaS, no ROS2/dashboard/bridge** | **8.19 ms** | 16.54 ms | 144 |
| **D: full Gazebo/ROS2/PX4/dashboard stack** | **778.97 ms** | 1192.12 ms | 40 |
| E: full stack, other AI disabled | not executed -- nothing to disable | — | — |
| **F: full stack, camera/depth rate 7.5 -> 2.0 Hz** | **24.76 ms** | 45.11 ms | 177 |

### A, B -- as before

Idle GPU standalone (5.71ms) and the real worker thread with synthetic
frames at production rate but no other process (13.69ms) both rule out
the model's own compute cost and the worker-thread/GIL mechanism as
sufficient explanations by themselves.

### C -- Gazebo headless + camera + MiDaS, no ROS2/dashboard/bridge

`core_range_contention_run_config_c.sh` starts a headless `gz sim` server
(no GUI) plus one PX4 SITL instance (UAV-01/`x500_custom_0`, the minimum
needed for a camera-bearing vehicle to exist), then a standalone
gz-transport subscriber (`core_range_contention_config_c_camera_worker.py`,
using the same `gz.transport13`/`gz.msgs10` bindings and the same
`_image_to_bgr` decode logic as `main.py`'s `GazeboDashboardBridge`,
copied read-only rather than importing the full dashboard module) feeding
frames directly into the real `LatestDepthWorker`/`MidasSmallAdapter`. No
MicroXRCEAgent, no second UAV, no ROS2 telemetry launch, no
`mavlink_manual_bridge.py`, no `main.py`/uvicorn.

**1260 camera frames received in 25s (~50Hz), 178 submitted at the
production 7.5Hz throttle, 147 processed, 31 dropped (cold-start,
analogous to config B). Median inference 8.19ms** -- close to A/B,
nowhere near D. **Gazebo's own rendering loop plus PX4 SITL physics,
running at the production camera rate, does not reproduce the live-stack
slowdown.** This rules out `GAZEBO_GPU_CONTENTION_DOMINANT` as the
dominant cause.

### D -- full stack (fresh capture)

`core_range_contention_run_live_scenario.sh D_fresh 7.5 ...` (mosquitto,
MicroXRCEAgent, gz sim server+GUI, PX4 UAV-01+02, ROS2 telemetry launch,
`mavlink_manual_bridge.py`, `uvicorn main:app`), observation-only smoke
capture, 16s approach trajectory. **worker_inference_s median 778.97ms,
P90 1110ms, P95 1192ms, n=40** (26 `raw_range_computed`, 8
`inverse_depth_filter_rejected`, 6 `calibration_only`) -- consistent with
(if a little higher than) the earlier task's reused reference figures
(612ms / 854-1076ms across sessions), confirming this is a stable,
reproducible phenomenon and not a one-off artifact of the earlier
capture.

### E -- full stack, other AI disabled (not executed -- nothing to disable)

Audited via `grep -rl 'torch\.|import torch'` across every production
module (excluding the stale `swarm_dashboard_handoff_20260729/` snapshot
bundle and this task's own scripts): the **only** torch/GPU consumer
anywhere in this repository is `depth_model_adapter.py` (MiDaS).
`tracking_hybrid.py`'s LightFC hybrid tracker and `m52_adapter.py`'s
XGBoost residual model are both CPU-only.

### F -- full stack, lower camera/depth rate (decisive)

Identical stack to D. Single variable changed:
`SWARM_RANGE_DATASET_DEPTH_RATE_HZ` 7.5 -> 2.0 Hz (main.py:2904-2905, the
only camera/depth-rate knob this codebase exposes without touching
M52/calibration/controller/PX4/Follow Target). **worker_inference_s
median 24.76ms, P90 38.88ms, P95 45.11ms, n=177** -- a **31x reduction**
from D's 778.97ms, from changing exactly one variable.

(171/177 rows were `calibration_rejected` in this session -- a
scenario-bootstrap calibration-convergence issue at the lower rate, not
something this task's latency question depends on: `worker_inference_s`
is recorded from `timestamp_stages` on every diagnostic row regardless of
calibration outcome, so this doesn't affect the measurement above. Not
investigated further, out of scope for a latency-contention task.)

## 3. Resource profiling

`resource_usage.csv` (GPU util/memory/clocks/power before+after A, B, C),
`config_a/b/c_nvidia_smi_dmon.log` and `config_d/config_f
*_nvidia_smi_dmon.log` (1Hz `nvidia-smi dmon -s pucv`: power, temp,
utilization, clocks, throttle violations), `*_pidstat.log` (1Hz
per-thread `pidstat -t -u -r -w`, read-only), `process_gpu_usage.csv`
(`nvidia-smi --query-compute-apps`).

**GPU utilization is essentially identical between D and F**, despite
their 31x inference-latency difference: D averaged 20.17% SM utilization
/ 746MHz over the capture window, F averaged 20.19% / 723MHz
(`D_fresh_nvidia_smi_dmon.log`, `F_fresh_nvidia_smi_dmon.log`). If GPU
rendering contention were driving D's slowdown, D would be expected to
show measurably higher sustained GPU load than F -- it does not. This is
additional evidence against a GPU-side explanation.

**Per-process CPU also looks similar between D and F**
(`D_fresh_pidstat.log`, `F_fresh_pidstat.log`, process-level rows only):
`px4` ~107-109%, `python` ~56-64%, `telemetry_node` ~12.5-12.8% average
CPU in both. No process shows an aggregate CPU spike proportional to the
31x latency difference -- consistent with the bottleneck being a
*scheduling/timing* effect (contention specifically at the moments the
depth worker thread needs to run) rather than a sustained CPU hog, which
is a subtler signature than raw `%CPU` averages can show and was not
further decomposed in this task (see "not fully isolated" below).

`nvidia-smi --query-compute-apps` reported **zero rows in every
configuration**, including D and F while MiDaS was demonstrably running
(`process_gpu_usage.csv`) -- this GPU/driver combination does not expose
per-process compute-app attribution to an unprivileged user on this
machine, so that specific sub-check is a tool limitation here, not
evidence of zero concurrent GPU users; the code-level audit (section 2/E)
is the reliable source for "is anything else using the GPU."
Frame-arrival-rate, pending-frame-replacement, and dropped-frame counts
are in `queue_drop_metrics.csv`.

## 4. Contention checks

| Check | Finding |
|---|---|
| Does Gazebo rendering claim GPU? | **No, not dominantly**: config C (Gazebo+PX4, same 7.5Hz rate as D) stayed at 8.19ms; GPU utilization is statistically identical between D (slow) and F (fast). |
| Does camera copy/decode block CUDA? | No: `frame_copy_decode_s` is ~0.5-0.9ms in every measured configuration (section 1), far too small. |
| Does another AI model share the GPU? | **No** -- confirmed by code audit (section 2/E): MiDaS is the only torch/GPU consumer in this repository. |
| Do multiple CUDA contexts compete? | Not observed in A/B/C (`process_gpu_usage.csv`, where attribution worked -- 0 concurrent compute-apps); the query itself doesn't work for D/F on this GPU/driver (see section 3), but the code audit already rules out a second CUDA-using process. |
| Does inference wait on CPU preprocessing? | No: CPU-side stages are consistently a few percent of total time everywhere they were measured (section 1); even at P95 they don't approach hundreds of milliseconds. |
| **Does logging/fsync or a ROS callback block the worker?** | **Best-supported yes.** D vs F (identical stack, only the depth/camera submission rate changed) shows a 31x latency swing that C (Gazebo+PX4 present, same rate as D, no ROS2/dashboard/mavlink-bridge) does not reproduce. Only the components present in D/F but absent from C -- ROS2 telemetry launch, `mavlink_manual_bridge.py`, the dashboard's own asyncio event loop / `tracking_web.py` consumer thread, the Gazebo GUI, the second UAV -- can plausibly scale with how *often* the worker needs to run, which is exactly what changing the submission rate tests. |
| Does PyTorch reload/re-warm the model per frame? | No: `MidasSmallAdapter.load()` returns immediately once `self._model is not None`, and `LatestDepthWorker` calls `load()` exactly once, before the wait loop, in `_run()`. Not per-frame. |

## Why the gate is `ROS_CALLBACK_OR_THREAD_BLOCKING_DOMINANT`

Two clean, single-variable comparisons converge on the same conclusion:

1. **C vs D** (same 7.5Hz submission rate; C omits ROS2 telemetry,
   mavlink bridge, dashboard, GUI, second UAV; D has all of them): 8.19ms
   vs. 778.97ms. Gazebo/PX4 alone, even at production rate, is not
   sufficient.
2. **D vs F** (identical full stack; only the submission rate changes,
   7.5 -> 2.0 Hz): 778.97ms vs. 24.76ms, a 31x reduction. If the
   bottleneck were Gazebo's GPU rendering (which runs on its own schedule
   regardless of MiDaS's submission rate), lowering *only* MiDaS's
   submission rate should not have helped nearly this much. It did --
   which points at something that scales with how often the depth worker
   thread runs, i.e., contention introduced by the layer C omits and D/F
   include: ROS2 topic callbacks, the mavlink bridge, and/or the
   dashboard's own asyncio event loop / consumer thread competing for the
   worker thread's CPU time or the Python GIL at the moment it needs to
   run.

Aggregate GPU utilization (identical between D and F) and aggregate
per-process CPU (also similar between D and F) argue against a
sustained-resource explanation and toward a scheduling/timing one --
consistent with intermittent callback- or lock-driven blocking rather
than a constant extra workload.

**Not fully isolated**: this task did not further subdivide *which*
specific component among ROS2 telemetry / mavlink bridge / dashboard
event loop / GUI / second UAV is the actual blocking mechanism -- only
that their aggregate is responsible and that the effect is strongly
rate-dependent. A follow-up ablation removing them one at a time (not run
here, to keep this task's scope bounded to the six planned
configurations) would pin down the exact culprit.

## Disposition

No code path used in production changed behavior by default. The two
instrumentation changes (`depth_model_adapter.py`, `depth_worker.py`) are
strictly additive and opt-in, verified by tests. No retrain, no
integration, no runtime-mode change follows from this task, per scope.

## Full test verification

- Focused: `test_depth_model_adapter_profiling.py` (7 cases, including a
  real-CUDA synchronize-call-counting test) and
  `test_depth_worker_profiling.py` (2 cases) -- all passing.
- Full repository suite (excluding the stale
  `swarm_dashboard_handoff_20260729/` snapshot bundle, same exclusion the
  prior task used): 380 passed (373 pre-existing + 7 new).
- `./run_all.sh --check`: passed.
- No PX4/Gazebo/ROS2/dashboard process left running after any ablation or
  at task completion (verified via `pgrep` and `nvidia-smi` after every
  run).

## Deliverables

```
artifacts/core_range_3_12m/live_stack_contention/
  profiling_plan.json
  stage_latency.csv / stage_latency_summary.json
  config_a_result.json / config_a_stage_latency.csv / config_a_nvidia_smi_dmon.log
  config_b_result.json / config_b_stage_latency.csv / config_b_nvidia_smi_dmon.log
  config_c/config_c_result.json / config_c/config_c_nvidia_smi_dmon.log
  config_d/dynamic_capture/ (capture_result.json, physical_diagnostics.jsonl, ...)
  config_d/D_fresh_nvidia_smi_dmon.log / config_d/D_fresh_pidstat.log
  config_f/dynamic_capture/ (capture_result.json, physical_diagnostics.jsonl, ...)
  config_f/F_fresh_nvidia_smi_dmon.log / config_f/F_fresh_pidstat.log
  configuration_comparison.csv
  resource_usage.csv
  process_gpu_usage.csv
  queue_drop_metrics.csv
  contention_manifest.json
  contention_report.md
```
