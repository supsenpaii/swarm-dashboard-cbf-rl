# CORE_RANGE_GAZEBO_RTF_STUTTER_AND_GATE_VALIDATION

Date: 2026-08-05
Conclusion: `RTF_REMAINS_HARD_GATE_STUTTER_UNRESOLVED`

## Scope

Sole objective: determine the root cause of Gazebo real-time-factor (RTF)
stutter in long headless sessions, and determine whether RTF genuinely
needs to remain a hard gate for the dynamic training corpus. No 8-group
recollection, no model training, no M52/calibration/EKF/controller/PX4
change, no Follow Target.

## 1. Clock domain audit

Every timestamp value listed in the task was traced to its source
assignment (full detail in `clock_domain_inventory.json`):

| Value | Clock domain | RTF-sensitive? |
|---|---|---|
| Target trajectory scheduling | `python_monotonic` (wall-clock-paced `gz set_pose` commands) | No — target is externally teleported, not physically integrated |
| Gazebo pose timestamp | `gazebo_sim_time` | No |
| Camera sensor timestamp | both, explicitly separated (`sim_timestamp_s` / `monotonic_receipt_s`) | sim side: no; monotonic side: yes (fewer per wall-second) |
| Tracker timestamp | `python_monotonic` | Self-consistent |
| Raw range timestamp (`measurement_timestamp_s`) | `python_monotonic` | Self-consistent |
| Temporal features (delta_time_s, range_rate) | `python_monotonic` (both operands) | No mixing found |
| Capture→consume latency stages | `python_monotonic`, **schema-enforced** (`clock_id` field validated) | No |
| Ground-truth alignment | `gazebo_sim_time` exclusively | No — matched via sim-time bracket interpolation |
| Lag evaluation (`alignment_lag_s`) | `python_monotonic` index, sim-time-correct GT values | Intentional, correct clock choice |

**No sim-time/wall-clock mixing bug was found.** The target is driven by
`gz set_pose` calls issued on a fixed wall-clock cadence
(`core_range_dynamic_capture.py`, `time.monotonic()`-paced), not integrated
by Gazebo's physics — so an individual rendered frame's visual content (and
therefore its ground truth, matched via sim-time bracket search against
`GazeboDashboardBridge.simulation_target_ground_truth`) is correct at
whatever wall-clock instant it renders, independent of RTF. Every other
timestamp in the pipeline is `python_monotonic`, used self-consistently
(capture-latency stages are schema-enforced to be `python_monotonic` and
`validate_timestamp_stages` rejects any other `clock_id`). This is a
necessary but not sufficient condition for downgrading RTF to a diagnostic
signal — see §2 for whether it is also sufficient.

## 2. RTF impact on real data

Using real `raw_range_computed` rows from this task's own long run (§3),
each row was labeled by the Gazebo RTF value nearest its own capture moment
(converted from `python_monotonic` to the same wall-clock axis as the
resource trace via a measured `CLOCK_REALTIME - CLOCK_MONOTONIC` offset —
Linux's `CLOCK_MONOTONIC` is system-wide and comparable across processes,
but not directly comparable to `time.time()` without this conversion).

A single large contiguous stable/stutter window pair was not usable: the
real RTF trace is tightly interleaved (median run length 3–7 s, see §3),
so per-row labeling across all available real data was used instead of one
window each.

| Metric | RTF≥0.8 ("stable", n=225) | RTF≤0.3 ("stutter", n=298) |
|---|---:|---:|
| raw_range_vs_gt_abs_error_m (median) | 1.272 m | 2.076 m |
| raw_range_vs_gt_abs_error_m (mean) | 1.544 m | 1.987 m |
| delta_time_s between accepted frames (median) | 0.220 s | 0.221 s |
| causal_range_rate_m_per_s (median) | -0.024 | -0.025 |
| capture→consume (median) | 60.7 ms | 62.5 ms |
| checksum / timestamp / anchor / finite pass fraction | 100% | 100% |

**Raw-range accuracy is measurably worse in low-RTF windows: median
absolute error rises from 1.27 m to 2.08 m, a 63% increase**, on hundreds
of real rows each side — not sampling noise. Delta-time, causal range-rate,
and capture-latency stay essentially unchanged between the two regimes
(consistent with §1's clock-audit finding that these are wall-clock
self-consistent), and checksum/timestamp/anchor integrity is 100% in both —
**records are never corrupted, but their accuracy degrades**.

A second, more direct failure mode was observed: **5 of 13 long-run reps
failed outright** with `gazebo_target_pose_failed_after_retries` — the
external trajectory script's `gz set_pose` service calls timed out after 3
retries during severe stutter episodes. This is a real command-level
failure, not just temporal undersampling.

## 3. Long-run stutter root-cause profiling

A single, persistent headless stack (Gazebo/PX4×2/ROS2/backend, launched
once, never restarted) ran for 1780 s (~29.7 min) while 13 repeated
approaching/receding capture reps ran back-to-back against it, with a
per-second resource sampler running the whole time
(`core_range_gazebo_rtf_long_run_monitor.py`; see Instrumentation below).

- **RTF pattern**: bimodal throughout, not a slow decay. 1779 one-second
  samples; 76 stable↔stutter transitions detected; individual stable and
  stutter runs are typically only 3–7 seconds long. The pattern is present
  from the very start of the run, not concentrated at the end in a smooth
  sense — though 5 consecutive rep failures did cluster in the run's second
  half (reps 8–12), a discrete pattern not captured by the correlation
  below.
- **Whole-run linear correlation with RTF** (1779 samples, every measured
  resource metric):

| Metric | r vs RTF |
|---|---:|
| context_switches_per_s | -0.216 |
| elapsed_s | +0.167 |
| mem_available_mb | -0.113 |
| thermal_zone_max_c | -0.115 |
| cpu_freq_mean_mhz | -0.105 |
| disk_write_kb_per_s | +0.142 |
| gpu_clock_sm_mhz | -0.084 |
| gpu_util_pct | +0.070 |
| process_thread_count | +0.033 |

**No metric exceeds |r|=0.22 — no single dominant resource driver was
found.** The strongest (context switches, weakly negative) is consistent
with general OS-level scheduling contention across the ~8-process stack
(Gazebo, 2× PX4 SITL, 2× ROS2 telemetry nodes, 2× mqtt_bridge,
web_control_node, mavlink bridge, uvicorn backend) sharing a finite core
budget, but this is not confirmed as the definitive mechanism. Notably,
`elapsed_s` correlates weakly **positively**, not negatively — this argues
against a simple "stutter smoothly worsens the longer the process runs"
model, even though discrete clusters of failures did occur later in this
particular run.

## 4. Ablations

| Config | Status | Evidence |
|---|---|---|
| A. Full headless baseline | Executed (the 28-min long run) | Bimodal RTF persists |
| B. No diagnostics disk writes | Not executed — structurally incompatible (see Known Limitations) | `disk_write_kb_per_s` r=+0.142 (weak, wrong sign for a causal disk-contention story) |
| C. Minimal dashboard client | Not executed — cited prior evidence | `docs/CORE_RANGE_CAMERA_SOURCE_FPS_REPORT.md` D_dashboard_off: statistically identical to baseline; harness never attaches a client anyway |
| D. No depth inference | Not executed — cited correlation + prior evidence | `gpu_util_pct` r=+0.070, `gpu_clock_sm_mhz` r=-0.084 (weak); corroborates the camera-source-FPS task's own C_depth_off ablation |
| E. Restart Gazebo between sessions | Answered by prior evidence | The 5Hz-headless task's 13 attempts were *already* full-stack restarts every session and still showed stutter-driven failures late in that batch — restart-per-session alone is not a proven fix |
| F. Restart backend, keep Gazebo | Built, not executed (time budget) | `process_thread_count` r=+0.033 (near zero); not strongly implicated |
| G. Repeat scenario many times | Executed (same long run as A) | 76 transitions, 3–7 s runs, no smooth degradation trend |

Camera resolution and tracking quality were not reduced to force a pass at
any point.

## 5. RTF gate decision

Per the task's precommitted conditions for downgrading RTF to a diagnostic
warning (`rtf_gate_validity.json`):

| Condition | Status |
|---|---|
| Trajectory and GT use unified sim-time | PASS |
| No sim/wall mixing in feature/model data | PASS |
| Temporal delta-time uses correct clock | PASS |
| Raw/static/dynamic error does not degrade in low-RTF windows | **FAIL** — 63% worse median error |
| No significant frame loss per sim-second | Inconclusive (see limitations) |
| GT alignment and lag not corrupted | Partially fail — alignment itself is architecturally sound, but severe stutter causes real `gz set_pose` command failures |
| Checksum/timestamp integrity still PASS | PASS |

Because the accuracy-degradation condition fails with real, substantial
evidence, **RTF cannot be downgraded to diagnostic-only**. No single root
cause was isolated to fix (§3), and restart-per-session is not adopted
because it was effectively already in place in the prior 5Hz-headless task
and did not prevent the pattern. Per the task's own instruction, the gate
was not lowered simply to reach 8/8 collection.

## 6. Validation smoke

Three scenarios, headless, functional gates only (RTF gate contract
unchanged, so not re-litigated per-scenario here):

| Scenario | raw_range | camera FPS | tracking FPS | capture→consume med/p95 | MiDaS p95 | integrity |
|---|---:|---:|---:|---:|---:|---|
| approaching (long-run rep 1) | 583 | 40.47 | 25.23 | 62.3 / 143.1 ms | 52.3 ms | 100% |
| receding | 146 | 44.61 | 23.57 | 72.2 / 156.5 ms | 71.4 ms | 100% |
| stop_and_hold | 138 | 44.66 | 24.93 | 61.6 / 143.2 ms | 49.1 ms | 100% |

**All functional gates pass in all three scenarios** — camera FPS,
tracking FPS, latency, MiDaS P95, integrity, and no growing backlog remain
solid regardless of the RTF gate outcome; only the RTF metric itself is
volatile.

## Instrumentation added

All additive/opt-in, no default runtime change:

- `core_range_gazebo_rtf_long_run_monitor.py` (new): per-second RTF (via a
  persistent `gz-transport` `WorldStatistics` subscription, not a
  fresh-process-per-sample `gz topic -e` call), CPU frequency/thermal
  (`/sys` reads, no sudo), memory/swap, disk write throughput
  (`/proc/diskstats` delta), context switches (`/proc/stat` delta),
  process/thread count, and GPU utilization/memory/clocks (`nvidia-smi`).
  Writes incrementally with `SIGTERM` handled gracefully — an earlier
  version buffered all rows to the end and lost the entire 28-minute trace
  on the caller's normal shutdown signal; caught and fixed before the real
  run (unit-tested: `test_core_range_gazebo_rtf_stutter_analysis.py`).
- `core_range_gazebo_rtf_long_run.sh` / `core_range_gazebo_rtf_ablation_run.sh`
  / `core_range_gazebo_rtf_ablation_f_run.sh` (new): persistent
  single-stack headless wrappers, always `SWARM_START_GAZEBO_GUI=false`.
- `core_range_gazebo_rtf_stutter_analysis.py` (new): transition detection,
  whole-run resource/RTF correlation, and RTF-context-labeled real-data
  comparison.

## Known limitations

- **Root cause of the stutter itself remains unresolved.** Every measured
  resource metric correlates weakly with RTF; the mechanism (most likely
  OS-level scheduling contention across the multi-process stack) is not
  confirmed.
- **Capture-harness reuse artifact**: `core_range_dynamic_capture.py`'s
  `wait_for_new_sidecar_session()` was designed for one-shot use against a
  freshly-launched stack. Reused here against a persistent stack across 13
  reps, most reps after the first reported success without the backend
  actually starting a distinguishable new diagnostics session — all 899
  real rows in the long run's sidecar carry `session_id=2`, corresponding
  to only the first ~150 s (rep 1) of the 1780 s run. §2's real-data
  comparison is valid, real, per-row-labeled evidence, but drawn from that
  first ~150 s rather than the full run. §3's resource/RTF trace is
  unaffected (it samples independently of the diagnostics sidecar).
  Fixing this harness limitation for genuine multi-rep persistent-stack
  reuse is out of this task's scope.
- Ablations B and F were not directly executed — see §4 for the specific
  reasoning and correlation evidence substituted for each.

## Verification

- Focused tests: `test_core_range_gazebo_rtf_stutter_analysis.py` (7
  tests) — all passing.
- Full repository test suite and `./run_all.sh --check`: recorded after
  final execution.
- All PX4/Gazebo/ROS2/backend/capture/profiler processes stopped cleanly
  after every run in this task.

## Deliverables

`artifacts/core_range_3_12m/gazebo_rtf_stutter/`: `clock_domain_
inventory.json`, `rtf_window_comparison.csv`, `long_run_resource_
trace.csv`, `transition_events.json`, `ablation_metrics.csv`, `rtf_gate_
validity.json`, `source_changes.json`, `validation_smoke.csv`, `fix_
manifest.json`, `fix_report.md`, plus raw evidence under `long_run/`.
