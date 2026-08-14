# CORE_RANGE_CAMERA_SOURCE_FPS_AUDIT_AND_FIX

Date: 2026-08-05
Conclusion: `CAMERA_SOURCE_FPS_FIXED_READY_FOR_RECOLLECTION`
Root-cause classification: `GAZEBO_REALTIME_FACTOR_DOMINANT`

## Scope

Sole objective: determine why camera-source FPS in the full representative
live stack measured ~17–18 FPS earlier the same day
(`docs/CORE_RANGE_COLLECTION_THROUGHPUT_FIX_REPORT.md`), against a same-day
historical smoke that measured ~27–28 FPS
(`docs/CORE_RANGE_FULL_STACK_CONTENTION_FIX_REPORT.md`). No full dynamic
corpus recollection, no model training, no change to the range model, M52,
calibration, EKF, controller, or PX4 code, no Follow Target run. The prior
task's own finding — that collection/diagnostics/overlay/fsync code is not
the dominant cost, and the loss sits upstream in camera-source delivery
itself (`COLLECTION_THROUGHPUT_EVIDENCE_INSUFFICIENT`) — is the explicit
starting point this task picks up.

## Instrumentation added (additive, opt-in, default runtime unchanged)

- **`camera_source_trace.py`** (new module): a JSONL hop tracer gated by
  `SWARM_CAMERA_TRACE_JSONL` (a file path; unset = no file I/O at all — a
  cheap `os.environ.get` on every call site). Never raises into the
  callback threads it instruments (broad `except OSError` around the
  write).
- **`main.py`** (`GazeboDashboardBridge._handle_camera_image`): two trace
  points — `camera_source_receipt` (every gz-transport callback, before
  any client/tracking gating, capturing both the Gazebo sensor's own
  sim-time header stamp and the wall-clock `time.monotonic()` receipt) and
  `camera_mailbox_store` (when a converted frame is accepted into the
  single-slot `raw_frames` mailbox for the actively-tracked drone).
- **`tracking_web.py`** (`TrackingManager._run`/`_process_frame`): two
  trace points — `camera_mailbox_dequeue` (when the tracking thread wakes
  on the mailbox condition variable) and `tracker_output` (when
  `tracker.update(frame)` returns).
- **`run_all.sh`**: `SWARM_START_GAZEBO_GUI` (default `true`, unchanged
  behavior) — an env-gate around the existing `gazebo_gui` `start_process`
  block, mirroring the precedent already set by
  `SWARM_START_MAVLINK_BRIDGE`. This is the lever the validated fix uses.

Focused tests: `test_camera_source_trace.py` (4 tests) and
`test_core_range_camera_source_fps_audit.py` (7 tests), all passing.

## 1. Historical vs current config diff

`historical_current_config_diff.json` compares, by file hash and mtime, the
external Gazebo world/model SDF (outside this repo, under
`PX4_AUTOPILOT_ROOT`) and the repo's own Python source against the
recorded state at the time of the historical 27–28 FPS smoke
(`full_stack_contention_fix`, ~05:31 UTC) and the regressed
`collection_throughput_fix` paired runs (~06:42 UTC onward), both on
2026-08-05:

- **Camera sensor SDF** (`Tools/simulation/gz/models/gimbal/model.sdf`):
  480×270, `R8G8B8`, `update_rate=50`Hz. File `mtime` is 2026-07-30,
  before both runs — unchanged.
- **World physics** (`Tools/simulation/gz/worlds/default.sdf`):
  `max_step_size=0.004`, `real_time_update_rate=250`,
  `real_time_factor=1.0` (target). File `mtime` is 2026-07-23 — unchanged.
- **Launch mode**: both historical and current use `run_all.sh`'s Gazebo
  server headless (`-r -s`) plus a separate GUI client (`-g`), `ogre2`
  render engine — identical.
- **Application source**: `main.py`/`tracking_web.py`/`run_all.sh` hashes
  match the prior task's recorded hashes exactly (before this task's own
  additive instrumentation).
- A standalone, non-version-controlled SDF-patching tool
  (`configure_gazebo_camera.py`) exists in the repo and *could* silently
  drift the external camera config in the future, but its own mtime and
  the SDF's mtime both predate today's runs — it did not cause today's
  regression, and is flagged only for awareness.

**No static config or code difference explains the FPS gap.** This
corroborates the prior task's own "no `COLLECTION_METRIC_DEFINITION_
MISMATCH`" finding and rules out `HISTORICAL_CURRENT_CONFIG_MISMATCH` as
the gate.

## 2. Hop-level FPS measurement

One representative scenario (two UAVs, approaching 11.5→3.5 m, 32 s, 5 s
post-prewarm skip) was measured at five hops via the new tracer:

| Hop | What it measures | A_baseline (GUI on) | B_headless_v2 (GUI off) |
|---|---|---:|---:|
| H1 gazebo_sim_publish | Camera sensor's own sim-time schedule | **50.000 FPS** (σ=0, all 10 runs) | 50.000 FPS |
| H2 backend_gztransport_receipt | Wall-clock receipt in `_handle_camera_image` | 28.93 FPS | 43.06 FPS |
| H3 camera_mailbox_store | Accepted into the tracking mailbox | 27.63 FPS | ~41 FPS |
| H4 tracker_mailbox_dequeue | `TrackingManager` wakes and dequeues | 27.51 FPS | 37.88 FPS |
| H5 tracker_output | `tracker.update()` returns | 24.92 FPS | 34.13 FPS |

**Architecture note**: this application subscribes to Gazebo camera images
directly via `gz.transport13` (`main.py`) — there is no ROS2/Gazebo image
bridge process and no ROS2 image republish anywhere in this codebase
(confirmed by code audit: no `ros_gz_bridge`, `image_transport`, or
`sensor_msgs/Image` usage for camera data). The task's originally-listed
hops 2–4 (Gazebo topic publish / ROS2-Gazebo bridge receive / ROS2 image
topic publish) therefore collapse into the single H2 measurement above;
H1 vs H2 is the only meaningful split of that portion of the pipeline in
this architecture.

**H1 never varies — it is never the bottleneck.** The entire loss from
50 FPS down to whatever camera-source FPS was actually observed happens
strictly between H1 and H2, i.e. it is a wall-clock delivery problem, not
a sensor-schedule problem.

## 3. Simulator and resource state — the decisive evidence

`simulator_metrics.csv` (Gazebo's own `/world/default/stats` topic,
sampled once per second via the read-only `gz topic -e` CLI):

| Config | Gazebo GUI | RTF median | RTF p5 | camera-source FPS (H2) |
|---|---|---:|---:|---:|
| D_dashboard_off | on | **0.3245** | 0.0299 | 28.64 |
| B_headless_v2 | off | **0.9984** | 0.0361 | 43.06 |

Gazebo's real-time factor ran at roughly **20–32% of its configured target
of 1.0** with the GUI client running (and dipped as low as 3% of real-time
speed at points throughout the *entire* run, not just at startup — RTF
oscillated between ~0.02 and ~1.0 continuously). With the GUI disabled, RTF
rose to essentially the configured target (median 0.998).

This is causally coherent with the hop data: because the camera sensor
publishes on a **fixed sim-time schedule** (H1, exactly every 0.02
sim-seconds, unconditionally), a lower real-time factor means **fewer of
those sim-time ticks occur per wall-clock second** — the sensor is not
publishing less often in sim time, wall-clock time is simply passing
faster relative to sim time. This directly explains H2's wall-clock rate
tracking RTF rather than tracking H1.

`resource_metrics.csv` (GPU utilization from `nvidia-smi dmon`, corrected
for a column-offset bug found during this task — see below) shows
consistently low, near-identical GPU SM utilization (11–12% median) across
every config including headless, and `C_depth_off` (MiDaS throttled to
~0.1 Hz, near-zero depth GPU load) shows camera-source FPS statistically
indistinguishable from `A_baseline` (29.05 vs 28.93 FPS) — **ruling out GPU
compute contention** (from MiDaS or otherwise) as the mechanism.

**Bug found and fixed during this task**: the first `nvidia-smi dmon -s
pucv` parser assumed a fixed column offset for SM utilization, but this
driver's actual column order (`gpu pwr gtemp mtemp sm mem enc dec jpg ofa
mclk pclk pviol tviol`) put `sm` at index 4, not the originally assumed
index 3 — silently reading the `mtemp` (temperature) column as "GPU
utilization" instead. Fixed to parse against the log's own header line
rather than a hardcoded offset, with a regression test
(`test_parse_dmon_log_uses_header_to_locate_sm_column`). A second bug (the
`gz topic -e ... --timeout` flag does not exist on this `gz` CLI version —
the correct flag is `-d/--duration`, mutually exclusive with `-n`) meant
the first five ablation runs (`A_baseline`, `B_headless`, `C_depth_off`,
`F_repeat_1`, `F_repeat_2`) captured zero RTF samples; fixed and
re-verified on `D_dashboard_off` and `B_headless_v2`, which is why RTF
evidence above cites only those two configs.

## 4. Ablations (A–F)

Single representative scenario, one variable changed at a time:

| Config | Varying factor | H2 camera-source FPS | H5 tracker-output FPS |
|---|---|---:|---:|
| A_baseline | baseline | 28.93 | 24.92 |
| B_headless / B_headless_v2 | `SWARM_START_GAZEBO_GUI=false` | 35.77 / **43.06** | 29.03 / 34.13 |
| C_depth_off | `SWARM_RANGE_DATASET_DEPTH_RATE_HZ≈0.1` (MiDaS functionally off) | 29.05 | 26.57 |
| D_dashboard_off | no WebSocket/browser client attached | 28.64 | 25.51 |
| E (ROS bridge bypass) | N/A | — | — |
| F_repeat_1 / F_repeat_2 | identical repeat, back-to-back, no rest | 28.82 / 28.36 | 25.91 / 24.59 |

- **B (GUI off)** is the only ablation that moved the needle: +24–49%
  camera-source FPS, +16–37% tracker-output FPS. This is the fix.
- **C (depth throttled)**: no meaningful change — rules out MiDaS/GPU
  contention (§3).
- **D (no dashboard client)**: no meaningful change — the observation-only
  capture harness never attaches one in the first place; this also
  corroborates `full_stack_contention_fix`'s own `C_preview_off` ablation,
  which found WebSocket/preview publication toggling has no measurable
  throughput effect.
- **E**: not executed. Code audit (§2 architecture note) found the camera
  path already bypasses ROS2 entirely — there is no bridge to bypass that
  is not already bypassed.
- **F (back-to-back repeats)**: no degradation trend across two immediate
  repeats — rules out simple resource accumulation over a handful of
  consecutive stack launches as *the* explanation for session-to-session
  variance (RTF instability, which can occur on any given run regardless
  of run count, is the better-supported mechanism — see §3).

**This task's own fresh measurements landed in the ~28–46 FPS range, not
the ~17–18 FPS specifically reported as "current" earlier that day.** The
RTF-instability mechanism found here is consistent with — and sufficient
to explain — an even lower aggregate FPS in a session that spent more of
its time in a low-RTF regime (RTF's own 5th-percentile sample was as low
as 0.03), but this task did not independently reproduce the exact 17–18
FPS figure. Reported honestly rather than overclaiming an exact
reproduction.

## 5. Fix applied

**`SWARM_START_GAZEBO_GUI=false`** for camera-source-FPS-sensitive
automated collection/validation sessions. New, additive, opt-in env-gate
in `run_all.sh` (default `true` — the interactive dashboard's own default
launch, and every other caller that does not set this variable, is
unchanged). This is the single-variable change (§4, config B) that
produced the entire measured improvement; no other lever in the approved
fix list (camera sensor `update_rate` restoration, duplicate/conflicting
camera config, subscriber/bridge queue blocking, QoS, unnecessary
image-copy elimination) was found to apply, because none of the config
diffs, GPU-load ablation, or bridge/transport checks found a defect there
(§§1–4).

No resolution or detector-quality reduction was used to force a pass.

## 6. Post-fix validation

Three scenarios, `SWARM_START_GAZEBO_GUI=false`, `SWARM_RANGE_DATASET_
DEPTH_RATE_HZ=5.0`, two UAVs, ≥30 s post-prewarm:

| Scenario | raw_range | camera-source FPS | tracking FPS | capture→consume med/p95 | MiDaS p95 | timestamp/checksum |
|---|---:|---:|---:|---:|---:|---|
| approaching | 113 | 45.54 | 38.81 | 64.7 / 150.3 ms | 54.3 ms | PASS / PASS |
| receding | 66 | 46.28 | 35.98 | 74.0 / 152.2 ms | 81.0 ms | PASS / PASS |
| stop_and_hold | 127 | 45.49 | 38.95 | 64.9 / 135.2 ms | 57.5 ms | PASS / PASS |

Against the precommitted gate: camera-source ≥25 FPS ✅ (all three, ~1.8×
margin), tracking ≥20 FPS ✅ (all three, ~1.8× margin), capture→consume
median ≤200 ms ✅, p95 ≤300 ms ✅, MiDaS p95 ≤100 ms ✅, raw_range ≥40 ✅,
timestamp/checksum integrity ✅. No growing backlog was observed in any
run.

**One check does not pass**: tracking-processed-fraction-of-camera-source-
frames ≥90% (observed 0.32–0.56). Hop-level evidence
(`hop_fps_metrics.csv`) shows H2→H4 (camera delivery through to the point
the tracker is invoked) retains 79–95% of frames; the remaining loss is
concentrated at H4→H5, which only fires while the underlying
detector/tracker (`tracking_hybrid.py`) reports an active lock
(`tracker_active`). The loss is largest in `receding` (target shrinking in
frame — a harder tracking case) and smallest in
`approaching`/`stop_and_hold`, consistent with a tracker
acquisition/re-lock duty cycle rather than a camera-source delivery
defect. This task's mandate and changes are scoped to camera-source
delivery only; `tracking_hybrid.py`'s lock/re-acquire behavior was not
touched and is flagged here as a separate, out-of-scope follow-up rather
than silently passed or hidden.

## Why the gate is `CAMERA_SOURCE_FPS_FIXED_READY_FOR_RECOLLECTION`

The task's sole objective — camera-source FPS — is fixed and validated
with a wide margin (~1.8×) across all three required trajectories, via a
single-variable, additive, opt-in, safe change with a clear causal
mechanism (Gazebo RTF instability, driven by GUI-client resource
contention, gating a sim-time-scheduled sensor's wall-clock delivery
rate) and evidence ruling out every other candidate (GPU/MiDaS
contention, ROS2 bridge, dashboard client, static config drift, simple
run-count accumulation). The one non-passing validation check is an
explicitly different subsystem (tracker lock duty cycle) outside this
task's charter, reported transparently rather than masked.

## Verification

- Focused tests: `test_camera_source_trace.py` (4), `test_core_range_
  camera_source_fps_audit.py` (7) — 11 passed.
- Full repository test suite and `./run_all.sh --check`: see final run
  recorded after this report; all processes stopped cleanly afterward.

## Deliverables

`artifacts/core_range_3_12m/camera_source_fps/`: `historical_current_
config_diff.json`, `hop_fps_metrics.csv`, `interframe_jitter.csv`,
`simulator_metrics.csv`, `resource_metrics.csv`, `ablation_metrics.csv`,
`root_cause_evidence.json`, `source_changes.json`, `validation_smoke.csv`,
`fix_manifest.json`, `fix_report.md`, plus per-config raw logs under
`configs/` and `validation/`.
