# CORE_RANGE_2HZ_DYNAMIC_RECOLLECTION_AND_RETRAIN

**Conclusion: `TWO_HZ_DYNAMIC_CORPUS_INCOMPLETE`**

Goal: recollect the 8-group dynamic development corpus at the new 2.0Hz
depth-scheduler default, then train/evaluate `PHYSICAL_ONLY` and
`PHYSICAL_TEMPORAL` XGBoost variants against it. Training was not reached —
the data integrity and latency gate (Phase 3) did not pass.

All artifacts referenced below live under
`artifacts/core_range_3_12m/dynamic_2hz_retrain/`.

## Phase 1 — Preflight: PASSED

- Scheduler default confirmed at 2.0Hz (`metric_target_fusion.py:118`), no
  env override present.
- Residual correction confirmed default-off.
- No stale PX4/Gazebo/backend/capture/train processes; 19GB free on `/`,
  14GB free on `/mnt/px4ssd` (healthy `rw` `/dev/loop14` overlay mount).
- `./run_all.sh --check` passed.
- Focused tests (logging/GT-trace/checksum/96-anchor contract): 31 passed.
- Full repository test suite: found and fixed 2 pre-existing regressions in
  `test_metric_target_fusion.py` caused by the 2.0Hz default change (two
  tests implicitly assumed the old 7.5Hz submission cadence at their fixed
  0.2s test timestep; fixed by pinning `fusion.depth_rate_hz = 10.0`
  explicitly in each test — no production code, gate, or calibration
  semantics changed). 380 tests passed after the fix.

See `preflight_report.json`.

## Phase 2 — Collection: 7/8 groups

Trajectories were numerically identical to the original frozen 8-session
plan (`core_range_direct_dynamic_replay.py:_sessions()`) for comparability;
new tooling (`core_range_collect_dynamic_2hz_batch.py`,
`core_range_collect_dynamic_2hz_scenario.sh`) used a distinct
`core_range_dynamic_2hz_development` dataset role and a raised 90s (from
30s) pre-motion capture timeout, since the original 30s floor was tuned for
the 7.5Hz era and proved too tight at 2.0Hz.

| Group | Scenario | Result | Frames |
|---|---|---|---|
| cdr_approach_center_2hz | approaching | ACCEPTED (attempt 2) | 42 |
| cdr_approach_left_2hz | approaching | ACCEPTED (attempt 1) | 42 |
| cdr_approach_right_yaw_2hz | approaching | ACCEPTED (attempt 2) | 46 |
| cdr_recede_center_2hz | receding | ACCEPTED (attempt 2) | 53 |
| **cdr_recede_left_yaw_2hz** | receding | **FAILED — 5/5 attempts** | 0 |
| cdr_recede_right_2hz | receding | ACCEPTED (attempt 1) | 56 |
| cdr_stop_approach_6_2hz | stop_and_hold | ACCEPTED (attempt 1) | 42 |
| cdr_stop_recede_9_2hz | stop_and_hold | ACCEPTED (attempt 1) | 49 |

`cdr_recede_left_yaw_2hz` was retried 5 times total (2 precommitted + 3
additional, all against the identical unmodified session spec — no
parameter or source changes between attempts) and failed identically every
time: `inverse_depth_filter_rejected` on ~97% of frames (calibration itself
converged fine; the target ROI's inverse-depth extraction did not). This
same session geometry (lateral -0.75m + yaw sweep, receding) also required
repeated recovery attempts in the original 7.5Hz-era corpus (see
`artifacts/core_range_3_12m/dynamic_repair_and_replay/`), so this is a
known-hard scenario, not a 2.0Hz-specific regression.

Accepted total: **7 groups, 330 frames** (3 approaching, 2 receding, 2
stop-and-hold).

See `collection_plan.json`, `dynamic_session_manifest.json`,
`accepted_sessions.json`, `quarantined_sessions.json`.

## Phase 3 — Data integrity and latency gate: FAILED

**Per-frame integrity: PASS.** All 7 accepted sessions individually passed
`audit_session()`'s checksum/trace/timestamp-ordering/96-anchor gate: 0
malformed JSON lines, 0 duplicate traces, 100% GT-trace and record-checksum
integrity, 100% timestamp ordering, 96 anchors/frame throughout.

**Group completeness: FAIL.** 7/8 (receding 2/3, required 3/3).

**Latency gate: FAIL — and this is the more consequential finding.**

| Corpus | Nominal rate | capture→consume median | P95 | n frames |
|---|---|---|---|---|
| Old dynamic corpus (7.5Hz era) | 7.5Hz | 1.131s | 1.566s | 844 |
| **New 2.0Hz corpus** | 2.0Hz | **1.226s** | **1.849s** | 330 |
| Required gate | — | ≤0.200s | ≤0.300s | — |

The new corpus **fails the latency gate by ~6x on median**, and is **not
materially better than the old 7.5Hz-era corpus** — despite the
`depth_scheduler_contention_fix` task's prior conclusion that 2.0Hz was the
only rate meeting this exact gate (median 42.8ms, P95 68.5ms; see
`artifacts/core_range_3_12m/depth_scheduler_contention_fix/`).

**Root cause of the discrepancy.** A sample-row check on both corpora
confirms this is not a measurement artifact:

- In this task's new corpus, `depth_worker_start → depth_complete` (the
  actual MiDaS inference call inside `LatestDepthWorker`) takes 0.5-1.3s per
  call — essentially the same contention symptom the depth-scheduler-
  contention-fix task set out to resolve.
- The earlier 42.8ms figure came from `config F` of the contention-isolation
  task: a short (16s trajectory), single-vehicle-adjacent isolated smoke
  test whose depth pipeline never once produced a successful
  `raw_range_computed` row (100% `calibration_rejected`/`insufficient_
  anchors`/`ransac_consensus_failed`). Every row in that specific run
  happened to have genuinely low MiDaS latency (~40ms) — that is real data,
  but it does not generalize to a realistic full two-vehicle, 32s-trajectory,
  multi-group dynamic collection, which is exactly what this task exercises.

In short: **lowering the depth-submission rate to 2.0Hz does not reliably
clear the ROS/thread contention under realistic load.** The
`depth_scheduler_contention_fix` conclusion was correct for the specific
evidence it had at the time, but that evidence was not representative.

See `integrity_report.json`, `latency_comparison.csv`, `raw_dynamic_metrics.csv`.

## Phases 4-8: not started

Per task instructions, once the Phase 3 gate failed, no training corpus was
assembled, no model variant was trained, no evaluation was run, and no
candidate was selected. No gate was lowered and no result was forced to
pass. The depth rate was left at 2.0Hz throughout (out of scope to change
in this task). See `retrain_manifest.json` and `retrain_report.md`.

## What is and isn't affected

- **Not touched:** M52, calibration semantics, controller, PX4, depth rate
  (remains 2.0Hz), residual correction (remains default-off).
- **No shadow, no active runtime integration, no Follow Target.**
- **New files added** (data-collection tooling only, no runtime behavior
  change): `core_range_collect_dynamic_2hz_batch.py`,
  `core_range_collect_dynamic_2hz_scenario.sh`, plus a new `--dataset-role`
  choice added to `core_range_dynamic_capture.py`'s argparse (`core_range_
  dynamic_2hz_development`, additive, does not change the existing
  `core_range_dynamic_development` default).
- **Test fix:** `test_metric_target_fusion.py`'s two convergence tests now
  pin `depth_rate_hz` explicitly instead of relying on the ambient default.

## Recommended follow-up (not performed here, out of this task's scope)

1. Re-investigate the ROS2/mavlink-bridge/dashboard-consumer-thread
   contention directly (the depth-scheduler-contention-fix task's own
   `not_fully_isolated` caveat already flagged this as unresolved) rather
   than relying on submission-rate throttling alone — throttling to 2.0Hz
   did not hold up under realistic load.
2. Root-cause why `cdr_recede_left_yaw`'s specific bbox/lateral/yaw
   combination systematically fails inverse-depth ROI extraction; this
   predates the 2.0Hz change and previously needed the
   `SWARM_TARGET_ROI_RECOVERY_POLICY` geometry-specific recovery statistic
   in the 7.5Hz-era corpus.
3. Once (1) produces a corpus that actually meets the latency gate, this
   task's Phase 2 collection tooling (`core_range_collect_dynamic_2hz_
   batch.py`) and Phase 4-8 plan can be reused directly.

## Final verification

- `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 pytest -q --ignore=.venv --ignore=swarm_dashboard_handoff_20260729 --ignore=build`: 380 passed.
- `./run_all.sh --check`: passed.
- No PX4/Gazebo/ROS2/backend/capture/training processes left running.
