# CORE_RANGE_SIM_TIME_VALIDATE_COLLECT_AND_TRAIN — Phase 1 report

## Outcome

```text
GROUND_TRUTH_ALIGNMENT_FAILED
```

Phase 1 (deterministic sim-time collection validation) did not pass. Per the
task's own phase-gating rule ("Nếu Phase 1 FAIL, dừng đúng tại bước đó...
Không chuyển sang collection hoặc training"), this session stopped here.
Phase 2 (verify the 5 accepted groups) was completed as a read-only check
and is reported below since it does not depend on Phase 1's outcome. Phase 3
(collect the 3 missing groups) and Phases 4–6 (freeze corpus / train /
evaluate) were **not** started.

## What was validated

Three production scenarios were run with the `sim_time_plugin` driver,
`SWARM_START_GAZEBO_GUI=false`, 5 Hz depth rate, and bbox presets taken
verbatim from `artifacts/core_range_3_12m/dynamic_5hz_headless_retrain/collection_plan.json`
(no fabricated bbox coordinates):

| Run | Scenario | Start→End (m) | BBox preset source |
|---|---|---|---|
| `approaching_a` / `approaching_b` | approaching | 11.5→3.5 | `cdr_approach_center_5hz_headless` |
| `receding` | receding | 3.5→11.5 | `cdr_recede_center_5hz_headless` |
| `stop_and_hold` | stop_and_hold | 11.5→6.0, hold 6s | `cdr_stop_approach_6_5hz_headless` |

`approaching` was run twice (`approaching_a`, `approaching_b`) specifically
to test pose determinism between independent runs of the same scenario/seed.

Full per-run numbers: `artifacts/core_range_3_12m/sim_time_dynamic_retrain/validation_smoke.csv`,
`gt_alignment.csv`, `raw_error_by_rtf.csv`, `trajectory_validation.json`,
`integrity_report.json`.

## Gate results

| Gate | approaching_a | approaching_b | receding | stop_and_hold |
|---|---|---|---|---|
| raw_ranges ≥ 40 | PASS (168) | PASS (201) | PASS (203) | PASS (162) |
| camera FPS ≥ 25 | PASS | PASS | PASS | PASS |
| tracking FPS ≥ 20 | PASS | PASS | PASS | PASS |
| capture→consume median ≤200ms | PASS | PASS | PASS | PASS |
| capture→consume P95 ≤300ms | PASS | PASS | PASS | PASS |
| MiDaS worker P95 ≤100ms | PASS | PASS | PASS | PASS |
| no sim-time driver timeout | PASS (after fix, see below) | PASS | PASS | PASS |
| raw error, RTF-low vs RTF-high ≤20% increase | PASS (−28%) | insufficient stutter samples | PASS (−46%, i.e. lower) | **FAIL (+46%)** |
| **GT alignment (0 malformed/incomplete)** | **FAIL (1/168 bad)** | PASS (0/201 bad) | **FAIL (1/203 bad)** | **FAIL (1/162 bad)** |
| **All gates** | **FAIL** | PASS | **FAIL** | **FAIL** |

3 of 4 runs fail on ground-truth alignment; `stop_and_hold` additionally
fails the RTF-window raw-error gate.

## Finding 1 — driver bug found and fixed: sim-time command publish race

`approaching_b`'s first attempt failed with `sim_time_trajectory_completion_timeout`
(trajectory never started; `trajectory_events.jsonl` stayed empty for the
whole run while raw diagnostics kept accumulating on a static target).

Root cause: `_publish_sim_time_command` in `core_range_dynamic_capture.py`
shells out to `gz topic -t ... -p ...`, which opens a **brand-new** transport
node per invocation and exits immediately after sending. gz-transport's
pub/sub link needs an async peer-discovery handshake between the plugin's
`Subscribe()` call (in `Configure()`) and any publisher; if the one-shot CLI
process publishes and exits before that handshake completes, the message is
silently dropped — no delivery guarantee, no retry. This is a genuine gap in
the sim-time trajectory driver that plain manual smoke-testing (a handful of
successful runs) had not exposed; back-to-back automated runs did.

Fix (`core_range_dynamic_capture.py`): added
`_wait_for_sim_time_command_subscriber()`, which polls
`gz topic -i -t /swarm/sim_time_trajectory/command` for a non-empty
Subscribers section (10s timeout) before the first `start` publish. The
`stop` publish at teardown is unaffected (best-effort, harmless if missed).

Verified: re-running `approaching_b` after the fix succeeded on the first
attempt (`exit=0`), with the trajectory completing normally and 100%
GT-trace-valid frames. Unit tests unaffected
(`test_sim_time_trajectory_and_gt.py` 7/7 passed).

This fix is scoped entirely to the sim-time trajectory driver added in the
prior session; it does not touch PX4, the controller, EKF, M52, or
calibration semantics.

## Finding 2 — pose determinism: PASS, exact

Comparing `approaching_a` vs `approaching_b` (same scenario, same seed,
independent runs): matching poses by **elapsed time since trajectory
activation** (not absolute `sim_timestamp_s`, which differs run to run
because prewarm/bbox-selection wall-clock timing shifts when the trajectory
activates — observed 19.62s vs 21.276s activation, both spanning exactly
31.8s / 160 steps) gives **160/160 matched pairs, max abs diff = 0.0 m**.
The trajectory is a pure function of elapsed sim-time, confirmed bit-exact
across independent runs regardless of RTF variation during either run.

## Finding 3 — ground-truth pose-bracket race (root cause of the FAIL conclusion)

3 of 4 runs each have exactly 1 frame (of 162–203) where
`main.py`'s Gazebo pose-history bracket lookup returns
`"Gazebo pose bracket unavailable"` (`diagnostic_complete=False`,
`ground_truth_trace_valid=False`):

| Run | Bad frame | source_sim_timestamp_s |
|---|---|---|
| approaching_a | frame 1400 | 37.74 |
| receding | frame 473 | 19.44 |
| stop_and_hold | frame 524 | 20.24 |
| approaching_b | none | — |

Checked and ruled out:
- **Not RTF-correlated.** approaching_a's bad frame at sim_t=37.74 falls in
  a window where the surrounding RTF samples (35.98→37.02→38.08) are all
  ≈1.0; the nearest actual RTF dips in that run are at sim_t=33.92 (0.60)
  and 42.32 (0.44), several seconds away.
- **Not deterministic / not tied to a fixed geometry moment** — three
  different absolute and elapsed timestamps across three scenario types.
- **Rate is consistent with a rare async race**, not a systemic break: ~0.5–0.6%
  of frames per run (1/168, 1/203, 1/162), 0/201 in the fourth run — plausible
  as an independent per-frame race at that probability (P(zero hits in
  ~150–200 draws at p≈0.005) ≈ 0.45–0.5, consistent with seeing 3 hits and 1
  miss across 4 runs).
- **Cross-checked against the historical accepted/quarantined corpus**:
  all 26 quarantined attempts for the 3 missing groups (legacy
  `legacy_wall_clock` driver) were rejected for `gazebo_rtf`/
  `no_long_gazebo_stutter` or `gt_coverage`/`stop_hold_evidence_insufficient`
  reasons — **none** show a `core_logging_gate_failed` (which is what a
  GT-trace failure would produce). This suggests the bracket-unavailable
  race is specific to, or at least much more frequent under, the
  `sim_time_plugin` driver's continuous per-physics-step pose commands
  versus the legacy driver's discrete 5 Hz synchronous `set_pose` calls —
  plausibly because it changes the relative timing between target-pose
  publishes and camera-frame arrivals in a way that occasionally exposes a
  pre-existing race in `main.py`'s pose-history bracket lookup
  (`lower_source`/`upper_source` search over a 600-entry deque).

This was **not** fixed in this session: `main.py`'s ground-truth pose
provider is shared runtime code serving the live dashboard, not code
introduced by the sim-time trajectory work, and root-causing why gz-sim's
pose-history publish cadence interacts differently with a PreUpdate-driven
target versus a discrete `set_pose` target needs its own investigation. The
task's Phase 1 gate list requires GT alignment PASS unconditionally (no
insufficient-evidence escape clause, unlike the RTF-window gate); 3/4 real
runs violating that bar is not something to wave through.

## Finding 4 — RTF-window raw-error gate: reinforces, does not contradict, the standing RTF hard-gate conclusion

`stop_and_hold`'s stutter-window (RTF≤0.3) raw-range MAE was 46% higher than
its stable-window (RTF≥0.8) MAE, exceeding the ≤20% threshold. This is fresh
`sim_time_plugin` evidence pointing the same direction as the prior
`CORE_RANGE_GAZEBO_RTF_STUTTER_AND_GATE_VALIDATION` finding (raw accuracy
measurably degrades at low RTF), so it does not change that conclusion —
**RTF stays a hard gate**.

## Phase 2 — five accepted groups re-verified (independent of Phase 1 outcome)

All 5 previously accepted dynamic groups
(`cdr_approach_center_5hz_headless`, `cdr_approach_left_5hz_headless`,
`cdr_approach_right_yaw_5hz_headless`, `cdr_recede_center_5hz_headless`,
`cdr_stop_recede_9_5hz_headless`) were re-hashed against their recorded
`source_capture_sha256`/`source_sidecar_sha256`/`source_trajectory_sha256`
in `accepted_sessions.json`: **all 5 match exactly**, `integrity_status:
PASS`, all per-session gates true, 96 anchors/frame. None appear in
`quarantined_sessions.json`. Source files unmodified since collection. These
five groups do not need to be recollected.

The 3 missing groups (`cdr_recede_left_yaw_5hz_headless`,
`cdr_recede_right_5hz_headless`, `cdr_stop_approach_6_5hz_headless`) are
each quarantined at 8/8 attempts, consistently for RTF/long-stutter reasons
— matches the standing `DATA_COLLECTION_BLOCKED_BY_PERSISTENT_RTF_STUTTER`
conclusion from the prior session.

## What was not done (blocked by the Phase 1 FAIL)

- Phase 3 (collect the 2 missing receding + 1 missing stop_and_hold groups)
- Phase 4 (freeze the 8/8 corpus manifest)
- Phase 5 (train PHYSICAL_ONLY / PHYSICAL_TEMPORAL)
- Phase 6 (static/dynamic/temporal/bbox-robustness evaluation)

A training driver already exists and is unit-tested for phases 4–6
(`core_range_5hz_headless_retrain.py`, built in a prior session) and is
ready to use once a real Phase 1 PASS is reached and the 8/8 corpus is
assembled — it was not invoked in this session.

## Scope guards honored

- No PX4, controller, EKF, M52, or calibration changes.
- No Follow Target, no FPS optimization work.
- No model trained; no missing-group collection attempted.
- The one code change made (`core_range_dynamic_capture.py`'s subscriber-wait
  fix) is scoped to the sim-time trajectory driver added in the prior
  session and was verified with a passing re-run plus the full test suite.

## Tests

- `test_sim_time_trajectory_and_gt.py` + `test_gazebo_ground_truth_history.py`
  + `test_range_physical_diagnostics.py`: 25 passed.
- Full repository suite (`pytest -q --ignore=swarm_dashboard_handoff_20260729`,
  the ignored path is an unrelated stale backup copy nested in the repo):
  **424 passed**.
- `./run_all.sh --check`: PASS.
- All Gazebo/PX4/ROS2/backend/capture processes stopped cleanly after every
  run in this session.

## Next steps

1. Root-cause the GT pose-bracket race under `sim_time_plugin` before
   attempting Phase 1 again — likely needs instrumentation in `main.py`'s
   pose-history callback and the SceneBroadcaster publish cadence under a
   PreUpdate-driven target, compared against the legacy driver's cadence.
2. Do not loosen the GT-alignment gate to force a PASS.
3. Do not proceed to Phase 3/4/5/6 until Phase 1 genuinely passes.

**Update 2026-08-06**: item 1 done — see
`docs/CORE_RANGE_GT_BRACKET_RACE_FIX_REPORT.md`. Items 3's blocker is
cleared; a later overnight session (`docs/CORE_RANGE_OVERNIGHT_SIM_TIME_TRAIN_REPORT.md`)
completed missing-group collection (8/8) and training. No candidate met
the accuracy/temporal/bbox gates
(`NO_SIM_TIME_DYNAMIC_ROBUST_MODEL_MEETS_GATE`), so this is a data-quality
finding, not a blocked pipeline.
