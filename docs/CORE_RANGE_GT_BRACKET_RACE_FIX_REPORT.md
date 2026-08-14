# CORE_RANGE_GT_BRACKET_RACE_FIX

## Outcome

```text
GT_BRACKET_RACE_FIXED_READY_FOR_MISSING_GROUPS
```

The race between camera-frame processing and Gazebo pose-history updates
that caused ~0.5–0.6% of frames to fail ground-truth alignment (found by
`CORE_RANGE_SIM_TIME_VALIDATE_COLLECT_AND_TRAIN` Phase 1) is fixed and
validated: **0 bad GT frames across 752 raw frames in 4 fresh runs**
(previously 3/4 runs each had exactly 1). RTF remains an unrelated, still
unresolved hard gate — one run (`receding`) failed the RTF-window raw-error
gate on a real stutter, which is expected and out of this fix's scope.

## Root cause (traced exactly as requested)

Pipeline: camera frame timestamp → pose-history append → bracket lookup →
camera pose interpolation → target pose interpolation → GT distance
construction → diagnostics write.

- Pose-history append (`GazeboDashboardBridge`, `main.py`) runs on the
  Gazebo pose-topic subscription callback thread, appending to a
  timestamp-ordered 600-entry deque under `self.pose_lock`.
- Bracket lookup (`simulation_target_ground_truth`) runs synchronously on
  the tracking loop thread (`tracking_web.py`'s
  `_update_metric_target_fusion_locked`, itself holding `TrackerState.lock`),
  scanning `pose_history` for the two records bracketing the camera frame's
  `source_sim_timestamp_s`.
- The race: if the camera frame's timestamp is *newer* than every pose
  sample received so far (`upper_source` not found), the old code failed
  the frame immediately with `"Gazebo pose bracket unavailable"` — even
  though the pose callback thread was about to append exactly the sample
  needed, often within a physics step or two. This is not RTF-correlated
  (confirmed in the prior validation): it's a pure ordering race between
  two independently scheduled threads.
- The other missing-bracket case — camera timestamp *older* than the
  oldest buffered sample (`lower_source` not found) — is fundamentally
  different: no future sample can ever fill that in, since time only moves
  forward. This case was, and still is, an immediate failure.

## Fix (`main.py`, smallest change that closes the observed race)

- Added `self.pose_history_condition = threading.Condition(self.pose_lock)`,
  notified every time a new sample is appended.
- `simulation_target_ground_truth`: when `upper_source` is missing, waits
  on that condition (bounded by `GT_BRACKET_WAIT_TIMEOUT_S = 0.12s`),
  re-running the bracket search after each wake, until either the bracket
  completes, a session-boundary reset fires, or the timeout elapses. The
  `lower_source`-missing case is **not** retried (unchanged, immediate
  failure) since waiting can never resolve it.
- On timeout, returns `"error": "GT_BRACKET_TIMEOUT"` (distinct from the
  unrelated immediate-failure message) — the frame is dropped with that
  reason; no GT value is fabricated.
- A bounded concurrent-waiter cap (`GT_BRACKET_MAX_PENDING_WAITERS = 4`)
  fails fast with `"GT_BRACKET_PENDING_LIMIT_EXCEEDED"` instead of ever
  queuing without bound (in this codebase's architecture there is in
  practice only ever one synchronous caller, so this is a defensive bound,
  not something normally hit).
- `reset_ground_truth_pending()`: bumps a generation counter and wakes any
  in-flight wait so it abandons immediately instead of resolving against —
  or blocking return of — data from a session that has already moved on.
  Hooked into `TrackerState.start()`, `stop()`, and `set_bbox()` in
  `tracking_web.py` (all three session-boundary points that change
  `follow_workflow.session_id` / the drone being tracked), guarded with
  `getattr(..., None)` so it's a no-op for any caller/test double that
  doesn't expose the method.
- Never uses nearest-pose fallback or extrapolation: the accept condition
  (`t_before <= t_camera <= t_after`, exact interpolation) is completely
  unchanged from before this fix — only *how long the code is willing to
  wait for that condition to become true* changed.
- Does not touch M52, calibration, PX4, the controller, or trajectory
  semantics (`sim_time_trajectory.py`/the Gazebo plugin are untouched).

Bounded-synchronous-wait rationale (vs. a fully async pending-frame
queue): `simulation_target_ground_truth` is called exactly once,
synchronously, per frame, from a single caller thread in this codebase —
there is no concurrent multi-frame queue to build. A short bounded wait
(120ms cap, 5Hz depth submission period is 200ms) on the rare race frame
adds negligible latency and does not measurably affect the tracker/depth
loop's throughput gates (see validation numbers below) — the ones this
task's "không block tracker/depth loop" constraint is protecting.

## Tests (10, all in `test_gazebo_ground_truth_history.py`)

1. `..._copies_only_bracketing_pose_records` — unchanged immediate-hit path (regression, pre-existing).
2. `..._lower_missing_fails_immediately_no_wait` — camera older than oldest sample: fails in <50ms, no retry attempted.
3. `..._upper_missing_times_out_with_no_fabricated_gt` — nothing ever arrives: `GT_BRACKET_TIMEOUT`, no distance value, waits the full bounded timeout (not instant, not indefinite).
4. `..._resolves_after_late_pose_arrives` — frame arrives before its bracket exists; a second thread appends the missing sample 30ms later; resolves correctly with the same interpolation as the no-wait path, `bracket_retry_count >= 1`.
5. `..._ignores_insufficient_intermediate_pose` — an intermediate pose sample that still doesn't reach the requested timestamp must not be accepted as the bracket; only the genuinely sufficient later sample is used (verifies the correct bracket bounds, not a premature pairing) — out-of-order-arrival-safety.
6. `test_reset_ground_truth_pending_abandons_wait_without_writing_gt` — a session-boundary reset during an in-flight wait abandons it immediately (well under the configured 5s timeout) with no fabricated GT.
7. `test_simulation_ground_truth_pending_limit_fails_fast` — at the concurrent-waiter cap, a new call fails immediately with `GT_BRACKET_PENDING_LIMIT_EXCEEDED`.
8. `test_simulation_ground_truth_called_exactly_once_per_frame` — the whole retry/wait lives inside one call; a caller never invokes it twice for one frame, so exactly one record is ever produced for it.
9/10. `..._target_entity_missing.../..._camera_entity_missing...` — a *time*-complete bracket whose snapshot is still missing one entity's pose (a different, unaddressed race) correctly still fails with the existing, unchanged `"camera link or target model pose unavailable"` error rather than silently retrying into a wrong answer or being swept into this fix's scope.

## Live validation (real Gazebo, not simulated)

Re-ran all four `CORE_RANGE_SIM_TIME_VALIDATE_COLLECT_AND_TRAIN` Phase 1
scenarios with the fix in place: `sim_time_plugin` driver, GUI off, 5Hz
depth, production bbox presets from `collection_plan.json`.

| Run | Raw frames | Bad GT frames | Frames recovered via retry | All gates |
|---|---:|---:|---:|---|
| approaching_a | 189 | **0** | 0 | PASS |
| approaching_b | 133 | **0** | 0 | PASS |
| receding | 255 | **0** | **3** (frames 477, 507, 537) | FAIL (RTF-window, unrelated) |
| stop_and_hold | 175 | **0** | **2** (frames 622, 1289) | PASS |
| **Total** | **752** | **0** | **5** | |

The 5 recovered-via-retry frames are direct, concrete proof the fix does
real work: each had `bracket_retry_count: 1` (one wait/retry cycle),
meaning the exact race condition fired and was closed by the fix — before
this change, each of those 5 frames would have failed with `"Gazebo pose
bracket unavailable"`, matching precisely the pattern found in Phase 1
(1 bad frame per ~150–200, not RTF-correlated).

Pose determinism (approaching_a vs approaching_b, same scenario/seed):
**still exact** — 160/160 matched elapsed-time pairs, max abs diff 0.0m.
Unaffected by this fix, as expected (it touches only the GT-lookup path,
not the trajectory driver).

Per-run throughput/latency (all comfortably inside gate):

| Run | Camera FPS | Tracking FPS | capture→consume P95 | MiDaS P95 |
|---|---:|---:|---:|---:|
| approaching_a | 49.3 | 45.2 | 111ms | 57ms |
| approaching_b | 49.4 | 45.8 | 103ms | 58ms |
| receding | 37.5 | 32.3 | 117ms | 69ms |
| stop_and_hold | 50.7 | 44.8 | 70ms | 39ms |

All ≥25/≥20 FPS and ≤300ms/≤100ms latency gates pass in every run,
including the three runs that hit the race and retried — confirming the
bounded wait does not measurably degrade throughput.

## The one gate failure: `receding`, RTF-window raw error — expected, out of scope

`receding`'s stutter-window (RTF≤0.3, n=43) raw-range MAE was 3.58m vs.
2.61m in the stable window (RTF≥0.8, n=159) — a 37% increase, over the
20% threshold. `gazebo_rtf.long_stutter_pass=False` (3 consecutive
low-RTF samples occurred during this run). This is the same,
already-standing `RTF_REMAINS_HARD_GATE_STUTTER_UNRESOLVED` finding, not
a regression from this fix and not something this task's gates ask to be
fixed. Per the task's own instruction, low-RTF windows are not used to
accept collection data — RTF stays a hard gate.

## Tests

- `test_gazebo_ground_truth_history.py`: 10/10 passed (2 pre-existing
  updated for the new bounded-wait/timeout behavior, 8 new).
- Broader regression sweep (`test_gazebo_ground_truth_history.py`,
  `test_sim_time_trajectory_and_gt.py`, `test_range_physical_diagnostics.py`,
  `test_follow_workflow.py`, plus the tracking/pointing/follow-target test
  files that exercise `TrackerState`): 100/100 passed.
- Full repository suite (`pytest -q --ignore=swarm_dashboard_handoff_20260729`,
  the ignored path is an unrelated stale backup copy nested in the repo):
  **432 passed** (up from 424 — 8 net new tests).
- `./run_all.sh --check`: PASS.
- All Gazebo/PX4/ROS2/backend/capture processes stopped cleanly after every
  run.

## Scope guards honored

- No PX4, controller, EKF, M52, or calibration change.
- No nearest-pose fallback, no extrapolation, no commanded-trajectory-as-GT
  substitution, no gate lowered.
- No missing-group collection or training attempted in this task.

## Next steps

1. This fix directly unblocks the Phase 1 gate that stopped the prior
   session (`GROUND_TRUTH_ALIGNMENT_FAILED`) — a fresh Phase 1 run would
   now be expected to pass that specific gate.
2. RTF remains a separate, still-open hard gate; missing-group collection
   (2 receding + 1 stop_and_hold) still needs runs that clear the RTF
   median/no-long-stutter gates on top of this fix, same as before.
3. Not done in this task: missing-group collection, corpus freeze, or
   training — those remain gated behind a full Phase 1 pass under the
   `CORE_RANGE_SIM_TIME_VALIDATE_COLLECT_AND_TRAIN` task, which this fix
   feeds into but does not itself re-run end to end.
