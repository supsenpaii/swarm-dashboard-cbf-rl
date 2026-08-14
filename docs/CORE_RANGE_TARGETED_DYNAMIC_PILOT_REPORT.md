# CORE_RANGE_TARGETED_DYNAMIC_PILOT_COLLECTION_STAGE_2A

## Outcome

```text
PILOT_BLOCKED_BY_RTF_STUTTER
```

Stage 2A of the residual-bias-correction follow-up roadmap set out to
collect 14 targeted, independent dynamic groups (near/mid/far distance
bands x center/lateral/yaw geometry x approach/recede/stop-and-hold
motion) to test whether Candidate B's persistent regional bias (found in
Giai đoạn 1, `docs/CORE_RANGE_RESIDUAL_BIAS_CORRECTION_REPORT.md`)
reproduces outside the original 8-group dynamic corpus. Phase 0
(preflight) and Phase 1 (precommit) both passed. Section 7's RTF soak
preflight -- a mandatory gate before any collection -- failed twice on
genuine grounds after clean restarts, which is this task's own explicit
stop condition. No 14-group collection was attempted.

## History within this task line

This exact task first ran earlier the same day and stopped at
`PILOT_BLOCKED_BY_STORAGE` (13.54GB free < 15GB required; see the
`storage_preflight.json` / prior report content preserved in
`artifacts/core_range_3_12m/targeted_dynamic_pilot/`'s git-less history
via the archived `rtf_soak_attempt*` and `source_changes.json` trail). A
follow-up storage-recovery task inventoried the filesystem and found one
verified-safe, wide-margin action (B1-B4: compress+checksum-verify+delete
~22GB of old, pre-core-range PX4 SITL console-spam logs, confirmed by
sampling to be non-telemetry shell-prompt-redraw output) but left it for
explicit user approval per its own risk-tiering rules. This run began by
presenting that choice to the user, who approved it. It was executed
(8/8 files, 0 checksum mismatches, ~22GB freed -> 35.76GB free), and this
task then re-ran from Phase 0.

## Phase 0 preflight

| Check | Result |
|---|---|
| 4.1 Candidate B reproduction | **PASS** -- bit-exact, `max_abs_diff_m=0.0`, re-verified after the one source edit below |
| 4.2 Runtime source verification | **PASS** -- all 10 required fixes/files confirmed present (bracket-only GT copy, GT bracket wait/retry, deterministic sim-time trajectory plugin + compiled `.so`, overlay world SDF, subscriber-ready wait, headless startup gate, production MiDaS rate, production bbox presets, scoped ROI recovery) |
| 4.3 Disk-space preflight | **PASS** -- 38.39GB free after the approved storage recovery |
| 4.4 Process cleanup | **PASS** -- no stale Gazebo/PX4/MicroXRCE/backend/collector process |

## Phase 1 precommit

14 scenarios were precommitted in `pilot_precommit.json` /
`scenario_matrix.csv` before any soak or collection attempt, reusing
geometry/trajectory conventions (lateral offset magnitude, yaw range,
gimbal-pitch-per-context pattern, `pose_update_rate_hz=5.0`, `seed=52`,
`roi_policy=default`) read directly from
`overnight_sim_time_train/collection_plan_snapshot.json` (the accepted
8-group corpus's own precommit). The only new values introduced, all
disclosed in `pilot_precommit.json`'s own `note` field, were: (a)
start/end distance restricted to each of the task's own bands, (b)
correspondingly reduced trajectory speed for the narrow near (0.04 m/s)
and far (0.09 m/s) bands so a >=20s observation window still fits inside
an 0.8-1.8m span without crossing the band boundary, and (c)
`initial_bbox_normalized` values for the new start distances, computed via
disclosed log-linear interpolation/extrapolation of the camera-projection
scale and per-context frame-center offset from the two calibrated
reference points (3.5m, 11.5m) already present in that same corpus.

`runtime_gate_contract.json`'s per-group gates are reused verbatim from
the same source (`camera_source_median_fps_min=25`,
`tracking_median_fps_min=20`, `gazebo_rtf_median_min=0.8`,
`gazebo_max_consecutive_rtf_le_0_3_samples=2`,
`capture_consume_p95_s_max=0.3`, `worker_p95_s_max=0.1`,
`effective_rate_hz=5.0`), not loosened.

## Section 7 — RTF soak preflight (where this run stopped)

Four attempts total; only the last two are genuine RTF measurements per
the task's own 2-attempt budget (the first two were tooling/infrastructure
failures that produced zero RTF data and are excluded from that budget,
consistent with the budget's purpose of bounding genuine stutter-recovery
retries, not scripting bugs):

| Attempt | Methodology | Result | Detail |
|---|---|---|---|
| 1 | Persistent multi-rep stack + `gz-transport` subscription monitor (`core_range_gazebo_rtf_long_run.sh`) | Infra failure, not counted | Own invocation bug: relative `--root` path vs. `core_range_dynamic_capture.py`'s absolute-path resolution mismatch; ~1100 reps failed instantly (`capture_root_does_not_match_backend_dataset_root`), zero RTF data obtained. This methodology also carries two documented caveats from the earlier `docs/CORE_RANGE_GAZEBO_RTF_STUTTER_REPORT.md` investigation (its persistent-subscription RTF sampling showed volatility the validated `gz topic -e` method doesn't necessarily see the same way; its multi-rep reuse of one stack has a known sidecar-session-tracking bug affecting tracking-FPS/latency measurement for reps after the first) -- switched away from it for that reason, not because attempt 1's own bug was unfixable. |
| 2 | Single continuous session, `gz topic -e` fresh-process RTF sampling (the method validated across all 8 accepted corpus groups) | Infra failure, not counted | Reproduced `--dataset-role` argparse-choices gap in `core_range_dynamic_capture.py` (every prior task line added its own value the same way; this task's value was simply not yet added). Fixed with a single additive line; Candidate B re-verified bit-exact afterward. |
| 3 | Same as 2, clean stack, ~410s window | **FAIL** (genuine, counted) | Median RTF 0.9987 (PASS), camera-source median FPS 46.6 (PASS), but 7 recurring streaks of 3-5 consecutive 1s samples at RTF<=0.3 spread across the session (longest=5, gate requires <=2), first appearing ~100s in and recurring roughly every 40-70s thereafter. |
| 4 | Same as 3, after full clean process restart (Section 7's required retry) | **FAIL** (genuine, counted) | The mandatory pre-capture RTF precheck (5 samples, run before any movement starts, same contract as every accepted per-group collection) measured median RTF=0.244, far below the 0.8 minimum; the script correctly aborted (no capture-window data obtained). |

Per Section 7's own explicit rule and the task's blanket instruction not
to lower any gate to force a pass, no third attempt was made.

## New evidence for the standing RTF-stutter question

Attempt 3's pattern -- excellent median RTF but recurring multi-second
stutter streaks that only begin to appear after roughly 100 seconds of
continuous runtime -- is new evidence consistent with, and sharpening, the
still-unresolved finding in `docs/CORE_RANGE_GAZEBO_RTF_STUTTER_REPORT.md`
("root cause of the stutter itself remains unresolved... most likely
OS-level scheduling contention"). It also explains why this pattern was
never observed in the accepted 8-group corpus: every one of those
sessions' individual capture windows (20-32s) ends well before the ~100s
mark where the pattern begins to appear in this run's longer window.
Attempt 4's outright precheck failure (median RTF 0.244 within the first
5 seconds of a fresh, otherwise-identical stack) suggests some additional
source of variability beyond pure duration-correlation, possibly
session-to-session machine load state; this task did not have budget to
investigate further under its own 2-attempt limit.

## What was not done (by design -- gated on this preflight)

Per Section 7's own rule, none of the following were attempted: Section 8
(14-group collection), Section 9 (per-group acceptance), Section 10
(pilot data audit / Candidate B offline inference / bias audit), or
Section 11 (feature stability audit). No Gazebo/PX4/ROS2/backend session
beyond the 4 soak attempts (all disarmed, no arm/takeoff/OFFBOARD/Follow
Target endpoint called in any of them) was started.

## Source changes

One additive line in `core_range_dynamic_capture.py` (new
`--dataset-role` choice), plus 3 new files specific to this task
(`core_range_collect_targeted_pilot_scenario.sh`,
`core_range_collect_targeted_pilot_batch.py`,
`core_range_rtf_soak_preflight_scenario.sh`). No change to GT definition,
M52/MiDaS semantics, camera geometry, calibration, alpha-beta parameters,
Candidate B, or PX4/controller code. See `source_changes.json`.

## Tests

- 12 new focused tests (`test_core_range_collect_targeted_pilot_batch.py`):
  scenario-matrix completeness, unique IDs, deterministic trajectory spec,
  distance-regime validation, direction validation, stop-and-hold
  detection, RTF median/streak gate logic, Candidate B feature
  order/checksum, partial-pilot-cannot-proceed, no PX4/controller side
  effect, and that this run's own gate-result/summary files are internally
  consistent -- all pass.
- Full repository suite: 471/471 passed (up from 459; no regression).
- `./run_all.sh --check`: PASS.
- No Gazebo/PX4/MicroXRCE/backend/collector process running at the end.

## Deliverables

`artifacts/core_range_3_12m/targeted_dynamic_pilot/`: `pilot_precommit.json`,
`scenario_matrix.csv`, `runtime_gate_contract.json`, `data_contract.json`,
`source_artifact_checksums.json`, `storage_preflight.json`,
`phase0_preflight.json`, `rtf_soak_rows.csv`, `rtf_soak_summary.json`,
`attempt_manifest.csv`, `accepted_group_manifest.json` (empty),
`quarantine_manifest.json` (empty), `source_file_manifest.json`,
`candidate_b_inference_contract.json`, `stage_2a_gate_result.json`,
`independent_review.json`, `leakage_review.json`, `source_changes.json`,
`final_manifest.json`, `final_summary.md`, plus the 4 archived raw soak
attempt directories (`rtf_soak_attempt1_*` through `rtf_soak_attempt4_*`).
Section 10-13 deliverables that depend on 14/14 accepted groups
(`integrity_audit.json`, `pilot_bias_map.csv`, `historical_bias_comparison.csv`,
etc.) were not produced -- see `final_manifest.json`'s
`deliverables_not_produced_and_why` for the itemized reason each one is
absent.

## Single next step

The RTF-stutter recurrence pattern in this environment matches an
unresolved, previously-documented issue whose root cause was never
isolated. Recommended options for the user to choose from before any
re-attempt: (a) investigate machine-level contention further (the leading
hypothesis in the prior stutter report is OS-level scheduling contention
across the multi-process stack); (b) retry Stage 2A at a time with less
other load on this machine, since duration-correlation is now more
strongly evidenced; or (c) if the user wants to proceed despite this,
explicitly authorize a different RTF-gate policy for this specific pilot
(a decision this task cannot make unilaterally, since the task's own
rules forbid loosening any gate to force a pass).
