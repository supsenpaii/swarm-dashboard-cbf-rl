# CORE_RANGE_OVERNIGHT_SIM_TIME_TRAIN

## Outcome

```text
NO_SIM_TIME_DYNAMIC_ROBUST_MODEL_MEETS_GATE
```

Corpus reached **8/8** dynamic groups (approaching 3/3, receding 3/3,
stop_and_hold 2/2) far ahead of the 07:00 deadline — collection completed
in ~13 minutes across only 4 total attempts. Six PHYSICAL_ONLY/
PHYSICAL_TEMPORAL × 3-hyperparameter-config models were trained and
evaluated on the full 35-group (27 static + 8 dynamic) frozen corpus; none
passed all four gate families (static, dynamic, temporal, bbox
robustness), so no candidate was frozen. This is a genuine accuracy result,
not a process failure — every phase (collection, integrity, training,
independent review) completed cleanly.

## Timeline

| Time (ICT) | Event |
|---|---|
| 00:19 | Session start, preflight |
| 00:22 | 5 existing accepted groups re-verified VALID |
| 00:24 | Collection driver launched |
| 00:26 | `cdr_stop_approach_6_5hz_headless` ACCEPTED, attempt 1/12 |
| 00:27 | `cdr_recede_left_yaw_5hz_headless` attempt 1/12 QUARANTINED (RTF long-stutter) |
| 00:30 | `cdr_recede_right_5hz_headless` ACCEPTED, attempt 1/12 |
| 00:32 | `cdr_recede_left_yaw_5hz_headless` ACCEPTED, attempt 2/12 — **corpus 8/8** |
| 00:33 | Dataset frozen (35 groups, 2081 frames) |
| 00:33 | Training complete (6 configs, 11s CPU) |
| 00:35 | Independent review complete |
| 00:37 | Tests + `run_all.sh --check` pass, cleanup verified |

Deadline was 07:00; the entire task completed in **under 20 minutes**,
leaving no need for the checkpoint/resume or 06:15/06:40/06:50 cutoff
rules to engage.

## Why collection was fast this time

The historical corpus (5/8, prior sessions) had exhausted 8/8 attempts on
*all three* missing groups, always for RTF/long-stutter reasons — see
`docs/CORE_RANGE_GAZEBO_RTF_STUTTER_REPORT.md` and the
`DATA_COLLECTION_BLOCKED_BY_PERSISTENT_RTF_STUTTER` conclusion. This
session's 4 attempts all measured RTF median ≈0.998–1.000 (only 1 of 4 hit
a long-stutter window). This looks like the same machine under
substantially lighter transient load right now, not a fix to the
underlying RTF stutter (which was never root-caused, per
`CORE_RANGE_GAZEBO_RTF_STUTTER_REPORT.md`) or a change in this session's
own code. **RTF was not downgraded or bypassed** — every accepted attempt
independently cleared the existing frozen RTF hard-gate contract
(median ≥0.8, no long-stutter window) via `core_range_collect_dynamic_5hz_headless_batch.audit()`,
reused completely unmodified.

## Phase 4/5 — Preflight and existing-group revalidation

- Focused tests (plugin, GT-bracket, bbox/session): 53 passed.
- `./run_all.sh --check`: PASS.
- All 5 existing accepted groups re-hash-verified against
  `accepted_sessions.json`'s recorded checksums: **all 5 VALID**, no
  changes needed (`artifacts/core_range_3_12m/overnight_sim_time_train/existing_groups_revalidation.json`).
  (First pass of this check had a false positive — flagging 2 groups as
  quarantined because an *earlier failed attempt* of the same group_id
  was in the quarantine list; fixed by comparing full source paths, not
  just session_id, before trusting the result.)

## Phase 6-11 — Collection

Driver: `core_range_overnight_sim_time_collect.py` (new, this session) —
round-robins one attempt per remaining missing group per cycle (per this
task's spec, to spread RTF exposure over time rather than exhausting one
group back-to-back), reusing `core_range_collect_dynamic_5hz_headless_batch.audit()`
and `core_range_logging_eval.py` **completely unmodified** for every
runtime/integrity/GT gate. `sim_time_plugin` driver, `SWARM_START_GAZEBO_GUI=false`,
5Hz depth, production bbox/geometry from `collection_plan.json`
(`central_quantile_region` ROI recovery used for `cdr_recede_left_yaw`
exactly as precommitted, unchanged).

| Group | Attempt | Result | Reason | RTF median | Frames |
|---|---|---|---|---:|---:|
| cdr_stop_approach_6_5hz_headless | 1 | ACCEPTED | — | 0.9989 | 174 |
| cdr_recede_left_yaw_5hz_headless | 1 | QUARANTINED | RTF_LONG_STUTTER | — | — |
| cdr_recede_right_5hz_headless | 1 | ACCEPTED | — | 0.9981 | 257 |
| cdr_recede_left_yaw_5hz_headless | 2 | ACCEPTED | — | 0.9984 | 248 |

Full ledger: `artifacts/core_range_3_12m/overnight_sim_time_train/collection/attempt_ledger.csv`.
The 1 quarantined attempt was preserved unmodified under `collection/quarantine/`.

## Phase 12 — Freeze

`artifacts/core_range_3_12m/overnight_sim_time_train/frozen_corpus/`:
8 accepted dynamic groups (approaching 3, receding 3, stop_and_hold 2),
combined with the unmodified 27-group/832-frame static corpus →
**35 groups, 2081 frames**. Integrity, 96 anchors, 0 GT_BRACKET_TIMEOUT, 0
malformed/duplicate-trace/checksum failures across all 8 new+existing
accepted groups — `integrity_report.json`. No quarantine, no 2Hz corpus,
no old unaccepted 5Hz attempts used.

## Phase 13-15 — Training

Reused `core_range_5hz_headless_retrain.py` (built and unit-tested in a
prior session) unmodified — `prepare()` then `run()`, seed 52,
group-disjoint 3-fold cross-validation (12/12/11 groups), 3 precommitted
shallow XGBoost configs per variant, no config added after seeing
out-of-fold results. Feature contract grep-verified to exclude bbox
width/height/area/aspect and all other forbidden features for both
variants (40 / 52 features respectively).

## Phase 16 — Gate results (all 6 configs)

| Variant | Config | Static MAE | Dynamic MAE | Lag (s) | BBox catastrophic @±10% | All gates |
|---|---|---:|---:|---:|---:|---|
| PHYSICAL_ONLY | A_conservative | 1.078 | 1.420 | 2.00 | 5.62% | FAIL |
| PHYSICAL_ONLY | B_balanced | 1.036 | 1.270 | 2.00 | 4.13% | FAIL |
| PHYSICAL_ONLY | C_shallow | 1.227 | 1.516 | 2.00 | 8.51% | FAIL |
| PHYSICAL_TEMPORAL | A_conservative | 1.086 | 1.403 | 2.00 | 4.85% | FAIL |
| PHYSICAL_TEMPORAL | B_balanced | 1.079 | 1.251 | 2.00 | 4.18% | FAIL |
| PHYSICAL_TEMPORAL | C_shallow | 1.237 | 1.512 | 2.00 | 8.36% | FAIL |

Gates: static MAE ≤1.0m, dynamic MAE ≤1.0m, lag ≤0.30s, bbox catastrophic
@±10% = 0%. **Every config fails static, dynamic, temporal, and bbox
gates simultaneously** — this is not a narrow miss on one gate.
`per_group_metrics.csv` shows the dynamic weakness concentrated in
`cdr_recede_center_5hz_headless` (MAE 2.31m) and the yaw-varying groups
(`cdr_approach_right_yaw` 1.81m, `cdr_recede_left_yaw` 1.46m) — lateral/yaw
geometry is where these models are weakest.

## Phase 17 — Independent review findings

Full detail: `independent_review.json`, `leakage_review.json`.

- Corpus provenance, group-disjoint folds, no static/dynamic overlap, no
  quarantine leakage: all verified clean.
- Feature contract: verified by direct inspection (not just asserted) —
  zero forbidden-feature hits in either variant's feature list.
- `frozen_model_checksum: null` correctly matches `selected_variant: null`
  (freeze step never triggers when no config passes all gates); `models/`
  directory confirmed empty.
- **One measurement-fidelity anomaly found, does not change the
  conclusion**: `median_absolute_lag_s` is exactly `2.0` (the search
  boundary of `alignment_lag_s(..., maximum_s=2.0)` in
  `core_range_direct_dynamic_replay.py`, unmodified pre-existing shared
  evaluation code) for all 6 configs — a search-saturation artifact, not
  a converged interior lag measurement. Not corrected in this session:
  the temporal gate already fails by a wide margin either way, and the
  dynamic and bbox gates already fail independently of this number, so
  widening the search window cannot change the conclusion. Touching
  shared, unmodified evaluation code for a result that wouldn't move the
  outcome was judged not worth the added blast radius under this task's
  conservative-by-default instructions. Flagged for future work.
- Bbox-stress degradation/shift being exactly `0.0` for every
  perturbation is expected, not a bug: PHYSICAL_ONLY/PHYSICAL_TEMPORAL
  never see bbox dimensions as a feature, so perturbing bbox cannot move
  their predictions — the reported catastrophic fraction under bbox
  stress is the model's baseline (unperturbed) error rate, which is what
  fails the gate. An accuracy problem, correctly attributed, not a
  bbox-robustness problem.

## Scope guards honored

- RTF hard gate never downgraded, bypassed, or narrowed — every accepted
  attempt cleared the existing frozen contract unmodified.
- `set_bbox()`, session-ID mechanism, sidecar writer, GT-bracket fix,
  sim-time plugin, and pose-history contention fix: all untouched.
- No PX4, controller, EKF, M52, calibration, or MiDaS-weight change.
- No Follow Target, no hardware, no `sudo`, no destructive git/file
  operations.
- No config added to the training precommit after seeing results; no gate
  lowered; no quarantine used for training.

## Tests

- Focused (plugin/GT-bracket/bbox-session/training-driver/static-eval/
  residual-dataset): 77 passed.
- Full repository suite (`pytest -q --ignore=swarm_dashboard_handoff_20260729`,
  the ignored path is an unrelated stale backup copy nested in the repo):
  **432 passed**, unchanged from before this session (no estimator/model/
  runtime source code was modified — only new orchestration scripts and
  generated artifacts).
- `./run_all.sh --check`: PASS.
- No Gazebo/PX4/ROS2/backend/capture/training process left running; GPU
  idle at review time (23% background, 76MiB — unrelated pre-existing
  load, unchanged from session start).

## Deliverables

`artifacts/core_range_3_12m/overnight_sim_time_train/`: `overnight_state.json`,
`attempt_ledger.csv` (`collection/`), `existing_groups_revalidation.json`,
`collection_plan_snapshot.json`, `runtime_metrics.csv`, `gazebo_rtf.csv`,
`camera_fps.csv`, `tracking_fps.csv`, `latency_metrics.csv`,
`gt_alignment.csv`, `accepted_sessions.json`, `quarantined_sessions.json`,
`integrity_report.json`, `frozen_dataset_manifest.json`,
`training_precommit.json`, `feature_contract.json`,
`fold_assignments.csv`, `static_metrics.csv`, `dynamic_metrics.csv`,
`temporal_metrics.csv`, `bbox_stress_metrics.csv`, `per_group_metrics.csv`,
`per_bin_metrics.csv`, `prediction_rows.csv`, `model_comparison.csv`,
`leakage_review.json`, `independent_review.json`, `retrain_manifest.json`,
`final_summary.md`, empty `models/`, `logs/collection_driver.log`.

## Next steps

1. The dynamic corpus is now genuinely complete (8/8) and frozen — future
   sessions can iterate on model quality (features, hyperparameters, more
   configs precommitted up front) without needing to touch collection
   again, as long as source hashes stay unchanged.
2. Lateral/yaw-varying dynamic geometry is the clearest weak spot (see
   per-group MAE) — worth investigating as a feature-engineering
   direction before the next training attempt.
3. If a future variant gets close to the other three gates, fix the
   `alignment_lag_s()` search-boundary saturation noted above before
   trusting its temporal-gate result.
4. RTF stutter itself is still not root-caused (unrelated to this
   session's collection success, which is best read as favorable
   transient machine load, not a fix) — `RTF_REMAINS_HARD_GATE_STUTTER_UNRESOLVED`
   stands.
