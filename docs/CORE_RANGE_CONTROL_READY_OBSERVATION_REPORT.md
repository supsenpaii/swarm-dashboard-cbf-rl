# CORE_RANGE_CONTROL_READY_OBSERVATION_PIPELINE

## Outcome

```text
OFFLINE_CONTROL_READY_RANGE_FAILED
```

A fully offline task (no Gazebo/PX4/ROS2/backend process started at any
point) building the `Candidate B raw prediction -> causal alpha-beta state
estimator -> measurement-age compensation -> predicted_current_range_m`
pipeline plus an independent, monitor-only causal windowed trend estimator
(0.6/0.8/1.0s, hysteresis), and evaluating both against control-usability
metrics (not the old per-frame accuracy gates). The offline practical gate
(Section 10) **failed**: windowed direction agreement never exceeded 0.516
(target ≥0.80) at any of the 3 precommitted windows, and stationary jitter
during stop-and-hold (0.588m) narrowly missed its 0.55m target. Global MAE,
worst-bin MAE, catastrophic-jump count, P95 measurement age, and
age-compensation degradation all passed. Per the task's own conditional
rule, Phase 3 (observation-only runtime integration) and Phase 4 (live
Gazebo smoke test) were correctly **not attempted**.

## Phase 0 — Candidate B artifact verification

Located the 3 fold boosters + 3 fold preprocessors for
`CANDIDATE_B_GROUP_AND_DISTANCE_BALANCED` in
`artifacts/core_range_3_12m/stable_relative_tracking/models/` (enumerated,
not guessed), confirmed the feature order matches both
`feature_contract.json` and `training_precommit.json` (27 features, no
bbox/session/scenario/group-ID/GT-derived field), reloaded the saved
boosters (no retraining) and recomputed group-disjoint out-of-fold
predictions. Result: **bit-exact** match against the previously-published
`prediction_rows.csv` (max abs diff = 0.0 across all 1897 combined
static+dynamic rows) and against `raw_candidate_metrics.csv`'s published
MAE/bias/std (`candidate_b_reproduction.json`). No
`CANDIDATE_B_REPRODUCTION_FAILED` triggered.

The frozen corpus records only a single `measurement_timestamp_s` per frame
(sim time at capture) with no separate capture/consume timestamp, so a live
"measurement age" is not directly present in the dataset — this is disclosed
and handled explicitly in Phase 2's methodology (see below).

## Phase 1 — `control_ready_range_estimator.py`

Two independent classes, both pure Python, no PX4/controller import,
27 unit tests (`test_control_ready_range_estimator.py`, all pass):

- **`ControlReadyRangeEstimator`**: causal alpha-beta filter (recurrence
  operates in measurement-time using the actual inter-measurement dt, not
  wall/query time), soft innovation limiter (clip, not hard-reject — verified
  the estimator still converges within ~1 update-cycle when a target
  genuinely changes distance), age-compensation projection to "now"
  (`predicted_current_range_m = filtered_range_m + rate * measurement_age_s`,
  no hard clamp — a separate `display_range_m` copy is clamped to [3,12]m for
  UI use only), extrapolation cap at 0.5s (beyond which `estimator_status =
  STALE_MEASUREMENT` and `estimator_valid = False`, so the pipeline does not
  keep predicting indefinitely off a stale measurement), and automatic reset
  on session/target change, model-signature change, timestamp rollback, or a
  measurement-gap timeout (1.0s default — an engineering choice, not a
  statistical hyperparameter, so not subject to the "no further grid search"
  rule that applies only to alpha/beta/threshold).
- **`RangeTrendEstimator`**: causal, windowed (0.6/0.8/1.0s), Theil-Sen
  robust slope, entry/exit hysteresis (0.08/0.04 m/s) with a 2-consecutive-
  decision confirmation rule, resets to `UNKNOWN` immediately on invalid
  input (no stale state held), and self-heals to `UNKNOWN` after a large
  temporal gap because the window filter naturally excludes samples older
  than the window (verified directly: a real 54-second gap found in one
  corpus group, `cdr_recede_center`, was confirmed to correctly drop the
  trend state to `UNKNOWN` rather than computing a slope across the gap).
  Monitor-only by construction — never fed back into the estimator above,
  never exposed as a control signal.

## Phase 2 — Offline replay (8/8 dynamic groups, Candidate B raw OOF)

Replayed all 3 approaching + 3 receding + 2 stop-and-hold groups (1249
frames) through the estimator + trend pipeline, fresh estimator/trend
instances per group (session boundary = group boundary, consistent with
`group_id == session_id` in the frozen corpus). No model retrain, no
dataset modification, no frame deletion, no re-tuning of alpha/beta after
seeing results.

**Measurement-age methodology (precommitted before running, see
`clock_domain_contract.json`)**: since the corpus has no live capture/
consume timestamp, the primary replay pass uses a constant, small,
realistic synthetic age (0.05s, one inference cycle) for all gate-relevant
metrics; a secondary, deterministic 5-bucket sweep (0.05/0.15/0.25/0.4/0.6s,
cycled by frame index within each group) is applied post-hoc to the
already-computed filtered state for Section 9.2 only — this cannot affect
the primary pass since the alpha-beta recursion depends only on
inter-measurement dt, not on the assigned query age.

### 9.1 Range quality

| Metric | Value |
|---|---:|
| Global MAE | 1.158 m |
| Global bias | 0.213 m |
| P90 / P95 abs error | 2.396 m / 2.835 m |
| Worst bin | 3-4m, MAE 1.885 m |
| Stationary std (stop-and-hold hold phase) | 0.588 m |
| Catastrophic jump count | 0 |

Worst-bin shifted from 11-12m (prior task) to **3-4m** in this replay —
both ends of the validated range show large raw-model error, in different
scenarios (`per_bin_metrics.csv`).

### 9.2 Age-compensation quality

`age_compensation_metrics.csv`: compensation is **neutral to mildly
negative** at this corpus's SNR — global MAE goes from 1.1547m
(uncompensated, 0-0.1s bucket) to 1.1995m (compensated, >0.5s bucket), i.e.
extrapolating with a noisy rate estimate adds variance slightly faster than
it removes bias from staleness. Worst-bucket degradation was 0.025m, inside
the ≤0.10m gate, but the direction of the effect across every bucket and
every scenario (approaching/receding/stop-and-hold) is consistently
non-improving. This is disclosed explicitly rather than framed as a win.

### 9.3 Windowed direction agreement — the decisive failure

| Window | Overall | Approaching | Receding | Unknown fraction | Decision latency |
|---|---:|---:|---:|---:|---:|
| 0.6s | 0.445 | 0.399 | 0.514 | 0.145 | 3.05s |
| 0.8s | 0.501 | 0.462 | 0.535 | 0.067 | 3.58s |
| 1.0s | **0.516** | 0.498 | 0.533 | 0.038 | 3.62s |

Selected window (per the precommitted priority order — best agreement,
then latency, then unknown fraction, then false transitions): **1.0s**.
Even at the best window, overall agreement is barely above chance (0.50).
Direct inspection of a monotonically-approaching group
(`cdr_approach_center_5hz_headless`, GT smoothly 11.47m→4.14m throughout)
found the cause is not simple noise that averages out: Candidate B's raw
prediction sustains a ~1.5-3.5m bias for dozens of consecutive frames in
the 4.5-8m true-range portion of this specific trajectory. A windowed
slope estimator cannot distinguish "the target is moving" from "the raw
model's regional bias is drifting" when both act on the same few-meter
scale within a 0.6-1.0s window — this generalizes and extends the prior
task's per-frame SNR finding rather than contradicting it.

### 9.4 Stop-and-hold

`stop_hold_metrics.csv`: `cdr_stop_approach_6` shows 0.74m hold jitter and
10-11 false approaching/receding switches during the hold phase (the trend
classifier oscillates because the filtered signal itself has not settled);
`cdr_stop_recede_9` is much better (0.43m jitter, 5 false switches).
Combined stationary std (0.588m) is what fails the ≤0.55m gate — driven
mainly by the first group.

### 9.5 Control usability

`control_usability_metrics.csv`: update rate ≈4.6Hz median (matches the
corpus's known ~5Hz capture rate with some session-boundary gaps), 0% stale
output, 0% invalid output, 6 total resets across 8 groups — all
`measurement_gap_timeout` (one group, `cdr_recede_center`, has a genuine
54s internal capture gap; the estimator correctly reset rather than
extrapolating across it).

## Phase — offline gate result

| Check | Value | Target | Passed |
|---|---:|---|:---:|
| Global MAE | 1.158m | ≤1.5m | Yes |
| Worst-bin MAE | 1.885m | ≤2.0m | Yes |
| Catastrophic jumps | 0 | =0 | Yes |
| Stationary std | 0.588m | ≤0.55m | **No** |
| Overall windowed direction agreement | 0.516 | ≥0.80 | **No** |
| Approaching agreement | 0.498 | ≥0.75 | **No** |
| Receding agreement | 0.533 | ≥0.75 | **No** |
| P95 measurement age (synthetic) | 0.05s | ≤0.35s | Yes |
| Age-compensation worst-bucket degradation | 0.025m | ≤0.10m | Yes |

`offline_gate_result.json`: `all_pass: false`, conclusion
`OFFLINE_CONTROL_READY_RANGE_FAILED`. No threshold was adjusted after
seeing these numbers (`offline_precommit.json` was written and its
thresholds fixed before either replay or metrics script ran — verified by
file mtimes and by the metrics script reading thresholds from that file
rather than embedding a second copy).

## Phase 3/4 — correctly skipped

Section 11's own rule: "Chỉ thực hiện khi offline gate PASS." The gate did
not pass, so no runtime integration was attempted and no Gazebo/PX4/ROS2/
backend process was started at any point in this task
(`source_changes.json`). No `main.py`/`tracking_web.py`/controller/PX4 file
was touched.

## Independent review

`independent_review.json` — re-verified precommit-before-results ordering,
re-verified alpha/beta/threshold were not re-tuned, manually spot-checked
the windowed-direction failure against raw data to confirm it reflects a
genuine raw-model near-field/regional-bias problem rather than a scoring or
filter-divergence bug (the rate estimate stays bounded, -0.5 to +0.2 m/s,
never diverges), re-verified the catastrophic-jump count, and confirmed
Phase 3/4 were correctly skipped. `leakage_review.json` confirms the
windowed-direction reference lookup and the trend estimator are both
strictly causal, the age-bucket sweep cannot affect the primary pass, and
no GT-derived value ever reaches the estimator.

## What was not done (by design, per this task's scope)

- No Gazebo/PX4/ROS2/backend process started.
- No model retraining, no calibration refit, no dataset/frame change.
- No alpha/beta/threshold re-tuning after seeing any result.
- No runtime integration (`SWARM_CONTROL_READY_RANGE_ENABLED`), no live
  smoke test — both correctly gated on the offline PASS that did not occur.
- No shadow controller, no PX4 FollowTarget, no hardware.

## Tests

- `test_control_ready_range_estimator.py`: 27 passed (alpha-beta numerics,
  irregular dt, soft innovation limiter + convergence, age compensation
  formula, extrapolation cap / stale marking, session/timestamp-rollback/
  model-signature reset with no state leak, duplicate-timestamp guard,
  windowed trend with irregular sampling, hysteresis, unknown state,
  stop-and-hold-style hold detection, large-gap self-healing, no-future-data).
- Full repository suite: unchanged pass count from the prior task (no
  runtime/estimator source outside the new files was modified).
- `./run_all.sh --check`: PASS.
- No process left running (none was ever started — task is fully offline).

## Deliverables

`artifacts/core_range_3_12m/control_ready_observation/`:
`candidate_b_reproduction.json`, `artifact_checksums.json`,
`estimator_contract.json`, `clock_domain_contract.json`,
`offline_precommit.json`, `offline_prediction_rows.csv`,
`age_compensation_metrics_raw.csv`, `age_compensation_metrics.csv`,
`windowed_direction_metrics.csv`, `stop_hold_metrics.csv`,
`per_bin_metrics.csv`, `per_group_metrics.csv`, `stationary_metrics.csv`,
`frame_jump_metrics.csv`, `control_usability_metrics.csv`,
`trend_window_comparison.csv`, `selected_trend_window.json`,
`offline_gate_result.json`, `leakage_review.json`, `independent_review.json`,
`source_changes.json`, `final_manifest.json`, `final_summary.md`, `plots/`
(empty — no plot was necessary to reach or explain the conclusion).
`runtime_integration_manifest.json`/`runtime_smoke_*.csv`/
`runtime_gate_result.json` were **not created** — Phase 3/4 correctly
skipped.

## Single next step

Two independent directions, either requiring its own fresh precommit round
before touching code:

1. **Fix the raw model's regional/near-field bias directly**, rather than
   asking a downstream filter to smooth it away — the persistent multi-frame
   bias found in the 4.5-8m portion of `cdr_approach_center` and in the
   3-4m/11-12m bins generally is exactly the kind of error a fixed-gain
   alpha-beta filter cannot distinguish from genuine motion. This likely
   requires revisiting Candidate B's feature set or the still-unexplained
   sign-inconsistent `raw_physical_range_m`/depth-feature finding from the
   stable_relative_tracking task, not the filter/trend layer built here.
2. **Re-examine whether a fixed±0.6-1.0s window is the right granularity**
   at this corpus's ~0.25 m/s dynamic speed — the true motion signal within
   any sub-1-second window is still only 0.15-0.25m, comparable to
   Candidate B's own bias excursions. A longer window would trade latency
   for SNR; whether that tradeoff is acceptable for Follow Target is a
   product decision, not something to decide unilaterally inside this task.
