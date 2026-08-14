# CORE_RANGE_STABLE_RELATIVE_TRACKING_3_12M

## Outcome

```text
STABLE_RELATIVE_RANGE_STILL_UNSTABLE
```

A fully offline task (no Gazebo/PX4/ROS2/backend process started at any
point) building the `features → Direct XGBoost → calibration →
alpha-beta filter → clamp [3,12]m` pipeline and evaluating it against
tracking-stability metrics (not raw accuracy) rather than the four old
gates. 171 (candidate × calibration × filter-setting) combinations were
evaluated; **0 pass the practical stability gate**, dominated by one
decisive failure: consecutive-frame direction agreement tops out around
0.53-0.57 against a 0.90 target, everywhere in the precommitted grid.

## Phase 3 — Baseline sanity

Reused the already bit-exact-verified `fold_assignments.csv`/corpus load
from the prior offline-analysis task (cross-checked identical again
here) rather than re-deriving from scratch — this task changes the
pipeline and feature set, so a literal "reproduce the old baseline
metrics" step doesn't apply the same way it did in the prior task; what
was verified is that the underlying corpus/fold machinery is unchanged
and correct.

## Phase 4/5 — Feature contract and monotonic audit: two important findings

`feature_contract.json`: 27 runtime-only features (physical range, target
inverse depth, ROI quantiles/std/iqr, calibration a/b/denominator/
residual/condition/inlier-count/span/age, image ray x/y, ray scale,
deterministic quality flags) — deliberately narrower than the prior
tasks' PHYSICAL_ONLY (excludes anchor_*, calibration_source_*,
calibration_inlier_fraction, none of which are named in this task's
section 5 list). No bbox size, no session/scenario/group ID, no future
frame. Temporal smoothing is delegated entirely to the alpha-beta filter
— the model itself is per-frame only, matching the pipeline diagram.

**Finding 1 — no globally-consistent monotonic feature.**
`monotonic_feature_audit_conclusion.json`: per-group correlation of
`raw_physical_range_m`, `target_inverse_depth`, and every `target_roi_q_*`
field against ground-truth distance **flips sign** between "center"
(no lateral offset) dynamic groups and lateral/yaw-varying dynamic groups
— e.g. `cdr_approach_left_5hz_headless` r=-0.94 vs
`cdr_recede_center_5hz_headless` r=+0.38, `cdr_stop_recede_9_5hz_headless`
r=+0.94. The corpus-wide `raw_physical_range_m` vs. GT correlation is
**-0.34 Pearson / -0.31 Spearman** — the opposite sign of the naive
physical prior a monotonic constraint would assume. This is a strong,
concrete explanation for why the prior offline-analysis task's Candidate
2 (a global `+1` monotonic constraint on `raw_physical_range_m`) showed
no real improvement: the constraint was fighting a relationship whose
sign is not even consistent across the corpus. Root cause not
investigated further (would require touching M52/calibration geometry
semantics, out of scope) — flagged as a high-value follow-up.

**Finding 2 — `ray_scale` is the one feature with a usable, evidence-backed
monotonic relationship** specifically in the dynamic/moving-target regime
this task cares about: zero variance within every static group (fixed
camera/gimbal geometry per static capture), but consistently **negative**
correlation across 6 of 7 dynamic groups with any signal at all
(-0.23 to -0.90). Used as Candidate C's monotonic constraint (`-1`,
non-increasing).

## Phase 6/7 — Training and calibration

3 candidates trained (group-disjoint 3-fold, seed 52,
`reg:squarederror` only — `reg:pseudohubererror` excluded per this task's
own instruction after the prior task's collapse finding):

| Candidate | Weighting | Constraint | Raw MAE (none/affine/isotonic) |
|---|---|---|---|
| A — group-balanced | equal weight per group | none | 1.259 / 1.267 / 1.260 |
| B — group + distance-range balanced | equal weight per group AND per 9 distance-bin, 1.5x boost 10-12m | none | **1.178** / 1.180 / 1.176 |
| C — monotonic ray_scale | equal weight per group | `ray_scale: -1` | 1.258 / 1.266 / 1.254 |

No candidate collapsed (verified via `raw_candidate_metrics.csv`, unlike
the prior task's pseudo-Huber failure). Candidate B's distance-range
balancing modestly improved raw MAE too, not just distance-bin evenness.
Calibration (affine/isotonic, fit per-fold on training predictions only,
applied to held-out folds) mainly reduces bias (isotonic bias 0.004-0.043m
vs. affine 0.06-0.10m) without materially changing MAE, as expected —
neither addresses the frame-to-frame noise that turns out to be the real
problem (see Phase 8/9).

## Phase 8/9 — Alpha-beta filter sweep: the decisive finding

Applied the causal alpha-beta filter (per-session, reset at session
boundary, soft innovation limiter) across the full precommitted grid
(alpha∈{0.35,0.50,0.65}, beta∈{0.03,0.08,0.15}, threshold∈{1.5,2.0}m) to
all 9 (candidate × calibration) combinations = 171 total evaluations
(`filter_comparison.csv`, `stability_score.csv`).

**Root cause of the direction-agreement failure**: at this corpus's 5Hz
capture rate and 0.25 m/s dynamic-scenario speed, the true GT
frame-to-frame motion has median magnitude **0.035m** (72% of consecutive
frames move less than the 0.05m motion-noise threshold precommitted for
scoring). The raw model's own frame-to-frame prediction noise has median
magnitude **0.148m** — over 4x the physical signal. No calibration stage
touches this (calibration is a per-frame monotonic remap, not a temporal
smoother). The alpha-beta filter is the only stage that could — swept
across the full grid, the best direction agreement achieved is **0.5714**
(Candidate B, no calibration, alpha=0.35 — the grid's most-smoothing
setting), with every other combination at or below that. Direction
agreement never approaches the 0.90 target anywhere in the precommitted
grid; going below alpha=0.35 was not precommitted and was not added after
seeing this result, per this task's own discipline.

Best-by-composite-stability-score combination overall (not the same
alpha as the best-direction-agreement point, since the score also weighs
stationary jitter and worst-bin MAE): **Candidate B, no calibration,
alpha=0.65, beta=0.03, threshold=2.0m** — direction agreement 0.5343,
Spearman 0.8513, stationary std 0.5044m, 0 catastrophic jumps, global MAE
1.154m, worst-bin (11-12m) MAE 1.876m.

### Practical gate result

| Metric | Target | Best achieved (any combination) | Met |
|---|---|---:|---|
| Direction agreement | ≥0.90 | 0.5714 | **No** |
| Spearman | ≥0.90 | 0.8524 | **No** (close) |
| Stationary std | ≤0.40m | ~0.50m (best combos) | **No** (close) |
| Catastrophic jumps | 0 | 0 | Yes |
| Global MAE | ≤1.5m | 1.154m | Yes |
| Worst-bin MAE | ≤2.0m | 1.876m | Yes |

**0 of 171 combinations pass all six simultaneously** — the gate requires
all at once, and direction agreement alone rules out every combination.

## Phase 10 — Selection

No candidate frozen (`selected_candidate.json`: `frozen: false`). The
stability score (direction 40%, Spearman 20%, stationary 15%, worst-bin
10%, bias-spread 5%, dynamic MAE only 5% and saturating — verified by
reading the scoring code directly, not MAE-dominated per this task's
instruction) still could not manufacture a passing combination, since
direction agreement's ceiling is a hard, grid-wide ~0.57.

## Phase 11/12 — Correctly skipped

This task's own rule: "Chỉ thực hiện [observation-only integration and
the live Gazebo smoke test] nếu có final development candidate đạt
stability gate." 0 of 171 combinations pass, so neither phase was
attempted. No `tracking_web.py`/`main.py` runtime source file was
touched; no Gazebo/PX4/ROS2/backend process was ever started
(`source_changes.json`).

## Independent review

`independent_review.json` — re-verified the monotonic sign-inconsistency
finding directly, confirmed no candidate collapsed, confirmed the
stability score is not MAE-dominated by reading the scoring function,
confirmed 0/171 pass the practical gate honestly (no criteria loosened),
and confirmed Phase 11/12 were correctly skipped per the task's own
conditional. `leakage_review.json` confirms group-disjoint folds,
per-fold-only calibration fitting, no GT-derived/scenario/session/bbox-
size/future-frame feature, and that `distance_bin_9` (used only to build
Candidate B's training-time sample weight) never reaches the model as a
feature.

## What was not done (by design, per this task's scope)

- No Gazebo/PX4/ROS2/backend process started.
- No recollection; frozen corpus untouched (read-only).
- No `reg:pseudohubererror` (explicitly excluded per this task).
- No near/core/far submodel split (avoided per instruction, to prevent
  boundary discontinuity).
- No hyperparameter/grid value added after seeing any result.
- No M52/MiDaS/calibration-source/PX4/controller/Follow Target change.
- No runtime integration (Phase 11 gated, not reached).

## Tests

- `test_core_range_5hz_headless_retrain.py`, `test_range_v2_baseline_eval.py`,
  `test_range_residual_dataset.py`, `test_range_physical_diagnostics.py`,
  `test_gazebo_ground_truth_history.py`, `test_sim_time_trajectory_and_gt.py`:
  77 passed.
- Full repository suite (`pytest -q --ignore=swarm_dashboard_handoff_20260729`):
  **432 passed**, unchanged (no runtime/estimator source code modified —
  new offline analysis/training scripts only).
- `./run_all.sh --check`: PASS.
- No process left running (none was ever started — task is fully offline).

## Deliverables

`artifacts/core_range_3_12m/stable_relative_tracking/`: `feature_contract.json`,
`monotonic_feature_audit_conclusion.json`, `monotonic_sign_consistency.json`,
`monotonic_feature_audit.csv`, `training_precommit.json`, `raw_candidate_metrics.csv`,
`calibration_comparison.csv`, `filter_comparison.csv`, `direction_metrics.csv`,
`rank_correlation.csv`, `stationary_metrics.csv`, `frame_jump_metrics.csv`,
`per_bin_metrics.csv`, `per_group_metrics.csv`, `dynamic_metrics.csv`,
`prediction_rows.csv`, `stability_score.csv`, `selected_candidate.json`,
`leakage_review.json`, `independent_review.json`, `source_changes.json`,
`final_manifest.json`, `final_summary.md`, `models/` (18 fold-model files
across the 3 candidates), `calibration/` (affine + isotonic parameters per
candidate).

## Single next step

The fundamental limiter is frame-to-frame model noise (~0.15m median)
being several times larger than the true physical motion signal
(~0.035m median) at this corpus's 5Hz/0.25 m/s regime — no calibration or
in-grid causal filter setting can recover 90% direction agreement from
that SNR. Two independent directions are worth pursuing, either of which
would need its own fresh precommit round: (1) reduce raw model
frame-to-frame noise directly (larger regularization, fewer/more stable
features, or addressing the depth/calibration static↔dynamic distribution
shift found in the prior offline-analysis task) before relying on
filtering to fix it downstream; (2) if frame-to-frame direction agreement
at native capture rate is inherently unachievable at this target speed,
consider whether the task's direction-agreement metric should be measured
over a longer causal window (e.g. compare against N frames ago, not 1)
instead of consecutive frames — but that is a metric-definition change
outside this task's scope to decide unilaterally, and was not done here.
