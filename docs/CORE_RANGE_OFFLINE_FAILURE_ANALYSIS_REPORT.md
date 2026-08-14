# CORE_RANGE_OFFLINE_FAILURE_ANALYSIS_AND_TARGETED_RETRAIN

## Outcome

```text
TARGETED_MODEL_IMPROVES_BUT_STILL_FAILS
```

Applied generously: the only nonzero delta found across 3 targeted
candidates is a 0.013m static-MAE improvement (noise-level), and no
candidate clears any of the four gate families. This was a fully offline
task — no Gazebo/PX4/ROS2/backend process was started at any point; the
already-frozen 8/8 dynamic + 27-group static corpus was read-only
throughout.

## Phase 1 — Baseline reproduction: bit-exact

Re-ran `core_range_5hz_headless_retrain.py prepare`+`run` unmodified
against the same `frozen_corpus`, seed 52, unchanged hyperparameters/
feature contract/fold logic. Every artifact checksum matched exactly
(not just "within tolerance"): `model_comparison.csv` identical row for
row, `prediction_rows.csv` SHA-256 identical
(`a43e578c...da6f6e07`), `frozen_dataset_manifest.json`,
`fold_assignments.csv`, `static/dynamic/temporal/bbox_stress_metrics.csv`
all byte-identical. See `baseline_reproduction.json`. No
`BASELINE_REPRODUCTION_FAILED` risk — proceeded to Phase 2 on a verified
foundation.

## Phase 2 — Failure matrix (752 dynamic + 832 static + 496 raw rows→2081 total)

Built `failure_matrix.csv` joining every out-of-fold prediction (both
variants × 3 configs = 6 columns) with the full runtime-quality feature
vector per frame, then `per_group_failure.csv`, `per_bin_failure.csv`,
`runtime_quality_correlation.csv`, and a dynamic trajectory-phase
breakdown (start/mid/end thirds; moving vs. stationary-after-stop, using
each session's own `movement_duration_s`/`hold_duration_s` from
`collection_plan.json`).

Headline numbers (PHYSICAL_TEMPORAL/B_balanced, the baseline's overall
best config):

- Static group MAE ranges **0.02m to 3.60m** across 27 groups (stdev
  0.85m) — a few groups, not a uniform shortfall.
- Worst static distance bin: **11-12m at 2.04m MAE**, 4x the best bins.
- Dynamic: moving-phase MAE **1.166m** vs. stationary-after-stop MAE
  **0.605m** — error nearly doubles while the target is actually moving.
- approaching (1.410m) vs receding (1.310m): only 0.100m asymmetry — not
  a dominant factor.
- `static_to_dynamic_mae_carryover_m` = 0.172m — part but not all of the
  dynamic gap is inherited static bias.

## Phase 3 — Root-cause diagnosis per gate family

### 3.1 Static

Concentrated, not diffuse: worst group `core115_center_pitch10` at
3.60m MAE is almost entirely a far-distance-regime (11-12m) case;
`feature_ablation.csv` shows the complete PHYSICAL_ONLY feature set
already reaches ~1.04-1.09m (near the 1.0m gate) — the shortfall looks
like a specific-regime problem, not a globally-underpowered model.

### 3.2 Dynamic

`feature_distribution_shift.csv`: several depth/calibration/ROI features
show real, systematic standardized-mean-difference between static and
dynamic domains — `image_ray_y` 0.62, `calibration_inlier_fraction` 0.53,
`calibration_inlier_count` 0.51, `anchor_q_median` 0.50,
`target_roi_q_p10` 0.46, `target_roi_q_min` 0.45,
`calibration_denominator` -0.45, `calibration_depth_span` 0.39,
`calibration_b` -0.37, `target_roi_q_p25` 0.34. Moderate but consistent
across the whole depth/calibration feature family — plausible explanation
for both the carryover and the extra motion-phase-specific degradation.

### 3.3 Temporal — the reported lag number is not trustworthy

`temporal_failure.csv`: `alignment_lag_s` saturates at its **2.0s search
boundary for the raw physical-range signal itself** (`fraction_at_boundary_2s
= 1.0`, all 8/8 dynamic sessions) — a signal with *zero* learned model
behind it. The trained models saturate slightly less often (7/8 = 0.875
for both variants). A metric that a raw, unprocessed physical measurement
also maxes out is measuring a property of the search window/method (or
this corpus's sampling rate/noise), not genuine model-introduced lag. The
baseline's `median_absolute_lag_s = 2.0s` for all 6 configs is very
likely not a faithful "the model lags by 2 seconds" statement.

### 3.4 BBox robustness — the gate cannot currently discriminate a real fix

Traced `perturb_bbox_features()` (`core_range_direct_dynamic_replay.py`)
directly: it only ever mutates `bbox_center_x/y_fraction`,
`bbox_width/height_fraction`, `bbox_area_fraction`, `bbox_aspect_ratio` —
confirmed empirically in `bbox_feature_sensitivity.csv` by diffing every
feature before/after perturbation across 200 sample rows × 14 perturbation
variants. It never recomputes `image_ray_x/y`, `ray_scale`,
`target_roi_q_*`, or any `anchor_*` statistic. PHYSICAL_ONLY/
PHYSICAL_TEMPORAL exclude every `bbox_*` field from their feature list
(`feature_contract.json`), so `bbox_prediction_sensitivity.csv` confirms
**median and max prediction shift are exactly 0.0** for every one of the
14 perturbation variants, both configs tested. The bbox_stress
catastrophic-error fraction reported for these two variants **is their
baseline (unperturbed) error rate under a different name**, not a
bbox-sensitivity measurement.

## Phase 4 — Feature information audit

- **Nearest-neighbor ambiguity**: minimum group-disjoint standardized
  feature distance across the whole 2081-row corpus is **1.10** — no
  near-duplicate feature vectors at all. Even the closest decile
  (z-distance ≤1.89) has a median GT gap of only 0.85m, not multi-meter
  ambiguity. `nearest_neighbor_ambiguity.csv`.
- **Distribution shift**: see 3.2 above, `feature_distribution_shift.csv`.
- **Feature ablation** (`feature_ablation.csv`, single B_balanced-style
  config per group for comparability): `physical_range_only` alone MAE
  2.57m; `depth_calibration_only` 1.75m; `geometry_only` 2.37m;
  `quality_only` 2.18m — no small subgroup gets close to the gate. The
  **complete** PHYSICAL_ONLY (40 features) and PHYSICAL_TEMPORAL (52
  features) sets reach 1.09m / 1.12m — dramatically better than any
  subset, i.e. the model is using the available information effectively
  and is not starved for it. This is evidence *against*
  `FEATURE_INFORMATION_INSUFFICIENT` as the dominant cause.

## Phase 5 — Root cause precommit (before any candidate training)

`root_cause_precommit.json` — conclusion `MULTIPLE_FAILURE_MODES`:

1. **`DISTANCE_REGIME_BIAS_DOMINANT`** (static) — 11-12m bin and a few
   outlier groups drag the equal-group aggregate.
2. **`STATIC_DYNAMIC_DISTRIBUTION_SHIFT_DOMINANT`** (dynamic) — moderate,
   systematic depth/calibration/ROI feature shift between domains.
3. **Evaluation-methodology artifacts** (temporal + bbox gates) — both
   gates fail for reasons unrelated to genuine model quality; see 3.3/3.4.

Deprioritized with evidence: `FEATURE_INFORMATION_INSUFFICIENT` (Phase 4),
literal "missing group weighting" (`unified_group_weights()` already
equalizes per-group weight in the baseline — verified by reading the
code, not assumed), `MEASUREMENT_AGE_COMPENSATION_MISSING` (temporal
features already present, didn't help in the baseline).

## Phase 6 — Candidate precommit (before training)

`candidate_precommit.json`, 3 candidates, each with a stated hypothesis,
exact feature order, exact hyperparameters, and precommitted rejection
criteria:

- **Candidate 1** (template A, robust group-balanced): `reg:pseudohubererror`
  objective, PHYSICAL_TEMPORAL features, otherwise baseline-identical
  hyperparameters/weighting.
- **Candidate 2** (template B, regime-aware/monotonic): PHYSICAL_ONLY
  features, `+1` monotonic constraint on `raw_physical_range_m` only (a
  runtime-causal physical measurement, never GT, never `distance_bin`),
  `reg:squarederror` objective (unchanged).
- **Candidate 3** (A+B combined): `reg:pseudohubererror` + the same
  monotonic constraint, PHYSICAL_TEMPORAL features.

Templates C (low-lag causal correction) and D (bbox-invariant
representation) were **not pursued** — not evidence-supported per Phase
5 (the "lag" and "bbox sensitivity" gate failures are methodology
artifacts, not genuine model deficiencies a new feature representation
could fix).

## Phase 7/8 — Results

| Candidate | Static MAE | Dynamic MAE | Temporal | BBox | All gates |
|---|---:|---:|---|---|---|
| Candidate 1 | 4.526m | 4.339m | FAIL | FAIL | FAIL |
| Candidate 2 | 1.023m | 1.273m | FAIL | FAIL | FAIL |
| Candidate 3 | 4.526m | 4.339m | FAIL | FAIL | FAIL |

**Candidates 1 and 3 collapsed**: verified directly from
`prediction_rows.csv` that both predict a **constant 12.0m** (the
`CLIP_BOUNDS` upper edge) for all 2081 rows, regardless of input — a
genuine training failure under `reg:pseudohubererror` with these shallow/
low-learning-rate hyperparameters and default `huber_slope`, not a
harness bug (the evaluation code — `evaluate_config`, `oof_bbox_stress`,
all four gate functions — is the exact unmodified baseline code, and
Candidate 2 trained normally through the identical pipeline). This is
**inconclusive on the robust-loss hypothesis**, not a disproof of it — a
future precommitted round would need a tuned `huber_slope`/`base_score`
to test it properly.

**Candidate 2 trained normally but the improvement is noise-level.** A
calculation bug was caught during review: the initial in-script
comparison used the PHYSICAL_TEMPORAL/B_balanced baseline for all three
candidates, but Candidate 2 uses PHYSICAL_ONLY features. Corrected against
the right like-for-like baseline (PHYSICAL_ONLY/B_balanced: static
1.036m, dynamic 1.270m):

- static MAE: 1.036m → 1.023m (**+0.013m**, target was ≥0.05m)
- dynamic MAE: 1.270m → 1.273m (**-0.003m**, i.e. slightly worse)
- worst-bin (11-12m) MAE: 1.924m → 1.919m (**+0.005m**, target was ≥0.2m)

All three candidates **fail their own precommitted rejection criteria**.
None frozen. `models/` is empty; `targeted_retrain_manifest.json`:
`frozen_candidate: null`.

Repeated-seed evaluation (seeds 52/103/211, different group-to-fold
partitions each) and group-level bootstrap confidence intervals are in
`candidate_metrics.csv` / `group_bootstrap.csv` — consistent with the
above, no candidate's interval separates meaningfully from baseline.

## Independent review

`independent_review.json` — confirmed the Candidate 1/3 degenerate
collapse directly from raw predictions (not inferred from aggregate
metrics alone), caught and corrected the Candidate 2 baseline-comparison
bug described above, verified gate/leakage/checksum consistency, and
confirmed no criteria were loosened after seeing results. `leakage_review.json`
confirms group-disjoint folds, no duplicate groups, no GT-derived/
scenario/group-id/future-frame features, consistent with the unchanged
baseline contract.

## What was not done (by design, per this task's scope)

- No Gazebo/PX4/ROS2/backend process started.
- No recollection; frozen corpus untouched (only read).
- No outlier removed, no GT modified.
- No M52/calibration/PX4/controller/Follow Target change.
- No hyperparameter grid search (3 precommitted candidates only, no
  post-hoc additions).
- No candidate frozen without passing all four gates.

## Tests

- `test_core_range_5hz_headless_retrain.py`, `test_range_v2_baseline_eval.py`,
  `test_range_residual_dataset.py`, `test_range_physical_diagnostics.py`,
  `test_gazebo_ground_truth_history.py`, `test_sim_time_trajectory_and_gt.py`:
  77 passed.
- Full repository suite (`pytest -q --ignore=swarm_dashboard_handoff_20260729`,
  the ignored path is an unrelated stale backup copy nested in the repo):
  **432 passed**, unchanged from before this task (no estimator/runtime
  source code was modified — new analysis/candidate scripts only, plus a
  precommit-file typo fix made before any candidate trained).
- `./run_all.sh --check`: PASS.
- No process left running at any point (task never started one).

## Deliverables

`artifacts/core_range_3_12m/offline_targeted_retrain/`: `baseline_reproduction.json`,
`baseline_repro/` (full reproduced baseline run), `failure_matrix.csv`,
`per_group_failure.csv`, `per_bin_failure.csv`, `runtime_quality_correlation.csv`,
`direction_metrics.csv`, `temporal_failure.csv`, `bbox_feature_sensitivity.csv`,
`bbox_prediction_sensitivity.csv`, `feature_distribution_shift.csv`,
`nearest_neighbor_ambiguity.csv`, `feature_ablation.csv`,
`root_cause_precommit.json`, `candidate_precommit.json`, `fold_assignments.csv`,
`candidate_metrics.csv`, `static_metrics.csv`, `dynamic_metrics.csv`,
`temporal_metrics.csv`, `bbox_stress_metrics.csv`, `group_bootstrap.csv`,
`prediction_rows.csv`, `model_comparison.csv`, `leakage_review.json`,
`independent_review.json`, `targeted_retrain_manifest.json`, `final_summary.md`,
empty `models/`.

## Single next step

Fix the two evaluation-methodology artifacts (bbox perturbation not
recomputing downstream geometric/ROI features it should; temporal lag
search saturating even for the untouched raw signal) before spending more
model-development effort against the temporal and bbox gates — they
cannot currently distinguish a better model from the current one. In
parallel, a properly-tuned regime-aware candidate (correct `huber_slope`,
or a genuine near/core/far blend instead of a single global monotonic
constraint) targeting the 11-12m static regime remains the most
evidence-backed accuracy direction, but needs its own fresh precommit
round.
