# Giai đoạn 1 — Residual Bias Correction (CORE_RANGE_RESIDUAL_BIAS_CORRECTION)

## Outcome

```text
RESIDUAL_BIAS_CORRECTOR_GATE_FAILED
```

Follow-up to `CORE_RANGE_CONTROL_READY_OBSERVATION_PIPELINE`, which found
that windowing (0.6-1.0s) could not fix Candidate B's direction-agreement
problem because part of its error is **persistent regional bias**, not
high-frequency noise. This task implemented the user's own follow-up
roadmap's Giai đoạn 1: audit that bias by geometry/distance context, then
train a small residual corrector on top of the frozen Candidate B backbone.
The bias audit confirmed the premise (real, large, persistent bias). The
residual corrector, however, **does not generalize** under group-disjoint
CV — held-out correlation between predicted and true residual is
weak-to-negative for all 3 precommitted configs. The Giai đoạn 1 offline
gate failed for all of them. Per the roadmap's own logic, this is the
diagnosed trigger condition for Giai đoạn 2 (targeted small data
collection) — not further corrector tuning, which was not attempted here.

## Phase 1.1 — Bias audit

`core_range_bias_audit.py` joined Candidate B's unchanged raw OOF
predictions (bit-exact reused from `stable_relative_tracking/
prediction_rows.csv`) with the full runtime feature set, and computed
residual `e = GT - Candidate B prediction` broken down by a precommitted,
disclosed `geometry_context` label (`center`/`lateral`/`yaw_oblique`,
derived from group-naming substrings, decided before any bias number was
computed) crossed with the 9 distance bins and static/dynamic domain.

- **20 of the (domain × geometry_context × distance_bin) cells show ≥1m
  mean signed bias**, up to **3.13m** (`static/center/11-12m`) and
  **-3.07m** (`static/lateral/3-4m`). This is large relative to the
  ≤1.5m global-MAE gate used throughout this line of work.
- Pooled (all-context) Spearman correlation of every audited runtime
  feature against the residual is weak (|r| ≤ 0.23) — consistent with the
  prior task's finding that `raw_physical_range_m`/ROI-depth features flip
  correlation sign across geometry contexts.
- Per-context correlations are moderate (up to ~0.40 for
  `target_roi_q_p25`/`p10`/`min` within `lateral`), but **only 4 of the 17
  audited features keep a consistent correlation sign across all 3
  geometry contexts**: `ray_scale`, `target_inverse_depth`,
  `target_roi_q_min`, `target_roi_q_p10`
  (`bias_feature_sign_consistency.csv`) — used as Config C's minimal
  feature set.
- **Flagged leakage risk, disclosed before training**: the dynamic corpus
  has only 8 groups; some (domain, geometry_context) cells are dominated
  by a single group (e.g. `dynamic/yaw_oblique` = only
  `cdr_approach_right_yaw` + `cdr_recede_left_yaw`). A residual corrector
  conditioned on features highly correlated with geometry_context risks
  learning group identity rather than a general geometric law — this
  concern is exactly what Phase 1.2/1.3 subsequently confirmed.

## Phase 1.2 — Residual corrector training

`core_range_bias_residual_corrector.py`, precommitted in
`residual_corrector_precommit.json` before training: 3 configs, all
predicting `residual_m = GT - candidate_b_pred_m` from
`STABLE_FEATURES (or the 4-feature audited-stable subset) +
candidate_b_pred_m`, group-disjoint 3-fold CV (same `assign_combined_folds`
seed=52, reused unchanged), `unified_group_weights()` sample weighting,
train-fold-only preprocessing. No session/scenario/geometry-context-
categorical/GT-bin/future-frame input; no `reg:pseudohubererror` (standing
ban from a prior task's collapse finding).

| Config | Model | Features |
|---|---|---|
| A | Ridge (alpha=1.0) | 27 STABLE_FEATURES + candidate_b_pred_m, standardized |
| B | XGBoost, strong reg (λ=10, α=0.5, depth=2) | same 28 inputs as A |
| C | XGBoost, minimal reg (λ=5, α=0.2, depth=2) | 4 audited-stable features + candidate_b_pred_m |

None of the 3 collapsed (residual predictions have healthy variance). But
`held_out_generalization_check.json` shows the OOF-predicted residual is
only weakly (Config A: ρ=0.20) or **negatively** (Config B: ρ=-0.30, Config
C: ρ=-0.14) correlated with the true residual on held-out folds. A control
check — fitting Config A's Ridge model on one fold's own training data and
scoring it in-sample — gives R²=0.53 / Spearman=0.45, proving the models
*can* fit a real relationship in-sample; the drop to near-zero/negative
out-of-fold is the classic signature of overfitting small-corpus,
group-specific structure, not a sign or feature-matrix bug.

Raw (pre-filter) corrected-signal MAE over the full combined corpus is
**worse** than Candidate B's own unchanged 1.178m for every config (A:
1.446m, B: 1.358m, C: 1.312m) — the correction is net-harmful on held-out
data, not merely neutral.

## Phase 1.3 — Gate evaluation

Reused `control_ready_range_estimator.py` unmodified (alpha=0.65/
beta=0.03/threshold=2.0m unchanged), with **age compensation forced off**
per the roadmap's own instruction (`measurement_age_s=0.0` on every call,
so `predicted_current_range_m == filtered_range_m`). Evaluated all 8
dynamic groups per config against the Giai đoạn 1 gate
(`residual_corrector_precommit.json`'s `residual_corrector_gate_thresholds`).

| Config | Bias reduction | Direction (best window) | Approach. | Recede | Stationary std | Global MAE | Worst-bin MAE | Catastrophic |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| A Ridge | 20.2% | 0.511 | 0.512 | 0.509 | 0.560m | 1.523m | 2.152m | 0 |
| B XGB strong-reg | 1.5% | 0.524 | 0.517 | 0.531 | 0.613m | 1.308m | 1.980m | 0 |
| C XGB stable-subset | -13.1% | 0.533 | 0.517 | 0.547 | 0.738m | 1.352m | 2.044m | 0 |

Targets: bias reduction ≥50%, direction agreement ≥0.80 (approach/recede
≥0.75), stationary std ≤0.55m, global MAE ≤1.5m, worst-bin MAE ≤2.0m,
catastrophic=0. **0 of 3 configs pass** — bias reduction and direction
agreement fail for all three (direction agreement is essentially unchanged
from the raw-Candidate-B baseline of 0.516 found in the prior task), and
2 of 3 also fail stationary std and/or worst-bin MAE.
`residual_corrector_selection.json`: `passing_configs: []`, conclusion
`RESIDUAL_BIAS_CORRECTOR_GATE_FAILED`.

## Independent review

`independent_review.json` — confirmed precommit-before-results ordering,
confirmed the failure is not a formulation/sign bug (in-sample fit is
healthy; the drop is specifically on held-out data), confirmed no 4th
config or hyperparameter change was tried after seeing the negative
correlations, confirmed the gate was evaluated honestly for all 8 checks
across all 3 configs rather than stopping early. `leakage_review.json`
confirms the corrector never sees session/scenario/geometry-context-
categorical/GT-bin/future-frame inputs and that Candidate B itself was
never retrained.

## What was not done (by design, per the roadmap's own gating)

- No Giai đoạn 2 (targeted data collection) — the roadmap's own trigger
  condition for it (residual corrector failing group-disjoint validation)
  was reached, but starting new live-Gazebo collection is left for the
  user to confirm, not decided autonomously here.
- No Giai đoạn 3 (observation-only runtime integration) or Giai đoạn 4
  (shadow controller / Follow Target SITL) — both explicitly gated on
  Giai đoạn 1 or 2 passing, which did not happen.
- No Gazebo/PX4/ROS2/backend process was started at any point in this
  task (fully offline, reusing already-verified bit-exact artifacts).
- No change to Candidate B, M52, MiDaS, calibration, controller, or PX4.

## Tests

- Full repository suite (`pytest -q --ignore=swarm_dashboard_handoff_20260729`):
  unchanged pass count from the prior task (no runtime/estimator source
  outside the 3 new analysis scripts was modified).
- `./run_all.sh --check`: PASS.
- No process left running (none was ever started).

## Deliverables

`artifacts/core_range_3_12m/residual_bias_correction/`: `bias_map.csv`,
`per_group_bias.csv`, `feature_bias_correlation.csv`,
`bias_feature_sign_consistency.csv`, `geometry_context_definition.json`,
`residual_corrector_precommit.json`, `raw_corrector_metrics.csv`,
`corrected_prediction_rows_{A,B,C}.csv`, `held_out_generalization_check.json`,
`gate_result_{A,B,C}.json`, `bias_reduction_summary.csv`,
`control_ready_metrics_by_config.csv`, `residual_corrector_selection.json`,
`leakage_review.json`, `independent_review.json`, `source_changes.json`,
`final_manifest.json`, `final_summary.md`.

## Single next step

Per the roadmap's own Giai đoạn 2: a small, **targeted** dynamic
collection batch (3-4m center+lateral, 4.5-8m approaching/receding,
10-12m, more yaw/lateral offset variety, stop-and-hold at 6m and 9m; each
configuration needs multiple independent sessions so a future residual
corrector can be group-disjoint-validated within each geometry context,
not just across contexts) is the diagnosed way to give a future corrector
enough independent per-context examples to generalize. This requires live
Gazebo sessions and is subject to the still-unresolved RTF-stutter data
collection cost documented in earlier tasks — recommended to confirm with
the user before starting, given its cost and the standing "no new
collection without explicit sign-off" caution in this task chain.
