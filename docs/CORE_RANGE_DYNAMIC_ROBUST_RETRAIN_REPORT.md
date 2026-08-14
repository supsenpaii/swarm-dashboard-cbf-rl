# CORE RANGE 3–12M — DYNAMIC-ROBUST RETRAIN REPORT

Retrain ID: `core_range_dynamic_robust_retrain_20260805_v001`
Seed: `52`

## Conclusion

```
NO_DYNAMIC_ROBUST_MODEL_MEETS_GATE
```

No feature variant / hyperparameter configuration passed all four gate families
(static, dynamic, temporal, bbox robustness) simultaneously. Step 2 (runtime
integration and Follow Target SITL) was **not** started. No PX4, Gazebo,
shadow, or Follow Target process was ever launched in this task.

## Dataset

Integrity-audited corpus only, combined via a group-disjoint 3-fold split
(static-plus-dynamic per fold), seed `52`:

| Domain | Groups | Frames |
|---|---:|---:|
| Static (`static_balance_collection`) | 27 | 832 |
| Dynamic (`direct_dynamic_replay`, accepted sessions) | 8 | 844 |
| **Total** | **35** | **1676** |

All static integrity checks (`gt_distance_bin`, `gt_trace_checksum`,
`logging_schema`, `one_dataset_one_group_session`, `record_checksum`,
`runtime_config_fingerprint`, `source_checksum`, `timestamp_ordering`) are
`PASS`; zero quarantine attempts used. Dynamic corpus: 3 approaching, 3
receding, 2 stop-and-hold sessions (matches the audited corpus exactly).

Baseline candidate (`Direct XGBoost C_shallow`,
`8c3f2bb450439e6141143ae63ca021ac1fbbc76f25cd62e655aa434c652671fc`) was not
retrained, read, or overwritten — verified unchanged
(`baseline_candidate_checksum_unchanged: true`).

## Feature variants and configurations (precommitted before outer-test)

Three variants × three shallow/regularized XGBoost configs each, precommitted
in `retrain_plan.json` before any fold was fit:

- `PHYSICAL_ONLY` — physical/geometric/calibration features only, no bbox.
- `PHYSICAL_TEMPORAL` — `PHYSICAL_ONLY` plus causal (past-only) temporal
  features, reset at session boundaries.
- `BBOX_AUGMENTED` — baseline candidate feature set, with bbox geometry
  augmented (±2/5/10% scale, ±2/5% center) on training rows only.

## Results

| Variant | Config | Static MAE (m) | Dynamic MAE (m) | Median lag (s) | BBox ±10% catastrophic | All gates |
|---|---|---:|---:|---:|---:|---|
| PHYSICAL_ONLY | A_conservative | 1.003 | 1.661 | 1.950 | 12.47% | FAIL |
| PHYSICAL_ONLY | B_balanced | 1.032 | 1.444 | 1.925 | 8.77% | FAIL |
| PHYSICAL_ONLY | C_shallow | 1.232 | 1.691 | 2.000 | 12.59% | FAIL |
| PHYSICAL_TEMPORAL | A_conservative | 0.990 | 1.660 | 1.950 | 12.11% | FAIL |
| PHYSICAL_TEMPORAL | B_balanced | 1.042 | 1.515 | 1.950 | 10.44% | FAIL |
| PHYSICAL_TEMPORAL | C_shallow | 1.249 | 1.695 | 1.975 | 12.89% | FAIL |
| BBOX_AUGMENTED | A_conservative | 0.839 | 1.579 | 2.000 | 11.04% | FAIL |
| BBOX_AUGMENTED | B_balanced | 0.840 | 1.421 | 2.000 | 8.11% | FAIL |
| BBOX_AUGMENTED | C_shallow | 0.858 | 1.778 | 2.000 | 14.44% | FAIL |

Full per-group, per-bin, and per-frame breakdowns:
`artifacts/core_range_3_12m/dynamic_robust_retrain/{static,dynamic,temporal,bbox_stress,per_group,per_bin,prediction_rows}*.csv`.

### Gate margins (why it failed)

- **Dynamic accuracy**: best equal-group dynamic MAE is 1.421 m
  (`BBOX_AUGMENTED`/`B_balanced`) against a 1.0 m gate — 42% over budget. No
  configuration reached the gate.
- **Temporal response**: best median absolute lag is 1.925 s against a 0.30 s
  gate — roughly 6.4× over budget across every variant, including
  `PHYSICAL_TEMPORAL`, which was specifically designed to reduce lag via
  causal features. This is the dominant, variant-independent failure mode.
- **BBox robustness**: every configuration produced catastrophic (>3 m) errors
  under ±10% bbox perturbation (8–14% of stressed frames), against a gate of
  zero catastrophic errors. Even the physical-only variants (no bbox features
  used for prediction) failed this because catastrophic error is measured on
  the underlying frame set, not just bbox-input sensitivity.
- **Static accuracy**: the least-bad dimension — 3 of 9 configurations came
  within or near the 1.0 m static gate (`BBOX_AUGMENTED` configs: 0.839–0.858
  m), but static-only performance is explicitly excluded as a selection
  criterion when other gates fail.

No configuration was selected. `frozen_model_checksum: null` — no model was
persisted as a runtime candidate.

## Precommit integrity note

During verification, the artifact set found under
`artifacts/core_range_3_12m/dynamic_robust_retrain/` had a `retrain_plan.json`
whose recorded `fold_assignments_sha256` did not match the `fold_assignments.csv`
present on disk (`a7add932…` vs `a8a8a535…`), and file timestamps showed
`fold_assignments.csv` had been rewritten after the precommit files were
frozen — a violation of the precommit/no-post-hoc-change requirement that this
retrain design otherwise enforces. Root cause: the fold-assignment file format
was extended (added a `domain` column) after the original run, and the older
two-column file was left in place without being regenerated alongside the
newer precommit-hash check added to `_validate_precommit`.

The pipeline was re-run end to end (fresh `prepare` + `run`, same workspace,
same seed) to restore a self-consistent, hash-verified precommit chain. The
regenerated `fold_assignments.csv` carries an identical group→fold mapping to
the original (only the added `domain` column differs), and the resulting
`model_comparison.csv` is byte-identical to the prior run's — confirming the
conclusion is reproducible and was not an artifact of the inconsistency. The
prior artifact set was discarded; the artifacts now on disk are the verified,
self-consistent set referenced above.

## Scope guards (all satisfied)

No additional data collection. No runtime code modified. No controller
effect. No shadow mode run. No Follow Target action. No final holdout
consumed. No PX4/Gazebo/SITL process started at any point in this task.

## Disposition

Per gate policy, `NO_DYNAMIC_ROBUST_MODEL_MEETS_GATE` is a hard stop:

- Step 2 (runtime integration, shadow mode, Follow Target SITL) is **not**
  performed.
- No model was integrated, shadowed, or connected to the Follow Target
  interface.
- No model is persisted as a research-only artifact beyond the OOF prediction
  rows already in `prediction_rows.csv` (all nine configurations failed by a
  wide margin — dominated by the ~2 s temporal lag gap — so none is a
  meaningful reference candidate to freeze separately).
- All processes used for this task (training only; no PX4/Gazebo/backend/
  capture/shadow) have exited; nothing was left running.

## Full test verification

- Focused tests (`test_core_range_xgboost_benchmark.py`,
  `test_core_range_direct_dynamic_replay.py`,
  `test_core_range_logging_eval.py`,
  `test_core_range_static_balance_collection.py`,
  `test_core_range_priority_collection_audit.py`): 43 passed.
- Full repository suite (excluding the unrelated stale
  `swarm_dashboard_handoff_20260729/` snapshot bundle, which has import-name
  collisions with live top-level test modules and is not part of the current
  codebase): 373 passed.
- `./run_all.sh --check`: passed.

## Deliverables

```
artifacts/core_range_3_12m/dynamic_robust_retrain/
  retrain_plan.json
  frozen_dataset_manifest.json
  feature_variants.json
  fold_assignments.csv
  static_metrics.csv
  dynamic_metrics.csv
  temporal_metrics.csv
  bbox_stress_metrics.csv
  per_group_metrics.csv
  per_bin_metrics.csv
  prediction_rows.csv
  model_comparison.csv
  retrain_manifest.json
  retrain_report.md
  models/   (empty — no candidate passed all gates)
```
