# Core Range XGBoost Offline Benchmark Report

## Outcome

```text
DIRECT_XGBOOST_BEST_DEVELOPMENT_CANDIDATE
```

This is a static-development result only. It is not a promotion decision and
does not authorize replay, shadow, backend integration or controller use.

## Scope and safety

- Offline input only: the accepted priority static collection and accepted
  static balance collection.
- 27 independent groups, 832 frames, nine 1 m bins from 3 m to 12 m.
- No quarantine attempt was used.
- Seed 52 and three test folds were frozen before the first fit.
- No runtime estimator, physical formula, calibration, anchor selection,
  temporal filter, EKF, controller or PX4 code was changed.
- Runtime residual correction remains default-off.
- No simulation, shadow, Follow, OFFBOARD, arm, takeoff, mode or motion command
  was run.
- No final holdout was opened.

## Input integrity

Every source was revalidated before precommit and again before fitting:

| Check | Result |
|---|---|
| Source manifest/file checksums | PASS |
| Record and trace checksums | PASS |
| GT trace/checksum | PASS |
| Timestamp ordering | PASS |
| One raw run/session/group per dataset | PASS |
| Correct GT distance bin | PASS |
| Exactly 96 anchors per frame | PASS |
| Duplicate trace identity | 0 |
| Logging schema | `core_range_logging_3_12m_v001` |
| Config fingerprint | PASS; older priority rows reconstructed only from logged runtime config |

The legacy `samples.jsonl` writer did not contain every valid sidecar frame.
It was therefore retained only as a provenance/cross-check source. Model
features and labels come from the checksummed physical diagnostic sidecar, so
all 832 accepted raw frames remain represented without inventing values.

## Frozen benchmark policy

The plan was written before fitting:

```text
benchmark_plan SHA-256:
86ae646cca5cefc2c65942a9b5a7138f5546bacf03f2e9f582372fb91f6e8620
```

Each fold has exactly one held-out group from every bin: 9 test groups and 18
training groups. Median imputation and fixed missing indicators are fit only
on each fold's training groups. StandardScaler is not used. Training weights
give every training group equal total weight.

Three precommitted configurations A/B/C were evaluated for both objectives.
The selected OOF configurations are:

- residual: `B_balanced`;
- direct: `C_shallow`.

## Primary results

Primary comparison applies the same `[3,12] m` clipping policy to raw,
residual and direct outputs. Metrics below are equal-group unless noted.

| Method | MAE | P90 | Worst-bin MAE | Error >3 m | Median group std |
|---|---:|---:|---:|---:|---:|
| Raw clipped | 3.9190 m | 4.1272 m | 8.4013 m | 55.93% | 0.1267 m |
| Residual clipped | 1.2941 m | 1.6077 m | 1.9725 m | 8.06% | 0.2205 m |
| Direct clipped | **0.4266 m** | **0.4456 m** | **1.3992 m** | **0.00%** | **0.0000 m** |

The direct result improves equal-group MAE over clipped raw by 89.11%. Its
frame-weighted MAE is 0.4219 m and frame-weighted P90 is 1.4493 m. The
difference between frame-weighted P90 and mean per-group P90 is retained in
the reports; the selection metric remains the precommitted equal-group metric.

The residual model improves raw substantially, but fails the frozen overall
MAE, worst-bin MAE and error-over-3 m gates. Applying the legacy correction
clamp degrades residual equal-group MAE to 3.2044 m, confirming that the old
clamp cannot repair the observed multi-metre raw biases.

## Direct per-bin result

| Bin | Equal-group MAE | P90 | Worst group MAE |
|---|---:|---:|---:|
| 3–4 m | 0.6350 m | 0.6357 m | 1.0551 m |
| 4–5 m | 0.0476 m | 0.0476 m | 0.0743 m |
| 5–6 m | 0.0770 m | 0.0783 m | 0.1254 m |
| 6–7 m | 0.0690 m | 0.0690 m | 0.1699 m |
| 7–8 m | 0.0564 m | 0.0572 m | 0.0835 m |
| 8–9 m | 0.5341 m | 0.5346 m | 1.4475 m |
| 9–10 m | 0.6346 m | 0.6979 m | 0.9381 m |
| 10–11 m | 0.3868 m | 0.3874 m | 0.6113 m |
| 11–12 m | 1.3992 m | 1.5031 m | 1.7278 m |

The 11–12 m bin passes the frozen bin-mean gate by only about 0.10 m and must
be treated as borderline. Group-level failures are not hidden by the bin mean.

## Development gates

Direct clipped passes all six precommitted checks:

- equal-group MAE <= 1.0 m: PASS;
- worst-bin equal-group MAE <= 1.5 m: PASS;
- equal-group P90 <= 2.0 m: PASS;
- error >3 m <= 1%: PASS;
- median stationary group std <= 0.3 m: PASS;
- at least 50% MAE improvement over clipped raw: PASS.

Residual clipped fails three checks and is not the selected development
candidate.

## Overfitting and shortcut audit

The direct result is promising but has a material static-domain shortcut risk:

- `bbox_width_fraction` is the top gain feature in all three folds.
- Global held-out permutation of bbox width increases equal-group MAE by
  2.30 m on average; all other features have much smaller effects.
- Direct prediction correlation with bbox width is -0.935, with GT 0.968 and
  with raw range -0.370. These are descriptive correlations, not causality.
- Top-five gain-feature stability is low across folds (mean pairwise Jaccard
  0.217).
- Train MAE is 0.117–0.189 m while test MAE is 0.383–0.450 m, showing a clear
  but bounded generalization gap.
- 18/27 static groups have near-zero output standard deviation. The model does
  not collapse to one global mean (36 rounded output values and overall std
  2.33 m versus GT std 2.57 m), but its output is piecewise constant on this
  corpus.
- Worst groups are 11–12 m oblique (1.728 m), 11–12 m center (1.611 m), and
  8–9 m lateral-left (1.447 m).
- No exact or feature-space near-duplicate cross-group pair was detected under
  the documented descriptive thresholds. Image-level near-duplicates remain
  `UNKNOWN` because images are not retained in this audit.
- Neither unclipped direct output nor clipped output saturates at 3 m or 12 m
  on this corpus.

Consequently, `DIRECT_XGBOOST_BEST_DEVELOPMENT_CANDIDATE` means only that it is
the best method for the static development objective and frozen metrics. It
does not establish dynamic tracking quality or robustness to target-scale,
camera, background or target-type changes.

## Unsupported conclusions

The static corpus cannot evaluate approaching/receding response, range-rate,
lag, stop settling, boundary crossing, reacquire or calibration-transition
behavior. These remain `N/A`; no temporal filter result is claimed.

## Artifacts

Primary artifacts are under:

```text
artifacts/core_range_3_12m/xgboost_benchmark/
```

They include the immutable plan, source manifest, feature contract, Parquet
feature table, fixed folds, leakage audit, all prediction/metric tables,
feature importance and shortcut audit, fold preprocessors, six selected model
artifacts, report and checksum manifest.

## Decision

Development static benchmark: **GO for a separate replay/dynamic-evidence
review of the direct candidate**.

Runtime integration, shadow and controller use: **NO-GO** until explicitly
approved and until the bbox shortcut risk and missing dynamic evidence are
addressed by the next gate.

## Verification

```text
Focused benchmark tests: 14 passed
Full repository suite: 348 passed, 1 known PytestReturnNotNoneWarning
./run_all.sh --check: PASS
Stale PX4/Gazebo/backend/train process check: PASS (none found)
SWARM_RANGE_RESIDUAL_MODE: unset -> runtime default off
```

Exact benchmark commands are recorded in `benchmark_manifest.json`. The
workspace is not a valid Git repository, so reproducibility is anchored by the
precommit, source, evaluator, model and output checksums rather than a commit.

## Files changed

```text
requirements-ml.txt
core_range_xgboost_benchmark.py
test_core_range_xgboost_benchmark.py
docs/CORE_RANGE_XGBOOST_BENCHMARK_REPORT.md
docs/CORE_RANGE_OPTIMIZATION_3_12M.md
docs/RANGE_V2_IMPLEMENTATION_REPORT.md
artifacts/core_range_3_12m/xgboost_benchmark/*
```

No production runtime module was changed in this patch. Rollback consists of
removing only the new offline benchmark script/test/report/artifact directory
and reverting the single `pyarrow` offline dependency line; source collections
remain untouched.
