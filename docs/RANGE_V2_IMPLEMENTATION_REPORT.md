# Range Estimation V2 Implementation Report

Date: 2026-08-04  
Frozen spec: `range_v2_near_core_far_r1_v001`  
Current completed patch: **R2 — offline derived-label and data coverage audit**  
Runtime status: **unchanged; residual correction default remains off**

## Patch history

### R1 — Audit and frozen specification

Status: complete, `GO WITH CONDITIONS` for R2 only.

Artifacts:

- `docs/RANGE_V2_NEAR_CORE_FAR_COMPATIBILITY_REPORT.md`
- `docs/RANGE_V2_FROZEN_SPEC.md`

No model was trained and no runtime path changed.

### R2 — Offline derived labels and coverage audit

Status: **complete**. The R2 tooling read the current 32-dataset development
corpus and wrote only derived files under `artifacts/range_v2/`. Original
dataset `manifest.json` and `samples.jsonl` SHA-256 values were captured before
and after the audit and matched exactly.

#### Files changed

```text
range_v2_labels.py
range_v2_dataset_audit.py
test_range_v2_labels.py
test_range_v2_dataset_audit.py
docs/RANGE_V2_IMPLEMENTATION_REPORT.md

artifacts/range_v2/derived_labels.jsonl
artifacts/range_v2/data_coverage_report.csv
artifacts/range_v2/data_coverage_report.md
artifacts/range_v2/coverage_status.csv
artifacts/range_v2/leakage_audit.json
artifacts/range_v2/source_dataset_manifest.json
artifacts/range_v2/r2_audit_manifest.json
```

No historical dataset, historical manifest, old model artifact, runtime module,
environment default, or controller integration file changed.

#### Derived-label contract

Each row in `derived_labels.jsonl` contains:

- source dataset/root and source manifest/samples paths;
- source manifest and samples SHA-256;
- exact source line number and exact source-record SHA-256;
- source run, session, group, frame, timestamps, and target identity;
- audited GT source/frame/quality/uncertainty/time-offset state;
- canonical distance zone, transition band, core bin, and frozen eligibility
  annotations;
- explicit `unknown` for validity, direction, location/background/pitch,
  epochs, and other evidence absent from the legacy schema.

The old deterministic applicability result is a separate policy annotation. It
is never used as `validity_gt`.

#### Exact audit result

```text
source datasets:                         32
atomic run/session groups:               32
derived rows:                         2,500
unique row trace identities:          2,500
source checksum pairs:                   32
source originals unchanged:            true
duplicate run/session/frame identities:   0
atomic groups spanning source roots:      0
one-run/session dataset violations:       0
group leakage audit:                   PASS
static-development GT rows:           2,500
dynamic/direction-eligible rows:           0
validity_gt=unknown rows:              2,500
deterministic policy accepted:         2,340
deterministic policy rejected:           160
```

All 160 policy rejections have the historical reason
`image_ray_x_outside_validated_envelope`. They are not annotated invalid ground
truth and therefore contribute zero invalid-regime promotion coverage.

#### Coverage decision

| Region/bin | Frames | Groups | Status |
|---|---:|---:|---|
| near `<3 m` | 0 | 0 | MISSING |
| core `3–4 m` | 121 | 2 | PARTIAL |
| core `4–5 m` | 98 | 3 | PARTIAL |
| core `5–6 m` | 84 | 2 | PARTIAL |
| core `6–7 m` | 168 | 1 | MISSING |
| core `7–8 m` | 1,236 | 13 | PARTIAL |
| core `8–9 m` | 349 | 6 | PARTIAL |
| core `9–10 m` | 444 | 5 | PARTIAL |
| core `10–11 m` | 0 | 0 | MISSING |
| core `11–12 m` | 0 | 0 | MISSING |
| far `>12 m` | 0 | 0 | MISSING |
| invalid | 0 annotated | 0 | MISSING |
| near/core transition | 0 | 0 | MISSING |
| core/far transition | 0 | 0 | MISSING |

No bin is `COVERED`: location, background, pitch/geometry, approach, recede,
lateral, calibration-event, and reacquire-event evidence is absent or not
authoritatively encoded. Dataset names were not treated as metadata.

Every invalid regime is `MISSING`: bbox lost, bbox too small, ROI noise,
calibration invalid, critical feature missing, unstable MiDaS, severe OOD
geometry, reacquire hold, and calibration hold.

#### Timestamp and GT limitations

All 32 groups have strictly increasing measurement timestamp, simulation source
timestamp, and frame index. All stored GT totals pass the legacy `0.20 m` and
`100 ms` limits and support static development distance labels.

The legacy records do not contain measurement clock ID, proven sensor-capture
timestamp, capture-to-receipt uncertainty, separately timestamped GT, full GT
uncertainty components, calibration epoch, or track epoch. They are therefore
not evidence for direction, response lag, calibration/reacquire events, or
promotion coverage. Those fields remain `unknown`; no value was inferred.

#### Tests run

```text
PYTHONPATH=/home/sup/swarm_dashboard/.venv/lib/python3.12/site-packages \
  python3 -m pytest -q test_range_v2_labels.py test_range_v2_dataset_audit.py
Result: 23 passed

PYTHONPATH=/home/sup/swarm_dashboard/.venv/lib/python3.12/site-packages \
  python3 -m pytest -q test_*.py
Result: 257 passed, 1 known PytestReturnNotNoneWarning

./run_all.sh --check
Result: Runtime check passed
```

No PX4, Gazebo, MicroXRCE, backend, runtime, or range-training process remained
after the checks.

#### Artifact SHA-256

```text
coverage_status.csv
  24cb43b1e655bc2d32dbd8e9c84d1684f420a7f919d2394e8e4868fdf8d326a2
data_coverage_report.csv
  1adc9cf91d6c28deb373a36d3b620d5fc711a9ff9cd7f1c1db6d9f9dcfa2d56a
data_coverage_report.md
  bd99bff5a57cf3a2c3a27552463d33a87134ec64e5f81aba1aaf4050ad1f2928
derived_labels.jsonl
  b6fd1e20a6bf1c38e1ef5ba61777f62631eab70109f9c25be21940043275332f
leakage_audit.json
  a4ef751d7078c5a3db28ce5d3e64965a4a0895291a7fa6ba84d478041f112506
r2_audit_manifest.json
  fc5ecca960aebd1dee4c8991f363064c7b595e8f1437dc2f260cca71534441f6
source_dataset_manifest.json
  674f42147f4f25be4b2674f1f0f1b765aa6ddb0ede2f02760202f826c8e2dcab
```

The authoritative per-source checksum list is embedded in
`source_dataset_manifest.json` and `r2_audit_manifest.json`.

#### Known limitations

- Workspace root still lacks usable Git metadata, so source identity relies on
  the frozen-spec and file SHA-256 manifests.
- R2 creates no train/calibration/validation/test split; cross-partition leakage
  is not applicable yet and must be audited when a later split is created.
- Development coverage is static and core-only.
- Promotion coverage has not been collected and remains `MISSING`.

#### Rollback

Remove only the two R2 Python modules, two R2 test files, this report, and the
derived `artifacts/range_v2/` directory. No original dataset/model/runtime state
requires restoration.

#### Go/No-Go

- R2 deliverable: **PASS**.
- R3 offline raw-baseline evaluation: **GO WITH CONDITIONS**, restricted to the
  available static core subset; unsupported metrics remain `unknown`/`N/A`.
- Model training, q90 fitting, threshold calibration, promotion, shadow,
  simulation, runtime integration, and controller effect: **NO-GO**.

R2 stopped pending approval and was subsequently approved for R3.

### R3 — Raw physical baseline offline

Status: **complete**. Outcome: `RAW_BASELINE_CHARACTERIZED`.

R3 evaluated only the 2,500 trustworthy static-core development rows locked in
the R2 source manifest. It reverified the frozen spec, seven R2 artifacts and
64 source files before/after evaluation. All 2,500 baseline rows have unique
source dataset/run/session/group/frame/line/record-checksum trace keys.

#### Files changed

```text
range_v2_baseline_eval.py
test_range_v2_baseline_eval.py
docs/RANGE_V2_IMPLEMENTATION_REPORT.md

artifacts/range_v2/raw_baseline/raw_baseline_rows.csv
artifacts/range_v2/raw_baseline/per_group_metrics.csv
artifacts/range_v2/raw_baseline/per_bin_metrics.csv
artifacts/range_v2/raw_baseline/policy_slice_metrics.csv
artifacts/range_v2/raw_baseline/unsupported_metrics.json
artifacts/range_v2/raw_baseline/raw_baseline_summary.json
artifacts/range_v2/raw_baseline/raw_baseline_manifest.json
artifacts/range_v2/raw_baseline/raw_baseline_report.md
```

No source dataset/manifest, R2 derived artifact, model, historical report,
geometry/calibration/MiDaS module, temporal filter, runtime, or controller path
changed.

#### Aggregate raw baseline

| Aggregation | Bias m | MedAE m | MAE m | P90 m | P95 m | Mean absolute relative error | Catastrophic `>3m` |
|---|---:|---:|---:|---:|---:|---:|---:|
| Frame weighted | -0.412 | 3.040 | 3.280 | 5.543 | 7.757 | 0.468 | 0.508 |
| Equal group | -0.745 | 3.233 | 3.246 | 4.052 | 4.122 | 0.483 | 0.488 |

The group-cluster bootstrap uses 5,000 whole-group resamples and seed 52. Its
equal-group 95% intervals are:

```text
signed bias: [-1.935, 0.672] m
MAE:         [ 2.545, 4.069] m
```

No frame-independent bootstrap was used.

#### Per-bin result

| Bin | Frames | Groups | Bias m | MAE m | P90 m | P95 m | Catastrophic rate | Raw limit comparison |
|---|---:|---:|---:|---:|---:|---:|---:|---|
| 3–4 m | 121 | 2 | 7.020 | 7.020 | 12.323 | 12.382 | 0.512 | EXCEEDS |
| 4–5 m | 98 | 3 | -0.135 | 0.982 | 2.255 | 3.681 | 0.071 | EXCEEDS |
| 5–6 m | 84 | 2 | 2.489 | 2.606 | 5.931 | 6.175 | 0.417 | EXCEEDS |
| 6–7 m | 168 | 1 | 3.787 | 3.787 | 5.753 | 5.974 | 0.905 | EXCEEDS |
| 7–8 m | 1,236 | 13 | -0.003 | 2.975 | 4.796 | 7.773 | 0.404 | EXCEEDS |
| 8–9 m | 349 | 6 | -2.836 | 2.874 | 4.456 | 5.040 | 0.450 | EXCEEDS |
| 9–10 m | 444 | 5 | -3.871 | 3.871 | 5.543 | 5.672 | 0.809 | EXCEEDS |
| 10–11 m | 0 | 0 | N/A | N/A | N/A | N/A | N/A | MISSING |
| 11–12 m | 0 | 0 | N/A | N/A | N/A | N/A | N/A | MISSING |

This table compares raw data to frozen absolute limits only. It is not a model
promotion PASS/FAIL result.

#### Deterministic-policy slices

| Slice | Frames | Groups | Bias m | MAE m | P90 m | P95 m |
|---|---:|---:|---:|---:|---:|---:|
| Accepted | 2,340 | 28 | -0.866 | 3.065 | 5.131 | 5.885 |
| Rejected | 160 | 4 | 6.224 | 6.424 | 12.293 | 12.355 |

Rejected reasons are 160
`image_ray_x_outside_validated_envelope` frames distributed across 3–4 m (62),
4–5 m (54), and 5–6 m (44). Policy-rejected rows remain `validity_gt=unknown`;
they were not converted to invalid ground truth. Same-bin accepted/rejected
differences are recorded descriptively in `policy_slice_metrics.csv`; every
current comparison lacks at least two independent groups in one slice, so frame
count is not promoted to independent evidence.

#### Stationary result and blockers

Twenty-nine of 32 groups exceed at least one comparable raw stationary limit.
The observed per-group extrema include:

```text
output std:             0.056 to 1.852 m
P95 frame delta:        0.014 to 0.679 m
linear drift slope:    -0.093 to 0.065 m/s
maximum 10-second span: 0.160 to 3.159 m
group signed bias:     -5.143 to 12.200 m
```

Systematic raw bias blockers are recorded for 3–4, 5–6, 6–7, 8–9, and 9–10
m. The sign changes strongly by bin/group, so R3 does not attribute the cause.
A separate audit of intrinsics, pitch, timestamp, calibration and geometry is
required. XGBoost must not be assumed to mask the root cause.

#### Unsupported metrics

Direction classification, response lag/range rate, stop response, both distance
boundaries and promotion performance are `N/A`. Calibration/reacquire event
jumps are `UNKNOWN`. Static data was not used to infer any of them.

#### Tests and checks

```text
PYTHONPATH=/home/sup/swarm_dashboard/.venv/lib/python3.12/site-packages \
  python3 -m pytest -q test_range_v2_baseline_eval.py
Result: 23 passed

PYTHONPATH=/home/sup/swarm_dashboard/.venv/lib/python3.12/site-packages \
  python3 -m pytest -q test_*.py
Result: 280 passed, 1 known PytestReturnNotNoneWarning

./run_all.sh --check
Result: Runtime check passed
```

Post-run verification: seven R3 output checksums, seven R2 artifact checksums,
64 source checksums, evaluator checksum, and 2,500 unique trace keys all pass.
No PX4/Gazebo/backend/range-training process remained.

#### Artifact SHA-256

```text
per_bin_metrics.csv
  3f0b2173714086c37411c9f90b9b7f59758d37c817d4dc577817d38daafc5e2d
per_group_metrics.csv
  dc9bd193eab122deef7c9751be95b79cabab8de66741978530c5289df56a15bc
policy_slice_metrics.csv
  745466e597c708e46c70594f812fbacd9d8c5ff1c8029187660b7b8b9550f3ab
raw_baseline_manifest.json
  f6e3c3c973dbc568553639651e9a3b817c65a2ddbb85dfc675dd9333506426b5
raw_baseline_report.md
  e59fc442a420708be54a2b6f70cd2e9346e3a1664ba768318741eb5abe860809
raw_baseline_rows.csv
  4433b9c5bb02722969be98de9ca43d8a5dffd93199124cf9db478cce64d60134
raw_baseline_summary.json
  65767c38d3e56aeea6ccdf28cdd45d2ecc1ee965dd83b8ae3be60aa06ad9d84c
unsupported_metrics.json
  a6697634a60ee2f56908300bda095fd019edd4136ebb74362986bae1b370cd13
```

The manifest intentionally excludes its own checksum from its internal output
map to avoid recursive hashing; its external SHA-256 is listed above.

#### Known limitations

- Only static core development data exists; 10–12 m remains missing.
- Receipt-domain timestamps permit descriptive stationary analysis but not
  dynamic latency/direction inference.
- Calibration/track epochs and authoritative context annotations are absent.
- Root Git metadata is still unusable; source identity relies on SHA-256.

#### Rollback

Remove only `range_v2_baseline_eval.py`, its test, this R3 report section, and
`artifacts/range_v2/raw_baseline/`. No original/R2/runtime state needs restore.

#### Go/No-Go

- R3 outcome: **`RAW_BASELINE_CHARACTERIZED`**.
- Range V2 model conclusion: **not applicable; no model was trained**.
- R4/R5, model training, q90, promotion, shadow, simulation and runtime:
  **not started / NO-GO pending explicit approval**.

Work stops after R3.

## Patch R3.5 — Physical Range Root-Cause Audit

### Outcome

```text
ROOT_CAUSE_CONFIRMED_FIX_REQUIRED
```

R3.5 completed a read-only/offline reconstruction of all 2,500 locked static
core development rows and 32 independent groups. It did not start R4/R5.

### Confirmed evidence

- R1, R2, R3 and all 64 locked source files passed checksum verification
  before and after the audit.
- All 2,500 stored raw values exactly satisfy
  `raw=(1/(a*q+b))*sqrt(1+x^2+y^2)`; maximum reconstruction error is zero.
- Target q is constant within all 32 groups; ray scale is constant in 31/32.
  Applied calibration a/b vary within every group. With q and ray fixed at
  group medians, a/b variation reproduces essentially all raw standard
  deviation (median calibration-only/full ratio `1.000000`, minimum
  `0.999385`). Affine calibration parameter variation is therefore confirmed
  as the direct cause of stationary raw variation in this corpus.
- The physical quantity contract is not identical: GT uses Gazebo
  `camera_link` origin to target model origin, while anchor geometry defaults
  to vehicle center and target depth is an eroded-bbox foreground statistic.
- Downstream `update_range` cannot cause R3 raw drift because the collector
  writes `physics_distance_m` before that filter.

The audit does not establish why calibration a/b vary or quantify how much of
the multi-metre bias comes from reference-center/target-surface mismatch.
Anchor-level logs, calibration epochs/age/source, camera/extrinsic hashes and
capture/pose/depth timestamps are absent.

### Files changed

```text
range_v2_physical_audit.py
test_range_v2_physical_audit.py
docs/RANGE_V2_PHYSICAL_ROOT_CAUSE_AUDIT.md
docs/RANGE_V2_IMPLEMENTATION_REPORT.md

artifacts/range_v2/physical_root_cause/quantity_contract.json
artifacts/range_v2/physical_root_cause/code_path_inventory.json
artifacts/range_v2/physical_root_cause/per_frame_diagnostic.csv
artifacts/range_v2/physical_root_cause/per_group_diagnostic.csv
artifacts/range_v2/physical_root_cause/configuration_fingerprints.csv
artifacts/range_v2/physical_root_cause/hypothesis_matrix.csv
artifacts/range_v2/physical_root_cause/missing_diagnostic_fields.json
artifacts/range_v2/physical_root_cause/physical_root_cause_manifest.json
artifacts/range_v2/physical_root_cause/physical_root_cause_report.md
```

No original dataset/manifest, R2/R3 artifact, model, production geometry,
calibration, MiDaS, temporal filter, runtime, controller or PX4 source changed.

### Tests and checks

```text
python3 -m pytest -q test_range_v2_physical_audit.py
Result: 14 passed

PYTHONPATH=/home/sup/swarm_dashboard/.venv/lib/python3.12/site-packages \
  python3 -m pytest -q test_*.py
Result: 294 passed, 1 known PytestReturnNotNoneWarning

./run_all.sh --check
Result: Runtime check passed
```

An initial full-suite collection with the system-only module path lacked
`fastapi`; this was an environment-path error. The same locked dependency path
used by R2/R3 produced the passing result above.

Final R3.5 SHA-256:

```text
range_v2_physical_audit.py
  dedbf86b0448b0d4ccce52d7b7eefd326c922495b53889972e08edbe348ed48c
test_range_v2_physical_audit.py
  9871c9beb5cebcf73d84e1652234b348706ccaba04182bdc7d37a5da12a52d10
docs/RANGE_V2_PHYSICAL_ROOT_CAUSE_AUDIT.md
  43ee3ecca8d4dc08468e041caaa756c37d4d46e55b5c9c84c26c4dbb9c69c4df
code_path_inventory.json
  7976792242d559c3c2802abb9fd6e13a1d49fe71ceb0aa059851743754cd691d
configuration_fingerprints.csv
  007001a34b7bd7f3d23ca45e61e7a50abdd6eebb79340fc100f2ff9be70d146f
hypothesis_matrix.csv
  4b8b5c7db2425ffa98f0eb860c16a2f3ed9fa5e9b9f945e377ee292978548100
missing_diagnostic_fields.json
  58997f916b6e77d71dd0083c14df9d11eec234c282b5e7d0c16cf6ea3bf1a3a1
per_frame_diagnostic.csv
  2fa9feae98056a31db6686f3a388af8f468826ede777a70a86cf69074de9df18
per_group_diagnostic.csv
  5d0492006b7c4227943bc634c96480c569706b20b17e477680cc96b3899269c9
physical_root_cause_report.md
  bc3c72a87902f7047b376f44db6c41933102fffc02958cfb63623eacec02ab57
quantity_contract.json
  a8778bf5d28d25bcf7474198e0a1582a9b2e7b91eb3da5712d616e204c6af7b5
physical_root_cause_manifest.json
  fdde2beb2953ad58e10d422e1fb87108d14e8152d5e9120ceaafc3cad3d096d2
```

### Known limitations

- Actual source-run CameraInfo, distortion/rectification, model/SDF checksum,
  poses and extrinsics were not recorded.
- Per-anchor q/depth/pixel samples, counts and rejection reasons were not
  retained.
- Calibration raw-fit parameters, cache age/source, change-point state and
  calibration/track epochs were not retained.
- Static legacy data cannot establish dynamic latency.
- Root Git metadata remains unusable; identity is SHA-256 based.

### Rollback

Remove only the R3.5 audit module/test, R3.5 report section/file, and
`artifacts/range_v2/physical_root_cause/`. No source or runtime restoration is
needed.

### Go/No-Go

- R3.5: **root-cause fix/instrumentation required before R4 or R5**.
- ML training/promotion: **not applicable and not started**.
- R4/R5, simulation, shadow and runtime: **NO-GO pending explicit approval**.

Work stops after R3.5.

## Patch R3.6A — Instrumentation only

### Outcome

```text
R3_6A_INSTRUMENTATION_COMPLETE_AWAITING_REVIEW
```

R3.6A added an opt-in append-only physical-range diagnostics sidecar for
future evidence collection. The schema records trace identity, reference
centers, runtime CameraInfo and extrinsics, every configured M52 anchor,
raw-versus-filtered fit and applied calibration, cache age/source/TTL,
recovery/reseed state, track/calibration epochs, depth queue/inference timing
and available pose/gimbal/GT synchronization evidence.

Unknown quantities remain explicit. In particular, the existing interfaces do
not provide a verified `camera_link → optical center` transform, target-center
to visible-surface vector, sensor-capture timestamp, or authoritative
distortion/rectification fields. R3.6A records those gaps instead of inferring
them.

The collector is disabled by default, fail-soft and observation-only. It
creates separate `physical_diagnostics_manifest.json` and
`physical_diagnostics.jsonl` files only when a diagnostics/dataset directory
is explicitly configured. It never overwrites original dataset files.

### Scope guards

- Raw-range equation and physical geometry: unchanged.
- M52 anchor selection and calibration input arrays: unchanged.
- Calibration fit/filter/cache/reseed decisions: unchanged.
- Runtime range output, residual-correction flag and controller: unchanged.
- Training, relabeling, simulation, shadow and final holdout: not run.
- R4/R5: not started.

### Files and artifacts

```text
range_physical_diagnostics.py
m52_adapter.py
depth_worker.py
metric_target_fusion.py
tracking_web.py
test_range_physical_diagnostics.py
docs/RANGE_V2_R3_6A_INSTRUMENTATION_REPORT.md
artifacts/range_v2/r3_6a_instrumentation/instrumentation_contract.json
artifacts/range_v2/r3_6a_instrumentation/r3_6a_manifest.json
```

### Tests and checks

```text
python3 -m py_compile range_physical_diagnostics.py m52_adapter.py \
  depth_worker.py metric_target_fusion.py tracking_web.py \
  test_range_physical_diagnostics.py
Result: PASS

Focused instrumentation/fusion/workflow/resolution tests
Result: 59 passed

PYTHONPATH=/home/sup/swarm_dashboard/.venv/lib/python3.12/site-packages \
  python3 -m pytest -q test_*.py
Result: 303 passed, 1 known PytestReturnNotNoneWarning

./run_all.sh --check
Result: Runtime check passed
```

The non-interference test asserts exact equality of raw/target range,
calibrator parameters, bearing-range filter state and EKF state between
diagnostics-disabled and diagnostics-enabled executions.

### Go/No-Go

- R3.6A instrumentation: **complete, awaiting review**.
- Quantity-contract/calibration fix: **not started / NO-GO without approval**.
- Training/promotion/R4/R5/simulation/shadow/controller integration:
  **not started / NO-GO**.

Work stops after R3.6A.

## Patch R3.6A-V — Controlled instrumentation validation

### Outcome

```text
DIAGNOSTIC_EVIDENCE_INSUFFICIENT
```

R3.6A-V ran an explicitly approved, static observation-only simulator capture
using the opt-in R3.6A sidecar. It created 11 independent
`physical_diagnostic_development` measurement groups with 389 raw-range frames
and 37,344 anchor records. It did not modify the production raw-range formula,
anchor selection, calibration/filter/cache/reseed behavior, runtime output,
controller or PX4.

Both UAVs were observed disarmed before and after every accepted scenario.
Follow, OFFBOARD, vehicle-motion, arm/takeoff/mode and controller endpoints
were not called. Residual correction remained off. Direct simulator poses and
zero-gravity static setup were applied only outside measurement windows and
are recorded in the isolated session manifest.

### Diagnostic result

- Exact current-live reconstruction passed for all selected rows; maximum
  error was `3.552713678800501e-15 m`.
- Full-run, same-config anchor refit replay matched logged raw fit; maximum
  raw-`a` delta was `6.505213034913027e-19`.
- Stable-window freeze reduced standard deviation in 10/11 groups but improved
  MAE in only 6/11. It does not demonstrate a bias/quantity fix.
- Nine of 11 groups had zero accepted-anchor membership churn despite
  nonzero range and `a/b` variation.
- The camera-link versus drone-center range contribution was at most
  `0.028684 m`, insufficient to explain multi-metre errors by itself.
- Raw foreground-surface and GT target-model-center semantics differ, but the
  surface offset and verified optical-center transform remain unavailable.
- 133/389 frames have `consume_now` recorded before depth completion.
- All 389 frames lack one or more mandatory fields; completeness is 0/389.
- One source sidecar has one malformed unselected tail line following disk
  exhaustion. It remains quarantined as an integrity failure; selected rows
  retain valid per-record checksums.

These limitations force the allowed conclusion
`DIAGNOSTIC_EVIDENCE_INSUFFICIENT`. Correlations are not reported as causal
fixes.

### Files and artifacts

```text
range_v2_r3_6a_capture.py
range_v2_r3_6a_validation.py
test_range_v2_r3_6a_validation.py
docs/RANGE_V2_R3_6A_VALIDATION_REPORT.md
docs/RANGE_V2_IMPLEMENTATION_REPORT.md
artifacts/range_v2/r3_6a_validation/collection_plan.json
artifacts/range_v2/r3_6a_validation/session_manifest.json
artifacts/range_v2/r3_6a_validation/sidecar_integrity_report.json
artifacts/range_v2/r3_6a_validation/diagnostic_frames.jsonl
artifacts/range_v2/r3_6a_validation/anchor_records.jsonl
artifacts/range_v2/r3_6a_validation/per_session_metrics.csv
artifacts/range_v2/r3_6a_validation/calibration_event_metrics.csv
artifacts/range_v2/r3_6a_validation/anchor_stability_metrics.csv
artifacts/range_v2/r3_6a_validation/quantity_center_comparison.csv
artifacts/range_v2/r3_6a_validation/hypothesis_matrix.csv
artifacts/range_v2/r3_6a_validation/missing_or_incomplete_fields.json
artifacts/range_v2/r3_6a_validation/r3_6a_validation_manifest.json
artifacts/range_v2/r3_6a_validation/r3_6a_validation_report.md
```

The retained runtime-log archives are checksummed in the validation manifest.
Original large runtime-log directories were removed only after archive
read-test and source sidecar checksum verification.

### Verification

Preflight:

```text
Focused R3.6A tests: 59 passed
Repository tests: 303 passed, 1 known PytestReturnNotNoneWarning
./run_all.sh --check: Runtime check passed
Instrumentation on/off non-interference: PASS
```

Final R3.6A-V regression results and exact output checksums are recorded in
`r3_6a_validation_manifest.json`.

### Go/No-Go

- R3.6A-V: **complete, evidence insufficient for a physical fix decision**.
- R3.6B/R4/R5: **not started / NO-GO pending explicit approval**.
- Training, scaler/calibrator ML, thresholds, shadow and controller effects:
  **not started**.

Work stops after R3.6A-V.
## 2026-08-04 — Scope pivot: Core Range Optimization 3–12 m

The broad near/core/far Range V2 roadmap is deferred at the user's direction.
The active objective is now stable metric range only in 3–12 m. Minimal logging
repair and one observation-only smoke completed with `CORE_LOGGING_READY`.

No Range V2 model was trained, promoted or integrated. Old v15/v16/v17 and
diagnostic v18 remain NO-GO; residual correction remains runtime-default off.
See `docs/CORE_RANGE_LOGGING_IMPLEMENTATION_REPORT.md` and
`docs/CORE_RANGE_OPTIMIZATION_3_12M.md`.

## 2026-08-04 — Core Range priority static collection

The reduced 3–12 m branch collected and audited nine independent static
development groups in the priority 6–7 m, 10–11 m and 11–12 m bins. Each bin
now has three accepted groups and 91–92 frames. The aggregate audit conclusion
is `PRIORITY_STATIC_COVERAGE_COLLECTED`.

This does not authorize training. Dynamic approach/recede sequences and
balanced independent-session coverage across the remaining 3–10 m bins are
still missing, so the training decision is
`NO_GO_DYNAMIC_AND_FULL_BIN_BALANCE_MISSING`.

Raw equal-group MAE is 0.906 m at 6–7 m, 5.377 m at 10–11 m and 8.772 m at
11–12 m. The physical estimate remains unusable as-is in the far-core bins.
No estimator/runtime/controller/PX4 code was changed, residual correction
remained off and no model/scaler/calibrator/threshold was fit.

Verification: 326 repository tests passed with one known warning;
`./run_all.sh --check` passed; no old PX4/Gazebo/backend/train process remains.
See `docs/CORE_RANGE_PRIORITY_COLLECTION_REPORT.md` and
`artifacts/core_range_3_12m/priority_collection/audit/`.

## 2026-08-04 — Core Range static balance collection

The reduced core-range branch completed the six-bin static balance batch with
`STATIC_3_12M_COVERAGE_READY_FOR_MODEL_BENCHMARK`: 18 accepted independent
groups, 558 raw frames and three groups in every requested bin from 3–4 through
9–10 m. Combined with the prior priority batch, all static 1 m bins from 3 m to
12 m now meet the minimum three-session gate.

All accepted frames passed timestamp, GT trace/checksum, record checksum,
96-anchor, exact-bin and unique-trace checks. Runtime logs were archived and
read-tested before raw copies were removed. Three technical failures remain
quarantined and excluded. No model/split/scaler/threshold was created, residual
correction remains off, and runtime/controller/PX4 were unchanged.

Verification: 334 repository tests passed with one known warning and
`./run_all.sh --check` passed. The next possible action is an explicitly
approved offline raw-versus-residual-versus-direct benchmark using
group-disjoint splits and seed 52.

## 2026-08-04 — Core Range XGBoost offline benchmark

The reduced branch completed the explicitly approved static-development
benchmark with `DIRECT_XGBOOST_BEST_DEVELOPMENT_CANDIDATE`. Inputs were 832
checksummed frames from 27 group-disjoint sessions across all nine 1 m bins.
The plan, features, folds, seed 52, three XGBoost configurations and accuracy
gates were checksummed before fitting.

Direct `C_shallow` passed the frozen development gates with 0.427 m
equal-group MAE, 0.446 m equal-group P90, 1.399 m worst-bin MAE and no errors
over 3 m. Residual `B_balanced` obtained 1.294 m MAE and failed the MAE,
worst-bin and catastrophic-error gates. The legacy residual clamp remained a
diagnostic baseline only and performed poorly.

The overfitting audit records a material limitation: bbox width is the top
gain feature in every fold and global held-out permutation raises MAE by about
2.30 m. Output is piecewise constant in many static groups, and dynamic
response is still N/A. Therefore this is neither a model promotion nor runtime
authorization. Runtime/controller/PX4 were unchanged, residual correction
remains default-off, and replay/shadow/final holdout were not run.

Verification: 14 focused benchmark tests and 348 full repository tests passed
with the one known warning; `./run_all.sh --check` passed and no stale
PX4/Gazebo/backend/train process remained. Work stopped after the offline
benchmark.

## 2026-08-04 — Core Range direct dynamic replay gate

The explicitly approved offline dynamic gate stopped with
`DYNAMIC_REPLAY_BLOCKED_BY_DATA_INTEGRITY`. Candidate checksum, feature
contract, preprocessing, `[3,12]` clipping, causal smoothing, seed 52 and gates
were frozen; no retraining was performed.

Collection accepted 4/8 independent sessions and 446 frames (3/3 approaching,
1/3 receding, 0/2 stop-and-hold). A near-start lateral receding setup repeatedly
produced zero accepted raw-range rows because inverse-depth uncertainty was too
large, including after one precommitted collection-only ROI correction. The
failed roots remain quarantined. Candidate replay was not run on the partial
corpus, so accuracy, lag, stop and bbox robustness metrics are N/A.

No runtime/model-output integration, shadow, Follow, OFFBOARD, arm, takeoff or
controller action occurred. Residual correction remains default-off. Work
stops at this gate pending a separate data-collection repair decision.

Verification: 16 focused dynamic-gate tests and 364 full repository tests
passed with one known warning; `./run_all.sh --check` passed. The blocked
manifest records zero prediction rows and revalidates every accepted source and
runtime archive checksum.

## 2026-08-05 — Core Range dynamic ROI recovery and replay

The blocked gate above was repaired and replayed:
`DIRECT_CANDIDATE_FAILS_BBOX_ROBUSTNESS`. Root cause was the MAD-gated
foreground-half depth statistic, not the ROI/bbox itself; an opt-in,
default-off, per-geometry recovery statistic
(`SWARM_TARGET_ROI_RECOVERY_POLICY=central_quantile_region` in
`target_depth_extractor.py`) was added and applied only to the one geometry
whose own deterministic, non-GT evidence justified it — the same policy was
confirmed to actively hurt two other near-start-lateral geometries, which
kept the unchanged default statistic instead.

All 8/8 dynamic groups (844 frames) reached integrity PASS. The frozen direct
`C_shallow` candidate, unchanged, was replayed on the complete corpus: it
beats raw physical range on 6/8 groups and both stop-and-hold sessions
individually clear the accuracy gate, but equal-group MAE (1.98 m vs 1.0 m
gate), P90, median lag and a ±10% bbox-width catastrophic-error check do not
meet the precommitted development gates.

No retraining, feature, gate or candidate-checksum change; no runtime/
controller/PX4/shadow/Follow-Target action; residual correction remains
default-off. See `docs/CORE_RANGE_DYNAMIC_REPAIR_AND_REPLAY_REPORT.md`.

Verification: 373 full repository tests passed with the one known warning;
`./run_all.sh --check` passed; candidate checksum and all 8 accepted-session
integrity records reconfirmed at gate close.

## 2026-08-05 — Core Range depth scheduler contention fix

Temporal audit found capture→consume median ≈1.13s on the frozen corpus vs
5-14ms for MiDaS standalone. Staged contention isolation (configs A-F)
pointed at `ROS_CALLBACK_OR_THREAD_BLOCKING_DOMINANT`: full-stack worker
inference averaged ~779ms at the 7.5Hz production depth-submit rate vs
~25ms at 2.0Hz in an isolated smoke test. The existing `LatestDepthWorker`/
`depth_rate_hz` scheduler already met the required decoupling/single-slot/
non-blocking properties, so no new scheduler code was written; a sweep of
2/3/4/5/7.5Hz found only 2.0Hz met the ≤200ms median/≤300ms P95 gate
(42.8ms), and the production default was changed from 7.5 to 2.0Hz on that
basis. See `artifacts/core_range_3_12m/depth_scheduler_contention_fix/`.

## 2026-08-05 — Core Range 2Hz dynamic recollection and retrain (blocked)

Recollecting the 8-group dynamic corpus at the new 2.0Hz default reached
7/8 groups (receding 2/3 — `cdr_recede_left_yaw_2hz` failed 5 attempts
against the unmodified precommitted spec; this geometry also needed repeated
recovery in the original 7.5Hz-era corpus, so it predates the 2.0Hz change).
More significantly, the latency gate did not hold under a realistic
two-vehicle, 32s-trajectory collection: capture→consume median came back at
1.226s (P95 1.849s), essentially unchanged from the old 7.5Hz-era corpus's
1.131s median. Root cause: the prior task's 42.8ms figure came from a short
isolated smoke test whose depth pipeline never once produced a successful
raw range measurement, which did not generalize to this realistic
multi-group collection — MiDaS worker inference is again taking 0.5-1.3s
per call. Per task scope the depth rate stayed at 2.0Hz, no gate was
lowered, and training was not started. **Conclusion:
`TWO_HZ_DYNAMIC_CORPUS_INCOMPLETE`.** See
`docs/CORE_RANGE_2HZ_DYNAMIC_RETRAIN_REPORT.md` and
`artifacts/core_range_3_12m/dynamic_2hz_retrain/`.

Verification: 380 full repository tests passed; `./run_all.sh --check`
passed; no PX4/Gazebo/backend processes remained running at close.
## 2026-08-05 representative full-stack contention fix

Representative profiling invalidated the old claim that 2 Hz alone fixed live
contention.  `simulation_target_ground_truth()` copied and scanned the full
600-entry pose history on every tracking frame, monopolizing the backend GIL.
The minimal bracket-only copy preserves all synchronization and range semantics
while changing tracking 5.7→18.05 FPS, capture-to-consume 1395.2→70.6 ms median,
MiDaS 748.3→35.5 ms median, and raw range 50→51.  The new post-fix default is
5 Hz based on the corrected representative 2/3/4/5/7.5 Hz sweep; the 7.5 Hz
session was INVALID at stable-calibration prewarm.  Full evidence is in
`docs/CORE_RANGE_FULL_STACK_CONTENTION_FIX_REPORT.md`.
## 2026-08-05 5 Hz dynamic recollection gate

`CORE_RANGE_5HZ_DYNAMIC_RECOLLECTION_AND_RETRAIN` stopped at the data gate.
The frozen 5 Hz collection plan ran 16 observation-only attempts, all preserved
in quarantine. Per-frame integrity passed across 1,888 raw rows and depth
latency remained corrected (capture P95 <=215 ms), but every attempt had
tracking median below 20 FPS (15.59--17.65); six also missed MiDaS P95 100 ms.
Accepted coverage is 0/8, so neither PHYSICAL_ONLY nor PHYSICAL_TEMPORAL was
trained and no candidate exists. Production integration was not attempted.
## 2026-08-05 collection-throughput isolation

Paired A–H isolation found no dominant dataset-collection component. Current
smoke and full collection were both source-limited at 16–18 FPS; removing
diagnostic serialization/file I/O, overlay, and frequent fsync improved at most
7.4%. In-process P95 timings were 3.60 ms record/checksum construction, 1.50 ms
JSON, 0.23 ms write and 4.92 ms fsync, with full tracking-loop P95 35.17 ms.
Only opt-in bounded-memory ablation and monotonic timing instrumentation were
added; production disk behavior and data schema remain unchanged. Conclusion:
`COLLECTION_THROUGHPUT_EVIDENCE_INSUFFICIENT`.
## 2026-08-05 camera-source FPS audit and fix

New hop-level tracing (`camera_source_trace.py`, opt-in via
`SWARM_CAMERA_TRACE_JSONL`) instrumented five points from the Gazebo camera
sensor's own sim-time publish schedule through to tracker output. The
sensor publishes a rock-solid 50 Hz in sim time in every run — never the
bottleneck. The loss to wall-clock camera-source FPS tracks Gazebo's
real-time factor directly: RTF measured a median of only 0.20–0.32 (P5 as
low as 0.03) against a configured target of 1.0 while the Gazebo GUI
client process ran; disabling it (`SWARM_START_GAZEBO_GUI=false`, new
additive opt-in `run_all.sh` env-gate, default unchanged) raised RTF to
~1.0 and camera-source FPS from ~29 to ~45 FPS. GPU/MiDaS contention (via
a depth-rate-throttled ablation), the dashboard client, a ROS2 image
bridge (confirmed not to exist in this architecture — camera ingestion is
direct `gz.transport13`, not ROS2), and static config/code drift were all
ruled out with direct evidence. Three post-fix validation scenarios
(approaching/receding/stop-and-hold, two UAVs, ≥30 s, ≥40
`raw_range_computed`, 5 Hz depth rate) pass camera-source/tracking FPS,
capture→consume, and MiDaS-P95 gates with ~1.8× margin; a downstream
tracker-lock-duty-cycle metric does not pass and is flagged as a separate,
out-of-scope follow-up rather than masked. Conclusion:
`CAMERA_SOURCE_FPS_FIXED_READY_FOR_RECOLLECTION`. See
`docs/CORE_RANGE_CAMERA_SOURCE_FPS_REPORT.md` and
`artifacts/core_range_3_12m/camera_source_fps/`.
## 2026-08-05 headless 5 Hz dynamic recollection and retrain (incomplete)

CORE_RANGE_5HZ_HEADLESS_DYNAMIC_RECOLLECTION_AND_RETRAIN recollected the
dynamic corpus at SWARM_START_GAZEBO_GUI=false (the fix validated by the
camera-source-FPS task). Camera-source FPS proved unconditionally fixed:
across 13 real capture attempts, camera-source median FPS never once fell
below the 25 FPS gate (36.15-47.60 range). A second, previously-unmeasured
instability surfaced instead: Gazebo real-time factor, sampled from
/world/default/stats and cross-checked with a continuous-burst read to
rule out a measurement artifact, showed a genuine, headless-independent
bimodal stutter (~60% of samples near RTF 1.0, the rest near 0.03-0.1,
throughout each run) that intermittently drops the per-session RTF median
below the 0.8 gate. 6 of 13 attempts (2 per planned session, matching the
precommitted maximum_attempts_per_session=2) failed on RTF, clustering in
the second half of the run and mostly on receding-direction sessions.
Collection reached 5/8 accepted groups (approaching 3/3, receding 1/3,
stop_and_hold 1/2) against the required 8/8, so per task instructions
training was not started: the static corpus was not combined with the
incomplete dynamic corpus, no folds were fit, no XGBoost model was
created. A training driver (core_range_5hz_headless_retrain.py) reusing
core_range_dynamic_robust_retrain.py's engine as pure functions was built
and unit-tested in advance so a future completed recollection can proceed
directly to training. The recede-left-yaw central_quantile_region ROI
recovery remained scoped to that one session only and was not the failure
cause (it produced raw-range measurements as designed; both attempts
failed on the RTF/tracking runtime gate instead). Conclusion:
`FIVE_HZ_HEADLESS_DYNAMIC_CORPUS_INCOMPLETE`. See
docs/CORE_RANGE_5HZ_HEADLESS_DYNAMIC_RETRAIN_REPORT.md and
artifacts/core_range_3_12m/dynamic_5hz_headless_retrain/.
## 2026-08-05 Gazebo RTF stutter root-cause and gate validation

CORE_RANGE_GAZEBO_RTF_STUTTER_AND_GATE_VALIDATION audited every clock
domain in the range pipeline (trajectory scheduling, pose/camera/tracker/
raw-range timestamps, temporal features, capture-latency stages, GT
alignment, lag evaluation) and found no sim-time/wall-clock mixing bug --
GT is matched to camera frames entirely in Gazebo sim time, and everything
else is self-consistently python_monotonic (schema-enforced for the
capture-latency stages). A 28-minute persistent single-stack headless long
run (never restarted, 13 repeated approaching/receding reps, 1780
one-second resource/RTF samples) found real, substantial-magnitude
evidence that low RTF is not merely a slowdown: raw-range accuracy against
ground truth degrades 63% in low-RTF windows (median absolute error 1.27m
at RTF>=0.8 vs 2.08m at RTF<=0.3, n=225/298 real rows each), while
checksum/timestamp/anchor integrity stayed 100% pass in both regimes. 5 of
13 long-run reps also failed outright on gz set_pose service-call timeouts
during severe stutter. No single resource factor (CPU frequency, thermal,
memory, disk I/O, context switches, GPU) correlates strongly with RTF
(all |r|<0.22), so no minimal root-cause fix was identified, and
restart-per-session is not adopted as a mitigation since the prior
5Hz-headless task's 13 attempts were already full-stack restarts every
session and still showed the same stutter pattern. Because the task's
precommitted condition for downgrading RTF to diagnostic-only (raw
accuracy must not degrade in low-RTF windows) failed, RTF remains a hard
gate; the gate was not lowered to force a collection pass. Fresh
approaching/receding/stop-and-hold validation confirms every other
functional gate (camera FPS, tracking FPS, latency, MiDaS P95, integrity,
backlog) remains solid. Conclusion:
`RTF_REMAINS_HARD_GATE_STUTTER_UNRESOLVED`. See
docs/CORE_RANGE_GAZEBO_RTF_STUTTER_REPORT.md and
artifacts/core_range_3_12m/gazebo_rtf_stutter/.

## 2026-08-05 headless missing-group continuation

The prepared 5 Hz headless collector was extended with a resume-only path
that preserves and verifies accepted source hashes, skips accepted groups,
uses a bounded maximum of 8 total attempts per missing group, and records
pre-capture plus collection-window RTF evidence separately. The collection
window gate now explicitly rejects a run containing at least three
consecutive one-second samples at RTF<=0.3 in addition to median RTF<0.8.
This is orchestration/data-gate logic only; no M52, MiDaS, calibration,
controller, PX4, or simulator-performance change was made.

The three missing groups exhausted their retry budgets with zero new
acceptance (16/18 continuation attempts failed RTF/long-stutter). The five
accepted groups remain checksum-identical and integrity PASS, leaving the
corpus at 5/8 groups / 570 frames. Training and model freeze were correctly
skipped. Conclusion: `DATA_COLLECTION_BLOCKED_BY_PERSISTENT_RTF_STUTTER`.

## 2026-08-06 sim-time driver Phase 1 validation

A deterministic sim-time trajectory driver (Gazebo `ISystemPreUpdate`
plugin, embedded in a per-run overlay world SDF; see
docs/CORE_RANGE_OPTIMIZATION_3_12M.md's 2026-08-05/06 entry for the
plugin-loading root cause and fix) was formally validated against
production bbox presets: pose determinism between independent runs is
exact (0.0 m diff at matching elapsed sim-time), a real publish-race driver
bug was found and fixed, and RTF-window raw-error evidence reinforced the
standing RTF hard gate. But 3 of 4 validation runs hit a rare (~0.5-0.6%
per frame) `main.py` Gazebo pose-history bracket race not present in the
historical legacy-driver corpus, failing the GT-alignment gate, which has
no insufficient-evidence exception. Missing-group collection and training
were not started. Conclusion: `GROUND_TRUTH_ALIGNMENT_FAILED`. See
docs/CORE_RANGE_SIM_TIME_DYNAMIC_RETRAIN_REPORT.md.

## 2026-08-06 overnight collection + training: corpus complete, no candidate meets gate

The GT-bracket race fix (above) unblocked collection. An overnight
session collected the remaining 3 dynamic groups (8/8 total: approaching
3, receding 3, stop_and_hold 2) in 4 attempts, froze a 35-group/2081-frame
combined static+dynamic corpus, and trained PHYSICAL_ONLY/
PHYSICAL_TEMPORAL (3 configs each). All 6 configs failed the static
(≤1.0m), dynamic (≤1.0m), temporal (≤0.30s lag), and bbox-robustness (0%
catastrophic @±10%) gates simultaneously -- best dynamic MAE was 1.25m.
No model was frozen. Conclusion:
`NO_SIM_TIME_DYNAMIC_ROBUST_MODEL_MEETS_GATE`. See
docs/CORE_RANGE_OVERNIGHT_SIM_TIME_TRAIN_REPORT.md.

## 2026-08-06 offline failure analysis: root cause found, 3 targeted candidates, still no gate pass

A fully offline follow-up (bit-exact baseline reproduction verified first)
traced the four gate failures to: (1) static error concentrated in the
11-12m regime and a few outlier groups, not diffuse underfitting; (2)
moderate, systematic static/dynamic distribution shift in depth/
calibration/ROI features; (3) the bbox-robustness gate not actually
exercising any feature PHYSICAL_ONLY/PHYSICAL_TEMPORAL use (verified by
reading `perturb_bbox_features()` and confirming 0.0 prediction shift
empirically); (4) the temporal lag gate saturating at its search boundary
even for the raw, unprocessed physical-range signal. Feature ablation and
nearest-neighbor analysis ruled out feature-information insufficiency.
Three precommitted candidates targeting the regime-bias/outlier-leverage
hypotheses were trained and evaluated with the same group-disjoint
protocol: 2 failed to train (degenerate constant-output collapse under a
pseudo-Huber objective), the third improved static MAE by a noise-level
0.013m. No candidate frozen. Conclusion:
`TARGETED_MODEL_IMPROVES_BUT_STILL_FAILS`. See
docs/CORE_RANGE_OFFLINE_FAILURE_ANALYSIS_REPORT.md.

## 2026-08-06 stable relative tracking: SNR limit found via alpha-beta filter grid sweep

Changed optimization target to tracking stability (direction agreement,
Spearman rank correlation, stationary jitter, zero catastrophic jumps)
and built a Direct XGBoost -> calibration -> causal alpha-beta filter
pipeline. Audit found raw_physical_range_m's correlation with GT distance
flips sign between center and lateral/yaw-varying dynamic groups
(explaining the prior task's ineffective monotonic constraint), while
ray_scale showed a consistent sign specifically in the dynamic regime.
Swept the full precommitted calibration x alpha-beta grid (171
combinations, no collapse in any candidate): 0 pass the practical gate.
True frame-to-frame GT motion (median 0.035m at 5Hz/0.25 m/s) is
dominated by the raw model's own frame-to-frame noise (median 0.148m),
capping achievable direction agreement at ~0.57 against a 0.90 target
regardless of calibration or in-grid filter smoothing. No candidate
frozen; observation-only integration and live smoke correctly not
attempted (gated on a passing candidate). Conclusion:
`STABLE_RELATIVE_RANGE_STILL_UNSTABLE`. See
docs/CORE_RANGE_STABLE_RELATIVE_TRACKING_REPORT.md.

## 2026-08-06 control-ready observation pipeline: windowed direction agreement still near chance

Built a practical, control-oriented pipeline on top of the unchanged,
bit-exact-reproduced Candidate B model: `raw prediction -> causal
alpha-beta estimator (alpha=0.65/beta=0.03/threshold=2.0m, unchanged) ->
measurement-age compensation -> predicted_current_range_m`, plus an
independent monitor-only causal windowed trend estimator (0.6/0.8/1.0s,
Theil-Sen slope, entry/exit hysteresis). Replayed all 8/8 dynamic groups
offline. Global MAE (1.158m), worst-bin MAE (1.885m), and catastrophic
jump count (0) all passed their gates; but windowed direction agreement
(scored over 0.6-1.0s windows instead of consecutive frames, motion
threshold 0.10m) only reached 0.516 overall at the best window -- barely
above chance against a 0.80 target -- and stop-and-hold stationary std
(0.588m) narrowly missed 0.55m. Direct inspection of a monotonically-
approaching group showed the cause: Candidate B sustains ~1.5-3.5m
regional bias over dozens of consecutive frames in parts of some
trajectories, which a windowed slope estimator cannot distinguish from
genuine motion at this scale -- extending, not contradicting, the prior
task's per-frame SNR finding. Age compensation itself was found to be
mildly net-negative (extrapolating with a noisy rate estimate added
variance faster than it removed staleness bias) though within its own
gate. No candidate frozen for runtime use; Phase 3 (observation-only
integration) and Phase 4 (live Gazebo smoke) were correctly not attempted
per the task's own PASS-gated rule -- no Gazebo/PX4/ROS2/backend process
was started. Conclusion: `OFFLINE_CONTROL_READY_RANGE_FAILED`. See
docs/CORE_RANGE_CONTROL_READY_OBSERVATION_REPORT.md.

## 2026-08-06 residual bias correction: corrector does not generalize past a 35-group corpus

Implemented Giai đoạn 1 of a 4-stage follow-up roadmap: audit Candidate
B's residual against distance/geometry context, then fit a small residual
corrector on the unchanged Candidate B backbone (candidate_b_pred_m +
runtime features -> predicted residual -> corrected_range), rather than
continuing to tune the downstream alpha-beta filter/trend window (already
shown exhausted in the prior task). The bias audit found real, large
persistent bias (up to 3.13m in one cell) and identified 4 features
(ray_scale, target_inverse_depth, target_roi_q_min/p10) with a
geometry-context-consistent correlation sign, out of 17 audited. Trained 3
precommitted configs (Ridge, XGBoost strong-reg, XGBoost on the 4 stable
features) with group-disjoint 3-fold CV over the full 35-group corpus.
None collapsed, but none generalized: held-out Spearman correlation
between predicted and true residual was 0.20/-0.30/-0.14 (weak to
negative), while the same model fit training-fold data with R^2=0.53 --
confirming small-corpus overfitting to group-specific residual structure,
not a bug. All 3 failed the Giai đoạn 1 gate (best bias reduction 20.2%
vs. a 50% target; best direction agreement 0.533 vs. 0.80, essentially
unchanged from the raw baseline). Per the roadmap's own diagnosis, this
result is the trigger condition for a small, targeted follow-up data
collection (not attempted here -- deferred to the user) rather than
further corrector tuning. Conclusion:
`RESIDUAL_BIAS_CORRECTOR_GATE_FAILED`. See
docs/CORE_RANGE_RESIDUAL_BIAS_CORRECTION_REPORT.md.

## 2026-08-06 targeted dynamic pilot Stage 2A: blocked before collection by disk space

Attempted to collect 14 targeted, independent dynamic groups (Stage 2A of
the bias-correction roadmap) to test whether Candidate B's persistent
regional bias reproduces outside the original 8-group corpus. Phase 0
preflight passed Candidate B reproduction (bit-exact) and runtime source
verification (all 10 required fixes/files present), but failed the
disk-space check: 13.54GB available on the workspace filesystem vs. the
task's own 15GB minimum, with `artifacts/` (29GB of 35GB) being the
accumulated historical record of every prior task. No historical artifact
was deleted and no gate was lowered to force a pass; no Gazebo/PX4/backend
process was started. Conclusion: `PILOT_BLOCKED_BY_STORAGE`. See
docs/CORE_RANGE_TARGETED_DYNAMIC_PILOT_REPORT.md.
