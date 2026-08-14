# Range Estimation V2 Near/Core/Far Compatibility Report

Date: 2026-08-04  
Patch: R1 — audit only  
Scope: repository, design documents, historical model bundles, dataset manifests,
and read-only dataset statistics  
Decision: **GO WITH CONDITIONS for R2; NO-GO for model training, promotion, or
runtime integration**

## 1. Outcome

The repository can support Range Estimation V2 without replacing the existing
MiDaS, M52 anchor, physical range, dataset integrity, grouped split, XGBoost, or
shadow-inference foundations. The old two-stage objective cannot be promoted as
Range V2: its classifier predicts whether a residual is within +/-3 m, not
validity or an ordered near/core/far zone, and its scalar held-out residual RMSE
is not a per-sample q90 uncertainty estimate.

R2 may add an offline derived label/audit layer without modifying original
datasets. Training is blocked because the audited development corpus contains
only static core samples, has no near/far/invalid examples, has no dynamic
direction sequences, and has no samples in the 10–11 m or 11–12 m core bins.

No PX4, Gazebo, backend, or range-training process was running during this
audit. Residual correction remains default-off. No simulation was started and
no runtime source was changed.

## 2. Audit basis

The audit used CodeGraph before targeted source inspection. The principal
sources were:

- `metric_target_fusion.py`, `metric_depth_calibrator.py`,
  `target_depth_extractor.py`, `depth_worker.py`, `m52_adapter.py`;
- `range_ground_truth.py`, `pose_time_sync.py`,
  `range_residual_dataset.py`;
- `range_applicability_gate.py`, `range_residual_correction.py`,
  `range_residual_training.py`;
- `main.py`, `tracking_web.py`, `visual_follow_target.py`,
  `bearing_range_filter.py`, `target_fusion_ekf.py`;
- current implementation/design reports, v15–v17 manifests/evaluations, and
  the diagnostic-v18 failure artifact;
- all retained v2 dataset manifests and read-only statistics from the 32
  development `samples.jsonl` files used in the v18 audit.

The workspace contains a `.git` directory without `HEAD` or `config`; it is not
a Git repository. Therefore R1 cannot be committed here. File SHA-256 digests
are used for the R1 handoff, and a real repository/commit identity remains a
condition before any candidate bundle is frozen.

## 3. Current end-to-end architecture

```text
Gazebo RGB callback
  -> BGR frame + monotonic receipt timestamp + Gazebo simulation timestamp
  -> TrackingManager tracker/bbox
  -> frame-time synchronized vehicle and camera pose
  -> LatestDepthWorker (latest frame wins)
  -> MiDaS inverse depth
  -> M52GroundAnchorAdapter ground anchors
  -> MetricDepthCalibrator affine inverse-depth calibration
  -> TargetDepthExtractor eroded-bbox robust target depth
  -> inverse-depth temporal filter
  -> physical slant ray range
  -> deterministic applicability gate
  -> optional applicability XGBoost + residual XGBoost
  -> range temporal filter
  -> target EKF / TargetEstimate
  -> TrackingManager target selection/handover
  -> optional visual target publication/control path
```

Dataset collection disables body-yaw, follow, motion, attitude, bootstrap, and
visual-target callbacks. Shadow residual inference computes and logs a candidate
but returns the physical range to the filter/EKF.

## 4. Required audit answers

### 4.1 Where is raw physical range computed?

`MetricTargetFusion._consume_depth_result()` in `metric_target_fusion.py` is the
authoritative path. It converts the temporally filtered relative inverse depth
through the M52-fitted affine calibration to optical metric depth and multiplies
by the camera-ray scale:

```text
ray_scale = sqrt(1 + image_ray_x^2 + image_ray_y^2)
filtered_optical_depth = calibrator.metric_depth(filtered_inverse_depth)
physics_ray_range = filtered_optical_depth * ray_scale
```

The raw ROI median metric depth before the inverse-depth temporal filter is also
recorded as `raw_ray_range`, but the residual model baseline is the filtered
`physics_ray_range`. This occurs before `RobustBearingRangeFilter.update_range`
and before `TargetFusionEKF.update_range/update_position`.

### 4.2 Where are MiDaS features created?

- `depth_model_adapter.py`: model loading/inference contract.
- `depth_worker.py`: asynchronous `DepthJob`/`DepthResult`, preserving frame
  index and measurement timestamp; latest pending frame replaces older work.
- `target_depth_extractor.py`: eroded-bbox ROI, finite positive inverse-depth
  selection, foreground-half median and MAD rejection.
- `metric_target_fusion.py`: combines ROI statistics with M52 anchors,
  calibration, bbox, ray geometry, previous physical range, and delta time.

The current model receives 23 scalar `float64` features. It does not receive
images, absolute timestamps, session IDs, ground truth, or control commands.

### 4.3 Where is the current feature schema?

`FEATURE_SCHEMA_VERSION`, `FEATURE_NAMES`, `FEATURE_TYPES`,
`MISSING_VALUE_POLICY`, `feature_schema()`, and `RangeResidualFeatures` are in
`range_residual_correction.py`.

Current version: `m52_midas_range_features_v2`.  
Current missing policy: `reject_sample`.  
Current feature count: 23.

The schema is repeated and checked in dataset and bundle manifests. Runtime
rejects feature count/order/schema mismatches.

### 4.4 Where are correctable/uncorrectable labels created?

The stored dataset label is the continuous residual:

```text
range_residual_m = ground_truth_distance_m - physics_distance_m
```

`range_residual_training.py` derives the old binary label in
`applicability_stratified_grouped_split()` and
`train_range_residual_bundle()`:

```text
correctable = abs(range_residual_m) <= maximum_absolute_residual_m
```

The locked old limit is 3.0 m. This derived label is not reused as the Range V2
validity or zone label.

### 4.5 Where are the v15–v18 train scripts?

There are no separate version-specific train programs. v15–v17 were invocations
of `range_residual_training.py train` with different development datasets while
retaining seed 52, threshold 0.8, residual limit 3.0 m, and grouped policy.
Their bundles live under `artifacts/range_residual_xgboost_candidate_v*_v2_*`.

Diagnostic v18 used the same trainer/policy and failed closed at validation with
`all_partition_predictions_rejected`; no v18 bundle exists. The immutable
diagnostic record is `artifacts/range_v18_diagnostic_failure_20260804.json`.

### 4.6 Where are dataset manifest and group split implemented?

- Dataset schema/manifest/collector/loader:
  `range_residual_dataset.py`.
- Dataset group key: `run_id/session_id`; the current Python group key uses
  `run_id/group_id`, where the collector encodes the session into `group_id`.
- Duplicate identity check: `(run_id, group_id, frame_index)`.
- Deterministic grouped split and old applicability stratification:
  `range_residual_training.py`.
- Seed: 52.
- Current group weighting: equal total weight per run/session group.

The current dataset manifest does not store sample-file checksum inside the
manifest. The loader calculates `samples.jsonl` SHA-256 into training
provenance. Range V2 should retain both immutable manifest and sample checksums
in a separate audit/model manifest.

### 4.7 Is StandardScaler used?

Yes. `StandardScaler.fit_transform()` is called only on the training partition;
validation and test call `transform()`. Current boosters always consume scaled
features and manifests state `model_input = scaled`.

The scaler is additionally used for per-feature z-score and shrunk-covariance
Mahalanobis OOD checks. The existing trainer does not benchmark bounded raw
features against scaled features, so that comparison is required later.

### 4.8 What is in the current model bundle?

Each v2 residual bundle contains:

```text
manifest.json
scaler.joblib
applicability_model.json
model.json
evaluation.json
```

The manifest contains schema, model input, OOD statistics/limits, applicability
threshold, correction limit, residual validation RMSE, training provenance,
group split/weights, dependency versions, and artifact checksums.

The existing `residual_prediction_std_m` is one scalar validation RMSE. It is
not a runtime predicted q90 and must not be renamed or reported as q90.

### 4.9 Does temporal filtering exist?

Temporal filtering exists, but it is not Range V2 zone-aware:

- `MetricDepthCalibrator`: temporal median/EMA, change-point detection/reseed,
  stable-sample gate, cached calibration with growing covariance.
- `RobustBearingRangeFilter`: inverse-depth and metric-range Hampel gates,
  exponential smoothing, range-rate and acceleration limits.
- `TargetFusionEKF`: position/range fusion and covariance.
- `TargetStateFilter`: alpha-beta position/velocity filter for the target path.

Missing pieces are explicit semantic zone state, asymmetric hysteresis,
near/core/far transition states, `calibration_epoch`, `track_epoch`, causal far
direction state, reacquire/calibration hold output semantics, and a guarantee
that stale values are not new measurements.

### 4.10 Where does runtime read model output?

`MetricTargetFusion._consume_depth_result()` constructs
`RangeResidualFeatures`, calls `RangeResidualCorrector.correct()`, records the
shadow/dataset result, and then passes `correction.output_distance_m` to the
existing range filter/EKF when deterministic applicability permits it.

`RangeResidualCorrector.from_environment()` reads:

```text
SWARM_RANGE_RESIDUAL_MODE=off|shadow|active
SWARM_RANGE_RESIDUAL_BUNDLE=...
```

Default mode is `off`. In `shadow`, a candidate is computed/logged while
`output_distance_m` remains the physical range. `tracking_web.py` consumes the
resulting `TargetEstimate`, performs provisional-to-metric handover, and can
eventually reach the visual target command path. Range V2 must not be connected
to that consumer during R1–R9.

### 4.11 Which modules can be retained?

Retain the MiDaS adapter/worker, target ROI extraction, M52 anchor adapter,
metric calibration, camera projection, physical range/uncertainty propagation,
deterministic quality envelope, ground-truth providers, pose synchronization,
append-only dataset storage, checksums, group identity, seed 52, XGBoost loading,
and the default-off/shadow-mode pattern.

### 4.12 Which modules require refactoring or adapters?

- Adapt current features into a versioned Range V2 schema; preserve old schema.
- Separate `validity` from ordered `distance_zone`.
- Replace old applicability-label splitting with validity/zone-aware grouped
  partitions and train/calibration/development-test separation.
- Replace scalar residual RMSE with a cross-fitted/calibrated q90 model.
- Extend dataset metadata for epochs, dynamic ground truth, direction labels,
  location/background/pitch regimes, and event markers.
- Add raw-vs-scaled preprocessing benchmark and bounded group/bin weighting.
- Adapt shadow logging to `RangeEstimateV2` without controller effect.

### 4.13 Which modules must be new?

- Canonical Range V2 labels/contract.
- Data coverage audit and derived-label view.
- Raw baseline evaluator with sequence/event metrics.
- Flat and hierarchical validity/ordinal zone benchmarks.
- Core q50 residual and cross-fitted q90 error model.
- Causal far trend/direction model.
- Zone-aware temporal state machine.
- Deterministic Range V2 replay/evaluation and bundle tooling.

## 5. Ground-truth audit

### 5.1 Source and spatial frame

Two sources are supported:

1. `gazebo_camera_to_target_center`: Euclidean slant range from the Gazebo
   `camera_link` origin to the target model center. Poses originate in the
   Gazebo world frame (ENU coordinates), but Euclidean distance is frame-rotation
   invariant. The runtime bearing/target position is represented in PX4 NED.
2. `rtk_fixed_camera_to_target_center`: RTK-fixed WGS84 positions converted to
   ECEF. Camera and target antenna lever arms are transformed from body FRD to
   NED/ECEF before center-to-center distance is calculated.

The canonical Range V2 metric label is always camera optical-center to target
reference-center slant range. Drone-center range, ground-plane distance, bbox
scale, MiDaS relative score, and controller setpoint are different quantities
and may not be substituted.

### 5.2 Frame and timestamp identity

Each sample stores `run_id`, `group_id`, `session_id`, `frame_index`,
`measurement_timestamp_s`, and `source_sim_timestamp_s`.

- The camera callback records a monotonic receipt timestamp after conversion as
  the tracking frame timestamp and separately keeps the Gazebo message time.
- Vehicle and camera pose buffers use the monotonic domain and interpolate at
  the frame timestamp; interpolation gaps above 250 ms and nearest samples older
  than 120 ms are rejected.
- Simulation ground truth interpolates Gazebo poses at the source simulation
  timestamp when the bracket span is at most 200 ms, otherwise uses the nearest
  sample and rejects age above 100 ms.
- RTK ground truth compares dashboard monotonic telemetry receipt times to the
  measurement timestamp and rejects offsets above 100 ms.

The current dataset does not store an explicit capture-vs-callback latency or a
full uncertainty decomposition. Range V2 must preserve both timestamp domains
and record every budget component rather than treating receipt time as exact
capture time.

### 5.3 Current uncertainty budget

For RTK, the present label budget is:

```text
sqrt(camera_eph^2 + camera_epv^2
   + target_eph^2 + target_epv^2
   + lever_arm_uncertainty^2)
```

It must be <=0.20 m, and absolute time offset must be <=100 ms.

For simulation, stored uncertainty is currently 0.0 m and pose age is stored as
the time offset. That is acceptable for the existing static SITL samples, but it
is insufficient for dynamic Range V2 labels because pose interpolation/age and
relative motion create a nonzero range uncertainty. The frozen V2 policy adds a
time-alignment component and rejects labels whose total budget exceeds 0.20 m.

## 6. Offline error, predicted q90, and empirical coverage

These terms are not interchangeable:

- **Offline error**: `abs(predicted_or_corrected_range - ground_truth_range)`;
  available only in labeled replay/evaluation.
- **Runtime predicted q90**: a model output estimating a per-frame upper 90th
  percentile of absolute core error; it must not use ground truth at runtime.
- **Empirical q90 coverage**: the fraction of independent labeled samples/groups
  for which `offline_error <= predicted_q90`.

The q90 target must be constructed from out-of-fold q50 predictions grouped by
run/session, or from a calibration partition not used to fit q50. In-sample q50
errors are prohibited because they make q90 optimistic. Final empirical coverage
is reported on a partition used by neither q50 nor q90 fitting/calibration.

## 7. Dataset audit and gaps

The v18 development input is the applicable current audit basis: 30 retained
`range_v2_*` datasets plus two opened former holdouts, 2,500 labeled samples and
32 independent groups. Both former holdouts are now development data and cannot
be final promotion evidence.

Read-only distance coverage:

| Canonical region/bin | Frames | Independent groups | R1 status |
|---|---:|---:|---|
| near `<3 m` | 0 | 0 | MISSING |
| core `3–4 m` | 121 | 2 | PARTIAL |
| core `4–5 m` | 98 | 3 | PARTIAL |
| core `5–6 m` | 84 | 2 | PARTIAL |
| core `6–7 m` | 168 | 1 | MISSING |
| core `7–8 m` | 1,236 | 13 | PARTIAL; overrepresented |
| core `8–9 m` | 349 | 6 | PARTIAL |
| core `9–10 m` | 444 | 5 | PARTIAL |
| core `10–11 m` | 0 | 0 | MISSING |
| core `11–12 m` | 0 | 0 | MISSING |
| far `>12 m` | 0 | 0 | MISSING |
| invalid regimes | 0 stored invalid labels | 0 | MISSING |

All 32 retained groups have constant ground-truth range within the group. There
are no labeled approaching/receding sequences, no true range boundary crossings,
and no promotion-grade near/far/invalid coverage. Calibration reseeds exist in
some static groups but the current schema lacks explicit calibration/track epoch
fields required for event-level evaluation.

### Development coverage versus promotion coverage

- **Development coverage** may include all previously opened labels, including
  H1/H2, and is used for feature design, fitting, calibration, model selection,
  thresholds, and development gates.
- **Promotion coverage** is a completely new, precommitted, label-blind final
  holdout collected only after code/schema/preprocessing/models/thresholds/filter
  are frozen. It is evaluated once and is never used to fill development gaps.

Passing development coverage does not imply promotion coverage. Existing 2,500
samples count only toward development.

## 8. Compatibility matrix

| Block | Current file/symbol | Keep | Adapter | Refactor | New | Risk |
|---|---|---:|---:|---:|---:|---|
| Raw range | `MetricTargetFusion._consume_depth_result` | yes | yes | no | no | Medium: calibration/domain bias |
| MiDaS features | `DepthModelAdapter`, `LatestDepthWorker`, `TargetDepthExtractor` | yes | yes | no | no | Medium: async/frame association |
| Dataset builder | `RangeResidualDatasetCollector` | yes | yes | yes | no | High: missing epochs/dynamic metadata |
| Labels | `RangeResidualDatasetSample.residual_target_m`; trainer correctable mask | historical only | no | yes | yes | High: objective changes |
| Group split | `grouped_split`, `applicability_stratified_grouped_split` | deterministic hash idea | yes | yes | no | High: calibration/q90 partitions |
| Scaler | train-only `StandardScaler` | yes for OOD | yes | yes | no | Medium: raw/scaled benchmark absent |
| XGBoost train | `train_range_residual_bundle` | infrastructure | yes | yes | phase-specific trainers | High |
| Replay | `evaluate_range_residual_bundle` | limited | no | yes | deterministic sequence replay | High |
| Runtime inference | `RangeResidualCorrector`; fusion call site | shadow pattern/checksums | yes | later only | Range V2 assembler | Critical |
| Temporal filter | calibrator/range filter/EKF/target filter | primitives | yes | yes | zone state machine | Critical |

## 9. Proposed files by later patch

R1 creates documentation only. Subject to R1 approval, prefer cohesive modules
over duplicating existing functionality:

```text
range_v2_spec.py
range_v2_labels.py
range_v2_dataset_audit.py
range_v2_baseline_eval.py
range_v2_training.py
range_v2_temporal_filter.py
range_v2_evaluate.py
range_v2_replay.py
range_v2_model_bundle.py
test_range_v2_*.py
```

Existing `range_residual_*` files and bundles remain untouched as historical
baseline code/artifacts. Runtime integration is deferred to R10 and must be
shadow-only.

## 10. R1 decision

**GO WITH CONDITIONS for R2 only.** R2 is limited to an offline derived-label
view, schema proposal, and coverage report; it may not rewrite original samples
or manifests.

**NO-GO for R3+ training/promotion/runtime at this point.** Before model training,
development data must add near, far, invalid, dynamic direction, 10–12 m core,
and independent coverage required by the frozen specification. Before freezing
any model bundle, the workspace must also have a real Git commit identity or an
equivalent immutable source snapshot manifest.

## 11. Patch R1 execution record

Files changed:

```text
docs/RANGE_V2_NEAR_CORE_FAR_COMPATIBILITY_REPORT.md
docs/RANGE_V2_FROZEN_SPEC.md
```

Tests/checks run:

```text
PYTHONPATH=/home/sup/swarm_dashboard/.venv/lib/python3.12/site-packages \
  python3 -m pytest -q test_*.py
Result: 234 passed, 1 known PytestReturnNotNoneWarning

./run_all.sh --check
Result: Runtime check passed
```

Artifacts: the two documentation files above; no dataset/model/runtime artifact
was created. No candidate was trained, no dataset was relabeled, and no
simulation was run.

Known limitations: no valid root Git metadata; dynamic capture-time uncertainty,
near/far/invalid data, direction/controller deadband evidence, and 10–12 m core
coverage remain absent.

Rollback: remove only the two R1 documentation files. No application behavior or
historical artifact needs rollback.

Go/No-Go: **GO WITH CONDITIONS for R2 only; NO-GO for training, promotion,
shadow/simulation, or runtime integration.**
