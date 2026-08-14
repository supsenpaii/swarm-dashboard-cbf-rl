# M52 + MiDaS range residual correction

`MetricTargetFusion` remains the physics baseline and the only metric target
pipeline. The optional residual corrector runs after M52-calibrated MiDaS has
produced a ray range and before the existing temporal range filter and target
EKF.

## Runtime modes

```text
SWARM_RANGE_RESIDUAL_MODE=off|shadow|active
SWARM_RANGE_RESIDUAL_BUNDLE=/absolute/path/to/trained_bundle
```

- `off` is the default and does not require ML dependencies or artifacts.
- `shadow` loads the real bundle and reports its candidate residual, but always
  returns the physics range to the filter/EKF.
- `active` may apply a residual only when the bundle, schema, checksums,
  StandardScaler transform, OOD gate and XGBoost prediction all validate.

Any missing dependency/artifact, checksum/schema mismatch, NaN/Inf, OOD sample,
invalid residual or inference exception returns the unchanged physics range.
The runtime code never calls `fit()`.

## Physics applicability gate

Before either the residual output or unchanged physics range can enter the
temporal range filter/EKF, a deterministic gate checks the image-ray envelope,
calibration condition/residual/inlier fraction, anchor spatial coverage,
target-anchor extrapolation and propagated relative range uncertainty:

```text
SWARM_RANGE_APPLICABILITY_MODE=off|shadow|active
```

The default is `active`: a measurement outside the locked, front-facing
validated envelope is recorded for diagnostics/dataset collection and then
discarded before `update_range()`. `shadow` reports the same decision without
changing fusion. The envelope thresholds have explicit
`SWARM_RANGE_APPLICABILITY_*` overrides, but widening them requires new locked
evidence; it is not a tuning shortcut.

With a valid residual bundle in `active`, univariate OOD, multivariate OOD or a
predicted residual outside the `+/-3 m` containment limit also abstains from the
measurement. Missing dependencies/artifacts and inference exceptions still
fall back to the physics pipeline; the deterministic gate remains independent
of ML availability.

## Feature contract

Schema `m52_midas_range_features_v2` contains these ordered `float64` values:

1. `m52_anchor_median_m`
2. `m52_anchor_quality`
3. `midas_target_inverse_depth_median`
4. `midas_target_inverse_depth_spread`
5. `bbox_width_px`
6. `bbox_height_px`
7. `bbox_area_fraction`
8. `previous_physics_distance_m`
9. `delta_time_s`
10. `bbox_center_x_fraction`
11. `bbox_center_y_fraction`
12. `image_ray_x`
13. `image_ray_y`
14. `target_bearing_down`
15. `camera_optical_axis_down`
16. `calibration_scale`
17. `calibration_offset`
18. `calibration_residual_m_inv`
19. `calibration_condition_number_log10`
20. `calibration_inlier_fraction`
21. `anchor_spatial_coverage_fraction`
22. `target_anchor_extrapolation_iqr`
23. `ray_range_relative_std`

The missing-value policy is `reject_sample`. In particular, the first sample
after reset has no previous fused range or valid `dt`, so correction remains
fail-closed until a prior accepted range exists. Images, absolute timestamps,
NED/WGS84 target coordinates and control commands are not features.

`m52_anchor_quality` is the accepted/candidate anchor fraction multiplied by
the M52 spatial-coverage fraction. `bbox_area_fraction` is bbox area divided by
frame area. These formulas and feature order are runtime contracts and must be
identical in the offline dataset/trainer.

The archived v1 candidates intentionally fail the v2 schema check. They remain
immutable evaluation evidence but cannot be promoted by the v2 runtime.

## Bundle contract

The bundle directory must contain `manifest.json`, a joblib-serialized fitted
StandardScaler, an XGBoost applicability classifier and an XGBoost residual
regressor. The manifest must contain:

```json
{
  "bundle_version": "m52_midas_range_residual_bundle_v2",
  "feature_schema_version": "m52_midas_range_features_v2",
  "feature_names": ["the exact ordered feature list above"],
  "feature_types": ["float64", "... one per feature"],
  "missing_value_policy": "reject_sample",
  "model_input": "scaled",
  "maximum_absolute_z_score": 8.0,
  "maximum_absolute_residual_m": 3.0,
  "minimum_applicability_probability": 0.8,
  "residual_prediction_std_m": 0.5,
  "ood": {
    "method": "zscore_and_shrunk_mahalanobis",
    "scaled_feature_mean": ["..."],
    "scaled_inverse_covariance": [["..."]],
    "maximum_mahalanobis_distance": 6.0
  },
  "training_manifest": {"dataset_sha256": "...", "split": "grouped"},
  "versions": {"scikit_learn": "...", "xgboost": "..."},
  "artifacts": {
    "scaler": {"filename": "scaler.joblib", "sha256": "..."},
    "applicability_model": {
      "filename": "applicability_model.json",
      "sha256": "..."
    },
    "model": {"filename": "model.json", "sha256": "..."}
  }
}
```

`model_input` explicitly locks whether the trained Booster consumes raw or
scaled features. The StandardScaler transform, per-feature z-score check and
shrunk-covariance Mahalanobis OOD check are always executed.
The classifier estimates whether the true residual is inside the correction
containment envelope; only applicable samples reach the residual regressor.
Offline training first reapplies the locked deterministic runtime envelope and
excludes samples that the pre-model gate would already reject. The remaining
groups are split with both correctable and uncorrectable labels represented in
train, validation and test; a hash-only split with single-class holdouts is not
accepted.
`residual_prediction_std_m` comes from held-out validation and is
added in quadrature to the physics range uncertainty when an active correction
is applied.

No bundle is enabled by default. Local candidate bundles and datasets under
`artifacts/` do not change that production-safe `off` default; promotion still
requires independent coverage and shadow/runtime evidence.

Every newly trained manifest records a fail-closed promotion gate. Validation
and test each require applicability precision of at least 0.95, recall of at
least 0.50, uncorrectable false-accept rate no greater than 0.05, corrected MAE
no greater than 1.0 m and positive MAE/RMSE improvement. Held-out residual
prediction standard deviation must be no greater than 1.0 m. A small positive
improvement alone is not a promotion result.

## Collect a synchronized SITL dataset

Collection is disabled unless all three variables are set:

```dotenv
SWARM_RANGE_DATASET_DIR=/absolute/path/to/dataset_run_01
SWARM_RANGE_DATASET_RUN_ID=sitl_static_5_15m_run_01
SWARM_RANGE_DATASET_TARGET_ID=UAV-02
SWARM_RANGE_RESIDUAL_MODE=off
SWARM_RANGE_DATASET_DEPTH_RATE_HZ=7.5
SWARM_RANGE_DATASET_MAX_DEPTH_AGE_S=3.0
SWARM_RANGE_DATASET_GIMBAL_TRACKING_ENABLED=false
```

`SWARM_RANGE_DATASET_TARGET_ID` is mandatory. The selected bbox must actually
track that Gazebo entity; otherwise the run must be discarded. The target ID,
run/session IDs and timestamps are grouping/label metadata and never model
features.
Collection is rejected in `active` mode so an already-corrected feedback
history cannot contaminate the physics-baseline training dataset. `off` is
recommended; `shadow` is also accepted because it does not alter fusion output.

When enabled, the existing timestamp-synchronized
`simulation_target_ground_truth()` supplies camera-link-to-target-center range.
The collector appends `samples.jsonl` beside a versioned `manifest.json` and
records invalid ground-truth reasons as diagnostics. It never changes the
physics range, correction result, target state or control command.

Dataset schema v2 also stores ground-truth source, uncertainty, time offset,
quality and lever-arm-correction state. The trainer accepts only
`simulation_exact`, `rtk_fixed`, `total_station` or `uwb_calibrated` labels,
requires lever-arm-corrected camera-center-to-target-center semantics, limits
uncertainty to 0.20 m and absolute timestamp offset to 100 ms. The online
collector is explicitly marked as Gazebo ground truth; hardware logs must be
merged into a new dataset offline rather than relabeling a SITL manifest.

Dataset mode is observation-only: body-yaw, follow, motion, attitude,
bootstrap and native Follow Target callbacks are removed. Gimbal tracking is
also disabled by default so a fixed Gazebo camera angle cannot silently create
vehicle pre-centering commands. Point the simulated gimbal before selecting
the bbox; for the flat SITL world, a downward pitch such as `-10 deg` provides
ground-facing M52 anchors while keeping an equal-height aerial target visible.
The dataset-only depth age is relaxed to 3 seconds for CPU inference, while the
normal flight profile remains unchanged.

`run_all.sh` accepts optional spawn-pose overrides without changing its normal
defaults:

```dotenv
SWARM_UAV_01_MODEL_POSE=0,0,0,0,0,0
SWARM_UAV_02_MODEL_POSE=3,0,0,0,0,0
```

Use a fresh directory and unique run ID for each independent condition or
trajectory. Collect at least three independent groups; useful coverage includes
static, constant-velocity and turning targets at several ranges, angles, bbox
sizes and backgrounds. Do not combine nearly identical frames from one session
across train, validation and test.

## Train and evaluate offline

Install the optional training dependencies outside the flight runtime when
needed:

```bash
python -m pip install -r requirements-ml.txt
```

Train from one or more completed dataset directories:

```bash
python range_residual_training.py train \
  artifacts/range_dataset_run_01 \
  artifacts/range_dataset_run_02 \
  artifacts/range_dataset_run_03 \
  --output artifacts/range_residual_bundle_v2
```

The trainer deterministically groups by entire run/session before splitting.
It calls `StandardScaler.fit_transform()` only on the training partition;
validation and test use `transform()` only. XGBoost learns
an applicability label from the correction limit, then learns
`ground_truth_distance_m - physics_distance_m` on correctable training samples.
It exports the scaler, both Boosters, multivariate OOD contract, artifact
checksums, dependency versions, split manifest and runtime-gated held-out
metrics.

Replay/evaluate a bundle against completed datasets with:

```bash
python range_residual_training.py evaluate \
  artifacts/range_dataset_holdout \
  --bundle artifacts/range_residual_bundle_v2 \
  --json-output artifacts/range_residual_holdout_report.json
```

Do not promote to `active` merely because training error is low. Promotion
requires positive held-out MAE/RMSE improvement over physics, acceptable p95,
OOD coverage and subsequent shadow/runtime gates.
