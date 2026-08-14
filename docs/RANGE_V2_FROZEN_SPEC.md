# Range Estimation V2 Frozen Specification

Spec ID: `range_v2_near_core_far_r1_v001`  
Frozen: 2026-08-04, before any Range V2 candidate training  
Seed: `52`  
Status: **frozen for R2–R9 development; runtime/control disabled**

## 1. Change control

This specification defines a new objective. It is not v18 tuning and does not
change the historical interpretation or NO-GO status of v15, v16, v17, or
diagnostic v18.

Any change to canonical labels, ground-truth semantics, split policy, feature
schema, model targets, transition bands, metric definitions, promotion gates,
or threshold-selection procedure requires a new spec ID and candidate version.
No value may be changed after seeing a final holdout. The current workspace is
not a valid Git repository; until that is repaired, SHA-256 of this file and an
immutable source snapshot manifest are required in place of a Git commit.

## 2. Scope and safety invariants

R1 freezes offline contracts only. Through R9:

- PX4 Follow remains off;
- no `FOLLOW_TARGET`, `PARAM_SET`, arm, takeoff, flight-mode, OFFBOARD, motion,
  body-yaw, or controller command is sent by Range V2;
- residual correction remains `off` in the runtime default;
- no new model output enters the beacon, target-position, or controller path;
- ground truth is label/evaluation-only and is never a runtime feature;
- all new features and bundles fail closed;
- final holdout is neither collected nor opened during development;
- no unexecuted experiment may be reported as PASS.

Shadow inference is permitted after offline replay passes. It may execute the
real inference and temporal state machine and append structured logs, but its
output must terminate at a shadow sink. It must have **zero controller effect**:
no target measurement replacement, no beacon publication, no mode request, and
no motion command.

## 3. Canonical label model

Validity and distance zone are separate labels:

```text
validity_gt:      valid | invalid
distance_zone_gt: near | core | far | unknown
```

Canonical distance labels, when trustworthy ground truth exists, are:

```text
near: d_gt < 3.0 m
core: 3.0 m <= d_gt <= 12.0 m
far:  d_gt > 12.0 m
```

Exact boundary cases are frozen:

```text
2.99 m -> near
3.00 m -> core
12.00 m -> core
12.01 m -> far
```

`invalid` is not a fourth physical distance. An invalid sample may retain a
label-only `distance_zone_gt` for audit if trustworthy ground truth exists, but
the emitted runtime `zone` is `invalid` and no range measurement is released.
If ground truth itself is unavailable/untrusted, `distance_zone_gt = unknown`.

## 4. Transition bands and canonical labels

The frozen evaluation bands are:

```text
near certain:              d_gt < 2.7 m
near/core transition:      2.7 m <= d_gt < 3.3 m
core certain:              3.3 m <= d_gt <= 11.7 m
core/far transition:       11.7 m < d_gt <= 12.3 m
far certain:               d_gt > 12.3 m
```

Transition bands never overwrite the canonical labels. For example, 2.9 m is
canonically `near` and `is_boundary_transition=True`; 3.1 m is canonically
`core` and `is_boundary_transition=True`.

Frozen training/evaluation policy:

- validity training may use transition samples because validity is orthogonal;
- flat/ordinal hard zone training excludes transition-band samples;
- core residual training uses every canonical core sample, including core-side
  transition samples, but assigns transition samples 0.5 of the corresponding
  non-transition bin weight;
- transition samples are always retained for dedicated boundary evaluation;
- no hard-zone promotion metric hides or merges transition errors into the
  certain-zone aggregate.

Runtime never has access to `d_gt`; it enters transition states using calibrated
probabilities, the accepted core estimate when available, stable-frame counts,
and hysteresis.

## 5. Ground-truth contract

### 5.1 Spatial quantity and frames

The metric label is Euclidean slant range from the camera optical/reference
center at the source frame time to the target reference center at the same time.

Accepted sources are versioned and explicit:

- Simulation: Gazebo `camera_link` origin to target model center in the Gazebo
  ENU world frame.
- RTK: lever-arm-corrected camera center to target center in ECEF, with antenna
  lever arms specified in body FRD and rotated through NED.
- Future total-station/UWB sources require a documented transform into the same
  center-to-center quantity before acceptance.

The output bearing/position pipeline uses NED, but rotating a common-frame
Euclidean range does not change its magnitude. Drone-center distance,
ground-plane distance, optical-axis depth, bbox scale, and relative MiDaS score
must not be stored as `ground_truth_distance_m`.

### 5.2 Frame and timestamp identity

Every sample/event must include:

```text
run_id
session_id
track_epoch
calibration_epoch
frame_index
measurement_timestamp_s
measurement_clock_id
sensor/source timestamp when available
source_sim_timestamp_s when applicable
pose sample timestamps/offsets or interpolation bracket
ground-truth timestamp/offset
```

`measurement_timestamp_s` is the timestamp associated with the source image,
not inference completion time. Inference completion and output timestamps are
logged separately for latency measurement. A reused timestamp or cross-session,
cross-track, or cross-calibration result is invalid.

Current camera timing uses a post-conversion monotonic receipt time, not a proven
sensor-capture timestamp. This is acceptable for current static development
audit only. Dynamic development/promotion data must provide a sensor timestamp
or an independently measured capture-to-receipt uncertainty. If that term is
unknown, the dynamic label is invalid for direction, lag, and promotion metrics.

### 5.3 Ground-truth uncertainty budget

The frozen label budget is:

```text
u_gt_total_m^2 =
    u_camera_position_m^2
  + u_target_position_m^2
  + u_lever_arm_m^2
  + u_reference_frame_m^2
  + u_interpolation_m^2
  + u_time_alignment_m^2

u_time_alignment_m =
    abs(d_dot_gt_m_s) * abs(delta_t_s)
  + 0.5 * a_relative_bound_m_s2 * delta_t_s^2
```

`a_relative_bound_m_s2` is frozen initially at `3.0 m/s^2`, matching the
existing physical range filter's acceleration containment. A different bound
requires a new spec revision supported by vehicle/controller evidence.

Acceptance requires all of:

```text
abs(delta_t_s) <= 0.100 s
u_gt_total_m <= 0.200 m
ground_truth source/quality accepted
camera and target reference centers defined
lever arm applied or proven zero by source definition
```

Simulation position quantization may be zero, but simulation uncertainty is not
automatically zero for dynamic data. Pose age/interpolation and relative motion
must contribute to the budget. RTK position components use camera/target
horizontal and vertical uncertainties plus lever-arm, frame, interpolation, and
time-alignment terms. All components are stored, not only the total.

Ground-truth uncertainty is used to validate labels and contextualize offline
error; it is never a model input in runtime inference.

## 6. Direction label contract

Direction labels use ground-truth radial range rate aligned to the source frame:

```text
approaching: d_dot_gt < -direction_deadband_m_s
stationary:  abs(d_dot_gt) <= direction_deadband_m_s
receding:    d_dot_gt > direction_deadband_m_s
```

The deadband is not an arbitrary hyperparameter. It is derived once from
training/calibration ground truth before any direction candidate is evaluated:

```text
gt_noise_bound = P99(abs(d_dot_gt)) on independently verified stationary GT
controller_bound = minimum radial rate that the reviewed controller is intended
                   to react to
direction_deadband = max(gt_noise_bound, controller_bound)
```

The calculation, source groups, and resulting number are written into the split
manifest. If stationary GT noise or the controller requirement is unavailable,
far-direction training is **NO-GO**. The value is then frozen before model
validation/development test and cannot be tuned against candidate results.

Offline GT rate may use a symmetric smoothing/differentiation window for label
quality if its alignment and uncertainty are recorded. Runtime features and the
far model are strictly causal: current and past frames only. Direction metrics
include the label-filter delay separately from model/temporal delay.

## 7. RangeEstimateV2 output contract

The implementation must provide an immutable dataclass or equivalent versioned
schema with at least:

```python
@dataclass(frozen=True)
class RangeEstimateV2:
    timestamp_us: int
    session_id: str
    frame_index: int

    zone: str                    # near | core | far | invalid
    temporal_state: str
    zone_confidence: float
    validity_confidence: float

    raw_metric_range_m: float | None
    corrected_metric_range_m: float | None
    uncertainty_q90_m: float | None

    relative_range_score: float | None
    radial_direction: str | None
    radial_direction_confidence: float | None
    proximity_state: str | None

    calibration_epoch: int
    track_epoch: int

    is_transition: bool
    is_reacquire_hold: bool
    is_calibration_hold: bool

    valid_for_metric_position: bool
    valid_for_range_control: bool
    valid_for_approach: bool

    stale: bool
    reason_code: str
```

`raw_metric_range_m` is diagnostic physical output. Its presence outside core
does not make it valid for controller use. `corrected_metric_range_m` is present
only for an accepted core state. Relative score is dimensionless/versioned and
must not be serialized into a field with `_m` units.

Frozen semantics:

| Zone | Corrected metric | Relative trend | Controller semantics after a future separate gate |
|---|---|---|---|
| near | `None` | optional | `proximity_state=too_close`; never continue approach |
| core | accepted value + q90 | yes | may be used only when all safety flags pass |
| far | `None` | required | shadow/tracking or separately approved conservative behavior |
| invalid | `None` | `None` | no new measurement/action |

During transition/hold, `is_transition` or the corresponding hold flag is true,
`valid_for_metric_position=False`, and no new corrected metric measurement is
released. A briefly held diagnostic value is marked `stale=True` and is never a
new controller measurement.

## 8. Required model benchmarks

### 8.1 Architecture A — flat baseline

A four-output flat XGBoost baseline (`near/core/far/invalid`) is required only
for comparison. Invalid overrides distance in its emitted class, so this model
is not automatically eligible for production.

### 8.2 Architecture B — hierarchical candidate

```text
deterministic schema/physical gate
  -> B1 validity classifier
  -> B2 ordinal zone model
       P(d >= 3 m)
       P(d > 12 m)
  -> near/core/far probability projection
  -> B3 core residual q50
  -> B4 core absolute-error q90
  -> B5 causal far direction/relative score
```

The ordinal probabilities must satisfy:

```text
0 <= P(d > 12 m) <= P(d >= 3 m) <= 1
```

Raw inconsistent probabilities are logged; inference projects them to the
nearest valid ordinal distribution. If the projection change exceeds `0.20` in
absolute probability for either threshold, zone inference is invalid.

No sample reaches an ML model if critical schema, calibration, bbox/ROI, MiDaS,
or physical-range checks fail. ML cannot convert a deterministic critical
failure into valid output.

## 9. Core residual and q90

The q50 target is core-only:

```text
residual_target_m = ground_truth_distance_m - raw_physical_range_m
corrected_metric_range_m = raw_physical_range_m + residual_q50_m
```

The correction limit is:

```text
max_abs_correction_m = min(3.0 m, 0.25 * raw_physical_range_m)
```

If the predicted correction exceeds this limit, no correction is applied and
the Range V2 metric output is invalid for that frame. Raw physical range remains
diagnostic only. Residual correction is prohibited outside core and remains
feature-flag off by default after bundle creation.

### 9.1 Three distinct uncertainty quantities

1. **Offline absolute error** is computed only with labels:

   ```text
   abs(corrected_metric_range_m - ground_truth_distance_m)
   ```

2. **Runtime predicted q90** is B4's causal per-frame prediction of the 90th
   percentile of that absolute error. Ground truth is unavailable at runtime.
3. **Empirical q90 coverage** is measured offline on independent groups:

   ```text
   mean(offline_absolute_error <= predicted_q90_m)
   ```

B4 must train from out-of-fold q50 predictions with group-disjoint folds, or
from an independent calibration partition not used to fit B3. In-sample q50
errors, test errors, and final-holdout errors are prohibited q90 targets.

A core metric is accepted only when validity and core confidence pass, the
sample is not OOD/transition/hold/stale, correction is within clamp, and:

```text
0 <= uncertainty_q90_m <= 1.50 m
```

## 10. Preprocessing contract

The ordered pipeline is:

```text
schema/version/order validation
-> finite and critical-missing validation
-> physical clipping from frozen engineering bounds
-> train-only quantile clipping
-> noncritical missing flags
-> train-only median imputation for noncritical features
-> selective log1p for predeclared positive skewed features
-> StandardScaler fitted on train only
-> z-score and multivariate OOD statistics fitted on train only
```

Critical missing fields—calibration validity, bbox/ROI validity, MiDaS target
depth, physical range, source timestamp, or required geometry—always produce
invalid output and may not be rescued by imputation.

Tree input must benchmark:

```text
bounded raw features
scaled features
```

Selection uses group-level validation metrics. If balanced accuracy/core MAE
differ by less than 1% relative and safety gates are equal, choose raw inputs for
the tree while retaining the scaler for OOD/drift monitoring. Bundle metadata
must state exactly `model_input=raw` or `model_input=scaled`; inference mismatch
is fatal.

## 11. Group split, calibration, and weighting

No random frame split is allowed. The atomic identity is the entire
run/session/track trajectory; location is added to the group key when multiple
runs share a location regime likely to leak appearance.

Development partitions are:

```text
train groups
calibration groups
validation/model-selection groups
locked development-test groups
```

Grouped cross-fitting inside train is used for q90 target generation. Model
calibrators and confidence threshold selection use calibration groups only.
Early stopping/model selection uses validation groups. The locked development
test is evaluated after choices are frozen. Final promotion holdout is separate
and newly collected.

Seed 52 controls every deterministic split/fold and stochastic learner.

Weighting benchmarks:

- historical equal-total group weighting;
- bounded square-root group weighting:

  ```text
  base_group_weight proportional to 1 / sqrt(group_frame_count)
  normalize sample weights to mean 1
  clip normalized per-sample weight to [0.25, 4.0]
  ```

Core-bin balancing multiplies by inverse square root of train-bin count, then is
renormalized and clipped to `[0.5, 2.0]` before multiplication with group
weight. Every effective group/bin total is reported. No partition weighting is
fit from validation/test labels.

## 12. Confidence thresholds

`0.80` remains the fixed baseline threshold for architecture comparison. A
single shared production threshold is prohibited.

Per-head thresholds are selected on calibration groups by this frozen procedure:

- validity: smallest threshold >=0.80 whose one-sided 95% upper confidence bound
  on false-valid rate is <=1%;
- near: smallest threshold >=0.80 satisfying near precision >=0.95, then near
  recall is checked against the promotion gate;
- core: smallest threshold >=0.80 satisfying core precision >=0.95 and zero
  critical-invalid metric leakage;
- far: smallest threshold >=0.80 satisfying far precision >=0.90;
- direction: per-class calibrated probabilities with abstention unless maximum
  class confidence >=0.80.

If no threshold meets its calibration constraint, the head is NO-GO. Selected
values and calibrator checksums are frozen before locked development-test and
final-holdout evaluation. Thresholds may not be adjusted from those results.

## 13. Temporal state machine

Required states:

```text
INVALID
NEAR
CORE
FAR
NEAR_CORE_TRANSITION
CORE_FAR_TRANSITION
REACQUIRE_HOLD
CALIBRATION_HOLD
```

Policies:

- critical invalid input enters `INVALID` immediately;
- near/core/far entry normally requires three consecutive accepted depth
  results above the entry threshold;
- high-confidence near (`P_near >=0.95`) sets `valid_for_approach=False`
  immediately, while semantic NEAR entry still requires two accepted results;
- the retain threshold is `max(0.70, entry_threshold - 0.10)`;
- leaving a stable zone requires three consecutive results below its retain
  threshold and evidence for the adjacent ordered zone;
- skipping directly from near to far or far to near is invalid unless an
  intervening stable core sequence occurs;
- reacquire increments `track_epoch`, resets causal direction/filter history,
  and requires three stable accepted results;
- calibration reseed increments `calibration_epoch`, prevents continuity across
  epochs, and requires the calibrator's five stable accepted depth results;
- stale diagnostic hold is at most `0.25 s`; after that all range fields clear;
- no stale or hold output is a new measurement.

Near filters state/proximity, not a fake metric. Core uses confidence- and
q90-aware smoothing plus rate/jump limits. Far filters only dimensionless
relative score and direction. Invalid clears measurement output.

Transition blending may blend probabilities and filter state only. It must not
blend a near/far relative score into a value labeled in meters. A corrected
metric becomes available only after stable CORE entry.

## 14. Development coverage minimum

Development coverage is data that may be inspected and used for fitting,
calibration, model selection, and development gates. Opened former holdouts are
development-only.

Each 1 m core bin from 3–12 m requires:

- at least 3 independent run/session groups;
- at least 2 location/background regimes;
- at least 2 pitch/geometry regimes;
- at least 1 stationary group;
- at least 1 approaching and 1 receding trajectory crossing the bin;
- lateral coverage;
- at least 30 valid frames per reported group/bin cell.

Priority bins 5–8 m require at least 5 independent groups. No single session
may contribute more than 40% of the effective weight in a bin.

Near and far each require at least three independent stationary, approaching,
and receding groups plus two independent boundary-crossing groups. Invalid data
must cover bbox lost/small, ROI noise, calibration invalid, missing critical
feature, unstable MiDaS, severe OOD geometry, and reacquire/calibration holds,
with at least two independent groups for every critical invalid regime.

Coverage status is `COVERED`, `PARTIAL`, or `MISSING`. Model promotion training
is blocked if any 5–8 m bin is MISSING, either 10–12 m bin is MISSING, or
near/far/critical-invalid/dynamic direction coverage is MISSING.

## 15. Promotion coverage minimum

Promotion coverage is a newly collected, precommitted, label-blind final
holdout. It is collected only after development replay and shadow pass and after
code, schema, preprocessing, weights, models, calibrators, thresholds, temporal
configuration, and reports are frozen.

The final holdout must contain, without sharing run/session/location identities
with development:

- at least 2 independent groups in every core bin, from at least 2 contexts;
- stationary and dynamic coverage in every core bin;
- at least 3 near groups, 3 far groups, and two crossings of each boundary;
- all critical invalid regimes, with no synthetic duplication of one failure;
- calibration transition and tracker reacquire events;
- pitch, background, and lateral regimes not copied from development scripts.

Promotion coverage is evaluated once. It may not repair development coverage,
select thresholds, fit q90, or tune temporal parameters. A fail makes the frozen
candidate NO-GO and requires a new version and a new final holdout.

## 16. Frozen metrics and development promotion gates

All metrics are reported aggregate, per group, and where applicable per core
bin. Transition samples have their own report.

### 16.1 Validity

- critical-invalid cases emitting a metric measurement: `0`;
- aggregate false-valid rate: <=`1%`;
- per-invalid-group false-valid rate: <=`5%`;
- PR-AUC: >=`0.95`;
- Brier score: <=`0.10`;
- checksum/schema/version mismatch: `100%` fail-closed.

### 16.2 Certain-zone classification

- balanced accuracy: >=`0.90`;
- near recall: >=`0.98`;
- core recall: >=`0.90`;
- far recall: >=`0.90`;
- certain-zone flip rate: <=`0.20 flips/s` and <=`1` false transition per
  30-second stationary sequence.

### 16.3 Core metric

For every 1 m core bin:

- absolute signed bias: <=`0.50 m`;
- median absolute error: <=`0.75 m`;
- MAE: <=`1.00 m`;
- P90 absolute error: <=`1.50 m`;
- P95 absolute error: <=`2.00 m`;
- catastrophic error `>3.0 m`: <=`1%`;
- residual MAE may not worsen raw physical MAE by more than `0.10 m`.

The residual q50 head additionally needs >=`5%` aggregate MAE improvement over
raw and no safety-gate regression. If the raw baseline already passes but q50
does not provide that improvement, q50 remains NO-GO/off; Range V2 may continue
development as raw-core plus validity/zone/temporal, but may not claim residual
promotion.

### 16.4 q90 uncertainty

- empirical coverage overall: >=`0.88`;
- empirical coverage per adequately sampled bin: >=`0.85`;
- median predicted q90: <=`1.00 m`;
- P90 predicted q90: <=`2.00 m`;
- accepted runtime core q90: <=`1.50 m`;
- coverage is reported separately for OOF/calibration, validation, development
  test, and final holdout.

### 16.5 Stationary stability

- output standard deviation: <=`0.20 m` per stationary core group;
- median absolute frame delta: reported, no aggregate-only gate;
- P95 frame delta: <=`0.25 m`;
- absolute linear drift slope: <=`0.02 m/s`;
- maximum 10-second drift: <=`0.30 m`;
- temporal filtering must reduce P95 delta by >=`30%` unless the unfiltered
  output already meets half the absolute limit; it may not increase drift.

### 16.6 Dynamic response and far trend

- direction balanced accuracy: >=`0.85`;
- approaching/stationary/receding recall: each >=`0.80`;
- direction transition delay: <=`0.75 s`;
- core range-response lag from causal cross-correlation: <=`0.50 s`;
- stop settling time: <=`1.00 s`;
- stop overshoot: <=`0.50 m`;
- range-rate error and per-group confusion matrices are mandatory reports.

### 16.7 Boundary and events

- semantic boundary transition delay: <=`0.75 s`;
- no one-frame zone transition;
- no corrected metric emitted during transition/hold;
- no output before required calibration/reacquire stable window;
- first accepted post-event core absolute error: <=`1.50 m`;
- post-event empirical q90 coverage is evaluated across independent events with
  the same >=`0.88` aggregate gate, rather than requiring every individual
  sample to fall below a nominal 90th-percentile bound;
- calibration/reacquire jump, settling frames, false transitions, and boundary
  overshoot are reported per event.

### 16.8 Latency and safety output

- end-to-end source-frame to Range V2 output P95 latency: <=`0.50 s`;
- temporal lag and inference latency are reported separately;
- near/far/invalid corrected metric leakage: `0`;
- far relative score is never serialized as meters;
- invalid/stale output never becomes a new measurement;
- residual correction remains default-off in runtime metadata/configuration.

A filter that passes jitter but fails lag, transition, or stop-settling limits is
NO-GO.

## 17. Evaluation order

The mandatory order is:

```text
offline deterministic replay
-> shadow inference against the same event contract
-> simulation observation-only
-> separate closed-loop safety review and explicit approval
```

Old models are compared only on metrics their historical contracts support;
incompatible fields are `N/A`, not relabeled. Every candidate report includes
raw physical baseline, architecture A, architecture B, q50/q90 heads, far head,
and full output with/without temporal filtering.

## 18. Bundle and fail-closed contract

An eventual `models/range_v2/v001/` bundle contains the schema, preprocessing,
physical bounds, train quantiles, all models/calibrators, temporal config,
thresholds, training/split manifests, feature statistics, evaluation summary,
metadata, and SHA-256 for every artifact.

Metadata records seed 52, dataset version, immutable source identity, schema
hash, preprocessing checksum, library versions, transition bands, selected
thresholds, direction deadband derivation, group IDs, GT contract version, and
all model checksums. Any missing/mismatched artifact, schema, checksum, clock,
epoch, or required feature yields invalid output.

No production bundle is created merely to satisfy a directory layout. If audit,
coverage, training, replay, or shadow fails, `model_manifest.json` records
`NOT_TRAINED` or `NO-GO` and no runtime candidate is installed.

## 19. Current R1 gate

R2 offline audit/relabel tooling: **GO WITH CONDITIONS**. Original dataset files
and manifests remain immutable; R2 writes only derived artifacts.

Training/promotion/runtime: **NO-GO** because current development coverage has no
near, far, invalid, approach, recede, boundary crossing, 10–11 m, or 11–12 m
data, and dynamic timestamp uncertainty is not yet bounded.
