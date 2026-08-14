# Range V2 Patch R3.5 — Physical Range Root-Cause Audit

## Decision

```text
ROOT_CAUSE_CONFIRMED_FIX_REQUIRED
```

This is a read-only/offline audit of the 2,500 locked static-core development
rows. It confirms defects in the physical quantity contract and confirms that
per-frame affine calibration parameter variation is the direct source of the
observed stationary raw-range variation in this corpus. It does **not** prove
that either finding explains the full multi-metre bias, and it does not claim
that ML can repair the physical pipeline.

R4 and R5 remain unstarted.

## Scope and integrity

The audit verified before and after reconstruction:

- frozen spec ID `range_v2_near_core_far_r1_v001` and SHA-256
  `5b2251aadf07d7dc12169d5cddc319a9abb07ead1998c26361c5dd96d3289112`;
- R2 manifest SHA-256
  `fc5ecca960aebd1dee4c8991f363064c7b595e8f1437dc2f260cca71534441f6`;
- R3 manifest SHA-256
  `f6e3c3c973dbc568553639651e9a3b817c65a2ddbb85dfc675dd9333506426b5`;
- all checksummed R2 and R3 outputs;
- 32 source manifests plus 32 source JSONL files (64 source files);
- 2,500 unique source rows and their dataset/run/session/group/frame identity.

All source rows have `correction_mode=disabled`. Runtime default remains
`SWARM_RANGE_RESIDUAL_MODE=off`. No model was trained; no scaler, calibrator,
or threshold was fitted on the corpus; no production geometry/calibration/
MiDaS/runtime/controller code was changed; no PX4, Gazebo, simulation, shadow,
data collection, or final holdout was used.

## 1. Quantity and reference-center contract

The two quantities are both expressed as slant-range magnitudes, but they are
**not the same physical quantity in the implementation**.

| Quantity | Source/reference center | Target/reference center | Frame | Timestamp evidence | Verdict |
|---|---|---|---|---|---|
| `ground_truth_distance_m` | Gazebo `camera_link` origin | target model origin | Gazebo ENU; Euclidean norm | source simulation timestamp; interpolation or nearest pose within 100 ms | Defined center-to-center, but `camera_link` origin is not proven to be optical center |
| `raw_physical_range_m` | `camera_position_ned_m` used by ground-anchor geometry; default runtime offset makes this vehicle center | foreground statistic from eroded bbox ROI | calibrated optical depth converted to slant | frame/context stored in `DepthJob` | Not target-center range and not guaranteed to use GT source center |

Evidence:

- `main.py:1109-1324` locates a pose ending in `camera_link` and computes its
  Euclidean distance to the target model pose.
- `main.py:98-110` defaults the tracking camera lever arm to `(0,0,0)`;
  `main.py:2507-2515` consequently constructs the anchor camera position from
  the vehicle center unless explicit environment offsets are supplied.
- `target_depth_extractor.py:23-112` erodes the bbox, selects the foreground
  half of ROI inverse depth, and returns its median metric depth. That is a
  visible-surface/domain statistic, not the target model center.
- The current local PX4 gimbal SDF places the camera sensor at
  `(-0.0412, 0, -0.162)` with yaw π relative to `camera_link`. This proves that
  a link origin and sensor optical center are distinct in the inspected model,
  but the exact source-run SDF checksum was not recorded, so the offset's
  numerical contribution to old datasets remains unquantified.

The mismatch is therefore **CONFIRMED**. The legacy corpus lacks camera,
vehicle, target and optical-center poses, so it cannot establish how much of
the observed 3.25 m MAE is caused by this mismatch.

## 2. Exact raw-range path

| Step | File / symbol | Input → output | Units/frame | Timestamp | State/cache | Stored in legacy rows |
|---|---|---|---|---|---|---|
| bbox ROI | `target_depth_extractor.py` / `TargetDepthExtractor.extract` | eroded bbox pixels → foreground inverse-depth median | pixels / MiDaS relative inverse depth | `DepthJob` frame | none | q, bbox sizes/fractions only |
| MiDaS | `depth_model_adapter.py` / adapter `infer` | BGR frame → aligned relative inverse-depth map | image frame | job timestamp | model object | model/version not stored |
| ground anchors | `m52_adapter.py` / `M52GroundAnchorAdapter.anchors` | sampled q + ray/ground intersection → metric optical depth | relative q, metres along optical axis, NED geometry | captured frame context | none | median/coverage only |
| affine fit | `metric_depth_calibrator.py` / `MetricDepthCalibrator.fit` | `1/depth = a*q+b` | inverse metres | current depth result | RANSAC RNG, histories, last-valid, change-point state | filtered a/b and summary quality only |
| optical depth | `MetricDepthCalibrator.metric_depth` | q → `1/(a*q+b)` | metres along optical axis | depth result timestamp | applied calibrator state | reconstructable |
| image ray | `metric_target_fusion.py` / `_consume_depth_result` | bbox center + fx/fy → x/y | dimensionless | captured frame context | bearing filter is separate | x/y stored |
| slant conversion | same | optical depth × `sqrt(1+x²+y²)` | metres slant | depth result timestamp | calibration/inverse-depth state | `physics_distance_m` stored |
| range filter | `bearing_range_filter.py` / `update_range` | raw slant → filtered range | metres | depth result timestamp | robust window/rate state | **not** stored as `physics_distance_m` |
| collector | `range_residual_dataset.py` / `record` | `correction.physics_distance_m` → JSONL | metres | depth result timestamp plus captured GT context | append-only | yes |

The verified equation is:

```text
denominator             = calibration_a * target_inverse_depth + calibration_b
metric_optical_depth_m  = 1 / denominator
ray_scale               = sqrt(1 + image_ray_x^2 + image_ray_y^2)
raw_physical_range_m    = metric_optical_depth_m * ray_scale
```

All 2,500 stored values reconstruct exactly; maximum absolute reconstruction
error is `0.0 m`. Optical depth and slant range are not assumed identical.

## 3. Per-group reconstruction result

Across 32 independent run/session groups:

```text
target inverse-depth unique count = 1 in 32/32 groups
ray_scale unique count            = 1 in 31/32 groups
calibration a/b pairs             = multiple in every group
median calibration-only/full std  = 1.000000
ratio range                       = 0.999385 to 1.000000
applied denominator range         = 0.061596 to 0.347822 m^-1
near-zero denominator flags       = 0
negative/non-positive scale flags = 0
fit residual > 0.025 m^-1 flags   = 0
```

For each group, the audit held `q` and `ray_scale` at their group medians and
allowed only recorded `a,b` to vary. This reproduces essentially 100% of the
stored raw standard deviation. Conversely, changing only q produces zero
variation in all groups, and changing only ray produces zero variation except
the one group with two bbox-ray values.

This is a mathematical reconstruction, not a correlation: **variation of the
applied affine calibration parameters directly causes the stationary raw
variation in this corpus**. It does not identify why those parameters vary.

The small recorded inlier residuals and positive denominators do not prove the
target prediction is correct. Residuals measure fit on selected ground anchors;
they do not measure transfer error from ground-anchor q to airborne target ROI
q, target-center error, semantic anchor validity, or temporal stability.

## 4. Camera and intrinsics audit

| Check | Evidence | Result |
|---|---|---|
| Resolution | bbox pixel area divided by area fraction equals exactly 129,600 pixels for every row; repository profile is 480×270 | `SUPPORTED`, not directly logged |
| fx/fy | off-axis rows reconstruct 299.8407 px, matching 480×270 at HFOV 1.35 | `SUPPORTED`, not CameraInfo-authenticated |
| cx/cy | active projector defaults to width/2, height/2; collector stores normalized center fractions | active-code contract known, actual run values not logged |
| Distortion model | absent | `NOT_RECORDED` |
| Rectification | absent | `NOT_RECORDED` |
| CameraInfo hash | absent | `NOT_RECORDED` |
| Pixel convention | bbox center uses pixel coordinates/fractions; analytic normalization `(u-cx)/fx`, `(v-cy)/fy` | internally consistent |
| K inverse | no matrix inverse; analytic pinhole normalization | internally consistent |
| width/height or x/y swap | feature/reconstruction evidence shows no swap in stored raw formula | no supporting evidence for a swap |
| double ray normalization/scale | projector normalizes direction for bearing; raw range independently applies one ray scale | no double-scale found |

All groups share one **partial legacy reconstruction fingerprint**. Because
actual per-run CameraInfo, distortion, rectification and environment snapshots
are absent, an intrinsics mismatch remains `INCONCLUSIVE`; the shared
fingerprint must not be treated as proof that configurations were identical.

## 5. Transform and pitch audit

The active static conventions are:

```text
image ray in sensor FLU: (1, -x, -y)
camera quaternion:       [x, y, z, w]
sensor FLU → Gazebo ENU: quaternion active rotation q*v*q^-1
Gazebo ENU → PX4 NED:    (east,north,up) → (north,east,-up)
vehicle lever arm:       body FRD rotated into NED, then added to vehicle NED
```

The R3.5 synthetic tests cover level camera, small pitch-down, yaw ±90°,
center/left/right/top/bottom pixels, known extrinsic offsets, ENU/NED,
FRD/FLU, and parent-child quaternion composition order. Those tests pass and
no degrees/radians mix or linear Euler-angle addition was found in the audited
path.

However, the legacy rows do not store vehicle/gimbal/camera quaternions,
static extrinsics, pitch, or pose-source hashes. A source-run transform/sign
error is therefore `INCONCLUSIVE`, not disproved.

## 6. M52 anchors and affine calibration

The implementation correctly converts ground-ray slant to optical depth before
fitting and uses the same relative inverse-depth domain for anchors and target.
It enforces downward rays, range bounds, a conservative lower-image region,
local inverse-depth coherence, target-bbox exclusion, positive scale, RANSAC
consensus and condition limits.

Risks that remain:

- there is no semantic ground mask; lower-image/coherence heuristics cannot
  prove an anchor is ground;
- target and anchor semantics differ (airborne target foreground versus ground);
- only applied filtered a/b and summary quality survive in the dataset;
- anchor count, anchor q/depth spread, pixels, rejection reasons, raw fit a/b,
  cache source/age, change-point state and calibration epoch are absent;
- live fitting continues after prewarm, so accepted a/b can evolve during a
  nominally stationary sequence.

Per-group flags are emitted as requested. `denominator_near_zero`,
`scale_sign_invalid`, and `high_fit_residual` are all false in this corpus.
`insufficient_depth_span`, `stale_calibration`, `context_reuse`, and
`ground_anchor_invalid` are `NOT_RECORDED`, not guessed.

## 7. Timestamp, worker and reset audit

- `LatestDepthWorker.submit` copies the frame and preserves frame index,
  measurement timestamp and its immutable context.
- It uses a single pending slot; a newer frame replaces an unprocessed one.
- `reset` increments a generation and clears pending/latest state; results from
  an older generation are not published.
- fusion consumes each depth version once and rejects results older than the
  configured maximum age.
- simulation GT is queried with the source simulation timestamp before the
  frame context is submitted; the value travels with that context.
- the GT provider interpolates pose snapshots over at most 200 ms or uses a
  nearest pose no more than 100 ms away.
- the calibrator can intentionally preserve a stable prewarm calibration and
  can use a stable cached calibration for up to 2.5 s.

Legacy rows store receipt-domain measurement timestamp, source simulation
timestamp and a claimed GT offset of zero, but not sensor-capture clock ID,
pose timestamp, GT timestamp, depth completion time, queue age, calibration
timestamp/source, track epoch, calibration epoch or reset events. Therefore
timestamp/frame association and stale/cross-context calibration are
`BLOCKED_BY_MISSING_LOGS`. Static data is not used to claim dynamic latency.

The downstream range filter is excluded as the cause of R3 raw drift because
the dataset field is written before `update_range`. Upstream inverse-depth and
calibration temporal state are still part of raw generation.

## 8. Hypothesis classification

| Hypothesis | Status |
|---|---|
| ground-truth quantity mismatch | `CONFIRMED` |
| optical-versus-slant mismatch | `NOT_SUPPORTED` |
| ray-scale misuse | `NOT_SUPPORTED` |
| intrinsics mismatch | `INCONCLUSIVE` |
| transform/frame/sign error | `INCONCLUSIVE` |
| target ROI/depth-domain mismatch | `SUPPORTED` |
| affine calibration failure | `CONFIRMED` for stationary variation; deeper cause unresolved |
| invalid ground anchors | `BLOCKED_BY_MISSING_LOGS` |
| stale/cross-context calibration | `BLOCKED_BY_MISSING_LOGS` |
| timestamp/frame association | `BLOCKED_BY_MISSING_LOGS` |
| temporal-filter drift | `NOT_SUPPORTED` for downstream range filter |
| context-dependent physical limitation | `SUPPORTED`, not causal proof |

## 9. Required follow-up (not implemented)

A separately approved fix/instrumentation patch should first align the quantity
contract and add capture-time diagnostics before any R4 collection or R5
training:

1. define and use one optical/reference center for GT, anchor geometry and raw
   range;
2. define target surface versus target center semantics explicitly;
3. log actual resolution/K/distortion/rectification and camera/extrinsic hashes;
4. log per-anchor samples/quantiles/counts/rejections and semantic evidence;
5. log raw and filtered a/b, fit source, age, cache/reseed state and epochs;
6. log capture, pose, GT, depth completion and queue timestamps;
7. replay stationary groups to test whether freezing a stable calibration
   removes jitter without hiding bias.

No fix is part of R3.5.

## 10. Deliverables and rollback

Verification results:

```text
focused R3.5 tests: 14 passed
full repository:    294 passed, 1 known PytestReturnNotNoneWarning
run_all --check:    Runtime check passed
```

Created:

```text
range_v2_physical_audit.py
test_range_v2_physical_audit.py
docs/RANGE_V2_PHYSICAL_ROOT_CAUSE_AUDIT.md
artifacts/range_v2/physical_root_cause/
```

Rollback removes only those two source/test files, this report section/file,
and the R3.5 artifact directory. No source dataset, R1/R2/R3 artifact or
runtime state requires restoration.
