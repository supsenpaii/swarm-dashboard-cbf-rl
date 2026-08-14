# Range V2 R3.6A Instrumentation-Only Report

Date: 2026-08-04  
Schema: `m52_physical_range_diagnostics_r3_6a_v001`  
Outcome: `R3_6A_INSTRUMENTATION_COMPLETE_AWAITING_REVIEW`

## Scope and safety result

R3.6A adds an opt-in, append-only diagnostics sidecar to future range data
collection. It does not modify the raw-range equation, anchor acceptance,
calibration fitting/filtering/cache decisions, range filtering, residual
correction, runtime measurement, controller or PX4 behavior.

The collector is disabled by default. It is enabled only when
`SWARM_RANGE_PHYSICAL_DIAGNOSTICS_DIR` is set, or alongside an explicitly
configured range dataset through `SWARM_RANGE_DATASET_DIR`. It creates
`physical_diagnostics_manifest.json` and appends
`physical_diagnostics.jsonl`; it does not create, modify or overwrite the
original `manifest.json` or `samples.jsonl`.

No PX4, Gazebo, simulation, shadow inference, model training, relabeling or
final-holdout operation was run in this patch. Residual correction remains
default-off. R4 and R5 were not started.

## Diagnostic coverage

Each sidecar row is traceable by run, target, group, session, frame,
measurement timestamp, source-simulation timestamp when available, track
epoch, calibration epoch and a canonical SHA-256 of the row.

### Optical center and reference centers

The row records the NED reference used by raw geometry and its source, the
configured camera lever arm, and—when the existing GT provider supplies
them—the Gazebo ENU `camera_link`, drone-center and target-model-origin
positions and distances. It separately declares:

- raw target semantics: eroded-bbox foreground depth statistic;
- GT target semantics: target model origin;
- optical-center verification status;
- unavailable `camera_link → sensor optical center` and
  `target model origin → visible surface` vectors.

This is observation only. R3.6A deliberately does not assert that configured
camera position is a verified optical center and does not fix the quantity
contract.

### CameraInfo and extrinsics

For every evaluated frame, instrumentation records actual projector
resolution, `fx/fy/cx/cy`, pixel convention, projection method,
distortion/rectification fields when known, source and deterministic camera
fingerprint. Missing ROS/Gazebo CameraInfo properties remain explicit `null`
plus `unavailable_fields`, never inferred.

Extrinsic evidence includes vehicle position/quaternion, camera position and
quaternion, configured body-FRD camera offset, frame labels and synchronized
camera-orientation source.

### Per-anchor evidence

`M52GroundAnchorAdapter` now emits a diagnostic object for every configured
grid point, including rejected points. Each entry records grid location,
pixel, accept/reject reason, NED ray, ground slant distance, metric optical
depth, relative inverse depth, normalized image ray, sensor-forward
component, and local sample count/mean/std/CV where available.

The existing accepted arrays, rejection counts and calibration inputs are
unchanged. A synthetic test verifies that diagnostics contain all 96 default
grid points and that accepted diagnostic values exactly equal the arrays
passed to calibration.

### Raw/filtered calibration, cache, reseed and epochs

The sidecar keeps the current frame's fit result separate from the calibration
actually applied after existing cache policy. Both contain raw and filtered
`a/b`, validity/reason, anchor/inlier counts, residual and spread,
conditioning, covariance/uncertainty, stability, quantiles and
change-point-reseed flag.

It also records live/cached source, cache age, cache TTL, recovery status,
reseed count, track epoch and calibration epoch. Epoch counters are diagnostic
metadata only: track epoch advances on the existing reset path; calibration
epoch advances when calibration is already reset or the existing calibrator
reports a change-point reseed. They are not consumed by runtime output.

### Timestamp association

The depth worker now carries its existing generation plus monotonic job
submission and inference-start timestamps into `DepthResult`. The sidecar
combines these with measurement receipt, depth completion/consume times,
queue wait, inference duration, depth-result age, source simulation time and
available pose/gimbal/GT synchronization diagnostics.

The current image path does not expose an authoritative sensor-capture
timestamp; this remains explicitly unavailable. R3.6A therefore improves
future association evidence but makes no dynamic-latency claim.

## Fail-soft behavior

Malformed/nonfinite diagnostic values, identity errors or sidecar manifest
mismatch reject the diagnostic row and surface collector status. They do not
alter the calibration, range, estimator or controller path. Each JSON row is
written under a lock and only finite JSON values are allowed.

## Files changed

```text
range_physical_diagnostics.py
m52_adapter.py
depth_worker.py
metric_target_fusion.py
tracking_web.py
test_range_physical_diagnostics.py
docs/RANGE_V2_R3_6A_INSTRUMENTATION_REPORT.md
docs/RANGE_V2_IMPLEMENTATION_REPORT.md
artifacts/range_v2/r3_6a_instrumentation/instrumentation_contract.json
artifacts/range_v2/r3_6a_instrumentation/r3_6a_manifest.json
```

## Verification

```text
python3 -m py_compile range_physical_diagnostics.py m52_adapter.py \
  depth_worker.py metric_target_fusion.py tracking_web.py \
  test_range_physical_diagnostics.py
Result: PASS

PYTHONPATH=/home/sup/swarm_dashboard/.venv/lib/python3.12/site-packages \
  python3 -m pytest -q test_range_physical_diagnostics.py \
  test_metric_target_fusion.py test_follow_workflow.py \
  test_camera_pipeline_resolution.py
Result: 59 passed

PYTHONPATH=/home/sup/swarm_dashboard/.venv/lib/python3.12/site-packages \
  python3 -m pytest -q test_*.py
Result: 303 passed, 1 known PytestReturnNotNoneWarning

./run_all.sh --check
Result: Runtime check passed
```

The strongest non-interference test runs identical synthetic depth frames
through fusion with diagnostics disabled and enabled. Raw/target range,
calibrator scale/offset, bearing-range filter state and EKF state are exactly
equal.

## Integrity

R1/R2/R3 locked inputs and all 64 source files were reverified by the R3.5
integrity checker. R3 and R3.5 manifests retained their prior SHA-256. A
process audit found no PX4, Gazebo, backend or range-training process.

Repository root Git metadata is unavailable, so R3.6A identity is frozen by
SHA-256 in `r3_6a_manifest.json`.

## Known limitations

- No R3.6A runtime dataset was collected; the sidecar contract and synthetic
  path are tested, but real CameraInfo/extrinsic availability awaits a future
  explicitly approved collection.
- Sensor-capture timestamp, verified optical-center transform, target visible
  surface offset, distortion coefficients and rectification state remain
  unavailable in the current source interfaces and are recorded as such.
- The collector is fail-soft to preserve runtime semantics. A rejected
  diagnostics row is observable in collector status but does not block a range
  measurement.
- JSONL is append-only rather than one atomic file transaction; per-record
  checksums support later compaction/integrity audit.

## Rollback

Remove `range_physical_diagnostics.py` and its test, remove the optional
diagnostic fields/wiring in `m52_adapter.py`, `depth_worker.py`,
`metric_target_fusion.py` and `tracking_web.py`, then remove only the R3.6A
report/artifact directory. No dataset, model, calibration or controller state
requires restoration.

## Decision

R3.6A is complete for review. This is not approval of the quantity contract or
calibration behavior, and it is not a model or promotion PASS. Any collection,
quantity-contract fix, calibration fix, R4 or R5 requires a separate approval.

