# CORE_RANGE full-stack contention fix

Date: 2026-08-05  
Conclusion: `FULL_STACK_CONTENTION_FIXED_READY_FOR_DYNAMIC_RECOLLECTION`

## Scope and representative baseline

No model training, corpus recollection, shadow/active residual model, Follow
Target, vehicle command, geometry, calibration, EKF, controller, or PX4 change
was made.  The valid pre-fix approaching run used two UAVs, an active tracker,
32 seconds of motion, stable calibration, and 50 `raw_range_computed` rows.
Live API sampling measured 5.7 tracking FPS with a 24.3 FPS camera source.
Capture-to-consume was 1395.2 ms median / 1823.7 ms P95; the depth worker was
748.3 / 1077.6 ms and queue wait was 227.5 / 585.6 ms.  An earlier relative-path
attempt and a Gazebo GUI startup crash are explicitly INVALID and excluded.

## Root cause

`GazeboDashboardBridge.simulation_target_ground_truth()` acquired the pose
history lock, deep-copied every one of the 600 retained Gazebo pose snapshots,
and scanned the copy twice for every tracking frame.  Large Python object-graph
copies held the GIL for roughly 150--250 ms per frame.  That directly reduced
tracking throughput and starved the MiDaS worker thread even though isolated
GPU inference remained fast.  The classification is
`SHARED_PROCESS_GIL_OR_SCHEDULING_DOMINANT`.

The patch performs one ordered bracket search under the existing lock and
deep-copies only the one or two selected records.  Timestamp gates, nearest
fallback, interpolation, geometry and ground-truth semantics are unchanged.
A regression test with 600 synthetic records verifies two copies and the exact
interpolated result.

The single-factor repeat improved tracking API median from 5.7 to 18.05 FPS
(the camera source itself was 18.8 FPS), capture-to-consume median from 1395.2
to 70.6 ms (-94.9%), worker median from 748.3 to 35.5 ms (-95.3%), and preserved
raw availability (50 to 51).  It therefore passes all precommitted dominance
conditions.

## Stage timing and ablations

Monotonic timestamps now cover receipt, submit, queue, frame copy, H2D,
preprocess, forward, output resize, D2H, NumPy postprocess, result publication,
consumer receipt, consume and diagnostic preparation.  Profiling synchronization
is behind `SWARM_DEPTH_PROFILE_STAGES` and remains off by default.  Tracking
status also exposes ground-truth provider, overlay preparation and complete
main-loop duration.

Post-fix single-variable ablations disabling preview/WebSocket publication,
the MAVLink consumer, and runtime log writes remained in the same fast latency
band and preserved 46--64 raw rows.  Physical-diagnostics-off was INVALID
because the observation-only capture harness uses a sidecar session as its
calibration/readiness gate; it is excluded rather than misrepresented.  The
measured diagnostic preparation was only about 10--14 ms median and overlay
preparation below 1 ms.  A separate depth process and thread-pool changes were
not justified after the pose lookup alone passed the dominance gate.

## Representative smoke and rate selection

All smoke sessions used two UAVs, an active tracker, stable calibration, at
least 30 seconds, and observation-only independent Gazebo target motion:

| Scenario | Rate | Raw | Tracking/frame throughput | Capture→consume med/P95 | Worker P95 |
|---|---:|---:|---:|---:|---:|
| approaching | 4 Hz | 113 | 26.99 FPS | 60.0 / 130.9 ms | 50.1 ms |
| receding | 3 Hz | 89 | 27.78 FPS | 70.8 / 151.9 ms | 80.8 ms |
| stop-and-hold | 4 Hz | 101 | 28.46 FPS | 56.8 / 135.2 ms | 47.9 ms |

The finite-window approaching rate is source-limited; live API sampling gave a
tracking/source ratio of 96%, within the allowed 20% depth-off loss alternative.
All rows pass timestamp ordering and checksum integrity, and no growing queue,
stale/nonfinite anomaly, crash, arm, or Follow endpoint was observed.

The corrected post-fix sweep used the dataset scheduler knob (an initial set of
runs labeled through the downstream metric knob was detected, marked
mislabeled, and excluded). At 3/4/5 Hz, capture P95 was 130.9--152.0 ms and
worker P95 48.4--80.8 ms. Five hertz passed with 141 raw rows and 27.29 FPS.
The 7.5 Hz run failed the stable-calibration prewarm gate and is INVALID rather
than being used to select a default. Therefore 5 Hz is the highest fully valid
rate and the safe default changes from 2 to 5 Hz; no gate was lowered.

## Verification and artifacts

Focused tests: 52 passed.  The complete evidence bundle is under
`artifacts/core_range_3_12m/full_stack_contention_fix/`, including invalid-run
annotations, stage distributions, ablations, resources, source hashes, smokes,
rate sweep, non-interference record and manifest.  Full repository tests and
`./run_all.sh --check` are recorded in the manifest after final execution.
