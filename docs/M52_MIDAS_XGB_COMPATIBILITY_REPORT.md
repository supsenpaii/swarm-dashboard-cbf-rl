# M52 + MiDaS + XGBoost Safe Follow — Phase 0 Compatibility Report

Date: 2026-08-02  
Scope: read-only code/config/test audit; no application source changed  
Decision: **C — core integration is feasible, but several control and data-contract paths require focused refactoring**  
Gate: **NO-GO for control/runtime feature patches; GO only for completing Phase 0 evidence and preparing a reviewed safety/instrumentation patch**

## 1. Executive conclusion

The repository already contains substantial reusable implementation: camera ingestion, hybrid tracking, bbox filtering, timestamped vehicle/camera pose buffers, raw MiDaS inference in a latest-frame-wins worker, M52 ground anchors, robust affine inverse-depth calibration, target ROI extraction, a 6-state NED EKF, local NED-to-WGS84 conversion, MAVLink `FOLLOW_TARGET`, a workflow machine, and extensive unit tests.

The current runtime is not compliant with the new safe-follow prompt in four critical areas:

1. `mavlink_manual_bridge.py` writes PX4 parameters before native Follow, including `FLW_TGT_*`, yaw tuning, and (by default) SITL EKF parameters. The new prompt requires read-only PX4 parameters.
2. The current workflow can use OFFBOARD/apparent-size or bootstrap motion before native Follow. The new target pipeline allows only `FOLLOW_TARGET` plus native Follow entry/exit commands as its control output.
3. A provisional target is constructed from the user-selected desired distance and can be auto-authorized/streamed before a metric estimate is proven. The new prompt forbids a virtual/fake target and requires explicit user action after readiness.
4. There is no deterministic replay/event schema, no XGBoost/StandardScaler/OOD/CI implementation, and no dataset sufficient for grouped XGBoost training.

These are evidence-backed static findings. They are plausible contributors to handover discontinuity, but the mode-entry jerk root cause is **not proven** because the PX4/Gazebo stack was not running and no replayable incident session was found.

## 2. Repository inventory

| Area | Current implementation |
|---|---|
| Primary language | Python 3.12 |
| Web framework | FastAPI/Uvicorn |
| Real-time integration | Gazebo Transport callbacks, ROS/PX4 telemetry through MQTT, separate MAVLink bridge |
| Main entry points | `main.py`, `mavlink_manual_bridge.py`, `run_all.sh` |
| Tracking loop | `TrackingManager._run()` / `_process_frame()` in `tracking_web.py` |
| Tracker | Hybrid package tracker with OpenCV fallback; bbox filter remains local |
| Camera capture | `GazeboDashboardBridge._handle_camera_image()` in `main.py` |
| Vehicle pose | MQTT fast-pose and telemetry callbacks feeding `TimestampedPoseBuffer` |
| Camera/gimbal pose | Gazebo camera IMU feeding a second `TimestampedPoseBuffer` |
| Gimbal control | Tracking controller and manual/home handlers publishing Gazebo joint commands |
| Metric depth | Local MiDaS-small adapter plus a one-slot async worker |
| Geometry/fusion | `CameraRayProjector`, M52 anchors, calibration, ROI extraction, bearing/range filters, EKF |
| Global beacon | `ned_target_to_wgs84()` then MQTT to `MavlinkWorker.send_visual_follow_target()` |
| Native mode request | `MAV_CMD_DO_SET_MODE`, ACK tracking, heartbeat main/sub-mode observation |
| Configuration | Environment variables; `run_all.sh` loads `.env` if present, otherwise `.env.example` |
| Logging | Human-readable process logs and runtime status dictionaries; no versioned per-event estimator log |
| Replay | No deterministic estimator replay implementation found |
| Tests | Pytest/unittest-style `test_*.py`; 187 tests passed during this audit |

Thread/process outline:

```text
Gazebo callback threads
  ├─ camera frame → latest raw frame slot
  ├─ camera/body IMU → pose buffers
  └─ world pose → optional simulation ground truth buffer

TrackingManager thread
  └─ tracker → bbox/gimbal → metric/bearing estimators → target beacon payload

LatestDepthWorker thread
  └─ one pending MiDaS frame, newest submission replaces older pending frame

MavlinkWorker per UAV
  └─ telemetry receive + MQTT state + 20 Hz command/beacon loop
```

## 3. Actual current call path

```text
Gazebo RGB image
  main.GazeboDashboardBridge._handle_camera_image
    → raw_frames/raw_frame_versions
    → TrackingManager._run
    → TrackingManager._process_frame
    → tracker.update
    → VisualBBoxFilter
    → TrackingManager._update_gimbal_locked
       ├─ command_tracking_gimbal → Gazebo gimbal joint publishers
       └─ TrackingManager._update_visual_follow_locked
          ├─ pose_provider(current_source_timestamp_s)
          │  → tracking_visual_pose
          │  → TimestampedPoseBuffer.sample_at for vehicle and camera
          ├─ MetricTargetFusion.update
          │  ├─ bearing filter / EKF bearing update
          │  ├─ LatestDepthWorker.submit
          │  └─ completed depth result
          │     → M52GroundAnchorAdapter.anchors
          │     → MetricDepthCalibrator.fit
          │     → TargetDepthExtractor.extract
          │     → robust inverse-depth/range filters
          │     → TargetFusionEKF.update_position
          ├─ optional passive multi-view candidate into the same EKF
          ├─ provisional/metric source selection and handover
          ├─ ned_target_to_wgs84
          └─ command_visual_follow_target
             → MQTT visual_follow_target
             → VisualFollowTargetState.update
             → MavlinkWorker 20 Hz loop
                → configure_visual_follow_parameters (currently writes PX4)
                → FOLLOW_TARGET send
                → MAV_CMD_DO_SET_MODE AUTO/FOLLOW
                → COMMAND_ACK + heartbeat main/sub-mode observation
```

Exit currently disables the visual target state and `request_position_mode()` requests PX4 Position mode. Follow-entry ACK is tracked, but an explicit, independently reported exit ACK/nav-state state machine is not complete to the new contract.

## 4. Compatibility matrix

| Block | Status | Current code | Required work | Risk |
|---|---|---|---|---|
| Tracker | REUSE | `TrackingManager`, hybrid/OpenCV trackers | Preserve behavior; expose a stable observation adapter | Low |
| `TargetObservation` | MISSING | Fields exist across manager/result state | Add versioned immutable contract with session/track ID, frame ID, capture clock and bbox coordinate space | High |
| Frame/session association | ADAPT | Tracking thread checks manager session/epoch; depth job has frame index/generation | Put session/track ID in depth context/result and reject cross-session/cross-bbox results explicitly | High |
| Pose synchronization | ADAPT | Linear interpolation + quaternion SLERP + stale rejection | Preserve; distinguish sensor capture time from callback receipt time and quantify clock offsets | High |
| Camera calibration | ADAPT | Width/FOV-derived `fx`; `fy=fx`; optional overrides and two profiles | Add full intrinsics/profile ID, pixel-center convention, resolution scaling, distortion contract and fail-closed profile mismatch | High |
| Camera/gimbal transform | ADAPT | Camera IMU quaternion, body-FRD offset rotated into NED | Verify optical-axis convention in SITL and measure camera/gimbal time offset; document extrinsics | High |
| M52 ground anchors | REUSE | 12x8 grid, target exclusion, lower-image gate, local depth consistency, coverage metrics | Add mask exclusion and stronger geometry/ground-plane uncertainty contract | Medium |
| M52 target candidate | MISSING | Ground mode only constrains the depth-derived candidate's D coordinate | Implement independent bottom-contact ray/ground candidate with covariance and reason codes | High |
| MiDaS raw inference | ADAPT | Raw float inverse-depth output, local-only model, newest-frame worker | Verify square-resize/aspect geometry, preserve raw contract, add schema/model ID and session association | Medium/High |
| Metric calibration | REUSE/ADAPT | RANSAC + least squares, temporal consensus, covariance, change-point handling, marked cache | Benchmark Huber; formalize valid-fraction/coverage gates and cached-calibration semantics | Medium |
| Target ROI depth | ADAPT | Eroded bbox, robust foreground half, median/MAD | Add mask path, border/occlusion features, explicit foreground fraction and covariance components | Medium |
| Candidate contracts | MISSING | One combined MiDaS+M52 estimate and one multiview cross-check | Emit independent M52 and MiDaS `TargetCandidate` objects before fusion | High |
| Rule baseline | ADAPT | Quality gates, disagreement checks and conservative filters exist | Make source selection explicit and replay-comparable; no implicit provisional fallback | Medium/High |
| EKF | REUSE/ADAPT | 6-state NED, variable dt, Joseph covariance update, dropout timeout, stationary/CV mode | Add Mahalanobis/NIS gate, bounded adaptive process noise, session ID reset evidence and send-time prediction contract | High |
| WGS84/AMSL | ADAPT | Local tangent conversion relative to synchronized follower global position | Add reference epoch/validity, round-trip tests over operational range, altitude-source proof and origin-reset handling | High |
| `FOLLOW_TARGET` sender | ADAPT | 20 Hz sender, POS/VEL capability selection, finite payload validation | Add no-zero/no-stale gate, duplicate/order guard, actual gap distribution and spec-level field inspection | High |
| Follow workflow | ADAPT | Explicit machine and tests exist | Remove auto authorization/provisional beacon, align new states/guards/timeouts, add explicit safe-reference/entry geometry | Critical |
| PX4 parameter handling | REPLACE | Runtime uses `PARAM_SET` for Follow/yaw/EKF settings | Replace with read-only request/cache/validation; never write under safe pipeline | Critical |
| Loss/exit | ADAPT | Grace then disable/Position request | Add idempotent exit request, ACK timeout and actual safe-nav-state confirmation; prohibit auto re-entry | Critical |
| Gimbal arbitration | MISSING | App writers are locally arbitrated; PX4/QGC native writers are not | Observe/audit PX4 native Follow gimbal output and define one authority at a time | Critical |
| Structured logging | MISSING | Human logs/status dictionaries only | Add versioned JSONL event schema and bounded writer | High |
| Deterministic replay | MISSING | No replay runner/fixture found | Add schema-validated replay with current/M52/MiDaS/baseline/ML/CI variants | High |
| Ground-truth dataset | MISSING | One old full-frame depth CSV/JSON; simulation GT helper has no production caller | Collect synchronized target-position sessions and group manifests | Critical for ML |
| StandardScaler/OOD | MISSING | No implementation/dependency | Add only after feature/dataset gate | Medium |
| XGBoost q90 | MISSING | Design documents only; package absent | Train/tune two mandatory grouped models after dataset gate | High |
| Covariance Intersection | MISSING | No implementation | Add offline then shadow after covariance calibration | High |

Overall compatibility level: **C**. The estimator foundation is reusable, but the safe control boundary and candidate/data contracts need focused refactoring.

## 5. Frame, clock, geometry and altitude findings

### Confirmed from source

- Geometry and estimator state are predominantly NED.
- Tracking frames are tagged with `time.monotonic()` when the converted Gazebo frame is stored.
- Vehicle telemetry and camera IMU samples are also stamped on callback receipt using `time.monotonic()`.
- `TimestampedPoseBuffer` rejects out-of-order samples, interpolates position/velocity linearly and attitude by shortest-path SLERP.
- Camera profile fallback is 480x270, horizontal FOV 1.35 rad when `.env.example` is loaded. No `.env` file exists, so `run_all.sh` currently falls back to `.env.example`.
- M52 ground plane defaults to local NED `D=0`, described as PX4 home plane.
- WGS84 altitude is calculated as follower AMSL minus target relative Down offset.

### Unresolved and gate-blocking

- Frame timestamp is callback/processing arrival, not proven sensor capture time; Gazebo simulation timestamp is retained separately but is not the estimator clock.
- Vehicle and camera samples share receipt-monotonic time, not proven source-time synchronization. Transport delay and fixed time offsets are unmeasured.
- Full camera intrinsics, principal point, distortion and pixel-center convention are not calibrated artifacts.
- `fy` defaults to the horizontal-FOV-derived `fx`; this is only valid under a specific pixel/aspect model and must be verified.
- Ground `D=0` is assumed, not validated as AGL/terrain truth.
- PX4 local/global origin and AMSL conventions have not been verified against the running version.
- Camera optical axes are documented in code, but comments elsewhere in the repository historically differ; SITL ray/corner/ground-intersection evidence is required.

## 6. Current control-path conflicts with the new prompt

### 6.1 PX4 parameter writes

`MavlinkWorker.configure_visual_follow_parameters()` sends `PARAM_SET` for:

- `FLW_TGT_ALT_M`
- `FLW_TGT_DST`
- `FLW_TGT_HT`
- `FLW_TGT_FA`
- `FLW_TGT_RS`
- `FLW_TGT_MAX_VEL`
- `MPC_YAWRAUTO_MAX`
- `MPC_YAWRAUTO_ACC`

Additionally, `SWARM_PX4_SINGLE_EKF_ENABLED` defaults true and writes `EKF2_MULTI_IMU` and `EKF2_MULTI_MAG` in SITL. Both paths violate the prompt's no-parameter-write rule.

### 6.2 OFFBOARD and provisional target

- `.env.example` enables tracking OFFBOARD and apparent-size control.
- Relative visual servo and RGB bootstrap can publish OFFBOARD motion before the native handover.
- `_initialize_provisional_target_locked()` places a target on the selection ray using desired/safe distance, not an observed metric range.
- `SWARM_VISUAL_FOLLOW_AUTO_START=true` by default, and the workflow can auto-authorize the selection anchor.

This does not satisfy the new requirement that a real, valid target beacon and entry gate precede explicit user action.

### 6.3 Gimbal authority

Known application writers are:

- automatic tracking via `command_tracking_gimbal()`;
- start/stop home commands via `publish_gimbal_home()`;
- manual websocket gimbal step/home handlers, which are blocked while tracking owns the gimbal;
- QGroundControl traffic forwarded through the MAVLink proxy, which can introduce an external command source.

No repository-side arbitration was found for gimbal commands that PX4 native Follow may produce. This must be observed on the actual PX4 version in SITL.

## 7. Dependency and runtime audit

Environment observed:

| Item | Result |
|---|---|
| Python | 3.12.3 |
| CPU | Intel Core i7-13700H, 20 logical CPUs |
| GPU | NVIDIA driver unavailable to `nvidia-smi`; usable CUDA status not proven |
| MiDaS repository/checkpoint | Present locally; checkpoint about 85.8 MB |
| numpy | 2.4.4 |
| OpenCV contrib | 5.0.0.93 |
| scipy | 1.18.0 |
| torch | 2.11.0+cu130 |
| pandas | 3.0.3 |
| pymavlink | 2.4.49 |
| pyproj | Missing |
| xgboost | Missing |
| scikit-learn | Missing |
| joblib | Missing |
| pytest in project venv | Missing; system pytest 7.4.4 is available |

No dependency was installed or upgraded. Installing XGBoost/scikit-learn/joblib later requires explicit user approval under the prompt.

Static configured rates:

- Camera profile: 50 Hz, 480x270, horizontal FOV 1.35 rad.
- MiDaS submission target: 7.5 Hz.
- MAVLink worker loop and `FOLLOW_TARGET`: 20 Hz nominal.
- Gimbal command: 40 Hz in `.env.example`.

Actual camera, telemetry, gimbal, MiDaS and beacon rate/latency distributions are **UNKNOWN** because the stack was not running.

## 8. Logging, replay and dataset evidence

- No versioned JSONL estimator event stream was found.
- No deterministic replay runner or fixed replay fixture was found.
- Existing `artifacts/metric_depth_ground_truth_full_frame_20260729.csv` and its evaluation JSON concern an older/full-frame depth evaluation and are not a grouped target-candidate training dataset.
- `simulation_target_ground_truth()` exists in `main.py`, but no active estimator/dataset caller was found.
- No XGBoost model bundle, scaler, feature schema, split manifest or CI artifact exists.
- No replayable mode-entry jerk session was found in current artifacts.

Therefore Phase 1 replay and Phase 9 dataset gates are not met.

## 9. Test baseline

Command used without bytecode or pytest cache writes:

```text
env PYTHONPATH=/home/sup/swarm_dashboard/.venv/lib/python3.12/site-packages \
    PYTHONDONTWRITEBYTECODE=1 PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
    pytest -p no:cacheprovider -q test_*.py
```

Result:

```text
187 passed, 1 warning in 1.42 s
```

The warning is `PytestReturnNotNoneWarning` for `test_bearing_target_estimator.py::test_config`. These tests establish a useful static baseline but do not prove runtime timing, coordinate conventions, PX4 parameter behavior, gimbal arbitration or safe handover.

## 10. Phase 0 gate assessment

| Required gate | Result |
|---|---|
| Call path identified | PASS (static) |
| Clock domains fully determined | FAIL — receipt time versus capture/source time unresolved |
| Camera intrinsics/profile verified | FAIL — config exists, calibration evidence absent |
| Frame/axis signs verified | PARTIAL — unit tests pass, SITL known-offset evidence absent |
| Altitude/AMSL convention verified | FAIL — static mapping only |
| `FOLLOW_TARGET` mapping inspected | PASS (static), runtime packet inspection not run |
| PX4 version and read-only `FLW_TGT_*` recorded | NOT RUN — PX4 stack inactive |
| Gimbal writers identified | PARTIAL — app writers found, PX4 native output not observed |
| Jerk reproduced/measured | NOT RUN |
| Replay session available | FAIL |
| Baseline tests | PASS — 187 tests |

Phase 0 outcome: **NO-GO**. Do not start estimator promotion, ML runtime, mode-command changes, or SITL acceptance claims yet.

## 11. Evidence required to close Phase 0

1. Start the controlled SITL stack with all native-follow control flags forced safe/off for observation.
2. Record exact PX4 version/commit and read (never write) `FLW_TGT_DST`, `FLW_TGT_HT`, `FLW_TGT_FA`, `FLW_TGT_ALT_M`, `FLW_TGT_MAX_VEL`, `FLW_TGT_RS`.
3. Capture camera sensor timestamp, callback monotonic timestamp, vehicle pose sample times and camera IMU sample times; quantify offsets and age distributions.
4. Validate center/corner rays and known N/E ground intersections against Gazebo truth.
5. Inspect outgoing `FOLLOW_TARGET` fields and measured rate/gaps without requesting Follow mode.
6. Record a replayable regression session containing the existing mode-entry discontinuity, PX4 setpoints, vehicle response and bbox retention.
7. Observe whether native PX4 Follow publishes gimbal/mount commands and prove authority behavior.

## 12. Proposed reviewed patch sequence

No patch should be applied until the user reviews this report.

### P1A — safety interlocks and read-only PX4 contract

Planned files:

```text
mavlink_manual_bridge.py
- prohibit PARAM_SET in the safe native-follow path;
- replace Follow parameter configuration with read-only request/cache/validation;
- keep mode requests disabled by default behind a dedicated feature flag;
- add explicit entry/exit ACK and observed-nav-state status.

tracking_web.py
- disable auto authorization and provisional target beacon for the safe path;
- prevent OFFBOARD/apparent-size/bootstrap from acting as the native pipeline backend;
- preserve tracker and gimbal behavior.

.env.example
- safe defaults: no automatic mode request, no provisional beacon, no PX4 writes.

tests
- assert no PARAM_SET, no auto mode entry, no provisional/zero/stale beacon.
```

Rollback: feature flags retain the legacy path only for separately authorized regression comparison; safe path remains default-off until SITL.

### P1B — schemas, structured logging and replay skeleton

Candidate new files (names subject to repository convention review):

```text
target_pipeline_contracts.py
structured_target_log.py
replay_target_pipeline.py
test_target_pipeline_contracts.py
test_target_replay.py
```

Adapted files:

```text
tracking_web.py
metric_target_fusion.py
mavlink_manual_bridge.py
main.py
```

Invariants:

- tracker and gimbal output remain unchanged;
- logging is bounded and feature-flagged;
- ground truth is evaluation-only;
- schema mismatches fail clearly;
- replay never uses future information;
- safe-control flags remain off.

Acceptance gate:

- deterministic replay on one fixed session;
- session/frame/clock schema enforced;
- current/M52-only/MiDaS-only/baseline variants run on the same events;
- no PX4 mode request or parameter write.

## 13. Go/No-Go

**NO-GO for implementation beyond Phase 0.**

Recommended next action: obtain the runtime/SITL evidence in Section 11, then review P1A and P1B. XGBoost remains mandatory, but training is blocked until independent M52/MiDaS candidates, deterministic replay, synchronized ground truth and grouped dataset manifests exist.

