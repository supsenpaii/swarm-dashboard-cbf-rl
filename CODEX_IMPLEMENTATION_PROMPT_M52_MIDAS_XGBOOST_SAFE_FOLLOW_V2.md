


Bạn là kỹ sư Robot AI cao cấp chịu trách nhiệm nâng cấp visual target geolocation và PX4 native Follow Target trong repository `swarm_dashboard`.

Hãy triển khai thực tế theo từng patch nhỏ, có test, replay, metrics và Go/No-Go. Không chỉ viết kế hoạch, nhưng không được vượt acceptance gate hoặc tự mở rộng quyền hạn.

Đọc đầy đủ trước:

```text
docs/M52_MIDAS_XGB_COMPATIBILITY_REPORT.md
CODEX_AUDIT_IMPLEMENTATION_PLAN_M52_MIDAS_XGBOOST.md
CODEX_IMPLEMENTATION_PROMPT_M52_MIDAS_XGBOOST_SAFE_FOLLOW.md
```

Repository có `.codegraph/`; bắt buộc dùng CodeGraph trước `rg`, `grep`, `find` hoặc đọc lan man khi tìm symbol/call path. Xác minh lại source hiện tại vì code có thể đã thay đổi.

## 1. Mục tiêu cuối

Giữ tracker và gimbal tracking hiện tại. Xây pipeline:

```text
RGB + sensor capture timestamp
→ bbox/mask + session/track/frame ID
→ synchronized vehicle/camera pose
→ M52 ground anchors + independent ground-contact candidate
→ MiDaS raw inverse depth metricized bằng M52 anchors
→ independent MiDaS candidate
→ rule-based hard gates
→ mandatory XGBoost q90 + OOD reliability
→ conservative source selection hoặc Covariance Intersection
→ Target EKF [N,E,D,vN,vE,vD] + covariance
→ bounded send-time prediction
→ NED → WGS84 + AMSL
→ MAVLink FOLLOW_TARGET
→ PX4 native Follow Target
```

Ứng dụng chỉ ước lượng/gửi target beacon và yêu cầu vào/thoát mode. PX4 tự tạo flight setpoint và điều khiển drone.

## 2. Trạng thái hiện tại đã audit

Phải coi đây là baseline ban đầu:

- Có tracker, bbox filtering và gimbal tracking cần tái sử dụng.
- Có pose buffer với linear interpolation và quaternion SLERP.
- Có local MiDaS raw-float adapter và latest-frame-wins worker.
- Có M52 ground anchors, affine inverse-depth calibration, ROI extraction và EKF NED 6 trạng thái.
- Có NED-to-WGS84 và `FOLLOW_TARGET` sender.
- Chưa có independent M52 target candidate đúng contract.
- Chưa có versioned structured logging/deterministic replay.
- Chưa có StandardScaler/OOD/XGBoost bundle/CI runtime.
- Chưa có grouped target dataset đủ để train.
- `xgboost`, `scikit-learn`, `joblib` chưa có trong project environment tại thời điểm audit.
- Current bridge đang có đường ghi PX4 parameters.
- Current workflow còn OFFBOARD/apparent-size/bootstrap, provisional target và auto-start.
- PX4 version, read-only `FLW_TGT_*`, jerk regression và native gimbal authority chưa được xác minh runtime.

Không được bỏ qua hoặc che các xung đột này.

## 3. Ràng buộc tuyệt đối

### PX4 và control

- Không sửa PX4 source/firmware/module/build output.
- Không gửi `PARAM_SET` hoặc thay PX4 parameter bằng bất kỳ cơ chế nào.
- Không thay `FLW_TGT_*`, `MPC_*`, `EKF2_*`, tuning hoặc failsafe parameters.
- Chỉ đọc parameter/status để validation.
- Safe pipeline không dùng OFFBOARD, MANUAL_CONTROL motion, raw attitude hoặc body-rate setpoint làm backend target follow.
- Đầu ra control duy nhất: valid `FOLLOW_TARGET`, idempotent native Follow entry command và verified safe Position/Hold exit command.
- Neutral heartbeat chỉ được giữ nếu chứng minh không tạo motion và có test/log riêng.

### Target semantics

- Không gửi virtual/provisional/selection-anchor target cho PX4.
- Desired/current follow distance không phải measurement.
- Không dùng apparent bbox size làm metric beacon nếu chưa được calibration và validation riêng.
- Không dùng drone position/GPS làm target fallback.
- Không gửi `(0,0)`, NaN/Inf, stale target hoặc state từ session trước.
- Không dùng measurement cũ như measurement mới.
- Không tự động re-enter Follow sau target loss.

### Runtime safety

- Mọi feature mới có feature flag.
- Mode request thật mặc định off tới khi replay và SITL pass.
- ML/CI chạy offline rồi shadow trước primary.
- Không flight test thật trước full replay/SITL acceptance và user approval.
- Không xóa/ghi đè log, dataset, model.
- Không cài dependency/tải model/thay môi trường trước khi báo exact package/version/impact và được người dùng đồng ý.
- Không broad-exception/silent fallback để che lỗi.
- Không sửa/format file ngoài patch scope.

## 4. Canonical contracts

Giữ NED làm internal frame. Mọi timestamp có clock domain; mọi vector có unit/frame; mọi schema/log/dataset/model có version.

### `TargetObservation`

```text
schema_version
session_id
track_id
frame_index
capture_timestamp_s
capture_clock_domain
arrival_timestamp_s
bbox_xyxy_px
frame_width_px
frame_height_px
tracker_score
tracker_state
lost_frames
optional_mask
valid
reason_codes
```

### `CameraIntrinsics`

```text
schema_version
profile_id
width_px, height_px
fx_px, fy_px, cx_px, cy_px
distortion_model, distortion_coefficients
pixel_center_convention
valid, reason_codes
```

### `GroundPlaneEstimate`

```text
schema_version
timestamp_s
frame: NED
ground_normal_ned
ground_plane_offset_m
camera_height_agl_m
source
covariance
valid_region_ned
valid
reason_codes
```

Không âm thầm coi home plane là ground plane.

### `SynchronizedFrameContext`

```text
observation
vehicle_position/velocity/attitude_at_capture
camera_position/attitude_at_capture
camera_intrinsics
ground_plane
pose/camera interpolation status and age
estimated_clock_offset_s
sync_valid
reason_codes
```

### `TargetCandidate`

```text
schema_version
source: m52 | midas
session_id, track_id, frame_index
measurement_timestamp_s
position_ned_m
position_covariance_ned_m2
horizontal_range_m, slant_range_m
range_variance_m2
bearing_ned_unit
quality_features
age_s
valid
reason_codes
```

### `FusedTargetState`

```text
schema_version
session_id, track_id
state_timestamp_s
position_ned_m, velocity_ned_mps
position/velocity covariance
estimate_age_s
fusion_mode, source_health
ready_for_beacon, ready_for_follow
reason_codes
```

### `FollowTargetBeacon`

```text
schema_version
session_id
source_state_timestamp_s, send_timestamp_s
lat_deg_e7, lon_deg_e7, alt_amsl_m
velocity_ned_mps
position_covariance_or_accuracy
est_capabilities
valid
reason_codes
```

## 5. Workflow bắt buộc

```text
IDLE
→ TRACKING_ONLY
→ ESTIMATE_STABLE
→ SAFE_REFERENCE_LOCKED
→ READY_FOR_PX4_FOLLOW
→ FOLLOW_MODE_REQUESTED
→ PX4_FOLLOW_ACTIVE
→ FOLLOW_DEGRADED
→ EXIT_MODE_REQUESTED
→ REACQUIRE_REQUIRED
```

Mỗi transition có guard, entry/exit action, timeout, reason code, idempotency và test. Entry cần explicit user action. UI chỉ báo active sau ACK và observed native Follow nav state. Sau loss phải có full stability window và user action mới.

## 6. Thứ tự triển khai

### Phase 0B — Đóng runtime evidence

Không sửa thuật toán. Xác minh:

- Exact PX4 version/commit.
- Read-only `FLW_TGT_DST`, `HT`, `FA`, `ALT_M`, `MAX_VEL`, `RS`.
- Sensor capture time versus callback monotonic time.
- Vehicle/camera/gimbal delay và clock offset.
- Runtime resolution/FOV/intrinsics/profile.
- Optical axes, quaternion order, Gazebo ENU ↔ PX4 NED signs.
- Local/global origin và AMSL convention.
- `FOLLOW_TARGET` fields/rate.
- Current mode-entry discontinuity, setpoint response và gimbal writers.

Tạo replayable jerk-regression session. Nếu phải bật mode, báo trước và chỉ chạy SITL.

Gate: không còn clock/frame/altitude convention chưa xác định; có measured baseline và replay fixture.

### Phase 1 — Safety containment patch

- Safe path không gọi `param_set_send`.
- Thay PX4 configuration bằng read-only request/cache/validation.
- Dedicated mode-request feature flag mặc định false.
- Auto-start mặc định false.
- Provisional/selection-anchor state không thể vào `FOLLOW_TARGET`.
- OFFBOARD/apparent-size/bootstrap không thể làm backend safe native pipeline.
- Legacy regression path nếu giữ phải có tên/flag riêng, mặc định off và mutual exclusion với safe path.
- Test no-param-write, no-auto-mode, no-fake/zero/stale beacon.

Gate: tracker/gimbal không regression; safe flags off ngăn mọi mode request và parameter write.

### Phase 2 — Schemas, structured log và deterministic replay

Log versioned per observation/context/candidate/gate/EKF/beacon/send/mode/ACK/nav/gimbal event. Ground truth chỉ evaluation channel.

Replay modes:

```text
current-safe
M52-only
MiDaS-only
rule-based selection
XGBoost gated
CI
```

Gate: deterministic tolerance, reject schema mismatch, no future data, jerk fixture replay được.

### Phase 3 — Capture-time sync, calibration và ground plane

- Dùng sensor capture timestamp; estimate/log clock offset.
- Position interpolation + quaternion SLERP; reject stale/gap/extrapolation.
- Full intrinsics, runtime scaling, distortion and pixel-center validation.
- Center/corner/horizon/known-intersection tests.
- `GroundPlaneEstimate` với AGL/covariance/valid region.
- Inflate/reject khi angular rate, blur hoặc rolling-shutter risk cao.

Gate: geometry correct trong synthetic + SITL; mismatch fail closed; AGL/ground source verified.

### Phase 4 — Independent M52 và MiDaS candidates

M52:

- Reuse anchors; grid 12x8 tối đa 96, không dùng threshold 100.
- Exclude bbox/mask; coverage/geometry/ground uncertainty.
- Bottom-center/mask contact ray-ground candidate.
- Reject horizon/behind/negative/non-ground/ill-conditioned.
- Propagate pixel/contact/pose/intrinsics/ground covariance.

MiDaS:

- Raw float inverse depth only.
- Latest-frame-wins; reject session/track/frame mismatch.
- Verify aspect/intrinsics mapping.
- RANSAC + least squares for `1/Z=a*q+b`; benchmark Huber.
- Gate sign/inliers/coverage/condition/residual/drift/support.
- Mask ROI or erosion + robust foreground.
- Independent position/range covariance.

Gate: M52-only/MiDaS-only replay; no stale-as-new; errors/covariance evaluated by range/attitude/terrain.

### Phase 5 — Rule baseline, EKF và send-time prediction

- Tracker/session, sync, age, M52, MiDaS, disagreement and EKF NIS gates.
- One good source → conservative source selection.
- Both bad → finite predict-only then invalid.
- No simple average of correlated sources.
- Reuse 6-state NED EKF; add Mahalanobis/NIS gate, bounded adaptive noise, covariance floor, session reset and finite dropout.
- Do not keep velocity indefinitely.

```text
age = send_time - state_timestamp
p_send = p_state + v_state * bounded(age)
```

Inflate covariance with age; stale send-time state is invalid.

Gate: baseline not worse on held-out median/p90/p95, jump, velocity bias, stale behavior; no real mode command.

### Phase 6 — WGS84/AMSL và beacon shadow

- Known N/E and round-trip validation.
- Reference epoch; invalidate on origin/home reset.
- Verify AMSL/ellipsoid/relative altitude.
- 2D readiness independent of unverified vertical target, but altitude field finite/spec-correct.
- Sender order/duplicate/gap/burst protection.
- Shadow beacon first, no mode request.

Gate: no invalid/zero/stale beacon; packet mapping and measured rate/gaps pass.

### Phase 7 — Native Follow SITL handover/loss/gimbal

Read-only `FLW_TGT_*` drives entry-geometry prediction; current measured distance is not PX4 configuration.

Warm-up requires stable tracker/estimator/beacon, covariance/age/jump/OOD gates, vehicle readiness and gimbal authority.

```text
valid warm beacon
→ entry geometry pass
→ explicit user press
→ one mode command
→ ACK accepted
→ observed Follow nav state
→ UI active
```

Short loss: bounded prediction + covariance inflation. Long/unsafe loss: verified Position/Hold exit with ACK + observed safe nav state. No fake target or auto re-entry.

Audit PX4/QGC/application gimbal writers; one explicit authority only. Bbox retention is mandatory.

Gate: no jerk/spike regression; bounded first setpoint deltas; safe exit time; bbox retention pass.

### Phase 8 — Dataset cho mandatory XGBoost

Only after replay and rule baseline pass. Collect grouped sessions across range, target/drone motion, bbox/occlusion, camera attitude, terrain, source failure combinations, session/location/lighting.

Labels: per-source horizontal/3D error, q90-compatible target, ground contact if applicable. Never runtime-use ground truth, future state/frame or group IDs.

Split by `flight/session/location`, never random frame split.

Gate: data dictionary, schema/version, split manifest, no leakage, adequate failure regimes. Insufficient data → collect more, do not train for show.

### Phase 9 — Mandatory XGBoost train/tune/calibrate

Train at least:

1. M52 error/q90 model.
2. MiDaS error/q90 model.
3. Ground classifier if use case contains ground and non-ground targets.

Before dependency installation, request approval with exact packages/versions.

- Lock grouped test split before tuning.
- Grouped validation/CV for features, hyperparameters, early stopping and calibration.
- Tune M52/MiDaS independently.
- Version search space; record trials/seeds.
- Tune learning rate, trees/early-stop, depth/leaves, child weight, row/column sampling and regularization.
- Quantile/q90-compatible objective for verified XGBoost version.
- Select on q90 coverage, p90/p95 accepted error, false accept/reject, jump, OOD and latency—not RMSE alone.
- Evaluate per source/range/attitude/failure regime.
- Validation/conformal q90 calibration allowed; bundle artifact.
- Prefer smaller/faster model when equivalent.
- Locked test only after final selection.

Tree models use raw continuous features by default. StandardScaler is mandatory for schema/OOD z-score monitoring and is fit on training only. Do not scale booleans/enums/reason masks/timestamps/images/NED/WGS84/beacon fields.

Bundle must contain schema/order/types, missing policy, preprocessing/scaler, models/calibrators, split manifest, metrics/coverage/OOD statistics, versions and checksums. Runtime fails closed on mismatch/NaN/Inf/severe OOD.

Gate: mandatory models reproducibly trained/tuned/calibrated; held-out q90 coverage and tail/gating improvement. If not better, continue offline/shadow tuning; do not unsafe-promote.

### Phase 10 — XGBoost shadow và CI

ML may only calibrate/inflate covariance, gate sources, determine M52 applicability, detect OOD and support conservative selection/CI. It must not output position/GPS or mode command.

CI only for valid, calibrated, age-valid, non-OOD, semantically compatible candidates below disagreement hard gate. Inflate for q90, age, OOD and correlation risk.

Promotion: offline → runtime shadow → held-out → SITL → controlled primary.

Gate: better tail/jump, consistent covariance, no overconfidence.

### Phase 11 — UI và controlled promotion

Show tracker/session, M52/MiDaS health, uncertainty, age, XGBoost/OOD/CI mode, beacon warm/rate/gaps, entry reason, requested mode, ACK, observed nav state and Follow disabled reason.

Follow button idempotent and disabled while pending. Primary promotion needs replay + held-out + SITL report and explicit user approval.

## 7. Mandatory tests and metrics

Tests include:

- session/frame/depth association;
- pixel ray, intrinsics scaling, transforms, ground/AGL;
- SLERP/clock offset/stale rejection;
- M52/MiDaS calibration/covariance/invalid cases;
- disagreement/OOD/quality gates;
- EKF NIS/reset/predict timeout/send-time compensation;
- WGS84 round trip/origin reset/AMSL;
- MAVLink mapping/rate/order/gaps;
- no PX4 parameter write;
- workflow ACK/nav/timeouts/loss/no-auto-reentry;
- model/scaler schema/checksum;
- worker/event-loop stalls.

SITL matrix includes static/moving/turning target, valid/invalid entry, bbox edge/occlusion/loss, MiDaS/pose stale, source disagreement/OOD, WGS84 outlier, stream interruption, mode/exit reject-timeout, origin reset and gimbal conflict.

Metrics:

- range and NED error median/p90/p95/max;
- velocity bias; stationary jitter/drift;
- q90 and covariance coverage by regime; NIS/NEES when truth exists;
- latency/age;
- WGS84 continuity, jumps, rate/p95 gaps/drops/duplicates;
- first PX4 setpoint deltas, vehicle peak speed/acceleration, overshoot/settling, safe-exit time and bbox retention.

Every threshold has unit, config/default and replay/SITL evidence. No magic numbers.

## 8. Patch/report protocol

Before patch:

```text
Phase/Patch:
Files planned:
Evidence:
Invariants:
Safety impact:
Migration/rollback:
Acceptance gate:
```

After patch:

```text
Outcome:
Files changed:
Tests and exact results:
Metrics before/after:
Safety checks:
Known limitations:
Go/No-Go:
Next patch:
```

If not run, say `not run` and why.

Patch order:

```text
P0B runtime evidence + jerk fixture
P1A safety containment/read-only PX4
P1B schemas + logs + replay
P2 time/camera/ground geometry
P3 independent candidates
P4 rule baseline + EKF + send prediction
P5 WGS84/beacon shadow
P6 native Follow SITL handover/loss/gimbal
P7 dataset/group splits
P8 scaler/OOD + XGBoost training bundle
P9 XGBoost shadow
P10 CI shadow
P11 UI + controlled promotion + regression
```

## 9. Stop conditions

Dừng và xin ý kiến nếu cần sửa PX4, ghi parameter, cài dependency/tải model, bật mode ngoài SITL, hoặc nếu clock/intrinsics/frame/altitude không xác minh được, replay/data thiếu, gate fail, command unsupported, gimbal conflict, jerk/spike/bbox regression, hay read-only `FLW_TGT_*` mâu thuẫn safe geometry.

Khi dừng: cung cấp evidence, impact, 2–3 options và recommended option.

## 10. Definition of Done

- Tracker/gimbal không regression.
- Session/frame/capture timestamp đúng.
- Camera/transform/AGL/altitude conventions tested.
- Structured log/replay deterministic.
- Independent M52/MiDaS candidates with covariance/reasons.
- Rule baseline validated; EKF/NIS/send prediction validated.
- WGS84 beacon correct, continuous, never invalid/stale/zero.
- No PX4 parameter write in safe path.
- No OFFBOARD/provisional target backend.
- Entry only after warm-up, gate, explicit user action, ACK and observed state.
- Loss has bounded verified exit and no auto re-entry.
- No gimbal dual control; bbox retention pass.
- Mandatory two XGBoost q90 models trained/tuned/calibrated/bundled reproducibly.
- XGBoost/CI promoted only after held-out/replay/SITL improvement.
- No PX4 modification or unauthorized real flight test.

## 11. Bắt đầu

1. Đọc compatibility report và source bằng CodeGraph.
2. Xác minh static findings, không audit lại lan man.
3. Trình kế hoạch Phase 0B và P1A.
4. Thu mọi Phase 0B evidence có thể bằng read-only checks.
5. Runtime/SITL không chạy thì ghi `not run`, không suy đoán pass.
6. Không sửa code trước khi user review Phase 0B gate.
7. Sau khi duyệt, triển khai P1A trước mọi estimator/ML feature khác.


