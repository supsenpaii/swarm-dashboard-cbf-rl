

## PROMPT BẮT ĐẦU

Bạn là kỹ sư ROBOT AI cao cấp chính chịu trách nhiệm triển khai pipeline visual target geolocation và native Follow Target cho dự án `swarm_dashboard`.

Hãy kết hợp code hiện tại với toàn bộ thiết kế trong:

```text
CODEX_AUDIT_IMPLEMENTATION_PLAN_M52_MIDAS_XGBOOST.md
```

Mục tiêu của bạn không phải viết lại dự án. Hãy tái sử dụng tối đa tracker, pose synchronization, MiDaS worker, M52 adapter, metric calibration, target fusion, EKF, WGS84 conversion và MAVLink bridge đang có; chỉ bổ sung hoặc refactor những phần thật sự cần thiết.

Làm việc theo từng phase nhỏ, có test, structured log, replay, metrics và cổng Go/No-Go. Không được đưa ML hoặc lệnh chuyển PX4 mode vào hoạt động trước khi các phase nền tảng vượt qua acceptance gate.

### Quyết định bắt buộc về XGBoost

- XGBoost là workstream bắt buộc của dự án, không phải hạng mục tùy chọn có thể bỏ qua.
- Khi dataset vượt Phase 9 gate, phải train, tune, calibrate và benchmark ít nhất hai model XGBoost dự báo sai số/q90 cho M52 và MiDaS.
- Phải tối ưu XGBoost có kiểm soát bằng grouped validation, early stopping và search space được ghi version; không chọn model theo một lần train hoặc theo test set.
- Việc train/tối ưu model là bắt buộc, nhưng quyền đưa model vào primary runtime vẫn phụ thuộc acceptance gate. Model chưa vượt baseline phải tiếp tục chạy offline/shadow, được phân tích và tối ưu hoặc bổ sung dữ liệu; không được ép promote một model kém an toàn.
- Ground-contact classifier chỉ bắt buộc nếu runtime thực sự có cả target tiếp xúc đất và không tiếp xúc đất; hai error/q90 model cho M52 và MiDaS luôn bắt buộc.

## 1. Kết quả cuối cùng phải đạt

Luồng hoàn chỉnh:

```text
RGB frame + capture timestamp
        ↓
bbox tracker hiện tại
        ↓
TargetObservation
        ↓
vehicle pose + camera pose đúng thời điểm capture
        ↓
┌─────────────────────────────────────┐
│ M52                                 │
│ - ground anchors                   │
│ - ground-contact target candidate  │
└─────────────────────────────────────┘
        +
┌─────────────────────────────────────┐
│ MiDaS raw inverse depth             │
│ - metricized bằng M52 anchors       │
│ - metric target candidate           │
└─────────────────────────────────────┘
        ↓
quality features + uncertainty
        ↓
rule-based gates → XGBoost q90/OOD khi đã đủ dữ liệu
        ↓
Covariance Intersection khi hợp lệ
        ↓
Target EKF
position NED + velocity NED + covariance
        ↓
latency compensation có giới hạn
        ↓
NED → WGS84 + altitude AMSL
        ↓
MAVLink FOLLOW_TARGET stream
        ↓
PX4 native Follow Target mode
```

Workflow người dùng:

1. Người dùng khoanh bbox.
2. Tracker bám mục tiêu; gimbal/camera tiếp tục giữ mục tiêu trong ảnh theo cơ chế hiện tại.
3. M52 và MiDaS ước lượng target position/range.
4. Fusion và EKF tạo target position/velocity ổn định.
5. Hệ thống ghi nhận khoảng cách hiện tại và đánh giá khả năng vào native Follow mode an toàn.
6. Chỉ khi target beacon và entry geometry hợp lệ, UI mới enable nút `Follow Target`.
7. Khi người dùng nhấn nút, ứng dụng tiếp tục stream `FOLLOW_TARGET`, yêu cầu PX4 chuyển sang native Follow Target và xác minh ACK/nav state.
8. PX4 tự tạo flight setpoint và điều khiển drone; ứng dụng không gửi OFFBOARD velocity setpoint.
9. Khi target estimate mất hiệu lực, ứng dụng yêu cầu PX4 thoát Follow sang mode an toàn đã thống nhất, rồi xác minh mode thực tế.

## 2. Ràng buộc tuyệt đối

### 2.1 Không can thiệp PX4

- Không sửa PX4 source, firmware, module, flight task, estimator, mixer hoặc build output.
- Không tự động ghi hoặc thay đổi PX4 parameter.
- Không sửa `FLW_TGT_*`, flight-control tuning hoặc failsafe parameters.
- Được phép đọc PX4 parameters/status để validation và dự đoán entry geometry, nhưng không được ghi.
- Không dùng OFFBOARD làm backend thay cho native Follow Target.
- Đầu ra điều khiển duy nhất của pipeline target là MAVLink `FOLLOW_TARGET` cộng command chuyển/thoát native Follow mode.
- Mọi thay đổi code chỉ nằm trong repository `swarm_dashboard` và test/artifact thuộc dự án này.

### 2.2 An toàn triển khai

- Tất cả feature mới phải có feature flag; mặc định không được điều khiển mode thật cho tới khi pass SITL.
- Estimator/fusion/ML mới phải chạy shadow mode trước khi trở thành nguồn `FOLLOW_TARGET` chính.
- Không cài dependency, tải model hoặc thay đổi môi trường khi chưa báo rõ và được người dùng đồng ý.
- Không xóa hoặc ghi đè log, dataset hay model hiện có.
- Không format hoặc sửa file ngoài phạm vi patch.
- Không che lỗi bằng broad exception, silent fallback hoặc tái sử dụng measurement cũ mà không đánh dấu stale.
- Không gửi `(lat, lon) = (0, 0)`, NaN/Inf, target cũ hoặc drone position giả làm target khi estimate invalid.
- Không tự động re-enter Follow sau target loss trong phiên bản an toàn đầu tiên.
- Không flight test thật trước khi vượt đầy đủ replay và SITL acceptance gates.

## 3. Những file/module phải đọc trước khi sửa

Đọc toàn bộ tài liệu audit và code liên quan:

- `CODEX_AUDIT_IMPLEMENTATION_PLAN_M52_MIDAS_XGBOOST.md`
- `m52_adapter.py`
- `depth_model_adapter.py`
- `depth_worker.py`
- `metric_depth_calibrator.py`
- `target_depth_extractor.py`
- `metric_target_fusion.py`
- `target_fusion_ekf.py`
- `pose_time_sync.py`
- `follow_target_quality_gate.py`
- `visual_follow_target.py`
- `mavlink_manual_bridge.py`
- Các call path liên quan trong `tracking_web.py` và `main.py`
- Camera profiles, runtime environment configuration, requirements và test hiện hữu

Nếu repository có `.codegraph/`, bắt buộc dùng CodeGraph trước `rg`, `grep`, `find` hoặc đọc lan man để tìm symbol và call path.

Trước patch đầu tiên, phải trình bày compatibility map cho từng thành phần với một trong bốn trạng thái:

```text
REUSE
ADAPT
REPLACE
MISSING
```

Xác minh lại mọi giả định vì code có thể đã thay đổi sau khi prompt này được viết.

## 4. Các sự thật kỹ thuật phải giữ

- Code hiện tại chủ yếu dùng NED. Giữ NED trong geometry, candidates, fusion và EKF; chỉ đổi frame ở interface cần thiết.
- `M52GroundAnchorAdapter` hiện chủ yếu tạo anchors để metric hóa MiDaS. Nó chưa phải target candidate độc lập hoàn chỉnh.
- MiDaS sau khi được metric hóa bằng M52 anchors có tương quan với M52. Không được giả định hai nguồn độc lập.
- Grid M52 mặc định `12 × 8` có tối đa 96 sample trước rejection. Không dùng ngưỡng cứng `anchor_count >= 100`.
- MiDaS geometry phải dùng raw floating-point inverse depth; không dùng visualization depth 8-bit.
- EKF position/velocity hiện tại phải được ưu tiên tái sử dụng.
- Camera FOV runtime có nguy cơ không khớp profile. Không tin range/position trước khi xác minh intrinsics.
- `FOLLOW_TARGET` cần vị trí target WGS84; target velocity chỉ là thông tin bổ sung, không thay thế target position.
- Native PX4 Follow mode tự tạo position/velocity/acceleration setpoint từ target beacon và cấu hình `FLW_TGT_*` hiện hữu.
- Ứng dụng không thể tuyên bố “set khoảng cách hiện tại thành PX4 follow distance” nếu không thay parameter hoặc không có command được PX4 hiện tại hỗ trợ và ACK xác nhận.
- Nếu tuyệt đối không ghi PX4 parameter, khoảng cách đo hiện tại phải được dùng làm readiness/entry-geometry gate so với cấu hình PX4 hiện hữu.
- PX4 Follow mode có thể phát sinh gimbal command. Phải audit và kiểm thử quyền điều khiển gimbal để tránh hai nguồn cùng điều khiển làm mất bbox.

## 5. Kiến trúc và data contracts

Không bắt buộc dùng đúng tên class nếu codebase có convention tốt hơn, nhưng semantic, units, frames, timestamps và validity phải rõ.

### 5.1 `TargetObservation`

```text
schema_version
session_or_track_id
frame_index
capture_timestamp
capture_clock_domain
bbox_xyxy
frame_width
frame_height
tracker_score
tracker_state
lost_frames_or_equivalent
optional_mask
```

### 5.2 `SynchronizedFrameContext`

```text
observation
vehicle_position_ned_at_capture
vehicle_attitude_at_capture
camera_position_ned_at_capture
camera_attitude_at_capture
camera_intrinsics
pose_interpolation_status
pose_age_s
sync_valid
reason_codes
```

### 5.3 `TargetCandidate`

```text
schema_version
source: m52 | midas
session_or_track_id
measurement_timestamp
position_ned_m
position_covariance_ned_m2
range_m
range_variance_m2
bearing_or_camera_ray
quality
age_s
valid
reason_codes
feature_snapshot
```

### 5.4 `FusedTargetState`

```text
schema_version
session_or_track_id
state_timestamp
position_ned_m
velocity_ned_mps
position_covariance_ned_m2
velocity_covariance_ned_m2ps2
estimate_age_s
fusion_mode
source_health
ready_for_beacon
ready_for_follow
reason_codes
```

### 5.5 `FollowTargetBeacon`

```text
schema_version
source_state_timestamp
send_timestamp
lat_deg_e7
lon_deg_e7
alt_amsl_m
velocity_ned_mps
position_covariance_or_accuracy
est_capabilities
valid
reason_codes
```

Mọi field phải ghi rõ unit và frame. Mọi schema/model/dataset phải có version. Không phụ thuộc âm thầm vào thứ tự dictionary keys.

## 6. Follow workflow state machine

Triển khai state machine rõ ràng, không suy ra state từ một vài boolean rời rạc:

```text
IDLE
  ↓ bbox selected
TRACKING_ONLY
  ↓ synchronized estimate stable
ESTIMATE_STABLE
  ↓ robust distance/reference captured
SAFE_REFERENCE_LOCKED
  ↓ target beacon stream warm and entry geometry valid
READY_FOR_PX4_FOLLOW
  ↓ user presses Follow
FOLLOW_MODE_REQUESTED
  ↓ command ACK + nav_state verified
PX4_FOLLOW_ACTIVE
  ↓ quality degraded briefly
FOLLOW_DEGRADED
  ↓ target invalid/timeout/unsafe
EXIT_MODE_REQUESTED
  ↓ safe nav_state verified
TRACKING_ONLY or REACQUIRE_REQUIRED
```

State transition phải có:

- Guard conditions.
- Entry/exit actions.
- Timeout.
- Reason code.
- Idempotency.
- Unit test.

UI chỉ hiển thị Follow active khi PX4 telemetry xác nhận đúng nav state; không dựa riêng vào việc đã gửi command.

## 7. Quy trình triển khai bắt buộc

Thực hiện tuần tự. Không chuyển phase khi acceptance gate chưa đạt. Nếu một gate fail, dừng mở rộng tính năng và xử lý nguyên nhân ở phase hiện tại.

### Phase 0 — Audit call path và baseline bất biến

Mục tiêu:

- Xác định call path thật từ camera frame → tracker → pose sync → depth → fusion → EKF → WGS84 → MAVLink sender → PX4 mode request.
- Xác định current writer của gimbal và mọi writer có thể xuất hiện trong native Follow mode.
- Tái hiện trường hợp chuyển Follow bị giật bằng log/replay/SITL.
- Ghi baseline trước khi thay đổi hành vi.

Phải xác minh:

- Clock domain của frame, pose, worker, EKF và MAVLink.
- `fx`, `fy`, `cx`, `cy`, FOV, resolution và pixel-center convention.
- Optical camera axes, camera-to-body transform, quaternion order và NED/ENU conversion.
- Local/global reference dùng để đổi NED sang WGS84.
- Altitude source và AMSL convention.
- Current `FOLLOW_TARGET` field mapping và send rate.
- Command dùng để vào/thoát Follow, ACK handling và nav-state verification.
- Read-only values của `FLW_TGT_DST`, `FLW_TGT_HT`, `FLW_TGT_FA`, `FLW_TGT_ALT_M`, `FLW_TGT_MAX_VEL`, `FLW_TGT_RS` trong môi trường test.
- PX4 version đang chạy; không giả định behavior của version khác.

Baseline metrics:

- Target position/range jitter khi đứng yên.
- Target velocity jitter.
- End-to-end estimate age.
- WGS84 target jump.
- First PX4 local position/velocity/acceleration setpoint sau mode switch.
- Drone velocity/acceleration peak.
- Bbox lost timing.

Gate:

- Không còn frame/clock/altitude convention chưa xác định.
- Có evidence-backed root cause hoặc ít nhất measured handover discontinuity.
- Có session có thể replay.

### Phase 1 — Structured logging và deterministic replay

Thêm logging machine-readable, có schema version, theo từng frame/estimate/send event:

- Observation và capture timestamp.
- Vehicle/camera pose đã sync, interpolation/stale status.
- Camera intrinsics/profile ID.
- M52 anchors, coverage, geometry condition và residual.
- MiDaS raw ROI statistics, calibration scale/shift/inliers/residual.
- M52/MiDaS candidates, covariance, age, validity và reason codes.
- XGBoost/scaler/OOD output khi có.
- CI weight/result khi có.
- EKF state, covariance, innovation và gating.
- Pre-compensation và send-time target state.
- WGS84/AMSL beacon fields.
- `FOLLOW_TARGET` send timestamps/rate.
- Mode command, ACK, nav state và PX4 setpoint/vehicle response nếu telemetry có.

Ground truth chỉ thuộc evaluation channel, không được đi vào runtime estimator.

Replay phải:

- Không cần bay lại.
- Tái tạo estimator/fusion output trong tolerance xác định.
- Cho phép chạy các biến thể current/M52-only/MiDaS-only/baseline/ML/CI trên cùng session.
- Không dùng dữ liệu chưa tồn tại tại thời điểm quyết định.

Gate:

- Replay deterministic trên fixture/session cố định.
- Schema mismatch bị từ chối rõ ràng.
- Có regression fixture cho sự cố mode-entry jerk.

### Phase 2 — Observation, time synchronization và camera calibration

Chuẩn hóa `TargetObservation` mà không viết lại tracker.

Yêu cầu:

- Mọi result gắn đúng `session_or_track_id` và `frame_index`.
- Không dùng depth của frame cũ cho bbox mới.
- Pose lookup theo timestamp capture, không theo thời điểm inference hoàn thành.
- Linear interpolation cho position và quaternion SLERP cho attitude.
- Stale/extrapolation rejection có threshold và reason code.
- Nếu interpolation chờ sample bao quanh timestamp, phần chờ phải tính vào latency/age.

Camera calibration:

- Xác minh FOV runtime với camera profile.
- Dùng full intrinsics nếu có; nếu Gazebo distortion-free thì ghi rõ assumption.
- Nếu camera thật có distortion, yêu cầu calibration/undistortion trước geometry.
- Test pixel center, four corners, horizon ray và known ground intersections.

Gate:

- Synthetic geometry tests qua tolerance.
- Không có axis/sign swap.
- FOV/profile mismatch được phát hiện fail-closed.

### Phase 3 — M52 ground anchors và independent target candidate

#### M52 anchors

Giữ `M52GroundAnchorAdapter` làm nguồn anchor metric cho MiDaS.

Anchor quality phải dùng kết hợp:

- Valid count và valid fraction trên số sample khả dụng.
- Spatial coverage theo cells/quadrants.
- Exclusion của target bbox/mask.
- Horizon/ground-plane geometry condition.
- Optical-depth distribution.
- Local inverse-depth consistency.
- Pose/camera attitude validity.

Không dùng một count threshold bất khả thi.

#### M52 target candidate

Khi target được xác định hoặc cấu hình là ground-contact:

- Dùng bottom-center hoặc robust contact point từ bbox/mask.
- Chiếu camera ray xuống ground plane.
- Reject near-horizon, behind-camera, negative intersection và ill-conditioned geometry.
- Tạo covariance từ pixel uncertainty, bbox/contact uncertainty, pose uncertainty, intrinsics uncertainty và ground-plane uncertainty.

Không dùng ground candidate cho target bay hoặc target không chắc tiếp xúc đất.

Gate:

- M52-only candidate có thể log/replay độc lập.
- Invalid candidate có reason code; không trả sample cũ như sample mới.
- Error/covariance được đánh giá theo range và camera attitude.

### Phase 4 — MiDaS metric calibration và target candidate

MiDaS phải giữ raw float inverse depth `q`.

Metric relation:

```text
1 / Z = a * q + b
```

Pipeline:

1. Nhận frame đúng session/frame ID.
2. MiDaS inference bằng transform đã được kiểm tra aspect ratio/intrinsics consistency.
3. Thu M52 ground anchors ngoài target ROI.
4. Robust fit bằng RANSAC.
5. Least-squares refinement hiện hữu; đánh giá Huber refinement bằng benchmark trước khi chấp nhận.
6. Kiểm tra `a > 0`, inlier ratio, spatial coverage, robust residual, condition và temporal drift.
7. Detect change point/reset khi calibration thay đổi thực sự.
8. Trích target ROI bằng mask nếu có; nếu không thì erosion + robust foreground selection.
9. Tạo metric target ray/position và covariance.

Uncertainty phải tính từ:

- ROI median/MAD/IQR.
- Foreground fraction.
- Calibration residual/inlier ratio.
- Anchor coverage/geometry.
- Pose and camera uncertainty.
- Depth result age.
- Tracker score/state và bbox border proximity.

Gate:

- MiDaS-only candidate có thể log/replay độc lập.
- Không có visualization normalization trong geometry path.
- Calibration invalid làm candidate invalid, không fallback âm thầm.

### Phase 5 — Rule-based fusion baseline

Trước XGBoost và CI, triển khai/evaluate baseline có thể giải thích:

- Tracker validity gate.
- Pose-sync gate.
- Measurement age gate.
- M52 geometry gate.
- MiDaS calibration/ROI gate.
- Candidate disagreement gate.
- EKF innovation/Mahalanobis gate.
- Một source tốt thì dùng source đó với covariance conservative.
- Cả hai xấu thì predict-only ngắn hạn, sau đó invalid.

Không average đơn giản hai nguồn tương quan. Nếu covariance chưa đáng tin cậy, source selection conservative tốt hơn fusion giả chính xác.

Gate:

- Baseline không kém current pipeline trên held-out replay về median/p90/p95 position error, velocity error, jump rate và stale behavior.
- Không có mode command thật trong phase này.

### Phase 6 — Target EKF và send-time prediction

Tái sử dụng `TargetFusionEKF` 6 trạng thái NED:

```text
x = [N, E, D, vN, vE, vD]
```

Yêu cầu:

- Variable `dt` từ timestamp đúng.
- Position/range/bearing update chỉ khi semantic/covariance hợp lệ.
- Mahalanobis innovation gate.
- Process noise thích ứng nhưng có bounds.
- Covariance không bị collapse.
- Session change reset đúng cách.
- Predict-only timeout hữu hạn.
- Không tiếp tục target velocity vô hạn khi mất bbox.

Vì PX4 receiver nhận beacon sau độ trễ inference/network, tạo state tại send time:

```text
age = send_time - state_timestamp
p_send = p_state + v_state * bounded(age)
```

- Chỉ extrapolate trong giới hạn cấu hình.
- Propagate/inflate covariance theo age.
- Nếu age vượt threshold, không gửi beacon như valid.
- Log cả state gốc và state đã compensate.

Gate:

- Stationary target velocity bias thấp.
- Moving target latency compensation cải thiện held-out error.
- Không tăng tail error khi target dừng hoặc đổi hướng.

### Phase 7 — NED to WGS84 và MAVLink `FOLLOW_TARGET`

Tạo target global position từ PX4 local/global reference và target NED.

Không được:

- Nhầm camera-relative vector thành local absolute position.
- Cộng drone position hai lần.
- Đảo North/East hoặc Down/Up.
- Trộn ellipsoid altitude, relative altitude và AMSL.
- Dùng current drone lat/lon làm target fallback.

Validate conversion bằng round-trip và known offsets theo North/East.

Beacon tối thiểu:

```text
timestamp       = đúng unit/clock theo MAVLink implementation
est_capabilities = POS bit; thêm VEL bit chỉ khi velocity valid
lat             = target latitude WGS84 degE7
lon             = target longitude WGS84 degE7
alt             = target altitude AMSL metres
vel[0]          = target velocity North m/s
vel[1]          = target velocity East m/s
vel[2]          = target velocity Down m/s
acc             = zero/unknown trừ khi thật sự valid
attitude/rates  = unknown trừ khi thật sự valid
position_cov    = map đúng spec/implementation nếu được dùng
```

Nếu đang dùng PX4 2D tracking, target altitude vẫn phải finite/spec-correct, nhưng application readiness không được phụ thuộc vào vertical target estimate. Nếu PX4 đang ở 3D target-altitude mode mà AMSL target altitude chưa được xác minh, block Follow.

Sender phải:

- Có configurable rate phù hợp với PX4 version và measured timeout.
- Chống duplicate/out-of-order state.
- Theo dõi actual send rate và gap distribution.
- Không burst bù hàng loạt message sau khi event loop bị block.
- Fail closed khi beacon invalid.

Gate:

- Static target WGS84 ổn định.
- Known N/E movement cho đúng dấu và độ lớn.
- No invalid/zero beacon packet.
- Message inspection xác nhận field mapping.

### Phase 8 — Native PX4 Follow handover và loss handling

Không dùng OFFBOARD.

#### Warm-up

Trước mode request:

- Tracker/estimator ổn định liên tục trong cửa sổ cấu hình.
- Beacon stream hoạt động ổn định trong cửa sổ warm-up.
- Position/velocity/covariance/age dưới ngưỡng.
- Không có target jump/disagreement/OOD.
- PX4 armed/in-air/local-position/home requirements hợp lệ.
- Gimbal authority đã được xác minh không xung đột.

#### Safe reference và entry geometry

Tính robust current range/relative vector từ một cửa sổ ổn định; không lấy một frame.

Đọc nhưng không ghi cấu hình PX4 Follow hiện hữu. Dự đoán geometry mà native mode sẽ yêu cầu từ:

- Current target estimate.
- Current drone state.
- `FLW_TGT_DST`.
- `FLW_TGT_HT`.
- `FLW_TGT_FA`.
- `FLW_TGT_ALT_M`.
- Target course/velocity.
- Observed behavior của đúng PX4 version trong SITL.

Tính và log:

```text
predicted_initial_position_error
predicted_altitude_error
target_position_jump
target_velocity_jump
```

Vì không ghi PX4 parameter, không được khẳng định current measured distance sẽ trở thành native follow distance. Chỉ enable Follow nếu current/expected entry geometry nằm trong tolerance an toàn.

Nếu muốn dùng `MAV_CMD_DO_FOLLOW_REPOSITION` hoặc cơ chế offset khác:

- Trước hết kiểm tra đúng PX4 version có implement hay không.
- Phải nhận `COMMAND_ACK = ACCEPTED`.
- Phải chứng minh bằng telemetry/SITL rằng offset thật sự được áp dụng.
- Nếu unsupported hoặc ambiguous, không dùng.
- Không dùng “virtual target” để đánh lừa PX4 trừ khi được thiết kế riêng, review, chứng minh không phá target semantics/gimbal và được người dùng phê duyệt.

#### Mode request

Trình tự:

```text
beacon valid + warm stream
        ↓
entry gate pass
        ↓
user presses Follow
        ↓
send one idempotent mode command
        ↓
wait COMMAND_ACK with timeout
        ↓
verify PX4 nav_state == native Follow
        ↓
mark UI PX4_FOLLOW_ACTIVE
```

Không spam mode command. Handle reject/timeout/mode mismatch rõ ràng.

#### Loss handling

Mất ngắn:

- EKF predict-only trong timeout ngắn.
- Inflate covariance theo age.
- Không coi prediction là measurement mới.

Mất dài/unsafe:

- Yêu cầu PX4 thoát native Follow sang safe hold/position mode đã xác minh.
- Chờ ACK và nav-state confirmation.
- Không gửi target giả nhảy về drone hoặc `(0,0)`.
- Không tự re-enter Follow khi bbox quay lại.
- Yêu cầu full stability window và user action mới.

Gate:

- First PX4 setpoint sau switch không vượt threshold đã xác định.
- Không có velocity/acceleration spike như regression session.
- Bbox không mất do handover trong test scenarios.
- Target loss dẫn tới safe exit có bounded response time.

### Phase 9 — Dataset chuẩn cho XGBoost

Chỉ bắt đầu khi rule-based geometry baseline và replay đã pass.

Thu dataset đa dạng:

- Range gần/vừa/xa.
- Target đứng yên, tăng tốc, dừng, đổi hướng.
- Drone đứng yên và di chuyển.
- Bbox nhỏ/lớn/edge/partial occlusion.
- Camera attitude và ground geometry khác nhau.
- M52 tốt/MiDaS xấu, M52 xấu/MiDaS tốt, cả hai tốt, cả hai xấu.
- Nhiều session/flight/location/lighting/background nếu hướng tới camera thật.

Label:

- Ground-contact class nếu use case có cả ground/non-ground target.
- Per-source absolute/3D position error.
- q90 target hoặc quantile-compatible label/objective.
- Runtime accept/reject quality chỉ dùng làm auxiliary analysis, không thay ground truth.

Chia dữ liệu theo group `flight/session/location`, không random frame split.

Không dùng:

- Ground truth-derived runtime feature.
- Flight/session ID làm predictor.
- Future EKF state.
- Feature tạo từ frame sau thời điểm quyết định.

Gate:

- Có data dictionary, feature schema/version và split manifest.
- Không leakage.
- Có đủ sample theo failure regime; nếu chưa đủ thì tiếp tục thu thập, không train để “cho có”.

### Phase 10 — StandardScaler, bắt buộc train/tối ưu XGBoost và OOD

StandardScaler phải được dùng có mục đích, không áp dụng mù quáng.

Mục tiêu của phase này không chỉ là chứng minh pipeline có thể load một model. Phải tạo được model bundle XGBoost đã train, tune, calibrate, benchmark và có thể tái lập. Không được kết thúc Phase 10 bằng heuristic-only với lý do XGBoost chưa tối ưu.

Thiết kế khuyến nghị:

```text
continuous feature vector
       ├── raw continuous features ──→ tree-based XGBoost
       └── StandardScaler transform ─→ OOD/z-score monitoring
```

Lý do:

- Tree-based XGBoost thường không cần scaling để tăng accuracy.
- StandardScaler hữu ích cho feature schema enforcement, distribution monitoring và OOD detection.
- Nếu benchmark chọn pipeline XGBoost nhận scaled features, runtime bắt buộc dùng đúng scaler của model bundle; không trộn raw/scaled.

Chỉ fit scaler trên training split. Validation/test/runtime chỉ transform.

Không scale:

- Boolean flags.
- Enum/category/state ID.
- Reason-code bitmask.
- Timestamp tuyệt đối.
- Raw images/depth maps.
- Position NED/WGS84 hoặc MAVLink beacon fields để gửi PX4.

Continuous feature candidates:

- Tracker score.
- Bbox area/aspect/normalized center/border distance.
- Anchor fraction/coverage/geometry condition.
- Calibration scale/shift/residual/inlier ratio/drift.
- MiDaS ROI median/MAD/IQR/foreground fraction.
- M52/MiDaS predicted covariance.
- Candidate disagreement.
- Pose/depth/estimate age.
- EKF innovation statistics.

XGBoost models:

1. M52 error/q90 model — bắt buộc.
2. MiDaS error/q90 model — bắt buộc.
3. Ground-contact classifier — bắt buộc khi use case/dataset có cả ground-contact và non-ground target; nếu không áp dụng, phải ghi rõ assumption và evidence.

Training và hyperparameter optimization bắt buộc:

- Khóa một test split theo group `flight/session/location` và không dùng split này để chọn feature, tune hyperparameter, early stopping, calibrate threshold hoặc chọn checkpoint.
- Dùng grouped train/validation hoặc grouped cross-validation ở phần dữ liệu còn lại; mọi frame cùng session/flight phải nằm cùng một group.
- Tối ưu riêng M52 error/q90 model và MiDaS error/q90 model; không mặc định một cấu hình tốt cho cả hai nguồn.
- Search có giới hạn, có seed và lưu đầy đủ trial history cho các nhóm tham số tối thiểu: learning rate, number of trees/early-stopping rounds, depth/leaves tương đương, minimum child weight, row/column subsampling và regularization.
- Dùng objective quantile/q90 tương thích với library version đã xác minh. Nếu phải dùng surrogate hoặc post-calibration, phải mô tả rõ và đo empirical q90 coverage.
- Chọn model bằng multi-metric trên validation: q90 coverage/calibration, p90/p95 accepted-source error, false-accept rate của measurement xấu, false-reject rate của measurement tốt, jump rate và runtime latency. Không tối ưu chỉ RMSE hoặc chỉ accuracy.
- Với model có chất lượng tương đương, ưu tiên model nhỏ hơn, inference nhanh hơn và ít nhạy với distribution shift hơn.
- Dùng early stopping; không chọn `n_estimators` bằng test set.
- Đánh giá calibration theo source, range bin, camera-attitude bin và failure regime; metric tổng hợp tốt không được che một nhóm nguy hiểm.
- Có thể dùng validation/conformal post-calibration cho q90 nếu cần, nhưng calibration artifact phải nằm trong bundle và chỉ được fit trên train/validation theo protocol đã công bố.
- Benchmark raw continuous features cho tree model; StandardScaler mặc định phục vụ schema/OOD. Nếu scaled features thắng benchmark và được chọn, bundle/runtime phải khóa đúng scaled contract.
- Ghi lại dataset version, split manifest, feature schema, preprocessing, search space, seed, best trial, library/hardware versions, training duration và checksum artifact.
- Sau khi chọn model bằng validation, chỉ đánh giá test split khóa đúng một lần cho báo cáo promotion. Nếu thay feature/hyperparameter sau khi xem test result, phải tạo test split/version mới và ghi rõ lý do.

ML output chỉ được dùng để:

- Calibrate/inflate covariance.
- Gate unreliable source.
- Chọn ground candidate applicability.
- Detect OOD.
- Hỗ trợ fusion weight.

ML không trực tiếp sinh lat/lon và không trực tiếp gửi mode command.

Model bundle tối thiểu:

```text
bundle_version
feature_schema_version
ordered_feature_names
feature_types
missing_value_policy
raw_or_scaled_contract
scaler/preprocessor
models
training_split_manifest
metrics
q90_coverage_report
OOD reference statistics/thresholds
library versions
checksum
```

Runtime fail closed khi:

- Missing/extra/reordered feature.
- Schema/version mismatch.
- NaN/Inf ngoài policy.
- Bundle checksum sai.
- OOD quá ngưỡng.

Gate:

- Hai model XGBoost M52 và MiDaS đã được train/tune/calibrate bằng pipeline tái lập; có trial history, best-model rationale và model bundle hợp lệ.
- q90 coverage đạt mục tiêu trên held-out grouped flights.
- ML cải thiện tail error/gating so với heuristic baseline.
- Nếu chưa cải thiện ổn định, XGBoost vẫn là workstream bắt buộc nhưng chỉ được giữ offline/shadow; phải có error analysis và kế hoạch retune/thu thêm dữ liệu trước lần đánh giá promotion tiếp theo.
- Không được bỏ qua XGBoost và tuyên bố hoàn thành bằng heuristic-only.

### Phase 11 — Covariance Intersection và shadow runtime

Chỉ áp dụng CI khi:

- M52 và MiDaS candidates đều hợp lệ.
- Covariance đã calibration.
- Ground-contact assumption hợp lệ cho M52 candidate.
- Disagreement không vượt hard gate.
- Input không OOD.
- Measurement age trong giới hạn.

CI phải:

- Chọn/tối ưu weight trong bound.
- Dùng tiêu chí conservative như trace/determinant.
- Inflate covariance theo predicted q90, age, OOD và correlation risk.
- Log weight, objective, source health và reason code.

Không coi CI là cách tạo independence. Nó chỉ là fusion bảo thủ khi cross-correlation không biết chính xác.

Triển khai:

```text
offline replay
    ↓
runtime shadow
    ↓
compare against rule-based primary
    ↓
promote only after held-out/SITL pass
```

Gate:

- Tail error và jump rate tốt hơn baseline.
- Covariance consistency đạt yêu cầu.
- Không tạo overconfident estimate.

### Phase 12 — UI, observability và operational safety

UI phải hiển thị ít nhất:

- Tracker state/score.
- Estimated range và uncertainty.
- Target position/velocity validity.
- Estimate age.
- Fusion mode/source health.
- Beacon stream warm/rate status.
- Entry geometry pass/fail và reason.
- Requested mode, ACK status và actual PX4 nav state.
- `Follow Target` enabled/disabled reason.

Nút Follow phải idempotent và disabled trong lúc request pending.

Gimbal:

- Audit PX4 native Follow gimbal output và application tracking-gimbal output.
- Không cho hai controller cùng ghi mà không có authority/arbitration rõ ràng.
- Bbox retention là acceptance metric bắt buộc.
- Nếu không thể giữ tracker/gimbal authority mà không sửa PX4, dừng và báo blocker; không che bằng workaround chưa kiểm chứng.

## 8. Test matrix bắt buộc

### Unit tests

- Pixel → camera ray.
- Camera/body/NED transforms.
- Ray-ground intersection.
- Pose interpolation/SLERP/stale rejection.
- M52 anchor quality/coverage.
- MiDaS robust calibration và invalid cases.
- ROI foreground extraction.
- Candidate covariance construction.
- Disagreement/OOD/quality gates.
- EKF updates, innovation gates, reset và predict timeout.
- Send-time latency compensation.
- NED ↔ WGS84 round-trip.
- AMSL altitude mapping.
- MAVLink field mapping/capability flags.
- Follow workflow transitions/ACK/timeouts.
- Target loss safe-exit behavior.

### Edge/property tests

- Empty/tiny/edge bbox.
- Horizon or behind-camera ray.
- Invalid camera quaternion/pose.
- Duplicate/out-of-order frame.
- Stale MiDaS result.
- NaN/Inf.
- Clock reset/jump.
- Session/target change.
- Model/scaler schema mismatch.
- Large target-position outlier.
- Zero target velocity versus unknown velocity.
- Event-loop stall/message gap.

### Replay comparisons

```text
current pipeline
M52-only
MiDaS-only
rule-based gated baseline
XGBoost gated/weighted
CI fusion
```

### SITL sequence

1. Drone và target đứng yên.
2. Target beacon stream khi chưa chuyển mode.
3. Mode entry ở geometry hợp lệ.
4. Mode entry ở geometry không hợp lệ phải bị block.
5. Target chuyển động chậm.
6. Target tăng/giảm tốc.
7. Target đổi hướng.
8. Target đi gần mép ảnh.
9. Occlusion ngắn.
10. Tracking loss dài.
11. MiDaS stale/worker lag.
12. Pose stale.
13. M52/MiDaS disagreement.
14. WGS84 outlier injection.
15. `FOLLOW_TARGET` stream interruption.
16. Mode command reject/timeout.
17. Gimbal-authority conflict check.

Không chỉ assert “không crash”; phải assert numerical bounds và safe state transitions.

## 9. Metrics bắt buộc

Estimation:

- Range MAE, median, p90/p95 và max.
- Target NED position error median/p90/p95.
- Target velocity error/bias.
- Stationary-target jitter/drift.
- q90 coverage theo source/range/scenario.
- Candidate invalid/OOD/disagreement rates.
- EKF innovation consistency.
- Covariance coverage/consistency.
- End-to-end latency và estimate age distribution.

Beacon:

- WGS84 continuity và error quy đổi về metres.
- Beacon position/velocity jump rates.
- Send rate, p95 gap và dropped/duplicate count.
- NED→WGS84 round-trip error.
- Altitude convention validation.

Native Follow handover:

- First PX4 position/velocity/acceleration setpoint delta.
- Drone velocity/acceleration peak sau mode switch.
- Overshoot và settling time.
- Distance-to-target behavior so với `FLW_TGT_DST` hiện hữu.
- Time-to-safe-exit sau target invalid.
- Bbox retention/lost rate.

Mọi threshold phải có unit, documented default và evidence từ replay/SITL. Không rải magic numbers.

## 10. Cách tổ chức patch

- Mỗi phase là patch nhỏ, reviewable, có test riêng.
- Trước patch: nêu files dự kiến sửa, invariants phải giữ, migration/rollback và acceptance gate.
- Sau patch: chạy test liên quan, báo exact result và test chưa chạy.
- Không thay public API nếu adapter có thể giữ compatibility.
- Tập trung config và reason codes.
- Feature flag mặc định giữ behavior cũ cho tới khi promotion gate pass.
- Không trộn refactor lớn với thay đổi thuật toán trong cùng patch.
- Không commit generated dataset/model/log vào source tree nếu repository policy không cho phép.

Thứ tự patch khuyến nghị:

```text
P0 audit/report only
P1 schemas + logging + replay skeleton
P2 time sync + camera geometry tests
P3 M52 candidate + uncertainty
P4 MiDaS candidate + calibration uncertainty
P5 rule-based fusion + EKF gates
P6 WGS84/beacon validation
P7 native Follow workflow + SITL handover gates
P8 dataset export/splits
P9 scaler/OOD/XGBoost offline + model bundle
P10 ML shadow runtime
P11 CI shadow runtime
P12 controlled promotion + regression suite
```

## 11. Điều kiện phải dừng và xin ý kiến

Dừng trước khi tiếp tục nếu:

- Cần sửa PX4 source/firmware/module.
- Cần ghi/thay PX4 parameter.
- Cần cài dependency hoặc tải model.
- Camera intrinsics/frame/altitude convention không xác minh được.
- Không có ground truth/dataset đủ cho ML.
- Baseline mới không pass acceptance gate.
- PX4 version không hỗ trợ command giả định.
- Native Follow gimbal output xung đột với tracking gimbal mà không có arbitration an toàn.
- SITL cho thấy setpoint/velocity spike hoặc bbox loss regression.
- Safe-distance requirement mâu thuẫn với read-only `FLW_TGT_*` configuration.

Khi dừng, cung cấp:

1. Evidence.
2. Impact.
3. Hai hoặc ba phương án.
4. Phương án khuyến nghị.

Không tự mở rộng quyền hạn.

## 12. Báo cáo sau mỗi phase

Trả lời theo cấu trúc:

```text
Phase:
Outcome:
Compatibility/invariants preserved:
Files changed:
Tests run and exact results:
Metrics before/after:
Safety checks:
Known limitations:
Go/No-Go for next phase:
Next proposed patch:
```

Nếu chưa chạy một test, phải nói rõ “not run” và lý do. Không được nói pass dựa trên suy đoán.

## 13. Definition of Done

Chỉ tuyên bố hoàn thành khi:

- Tracker hiện tại vẫn hoạt động và session/frame association đúng.
- Camera calibration, geometry và timestamp sync đã được test.
- Structured logging và deterministic replay hoạt động.
- M52 anchors và M52 target candidate có semantic/uncertainty rõ.
- MiDaS metric calibration và candidate có validity/covariance rõ.
- Rule-based baseline vượt hoặc ít nhất không kém current pipeline trên held-out data.
- Target EKF position/velocity và latency compensation được validation.
- NED→WGS84/AMSL beacon conversion đúng và liên tục.
- `FOLLOW_TARGET` field mapping/rate/gap behavior được kiểm chứng.
- Native PX4 Follow entry chỉ xảy ra sau warm-up, entry gate, user action, ACK và nav-state confirmation.
- Regression mode-entry jerk không tái diễn trong SITL acceptance set.
- Target loss dẫn đến safe exit có timeout hữu hạn.
- Gimbal không bị dual-control và bbox retention đạt yêu cầu.
- StandardScaler chỉ fit trên training data, bundle có schema/version/checksum và OOD fail-closed.
- Hai XGBoost error/q90 model bắt buộc đã được train, tune, calibrate và đóng bundle tái lập; model chỉ được promote khi chứng minh cải thiện trên held-out grouped data, nếu chưa đạt phải giữ offline/shadow kèm error analysis và kế hoạch tối ưu tiếp.
- CI chứng minh cải thiện tail error/covariance consistency trước promotion.
- Không sửa PX4, không ghi PX4 parameter và không dùng OFFBOARD thay native Follow.
- Mọi giới hạn còn lại được ghi rõ bằng evidence.

Bắt đầu bằng Phase 0. Chưa sửa code trước khi hoàn thành CodeGraph/code audit, compatibility map, call-path summary, risk list và kế hoạch patch P1 để người dùng review.

## PROMPT KẾT THÚC
