# Prompt triển khai: RGB-only gimbal/body-yaw tracking và PX4 Follow Target

## Vai trò

Bạn là kỹ sư robotics/UAV cấp cao, chịu trách nhiệm triển khai và kiểm chứng một pipeline tracking–control an toàn trong repository:

```text
/home/sup/swarm_dashboard
```

Hãy làm việc trực tiếp trên source hiện hành ở thư mục gốc. Không chỉnh sửa các bản backup, archive hoặc source handoff cũ như:

- `swarm_dashboard_handoff_20260729/`
- `main.py.backup*`
- `static/index.html.backup*`
- các file `.zip`, `.tar.gz`

Repository có `.codegraph/`; phải dùng CodeGraph trước khi đọc/tìm code để nắm call path và blast radius.

Triển khai theo đúng thứ tự bắt buộc:

1. Khắc phục và tối ưu gimbal + body yaw để quay bám target nhanh, mượt và độc lập.
2. Thêm workflow Follow rõ ràng do người dùng chủ động kích hoạt.
3. Chỉ dùng ảnh RGB để quan sát target.
4. Dùng multi-view bearing triangulation và EKF để tạo target 3D.
5. Lưu khoảng cách an toàn tại thời điểm chọn bbox.
6. Handover sang native PX4 `AUTO FOLLOW TARGET`.

Không được nhảy thẳng sang phần Follow khi gimbal/body-yaw chưa đạt tiêu chí nghiệm thu.

---

## Mục tiêu sản phẩm

### Giai đoạn pointing

Sau khi người dùng chọn bbox:

- Gimbal pan/tilt liên tục giữ target ở gần tâm ảnh.
- Body drone quay yaw để đưa gimbal yaw trở về gần góc home.
- Gimbal là vòng điều khiển nhanh; body yaw là vòng recenter chậm hơn.
- Trong giai đoạn này drone không được tiến/lùi theo target.
- Lỗi range, TTC, triangulation hoặc Follow không được làm gimbal ngừng bám.

### Giai đoạn Follow

Khi pointing đã ổn định:

- UI hiển thị một nút riêng: `BẮT ĐẦU FOLLOW`.
- Người dùng nhấn nút để cấp quyền bắt đầu workflow calibration + Follow.
- Drone tạo một baseline ngang có kiểm soát trong khi gimbal tiếp tục bám target.
- Hệ thống triangulate target từ nhiều RGB bearing và pose của chính drone.
- Khoảng cách an toàn được tính tương ứng với pose drone tại đúng thời điểm chọn bbox, không phải pose sau khi bootstrap.
- Hệ thống prestream target rồi yêu cầu PX4 vào native `AUTO FOLLOW TARGET`.
- Khi PX4 xác nhận Follow mode, PX4 là motion authority duy nhất.
- Target bắt đầu di chuyển và drone bám target, đưa khoảng cách ngang về khoảng cách an toàn ban đầu.

---

## Ràng buộc cứng: RGB-only

Nguồn quan sát target duy nhất được phép là camera RGB.

### Được phép dùng

- Ảnh RGB.
- BBox, tracking score, tracker diagnostics.
- Camera intrinsics đã hiệu chuẩn.
- Gimbal IMU/encoder để biết hướng camera.
- Pose, attitude, heading, velocity và timestamp của chính follower từ PX4.
- Chuyển động của chính follower để tạo baseline.
- Thuật toán hình học, Kalman/EKF và các prior toán học.

### Tuyệt đối không được dùng

- Metric depth hoặc monocular metric-depth model.
- LiDAR/rangefinder.
- Stereo camera.
- GPS/telemetry của target.
- Gazebo world pose/ground-truth của target.
- Biết trước tọa độ target.
- Giao tia với mặt đất cho target UAV đang bay.
- Kích thước thật của target làm nguồn range chính.

Kích thước bbox ban đầu chỉ được dùng làm apparent-size safety guard, TTC hoặc weak prior; không được giả vờ coi đó là khoảng cách mét.

Nếu không có pose/attitude của chính follower thì phải báo blocked; không được tạo target global giả.

---

## Tài liệu thuật toán phải tuân theo

Hai tài liệu tham chiếu:

```text
/home/sup/Downloads/dinh_vi_khoang_cach_uav-1.pdf
/home/sup/Downloads/m52_target_geolocation.docx
```

Áp dụng:

- Pixel → bearing bằng mô hình pinhole và camera intrinsics.
- Bearing camera → NED bằng full rotation composition.
- H5: multi-view triangulation từ camera ego-motion.
- H6: bearing-only constant-velocity EKF.
- Công thức độ nhạy range theo baseline:

```text
sigma_range ≈ k * range^2 * sigma_pixel / (focal_length * baseline)
```

Không áp dụng ray–ground M52 cho UAV mục tiêu đang bay.

Không thay full quaternion composition hiện có bằng phép cộng Euler tuyến tính. Full quaternion chính xác và ổn định hơn.

---

## Hiện trạng và lỗi đã xác định

Runtime đã quan sát được:

- Tracker ở trạng thái `tracking`.
- Score khoảng `0.98`.
- BBox filter đã ổn định.
- Gazebo gimbal publisher và gimbal feedback hoạt động.
- UAV đã armed, telemetry hợp lệ.
- Gimbal/follow/yaw bị dừng bởi lỗi:

```text
Gimbal control failed: 'BBoxMotionEstimate' object has no attribute 'scale_px'
```

Nguyên nhân:

- `visual_follow_target.py::BBoxMotionEstimate` chỉ có `scale_ratio`, `area_ratio`, `quality`, `ttc_s`, `stable_frames`.
- `tracking_web.py::_update_visual_follow_locked()` truy cập `estimate.scale_px`.
- Exception bị outer catch trong `_update_gimbal_locked()` bắt và reset toàn bộ gimbal/yaw/follow.

Đây là lỗi P0 và phải sửa trước.

### Cách sửa mong muốn

Ưu tiên giữ đúng semantics của `BBoxMotionSafetyEstimator`:

- Không biến nó thành metric-range estimator.
- Nếu UI cần `visual_bbox_scale_px`, tính trực tiếp từ bbox:

```text
sqrt(bbox_width_px * bbox_height_px)
```

hoặc đổi field UI/status sang scale ratio với tên đúng semantics.

Không chỉ sửa AttributeError; phải tách error boundary để lỗi estimator không bao giờ reset pointing controller.

---

## Nguyên tắc kiến trúc bắt buộc

### 1. Tách pointing khỏi follow

Tạo ranh giới rõ:

```text
TargetPointing
  ├── gimbal pan/tilt
  └── body yaw recenter

TargetGeolocation
  ├── RGB bearing observations
  ├── triangulation
  └── EKF

FollowWorkflow
  ├── user authorization
  ├── bootstrap
  ├── handover
  └── native PX4 Follow Target
```

Lỗi ở `TargetGeolocation` hoặc `FollowWorkflow` chỉ được:

- đánh dấu estimate/follow blocked;
- phát neutral/stop translation;
- giữ Position/Hold;
- ghi diagnostic.

Nó không được reset tracker hoặc gimbal pointing nếu bbox vẫn hợp lệ.

### 2. Một motion authority tại một thời điểm

Không được có hai nguồn cùng điều khiển chuyển động drone.

| Giai đoạn | Motion authority |
|---|---|
| Pointing ổn định | Position/Hold; gimbal trực tiếp |
| Body-yaw recenter | Unified Offboard yaw, translation bằng 0 |
| RGB bootstrap | Unified Offboard local-NED lateral + yaw + altitude hold |
| Follow handover | Prestream target; không phát lệnh cạnh tranh |
| Native Follow active | PX4 `AUTO FOLLOW TARGET` duy nhất |
| Tracking/estimate lost | Position/Hold |

`tracking_yaw` legacy không được chạy song song với `offboard_follow`.

### 3. Không auto-start Follow sau khi chọn bbox

Loại bỏ hành vi tự động chuyển Follow chỉ vì target estimator ready.

Chọn bbox chỉ bắt đầu:

- tracking;
- pointing;
- ổn định bbox/gimbal;
- lưu snapshot ban đầu.

Follow workflow chỉ bắt đầu sau hành động rõ ràng của người dùng.

---

## State machine yêu cầu

Triển khai state machine rõ ràng, không suy luận trạng thái rải rác bằng nhiều boolean:

```text
IDLE
  → SELECTING
  → POINTING
  → POINTING_STABLE
  → READY_FOR_FOLLOW
  → RGB_BOOTSTRAP
  → TARGET_3D_READY
  → FOLLOW_PRESTREAM
  → FOLLOW_MODE_REQUESTED
  → FOLLOWING
```

Các nhánh lỗi:

```text
POINTING/BOOTSTRAP/FOLLOWING
  → DEGRADED
  → HOLD
```

Ý nghĩa:

- `SELECTING`: chờ bbox.
- `POINTING`: tracker đã active, gimbal/body đang căn.
- `POINTING_STABLE`: bbox, score, gimbal feedback và line-of-sight ổn định.
- `READY_FOR_FOLLOW`: nút Follow được enable.
- `RGB_BOOTSTRAP`: người dùng đã cấp quyền; drone dịch ngang tạo parallax.
- `TARGET_3D_READY`: target position và initial safe distance hợp lệ.
- `FOLLOW_PRESTREAM`: phát `FOLLOW_TARGET` nhưng chưa đổi PX4 mode.
- `FOLLOW_MODE_REQUESTED`: đã yêu cầu Auto Follow Target, đang chờ xác nhận.
- `FOLLOWING`: PX4 đã xác nhận nav state Follow.
- `DEGRADED`: estimate tạm kém, giảm/khóa chuyển động.
- `HOLD`: dừng automation và yêu cầu Position/Hold.

Mọi transition phải có:

- guard rõ ràng;
- timestamp;
- reason;
- timeout;
- log/status cho UI.

---

## Pha 1 — Gimbal và body yaw nhanh, mượt

### 1.1 Gimbal loop

Đầu vào:

- bbox center đã lọc;
- frame width/height;
- `fx`, `fy`, `cx`, `cy`;
- frame timestamp;
- gimbal feedback;
- tracker velocity/Kalman velocity nếu có.

Chuyển pixel error sang angular error:

```text
yaw_error   = atan2(bbox_cx - image_cx, fx)
pitch_error = atan2(image_cy - bbox_cy, fy)
```

Không tune trực tiếp theo raw pixel nếu resolution thay đổi.

Controller nên gồm:

- low-latency filtering;
- proportional/derivative hoặc velocity feed-forward;
- rate limit;
- acceleration/slew limit;
- braking acceleration;
- anti-windup;
- deadband nhỏ;
- stale-feedback guard.

Tracker velocity được dùng để dự đoán tâm bbox tại thời điểm command:

```text
predicted_center = measured_center + bbox_velocity * end_to_end_latency
```

Không dự đoán quá xa; clamp prediction horizon.

### 1.2 Tham số khởi đầu để tune

Đây là starting range, phải kiểm chứng bằng step response:

| Tham số | Starting range |
|---|---:|
| Gimbal yaw max rate | 25–30 deg/s |
| Gimbal pitch max rate | 20–25 deg/s |
| Gimbal acceleration | 120–180 deg/s² |
| Gimbal braking acceleration | 240–300 deg/s² |
| Angular deadband | 0.25–0.5 deg |
| Body yaw enter | 3.5–4.0 deg |
| Body yaw exit | 1.0–1.5 deg |
| Body yaw enter hold | 0.10–0.15 s |
| Body yaw max rate | 18–25 deg/s |
| Body yaw slew | 60–100 deg/s² |

Không hard-code nếu project đã dùng environment-based config. Thêm config có default an toàn và expose qua status.

### 1.3 Phối hợp gimbal/body yaw

- Gimbal xử lý sai số line-of-sight tần số cao.
- Body yaw chỉ recenter dựa chủ yếu vào filtered gimbal yaw.
- BBox horizontal error chỉ dùng feed-forward nhỏ nếu cần.
- Body yaw không được giành target trực tiếp khỏi gimbal.
- Khi body bắt đầu quay, gimbal tiếp tục bù để target đứng gần tâm.
- Forward/lateral velocity bằng 0 trong pure yaw recenter.
- Khi filtered gimbal yaw xuống dưới exit threshold, body yaw dừng mượt.

Không reset hysteresis timer mỗi frame vì lỗi không liên quan.

### 1.4 Tần số và latency

Runtime hiện khoảng 12–13 FPS là chưa đủ cho cảm giác “nhanh và mượt”.

Mục tiêu:

- camera/tracker thực tế: tối thiểu 25 FPS, mục tiêu 30 FPS;
- gimbal command loop: 30–50 Hz;
- body yaw loop: 20–30 Hz;
- frame queue age thấp và không tăng dần;
- UI JPEG encode không được block control loop.

Nếu camera source không thể lên 30 FPS, controller phải chạy asynchronous bằng predicted target state, nhưng không được che giấu giới hạn sensor FPS trong diagnostic.

### 1.5 Tiêu chí nghiệm thu Pha 1

- Không còn `gimbal_error` do follow/range estimator.
- Khi target lệch ngang, gimbal rate phản hồi đúng chiều trong tối đa 2 frame.
- Không oscillation liên tục quanh tâm.
- Gimbal yaw đạt enter threshold thì body yaw bắt đầu trong khoảng 0.10–0.20 s.
- Body yaw đưa gimbal yaw về dưới exit threshold mà không đảo chiều giật.
- Khi target mất, yaw/translation về neutral và Position/Hold.
- Gimbal vẫn hoạt động khi geolocation estimator báo `insufficient_baseline`.
- Không có forward velocity trước khi người dùng nhấn Follow.

Chỉ khi tất cả tiêu chí trên đạt mới tiếp tục Pha 2.

---

## Pha 2 — Snapshot khoảng cách an toàn từ thời điểm chọn bbox

Khi `set_bbox()` thành công, lưu một immutable selection snapshot:

```text
selection_timestamp_s
selection_frame_index
selection_vehicle_position_ned
selection_camera_position_ned
selection_vehicle_global_position
selection_camera_quaternion_xyzw
selection_bearing_ned
selection_bbox
selection_bbox_scale_px
selection_heading_rad
selection_altitude_m
```

Snapshot phải dùng pose được đồng bộ với frame chọn bbox hoặc frame tracking hợp lệ gần nhất. Không dùng pose “now” không đồng bộ.

Nếu pose/gimbal timestamp stale thì:

- bbox vẫn có thể tracking/pointing;
- Follow button vẫn disabled;
- status nêu rõ `selection_pose_stale`.

### Pointing stable gate

Nút Follow chỉ enable khi liên tục trong một khoảng thời gian cấu hình được:

- tracker state `tracking`;
- không redetecting;
- không ambiguous/reject reason;
- score đạt ngưỡng;
- bbox filter ready;
- bbox center error nằm trong angular deadband;
- gimbal feedback fresh;
- gimbal yaw không đang saturated outward;
- body yaw không đang recenter mạnh;
- telemetry/local/global position hợp lệ;
- drone armed, không failsafe;
- altitude đạt ngưỡng an toàn;
- không có manual override.

Không yêu cầu target 3D trước khi enable nút, vì target 3D chỉ có sau user-authorized bootstrap.

---

## Pha 3 — User-authorized RGB bootstrap

Khi người dùng nhấn `BẮT ĐẦU FOLLOW`, chưa được giả vờ rằng PX4 đã ở Follow mode.

Nút này bắt đầu workflow:

```text
READY_FOR_FOLLOW
→ RGB_BOOTSTRAP
→ TARGET_3D_READY
→ FOLLOW_PRESTREAM
→ FOLLOWING
```

UI phải thể hiện rõ `ĐANG HIỆU CHUẨN RGB` trước khi hiển thị `FOLLOWING`.

### 3.1 Hướng chuyển động bootstrap

Tạo chuyển động gần vuông góc với horizontal line-of-sight:

```text
horizontal_bearing = normalize([ray_north, ray_east])
lateral_right      = normalize([-ray_east, ray_north])
```

Hướng trái/phải phải cấu hình được. Không tự cho rằng một phía luôn an toàn.

Vì hệ thống không có obstacle sensor:

- Trong SITL có thể dùng direction cấu hình cố định.
- Trên phương tiện thật phải yêu cầu operator xác nhận hành lang ngang an toàn.
- Không được tuyên bố có obstacle avoidance.

Không dùng body-right đơn giản nếu body yaw đang thay đổi; dùng local-NED lateral vector từ bearing đã lưu.

### 3.2 Unified bootstrap command

Bootstrap phải dùng một command authority duy nhất có thể đồng thời:

- phát local-NED north/east velocity;
- giữ `z_down` ban đầu;
- phát yaw rate recenter;
- đặt forward radial velocity bằng 0.

Không phát một publisher riêng cạnh tranh với `offboard_follow`.

Nếu callback hiện tại chỉ nhận forward velocity thì mở rộng contract rõ ràng hoặc thêm callback bootstrap chuyên biệt; không giả mạo lateral thành forward.

### 3.3 Giới hạn bootstrap

Starting defaults:

```text
lateral speed:          0.30–0.50 m/s
minimum baseline:       0.50 m
nominal baseline:       0.75 m
maximum baseline:       1.00–1.50 m
minimum observations:   6
minimum intersection:   3 deg
maximum reprojection:   3 px
timeout:                2–4 s, phù hợp với maximum baseline/speed
```

Không dùng timeout 1.5 s nếu cấu hình speed/baseline khiến về vật lý không thể hoàn thành.

Bootstrap phải dừng theo geometry quality, không chỉ theo quãng đường:

- đủ observations;
- baseline đủ;
- intersection angle đủ;
- condition number hợp lệ;
- target nằm trước camera;
- reprojection error đạt ngưỡng;
- range nằm trong bounds;
- range uncertainty tuyệt đối/tương đối đạt ngưỡng;
- tracker/gimbal/pose vẫn fresh.

### 3.4 Abort ngay khi

- tracker lost/redetecting/ambiguous;
- bbox jump kéo dài;
- gimbal feedback stale;
- pose stale;
- manual override;
- PX4 failsafe;
- local/global position invalid;
- altitude vi phạm;
- reprojection diverges;
- timeout;
- operator hủy;
- Offboard entry không được PX4 xác nhận.

Abort behavior:

1. Phát neutral local setpoint trong thời gian chuyển tiếp cần thiết.
2. Yêu cầu Position/Hold.
3. Dừng bootstrap/follow state.
4. Giữ tracker và gimbal pointing nếu target vẫn thấy.
5. Hiển thị lý do cụ thể.

---

## Pha 4 — Triangulation và EKF

Tận dụng và hoàn thiện `bearing_target_estimator.py`, không viết lại tùy tiện.

### 4.1 Observation

Mỗi observation phải chứa:

```text
frame timestamp
camera position NED tại frame timestamp
unit bearing NED
bbox center/size
tracking score
focal length
pose age
gimbal age
frame index
redetecting/ambiguous/reject reason
```

### 4.2 Triangulation

Giữ robust multi-ray least squares:

```text
minimize Σ wi * ||(I - ri riᵀ)(X - Pi)||²
```

Yêu cầu:

- pair consensus/RANSAC-like bootstrap;
- MAD/inlier rejection;
- condition-number gate;
- positive-depth gate;
- range bounds;
- reprojection gate;
- covariance.

Không dùng intersection của đúng hai tia duy nhất làm final estimate nếu đã có nhiều observations.

### 4.3 EKF

State:

```text
[north, east, down, velocity_north, velocity_east, velocity_down]
```

Model:

- constant velocity;
- process noise cấu hình được;
- bearing measurement update;
- innovation/Mahalanobis gate;
- stale timeout;
- covariance propagation.

Target được giả định đứng yên hoặc di chuyển rất chậm trong bootstrap. Nếu residual cho thấy chuyển động đáng kể trước khi scale observable thì abort với reason rõ ràng, không trả target 3D chất lượng giả.

### 4.4 Khoảng cách an toàn ban đầu

Sau khi target position `X_target_ned` hợp lệ, tính khoảng cách an toàn từ selection snapshot:

```text
delta_initial = X_target_ned - selection_vehicle_position_ned

D0_horizontal = hypot(
    delta_initial_north,
    delta_initial_east
)

D0_slant = norm(delta_initial)
delta_z0 = delta_initial_down
```

`FLW_TGT_DST` phải dùng `D0_horizontal`, không dùng:

- distance từ vị trí follower sau bootstrap;
- bbox scale;
- camera slant range;
- giá trị input cũ không liên quan.

Lưu:

```text
initial_safe_distance_horizontal_m
initial_safe_distance_slant_m
initial_vertical_offset_m
initial_bbox_scale_px
initial_range_std_m
```

Nếu covariance/range uncertainty chưa đạt ngưỡng thì không được chuyển `TARGET_3D_READY`.

---

## Pha 5 — Native PX4 Follow Target handover

### 5.1 Prestream

Trước khi đổi mode:

- phát target global position + NED velocity đều đặn;
- tần số 10–20 Hz;
- target timestamp không stale;
- phát ít nhất 5–10 message hợp lệ;
- cấu hình Follow parameters một lần cho session.

### 5.2 PX4 parameters

Thiết lập:

```text
FLW_TGT_DST = initial_safe_distance_horizontal_m
```

Giữ chính sách height rõ ràng. Code hiện clamp `FLW_TGT_HT >= 8 m`; không được để drone đang ở khoảng 5 m bất ngờ leo cao khi Follow được kích hoạt.

Chọn một trong hai:

- block Follow và yêu cầu operator đưa drone lên độ cao tối thiểu; hoặc
- chứng minh bằng PX4 version/test rằng chính sách height khác an toàn.

Ưu tiên block với thông báo rõ, không tự động leo ngoài dự kiến.

### 5.3 Mode request

Sau prestream:

- yêu cầu PX4 `AUTO FOLLOW TARGET`;
- retry giới hạn;
- chờ heartbeat/nav state xác nhận;
- chỉ đặt UI/state `FOLLOWING` khi `nav_state == 19`;
- nếu không xác nhận sau timeout: Position/Hold và blocked.

### 5.4 Mutual exclusion

Khi native Follow active:

- dừng Offboard yaw;
- dừng Offboard forward/lateral/vertical;
- dừng legacy tracking stick;
- không phát attitude/thrust controller;
- gimbal vẫn được phép pointing;
- PX4 là motion authority duy nhất.

### 5.5 Cập nhật target

Trong `FOLLOWING`:

- RGB tracker tiếp tục cung cấp bearing;
- EKF cập nhật position/velocity;
- target global và velocity phát 10–20 Hz;
- bbox initial scale dùng làm safety/TTC guard;
- không dùng apparent-size để thay đổi `D0` âm thầm.

Nếu geometry tạm yếu nhưng EKF còn fresh/covariance cho phép:

- giữ estimate trong grace period ngắn;
- hạ quality;
- không tăng tốc đột ngột.

Nếu quá stale/uncertain:

- dừng Follow;
- yêu cầu Position/Hold;
- giữ gimbal pointing nếu target còn thấy.

---

## UI/API yêu cầu

### Nút và nhãn

Tách rõ:

1. `BẮT ĐẦU TRACKING`
2. `DỪNG TRACKING`
3. `BẮT ĐẦU FOLLOW`
4. `DỪNG FOLLOW`

Không dùng một nút có tên `FOLLOW TARGET` để chỉ khởi động camera/tracker.

Nút `BẮT ĐẦU FOLLOW`:

- chỉ hiện/enable ở `READY_FOR_FOLLOW`;
- khi click đổi thành `ĐANG HIỆU CHUẨN RGB`;
- sau bootstrap đổi thành `ĐANG CHUYỂN PX4 FOLLOW`;
- chỉ hiển thị `ĐANG FOLLOW` sau nav-state confirmation.

### Status cần hiển thị

Pointing:

```text
gimbal error deg
gimbal rates
gimbal feedback age
body yaw state
requested/published yaw rate
yaw authority
pointing stable time
```

Bootstrap/geometry:

```text
observation count
baseline m
intersection angle deg
reprojection error px
condition number
bootstrap progress
range m
range std m
range relative std
target estimator state/reason
```

Follow:

```text
initial safe horizontal distance
current estimated horizontal distance
distance error
target estimate age
target velocity valid
prestream count
mode request attempts
PX4 nav state
follow workflow state/reason
```

### API semantics

Endpoint/user command Follow phải idempotent:

- click start khi đã active không khởi tạo session thứ hai;
- stop luôn đưa automation về trạng thái an toàn;
- target/bbox session ID phải chống command/frame cũ;
- chọn bbox mới reset toàn bộ bootstrap/estimate/follow snapshot cũ.

---

## Safety requirements

### Manual control priority

Manual input luôn thắng:

- hủy bootstrap;
- hủy Follow;
- về Position/Hold theo handover an toàn;
- không tự resume khi manual timeout.

### Freshness

Định nghĩa và kiểm tra riêng:

- frame age;
- tracker result age;
- vehicle pose age;
- gimbal pose age;
- target estimate age;
- MQTT command age;
- MAVLink target age.

Không dùng một timestamp chung thay cho tất cả.

### Target loss

- Dropout rất ngắn: neutral/grace, không mode oscillation.
- Dropout quá timeout: Hold.
- Không tiến về phía bbox stale.
- Không tự chọn target mới.

### Không có obstacle avoidance

Vì chỉ dùng RGB target camera và không triển khai obstacle perception:

- bootstrap direction phải được operator/config xác nhận;
- tài liệu/UI phải nói rõ không có obstacle avoidance;
- integration test thực hiện trong SITL vùng trống.

---

## Các file dự kiến liên quan

Khảo sát call graph trước khi chỉnh. Các file nhiều khả năng cần thay đổi:

- `tracking_web.py`
  - `TrackingManager`
  - `set_bbox`
  - `_update_gimbal_locked`
  - `_update_visual_follow_locked`
  - `_update_native_precenter_yaw_locked`
  - `_tracking_motion_gate_locked`
  - `_update_motion_locked`
  - state/status/reset/stop methods
- `visual_follow_target.py`
  - pixel bearing helpers
  - bbox/TTC safety estimate semantics
- `body_yaw_recenter.py`
  - hysteresis, slew, config và diagnostics
- `bearing_target_estimator.py`
  - triangulation/EKF quality gates và snapshot-compatible output
- `pose_time_sync.py`
  - frame-time pose sampling nếu cần
- `main.py`
  - command callbacks
  - local-NED bootstrap motion
  - safety gates
  - API
- `mavlink_manual_bridge.py`
  - visual target stream
  - parameter setup
  - mode handover
  - authority mutual exclusion
- `static/index.html`
  - button/state/status mới
- `.env.example`
  - config mới, default an toàn
- test files hiện có và test integration mới

Không thay đổi contract giữa các file mà không cập nhật toàn bộ callers, status và tests.

---

## Test bắt buộc

Project hiện không có `pytest` trong `.venv`; tests hiện chạy bằng `unittest`. Dùng:

```bash
.venv/bin/python -m unittest -q ...
```

### Unit tests

1. `BBoxMotionEstimate` integration không còn truy cập field không tồn tại.
2. Estimator exception không reset gimbal controller.
3. Gimbal angular error đúng với nhiều resolution/FOV.
4. Gimbal rate/acceleration/braking limiter.
5. Body-yaw enter/exit hysteresis.
6. Body-yaw không đảo chiều trực tiếp.
7. Gimbal stale khóa body motion nhưng không phá tracker.
8. Selection snapshot dùng pose đồng bộ đúng timestamp.
9. Triangulation synthetic cho target UAV bay.
10. Ground intersection không được gọi cho airborne pipeline.
11. D0 được tính từ selection pose, không phải post-bootstrap pose.
12. Bootstrap direction local-NED đúng khi body heading thay đổi.
13. Bootstrap abort khi tracker lost/failsafe/manual/stale.
14. Follow button/state transition.
15. Follow không auto-start sau bbox.
16. Prestream rồi mới request mode.
17. `FOLLOWING` chỉ khi nav state xác nhận.
18. Native Follow mutual exclusion với Offboard.
19. Target stream timeout đưa về Hold.
20. Chọn bbox mới xóa estimate/session cũ.

### Synthetic geometry tests

Tạo camera trajectory ngang và target bay tại range 8–15 m:

- baseline 0 m → `insufficient_baseline`;
- baseline 0.5–1.0 m → estimate hợp lệ khi parallax đủ;
- noise 1–2 px;
- một số outlier bbox;
- pose/gimbal latency;
- target đứng yên trong bootstrap;
- target bắt đầu chuyển động sau bootstrap.

Kiểm tra:

- range bias;
- range standard deviation;
- reprojection error;
- covariance;
- velocity convergence;
- D0 reconstruction từ pose ban đầu.

### SITL integration tests

#### Test A — Pointing

- UAV-02 armed và giữ vị trí.
- Chọn UAV-01 làm bbox.
- Di chuyển target ngang trong ảnh.
- Gimbal bám nhanh, body yaw recenter mượt.
- Không có forward/lateral translation.

#### Test B — RGB bootstrap

- Target đứng yên.
- Nhấn Follow.
- UAV-02 dịch ngang trong vùng trống.
- Baseline/parallax tăng.
- Estimator chuyển `BOOTSTRAPPING → VALID`.
- UAV về/giữ trạng thái ổn định trước handover.

#### Test C — Initial safe distance

- Ghi range thật trong test chỉ để đánh giá, không đưa vào controller.
- Đảm bảo `D0` estimate tương ứng với pose lúc chọn bbox.
- Sai số nằm trong ngưỡng được định nghĩa.

#### Test D — Native Follow

- Prestream target.
- PX4 chuyển nav state 19.
- Target bắt đầu di chuyển.
- UAV giữ khoảng cách gần D0.
- Không còn Offboard authority cạnh tranh.

#### Test E — Fault handling

- Che target.
- Làm tracker ambiguous.
- Làm gimbal feedback stale.
- Ngắt target stream.
- Manual override.
- PX4 từ chối Follow mode.

Mọi trường hợp phải về Hold an toàn và có reason quan sát được.

---

## Tiêu chí nghiệm thu tổng thể

### Pointing

- Gimbal phản ứng trong tối đa 2 frame.
- Không jitter/oscillation đáng kể khi target đứng yên.
- Body yaw recenter không giật và không tranh quyền gimbal.
- Target vẫn gần tâm trong lúc thân drone quay.
- Range/follow lỗi không làm pointing dừng.

### RGB geometry

- Không dùng nguồn scale bị cấm.
- Bootstrap chỉ chạy sau user authorization.
- Target 3D chỉ `VALID` khi geometry và covariance đạt ngưỡng.
- D0 lấy từ selection pose.
- Status cung cấp đủ baseline/parallax/reprojection/uncertainty.

### Follow

- Không auto-start.
- PX4 native Follow chỉ bắt đầu sau target stream hợp lệ.
- UI không báo Follow trước nav-state confirmation.
- PX4 giữ D0 trong giới hạn thiết kế.
- Không có simultaneous motion authority.
- Target loss/manual/failsafe đưa về Hold.

### Chất lượng code

- Không thêm broad `except Exception` làm che lỗi điều khiển.
- Error boundaries theo subsystem.
- Không duplicate state ở nhiều boolean mâu thuẫn.
- Config có validation và default an toàn.
- Tests tái hiện được bug cũ và bảo vệ behavior mới.
- Không chỉnh backup/handoff/archive.

---

## Quy trình thực hiện yêu cầu

1. Dùng CodeGraph lập call graph hiện tại.
2. Ghi lại baseline runtime/status trước chỉnh sửa.
3. Viết hoặc cập nhật tests tái hiện lỗi `scale_px`.
4. Triển khai Pha 1.
5. Chạy unit tests và SITL pointing test.
6. Dừng và đánh giá Pha 1 trước khi làm Follow.
7. Triển khai selection snapshot và explicit workflow state.
8. Triển khai user-authorized lateral bootstrap.
9. Hoàn thiện triangulation/EKF/D0.
10. Triển khai native Follow handover.
11. Chạy toàn bộ unit/integration/SITL tests.
12. Cập nhật README/config/report.

Không được chỉ sửa để dashboard hiển thị “OK”; phải kiểm chứng setpoint thực, mode PX4 và motion thực trong SITL.

---

## Deliverables

Khi hoàn thành, cung cấp:

1. Source code đã triển khai.
2. Tests mới và tests đã cập nhật.
3. `.env.example` với config và giải thích.
4. Tài liệu state machine.
5. Báo cáo:
   - root cause;
   - kiến trúc;
   - files thay đổi;
   - test commands;
   - kết quả unit/SITL;
   - tham số tuning cuối;
   - giới hạn còn lại;
   - bằng chứng chỉ dùng RGB cho target observation.
6. Snapshot status chứng minh:
   - gimbal/body yaw hoạt động;
   - bootstrap geometry hợp lệ;
   - D0 được lưu;
   - native Follow active;
   - không có authority conflict.

---

## Định dạng báo cáo cuối của agent

Trả lời bằng tiếng Việt, theo thứ tự:

1. Kết quả đạt được.
2. Root cause đã xử lý.
3. Kiến trúc và state machine thực tế.
4. Thay đổi theo từng file.
5. Các nguồn dữ liệu được dùng và xác nhận RGB-only.
6. Tham số gimbal/body yaw cuối cùng.
7. Cách D0 được tạo từ selection pose.
8. Cách bootstrap và Follow handover hoạt động.
9. Test đã chạy và kết quả.
10. Hướng dẫn vận hành trên UI.
11. Rủi ro/giới hạn còn lại.

Không tuyên bố “hoàn thành” nếu chưa có bằng chứng SITL cho cả pointing, bootstrap và native Follow Target.
