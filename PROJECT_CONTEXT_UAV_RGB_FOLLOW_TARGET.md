# PROJECT_CONTEXT — UAV RGB Target Tracking and PX4 Follow Target

## 1. Mục tiêu dự án

Xây dựng hệ thống cho UAV có **một camera RGB đơn gắn trên gimbal** có thể:

1. Cho phép chọn và tracking một vật thể chuyển động bất kỳ.
2. Ước lượng khoảng cách từ UAV/camera tới vật thể.
3. Ước lượng tọa độ 3D của vật thể trong hệ tọa độ thế giới.
4. Chuyển tọa độ sang WGS84.
5. Gửi vị trí và vận tốc mục tiêu qua MAVLink `FOLLOW_TARGET`.
6. Cho PX4 hoạt động ở chế độ Follow Target/Follow Me để UAV bám theo mục tiêu.

Mục tiêu không nhất thiết là drone. Mục tiêu có thể là người, xe, robot, động vật, vật thể bay, vật treo hoặc một vật thể bất kỳ mà tracker giữ được ID.

---

## 2. Ràng buộc bắt buộc

### Phần cứng

- Chỉ dùng **một camera RGB monocular**.
- Camera được gắn trên gimbal.
- Không dùng camera depth.
- Không dùng RealSense.
- Không dùng stereo camera.
- Không dùng LiDAR.
- Không dùng ToF.
- Không dùng laser rangefinder.
- Không giả định mục tiêu có GPS hoặc phát telemetry.
- Không giả định biết trước kích thước thật của mục tiêu.

### Phần mềm và nguồn dữ liệu

Chỉ dùng bốn luồng chính:

1. **RGB tracking stream**
   - Ảnh RGB.
   - Bounding box hoặc segmentation mask.
   - Track ID.
   - Pixel tâm mục tiêu.
   - Confidence.

2. **MiDaS stream**
   - Nhận ảnh RGB.
   - Sinh bản đồ relative inverse depth.
   - Không coi output MiDaS là khoảng cách mét trực tiếp.

3. **M52 geometry/geolocation stream**
   - Camera intrinsics.
   - Drone GPS/local pose.
   - Drone attitude.
   - Gimbal yaw/pitch/roll.
   - Extrinsic camera–gimbal–body.
   - Chuyển đổi Camera → ENU → ECEF → WGS84.
   - Ray-cast giao mặt đất để tạo metric anchors.

4. **PX4 Follow Target stream**
   - Nhận latitude, longitude, altitude và velocity mục tiêu.
   - Gửi MAVLink `FOLLOW_TARGET`.
   - Giai đoạn đầu ưu tiên Follow Target 2D.

Các telemetry của UAV và gimbal được xem là dữ liệu nội bộ cần thiết của M52, không phải một cảm biến khoảng cách bổ sung.

---

## 3. Ý tưởng cốt lõi

Camera RGB đơn chỉ cung cấp hướng nhìn đến mục tiêu. MiDaS chỉ cung cấp độ sâu tương đối.

Do đó, hệ thống phải tạo nguồn scale metric bằng hình học:

1. Chọn nhiều pixel mặt đất nhìn thấy trong ảnh.
2. Dùng M52 ray-ground để tính khoảng cách metric của các pixel đó.
3. Lấy output MiDaS tại cùng các pixel.
4. Fit quan hệ affine giữa inverse metric depth và MiDaS output.
5. Dùng quan hệ này để đổi MiDaS target depth sang mét.
6. Dùng pixel target và metric depth để tạo điểm 3D trong camera frame.
7. Chuyển điểm target sang ENU/WGS84.
8. Lọc qua nhiều frame bằng EKF.
9. Chỉ gửi Follow Target khi uncertainty đủ thấp.

Mô hình hiệu chỉnh:

```math
\rho_i = \frac{1}{Z_i}
```

```math
\rho_i = a q_i + b
```

Trong đó:

- `q_i`: raw output MiDaS tại pixel `i`.
- `Z_i`: camera-axis metric depth của pixel `i`, được tính bằng M52.
- `a`: scale.
- `b`: shift.

Sau khi fit:

```math
Z(u,v) = \frac{1}{a q(u,v) + b}
```

Không được normalize output MiDaS về ảnh 8-bit trước khi tính toán.

---

## 4. Kiến trúc tổng thể

```text
RGB camera frame + capture timestamp
            |
            +-----------------------+
            |                       |
            v                       v
 Generic visual tracker          MiDaS
 bbox/mask/ID/confidence    raw relative depth map
            |                       |
            +-----------+-----------+
                        |
                        v
               M52 camera geometry
     camera intrinsics + UAV pose + gimbal pose
                        |
            +-----------+------------+
            |                        |
            v                        v
  Ground metric anchors      Target depth samples
            |                        |
            +-----------+------------+
                        |
                        v
       RANSAC + robust scale/shift fitting
                        |
                        v
             Scale/shift temporal filter
                        |
                        v
      Target metric 3D point in camera frame
                        |
                        v
          Camera → ENU → ECEF → WGS84
                        |
                        v
                Target EKF estimator
        position + velocity + covariance
                        |
                        v
                  Safety gating
                        |
                        v
             MAVLink FOLLOW_TARGET
                        |
                        v
                 PX4 Follow Me
```

---

## 5. Tracking vật thể bất kỳ

Tracker không phụ thuộc class.

Đầu vào có thể được khởi tạo bằng:

- Operator click vào vật thể.
- Operator kéo bounding box.
- Detector phát hiện đối tượng rồi khởi tạo tracker.
- Segmentation model tạo mask ban đầu.

Đầu ra tối thiểu:

```python
@dataclass
class TargetObservation:
    timestamp_us: int
    track_id: int
    bbox_xyxy: tuple[float, float, float, float]
    center_uv: tuple[float, float]
    confidence: float
    mask: object | None = None
    lost_frames: int = 0
```

Nếu có mask thì ưu tiên mask. Nếu chỉ có bbox thì dùng vùng lõi của bbox, không dùng toàn bbox.

Ví dụ:

```python
inner_bbox = shrink_bbox(bbox, scale=0.55)
```

Mục tiêu:

- Giảm background lọt vào phép đo depth.
- Giảm nhiễu ở biên vật thể.
- Giữ depth samples ổn định qua thời gian.

---

## 6. Camera model

Ma trận nội tại:

```math
K =
\begin{bmatrix}
f_x & 0 & c_x \\
0 & f_y & c_y \\
0 & 0 & 1
\end{bmatrix}
```

Pixel:

```math
p =
\begin{bmatrix}
u \\
v \\
1
\end{bmatrix}
```

Tia trong camera frame:

```math
r_c = K^{-1}p
```

Trước khi dùng pixel phải undistort ảnh hoặc undistort pixel bằng distortion coefficients đã calibration.

---

## 7. Camera pose trên gimbal

Không cộng Euler đơn giản.

Dùng chuỗi biến đổi:

```math
T_{wc}(t) = T_{wb}(t) T_{bg}(t) T_{gc}
```

Trong đó:

- `T_wb`: body UAV sang world.
- `T_bg`: gimbal sang body, thay đổi theo encoder.
- `T_gc`: camera sang gimbal, cố định sau calibration.

Rotation:

```math
R_{wc}(t) = R_{wb}(t) R_{bg}(t) R_{gc}
```

Camera position:

```math
P_c^w = P_b^w + R_{wb} t_{bc}
```

Phải dùng pose tại đúng thời điểm camera exposure.

Không dùng pose tại lúc MiDaS inference hoàn tất.

Cần pose buffer:

```python
camera_pose = interpolate_pose(
    telemetry_buffer,
    frame.capture_timestamp_us,
)
```

- Position interpolation: tuyến tính.
- Quaternion interpolation: SLERP.
- Gimbal angle interpolation: theo timestamp.

---

## 8. Tạo ground metric anchors bằng M52

### 8.1 Chọn candidate pixels

Chọn khoảng `100–500` pixel:

- Chủ yếu ở nửa dưới ảnh.
- Không nằm trong target bbox/mask.
- Không sát biên ảnh.
- Không ở vùng sky.
- Không ở vùng target.
- Phân bố trên nhiều hàng và cột.
- Tia phải hướng xuống mặt đất.
- Nên bao phủ cả vùng gần, trung bình và xa.

Không cần chắc chắn mọi pixel là mặt đất. RANSAC sẽ loại outlier.

### 8.2 Ray-ground

Với pixel `i`:

```math
r_i^c = K^{-1}p_i
```

```math
r_i^w = R_{wc}r_i^c
```

Tia thế giới:

```math
P_i(\lambda) = P_c^w + \lambda r_i^w
```

Mặt đất phẳng:

```math
z = h_g
```

Giao điểm:

```math
\lambda_i =
\frac{h_g - P_{c,z}^w}
{r_{i,z}^w}
```

Điều kiện hợp lệ:

```math
r_{i,z}^w < 0
```

```math
\lambda_i > 0
```

Điểm mặt đất:

```math
P_{g,i}^w = P_c^w + \lambda_i r_i^w
```

### 8.3 Đổi về camera-depth metric

```math
P_{g,i}^c =
R_{cw}(P_{g,i}^w - P_c^w)
```

```math
P_{g,i}^c =
\begin{bmatrix}
X_i \\
Y_i \\
Z_i
\end{bmatrix}
```

`Z_i` là tọa độ theo trục quang học camera, được suy ra bằng hình học.

Đây không phải dữ liệu từ camera depth.

---

## 9. Hiệu chỉnh MiDaS relative depth sang metric depth

Tại mỗi anchor:

```python
q_i = midas_raw[v_i, u_i]
rho_i = 1.0 / Z_i
```

Fit:

```math
\rho_i = a q_i + b
```

### 9.1 Không dùng ordinary least squares trực tiếp

Candidate anchors có thể chứa:

- Xe.
- Người.
- Cây.
- Tường.
- Mái nhà.
- Vùng MiDaS lỗi.
- Pixel bị motion blur.
- Pixel không thực sự thuộc ground plane.

Quy trình bắt buộc:

```text
candidate anchors
      |
      v
RANSAC affine fit
      |
      v
inlier set
      |
      v
Huber or Tukey robust regression
      |
      v
a_fit, b_fit, residual statistics
```

### 9.2 Điều kiện chấp nhận scale

Các giá trị ban đầu để tuning:

```python
scale_valid = (
    num_inliers >= 80
    and inlier_ratio >= 0.60
    and median_relative_error <= 0.15
    and a_fit > 0
)
```

Ngoài ra cần kiểm tra:

- `a*q_target + b > epsilon`.
- Anchor depths có độ phân tán đủ lớn.
- Không để tất cả anchors nằm cùng một khoảng cách.
- Fit không tạo negative depth trong vùng hợp lệ.
- Fit không thay đổi quá đột ngột so với frame trước.

### 9.3 Temporal filter cho scale và shift

State:

```math
x_s =
\begin{bmatrix}
a \\
b
\end{bmatrix}
```

Process model:

```math
x_{s,k+1} = x_{s,k} + w_k
```

Measurement:

```math
z_{s,k} =
\begin{bmatrix}
a_{fit} \\
b_{fit}
\end{bmatrix}
```

Measurement covariance phụ thuộc:

- Inlier ratio.
- Robust fit RMSE.
- Số lượng anchors.
- Anchor depth spread.
- Gimbal angular rate.
- Pose validity.
- Image blur.

Nếu fit không hợp lệ:

- Không update `a,b`.
- Chỉ prediction.
- Tăng scale covariance.
- Không lập tức dùng fit xấu.

---

## 10. Ước lượng metric depth của target

### 10.1 Sample trong vùng target

Nếu có mask:

```python
inner_mask = erode(mask)
```

Nếu chỉ có bbox:

```python
inner_bbox = shrink_bbox(bbox, scale=0.55)
```

Sample `30–150` pixel.

Với mỗi pixel:

```math
Z_j = \frac{1}{a q_j + b}
```

Bỏ pixel nếu:

- Denominator không dương.
- Depth ngoài giới hạn vận hành.
- Pixel sát biên mask/bbox.
- Pixel có MiDaS gradient quá lớn.
- Pixel nằm trong vùng không ổn định.

### 10.2 Robust target depth

Median:

```math
m = median(Z_j)
```

Median absolute deviation:

```math
MAD = median(|Z_j - m|)
```

Inliers:

```math
|Z_j - m| < 2.5 MAD
```

Target depth:

```math
Z_t = median(Z_j^{valid})
```

Target depth uncertainty có thể khởi tạo từ:

```math
\sigma_Z =
k_{mad} MAD
+
k_{fit} e_{scale}
+
k_{bbox} \frac{1}{A_{bbox}}
```

---

## 11. Từ depth và pixel sang điểm 3D

Dùng target center hoặc robust representative pixel:

```math
p_t =
\begin{bmatrix}
u_t \\
v_t \\
1
\end{bmatrix}
```

```math
n_t = K^{-1}p_t
```

Với:

```math
n_t =
\begin{bmatrix}
x_t \\
y_t \\
1
\end{bmatrix}
```

Điểm target trong camera frame:

```math
P_t^c = Z_t n_t
```

```math
P_t^c =
\begin{bmatrix}
Z_t x_t \\
Z_t y_t \\
Z_t
\end{bmatrix}
```

Slant range:

```math
d_t = \|P_t^c\|
```

Đổi sang world/ENU:

```math
P_t^w = P_c^w + R_{wc}P_t^c
```

Đầu ra:

```text
E_t
N_t
U_t
range_m
```

Sau đó:

```text
ENU → ECEF → WGS84
```

Đầu ra WGS84:

```text
target_lat
target_lon
target_alt
```

---

## 12. Hai candidate vị trí song song

Vì target có thể là bất kỳ vật thể nào, duy trì hai phép đo:

### 12.1 Ground candidate

Dùng bottom-center của bbox/mask:

```math
P_t^{ground} = RayGround(u_{bottom}, v_{bottom})
```

Phù hợp cho:

- Người.
- Xe.
- Robot mặt đất.
- Vật thể trượt trên đất.

### 12.2 Free-space candidate

Dùng MiDaS metric:

```math
P_t^{free} = P_c^w + R_{wc}P_t^c
```

Phù hợp cho:

- Drone.
- Chim.
- Vật treo.
- Vật thể bay.
- Vật thể không chạm đất.

Không bắt buộc dùng classifier để chọn loại mục tiêu.

Target EKF sẽ chọn candidate dựa trên consistency.

---

## 13. Target EKF

State ban đầu:

```math
x_t =
\begin{bmatrix}
E \\
N \\
U \\
v_E \\
v_N \\
v_U
\end{bmatrix}
```

Constant velocity model:

```math
P_{k+1} = P_k + v_k \Delta t
```

```math
v_{k+1} = v_k + w_k
```

Measurement options:

- `P_t_ground`.
- `P_t_free`.

Innovation:

```math
\nu = z - H\hat{x}
```

Innovation covariance:

```math
S = HPH^T + R
```

Normalized Innovation Squared:

```math
NIS = \nu^T S^{-1}\nu
```

Candidate selection:

```python
if nis_ground < gate and nis_ground < nis_free:
    ekf.update(ground_candidate)
elif nis_free < gate:
    ekf.update(free_candidate)
else:
    ekf.predict_only()
```

Dùng hysteresis:

- Candidate phải thắng liên tục `5–10` frame mới đổi active mode.
- Không đổi ground/free mode chỉ vì một frame.

EKF output:

```python
@dataclass
class TargetState:
    timestamp_us: int
    position_enu_m: tuple[float, float, float]
    velocity_enu_mps: tuple[float, float, float]
    covariance_enu: object
    range_m: float
    active_mode: str
    valid: bool
    stale: bool
```

---

## 14. Uncertainty và quality score

Không coi mọi frame có độ tin cậy như nhau.

Quality components:

```text
tracker confidence
target sample count
target depth MAD
scale fit RMSE
scale inlier ratio
anchor count
anchor depth spread
pose validity
gimbal telemetry age
target position in image
bbox/mask area
```

Ví dụ:

```python
quality = (
    tracker_quality
    * scale_quality
    * target_depth_quality
    * pose_quality
)
```

Measurement covariance phải tăng khi quality giảm.

Không cần công thức hoàn hảo từ đầu, nhưng bắt buộc phải:

- Gán covariance động.
- Có outlier rejection.
- Có stale timeout.
- Có safety gate.

---

## 15. Trường hợp không nhìn thấy mặt đất

Nếu camera nhìn lên trời hoặc không còn ground anchors:

```text
no metric anchors
→ no new scale observation
→ MiDaS metric scale không thể được cập nhật tuyệt đối
```

Xử lý:

1. Giữ `a,b` gần nhất.
2. Chỉ prediction scale filter.
3. Tăng scale covariance theo thời gian.
4. Target EKF tiếp tục dự đoán ngắn hạn.
5. Không cho UAV tiến gần nếu uncertainty vượt ngưỡng.
6. Chỉ giữ target trong FOV bằng gimbal/yaw.
7. Chờ ground anchors xuất hiện lại.

Không được giả vờ rằng MiDaS vẫn có metric scale chính xác khi không còn nguồn scale.

---

## 16. Safety state machine

```text
IDLE
  |
  v
TARGET_SELECTED
  |
  v
VISUAL_TRACKING
  |
  v
SCALE_INITIALIZING
  |
  v
METRIC_TRACKING
  |
  v
FOLLOW_READY
  |
  v
FOLLOWING
```

Failure states:

```text
TARGET_LOST
SCALE_INVALID
POSE_INVALID
UNCERTAINTY_HIGH
FOLLOW_ABORT
```

Logic gợi ý:

```python
if tracker_confidence < 0.3:
    state = "TARGET_LOST"

elif not pose_valid:
    state = "POSE_INVALID"

elif not scale_valid:
    state = "SCALE_INITIALIZING"

elif position_sigma_horizontal > max_position_sigma:
    state = "UNCERTAINTY_HIGH"

elif ekf_initialized:
    state = "FOLLOW_READY"
```

Khi uncertainty cao:

- Không gửi target mới.
- Hoặc đánh dấu stale.
- UAV giữ vị trí.
- Không tiếp tục tiến về mục tiêu.

---

## 17. PX4 Follow Target

### 17.1 Bản đầu dùng 2D Follow

Ưu tiên:

```text
FLW_TGT_ALT_M = 0
```

Mục đích:

- Follow latitude/longitude.
- UAV giữ độ cao hiện tại/home-relative.
- Không dùng target altitude để điều khiển vertical motion.

Lý do:

- Vertical depth là thành phần khó và nhạy sai số nhất.
- Altitude reference MSL dễ sai.
- 2D Follow an toàn và thực tế hơn cho prototype.

### 17.2 Dữ liệu MAVLink

Gửi:

```python
FOLLOW_TARGET(
    timestamp=...,
    est_capabilities=...,
    lat=int(target_lat * 1e7),
    lon=int(target_lon * 1e7),
    alt=target_alt_msl,
    vel=[v_north, v_east, v_down],
    acc=[0.0, 0.0, 0.0],
    attitude_q=[0.0, 0.0, 0.0, 0.0],
    rates=[0.0, 0.0, 0.0],
    position_cov=[sigma_h, sigma_h, sigma_v],
    custom_state=0,
)
```

Chú ý đổi ENU velocity sang NED:

```text
v_north = v_N
v_east  = v_E
v_down  = -v_U
```

### 17.3 Điều kiện publish

Giá trị khởi đầu để tuning:

```python
follow_allowed = (
    tracker_confidence > 0.60
    and scale_valid
    and ekf_initialized
    and horizontal_position_sigma_m < 3.0
    and relative_range_sigma < 0.20
    and not target_lost
    and not pose_stale
)
```

Tần số publish:

```text
10–20 Hz
```

MiDaS có thể chạy chậm hơn; target EKF prediction sẽ cung cấp state ở tần số publish.

---

## 18. Tần số đề xuất

```text
RGB camera capture:          20–30 Hz
Tracker:                     20–30 Hz
MiDaS:                        5–10 Hz
M52 anchor generation:        theo MiDaS frame
Scale/shift filter:           theo MiDaS frame
Target EKF prediction:       30–50 Hz
Target EKF measurement:       theo measurement tới
FOLLOW_TARGET publish:       10–20 Hz
```

---

## 19. Cấu trúc thư mục đề xuất

```text
uav_rgb_follow/
├── README.md
├── PROJECT_CONTEXT.md
├── pyproject.toml
├── config/
│   ├── camera.yaml
│   ├── extrinsics.yaml
│   ├── estimator.yaml
│   └── px4_follow.yaml
├── src/
│   ├── camera/
│   │   ├── capture.py
│   │   ├── calibration.py
│   │   └── frame_types.py
│   ├── tracking/
│   │   ├── tracker_interface.py
│   │   ├── target_selector.py
│   │   └── target_observation.py
│   ├── midas/
│   │   ├── model.py
│   │   ├── inference.py
│   │   └── raw_depth.py
│   ├── telemetry/
│   │   ├── px4_client.py
│   │   ├── gimbal_client.py
│   │   ├── pose_buffer.py
│   │   └── time_sync.py
│   ├── geometry/
│   │   ├── camera_model.py
│   │   ├── transforms.py
│   │   ├── ray_ground.py
│   │   ├── enu_ecef_wgs84.py
│   │   └── m52_adapter.py
│   ├── scale/
│   │   ├── anchor_sampler.py
│   │   ├── anchor_builder.py
│   │   ├── robust_affine_fit.py
│   │   └── scale_filter.py
│   ├── target_estimation/
│   │   ├── target_depth.py
│   │   ├── candidate_builder.py
│   │   ├── target_ekf.py
│   │   └── quality.py
│   ├── follow/
│   │   ├── safety_gate.py
│   │   ├── follow_target_sender.py
│   │   └── state_machine.py
│   └── main.py
├── tests/
│   ├── test_camera_geometry.py
│   ├── test_ray_ground.py
│   ├── test_scale_fit.py
│   ├── test_target_depth.py
│   ├── test_target_ekf.py
│   └── test_follow_gate.py
├── scripts/
│   ├── calibrate_camera.py
│   ├── inspect_midas_raw.py
│   ├── replay_log.py
│   └── run_sitl.py
└── logs/
```

---

## 20. Data contracts

### RGB frame

```python
@dataclass
class RGBFrame:
    timestamp_us: int
    image_bgr: object
    width: int
    height: int
```

### Camera pose

```python
@dataclass
class CameraPose:
    timestamp_us: int
    position_enu_m: tuple[float, float, float]
    rotation_camera_to_enu: object
    valid: bool
```

### Scale estimate

```python
@dataclass
class ScaleEstimate:
    timestamp_us: int
    scale_a: float
    shift_b: float
    covariance: object
    num_inliers: int
    inlier_ratio: float
    fit_rmse: float
    valid: bool
```

### Target metric measurement

```python
@dataclass
class TargetMeasurement:
    timestamp_us: int
    position_enu_m: tuple[float, float, float]
    covariance_enu: object
    range_m: float
    source: str
    valid: bool
```

---

## 21. Logging bắt buộc

Mỗi frame cần log:

```text
capture timestamp
tracker bbox/mask/confidence
raw MiDaS target samples
ground anchor count
RANSAC inlier count
scale a
shift b
scale covariance
target depth median
target depth MAD
ground candidate
free-space candidate
active EKF measurement source
EKF position
EKF velocity
EKF covariance
follow_allowed
FOLLOW_TARGET payload
```

Dùng log để replay offline mà không cần bay thật.

---

## 22. Kiểm thử theo giai đoạn

### Giai đoạn 1 — Geometry unit tests

Kiểm tra:

- Pixel center tạo đúng optical ray.
- Nadir ray giao đúng vị trí dưới UAV.
- ENU/WGS84 round-trip.
- Camera–gimbal–body transform.
- Quaternion composition.

### Giai đoạn 2 — MiDaS scale offline

Dùng video RGB đã ghi:

- Chạy MiDaS raw.
- Tạo synthetic ground anchors.
- Fit `1/Z = aq+b`.
- Vẽ residual.
- Kiểm tra ổn định `a,b` qua frame.

### Giai đoạn 3 — Target range test tĩnh

Đặt vật thể tại khoảng cách đã đo thủ công:

```text
5 m
10 m
15 m
20 m
30 m
```

Đo:

```text
absolute error
relative error
jitter
outlier rate
```

### Giai đoạn 4 — Target chuyển động

- Người đi bộ.
- Xe chạy chậm.
- Vật thể bay thử nghiệm nếu có.
- So sánh quỹ đạo estimate với ground truth thủ công/GPS tham chiếu chỉ để đánh giá.

### Giai đoạn 5 — PX4 SITL

- Không bay thật.
- Publish target giả.
- Kiểm tra Follow Me.
- Kiểm tra timeout.
- Kiểm tra target lost.
- Kiểm tra uncertainty gate.

### Giai đoạn 6 — Flight test an toàn

- Khu vực trống.
- Vận tốc thấp.
- Khoảng cách follow lớn.
- 2D Follow.
- Có operator takeover.
- Có geofence.
- Có RTL/Hold failsafe.

---

## 23. Acceptance criteria cho prototype

Prototype đạt yêu cầu khi:

1. Tracker giữ đúng mục tiêu liên tục trong điều kiện thử.
2. MiDaS scale/shift không nhảy bất thường khi ground anchors hợp lệ.
3. Sai số range ở khoảng cách vận hành chấp nhận được.
4. Target EKF không phản ứng theo outlier một frame.
5. Target velocity có dấu và hướng hợp lý.
6. Tọa độ WGS84 không nhảy lớn giữa các frame.
7. Follow Target chỉ được publish khi quality đủ tốt.
8. Khi mất target, UAV không tiếp tục lao theo prediction quá lâu.
9. Khi mất ground anchors, uncertainty tăng đúng.
10. SITL chuyển Hold/abort đúng khi target stale.

Mục tiêu sai số ban đầu nên thực tế:

```text
5–15 m range:   relative error <= 15%
15–30 m range:  relative error <= 25%
horizontal position jitter sau EKF: <= 2–3 m
target lost abort: <= 1 s
```

Các ngưỡng này là mục tiêu prototype, không phải cam kết lý thuyết.

---

## 24. Giới hạn vật lý

Phải hiểu rõ:

- MiDaS không phải cảm biến metric depth.
- Camera RGB đơn không thể đảm bảo absolute scale trong mọi frame.
- Khi không có ground anchors, metric scale chỉ được giữ tạm thời.
- Flat-ground M52 sai trên địa hình dốc.
- Gimbal timestamp sai có thể gây lỗi vị trí lớn.
- Mục tiêu nhỏ trên ảnh làm range uncertainty tăng mạnh.
- Bầu trời hoặc background đồng nhất làm MiDaS kém ổn định.
- Follow Me của PX4 không tự đảm bảo obstacle avoidance.
- Không được bật 3D Follow trước khi altitude reference được kiểm chứng.

---

## 25. Không được làm

Codex không được tự ý:

- Thêm depth camera.
- Thêm LiDAR.
- Thêm stereo.
- Giả định biết kích thước target.
- Dùng MiDaS 8-bit normalized output để tính mét.
- Dùng một pixel duy nhất trong bbox.
- Gửi measurement chưa lọc thẳng sang PX4.
- Dùng pose tại inference completion time.
- Ép mọi mục tiêu xuống ground plane.
- Bật PX4 Follow khi covariance cao.
- Bật 3D Follow ngay từ đầu.
- Coi `a,b` cố định cho toàn bộ video.
- Bỏ qua target-lost và stale timeout.

---

## 26. Thứ tự công việc cho Codex

### Task 1 — Tạo skeleton project

Tạo cấu trúc thư mục, dataclasses và config.

### Task 2 — Camera geometry

Implement:

```text
undistort_pixel
pixel_to_camera_ray
compose_camera_pose
camera_to_enu
enu_to_wgs84
ray_ground_intersection
```

Viết unit tests.

### Task 3 — MiDaS raw inference

Implement:

```text
load MiDaS model
RGB preprocessing
raw float inference
resize raw output về image size
không normalize 8-bit trong core
```

### Task 4 — Ground anchors

Implement:

```text
sample candidate pixels
exclude target
ray-ground metric Z
read MiDaS q
build anchor pairs
```

### Task 5 — Robust scale fit

Implement:

```text
RANSAC affine fit
Huber refinement
quality metrics
scale validity checks
```

### Task 6 — Target metric depth

Implement:

```text
inner mask/bbox
multi-pixel sampling
median/MAD filtering
metric target depth
camera 3D point
ENU point
```

### Task 7 — Scale filter và target EKF

Implement:

```text
scale/shift Kalman filter
target constant-velocity EKF
NIS gating
ground/free candidate selection
hysteresis
```

### Task 8 — PX4 sender

Implement:

```text
ENU velocity → NED
ENU/WGS84 conversion
FOLLOW_TARGET payload
publish rate
stale handling
safety gate
```

### Task 9 — Offline replay

Tạo tool đọc log/video và chạy toàn pipeline không cần UAV.

### Task 10 — SITL

Kết nối PX4 SITL và kiểm tra Follow Target 2D.

---

## 27. Definition of Done cho mỗi module

Mỗi module phải có:

- Type hints.
- Docstrings.
- Unit tests.
- Input validation.
- Timestamp propagation.
- Explicit coordinate-frame naming.
- No hidden global state.
- Configurable thresholds.
- Structured logging.
- Failure mode rõ ràng.
- Không trả dữ liệu `valid` khi uncertainty chưa biết.

---

## 28. Quy ước hệ tọa độ

Bắt buộc ghi rõ suffix trong tên biến:

```text
_c      camera frame
_b      body frame
_g      gimbal frame
_enu    East-North-Up
_ned    North-East-Down
_ecef   Earth-Centered Earth-Fixed
_wgs84  latitude/longitude/altitude
```

Ví dụ:

```python
ray_c
ray_enu
position_camera_enu
target_position_enu
target_velocity_ned
```

Không dùng tên chung như:

```python
pos
rot
vec
```

nếu không thể hiện frame.

---

## 29. Nguồn tài liệu dự án

- MiDaS repository:
  - `https://github.com/isl-org/MiDaS`
- PX4 FollowTarget message:
  - `https://docs.px4.io/main/en/msg_docs/FollowTarget`
- PX4 Follow Me mode:
  - `https://docs.px4.io/main/en/flight_modes_mc/follow_me`
- M52 target geolocation document:
  - `m52_target_geolocation.docx`
- Supporting UAV distance/geolocation analysis:
  - `dinh_vi_khoang_cach_uav-1.pdf`

---

## 30. Tóm tắt một câu

Xây dựng hệ thống chỉ dùng camera RGB trên gimbal, trong đó tracker cung cấp pixel mục tiêu, MiDaS cung cấp relative depth, M52 dùng ground geometry để hiệu chỉnh MiDaS sang metric scale, target EKF lọc vị trí/vận tốc và PX4 nhận `FOLLOW_TARGET` 2D chỉ khi estimate đủ tin cậy.
