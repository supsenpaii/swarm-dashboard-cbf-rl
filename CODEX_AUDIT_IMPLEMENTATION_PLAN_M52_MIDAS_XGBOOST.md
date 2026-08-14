# CODEX AUDIT & IMPLEMENTATION PLAN
## M52 + MiDaS + StandardScaler + XGBoost + EKF + PX4 Follow Target

> Tài liệu này dùng để Codex kiểm tra khả năng áp dụng kiến trúc ước lượng khoảng cách và tọa độ mục tiêu vào chương trình UAV hiện tại.  
> **Tracking và điều khiển gimbal quay bám mục tiêu đã hoàn thành, không được viết lại hoặc thay đổi nếu không thực sự cần thiết cho interface dữ liệu.**

---

# 1. Mục tiêu của lần audit

Codex phải kiểm tra codebase hiện tại và trả lời rõ:

1. Chương trình hiện tại đã có những module nào liên quan đến:
   - camera frame;
   - target bbox/mask;
   - timestamp;
   - telemetry UAV;
   - telemetry gimbal;
   - camera calibration;
   - M52;
   - MiDaS;
   - ENU/ECEF/WGS84;
   - PX4/MAVLink;
   - EKF hoặc Kalman filter;
   - logging và replay.

2. Kiến trúc dưới đây có thể tích hợp vào code hiện tại ở mức nào:
   - dùng lại trực tiếp;
   - cần adapter;
   - cần refactor;
   - cần viết mới;
   - chưa đủ dữ liệu để thực hiện.

3. Những rủi ro kỹ thuật nào có thể khiến kết quả khoảng cách hoặc tọa độ sai:
   - sai timestamp;
   - sai hệ tọa độ;
   - sai transform gimbal;
   - dùng MiDaS output đã normalize;
   - nhầm camera-depth với slant range;
   - ground plane không đúng;
   - feature schema khác nhau giữa train và runtime;
   - scaler bị fit sai;
   - model XGBoost bị dùng ngoài miền huấn luyện;
   - PX4 nhận altitude sai reference.

4. Lộ trình triển khai tối thiểu, theo từng pull request hoặc từng patch nhỏ.

5. Không được sửa code ngay khi chưa hoàn thành audit.

---

# 2. Trạng thái hiện tại đã xác nhận

Các phần sau được xem là đã có và đang hoạt động tốt:

```text
RGB camera
→ phát hiện/chọn mục tiêu
→ visual tracking
→ giữ đúng track ID
→ gimbal quay giữ mục tiêu trong khung hình
```

Codex không được:

- thay tracker;
- thay detector;
- viết lại logic chọn mục tiêu;
- thay bộ điều khiển gimbal;
- thay pipeline hiển thị tracking;

trừ khi cần thêm một adapter rất nhỏ để xuất các trường dữ liệu bắt buộc.

Đầu vào mong muốn từ phần tracking hiện tại:

```python
TargetObservation(
    timestamp_us,
    track_id,
    bbox_xyxy,
    center_uv,
    bottom_center_uv,
    confidence,
    mask,
    frame_width,
    frame_height,
    lost_frames,
)
```

Nếu code hiện tại không có đủ các trường này, Codex phải chỉ ra:

- trường nào đã có;
- trường nào có thể tính từ dữ liệu hiện tại;
- trường nào phải thêm;
- file và hàm nào nên được sửa ít nhất.

---

# 3. Ràng buộc không được vi phạm

## 3.1 Phần cứng

Chỉ dùng:

- một camera RGB monocular;
- camera gắn trên gimbal;
- telemetry UAV;
- telemetry gimbal.

Không dùng trong runtime:

- camera depth;
- RealSense;
- stereo;
- LiDAR;
- ToF;
- laser rangefinder;
- GPS của mục tiêu;
- kích thước thật của mục tiêu.

## 3.2 Kiến trúc thuật toán

Pipeline mục tiêu:

```text
Tracking đã có
    ↓
MiDaS raw relative depth
    ↓
M52 geometry + ground metric anchors
    ↓
Scale/shift MiDaS
    ↓
M52 candidate + MiDaS candidate
    ↓
Feature preprocessing
    ↓
StandardScaler + XGBoost
    ↓
Dynamic reliability weights
    ↓
Disagreement/OOD gate
    ↓
Covariance Intersection
    ↓
Target EKF
    ↓
ENU → WGS84
    ↓
MAVLink FOLLOW_TARGET 2D
```

## 3.3 Follow Target

Giai đoạn đầu chỉ triển khai:

```text
PX4 Follow Target 2D
```

Không dùng altitude mục tiêu để điều khiển độ cao cho đến khi vertical estimate được kiểm chứng.

---

# 4. Kiến trúc đích cần kiểm tra khả năng áp dụng

```text
                         TRACKER HIỆN TẠI
        bbox/mask + center + bottom-center + timestamp
                                  │
RGB frame ────────────────────────┤
                                  ▼
                                MiDaS
                       raw relative depth map
                                  │
                                  ▼
                         Camera Pose Builder
         UAV pose + gimbal pose + camera extrinsics + timestamp
                                  │
                    ┌─────────────┴─────────────┐
                    │                           │
                    ▼                           ▼
              M52 target candidate       Ground anchor sampler
                    │                           │
                    │                  M52 metric camera-depth
                    │                           │
                    │                 Robust scale/shift fitting
                    │                           │
                    │                  MiDaS metric target depth
                    │                           │
                    └─────────────┬─────────────┘
                                  ▼
                         MiDaS target candidate
                                  │
                    M52 candidate + MiDaS candidate
                                  │
                                  ▼
                           Feature Builder
                                  │
          physical clip → impute → log1p → StandardScaler
                                  │
                                  ▼
         Ground classifier + M52 q90 + MiDaS q90 XGBoost
                                  │
                                  ▼
                     Reliability and weight engine
                                  │
                    disagreement gate + OOD gate
                                  │
                                  ▼
                      Covariance Intersection 2D
                                  │
                                  ▼
                              Target EKF
                                  │
                     ENU position + ENU velocity
                                  │
                                  ▼
                        ENU/WGS84 + ENU/NED
                                  │
                                  ▼
                       MAVLink FOLLOW_TARGET 2D
```

---

# 5. Audit bắt buộc trước khi viết code

Codex phải duyệt toàn bộ repository và tạo báo cáo theo mẫu sau.

## 5.1 Repository inventory

Liệt kê:

```text
- Ngôn ngữ chính.
- Framework đang dùng.
- Entry point.
- Luồng chạy real-time.
- Thread/process/async architecture.
- Camera capture module.
- Tracking output module.
- Telemetry module.
- Gimbal module.
- PX4/MAVLink module.
- Geometry/geolocation module.
- Config system.
- Logging system.
- Test system.
```

## 5.2 Tìm các symbol liên quan

Codex phải tìm các class/function tương đương với:

```text
TargetObservation
CameraIntrinsics
DronePose
GimbalPose
CameraPose
pixel_to_ray
ray_ground_intersection
wgs84_to_enu
enu_to_wgs84
midas inference
kalman/ekf
follow_target sender
mavlink connection
frame timestamp
```

Với mỗi symbol tìm được, báo cáo:

| Chức năng yêu cầu | File hiện tại | Symbol hiện tại | Có thể dùng lại? | Vấn đề |
|---|---|---|---|---|
| Camera intrinsics | ... | ... | Có/Không/Một phần | ... |
| Pixel → ray | ... | ... | ... | ... |
| Gimbal transform | ... | ... | ... | ... |
| ENU → WGS84 | ... | ... | ... | ... |
| MAVLink sender | ... | ... | ... | ... |

## 5.3 Kiểm tra dependency

Kiểm tra có sẵn hay chưa:

```text
numpy
opencv-python
scipy
pyproj
torch
xgboost
scikit-learn
joblib
pandas
pymavlink hoặc MAVSDK
pytest
```

Không tự ý cài hoặc nâng version trước khi báo cáo conflict.

## 5.4 Kiểm tra hardware/runtime constraints

Báo cáo:

```text
CPU
GPU
CUDA
camera FPS
image resolution
MiDaS model dự kiến
MiDaS inference latency
available RAM/VRAM
PX4 telemetry rate
gimbal telemetry rate
```

Nếu không thể suy ra từ repo, ghi `UNKNOWN` và chỉ ra cần đo ở đâu.

---

# 6. Kết quả audit Codex phải trả về

Trước khi patch code, Codex phải tạo file:

```text
docs/M52_MIDAS_XGB_COMPATIBILITY_REPORT.md
```

File phải có:

## 6.1 Kết luận tổng quát

Một trong bốn mức:

```text
A — Có thể tích hợp trực tiếp.
B — Có thể tích hợp với adapter nhỏ.
C — Cần refactor một số module lõi.
D — Chưa thể tích hợp vì thiếu dữ liệu hoặc kiến trúc không tương thích.
```

## 6.2 Ma trận tương thích

| Khối | Trạng thái | Code hiện tại | Việc cần làm | Mức rủi ro |
|---|---|---|---|---|
| Tracker adapter | | | | |
| Timestamp sync | | | | |
| Camera calibration | | | | |
| Gimbal transform | | | | |
| M52 | | | | |
| MiDaS raw | | | | |
| Scale fit | | | | |
| StandardScaler | | | | |
| XGBoost | | | | |
| EKF | | | | |
| WGS84 | | | | |
| Follow Target | | | | |

## 6.3 Danh sách file sẽ sửa

Không được dùng mô tả chung. Phải liệt kê:

```text
path/to/file.py
- thêm gì;
- sửa function nào;
- giữ nguyên API nào;
- rủi ro regression nào.
```

## 6.4 Danh sách file viết mới

Ví dụ:

```text
src/geometry/camera_pose.py
src/scale_estimation/robust_fit.py
src/fusion/preprocessing.py
src/fusion/model_bundle.py
src/estimation/target_ekf.py
```

## 6.5 Go/No-Go

Codex phải kết luận:

```text
GO
GO WITH CONDITIONS
NO-GO
```

Kèm điều kiện cụ thể.

---

# 7. Phase 1 — Chuẩn hóa interface sau tracking

## 7.1 Mục tiêu

Không sửa tracking logic. Chỉ tạo adapter xuất dữ liệu ổn định cho estimator.

## 7.2 Interface yêu cầu

```python
from dataclasses import dataclass
from typing import Any


@dataclass(frozen=True)
class TargetObservation:
    timestamp_us: int
    track_id: int

    bbox_xyxy: tuple[float, float, float, float]
    center_uv: tuple[float, float]
    bottom_center_uv: tuple[float, float]

    confidence: float
    mask: Any | None

    frame_width: int
    frame_height: int
    lost_frames: int
```

## 7.3 Validation

```python
0 <= center_u < frame_width
0 <= center_v < frame_height
0 <= bottom_u < frame_width
0 <= bottom_v < frame_height
0.0 <= confidence <= 1.0
x1 < x2
y1 < y2
```

## 7.4 Codex phải kiểm tra

- Tracker có chạy trên ảnh đã resize không.
- Bbox có ở coordinate space của ảnh gốc hay ảnh model.
- MiDaS output sẽ resize về coordinate space nào.
- Timestamp hiện tại là capture timestamp hay processing timestamp.
- Mask có cùng resolution với frame không.

## 7.5 Definition of Done

Một frame log được:

```json
{
  "timestamp_us": 0,
  "track_id": 1,
  "bbox_xyxy": [0, 0, 0, 0],
  "center_uv": [0, 0],
  "bottom_center_uv": [0, 0],
  "confidence": 0.0,
  "frame_width": 1920,
  "frame_height": 1080
}
```

---

# 8. Phase 2 — Camera calibration và pose synchronization

## 8.1 Camera intrinsics

Yêu cầu config:

```yaml
camera:
  width: 1920
  height: 1080

  fx: 0.0
  fy: 0.0
  cx: 0.0
  cy: 0.0

  distortion:
    k1: 0.0
    k2: 0.0
    p1: 0.0
    p2: 0.0
    k3: 0.0
```

Codex phải kiểm tra:

- code hiện tại có calibration file chưa;
- resolution calibration có đúng với resolution runtime không;
- ảnh có bị crop/resize nhưng intrinsics chưa scale không;
- pixel có được undistort trước khi tạo ray không.

## 8.2 Transform chain

Bắt buộc:

```math
T_{wc}(t)=T_{wb}(t)T_{bg}(t)T_{gc}
```

Trong đó:

```text
w = ENU/world
b = UAV body
g = gimbal
c = camera
```

Không cộng Euler đơn giản trong implementation cuối.

## 8.3 Pose buffer

Cần lưu theo timestamp:

```python
@dataclass
class TelemetrySample:
    timestamp_us: int
    position_enu_m: tuple[float, float, float]
    attitude_quaternion: tuple[float, float, float, float]
    gimbal_angles_rad: tuple[float, float, float]
```

Nội suy:

```text
position: linear interpolation
attitude: quaternion SLERP
gimbal angle: shortest-path angle interpolation
```

## 8.4 Codex phải kiểm tra

- PX4 timestamp domain.
- Camera timestamp domain.
- Có time synchronization hay không.
- Gimbal data có timestamp riêng hay chỉ giá trị mới nhất.
- Có thể truy xuất pose tại capture time hay không.
- Frame latency hiện tại.

## 8.5 Failure policy

Nếu không có pose đúng timestamp:

```text
pose_valid = false
không tạo M52 candidate
không tạo MiDaS world candidate
không gửi FOLLOW_TARGET
```

---

# 9. Phase 3 — Hoàn thiện M52 geometry

## 9.1 Pixel thành ray

```math
r_c = K^{-1}
\begin{bmatrix}
u\\v\\1
\end{bmatrix}
```

API:

```python
def pixel_to_camera_ray(
    u_px: float,
    v_px: float,
    intrinsics: CameraIntrinsics,
) -> np.ndarray:
    # Return an unnormalized ray in camera frame.
    ...
```

## 9.2 Camera ray sang ENU

```math
r_{enu}=R_{wc}r_c
```

## 9.3 Ray-ground

```math
P(\lambda)=P_c+\lambda r_{enu}
```

```math
\lambda=
\frac{h_{ground}-P_{c,z}}
{r_{enu,z}}
```

Điều kiện:

```text
ray_z < 0
lambda > 0
abs(ray_z) > minimum threshold
depression angle > minimum threshold
```

## 9.4 M52 target candidate

Dùng `bottom_center_uv`.

```python
@dataclass
class M52Candidate:
    timestamp_us: int
    position_enu_m: tuple[float, float, float]

    horizontal_range_m: float
    slant_range_m: float

    ray_depression_angle_deg: float
    ray_z_enu: float

    valid: bool
    invalid_reason: str | None
```

## 9.5 Không được

- dùng center bbox cho vật thể mặt đất nếu bottom-center có sẵn;
- dùng M52 target candidate khi tia nhìn lên;
- ép M52 candidate thành valid cho vật thể bay;
- bỏ qua ground altitude reference.

## 9.6 Unit tests

```text
nadir center ray
right pixel sign
left pixel sign
ground altitude change
near-horizon invalid
upward ray invalid
ENU/WGS84 round trip
```

---

# 10. Phase 4 — Tích hợp MiDaS raw

## 10.1 Yêu cầu

MiDaS chỉ nhận RGB và trả raw relative depth/inverse depth.

Không dùng:

```python
cv2.normalize(depth, None, 0, 255, cv2.NORM_MINMAX)
```

trong core estimator.

## 10.2 Data contract

```python
@dataclass
class MidasOutput:
    timestamp_us: int
    raw_relative_depth: np.ndarray
    input_width: int
    input_height: int
    inference_time_ms: float
    valid: bool
```

## 10.3 Resize

Output phải được đưa về đúng coordinate space của `TargetObservation`.

Codex phải kiểm tra:

- tracker frame resolution;
- MiDaS input resolution;
- MiDaS output resolution;
- aspect-ratio handling;
- letterbox/crop;
- interpolation mode.

## 10.4 Target sampling

Nếu có mask:

```text
erode mask
sample vùng lõi
```

Nếu chỉ có bbox:

```text
shrink bbox khoảng 0.5–0.6
```

Tính:

```text
raw median
raw MAD
valid ratio
sample count
gradient mean
gradient max
```

---

# 11. Phase 5 — M52 ground anchors và scale MiDaS

## 11.1 Ý tưởng

Với pixel mặt đất:

```text
M52 → metric camera-axis depth Z_i
MiDaS → raw q_i
```

Fit:

```math
\frac{1}{Z_i}=a q_i+b
```

## 11.2 Ground anchor sampling

Chọn:

```text
100–500 candidate pixels/frame
```

Loại:

- target bbox/mask;
- vùng sát biên;
- sky;
- ray hướng lên;
- ray gần chân trời.

Các anchor phải có depth spread đủ lớn.

## 11.3 Camera-axis depth

Sau ray-ground:

```math
P_{g,i}^c=R_{cw}(P_{g,i}^{enu}-P_c^{enu})
```

Dùng:

```math
Z_i=P_{g,i,z}^c
```

Không dùng slant range thay cho `Z_i`.

## 11.4 Robust fitting

```text
candidate pairs
→ RANSAC
→ inlier selection
→ Huber refinement
→ quality statistics
```

## 11.5 Scale state

```python
@dataclass
class ScaleEstimate:
    timestamp_us: int

    scale_a: float
    shift_b: float
    covariance_ab: np.ndarray

    anchor_count: int
    inlier_count: int
    inlier_ratio: float
    fit_rmse: float
    depth_spread_m: float

    valid: bool
    frames_since_valid_update: int
```

## 11.6 Initial validity rules

```python
scale_valid = (
    anchor_count >= 100
    and inlier_count >= 60
    and inlier_ratio >= 0.60
    and fit_relative_error <= 0.15
    and scale_a > 0.0
    and depth_spread_m >= configured_minimum
)
```

Các ngưỡng phải nằm trong config, không hard-code rải rác.

## 11.7 Scale filter

Nếu fit tốt:

```text
Kalman update hoặc controlled EMA
```

Nếu fit xấu:

```text
prediction only
increase covariance
increase scale age
```

---

# 12. Phase 6 — Tạo MiDaS metric candidate

## 12.1 Metric depth

```math
Z_j=\frac{1}{a q_j+b}
```

Reject khi:

```text
a*q+b <= epsilon
NaN/Inf
depth ngoài physical range
```

## 12.2 Robust target depth

```math
m=median(Z_j)
```

```math
MAD=median(|Z_j-m|)
```

Giữ inlier:

```math
|Z_j-m| < 2.5MAD
```

## 12.3 Target 3D camera point

```math
P_t^c=Z_tK^{-1}
\begin{bmatrix}
u_t\\v_t\\1
\end{bmatrix}
```

## 12.4 ENU point

```math
P_t^{enu}=P_c^{enu}+R_{wc}P_t^c
```

## 12.5 Data contract

```python
@dataclass
class MidasCandidate:
    timestamp_us: int
    position_enu_m: tuple[float, float, float]

    horizontal_range_m: float
    slant_range_m: float

    metric_depth_median_m: float
    metric_depth_mad_m: float
    valid_depth_ratio: float
    sample_count: int

    scale_age_frames: int
    valid: bool
    invalid_reason: str | None
```

## 12.6 Hard invalid conditions

```text
scale invalid
scale quá cũ
sample quá ít
target quá nhỏ
depth MAD quá cao
pose invalid
denominator không dương
world point không finite
```

---

# 13. Phase 7 — Baseline không ML

Trước khi dùng XGBoost, phải có baseline.

## 13.1 Mục đích

- xác minh geometry;
- xác minh MiDaS metricization;
- tạo dữ liệu training;
- phát hiện bug trước khi ML che bug;
- có baseline để so sánh.

## 13.2 Baseline fusion

```python
if m52_valid and likely_ground_rule:
    mode = "m52_preferred"
elif midas_valid:
    mode = "midas_only"
else:
    mode = "prediction_only"
```

## 13.3 Không được bắt đầu train XGBoost nếu

- M52 chưa qua unit test;
- transform gimbal chưa xác nhận;
- MiDaS raw bị normalize;
- scale fit chưa log được;
- ground truth chưa đồng bộ;
- không có offline replay.

---

# 14. Phase 8 — Logging và dataset

## 14.1 Log mỗi frame

```text
timestamp_us
flight_id
sequence_id

target observation
camera pose
gimbal pose

M52 candidate
MiDaS raw statistics
scale estimate
MiDaS candidate

EKF prediction
innovations
NIS

ground truth
ground/free label
```

## 14.2 Ground truth chỉ dùng khi train/evaluate

Có thể lấy từ:

```text
SITL/Gazebo
RTK target trong giai đoạn thu dữ liệu
surveyed waypoint
manual measured static positions
```

Runtime cuối vẫn chỉ dùng RGB và telemetry UAV.

## 14.3 Dataset row

Một row/frame:

```text
features
ground_label
M52 horizontal error
MiDaS horizontal error
flight_id
sequence_id
location_id
timestamp
```

## 14.4 Chia dataset

Không random từng frame.

Dùng group theo:

```text
flight_id
sequence_id
location_id
```

---

# 15. Phase 9 — Preprocessing tối ưu

## 15.1 Mục tiêu

Ngăn dữ liệu runtime bất thường làm model cho kết quả không kiểm soát.

## 15.2 Chuỗi preprocessing

```text
schema validation
→ finite validation
→ physical clipping
→ train quantile clipping
→ missing flags
→ median imputation
→ log1p cho feature đuôi dài
→ StandardScaler
→ z-score clipping
```

## 15.3 StandardScaler dùng để làm gì

- thống nhất scale feature;
- giữ preprocessing train/runtime giống nhau;
- hỗ trợ OOD bằng z-score;
- tránh feature range rất khác nhau trong toàn pipeline.

XGBoost `gbtree` không bắt buộc scaler để học tốt, nhưng scaler vẫn được dùng vì pipeline còn có:

- OOD detector;
- feature monitoring;
- model compatibility checks;
- khả năng thay estimator sau này.

## 15.4 Nhóm feature

### Continuous + StandardScaler

```text
ray_depression_angle_deg
ray_z_enu
gimbal_pitch_deg
gimbal_roll_deg
normalized pixel coordinates
anchor_inlier_ratio
ground_visible_ratio
```

### log1p + StandardScaler

```text
ranges
RMSE
MAD
NIS
innovation
anchor count
scale age
candidate disagreement
variance
```

### Binary

```text
m52_valid
midas_valid
scale_valid
pose_valid
```

### Cyclic

```text
yaw_sin
yaw_cos
```

## 15.5 Physical bounds

Bắt buộc có config:

```yaml
feature_bounds:
  m52_horizontal_range_m: [0.0, 500.0]
  midas_horizontal_range_m: [0.0, 500.0]
  drone_agl_m: [1.0, 300.0]
  ray_z_enu: [-1.0, 1.0]
  anchor_inlier_ratio: [0.0, 1.0]
  target_valid_depth_ratio: [0.0, 1.0]
```

## 15.6 Z-score clipping

```python
scaled = np.clip(scaled, -8.0, 8.0)
```

Lưu trạng thái trước clip để phát hiện OOD.

## 15.7 Không leakage

Scaler, imputer, quantiles chỉ fit trên train.

---

# 16. Phase 10 — XGBoost models

## 16.1 Model A: Ground classifier

```math
p_{ground}=P(target\ touches\ ground|features)
```

Yêu cầu:

- `XGBClassifier`;
- probability calibration;
- ưu tiên giảm false-ground;
- validate theo group split.

## 16.2 Model B: M52 q90 error

Nhãn:

```math
e_M=\|P_{M52,xy}-P_{GT,xy}\|
```

Output:

```text
q90_m52_m
```

## 16.3 Model C: MiDaS q90 error

Nhãn:

```math
e_D=\|P_{MiDaS,xy}-P_{GT,xy}\|
```

Output:

```text
q90_midas_m
```

## 16.4 Vì sao q90

- tạo uncertainty bảo thủ;
- giảm nguy cơ model quá lạc quan;
- phù hợp safety gate;
- phù hợp tính reliability.

## 16.5 Runtime clamps

```python
p_ground = np.clip(p_ground, 0.001, 0.999)
q90_m52 = np.clip(q90_m52, 0.3, 200.0)
q90_midas = np.clip(q90_midas, 0.5, 200.0)
```

---

# 17. Phase 11 — Feature schema

Phải có một schema duy nhất.

Ví dụ tối thiểu:

```python
FEATURE_COLUMNS = [
    "m52_horizontal_range_m",
    "m52_slant_range_m",
    "ray_depression_angle_deg",
    "ray_z_enu",
    "drone_agl_m",

    "gimbal_pitch_deg",
    "gimbal_roll_deg",
    "gimbal_yaw_sin",
    "gimbal_yaw_cos",

    "target_center_u_norm",
    "target_center_v_norm",
    "target_bottom_u_norm",
    "target_bottom_v_norm",

    "midas_horizontal_range_m",
    "midas_slant_range_m",
    "midas_metric_mad_m",
    "target_valid_depth_ratio",

    "scale_a",
    "shift_b",
    "scale_var_a",
    "scale_var_b",

    "anchor_count",
    "anchor_inlier_count",
    "anchor_inlier_ratio",
    "anchor_fit_rmse",
    "anchor_depth_spread_m",
    "frames_since_last_valid_scale",

    "abs_range_difference_m",
    "candidate_distance_horizontal_m",

    "m52_innovation_m",
    "midas_innovation_m",
    "m52_nis",
    "midas_nis",

    "previous_weight_m52",
    "previous_weight_midas",

    "m52_valid",
    "midas_valid",
    "scale_valid",
    "pose_valid",
]
```

Codex phải điều chỉnh schema theo dữ liệu thật đang có, nhưng không được xóa feature quan trọng mà không giải thích.

---

# 18. Phase 12 — Dynamic reliability weights

## 18.1 Reliability

```math
r_M=
\frac{p_{ground}}
{\max(q90_M,e_{floor})^2}
```

```math
r_D=
\frac{p_{midas-valid}}
{\max(q90_D,e_{floor})^2}
```

## 18.2 Weight

```math
w_M=\frac{r_M}{r_M+r_D}
```

```math
w_D=1-w_M
```

## 18.3 Hard gates

```text
M52 invalid → r_M = 0
MiDaS invalid → r_D = 0
p_ground < threshold → r_M = 0
scale quá cũ → r_D giảm hoặc bằng 0
```

## 18.4 Temporal smoothing

```math
\alpha=1-e^{-\Delta t/\tau}
```

```math
w_k=w_{k-1}+\alpha(w_{raw}-w_{k-1})
```

Thêm rate limiter theo giây.

---

# 19. Phase 13 — OOD detector

## 19.1 Dựa trên raw features

Kiểm tra:

- physical bound violation;
- train quantile violation;
- missing pattern chưa từng thấy;
- schema mismatch;
- camera/MiDaS model mismatch.

## 19.2 Dựa trên StandardScaler

```text
max absolute z-score
fraction |z| > 4
fraction |z| > 6
```

## 19.3 Policy

```text
OOD nhẹ:
inflate q90

OOD cao:
prediction only

schema/model mismatch:
fail closed
```

Không được tiếp tục follow với model bundle không tương thích.

---

# 20. Phase 14 — Disagreement gate

```math
\Delta=\|P_{M52,xy}-P_{MiDaS,xy}\|
```

Ngưỡng:

```math
\tau=max(\tau_{min},k(q90_M+q90_D))
```

Logic:

```text
delta nhỏ:
fuse

delta lớn + target ground rất chắc:
M52 only

delta lớn + target không ground rất chắc:
MiDaS only

delta lớn + không rõ:
prediction only
follow disallowed
```

Không lấy trung bình mù.

---

# 21. Phase 15 — Covariance Intersection

MiDaS được scale bằng M52 anchors nên hai nguồn có tương quan.

Dùng:

```math
R_F^{-1}=w_MR_M^{-1}+w_DR_D^{-1}
```

```math
z_F=
R_F(
w_MR_M^{-1}z_M+
w_DR_D^{-1}z_D
)
```

Trong đó:

```math
R_M=q90_M^2I
```

```math
R_D=q90_D^2I
```

Chỉ fuse horizontal ENU ở giai đoạn đầu.

---

# 22. Phase 16 — Target EKF

## 22.1 State

```math
x=
[E,N,U,v_E,v_N,v_U]^T
```

## 22.2 Bản đầu

Update measurement phần:

```text
E
N
```

Vertical:

```text
prediction/log only
không dùng để điều khiển PX4 Follow Target
```

## 22.3 EKF phải có

- variable `dt`;
- constant velocity process model;
- process covariance config;
- NIS gate;
- max jump gate;
- prediction-only mode;
- stale timeout;
- reset policy khi đổi track ID.

## 22.4 Reset

Khi `track_id` thay đổi:

```text
reset target EKF
reset fusion weights
reset stale timers
không kế thừa target trước
```

---

# 23. Phase 17 — PX4 Follow Target

## 23.1 Chỉ dùng 2D trước

```text
Follow latitude/longitude
Giữ altitude UAV theo cấu hình PX4
```

## 23.2 ENU velocity → NED

```text
v_north = v_N
v_east  = v_E
v_down  = -v_U
```

## 23.3 Safety gate

Chỉ publish nếu:

```python
follow_allowed = (
    pose_valid
    and target_ekf_initialized
    and fusion_valid
    and not target_lost
    and not severe_ood
    and not telemetry_stale
    and horizontal_sigma_m < configured_threshold
    and disagreement_safe
)
```

## 23.4 Lost target policy

Ví dụ:

```text
0–0.3 s:
EKF prediction

0.3–1.0 s:
không tiến gần, giữ state bảo thủ

> 1.0 s:
target stale
stop publishing hoặc chuyển Hold theo kiến trúc hiện tại
```

Codex phải map policy này vào failsafe hiện có, không tự tạo flight mode behavior trái với chương trình.

---

# 24. Phase 18 — Model bundle

Bắt buộc lưu cùng nhau:

```text
models/xgboost_fusion/v001/
├── feature_schema.json
├── physical_bounds.json
├── train_quantiles.json
├── preprocessor.joblib
├── ground_classifier.json
├── ground_calibrator.joblib
├── m52_q90.json
├── midas_q90.json
├── feature_statistics.json
└── metadata.json
```

Metadata:

```json
{
  "model_version": "1.0.0",
  "feature_schema_hash": "",
  "camera_calibration_id": "",
  "midas_model_id": "",
  "training_dataset_version": "",
  "xgboost_version": "",
  "sklearn_version": ""
}
```

Runtime fail closed nếu mismatch.

---

# 25. Phase 19 — Offline replay

Codex phải kiểm tra codebase hiện tại có replay mechanism chưa.

Nếu chưa có, đề xuất:

```bash
python scripts/replay_estimator.py \
  --video data/flight_001.mp4 \
  --telemetry data/flight_001.csv \
  --tracking data/flight_001_tracking.jsonl \
  --model models/xgboost_fusion/v001/
```

Replay phải cho phép so sánh:

```text
M52 only
MiDaS only
fixed weights
heuristic fusion
XGBoost fusion
XGBoost + EKF
```

Output:

```text
range plot
horizontal error plot
weight plot
q90 vs actual error
candidate disagreement
EKF covariance
follow_allowed timeline
```

---

# 26. Phase 20 — Tests bắt buộc

## Geometry

```text
pixel center ray
camera nadir
gimbal rotation sign
ENU/NED sign
ray-ground
WGS84 round trip
```

## Scale fit

```text
synthetic a,b
outlier robustness
insufficient depth spread
invalid denominator
scale age
```

## Preprocessing

```text
schema order
missing value
physical clip
log1p
StandardScaler consistency
z-clip
train/runtime equality
```

## XGBoost fusion

```text
p_ground=0 → M52 weight=0
M52 q90 thấp → M52 weight tăng
MiDaS q90 thấp → MiDaS weight tăng
invalid source → reliability=0
weight sum=1
rate limiter
```

## Disagreement

```text
small delta → fuse
large delta + ground → M52 only
large delta + not-ground → MiDaS only
large delta ambiguous → prediction only
```

## EKF

```text
steady target
constant velocity
one-frame outlier
lost target
new track ID reset
```

## PX4 gate

```text
fusion invalid
pose stale
OOD
high covariance
target stale
```

---

# 27. Thứ tự patch khuyến nghị

Codex phải đề xuất patch nhỏ, không tạo một patch khổng lồ.

## Patch 1 — Audit và interfaces

```text
compatibility report
TargetObservation adapter
coordinate-frame naming
config placeholders
```

## Patch 2 — Geometry

```text
camera calibration loader
camera pose builder
pose buffer
pixel-to-ray
ray-ground
unit tests
```

## Patch 3 — MiDaS raw

```text
MiDaS adapter
raw output
resolution mapping
target sampling
```

## Patch 4 — Scale estimation

```text
anchor sampler
RANSAC
Huber fit
scale filter
tests
```

## Patch 5 — Candidates và baseline

```text
M52Candidate
MidasCandidate
heuristic fusion
structured logging
```

## Patch 6 — Replay và dataset

```text
offline replay
dataset exporter
ground-truth alignment
```

## Patch 7 — Preprocessing

```text
feature schema
physical clipper
imputer
log1p
StandardScaler
OOD statistics
```

## Patch 8 — XGBoost training

```text
group split
ground classifier
M52 q90
MiDaS q90
model bundle
evaluation
```

## Patch 9 — Runtime fusion

```text
model loader
reliability
weight smoothing
disagreement gate
covariance intersection
```

## Patch 10 — EKF và PX4

```text
Target EKF
safety gate
WGS84 conversion
FOLLOW_TARGET 2D
SITL tests
```

---

# 28. Acceptance criteria

## 28.1 Code compatibility

- Không phá tracking hiện tại.
- Không phá gimbal control hiện tại.
- Không đổi public API không cần thiết.
- Có adapter rõ ràng.
- Có config migration nếu cần.

## 28.2 Geometry

- Không nhầm ENU/NED.
- Không cộng Euler đơn giản.
- Pose dùng capture timestamp.
- Pixel coordinate space nhất quán.

## 28.3 MiDaS

- Dùng raw float output.
- Không dùng normalized visualization.
- Scale invalid được phát hiện.
- Không ground anchors thì uncertainty tăng.

## 28.4 Preprocessing

- Scaler chỉ fit trên train.
- Model bundle luôn đi kèm scaler.
- Feature schema hash được kiểm tra.
- Runtime không tạo NaN/Inf.

## 28.5 Fusion

- Không fuse khi disagreement quá lớn.
- Weight không đổi đột ngột.
- M52 weight về 0 khi target không ground.
- MiDaS weight giảm khi scale cũ hoặc quality kém.

## 28.6 EKF và Follow Target

- One-frame outlier không làm target nhảy lớn.
- Mất target không làm UAV tiếp tục lao theo vô hạn.
- Chỉ publish khi covariance đủ thấp.
- Chỉ dùng 2D Follow trong bản đầu.

---

# 29. Metrics cần báo cáo

## Geometry/M52

```text
M52 horizontal MAE
M52 P95
invalid rate
near-horizon rejection rate
```

## MiDaS

```text
range MAE
range relative error
scale fit RMSE
scale invalid rate
```

## XGBoost

```text
ground false-positive rate
Brier score
q90 coverage
q90 underestimation rate
```

## Fusion

```text
horizontal MAE
P90/P95/P99 error
catastrophic error rate
position jitter
prediction-only ratio
```

## Follow safety

```text
false follow permission
stale target response
maximum command jump
abort/hold response
```

---

# 30. Những điều Codex không được làm

- Không viết lại tracker.
- Không thay gimbal controller.
- Không thêm depth camera.
- Không thêm LiDAR/stereo.
- Không dùng kích thước target làm giả định bắt buộc.
- Không dùng MiDaS visualization output.
- Không fit scaler trên toàn dataset.
- Không random split theo frame.
- Không dùng một trọng số cố định.
- Không dùng XGBoost output trực tiếp làm tọa độ.
- Không bỏ EKF.
- Không bỏ safety gate.
- Không bật Follow Target 3D ngay.
- Không sửa PX4 parameters tự động.
- Không tạo patch lớn trước khi có compatibility report.

---

# 31. Prompt đề xuất để chạy với Codex

```text
Hãy đọc toàn bộ repository và tài liệu CODEX_AUDIT_IMPLEMENTATION_PLAN_M52_MIDAS_XGBOOST.md.

Bối cảnh:
- Tracking mục tiêu và điều khiển gimbal quay bám đã hoạt động tốt.
- Không được viết lại tracking hoặc gimbal control.
- Runtime chỉ dùng một camera RGB monocular trên gimbal.
- Không dùng depth camera, stereo, LiDAR, ToF, laser hoặc GPS mục tiêu.
- Mục tiêu là tích hợp M52 + MiDaS + StandardScaler + XGBoost + Covariance Intersection + Target EKF + PX4 FOLLOW_TARGET 2D.

Nhiệm vụ đầu tiên:
1. Không sửa code.
2. Inventory repository.
3. Tìm và ánh xạ các module hiện tại với từng khối trong tài liệu.
4. Kiểm tra timestamp, coordinate frames, camera/gimbal transforms, MiDaS output, telemetry và MAVLink.
5. Tạo docs/M52_MIDAS_XGB_COMPATIBILITY_REPORT.md.
6. Báo cáo:
   - phần có thể dùng lại;
   - phần cần adapter;
   - phần cần refactor;
   - phần phải viết mới;
   - file cụ thể sẽ sửa;
   - rủi ro;
   - Go/No-Go;
   - thứ tự patch nhỏ.
7. Chờ tôi duyệt compatibility report trước khi viết code.

Không được tự ý thay đổi kiến trúc tracking hiện tại.
```

---

# 32. Kết quả cuối cùng mong muốn

Sau khi hoàn tất toàn bộ plan, chương trình phải tạo được:

```python
@dataclass
class FinalTargetState:
    timestamp_us: int

    position_enu_m: tuple[float, float, float]
    velocity_enu_mps: tuple[float, float, float]

    covariance_enu: object

    weight_m52: float
    weight_midas: float

    fusion_mode: str

    valid: bool
    stale: bool
    follow_allowed: bool
```

Luồng cuối:

```text
tracking hiện tại
→ M52 + MiDaS metric candidates
→ StandardScaler + XGBoost reliability
→ Covariance Intersection
→ Target EKF
→ ENU/WGS84
→ FOLLOW_TARGET 2D
```

---

# 33. Nguyên tắc triển khai quan trọng nhất

```text
Không dùng machine learning để che lỗi geometry.
Không train XGBoost trước khi M52 và MiDaS candidate chạy đúng offline.
Không gửi PX4 một estimate mà hệ thống không biết uncertainty.
Không cho Follow Target chạy chỉ vì model trả về một tọa độ.
```
