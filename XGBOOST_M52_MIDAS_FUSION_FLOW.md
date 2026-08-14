# XGBoost Fusion Flow — M52 + MiDaS for UAV RGB Follow Target

## 1. Mục tiêu

Bổ sung **XGBoost** vào pipeline UAV chỉ dùng camera RGB trên gimbal để:

1. Đánh giá độ tin cậy của kết quả M52.
2. Đánh giá độ tin cậy của kết quả MiDaS.
3. Tính trọng số động cho hai nguồn theo từng frame.
4. Fuse vị trí mục tiêu trong hệ ENU.
5. Đưa measurement đã fuse vào Target EKF.
6. Chỉ gửi `FOLLOW_TARGET` khi uncertainty đủ thấp.

XGBoost **không thay thế** M52, MiDaS hoặc EKF. Vai trò của nó là dự đoán nguồn nào đáng tin hơn và nguồn đó có thể đang sai bao nhiêu.

---

## 2. Ràng buộc

Hệ thống vận hành chỉ dùng:

- Một camera RGB monocular trên gimbal.
- Generic visual tracker.
- MiDaS relative depth.
- M52 geometry/geolocation.
- Telemetry UAV và gimbal cần thiết cho M52.
- XGBoost.
- Target EKF.
- MAVLink `FOLLOW_TARGET`.
- PX4 Follow Me, ưu tiên 2D Follow.

Không dùng khi vận hành:

- Camera depth, RealSense, stereo, LiDAR, ToF hoặc laser rangefinder.
- GPS của mục tiêu.
- Kích thước thật biết trước của mục tiêu.

Thiết bị đo ngoài chỉ được dùng để tạo ground truth khi thu thập dữ liệu huấn luyện.

---

## 3. Bản chất hai nguồn

### 3.1 M52

M52 lấy pixel, camera pose và giao tia với mặt đất.

Ưu điểm:

- Có metric scale từ hình học.
- Tốt khi mục tiêu thật sự nằm trên mặt đất.
- Không phụ thuộc MiDaS.

Hạn chế:

- Sai nghiêm trọng với vật thể bay, treo hoặc không chạm đất.
- Nhạy khi tia gần song song mặt đất.
- Nhạy với attitude, góc gimbal, độ cao UAV và flat-ground assumption.

### 3.2 MiDaS

MiDaS tạo relative inverse depth từ ảnh RGB.

Ưu điểm:

- Tạo free-space candidate.
- Có thể dùng cho vật thể không nằm trên đất.

Hạn chế:

- Không tự có metric scale.
- Sai số thay đổi theo cảnh, kích thước target, background và chất lượng scale fit.
- Trong pipeline này MiDaS được metric hóa bằng ground anchors của M52, vì vậy hai nguồn có tương quan.

### 3.3 Không dùng trọng số cố định

Không dùng:

```math
P_f = 0.5P_{M52} + 0.5P_{MiDaS}
```

Mỗi frame cần trọng số khác nhau theo geometry, tracker quality, MiDaS scale quality và temporal consistency.

---

## 4. Kiến trúc tổng thể

```text
RGB frame + timestamp
        |
        +-------------------+
        |                   |
        v                   v
Generic tracker           MiDaS
bbox/mask/ID/conf    relative depth map
        |                   |
        +---------+---------+
                  |
                  v
          M52 geometry layer
  camera pose + gimbal + ray-ground
                  |
        +---------+----------+
        |                    |
        v                    v
 M52 target candidate   MiDaS target candidate
        |                    |
        +---------+----------+
                  |
                  v
            Feature Builder
                  |
       +----------+-----------+
       |          |           |
       v          v           v
XGB Ground   XGB M52 Error  XGB MiDaS Error
Classifier     Regressor       Regressor
       |          |           |
       +----------+-----------+
                  |
                  v
      Reliability Weight Calculator
                  |
                  v
     Disagreement and Safety Gating
                  |
                  v
        Fused ENU measurement
                  |
                  v
              Target EKF
                  |
                  v
         ENU → WGS84 / NED velocity
                  |
                  v
          MAVLink FOLLOW_TARGET
```

---

## 5. Data contracts

### 5.1 Tracker output

```python
@dataclass
class TargetObservation:
    timestamp_us: int
    track_id: int
    bbox_xyxy: tuple[float, float, float, float]
    center_uv: tuple[float, float]
    bottom_center_uv: tuple[float, float]
    confidence: float
    mask: object | None
    lost_frames: int
```

### 5.2 M52 candidate

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
```

### 5.3 MiDaS candidate

```python
@dataclass
class MidasCandidate:
    timestamp_us: int
    position_enu_m: tuple[float, float, float]
    horizontal_range_m: float
    slant_range_m: float
    target_depth_median_m: float
    target_depth_mad_m: float
    valid_depth_ratio: float
    valid: bool
```

### 5.4 Scale estimate

```python
@dataclass
class ScaleEstimate:
    timestamp_us: int
    scale_a: float
    shift_b: float
    covariance_ab: object
    anchor_count: int
    anchor_inlier_count: int
    anchor_inlier_ratio: float
    fit_rmse: float
    depth_spread_m: float
    frames_since_last_valid_update: int
    valid: bool
```

### 5.5 XGBoost predictions

```python
@dataclass
class XGBoostPredictions:
    timestamp_us: int
    p_ground: float
    variance_m52_m2: float
    variance_midas_m2: float
    ood_score: float
    valid: bool
```

### 5.6 Fusion result

```python
@dataclass
class FusionResult:
    timestamp_us: int
    position_enu_xy_m: tuple[float, float]
    covariance_xy: object
    weight_m52: float
    weight_midas: float
    candidate_disagreement_m: float
    mode: str
    valid: bool
    reason: str
```

Fusion modes:

```text
weighted
m52_only
midas_only
prediction_only
invalid
```

---

## 6. Ba model XGBoost chính

### 6.1 Model A — Ground Validity Classifier

Dùng `XGBClassifier` để dự đoán:

```math
p_g = P(\text{target nằm trên mặt đất} \mid x)
```

Output:

```text
p_ground ∈ [0,1]
```

Label:

```python
ground_label = int(target_touches_ground)
```

Diễn giải:

- `p_ground` gần 1: M52 có khả năng hợp lệ.
- `p_ground` gần 0: không dùng M52 target ray-ground.

### 6.2 Model B — M52 Error Regressor

Dùng `XGBRegressor` dự đoán log squared error:

```math
y_M = \log(e_M^2 + \epsilon)
```

Trong đó:

```math
e_M = \left\|P_{M52,xy} - P_{GT,xy}\right\|
```

Khôi phục phương sai:

```math
\hat{\sigma}_M^2 = \exp(\hat{y}_M)
```

### 6.3 Model C — MiDaS Error Regressor

```math
y_D = \log(e_D^2 + \epsilon)
```

```math
e_D = \left\|P_{MiDaS,xy} - P_{GT,xy}\right\|
```

```math
\hat{\sigma}_D^2 = \exp(\hat{y}_D)
```

Không nên cho XGBoost dự đoán trực tiếp tọa độ mục tiêu. XGBoost chỉ dự đoán validity và error để giữ lại cấu trúc vật lý của M52/MiDaS.

---

## 7. Feature engineering

### 7.1 M52 geometry features

```text
m52_horizontal_range_m
m52_slant_range_m
ray_depression_angle_deg
ray_z_enu
drone_agl_m
gimbal_pitch_deg
gimbal_yaw_deg
gimbal_roll_deg
target_bottom_u_norm
target_bottom_v_norm
target_center_u_norm
target_center_v_norm
distance_to_horizon_px
m52_valid
```

Lưu ý:

- `ray_z_enu` gần 0 nghĩa là tia gần song song mặt đất.
- Depression angle nhỏ làm M52 nhạy sai số.
- Bottom-center gần ground region làm M52 đáng tin hơn.

### 7.2 MiDaS features

```text
midas_horizontal_range_m
midas_slant_range_m
midas_raw_median
midas_raw_mad
midas_metric_median_m
midas_metric_mad_m
target_valid_depth_ratio
target_depth_gradient_mean
target_depth_gradient_max
target_depth_sample_count
midas_candidate_valid
```

### 7.3 Scale-fit features

```text
scale_a
shift_b
scale_var_a
scale_var_b
anchor_count
anchor_inlier_count
anchor_inlier_ratio
anchor_fit_rmse
anchor_depth_min_m
anchor_depth_max_m
anchor_depth_spread_m
ground_visible_ratio
frames_since_last_valid_scale
scale_valid
```

### 7.4 Tracker features

```text
tracker_confidence
bbox_area_ratio
bbox_width_px
bbox_height_px
bbox_aspect_ratio
bbox_center_u_norm
bbox_center_v_norm
bbox_center_velocity_px_s
bbox_scale_change_rate
target_near_image_edge
lost_frames
mask_available
mask_area_ratio
```

### 7.5 Cross-source features

```text
abs_range_difference_m
relative_range_difference
candidate_distance_enu_m
candidate_distance_horizontal_m
m52_minus_midas_e_m
m52_minus_midas_n_m
```

```math
\Delta d = |d_M-d_D|
```

```math
\Delta d_{rel} = \frac{|d_M-d_D|}{\max(d_M,d_D,\epsilon)}
```

### 7.6 Temporal/EKF features

```text
m52_innovation_m
midas_innovation_m
m52_nis
midas_nis
previous_weight_m52
previous_weight_midas
ekf_horizontal_sigma_m
ekf_speed_mps
time_since_last_valid_target_s
```

---

## 8. Feature schema cố định

Tạo một danh sách `FEATURE_COLUMNS` cố định. Mỗi model phải lưu kèm:

```text
model version
feature schema hash
training dataset version
MiDaS model version
camera calibration version
Git commit
training date
metrics
```

Runtime bắt buộc kiểm tra:

```python
assert list(features.columns) == FEATURE_COLUMNS
```

Không được thay đổi thứ tự hoặc ý nghĩa feature mà không train lại model.

---

## 9. Ground truth và training labels

Trong giai đoạn thu thập dữ liệu, cần có:

```text
target_position_ground_truth
target_range_ground_truth
target_ground_or_free_label
```

Nguồn ground truth có thể là:

- PX4 SITL/Gazebo.
- Target mang RTK chỉ trong giai đoạn thu dataset.
- Vị trí đã khảo sát trước.
- Thiết bị đo ngoài dùng offline.

### 9.1 Labels

```python
ground_label = int(target_touches_ground)

error_m52_m = np.linalg.norm(
    np.asarray(m52_position_enu_m[:2])
    - np.asarray(gt_position_enu_m[:2])
)

target_log_var_m52 = np.log(
    error_m52_m**2 + 1e-4
)

error_midas_m = np.linalg.norm(
    np.asarray(midas_position_enu_m[:2])
    - np.asarray(gt_position_enu_m[:2])
)

target_log_var_midas = np.log(
    error_midas_m**2 + 1e-4
)
```

Nếu candidate invalid:

- Không dùng sample đó để train error regressor tương ứng.
- Vẫn có thể dùng để train ground classifier.

---

## 10. Chia dataset

Không chia random theo frame.

Chia theo:

```text
flight_id
video_id
location_id
target_sequence_id
```

Ví dụ:

```text
Train: Flight 001–020
Validation: Flight 021–025
Test: Flight 026–030
```

Mục tiêu là tránh leakage giữa các frame liên tiếp gần như giống nhau.

---

## 11. Cấu hình XGBoost baseline

```python
COMMON_PARAMS = {
    "n_estimators": 500,
    "max_depth": 5,
    "learning_rate": 0.05,
    "subsample": 0.8,
    "colsample_bytree": 0.8,
    "min_child_weight": 5,
    "reg_lambda": 5.0,
    "reg_alpha": 0.1,
    "n_jobs": -1,
    "random_state": 42,
}
```

Classifier:

```python
ground_model = XGBClassifier(
    **COMMON_PARAMS,
    objective="binary:logistic",
    eval_metric="logloss",
)
```

Regressors:

```python
m52_error_model = XGBRegressor(
    **COMMON_PARAMS,
    objective="reg:squarederror",
    eval_metric="mae",
)

midas_error_model = XGBRegressor(
    **COMMON_PARAMS,
    objective="reg:squarederror",
    eval_metric="mae",
)
```

Dùng early stopping với validation set.

---

## 12. Runtime inference

### 12.1 Predict validity và error

```python
p_ground = float(
    ground_model.predict_proba(features)[0, 1]
)

log_var_m52 = float(
    m52_error_model.predict(features)[0]
)

log_var_midas = float(
    midas_error_model.predict(features)[0]
)

sigma2_m52 = float(np.exp(log_var_m52))
sigma2_midas = float(np.exp(log_var_midas))
```

Clamp:

```python
sigma2_m52 = np.clip(
    sigma2_m52,
    MIN_VARIANCE_M2,
    MAX_VARIANCE_M2,
)

sigma2_midas = np.clip(
    sigma2_midas,
    MIN_VARIANCE_M2,
    MAX_VARIANCE_M2,
)
```

---

## 13. Reliability và trọng số

### 13.1 Reliability M52

```math
r_M = \frac{p_g}{\hat{\sigma}_M^2+\epsilon}
```

```python
reliability_m52 = (
    p_ground / (sigma2_m52 + 1e-6)
)
```

### 13.2 Reliability MiDaS

```math
r_D = \frac{1}{\hat{\sigma}_D^2+\epsilon}
```

```python
reliability_midas = (
    1.0 / (sigma2_midas + 1e-6)
)
```

### 13.3 Candidate validity

```python
if not m52_candidate.valid:
    reliability_m52 = 0.0

if not midas_candidate.valid:
    reliability_midas = 0.0

if p_ground < 0.15:
    reliability_m52 = 0.0
```

### 13.4 Chuyển reliability thành trọng số

```math
w_M = \frac{r_M}{r_M+r_D}
```

```math
w_D = 1-w_M
```

```python
total_reliability = reliability_m52 + reliability_midas

if total_reliability <= 1e-9:
    fusion_valid = False
else:
    raw_weight_m52 = reliability_m52 / total_reliability
    raw_weight_midas = 1.0 - raw_weight_m52
```

---

## 14. Làm mượt trọng số

Không dùng raw weights trực tiếp.

### EMA

```python
smoothed_weight_m52 = (
    0.8 * previous_weight_m52
    + 0.2 * raw_weight_m52
)
```

### Rate limiting

```python
smoothed_weight_m52 = np.clip(
    smoothed_weight_m52,
    previous_weight_m52 - 0.10,
    previous_weight_m52 + 0.10,
)
```

### Normalize

```python
weight_m52 = float(
    np.clip(smoothed_weight_m52, 0.0, 1.0)
)
weight_midas = 1.0 - weight_m52
```

Mục tiêu: ngăn trọng số đổi từ `0.9` sang `0.1` chỉ vì một frame lỗi.

---

## 15. Disagreement gate

Tính khoảng cách ngang ENU giữa hai candidate:

```math
\Delta = \left\|P_{M,xy}-P_{D,xy}\right\|
```

Ngưỡng:

```math
\tau = 3\sqrt{\hat{\sigma}_M^2+\hat{\sigma}_D^2}
```

```python
candidate_disagreement_m = np.linalg.norm(
    np.asarray(m52_candidate.position_enu_m[:2])
    - np.asarray(midas_candidate.position_enu_m[:2])
)

disagreement_gate_m = (
    3.0 * np.sqrt(sigma2_m52 + sigma2_midas)
)
```

Nếu bất đồng vượt ngưỡng:

```python
if candidate_disagreement_m > disagreement_gate_m:
    if (
        p_ground > 0.85
        and sigma2_m52 < sigma2_midas
        and m52_candidate.valid
    ):
        mode = "m52_only"

    elif (
        p_ground < 0.15
        and midas_candidate.valid
        and sigma2_midas < MAX_USABLE_VARIANCE_M2
    ):
        mode = "midas_only"

    else:
        mode = "prediction_only"
        fusion_valid = False
        follow_allowed = False
```

Không lấy trung bình mù khi hai nguồn bất đồng mạnh.

---

## 16. Fusion horizontal ENU

Bản đầu chỉ fuse thành phần ngang.

```math
P_M = \begin{bmatrix}E_M\\N_M\end{bmatrix}
```

```math
P_D = \begin{bmatrix}E_D\\N_D\end{bmatrix}
```

```math
P_F = w_MP_M+w_DP_D
```

```python
position_fused_xy = (
    weight_m52 * np.asarray(m52_candidate.position_enu_m[:2])
    + weight_midas * np.asarray(midas_candidate.position_enu_m[:2])
)
```

Không fuse altitude trực tiếp trong phiên bản đầu. Dùng PX4 2D Follow.

---

## 17. Covariance sau fusion

Giả định độc lập đơn giản:

```math
\sigma_F^2 = \frac{1}{r_M+r_D}
```

Nhưng vì M52 ground anchors được dùng để scale MiDaS, hai nguồn có tương quan. Inflate covariance:

```math
\sigma_{F,safe}^2 = \gamma\sigma_F^2
```

Khởi đầu:

```text
gamma = 2.0
```

```python
base_fused_variance = 1.0 / max(
    total_reliability,
    1e-9,
)

safe_fused_variance = (
    covariance_inflation_gamma
    * base_fused_variance
)

measurement_covariance_xy = np.diag([
    safe_fused_variance,
    safe_fused_variance,
])
```

---

## 18. Target EKF integration

XGBoost fusion chỉ tạo measurement và covariance.

Target EKF chịu trách nhiệm:

- Temporal smoothing.
- Velocity estimation.
- NIS gating.
- Target-lost prediction.
- Output state cho Follow Target.

```python
if fusion_result.valid:
    target_ekf.update_horizontal(
        position_enu_xy=fusion_result.position_enu_xy_m,
        covariance_xy=fusion_result.covariance_xy,
    )
else:
    target_ekf.predict_only()
```

EKF vẫn phải kiểm tra NIS của measurement fused.

---

## 19. Safety gate trước PX4

```python
follow_allowed = (
    tracker_observation.confidence > 0.60
    and target_ekf.initialized
    and fusion_result.valid
    and not pose_stale
    and not target_lost
    and target_ekf.horizontal_sigma_m < 3.0
    and max(
        fusion_result.weight_m52,
        fusion_result.weight_midas,
    ) > 0.55
    and fusion_result.candidate_disagreement_m
        < MAX_FOLLOW_DISAGREEMENT_M
)
```

Nếu không đạt:

- Không publish target mới hoặc đánh dấu stale.
- UAV giữ vị trí/Hold.
- Không tiếp tục tiến về target.
- Tracker/gimbal vẫn có thể giữ mục tiêu trong FOV.

---

## 20. PX4 output

Giai đoạn đầu dùng:

```text
FLW_TGT_ALT_M = 0
```

Dùng Target EKF state để tạo:

```text
latitude
longitude
velocity NED
position covariance
```

ENU velocity sang NED:

```python
v_north = target_velocity_enu[1]
v_east = target_velocity_enu[0]
v_down = -target_velocity_enu[2]
```

Gửi `FOLLOW_TARGET` ở 10–20 Hz.

---

## 21. Pseudocode runtime hoàn chỉnh

```python
def process_frame(frame, telemetry, previous_state):
    tracker_obs = tracker.update(frame.image_bgr)
    midas_raw = midas.infer_raw(frame.image_bgr)

    camera_pose = pose_buffer.interpolate(
        frame.timestamp_us
    )

    scale_estimate = scale_pipeline.update(
        frame=frame,
        midas_raw=midas_raw,
        camera_pose=camera_pose,
        target_exclusion=tracker_obs,
    )

    m52_candidate = m52_target_builder.build(
        tracker_observation=tracker_obs,
        camera_pose=camera_pose,
    )

    midas_candidate = midas_target_builder.build(
        tracker_observation=tracker_obs,
        midas_raw=midas_raw,
        scale_estimate=scale_estimate,
        camera_pose=camera_pose,
    )

    ekf_prediction = target_ekf.predict(
        frame.timestamp_us
    )

    features = feature_builder.build(
        tracker_observation=tracker_obs,
        m52_candidate=m52_candidate,
        midas_candidate=midas_candidate,
        scale_estimate=scale_estimate,
        ekf_prediction=ekf_prediction,
        previous_weights=previous_state.weights,
    )

    p_ground = ground_model.predict_proba(
        features
    )[0, 1]

    sigma2_m52 = np.exp(
        m52_error_model.predict(features)[0]
    )

    sigma2_midas = np.exp(
        midas_error_model.predict(features)[0]
    )

    fusion_result = fusion_engine.fuse(
        p_ground=p_ground,
        sigma2_m52=sigma2_m52,
        sigma2_midas=sigma2_midas,
        m52_candidate=m52_candidate,
        midas_candidate=midas_candidate,
        previous_weights=previous_state.weights,
    )

    if fusion_result.valid:
        target_ekf.update_horizontal(
            fusion_result.position_enu_xy_m,
            fusion_result.covariance_xy,
        )

    target_state = target_ekf.state()

    follow_allowed = safety_gate.evaluate(
        tracker_observation=tracker_obs,
        fusion_result=fusion_result,
        target_state=target_state,
        camera_pose=camera_pose,
    )

    if follow_allowed:
        follow_target_sender.publish(target_state)

    logger.write(
        frame=frame,
        tracker=tracker_obs,
        scale=scale_estimate,
        m52=m52_candidate,
        midas=midas_candidate,
        p_ground=p_ground,
        sigma2_m52=sigma2_m52,
        sigma2_midas=sigma2_midas,
        fusion=fusion_result,
        target_state=target_state,
        follow_allowed=follow_allowed,
    )
```

---

## 22. Logging bắt buộc

Mỗi frame cần log:

```text
timestamp
flight_id
sequence_id
tracker confidence
bbox/mask stats
camera pose
gimbal angles
M52 position/range
MiDaS position/range
MiDaS target MAD
scale a/shift b
anchor count/inlier ratio/fit RMSE
all feature values
p_ground
predicted sigma M52
predicted sigma MiDaS
raw weights
smoothed weights
candidate disagreement
fusion mode
fused position/covariance
EKF innovation/NIS
EKF position/velocity
follow_allowed
ground truth nếu có
```

Log phải đủ để train lại model và replay offline.

---

## 23. OOD và fallback

XGBoost có thể gặp dữ liệu ngoài distribution:

- Loại target mới.
- Ánh sáng khác.
- Camera/gimbal geometry mới.
- MiDaS model thay đổi.
- Camera calibration thay đổi.
- Địa hình mới.

Lưu statistics của training features:

```text
min
max
quantile 1%
quantile 99%
```

Nếu OOD cao:

```python
sigma2_m52 *= 3.0
sigma2_midas *= 3.0
```

Nếu model không load được, dùng heuristic fallback. Nếu cả hai nguồn không đáng tin:

```text
prediction_only
follow disallowed
```

---

## 24. Cấu trúc thư mục đề xuất

```text
src/
├── fusion_xgboost/
│   ├── feature_schema.py
│   ├── feature_builder.py
│   ├── model_bundle.py
│   ├── reliability.py
│   ├── disagreement_gate.py
│   ├── weight_filter.py
│   ├── fusion_engine.py
│   ├── ood_detector.py
│   └── types.py
├── training/
│   ├── build_dataset.py
│   ├── split_dataset.py
│   ├── train_ground_classifier.py
│   ├── train_m52_error_model.py
│   ├── train_midas_error_model.py
│   ├── evaluate_models.py
│   └── export_model_bundle.py
├── models/
│   └── xgboost_fusion/
│       ├── ground_classifier.json
│       ├── m52_error_model.json
│       ├── midas_error_model.json
│       ├── feature_schema.json
│       ├── feature_stats.json
│       └── metadata.json
└── tests/
    ├── test_feature_builder.py
    ├── test_reliability.py
    ├── test_disagreement_gate.py
    ├── test_weight_filter.py
    └── test_fusion_engine.py
```

---

## 25. Thứ tự triển khai cho Codex

1. Bổ sung logging đủ feature và ground truth.
2. Tạo feature schema và schema hash.
3. Viết dataset builder.
4. Chia train/validation/test theo flight hoặc sequence.
5. Train Ground Validity Classifier.
6. Train M52 Error Regressor.
7. Train MiDaS Error Regressor.
8. Viết model bundle loader.
9. Implement reliability calculator.
10. Implement weight smoothing và rate limiter.
11. Implement disagreement gate.
12. Implement horizontal ENU fusion và covariance inflation.
13. Tích hợp Target EKF.
14. Tích hợp safety gate trước PX4.
15. Viết offline replay so sánh các baseline.
16. Test trong PX4 SITL.

---

## 26. Unit tests bắt buộc

- `p_ground=0` → M52 weight bằng 0.
- M52 variance nhỏ hơn → M52 weight cao hơn.
- MiDaS variance nhỏ hơn → MiDaS weight cao hơn.
- Candidate invalid → reliability bằng 0.
- Weight luôn trong `[0,1]`.
- Tổng weight bằng 1 khi fusion valid.
- Weight jump không quá giới hạn mỗi frame.
- High disagreement kích hoạt single-source hoặc prediction-only.
- Fusion invalid chặn Follow Target.
- Feature schema mismatch làm model không được chạy.

---

## 27. Acceptance criteria

Pipeline đạt yêu cầu prototype khi:

1. Giảm horizontal MAE so với heuristic baseline.
2. Giảm 95th percentile error so với fixed 50/50.
3. Không dùng M52 thường xuyên với target không nằm trên đất.
4. Khi MiDaS scale kém, trọng số MiDaS giảm rõ ràng.
5. Trọng số không rung mạnh giữa các frame.
6. High disagreement không bị fuse mù.
7. EKF covariance tăng khi cả hai nguồn kém.
8. Follow Target bị chặn khi fusion invalid.
9. Offline replay tái tạo đúng runtime.
10. Model metadata và feature schema được kiểm tra trước khi chạy.

Mục tiêu ban đầu:

```text
Giảm horizontal MAE ít nhất 10% so với heuristic baseline.
Giảm 95th percentile error ít nhất 15%.
False follow permission khi measurement lỗi: dưới 1%.
Weight jump mỗi frame: không quá 0.10.
```

---

## 28. Quy tắc an toàn

Không được:

- Cho XGBoost xuất target position trực tiếp và bỏ qua M52/MiDaS.
- Cho XGBoost bypass Target EKF.
- Dùng M52 khi `p_ground` rất thấp.
- Dùng MiDaS khi scale invalid lâu.
- Fuse hai candidate khi disagreement quá lớn.
- Cho weight đổi không giới hạn giữa hai frame.
- Gửi `FOLLOW_TARGET` khi fusion invalid.
- Chia train/test random theo frame.
- Thay feature schema mà không retrain model.
- Dùng model với camera, calibration hoặc MiDaS version khác mà không đánh giá lại.

---

## 29. Tóm tắt một câu

```text
XGBoost không trực tiếp đo khoảng cách.
XGBoost dự đoán M52 và MiDaS có thể sai bao nhiêu,
chuyển sai số đó thành trọng số động,
fuse measurement ngang ENU,
sau đó Target EKF và safety gate mới quyết định
có gửi FOLLOW_TARGET cho PX4 hay không.
```
