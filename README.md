# Swarm Sparrow Dashboard

This copy keeps the original dashboard runtime and uses the native
`sparrow_gimbal` Gazebo plant for both PX4 instances. The project-local model
root contains the Sparrow body, motors, three-axis camera gimbal and the front
lidar retained for dashboard range features.

Install `patches/px4-airframes/4020_gz_sparrow_gimbal` into the matching PX4
airframe directory, list it in PX4's airframe `CMakeLists.txt`, rebuild
`px4_sitl_default`, then check the stack without starting it:

```bash
cd /home/sup/swarm_sparrow_dashboard
./run_all.sh --check
```

The default `.env` keeps OFFBOARD authority, missions and CBF-RL inactive.
Use `sparrow_20m_10ms_shadow.env` after its generated policy and SHA are
present. ACTIVE flight still requires the existing explicit acknowledgement
and flight-readiness gates.

> **Kiến trúc hiện tại (2026-07-29):** runtime đã loại bỏ hoàn toàn UniDepth
> và các module `metric_depth_*`. Follow Target hiện dùng bearing-only
> multi-view triangulation/EKF từ camera RGB, có timestamp synchronization và
> observability gate. Các đoạn UniDepth nằm sâu hơn trong tài liệu này là ghi
> chép lịch sử của pipeline cũ, không còn là hướng dẫn runtime hợp lệ. Xem
> `VISUAL_FOLLOW_TARGET.md` và `.env.example` cho cấu hình hiện hành.
> Hợp đồng StandardScaler/XGBoost residual correction offline/shadow nằm trong
> `RANGE_RESIDUAL_CORRECTION.md`; mặc định runtime vẫn là `off`.

Dashboard điều khiển và theo dõi hai UAV PX4 (`UAV-01`, `UAV-02`) trong
Gazebo. Backend FastAPI nhận telemetry qua MQTT, đọc camera/IMU từ Gazebo,
chạy visual tracking và gửi lệnh điều khiển qua MQTT. Tiến trình
`mavlink_manual_bridge.py` chuyển lệnh MQTT thành MAVLink cho PX4.

> **Cảnh báo an toàn:** đây là phần mềm thử nghiệm cho PX4 SITL. Chỉ chạy với
> một instance `mavlink_manual_bridge.py`. Trước khi thử tracking/follow, kiểm
> tra UAV không ở trạng thái failsafe và luôn có cách dừng tracking, Hold/Land
> hoặc Disarm. Không dùng trực tiếp trên UAV thật nếu chưa có quy trình kiểm
> thử và giới hạn an toàn riêng.

## 1. Nội dung gói bàn giao

Các file chạy chính:

- `main.py`: FastAPI backend, MQTT, Gazebo bridge và API/WebSocket.
- `static/index.html`: giao diện web.
- `mavlink_manual_bridge.py`: MQTT → MAVLink cho PX4.
- `tracking_web.py`: quản lý tracker, gimbal và visual follow.
- `tracking_hybrid.py`: KCF tracker kết hợp các quality gate từ pipeline
  LightFC: Kalman, temporal template memory, PSR, shape/motion gate,
  distractor ambiguity và motion-guided re-detection.
- `visual_follow_target.py`: lọc bbox, ước lượng khoảng cách và tọa độ mục tiêu.
- `body_attitude_recenter.py`: bộ điều khiển recenter thân UAV.
- `metric_depth_estimator.py`: UniDepth V2 và worker inference bất đồng bộ.
- `isolated_swarm.launch.py`: ROS 2 launch cho telemetry/control.
- `run_ros_telemetry_local.sh`: chạy ROS stack từ workspace ngoài.
- `start_gazebo_optimized.sh`: chạy Gazebo server/GUI.
- `gazebo_gui_light.config`: cấu hình Gazebo GUI nhẹ.
- `test_*.py`: unit test.

Các thư mục `.venv`, `build`, `install`, `log`, `__pycache__`, file backup và
bản `swarm_app/` cũ không nằm trong gói. Model UniDepth và Hugging Face cache
cũng không được đóng gói.

## 2. Phụ thuộc ngoài gói

Máy nhận cần có:

- Linux, Python 3.12 và công cụ tạo virtual environment.
- MQTT broker tại `127.0.0.1:1883` (thường dùng Mosquitto).
- PX4 SITL và Gazebo tương thích với Python bindings `gz.transport13` và
  `gz.msgs10`.
- ROS 2 workspace có package `swarm_telemetry` với các executable
  `telemetry_node`, `mqtt_bridge`, `web_control_node`.
- Tracking package `lfc_gimbal_gazebo`; đặt đường dẫn bằng
  `SWARM_TRACKING_PACKAGE_ROOT`.
- PX4 gửi MAVLink heartbeat của system ID 1/2 đến UDP 14540/14541.

Frontend tải Leaflet và bản đồ OpenStreetMap qua Internet. Khi offline,
dashboard vẫn có thể mở nhưng phần bản đồ có thể không hiển thị.

Các model/world đã chỉnh sửa nằm ngoài thư mục dự án này nên **không có trong
file nén**. Để tái tạo đúng môi trường hiện tại, cần bàn giao thêm PX4
Autopilot workspace và ROS 2 workspace tương ứng.

## 3. Cài đặt Python

Giải nén, mở terminal tại thư mục dự án:

```bash
python3.12 -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install -r requirements.txt
```

Gazebo Python bindings thường được cài từ hệ điều hành/PX4 setup, không qua
`requirements.txt`. `main.py` tự thêm `/usr/lib/python3/dist-packages` để
virtualenv có thể tìm các binding này.

Để dùng chế độ follow 8–12 m của cấu hình bàn giao, cài thêm PyTorch,
UniDepth và model `lpiccinelli/unidepth-v2-vitl14`. Các biến tương ứng đã
được bật trong `.env.example`. Nếu UniDepth chưa sẵn sàng, invalid hoặc quá
cũ, controller giữ vận tốc tiến/lùi bằng 0 và không fallback sang bbox/LUT.

Backend tracking mặc định là `hybrid`. Nó dùng KCF cho đường chạy nhanh mỗi
frame và kết hợp các phần an toàn từ
`lfc_gimbal_gazebo/services/tracker_service.py`: Kalman prediction, appearance
memory, PSR gate, scale/shape penalty, phát hiện peak mơ hồ, tìm lại cục bộ
theo hướng chuyển động rồi mới mở rộng toàn frame. Bbox đang redetect, mơ hồ
hoặc bị quality gate từ chối không được gửi sang UniDepth/controller.

Trong trạng thái tracking bình thường, `MultiScaleBBoxRefiner` thử một tập
scale nhỏ quanh tâm KCF/Kalman. Candidate được chấm bằng template correlation,
appearance memory, texture và shape continuity; scale mơ hồ/outlier bị giữ ở
kích thước trước đó. KCF được khởi tạo lại khi scale đã chấp nhận thay đổi đủ
lớn. `VisualBBoxFilter` dùng alpha kích thước động theo confidence, vì vậy bbox
vẫn chặn jump nhưng không còn che mất thay đổi scale thật.

Neural core LightFC chỉ có thể chạy bằng `SWARM_TRACKING_BACKEND=onnx` khi có
file `lightfc_core.onnx` thật và package `onnxruntime`. Không chuyển mặc định
sang ONNX nếu model đang là symlink hỏng; status của hybrid hiển thị
`lightfc_robust_gates=true`, `neural_core=false` để không gây hiểu nhầm.

Camera Gazebo của profile accuracy phát frame 1280×720. `raw_frames` giữ
nguyên độ phân giải này cho tracker và UniDepth; chỉ JPEG gửi tới trình duyệt
được resize theo `SWARM_CAMERA_PREVIEW_MAX_WIDTH` hoặc
`SWARM_TRACKING_PREVIEW_MAX_WIDTH` (mặc định 640 px). Không tăng preview width
để “tăng accuracy”, vì preview không có motion authority.

## 4. Cấu hình đường dẫn

Tạo file cấu hình cá nhân:

```bash
cp .env.example .env
```

Sửa ít nhất:

```dotenv
SWARM_TRACKING_PACKAGE_ROOT=/duong/dan/toi/lfc_gimbal_gazebo
SWARM_ROS_SETUP=/duong/dan/toi/ros_workspace/install/setup.bash
PX4_AUTOPILOT_ROOT=/duong/dan/toi/PX4-Autopilot
```

Nạp cấu hình vào mỗi terminal trước khi chạy:

```bash
set -a
source .env
set +a
```

Profile follow UniDepth V2 hiện tại:

```dotenv
SWARM_METRIC_DEPTH_ENABLED=true
SWARM_METRIC_DEPTH_BACKEND=unidepth
SWARM_METRIC_DEPTH_MODEL=lpiccinelli/unidepth-v2-vitl14
SWARM_METRIC_DEPTH_DEVICE=auto
SWARM_METRIC_DEPTH_EVERY_N=1
SWARM_METRIC_DEPTH_FOLLOW_CONTROL=true
SWARM_METRIC_DEPTH_MAX_AGE_S=0.5
SWARM_CAMERA_WIDTH=1280
SWARM_CAMERA_HEIGHT=720
SWARM_CAMERA_HORIZONTAL_FOV_RAD=1.35
SWARM_METRIC_DEPTH_RESOLUTION_LEVEL=9.0
SWARM_METRIC_DEPTH_ROI_INSET=0.20
SWARM_METRIC_DEPTH_UNCERTAINTY_KEEP_QUANTILE=0.70
SWARM_METRIC_DEPTH_MAX_ROI_MAD_M=1.0
SWARM_METRIC_DEPTH_MAX_BACKGROUND_FRACTION=0.85
SWARM_METRIC_DEPTH_TARGET_CROP_ENABLED=false
SWARM_METRIC_DEPTH_FILTER=confidence_weighted_ema
SWARM_METRIC_DEPTH_EMA_ALPHA=0.65
SWARM_METRIC_DEPTH_CALIBRATION_ENABLED=false
SWARM_METRIC_DEPTH_GROUND_TRUTH_ENABLED=false
SWARM_TRACKING_FOLLOW_MIN_DISTANCE_M=8.0
SWARM_TRACKING_FOLLOW_MAX_DISTANCE_M=12.0
SWARM_TRACKING_FOLLOW_BRAKE_M_S2=1.5
SWARM_TRACKING_OFFBOARD_MAX_FORWARD_M_S=3.0
SWARM_TRACKING_SCALE_REFINER_ENABLED=true
SWARM_TRACKING_SCALE_UPDATE_EVERY_N=1
SWARM_TRACKING_SCALE_CANDIDATES=0.85,0.92,0.97,1.0,1.03,1.08,1.15
SWARM_TRACKING_SCALE_MIN_PSR=3.5
SWARM_TRACKING_SCALE_MIN_SCORE=0.68
SWARM_TRACKING_SCALE_MAX_STEP_RATIO=0.18
SWARM_TRACKING_SCALE_KCF_REINIT_RATIO=0.05
SWARM_TRACKING_BBOX_CENTER_ALPHA=0.55
SWARM_TRACKING_BBOX_SIZE_ALPHA=0.20
SWARM_TRACKING_BBOX_MAX_SIZE_ALPHA=0.48
SWARM_TRACKING_OFFBOARD_MAX_YAW_RATE_DEG_S=35
SWARM_TRACKING_OFFBOARD_YAW_SLEW_DEG_S2=120
SWARM_TRACKING_GIMBAL_KP=2.5
SWARM_TRACKING_GIMBAL_KI=0.0
SWARM_TRACKING_GIMBAL_KD=0.05
SWARM_TRACKING_GIMBAL_EMA_ALPHA=0.75
SWARM_TRACKING_GIMBAL_DEADBAND_DEG=1.25
SWARM_TRACKING_GIMBAL_OMEGA_MAX_DEG=30
SWARM_TRACKING_GIMBAL_SLEW_MAX_DEG=80
SWARM_TRACKING_GIMBAL_MAX_PITCH_RATE_DEG_S=15
SWARM_TRACKING_GIMBAL_MAX_YAW_RATE_DEG_S=12
SWARM_TRACKING_GIMBAL_MAX_ACCEL_DEG_S2=60
SWARM_TRACKING_GIMBAL_MAX_BRAKE_ACCEL_DEG_S2=180
SWARM_TRACKING_GIMBAL_COMMAND_STALE_RESET_S=0.25
SWARM_TRACKING_GIMBAL_YAW_LIMIT_DEG=10
SWARM_TRACKING_GIMBAL_YAW_HOME_EXIT_DEG=1.0
SWARM_TRACKING_GIMBAL_BBOX_CENTER_EXIT_DEG=1.0
SWARM_NATIVE_FOLLOW_TARGET_ENABLED=false
SWARM_TRACKING_BODY_ATTITUDE_ENABLED=false
```

UniDepth chạy trên worker có một ô chờ. Nếu inference đang bận, frame mới
thay frame đang chờ thay vì tạo backlog. Sau khi estimator đạt `ready` lần
đầu cho bbox hiện tại, thay đổi khoảng cách hợp lệ được xử lý ngay mà không
chờ lại nhiều stable frame. Kết quả quá 500 ms, invalid hoặc chưa ready làm
controller phát vận tốc tiến/lùi 0 trong khi vẫn giữ altitude target. Quyền
yaw được tách riêng: bbox/tracker hợp lệ vẫn cho unified `offboard_follow`
recenter thân theo gimbal khi depth loading/invalid/stale; bbox mất, redetect
hoặc mơ hồ chặn cả yaw lẫn tiến/lùi. API/UI hiển thị
`follow_block_reason`, `yaw_authority`, raw/refined/filtered bbox và từng thành
phần quality score.

Lệnh gimbal có hai tầng bảo vệ để tránh bbox rời khung hình khi camera chỉ đạt
10–20 FPS. PID tracking được giới hạn ở 30°/s và 80°/s²; tầng publish độc lập
giới hạn pitch 15°/s, yaw 12°/s, gia tốc 60°/s² và phanh 180°/s². Khi lệnh đổi
dấu, gimbal phải phanh về 0 trước khi quay ngược lại. Nếu không có lệnh mới
trong 250 ms (ví dụ mất bbox), trạng thái slew cũ bị xóa trước khi tracking
tiếp tục. API trả cả `gimbal_raw_requested_rates_deg_s`,
`gimbal_limited_rates_deg_s` và `gimbal_rates_deg_s` để phân biệt yêu cầu PID,
lệnh sau limiter và chuyển động thực tế được publish.

Yaw gimbal trong tracking bị clamp ở `[-10°, +10°]`. Khi controller yêu cầu
quay ra ngoài biên hoặc feedback chạm biên, trạng thái recenter được latch:
thân drone yaw theo tổng sai số `gimbal yaw + bbox image yaw`, còn forward bị
giữ bằng 0. Latch chỉ nhả khi yaw gimbal về trong ±1° và bbox cũng về trong
±1° quanh tâm ảnh; ngưỡng nhả cấu hình bằng các biến
`SWARM_TRACKING_GIMBAL_*_EXIT_DEG`. Khi chưa chạm giới hạn, chỉ gimbal được
phép căn bbox và body yaw bằng 0; điều này tránh hai vòng điều khiển cùng quay
camera. Body yaw recenter được giới hạn 35°/s và 120°/s².

### Profile camera ưu tiên độ chính xác

Mục tiêu nhỏ ở camera 640×360/HFOV 2.0 là giới hạn accuracy lớn nhất. Profile
đề xuất dùng 1280×720/HFOV 1.35, tăng focal length theo pixel từ khoảng
205,47 lên 799,58 px (xấp xỉ 3,89 lần). Dùng full-frame, không digital crop.

Kiểm tra hoặc áp dụng profile khi Gazebo đã dừng:

```bash
python configure_gazebo_camera.py \
  camera_profiles/unidepth_accuracy_1280x720.json \
  /mnt/px4ssd/PX4-Autopilot/Tools/simulation/gz/models/gimbal/model.sdf \
  --check

python configure_gazebo_camera.py \
  camera_profiles/unidepth_accuracy_1280x720.json \
  /mnt/px4ssd/PX4-Autopilot/Tools/simulation/gz/models/gimbal/model.sdf \
  --apply
```

UniDepth V2 chính thức hiện chỉ công bố training pipeline cho V1; vì vậy
không dùng một trainer V2 tự chế không tương thích. Thu lại dataset evaluation
đồng bộ với camera profile mới và chỉ bật calibration khi validation riêng
đạt `production_recommended=true`.

Estimator dùng đầy đủ output UniDepth V2:

- `depth`: optical Z theo trục camera;
- `radius`/`points`: khoảng cách theo tia nhìn tới bề mặt;
- `confidence`: được xử lý như predicted uncertainty (giá trị thấp tốt hơn);
- `intrinsics`: chỉ dùng để chẩn đoán, camera matrix production lấy từ geometry
  Gazebo đã khai báo.

ROI được clamp và inset, lọc uncertainty, tách foreground cluster gần nhất đủ
lớn, rồi loại sample có MAD/IQR hoặc background fraction quá cao. API hiển thị
riêng optical Z, radial/surface, calibrated, temporal-filtered và control
distance. Ground truth mặc định tắt và không nằm trong control path.
CSV debug còn ghi `legacy_center_median_m`, là median vùng tâm kiểu cũ được
tính trên đúng cùng depth map. Báo cáo dùng nó làm baseline “trước”, tránh
sai lệch do so sánh hai phiên Gazebo khác điều kiện.

Target-crop có scale intrinsics đúng được cài dưới dạng thử nghiệm, nhưng mặc
định tắt. A/B trên camera 640x360 HFOV 2.0 rad cho thấy crop làm thay đổi
metric prior của UniDepth: có cảnh tốt hơn nhưng có cảnh sai nặng hơn. Chỉ bật
`SWARM_METRIC_DEPTH_TARGET_CROP_ENABLED=true` để đánh giá offline, không dùng
cho motion authority nếu chưa có validation riêng.

### Thu và hiệu chuẩn ground truth trong SITL

Chỉ bật khi cần đánh giá, sau khi UAV ở trạng thái thử nghiệm an toàn:

```dotenv
SWARM_METRIC_DEPTH_GROUND_TRUTH_ENABLED=true
SWARM_METRIC_DEPTH_GROUND_TRUTH_CSV=metric_depth_ground_truth.csv
```

Backend đồng bộ timestamp ảnh Gazebo với `/world/default/pose/info`, ghi
camera-link-to-target-center và drone-center-to-center cùng diagnostics
UniDepth. Thu tối thiểu 12 mẫu hợp lệ và nên bao phủ 5/8/9/10/12/15 m, nhiều
góc nhìn/background. Sau đó chạy:

```bash
python metric_depth_evaluation.py metric_depth_ground_truth.csv \
  --model lpiccinelli/unidepth-v2-vitl14 \
  --json-output metric_depth_evaluation.json \
  --report-output METRIC_DEPTH_ACCURACY_REPORT.md \
  --profile-output calibration/unidepth_v2_gazebo.json
```

Tool chia calibration/validation theo toàn bộ capture condition
(distance/angle/bbox-size), nên các frame gần như giống nhau trong cùng cảnh
không bị rò sang hai tập. Fit cân bằng theo median của từng condition, so sánh
none/scale/affine/piecewise-linear bằng MAE, RMSE, bias, AbsRel và p95. Affine
có scale không dương và piecewise không đơn điệu bị loại khỏi lựa chọn. Chỉ
bật profile khi `production_recommended=true` và validation tốt hơn baseline:

```dotenv
SWARM_METRIC_DEPTH_CALIBRATION_ENABLED=true
SWARM_METRIC_DEPTH_CALIBRATION_PROFILE=calibration/unidepth_v2_gazebo.json
```

Profile bị từ chối nếu model, độ phân giải hoặc intrinsics không khớp. Không
fit hoặc cập nhật calibration trong lúc bay. Profile
`calibration/unidepth_v2_gazebo_640x360_candidate.json` đi kèm repo chỉ là
artifact đánh giá: bộ dữ liệu hiện tại cho `production_recommended=false`,
không được bật cho flight control.

### Thu dataset range v2 bằng hai RTK-fixed receiver

RTK chỉ cung cấp label để train/evaluate; nó không đi vào feature model hoặc
control path. Cả camera vehicle và target phải có `gps.fix_type >= 6`, telemetry
cách timestamp ảnh không quá 100 ms và uncertainty tổng hợp không quá 0.20 m.
Hai lever arm là vector từ antenna GNSS tới camera optical center/target center,
biểu diễn trong body FRD. Phải đo và khai báo rõ ràng, kể cả khi là `0,0,0`:

```dotenv
SWARM_RANGE_DATASET_DIR=/absolute/path/to/range_v2_hardware_run_01
SWARM_RANGE_DATASET_RUN_ID=hardware_run_01
SWARM_RANGE_DATASET_TARGET_ID=UAV-02
SWARM_RANGE_DATASET_GROUND_TRUTH_SOURCE=rtk_fixed_camera_to_target_center
SWARM_RANGE_RTK_CAMERA_ANTENNA_TO_CENTER_BODY_FRD_M=0.12,0.00,0.08
SWARM_RANGE_RTK_TARGET_ANTENNA_TO_CENTER_BODY_FRD_M=0.00,0.00,0.10
SWARM_RANGE_RTK_LEVER_ARM_UNCERTAINTY_M=0.02
SWARM_RANGE_RTK_MAX_TIME_OFFSET_MS=100
SWARM_RANGE_RTK_MAX_UNCERTAINTY_M=0.20
SWARM_RANGE_RESIDUAL_MODE=off
```

Giữ gimbal ở đúng pose cố định dùng khi đo lever arm; auto gimbal tracking,
follow và motion phải tắt trong lúc thu dataset. Runtime tính khoảng cách slant
range camera-center tới target-center bằng WGS84/ECEF, xoay lever arm body-FRD
theo attitude từng vật và ghi `rtk_fixed`, uncertainty, time offset vào từng
record. GPS thường, RTK float, telemetry stale, source/quality không khớp hoặc
thiếu hiệu chỉnh lever arm đều fail closed. Trạng thái được hiển thị tại
`tracking.range_ground_truth` và `tracking.metric_target_fusion.range_residual_dataset`.

## 5. Thứ tự chạy

Mỗi khối dưới đây chạy trong một terminal riêng.

### Terminal 1 — MQTT broker

```bash
mosquitto -v
```

### Terminal 2 — PX4/Gazebo

Khởi động PX4 SITL theo cấu hình hai UAV của workspace. Nếu chỉ cần mở Gazebo
world đã build:

```bash
./start_gazebo_optimized.sh
```

Script dùng `PX4_AUTOPILOT_ROOT`; mặc định là
`/mnt/px4ssd/PX4-Autopilot`.

### Terminal 3 — ROS telemetry/control

```bash
./run_ros_telemetry_local.sh
```

Script source file được chỉ bởi `SWARM_ROS_SETUP`, rồi chạy:

```bash
ros2 launch swarm_telemetry two_uav_nodes.launch.py
```

Nếu workspace không có launch file trên, có thể dùng launch kèm gói:

```bash
ros2 launch ./isolated_swarm.launch.py
```

### Terminal 4 — MAVLink bridge

```bash
source .venv/bin/activate
python mavlink_manual_bridge.py
```

Log đúng sẽ cho thấy bridge chờ heartbeat trên UDP 14540 và 14541. Không chạy
hai bridge cùng lúc vì hai process có thể gửi setpoint trái nhau.

### Terminal 5 — Web backend

```bash
source .venv/bin/activate
python -m uvicorn main:app --host 0.0.0.0 --port 8000
```

Mở `http://127.0.0.1:8000`.

### Quy trình RGB-only Follow trên UI

1. Chọn UAV follower và bật tracking; hệ thống chỉ pointing, không tự Follow.
2. Vẽ bbox mới quanh mục tiêu và chờ `READY_FOR_FOLLOW`.
3. Kiểm tra tracker/FPS, chọn hành lang bootstrap trái hoặc phải.
4. Nhấn `BẮT ĐẦU FOLLOW` và xác nhận hành lang trống.
5. Theo dõi lần lượt `RGB_BOOTSTRAP`, `TARGET_3D_READY`,
   `FOLLOW_PRESTREAM`, `FOLLOW_MODE_REQUESTED`.
6. Chỉ coi PX4 đang Follow khi UI hiển thị `FOLLOWING` và telemetry báo
   `nav_state=19`.
7. Nhấn dừng ngay khi cảnh không còn an toàn. Target loss, pose/gimbal stale,
   estimator invalid, failsafe hoặc manual override đều phải về Position/Hold.

Không dùng depth, LiDAR hoặc telemetry/GPS của target làm đầu vào controller.
Gazebo target pose chỉ dành cho bố trí/evaluator SITL.

## 6. Kiểm tra sau khi chạy

```bash
curl -sS http://127.0.0.1:8000/health
curl -sS http://127.0.0.1:8000/api/drones
pgrep -af 'mavlink_manual_bridge.py|uvicorn main:app|ros2 launch'
```

Kỳ vọng:

- MQTT và Gazebo báo connected/started.
- Có telemetry cho `UAV-01` và `UAV-02`.
- Chỉ có một process `mavlink_manual_bridge.py`.
- Không có UAV báo failsafe trước khi phát lệnh.

## 7. Chạy test

Test logic không yêu cầu khởi động PX4/Gazebo:

```bash
source .venv/bin/activate
python -m compileall -q \
  main.py tracking_web.py tracking_hybrid.py metric_depth_estimator.py \
  body_attitude_recenter.py visual_follow_target.py mavlink_manual_bridge.py
python -m unittest discover -p 'test_*.py'
```

## 8. Port và topic mặc định

| Thành phần | Giá trị |
|---|---|
| Dashboard HTTP | TCP 8000 |
| MQTT broker | TCP 1883 |
| UAV-01 MAVLink | UDP 14540, system ID 1 |
| UAV-02 MAVLink | UDP 14541, system ID 2 |
| QGroundControl | UDP 14550 |
| MQTT telemetry | `swarm/+/telemetry/state` |
| MQTT command | `swarm/{drone_id}/control/command` |
| MQTT result | `swarm/+/control/result` |

Model Gazebo mặc định là `sparrow_gimbal_0` và `sparrow_gimbal_1`. Có thể đổi
qua `SWARM_GAZEBO_MODEL_UAV_01/02`; camera, IMU, gimbal, lidar và ground-truth
đều dùng cùng mapping này.

## 9. Lỗi thường gặp

- `address already in use`: đã có backend dùng port 8000; kiểm tra bằng
  `ss -ltnp | grep ':8000'`.
- `MQTT connection refused`: Mosquitto chưa chạy hoặc không nghe tại
  `127.0.0.1:1883`.
- `TRACKING_IMPORT_ERROR`: sai `SWARM_TRACKING_PACKAGE_ROOT`, thiếu
  `lfc_gimbal_gazebo` hoặc thiếu OpenCV contrib.
- Không có camera/Gazebo: kiểm tra version `gz.transport13`, `gz.msgs10`,
  `GZ_PARTITION` và tên model.
- Bridge chờ heartbeat: PX4 chưa gửi đến UDP 14540/14541 hoặc system ID không
  phải 1/2.
- Giao diện có telemetry nhưng UAV không nhận lệnh: kiểm tra chỉ có một bridge,
  MQTT topic và trạng thái arm/mode/failsafe.

## 10. Trạng thái kỹ thuật khi bàn giao

- Luồng active là RGB bearing-only: explicit lateral bootstrap, robust
  triangulation + CV EKF, D0 từ immutable selection pose, target prestream và
  native PX4 AUTO FOLLOW TARGET.
- Pointing và bootstrap dùng unified Offboard có giới hạn; khi PX4
  `nav_state=19`, PX4 là motion authority duy nhất.
- Gate reprojection/covariance không được nới để ép một phiên SITL pass.
- Metric depth vẫn là tính năng thử nghiệm. Khi
  `SWARM_METRIC_DEPTH_FOLLOW_CONTROL=true`, UniDepth V2 là nguồn tiến/lùi duy
  nhất; depth chưa sẵn sàng làm drone giữ vị trí, không fallback sang bbox/LUT.
- Inference metric depth chạy trên worker latest-frame-wins, không nằm trong
  tracking lock; status hiển thị FPS, queue, số frame bỏ và latency.
- Calibration production mặc định tắt cho đến khi có dataset đồng bộ và
  validation độc lập; ground truth chỉ dành cho SITL/debug.
- Cần flight-test đầy đủ các trường hợp nhỏ hơn 8 m, trong 8–12 m và lớn hơn
  12 m trước khi coi controller là hoàn tất.

Thông tin triển khai và trạng thái kiểm chứng gần nhất nằm trong
`FOLLOW_SAFE_BAND_8_12_REPORT_20260729.md` và
`TRACKING_FOLLOW_PIPELINE_REPORT_20260729.md`.
