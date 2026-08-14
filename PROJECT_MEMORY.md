# Project memory - Swarm UAV Dashboard

Ghi chú này được tạo để lần sau có thể đọc nhanh thay vì rà toàn bộ repo. Nội dung phản ánh source hiện tại trong `/home/sup/swarm_dashboard` tại thời điểm đọc, không coi các file backup/log/build/install là nguồn chính.

## 1. Mục tiêu dự án

Dự án là một dashboard web để theo dõi và điều khiển 2 UAV PX4/Gazebo:

- `UAV-01`, model Gazebo `x500_custom_0`, MAVLink system id `1`, UDP input `14540`.
- `UAV-02`, model Gazebo `x500_custom_1`, MAVLink system id `2`, UDP input `14541`.

Chức năng chính:

- Hiển thị telemetry từ PX4/ROS qua MQTT và WebSocket.
- Điều khiển hành động cơ bản: arm, disarm, hold, land, RTL, takeoff, position, offboard map/goto.
- Điều khiển manual bằng bàn phím qua MAVLink `MANUAL_CONTROL`.
- Hiển thị bản đồ Leaflet và gửi waypoint global.
- Điều khiển gimbal Gazebo roll/pitch/yaw bằng tay hoặc tự động bởi tracker.
- Hiển thị camera Gazebo dạng MJPEG.
- Tracking mục tiêu trên ảnh camera, chọn bbox trên web, bám mục tiêu bằng gimbal.
- Apparent-size follow: bbox lúc chọn target được dùng làm kích thước tham chiếu an toàn; drone tiến/lùi chậm để giữ target có scale gần như lúc đầu.
- Visual Follow Target: ước lượng target từ bbox + pose camera, gửi MAVLink `FOLLOW_TARGET` cho PX4 native Follow mode.
- Có các chế độ legacy/offboard follow và body-attitude recenter nhưng source hiện tại ưu tiên Visual Follow Target nếu feature này bật.

## 2. File nguồn chính

### `main.py`

Backend FastAPI chính.

Vai trò:

- Serve frontend tại `/` từ `static/index.html`.
- Kết nối MQTT broker `127.0.0.1:1883`.
- Subscribe:
  - `swarm/+/telemetry/state`
  - `swarm/+/control/result`
- Publish command:
  - `swarm/{drone_id}/control/command`
- Mở WebSocket `/ws` để frontend nhận snapshot 5 Hz và gửi command.
- Tích hợp Gazebo Transport cho:
  - gimbal publishers `/model/{model}/command/gimbal_{axis}`;
  - camera image subscriptions;
  - body/camera IMU subscriptions để lấy gimbal feedback;
  - lidar scan optional `/x500_custom/front_lidar`.
- Tạo `TrackingManager` từ `tracking_web.py`.
- Cung cấp REST API:
  - `GET /health`
  - `GET /api/drones`
  - `GET /api/camera/{drone_id}/stream`
  - `POST /api/tracking/start`
  - `POST /api/tracking/bbox`
  - `POST /api/tracking/follow`
  - `POST /api/tracking/stop`
  - `GET /api/tracking/stream`

Các WebSocket message frontend gửi:

- `control_action`: action nằm trong `ALLOWED_ACTIONS`.
- `manual_control`: normalized axes `forward/right/up/yaw` trong `[-1, 1]`.
- `goto_global`: lat/lon/alt target.
- `gimbal_control`: delta gimbal từng axis.
- `gimbal_home`: đưa gimbal về 0.

Safety/validation đáng chú ý:

- Chỉ nhận `UAV-01`, `UAV-02`.
- Takeoff altitude backend giới hạn `2.5..10.0 m`.
- Goto altitude backend giới hạn `1..30 m`.
- Manual override có deadline `0.4 s`; tracking/body yaw/follow bị pause khi manual đang ưu tiên.
- ARM recovery: nếu PX4 đang RTL/Offboard/Takeoff/Land hoặc preflight chưa sẵn, backend gửi `position` trước và trả lỗi hướng dẫn user đợi rồi arm lại.
- Body yaw/follow/offboard tracking kiểm tra telemetry online, armed, failsafe, nav state, manual-control signal, altitude.

### `static/index.html`

Frontend single-file HTML/CSS/JS.

Vai trò:

- UI tiếng Việt cho sidebar telemetry, mode/action buttons, map, keyboard manual, takeoff, gimbal, tracking.
- Dùng Leaflet CDN (`unpkg.com`) cho map.
- Kết nối WebSocket tới `/ws`, tự reconnect mỗi 2 giây.
- Render telemetry snapshot từ backend: drones, control_results, gimbal state, gazebo status, tracking status, mqtt_connected.
- Gửi manual frame định kỳ khoảng 20 Hz khi keyboard control bật.
- Keyboard mapping:
  - `W/S`: forward/back
  - `A/D`: left/right
  - `Q/E`: yaw left/right
  - `R/F`: up/down
  - `Space`: stop
- Chỉ bật manual khi drone armed, mode Position và manual link OK.
- Tracking UI:
  - nút start/stop Follow Target;
  - input desired distance `1..20 m`;
  - stream `/api/tracking/stream`;
  - canvas để kéo bbox normalized rồi gọi `/api/tracking/bbox`.
- Gimbal UI:
  - step ± theo axis roll/pitch/yaw;
  - home;
  - disable manual gimbal khi tracking đang tự điều khiển gimbal.

### `tracking_web.py`

Quản lý tracking/video/control loop.

Vai trò:

- Import package ngoài từ `SWARM_TRACKING_PACKAGE_ROOT`, mặc định `/home/sup/ws_px4/src/lfc_gimbal_gazebo`.
- Hỗ trợ backend tracking:
  - `opencv`
  - `hybrid` mặc định
  - `onnx`
  - `board`
- Với `hybrid`, dùng `HybridMemoryTrackerService` trong `tracking_hybrid.py`.
- Với `opencv`, dùng tracker OpenCV trực tiếp (`KCF` mặc định, có `CSRT`, `MOSSE` nếu build hỗ trợ).
- Chạy worker thread đọc raw frames từ `GazeboDashboardBridge`.
- Luồng tracking:
  1. Start tracking cho một drone.
  2. Chờ frame camera.
  3. User chọn bbox normalized.
  4. Init tracker.
  5. Mỗi frame update tracker, lọc bbox, điều khiển gimbal, overlay ảnh, encode MJPEG.
- Gimbal auto:
  - dùng `GimbalService` từ package ngoài;
  - PID-like config bằng env;
  - command qua callback `command_tracking_gimbal()` trong `main.py`.
- Visual follow:
  - dùng `VisualBBoxFilter`, `VisualBBoxRangeEstimator`, `CameraRayProjector`, `TargetStateFilter`;
  - ước lượng range bằng LUT image-only, không dùng lidar/ground-plane;
  - project pixel center + camera quaternion + local pose thành target NED;
  - convert target NED sang WGS84;
  - publish command type `visual_follow_target`;
  - auto-start bật mặc định.
- Khi `visual_follow_feature_enabled` bật, native PX4 Follow Target là authority cho motion. Legacy body yaw/follow/offboard motion bị tắt để tránh tranh quyền.
- Source hiện tại có thêm `SWARM_APPARENT_SIZE_FOLLOW_ENABLED=true` mặc định. Khi bật, apparent-size follow chiếm motion authority và bỏ qua native Visual Follow Target để dùng offboard/local follow theo kích thước bbox tham chiếu.
- Khi feature này tắt, có legacy pipeline:
  - body yaw bằng manual-control yaw;
  - distance follow bằng visual LUT;
  - offboard velocity hoặc offboard acceleration/body attitude recenter.

### `mavlink_manual_bridge.py`

Bridge MQTT command sang MAVLink cho từng UAV. Tên file cũ là manual bridge nhưng hiện tại xử lý nhiều loại automation.

Vai trò:

- Subscribe `swarm/+/control/command`.
- Chạy worker thread cho mỗi UAV:
  - `UAV-01`: UDP `udpin:0.0.0.0:14540`, expected system id 1.
  - `UAV-02`: UDP `udpin:0.0.0.0:14541`, expected system id 2.
- Gửi output ở `20 Hz`.
- Command types được xử lý:
  - `manual_control`
  - `tracking_yaw`
  - `tracking_follow`
  - `visual_follow_target`
  - `offboard_follow`
  - `offboard_attitude_follow` chỉ nhận nếu `SWARM_ALLOW_RAW_ATTITUDE_OFFBOARD=true`.
- Priority:
  1. Human manual input thắng, tắt automation stale.
  2. Visual Follow Target loại trừ legacy manual-stick/offboard tracking controllers.
  3. Offboard attitude và offboard follow loại trừ nhau.
  4. Tracking yaw/follow legacy loại trừ visual follow target.
- Manual output:
  - `MANUAL_CONTROL x/y/z/r`;
  - throttle center là `500`, không phải `0`;
  - timeout command `0.35 s` thì trả stick về giữa.
- Visual Follow Target:
  - gửi MAVLink `FOLLOW_TARGET`;
  - cấu hình PX4 params `FLW_TGT_*`;
  - pre-stream vài frame rồi request PX4 `AUTO FOLLOW TARGET`;
  - retry mode tối đa 3 lần, nếu PX4 không confirm thì block visual follow đến khi tracking stop.
- Offboard follow:
  - gửi `SET_POSITION_TARGET_LOCAL_NED`;
  - có body frame velocity, local NED altitude hold, hoặc local NED acceleration + altitude hold.
  - pre-stream setpoint rồi request PX4 Offboard.
- Offboard attitude:
  - gửi `SET_ATTITUDE_TARGET` nếu env cho phép raw attitude/thrust;
  - có release timeout để chuyển về Position an toàn.
- Có QGroundControl MAVLink proxy:
  - bật mặc định `SWARM_QGC_MAVLINK_PROXY_ENABLED=true`;
  - proxy base port `14650`, host QGC `127.0.0.1:14550`.
- Có SITL tuning single-EKF:
  - `SWARM_PX4_SINGLE_EKF_ENABLED=true` mặc định;
  - set `EKF2_MULTI_IMU=0`, `EKF2_MULTI_MAG=0`.

### `visual_follow_target.py`

Các helper cho Visual Follow Target.

Vai trò:

- `VisualRangeProfile`: LUT empirical cho target UAV ở frame 640x360:
  - 5 m: 150x90 px
  - 8 m: 95x57 px
  - 10 m: 76x46 px
  - 12 m: 63x38 px
  - 15 m: 51x31 px
- `VisualBBoxFilter`: lọc/giữ bbox, reject jump/outlier, yêu cầu stable frames.
- `VisualBBoxRangeEstimator`: nội suy distance từ bbox scale, EMA, aspect guard, TTC block.
- `CameraRayProjector`: chuyển pixel camera sang ray NED bằng camera quaternion.
- `TargetStateFilter`: alpha-beta filter cho target NED, giới hạn jump/speed/rate.
- `ned_target_to_wgs84`: convert target NED tương đối follower sang lat/lon/alt.

### `body_attitude_recenter.py`

Controller body attitude/offboard acceleration để “unload” gimbal về home.

Vai trò:

- Đọc config từ env `SWARM_BODY_RECENTER_*`.
- Tính lỗi gimbal home roll/pitch/yaw.
- Sinh body rates roll/pitch/yaw, thrust và horizontal acceleration body frame.
- Có altitude guard:
  - nếu altitude error vượt pause threshold thì pause horizontal acceleration;
  - nếu vượt exit threshold thì output inactive.
- Trong source hiện tại, `TrackingManager` dùng controller này khi `body_attitude_enabled` và visual-follow feature không chiếm motion authority.

Lưu ý: Trong `TrackingManager`, env default cho `SWARM_TRACKING_BODY_ATTITUDE_ENABLED` được parse là bật (`true`), còn trong `BodyAttitudeRecenterConfig.from_environment()` default là tắt (`false`). Khi muốn chắc chắn bật/tắt, set env rõ ràng.

### `tracking_hybrid.py`

Tracker hybrid nhanh: OpenCV KCF + Kalman + template memory + re-identification.

Vai trò:

- Init KCF trên bbox.
- Mỗi frame:
  - predict bằng Kalman;
  - update KCF;
  - kiểm tra appearance similarity và motion score;
  - cập nhật memory nếu confidence đủ cao;
  - nếu mất target thì chuyển `OCCLUDED` rồi `LOST`;
  - redetect bằng multi-scale template matching + reid + motion/exit score.
- Env tuning:
  - `SWARM_REID_AFTER`
  - `SWARM_REID_INTERVAL`
  - `SWARM_REID_CONFIRM`
  - `SWARM_REID_TRACK_THRESHOLD`
  - `SWARM_REID_THRESHOLD`
  - `SWARM_REID_SCORE_THRESHOLD`
  - `SWARM_MEMORY_UPDATE_THRESHOLD`

### `isolated_swarm.launch.py`

ROS 2 launch file phụ trợ.

Vai trò:

- Launch `swarm_telemetry` nodes cho 2 UAV:
  - `telemetry_node`
  - `mqtt_bridge`
- Mapping:
  - `/swarm/uav_01/telemetry_json` -> `swarm/UAV-01/telemetry/state`
  - `/swarm/uav_02/telemetry_json` -> `swarm/UAV-02/telemetry/state`
- Launch `web_control_node`, nhưng remapping hiện tại chỉ map control topics cho `/px4_0`/UAV-01.

### `run_ros_telemetry_local.sh`

Script nhỏ để source ROS workspace `/home/sup/ws_px4/install/setup.bash` rồi chạy launch:

```bash
ros2 launch /home/sup/swarm_dashboard/isolated_swarm.launch.py
```

## 3. Luồng hệ thống

### Telemetry

```text
PX4/ROS telemetry_node
  -> ROS telemetry_json
  -> mqtt_bridge
  -> MQTT swarm/UAV-*/telemetry/state
  -> main.py MQTT callback
  -> latest_drones
  -> /api/drones và WebSocket /ws
  -> static/index.html render UI/map
```

### Flight action

```text
User bấm action trên web
  -> WebSocket /ws type=control_action
  -> main.py validate
  -> MQTT swarm/{drone_id}/control/command
  -> ROS web_control_node hoặc consumer khác
  -> PX4
  -> control result về MQTT swarm/{drone_id}/control/result
  -> dashboard hiển thị
```

### Keyboard manual

```text
User bật keyboard + nhấn W/A/S/D/Q/E/R/F/Space
  -> frontend gửi manual_control khoảng 20 Hz
  -> main.py publish MQTT
  -> mavlink_manual_bridge.py
  -> MAVLink MANUAL_CONTROL 20 Hz
  -> PX4 Position mode
```

### Manual gimbal

```text
User bấm gimbal step/home
  -> WebSocket gimbal_control/gimbal_home
  -> main.py GazeboDashboardBridge.publish_gimbal()
  -> Gazebo topic /model/{model}/command/gimbal_{axis}
```

### Camera stream

```text
Gazebo camera image
  -> main.py Gazebo Transport callback
  -> raw BGR frame nếu tracking cần
  -> JPEG frame nếu camera clients cần
  -> /api/camera/{drone_id}/stream hoặc tracking manager
```

### Tracking + Visual Follow Target

```text
User start Follow Target
  -> POST /api/tracking/start
  -> TrackingManager active, Gazebo bridge giữ raw frame cho drone
  -> User kéo bbox
  -> POST /api/tracking/bbox
  -> tracker update mỗi frame
  -> gimbal auto giữ target giữa ảnh
  -> bbox filter + range LUT + camera quaternion + local/global pose
  -> target WGS84 + velocity estimate
  -> MQTT command type=visual_follow_target
  -> mavlink_manual_bridge.py
  -> MAVLink FOLLOW_TARGET + request PX4 AUTO FOLLOW TARGET
```

## 4. Runtime dependencies/điều kiện môi trường

Backend Python cần ít nhất:

- FastAPI/uvicorn
- paho-mqtt
- pydantic
- OpenCV (`cv2`)
- numpy
- Gazebo Python bindings:
  - `gz.transport13`
  - `gz.msgs10.*`
- Tracking package ngoài `lfc_gimbal_gazebo` tại `SWARM_TRACKING_PACKAGE_ROOT`.

Bridge MAVLink cần:

- paho-mqtt
- pymavlink
- PX4 SITL/MAVLink đang phát heartbeat vào UDP ports 14540/14541.

Ngoài ra cần:

- MQTT broker local ở `127.0.0.1:1883` thường là mosquitto.
- Gazebo world/model đúng tên `x500_custom_0`, `x500_custom_1`.
- ROS/PX4 telemetry/control backend đang publish/consume đúng MQTT topics.
- Frontend cần internet hoặc cached asset để tải Leaflet từ CDN.

## 5. Biến môi trường quan trọng

Backend/tracking:

- `SWARM_DASHBOARD_MQTT_CLIENT_ID`
- `SWARM_TRACKING_GIMBAL_YAW_LIMIT_DEG` default 15, clamp 5..90.
- `SWARM_TRACKING_GIMBAL_ROLL_RATE_DEG_S` default 90.
- `SWARM_TRACKING_BODY_YAW_MIN_ALTITUDE_M` default 0.5.
- `SWARM_TRACKING_OFFBOARD_ENABLED` default true.
- `SWARM_TRACKING_PACKAGE_ROOT` default `/home/sup/ws_px4/src/lfc_gimbal_gazebo`.
- `SWARM_TRACKING_BACKEND` default `hybrid`.
- `SWARM_TRACKING_BOARD_IP` default `192.168.4.89`.
- `SWARM_OPENCV_TRACKER` default `KCF`.
- `SWARM_TRACKING_JPEG_QUALITY` default 72.
- `SWARM_TRACKING_GIMBAL_ENABLED` default true.
- `SWARM_TRACKING_BODY_YAW_ENABLED` default false.
- `SWARM_TRACKING_FOLLOW_ENABLED` default true.
- `SWARM_VISUAL_FOLLOW_TARGET_ENABLED` default true.
- `SWARM_APPARENT_SIZE_FOLLOW_ENABLED` default true.
- `SWARM_APPARENT_SIZE_FOLLOW_KP` default 0.90.
- `SWARM_APPARENT_SIZE_FOLLOW_DEADBAND` default 0.05 theo log-scale error.
- `SWARM_APPARENT_SIZE_YAW_KP` default 4.0 deg/s per deg gimbal-yaw error sau deadband.
- `SWARM_APPARENT_SIZE_YAW_DEADBAND_DEG` default 0.5.
- `SWARM_APPARENT_SIZE_YAW_INVERT` default false; set true nếu thân drone yaw ngược hướng làm gimbal lệch xa home hơn.
- `SWARM_VISUAL_FOLLOW_AUTO_START` default true.
- `SWARM_VISUAL_FOLLOW_AUTO_RETRY_S` default 1.0.
- `SWARM_TRACKING_BODY_ATTITUDE_ENABLED` default true in TrackingManager logic.
- Gimbal PID/tuning: `SWARM_TRACKING_GIMBAL_FX/FY/KP/KI/KD/EMA_ALPHA/DEADBAND_DEG/OMEGA_MAX_DEG/SLEW_MAX_DEG`.
- Body yaw/follow/offboard tuning: `SWARM_TRACKING_BODY_YAW_*`, `SWARM_TRACKING_FOLLOW_*`, `SWARM_TRACKING_OFFBOARD_*`.

MAVLink bridge:

- `SWARM_ALLOW_RAW_ATTITUDE_OFFBOARD` default false.
- `SWARM_PX4_SINGLE_EKF_ENABLED` default true.
- `SWARM_QGC_MAVLINK_PROXY_ENABLED` default true.
- `SWARM_QGC_MAVLINK_HOST` default `127.0.0.1`.
- `SWARM_QGC_MAVLINK_PORT` default `14550`.
- `SWARM_QGC_PROXY_BASE_PORT` default `14650`.

Body recenter:

- `SWARM_GIMBAL_HOME_ROLL_DEG/PITCH_DEG/YAW_DEG`
- `SWARM_BODY_RECENTER_*` nhiều tham số gain/limit/thrust/altitude guard.

Hybrid tracker:

- `SWARM_REID_AFTER`
- `SWARM_REID_INTERVAL`
- `SWARM_REID_CONFIRM`
- `SWARM_REID_TRACK_THRESHOLD`
- `SWARM_REID_THRESHOLD`
- `SWARM_REID_SCORE_THRESHOLD`
- `SWARM_MEMORY_UPDATE_THRESHOLD`

## 6. Cách chạy thường dùng

Một setup đầy đủ thường cần nhiều terminal/process:

1. MQTT broker:

```bash
mosquitto -v
```

2. PX4/Gazebo/ROS telemetry stack ngoài repo này.

3. ROS telemetry/control launch nếu dùng script local:

```bash
cd /home/sup/swarm_dashboard
./run_ros_telemetry_local.sh
```

4. Web backend:

```bash
cd /home/sup/swarm_dashboard
source .venv/bin/activate
python -m uvicorn main:app --host 0.0.0.0 --port 8000
```

5. MAVLink bridge:

```bash
cd /home/sup/swarm_dashboard
source .venv/bin/activate
python3 mavlink_manual_bridge.py
```

Chỉ nên chạy một instance `mavlink_manual_bridge.py`. Nếu chạy nhiều instance, một process có thể gửi command còn process khác gửi stick center, làm UAV không phản ứng.

## 7. File phụ/trạng thái repo

Các file tài liệu sẵn có:

- `PROJECT_EXPLANATION.md`: giải thích bản cũ hơn, tập trung dashboard/manual.
- `BAO_CAO_DU_AN.md`: báo cáo tiếng Việt.
- `GIAI_THICH_HE_THONG_TRACKING_DRONE.md`: giải thích tracking drone.
- `TRACKING_30FPS_CHANGES.md`: ghi chú thay đổi tracking 30 FPS.
- `VISUAL_FOLLOW_TARGET.md`: tài liệu visual follow target.
- `BODY_ATTITUDE_RECENTER.md`: tài liệu body attitude recenter.

Backup/source phụ:

- `main.py.backup*`, `main.py.before_map_control`, `static/index.html.backup*`, `static/index.html.before_map_control`.
- `swarm_app/` và `swarm_app.zip` là bản đóng gói/cũ hơn, không phải source chính hiện tại.
- `build/`, `install/`, `log/`, `__pycache__/` là output/generated.

Tests:

- `test_visual_follow_target.py`
- `test_body_attitude_recenter.py`
- `test_mavlink_attitude_bridge.py`

## 8. Lưu ý/rủi ro kỹ thuật

- Không có git repository ở `/home/sup/swarm_dashboard` theo `git status`; không thể dựa vào git diff/history tại root này.
- Backend tự append `/usr/lib/python3/dist-packages` để thấy Gazebo Python bindings khi chạy trong virtualenv.
- Nếu `GZ_PARTITION` bị set sai, Gazebo publishers/subscribers có thể không connect.
- `GazeboDashboardBridge.publish_gimbal()` yêu cầu publisher có connections; nếu model/topic sai sẽ báo no subscriber.
- Camera conversion chỉ hỗ trợ một số pixel format Gazebo: gray, RGB, RGBA, BGRA, BGR.
- Camera JPEG stream bị giới hạn width 640, quality 72, max FPS 10 cho client stream; raw frames vẫn dùng cho tracking khi tracking active.
- Visual range LUT là empirical cho target/camera/crop cụ thể; đổi camera FOV, độ phân giải, target size hoặc bbox semantics thì phải hiệu chỉnh lại.
- Visual Follow Target cần local/global pose + camera quaternion còn mới; telemetry >500 ms hoặc camera orientation stale sẽ block.
- Native PX4 Follow Target mode có min follow height theo code đang set `FLW_TGT_HT >= 8.0`.
- `isolated_swarm.launch.py` chỉ remap web_control_node cho UAV-01; nếu cần action control cho UAV-02 qua ROS node này phải kiểm tra lại.
- `static/index.html` dùng Leaflet CDN; môi trường offline có thể làm map CSS/JS không tải nếu chưa cache.
- `mavlink_manual_bridge.py` dùng QGC proxy mặc định; nếu QGC hoặc ports conflict, kiểm tra `SWARM_QGC_*`.
- `SWARM_TRACKING_BODY_ATTITUDE_ENABLED` có default không đồng nhất giữa `TrackingManager` và `BodyAttitudeRecenterConfig`; set env rõ khi debug.

## 9. Khi quay lại dự án nên đọc theo thứ tự

Nếu cần hiểu nhanh mà không đọc full repo:

1. Đọc file này trước.
2. Nếu sửa backend/API/MQTT/Gazebo: đọc `main.py`.
3. Nếu sửa UI/WebSocket/keyboard/tracking UI: đọc `static/index.html`.
4. Nếu sửa tracking/motion authority: đọc `tracking_web.py`.
5. Nếu sửa MAVLink/PX4 mode/control output: đọc `mavlink_manual_bridge.py`.
6. Nếu sửa Visual Follow Target math/filter/range: đọc `visual_follow_target.py`.
7. Nếu sửa recenter/offboard acceleration: đọc `body_attitude_recenter.py`.
8. Nếu sửa tracker hybrid/reid: đọc `tracking_hybrid.py`.
