# Swarm UAV Dashboard

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
- `tracking_hybrid.py`: KCF/Kalman/template-memory tracker.
- `visual_follow_target.py`: lọc bbox, ước lượng khoảng cách và tọa độ mục tiêu.
- `body_attitude_recenter.py`: bộ điều khiển recenter thân UAV.
- `metric_depth_estimator.py`: UniDepth tùy chọn; mặc định tắt.
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

Nếu cần metric depth, cài thêm PyTorch, UniDepth và model
`lpiccinelli/unidepth-v2-vitl14`, sau đó bật các biến trong `.env.example`.
Tính năng này mặc định tắt và không bắt buộc để dashboard chạy.

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

Model Gazebo phải có tên `x500_custom_0` và `x500_custom_1`, khớp cấu hình
trong `main.py`.

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

- Luồng visual follow hiện ưu tiên Offboard dynamic range, giữ độ cao local
  NED và khoảng cách an toàn 8–12 m.
- Metric depth là tính năng thử nghiệm; UniDepth chưa đủ ổn định để làm nguồn
  khoảng cách an toàn duy nhất. Khi tắt/chưa sẵn sàng, hệ thống dùng bbox/LUT.
- Inference metric depth vẫn chạy đồng bộ trong tracking path.
- Cần flight-test đầy đủ các trường hợp nhỏ hơn 8 m, trong 8–12 m và lớn hơn
  12 m trước khi coi controller là hoàn tất.

Thông tin chi tiết về quyết định kỹ thuật gần nhất nằm trong
`SESSION_HANDOFF.md`.
