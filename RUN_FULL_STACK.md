# Chạy toàn bộ Swarm Dashboard RGB Follow

## Chạy tất cả bằng một file

Không cần mở 8 terminal thủ công. Từ thư mục dự án, chạy:

```bash
./run_all.sh
```

Dashboard sẽ mở tại `http://127.0.0.1:8000`. Mỗi thành phần ghi log riêng
trong `artifacts/run_<thời_gian>/`. Nhấn `Ctrl+C` để dừng toàn bộ tiến trình
do script tạo. Có thể chỉ kiểm tra môi trường mà không khởi động bằng:

```bash
./run_all.sh --check
```

Các mục terminal bên dưới được giữ lại để chẩn đoán hoặc chạy từng thành phần
riêng lẻ.

Các lệnh dưới đây khớp với workspace hiện tại trên máy này. Mỗi mục phải chạy
trong một terminal riêng. Trước khi chạy lại, kiểm tra để không tạo hai PX4,
bridge hoặc backend cùng lúc:

```bash
pgrep -af 'mosquitto|MicroXRCEAgent|gz sim|px4 -i|mavlink_manual_bridge.py|uvicorn main:app|ros2 launch'
```

## Terminal 1 — MQTT broker

Nếu Mosquitto đã chạy bằng system service thì bỏ qua terminal này.

```bash
mosquitto -v
```

## Terminal 2 — Micro XRCE-DDS Agent

```bash
cd /home/sup/swarm_dashboard
MicroXRCEAgent udp4 -p 8888
```

## Terminal 3 — Gazebo server và GUI

```bash
cd /home/sup/swarm_dashboard
export PX4_AUTOPILOT_ROOT=/mnt/px4ssd/PX4-Autopilot
./start_gazebo_optimized.sh
```

## Terminal 4 — PX4 UAV-01

Chỉ chạy sau khi Gazebo server đã lên.

```bash
cd /mnt/px4ssd/PX4-Autopilot
source /opt/ros/jazzy/setup.bash
source build/px4_sitl_default/rootfs/gz_env.sh

PX4_SYS_AUTOSTART=22000 \
PX4_SIM_MODEL=gz_x500_custom \
PX4_GZ_STANDALONE=1 \
PX4_GZ_MODEL_POSE="0,0,0,0,0,0" \
./build/px4_sitl_default/bin/px4 -i 0
```

## Terminal 5 — PX4 UAV-02

```bash
cd /mnt/px4ssd/PX4-Autopilot
source /opt/ros/jazzy/setup.bash
source build/px4_sitl_default/rootfs/gz_env.sh

PX4_SYS_AUTOSTART=22000 \
PX4_SIM_MODEL=gz_x500_custom \
PX4_GZ_STANDALONE=1 \
PX4_GZ_MODEL_POSE="0,5,0,0,0,0" \
./build/px4_sitl_default/bin/px4 -i 1
```

## Terminal 6 — ROS telemetry và MQTT bridge

```bash
cd /home/sup/swarm_dashboard
export SWARM_ROS_SETUP=/home/sup/ws_px4/install/setup.bash
./run_ros_telemetry_local.sh
```

Nếu launch package không tồn tại, dùng launch file kèm repository:

```bash
source /home/sup/ws_px4/install/setup.bash
export ROS_LOCALHOST_ONLY=1
ros2 launch ./isolated_swarm.launch.py
```

## Terminal 7 — MAVLink manual/native Follow bridge

```bash
cd /home/sup/swarm_dashboard
set -a
source .env.example
set +a

export SWARM_TRACKING_PACKAGE_ROOT=/home/sup/ws_px4/src/lfc_gimbal_gazebo
export SWARM_ROS_SETUP=/home/sup/ws_px4/install/setup.bash
export PX4_AUTOPILOT_ROOT=/mnt/px4ssd/PX4-Autopilot

.venv/bin/python mavlink_manual_bridge.py
```

Chỉ được có một process `mavlink_manual_bridge.py`.

## Terminal 8 — Web backend

```bash
cd /home/sup/swarm_dashboard
set -a
source .env.example
set +a

export SWARM_TRACKING_PACKAGE_ROOT=/home/sup/ws_px4/src/lfc_gimbal_gazebo
export SWARM_ROS_SETUP=/home/sup/ws_px4/install/setup.bash
export PX4_AUTOPILOT_ROOT=/mnt/px4ssd/PX4-Autopilot

.venv/bin/python -m uvicorn main:app --host 0.0.0.0 --port 8000
```

Mở:

```text
http://127.0.0.1:8000
```

## Kiểm tra nhanh

```bash
curl -sS http://127.0.0.1:8000/health
curl -sS http://127.0.0.1:8000/api/drones
ss -ltnup | grep -E ':1883|:8888|:14540|:14541|:8000'
```

Trước khi Follow, arm/takeoff follower lên tối thiểu 8 m. Trên UI: bật
tracking, vẽ bbox, chờ `READY_FOR_FOLLOW`, chọn hành lang bootstrap, nhấn
`BẮT ĐẦU FOLLOW`, và chỉ coi PX4 đã nhận Follow khi telemetry báo
`nav_state=19`.
