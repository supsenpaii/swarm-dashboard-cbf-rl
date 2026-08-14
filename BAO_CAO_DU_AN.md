# Báo cáo dự án: Swarm UAV Dashboard

## 1. Thông tin chung

Tên dự án: **Swarm UAV Dashboard**

Mục tiêu của dự án là xây dựng một giao diện web để giám sát và điều khiển nhiều UAV trong môi trường mô phỏng Gazebo/PX4. Hệ thống hiện hỗ trợ 2 UAV:

- `UAV-01`
- `UAV-02`

Dashboard cho phép người dùng theo dõi trạng thái bay, xem vị trí UAV trên bản đồ, gửi các lệnh điều khiển bay cơ bản, điều khiển UAV bằng bàn phím trong chế độ Position, và gửi điểm đích trên bản đồ trong chế độ Offboard.

## 2. Mục tiêu đã thực hiện

Dự án đã hoàn thành các mục tiêu chính sau:

- Xây dựng backend bằng FastAPI để phục vụ giao diện web và WebSocket.
- Kết nối backend với MQTT broker để nhận telemetry và gửi lệnh điều khiển.
- Xây dựng frontend web bằng HTML, CSS và JavaScript.
- Hiển thị telemetry của từng UAV theo thời gian thực.
- Hiển thị vị trí UAV trên bản đồ Leaflet.
- Cho phép chọn UAV đang điều khiển.
- Hỗ trợ các lệnh bay cơ bản: `ARM`, `DISARM`, `HOLD`, `LAND`, `RETURN HOME`.
- Hỗ trợ lệnh `TAKEOFF` với giới hạn độ cao an toàn.
- Hỗ trợ chọn điểm đích trên bản đồ trong chế độ `OFFBOARD MAP`.
- Hỗ trợ điều khiển thủ công bằng bàn phím trong chế độ `POSITION`.
- Xây dựng MAVLink bridge để chuyển lệnh bàn phím từ MQTT sang PX4 qua MAVLink.
- Bổ sung kiểm tra an toàn trước khi ARM và trước khi bật keyboard control.
- Đóng gói các file chính thành `swarm_app.zip`.

## 3. Kiến trúc hệ thống

Hệ thống gồm 3 thành phần chính:

### 3.1. Backend FastAPI

File chính: `main.py`

Backend chịu trách nhiệm:

- Phục vụ giao diện web tại route `/`.
- Cung cấp API `/health` để kiểm tra trạng thái server.
- Cung cấp API `/api/drones` để lấy telemetry hiện tại.
- Mở WebSocket `/ws` để frontend nhận telemetry và gửi lệnh điều khiển.
- Kết nối MQTT broker tại `127.0.0.1:1883`.
- Subscribe telemetry từ các topic:
  - `swarm/+/telemetry/state`
  - `swarm/+/control/result`
- Publish lệnh điều khiển tới topic:
  - `swarm/{drone_id}/control/command`

Backend đóng vai trò trung gian giữa giao diện web và hệ thống điều khiển UAV thông qua MQTT.

### 3.2. Frontend Web Dashboard

File chính: `static/index.html`

Frontend chịu trách nhiệm:

- Hiển thị giao diện điều khiển.
- Kết nối WebSocket với backend.
- Hiển thị telemetry của UAV.
- Hiển thị bản đồ bằng Leaflet.
- Hiển thị marker và trail của từng UAV.
- Cho phép người dùng chọn UAV.
- Cho phép gửi lệnh bay bằng các nút điều khiển.
- Cho phép điều khiển bằng bàn phím trong chế độ Position.

Các phím điều khiển thủ công:

- `W`: tiến
- `S`: lùi
- `A`: sang trái
- `D`: sang phải
- `Q`: xoay trái
- `E`: xoay phải
- `R`: bay lên
- `F`: bay xuống
- `Space`: dừng

### 3.3. MAVLink Manual Bridge

File chính: `mavlink_manual_bridge.py`

Bridge này chịu trách nhiệm chuyển lệnh keyboard manual từ MQTT sang MAVLink `MANUAL_CONTROL`.

Bridge kết nối với PX4 qua UDP:

- `UAV-01`: `udpin:0.0.0.0:14540`, system id `1`
- `UAV-02`: `udpin:0.0.0.0:14541`, system id `2`

Bridge gửi lệnh manual ở tần số 20 Hz. Khi không nhận lệnh mới trong thời gian ngắn, bridge tự đưa stick về vị trí trung tâm để tránh UAV tiếp tục di chuyển ngoài ý muốn.

## 4. Luồng dữ liệu và điều khiển

### 4.1. Luồng telemetry

```text
PX4 / ROS backend
  -> MQTT telemetry topic
  -> main.py
  -> WebSocket / API
  -> static/index.html
  -> người dùng quan sát trên dashboard
```

### 4.2. Luồng lệnh điều khiển bay

```text
Người dùng bấm nút trên dashboard
  -> static/index.html
  -> WebSocket /ws
  -> main.py
  -> MQTT command topic
  -> ROS/PX4 control backend
  -> PX4
```

### 4.3. Luồng điều khiển bàn phím

```text
Người dùng nhấn W/A/S/D/Q/E/R/F/Space
  -> static/index.html gửi manual_control 20 Hz
  -> WebSocket /ws
  -> main.py publish MQTT
  -> mavlink_manual_bridge.py nhận MQTT
  -> MAVLink MANUAL_CONTROL
  -> PX4
  -> UAV di chuyển
```

## 5. Các chức năng đã hoàn thành

### 5.1. Giám sát UAV

Dashboard hiển thị các thông tin:

- Mode bay hiện tại.
- Trạng thái Armed.
- Trạng thái Preflight.
- Trạng thái Manual link.
- Failsafe.
- Battery.
- Voltage.
- GPS satellites.
- Local position.
- Global position.

### 5.2. Bản đồ

Dashboard sử dụng Leaflet để hiển thị bản đồ. Mỗi UAV có marker riêng, đồng thời có trail để quan sát quỹ đạo di chuyển.

### 5.3. Điều khiển chế độ bay

Các chế độ/chức năng được hỗ trợ:

- `POSITION`
- `OFFBOARD MAP`
- `TAKE OFF`
- `RETURN HOME`
- `ARM`
- `HOLD`
- `LAND`
- `DISARM`

### 5.4. Takeoff an toàn

Lệnh takeoff yêu cầu UAV đã được ARM trước. Độ cao takeoff được giới hạn trong khoảng:

```text
2.5 m đến 10.0 m
```

Frontend và backend đều kiểm tra giá trị độ cao để tránh gửi lệnh không hợp lệ.

### 5.5. Điều khiển bằng bàn phím

Keyboard control chỉ được bật khi UAV thỏa các điều kiện:

- UAV đã ARM.
- PX4 đang ở Position mode.
- Manual link đang OK.
- Có telemetry hợp lệ.

Điều này giúp tránh trường hợp giao diện báo đã bật điều khiển nhưng PX4 không nhận manual input.

## 6. Các lỗi đã phát hiện và xử lý

### 6.1. Không ARM lại được sau LAND hoặc RETURN HOME

Hiện tượng:

- Sau khi bấm `LAND` hoặc `RETURN HOME`, UAV không ARM lại được.
- PX4 trả kết quả `TEMPORARILY_REJECTED`.

Nguyên nhân:

- PX4 vẫn còn ở trạng thái không phù hợp như Offboard, RTL hoặc Land.
- `preflight_checks_pass=false`.
- App gửi lệnh ARM ngay khi PX4 chưa sẵn sàng.

Cách xử lý đã thực hiện:

- Frontend kiểm tra trạng thái trước khi gửi ARM.
- Backend cũng chặn ARM nếu telemetry báo UAV chưa sẵn sàng.
- Khi cần, hệ thống yêu cầu chuyển về Position trước, sau đó người dùng ARM lại.

### 6.2. App nhận phím nhưng UAV không di chuyển

Hiện tượng:

- Người dùng nhấn phím trên app.
- Giao diện nhận phím.
- UAV không di chuyển.

Nguyên nhân:

- `mavlink_manual_bridge.py` chưa chạy.
- Hoặc có 2 process bridge chạy cùng lúc, một process gửi lệnh, process còn lại gửi stick về trung tâm.
- Hoặc UAV chưa ARM / chưa ở Position / Manual link bị mất.

Cách xử lý đã thực hiện:

- Thêm hiển thị `Manual link: OK / LOST`.
- Chỉ cho bật keyboard khi đủ điều kiện.
- Kiểm tra và dừng process bridge trùng lặp.
- Xác nhận mỗi UDP port `14540` và `14541` chỉ có 1 process bridge sử dụng.

### 6.3. Lỗi port 8000 đã được sử dụng

Hiện tượng:

```text
address already in use
```

Nguyên nhân:

- Đã có một process `uvicorn` khác chạy trên port `8000`.

Cách xử lý:

```bash
fuser -v -n tcp 8000
kill <PID>
```

Sau đó chạy lại backend.

## 7. Cách vận hành hệ thống

### 7.1. Chạy backend

Mở terminal thứ nhất:

```bash
cd /home/sup/swarm_dashboard
source .venv/bin/activate
python -m uvicorn main:app --host 0.0.0.0 --port 8000
```

### 7.2. Chạy MAVLink manual bridge

Mở terminal thứ hai:

```bash
cd /home/sup/swarm_dashboard
source .venv/bin/activate
python3 mavlink_manual_bridge.py
```

Khi bridge chạy đúng, log sẽ có dạng:

```text
MQTT connected: Success
UAV-01 connected: system=1 component=1 port=14540
UAV-02 connected: system=2 component=1 port=14541
Manual bridge started at 20.0 Hz
```

### 7.3. Sử dụng dashboard

Truy cập trình duyệt:

```text
http://127.0.0.1:8000
```

Quy trình điều khiển Position bằng bàn phím:

1. Chọn UAV cần điều khiển.
2. Bấm `POSITION`.
3. Bấm `ARM`.
4. Chờ telemetry hiển thị:
   - `Armed = YES`
   - `Mode = Position`
   - `Manual link = OK`
5. Bấm `ENABLE KEYBOARD CONTROL`.
6. Dùng các phím `W/A/S/D/Q/E/R/F/Space` để điều khiển.

## 8. Kết quả kiểm thử

Các kiểm thử đã thực hiện:

- Kiểm tra cú pháp Python bằng `py_compile`.
- Kiểm tra HTML parse.
- Kiểm tra `/health`.
- Kiểm tra `/api/drones`.
- Kiểm tra WebSocket `/ws`.
- Kiểm tra nhận telemetry từ cả `UAV-01` và `UAV-02`.
- Kiểm tra validation cho:
  - JSON sai.
  - Drone ID sai.
  - Action không hỗ trợ.
  - Takeoff altitude sai.
  - Tọa độ map sai.
  - Manual control sai kiểu dữ liệu.
- Kiểm tra MAVLink bridge kết nối được PX4 UDP.
- Kiểm tra keyboard manual đi đúng luồng:
  - WebSocket
  - MQTT
  - MAVLink bridge
  - PX4

Kết quả backend/WebSocket/handler trước đó đạt:

```text
25 passed, 0 failed
```

## 9. Các file sản phẩm

Các file chính của dự án:

```text
main.py
mavlink_manual_bridge.py
static/index.html
```

File giải thích kỹ thuật:

```text
PROJECT_EXPLANATION.md
```

File báo cáo này:

```text
BAO_CAO_DU_AN.md
```

File đóng gói:

```text
swarm_app.zip
```

Nội dung `swarm_app.zip` gồm:

```text
swarm_app/main.py
swarm_app/mavlink_manual_bridge.py
swarm_app/static/index.html
```

## 10. Đánh giá kết quả

Dự án đã xây dựng được một dashboard điều khiển UAV hoạt động theo thời gian thực, có khả năng nhận telemetry, hiển thị bản đồ, gửi lệnh điều khiển và điều khiển UAV bằng bàn phím trong Gazebo/PX4.

Điểm quan trọng của dự án là đã tách rõ các thành phần:

- Web dashboard để người dùng thao tác.
- Backend để quản lý WebSocket và MQTT.
- MAVLink bridge để chuyển manual control sang PX4.

Việc tách riêng này giúp hệ thống dễ mở rộng, dễ debug và dễ thay thế từng thành phần.

## 11. Hướng phát triển tiếp theo

Một số hướng có thể phát triển thêm:

- Thêm giao diện điều chỉnh tốc độ keyboard trực tiếp trên dashboard.
- Thêm log lệnh điều khiển theo thời gian.
- Thêm trạng thái chi tiết của command ACK.
- Thêm cảnh báo rõ hơn khi PX4 từ chối lệnh.
- Thêm chức năng lưu quỹ đạo bay.
- Thêm hỗ trợ nhiều hơn 2 UAV.
- Thêm cơ chế tự khởi động bridge cùng backend.
- Thêm test tự động cho frontend.
- Thêm file cấu hình riêng cho danh sách UAV, port UDP và system id.

## 12. Kết luận

Dự án đã hoàn thành được một hệ thống dashboard điều khiển swarm UAV cơ bản nhưng đầy đủ các thành phần cần thiết: giao diện web, backend realtime, MQTT communication và MAVLink manual bridge. Hệ thống đã được kiểm thử trên môi trường Gazebo/PX4 với 2 UAV và đã xử lý được các lỗi vận hành quan trọng như không ARM lại được sau LAND/RTL, keyboard không điều khiển được do thiếu bridge, và lỗi chạy trùng process.

Kết quả này có thể dùng làm nền tảng để phát triển tiếp các chức năng điều khiển swarm nâng cao hơn như bay đội hình, điều khiển waypoint đồng thời, phân công nhiệm vụ và giám sát trạng thái nhiều UAV trên cùng một dashboard.
