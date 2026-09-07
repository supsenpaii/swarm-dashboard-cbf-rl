# Tài liệu thay đổi Dashboard UAV và Tracking 30 FPS

## 1. Mục tiêu

Các thay đổi được thực hiện nhằm giải quyết bốn yêu cầu chính:

1. Loại bỏ hai khung camera riêng trong từng tab UAV.
2. Thêm một nút `TRACKING` dùng chung bên ngoài các tab UAV.
3. Tích hợp trực tiếp mã tracking từ package:
   `/home/sup/ws_px4/src/lfc_gimbal_gazebo`.
4. Lấy ảnh trực tiếp từ Gazebo Transport, sửa lỗi telemetry UAV-01 và tối ưu
   luồng tracking trên web lên khoảng 30 FPS thực tế.

## 2. Các file đã thay đổi

### Trong dashboard

- `main.py`
  - Quản lý kết nối MQTT, API, Gazebo Transport và camera.
  - Đọc trực tiếp `gz.msgs.Image` từ topic camera.
  - Chuyển ảnh Gazebo sang BGR bằng NumPy/OpenCV.
  - Chỉ xử lý camera khi có client camera cũ hoặc tracking đang chạy.
  - Cung cấp API tracking và các chỉ số hiệu năng.

- `tracking_web.py`
  - Import kiểu dữ liệu và tracker từ `lfc_gimbal_gazebo`.
  - Quản lý vòng đời của một phiên tracking.
  - Dùng KCF làm OpenCV tracker mặc định.
  - Nhận frame BGR mới nhất, tracking, vẽ overlay và encode JPEG.
  - Phát MJPEG bằng async generator.

- `static/index.html`
  - Loại bỏ camera riêng khỏi từng tab UAV.
  - Thêm nút `TRACKING` dùng chung.
  - Thêm khung tracking và canvas để kéo chọn bounding box.
  - Hiển thị trạng thái, tracker, FPS output và FPS nguồn.

- `isolated_swarm.launch.py`
  - Tách PX4 DDS namespace thành `/px4_0` và `/px4_1`.
  - Remap telemetry và control của UAV-01 đúng về `/px4_0`.
  - Giữ UAV-02 trên `/px4_1`.

### Ngoài dashboard

- `/mnt/px4ssd/PX4-Autopilot/Tools/simulation/gz/models/gimbal/model.sdf`
  - Đổi camera từ `1280x720` xuống `640x360`.
  - Đặt sensor camera thành `32 Hz` để bù sai số scheduler và duy trì ít
    nhất 30 FPS thực trên MJPEG client.

## 3. Kiến trúc tracking mới

Luồng dữ liệu hiện tại:

```text
Gazebo camera topic
    -> gz.msgs.Image
    -> NumPy/OpenCV BGR
    -> latest-frame slot
    -> KCF tracker
    -> vẽ bbox + FPS overlay
    -> JPEG encode một lần
    -> async MJPEG
    -> thẻ <img> trên trình duyệt
```

Topic UAV-01:

```text
/world/default/model/x500_custom_0/link/camera_link/sensor/camera/image
```

Topic UAV-02:

```text
/world/default/model/x500_custom_1/link/camera_link/sensor/camera/image
```

### Khác biệt so với luồng cũ

Luồng cũ thực hiện:

```text
Gazebo Image -> JPEG -> lưu bridge -> JPEG decode -> tracker -> JPEG encode
```

Luồng mới loại bỏ hoàn toàn bước JPEG encode/decode trung gian:

```text
Gazebo Image -> BGR -> tracker -> JPEG encode
```

Nhờ vậy mỗi frame tracking chỉ được encode JPEG đúng một lần.

## 4. Latest-frame slot

Dashboard không dùng queue tích lũy nhiều frame. Bridge chỉ giữ:

- Frame BGR mới nhất.
- Phiên bản frame mới nhất.
- Thời điểm nhận frame gần nhất.

Nếu tracker đang bận và Gazebo gửi thêm frame, frame cũ chưa xử lý sẽ được thay
bằng frame mới. Cách này có các ưu điểm:

- Không tạo backlog.
- Không tăng độ trễ dần theo thời gian.
- Web luôn hiển thị trạng thái camera gần thời gian thực nhất.
- Tracker không phải xử lý các frame đã lỗi thời.

## 5. Tracker OpenCV

### Mặc định hiện tại

Tracker mặc định là `KCF`:

```text
SWARM_OPENCV_TRACKER=KCF
```

KCF được chọn vì nhanh hơn CSRT trong tình huống tracking trực tiếp trên web.
Mã vẫn sử dụng giao diện, `BBox`, `TrackResult` và `TrackState` của package
`lfc_gimbal_gazebo`.

### Chuyển lại CSRT

Nếu ưu tiên độ chính xác hơn FPS, có thể chạy dashboard với:

```bash
SWARM_OPENCV_TRACKER=CSRT
```

CSRT thường dùng nhiều CPU hơn và FPS thực tế thấp hơn KCF.

Các tracker được hỗ trợ trong adapter hiện tại:

- `KCF`
- `CSRT`
- `MOSSE`

MOSSE rất nhanh nhưng không ổn định bằng KCF trên cảnh Gazebo đã thử nghiệm.

## 6. API tracking

### Bắt đầu tracking

```http
POST /api/tracking/start
Content-Type: application/json

{"drone_id":"UAV-01"}
```

### Chọn bounding box

Các giá trị đều được chuẩn hóa từ `0` đến `1` theo kích thước frame:

```http
POST /api/tracking/bbox
Content-Type: application/json

{
  "x": 0.35,
  "y": 0.30,
  "width": 0.25,
  "height": 0.30
}
```

### Xem stream

```http
GET /api/tracking/stream
```

Response là:

```text
multipart/x-mixed-replace; boundary=frame
```

### Dừng tracking

```http
POST /api/tracking/stop
```

Khi dừng, dashboard sẽ:

- Reset tracker.
- Xóa frame output.
- Xóa raw frame khỏi bridge.
- Đóng async MJPEG stream hiện tại.
- Ngừng convert và encode camera nếu không còn client camera cũ.

## 7. Giao diện web

Nút `TRACKING` nằm bên ngoài tab UAV và dùng UAV đang được chọn trên giao diện.

Quy trình sử dụng:

1. Chọn tab UAV-01 hoặc UAV-02.
2. Nhấn `TRACKING`.
3. Chờ hình camera xuất hiện.
4. Kéo chuột trên hình để chọn mục tiêu.
5. Tracker bắt đầu cập nhật bounding box.
6. Nhấn `DỪNG TRACKING` để kết thúc.

Overlay trên ảnh hiển thị dạng:

```text
UAV-01 | KCF | OUT 31.9 FPS | SRC 32.1
TRACKING | score=1.000
```

Trong đó:

- `OUT`: tốc độ frame đã tracking, vẽ overlay và encode.
- `SRC`: tốc độ frame dashboard nhận từ topic Gazebo.

Chữ overlay được vẽ hai lớp, gồm viền đen và chữ sáng, để đọc được trên cả
nền trời sáng và nền tối.

## 8. Các chỉ số hiệu năng

`GET /health` và `GET /api/drones` trả thêm các trường:

### Trong `tracking`

- `tracker`: tracker đang dùng, ví dụ `KCF`.
- `fps`: FPS output sau tracking và encode.
- `source_fps`: FPS nhận từ Gazebo topic.
- `tracking_ms`: thời gian tracker xử lý một frame.
- `encode_ms`: thời gian encode JPEG một frame.
- `last_frame_age_ms`: tuổi của output frame mới nhất.

### Trong `gazebo.cameras.<drone_id>`

- `source_fps`: FPS callback của topic camera.
- `conversion_ms`: thời gian chuyển `gz.msgs.Image` sang BGR.
- `has_raw_frame`: bridge có raw frame đang hoạt động hay không.
- `last_raw_frame_age_ms`: tuổi raw frame mới nhất.
- `clients`: số client đang dùng stream camera cũ.
- `tracking`: camera UAV này có đang được tracking hay không.

## 9. Kết quả benchmark

### Trước tối ưu trực tiếp

- Tracking stream khoảng `8.6-13 FPS`.
- Camera Gazebo là `1280x720`.
- Tracker mặc định là CSRT.
- Có bước JPEG encode/decode trung gian.
- MJPEG dùng sync generator qua threadpool.

### Sau tối ưu

Benchmark clean simulation với UAV-01 và KCF active:

| Chỉ số | Kết quả |
|---|---:|
| Gazebo source FPS | 32.1 FPS |
| Tracking output FPS | 32.1 FPS |
| MJPEG client FPS | 31.9 FPS |
| Gazebo Image -> BGR | 0.57 ms |
| KCF tracking | 19.72 ms |
| JPEG encode | 2.18 ms |
| Dashboard CPU khi tracking | khoảng 94.5% của một CPU core |

Mục tiêu 30 FPS trên web đã đạt. Sensor dùng 32 Hz để FPS thực nhận tại client
không tụt dưới 30 do sai số scheduler, transport và MJPEG delivery.

## 10. Lazy camera processing

Dashboard vẫn subscribe hai camera Gazebo để có thể bật tracking ngay, nhưng
không convert hoặc encode ảnh liên tục khi không có nhu cầu.

Camera chỉ được xử lý khi:

- Có client dùng API camera cũ; hoặc
- Tracking đang chạy trên UAV tương ứng.

Khi tracking tắt và không có camera client:

- Callback chỉ cập nhật bộ đếm source FPS.
- Không resize ảnh.
- Không chạy tracker.
- Không encode JPEG.

Thay đổi này làm giảm CPU dashboard khi ở trạng thái chờ.

## 11. Sửa lỗi UAV-01 bị giật và không điều khiển được

### Nguyên nhân

UAV-01 trước đây dùng các topic PX4 ở namespace gốc `/fmu/...`. Có hai nguồn
DDS cùng publish vào một số topic này, làm dashboard nhận dữ liệu xen kẽ giữa
hai UAV.

Biểu hiện đã quan sát:

- Tọa độ UAV-01 nhảy qua lại giữa hai vị trí.
- `manual_control_signal_lost` thay đổi liên tục.
- Lệnh điều khiển app không đi đúng PX4 instance.
- Web hiển thị telemetry bị giật.

### Cách sửa

PX4 được tách namespace:

```text
UAV-01 -> /px4_0
UAV-02 -> /px4_1
```

PX4 được chạy với:

```text
PX4_UXRCE_DDS_NS=px4_0
PX4_UXRCE_DDS_NS=px4_1
```

`isolated_swarm.launch.py` remap telemetry và control tương ứng.

Kết quả kiểm tra cuối:

- `/px4_0/fmu/out/vehicle_local_position`: một publisher.
- `/px4_1/fmu/out/vehicle_local_position`: một publisher.
- `/px4_0/fmu/in/manual_control_input`: một publisher và một subscriber.
- UAV-01 online ổn định.
- `manual_control_signal_lost=false`.

## 12. Dịch vụ đang sử dụng

Các tiến trình hiện được quản lý bằng user systemd:

```text
swarm-dashboard.service
swarm-telemetry-isolated.service
swarm-xrce.service
swarm-px4-0.service
swarm-px4-1.service
```

Kiểm tra nhanh:

```bash
systemctl --user is-active \
  swarm-dashboard.service \
  swarm-telemetry-isolated.service \
  swarm-xrce.service \
  swarm-px4-0.service \
  swarm-px4-1.service
```

PX4 chạy với cờ `-d` để tắt PXH interactive shell. Nếu không có cờ này khi
chạy qua systemd, PX4 có thể spam journal vì stdin không phải terminal.

Hai PX4 service và dashboard hiện là transient user services. Chúng hoạt động
trong phiên user hiện tại nhưng không phải unit file cài cố định cho mọi lần
khởi động máy.

## 13. Kiểm tra health

```bash
curl -sS http://127.0.0.1:8000/health
```

Các giá trị quan trọng:

```text
ok=true
mqtt_connected=true
gazebo.started=true
tracking.available=true
tracking.tracker=KCF
```

Khi tracking đang chạy, kiểm tra thêm:

```text
tracking.active=true
tracking.has_frame=true
tracking.fps >= 30
tracking.source_fps >= 30
```

## 14. Cấu hình môi trường

### Package tracking

```text
SWARM_TRACKING_PACKAGE_ROOT=/home/sup/ws_px4/src/lfc_gimbal_gazebo
```

### Backend

```text
SWARM_TRACKING_BACKEND=opencv
```

Các backend khác vẫn được giữ theo package gốc:

- `onnx`
- `board`

### OpenCV tracker

```text
SWARM_OPENCV_TRACKER=KCF
```

### JPEG quality

```text
SWARM_TRACKING_JPEG_QUALITY=72
```

Giá trị được giới hạn trong khoảng `45-95`.

## 15. Lưu ý vận hành

- Không chạy thêm một Gazebo server cùng partition `default`.
- Nếu có hai server Gazebo cùng world, topic camera và model có thể bị trùng.
- Trước khi launch lại PX4, kiểm tra không còn server cũ bằng:

```bash
ps -eo pid,ppid,cmd | rg 'gz sim|px4 -d -i'
```

- Chỉ nên có:
  - Một `gz sim --verbose ... -s`.
  - Một `gz sim -g`.
  - Một PX4 instance `-i 0`.
  - Một PX4 instance `-i 1`.

- Thay đổi `model.sdf` chỉ có hiệu lực sau khi restart Gazebo/PX4 simulation.
- Sau khi restart PX4 phải giữ `PX4_UXRCE_DDS_NS`, nếu không telemetry có thể
  quay lại namespace gốc và gây trộn dữ liệu.
- Khi không sử dụng tracking nên nhấn `DỪNG TRACKING` để giảm CPU dashboard.

## 16. Kiểm thử đã thực hiện

- Python compile:

```bash
.venv/bin/python -m py_compile main.py tracking_web.py isolated_swarm.launch.py
```

- Kiểm tra HTML parser:

```bash
python3 -m html.parser static/index.html
```

- Kiểm tra XML của model Gazebo.
- Kiểm tra start, bbox, stream và stop tracking.
- Kiểm tra MJPEG client trong 8 giây.
- Kiểm tra source/output/client FPS riêng biệt.
- Kiểm tra một publisher cho từng DDS namespace.
- Lấy nhiều mẫu telemetry UAV-01 để xác nhận không còn nhảy dữ liệu.
- Kiểm tra chỉ còn một Gazebo server và hai PX4 instance.

## 17. Trạng thái bàn giao

- Dashboard hoạt động.
- MQTT kết nối.
- Gazebo bridge hoạt động.
- UAV-01 và UAV-02 dùng DDS namespace riêng.
- Tracking dùng KCF và raw Gazebo frame trực tiếp.
- Web đạt trên 30 FPS trong benchmark.
- FPS được hiển thị trực tiếp trên khung tracking.
- Tracking đã được dừng sau benchmark để trả CPU về trạng thái chờ.

## 18. Auto-gimbal theo vật thể tracking

Dashboard đã tích hợp trực tiếp `GimbalService` từ:

```text
/home/sup/ws_px4/src/lfc_gimbal_gazebo/lfc_gimbal_gazebo/services/gimbal_service.py
```

Controller gốc của package gồm:

- Chuyển sai số pixel sang sai số góc camera.
- EMA lọc góc mục tiêu.
- Deadband 1 độ quanh tâm ảnh.
- PID pan và tilt có anti-windup.
- Giới hạn tốc độ góc.
- Slew-rate limit để tránh giật gimbal.

Dashboard dùng output PID như vận tốc góc:

```text
omega[0] -> yaw rate
omega[1] -> pitch rate
```

Sau đó tích phân theo thời gian frame:

```text
yaw_target   = yaw_current   + degrees(omega_pan)  * dt
pitch_target = pitch_current + degrees(omega_tilt) * dt
```

Ảnh tracking chỉ cung cấp hai sai số không gian ảnh nên không thể suy ra roll
của mục tiêu. Roll được điều khiển riêng về `0°` với slew limit để giữ đường
chân trời ổn định.

Ba target được giới hạn theo joint gimbal:

```text
roll:   -45° ..  45°
pitch: -135° ..  45°
yaw:   -180° .. 180°
```

Mỗi frame tracking gửi ba message `gz.msgs.Double` dạng radian tới đúng model
của UAV đang tracking:

```text
/model/x500_custom_0/command/gimbal_roll
/model/x500_custom_0/command/gimbal_pitch
/model/x500_custom_0/command/gimbal_yaw
```

Với UAV-02, model tự đổi thành `x500_custom_1`.

Ví dụ lệnh tương đương trên CLI:

```bash
gz topic -t /model/x500_custom_0/command/gimbal_pitch \
  -m gz.msgs.Double -p 'data: -0.45'
```

### Camera calibration dùng cho PID

Camera hiện có độ phân giải `640x360`, horizontal FOV là `2.0 rad`. Focal
length mặc định của controller web được hiệu chỉnh thành khoảng `205.5 px`:

```text
SWARM_TRACKING_GIMBAL_FX=205.5
SWARM_TRACKING_GIMBAL_FY=205.5
```

Các cấu hình bổ sung:

```text
SWARM_TRACKING_GIMBAL_ENABLED=true
SWARM_TRACKING_GIMBAL_ROLL_RATE_DEG_S=90
```

### Bảo vệ xung đột điều khiển

Khi bbox đã được chọn và auto-gimbal đang chạy:

- Nút roll/pitch/yaw thủ công bị disable trên web.
- Lệnh thủ công gửi trực tiếp qua WebSocket bị backend từ chối.
- Lệnh HOME cũng bị từ chối cho tới khi tracking được dừng.
- Khi tracker LOST, PID được reset và yaw/pitch giữ target gần nhất.
- Khi dừng tracking, gimbal giữ góc cuối; người dùng có thể nhấn HOME.

### Overlay và health

Overlay tracking hiển thị:

```text
GIMBAL AUTO | R +0.0 P -26.1 Y +8.3 deg
RATE | P +0.2 Y +0.2 deg/s
```

Tracking status trả thêm:

- `gimbal_enabled`
- `gimbal_active`
- `gimbal_angles_deg`
- `gimbal_rates_deg_s`
- `gimbal_error`
- `gimbal_last_command_age_ms`

### Kết quả live test

Bbox thử nghiệm ban đầu có tâm dọc khoảng `y=295`, trong khi tâm frame là
`y=180`. Auto-gimbal đã điều khiển pitch xuống khoảng `-30.6°`, đưa tâm bbox
về khoảng `y=179`. Các lệnh thực đọc được trên Gazebo gồm:

```text
pitch data: -0.4558 rad
yaw data:    0.1446 rad
roll data:   0 rad
```

Sau test, tracking đã được stop và HOME đưa roll/pitch/yaw về `0°`.

## 19. Bổ sung Kalman, Memory và ReID

Ngày 18/07/2026, package nguồn đã được đọc và tích hợp lại từ:

```text
/home/sup/ws_px4/src/lfc_gimbal_gazebo
```

Tên thư mục cũ có một dấu cách ở cuối (`lfc_gimbal_gazebo `), khiến Python và
colcon không tìm thấy package qua đường dẫn được cấu hình. Thư mục đã được đổi
về tên chuẩn không có dấu cách. `tracking_web.py` vẫn có cơ chế tự dò cả hai
dạng tên để tránh lỗi nếu package bị copy lại với tên cũ.

### Backend mặc định mới

Backend mặc định đổi từ `opencv` sang `hybrid`:

```text
KCF mỗi frame -> kiểm tra appearance ReID -> Kalman correct
       | thất bại
       v
Kalman predict -> OCCLUDED -> quét re-detect -> xác nhận 2 lần -> KCF re-init
```

Backend này giữ KCF ở luồng xử lý thường để không bắt buộc model AI và hạn chế
ảnh hưởng FPS. Quét nhiều tỉ lệ chỉ chạy khi mục tiêu đã mất.

### Kalman Filter

Dashboard dùng trực tiếp `KalmanCV` của package. Vector trạng thái gồm:

```text
[center_x, center_y, velocity_x, velocity_y]
```

- Khi KCF/ReID chấp nhận bbox, tâm bbox được đưa vào bước `correct`.
- Khi KCF thất bại, bước `predict` tiếp tục ước lượng vị trí mục tiêu.
- Vận tốc được giảm dần bằng hệ số `0.92` khi mục tiêu bị che.
- Trước ngưỡng re-detect, bbox dự đoán được trả với trạng thái `OCCLUDED`.

### Appearance Memory

Dashboard dùng trực tiếp `TemplateMemory` của package và lưu:

- feature LAB chuẩn hóa từ crop `24x24`;
- histogram hue/saturation;
- histogram gradient texture;
- tối đa 30 feature theo thời gian;
- lịch sử vận tốc, vị trí thoát và blacklist distractor.

Memory chỉ cập nhật khi similarity đủ cao và theo chu kỳ 5 frame để giảm nguy
cơ model bị trôi sang vật thể khác.

### ReID và re-detect

ReID hiện là appearance ReID nhẹ, không phải mạng embedding người/xe riêng.
Điểm nhận dạng kết hợp cosine similarity của feature LAB, màu, texture và
temporal memory.

Khi mục tiêu mất, hệ thống:

1. Dùng template matching trên toàn frame với 5 mức scale.
2. Lấy tối đa 3 peak ở mỗi scale.
3. Chấm điểm template, appearance ReID, khoảng cách Kalman và exit memory.
4. Loại bbox nằm trong blacklist.
5. Yêu cầu cùng ứng viên xuất hiện liên tiếp trước khi khởi tạo lại KCF.

Package gốc cũng đã được sửa để `TrackerService`:

- không còn đưa boolean `True` vào `TemplateMemory`;
- trích feature thật từ template/crop;
- so ReID với crop ứng viên re-detect thay vì so template với chính crop cũ;
- cập nhật temporal memory bằng crop đã được tracker chấp nhận.

`tracking_node` trước đây publish `/tracker/bbox` bằng `geometry_msgs/Point`
trong khi `gimbal_control_node` subscribe `geometry_msgs/Pose`. Topic đã được
đồng bộ sang `Pose`, với `position.x/y/z = x/y/w` và `orientation.x = h`.

### Cấu hình

```bash
SWARM_TRACKING_BACKEND=hybrid
SWARM_OPENCV_TRACKER=KCF
SWARM_REID_AFTER=5
SWARM_REID_INTERVAL=2
SWARM_REID_CONFIRM=2
SWARM_REID_TRACK_THRESHOLD=0.38
SWARM_REID_THRESHOLD=0.48
SWARM_REID_SCORE_THRESHOLD=0.58
SWARM_MEMORY_UPDATE_THRESHOLD=0.72
```

Có thể quay lại KCF thuần bằng:

```bash
SWARM_TRACKING_BACKEND=opencv
```

Hai backend LightFC vẫn được giữ:

```bash
SWARM_TRACKING_BACKEND=onnx
SWARM_TRACKING_BACKEND=board
```

Máy hiện chưa có `lightfc_core.onnx`, vì vậy `onnx` chỉ chạy sau khi copy model
vào `lfc_gimbal_gazebo/onnx/` hoặc cấu hình `EDGE_MODEL_URL`. Backend `hybrid`
không cần model này.

### Đường dẫn được đổi theo máy hiện tại

- Package: `/home/sup/ws_px4/src/lfc_gimbal_gazebo`
- GCU Python: `/home/sup/Downloads/gimbal-server-main`
- Dashboard camera URL: `http://127.0.0.1:8000/api/camera/UAV-01/stream`
- Gazebo camera topic vẫn là
  `/world/default/model/x500_custom_0/link/camera_link/sensor/camera/image`.

### Dữ liệu hiển thị mới

`tracking.advanced` trong API chứa:

- `kalman`, `memory`, `reid`;
- `memory_entries`;
- `reid_score`, `match_score`;
- `lost_count`;
- `kalman_velocity_px`.

Overlay camera hiển thị `KALMAN + MEMORY + REID`, similarity và số mẫu memory.
Thanh trạng thái web cũng hiển thị `Kalman/Memory/ReID` và điểm ReID hiện tại.

### Kiểm thử đã chạy

- `py_compile` cho dashboard và toàn bộ file package đã sửa.
- HTML parser cho `static/index.html`.
- Import từ đúng package root không có lỗi.
- Test LightFC bằng inference giả xác nhận tracker init/update và memory tăng mẫu.
- Test video tổng hợp xác nhận chuỗi trạng thái
  `TRACKING -> OCCLUDED -> LOST -> TRACKING` sau khi vật thể xuất hiện lại.

## 20. Drone yaw follow theo gimbal

Dashboard đã bổ sung vòng điều khiển ngoài để thân drone quay theo hướng yaw
của gimbal trong khi gimbal vẫn tiếp tục giữ vật thể ở giữa ảnh.

```text
gimbal yaw + horizontal bbox error -> body-yaw coordinator -> MAVLink MANUAL_CONTROL
          |
          +-> gimbal yaw trim [-15°, +15°]

vertical bbox error   -> gimbal pitch PID
```

### Luật điều khiển

- Sai số hướng của drone là tổng `gimbal_yaw + bbox_image_error`. Đây là góc
  line-of-sight của mục tiêu so với hướng thân drone.
- Khi tracking, yaw gimbal trim trong `[-15°, +15°]` để hấp thụ chuyển động nhanh
  và giữ mục tiêu trong khung lúc thân drone có quán tính.
- Drone tiếp tục yaw cho tới khi line-of-sight gần `0°`; kết quả là yaw gimbal
  cũng có xu hướng tự trở về gần `0°` thay vì nằm ở biên.
- Drone bắt đầu yaw khi LOS đã lọc vượt `0.8°` và dừng dưới `0.25°`, nhờ đó
  bbox và gimbal được kéo sát tâm hơn.
- Hai ngưỡng khác nhau tạo hysteresis, tránh bật/tắt liên tục quanh một góc.
- Góc gimbal được lọc EMA trước khi tính lệnh.
- Lệnh yaw được giới hạn và áp dụng slew-rate để drone không giật.
- Chiều mặc định: gimbal yaw dương thì thân drone yaw dương.

Công thức lệnh chuẩn hóa:

```text
image_error = atan2(bbox_center_x - image_center_x, camera_fx)
target_error = gimbal_yaw + image_error
direction = sign(filtered_target_error)
command = direction * min(max_command, Kp * (abs(error) - exit_deadband))
```

### Điều kiện an toàn

Body yaw chỉ được gửi khi:

- tracker ở trạng thái `TRACKING`;
- confidence đạt ngưỡng tối thiểu;
- lệnh gimbal Gazebo thành công;
- UAV online và đã ARM;
- PX4 đang ở Position mode (`nav_state=2`);
- PX4 không failsafe;
- manual-control signal không bị lost;
- độ cao local tối thiểu `0.5 m`.

Khi tracker chuyển `OCCLUDED` hoặc `LOST`, stop tracking, lỗi gimbal hay mất
MQTT, dashboard gửi `tracking_yaw enabled=false`. Bridge cũng có timeout
`0.35 s`; nếu không nhận lệnh mới thì tự trả yaw về giữa.

### Ưu tiên điều khiển

`mavlink_manual_bridge.py` có hai state độc lập:

1. `ManualState` cho bàn phím W/A/S/D/Q/E/R/F.
2. `TrackingYawState` cho body yaw tự động.

Thứ tự ưu tiên:

```text
MANUAL > TRACKING_YAW > CENTERED
```

Nếu keyboard control đang enabled, lệnh người dùng được dùng toàn bộ và
tracking yaw không ghi đè. Khi manual timeout/disable, tracking yaw tự tiếp tục
nếu vẫn còn lệnh hợp lệ.

Dashboard cũng giữ một manual-override deadline `0.4 s` sau mỗi frame bàn phím.
Trong khoảng này outer-loop không báo body yaw active; nếu WebSocket mất thì
deadline tự hết hạn thay vì khóa auto-yaw vĩnh viễn.

### MQTT message mới

```json
{
  "type": "tracking_yaw",
  "drone_id": "UAV-01",
  "enabled": true,
  "yaw": 0.25,
  "target_yaw_error_deg": 12.5,
  "track_state": "tracking"
}
```

`yaw` là stick chuẩn hóa, được bridge giới hạn trong `[-0.55, 0.55]`, sau đó
đổi sang trường `r` của MAVLink `MANUAL_CONTROL` bằng
`TRACKING_YAW_SCALE=600`. Manual keyboard vẫn dùng `YAW_SCALE=400`.

### Cấu hình môi trường

```bash
SWARM_TRACKING_BODY_YAW_ENABLED=true
SWARM_TRACKING_GIMBAL_YAW_LIMIT_DEG=15
SWARM_TRACKING_GIMBAL_KP=6.5
SWARM_TRACKING_GIMBAL_KI=0.25
SWARM_TRACKING_GIMBAL_KD=0.20
SWARM_TRACKING_GIMBAL_EMA_ALPHA=0.55
SWARM_TRACKING_GIMBAL_DEADBAND_DEG=0.5
SWARM_TRACKING_GIMBAL_OMEGA_MAX_DEG=120
SWARM_TRACKING_GIMBAL_SLEW_MAX_DEG=300
SWARM_TRACKING_BODY_YAW_ENTER_DEG=2.5
SWARM_TRACKING_BODY_YAW_EXIT_DEG=1.0
SWARM_TRACKING_BODY_YAW_KP=0.018
SWARM_TRACKING_BODY_YAW_MAX=0.55
SWARM_TRACKING_BODY_YAW_SLEW=1.4
SWARM_TRACKING_BODY_YAW_EMA_ALPHA=0.22
SWARM_TRACKING_BODY_YAW_MIN_SCORE=0.35
SWARM_TRACKING_BODY_YAW_MIN_ALTITUDE_M=0.5
SWARM_TRACKING_BODY_YAW_INVERT=false
```

PID gimbal đã được giảm gain tích phân, giảm EMA response, giới hạn tốc độ góc
và slew để hạn chế giật/rung. Body yaw dùng LOS tổng hợp nên vừa giữ mục tiêu,
vừa chủ động recenter gimbal. Tracking yaw dùng scale MAVLink riêng `600`, giới
hạn lệnh `0.55`; Manual vẫn dùng scale `400`, do đó không bị tăng độ nhạy.

Nếu test thực tế cho thấy thân drone quay ngược hướng cần thiết, đặt:

```bash
SWARM_TRACKING_BODY_YAW_INVERT=true
```

### Feedback góc gimbal

Gazebo hiện chỉ expose ba command topic gimbal, không có joint-state topic
riêng. World pose có `Pose_V` nhưng các link `camera_link` không scoped theo
model một cách ổn định. Vì vậy outer-loop hiện dùng góc yaw lệnh đã publish
thành công và được lưu trong `gimbal_angles_deg`. Camera tracking vẫn đóng vòng
qua ảnh nên khi thân drone quay, gimbal PID sẽ điều chỉnh góc tương đối ngược
lại để giữ vật thể.

### Service đang chạy

Bridge thủ công cũ đã được thay bằng user service:

```bash
systemctl --user status swarm-mavlink-manual.service
```

Service dùng `Restart=on-failure` và chạy:

```text
/usr/bin/python3 /home/sup/swarm_dashboard/mavlink_manual_bridge.py
```

### Kiểm thử đã chạy

- `py_compile` và HTML parser thành công.
- Unit test xác nhận hysteresis, EMA/slew, hai chiều yaw và LOST-stop.
- Unit test xác nhận manual command có ưu tiên cao hơn tracking yaw.
- Safety test xác nhận chặn khi disarmed hoặc độ cao dưới ngưỡng.
- Live MQTT test trên UAV-01 khi disarmed xác nhận chuỗi log:
  `TRACKING_YAW -> MANUAL -> TRACKING_YAW -> CENTERED`.
- Dashboard và MAVLink bridge đều được restart, MQTT/MAVLink kết nối thành công.

## 21. Drone tự bám và giữ khoảng cách mục tiêu

### Mục tiêu

Drone tracking giờ không chỉ quay thân theo yaw gimbal mà còn tự tiến hoặc lùi
để giữ khoảng cách do người dùng chọn. Giao diện có ô `Khoảng cách bám mục tiêu`
ngay dưới nút `TRACKING`, nhận giá trị từ `1` đến `20 m`, mặc định `5 m` và lưu
riêng cho từng UAV.

Không điều khiển tốc độ từng motor trực tiếp. Dashboard gửi `MANUAL_CONTROL`
theo trục tiến/lùi; PX4 Position mode tự giữ ổn định attitude/altitude và mixer
của PX4 tính lệnh motor. Cách này an toàn và đúng tầng điều khiển hơn việc bypass
flight controller.

### Nguồn đo khoảng cách

Dashboard subscribe trực tiếp Gazebo topic:

```text
/x500_custom/front_lidar
```

Topic có hai publisher `gz.msgs.LaserScan`. Trường `frame` được dùng để ánh xạ
scan về đúng model `x500_custom_0` hoặc `x500_custom_1`. Bộ điều khiển lấy median
các tia hợp lệ trong cửa sổ khoảng `±2°` quanh hướng yaw hiện tại của gimbal để
giảm nhiễu và loại `inf`, `NaN` hoặc giá trị ngoài giới hạn sensor.

Nếu LiDAR không có phản hồi hợp lệ, hệ thống dùng kích thước bbox làm fallback:

```text
distance = reference_distance * sqrt(reference_bbox_area / current_bbox_area)
```

Khi LiDAR có số đo, bbox reference và reference distance được hiệu chỉnh lại.
Nếu lúc chọn mục tiêu chưa có LiDAR, kích thước bbox được coi là kích thước tại
khoảng cách mục tiêu đã nhập; fallback visual sau đó vẫn giữ được khoảng cách
tương đối.

### Bộ điều khiển tiến/lùi

Sai số khoảng cách:

```text
error = measured_distance - target_distance
```

- `error > 0`: vật thể xa, gửi lệnh tiến.
- `error < 0`: vật thể quá gần, gửi lệnh lùi.
- `|error| <= 0.4 m`: trạng thái `HOLDING`, lệnh tiến/lùi bằng `0`.
- `|bbox_yaw_error| > 8°`: trạng thái `ALIGNING`, dừng tịnh tiến để thân drone
  quay về hướng mục tiêu trước rồi mới tiếp tục tiến/lùi.
- Khoảng cách được lọc EMA; lệnh được giới hạn và qua slew-rate để tránh giật.

Thông số mặc định:

```bash
SWARM_TRACKING_FOLLOW_ENABLED=true
SWARM_TRACKING_FOLLOW_KP=0.12
SWARM_TRACKING_FOLLOW_MAX=0.30
SWARM_TRACKING_FOLLOW_SLEW=0.60
SWARM_TRACKING_FOLLOW_DEADBAND_M=0.40
SWARM_TRACKING_FOLLOW_ALIGN_YAW_DEG=10
SWARM_TRACKING_FOLLOW_EMA_ALPHA=0.35
SWARM_TRACKING_FOLLOW_MIN_SCORE=0.70
```

### MQTT và MAVLink

Dashboard publish:

```json
{
  "type": "tracking_follow",
  "drone_id": "UAV-01",
  "enabled": true,
  "forward": 0.2,
  "right": 0.0,
  "up": 0.0,
  "measured_distance_m": 7.1,
  "target_distance_m": 5.0,
  "distance_source": "lidar",
  "track_state": "tracking"
}
```

`mavlink_manual_bridge.py` ghép `forward/right/up` của follow với `yaw` của
body-yaw thành một frame MAVLink `MANUAL_CONTROL`. Thứ tự ưu tiên là:

```text
MANUAL > TRACKING_FOLLOW + TRACKING_YAW > TRACKING_YAW > CENTERED
```

Follow giới hạn trục ngang ở `±0.35`, hiện chỉ sử dụng tiến/lùi tối đa `±0.30`.
Bridge chạy `20 Hz`; watchdog `0.35 s` tự đưa mọi trục về giữa nếu mất lệnh.
Riêng follow dùng toàn dải MAVLink `1000` cho trục ngang; vì vậy `F=0.15`
thành `x=150`. Manual keyboard vẫn dùng scale `500`. Việc tách hai scale tránh
lệnh follow nhỏ rơi vào deadzone của PX4 Position mode mà không làm keyboard
nhạy gấp đôi.

### Điều kiện dừng và an toàn

Lệnh follow chỉ được phép khi UAV online, đã ARM, ở Position mode, không failsafe,
manual-control signal hợp lệ và cao tối thiểu `0.5 m`. Nó dừng ngay khi:

- tracker không còn `TRACKING`, score dưới ngưỡng hoặc bbox mất;
- tracker chuyển `OCCLUDED`/`LOST`;
- gimbal command lỗi;
- người dùng dừng tracking;
- yaw gimbal đang cần căn hướng;
- khoảng cách đã nằm trong deadband;
- MQTT không có frame mới quá `0.35 s`.

Manual keyboard luôn thắng toàn bộ lệnh tự động. Khi Manual hết timeout, follow
chỉ tiếp tục nếu dashboard vẫn đang publish frame hợp lệ.

### Dữ liệu hiển thị

Khung tracking và dòng trạng thái web hiển thị:

- khoảng cách đo / khoảng cách mục tiêu;
- nguồn `lidar` hoặc `visual`;
- state `READY`, `FOLLOWING`, `HOLDING`, `ALIGNING`, `BLOCKED`, `LOST`;
- lệnh tiến/lùi chuẩn hóa `F`;
- FPS tracking và FPS nguồn camera như trước.

### Kiểm thử đã chạy

- Python compile và HTML parser thành công.
- Unit test xác nhận bbox nhỏ hơn làm drone tiến, bbox lớn hơn làm drone lùi.
- Unit test xác nhận LiDAR được ưu tiên, lệch yaw dừng tịnh tiến và LOST dừng ngay.
- API test trên UAV-01 disarm xác nhận giá trị mục tiêu `7.5 m` truyền đúng.
- Gazebo bridge đã subscribe camera và `LaserScan` thành công.
- Live MQTT test trên UAV-01 disarm xác nhận:
  `TRACKING_FOLLOW -> MANUAL -> CENTERED`.
- Dashboard và MAVLink bridge đã restart, MQTT/MAVLink đều hoạt động.

Khi thử bay thật, nên bắt đầu ở khoảng trống, đặt mục tiêu `5-8 m`, giữ tay trên
nút Manual/stop và xác nhận chiều tiến/lùi ở vận tốc thấp trước khi tăng
`SWARM_TRACKING_FOLLOW_MAX`.

## 22. Follow Target chuyển sang PX4 Offboard velocity

Backend chính không còn mô phỏng stick tiến/lùi/yaw bằng `MANUAL_CONTROL`.
`tracking_web.py` hợp nhất distance controller và LOS controller thành:

```text
body-frame forward velocity + yaw-rate
```

Dashboard publish MQTT `offboard_follow`. MAVLink bridge stream
`SET_POSITION_TARGET_LOCAL_NED` ở `20 Hz`, frame `MAV_FRAME_BODY_NED`, type mask
`1479`. Bridge prestream 10 frame trước khi yêu cầu `PX4 OFFBOARD`.

Giới hạn mặc định:

```bash
SWARM_TRACKING_OFFBOARD_ENABLED=true
SWARM_TRACKING_OFFBOARD_MAX_FORWARD_M_S=0.4
SWARM_TRACKING_OFFBOARD_FORWARD_SIGN=1
SWARM_TRACKING_OFFBOARD_MAX_YAW_RATE_DEG_S=45
```

Manual vẫn ưu tiên cao nhất. Manual, LOST/OCCLUDED, stop tracking, mất MQTT hoặc
watchdog `0.35 s` đều làm bridge yêu cầu PX4 trở về Position. Backend
`MANUAL_CONTROL` cũ được giữ làm fallback khi tắt `SWARM_TRACKING_OFFBOARD_ENABLED`.

Unit-test xác nhận type mask/body-frame, velocity/yaw-rate, heartbeat prestream,
Offboard mode request và Manual-to-Position override.
