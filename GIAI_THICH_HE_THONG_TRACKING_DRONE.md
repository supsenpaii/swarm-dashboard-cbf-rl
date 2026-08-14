# Giải thích hệ thống tracking, gimbal và điều khiển drone

## 1. Mục tiêu của hệ thống

Hệ thống cho phép chọn một vật thể trực tiếp trên khung camera của UAV, sau đó:

1. Theo dõi vật thể bằng tracker kết hợp Kalman, Memory và ReID.
2. Điều khiển gimbal pitch để giữ vật thể theo phương dọc.
3. Cho gimbal yaw trim trong khoảng `-15°` đến `+15°` để hấp thụ chuyển động nhanh.
4. Điều khiển yaw thân drone để hướng camera và bounding box trở về giữa ảnh.
5. Điều khiển drone tiến/lùi để giữ khoảng cách mục tiêu.
6. Cho phép người điều khiển giành quyền ưu tiên bằng Manual bất kỳ lúc nào.

Hệ thống không điều khiển trực tiếp từng motor. Follow Target gửi body-frame
velocity và yaw-rate qua PX4 Offboard; Manual vẫn dùng MAVLink `MANUAL_CONTROL`.
PX4 chịu trách nhiệm ổn định attitude, altitude và trộn lệnh motor.

## 2. Luồng dữ liệu tổng thể

```text
Gazebo camera topic
        |
        v
raw frame -> Hybrid Tracker -> bbox + score + state
                |                  |
                |                  +-> Kalman / Memory / ReID
                |
                +-> vertical error -> gimbal pitch PID
                |
                +-> horizontal image error -> gimbal yaw trim ±15°
                |
                +-> LOS error -> body yaw controller
                |
                +-> bbox area / LiDAR -> follow distance controller
                                         |
                                         v
Dashboard MQTT -> MAVLink bridge -> PX4 Offboard velocity -> PX4 motor mixer
```

## 3. Các file chính đã sửa

### `main.py`

- Subscribe trực tiếp camera Gazebo.
- Subscribe topic LiDAR Gazebo.
- Publish lệnh roll, pitch và yaw tới gimbal.
- Giới hạn yaw gimbal khi tracking.
- Kiểm tra điều kiện an toàn trước khi gửi body-yaw hoặc follow.
- Hợp nhất body yaw và follow thành MQTT `offboard_follow`.
- Cung cấp API start/stop/select bbox và MJPEG stream cho web.

### `tracking_web.py`

- Quản lý phiên tracking và tracker backend.
- Đọc kết quả bbox, score và state.
- Chạy PID gimbal.
- Tính LOS error cho body yaw.
- Tính khoảng cách và lệnh tiến/lùi.
- Dừng điều khiển khi tracker LOST/OCCLUDED hoặc gặp lỗi.
- Vẽ overlay gồm FPS, score, LOS, góc gimbal, body-yaw và follow.

### `tracking_hybrid.py`

- Kết hợp tracker nhanh với Kalman Filter.
- Lưu appearance memory của mục tiêu.
- Dùng ReID để tìm lại mục tiêu sau che khuất hoặc mất dấu.

### `mavlink_manual_bridge.py`

- Nhận command từ MQTT.
- Quản lý ba state độc lập: Manual, TrackingYaw và TrackingFollow.
- Ghép yaw tự động với lệnh tiến/lùi vào một frame `MANUAL_CONTROL`.
- Áp dụng timeout để tự trả cần điều khiển về giữa.
- Bảo đảm Manual luôn có ưu tiên cao nhất.

### `static/index.html`

- Nút bật/tắt tracking nằm ngoài tab UAV.
- Khung tracking nhận stream trực tiếp từ camera Gazebo.
- Cho phép kéo chuột chọn bbox.
- Cho phép nhập khoảng cách mục tiêu từ `1` đến `20 m`.
- Hiển thị FPS, tracker score, LOS, góc gimbal, body-yaw command và follow state.

## 4. Camera tracking trực tiếp từ Gazebo

Camera UAV-01:

```text
/world/default/model/x500_custom_0/link/camera_link/sensor/camera/image
```

Camera UAV-02:

```text
/world/default/model/x500_custom_1/link/camera_link/sensor/camera/image
```

Dashboard dùng Gazebo Transport Python để nhận `gz.msgs.Image`, chuyển dữ liệu
thành NumPy/OpenCV frame và chỉ encode JPEG cho client đang xem tracking. Cách
này loại bỏ bước đọc camera trung gian, giảm copy frame và giảm độ trễ.

Hai loại FPS được hiển thị:

- `source_fps`: tốc độ frame nhận từ Gazebo.
- `fps`: tốc độ xử lý và xuất frame tracking.

FPS thực tế phụ thuộc tốc độ simulation, tải CPU/GPU, tracker và client MJPEG;
không thể cao hơn ổn định so với tốc độ nguồn camera.

## 5. Chọn và theo dõi mục tiêu

Khi nhấn `TRACKING`, dashboard chuyển sang state `selecting`. Người dùng kéo
chuột trên ảnh để gửi bbox chuẩn hóa về API. Bbox được đổi lại sang pixel và dùng
để khởi tạo tracker.

Các state quan trọng:

```text
SELECTING -> TRACKING -> OCCLUDED -> LOST
                    \-> TRACKING sau khi ReID thành công
```

Chỉ state `TRACKING` với score đủ cao mới được phép điều khiển drone.

## 6. Kalman, Memory và ReID

### Kalman Filter

Kalman dự đoán vị trí và vận tốc bbox giữa các frame. Khi detector/tracker nhiễu
hoặc vật thể bị che ngắn, dự đoán giúp bbox không nhảy đột ngột.

### Appearance Memory

Các đặc trưng ảnh của vật thể được lưu vào memory. Memory không chỉ giữ frame
cuối mà lưu nhiều mẫu appearance để chịu được thay đổi góc nhìn và kích thước.

### ReID

Khi tracker mất mục tiêu, ứng viên mới được so sánh với appearance memory. Ứng
viên đạt similarity và motion gate phù hợp sẽ được dùng để khởi tạo lại tracker.

Thông tin `memory_entries`, `reid_score`, `match_score` và `lost_count` được gửi
lên status web để quan sát.

## 7. PID gimbal chống rung

Gimbal pitch tiếp tục chịu trách nhiệm giữ bbox theo phương dọc. Gimbal yaw chỉ
được dùng làm trim nhanh trong `[-15°, +15°]`.

Thông số mặc định hiện tại:

```bash
SWARM_TRACKING_GIMBAL_YAW_LIMIT_DEG=15
SWARM_TRACKING_GIMBAL_KP=6.5
SWARM_TRACKING_GIMBAL_KI=0.25
SWARM_TRACKING_GIMBAL_KD=0.20
SWARM_TRACKING_GIMBAL_EMA_ALPHA=0.55
SWARM_TRACKING_GIMBAL_DEADBAND_DEG=0.5
SWARM_TRACKING_GIMBAL_OMEGA_MAX_DEG=120
SWARM_TRACKING_GIMBAL_SLEW_MAX_DEG=300
```

Ý nghĩa:

- Giảm `KI` để tránh tích lũy sai số gây rung hoặc overshoot.
- `KD` tạo damping khi bbox đổi hướng nhanh.
- EMA làm mượt góc lỗi trước PID.
- Deadband `0.5°` vẫn giữ bbox sát tâm nhưng tránh phản ứng với nhiễu pixel nhỏ.
- `omega_max` giới hạn tốc độ góc.
- `slew_max` giới hạn tốc độ thay đổi của lệnh góc.

Lệnh gimbal được gửi trực tiếp tới các topic:

```text
/model/x500_custom_0/command/gimbal_roll
/model/x500_custom_0/command/gimbal_pitch
/model/x500_custom_0/command/gimbal_yaw
```

UAV-02 sử dụng cùng cấu trúc với model `x500_custom_1`.

## 8. Điều khiển body yaw bằng LOS

Nếu chỉ dùng sai số bbox, gimbal có thể giữ bbox ở giữa nhưng dừng tại một góc
lệch lớn. Vì vậy body-yaw dùng góc line-of-sight tổng hợp:

```text
image_error = atan2(bbox_center_x - image_center_x, camera_fx)
LOS_error = gimbal_yaw + image_error
```

Trong đó:

- `image_error` là góc mục tiêu so với trục quang học hiện tại.
- `gimbal_yaw` là góc camera so với hướng thân drone.
- `LOS_error` là hướng mục tiêu so với hướng thân drone.

Drone quay theo LOS. Khi drone quay đúng hướng, gimbal PID phải quay ngược lại để
tiếp tục giữ mục tiêu; vì vậy góc gimbal tự tiến dần về `0°`. Hệ thống đạt đồng
thời hai mục tiêu:

1. Bbox nằm gần tâm ảnh.
2. Hướng thân drone gần trùng hướng mục tiêu.

Body-yaw dùng hysteresis:

```bash
SWARM_TRACKING_BODY_YAW_ENTER_DEG=2.5
SWARM_TRACKING_BODY_YAW_EXIT_DEG=1.0
```

- Khi `|LOS| >= 0.8°`, bộ điều khiển bắt đầu yaw.
- Khi `|LOS| <= 0.25°`, bộ điều khiển dừng.
- Hai ngưỡng khác nhau tránh bật/tắt liên tục quanh tâm.

Luật điều khiển:

```text
direction = sign(filtered_LOS)
target = direction * min(max_command, Kp * (abs(filtered_LOS) - exit_deadband))
```

Thông số tốc độ:

```bash
SWARM_TRACKING_BODY_YAW_KP=0.018
SWARM_TRACKING_BODY_YAW_MAX=0.55
SWARM_TRACKING_BODY_YAW_SLEW=1.4
SWARM_TRACKING_BODY_YAW_EMA_ALPHA=0.22
SWARM_TRACKING_BODY_YAW_MIN_SCORE=0.35
```

Nếu drone quay ngược chiều cần thiết:

```bash
SWARM_TRACKING_BODY_YAW_INVERT=true
```

## 9. Scale yaw tự động và Manual

Trước khi chuyển sang Offboard, tracking yaw và Manual đã dùng scale riêng:

```text
Tracking: max command 0.55, scale 600, r tối đa 330
Manual:   input theo web, scale 400
```

Đây hiện là đường fallback nếu đặt `SWARM_TRACKING_OFFBOARD_ENABLED=false`.
Live test fallback khi UAV disarm xác nhận:

```text
TRACKING_YAW yaw=0.55 -> r=330
MANUAL yaw=0.55       -> r=220
CENTERED              -> r=0
```

## 10. Giữ khoảng cách mục tiêu

Khoảng cách ưu tiên lấy từ topic:

```text
/x500_custom/front_lidar
```

Topic có nhiều publisher. Dashboard dùng trường `frame` để ánh xạ scan về
`x500_custom_0` hoặc `x500_custom_1`. Giá trị đo là median các tia hợp lệ quanh
hướng yaw gimbal.

Nếu không có LiDAR hợp lệ, hệ thống ước lượng từ diện tích bbox:

```text
distance = reference_distance * sqrt(reference_area / current_area)
```

Sai số khoảng cách:

```text
distance_error = measured_distance - target_distance
```

- Sai số dương: mục tiêu xa, drone tiến.
- Sai số âm: mục tiêu quá gần, drone lùi.
- Trong deadband: drone giữ vị trí.

Thông số:

```bash
SWARM_TRACKING_FOLLOW_KP=0.12
SWARM_TRACKING_FOLLOW_MAX=0.30
SWARM_TRACKING_FOLLOW_SLEW=0.60
SWARM_TRACKING_FOLLOW_DEADBAND_M=0.40
SWARM_TRACKING_FOLLOW_ALIGN_YAW_DEG=10
SWARM_TRACKING_FOLLOW_EMA_ALPHA=0.35
SWARM_TRACKING_FOLLOW_MIN_SCORE=0.70
SWARM_TRACKING_OFFBOARD_ENABLED=true
SWARM_TRACKING_OFFBOARD_MAX_FORWARD_M_S=0.4
SWARM_TRACKING_OFFBOARD_FORWARD_SIGN=1
SWARM_TRACKING_OFFBOARD_MAX_YAW_RATE_DEG_S=45
```

Drone chỉ tiến/lùi khi `|LOS| <= 5°`. Nếu chưa căn hướng, follow chuyển sang
`ALIGNING` và gửi lệnh tiến/lùi bằng `0`.

## 11. Sửa lỗi follow không di chuyển ở backend cũ

Ban đầu follow dùng chung horizontal scale `500`. Ví dụ `F=0.15` chỉ tạo
`x=75/1000`, có thể nằm trong deadzone Position mode của PX4.

Backend `MANUAL_CONTROL` fallback dùng:

```text
TRACKING_FOLLOW_HORIZONTAL_SCALE=1000
```

Do đó:

```text
F=0.15 -> x=150
```

Manual vẫn dùng scale `500`, nên không bị tăng độ nhạy.

## 12. MQTT message Offboard hiện tại

```json
{
  "type": "offboard_follow",
  "drone_id": "UAV-01",
  "enabled": true,
  "forward_velocity_m_s": 0.6,
  "right_velocity_m_s": 0.0,
  "down_velocity_m_s": 0.0,
  "yaw_rate_deg_s": 25.0,
  "measured_distance_m": 7.0,
  "target_distance_m": 5.0,
  "distance_source": "lidar",
  "track_state": "tracking"
}
```

## 13. PX4 Offboard velocity bridge

Bridge gửi `SET_POSITION_TARGET_LOCAL_NED` với frame `MAV_FRAME_BODY_NED`:

```text
vx = vận tốc tiến/lùi theo hướng thân drone
vy = vận tốc ngang, hiện bằng 0
vz = 0, PX4 giữ độ cao
yaw_rate = tốc độ yaw từ LOS controller
```

Type mask `1479` bỏ qua position, acceleration và yaw angle; velocity cùng
yaw-rate vẫn được sử dụng. Setpoint được stream ở `20 Hz`. Bridge gửi ít nhất
10 frame trước khi yêu cầu PX4 chuyển sang Offboard.

Thứ tự ưu tiên:

```text
MANUAL > OFFBOARD_FOLLOW > legacy tracking fallback > CENTERED
```

Khi Manual active, bridge yêu cầu PX4 trở về Position trước khi gửi
`MANUAL_CONTROL`. Khi Offboard timeout, LOST hoặc stop tracking, bridge cũng yêu
cầu Position và xóa bộ đếm prestream.

## 14. Watchdog và điều kiện an toàn

Bridge timeout sau `0.35 s`. Nếu mất MQTT hoặc dashboard ngừng gửi command,
Offboard bị disable và PX4 được yêu cầu trở về Position.

```text
vx=0, vy=0, vz=0, yaw_rate=0
```

Body-yaw và follow chỉ được phép khi:

- UAV online.
- UAV đã ARM.
- PX4 ở Position hoặc Offboard mode, `nav_state=2/14`.
- Không có failsafe.
- Manual-control signal không bị lost.
- Local altitude ít nhất `0.5 m`.
- Tracker ở `TRACKING` và score đạt ngưỡng.
- Gimbal command thành công.

Khi `LOST`, `OCCLUDED`, lỗi gimbal, stop tracking hoặc mất MQTT, setpoint bị
disable và watchdog yêu cầu PX4 trở về Position.

## 15. Trạng thái hiển thị trên web

Web hiển thị:

- `LOS`: góc mục tiêu so với thân drone.
- `GIMBAL`: yaw hiện tại của gimbal.
- `CMD`: lệnh yaw chuẩn hóa.
- `YAW ±15°`: giới hạn yaw trim.
- `EDGE`: PID gimbal đang yêu cầu vượt giới hạn trim.
- `FOLLOW`: `IDLE`, `ALIGNING`, `FOLLOWING`, `HOLDING`, `BLOCKED` hoặc `LOST`.
- `OFFBOARD`: state, forward velocity `V` và yaw-rate `YR`.
- Khoảng cách đo, khoảng cách mục tiêu và nguồn `lidar`/`visual`.
- Tracking FPS và source FPS.

## 16. Quy trình sử dụng

1. Khởi động PX4, Gazebo, MQTT gateway, dashboard và MAVLink bridge.
2. Kiểm tra cả UAV online trên web.
3. Chuyển UAV cần test sang Position mode.
4. ARM và takeoff tới vùng trống an toàn.
5. Nhập khoảng cách bám mục tiêu.
6. Nhấn `FOLLOW TARGET`.
7. Kéo bbox quanh vật thể.
8. Bridge prestream setpoint rồi tự chuyển PX4 sang Offboard.
9. Quan sát `score`, `LOS`, `GIMBAL`, `OFFBOARD`, velocity và FPS.
10. Nếu có hành vi bất thường, dùng Manual hoặc dừng tracking ngay.

## 17. Quy trình tune an toàn

Mỗi lần chỉ nên đổi một nhóm tham số:

### Nếu yaw drone quá chậm

- Tăng nhẹ `SWARM_TRACKING_BODY_YAW_KP`.
- Sau đó mới tăng `SWARM_TRACKING_BODY_YAW_SLEW`.

## Sửa lỗi drone không tiến khi Follow Target

Kết quả kiểm tra live cho thấy PX4 đã vào Offboard nhưng lệnh yaw thường xuyên
đảo giữa `-45` và `+45 deg/s`. Gimbal đồng thời chạm hai biên `-15/+15 deg`,
làm sai số hướng tổng liên tục vượt ngưỡng và vận tốc tiến bị đặt về `0`.

Các thay đổi khắc phục:

- Giảm gain yaw và tăng lọc để tránh drone quay vượt mục tiêu rồi đảo chiều.
- Dùng sai số tâm bbox trên ảnh để quyết định có được tiến hay không. Góc gimbal
  vẫn được dùng để drone từ từ xoay thân và đưa yaw gimbal về gần `0`.
- Cho phép tiếp tục phát setpoint khi PX4 fallback sang Hold (`nav_state=4`) để
  bridge có thể prestream và tự đưa PX4 trở lại Offboard.
- Tăng ngưỡng căn ảnh lên `10 deg`; bbox lệch lớn hơn ngưỡng này thì drone chỉ
  quay, bbox nằm trong ngưỡng thì drone được phép vừa quay vừa tiến.

Backend phải được restart sau khi cả UAV đã disarm thì thay đổi mới có hiệu lực.

### Sửa vòng lặp Offboard do mất manual input

Trong chuyến thử live, PX4 báo `manual_control_signal_lost` sau khi bridge chuyển
sang Offboard. Backend dừng `offboard_follow`, bridge trả về Position, tín hiệu
manual xuất hiện lại rồi bridge lại xin Offboard. Chu kỳ này lặp khoảng mỗi
`1-1.5 s`, khiến vận tốc tiến không được duy trì đủ lâu.

Bridge hiện gửi thêm một gói `MANUAL_CONTROL` trung tính ở mỗi chu kỳ Offboard,
song song với `SET_POSITION_TARGET_LOCAL_NED`. Gói trung tính chỉ giữ heartbeat
manual cho PX4, còn chuyển động vẫn lấy từ velocity setpoint Offboard. Lệnh bàn
phím thật vẫn có độ ưu tiên cao hơn và sẽ đưa PX4 về Position như trước.

### Sửa chiều tiến bị ngược với camera

Log chuyến thử cho thấy hệ thống chỉ phát `vx` dương nhưng drone lại rời xa vật
thể trong ảnh. Bbox vì vậy nhỏ dần, bộ ước lượng visual cho rằng khoảng cách tăng
và tiếp tục tăng `vx` dương, tạo thành phản hồi dương nguy hiểm.

Kiểm tra live sau đó xác nhận trục camera hiện tại trùng với `+X body`; dấu `-1`
làm drone đạt gần `-1 m/s` theo trục thân và lùi khỏi vật thể. Cấu hình đúng là
`SWARM_TRACKING_OFFBOARD_FORWARD_SIGN=1`.

Nguyên nhân gốc là bbox do tracker trả về nhỏ hơn nhiều bbox người dùng chọn.
Visual distance lập tức nhảy lên `11.78 m`, dù score chỉ còn `0.533`, rồi yêu cầu
vận tốc tối đa. Bản sửa mới hiệu chuẩn reference area bằng bbox đầu tiên do
tracker trả về, yêu cầu score tối thiểu `0.70` cho chuyển động và giới hạn tốc độ
tiến mặc định ở `0.4 m/s`.
- Chỉ tăng `BODY_YAW_MAX` nếu lệnh thường xuyên chạm giới hạn nhưng drone vẫn chậm.

### Nếu yaw drone overshoot hoặc rung

- Giảm `BODY_YAW_KP`.
- Giảm `BODY_YAW_SLEW`.
- Giảm `BODY_YAW_EMA_ALPHA` để lọc mạnh hơn.
- Tăng nhẹ `BODY_YAW_EXIT_DEG`.

### Nếu camera rung

- Giảm `GIMBAL_KP` hoặc `GIMBAL_SLEW_MAX_DEG`.
- Giảm `GIMBAL_EMA_ALPHA`.
- Tăng nhẹ `GIMBAL_DEADBAND_DEG`.
- Không tăng đồng thời Kp, omega và slew.

### Nếu bbox lệch tâm lâu

- Kiểm tra đúng chiều yaw trước.
- Giảm deadband gimbal từng bước nhỏ.
- Kiểm tra FPS nguồn và tracking score.
- Kiểm tra gimbal có thường xuyên nằm ở `±15°` hay không.

## 18. Các kiểm thử đã thực hiện

- Python compile thành công cho `main.py`, `tracking_web.py` và bridge.
- HTML parser thành công cho giao diện web.
- Test clamp gimbal yaw `-15°` và `+15°`.
- Test mục tiêu bên trái tạo yaw âm, bên phải tạo yaw dương.
- Test bbox giữa nhưng gimbal `+10°` vẫn tạo body yaw để recenter.
- Test `gimbal +10°` và image error `-10°` tạo LOS gần `0°` và không yaw.
- Test follow dừng ở `ALIGNING` khi LOS lớn hơn ngưỡng.
- Test LOST/OCCLUDED dừng lệnh.
- Test follow xa tiến, quá gần lùi.
- Test LiDAR được ưu tiên, visual bbox là fallback.
- Live MQTT test xác nhận Manual ưu tiên hơn tracking.
- Live MQTT test xác nhận watchdog trả `CENTERED`.
- Live test xác nhận tracking yaw `0.55 -> r=330` và Manual `0.55 -> r=220`.

## 19. Dịch vụ hiện tại

Kiểm tra dashboard:

```bash
systemctl --user status swarm-dashboard.service
```

Kiểm tra MAVLink bridge:

```bash
systemctl --user status swarm-mavlink-manual.service
```

Health API:

```bash
curl http://127.0.0.1:8000/health
```

Hai service hiện được tạo dưới dạng user transient service. Nếu reboot máy, cần
khởi động lại theo launcher/quy trình đang sử dụng hoặc tạo unit file cố định.

## 20. Hạn chế hiện tại

- LiDAR có thể đo vật cản cùng hướng thay vì đúng vật thể tracking.
- Ước lượng khoảng cách từ bbox chỉ đáng tin theo tỷ lệ tương đối.
- Gimbal angle hiện dựa trên command đã publish, chưa có joint encoder feedback
  độc lập ổn định.
- Tracking FPS phụ thuộc simulation rate và tải máy.
- Body yaw bằng `MANUAL_CONTROL` phù hợp thử nghiệm hiện tại nhưng Offboard yaw
  rate/velocity setpoint sẽ phù hợp hơn nếu cần điều khiển quỹ đạo chính xác.
- Cần thử nghiệm trong không gian trống trước khi tăng thêm gain hoặc tốc độ.

## 21. Nguyên tắc an toàn khi thay đổi code

Không restart dashboard hoặc MAVLink bridge khi UAV đang ARM và bay. Quy trình
đã áp dụng là:

1. Dừng tracking.
2. Hạ cánh và disarm cả hai UAV.
3. Compile và unit-test code.
4. Restart service.
5. Home gimbal.
6. Test MQTT/API khi UAV vẫn disarm.
7. Chỉ sau đó mới thử bay lại.
