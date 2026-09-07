# Body attitude recenter theo gimbal

## Mục tiêu

Khi gimbal tracking lệch khỏi Home, drone dùng horizontal acceleration và yaw-rate
để bám theo hướng nhìn. PX4 biến acceleration thành roll/pitch; gimbal counter-rotate
nên góc gimbal tương đối với thân được kéo dần về Home.

Pitch/roll của drone tạo gia tốc ngang, vì vậy dashboard giới hạn tilt, acceleration,
jerk và tốc độ thân.
PX4 tự sinh attitude và thrust. Dashboard chụp độ cao lúc tracking bắt đầu và giữ
tọa độ Local-NED `z` tuyệt đối; không gửi raw thrust trong tracking.

## Luồng điều khiển

1. Gazebo IMU cung cấp quaternion của `base_link` và `camera_link`.
2. Dashboard tính quaternion tương đối để lấy roll/pitch/yaw thực của gimbal.
3. Vòng ngoài tính sai số gimbal và sinh desired tilt/yaw-rate có adaptive gain.
4. Desired roll/pitch được đổi thành acceleration thân rồi xoay sang North/East.
   Follow-distance được cộng vào forward acceleration.
5. MQTT gửi `offboard_follow` sang MAVLink bridge.
6. Bridge phát `SET_POSITION_TARGET_LOCAL_NED`: giữ `z` tuyệt đối, dùng `ax/ay`
   theo North/East và yaw-rate; PX4 tự điều khiển thrust và attitude.
7. Tracking gián đoạn dưới 0,75 giây dùng acceleration trung tính, không đổi mode.
   Mất lâu hơn mới yêu cầu Position.
8. Sai số độ cao từ 0,25 m tạm dừng acceleration ngang; từ 0,50 m thoát Offboard.

## Kích hoạt

Tính năng mặc định **tắt**. Chỉ kích hoạt sau khi tất cả drone đã disarm:

```bash
export SWARM_TRACKING_BODY_ATTITUDE_ENABLED=true
```

Sau đó restart cả dashboard và `mavlink_manual_bridge.py` trong cùng môi trường có
biến trên. Không restart khi drone đang armed.

Các giới hạn mặc định quan trọng:

- Góc nghiêng tối đa: `5 deg`
- Roll/pitch rate tối đa: `5 deg/s`
- Yaw rate tối đa: `15 deg/s`
- Deadband gimbal: `2 deg`
- Hover thrust ban đầu: `0.50`
- Watchdog lệnh attitude tại bridge: `0.35 s`
- Acceleration ngang tối đa: `0.60 m/s²`
- Jerk ngang tối đa: `1.0 m/s³`
- Altitude guard/exit: `0.25/0.50 m`

Lần kích hoạt đầu sau thay đổi acceleration phải dùng cấu hình staged:

```bash
export SWARM_BODY_RECENTER_MAX_YAW_RATE_DEG_S=8
export SWARM_BODY_RECENTER_YAW_KP=0.50
export SWARM_BODY_RECENTER_YAW_KP_FAR=0.75
export SWARM_BODY_RECENTER_MAX_TILT_DEG=2
export SWARM_BODY_RECENTER_ROLL_KP=0
export SWARM_BODY_RECENTER_PITCH_KP=0
export SWARM_BODY_RECENTER_MAX_HORIZONTAL_ACCEL_M_S2=0.35
export SWARM_BODY_RECENTER_ACCEL_JERK_M_S3=0.60
```

Sau khi yaw-only đạt mới đặt pitch Kp `0.15`; sau đó roll Kp `0.15`. Không mở hai
trục cùng lúc trong lần thử đầu.

## Trình tự thử an toàn

1. SITL, propeller-free hoặc khu vực thử kín; xác minh dấu từng trục khi disarm.
2. Cho yaw-only với giới hạn nhỏ, xác minh gimbal tiến về Home đúng chiều.
3. Cho pitch rồi roll riêng lẻ, góc nghiêng tối đa 2 độ trước khi dùng mặc định.
4. Hover thấp, kiểm tra giữ độ cao và watchdog khi chủ động dừng tracking.
5. Mới bật follow-distance; kiểm tra target lost và chuyển Position.

Không coi unit test là xác nhận flight safety. Cần flight test theo từng bước trước
khi dùng với drone thật hoặc tăng các giới hạn.
