# Sparrow thật — các bước dựng model, AutoTune và Follow

Ngày chốt tài liệu: 2026-08-13 (UTC+7)

Lệnh chạy chuẩn:

```bash
cd ~/px4_ros2_ws
./sparrow.sh
```

## 1. Chọn kiến trúc đúng

- Dùng Sparrow thật với airframe PX4 `4020` và Gazebo model `sparrow_gimbal`.
- Không dùng x500 làm physics rồi chỉ phủ vỏ Sparrow.
- Giữ x500 và lệnh chạy x500 riêng, không dùng bộ tune Sparrow cho x500.
- Chọn một nguồn model duy nhất trong `models/` để tránh Gazebo nạp nhầm bản cũ.

## 2. Chuẩn hóa hệ trục và hướng thân

- Gazebo dùng FLU; PX4 control allocation dùng FRD.
- CAD Sparrow có mũi theo `-Y`, nên xoay CAD `Rz(pi/2)` để mũi, camera trung tâm và hướng tiến PX4 cùng là `+X`.
- Không xoay toàn model ở pose gốc vì sẽ làm lệnh tiến/trái bị hoán đổi.
- Sau sửa, lệnh tiến bay theo mũi và lệnh trái bay sang trái đúng hình học.

## 3. Đặt đúng bốn rotor

Tọa độ rotor Gazebo FLU quanh CG:

| Motor | Vị trí `(x, y, z)` m | Chiều quay | Vị trí |
|---|---:|---|---|
| M1 / motor 0 | `(0.17140, -0.21837, -0.03325)` | CCW | trước-phải |
| M2 / motor 1 | `(-0.17140, 0.19143, 0.03325)` | CCW | sau-trái |
| M3 / motor 2 | `(0.17140, 0.21837, -0.03325)` | CW | trước-trái |
| M4 / motor 3 | `(-0.17140, -0.19143, 0.03325)` | CW | sau-phải |

- Chuyển các vị trí trên sang FRD cho `CA_ROTOR0..3` trong airframe `4020`.
- Giữ mapping PX4 output 1..4 tương ứng motor 0..3.
- Dùng `KM=+0.016` cho CCW và `KM=-0.016` cho CW, khớp motor plugin.
- Dùng `maxRotVelocity=1400`; kiểm tra từng motor trước khi cất cánh.

## 4. Dựng khối lượng, CG và quán tính

- Bỏ dữ liệu URDF SolidWorks 49.616 kg vì COM nằm ngoài mesh và không phải số all-up SI đáng tin.
- Mô hình SITL hiện dùng base + rotor 2.70 kg, gimbal 0.42 kg, tổng 3.12 kg.
- Chọn model origin tại CG khi gimbal home và bù inertial của base để composite CG về gốc.
- Các số này là provisional cho mô phỏng; khi có phần cứng phải cân lại mass, CG, inertia, kT và RPM.

## 5. Làm gimbal Sparrow đúng ngoại hình và đúng cơ cấu

- Exterior dùng đúng `P.STL` Sparrow.
- Bên trong dùng chuỗi CGO3 ẩn, ba trục đồng tâm: yaw → roll → pitch.
- Giữ contract tracker: yaw `(0,0,-1)`, roll `(1,0,0)`, pitch `(0,-1,0)`; pitch âm là nhìn xuống.
- Giữ ba topic:
  - `/model/sparrow_gimbal_0/command/gimbal_yaw`
  - `/model/sparrow_gimbal_0/command/gimbal_roll`
  - `/model/sparrow_gimbal_0/command/gimbal_pitch`
- Camera ở optical center, 640×360 khoảng 30 Hz; camera IMU 250 Hz.
- Dùng fixed joint `GimbalAttachJoint` và adapter visual để gimbal dính đúng vào thân.

## 6. Sửa hướng camera, vị trí mount và mặt đất

- Pose P.STL dùng composition URDF: roll 90°, yaw xấp xỉ 2° sau transform thân.
- Bỏ offset hình ảnh giả `x=-0.06 m` để vỏ P nằm đúng quanh optical center.
- Do `sparrow_base` đặt `base_link` ở model z=0.10 m, include gimbal phải dùng z=0.036158 m để mount vẫn cách base `-0.063842 m`.
- Căn collision thân có đáy đúng `-0.09075 m`, trùng đáy CAD; khi Gazebo settle, mesh không còn lún dưới đất.

## 7. Kiểm tra model trước khi tune

- Validate SDF cho `sparrow_base`, `sparrow`, `sparrow_gimbal_core`, `sparrow_gimbal`.
- Step riêng yaw, roll, pitch; xác nhận dấu trục, camera không bị che và không cross-coupling.
- Hover thấp không gió, kiểm tra bốn motor cân, allocator đạt và không rail/failsafe.
- Không dùng AutoTune để che lỗi geometry, CG, motor order hoặc dấu rotor.

## 8. Thực hành AutoTune thủ công bằng ba terminal

Mục tiêu của bài này là hiểu từng khối riêng. Không chạy `./sparrow.sh` cùng lúc,
không bật tracker, Follow hoặc wind trong lúc AutoTune.

### 8.1. AutoTune chỉnh gì?

- Một lần AutoTune tự chạy đủ `ROLL → PITCH → YAW`; không chạy ba lần riêng.
- Nó chỉnh vòng attitude/rate của thân drone: `MC_ROLL*`, `MC_PITCH*`, `MC_YAW*`.
- Nó không chỉnh độ cao Z, Takeoff, Follow Target hay PID gimbal. Các phần đó phải kiểm tra riêng sau khi thân bay ổn.

### 8.2. Dọn phiên cũ

Đóng phiên `./sparrow.sh` cũ bằng `Ctrl+C`. Không được để hai PX4 hoặc hai Gazebo
cùng chạy. Kiểm tra:

```bash
pgrep -af "px4_sitl_default/bin/px4|MicroXRCEAgent|gz sim|QGroundControl"
```

Nếu còn tiến trình của phiên cũ, đóng đúng terminal của nó trước khi tiếp tục.

### 8.3. Terminal 1 — kết nối PX4 với ROS 2

```bash
MicroXRCEAgent udp4 -p 8888
```

Giữ terminal này mở. Đây là cầu DDS cho `/fmu/...`; QGroundControl dùng MAVLink
riêng nên vẫn có thể kết nối nếu ROS 2 agent chưa chạy.

### 8.4. Terminal 2 — spawn đúng Sparrow và mở Gazebo

```bash
cd ~/PX4-Autopilot
export PX4_GZ_WORLD=default
export GZ_SIM_RESOURCE_PATH="$HOME/px4_ros2_ws/models:$HOME/.simulation-gazebo/models:$HOME/.simulation-gazebo/worlds:$PWD/Tools/simulation/gz/models:$PWD/Tools/simulation/gz/worlds"
unset HEADLESS PX4_GZ_MODEL_NAME
PX4_SYS_AUTOSTART=4020 \
PX4_SIM_MODEL=sparrow_gimbal \
PX4_GZ_MODEL_POSE="0,0,0,0,0,0" \
./build/px4_sitl_default/bin/px4 -i 0
```

PX4 tự mở Gazebo GUI vì `HEADLESS` đã được bỏ. Giữ terminal PX4 mở để có thể
dùng các lệnh `param show` và `param save` sau khi tune.

### 8.5. Terminal 3 — mở QGroundControl

```bash
~/Applications/QGC-v4.4.3.AppImage
```

Đợi QGC tự nhận vehicle `MAV_SYS_ID=1` và tải xong parameter. Không tạo thêm
UDP link thủ công; PX4 SITL đã phát MAVLink tới QGC.

### 8.6. Chuẩn bị parameter trước khi bay

Trong QGC vào **Vehicle Setup → Parameters**, tìm và kiểm tra:

| Parameter | Giá trị | Ý nghĩa |
|---|---:|---|
| `MC_AT_SYSID_AMP` | `0.5` | Biên độ kích thích đủ cho Sparrow |
| `MC_AT_APPLY` | `1` | Chỉ áp dụng gain mới sau khi disarm |
| `MC_AIRMODE` | `1` | Roll/Pitch Airmode; tránh cảnh báo arm bằng yaw stick |
| `MPC_THR_HOVER` | `0.64` | Hover thrust đã đo của Sparrow |

Không sửa gain Roll/Pitch/Yaw bằng tay trước AutoTune. Nếu QGC báo Yaw Airmode
xung đột với arm bằng cần yaw và tự hạ `MC_AIRMODE` về `1`, đó là cảnh báo cấu
hình arm, không phải AutoTune thất bại.

### 8.7. Cất cánh và bắt đầu AutoTune

1. Trong QGC, Arm rồi Takeoff lên khoảng `5–10 m`.
2. Chuyển sang **Position** và chờ hover ổn định ít nhất `3–5 s`.
3. Không chạy Follow, không điều khiển gimbal và không tạo gió.
4. Vào **Vehicle Setup → PID Tuning** (QGC 4.4 có thể ghi ngắn là **Tuning**).
5. Chọn tab **Rate Controller** hoặc **Attitude Controller**.
6. Bật **Autotune enabled**, bấm **Autotune** và xác nhận cảnh báo.
7. Không chạm joystick/setpoint trong lúc PX4 tự rung lần lượt Roll, Pitch, Yaw.

Trạng thái đúng:

```text
INIT → ROLL → PITCH → YAW → VERIFICATION → APPLY → WAIT_FOR_DISARM
```

Quá trình thường mất khoảng `20–70 s`. Nếu drone dao động tăng mạnh hoặc không
giữ được vị trí an toàn, dừng AutoTune, Land và Disarm; không lưu kết quả đó.
Đây là bài thực hành thủ công nên nút QGC bỏ qua safety gate của
`scripts/sparrow_autotune.py`; chỉ bấm sau khi chính bạn đã xác nhận hover ổn định.

### 8.8. Kết thúc và lưu gain

1. Khi QGC báo hoàn tất/đợi disarm, bấm **Land**.
2. Chờ drone chạm đất rồi **Disarm**. Với `MC_AT_APPLY=1`, chính bước này áp dụng gain mới.
3. Trong terminal PX4 chạy:

```text
param save
param show MC_ROLLRATE_P
param show MC_PITCHRATE_P
param show MC_YAWRATE_P
```

4. Trong QGC vào **Vehicle Setup → Parameters → Tools → Save to file** và lưu
   một bản dễ đọc, ví dụ:

```text
~/px4_ros2_ws/config/sparrow_autotune_2026-08-13.params
```

### 8.9. PX4 lưu ở đâu?

- Bản đang chạy của SITL instance 0 được PX4 lưu tự động tại:
  `~/PX4-Autopilot/build/px4_sitl_default/rootfs/0/parameters.bson`.
- Bản dự phòng tự động nằm cạnh đó: `parameters_backup.bson`.
- File `.params` xuất từ QGC là bản sao lưu thủ công, dễ xem và khôi phục.
- AutoTune không tự sửa airframe trong Git. Mặc định lâu dài của Sparrow nằm ở
  `patches/px4-airframes/4020_gz_sparrow_gimbal` và bản PX4 đang build nằm ở
  `~/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/4020_gz_sparrow_gimbal`.

Chỉ chép gain đã kiểm chứng vào airframe sau khi reboot và hover thử lại. Nếu
xóa `parameters.bson` hoặc reset toàn bộ parameter, PX4 sẽ quay về mặc định
airframe chứ không đọc file `.params` của QGC tự động.

### 8.10. Xác nhận sau khi lưu

1. Land, Disarm và đóng ba terminal.
2. Mở lại đúng ba terminal theo các bước 8.3–8.5.
3. Trong QGC kiểm tra các gain vẫn giữ nguyên.
4. Takeoff thấp `3–5 m`, hover và thử lệnh Roll/Pitch/Yaw nhỏ.
5. Chỉ khi thân ổn định mới chạy lại `./sparrow.sh` để tune Takeoff, Follow và gimbal.

Lượt amplitude `0.3` trước đây không đủ kích thích Roll; baseline đã xác nhận là
`MC_AT_SYSID_AMP=0.5`, `MC_AT_APPLY=1`.

Gain chốt hiện được ghi trong airframe `4020`:

| Trục | Rate P | Rate I | Rate D | Attitude P |
|---|---:|---:|---:|---:|
| Roll | 0.167565 | 0.225351 | 0.003103 | 5.698846 |
| Pitch | 0.169707 | 0.228323 | 0.003174 | 5.626944 |
| Yaw | 0.172427 | 0.180053 | 0.003857 | 5.538166 |

## 9. Kết quả hover đã xác nhận

- ULog `06_52_14.ulg`: hover 56.7 s.
- Roll RMS 0.190°, pitch RMS 0.207°.
- Roll tối đa 0.543°, pitch tối đa 0.566°.
- Bốn motor trung bình 0.636; allocator torque/thrust 100%; không failure.

## 10. Tune Follow riêng cho Sparrow

Log Sparrow `07_10_40.ulg` cho thấy lúc target bắt đầu chạy, speed-scale ghi lại tham số PX4 liên tục và tăng `FLW_TGT_MAX_VEL` từ 0.95 lên 10.816 m/s trong khoảng 7 giây. Quyền điều khiển tăng quá nhanh làm nhiễu hướng target từ RGB biến thành lạng trái/phải.

Baseline Sparrow mới:

- Giữ `FLW_TGT_MAX_VEL=0.95 m/s`.
- Tắt speed-scale riêng cho Sparrow; không đổi parameter dynamics giữa chuyến bay.
- Đặt `FLW_TGT_RS=0.55` để estimator PX4 lọc target mạnh hơn cấu hình x500 0.40.
- Giới hạn course của Kalman ở 12°/s thay vì 20°/s để offset Follow 15 m không quét ngang theo nhiễu bbox.
- Giữ nguyên gain AutoTune, khoảng cách 15 m, độ cao 30 m, gimbal và toàn bộ cấu hình x500.

Các knob chỉ dành cho Sparrow nằm trong `sparrow.sh`:

```bash
SPARROW_FOLLOW_MAX_VEL=0.95
SPARROW_FOLLOW_RS=0.55
SPARROW_FOLLOW_COURSE_RATE_DEG_S=12.0
```

Sau khi chuyến 0.95 được xác nhận sạch bằng ULog, mới tăng tốc từng nấc 0.10–0.15 m/s. Mỗi nấc phải kiểm tra roll/pitch RMS, số lần đổi hướng vận tốc, sai số khoảng cách 15 m và khung hình gimbal. Không bật lại speed-scale 0.95→10.8 trong một chuyến.

## 11. File nguồn quan trọng

- Entrypoint: `sparrow.sh`.
- Launcher Follow chung: `CHAY_PX4_TRACKING_XE_HOI_1_TERMINAL.sh`.
- Sparrow physics: `models/sparrow_base`, `models/sparrow`.
- Gimbal: `models/sparrow_gimbal_core`, `models/sparrow_gimbal`.
- Airframe/tune: `patches/px4-airframes/4020_gz_sparrow_gimbal`.
- Test contract: `scripts/tests/unit/test_sparrow_model.py`.

## 12. Trình tự test tiếp theo

1. Chạy `./sparrow.sh`, cất cánh 30 m và chọn bbox xe đứng yên.
2. Xác nhận chuyển `AUTO_FOLLOW_TARGET` một lần, tiếp cận 15 m không lạng.
3. Cho xe chạy chậm và cua; xác nhận `FLW_TGT_MAX_VEL` vẫn cố định 0.95.
4. Kiểm tra video, khoảng cách, roll/pitch và ULog.
5. Nếu đạt, tăng `SPARROW_FOLLOW_MAX_VEL` lên nấc kế tiếp và tune đồng bộ sau khi đọc ULog; không thay x500.
