# RGB-only Follow validation — 2026-07-30

## Kết luận

Trạng thái: **chưa hoàn thành end-to-end** theo tiêu chí continuation prompt.

Đã chứng minh trong SITL:

- pointing ổn định 35–48 FPS, tracker score thường 0.94–1.00;
- explicit user authorization và lateral RGB bootstrap;
- `TARGET_3D_READY` với covariance/range gate hợp lệ;
- D0 runtime từ immutable selection pose;
- target prestream đủ 6 message trước mode request;
- PX4 thực sự nhận native Follow, telemetry quan sát `nav_state=19`;
- target/estimate loss fail-close về Position/Hold, không giữ Offboard.

Chưa chứng minh:

- native Follow bền đủ lâu để di chuyển target rồi đo convergence về D0;
- toàn bộ fault suite runtime, đặc biệt manual override/failsafe/mode reject;
- moving-target distance hold không có authority conflict.

## Snapshot SITL chính

Phiên session 12:

```text
selection pose valid:           true
selection pose age:             29.15 ms
selection gimbal age:           21.48 ms
RGB bootstrap travel:           0.381 m
D0 horizontal:                  7.007090553 m
D0 slant:                       7.102044550 m
prestream count:                6
mode request attempts:          1
PX4 nav_state observed:         19
fail-close reason:              no_bbox
post-fault PX4 nav_state:       2 (Position)
```

Phiên session 14:

```text
RGB bootstrap travel:           0.478 m
D0 horizontal:                  9.480177950 m
D0 slant:                       9.491708543 m
abort reason:                   target_estimate_invalid_dt
last invalid prediction dt:     1.983505656 s
retained geometry reason:       insufficient_baseline
```

Các phiên không đủ chất lượng dừng đúng tại `bootstrap_reprojection_error`;
gate `SWARM_BEARING_MAX_REPROJECTION_PX=3.0` không bị nới.

## Root cause/runtime observations

- PX4/Gazebo liên tục báo timesync reset; follower/target có giai đoạn trôi
  cao độ, làm tăng reprojection error khi bootstrap.
- Polling `/api/drones` ở 5 Hz tạo lock/GIL contention và có thể làm frame xử
  lý bị gap gần 2 s dù camera source vẫn khoảng 30–35 FPS. Validation production
  cần telemetry trace riêng thay vì polling status object lớn.
- Tracker mục tiêu nhỏ có thể mất khi PX4 chuyển mode/yaw; boundary dừng ngay
  và không tự tái chiếm motion authority.
- Không có bằng chứng cho giả thuyết đảo trục camera. So sánh đúng trong cùng
  world/global frame cho thấy ENU→NED hiện tại khớp bearing RGB; mỗi PX4
  instance có local-NED origin riêng.

## RGB-only audit

Controller active chỉ nhận RGB/bbox, intrinsics, synchronized camera quaternion,
follower pose/velocity/ego-motion và bearing triangulation/EKF.

`simulation_target_ground_truth()` không có caller. LiDAR chỉ xuất telemetry
dashboard; không đi vào `TrackingManager`, estimator hoặc native Follow target.
Target GPS/Gazebo pose chỉ được dùng ngoài controller để bố trí và đánh giá SITL.

## Test commands

```bash
.venv/bin/python -m unittest discover -q
# Ran 129 tests — OK

.venv/bin/python -m py_compile \
  main.py tracking_web.py visual_follow_target.py \
  bearing_target_estimator.py body_yaw_recenter.py \
  pose_time_sync.py follow_workflow.py mavlink_manual_bridge.py

node --check /tmp/swarm-dashboard-inline.js
```

Kết quả: unit tests, `py_compile` và inline frontend JavaScript syntax đều pass.

## Trạng thái an toàn khi kết thúc

```text
tracking active: false
workflow:        IDLE
UAV-02 nav_state: 2 (Position)
UAV-02 failsafe: false
```
