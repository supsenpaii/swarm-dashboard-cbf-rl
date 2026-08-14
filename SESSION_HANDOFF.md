# Session handoff — 2026-07-23

Project: `/home/sup/swarm_dashboard`  
Timezone: Asia/Ho_Chi_Minh

## Mục tiêu đang thực hiện

Ổn định visual tracking/follow trong Gazebo để:

- Gimbal giữ bbox, thân drone chỉ xoay yaw theo gimbal; roll/pitch thân không dùng để recenter.
- Drone giữ độ cao tuyệt đối khi tracking.
- Drone follow mục tiêu bằng tiến/lùi trong Offboard, giữ khoảng cách trong dải an toàn 8–12 m.
- Ưu tiên UniDepth V2 khi metric depth đủ ổn định; dùng bbox/LUT khi metric depth chưa sẵn sàng.
- Chuyển động tăng tốc và giảm tốc êm, dừng tại biên an toàn thay vì đổi vận tốc đột ngột.

## Phần đã hoàn thành

### Tối ưu Gazebo và tracking nền

- Giữ các tối ưu đã thực hiện đầu phiên: GUI nhẹ, tắt shadow, giảm tải lidar, camera 30 Hz, camera IMU 100 Hz và tắt sensor visualization.
- Thêm/làm chắc bộ lọc bbox, gimbal recenter, center lock và yaw precenter.
- `BodyAttitudeRecenterController` hiện cố ý yaw-only:
  - roll/pitch rate luôn bằng 0;
  - horizontal acceleration do recenter bằng 0;
  - PX4 giữ roll/pitch ổn định;
  - gimbal xử lý sai số ảnh theo phương dọc.

### Chuyển từ native PX4 Follow Target sang Offboard dynamic range

- Đã thử native PX4 Follow Target (phương án 1), nhưng lúc handover drone giật và tăng độ cao.
- Nguyên nhân quan trọng: PX4 Follow Target có ràng buộc `FLW_TGT_HT >= 8 m` và tạo chuyển động không phù hợp với yêu cầu chỉ tiến/lùi, giữ độ cao hiện tại.
- Runtime mặc định hiện dùng:
  - `motion_control_mode=offboard_dynamic_range`;
  - `native_visual_follow_enabled=false`;
  - `body_attitude_enabled=false`;
  - yaw-rate + vận tốc ngang, không điều khiển roll/pitch trực tiếp.

### Giữ độ cao tuyệt đối

- `command_tracking_motion()` chụp `z_down` hiện tại khi bắt đầu motion.
- Vận tốc body-forward được đổi sang local North/East theo heading.
- Setpoint phát bằng `velocity_frame=local_ned_altitude_hold`, `hold_z_down_m` cố định và `down_velocity_m_s=0`.
- Khi motion dừng hoặc bị block, altitude-hold target được reset.

### Follow theo dải an toàn 8–12 m

- Thêm `safe_follow_band_error()`:
  - `<8 m`: sai số âm, yêu cầu lùi;
  - `8–12 m`: sai số 0, giữ vị trí;
  - `>12 m`: sai số dương, yêu cầu tiến.
- Thêm `braking_speed_limit_m_s()` theo `sqrt(2*a*d)` để giới hạn tốc độ có thể phanh về 0 tại biên.
- Vẫn dùng `follow_slew` để giới hạn gia tốc/giảm tốc.
- Radial feed-forward không còn tiếp tục đẩy drone khi đã ở trong dải an toàn.
- Cấu hình runtime hiện tại:
  - safe min `8.0 m`;
  - safe max `12.0 m`;
  - brake deceleration `1.5 m/s²`;
  - offboard max forward `3.0 m/s`.

### Metric depth và fallback

- UniDepth V2 ViT-L vẫn là backend metric depth hiện tại.
- Khi metric depth chưa `ready`, Offboard follow không còn dừng hoàn toàn; controller dùng `visual_lut`.
- Đã xác nhận UniDepth không phản ánh khoảng cách đáng tin cậy trong scene drone nhỏ/nền trời:
  - raw depth dao động nhiều ngay cả khi vị trí gần như đứng yên;
  - stability gate `max step 0.5 m` thường reset;
  - metric state thường ở `stabilizing`;
  - EMA kép làm phản hồi chậm;
  - metric depth và ground truth có thể chênh lệch lớn.
- Model 1.42 GB đã được tải đầy đủ vào Hugging Face cache local.
- Thêm preload:
  - `MetricDepthEstimator.preload()`;
  - `TrackingManager.preload_metric_depth()`;
  - `main.lifespan()` preload model bằng `asyncio.to_thread()` trước khi dashboard nhận request.
- Runtime hiện được khởi động với cache offline và inference mỗi 6 frame để giảm tải.

### Khắc phục dashboard treo khi khoanh bbox

- Lần đầu bật UniDepth, `from_pretrained()` tải model 1.42 GB ngay trong tracking thread.
- `_process_frame()` đang giữ lock trong khi `_update_metric_depth_locked()` gọi inference/load, nên stream/status trông như treo.
- Đã dừng process bị kẹt, tải model riêng bằng CLI, kiểm tra load offline thành công, rồi chuyển model load sang startup preload.
- Dashboard hiện phản hồi bình thường và UniDepth báo `available=true`.

## File và function đã sửa

### Code chính

- `body_attitude_recenter.py`
  - `BodyAttitudeRecenterController.update()`
  - Chuyển recenter thân sang yaw-only; roll/pitch và horizontal recenter acceleration giữ 0.
- `main.py`
  - `command_tracking_motion()`
  - Đổi body-forward sang local N/E và giữ tuyệt đối `z_down`.
  - `lifespan()`
  - Preload metric-depth model trước khi nhận request.
- `tracking_web.py`
  - `safe_follow_band_error()`
  - `braking_speed_limit_m_s()`
  - `TrackingManager.preload_metric_depth()`
  - `TrackingManager._update_follow_locked()`
  - `TrackingManager._update_motion_locked()`
  - Dải 8–12 m, braking cap, slew, fallback LUT, yaw-only Offboard motion.
- `metric_depth_estimator.py`
  - `MetricDepthEstimator.preload()`
  - `MetricDepthEstimator.update()`
  - `MetricDepthEstimator._ensure_loaded()`
  - Preload model ngoài thao tác bbox; inference/ROI/EMA và stability gate vẫn giữ nguyên.
- `mavlink_manual_bridge.py`
  - Các thay đổi Follow Target/Offboard bridge từ đầu phiên vẫn còn trong source.

### Test

- `test_body_attitude_recenter.py`
  - Cập nhật test để xác nhận roll/pitch không recenter thân và yaw vẫn hoạt động.
- `test_visual_follow_target.py`
  - Thêm test dải 8–12 m và giới hạn tốc độ phanh.
- `test_metric_depth_estimator.py`
  - Test metric depth estimator hiện có.
- `test_mavlink_attitude_bridge.py`
  - Test bridge/Follow Target hiện có.

### Hạ tầng và giao diện đã thay đổi trong ngày

- `static/index.html`
- `gazebo_gui_light.config`
- `start_gazebo_optimized.sh`
- Ngoài project:
  - PX4 `default.sdf`;
  - `x500_custom/model.sdf`;
  - `gimbal/model.sdf`.

Các SDF ngoài project có backup `.pre_gui_optimization_20260723`.

## Lý do của các thay đổi quan trọng

- Yaw-only đúng với yêu cầu: gimbal theo mục tiêu theo cả ba trục, nhưng thân chỉ quay yaw; PX4 tự ổn định roll/pitch.
- Native Follow Target không phù hợp vì tạo handover mạnh và thay đổi độ cao; Offboard cho phép chỉ tiến/lùi + yaw và giữ `z_down`.
- Deadband quanh một target đơn không mô tả đúng yêu cầu 8–12 m; dùng hai biên độc lập làm rõ hành vi.
- P-control và slew chưa bảo đảm phanh đúng biên khi đang chạy nhanh; braking cap theo quãng đường giúp giảm tốc trước biên.
- Metric depth chưa ổn định không được chặn toàn bộ follow; LUT phản ứng theo bbox và là fallback rõ ràng.
- Model load/download không được xảy ra khi người dùng khoanh bbox; preload ở startup loại nguyên nhân treo đầu tiên.

## Lỗi đã gặp và cách xử lý

- Native Follow Target giật mạnh/mất bbox: thêm handover/center lock rồi cuối cùng chuyển sang Offboard dynamic range.
- Drone đột ngột tăng độ cao: xác định native Follow Target và `FLW_TGT_HT` là nguyên nhân; bỏ native mode.
- Drone có xu hướng tăng/giảm độ cao trong tracking: chuyển sang absolute local-NED altitude hold.
- Follow quá chậm hoặc không bám mục tiêu: tăng max forward lên 3 m/s, thêm radial velocity feed-forward và sau đó giới hạn lại bằng safe-band braking.
- Drone không follow khi UniDepth không `ready`: bỏ hard wait trong Offboard follow và fallback sang `visual_lut`.
- Khoảng cách UniDepth thay đổi rất ít hoặc sai thực tế: xác nhận raw depth nhiễu, stability gate thường reset, EMA kép gây trễ và domain mismatch drone nhỏ/nền trời.
- Dashboard treo sau khi khoanh bbox: model 1.42 GB đang tải đồng bộ trong tracking lock; tải riêng, cache local, preload lúc startup.
- Lần dừng dashboard đang tải model không phản hồi SIGTERM: buộc dừng đúng PID, sau đó khởi động lại sạch.
- Repository không có Git metadata hợp lệ: `git status` và `git log -1` đều báo `not a git repository`; không thể tạo diff/commit hoặc xác nhận commit gần nhất.

## Kiểm tra đã chạy cuối phiên

```bash
.venv/bin/python -m compileall -q \
  main.py tracking_web.py metric_depth_estimator.py \
  body_attitude_recenter.py visual_follow_target.py \
  mavlink_manual_bridge.py \
  test_body_attitude_recenter.py test_visual_follow_target.py \
  test_mavlink_attitude_bridge.py test_metric_depth_estimator.py

.venv/bin/python -m unittest discover -p 'test_*.py'
```

Kết quả:

- Compile: thành công, không có lỗi syntax.
- Unit test: `Ran 63 tests in 0.018s` — `OK`.
- Load UniDepth offline: `loaded=true`, `available=true`, không có load error.
- API dashboard: MQTT/Gazebo connected; metric depth enabled/available.

## Kiểm tra CodeGraph

Call graph hiện tại:

```text
main.lifespan()
  -> asyncio.to_thread(TrackingManager.preload_metric_depth)
     -> MetricDepthEstimator.preload()
        -> MetricDepthEstimator._ensure_loaded()

TrackingManager._process_frame()
  -> TrackingManager._update_gimbal_locked()
     -> TrackingManager._update_metric_depth_locked()
        -> MetricDepthEstimator.update()
           -> _infer_depth()
           -> _distance_from_bbox()
     -> TrackingManager._update_follow_locked()
        -> safe_follow_band_error()
        -> braking_speed_limit_m_s()
     -> TrackingManager._update_motion_locked()
        -> motion_command callback
           -> main.command_tracking_motion()
              -> publish_control_message(type="offboard_follow")
```

Blast radius:

- `TrackingManager`: 1 constructor caller trong `main.py`; chưa có integration test toàn manager.
- `command_tracking_motion()`: 1 callback caller trong `main.py`; chưa có test trực tiếp.
- `safe_follow_band_error()`: CodeGraph thấy 4 caller; được phủ bởi `test_visual_follow_target.py`.
- `braking_speed_limit_m_s()`: CodeGraph thấy 3 caller; được phủ bởi `test_visual_follow_target.py`.
- `MetricDepthEstimator`: 4 caller trong `tracking_web.py`; được phủ một phần bởi `test_metric_depth_estimator.py`.
- `BodyAttitudeRecenterController`: 3 caller trong `main.py`; được phủ bởi `test_body_attitude_recenter.py`.

Không cần cập nhật index CodeGraph thủ công; truy vấn cuối đọc source hiện tại trên đĩa.

## Trạng thái runtime lúc lưu phiên

Snapshot cuối:

- Dashboard PID `183036`, MQTT/Gazebo connected.
- `tracking.active=true`, drone được tracking là `UAV-02`.
- `motion_control_mode=offboard_dynamic_range`.
- `follow_state=holding`, `motion_state=holding`.
- Forward velocity và yaw-rate command đều `0`.
- Cả `UAV-01` và `UAV-02` đang báo `armed=true`, không failsafe.
- Safe band `8–12 m`.
- `metric_depth_enabled=true`, `metric_depth_available=true`.
- UniDepth vẫn `stabilizing`; raw khoảng `2.76 m`, filtered khoảng `4.32 m`.
- Controller đang dùng `visual_lut`, follow distance snapshot `8.0 m`.
- Có hai `mavlink_manual_bridge.py` đang chạy: PID `149710` và `172633`.
- Có một ROS launch PID `137692`.

Không tự ý stop tracking, disarm hoặc kill bridge trong quy trình lưu phiên.

## Công việc còn dang dở

1. **An toàn runtime:** xác nhận/dừng tracking và disarm trước khi sửa hoặc test; kiểm tra hai bridge trùng và chỉ giữ process đúng.
2. **Metric depth chưa đáng tin cậy:** UniDepth vẫn không theo ground truth; chưa được dùng làm source vì chưa `ready`.
3. **Inference vẫn synchronous:** preload đã loại download/load khỏi thao tác bbox, nhưng `_infer_depth()` vẫn chạy trong tracking lock. Cần depth worker bất đồng bộ queue size 1 để UI/KCF/gimbal không bị block bởi inference.
4. **Safe-band controller chưa flight-test đầy đủ:** cần thử `<8 m`, `8–12 m`, `>12 m`, mục tiêu chuyển động nhanh, và kiểm tra phanh tại biên.
5. **Altitude hold chưa có integration test:** cần log `z_down`, `hold_z_down_m`, `vz` trong flight test.
6. **Khoảng cách:** cần benchmark UniDepth/VDA/Depth Anything V2 với ground truth Gazebo; không nên dùng monocular depth làm nguồn an toàn duy nhất.
7. **Git:** project hiện không có repository hợp lệ; cần khôi phục/init Git theo quyết định của người dùng trước khi có thể diff/commit.

## Bước đầu tiên cho phiên sau

Trước mọi thay đổi, đọc trạng thái an toàn:

```bash
curl -sS http://127.0.0.1:8000/api/drones | jq '{
  drones:(.drones|with_entries(.value={
    armed:.value.status.armed,
    nav_state:.value.status.nav_state,
    failsafe:.value.status.failsafe
  })),
  tracking:{
    active:.tracking.active,
    drone_id:.tracking.drone_id,
    state:.tracking.state,
    motion_state:.tracking.motion_state,
    distance_source:.tracking.follow_distance_source,
    metric_state:.tracking.metric_depth_state
  }
}'
```

Nếu tracking còn active hoặc UAV còn armed, dùng dashboard để Stop Tracking và disarm trước. Sau đó:

```bash
pgrep -af 'ros2 launch swarm_telemetry two_uav_nodes.launch.py|mavlink_manual_bridge.py|uvicorn main:app'
```

Xác định bridge nào đang sở hữu socket/port và chỉ dừng bản trùng. Công việc code ưu tiên sau đó là tách UniDepth inference sang worker bất đồng bộ queue size 1, rồi benchmark khoảng cách với ground truth trước khi flight-test safe band.

## Bảo mật

Handoff không chứa API key, mật khẩu, token, nội dung `.env` hoặc bí mật truy cập.
