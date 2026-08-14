# Báo cáo hoàn thiện UniDepth follow 8–12 m

Ngày: 2026-07-29

## Kết quả

Controller dùng dải an toàn 8–12 m:

- nhỏ hơn 8 m: lệnh lùi;
- từ 8 m đến hết 12 m: target tiến/lùi bằng 0, phanh rồi giữ;
- lớn hơn 12 m: lệnh tiến.

Hai biên là inclusive và không có deadband/hysteresis phụ. Sau khi estimator
đã `ready`, một depth hợp lệ mới vượt biên được đưa vào controller ở update
kế tiếp. Braking cap dùng `sqrt(2*a*d)` và lệnh được giới hạn bởi P gain,
maximum velocity và slew-rate.

Khi `SWARM_METRIC_DEPTH_FOLLOW_CONTROL=true`, chỉ `unidepth_v2` có quyền tạo
lệnh tiến/lùi. Depth chưa ready, invalid, inference error hoặc quá 500 ms tạo
lệnh ngang trung tính; LUT/bbox/lidar/ground truth không được fallback vào
control path.

## Kiến trúc và an toàn

```text
Gazebo camera
  -> tracker/bbox
  -> one-slot latest-frame depth mailbox
  -> UniDepth worker ngoài TrackingManager lock
  -> publish newest valid result
  -> validity/freshness/quality gate
  -> 8–12 m P + braking-cap + slew controller
  -> motion callback
  -> local-NED altitude-hold command
  -> MQTT -> MAVLink bridge -> PX4
```

- Queue worker tối đa một frame; frame chờ cũ bị thay, không tích backlog.
- Kết quả session/frame cũ không ghi đè kết quả mới.
- Worker shutdown trong FastAPI lifespan; inference exception không làm chết
  tracking/backend.
- `command_tracking_motion()` chụp `z_down` khi motion bắt đầu. Thay đổi depth
  không đổi `hold_z_down_m`, và forward/backward không tạo vertical velocity.
- Backend và bridge đều có watchdog; command cũ được neutralize khi timeout.
- Calibration production vẫn tắt. Candidate hiện tại có
  `production_recommended=false` và loader từ chối profile evaluation-only.

## API và giao diện

Status/UI lấy safe band động từ controller và hiển thị:

- safe band 8–12 m và authority `unidepth_v2`;
- optical Z, radial, calibrated và filtered/control distance;
- depth/follow state, queue/drop, source/tracking/depth FPS và latency;
- requested/published forward velocity và `hold_z_down_m`;
- timestamp monotonic lúc depth result được công bố;
- timestamp monotonic lúc motion command được tạo;
- depth-to-motion-command latency.

Các field API mới:

- `depth_result_published_timestamp`;
- `motion_command_created_timestamp`;
- `depth_to_motion_command_latency_ms`;
- `requested_forward_velocity_m_s`;
- `published_forward_velocity_m_s`.

## File và function thay đổi trong lượt này

- `tracking_web.py`
  - default/fallback `follow_max_distance_m=12.0`;
  - `_on_metric_depth_result()` ghi timestamp công bố;
  - `_mark_metric_depth_command_locked()` ghi timestamp command và latency;
  - `status()` xuất safe band, timestamp và requested/published velocity.
- `metric_depth_estimator.py`
  - default upper crossing boundary đổi thành 12 m.
- `test_visual_follow_target.py`
  - test 7,9/8/9/10/11/12/12,1 m, crossing tức thời và timestamp latency.
- `test_metric_depth_estimator.py`
  - regression filter không che crossing 11,5 -> 12,1 m.
- `static/index.html`
  - hiển thị requested và published forward velocity.
- `.env.example`, `README.md`
  - cấu hình/tài liệu đồng bộ 8–12 m.

`main.py`, `mavlink_manual_bridge.py`, altitude-hold và worker không cần sửa
trong lượt này vì các cơ chế tương ứng đã có test regression và đạt.

## Kiểm thử

```text
python -m compileall ...                         OK
python -m unittest discover -p 'test_*.py'       113 tests, OK
```

Test suite bao phủ safe-band/crossing, invalid-stale-unready neutral command,
không fallback, old-result rejection, latest-frame-wins, worker shutdown,
calibration không vật lý/evaluation-only, altitude hold và bridge watchdog.

## FPS và latency

Sau khi người vận hành xác nhận disarm, backend đã được restart an toàn với
UniDepth preload và cấu hình 8–12 m. Kiểm tra live read-only:

- health API: HTTP 200, first byte 5,69 ms, tổng 5,84 ms;
- MJPEG UAV-02: HTTP 200, first byte 4,00 ms, truyền 119.952 byte trong 2 s;
- worker đang chạy, queue 0/1, dropped job 0;
- tracking không được bật nên source/tracking/depth FPS ở snapshot cuối bằng
  0 và không được coi là benchmark inference.

Số đo gần nhất từ lượt runtime an toàn trước đó:

- camera source khi tracking: khoảng 13,4 FPS;
- tracking: khoảng 13,4 FPS;
- UniDepth: khoảng 6,11–8,47 FPS;
- inference average/p95: 124,46/190,52 ms;
- depth source age: 229,12 ms;
- depth result age: 18,58 ms;
- depth-to-command latency: 13,41 ms;
- queue không vượt 1.

Tracking đã bám FPS nguồn và UniDepth không còn block loop. Mục tiêu 25 FPS
chưa đạt vì Gazebo, KCF và UniDepth tranh chấp CPU/GPU; đây không phải queue
backlog. Startup hiện vẫn cảnh báo xFormers và CUDA-optimized
`EdgeGuidedLocalSSI` chưa có, nên inference chưa chạy theo đường tối ưu.

## Trạng thái runtime cuối

- UAV-01: `armed=false`, `failsafe=false`, nav state 2.
- UAV-02: `armed=false`, `failsafe=false`, nav state 2.
- Tracking: inactive; UniDepth `preloaded/available`; depth queue 0.
- MQTT connected.
- Một Uvicorn process và một `mavlink_manual_bridge.py`.
- Backend mới đang chạy và API xác nhận safe band `8.0–12.0 m`, metric control
  bật, calibration tắt.
- Không có lệnh arm/takeoff/land/MQTT/MAVLink nào được phát trong lượt này.

## Giới hạn accuracy và điều kiện flight-test

Dataset held-out hiện tại cho UniDepth không calibration có MAE 8,0928 m.
Scale-only hợp lệ tốt nhất vẫn có MAE 2,3651 m; output theo khoảng cách không
đơn điệu và candidate được đánh dấu `production_recommended=false`.

Do đó logic controller đã hoàn thiện nhưng **chưa đủ điều kiện flight-test**:
accuracy hiện tại chưa đáng tin cậy để phân biệt an toàn các biên 8 và 12 m.
Không giảm quality gate và không bật calibration candidate để che giới hạn
này.

## Cách chạy lại

1. Backend hiện đã được restart sau khi xác nhận cả hai UAV
   `armed=false`, `failsafe=false`. Luôn kiểm tra lại điều kiện này trước lần
   restart hoặc thử nghiệm tiếp theo.
2. Nạp `.env.example` hoặc `.env` đã cấu hình `MIN=8.0`, `MAX=12.0`,
   metric control bật và calibration tắt.
3. Restart đúng một backend Uvicorn để source/config mới có hiệu lực.
4. Chạy lại:

   ```bash
   python -m compileall -q \
     main.py tracking_web.py tracking_hybrid.py \
     metric_depth_estimator.py metric_depth_accuracy.py \
     metric_depth_evaluation.py visual_follow_target.py \
     body_attitude_recenter.py mavlink_manual_bridge.py

   python -m unittest discover -p 'test_*.py'
   ```

5. Chỉ làm flight-test sau khi có validation accuracy mới đạt yêu cầu và người
   vận hành phê duyệt arm/takeoff rõ ràng.
