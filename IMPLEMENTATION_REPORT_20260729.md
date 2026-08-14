# Báo cáo FPS và UniDepth follow 8–10 m

Ngày: 2026-07-29

## Baseline đã quan sát

Snapshot đầu phiên khi tracking UAV-02:

- Cả UAV-01 và UAV-02 đều armed, không failsafe.
- Tracking active trên UAV-02.
- Chỉ có một `mavlink_manual_bridge.py` và một Uvicorn backend.
- Gazebo và MQTT connected.
- Camera source: khoảng 16,1 FPS.
- Khoảng cách điều khiển: 11,4 m từ `visual_lut`.
- UniDepth đã load nhưng ở `stabilizing`.
- API cũ chưa xuất riêng `tracking_fps`.

Camera trong `gimbal/model.sdf` được cấu hình 30 Hz. Mười mẫu Gazebo
`real_time_factor` sau một transient chủ yếu nằm khoảng 0,998–1,003. Vì vậy
30 FPS là mức nguồn mong đợi khi GPU/CPU không bị contention.

Đo API khi tracking đã inactive:

- 20 mẫu.
- Average: 1,71 ms.
- P95: 2,16 ms.

Số API này không đại diện cho lúc UniDepth cũ đang inference.

## Nguyên nhân trong source cũ

`TrackingManager._process_frame()` giữ cùng một `RLock` trong toàn bộ:

1. tracker update;
2. bbox filtering;
3. gimbal/controller;
4. UniDepth inference;
5. overlay;
6. JPEG encode.

UniDepth V2 vì vậy block tracking stream và mọi API/WebSocket cần lấy
`TrackingManager.status()`. JPEG encode cũng kéo dài lock. Source cũ còn chọn
`visual_lut` khi UniDepth chưa ready nên UniDepth không có motion authority
độc quyền.

## Kiến trúc mới

```text
Gazebo camera (latest raw frame)
  -> tracking thread: tracker (dedicated lock)
       -> bbox + controller (short state lock)
  -> one-slot depth mailbox
       -> UniDepth worker
       -> publish only current session/newest frame result
  -> next controller update
       -> freshness/valid/ready gate
       -> 8–10 m safe-band controller
       -> altitude-hold Offboard command
```

Đặc tính:

- Queue depth tối đa 1, frame chờ mới thay frame chờ cũ.
- Model preload trong FastAPI lifespan.
- Inference không giữ `TrackingManager.lock`.
- KCF/hybrid tracker không còn giữ state/API lock; `tracker_epoch` loại bỏ
  kết quả đang chạy nếu bbox/session đã đổi.
- JPEG encode của tracking stream chạy ngoài manager lock.
- Kết quả cũ hoặc session cũ không thể ghi đè kết quả mới.
- Worker shutdown theo FastAPI lifespan.
- Sau khi depth đạt ready lần đầu cho bbox hiện tại, ready được latch; thay
  đổi khoảng cách hợp lệ không phải chờ lại nhiều stable frame.
- Freshness dùng tuổi của frame nguồn, mặc định tối đa 500 ms.
- Invalid/not-ready/stale depth phát neutral horizontal velocity nhưng giữ
  `hold_z_down_m`.
- Khi metric-depth control bật, không fallback sang bbox/LUT.

## Controller 8–10 m

- `< 8,0 m`: lùi.
- `8,0–10,0 m`: phanh về 0 và giữ.
- `> 10,0 m`: tiến.
- Không có deadband bổ sung ngoài hai biên.
- P + radial velocity feed-forward.
- Braking-speed cap theo `sqrt(2*a*d)`.
- Slew-rate giới hạn gia tốc/giảm tốc.
- Kết quả depth mới được phép bypass giới hạn publish 45 ms để motion command
  phản ứng ở controller update kế tiếp.

## Diagnostics mới

API/UI có:

- source/tracking/UniDepth/JPEG FPS;
- frame drop và queue depth;
- frame queue age;
- tracking/controller/JPEG average và P95;
- API status-lock wait average và P95;
- inference average/P95;
- source-frame age và result age;
- raw/filtered depth;
- inference time;
- depth-to-command latency;
- safe band, follow state, forward velocity và `motion_hold_z_down_m`.

## File thay đổi

- `metric_depth_estimator.py`
- `tracking_web.py`
- `main.py`
- `static/index.html`
- `.env.example`
- `README.md`
- `test_metric_depth_estimator.py`
- `test_visual_follow_target.py`
- `test_mavlink_attitude_bridge.py`
- `test_tracking_motion_altitude_hold.py` (mới)

## Kiểm thử

```text
Python compileall: OK
HTML parser: OK
JavaScript syntax (node --check): OK
Unit tests: Ran 82 tests — OK
```

Các test mới phủ:

- 7,9/8,0/9,0/10,0/10,1 m;
- phản ứng ngay khi vượt biên;
- braking trong safe band;
- stale/invalid/not-ready depth;
- không fallback LUT;
- old result rejection;
- latest-frame mailbox;
- worker reset/shutdown;
- altitude target giữ nguyên;
- MAVLink Offboard watchdog.

## Runtime sau khi restart

Backend được restart với cấu hình mục tiêu sau khi cả hai UAV đã disarm.
Không có lệnh arm/takeoff nào được gửi.

### Không có bbox/UniDepth inference

- Camera source: 19,9 FPS.
- Tracking loop: 19,9 FPS.
- Frame queue age average/p95: 0,10/0,18 ms.
- JPEG encode average/p95: 0,65/1,03 ms.
- Không drop frame.

### Có bbox, KCF và UniDepth

Một mẫu ổn định:

- Camera source: 13,4 FPS.
- Tracking: 13,4 FPS.
- KCF average/p95: 45,14/70,95 ms.
- Controller average/p95: 2,35/6,42 ms.
- JPEG average/p95: 2,30/7,47 ms.
- UniDepth: 6,11 FPS; về sau dao động tới 8,47 FPS.
- UniDepth inference average/p95: 124,46/190,52 ms.
- Depth source age: 229,12 ms; result age: 18,58 ms.
- Depth-to-command latency: 13,41 ms.
- Depth worker queue tại thời điểm lấy mẫu: 0; không bao giờ vượt 1.
- Frame chờ bị thay là hành vi chủ ý `latest-frame-wins`, không phải backlog.
- Source frame drop trong lượt đo: 5.

Estimate thực tế là 9,64 m, source `unidepth_v2`, trạng thái `ready`; controller
vào `holding`, forward velocity bằng 0. Motion bị tầng an toàn chặn với lý do
vehicle chưa armed, đúng với điều kiện thử nghiệm.

### API và stream khi inference đang chạy

30 request liên tiếp trước/sau khi đưa KCF ra khỏi state lock:

- Trước: average 12,15 ms, p95 35,93 ms, max 84,52 ms.
- Sau: average 10,33 ms, p95 15,39 ms, max 50,49 ms.
- Status-lock wait sau sửa: average 0,18 ms, p95 0,00 ms trong cửa sổ đo.
- MJPEG trả HTTP 200, first byte sau 7,5 ms và truyền 244.004 byte trong
  phép thử 2 giây khi UniDepth đang chạy.

### Kết luận FPS

Tracking bám sát FPS nguồn, nên không còn bị UniDepth tuần tự chặn. Tuy nhiên
trên máy thử, khi UniDepth V2 và Gazebo dùng tài nguyên đồng thời, FPS nguồn
giảm từ khoảng 19,9 xuống 13–14 FPS; vì vậy chưa thể đạt mục tiêu 25 FPS.
Log UniDepth cũng báo `EdgeGuidedLocalSSI` đang dùng đường chạy không tối ưu
CUDA và xFormers không có. Đây là giới hạn contention/extension runtime còn
lại, không phải backlog hoặc inference nằm trong camera callback.

## Trạng thái an toàn cuối

- UAV-01 và UAV-02: disarmed, không failsafe.
- Tracking: inactive.
- MQTT và Gazebo: connected/running.
- Một `mavlink_manual_bridge.py`, một Uvicorn backend.

## Chưa xác minh bằng flight-test

Không thực hiện flight-test thay đổi trạng thái UAV. Các trường hợp 7,9 m,
8–10 m, 10,1 m, giữ `hold_z_down_m` khi bay và watchdog đã được unit test,
nhưng vẫn cần một lượt SITL có arm/takeoff do người vận hành phê duyệt để
xác nhận động học PX4 thực tế.
