# Báo cáo hoàn thiện tracking/yaw/follow — 2026-07-29

## Kết luận

Đã hoàn thiện và kiểm thử offline ba nhánh trong cùng pipeline:

1. bbox có scale refinement cục bộ quanh tâm KCF/Kalman;
2. body yaw dùng duy nhất unified `offboard_follow` và không còn phụ thuộc
   trạng thái UniDepth;
3. forward/backward chỉ dùng UniDepth V2, giữ safe band 8–12 m.

Không có flight/motion test. Hai UAV vẫn armed trong suốt lần kiểm tra cuối,
do đó backend đang chạy không được restart và không chọn bbox.

## Trạng thái an toàn runtime

Read-only API cuối phiên:

| Mục | Giá trị |
|---|---:|
| UAV-01 | armed=true, failsafe=false, nav_state=2, z_down=-5.23 m |
| UAV-02 | armed=true, failsafe=false, nav_state=2, z_down=-6.53 m |
| Tracking | inactive, bbox=null |
| Camera UAV-02 | 14.7 FPS |
| Uvicorn | 1 process, PID 244352 |
| `mavlink_manual_bridge.py` | 1 process, PID 11771 |
| PX4 SITL | 2 process, PID 237332/237769 |

Backend PID 244352 vẫn là process được khởi động trước thay đổi mã. Các trường
API mới chỉ xuất hiện sau lần restart an toàn khi cả UAV đã disarm.

## Nguyên nhân ba lỗi

### Bbox không đổi kích thước

Đường normal tracking chỉ dùng bbox do OpenCV KCF trả về. KCF bám translation
tốt nhưng không có scale estimator riêng; multi-scale chỉ xuất hiện ở nhánh
redetect. Sau đó `VisualBBoxFilter(size_alpha=0.12)` tiếp tục làm chậm thay
đổi width/height. ROI UniDepth vì thế không còn khớp vật thể khi khoảng cách
thay đổi.

### Drone không quay theo gimbal

Runtime cũ có `body_yaw_enabled=false`. Đồng thời yaw-only callback không phải
motion authority khi Offboard follow bật. Trong unified path, depth
loading/invalid/stale đặt cả forward và yaw về 0, khiến một lỗi range vô tình
tước luôn quyền recenter yaw.

### Drone chưa follow

Forward gate và yaw gate bị ghép chung, tracker quality score chưa có ý nghĩa
rõ ràng, bbox scale sai làm ROI/quality gate depth sai. Controller 8–12 m đã
có nhưng không nhận đủ chuỗi `tracking quality -> bbox valid -> depth valid`
để phát command.

## Kiến trúc trước và sau

Trước:

```text
KCF fixed-scale bbox
  -> bbox filter alpha 0.12
  -> UniDepth gate
       -> invalid: forward=0, yaw=0
       -> valid: forward + yaw
```

Sau:

```text
raw 1280x720
  -> KCF translation + Kalman prediction
  -> local MultiScaleBBoxRefiner
       template correlation + spatial PSR
       appearance memory + texture + shape/motion continuity
       ambiguity/outlier/boundary gates
  -> confidence-adaptive VisualBBoxFilter
  -> quality gates
       bbox/tracker valid ---------> yaw authority
       UniDepth valid/ready/fresh -> forward authority
  -> one unified offboard_follow command
       vx body-forward + yaw-rate + vz=0 + fixed hold_z_down
  -> bridge watchdog -> stale command neutralized
```

UniDepth worker vẫn chạy latest-frame-wins ngoài tracking lock. Preview vẫn
640×360; tracker và UniDepth nhận raw 1280×720.

## Scale refinement

`MultiScaleBBoxRefiner` thử:

```text
0.85, 0.92, 0.97, 1.00, 1.03, 1.08, 1.15
```

Mỗi candidate chỉ tìm trong cửa sổ nhỏ quanh tâm KCF/Kalman. Candidate được
định vị bằng normalized correlation response, sau đó chấm bởi correlation,
appearance memory, texture, scale continuity và PSR. Scale chỉ được nhận nếu:

- score >= 0.68;
- PSR >= 3.5 cho thay đổi scale đáng kể;
- không có peak scale mơ hồ;
- bước scale <= 18%;
- candidate nằm trong frame;
- có gain đủ so với scale 1.0.

Scale invalid giữ kích thước trước ở tâm translation mới và ghi rõ
`scale_reject_reason`. Template chỉ update khi confidence >= 0.78; KCF reinit
khi scale accepted lệch >= 5%.

`VisualBBoxFilter` dùng size alpha 0.20–0.48 theo confidence. Synthetic test
xác nhận filtered width tiến từ 60 px tới trên 70 px sau 5 frame khi refined
width là 72 px, nhưng scale jump vẫn bị reject.

## Tracker score

Quality score mới xuất riêng correlation, appearance, motion, shape, texture
và final score. Validation synthetic 30 mẫu/lớp:

| Lớp | Mean score | p05 | p95 | Tracking accepted | Follow gate >= 0.70 |
|---|---:|---:|---:|---:|---:|
| đúng target | 0.994 | 0.994 | 0.994 | 30/30 | 30/30 |
| background | 0.390 | 0.390 | 0.390 | 0/30 | 0/30 |
| distractor | 0.593 | 0.580 | 0.604 | 0/30 | 0/30 |
| lost target | 0.390 | 0.390 | 0.390 | 0/30 | 0/30 |

Từ phân bố này, `SWARM_REID_TRACK_THRESHOLD=0.62` và follow/depth quality
gate 0.70 được giữ; không hạ threshold tùy ý.

## Yaw authority và dấu

`unified_recenter_yaw_rate_deg_s()` dùng:

```text
combined_error = gimbal_yaw_from_home + image_yaw_error
yaw_rate = limited(Kp * error, max_rate, slew_rate)
```

Mặc định:

- error dương/gimbal lệch phải -> PX4 BODY_NED yaw-rate dương;
- error âm/gimbal lệch trái -> yaw-rate âm;
- deadband -> 0;
- `SWARM_APPARENT_SIZE_YAW_INVERT=true` chỉ dùng nếu airframe/joint convention
  thực tế đảo dấu.

Depth loading/invalid/stale chỉ làm forward=0 và down=0; yaw vẫn hoạt động nếu
bbox/tracker/gimbal hợp lệ. Image chưa align cũng chặn forward nhưng giữ yaw.
Bbox mất, redetect, ambiguous hoặc safety gate fail chặn cả forward và yaw.
Legacy yaw-only publisher bị vô hiệu khi unified motion callback tồn tại.

## Follow gate 8–12 m

- `< 8.0 m`: moving backward;
- `8.0–12.0 m`, gồm cả hai biên: braking/holding, target forward=0;
- `> 12.0 m`: moving forward.

UniDepth V2 là distance authority duy nhất khi
`SWARM_METRIC_DEPTH_FOLLOW_CONTROL=true`. Không fallback LUT/bbox/lidar/ground
truth. Kết quả mới vượt biên vẫn được xử lý ở update kế tiếp. Depth stale,
invalid, NaN/inf hoặc unready phát neutral forward ngay. Yaw authority tách
riêng như trên. Vertical command trong metric follow luôn bằng 0 và
`hold_z_down_m` không đổi.

`follow_block_reason` có các trạng thái đo được như `no_bbox`,
`tracker_redetecting`, `tracker_ambiguous`, `low_tracker_score`,
`bbox_stabilizing`, `waiting_for_depth`, `invalid_depth`, `stale_depth`,
`aligning_yaw`, `motion_safety_blocked`, `holding_safe_band`.

## FPS/latency

Runtime trước/sau live không được đo bằng cách kích hoạt tracking vì UAV đang
armed. Baseline read-only camera cuối phiên là 14.7 FPS; tracking inactive.

Benchmark synthetic 1280×720, 60 update:

| Pipeline | Average | p95 | Throughput tương đương |
|---|---:|---:|---:|
| KCF/quality, scale refiner tắt | 4.48 ms | 7.97 ms | 223.2 FPS |
| KCF + local scale refiner | 13.89 ms | 21.76 ms | 72.0 FPS |

Scale refinement tăng khoảng 9.41 ms/frame nhưng vẫn có headroom lớn so với
camera 13–20 FPS. Đây là benchmark CPU synthetic, không phải FPS live có
UniDepth. Cần đo lại live sau restart/disarm.

## File/function đã sửa

- `tracking_hybrid.py`
  - `MultiScaleBBoxRefiner`, `ScaleRefinementResult`;
  - `HybridMemoryTrackerService.update`, `_accept_tracking`, diagnostics;
  - chuẩn hóa final quality và threshold.
- `visual_follow_target.py`
  - `VisualBBoxFilter`: dynamic confidence size alpha.
- `tracking_web.py`
  - `unified_recenter_yaw_rate_deg_s`;
  - `_tracking_motion_gate_locked`, `_update_motion_locked`;
  - `_filter_bbox_locked`, status diagnostics, `follow_block_reason`;
  - legacy yaw bị khóa khi unified authority tồn tại.
- `static/index.html`
  - raw/refined/filtered bbox, scale score/PSR, yaw authority và block reason.
- `.env.example`, `README.md`
  - cấu hình scale/yaw và hướng dẫn pipeline.
- `test_tracking_scale_refiner.py`, `test_tracking_yaw_authority.py`
  - 19 test mới.

`main.py` và `mavlink_manual_bridge.py` không cần thay đổi trong lượt này:
`command_tracking_motion()` đã giữ `hold_z_down_m`; bridge watchdog hiện có đã
neutralize setpoint stale và có regression test.

## Kết quả test

```text
python -m compileall -q ...                          PASS
python -m unittest discover -p 'test_*.py'          146/146 PASS
test_mavlink_attitude_bridge.py                     14/14 PASS
```

Bao phủ scale tăng/giảm/translation/outlier/distractor/ambiguity/KCF reinit,
yaw hai dấu/deadband/slew, depth stale/unready vẫn yaw, bbox mất/ambiguous
neutralize, safe band 8–12 m, next-update boundary response, no fallback,
latest-frame-wins, altitude hold và watchdog.

## Cách chạy lại an toàn

1. Đưa cả UAV về trạng thái disarm và xác nhận failsafe=false.
2. Nạp `.env.example` đã điều chỉnh vào `.env`.
3. Dừng tracking trước khi restart backend.
4. Restart đúng một Uvicorn và giữ đúng một bridge.
5. Chạy:

```bash
python -m compileall -q \
  main.py tracking_web.py tracking_hybrid.py \
  visual_follow_target.py metric_depth_estimator.py \
  metric_depth_accuracy.py metric_depth_evaluation.py \
  body_attitude_recenter.py mavlink_manual_bridge.py

python -m unittest discover -p 'test_*.py'
```

6. Khi vẫn disarm, chọn bbox trên target synthetic/SITL đứng yên và kiểm tra
   `advanced.raw_tracker_bbox`, `advanced.refined_bbox`, `filtered_bbox`,
   `scale_score`, `scale_psr`, `follow_block_reason`.
7. Chỉ sau phê duyệt riêng mới test chuyển động:
   gimbal phải/trái, depth stale, 7.9 m, 8–12 m và 12.1 m.

## Giới hạn và flight readiness

Chưa đủ điều kiện tuyên bố flight-ready:

- backend mới chưa được restart/đo live vì cả UAV đang armed;
- chưa có movement test xác nhận dấu yaw trên airframe thực;
- accuracy UniDepth production vẫn cần validation đủ phân biệt hai biên;
- benchmark score/FPS ở trên là synthetic, cần recorded/live dataset bổ sung.

Logic controller và các fail-safe đã qua unit/synthetic test, nhưng flight
test chỉ được thực hiện sau khi UAV disarm và người vận hành phê duyệt rõ ràng.

## Runtime activation sau khi người vận hành disarm

Sau khi người vận hành báo disarm, API được kiểm tra lại trước mọi thay đổi:

- UAV-01: `armed=false`, `failsafe=false`;
- UAV-02: `armed=false`, `failsafe=false`;
- tracking: inactive, bbox=null;
- một Uvicorn, một `mavlink_manual_bridge.py`, hai PX4 SITL.

Backend cũ PID 244352 được dừng bằng SIGTERM và thay bằng đúng một Uvicorn PID
273587, giữ nguyên toàn bộ cấu hình SWARM của process cũ. Startup hoàn tất,
MQTT kết nối thành công và UniDepth có trạng thái `preloaded`.

API mới đã xác nhận:

| Trường | Giá trị |
|---|---|
| `follow_block_reason` | `tracking_inactive` |
| `yaw_authority` | `none` |
| `yaw_block_reason` | `tracking_inactive` |
| safe band | 8.0–12.0 m |
| metric depth | `preloaded` |
| Uvicorn/bridge | 1/1 process |

Camera stream UAV-02 được mở read-only trong 6 giây, không start tracking và
không chọn bbox:

- source FPS: 11.3–11.4;
- conversion average/p95: 0.82/1.74 ms;
- JPEG encode average/p95: 0.79/0.94 ms;
- JPEG stream trả HTTP 200, sau đóng stream client count trở về 0;
- năm lần gọi `/api/drones`: 1.17–2.05 ms.

Smoke test sau restart: 33/33 test scale/yaw/watchdog đạt. Không có lệnh arm,
takeoff, bbox selection, follow, body yaw hoặc forward/backward nào được phát.

Log preload cũng cho thấy `xFormers` và CUDA-optimized
`unidepth/ops/extract_patches` chưa được cài. Backend vẫn chạy, nhưng
EdgeGuidedLocalSSI phải dùng đường non-CUDA tối ưu và có thể làm UniDepth
inference chậm. Đây là giới hạn hiệu năng cần đo khi được phép chọn bbox và
chạy inference, không nên xử lý bằng cách tăng depth timeout.

## Giới hạn yaw gimbal ±10° và limit recenter

Yaw gimbal tracking được đổi từ mặc định ±15° sang ±10°. Mọi target yaw gửi
tới Gazebo đều bị clamp trong `[-10°, +10°]`; API trả thêm giới hạn min/max và
trạng thái yêu cầu quay ra ngoài biên.

Khi feedback chạm biên hoặc tracker yêu cầu vượt biên, body recenter latch
được bật. Unified body yaw dùng tổng:

```text
gimbal_yaw_from_home + bbox_horizontal_image_error
```

để vừa unload gimbal về 0° vừa đưa bbox về tâm ảnh. Trong thời gian latch,
forward velocity bị ép về 0. Latch chỉ nhả khi cả yaw gimbal và bbox image
error đều trong ±1°; yaw vẫn có max-rate và slew-rate như trước.

Cấu hình:

```dotenv
SWARM_TRACKING_GIMBAL_YAW_LIMIT_DEG=10
SWARM_TRACKING_GIMBAL_YAW_HOME_EXIT_DEG=1.0
SWARM_TRACKING_GIMBAL_BBOX_CENTER_EXIT_DEG=1.0
```

Regression suite sau thay đổi: 150/150 test đạt. Bốn test mới kiểm tra clamp
hai dấu, target Gazebo không vượt biên, latch chỉ nhả khi cả hai error về tâm,
và limit recenter chặn forward nhưng vẫn phát yaw đúng dấu.

Backend chưa được restart sau thay đổi này: kiểm tra an toàn ngay trước restart
cho thấy UAV-02 đã `armed=true` và tracking ở trạng thái `selecting`. Runtime
được giữ nguyên, không stop tracking và không phát gimbal/yaw/motion command.

Sau khi người vận hành disarm lần nữa, trạng thái được xác nhận lại: cả hai UAV
`armed=false`, `failsafe=false`, tracking inactive, bbox=null và motion
inactive. Backend được restart an toàn thành PID 281362. API runtime mới xác
nhận `gimbal_yaw_limit_deg=10.0`, hai exit threshold bằng 1.0° và recenter
latch đang false khi idle. Vẫn chỉ có một Uvicorn và một bridge; không thực
hiện tracking/bbox/motion test.

## Giảm tốc gimbal để tránh mất bbox

Baseline read-only khi lỗi xảy ra cho thấy source/tracking chỉ khoảng 10 FPS,
nhưng pitch gimbal được yêu cầu khoảng 256°/s và yaw khoảng 12,7°/s. Riêng
pitch có thể đổi hơn 25° giữa hai frame, đủ làm mục tiêu rời khung hình trước
khi tracker kịp cập nhật. Nguyên nhân cấu hình là PID `Kp=18`, EMA alpha 0,92,
trần 360°/s và slew 1200°/s².

PID mặc định được hạ còn `Kp=6`, `Ki=0,10`, `Kd=0,15`, EMA alpha 0,60,
deadband 0,75°, trần 60°/s và slew 180°/s². Tầng publish có limiter độc lập:
pitch tối đa 30°/s, yaw tối đa 25°/s và gia tốc tối đa 90°/s². Ở chu kỳ đầu
100 ms, xung pitch 256°/s chỉ tạo lệnh 9°/s, tương đương bước góc 0,9°. Nếu
không có lệnh mới trong 250 ms, tốc độ trước đó bị reset để không gây giật khi
tracker bắt lại mục tiêu.

API bổ sung raw PID rate, rate sau limiter, rate thực tế publish, giới hạn từng
trục và giới hạn gia tốc. Regression sau thay đổi: compileall đạt và 154/154
unit test đạt.

Sau khi xác nhận cả UAV-01/UAV-02 đều disarmed, không failsafe và tracking
inactive, backend được restart an toàn thành PID 287706. Runtime xác nhận giới
hạn roll/pitch/yaw lần lượt là 30/30/25°/s và gia tốc 90°/s². Có đúng một
Uvicorn và một `mavlink_manual_bridge.py`. Không chọn bbox, không arm/takeoff
và không thực hiện movement test; cần kiểm tra tracking live khi người vận
hành chủ động thử lại.

## Bổ sung chống dao động gimbal/body-yaw

Lần validation live tiếp theo vẫn ghi nhận mất bbox. Chuỗi 30 mẫu read-only
cho thấy tracker score ổn định 0,96–0,99 nhưng yaw PID liên tục bão hòa ở
±60°/s, limiter publish bão hòa ở ±25°/s và tâm bbox quét qua lại gần 200 px.
Gimbal và unified body yaw cùng dùng sai số ảnh trong lúc gimbal chưa chạm
±10°, tạo hai vòng điều khiển đồng thời. BBox center filter alpha 0,22 cũng tạo
residual tới khoảng 60 px, khiến dấu điều khiển đổi muộn; limiter gia tốc cũ
tiếp tục quay theo hướng cũ khi dấu đã đảo.

Thay đổi tiếp theo:

- body yaw bằng 0 khi gimbal chưa latch tại giới hạn ±10°;
- khi chạm giới hạn, body yaw mới nhận authority để unload gimbal và căn bbox;
- gimbal PID dùng `Kp=2,5`, `Ki=0`, `Kd=0,05`, trần 30°/s;
- publish limit pitch/yaw còn 15/12°/s, gia tốc 60°/s²;
- phanh 180°/s² và bắt buộc về 0 trước khi đảo chiều;
- body yaw recenter giới hạn 20°/s, slew 60°/s²;
- bbox center alpha tăng từ 0,22 lên 0,55 để giảm trễ điều khiển, trong khi gate
  bbox jump vẫn giữ nguyên.

Compileall đạt và toàn bộ 157/157 unit test đạt. Chưa restart backend để nạp
thay đổi vì lần kiểm tra cuối cho thấy cả UAV-01 và UAV-02 đang armed,
tracking UAV-02 đang active. Runtime PID 287706 vẫn là phiên bản trước thay
đổi này; không có lệnh bay, gimbal hoặc restart nào được phát.

Sau khi người vận hành xác nhận disarm, API được kiểm tra lại: cả hai UAV
`armed=false`, `failsafe=false`, tracking inactive, bbox null, forward/yaw
velocity bằng 0. Backend được restart an toàn thành PID 295646. Runtime mới
xác nhận giới hạn roll/pitch/yaw là 15/15/12°/s, gia tốc 60°/s², phanh
180°/s² và yaw gimbal vẫn giới hạn ±10°. Có đúng một Uvicorn và một
`mavlink_manual_bridge.py`; không chọn bbox, arm/takeoff hoặc phát motion test.

## Tăng tốc body yaw recenter

Theo yêu cầu vận hành, body yaw khi gimbal đã latch tại giới hạn ±10° được tăng
từ 20 lên 35°/s; slew tăng từ 60 lên 120°/s². Gimbal vẫn giữ giới hạn chậm
15/12°/s và thân drone vẫn không có yaw authority trước khi gimbal chạm biên,
do đó thay đổi không tái tạo hai vòng điều khiển đồng thời.

API bổ sung `motion_max_yaw_rate_deg_s` và `motion_yaw_slew_deg_s2`. Unit test
xác nhận profile mới tăng 0 → 12 → 24 → 35°/s trong ba chu kỳ 100 ms và không
vượt 35°/s. Compileall đạt, toàn bộ 158/158 test đạt.

Chưa restart backend để nạp profile 35°/s vì kiểm tra runtime cho thấy cả hai
UAV đang armed, UAV-02 ở Offboard với tracking active. Runtime vẫn giữ profile
20°/s cũ; không phát lệnh yaw hoặc thay đổi trạng thái UAV.

Sau khi người vận hành disarm, API xác nhận hai UAV `armed=false`,
`failsafe=false`, tracking inactive và motion bằng 0. Backend được restart
an toàn thành PID 298593. Runtime mới xác nhận body yaw tối đa 35°/s, slew
120°/s²; gimbal vẫn giới hạn yaw 12°/s và góc ±10°. Không chọn bbox,
arm/takeoff hoặc phát lệnh movement test.
