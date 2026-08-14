# Core Range Stage 2A — audit read-only các attempt bị loại (2026-08-06)

Phạm vi: audit chỉ đọc, theo mục 3 của `2026-08-06 Core Range Stage 2A Handoff`.
Không sửa mã nguồn, không train, không hạ gate, không rerun collector.

Nguồn: `artifacts/core_range_3_12m/targeted_dynamic_pilot_clean_20260806`
(4 nhóm accepted trong `raw_sessions/`, 13 attempt trong `quarantine/`).

## 0. Đính chính phương pháp

Lần tính latency đầu tiên trong phiên này đã tính trên **mọi** row của
`physical_diagnostics.jsonl`. Gate thật (`core_range_collect_targeted_pilot_batch.py:141`)
chỉ dùng row có `stage == "raw_range_computed"` và `session_id == runtime_session`.
Toàn bộ số liệu dưới đây đã tính lại đúng theo bộ lọc của gate và tái hiện
chính xác kết luận pass/fail mà collector đã ghi.

## 1. `worker_p95` — không phải nhiễu đuôi, mà là dịch cả phân phối

Gate: `percentile(worker, 95) <= 0.100` (`:225`), với
`worker = depth_complete - depth_worker_start`.

| Attempt | Trạng thái | n_raw | median | p95 | Gate |
|---|---|---:|---:|---:|---|
| mid_lateral_yaw_recede_1 | quarantine | 223 | 38.4 ms | 55.9 ms | PASS |
| mid_lateral_yaw_approach_1 | quarantine | 119 | 38.8 ms | 56.2 ms | PASS |
| **near_center_recede_2** | **accepted** | 162 | 37.7 ms | 56.5 ms | PASS |
| mid_lateral_yaw_approach_2 | quarantine | 90 | 35.7 ms | 57.9 ms | PASS |
| mid_lateral_yaw_approach_3 | quarantine | 578 | 39.3 ms | 58.7 ms | PASS |
| **near_lateral_recede_1** | **accepted** | 107 | 36.4 ms | 62.2 ms | PASS |
| mid_lateral_yaw_recede_2 | quarantine | 107 | 40.5 ms | 65.6 ms | PASS |
| **mid_center_approach_1** | **accepted** | 133 | 40.6 ms | 84.1 ms | PASS |
| mid_center_recede_3 | quarantine | 42 | 40.1 ms | 96.9 ms | PASS |
| mid_lateral_yaw_recede_3 | quarantine | 42 | 55.6 ms | 97.4 ms | PASS |
| **near_center_approach_1** | **accepted** | 127 | 42.7 ms | 97.8 ms | PASS |
| near_lateral_approach_1 | quarantine | 111 | 67.7 ms | 104.2 ms | **FAIL** |
| near_lateral_approach_2 | quarantine | 177 | 62.4 ms | 107.0 ms | **FAIL** |
| near_lateral_approach_3 | quarantine | 120 | 62.4 ms | 108.7 ms | **FAIL** |
| mid_center_recede_2 | quarantine | 198 | 55.5 ms | 113.5 ms | **FAIL** |
| near_center_recede_1 | quarantine | 372 | 74.3 ms | 134.2 ms | **FAIL** |
| mid_center_recede_1 | quarantine | 130 | 52.8 ms | 135.9 ms | **FAIL** |

Kết luận:

- Hai nhóm tách **lưỡng cực sạch**, không chồng lấn:
  - pass gate: median `35.7–42.7 ms`, p95 `55.9–97.8 ms`;
  - fail gate: median `52.8–74.3 ms`, p95 `104.2–135.9 ms`.
- Vì **median** cũng cao gấp ~1.5×, đây không phải vài frame outlier ở đuôi mà là
  toàn bộ depth worker chạy chậm hơn trong suốt run. Hạ ngưỡng p95 sẽ che mất
  một khác biệt runtime có thật, không phải sửa lỗi đo.
- Đã loại các giả thuyết sau bằng dữ liệu:
  - **Drift nhiệt/thời gian**: không đơn điệu. `near_lateral_recede_1` (17:56)
    đạt median tốt nhất `36.4 ms` ngay sau ba lần fail liên tiếp 17:52–17:55.
  - **Kích thước ROI/bbox**: run có `bbox_area` lớn nhất (`0.0708`,
    `near_center_recede_2`) lại nhanh nhất; run fail có bbox nhỏ hơn.
  - **Teardown sót process từ run trước**: run ngay sau lần phải `SIGKILL`
    (`near_center_recede_2`, 17:51) vẫn PASS với `37.7 ms`.
- **Chưa xác định được** nguyên nhân gây dịch phân phối. Cần đo thêm ở lần thu tới.
- Cảnh báo biên: `near_center_approach_1` được accept ở `97.8 ms`, chỉ dưới ngưỡng
  `2.2 ms`. Một trong bốn nhóm accepted đang nằm sát mép gate.

## 2. `gt_coverage` — handoff mô tả sai bản chất

Handoff ghi hai lỗi này là "GT coverage thiếu / chỉ tới 6.69 m thay vì 8.0 m",
hàm ý quỹ đạo không chạy hết. **Dữ liệu thô cho thấy ngược lại.**

`mid_lateral_yaw_approach_2` (`gt_coverage:7.9589:5.6110:tolerance=0.3`):

- `trajectory_events.jsonl` có `trajectory_completed` tại `range_m = 4.6`.
- GT trace đầy đủ chạy đủ `7.959 → 4.561` trong `29.8 s`
  (`4.561` lệch `0.039 m` so với đích `4.6`, **trong** dung sai `0.3`).
- Nhưng chỉ có `90 raw_range_computed`, và row cuối cùng dừng ở `5.611`.

`mid_lateral_yaw_recede_3` (`gt_coverage:4.5598:6.6930`):

- `trajectory_completed` tại `range_m = 8`; GT trace đầy đủ `4.560 → 7.960`.
- Chỉ có `42 raw_range_computed`, row cuối dừng ở `6.693`.

Nguyên nhân thật: quỹ đạo chạy đủ, nhưng **pipeline range ngừng sinh row
`raw_range_computed` ở nửa sau của run** vì frame bị chặn ở tầng calibration.
Gate đọc `gt[0]`/`gt[-1]` từ danh sách row đã bị cụt nên hiểu nhầm thành
"target không đi hết quãng đường".

Phân bố stage trong hai run đó:

- `yaw_approach_2`: `raw_range_computed 90` vs `calibration_rejected 132`
  (`ransac_consensus_failed 110`, `calibration_temporal_jump 10`).
  Theo tiến trình run, row hợp lệ **chỉ tồn tại ở khúc giữa**; đoạn đầu và đoạn
  cuối bị `ransac_consensus_failed` chiếm hoàn toàn.
- `yaw_recede_3`: `raw_range_computed 42` vs `calibration_rejected 147`
  (`calibration_temporal_jump 54`, `invalid_calibration 33`,
  `insufficient_anchors 26`, `ransac_consensus_failed 25`); row hợp lệ tắt hẳn
  từ segment 5/8 trở đi.

## 3. Hai lỗi "prewarm timeout" cùng một gốc

`mid_lateral_yaw_recede` attempt 1 và 2 (`yaw_start_deg = +15`):

| Attempt | tổng row | raw_range_computed | calibration_rejected | lý do áp đảo | GT |
|---|---:|---:|---:|---|---|
| recede_1 | 437 | 223 | 208 | `insufficient_anchors` (86) | `4.560 → 4.560` |
| recede_2 | 433 | 107 | 320 | `insufficient_anchors` (266) | `4.560 → 4.560` |

GT đứng yên ở `4.560` suốt cả hai run: **motion chưa từng khởi động**. Prewarm
không gom đủ cửa sổ row ổn định nên hết thời gian chờ trước khi phát lệnh chạy
quỹ đạo. Đây không phải lỗi camera/logging startup mà là hệ quả của tỉ lệ
rejection calibration cao ngay tại yaw khởi đầu `+15°`.

## 4. Lỗi "24/40 raw frame" và "logging audit blocked"

- `yaw_approach_3`: tổng cộng có `578 raw_range_computed` phủ đủ `7.959 → 4.562`,
  nhưng `285 calibration_rejected` (`ransac_consensus_failed 185`) làm row hợp lệ
  thưa trong cửa sổ đếm của gate → script thoát mã 1.
- `yaw_approach_1`: dữ liệu **gần như sạch** — `149 raw_range_computed`, phủ đủ
  `7.959 → 4.561`, chỉ `20` rejection (`11.4%`). Nó bị chặn bởi gate sổ sách
  `single_raw_session_group` trong `smoke_report.md`, không phải do chất lượng
  dữ liệu. Đây là attempt đáng cứu nhất trong nhóm quarantine.

## 5. Root cause: độ cao camera ước lượng, không phải yaw

### 5.1. Bác bỏ giả thuyết yaw

Thống kê thô ban đầu cho thấy scenario yaw có tỉ lệ rejection `49.7 %` so với
`28.3 %` ở scenario không yaw, và cả 6 attempt yaw đều fail. **Đây là tương quan
giả.** Hai attempt không yaw cũng hỏng nặng (`mid_center_recede_3` `88.4 %`,
`near_center_recede_1` `53.4 %`), và khi sắp xếp theo biến thật ở mục 5.2 thì
yaw không còn sức giải thích: các run yaw chỉ tình cờ rơi vào vùng độ cao thấp.

### 5.2. Biến thật: `camera_position_ned_m[2]`

Sắp xếp mọi attempt theo độ cao camera so với mặt phẳng đất giả định
(`ground_down_m = 0.0`) cho quan hệ **đơn điệu**:

| Attempt | yaw | cam_alt | anchor (median) | frame < 12 anchor |
|---|---|---:|---:|---:|
| mid_center_recede_3 | – | 0.074 m | 0 | 52.9 % |
| mid_lateral_yaw_recede_2 | YAW | 0.125 m | 0 | 61.4 % |
| far_center_approach_1 | – | 0.129 m | 2 | 80.0 % |
| mid_lateral_yaw_recede_3 | YAW | 0.328 m | 28 | 12.8 % |
| mid_lateral_yaw_approach_2 | YAW | 0.412 m | 38 | 0.0 % |
| mid_lateral_yaw_recede_1 | YAW | 0.486 m | 52 | 19.7 % |
| near_center_recede_1 | – | 0.512 m | 56 | 18.0 % |
| mid_lateral_yaw_approach_3 | YAW | 0.591 m | 60 | 6.3 % |
| **mọi attempt có cam_alt ≥ 0.640 m** | – | 0.64–1.69 m | **60** | **0.0 %** |

### 5.3. Cơ chế

`m52_adapter.py:189`:

```python
ray_range = (float(ground_down_m) - camera[2]) / ray[2]
if not self.minimum_range_m <= ray_range <= self.maximum_range_m:
    reject("ground_range_outside_limits")
```

Với `ground_down_m = 0.0`, biểu thức rút gọn thành `ray_range = h / sin(θ)`,
trong đó `h` là độ cao camera và `θ` là góc chúc của tia. Anchor bị loại khi
`ray_range < minimum_range_m = 1.0`, tức khi `sin(θ) > h`.

Camera: `fx = fy = 299.84`, `cy = 135` → nửa FOV dọc `24.24°`. Gimbal pitch
`9.74–11.69°` → tia chúc sâu nhất `33.98–35.93°`. Vậy độ cao tối thiểu để
**không** tia nào bị loại là `sin(35°) ≈ 0.56–0.59 m`.

Kiểm chứng mô hình `frac = asin(h) / θ_max` với anchor quan sát được:

| h | anchor dự đoán | anchor quan sát |
|---:|---:|---:|
| 0.074 m | 7 | 0 |
| 0.129 m | 13 | 2 |
| 0.328 m | 33 | 28 |
| 0.412 m | 42 | 38 |
| 0.486 m | 50 | 52 |
| 0.512 m | 53 | 56 |
| 0.591 m | 60 | 60 |
| ≥ 0.64 m | 60 | 60 |

Ngưỡng lý thuyết `0.559–0.587 m` nằm **đúng giữa** hai giá trị quan sát
`0.512 m` (đã suy giảm) và `0.591 m` (đầy đủ). Mô hình hình học giải thích
trọn vẹn dữ liệu.

### 5.4. Vì sao độ cao thay đổi khi UAV không cất cánh

Cả hai UAV `disarmed` trước và sau mọi capture, nhưng `camera_position_ned_m[2]`
dao động từ `0.074 m` đến `1.686 m` giữa các attempt. Đây là ước lượng vị trí
local của PX4, được khởi tạo lại sau mỗi lần restart stack và trôi theo từng
phiên. Nửa sau của batch (sau 18:00) trôi xuống thấp rõ rệt, kéo theo toàn bộ
cụm scenario yaw — đó chính là nguồn gốc của tương quan giả ở mục 5.1.

Hệ quả: kết quả thu thập phụ thuộc vào một đại lượng **không được kiểm soát và
không được gate kiểm tra**. Cùng một scenario có thể pass hay fail chỉ tùy vào
ước lượng độ cao mà PX4 tình cờ khởi tạo cho lần chạy đó.

## 6. Kết luận

Ba nhóm nguyên nhân độc lập:

1. **Anchor starvation do độ cao camera** — gốc rễ thật của mọi lỗi
   `gt_coverage`, prewarm timeout và thiếu raw frame. Đã xác định dứt điểm bằng
   mô hình hình học khớp dữ liệu. **Không liên quan tới yaw.**
2. **`worker_p95`** — depth worker chậm ~1.5× ở một số run, nguyên nhân **chưa**
   xác định. Chưa đủ cơ sở để sửa code.
3. **Gate sổ sách** — `single_raw_session_group` loại `yaw_approach_1` dù dữ
   liệu sạch.

### Precondition đã triển khai

`ground_anchor_readiness()` trong `core_range_dynamic_capture.py`, gọi ngay sau
khi prewarm ổn định và **trước** khi chọn bbox/chạy quỹ đạo, nên attempt hỏng
thoát sớm thay vì tiêu hết ngân sách prewarm + motion.

Điều kiện: `median(accepted_ground_anchor_count) ≥ 3 × minimum_anchors` **và**
tỉ lệ frame dưới `minimum_anchors` `≤ 5 %`. `minimum_anchors` đọc thẳng từ
`MetricDepthCalibrator()` nên không trôi khỏi calibrator.

**Đã đổi thiết kế sau khi kiểm chứng.** Bản đầu gate bằng ngưỡng hình học
`h ≥ 1.10 × minimum_range_m × sin(θ_max)`. Chạy đối chứng cho thấy nó **chặn
nhầm run tốt**: `attempt_3` ở `0.529 m` giữ 56/60 anchor, 0 % frame đói,
rejection 2.7 % — nhưng ngưỡng hình học đòi `0.635 m`. Ở `h = 0.529 m` chỉ mất
~9 % tia, còn cách rất xa sàn 12 anchor.

Hình học đúng để **chẩn đoán** nguyên nhân, nhưng sai để **làm gate**: nó đòi
mọi tia sống sót, trong khi thứ thực sự quan trọng là còn đủ anchor. Gate cuối
đo thẳng số anchor, không dự đoán từ độ cao — không phụ thuộc pitch, ống kính
hay nội dung cảnh.

Phạm vi: gate này xử lý `insufficient_anchors`. Nó **không** xử lý
`ransac_consensus_failed` — `yaw_approach_2` có 38 anchor, 0 % frame đói nhưng
vẫn hỏng vì 110 lần RANSAC không đạt đồng thuận. Đó là cơ chế khác, chưa xử lý.

Kiểm chứng: 21 test trong `test_ground_anchor_readiness_precondition.py`, gồm
một test tham số hoá chạy trên số anchor **đo thật** của 13 attempt Stage 2A và
4 run chẩn đoán. Regression: 100 test pass trên các module liên quan.

- Không đổi `minimum_range_m` hay `minimum_anchors`: chúng đang hoạt động đúng
  thiết kế; vấn đề nằm ở điều kiện đầu vào chứ không ở ngưỡng.

---

## 8. Độ cao PX4 và sai số range — giả thuyết ĐÃ BỊ BÁC BỎ

> **Đọc mục 8.7 trước.** Các mục 8.1–8.6 ghi lại một giả thuyết đã được kiểm
> chứng và **bác bỏ**. Chúng được giữ nguyên để lưu vết suy luận, không phải để
> kết luận.


### 8.1. Hai nguồn pose khác nhau trong cùng một record

| Trường | Nguồn | Giá trị điển hình |
|---|---|---|
| `ground_truth.provider_diagnostics.camera_drone_center_xyz[2]` | **Gazebo (sự thật)** | `1.03 m`, ổn định mọi run |
| `extrinsics.camera_position_ned_m[2]` | **PX4 EKF (ước lượng)** | `0.375 → 2.833 m` tuỳ run |

Nhãn ground truth dùng pose Gazebo nên **đúng**. Nhưng hình học anchor và range
dùng `extrinsics`, tức ước lượng PX4.

### 8.2. PX4 không hội tụ khi drone nằm đất, disarmed

Gazebo luôn đặt drone quan sát ở đúng `z = 1.0`
(`core_range_collect_targeted_pilot_scenario.sh:159`). Ước lượng PX4 lang thang,
có lúc **âm** — tức camera bị cho là nằm dưới mặt đất:

```
STARVED:  0.288 -> -0.123 -> -0.488 -> -0.433 ->  0.309 -> 0.503
STARVED:  0.059 ->  0.105 ->  0.374 ->  1.238 ->  0.159 -> 0.024
HEALTHY:  1.231 ->  1.298 ->  1.490 ->  1.539 ->  1.746 -> 1.774
```

Sai số so với sự thật `1.0 m`: từ `-1.49 m` đến `+1.83 m`.

### 8.3. Cơ chế

`m52_adapter.py:189` tính độ sâu anchor mặt đất bằng
`(ground_down_m - camera_z) / ray_down`, tức **tỉ lệ thuận với độ cao PX4**.
`MetricDepthCalibrator` fit `1/depth = a·q + b` trên các anchor đó, nên scale
thừa hưởng nguyên sai số độ cao, và range mục tiêu tỉ lệ theo scale.

### 8.4. Bằng chứng định lượng

Đặt `alt_ratio = px4_alt / true_alt` và `range_ratio = raw_range / gt_range`:

| alt_ratio | range_ratio |
|---:|---:|
| 0.36 | 0.417 |
| 0.47 | 0.385 |
| 0.59 | 0.986 |
| 1.07 | 1.019 |
| 1.43 | 2.361 |
| 1.65 | 3.313 |
| 2.74 | 4.997 |

- **Giữa các run: Pearson r = 0.897** (n = 22 run).
- **Trong từng run: r = +0.53 đến +0.96** ở 5/6 run kiểm tra. Run còn lại
  (`r = -0.515`) nằm ở vùng bão hoà `alt_ratio 2.5–2.97`, `range_ratio ≈ 5`.

### 8.5. Ý nghĩa

Sai số range thô bị chi phối bởi sai số pose của PX4, **không** phải bởi sai số
mô hình độ sâu. Điều đó soi lại một loạt kết quả cũ:

- `signed_bias = -2.949 m` trong `smoke_report.md` khớp với chế độ `alt_ratio < 1`;
- loạt candidate XGBoost residual bị quarantine vì không tổng quát hoá — chúng
  đang cố hồi quy một bias mà biến gây nhiễu thật sự là pose trôi theo từng run;
- cùng một scenario cho range khác nhau giữa các lần chạy.

**Huấn luyện trên corpus hiện tại sẽ học luôn độ trôi của PX4.** Việc thu đủ
14/14 không giải quyết được điều này — nó chỉ thu thêm dữ liệu mang cùng khuyết tật.

### 8.6. Mức độ chắc chắn và bước kiểm chứng quyết định

Đã chắc: liên hệ thống kê mạnh ở cả hai mức, và cơ chế cụ thể truy được tới
dòng code. Chưa chắc: **quan hệ nhân quả chưa được chứng minh bằng can thiệp** —
`alt_ratio ≈ 1` vẫn có run cho `range_ratio` lệch (0.935 đến 2.42), nên độ cao
là yếu tố chi phối chứ không phải yếu tố duy nhất.

Kiểm chứng quyết định, **offline, không cần thu lại**: replay corpus đã có,
thay `camera_position_ned_m[2]` bằng độ cao thật của Gazebo, fit lại calibration
và đo lại range. Nếu sai số sụt mạnh thì nhân quả được xác lập.

Lưu ý: dùng pose Gazebo chỉ hợp lệ để **chẩn đoán**. Nó không phải giải pháp
triển khai — bay thật không có sự thật đó. Giải pháp thật phải là nguồn độ cao
đáng tin (EKF hội tụ, rangefinder, hoặc lidar sẵn có trên drone).

### 8.7. KẾT QUẢ CAN THIỆP — giả thuyết bị bác bỏ

Nếu mọi anchor depth nhân `k` thì cả `a` và `b` trong fit `1/depth = a·q + b`
đều chia `k`, nên range nhân đúng `k`. Đây là hệ quả **giải tích chính xác**,
nên hiệu chỉnh độ cao tương đương nhân range đã ghi với `k = true_alt / px4_alt`
— không cần refit, không dính trạng thái calibrator, không dính logic cache.

Kết quả trên 22 run:

| | median \|sai số\| |
|---|---:|
| Ghi nhận | **3.045 m** |
| Sau hiệu chỉnh độ cao | **3.023 m** |

15/22 run "cải thiện" nhưng median đứng yên, và phân bố cho thấy đây là quy về
trung bình chứ không phải sửa lỗi:

| Run | trước | sau |
|---|---:|---:|
| `attempt_3` | 0.489 | **4.320** |
| `control_near_lateral_recede` | 0.384 | **3.544** |
| `near_center_recede_1` | 1.113 | **4.763** |
| `control_profiled_recede` | 14.516 | 3.683 |
| `near_lateral_approach_3` | 8.146 | 4.026 |

Những run vốn **chính xác nhất** trở nên sai nặng sau hiệu chỉnh. Nghĩa là độ
cao sai đã tình cờ **triệt tiêu** một sai số nền, chứ không gây ra nó.

Kết luận đúng:

- Cơ chế `range ∝ độ cao giả định` là có thật và tất yếu về số học.
- Nhưng độ cao PX4 **không** phải nguồn sai số chính. Sau hiệu chỉnh, sai số vẫn
  ~3 m trên dải mục tiêu 3–12 m.
- Tồn tại một sai số **độc lập, lớn (~3 m)** trong tầng depth/calibration, khớp
  với `signed_bias = -2.949 m` đã ghi trong `smoke_report.md` từ trước.
- Tương quan `r = 0.897` là thật nhưng **không phải nhân quả theo hướng giả định**.

Công cụ: `core_range_pose_substitution_replay.py` (replay refit đầy đủ; lưu ý
độ trung thực hạn chế vì pipeline dùng calibration có cache `cache_ttl_s = 2.5`,
nên phép thử giải tích ở trên mới là căn cứ).

Điều vẫn đúng: ước lượng độ cao PX4 thật sự hỏng, **có** gây anchor starvation
(mục 5) và **có** thêm phương sai cho range. Chỉ là nó không phải nút thắt chính.

Corpus hiện tại vẫn **không đạt 14/14**. Giữ nguyên trạng thái: chưa train,
chưa audit tổng, chưa freeze model.

---

## 7. Điều tra `worker_p95` (2026-08-06, buổi tối) — CHƯA KẾT LUẬN

Sau khi người dùng khôi phục `px4-linux.img`, đã chạy 5 run chẩn đoán bổ sung.
Storage được xác minh lành trước khi chạy: 4 file then chốt đọc đủ byte, và quét
tuần tự `Tools/simulation`, `bin`, `rootfs`, `etc` cho **0 lỗi đọc**.

### 7.1. Sáu giả thuyết đã bị bác bỏ bằng đo đạc

| Giả thuyết | Bằng chứng bác bỏ |
|---|---|
| Storage hỏng gây chậm | Storage lành, `near_lateral_approach` vẫn fail (74.4 ms, tệ hơn sáng) |
| Residual GIL (fix 2026-08-05) | 96 % thời gian nằm ở stage GPU-adjacent, không ở CPU-side stage |
| Gazebo tranh chấp GPU | `nvidia-smi`: util median 0 %, p95 4 %, chỉ 1 process compute, mem 384 MiB |
| Nội dung khung hình / khoảng cách | Latency **phẳng** theo mọi bin khoảng cách trong cả hai run |
| Frame vào khác kích thước | `frame_copy_decode_s` giống hệt (0.84 vs 0.82 ms); camera dims `480×270` cả hai |
| Throttle xung nhịp GPU | Stage GPU **ngắn** không đổi (0.30 → 0.31 ms); throttle thì mọi op phải chậm theo |
| `approach` chậm / `recede` nhanh | Bảng thời gian đầy đủ cho thấy **cả hai loại đều hỗn hợp** (xem 7.4) |

### 7.2. Phân rã sub-stage (profiling ON cả hai phía)

| sub-stage | RECEDE (nhanh) | APPROACH (chậm) | tỉ lệ |
|---|---:|---:|---:|
| `host_to_device_transfer_s` | 19.45 ms | 38.23 ms | **1.97×** |
| `model_forward_s` | 17.21 ms | 25.42 ms | **1.48×** |
| `numpy_postprocess_s` | 1.20 ms | 1.57 ms | 1.31× |
| `frame_copy_decode_s` | 0.84 ms | 0.82 ms | 0.97× |
| `gpu_preprocess_resize_normalize_s` | 0.31 ms | 0.30 ms | 0.94× |
| `device_to_host_transfer_s` | 0.22 ms | 0.22 ms | 1.01× |
| `output_resize_interpolation_s` | 0.12 ms | 0.12 ms | 1.01× |

Chỉ stage **dài** phồng, stage **ngắn** đứng yên — gần tỉ lệ với độ dài. Đây là
dấu hiệu thread bị deschedule (stage càng dài càng dễ trúng), không phải GPU chậm.

**Cảnh báo về phép đo:** `_sync()` chỉ chạy khi bật profiling
(`depth_model_adapter.py:216`), nên `cuda.synchronize()` sau mỗi stage tuần tự
hoá pipeline và thổi phồng số tuyệt đối. Bằng chứng: cùng scenario cho 74.4 ms
(profiling on) so với 62–68 ms (profiling off). Chỉ dùng **tỉ lệ** giữa hai cột,
không dùng số tuyệt đối.

### 7.3. Môi trường đo không được kiểm soát

Lấy mẫu CPU 1 Hz trong một run chậm, máy 20 core:

```
loadavg      min 4.11   median 7.07   max 18.97
python       avg 361%   peak 1900%
px4          avg  86%
claude       avg  72%   peak  309%     <- tiến trình phân tích của trợ lý
ruby         avg  69%
kswapd0      avg  39%   peak   80%     <- áp lực bộ nhớ, kernel đang reclaim
firefox / gnome-shell / Xorg   27% / 21% / 19%
```

`kswapd0` hoạt động mạnh nghĩa là hệ thống thiếu bộ nhớ và kernel phải reclaim —
đúng loại stall làm thread bị deschedule. Các phiên thu Stage 2A và mọi run chẩn
đoán đều diễn ra trên máy có desktop, trình duyệt và tiến trình phân tích chạy
song song. **Gate 100 ms đang được chấm trên một máy không tĩnh.**

### 7.4. Lỗi thiết kế thí nghiệm

Toàn bộ 17 run sáng 2026-08-06, theo trục thời gian:

```
17:46 near_center_approach_1    42.7 FAST     17:57 mid_center_approach_1  40.6 FAST
17:50 near_center_recede_1      74.3 SLOW     17:58 mid_center_recede_1    52.8 SLOW
17:51 near_center_recede_2      37.7 FAST     18:00 mid_center_recede_2    55.5 SLOW
17:52 near_lateral_approach_1   67.7 SLOW     18:02 mid_center_recede_3    40.1 FAST
17:54 near_lateral_approach_2   62.4 SLOW     18:04 yaw_approach_1         38.8 FAST
17:55 near_lateral_approach_3   62.4 SLOW     18:05 yaw_approach_2         35.7 FAST
17:56 near_lateral_recede_1     36.4 FAST     18:09 yaw_approach_3         39.3 FAST
                                              18:13 yaw_recede_1           38.4 FAST
                                              18:15 yaw_recede_2           40.5 FAST
                                              18:17 yaw_recede_3           55.6 SLOW
```

Mọi run chậm buổi sáng nằm trong cửa sổ **17:50–18:00**; từ 18:02 trở đi đều
nhanh trừ run cuối. Điều đó nghiêng về **giai đoạn môi trường**, không phải
scenario. Nhưng buổi tối, thứ tự approach/approach/recede/recede/approach cho
chậm/chậm/nhanh/nhanh/**chậm** — run cuối phá vỡ tính liên tục thời gian, nghiêng
về scenario.

Hai phiên cho hai kết luận trái nhau vì **cả hai đều chạy theo khối**. Thiết kế
khối không tách được hiệu ứng thời gian khỏi hiệu ứng scenario. Đây là lỗi thiết
kế, không phải phát hiện.

### 7.5. Việc cần làm

Chạy **xen kẽ A/B/A/B/A/B** giữa `pilot_near_lateral_approach` và
`pilot_near_lateral_recede`, trên máy đã quiesce:

- đóng trình duyệt và ứng dụng desktop; xác nhận `kswapd0` không hoạt động;
- không chạy tiến trình phân tích nào trong lúc thu;
- ghi `loadavg` và CPU theo tiến trình suốt cả 6 run;
- profiling **tắt** (tránh nhiễu đo), so bằng `worker` median/p95.

Diễn giải:

- chênh lệch bám theo **scenario** dù đã xen kẽ → thuộc về scenario, đào tiếp;
- chênh lệch bám theo **thời gian** → nhiễu môi trường, và vấn đề nằm ở cách vận
  hành thu thập chứ không ở code;
- chênh lệch **biến mất** trên máy tĩnh → `worker_p95` chưa bao giờ là lỗi phần
  mềm, và cần thu lại toàn bộ Stage 2A trên máy tĩnh.

Chưa đủ căn cứ để sửa code cho `worker_p95`. Precondition độ cao camera ở mục 6
vẫn đứng độc lập và vẫn nên làm.

---

## 9. Định vị được sai số nền ~3 m: calibration anchor không chuyển sang target

Sau khi loại giả thuyết pose (mục 8.7), câu hỏi còn lại là vì sao sai số vẫn
~3 m khi hình học đúng. Hai giả thuyết tiếp theo cũng đã bị bác bỏ bằng dữ liệu:

- **Ngoại suy ngoài miền anchor**: `r = +0.013` giữa mức ngoại suy (đơn vị IQR)
  và sai số, trên 22 run. Không giải thích được, dù 46.4% frame có target nằm
  ngoài span anchor.

### 9.1. Chỗ sai số thật sự nằm

Đặt `q_needed = (1/d_true - b) / a`, tức giá trị inverse depth mà target **phải**
có để quan hệ affine đã fit trên anchor cho ra đúng khoảng cách thật của nó:

| Run | `q_target / q_needed` | sai số depth |
|---|---:|---:|
| `control_profiled_recede` | 0.308 | +14.08 m |
| `near_center_recede_2` | 0.474 | +4.68 m |
| `near_center_approach_1` | 0.524 | +4.25 m |
| `near_lateral_recede_1` | 0.549 | +4.84 m |
| `attempt_2` | 0.653 | +3.60 m |
| `cpu_probe_approach` | 0.655 | +3.31 m |
| `attempt_3` | 0.994 | +0.04 m |
| `control_near_lateral_recede` | 1.011 | −0.05 m |
| `mid_center_approach_1` | 1.920 | −3.06 m |

Median `0.653`. Quan hệ với sai số gần như một-một: tỉ số lệch khỏi 1.0 bao
nhiêu thì sai số theo bấy nhiêu.

**Quan hệ affine fit trên anchor mặt đất không chuyển đúng sang target.** Đây là
sai số nền, không phải pose, không phải ngoại suy.

Hai run có tỉ số ≈ 1.00 (`attempt_3`, `control_near_lateral_recede`) chính là hai
run có độ cao PX4 sai gần một nửa — củng cố kết luận mục 8.7 rằng sai số độ cao
đã triệt tiêu sai số này chứ không gây ra nó.

### 9.2. Vì sao residual của chính fit cũng đáng ngờ

Residual affine trên chính các anchor: `0.007–0.097 m⁻¹`, trong khi ngưỡng
RANSAC là `residual_threshold_m_inv = 0.025`. Nhiều run vượt ngưỡng, tức quan hệ
affine không mô tả tốt ngay cả tập anchor.

### 9.3. Hai nguyên nhân ứng viên, chưa phân định

1. **Lệch thống kê lấy mẫu**: anchor dùng `local_inverse_depth_mean` trên vùng
   3×3 (`local_sample_count = 9`); target dùng trung vị foreground trên cả ROI
   (`sample_count ≈ 1226`). Hai ước lượng khác nhau về bản chất.
2. **MiDaS không nhất quán theo không gian/nội dung**: depth tương đối chỉ xác
   định tới một phép affine *trên lý thuyết*; thực tế ánh xạ đổi theo vùng ảnh
   và theo nội dung, nên quan hệ fit trên mặt đất không áp được cho vật thể.

Nếu là (2) thì đây là **giới hạn phương pháp**, không phải lỗi triển khai, và
toàn bộ hướng "fit affine trên anchor mặt đất rồi áp cho target" cần xem lại —
kể cả tầng residual XGBoost, vốn đang cố hồi quy chính bias này (`0.31–1.92`,
thay đổi theo từng run, nên không tổng quát hoá được).

---

## 10. Model học M52 (2026-08-07) — hiện trạng cuối và vùng rủi ro đã biết

Model `with_bbox`/`size_agnostic` (`core_range_stage2a_relative_range_model.py`)
đã wire vào `metric_target_fusion.py` sau cờ `SWARM_METRIC_TARGET_RANGE_MODEL_ENABLED`
(mặc định **tắt**). Kiểm chứng trên 4 capture thật (exit code đã xác minh từng
lần): MAE tổng hợp 3.07 → 1.03 m (+66.6%), Spearman đảo dấu đúng ở cả 4 nhóm.

### 10.1. Giới hạn đã đo: slope trong-kịch-bản thấp

Model bám đúng "vùng khoảng cách" nhưng phản ứng yếu với thay đổi trong một
kịch bản (median slope trong-nhóm ~0.26–0.28, lý tưởng là 1.0). 3/12 nhóm có
slope **âm** — model báo hướng ngược với thực tế:

- `pilot_far_lateral_yaw_recede`
- `pilot_mid_lateral_yaw_approach`
- `pilot_near_lateral_recede`

Điểm chung: cả ba đều có `|lateral_offset_m| = 0.75`, không có mẫu hình sạch
nào khác phân biệt được với các nhóm lateral khác vẫn ổn (ví dụ
`far_lateral_yaw_approach` cùng lateral offset nhưng slope dương).

### 10.2. Bốn hướng vá đã thử, cả bốn đều thất bại hoặc gây hại

| Hướng | Kết quả |
|---|---|
| Thêm đặc trưng chuyển động | Không có tín hiệu (đã reasoned trước khi chạy) |
| Làm mượt theo thời gian (moving-avg/EMA) | Làm tệ hơn — slope 0.263→0.171 khi cửa sổ tăng |
| Đổi mục tiêu train sang `rank:pairwise` | Tệ hơn nhiều — MAE 0.69→2.78m, số nhóm slope âm 3→4 |
| Cross-check với `physics_slant_range_m` | Không độc lập (chung gốc MiDaS/M52) — đồng thuận sai đúng ở 3 nhóm nguy hiểm, bất đồng ở nhiều nhóm đang ổn |

Bốn kỹ thuật độc lập cùng hội tụ về một kết luận: đây là **giới hạn dữ liệu**
(12 nhóm, mỗi tổ hợp band×context×hướng đúng 1 lần lặp), không phải lỗi kỹ
thuật train sửa được bằng feature/objective/hậu xử lý.

### 10.3. Safety net đã triển khai — giảm rủi ro, không sửa gốc rễ

`stage2a_range_model.is_known_risk_geometry()`: gắn cờ khi `|image_ray_x| ≥ 0.13`
(mục tiêu lệch xa khỏi tâm ảnh theo chiều ngang). Ngưỡng suy từ dữ liệu thật,
**không** dùng `image_ray_y` (luôn ~0.18–0.20 ở mọi nhóm do gimbal pitch cố
định — dùng nó gây cờ dương giả 100% mọi nhóm, đã phát hiện và sửa).

Kiểm chứng trên corpus: gắn cờ đúng 100%, 100%, 92.5% ở ba nhóm nguy hiểm,
**0%** ở toàn bộ 9 nhóm còn lại — không có dương giả nào trên dữ liệu đã có.

Khi cờ kích hoạt, `raw_range_std` bị nhân `KNOWN_RISK_RANGE_STD_MULTIPLIER = 2.5`
trong `metric_target_fusion.py`, tận dụng cơ chế uncertainty-driven gating có
sẵn (`FollowTargetQualityGate`, EKF covariance) thay vì thêm luồng logic mới.
Đây là làm giảm tin tưởng một cách trung thực, **không phải** sửa được độ
chệch — 3 nhóm đó vẫn sai hướng khi model được dùng.

Test: `test_stage2a_range_model.py` (10 test, gồm 1 test grounded trực tiếp
trên corpus thật). Regression: 65 test pass trên toàn bộ module liên quan.

### 10.4. Việc còn lại nếu muốn sửa gốc rễ

Không có lối tắt kỹ thuật nào khác trên dữ liệu hiện có. Cần một trong hai:

- Thu thêm lần lặp Stage 2A, ưu tiên các tổ hợp lateral+yaw, để model có đủ
  mẫu học độ nhạy trong-kịch-bản.
- Dữ liệu bay thật (drone quan sát thực sự di chuyển) để `multiview`
  (bearing-triangulation) có baseline hoạt động — nguồn độc lập thật sự duy
  nhất trong kiến trúc, không chia sẻ lỗi với chuỗi MiDaS/M52. Yêu cầu arm +
  offboard control, vượt khỏi quy ước "observation-only" nhất quán của toàn
  dự án — cần quyết định riêng, chưa thực hiện trong phiên này.

### 10.5. Thử nghiệm bổ sung (2026-08-07, muộn hơn) — regularization/hyperparameter

Giả thuyết: nếu XGBoost bị ép quá chặt (`max_depth=4, reg_lambda=2.0`), nới lỏng
có thể giảm hiện tượng co về trung bình. Quét từ nhẹ tới tắt hẳn regularization:

| Cấu hình | MAE | median slope | số nhóm âm |
|---|---:|---:|---:|
| Hiện tại (`depth=4, λ=2.0`) | 0.820 | 0.183 | 3 |
| `λ=0.1` | 0.827 | 0.238 | 4 |
| `depth=8, λ=0.1` | 0.748 | 0.165 | 3 |
| `depth=10, λ=0, n=500` | 0.780 | 0.131 | 4 |
| `depth=15, λ=0, n=800, lr=0.1` | 0.788 | 0.168 | 4 |

Slope không cải thiện dù nới lỏng gần hết mức. Đây là hướng thứ 6 (sau đặc
trưng chuyển động, làm mượt, ranking objective, cross-check physics, thu
thêm dữ liệu lặp lại) đều không sửa được vấn đề. Kết luận cuối: đặc trưng
đầu vào không mang đủ tín hiệu phân biệt khoảng cách trong-kịch-bản — không
phải hạn chế của thuật toán train, dù linh hoạt tới đâu.

### 10.6. Cải thiện thật tìm được (2026-08-07, muộn) — tăng trọng số nhóm lateral offset

Hai ý tưởng mới thử, khác 6 hướng trước:

1. **Thêm `ray_scale = sqrt(1+x²+y²)` tường minh** — không giúp (median slope
   0.183→0.200, MAE tệ nhẹ hơn). XGBoost đã tự học được quan hệ này từ
   `image_ray_x`/`image_ray_y` sẵn có, thêm tường minh không bổ sung thông tin.
2. **Tăng trọng số ×5 cho các nhóm `|lateral_offset_m| > 0.1` lúc train** —
   có tác dụng thật, không phải nhiễu.

Kết quả trên model production (`with_bbox`, corpus 14 nhóm):

| Nhóm nguy hiểm | slope trước | slope sau |
|---|---:|---:|
| `far_lateral_yaw_recede` | −0.670 | **−0.264** (giảm ~60% độ lớn sai) |
| `mid_lateral_yaw_approach` | −0.371 | **−0.115** (giảm ~69%) |
| `near_lateral_recede` | −0.459 | −0.335 (giảm ~27%) |

MAE tổng thể không đổi đáng kể (0.820→0.817m). Một số nhóm trung tâm còn
cải thiện thêm. Đã áp dụng vào model production (`train_final_model` +
`evaluate` trong `core_range_stage2a_relative_range_model.py`, hàm
`lateral_offset_weights()`).

**Vẫn không phải fix hoàn chỉnh**: cả ba nhóm vẫn slope âm, chỉ giảm mức độ
sai chứ chưa đảo được chiều. Với `size_agnostic` (không dùng bbox), hiệu ứng
yếu và trộn lẫn hơn (2/3 nhóm cải thiện nhẹ, 1 nhóm tệ đi) — cải thiện này
gắn liền với model có bbox, không phải hiệu ứng chung của mọi feature set.

Khuyến nghị: **giữ nguyên** kết luận mục 10.4 — sửa triệt để vẫn cần dữ liệu
khác loại. Đây là một mitigation thật, đáng giữ, nhưng không thay đổi kết
luận tổng thể.
