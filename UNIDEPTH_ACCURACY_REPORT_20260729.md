# Báo cáo tối ưu độ chính xác UniDepth V2

Ngày kiểm tra: 2026-07-29

## Kết quả triển khai

Đã thay cách lấy median đơn giản trong tâm bbox bằng estimator ROI robust,
tách rõ optical Z/radial/surface/target-center, thêm kiểm tra camera geometry,
confidence/uncertainty, bbox drift, background contamination, calibration
profile có khóa camera/model, temporal filter ít trễ, bù latency có giới hạn,
ground-truth recorder đồng bộ timestamp Gazebo và công cụ đánh giá offline.

Ground truth không xuất hiện trong control path. Khi metric-depth follow bật,
controller vẫn chỉ nhận `unidepth_v2`; estimate invalid/stale bị chặn và
forward velocity về 0, không fallback LUT/bbox/lidar.

## Nguyên nhân estimate cũ dễ sai

- Median vùng tâm vẫn có thể chứa phần lớn bầu trời/mặt đất qua các khoảng rỗng
  của drone.
- Optical Z từng dễ bị diễn giải như khoảng cách tia nhìn; hai đại lượng chỉ
  bằng nhau ở principal point.
- Bbox drift/redetect và depth distribution đa mode chưa được gate đủ mạnh.
- Confidence của UniDepth V2 thực tế là predicted uncertainty (thấp tốt hơn);
  nếu dùng ngược dấu sẽ ưu tiên pixel xấu.
- Camera intrinsics, resize/crop và timestamp pose/frame không được biểu diễn
  đủ rõ để hiệu chuẩn đúng.
- EMA alpha thấp giảm jitter nhưng có thể che một lần vượt biên 8 m/10 m.

## Camera geometry

Cấu hình mục tiêu 640x360, HFOV 2.0 rad, square pixels cho:

- `fx = fy = 205.469637 px`
- `cx = 320`, `cy = 180`
- VFOV suy ra `1.438840 rad` (`82.4394 deg`)

`CameraIntrinsics.transformed()` scale lại `fx/fy/cx/cy` sau crop/resize.
API hiển thị matrix, FOV, kích thước, nguồn và trạng thái calibration. Profile
calibration bị từ chối nếu model, resolution hoặc intrinsics không khớp.

UniDepth V2 cài trong môi trường trả `depth = points[:, -1:]` (optical Z) và
`radius = norm(points)` (radial distance). `points`, `intrinsics` và
`confidence` được lấy nếu model cung cấp.

## Thuật toán ROI mới

1. Clamp bbox và reject nếu hơn 10% nằm ngoài frame.
2. Inset ROI, dùng ellipse lõi và loại pixel NaN/Inf/không dương.
3. Dùng predicted uncertainty theo quantile/absolute threshold.
4. Tách các mode theo depth gap, chọn foreground cluster gần nhất có đủ pixel.
5. Dùng uncertainty-weighted median; không để background mode lớn giành kết
   quả.
6. Reject theo valid fraction, uncertainty, MAD, IQR, background fraction,
   tracker score, bbox drift và range-rate vật lý.
7. Xuất p10/p20/p50/p80/p90, MAD/IQR, accepted pixels, background fraction và
   reject reason.

`legacy_center_median_m` được tính song song trên cùng depth map chỉ để đánh
giá trước/sau, không có motion authority.

## Calibration và filter

`metric_depth_evaluation.py` chia calibration/validation theo capture
condition (distance/angle/bbox-size), tránh rò các frame tĩnh gần như giống
nhau sang hai tập. Fit dùng median cân bằng của từng condition, so sánh:

- none;
- scale-only;
- affine;
- piecewise-linear.

Tool đồng thời so sánh none, EMA 0.30, EMA 0.65, median-3 và
confidence-weighted EMA, gồm MAE/RMSE, estimated delay và số lần filter làm
khác trạng thái safe-band. Profile sinh ra được version hóa và khóa theo
camera/model; không tự học trong lúc bay. Affine scale không dương và
piecewise không đơn điệu bị loại khỏi lựa chọn vì đảo dấu điều khiển.

Cấu hình mặc định đề xuất giữ calibration và ground-truth debug **tắt** cho
đến khi có validation dataset. Filter production là confidence-weighted EMA
alpha 0.65. Một sample valid mới vượt 8 m hoặc 10 m được bypass phần filter có
thể che crossing, nên controller phản ứng ở update kế tiếp.

## Kiểm thử

- `compileall`: đạt.
- `python -m unittest discover -p 'test_*.py'`: **112/112 đạt**; đã bổ sung
  regression cho gimbal-disabled depth, chế độ ground-truth tuyệt đối không
  gọi motion callback, crop/intrinsics, split không rò dữ liệu và reject
  calibration không an toàn.
- HTML parse: đạt.
- JavaScript parse bằng Node: đạt.
- Ruff/mypy: không có trong môi trường.

Test bao phủ geometry, optical-Z/radial conversion, resize/crop, background,
outlier, NaN/Inf, uncertainty, dispersion, bbox drift, calibration mismatch,
filter/jitter, latency sign, safe-band crossing, stale/invalid fail-safe,
latest-frame-wins, shutdown và altitude hold.

## Runtime và số liệu

Kiểm tra chỉ-đọc cuối:

- UAV-01: disarmed, không failsafe.
- UAV-02: disarmed, không failsafe.
- Tracking cuối phép đo: inactive.
- Uvicorn được dừng sạch sau phép đo offline.
- `mavlink_manual_bridge.py`: 1 process (PID 11771).
- Camera source trước restart: khoảng 15.9-16.0 FPS khi idle.
- Camera source sau restart: khoảng 17.9-18.1 FPS khi idle. Đây không phải
  benchmark tracking vì chưa chọn target.
- JPEG encode khi mở stream: trung bình 0.76 ms, p95 1.03 ms.
- API `/api/drones`, 20 request sau preload model: trung bình 1.759 ms,
  p95 2.864 ms, tối đa 3.143 ms.
- Pose ground truth: subscribed, buffer 600 mẫu và timestamp Gazebo cập nhật.
- Trong lúc depth bật: camera source 16.8-18.3 FPS, tracking 12.2-13.8 FPS,
  UniDepth 2.38-2.78 FPS, inference gần nhất 138-150 ms.
- Worker dùng queue 0/1 và latest-frame-wins; dropped job tăng thay vì backlog.
- Motion callback sau bản sửa: `motion_active=false`, `motion_state=disabled`
  khi follow/offboard tắt.

Backend mới đã được preload và restart an toàn. Runtime validation phát hiện
Pose_V dùng tên link ngắn `camera_link`; collector cũ trong bản sửa đầu tiên
đã bỏ link này. Collector hiện ánh xạ link theo entity-ID range, compose local
link pose với model world pose, và có test regression.

Đã thu 5.246 mẫu đồng bộ ở 5/8/9/10/12/15 m; 4.950 mẫu hợp lệ, invalid rate
5,64%. Ground truth dùng camera-link-to-target-center ở đúng timestamp frame.
Dataset: `artifacts/metric_depth_ground_truth_full_frame_20260729.csv`.

Kết quả held-out theo capture condition:

- Không calibration: MAE 8,0928 m; RMSE 8,7991 m; bias +8,0928 m;
  AbsRel 0,8641; p95 12,7560 m.
- Scale-only tốt nhất trong các mapping hợp lệ vật lý: MAE 2,3651 m;
  RMSE 3,2868 m; bias +1,5220 m; AbsRel 0,2779; p95 5,4457 m.
- Affine có MAE toán học 2,1851 m nhưng scale -0,5576 nên bị reject.
- Piecewise có mapping không đơn điệu nên bị reject.
- `production_recommended=false`; calibration production tiếp tục **tắt**.

Output full-frame theo các điểm quan sát đại diện là 5→15,49 m,
8→18,72 m, 9→14,91 m, 10→15,83 m, 12→17,51 m, 15→12,69 m. Sự không đơn điệu
cho thấy mục tiêu 14-36 px và background/domain gap là giới hạn chính, không
thể sửa an toàn chỉ bằng một hệ số.

Target-crop có intrinsics scale đúng cũng được A/B. Nó cải thiện mẫu 5 m
(15,49→8,45 m), nhưng làm 10 m thành 6,15 m và 15 m thành 3,52 m ngay cả khi
giới hạn zoom 1,5x. Vì crop làm thay đổi metric prior, tính năng này mặc định
tắt và chỉ giữ để nghiên cứu offline.

## Cách chạy lại an toàn

1. Giữ cả hai UAV disarm; xác nhận `armed=false`, `failsafe=false`.
2. Backend mới hiện đã chạy. Khi restart lần sau, bật:

   ```dotenv
   SWARM_METRIC_DEPTH_GROUND_TRUTH_ENABLED=true
   SWARM_METRIC_DEPTH_GROUND_TRUTH_CSV=metric_depth_ground_truth.csv
   SWARM_METRIC_DEPTH_CALIBRATION_ENABLED=false
   ```

3. Dataset hiện tại bao phủ dãy khoảng cách trung tâm. Thu bổ sung cảnh
   trái/phải, trên/dưới, pitch/yaw và background khác trước khi cân nhắc
   calibration production. Mọi thay đổi trạng thái bay vẫn cần phê duyệt riêng.
4. Chạy:

   ```bash
   python metric_depth_evaluation.py metric_depth_ground_truth.csv \
     --model lpiccinelli/unidepth-v2-vitl14 \
     --json-output metric_depth_evaluation.json \
     --report-output METRIC_DEPTH_ACCURACY_REPORT.md \
     --profile-output calibration/unidepth_v2_gazebo.json
   ```

5. Chỉ bật calibration nếu báo cáo mới có
   `production_recommended=true` và profile khớp camera. Candidate hiện tại
   không đạt:

   ```dotenv
   SWARM_METRIC_DEPTH_CALIBRATION_ENABLED=true
   SWARM_METRIC_DEPTH_CALIBRATION_PROFILE=calibration/unidepth_v2_gazebo.json
   ```

## File chính đã sửa/thêm

- `metric_depth_accuracy.py`
- `metric_depth_estimator.py`
- `metric_depth_evaluation.py`
- `tracking_web.py`
- `main.py`
- `static/index.html`
- `.env.example`
- `README.md`
- `test_metric_depth_estimator.py`
- `test_metric_depth_ground_truth.py`
