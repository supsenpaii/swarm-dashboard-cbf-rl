# Tracking Follow Updates - 2026-07-21

Thoi gian cap nhat: 2026-07-21 14:20:46 +07

## Muc tieu

Hoan thien che do tracking/follow sao cho drone:

- Bam target bang tracker va gimbal.
- Tien/lui theo khoang cach an toan duoc set tai thoi diem keo bbox.
- Giam toc va dung khi target gan bang kich thuoc luc dau tracking.
- Dieu chinh do cao theo pitch gimbal de dua gimbal dan ve HOME.

## Thay doi chinh

### 1. Apparent-size follow nhanh hon

Da tang do phan ung cua controller tien/lui:

- `SWARM_APPARENT_SIZE_FOLLOW_KP=1.15`
- `SWARM_APPARENT_SIZE_FOLLOW_DEADBAND=0.005`
- `SWARM_APPARENT_SIZE_FOLLOW_EMA_ALPHA=0.60`
- `SWARM_APPARENT_SIZE_FOLLOW_SLEW=1.40`
- `SWARM_TRACKING_OFFBOARD_MAX_FORWARD_M_S=1.2`

Them derivative boost co gioi han:

- `SWARM_APPARENT_SIZE_FOLLOW_KD=0.12`
- `SWARM_APPARENT_SIZE_FOLLOW_BOOST_MAX=0.20`
- `SWARM_APPARENT_SIZE_RATE_DEADBAND=0.03`

### 2. Object-pixel scale

Da them tinh kich thuoc pixel cua vat trong bbox bang `object_pixel_scale_px()`.

Sau khi test, GrabCut va contrast co the cho scale khac nhau, gay ra tinh trang:

- Luc dau reference la `object_grabcut`.
- Runtime lai la `object_contrast`.
- Ratio sai, drone co the khong dung hoac khong follow.

Da chuyen mac dinh sang `object_contrast` nhat quan cho ca reference va runtime. GrabCut chi dung neu bat:

```bash
SWARM_OBJECT_PIXEL_SCALE_ALLOW_GRABCUT=true
```

### 3. Hybrid bbox/object follow

Ket luan thiet ke hien tai:

- Bbox scale la range/safety controller chinh.
- Object-pixel scale chi la fine-tuning khi source on dinh.
- Object-pixel khong duoc override bbox safety.

Logic hien tai:

```text
BR = current_bbox_scale / reference_bbox_scale

BR < 0.97       -> drone duoc tien
0.97 <= BR <= 1.10 -> drone dung/giu khoang cach
BR > 1.10       -> drone lui
```

Object-pixel chi blend vao command voi:

```text
SWARM_OBJECT_SCALE_BLEND=0.25
```

Status moi:

- `follow_reference_bbox_scale_px`
- `follow_bbox_scale_ratio`
- `follow_forward_blocked_by_bbox_guard`
- `bbox_forward_stop_ratio`
- `bbox_reverse_start_ratio`
- `object_scale_blend`

UI hien:

- `BR ...`
- `BBOX-HOLD` neu bbox guard dang chan forward.

### 4. Giam toc va dung gan reference

Da them slowband:

```text
SWARM_APPARENT_SIZE_SLOWBAND=0.20
```

Khi sai lech scale nho dan, command giam muot bang taper. Khi vao vung gan bang reference thi dung.

Them hold ratio tolerance:

```text
SWARM_APPARENT_SIZE_HOLD_RATIO_TOLERANCE=0.12
```

Neu object-pixel ratio trong khoang gan reference thi:

```text
follow_scale_error = 0
follow_scale_error_rate = 0
forward velocity = 0
```

### 5. Vertical gimbal recenter

Da them dieu khien do cao theo pitch gimbal trong `apparent_size_offboard`.

Y tuong:

```text
gimbal van tracking target
neu gimbal pitch lech HOME -> drone bay len/xuong de dua pitch ve HOME
```

PX4 dung NED:

- `down_velocity_m_s < 0`: bay len.
- `down_velocity_m_s > 0`: bay xuong.

Thong so:

```text
SWARM_TRACKING_VERTICAL_RECENTER_KP=0.035
SWARM_TRACKING_VERTICAL_RECENTER_DEADBAND_DEG=2.0
SWARM_TRACKING_VERTICAL_RECENTER_MAX_DOWN_M_S=0.45
SWARM_TRACKING_VERTICAL_RECENTER_SLEW_M_S2=0.70
SWARM_TRACKING_VERTICAL_RECENTER_INVERT=false
```

Neu drone bay nguoc chieu pitch gimbal, set:

```bash
SWARM_TRACKING_VERTICAL_RECENTER_INVERT=true
```

Status moi:

- `motion_down_velocity_m_s`
- `motion_vertical_recenter_active`
- `motion_vertical_pitch_error_deg`
- `vertical_recenter_deadband_deg`
- `vertical_recenter_max_down_m_s`

UI hien:

- `DZ ...m/s`
- `PE ...deg`

### 6. Command path da update

Da update `command_tracking_motion()` trong `main.py` de nhan va publish:

```text
down_velocity_m_s
```

Bridge `mavlink_manual_bridge.py` da co san support `down_velocity_m_s`, nen chi can truyen qua payload `offboard_follow`.

## Trang thai live cuoi cung

Lan kiem tra cuoi:

```text
tracking.active=false
motion_forward_velocity_m_s=0.0
motion_down_velocity_m_s=0.0
mqtt_connected=true
gazebo.started=true
```

Backend dang chay:

```bash
python -m uvicorn main:app --host 0.0.0.0 --port 8000
```

## Cach test hien tai

1. Bam `FOLLOW TARGET`.
2. Chon bbox target o khoang cach mong muon, day la khoang cach an toan.
3. Quan sat status:

```text
BR < 0.97       -> drone tien
BR 0.97-1.10    -> drone dung/giu
BR > 1.10       -> drone lui
DZ +/-...       -> drone len/xuong theo pitch gimbal
PE +/-...       -> pitch error cua gimbal so voi HOME
```

Neu drone khong tien ma `BR` da gan `1.0`, do la dung logic: no dang o khoang cach an toan.

Neu muon drone tien som hon, giam:

```bash
SWARM_BBOX_FOLLOW_FORWARD_STOP_RATIO=0.90
```

Neu muon dung xa hon, tang:

```bash
SWARM_BBOX_FOLLOW_FORWARD_STOP_RATIO=0.99
```

Neu muon lui som hon khi qua gan, giam:

```bash
SWARM_BBOX_FOLLOW_REVERSE_START_RATIO=1.05
```

## Files da sua

- `tracking_web.py`
- `main.py`
- `static/index.html`
- `test_visual_follow_target.py`

## Verification

Da chay:

```bash
python3 -m py_compile main.py tracking_web.py test_visual_follow_target.py
python3 -m unittest test_visual_follow_target.py
```

Ket qua:

```text
Ran 19 tests
OK
```
