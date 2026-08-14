# Giai thich du an Swarm UAV Dashboard

Tai lieu nay tom tat nhung gi du an dang lam, cach cac file lien ket voi nhau, cach chay chuong trinh, va nhung loi da gap trong qua trinh test.

## 1. Muc tieu cua du an

Du an nay tao mot dashboard web de theo doi va dieu khien 2 UAV trong Gazebo/PX4:

- `UAV-01`
- `UAV-02`

Dashboard co cac chuc nang chinh:

- Xem telemetry cua UAV: mode, armed, preflight, pin, GPS, vi tri local/global.
- Hien thi vi tri UAV tren ban do Leaflet.
- Chon UAV dang dieu khien.
- Gui lenh flight action: `ARM`, `DISARM`, `HOLD`, `LAND`, `RETURN HOME`.
- Gui lenh `TAKEOFF` voi do cao gioi han.
- Gui target tren ban do trong che do `OFFBOARD MAP`.
- Dieu khien bang ban phim trong che do `POSITION`.

## 2. Cac file chinh

### `main.py`

Day la backend FastAPI cua dashboard.

Nhiem vu:

- Serve giao dien web tai route `/`.
- Cung cap API `/health` de kiem tra server.
- Cung cap API `/api/drones` de xem telemetry hien tai.
- Mo WebSocket `/ws` de frontend nhan telemetry lien tuc va gui lenh dieu khien.
- Ket noi MQTT broker tai `127.0.0.1:1883`.
- Subscribe telemetry:
  - `swarm/+/telemetry/state`
  - `swarm/+/control/result`
- Publish lenh dieu khien:
  - `swarm/{drone_id}/control/command`

Backend khong truc tiep noi voi PX4. Backend chi la cau noi giua web app va MQTT.

### `static/index.html`

Day la frontend cua dashboard.

Nhiem vu:

- Tao giao dien dieu khien bang HTML/CSS/JavaScript.
- Ket noi WebSocket toi backend.
- Hien thi telemetry cua tung UAV.
- Hien thi ban do bang Leaflet.
- Gui command qua WebSocket khi nguoi dung bam nut.
- Gui frame manual control 20 Hz khi keyboard control dang bat.

Phim manual:

- `W`: tien
- `S`: lui
- `A`: trai
- `D`: phai
- `Q`: xoay trai
- `E`: xoay phai
- `R`: len
- `F`: xuong
- `Space`: dung

Toc do keyboard dang duoc cau hinh tai:

```js
const HORIZONTAL_VALUE=.75, VERTICAL_VALUE=.65, YAW_VALUE=.50;
```

Y nghia:

- `HORIZONTAL_VALUE`: tien/lui/trai/phai.
- `VERTICAL_VALUE`: len/xuong.
- `YAW_VALUE`: xoay trai/phai.

Gia tri hop le nen nam trong khoang `0.0` den `1.0`.

### `mavlink_manual_bridge.py`

Day la bridge chuyen lenh manual tu MQTT sang MAVLink.

Nhiem vu:

- Subscribe MQTT topic:
  - `swarm/+/control/command`
- Chi xu ly message co:
  - `"type": "manual_control"`
- Ket noi PX4 qua UDP:
  - `UAV-01`: `udpin:0.0.0.0:14540`, system id `1`
  - `UAV-02`: `udpin:0.0.0.0:14541`, system id `2`
- Gui MAVLink `MANUAL_CONTROL` toi PX4 o tan so 20 Hz.

Neu file nay khong chay thi web app van nhan phim, backend van publish MQTT, nhung drone se khong di chuyen vi khong co thanh phan nao chuyen lenh sang PX4.

## 3. Kien truc tong the

Luong telemetry:

```text
PX4 / ROS backend
  -> MQTT telemetry topic
  -> main.py
  -> WebSocket /api/drones
  -> static/index.html
  -> hien thi tren dashboard
```

Luong lenh flight action:

```text
Nguoi dung bam ARM/LAND/RTL/...
  -> static/index.html
  -> WebSocket /ws
  -> main.py
  -> MQTT command topic
  -> ROS/PX4 control backend
  -> PX4
```

Luong keyboard manual:

```text
Nguoi dung bam W/A/S/D
  -> static/index.html gui manual_control 20 Hz
  -> WebSocket /ws
  -> main.py publish MQTT
  -> mavlink_manual_bridge.py nhan MQTT
  -> MAVLink MANUAL_CONTROL
  -> PX4
  -> drone di chuyen
```

## 4. Cach chay chuong trinh

Can mo it nhat 2 terminal.

### Terminal 1: chay web backend

```bash
cd /home/sup/swarm_dashboard
source .venv/bin/activate
python -m uvicorn main:app --host 0.0.0.0 --port 8000
```

Neu gap loi:

```text
address already in use
```

thi port `8000` dang co server khac chay. Kiem tra va dung process:

```bash
fuser -v -n tcp 8000
kill <PID>
```

### Terminal 2: chay MAVLink manual bridge

```bash
cd /home/sup/swarm_dashboard
source .venv/bin/activate
python3 mavlink_manual_bridge.py
```

Log dung se co dang:

```text
MQTT connected: Success
UAV-01: listening for PX4 on udpin:0.0.0.0:14540
UAV-02: listening for PX4 on udpin:0.0.0.0:14541
Manual bridge started at 20.0 Hz
UAV-01 connected: system=1 component=1 port=14540
UAV-02 connected: system=2 component=1 port=14541
```

Chi nen chay 1 process `mavlink_manual_bridge.py`. Neu chay 2 process cung luc, hai process co the cung gui `MANUAL_CONTROL`; mot process nhan lenh, process kia gui stick ve giua, lam drone khong di chuyen.

Kiem tra bridge dang chay:

```bash
fuser -v -n udp 14540
fuser -v -n udp 14541
```

Moi port chi nen co 1 PID.

## 5. Dieu kien de keyboard control hoat dong

Trong dashboard, keyboard chi nen bat khi telemetry bao:

- `Armed = YES`
- `Mode = Position`
- `Manual link = OK`
- `Preflight = OK`

Neu app hien `KEYBOARD NOT READY`, can xem ly do:

- UAV chua ARM.
- PX4 chua that su o Position mode.
- Manual bridge chua chay.
- PX4 chua nhan manual input.

Sau khi dung `LAND` hoac `RETURN HOME`, PX4 co the bi ket trong `Offboard`, `RTL`, `Land`, hoac preflight chua san sang. App da duoc sua de khi bam `ARM` trong trang thai nay, no se yeu cau `POSITION` truoc thay vi gui ARM ngay.

## 6. Nhung loi da gap va cach fix

### Loi 1: Bam LAND/RETURN HOME xong khong ARM lai duoc

Hien tuong:

- Bam `LAND` hoac `RETURN HOME`.
- Sau do bam `ARM` lai thi PX4 tu choi.
- Control result co the bao `TEMPORARILY_REJECTED`.

Nguyen nhan:

- PX4 van dang o mode khong phu hop nhu `Offboard`, `RTL`, `Land`.
- `preflight_checks_pass=false`.
- App gui ARM ngay khi PX4 chua san sang.

Da fix:

- Frontend khong gui ARM ngay neu UAV chua san sang.
- Backend cung chan ARM neu telemetry bao chua san sang.
- Neu can, backend/frontend gui `position` truoc, cho PX4 quay ve trang thai phu hop roi moi ARM lai.

### Loi 2: App nhan phim nhung drone khong di chuyen

Nguyen nhan da gap:

- `mavlink_manual_bridge.py` chua chay.
- Hoac chay 2 bridge cung luc.
- Hoac UAV chua `Armed`, chua `Position`, hoac `Manual link = LOST`.

Da fix:

- Them hien thi `Manual link`.
- Chi cho keyboard bat khi UAV that su san sang.
- Dung process bridge trung lap.

### Loi 3: Chay uvicorn bi `address already in use`

Nguyen nhan:

- Port `8000` da co backend dang chay.

Cach xu ly:

```bash
fuser -v -n tcp 8000
kill <PID>
```

Sau do chay lai `uvicorn`.

## 7. Cac route/API quan trong

### `/`

Tra ve frontend `static/index.html`.

### `/health`

Kiem tra server:

```json
{
  "ok": true,
  "mqtt_connected": true,
  "index_exists": true
}
```

### `/api/drones`

Tra ve telemetry moi nhat:

- `drones`
- `control_results`
- `count`
- `mqtt_connected`

### `/ws`

WebSocket dung cho:

- Frontend nhan telemetry snapshot.
- Frontend gui command dieu khien.

## 8. MQTT topics

Backend subscribe:

```text
swarm/+/telemetry/state
swarm/+/control/result
```

Backend publish:

```text
swarm/{drone_id}/control/command
```

Manual bridge subscribe:

```text
swarm/+/control/command
```

## 9. Cac action duoc backend chap nhan

Trong `main.py`, danh sach action hop le gom:

```text
position
offboard_map
takeoff
arm
disarm
land
hold
rtl
hold_current
enable_offboard
keyboard_off
stop
```

`takeoff` co them tham so:

```json
{
  "altitude_m": 5.0
}
```

Backend gioi han takeoff altitude trong khoang `2.5` den `10.0` met.

## 10. Cach dong goi zip

File zip da tao:

```text
swarm_app.zip
```

No gom:

```text
swarm_app/main.py
swarm_app/mavlink_manual_bridge.py
swarm_app/static/index.html
```

Khong gom:

- `.venv`
- backup files
- log
- build
- install

## 11. Ghi nho khi van hanh

- Luon chay `uvicorn` va `mavlink_manual_bridge.py` song song.
- Khong chay 2 bridge cung luc.
- Neu keyboard khong chay, kiem tra `Manual link`.
- Neu ARM bi reject, kiem tra `Preflight` va `Mode`.
- Sau khi sua `static/index.html`, refresh trinh duyet bang `Ctrl + F5`.
- Sau khi sua `main.py`, restart `uvicorn`.
- Sau khi sua `mavlink_manual_bridge.py`, restart bridge.
