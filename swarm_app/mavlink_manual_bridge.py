#!/usr/bin/env python3

from __future__ import annotations

import json
import logging
import math
import os
import signal
import threading
import time
from dataclasses import dataclass, field
from typing import Any

os.environ.setdefault("MAVLINK20", "1")

import paho.mqtt.client as mqtt
from pymavlink import mavutil


MQTT_HOST = "127.0.0.1"
MQTT_PORT = 1883
MQTT_TOPIC = "swarm/+/control/command"

SEND_RATE_HZ = 20.0
SEND_PERIOD_S = 1.0 / SEND_RATE_HZ

# Khi không nhận lệnh mới trong khoảng thời gian này,
# bridge tự đưa toàn bộ cần điều khiển về giữa.
COMMAND_TIMEOUT_S = 0.35

# Giới hạn stick để UAV không tăng tốc quá mạnh khi thử nghiệm.
HORIZONTAL_SCALE = 500
VERTICAL_SCALE = 500
YAW_SCALE = 400
# MAVLink MANUAL_CONTROL uses z as throttle in the 0..1000 range.
# 500 is centered/hold; 0 is minimum throttle, not neutral.
THROTTLE_CENTER = 500

SOURCE_SYSTEM_ID = 250
SOURCE_COMPONENT_ID = 191

VEHICLES = {
    "UAV-01": {
        "port": 14540,
        "system_id": 1,
    },
    "UAV-02": {
        "port": 14541,
        "system_id": 2,
    },
}


logging.basicConfig(
    level=logging.INFO,
    format=(
        "%(asctime)s | %(levelname)s | %(message)s"
    ),
)

LOGGER = logging.getLogger("mavlink-manual-bridge")


def clamp(value: Any, minimum: float, maximum: float) -> float:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return 0.0

    if not math.isfinite(number):
        return 0.0

    return max(minimum, min(maximum, number))


@dataclass
class ManualState:
    enabled: bool = False
    forward: float = 0.0
    right: float = 0.0
    up: float = 0.0
    yaw: float = 0.0
    last_command_time: float = 0.0
    lock: threading.Lock = field(
        default_factory=threading.Lock
    )

    def update(self, payload: dict[str, Any]) -> None:
        with self.lock:
            self.enabled = bool(
                payload.get("enabled", False)
            )

            self.forward = clamp(
                payload.get("forward", 0.0),
                -1.0,
                1.0,
            )

            self.right = clamp(
                payload.get("right", 0.0),
                -1.0,
                1.0,
            )

            self.up = clamp(
                payload.get("up", 0.0),
                -1.0,
                1.0,
            )

            self.yaw = clamp(
                payload.get("yaw", 0.0),
                -1.0,
                1.0,
            )

            self.last_command_time = time.monotonic()

    def disable(self) -> None:
        with self.lock:
            self.enabled = False
            self.forward = 0.0
            self.right = 0.0
            self.up = 0.0
            self.yaw = 0.0
            self.last_command_time = time.monotonic()

    def output(self) -> tuple[int, int, int, int, bool]:
        with self.lock:
            age = (
                time.monotonic()
                - self.last_command_time
            )

            active = (
                self.enabled
                and age <= COMMAND_TIMEOUT_S
            )

            if not active:
                return 0, 0, THROTTLE_CENTER, 0, False

            x = int(
                round(
                    self.forward
                    * HORIZONTAL_SCALE
                )
            )

            y = int(
                round(
                    self.right
                    * HORIZONTAL_SCALE
                )
            )

            z = int(
                round(
                    THROTTLE_CENTER
                    + self.up
                    * VERTICAL_SCALE
                )
            )
            z = max(0, min(1000, z))

            r = int(
                round(
                    self.yaw
                    * YAW_SCALE
                )
            )

            return x, y, z, r, True


class MavlinkWorker(threading.Thread):
    def __init__(
        self,
        drone_id: str,
        port: int,
        expected_system_id: int,
        state: ManualState,
        stop_event: threading.Event,
    ) -> None:
        super().__init__(
            name=f"mavlink-{drone_id}",
            daemon=True,
        )

        self.drone_id = drone_id
        self.port = port
        self.expected_system_id = expected_system_id
        self.state = state
        self.stop_event = stop_event
        self.connection: Any = None

    def connect(self) -> Any:
        connection_string = (
            f"udpin:0.0.0.0:{self.port}"
        )

        LOGGER.info(
            "%s: listening for PX4 on %s",
            self.drone_id,
            connection_string,
        )

        connection = mavutil.mavlink_connection(
            connection_string,
            source_system=SOURCE_SYSTEM_ID,
            source_component=SOURCE_COMPONENT_ID,
            dialect="common",
            robust_parsing=True,
        )

        heartbeat = connection.wait_heartbeat(
            timeout=15
        )

        if heartbeat is None:
            connection.close()
            raise TimeoutError(
                f"{self.drone_id}: heartbeat timeout "
                f"on UDP {self.port}"
            )

        received_system_id = (
            heartbeat.get_srcSystem()
        )

        received_component_id = (
            heartbeat.get_srcComponent()
        )

        if (
            received_system_id
            != self.expected_system_id
        ):
            connection.close()
            raise RuntimeError(
                f"{self.drone_id}: expected system "
                f"{self.expected_system_id}, received "
                f"{received_system_id}"
            )

        LOGGER.info(
            "%s connected: system=%d component=%d port=%d",
            self.drone_id,
            received_system_id,
            received_component_id,
            self.port,
        )

        return connection

    def send_centered_controls(
        self,
        count: int = 5,
    ) -> None:
        if self.connection is None:
            return

        for _ in range(count):
            try:
                self.connection.mav.manual_control_send(
                    self.expected_system_id,
                    0,
                    0,
                    THROTTLE_CENTER,
                    0,
                    0,
                )
            except Exception:
                break

            time.sleep(0.03)

    def run_connected(self) -> None:
        assert self.connection is not None

        last_heartbeat_sent = 0.0
        last_active_state: bool | None = None

        while not self.stop_event.is_set():
            loop_started = time.monotonic()

            # Đọc và bỏ các gói telemetry đang chờ để duy trì socket.
            for _ in range(30):
                message = self.connection.recv_match(
                    blocking=False
                )

                if message is None:
                    break

            x, y, z, r, active = self.state.output()

            self.connection.mav.manual_control_send(
                self.expected_system_id,
                x,
                y,
                z,
                r,
                0,
            )

            now = time.monotonic()

            if now - last_heartbeat_sent >= 1.0:
                self.connection.mav.heartbeat_send(
                    mavutil.mavlink.MAV_TYPE_GCS,
                    mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                    0,
                    0,
                    mavutil.mavlink.MAV_STATE_ACTIVE,
                )

                last_heartbeat_sent = now

            if active != last_active_state:
                LOGGER.info(
                    "%s manual output: %s "
                    "x=%d y=%d z=%d r=%d",
                    self.drone_id,
                    "ACTIVE" if active else "CENTERED",
                    x,
                    y,
                    z,
                    r,
                )

                last_active_state = active

            elapsed = (
                time.monotonic()
                - loop_started
            )

            sleep_time = (
                SEND_PERIOD_S - elapsed
            )

            if sleep_time > 0:
                self.stop_event.wait(
                    sleep_time
                )

    def run(self) -> None:
        while not self.stop_event.is_set():
            try:
                self.connection = self.connect()
                self.send_centered_controls()
                self.run_connected()

            except Exception as error:
                LOGGER.error(
                    "%s MAVLink error: %s",
                    self.drone_id,
                    error,
                )

                self.stop_event.wait(2.0)

            finally:
                self.send_centered_controls()

                if self.connection is not None:
                    try:
                        self.connection.close()
                    except Exception:
                        pass

                self.connection = None


states = {
    drone_id: ManualState()
    for drone_id in VEHICLES
}

stop_event = threading.Event()


def on_mqtt_connect(
    client: mqtt.Client,
    userdata: Any,
    flags: Any,
    reason_code: Any,
    properties: Any = None,
) -> None:
    LOGGER.info(
        "MQTT connected: %s",
        reason_code,
    )

    client.subscribe(
        MQTT_TOPIC,
        qos=0,
    )


def on_mqtt_disconnect(
    client: mqtt.Client,
    userdata: Any,
    disconnect_flags: Any = None,
    reason_code: Any = None,
    properties: Any = None,
) -> None:
    LOGGER.warning(
        "MQTT disconnected: %s",
        reason_code,
    )

    for state in states.values():
        state.disable()


def on_mqtt_message(
    client: mqtt.Client,
    userdata: Any,
    message: mqtt.MQTTMessage,
) -> None:
    try:
        payload = json.loads(
            message.payload.decode("utf-8")
        )
    except (
        UnicodeDecodeError,
        json.JSONDecodeError,
    ) as error:
        LOGGER.warning(
            "Invalid MQTT JSON: %s",
            error,
        )
        return

    if not isinstance(payload, dict):
        return

    if payload.get("type") != "manual_control":
        return

    drone_id = str(
        payload.get("drone_id", "")
    )

    state = states.get(drone_id)

    if state is None:
        LOGGER.warning(
            "Unknown drone ID: %s",
            drone_id,
        )
        return

    state.update(payload)


def create_mqtt_client() -> mqtt.Client:
    try:
        client = mqtt.Client(
            mqtt.CallbackAPIVersion.VERSION2,
            client_id="mavlink-manual-bridge",
        )
    except AttributeError:
        client = mqtt.Client(
            client_id="mavlink-manual-bridge",
        )

    client.on_connect = on_mqtt_connect
    client.on_disconnect = on_mqtt_disconnect
    client.on_message = on_mqtt_message

    client.reconnect_delay_set(
        min_delay=1,
        max_delay=10,
    )

    return client


def request_shutdown(
    signum: int,
    frame: Any,
) -> None:
    LOGGER.info(
        "Shutdown requested"
    )

    stop_event.set()


def main() -> None:
    signal.signal(
        signal.SIGINT,
        request_shutdown,
    )

    signal.signal(
        signal.SIGTERM,
        request_shutdown,
    )

    mqtt_client = create_mqtt_client()

    mqtt_client.connect(
        MQTT_HOST,
        MQTT_PORT,
        keepalive=30,
    )

    mqtt_client.loop_start()

    workers = []

    for drone_id, config in VEHICLES.items():
        worker = MavlinkWorker(
            drone_id=drone_id,
            port=int(config["port"]),
            expected_system_id=int(
                config["system_id"]
            ),
            state=states[drone_id],
            stop_event=stop_event,
        )

        workers.append(worker)
        worker.start()

    LOGGER.info(
        "Manual bridge started at %.1f Hz",
        SEND_RATE_HZ,
    )

    LOGGER.info(
        "MQTT command topic: %s",
        MQTT_TOPIC,
    )

    try:
        while not stop_event.wait(1.0):
            pass

    finally:
        stop_event.set()

        for state in states.values():
            state.disable()

        for worker in workers:
            worker.join(timeout=3.0)

        mqtt_client.loop_stop()
        mqtt_client.disconnect()

        LOGGER.info(
            "Manual bridge stopped"
        )


if __name__ == "__main__":
    main()
