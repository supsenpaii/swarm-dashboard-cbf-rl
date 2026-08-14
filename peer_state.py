"""Direct UDP peer-state transport used by companion computers.

The transport only distributes observations.  It deliberately contains no
flight-command API, so a peer packet cannot arm, change mode, or command PX4.
"""

from __future__ import annotations

import json
import math
import socket
import threading
import time
from typing import Any, Iterable


PEER_STATE_SCHEMA_VERSION = 1
MAX_UDP_PAYLOAD_BYTES = 4096


def parse_endpoints(value: str) -> tuple[tuple[str, int], ...]:
    """Parse comma-separated IPv4/hostname ``host:port`` endpoints."""
    endpoints: list[tuple[str, int]] = []
    for item in value.split(","):
        candidate = item.strip()
        if not candidate:
            continue
        host, separator, raw_port = candidate.rpartition(":")
        if not separator or not host:
            raise ValueError(f"invalid peer endpoint: {candidate!r}")
        try:
            port = int(raw_port)
        except ValueError as error:
            raise ValueError(f"invalid peer endpoint port: {candidate!r}") from error
        if not 1 <= port <= 65535:
            raise ValueError(f"peer endpoint port is out of range: {candidate!r}")
        endpoints.append((host, port))
    return tuple(endpoints)


def make_peer_state(
    *,
    drone_id: str,
    sequence: int,
    position_enu_m: tuple[float, float, float],
    velocity_enu_m_s: tuple[float, float, float],
    healthy: bool,
    timestamp_ms: int | None = None,
    position_covariance_m2: tuple[float, float, float] | None = None,
) -> dict[str, Any]:
    if not drone_id.strip() or sequence < 0:
        raise ValueError("peer state identity is invalid")
    vectors = position_enu_m + velocity_enu_m_s
    if not all(math.isfinite(value) for value in vectors):
        raise ValueError("peer state vectors must be finite")
    if position_covariance_m2 is not None and (
        not all(math.isfinite(value) and value >= 0.0 for value in position_covariance_m2)
    ):
        raise ValueError("peer state covariance is invalid")
    return {
        "type": "peer_state",
        "schema_version": PEER_STATE_SCHEMA_VERSION,
        "drone_id": drone_id,
        "sequence": int(sequence),
        "timestamp_ms": int(time.time() * 1000.0) if timestamp_ms is None else int(timestamp_ms),
        "frame": "ENU",
        "position_enu_m": list(position_enu_m),
        "velocity_enu_m_s": list(velocity_enu_m_s),
        "position_covariance_m2": list(position_covariance_m2)
        if position_covariance_m2 is not None
        else None,
        "healthy": bool(healthy),
    }


class PeerStateRegistry:
    """Validates peer packets and tracks freshness and sequence gaps."""

    def __init__(self, max_age_s: float = 0.5) -> None:
        if not math.isfinite(max_age_s) or max_age_s <= 0.0:
            raise ValueError("max_age_s must be positive and finite")
        self.max_age_s = float(max_age_s)
        self._states: dict[str, dict[str, Any]] = {}
        self._received_s: dict[str, float] = {}
        self._last_sequence: dict[str, int] = {}
        self._lost_packets: dict[str, int] = {}
        self._rejected_packets = 0
        self._lock = threading.Lock()

    def ingest(self, payload: Any, received_monotonic_s: float | None = None) -> bool:
        received = time.monotonic() if received_monotonic_s is None else float(received_monotonic_s)
        try:
            state = self._validated(payload)
            if not math.isfinite(received):
                raise ValueError("receive timestamp is invalid")
        except (TypeError, ValueError, KeyError):
            with self._lock:
                self._rejected_packets += 1
            return False
        drone_id = state["drone_id"]
        sequence = state["sequence"]
        with self._lock:
            previous = self._last_sequence.get(drone_id)
            if previous is not None and sequence <= previous:
                self._rejected_packets += 1
                return False
            if previous is not None:
                self._lost_packets[drone_id] = self._lost_packets.get(drone_id, 0) + max(0, sequence - previous - 1)
            self._last_sequence[drone_id] = sequence
            self._states[drone_id] = state
            self._received_s[drone_id] = received
        return True

    def snapshot(self, now_monotonic_s: float | None = None) -> dict[str, Any]:
        now = time.monotonic() if now_monotonic_s is None else float(now_monotonic_s)
        if not math.isfinite(now):
            raise ValueError("now_monotonic_s must be finite")
        with self._lock:
            peers = {}
            for drone_id, state in self._states.items():
                age_ms = max(0.0, now - self._received_s[drone_id]) * 1000.0
                peers[drone_id] = {
                    **state,
                    "message_age_ms": round(age_ms, 2),
                    "valid": bool(
                        state["healthy"] and age_ms <= self.max_age_s * 1000.0
                    ),
                    "reason": (
                        "ok"
                        if state["healthy"] and age_ms <= self.max_age_s * 1000.0
                        else "peer_stale"
                        if age_ms > self.max_age_s * 1000.0
                        else "peer_unhealthy"
                    ),
                    "lost_packets": self._lost_packets.get(drone_id, 0),
                }
            return {"peers": peers, "rejected_packets": self._rejected_packets}

    @staticmethod
    def _validated(payload: Any) -> dict[str, Any]:
        if not isinstance(payload, dict):
            raise ValueError("peer payload is not an object")
        if payload.get("type") != "peer_state" or payload.get("schema_version") != PEER_STATE_SCHEMA_VERSION:
            raise ValueError("peer payload schema is invalid")
        drone_id = str(payload["drone_id"]).strip()
        sequence = int(payload["sequence"])
        if not drone_id or sequence < 0 or payload.get("frame") != "ENU":
            raise ValueError("peer payload identity or frame is invalid")
        position = _vector(payload["position_enu_m"], "position")
        velocity = _vector(payload["velocity_enu_m_s"], "velocity")
        covariance_raw = payload.get("position_covariance_m2")
        covariance = None if covariance_raw is None else _vector(covariance_raw, "covariance", nonnegative=True)
        timestamp_ms = int(payload["timestamp_ms"])
        if timestamp_ms < 0:
            raise ValueError("peer timestamp is invalid")
        return {
            "type": "peer_state",
            "schema_version": PEER_STATE_SCHEMA_VERSION,
            "drone_id": drone_id,
            "sequence": sequence,
            "timestamp_ms": timestamp_ms,
            "frame": "ENU",
            "position_enu_m": list(position),
            "velocity_enu_m_s": list(velocity),
            "position_covariance_m2": list(covariance) if covariance else None,
            "healthy": bool(payload.get("healthy", False)),
        }


class DirectPeerStateTransport:
    """Unicast UDP publisher/receiver; callers choose peer endpoints explicitly."""

    def __init__(self, bind_host: str, bind_port: int, endpoints: Iterable[tuple[str, int]], registry: PeerStateRegistry) -> None:
        self.endpoints = tuple(endpoints)
        if not self.endpoints:
            raise ValueError("at least one peer endpoint is required")
        self.registry = registry
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._socket.bind((bind_host, bind_port))
        self._socket.settimeout(0.2)
        self._stop_event = threading.Event()
        self._thread = threading.Thread(target=self._receive_loop, name="peer-state-udp", daemon=True)

    def start(self) -> None:
        self._thread.start()

    def stop(self) -> None:
        self._stop_event.set()
        self._thread.join(timeout=1.0)
        self._socket.close()

    def publish(self, payload: dict[str, Any]) -> bool:
        try:
            encoded = json.dumps(payload, separators=(",", ":")).encode("utf-8")
            if len(encoded) > MAX_UDP_PAYLOAD_BYTES:
                return False
            for endpoint in self.endpoints:
                self._socket.sendto(encoded, endpoint)
        except (OSError, TypeError, ValueError):
            return False
        return True

    def status(self, now_monotonic_s: float | None = None) -> dict[str, Any]:
        """Monitoring snapshot, published as telemetry.

        The command path reads `self.registry` directly (see
        `companion_safety`); this accessor exists so callers that only want to
        report freshness do not have to reach through to the registry.
        """
        return self.registry.snapshot(now_monotonic_s)

    def _receive_loop(self) -> None:
        while not self._stop_event.is_set():
            try:
                raw, _ = self._socket.recvfrom(MAX_UDP_PAYLOAD_BYTES)
                payload = json.loads(raw.decode("utf-8"))
            except socket.timeout:
                continue
            except (OSError, UnicodeDecodeError, json.JSONDecodeError):
                continue
            self.registry.ingest(payload)


def _vector(value: Any, name: str, *, nonnegative: bool = False) -> tuple[float, float, float]:
    if not isinstance(value, (list, tuple)) or len(value) != 3:
        raise ValueError(f"peer {name} is invalid")
    vector = tuple(float(item) for item in value)
    if not all(math.isfinite(item) and (item >= 0.0 if nonnegative else True) for item in vector):
        raise ValueError(f"peer {name} is invalid")
    return vector  # type: ignore[return-value]
