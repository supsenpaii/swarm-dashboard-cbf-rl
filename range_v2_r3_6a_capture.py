"""Controlled observation-only capture helper for R3.6A-V.

The helper talks only to tracking start/bbox/stop and manual gimbal endpoints.
It never calls follow, arm, takeoff, flight-mode, manual vehicle control, or
motion endpoints.  Full-stack collection mode must already be active so all
motion/follow callbacks are absent in the backend.
"""

from __future__ import annotations

import argparse
import asyncio
import json
import os
from pathlib import Path
import time
from typing import Any, Mapping

import requests
import websockets

from range_physical_diagnostics import DIAGNOSTICS_FILENAME


BASE_URL = "http://127.0.0.1:8000"
WEBSOCKET_URL = "ws://127.0.0.1:8000/ws"


def _request(method: str, path: str, payload: Mapping[str, Any] | None = None) -> dict[str, Any]:
    response = requests.request(
        method,
        BASE_URL + path,
        json=None if payload is None else dict(payload),
        timeout=10,
    )
    response.raise_for_status()
    value = response.json()
    if not isinstance(value, dict):
        raise RuntimeError("dashboard_response_not_object")
    return value


def status() -> dict[str, Any]:
    return _request("GET", "/api/drones")


def _armed_values(value: Any, path: str = "root") -> list[tuple[str, bool]]:
    output: list[tuple[str, bool]] = []
    if isinstance(value, Mapping):
        for key, item in value.items():
            child = f"{path}.{key}"
            if key == "armed" and isinstance(item, bool):
                output.append((child, item))
            else:
                output.extend(_armed_values(item, child))
    elif isinstance(value, list):
        for index, item in enumerate(value):
            output.extend(_armed_values(item, f"{path}[{index}]"))
    return output


def assert_observation_only(current: Mapping[str, Any]) -> None:
    armed = _armed_values(current.get("drones"))
    if not armed:
        raise RuntimeError("armed_status_not_observed")
    if any(value for _, value in armed):
        raise RuntimeError(f"armed_vehicle_detected:{armed}")
    tracking = current.get("tracking")
    if isinstance(tracking, Mapping):
        forbidden_true = (
            "follow_requested",
            "follow_active",
            "visual_follow_requested",
            "visual_follow_active",
            "motion_active",
            "body_yaw_active",
        )
        active = [name for name in forbidden_true if tracking.get(name) is True]
        if active:
            raise RuntimeError(f"observation_only_violation:{','.join(active)}")


def wait_for_observation_only_ready(timeout_s: float = 45.0) -> dict[str, Any]:
    deadline = time.monotonic() + timeout_s
    last_reason = "armed_status_not_observed"
    while time.monotonic() < deadline:
        current = status()
        try:
            assert_observation_only(current)
            return current
        except RuntimeError as error:
            last_reason = str(error)
            if last_reason != "armed_status_not_observed":
                raise
        time.sleep(0.5)
    raise RuntimeError(f"observation_only_readiness_timeout:{last_reason}")


async def set_gimbal_pitch(delta_deg: float) -> dict[str, Any]:
    async with websockets.connect(WEBSOCKET_URL, open_timeout=5) as websocket:
        await websocket.send(
            json.dumps(
                {
                    "type": "gimbal_control",
                    "drone_id": "UAV-01",
                    "axis": "pitch",
                    "delta_deg": float(delta_deg),
                }
            )
        )
        deadline = time.monotonic() + 8.0
        while time.monotonic() < deadline:
            message = json.loads(await asyncio.wait_for(websocket.recv(), timeout=3.0))
            if message.get("type") == "gimbal_control_result" and message.get("axis") == "pitch":
                if not message.get("ok"):
                    raise RuntimeError(f"gimbal_position_failed:{message.get('error')}")
                return message
    raise RuntimeError("gimbal_position_timeout")


def sidecar_rows(root: Path) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    for path in sorted(root.glob("physical_diagnostics*.jsonl")):
        for line in path.read_text(encoding="utf-8").splitlines():
            if not line.strip():
                continue
            try:
                row = json.loads(line)
            except json.JSONDecodeError:
                continue
            if isinstance(row, dict):
                rows.append(row)
    return rows


def session_rows(root: Path, session_id: int) -> list[dict[str, Any]]:
    return [row for row in sidecar_rows(root) if int(row.get("session_id", -1)) == session_id]


def wait_for_new_sidecar_session(
    root: Path,
    existing_sessions: set[int],
    timeout_s: float = 30.0,
) -> int:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        sessions = {
            int(row["session_id"])
            for row in sidecar_rows(root)
            if isinstance(row.get("session_id"), int)
            and int(row["session_id"]) > 0
        }
        new_sessions = sessions - existing_sessions
        if new_sessions:
            return max(new_sessions)
        time.sleep(0.25)
    raise RuntimeError("new_sidecar_session_not_observed")


def wait_for_prewarm(root: Path, session_id: int, timeout_s: float) -> dict[str, Any]:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        rows = session_rows(root, session_id)
        stable = [
            row
            for row in rows
            if row.get("stage") == "calibration_only"
            and bool(((row.get("calibration") or {}).get("applied") or {}).get("stable"))
        ]
        if stable:
            return {"status": "stable", "frame_index": stable[-1].get("frame_index"), "row_count": len(rows)}
        time.sleep(0.5)
    return {"status": "timeout", "row_count": len(session_rows(root, session_id))}


def wait_for_raw_rows(root: Path, session_id: int, minimum: int, timeout_s: float) -> list[dict[str, Any]]:
    deadline = time.monotonic() + timeout_s
    latest: list[dict[str, Any]] = []
    while time.monotonic() < deadline:
        assert_observation_only(status())
        latest = [row for row in session_rows(root, session_id) if row.get("stage") == "raw_range_computed"]
        if len(latest) >= minimum:
            return latest
        time.sleep(0.75)
    raise RuntimeError(
        f"minimum_raw_rows_not_met:session={session_id}:"
        f"observed={len(latest)}:required={minimum}"
    )


def append_capture_metadata(root: Path, payload: Mapping[str, Any]) -> None:
    path = root / "capture_events.jsonl"
    with path.open("a", encoding="utf-8") as stream:
        stream.write(json.dumps(dict(payload), sort_keys=True, allow_nan=False) + "\n")


def capture(args: argparse.Namespace) -> dict[str, Any]:
    root = args.root.resolve()
    root.mkdir(parents=True, exist_ok=True)
    if os.getenv("SWARM_RANGE_RESIDUAL_MODE", "off").strip().lower() != "off":
        raise RuntimeError("residual_mode_must_be_off")
    if os.getenv("SWARM_RANGE_DATASET_DIR", "").strip() != str(root):
        raise RuntimeError("capture_root_does_not_match_backend_dataset_root")
    before = wait_for_observation_only_ready()
    existing_sessions = {
        int(row["session_id"])
        for row in sidecar_rows(root)
        if isinstance(row.get("session_id"), int)
        and int(row["session_id"]) > 0
    }
    started = _request("POST", "/api/tracking/start", {"drone_id": "UAV-01", "desired_distance_m": args.distance})
    tracking = started.get("tracking") or {}
    workflow = tracking.get("follow_workflow") or {}
    tracking_workflow_session_id = int(workflow.get("session_id", 0))
    if tracking_workflow_session_id <= 0:
        raise RuntimeError("tracking_session_id_invalid")
    gimbal = asyncio.run(set_gimbal_pitch(args.pitch))
    prewarm_session_id = wait_for_new_sidecar_session(root, existing_sessions)
    if args.prewarm:
        prewarm = wait_for_prewarm(root, prewarm_session_id, args.prewarm_timeout)
    else:
        time.sleep(0.5)
        prewarm = {"status": "not_requested", "row_count": len(session_rows(root, prewarm_session_id))}
    bbox = {
        "x": args.bbox_x,
        "y": args.bbox_y,
        "width": args.bbox_width,
        "height": args.bbox_height,
    }
    sessions_before_bbox = {
        int(row["session_id"])
        for row in sidecar_rows(root)
        if isinstance(row.get("session_id"), int)
        and int(row["session_id"]) > 0
    }
    selected = _request("POST", "/api/tracking/bbox", bbox)
    session_id = wait_for_new_sidecar_session(root, sessions_before_bbox)
    raw_before_perturbation = wait_for_raw_rows(root, session_id, args.before_perturbation_rows, args.capture_timeout)
    perturbation: dict[str, Any] | None = None
    if args.perturbation == "cache_pitch_zero":
        to_zero = asyncio.run(set_gimbal_pitch(-args.pitch))
        time.sleep(6.0)
        restore = asyncio.run(set_gimbal_pitch(args.pitch))
        perturbation = {"kind": args.perturbation, "to_zero": to_zero, "restore": restore}
    elif args.perturbation == "pitch_minus_two":
        down = asyncio.run(set_gimbal_pitch(-2.0))
        time.sleep(8.0)
        restore = asyncio.run(set_gimbal_pitch(2.0))
        perturbation = {"kind": args.perturbation, "down": down, "restore": restore}
    elif args.perturbation == "reacquire_not_forced":
        perturbation = {"kind": args.perturbation, "event_forced": False, "reason": "no_verified_safe_direct_pose_service_in_capture_helper"}
    raw_final = wait_for_raw_rows(root, session_id, args.minimum_raw_rows, args.capture_timeout)
    final_status = status()
    assert_observation_only(final_status)
    stopped = _request("POST", "/api/tracking/stop")
    payload = {
        "scenario_id": args.scenario_id,
        "dataset_role": args.dataset_role,
        "run_id": os.getenv("SWARM_RANGE_DATASET_RUN_ID"),
        "session_id": session_id,
        "prewarm_session_id": prewarm_session_id,
        "tracking_workflow_session_id": tracking_workflow_session_id,
        "group_id": f"UAV-01.{session_id}",
        "root": str(root),
        "distance_m_nominal": args.distance,
        "manual_gimbal_pitch_deg": args.pitch,
        "prewarm": prewarm,
        "bbox_normalized": bbox,
        "raw_rows_before_perturbation": len(raw_before_perturbation),
        "raw_rows_final": len(raw_final),
        "perturbation": perturbation,
        "started_tracking_state": tracking.get("state"),
        "selected_tracking_state": (selected.get("tracking") or {}).get("state"),
        "stopped_tracking_state": (stopped.get("tracking") or {}).get("state"),
        "armed_observations_before": _armed_values(before.get("drones")),
        "armed_observations_after": _armed_values(final_status.get("drones")),
        "follow_endpoint_called": False,
        "motion_or_vehicle_control_endpoint_called": False,
    }
    append_capture_metadata(root, payload)
    return payload


def parser() -> argparse.ArgumentParser:
    value = argparse.ArgumentParser()
    value.add_argument("--root", required=True, type=Path)
    value.add_argument("--scenario-id", required=True)
    value.add_argument(
        "--dataset-role",
        choices=(
            "physical_diagnostic_development",
            "core_range_development",
            "core_range_static_development",
            "core_range_dynamic_development",
        ),
        default="physical_diagnostic_development",
    )
    value.add_argument("--distance", required=True, type=float)
    value.add_argument("--pitch", required=True, type=float)
    value.add_argument("--bbox-x", required=True, type=float)
    value.add_argument("--bbox-y", required=True, type=float)
    value.add_argument("--bbox-width", required=True, type=float)
    value.add_argument("--bbox-height", required=True, type=float)
    value.add_argument("--prewarm", action="store_true")
    value.add_argument("--prewarm-timeout", type=float, default=90.0)
    value.add_argument("--minimum-raw-rows", type=int, default=12)
    value.add_argument("--before-perturbation-rows", type=int, default=8)
    value.add_argument("--capture-timeout", type=float, default=90.0)
    value.add_argument("--perturbation", choices=("none", "cache_pitch_zero", "pitch_minus_two", "reacquire_not_forced"), default="none")
    return value


def main() -> int:
    args = parser().parse_args()
    result = capture(args)
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
