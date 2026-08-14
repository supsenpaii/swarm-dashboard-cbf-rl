"""Observation-only dynamic capture using only a Gazebo target pose script."""

from __future__ import annotations

import argparse
import asyncio
import json
import math
import os
from pathlib import Path
from statistics import median
import subprocess
import time
from typing import Any

from sim_time_trajectory import (
    TRAJECTORY_CONTRACT_SHA256,
    TRAJECTORY_DRIVER_VERSION,
    contract_from_args,
)

from metric_depth_calibrator import MetricDepthCalibrator

from range_v2_r3_6a_capture import (
    _armed_values,
    _request,
    append_capture_metadata,
    assert_observation_only,
    session_rows,
    set_gimbal_pitch,
    sidecar_rows,
    status,
    wait_for_new_sidecar_session,
    wait_for_observation_only_ready,
    wait_for_prewarm,
    wait_for_raw_rows,
)


GZ_ENV = "/mnt/px4ssd/PX4-Autopilot/build/px4_sitl_default/rootfs/gz_env.sh"

# Anchor yield has to clear the calibrator's floor by enough that ordinary
# frame-to-frame variation cannot dip under it, and almost no frame may already
# be under. Stage 2A separates cleanly here: healthy attempts held 56--60
# anchors with no starved frame, while every lost attempt either sat at 0--30
# anchors or spent 12.8--80% of its frames below the floor.
GROUND_ANCHOR_YIELD_FACTOR = 3.0
GROUND_ANCHOR_STARVED_FRAME_FRACTION = 0.05


def ground_anchor_readiness(root: Path, session_id: int) -> dict[str, Any]:
    """Report whether ground anchors are healthy enough to start a capture.

    `m52_adapter` turns each grid ray into a ground intersection at
    `(ground_down_m - camera_z) / ray_down` and drops the anchor once that range
    falls below `minimum_range_m`. A camera `h` above the ground plane therefore
    loses every ray steeper than `asin(h)`, so a low camera starves the fit --
    that geometry is what explains the Stage 2A losses.

    The gate itself measures the anchor count instead of predicting it from `h`.
    Predicting needs the gimbal pitch, the lens and the scene to all hold, and a
    camera at 0.529 m still kept 56 of 60 anchors -- comfortably clear of the
    floor -- so a purely geometric threshold rejects usable runs.

    Stage 2A lost six attempts to anchor starvation, reported as `gt_coverage`,
    prewarm timeouts and missing raw frames, because the PX4 local position
    estimate re-initialises per stack restart and nothing checked anchor health
    before a capture spent its prewarm and motion budget.
    """

    counts: list[int] = []
    altitudes: list[float] = []
    for row in session_rows(root, session_id):
        anchors = row.get("anchors") or {}
        count = anchors.get("accepted_ground_anchor_count")
        if count is not None:
            counts.append(int(count))
        position = (row.get("extrinsics") or {}).get("camera_position_ned_m")
        config = anchors.get("adapter_config") or {}
        if isinstance(position, (list, tuple)) and len(position) == 3:
            altitudes.append(float(config.get("ground_down_m", 0.0)) - float(position[2]))

    if not counts:
        # Never silently pass: an unmeasurable precondition is not a met one.
        return {"satisfied": False, "reason": "ground_anchor_counts_unavailable"}

    minimum_anchors = MetricDepthCalibrator().minimum_anchors
    required = GROUND_ANCHOR_YIELD_FACTOR * minimum_anchors
    starved_fraction = sum(1 for count in counts if count < minimum_anchors) / len(counts)
    median_count = median(counts)
    satisfied = (
        median_count >= required
        and starved_fraction <= GROUND_ANCHOR_STARVED_FRAME_FRACTION
    )
    if satisfied:
        reason = "ok"
    elif median_count < required:
        reason = "ground_anchor_yield_below_minimum"
    else:
        reason = "ground_anchor_starved_frames_above_limit"
    return {
        "satisfied": satisfied,
        "reason": reason,
        "median_anchor_count": median_count,
        "required_anchor_count": required,
        "minimum_anchors": minimum_anchors,
        "starved_frame_fraction": starved_fraction,
        "starved_frame_limit": GROUND_ANCHOR_STARVED_FRAME_FRACTION,
        "camera_altitude_m": median(altitudes) if altitudes else None,
        "sample_count": len(counts),
    }


def set_target_pose(range_m: float, lateral_m: float, z_m: float, yaw_deg: float) -> dict[str, Any]:
    if not 3.0 <= float(range_m) <= 12.0:
        raise ValueError("target_range_outside_core_domain")
    if abs(float(lateral_m)) > 2.0 or not 0.5 <= float(z_m) <= 1.5 or abs(float(yaw_deg)) > 30.0:
        raise ValueError("target_pose_outside_precommitted_bounds")
    x_squared = float(range_m) ** 2 - float(lateral_m) ** 2
    if x_squared <= 0.0:
        raise ValueError("target_lateral_exceeds_range")
    x_m = math.sqrt(x_squared)
    yaw_rad = math.radians(float(yaw_deg))
    request = (
        f'name: "x500_custom_1" position {{x: {x_m:.9f} y: {float(lateral_m):.9f} z: {float(z_m):.9f}}} '
        f'orientation {{z: {math.sin(0.5 * yaw_rad):.12f} w: {math.cos(0.5 * yaw_rad):.12f}}}'
    )
    command = [
        "bash", "-c",
        'set -euo pipefail; source "$1"; gz service -s /world/default/set_pose '
        '--reqtype gz.msgs.Pose --reptype gz.msgs.Boolean --timeout 5000 --req "$2"',
        "_", GZ_ENV, request,
    ]
    failures = []
    for attempt in range(1, 4):
        completed = subprocess.run(command, check=False, text=True, capture_output=True)
        if completed.returncode == 0 and "data: true" in completed.stdout:
            return {
                "range_m": float(range_m), "x_m": x_m,
                "y_m": float(lateral_m), "z_m": float(z_m),
                "yaw_deg": float(yaw_deg), "gazebo_service_attempts": attempt,
            }
        failures.append({
            "attempt": attempt,
            "returncode": completed.returncode,
            "stdout_tail": completed.stdout[-200:],
            "stderr_tail": completed.stderr[-200:],
        })
        if attempt < 3:
            time.sleep(0.1)
    raise RuntimeError(f"gazebo_target_pose_failed_after_retries:{failures}")


def run_legacy_wall_clock_trajectory(args: argparse.Namespace, event_path: Path) -> dict[str, Any]:
    duration = float(args.movement_duration_s)
    interval = 1.0 / float(args.pose_update_rate_hz)
    start = time.monotonic()
    command_count = 0
    with event_path.open("a", encoding="utf-8") as stream:
        while True:
            now = time.monotonic()
            progress = min(1.0, max(0.0, (now - start) / duration))
            range_m = args.start_range_m + progress * (args.end_range_m - args.start_range_m)
            yaw_deg = args.yaw_start_deg + progress * (args.yaw_end_deg - args.yaw_start_deg)
            pose = set_target_pose(range_m, args.lateral_m, args.target_z_m, yaw_deg)
            event = {
                "kind": "target_pose_script_step", "command_monotonic_s": time.monotonic(),
                "progress": progress, **pose,
                "model_output_used": False, "gazebo_entity": "x500_custom_1",
            }
            stream.write(json.dumps(event, sort_keys=True, allow_nan=False) + "\n")
            stream.flush()
            command_count += 1
            if command_count % 5 == 0:
                assert_observation_only(status())
            if progress >= 1.0:
                break
            sleep_until = start + command_count * interval
            time.sleep(max(0.0, sleep_until - time.monotonic()))
        hold_start = time.monotonic()
        while time.monotonic() - hold_start < float(args.hold_duration_s):
            pose = set_target_pose(args.end_range_m, args.lateral_m, args.target_z_m, args.yaw_end_deg)
            event = {
                "kind": "target_pose_script_hold", "command_monotonic_s": time.monotonic(),
                "progress": 1.0, **pose,
                "model_output_used": False, "gazebo_entity": "x500_custom_1",
            }
            stream.write(json.dumps(event, sort_keys=True, allow_nan=False) + "\n")
            stream.flush()
            command_count += 1
            if command_count % 5 == 0:
                assert_observation_only(status())
            time.sleep(interval)
    return {
        "driver": "legacy_wall_clock_set_pose",
        "movement_started_monotonic_s": start,
        "movement_completed_monotonic_s": start + duration,
        "trajectory_completed_monotonic_s": time.monotonic(),
        "command_count": command_count,
    }


def _sim_time_command_subscriber_present() -> bool:
    """`gz topic -p` opens a brand-new transport node per invocation and
    exits immediately after sending; if the plugin's subscriber has not
    finished gz-transport's async peer discovery yet, that one message is
    silently dropped (no delivery guarantee, no retry). Poll `gz topic -i`
    for a non-empty Subscribers section so the first publish is not sent
    into that discovery window."""
    completed = subprocess.run(
        [
            "bash", "-c",
            'set -euo pipefail; source "$1"; gz topic -i -t /swarm/sim_time_trajectory/command',
            "_", GZ_ENV,
        ],
        check=False,
        text=True,
        capture_output=True,
        timeout=5,
    )
    return completed.returncode == 0 and "No subscribers" not in completed.stdout


def _wait_for_sim_time_command_subscriber(timeout_s: float = 10.0) -> None:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if _sim_time_command_subscriber_present():
            return
        time.sleep(0.25)
    raise RuntimeError("sim_time_trajectory_subscriber_not_observed")


def _publish_sim_time_command(payload: str, *, wait_for_subscriber: bool = False) -> None:
    if wait_for_subscriber:
        _wait_for_sim_time_command_subscriber()
    request = f'data: "{payload}"'
    completed = subprocess.run(
        [
            "bash", "-c",
            'set -euo pipefail; source "$1"; gz topic -t /swarm/sim_time_trajectory/command '
            '-m gz.msgs.StringMsg -p "$2"',
            "_", GZ_ENV, request,
        ],
        check=False,
        text=True,
        capture_output=True,
        timeout=10,
    )
    if completed.returncode != 0:
        raise RuntimeError(
            "sim_time_trajectory_publish_failed:"
            f"{completed.returncode}:{completed.stderr[-300:]}"
        )


def _trajectory_events(path: Path, session_token: str) -> list[dict[str, Any]]:
    if not path.exists():
        return []
    rows: list[dict[str, Any]] = []
    for line in path.read_text(encoding="utf-8").splitlines():
        try:
            row = json.loads(line)
        except json.JSONDecodeError:
            continue
        if isinstance(row, dict) and row.get("session") == session_token:
            rows.append(row)
    return rows


def run_sim_time_trajectory(args: argparse.Namespace, event_path: Path) -> dict[str, Any]:
    contract = contract_from_args(args)
    session_token = str(args.scenario_id)
    publish_attempts = 0
    activated_event: dict[str, Any] | None = None
    # A fresh `gz topic -p` process has no delivery acknowledgement.  Peer
    # discovery can report the plugin subscriber immediately before the first
    # datagram is still dropped.  Treat the plugin's durable activation event
    # as the ACK and retry only while no activation has been observed.
    for publish_attempts in range(1, 4):
        _publish_sim_time_command(
            contract.command_payload(session_token),
            wait_for_subscriber=True,
        )
        ack_deadline = time.monotonic() + 3.0
        while time.monotonic() < ack_deadline:
            events = _trajectory_events(event_path, session_token)
            activated_event = next(
                (row for row in reversed(events) if row.get("kind") == "trajectory_activated"),
                None,
            )
            if activated_event is not None:
                break
            assert_observation_only(status())
            time.sleep(0.1)
        if activated_event is not None:
            break
    if activated_event is None:
        raise RuntimeError(
            f"sim_time_trajectory_activation_not_acknowledged:{publish_attempts}"
        )
    deadline = time.monotonic() + max(
        float(args.capture_timeout_s),
        4.0 * contract.total_duration_s + 30.0,
    )
    events: list[dict[str, Any]] = []
    while time.monotonic() < deadline:
        events = _trajectory_events(event_path, session_token)
        if any(row.get("kind") == "trajectory_completed" for row in events):
            break
        assert_observation_only(status())
        time.sleep(0.25)  # wait only; target pose is never derived from this clock
    else:
        _publish_sim_time_command(f"stop|{session_token}")
        raise RuntimeError("sim_time_trajectory_completion_timeout")
    _publish_sim_time_command(f"stop|{session_token}")
    activated = activated_event
    completed = next(row for row in reversed(events) if row.get("kind") == "trajectory_completed")
    return {
        "driver": "gazebo_preupdate_sim_time_plugin",
        "clock_domain": "gazebo_sim_time",
        "movement_started_sim_timestamp_s": activated["sim_timestamp_s"],
        "trajectory_completed_sim_timestamp_s": completed["sim_timestamp_s"],
        "event_count": len(events),
        "synchronous_set_pose_command_count": 0,
        "transport_publish_count": publish_attempts + 1,
        "start_publish_attempts": publish_attempts,
        "driver_version": TRAJECTORY_DRIVER_VERSION,
        "trajectory_contract_sha256": TRAJECTORY_CONTRACT_SHA256,
    }


def run_trajectory(args: argparse.Namespace, event_path: Path) -> dict[str, Any]:
    driver = str(getattr(args, "trajectory_driver", "legacy_wall_clock"))
    if driver == "sim_time_plugin":
        return run_sim_time_trajectory(args, event_path)
    if driver != "legacy_wall_clock":
        raise ValueError("trajectory_driver_invalid")
    return run_legacy_wall_clock_trajectory(args, event_path)


def capture(args: argparse.Namespace) -> dict[str, Any]:
    root = args.root.resolve()
    root.mkdir(parents=True, exist_ok=True)
    if os.getenv("SWARM_RANGE_RESIDUAL_MODE", "off").strip().lower() != "off":
        raise RuntimeError("residual_mode_must_be_off")
    if os.getenv("SWARM_RANGE_DATASET_DIR", "").strip() != str(root):
        raise RuntimeError("capture_root_does_not_match_backend_dataset_root")
    before = wait_for_observation_only_ready()
    existing_sessions = {int(row["session_id"]) for row in sidecar_rows(root) if isinstance(row.get("session_id"), int) and int(row["session_id"]) > 0}
    started = _request("POST", "/api/tracking/start", {"drone_id": "UAV-01", "desired_distance_m": args.start_range_m})
    workflow = ((started.get("tracking") or {}).get("follow_workflow") or {})
    tracking_workflow_session_id = int(workflow.get("session_id", 0))
    if tracking_workflow_session_id <= 0:
        raise RuntimeError("tracking_session_id_invalid")
    gimbal = asyncio.run(set_gimbal_pitch(args.pitch_deg))
    prewarm_session_id = wait_for_new_sidecar_session(root, existing_sessions)
    prewarm = wait_for_prewarm(root, prewarm_session_id, args.prewarm_timeout_s)
    if prewarm.get("status") != "stable":
        raise RuntimeError(f"prewarm_not_stable:{prewarm}")
    anchor_readiness = ground_anchor_readiness(root, prewarm_session_id)
    if not anchor_readiness["satisfied"]:
        raise RuntimeError(f"ground_anchor_precondition:{anchor_readiness}")
    sessions_before_bbox = {int(row["session_id"]) for row in sidecar_rows(root) if isinstance(row.get("session_id"), int) and int(row["session_id"]) > 0}
    bbox = {"x": args.bbox_x, "y": args.bbox_y, "width": args.bbox_width, "height": args.bbox_height}
    selected = _request("POST", "/api/tracking/bbox", bbox)
    session_id = wait_for_new_sidecar_session(root, sessions_before_bbox)
    raw_before = wait_for_raw_rows(root, session_id, 5, args.capture_timeout_s)
    trajectory = run_trajectory(args, root / "trajectory_events.jsonl")
    raw_final = wait_for_raw_rows(root, session_id, args.minimum_raw_frames, args.capture_timeout_s)
    time.sleep(1.0)
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
        "start_range_m": args.start_range_m, "end_range_m": args.end_range_m,
        "lateral_m": args.lateral_m, "target_z_m": args.target_z_m,
        "yaw_start_deg": args.yaw_start_deg, "yaw_end_deg": args.yaw_end_deg,
        "movement_duration_s": args.movement_duration_s, "hold_duration_s": args.hold_duration_s,
        "pose_update_rate_hz": args.pose_update_rate_hz,
        "manual_gimbal_pitch_deg": args.pitch_deg,
        "prewarm": prewarm, "gimbal": gimbal, "bbox_normalized": bbox,
        "ground_anchor_readiness": anchor_readiness,
        "raw_rows_before_motion": len(raw_before), "raw_rows_final": len(raw_final),
        "trajectory": trajectory,
        "selected_tracking_state": (selected.get("tracking") or {}).get("state"),
        "stopped_tracking_state": (stopped.get("tracking") or {}).get("state"),
        "armed_observations_before": _armed_values(before.get("drones")),
        "armed_observations_after": _armed_values(final_status.get("drones")),
        "target_motion_source": (
            "gazebo_preupdate_sim_time_plugin"
            if args.trajectory_driver == "sim_time_plugin"
            else "gazebo_set_pose_independent_script"
        ),
        "model_output_used_for_motion": False,
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
        default="core_range_dynamic_development",
        choices=(
            "core_range_dynamic_development",
            "core_range_dynamic_2hz_development",
            "core_range_dynamic_5hz_post_contention_fix",
            "core_range_dynamic_5hz_headless_post_fixes",
            "targeted_dynamic_pilot_stage_2a",
        ),
    )
    value.add_argument("--start-range-m", required=True, type=float)
    value.add_argument("--end-range-m", required=True, type=float)
    value.add_argument("--lateral-m", required=True, type=float)
    value.add_argument("--target-z-m", required=True, type=float)
    value.add_argument("--yaw-start-deg", required=True, type=float)
    value.add_argument("--yaw-end-deg", required=True, type=float)
    value.add_argument("--movement-duration-s", required=True, type=float)
    value.add_argument("--hold-duration-s", required=True, type=float)
    value.add_argument("--pose-update-rate-hz", required=True, type=float)
    value.add_argument(
        "--trajectory-driver",
        choices=("legacy_wall_clock", "sim_time_plugin"),
        default="legacy_wall_clock",
    )
    value.add_argument("--pitch-deg", required=True, type=float)
    value.add_argument("--bbox-x", required=True, type=float)
    value.add_argument("--bbox-y", required=True, type=float)
    value.add_argument("--bbox-width", required=True, type=float)
    value.add_argument("--bbox-height", required=True, type=float)
    value.add_argument("--prewarm-timeout-s", default=90.0, type=float)
    value.add_argument("--minimum-raw-frames", default=40, type=int)
    value.add_argument("--capture-timeout-s", default=30.0, type=float)
    return value


if __name__ == "__main__":
    args = parser().parse_args()
    print(json.dumps(capture(args), indent=2, sort_keys=True))
