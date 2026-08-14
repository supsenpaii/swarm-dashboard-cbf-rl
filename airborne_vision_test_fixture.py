#!/usr/bin/env python3
"""SITL/test-only fixture: hold two disarmed drones airborne via repeated
Gazebo pose teleport, so the real camera/tracker/vision pipeline can be
exercised at a realistic altitude without arming or OFFBOARD.

Why repeated teleport, not one-shot: an unarmed vehicle has no thrust, so a
single `gz service set_pose` call is immediately overridden by gravity in the
next physics step -- the model free-falls back to the ground within about a
second. This mirrors the technique `core_range_dynamic_capture.py` already
uses for repositioning the target model during corpus collection
(`run_legacy_wall_clock_trajectory`'s per-tick `set_target_pose` loop); this
fixture applies the same idea to holding BOTH vehicles at altitude, not just
moving one target.

Known limitation, measured empirically before writing this script: PX4's own
GLOBAL_POSITION_INT/LOCAL_POSITION_NED telemetry does not track a pose
teleport in real time -- it converges slowly (tens of seconds) and can retain
a persistent bias (barometer reference is not reset by the teleport). Do not
use PX4 telemetry as ground truth for a just-teleported vehicle. This
fixture's own hold loop re-publishes the exact commanded Gazebo pose every
cycle, so the Gazebo world pose itself (read back via `gz topic -e
/world/default/pose/info`) is the authoritative ground truth, independent of
whatever PX4 currently believes.

Zero actuation: this script only calls `gz service set_pose`, a Gazebo
simulation-only RPC. It never touches MAVLink, MQTT, arms, or requests
OFFBOARD. Vehicles remain disarmed throughout -- verify with
`/api/drones` before and after any run.

KNOWN LIMITATION, measured empirically (2026-08-08), not yet solved: `hold`
does not sustain a stable altitude for a vehicle several metres up. Each
`gz service set_pose` call spawns a new shell + `gz` process (tens to
hundreds of ms of overhead), which cannot sustain a tight enough publish
rate against gravity once the model has real fall distance to cover -- a
single missed cycle at 8 m lets the model free-fall for a fraction of a
second, and the *next* teleport then snaps it back, producing large visible
oscillation (observed swinging between ~1.8 m and ~8 m) rather than a hold.
This is consistent with `core_range_dynamic_capture.py`'s own documented
reason for building a proper in-loop Gazebo plugin
(`gazebo_preupdate_sim_time_plugin`, driving `/swarm/sim_time_trajectory`)
instead of relying on the legacy per-call subprocess path for anything but
a low, near-ground target -- see `sim_time_trajectory.py`, whose contract is
hardcoded to a single entity (`x500_custom_1`) and a low target-height
envelope (0.5-1.5 m), and was never built to hold two vehicles at realistic
flight altitude. Reaching a stable multi-metre hold for both vehicles would
need an equivalent in-loop plugin (or gravity disabled on the models), which
is out of scope for a subprocess-based Python script. Treat `hold` as a
diagnostic tool for reproducing this limitation, not as a working airborne
fixture, until that gap is closed.
"""

from __future__ import annotations

import argparse
import json
import math
import signal
import subprocess
import sys
import threading
import time
from dataclasses import dataclass

GZ_ENV = "/mnt/px4ssd/PX4-Autopilot/build/px4_sitl_default/rootfs/gz_env.sh"
API_URL = "http://127.0.0.1:8000/api/drones"


@dataclass(frozen=True)
class Pose:
    model: str
    x: float
    y: float
    z: float
    # Yaw-only quaternion around Z, in Gazebo's math convention (0 = facing
    # +X/East, counter-clockwise positive) -- NOT PX4's compass convention.
    # Use compass_to_math_yaw_deg() to convert from a bearing in degrees.
    yaw_rad: float = 0.0

    def request(self) -> str:
        qz = math.sin(0.5 * self.yaw_rad)
        qw = math.cos(0.5 * self.yaw_rad)
        return (
            f'name: "{self.model}" '
            f"position {{x: {self.x:.9f} y: {self.y:.9f} z: {self.z:.9f}}} "
            f"orientation {{z: {qz:.12f} w: {qw:.12f}}}"
        )


def compass_to_math_yaw_deg(compass_deg: float) -> float:
    """PX4/compass yaw (0=North, clockwise) -> Gazebo quaternion yaw
    (0=+X/East, counter-clockwise). Derived and verified empirically: at
    compass 0 deg (facing North) the model should face world +Y, i.e. math
    yaw 90 deg; at compass 90 deg (East) math yaw is 0 deg."""
    return 90.0 - compass_deg


def gz_set_pose(pose: Pose) -> bool:
    command = [
        "bash", "-c",
        'set -euo pipefail; source "$1"; gz service -s /world/default/set_pose '
        '--reqtype gz.msgs.Pose --reptype gz.msgs.Boolean --timeout 2000 --req "$2"',
        "_", GZ_ENV, pose.request(),
    ]
    completed = subprocess.run(command, check=False, text=True, capture_output=True)
    return completed.returncode == 0 and "data: true" in completed.stdout


def gz_read_poses(model_names: tuple[str, ...], timeout_s: float = 3.0) -> dict[str, tuple[float, float, float]]:
    """Read-only ground truth: one live sample from Gazebo's own pose topic."""
    command = [
        "bash", "-c",
        f'set -euo pipefail; source "$1"; timeout {timeout_s} gz topic -e -t /world/default/pose/info -n 1',
        "_", GZ_ENV,
    ]
    completed = subprocess.run(command, check=False, text=True, capture_output=True, timeout=timeout_s + 2)
    text = completed.stdout
    result: dict[str, tuple[float, float, float]] = {}
    for name in model_names:
        marker = f'name: "{name}"'
        index = text.find(marker)
        if index == -1:
            continue
        block = text[index:index + 400]
        try:
            x = float(block.split("x: ")[1].split("\n")[0])
            y = float(block.split("y: ")[1].split("\n")[0])
            z = float(block.split("z: ")[1].split("\n")[0])
        except (IndexError, ValueError):
            continue
        result[name] = (x, y, z)
    return result


def armed_status() -> dict[str, bool]:
    command = ["curl", "-fsS", "-m", "3", API_URL]
    completed = subprocess.run(command, check=False, text=True, capture_output=True)
    if completed.returncode != 0:
        raise RuntimeError("api_unreachable")
    payload = json.loads(completed.stdout)
    return {
        drone_id: bool(info["status"]["armed"])
        for drone_id, info in payload["drones"].items()
    }


class HoldLoop:
    """Background thread that keeps re-publishing fixed poses at a steady
    rate, since an unarmed model has no thrust and free-falls between calls."""

    def __init__(self, poses: tuple[Pose, ...], rate_hz: float = 15.0) -> None:
        self.poses = poses
        self.period_s = 1.0 / rate_hz
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self.publish_count = 0
        self.failure_count = 0

    def _run(self) -> None:
        while not self._stop.is_set():
            cycle_start = time.monotonic()
            for pose in self.poses:
                if gz_set_pose(pose):
                    self.publish_count += 1
                else:
                    self.failure_count += 1
            elapsed = time.monotonic() - cycle_start
            time.sleep(max(0.0, self.period_s - elapsed))

    def start(self) -> None:
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        self._thread.join(timeout=3.0)


def cmd_hold(args: argparse.Namespace) -> int:
    if any(armed_status().values()):
        print("REFUSING: a vehicle is armed", file=sys.stderr)
        return 2
    observer_yaw = math.radians(
        compass_to_math_yaw_deg(args.observer_bearing_to_target_deg)
    )
    poses = (
        Pose("x500_custom_0", 0.0, 0.0, args.observer_altitude_m, observer_yaw),
        Pose("x500_custom_1", 0.0, args.horizontal_separation_m, args.target_altitude_m),
    )
    loop = HoldLoop(poses, rate_hz=args.rate_hz)
    loop.start()
    print(
        f"holding: observer z={args.observer_altitude_m}m yaw_compass="
        f"{args.observer_bearing_to_target_deg}deg, target z={args.target_altitude_m}m "
        f"at {args.horizontal_separation_m}m north, rate={args.rate_hz}Hz",
        file=sys.stderr,
    )

    def handle_signal(signum: int, frame: object) -> None:
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, handle_signal)
    try:
        deadline = time.monotonic() + args.duration_s if args.duration_s > 0 else None
        while deadline is None or time.monotonic() < deadline:
            time.sleep(1.0)
            ground_truth = gz_read_poses(("x500_custom_0", "x500_custom_1"))
            armed = armed_status()
            if any(armed.values()):
                print(f"ABORT: armed detected mid-hold: {armed}", file=sys.stderr)
                break
            print(
                json.dumps(
                    {
                        "monotonic_s": round(time.monotonic(), 2),
                        "ground_truth": ground_truth,
                        "publish_count": loop.publish_count,
                        "failure_count": loop.failure_count,
                        "armed": armed,
                    }
                )
            )
    except KeyboardInterrupt:
        pass
    finally:
        loop.stop()
        print(f"stopped: published={loop.publish_count} failed={loop.failure_count}", file=sys.stderr)
    return 0


def cmd_read_pose(args: argparse.Namespace) -> int:
    print(json.dumps(gz_read_poses(("x500_custom_0", "x500_custom_1")), indent=2))
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)

    hold = sub.add_parser("hold", help="continuously hold both drones airborne")
    hold.add_argument("--observer-altitude-m", type=float, default=8.0)
    hold.add_argument("--target-altitude-m", type=float, default=5.0)
    hold.add_argument("--horizontal-separation-m", type=float, default=5.0)
    hold.add_argument("--observer-bearing-to-target-deg", type=float, default=0.0)
    hold.add_argument("--rate-hz", type=float, default=15.0)
    hold.add_argument(
        "--duration-s", type=float, default=0.0,
        help="0 means hold until Ctrl-C / SIGTERM",
    )
    hold.set_defaults(func=cmd_hold)

    read = sub.add_parser("read-pose", help="read-only: print current Gazebo ground truth")
    read.set_defaults(func=cmd_read_pose)

    args = parser.parse_args()
    return args.func(args)


if __name__ == "__main__":
    raise SystemExit(main())
