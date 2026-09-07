#!/usr/bin/env bash
set -euo pipefail

ros_setup="${SWARM_ROS_SETUP:-/home/sup/ws_px4/install/setup.bash}"

if [[ ! -f "${ros_setup}" ]]; then
  echo "ROS setup file not found: ${ros_setup}" >&2
  echo "Set SWARM_ROS_SETUP to the workspace install/setup.bash path." >&2
  exit 1
fi

source "${ros_setup}"

# Keep PX4 DDS discovery on this machine. Another PX4 system on the LAN may
# publish the same root /fmu topics and corrupt UAV-01 telemetry/control state.
export ROS_LOCALHOST_ONLY=1

exec ros2 launch swarm_telemetry two_uav_nodes.launch.py
