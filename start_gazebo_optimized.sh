#!/usr/bin/env bash

set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
gazebo_px4_root="${PX4_AUTOPILOT_ROOT:-/mnt/px4ssd/PX4-Autopilot}"
gazebo_world="${gazebo_px4_root}/Tools/simulation/gz/worlds/default.sdf"
gazebo_environment="${gazebo_px4_root}/build/px4_sitl_default/rootfs/gz_env.sh"
gazebo_gui_config="${script_dir}/gazebo_gui_light.config"

if [[ ! -f "${gazebo_world}" || ! -f "${gazebo_environment}" ]]; then
  echo "PX4 Gazebo files not found under: ${gazebo_px4_root}" >&2
  echo "Set PX4_AUTOPILOT_ROOT to the PX4-Autopilot path." >&2
  exit 1
fi

if pgrep -f '^gz sim' >/dev/null; then
  echo "Gazebo is already running. Stop the existing server and GUI first."
  exit 1
fi

source "${gazebo_environment}"

gz sim --verbose=1 -r -s "${gazebo_world}" &
gazebo_server_pid=$!

cleanup_gazebo() {
  if kill -0 "${gazebo_gui_pid:-0}" 2>/dev/null; then
    kill -INT "${gazebo_gui_pid}"
  fi
  if kill -0 "${gazebo_server_pid}" 2>/dev/null; then
    kill -INT "${gazebo_server_pid}"
  fi
}

trap cleanup_gazebo EXIT INT TERM

sleep 1
gz sim -g --gui-config "${gazebo_gui_config}" &
gazebo_gui_pid=$!

echo "Gazebo server PID: ${gazebo_server_pid}"
echo "Gazebo GUI PID:    ${gazebo_gui_pid}"
echo "GUI config:        ${gazebo_gui_config}"

wait "${gazebo_server_pid}"
