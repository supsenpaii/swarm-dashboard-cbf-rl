#!/usr/bin/env bash
# Wraps the existing core_range_collect_dynamic_scenario.sh (full
# Gazebo/ROS2/PX4/dashboard stack, observation-only smoke capture) with
# read-only nvidia-smi dmon + pidstat resource monitors running in parallel,
# for CORE_RANGE_LIVE_STACK_CONTENTION_ISOLATION configs D and F.
#
# Config F is the SAME stack as D with exactly one variable changed: the
# camera/depth submission rate, via SWARM_RANGE_DATASET_DEPTH_RATE_HZ (see
# main.py:2904-2905). Pass the production default (7.5) for D, a lower
# value (e.g. 2.0) for F.

set -Eeuo pipefail

if [[ $# -ne 18 ]]; then
  echo "usage: $0 LABEL DEPTH_RATE_HZ ID START END LATERAL Z YAW_START YAW_END DURATION HOLD RATE PITCH BBOX_X BBOX_Y BBOX_W BBOX_H OUTPUT" >&2
  exit 2
fi

label="$1"; depth_rate_hz="$2"; scenario_id="$3"; start_range="$4"; end_range="$5"
lateral="$6"; target_z="$7"; yaw_start="$8"; yaw_end="$9"; duration="${10}"
hold="${11}"; rate="${12}"; pitch="${13}"; bbox_x="${14}"; bbox_y="${15}"
bbox_w="${16}"; bbox_h="${17}"; output_root="${18}"

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
mkdir -p "${output_root}"

monitor_duration_s=240
"${script_dir}/.venv/bin/python" "${script_dir}/core_range_live_stack_contention_isolation.py" monitor \
  --output "${output_root}" --duration-s "${monitor_duration_s}" --label "${label}" \
  >"${output_root}/monitor_stdout.log" 2>&1 &
monitor_pid=$!

"${script_dir}/.venv/bin/python" "${script_dir}/core_range_live_stack_contention_isolation.py" gpu-snapshot \
  --output "${output_root}" --label "${label}_before" || true

export SWARM_RANGE_DATASET_DEPTH_RATE_HZ="${depth_rate_hz}"

set +e
# Uses the *_variant.sh wrapper (identical stack-launch behavior to the
# unmodified core_range_collect_dynamic_scenario.sh, just with a raised
# raw-frame/timeout floor) because live-stack inference latency -- the
# thing this task investigates -- can legitimately be too slow to reach
# the original script's fixed 40-frames-in-30s gate.
"${script_dir}/core_range_contention_dynamic_capture_variant.sh" \
  15 90 \
  "${scenario_id}" "${start_range}" "${end_range}" "${lateral}" "${target_z}" \
  "${yaw_start}" "${yaw_end}" "${duration}" "${hold}" "${rate}" "${pitch}" \
  "${bbox_x}" "${bbox_y}" "${bbox_w}" "${bbox_h}" "${output_root}/dynamic_capture"
capture_status=$?
set -e

"${script_dir}/.venv/bin/python" "${script_dir}/core_range_live_stack_contention_isolation.py" gpu-snapshot \
  --output "${output_root}" --label "${label}_after" || true

if kill -0 "${monitor_pid}" 2>/dev/null; then
  kill -TERM "${monitor_pid}" 2>/dev/null || true
  wait "${monitor_pid}" 2>/dev/null || true
fi

exit "${capture_status}"
