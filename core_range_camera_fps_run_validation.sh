#!/usr/bin/env bash
# Post-fix validation runner for CORE_RANGE_CAMERA_SOURCE_FPS_AUDIT_AND_FIX:
# same wiring as core_range_camera_fps_run_scenario.sh (camera trace, GUI
# toggle, depth-rate knob, resource monitors) but with the full trajectory
# exposed so approaching/receding/stop-and-hold can each use their own
# parameters (reused from docs/CORE_RANGE_FULL_STACK_CONTENTION_FIX_REPORT.md's
# own smoke trajectories).

set -Eeuo pipefail

if [[ $# -ne 17 ]]; then
  echo "usage: $0 LABEL START_GAZEBO_GUI DEPTH_RATE_HZ START END LATERAL Z YAW_START YAW_END DURATION HOLD RATE PITCH BBOX_X BBOX_Y BBOX_W BBOX_H" >&2
  exit 2
fi

label="$1"; start_gazebo_gui="$2"; depth_rate_hz="$3"
start_range="$4"; end_range="$5"; lateral="$6"; target_z="$7"
yaw_start="$8"; yaw_end="$9"; duration="${10}"; hold="${11}"; rate="${12}"; pitch="${13}"
bbox_x="${14}"; bbox_y="${15}"; bbox_w="${16}"; bbox_h="${17}"

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
output_root="${script_dir}/artifacts/core_range_3_12m/camera_source_fps/validation/${label}"
mkdir -p "${output_root}"

monitor_duration_s=300
env -u PYTHONPATH "${script_dir}/.venv/bin/python" "${script_dir}/core_range_camera_source_fps_audit.py" monitor \
  --output "${output_root}" --duration-s "${monitor_duration_s}" --label "${label}" \
  >"${output_root}/monitor_stdout.log" 2>&1 &
monitor_pid=$!

export SWARM_START_GAZEBO_GUI="${start_gazebo_gui}"
export SWARM_CAMERA_TRACE_JSONL="${output_root}/camera_source_trace.jsonl"
export SWARM_RANGE_DATASET_DEPTH_RATE_HZ="${depth_rate_hz}"

set +e
"${script_dir}/core_range_contention_dynamic_capture_variant.sh" \
  15 90 \
  "camera_fps_validation_${label}" "${start_range}" "${end_range}" "${lateral}" "${target_z}" \
  "${yaw_start}" "${yaw_end}" "${duration}" "${hold}" "${rate}" "${pitch}" \
  "${bbox_x}" "${bbox_y}" "${bbox_w}" "${bbox_h}" \
  "${output_root}/dynamic_capture"
capture_status=$?
set -e

if kill -0 "${monitor_pid}" 2>/dev/null; then
  kill -TERM "${monitor_pid}" 2>/dev/null || true
  wait "${monitor_pid}" 2>/dev/null || true
fi

if [[ -f "${output_root}/camera_source_trace.jsonl" && ! -f "${output_root}/dynamic_capture/camera_source_trace.jsonl" ]]; then
  cp "${output_root}/camera_source_trace.jsonl" "${output_root}/dynamic_capture/camera_source_trace.jsonl"
fi

exit "${capture_status}"
