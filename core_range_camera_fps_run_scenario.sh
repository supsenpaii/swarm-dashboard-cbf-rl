#!/usr/bin/env bash
# Wraps core_range_contention_dynamic_capture_variant.sh (full
# Gazebo/ROS2/PX4/dashboard stack, observation-only smoke capture, already
# used and proven by CORE_RANGE_LIVE_STACK_CONTENTION_ISOLATION) with:
#   - SWARM_CAMERA_TRACE_JSONL set, so main.py/tracking_web.py write
#     per-hop camera-source trace events (opt-in, additive; see
#     camera_source_trace.py);
#   - SWARM_START_GAZEBO_GUI passed through (opt-in additive run_all.sh
#     env-gate; default true, unchanged behavior);
#   - SWARM_RANGE_DATASET_DEPTH_RATE_HZ passed through when non-empty (the
#     existing production depth-rate knob, main.py:2941-2946);
#   - read-only nvidia-smi dmon/pmon + pidstat + gz world-stats monitors
#     running in parallel for CORE_RANGE_CAMERA_SOURCE_FPS_AUDIT_AND_FIX.
#
# The same representative approaching scenario (11.5m -> 3.5m, 32s, two
# UAVs) is reused across every config -- collection_throughput_fix's own
# A_smoke parameters -- so only the one declared variable per config
# changes.

set -Eeuo pipefail

if [[ $# -ne 4 ]]; then
  echo "usage: $0 LABEL START_GAZEBO_GUI DEPTH_RATE_HZ_OR_EMPTY OUTPUT_ROOT" >&2
  exit 2
fi

label="$1"; start_gazebo_gui="$2"; depth_rate_hz="$3"; output_root="$4"
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
mkdir -p "${output_root}"

monitor_duration_s=300
env -u PYTHONPATH "${script_dir}/.venv/bin/python" "${script_dir}/core_range_camera_source_fps_audit.py" monitor \
  --output "${output_root}" --duration-s "${monitor_duration_s}" --label "${label}" \
  >"${output_root}/monitor_stdout.log" 2>&1 &
monitor_pid=$!

env -u PYTHONPATH "${script_dir}/.venv/bin/python" "${script_dir}/core_range_camera_source_fps_audit.py" gpu-snapshot \
  --output "${output_root}" --label "${label}_before" || true

export SWARM_START_GAZEBO_GUI="${start_gazebo_gui}"
export SWARM_CAMERA_TRACE_JSONL="${output_root}/camera_source_trace.jsonl"
if [[ -n "${depth_rate_hz}" ]]; then
  export SWARM_RANGE_DATASET_DEPTH_RATE_HZ="${depth_rate_hz}"
fi

set +e
"${script_dir}/core_range_contention_dynamic_capture_variant.sh" \
  15 90 \
  "camera_fps_${label}" 11.5 3.5 0.0 1.0 0.0 0.0 32 0.0 5.0 -10.0 \
  0.398 0.155 0.204 0.279 \
  "${output_root}/dynamic_capture"
capture_status=$?
set -e

env -u PYTHONPATH "${script_dir}/.venv/bin/python" "${script_dir}/core_range_camera_source_fps_audit.py" gpu-snapshot \
  --output "${output_root}" --label "${label}_after" || true

if kill -0 "${monitor_pid}" 2>/dev/null; then
  kill -TERM "${monitor_pid}" 2>/dev/null || true
  wait "${monitor_pid}" 2>/dev/null || true
fi

# Move the trace file next to the rest of this run's evidence for
# analyze-run convenience (it is written directly under output_root, the
# same directory dynamic_capture's sibling artifacts live in).
if [[ -f "${output_root}/camera_source_trace.jsonl" && ! -f "${output_root}/dynamic_capture/camera_source_trace.jsonl" ]]; then
  cp "${output_root}/camera_source_trace.jsonl" "${output_root}/dynamic_capture/camera_source_trace.jsonl"
fi

exit "${capture_status}"
