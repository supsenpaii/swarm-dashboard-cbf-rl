#!/usr/bin/env bash
# CORE_RANGE_GAZEBO_RTF_STUTTER_AND_GATE_VALIDATION section 4 ablations
# B/C/D/F/G: single-variable-at-a-time variants of
# core_range_gazebo_rtf_long_run.sh. Always headless
# (SWARM_START_GAZEBO_GUI=false unconditional). Ablations A (full baseline)
# and G (repeat same scenario many times) are directly the unmodified
# long-run script itself -- not duplicated here. Ablation E (restart
# Gazebo between sessions) reuses the prior 5Hz-headless task's own
# 13-attempt evidence (every attempt was already a fresh full-stack
# relaunch) rather than repeating an equivalent, expensive run.

set -Eeuo pipefail

if [[ $# -ne 6 ]]; then
  echo "usage: $0 LABEL MIN_DURATION_S REP_DURATION_S DIAGNOSTICS_WRITE_MODE DEPTH_RATE_HZ OUTPUT_ROOT" >&2
  echo "  DIAGNOSTICS_WRITE_MODE: disk | memory" >&2
  echo "  DEPTH_RATE_HZ: 5.0 (baseline-equivalent) or a low value like 0.1 (ablation D, depth functionally off)" >&2
  exit 2
fi

label="$1"; min_duration_s="$2"; rep_duration_s="$3"
diagnostics_write_mode="$4"; depth_rate_hz="$5"; output_root="$6"
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
stack_pid=""
monitor_pid=""

if [[ -e "${output_root}" ]]; then
  echo "refusing to overwrite existing root: ${output_root}" >&2
  exit 3
fi
mkdir -p "${output_root}/reps"

stop_all() {
  if [[ -n "${monitor_pid}" ]] && kill -0 "${monitor_pid}" 2>/dev/null; then
    kill -TERM "${monitor_pid}" 2>/dev/null || true
    wait "${monitor_pid}" 2>/dev/null || true
  fi
  monitor_pid=""
  if [[ -n "${stack_pid}" ]] && kill -0 "${stack_pid}" 2>/dev/null; then
    kill -TERM "${stack_pid}" 2>/dev/null || true
    local deadline=$((SECONDS + 20))
    while kill -0 "${stack_pid}" 2>/dev/null && ((SECONDS < deadline)); do sleep 0.25; done
    if kill -0 "${stack_pid}" 2>/dev/null; then kill -KILL "${stack_pid}" 2>/dev/null || true; fi
    wait "${stack_pid}" 2>/dev/null || true
  fi
  stack_pid=""
}
cleanup() { local code=$?; trap - EXIT INT TERM; stop_all; exit "${code}"; }
trap cleanup EXIT INT TERM

export SWARM_START_GAZEBO_GUI=false
export SWARM_CAMERA_TRACE_JSONL="${output_root}/camera_source_trace.jsonl"

env \
  SWARM_RANGE_DATASET_DIR="${output_root}/shared" \
  SWARM_RANGE_PHYSICAL_DIAGNOSTICS_DIR="${output_root}/shared" \
  SWARM_RANGE_DATASET_RUN_ID="core_rtf_ablation_${label}" \
  SWARM_RANGE_DATASET_TARGET_ID=UAV-02 \
  SWARM_RANGE_DATASET_DEPTH_RATE_HZ="${depth_rate_hz}" \
  SWARM_RANGE_DATASET_WRITE_MODE="${diagnostics_write_mode}" \
  SWARM_RANGE_DIAGNOSTICS_WRITE_MODE="${diagnostics_write_mode}" \
  SWARM_RANGE_RESIDUAL_MODE=off \
  SWARM_UAV_01_MODEL_POSE="0,0,1,0,0,0" \
  SWARM_UAV_02_MODEL_POSE="11.5,0,1,0,0,0" \
  SWARM_LOG_DIR="${output_root}/runtime_logs" \
  SWARM_RUNTIME_LOG_MAX_BYTES=8388608 \
  "${script_dir}/run_all.sh" >"${output_root}/stack_supervisor.log" 2>&1 &
stack_pid=$!

deadline=$((SECONDS + 60))
while ((SECONDS < deadline)); do
  if curl --fail --silent --max-time 1 http://127.0.0.1:8000/health >/dev/null 2>&1; then break; fi
  if ! kill -0 "${stack_pid}" 2>/dev/null; then echo "stack exited before health gate" >&2; exit 4; fi
  sleep 0.5
done
curl --fail --silent --max-time 1 http://127.0.0.1:8000/health >/dev/null

if pgrep -f "gz sim -g" >/dev/null 2>&1; then
  echo "gazebo GUI client detected despite SWARM_START_GAZEBO_GUI=false" >&2
  exit 8
fi

env PYTHONPATH="${script_dir}/.venv/lib/python3.12/site-packages" \
  "${script_dir}/.venv/bin/python" -c \
  'from range_v2_r3_6a_capture import wait_for_observation_only_ready; wait_for_observation_only_ready()'

setup_log="${output_root}/static_environment_services.log"
bash -c '
  set -euo pipefail; source "$1"
  gz service -s /world/default/set_physics --reqtype gz.msgs.Physics --reptype gz.msgs.Boolean --timeout 5000 --req "gravity {x: 0 y: 0 z: 0}"
  gz service -s /world/default/set_pose --reqtype gz.msgs.Pose --reptype gz.msgs.Boolean --timeout 5000 --req "name: \"x500_custom_0\" position {x: 0 y: 0 z: 1} orientation {w: 1}"
  gz service -s /world/default/set_pose --reqtype gz.msgs.Pose --reptype gz.msgs.Boolean --timeout 5000 --req "name: \"x500_custom_1\" position {x: 11.5 y: 0 z: 1} orientation {w: 1}"
' _ /mnt/px4ssd/PX4-Autopilot/build/px4_sitl_default/rootfs/gz_env.sh >"${setup_log}"
if [[ "$(grep -c 'data: true' "${setup_log}")" -ne 3 ]]; then echo "static setup failed" >&2; exit 6; fi
sleep 2

env -u PYTHONPATH "${script_dir}/.venv/bin/python" "${script_dir}/core_range_gazebo_rtf_long_run_monitor.py" \
  --output "${output_root}" --duration-s "$((min_duration_s + 180))" \
  >"${output_root}/monitor_stdout.log" 2>&1 &
monitor_pid=$!

# See core_range_gazebo_rtf_long_run.sh for why every rep must target the
# SAME shared diagnostics/dataset directory the backend was launched with
# (a per-rep directory here makes wait_for_new_sidecar_session poll a path
# the already-running backend never writes to).
shared_root="${output_root}/shared"
run_started=$SECONDS
rep=0
direction="approach"
while (( SECONDS - run_started < min_duration_s )); do
  rep=$((rep + 1))
  rep_label="${output_root}/reps/rep_$(printf '%02d' "${rep}")_${direction}"
  mkdir -p "${output_root}/reps"
  if [[ "${direction}" == "approach" ]]; then
    start=11.5; end=3.5; bbox_x=0.465; bbox_y=0.25; bbox_w=0.07; bbox_h=0.095
  else
    start=3.5; end=11.5; bbox_x=0.398; bbox_y=0.155; bbox_w=0.204; bbox_h=0.279
  fi
  echo "REP ${rep} direction=${direction} start=${start} end=${end} elapsed_s=$((SECONDS - run_started))" | tee -a "${output_root}/rep_log.txt"

  set +e
  env \
    SWARM_RANGE_DATASET_DIR="${shared_root}" \
    SWARM_RANGE_PHYSICAL_DIAGNOSTICS_DIR="${shared_root}" \
    SWARM_RANGE_DATASET_RUN_ID="core_rtf_ablation_${label}_rep_${rep}" \
    SWARM_RANGE_DATASET_TARGET_ID=UAV-02 \
    SWARM_RANGE_DATASET_DEPTH_RATE_HZ="${depth_rate_hz}" \
    SWARM_RANGE_DATASET_WRITE_MODE="${diagnostics_write_mode}" \
    SWARM_RANGE_DIAGNOSTICS_WRITE_MODE="${diagnostics_write_mode}" \
    SWARM_RANGE_RESIDUAL_MODE=off \
    "${script_dir}/.venv/bin/python" "${script_dir}/core_range_dynamic_capture.py" \
    --root "${shared_root}" --scenario-id "rtf_ablation_${label}_rep_${rep}_${direction}" \
    --dataset-role core_range_dynamic_5hz_headless_post_fixes \
    --start-range-m "${start}" --end-range-m "${end}" \
    --lateral-m 0.0 --target-z-m 1.0 \
    --yaw-start-deg 0.0 --yaw-end-deg 0.0 \
    --movement-duration-s "${rep_duration_s}" --hold-duration-s 0.0 \
    --pose-update-rate-hz 5.0 --pitch-deg -10.0 \
    --bbox-x "${bbox_x}" --bbox-y "${bbox_y}" --bbox-width "${bbox_w}" --bbox-height "${bbox_h}" \
    --minimum-raw-frames 15 --capture-timeout-s 300 \
    >"${rep_label}.capture_result.json" 2>"${rep_label}.capture_stderr.log"
  rep_status=$?
  set -e
  echo "REP ${rep} status=${rep_status}" | tee -a "${output_root}/rep_log.txt"

  if [[ "${direction}" == "approach" ]]; then direction="recede"; else direction="approach"; fi
done

echo "ablation ${label} capture loop complete: ${rep} reps, elapsed $((SECONDS - run_started))s" | tee -a "${output_root}/rep_log.txt"

stop_all
