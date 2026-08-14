#!/usr/bin/env bash
# Execute the frozen subscription matrix in repeat-major order.
set -Eeuo pipefail

if [[ $# -lt 1 || $# -gt 2 ]]; then
  echo "usage: $0 OUTPUT_ROOT [DURATION_S]" >&2
  exit 2
fi

output_root="$1"
duration_s="${2:-600}"
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
configs=(
  S0 S1
  S2_camera_uav01 S2_camera_uav02
  S2_body_imu_uav01 S2_camera_imu_uav01
  S2_body_imu_uav02 S2_camera_imu_uav02
  S2_front_lidar S3 S4 S5
)

mkdir -p "${output_root}"
for repeat in 1 2 3; do
  for config_id in "${configs[@]}"; do
    run_dir="${output_root}/runs/${config_id}_run${repeat}"
    if [[ -f "${run_dir}/exit_code.txt" ]] && \
       [[ "$(<"${run_dir}/exit_code.txt")" == 0 ]]; then
      echo "already complete: ${config_id} repeat ${repeat}"
      continue
    fi
    "${script_dir}/core_range_subscription_ablation_run.sh" \
      "${config_id}" "${repeat}" "${duration_s}" "${output_root}"
  done
done

"${script_dir}/.venv/bin/python" \
  "${script_dir}/core_range_subscription_ablation_analyze.py" \
  --root "${output_root}"
