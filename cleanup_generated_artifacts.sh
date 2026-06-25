#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DOC_DIR="${ROOT_DIR}/document/docsource"
KEEP_RUN="${KEEP_RUN:-closed_loop_joy_frame_fixed_60_180_20260501_192053}"
UUV_RUNTIME_DIR="${UUV_MUJOCO_RUNTIME_DIR:-${ROOT_DIR}/uuv_mujoco/current}"
if [[ ! -d "${UUV_RUNTIME_DIR}" ]]; then
  echo "[cleanup] active runtime not found: ${UUV_RUNTIME_DIR}" >&2
  echo "[cleanup] Expected uuv_mujoco/current, or set UUV_MUJOCO_RUNTIME_DIR explicitly." >&2
  exit 1
fi

if [[ -d "${DOC_DIR}" ]]; then
  find "${DOC_DIR}" -maxdepth 1 -type d \( \
    -name 'closed_loop*' -o \
    -name 'real_bag_*' -o \
    -name 'gui_autotune_*' -o \
    -name 'gui_replay*' -o \
    -name 'ellipsoid5*' -o \
    -name 'joy_node*' -o \
    -name '*overlay_20260429' -o \
    -name 'real_vs_sim_analysis*' -o \
    -name 'sensor_sign_plots_20260429' -o \
    -name 'thrust_command_comparison_20260430' -o \
    -name 'rcout_*_20260430' -o \
    -name 'post_reverse_mapping_rcout_eval_20260430' -o \
    -name 'depth_hold_neutral_alt_only_70s' \
  \) ! -name "${KEEP_RUN}" -exec rm -rf {} +

  rm -rf "${DOC_DIR}/measurements"

  find "${DOC_DIR}" -maxdepth 1 -type f \( \
    -name '*.aux' -o \
    -name '*.log' -o \
    -name '*.nav' -o \
    -name '*.out' -o \
    -name '*.snm' -o \
    -name '*.toc' \
  \) -delete
fi

find "${ROOT_DIR}/uuv_mujoco" "${ROOT_DIR}/document" \
  -name '__pycache__' -type d -prune -exec rm -rf {} + 2>/dev/null || true

rm -rf "${ROOT_DIR}/__pycache__"

find "${ROOT_DIR}" -path "${ROOT_DIR}/ardupilot" -prune -o \( \
  -name '.DS_Store' -o \
  -name '*.pyc' \
\) -type f -delete

find "${UUV_RUNTIME_DIR}/logs" -type f -name '*.log' -delete 2>/dev/null || true
rm -rf "${ROOT_DIR}/document/measurements"

echo "generated artifacts cleaned; kept ${KEEP_RUN}"
