#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [[ -f "${ROOT_DIR}/environment.sh" ]]; then
  # shellcheck source=/dev/null
  source "${ROOT_DIR}/environment.sh"
fi

RUNTIME_DIR="${UUV_MUJOCO_RUNTIME_DIR:-${ROOT_DIR}/current}"
SCRIPT="${RUNTIME_DIR}/reset_uuv_sim.sh"

if [[ ! -x "$SCRIPT" ]]; then
  echo "[uuv_mujoco] active reset script not found or not executable: ${SCRIPT}" >&2
  echo "[uuv_mujoco] recreate sim/current or set UUV_MUJOCO_RUNTIME_DIR" >&2
  exit 2
fi

if [[ "${UUV_MUJOCO_SKIP_FRESHNESS_CHECK:-0}" != "1" ]] && command -v python3 >/dev/null 2>&1; then
  python3 "${RUNTIME_DIR}/tools/check_runtime_freshness.py" \
    --workspace "${ROOT_DIR}/.." \
    --runtime-dir "${RUNTIME_DIR}" \
    --fetch \
    --refresh-version \
    --warn-only
fi
exec "$SCRIPT" "$@"
