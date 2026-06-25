#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [[ -f "${ROOT_DIR}/../.uuv_mujoco_env.sh" ]]; then
  # shellcheck source=/dev/null
  source "${ROOT_DIR}/../.uuv_mujoco_env.sh"
fi

RUNTIME_DIR="${UUV_MUJOCO_RUNTIME_DIR:-${ROOT_DIR}/current}"

if [[ "$(basename "${RUNTIME_DIR}")" == "v2.2" ]]; then
  echo "[uuv_mujoco] refusing direct compatibility runtime: ${RUNTIME_DIR}" >&2
  echo "[uuv_mujoco] use ${ROOT_DIR}/current so freshness and launch contracts stay active" >&2
  exit 2
fi

resolve_python() {
  if [[ -n "${MJ311_PYTHON:-}" ]]; then
    if [[ -x "$MJ311_PYTHON" ]]; then
      printf '%s\n' "$MJ311_PYTHON"
      return 0
    fi
    echo "[uuv_mujoco] MJ311_PYTHON is not executable: ${MJ311_PYTHON}" >&2
    return 1
  fi
  if [[ -n "${MJ311_ROOT:-}" && -x "${MJ311_ROOT}/bin/python" ]]; then
    printf '%s\n' "${MJ311_ROOT}/bin/python"
    return 0
  fi
  if [[ -x "$HOME/.venvs/uuv_mujoco/bin/python" ]]; then
    printf '%s\n' "$HOME/.venvs/uuv_mujoco/bin/python"
    return 0
  fi
  if [[ -x "$HOME/.venvs/mujoco311/bin/python" ]]; then
    printf '%s\n' "$HOME/.venvs/mujoco311/bin/python"
    return 0
  fi
  if [[ -n "${PYTHON:-}" ]]; then
    printf '%s\n' "$PYTHON"
    return 0
  fi
  command -v python3
}

if [[ ! -f "${RUNTIME_DIR}/run_uuv_mujoco.py" ]]; then
  echo "[uuv_mujoco] active runtime runner not found: ${RUNTIME_DIR}/run_uuv_mujoco.py" >&2
  echo "[uuv_mujoco] recreate uuv_mujoco/current or set UUV_MUJOCO_RUNTIME_DIR" >&2
  exit 2
fi

PYTHON_BIN="$(resolve_python)"
if [[ "${UUV_MUJOCO_SKIP_FRESHNESS_CHECK:-0}" != "1" ]]; then
  "$PYTHON_BIN" "${RUNTIME_DIR}/tools/check_runtime_freshness.py" \
    --workspace "${ROOT_DIR}/.." \
    --runtime-dir "${RUNTIME_DIR}" \
    --fetch \
    --refresh-version \
    --warn-only
fi
cd "$RUNTIME_DIR"
exec "$PYTHON_BIN" run_uuv_mujoco.py "$@"
