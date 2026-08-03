#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VGL_DEVICE="${UUV_VGL_DEVICE:-egl0}"

if ! command -v vglrun >/dev/null 2>&1; then
  echo "[fast] vglrun is required for the GPU viewer but was not found." >&2
  echo "[fast] Install VirtualGL or run start_sitl_mujoco_mj311.sh --competition-fast directly." >&2
  exit 127
fi

echo "[fast] starting 2x competition validation mode with VirtualGL device ${VGL_DEVICE}"
exec vglrun -d "$VGL_DEVICE" \
  "$SCRIPT_DIR/start_sitl_mujoco_mj311.sh" \
  --competition-fast \
  "$@"
