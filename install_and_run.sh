#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
INSTALLER="${ROOT_DIR}/setup/install_uuv_mujoco.sh"

if [[ ! -x "$INSTALLER" ]]; then
  echo "[install-and-run] installer missing or not executable: $INSTALLER" >&2
  exit 1
fi

exec "$INSTALLER" --with-ros2 --run-after-install "$@"
