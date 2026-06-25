#!/usr/bin/env bash
set -euo pipefail

export PATH="$HOME/.local/bin:$PATH"

WORKSPACE_DIR="${WORKSPACE_DIR:-/workspace}"
ARDUPILOT_DIR="${ARDUPILOT_DIR:-${WORKSPACE_DIR}/ardupilot}"
UUV_MUJOCO_RUNTIME_DIR="${UUV_MUJOCO_RUNTIME_DIR:-${WORKSPACE_DIR}/uuv_mujoco/current}"
if [[ ! -d "${UUV_MUJOCO_RUNTIME_DIR}" ]]; then
  echo "[docker-entrypoint] active runtime not found: ${UUV_MUJOCO_RUNTIME_DIR}" >&2
  echo "[docker-entrypoint] Expected uuv_mujoco/current, or set UUV_MUJOCO_RUNTIME_DIR explicitly." >&2
  exit 1
fi
SITL_SCRIPT="${SITL_SCRIPT:-${UUV_MUJOCO_RUNTIME_DIR}/start_ardusub_sitl_mj311.sh}"

if [[ ! -d "$ARDUPILOT_DIR" ]]; then
  echo "[docker-sitl] missing ArduPilot checkout: ${ARDUPILOT_DIR}" >&2
  exit 1
fi
if [[ ! -x "$SITL_SCRIPT" ]]; then
  echo "[docker-sitl] missing SITL launcher: ${SITL_SCRIPT}" >&2
  exit 1
fi

cd "$ARDUPILOT_DIR"
git config --global --add safe.directory "$ARDUPILOT_DIR" >/dev/null 2>&1 || true

sudo mkdir -p "$ARDUPILOT_DIR/build" "$HOME/.ccache"
sudo chown -R "$(id -u):$(id -g)" "$ARDUPILOT_DIR/build" "$HOME/.ccache"

BINARY="$ARDUPILOT_DIR/build/sitl/bin/ardusub"
SOURCE_REBUILD_INPUTS=(
  "$ARDUPILOT_DIR/ArduSub/control_althold.cpp"
  "$ARDUPILOT_DIR/libraries/AP_Motors/AP_Motors6DOF.cpp"
  "$ARDUPILOT_DIR/libraries/SITL/SIM_JSON.cpp"
  "$ARDUPILOT_DIR/libraries/SITL/SIM_JSON.h"
)
BUILD_NEEDED=0
if [[ ! -x "$BINARY" || "${SITL_REBUILD:-0}" == "1" ]]; then
  BUILD_NEEDED=1
else
  for source_file in "${SOURCE_REBUILD_INPUTS[@]}"; do
    if [[ -f "$source_file" && "$source_file" -nt "$BINARY" ]]; then
      echo "[docker-sitl] source newer than binary: ${source_file#$ARDUPILOT_DIR/}"
      BUILD_NEEDED=1
      break
    fi
  done
fi

if [[ "$BUILD_NEEDED" == "1" ]]; then
  echo "[docker-sitl] building ArduSub SITL binary"
  ./waf configure --board sitl
  ./waf build --target bin/ardusub
else
  echo "[docker-sitl] using existing ArduSub binary: $BINARY"
fi

export WORKSPACE_DIR
export ARDUPILOT_DIR
export MJ311_PYTHON="${MJ311_PYTHON:-/usr/bin/python3}"
export SITL_DIRECT_MAVLINK="${SITL_DIRECT_MAVLINK:-0}"
export SITL_FORCE_NO_DISPLAY="${SITL_FORCE_NO_DISPLAY:-1}"
export SITL_USE_REAL_PARAM_FILE="${SITL_USE_REAL_PARAM_FILE:-1}"
export SITL_REAL_PARAM_FILE="${SITL_REAL_PARAM_FILE:-${WORKSPACE_DIR}/real_robot.param}"

resolve_ipv4() {
  python3 - "$1" <<'PY'
import socket
import sys

host = sys.argv[1]
try:
    print(socket.gethostbyname(host))
except OSError:
    print(host)
PY
}

normalize_host() {
  local host="$1"
  if [[ "$host" =~ ^[0-9]+(\.[0-9]+){3}$ ]]; then
    echo "$host"
  else
    resolve_ipv4 "$host"
  fi
}

export SITL_JSON_HOST="$(normalize_host "${SITL_JSON_HOST:-host.docker.internal}")"
export SITL_CONSOLE_HOST="$(normalize_host "${SITL_CONSOLE_HOST:-host.docker.internal}")"
export SITL_QGC_HOST="$(normalize_host "${SITL_QGC_HOST:-host.docker.internal}")"
export SITL_MAVROS_HOST="$(normalize_host "${SITL_MAVROS_HOST:-host.docker.internal}")"
export SITL_MUJOCO_MAV_HOST="$(normalize_host "${SITL_MUJOCO_MAV_HOST:-host.docker.internal}")"

echo "[docker-sitl] JSON target     ${SITL_JSON_HOST}:${SITL_JSON_SERVO_PORT:-9002}"
echo "[docker-sitl] SERIAL0 target  ${SITL_CONSOLE_HOST}:${SITL_CONSOLE_PORT:-14552}"
echo "[docker-sitl] QGC MAVLink     ${SITL_QGC_HOST}:${SITL_QGC_PORT:-14550} enable=${SITL_QGC_OUTPUT_ENABLE:-0}"
echo "[docker-sitl] MAVROS MAVLink  ${SITL_MAVROS_HOST}:${SITL_MAVROS_PORT:-14551} enable=${SITL_MAVROS_OUTPUT_ENABLE:-0}"
echo "[docker-sitl] MuJoCo MAVLink  ${SITL_MUJOCO_MAV_HOST}:${SITL_MUJOCO_MAV_PORT:-14660}"
case "${SITL_DEDICATED_COMMAND_MAVLINK:-0}" in
  1|true|TRUE|yes|YES|on|ON|enable|enabled)
    export SITL_COMMAND_MAV_HOST="$(normalize_host "${SITL_COMMAND_MAV_HOST:-host.docker.internal}")"
    echo "[docker-sitl] Command MAVLink ${SITL_COMMAND_MAV_HOST}:${SITL_COMMAND_MAV_PORT:-14661}"
    ;;
  *)
    echo "[docker-sitl] Command MAVLink shared on MuJoCo MAVLink"
    ;;
esac

if [[ "${SITL_DIRECT_MAVLINK}" =~ ^(1|true|TRUE|yes|YES|on|ON|enable|enabled)$ ]]; then
  exec "$SITL_SCRIPT" --direct-mavlink --no-rebuild --force-no-display "$@"
fi
exec "$SITL_SCRIPT" --no-rebuild --force-no-display "$@"
