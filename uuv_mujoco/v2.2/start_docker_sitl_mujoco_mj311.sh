#!/usr/bin/env bash
set -euo pipefail

WIPE_EEPROM=1
NO_RESET=0
ROS2_MODE="full"
MUJOCO_EXTRA_ARGS=()
DOCKER_LOG_PID=""
LAUNCH_PID=""
CLEANED_UP=0

usage() {
  cat <<'USAGE'
Usage: ./start_docker_sitl_mujoco_mj311.sh [options] [-- <extra launch_uuv_sim args>]

Options are intentionally aligned with start_sitl_mujoco_mj311.sh for GUI use:
  --wipe-eeprom
  --keep-eeprom
  --no-reset
  --no-ros2
  --ros2
  --ros2-real-pkg-compat
  -h, --help
USAGE
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --wipe-eeprom)
      WIPE_EEPROM=1
      shift
      ;;
    --keep-eeprom)
      WIPE_EEPROM=0
      shift
      ;;
    --no-reset)
      NO_RESET=1
      shift
      ;;
    --no-ros2)
      ROS2_MODE="off"
      shift
      ;;
    --ros2)
      ROS2_MODE="full"
      shift
      ;;
    --ros2-real-pkg-compat)
      ROS2_MODE="compat"
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    --)
      shift
      MUJOCO_EXTRA_ARGS+=("$@")
      break
      ;;
    *)
      MUJOCO_EXTRA_ARGS+=("$1")
      shift
      ;;
  esac
done

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="${WORKSPACE_DIR:-$(cd "${SCRIPT_DIR}/../.." && pwd)}"
RESET_SCRIPT="${SCRIPT_DIR}/reset_uuv_sim.sh"
STOP_DOCKER_SCRIPT="${SCRIPT_DIR}/stop_docker_ardusub_sitl.sh"
LAUNCH_SCRIPT="${SCRIPT_DIR}/launch_uuv_sim.sh"
COMPOSE_FILE="${UUV_ARDUSUB_DOCKER_COMPOSE:-${WORKSPACE_DIR}/docker/ardusub/docker-compose.yml}"
DOCKER_SITL_CONTAINER="${DOCKER_SITL_CONTAINER:-uuv-ardusub-sitl}"
LOG_DIR="${SCRIPT_DIR}/logs"
mkdir -p "$LOG_DIR"

docker_child_log() {
  docker exec "$DOCKER_SITL_CONTAINER" bash -lc 'cat /tmp/ArduSub.log 2>/dev/null || true' 2>/dev/null || true
}

file_has_fixed() {
  local pattern="$1"
  local file="$2"
  grep -Fq -- "$pattern" "$file" 2>/dev/null
}

sitl_log_has() {
  local pattern="$1"
  file_has_fixed "$pattern" "$DOCKER_LOG" || docker_child_log | grep -Fq -- "$pattern"
}

docker_sitl_running() {
  [[ "$(docker inspect -f '{{.State.Running}}' "$DOCKER_SITL_CONTAINER" 2>/dev/null || true)" == "true" ]]
}

dump_sitl_logs() {
  tail -n 120 "$DOCKER_LOG" || true
  echo "[docker-start] recent child ArduSub log:"
  docker_child_log | tail -n 120 || true
}

terminate_tree() {
  local pid="$1"
  local child
  while IFS= read -r child; do
    [[ -n "$child" ]] || continue
    terminate_tree "$child"
  done < <(pgrep -P "$pid" 2>/dev/null || true)
  kill "$pid" 2>/dev/null || true
}

cleanup() {
  if [[ "$CLEANED_UP" -eq 1 ]]; then
    return
  fi
  CLEANED_UP=1
  if [[ -n "${DOCKER_LOG_PID:-}" ]]; then
    kill "$DOCKER_LOG_PID" 2>/dev/null || true
  fi
  if [[ -n "${LAUNCH_PID:-}" ]] && kill -0 "$LAUNCH_PID" 2>/dev/null; then
    terminate_tree "$LAUNCH_PID"
  fi
  if [[ "${UUV_DOCKER_KEEP_SITL:-0}" != "1" ]]; then
    "$STOP_DOCKER_SCRIPT" >/dev/null 2>&1 || true
  fi
}

handle_signal() {
  cleanup
  exit 130
}

trap cleanup EXIT
trap handle_signal INT TERM

if [[ "$NO_RESET" -eq 0 ]]; then
  RESET_ARGS=()
  [[ "$WIPE_EEPROM" -eq 1 ]] && RESET_ARGS+=(--wipe-eeprom)
  echo "[docker-start] step 1/3: reset local MuJoCo/ROS/SITL leftovers"
  if ((${#RESET_ARGS[@]} > 0)); then
    "$RESET_SCRIPT" "${RESET_ARGS[@]}"
  else
    "$RESET_SCRIPT"
  fi
else
  echo "[docker-start] step 1/3: reset skipped (--no-reset)"
fi

TS="$(date +%Y%m%d_%H%M%S)"
export SITL_EKF3_EXTNAV="${SITL_EKF3_EXTNAV:-1}"
export SITL_AHRS_EKF_TYPE="${SITL_AHRS_EKF_TYPE:-3}"
# Keep the Docker/Mac boundary narrow: one bidirectional MAVLink link carries
# HEARTBEAT, SERVO_OUTPUT_RAW, arm/mode, and RC override. Set
# SITL_DEDICATED_COMMAND_MAVLINK=1 plus a command endpoint only for split-link
# debugging.
export SITL_DEDICATED_COMMAND_MAVLINK="${SITL_DEDICATED_COMMAND_MAVLINK:-0}"
if [[ "${SITL_DEDICATED_COMMAND_MAVLINK}" =~ ^(1|true|TRUE|yes|YES|on|ON|enable|enabled)$ ]]; then
  export SITL_COMMAND_MAV_PORT="${SITL_COMMAND_MAV_PORT:-14661}"
  export ROS2_UUV_SITL_COMMAND_MAVLINK_ENDPOINT="${ROS2_UUV_SITL_COMMAND_MAVLINK_ENDPOINT:-udpin:0.0.0.0:${SITL_COMMAND_MAV_PORT}}"
else
  export ROS2_UUV_SITL_COMMAND_MAVLINK_ENDPOINT="${ROS2_UUV_SITL_COMMAND_MAVLINK_ENDPOINT:-same}"
fi
LAUNCH_ARGS=(
  --sitl
  --sitl-mavlink-endpoint "udpin:0.0.0.0:${SITL_MUJOCO_MAV_PORT:-14660}"
  --force-clean
)
case "$ROS2_MODE" in
  off)
    LAUNCH_ARGS+=(--no-ros2)
    ;;
  full)
    LAUNCH_ARGS+=(--ros2)
    ;;
  compat)
    LAUNCH_ARGS+=(--ros2-real-pkg-compat)
    ;;
  *)
    echo "[docker-start] unknown ROS2 mode: ${ROS2_MODE}" >&2
    exit 2
    ;;
esac

LAUNCH_LOG="${LOG_DIR}/mujoco_docker_${TS}.log"
LAUNCH_CMD=("$LAUNCH_SCRIPT" "${LAUNCH_ARGS[@]}")
if ((${#MUJOCO_EXTRA_ARGS[@]} > 0)); then
  LAUNCH_CMD+=("${MUJOCO_EXTRA_ARGS[@]}")
fi

echo "[docker-start] step 2/3: start Mac MuJoCo/ROS2 listeners"
"${LAUNCH_CMD[@]}" >"$LAUNCH_LOG" 2>&1 &
LAUNCH_PID=$!
echo "[docker-start] MuJoCo pid=${LAUNCH_PID}, log=${LAUNCH_LOG}"

LISTENER_TIMEOUT="${MUJOCO_LISTENER_WAIT_SECS:-45}"
LISTENER_READY=0
for ((i=1; i<=LISTENER_TIMEOUT; i++)); do
  if ! kill -0 "$LAUNCH_PID" 2>/dev/null; then
    echo "[docker-start] MuJoCo exited early before Docker SITL. recent log:"
    tail -n 120 "$LAUNCH_LOG" || true
    exit 1
  fi
  if file_has_fixed "SITL socket initialized" "$LAUNCH_LOG" && \
     file_has_fixed "SITL MAVLink servo input enabled" "$LAUNCH_LOG"; then
    LISTENER_READY=1
    echo "[docker-start] MuJoCo SITL listeners ready"
    break
  fi
  sleep 1
done

if [[ "$LISTENER_READY" -ne 1 ]]; then
  echo "[docker-start] MuJoCo listener startup timeout (${LISTENER_TIMEOUT}s). recent log:"
  tail -n 120 "$LAUNCH_LOG" || true
  exit 1
fi

echo "[docker-start] step 3/3: start Ubuntu Docker ArduSub SITL"
cd "$WORKSPACE_DIR"
docker compose -f "$COMPOSE_FILE" down --remove-orphans >/dev/null 2>&1 || true
docker compose -f "$COMPOSE_FILE" up --build -d ardusub-sitl

DOCKER_LOG="${LOG_DIR}/docker_sitl_${TS}.log"
(
  docker compose -f "$COMPOSE_FILE" logs -f ardusub-sitl
) >"$DOCKER_LOG" 2>&1 &
DOCKER_LOG_PID=$!
echo "[docker-start] Docker SITL log=${DOCKER_LOG}"

SITL_READY=0
WAIT_SECS="${SITL_WAIT_SECS:-240}"
for ((i=1; i<=WAIT_SECS; i++)); do
  if ! docker_sitl_running; then
    echo "[docker-start] Docker SITL exited early. recent log:"
    dump_sitl_logs
    exit 1
  fi
  if sitl_log_has "JSON control interface set to" || sitl_log_has "No JSON sensor message received, resending servos"; then
    SITL_READY=1
    echo "[docker-start] Docker SITL ready"
    break
  fi
  sleep 1
done

if [[ "$SITL_READY" -ne 1 ]]; then
  echo "[docker-start] Docker SITL startup timeout (${WAIT_SECS}s). recent log:"
  dump_sitl_logs
  exit 1
fi

echo "[docker-start] startup handoff: Docker SITL and MuJoCo are running; GUI/QGC may connect immediately"

wait "$LAUNCH_PID"
kill "$DOCKER_LOG_PID" 2>/dev/null || true
