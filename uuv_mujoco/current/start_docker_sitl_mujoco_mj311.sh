#!/usr/bin/env bash
set -euo pipefail

WIPE_EEPROM=1
NO_RESET=0
ROS2_MODE="full"
SITL_DIRECT_MAVLINK="${SITL_DIRECT_MAVLINK:-0}"
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
  --direct-mavlink
  --legacy-mavproxy
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
    --direct-mavlink)
      SITL_DIRECT_MAVLINK=1
      shift
      ;;
    --legacy-mavproxy)
      SITL_DIRECT_MAVLINK=0
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
ACTIVE_RUNTIME_DIR="${UUV_MUJOCO_RUNTIME_DIR:-${WORKSPACE_DIR}/uuv_mujoco/current}"
FRESHNESS_RUNTIME_DIR="$SCRIPT_DIR"
if [[ -e "$ACTIVE_RUNTIME_DIR" ]]; then
  ACTIVE_RUNTIME_RESOLVED="$(cd "$ACTIVE_RUNTIME_DIR" 2>/dev/null && pwd -P || true)"
  SCRIPT_DIR_RESOLVED="$(cd "$SCRIPT_DIR" && pwd -P)"
  if [[ "$ACTIVE_RUNTIME_RESOLVED" == "$SCRIPT_DIR_RESOLVED" ]]; then
    FRESHNESS_RUNTIME_DIR="$ACTIVE_RUNTIME_DIR"
  fi
fi
RESET_SCRIPT="${SCRIPT_DIR}/reset_uuv_sim.sh"
STOP_DOCKER_SCRIPT="${SCRIPT_DIR}/stop_docker_ardusub_sitl.sh"
LAUNCH_SCRIPT="${SCRIPT_DIR}/launch_uuv_sim.sh"
COMPOSE_FILE="${UUV_ARDUSUB_DOCKER_COMPOSE:-${WORKSPACE_DIR}/docker/ardusub/docker-compose.yml}"
DOCKER_SITL_CONTAINER="${DOCKER_SITL_CONTAINER:-uuv-ardusub-sitl}"
LOG_DIR="${SCRIPT_DIR}/logs"
mkdir -p "$LOG_DIR"
COMPOSE_ARGS=(-f "$COMPOSE_FILE")

if [[ "${UUV_MUJOCO_SKIP_FRESHNESS_CHECK:-0}" != "1" ]]; then
  FRESHNESS_CHECKER="${SCRIPT_DIR}/tools/check_runtime_freshness.py"
  FRESHNESS_PYTHON="${MJ311_PYTHON:-python3}"
  if [[ -f "$FRESHNESS_CHECKER" ]]; then
    if ! "$FRESHNESS_PYTHON" "$FRESHNESS_CHECKER" \
      --workspace "$WORKSPACE_DIR" \
      --runtime-dir "$FRESHNESS_RUNTIME_DIR" \
      --fetch \
      --refresh-version \
      --warn-only; then
      echo "[docker-start] warning: runtime freshness preflight could not run with ${FRESHNESS_PYTHON}" >&2
    fi
  else
    echo "[docker-start] warning: runtime freshness checker missing: ${FRESHNESS_CHECKER}" >&2
  fi
fi

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
STARTUP_BEGIN_SECONDS=$SECONDS
export SITL_DIRECT_MAVLINK="${SITL_DIRECT_MAVLINK:-0}"
export SITL_FORCE_NO_DISPLAY="${SITL_FORCE_NO_DISPLAY:-1}"
export SITL_NO_REBUILD="${SITL_NO_REBUILD:-1}"
export SITL_USE_REAL_PARAM_FILE="${SITL_USE_REAL_PARAM_FILE:-1}"
export SITL_REAL_PARAM_FILE="${SITL_REAL_PARAM_FILE:-/workspace/real_robot.param}"
export SITL_WIPE_EEPROM="${SITL_WIPE_EEPROM:-$WIPE_EEPROM}"
export UUV_EKF_CONTRACT="${UUV_EKF_CONTRACT:-althold_baro}"
case "$UUV_EKF_CONTRACT" in
  real_param_parity)
    export UUV_EKF_CONTRACT="real_param_parity"
    export SITL_EKF3_EXTNAV="${SITL_EKF3_EXTNAV:-1}"
    export SITL_EKF3_EXTNAV_POSZ="${SITL_EKF3_EXTNAV_POSZ:-1}"
    export SITL_EKF3_EXTNAV_VELZ="${SITL_EKF3_EXTNAV_VELZ:-6}"
    export ROS2_UUV_SITL_DVL_RANGEFINDER_ENABLE="${ROS2_UUV_SITL_DVL_RANGEFINDER_ENABLE:-1}"
    ;;
  poshold_extnav|poshold_extnav_412|real-ekf|real_ekf|extnav)
    export UUV_EKF_CONTRACT="poshold_extnav"
    export SITL_EKF3_EXTNAV="${SITL_EKF3_EXTNAV:-1}"
    export SITL_EKF3_EXTNAV_POSZ="${SITL_EKF3_EXTNAV_POSZ:-1}"
    export SITL_EKF3_EXTNAV_VELZ="${SITL_EKF3_EXTNAV_VELZ:-6}"
    export ROS2_UUV_SITL_DVL_RANGEFINDER_ENABLE="${ROS2_UUV_SITL_DVL_RANGEFINDER_ENABLE:-1}"
    ;;
  althold_baro|baro|baro-ekf|depthhold_baro)
    export UUV_EKF_CONTRACT="althold_baro"
    export SITL_EKF3_EXTNAV="${SITL_EKF3_EXTNAV:-0}"
    export SITL_EKF3_EXTNAV_POSZ="${SITL_EKF3_EXTNAV_POSZ:-1}"
    export SITL_EKF3_EXTNAV_VELZ="${SITL_EKF3_EXTNAV_VELZ:-0}"
    export ROS2_UUV_SITL_DVL_RANGEFINDER_ENABLE="${ROS2_UUV_SITL_DVL_RANGEFINDER_ENABLE:-0}"
    ;;
  *)
    echo "[docker-start] unknown UUV_EKF_CONTRACT=$UUV_EKF_CONTRACT" >&2
    exit 2
    ;;
esac
# MuJoCo's JSON stream is asynchronous (no_time_sync/no_lockstep), for which
# ArduPilot's SIM_JSON backend requires the direct simulated AHRS path.
export SITL_AHRS_EKF_TYPE="${SITL_AHRS_EKF_TYPE:-10}"
export SITL_JSON_HOST="${SITL_JSON_HOST:-host.docker.internal}"
export SITL_JSON_SENSOR_PORT="${SITL_JSON_SENSOR_PORT:-9003}"
export SITL_JSON_SERVO_PORT="${SITL_JSON_SERVO_PORT:-9002}"
export SITL_QGC_HOST="${SITL_QGC_HOST:-host.docker.internal}"
export SITL_QGC_PORT="${SITL_QGC_PORT:-14550}"
# Keep the default Docker/QGC contract aligned with dist: ArduSub 4.1.2 uses
# MAVProxy fan-out for QGC, while serial2 stays reserved for MuJoCo commands.
# SERIAL0 bootstrap is only needed for explicit direct-MAVLink experiments.
export SITL_SERIAL0_UDPCLIENT="${SITL_SERIAL0_UDPCLIENT:-0}"
export SITL_CONSOLE_HOST="${SITL_CONSOLE_HOST:-$SITL_QGC_HOST}"
export SITL_CONSOLE_PORT="${SITL_CONSOLE_PORT:-14552}"
export SITL_QGC_OUTPUT_ENABLE="${SITL_QGC_OUTPUT_ENABLE:-1}"
export SITL_QGC_DIRECT_SERIAL_ENABLE="${SITL_QGC_DIRECT_SERIAL_ENABLE:-0}"
export ROS2_UUV_QGC_MAVLINK_RELAY_ENABLE="${ROS2_UUV_QGC_MAVLINK_RELAY_ENABLE:-0}"
export ROS2_UUV_QGC_MAVLINK_RELAY_HOST="${ROS2_UUV_QGC_MAVLINK_RELAY_HOST:-127.0.0.1}"
export ROS2_UUV_QGC_MAVLINK_RELAY_PORT="${ROS2_UUV_QGC_MAVLINK_RELAY_PORT:-$SITL_QGC_PORT}"
export SITL_MAVROS_HOST="${SITL_MAVROS_HOST:-host.docker.internal}"
export SITL_MAVROS_PORT="${SITL_MAVROS_PORT:-14551}"
export SITL_MAVROS_OUTPUT_ENABLE="${SITL_MAVROS_OUTPUT_ENABLE:-0}"
export SITL_MUJOCO_MAV_HOST="${SITL_MUJOCO_MAV_HOST:-host.docker.internal}"
export SITL_MUJOCO_MAV_PORT="${SITL_MUJOCO_MAV_PORT:-14660}"
export SITL_QGC_CONTROL_AUTHORITY="${SITL_QGC_CONTROL_AUTHORITY:-1}"
case "${SITL_QGC_CONTROL_AUTHORITY}" in
  1|true|TRUE|yes|YES|on|ON|enable|enabled)
    DEFAULT_MAVLINK_SOURCE_SYSID=255
    DEFAULT_MAVLINK_SOURCE_COMPID=190
    ;;
  *)
    DEFAULT_MAVLINK_SOURCE_SYSID=254
    DEFAULT_MAVLINK_SOURCE_COMPID=240
    ;;
esac
export SITL_MAVLINK_SOURCE_SYSID="${SITL_MAVLINK_SOURCE_SYSID:-$DEFAULT_MAVLINK_SOURCE_SYSID}"
export SITL_MAVLINK_SOURCE_COMPID="${SITL_MAVLINK_SOURCE_COMPID:-$DEFAULT_MAVLINK_SOURCE_COMPID}"
export SITL_SYSID_MYGCS="${SITL_SYSID_MYGCS:-$SITL_MAVLINK_SOURCE_SYSID}"
export SITL_MAV_GCS_SYSID="${SITL_MAV_GCS_SYSID:-$SITL_SYSID_MYGCS}"
export SITL_MAV_GCS_SYSID_HI="${SITL_MAV_GCS_SYSID_HI:-0}"
export SITL_NO_EXTRA_PORTS="${SITL_NO_EXTRA_PORTS:-1}"
export SITL_REPLAY_LINK="${SITL_REPLAY_LINK:-}"
export SITL_REPLAY_MAV_PORT="${SITL_REPLAY_MAV_PORT:-}"
export SITL_CUSTOM_LOCATION="${SITL_CUSTOM_LOCATION:-}"
# Keep arm/mode/RC command traffic on the already-open MuJoCo MAVLink peer
# instead of adding a serial4/14661 command port; the split command link remains
# an explicit A/B knob only.
export SITL_DEDICATED_COMMAND_MAVLINK="${SITL_DEDICATED_COMMAND_MAVLINK:-0}"
export ROS2_UUV_COMMAND_LINK_TELEMETRY="${ROS2_UUV_COMMAND_LINK_TELEMETRY:-0}"
export ROS2_UUV_COMMAND_LINK_AP_TELEMETRY="${ROS2_UUV_COMMAND_LINK_AP_TELEMETRY:-0}"
if [[ "${SITL_DEDICATED_COMMAND_MAVLINK}" =~ ^(1|true|TRUE|yes|YES|on|ON|enable|enabled)$ ]]; then
  export SITL_COMMAND_MAV_PORT="${SITL_COMMAND_MAV_PORT:-14661}"
  export SITL_COMMAND_MAV_HOST="${SITL_COMMAND_MAV_HOST:-host.docker.internal}"
  if [[ -z "${ROS2_UUV_SITL_COMMAND_MAVLINK_ENDPOINT:-}" ]]; then
    export ROS2_UUV_SITL_COMMAND_MAVLINK_ENDPOINT="${ROS2_UUV_DOCKER_SITL_COMMAND_MAVLINK_ENDPOINT:-udpin:0.0.0.0:${SITL_COMMAND_MAV_PORT}}"
  fi
else
  export ROS2_UUV_SITL_COMMAND_MAVLINK_ENDPOINT="${ROS2_UUV_SITL_COMMAND_MAVLINK_ENDPOINT:-same}"
fi
export SITL_AUTO_SAFE_SEQUENCE="${SITL_AUTO_SAFE_SEQUENCE:-0}"
export ROS2_UUV_SITL_AUTO_READY="${ROS2_UUV_SITL_AUTO_READY:-0}"
export ROS2_UUV_SITL_AUTO_READY_MODE="${ROS2_UUV_SITL_AUTO_READY_MODE:-MANUAL}"
export ROS2_UUV_MAVROS_FORWARD_ARM_MODE="${ROS2_UUV_MAVROS_FORWARD_ARM_MODE:-1}"
export ROS2_UUV_DEDICATED_SPIN_THREAD="${ROS2_UUV_DEDICATED_SPIN_THREAD:-1}"
export ROS2_UUV_SPIN_HZ="${ROS2_UUV_SPIN_HZ:-400}"
export ROS2_UUV_SPIN_TIMEOUT_S="${ROS2_UUV_SPIN_TIMEOUT_S:-0.005}"
export ROS2_UUV_SITL_COMMAND_POLL_HZ="${ROS2_UUV_SITL_COMMAND_POLL_HZ:-400}"
export ROS2_UUV_SITL_MAVLINK_POLL_HZ="${ROS2_UUV_SITL_MAVLINK_POLL_HZ:-200}"
export ROS2_UUV_MAVROS_RC_OVERRIDE_FORWARD_HZ="${ROS2_UUV_MAVROS_RC_OVERRIDE_FORWARD_HZ:-400}"
export ROS2_UUV_CMD_TIMEOUT_S="${ROS2_UUV_CMD_TIMEOUT_S:-0.25}"
export ROS2_UUV_CMD_DEADBAND="${ROS2_UUV_CMD_DEADBAND:-0.0}"
export ROS2_UUV_CMD_SLEW_RATE="${ROS2_UUV_CMD_SLEW_RATE:-0}"
export UUV_GUI_REQUIRE_ARM_MODE_EKF_SETTLE="${UUV_GUI_REQUIRE_ARM_MODE_EKF_SETTLE:-0}"
export UUV_GUI_ARM_MODE_EKF_SETTLE_S="${UUV_GUI_ARM_MODE_EKF_SETTLE_S:-0.0}"
export ROS2_UUV_SITL_NEUTRAL_RC_KEEPALIVE="${ROS2_UUV_SITL_NEUTRAL_RC_KEEPALIVE:-0}"
export ROS2_UUV_SITL_JSON_SERVO_FALLBACK="${ROS2_UUV_SITL_JSON_SERVO_FALLBACK:-1}"
export ROS2_UUV_ALLOW_RCOUT_PLANT_OVERRIDE="${ROS2_UUV_ALLOW_RCOUT_PLANT_OVERRIDE:-0}"
export UUV_GUI_PILOT_CONTROL_MODE="${UUV_GUI_PILOT_CONTROL_MODE:-rc_override}"
export ROS2_UUV_MAVROS_RC_OVERRIDE_BACKEND="${ROS2_UUV_MAVROS_RC_OVERRIDE_BACKEND:-rc_channels_override}"

# Layer the exact runtime contract used by this launcher over the shared base
# compose file. The default keeps serial0 as a bootstrap only and shares
# arm/mode/RC command traffic on MuJoCo's serial2 MAVLink peer.
COMPOSE_OVERRIDE_FILE="${LOG_DIR}/docker_compose_sitl_contract_${TS}.yml"
cat >"$COMPOSE_OVERRIDE_FILE" <<EOF
services:
  ardusub-sitl:
    environment:
      SITL_DIRECT_MAVLINK: "${SITL_DIRECT_MAVLINK}"
      SITL_FORCE_NO_DISPLAY: "${SITL_FORCE_NO_DISPLAY}"
      SITL_NO_REBUILD: "${SITL_NO_REBUILD}"
      SITL_USE_REAL_PARAM_FILE: "${SITL_USE_REAL_PARAM_FILE}"
      SITL_REAL_PARAM_FILE: "${SITL_REAL_PARAM_FILE}"
      SITL_WIPE_EEPROM: "${SITL_WIPE_EEPROM}"
      UUV_EKF_CONTRACT: "${UUV_EKF_CONTRACT}"
      SITL_EKF3_EXTNAV: "${SITL_EKF3_EXTNAV}"
      SITL_AHRS_EKF_TYPE: "${SITL_AHRS_EKF_TYPE}"
      SITL_JSON_HOST: "${SITL_JSON_HOST}"
      SITL_JSON_SENSOR_PORT: "${SITL_JSON_SENSOR_PORT}"
      SITL_JSON_SERVO_PORT: "${SITL_JSON_SERVO_PORT}"
      SITL_SERIAL0_UDPCLIENT: "${SITL_SERIAL0_UDPCLIENT}"
      SITL_CONSOLE_HOST: "${SITL_CONSOLE_HOST}"
      SITL_CONSOLE_PORT: "${SITL_CONSOLE_PORT}"
      SITL_QGC_HOST: "${SITL_QGC_HOST}"
      SITL_QGC_PORT: "${SITL_QGC_PORT}"
      SITL_QGC_OUTPUT_ENABLE: "${SITL_QGC_OUTPUT_ENABLE}"
      SITL_QGC_DIRECT_SERIAL_ENABLE: "${SITL_QGC_DIRECT_SERIAL_ENABLE}"
      ROS2_UUV_QGC_MAVLINK_RELAY_ENABLE: "${ROS2_UUV_QGC_MAVLINK_RELAY_ENABLE}"
      SITL_MAVROS_HOST: "${SITL_MAVROS_HOST}"
      SITL_MAVROS_PORT: "${SITL_MAVROS_PORT}"
      SITL_MAVROS_OUTPUT_ENABLE: "${SITL_MAVROS_OUTPUT_ENABLE}"
      SITL_MUJOCO_MAV_HOST: "${SITL_MUJOCO_MAV_HOST}"
      SITL_MUJOCO_MAV_PORT: "${SITL_MUJOCO_MAV_PORT}"
      SITL_MAVLINK_SOURCE_SYSID: "${SITL_MAVLINK_SOURCE_SYSID}"
      SITL_MAVLINK_SOURCE_COMPID: "${SITL_MAVLINK_SOURCE_COMPID}"
      SITL_SYSID_MYGCS: "${SITL_SYSID_MYGCS}"
      SITL_MAV_GCS_SYSID: "${SITL_MAV_GCS_SYSID}"
      SITL_MAV_GCS_SYSID_HI: "${SITL_MAV_GCS_SYSID_HI}"
      SITL_DEDICATED_COMMAND_MAVLINK: "${SITL_DEDICATED_COMMAND_MAVLINK}"
      SITL_COMMAND_MAV_HOST: "${SITL_COMMAND_MAV_HOST:-host.docker.internal}"
      SITL_COMMAND_MAV_PORT: "${SITL_COMMAND_MAV_PORT:-14661}"
      SITL_NO_EXTRA_PORTS: "${SITL_NO_EXTRA_PORTS}"
      SITL_REPLAY_LINK: "${SITL_REPLAY_LINK}"
      SITL_REPLAY_MAV_PORT: "${SITL_REPLAY_MAV_PORT}"
      SITL_CUSTOM_LOCATION: "${SITL_CUSTOM_LOCATION}"
      SITL_PARAM_AB_PROFILE: "${SITL_PARAM_AB_PROFILE:-real}"
      SITL_GPS_TYPE: "${SITL_GPS_TYPE:-}"
      SITL_GPS_TYPE2: "${SITL_GPS_TYPE2:-}"
      SITL_SIM_GPS_TYPE: "${SITL_SIM_GPS_TYPE:-}"
      SITL_AHRS_GPS_USE: "${SITL_AHRS_GPS_USE:-}"
      SITL_EKF3_EXTNAV_POSXY: "${SITL_EKF3_EXTNAV_POSXY:-}"
      SITL_EKF3_EXTNAV_VELXY: "${SITL_EKF3_EXTNAV_VELXY:-}"
      SITL_EKF3_EXTNAV_POSZ: "${SITL_EKF3_EXTNAV_POSZ:-}"
      SITL_EKF3_EXTNAV_VELZ: "${SITL_EKF3_EXTNAV_VELZ:-}"
      SITL_EKF3_EXTNAV_YAW: "${SITL_EKF3_EXTNAV_YAW:-}"
      SITL_VISO_TYPE: "${SITL_VISO_TYPE:-}"
EOF
COMPOSE_ARGS=(-f "$COMPOSE_FILE" -f "$COMPOSE_OVERRIDE_FILE")
echo "[docker-start] Docker compose override=${COMPOSE_OVERRIDE_FILE}"
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

echo "[docker-start] step 2/3: start host MuJoCo/ROS2 listeners"
"${LAUNCH_CMD[@]}" >"$LAUNCH_LOG" 2>&1 &
LAUNCH_PID=$!
echo "[docker-start] MuJoCo pid=${LAUNCH_PID}, log=${LAUNCH_LOG}"

LISTENER_TIMEOUT="${MUJOCO_LISTENER_WAIT_SECS:-30}"
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

echo "[docker-start] step 3/3: start Docker ArduSub SITL"
cd "$WORKSPACE_DIR"
docker compose "${COMPOSE_ARGS[@]}" down --remove-orphans >/dev/null 2>&1 || true
DOCKER_UP_ARGS=(up -d)
if [[ "${UUV_DOCKER_BUILD_ON_START:-0}" =~ ^(1|true|TRUE|yes|YES|on|ON|enable|enabled)$ ]]; then
  DOCKER_UP_ARGS+=(--build)
fi
DOCKER_UP_ARGS+=(ardusub-sitl)
docker compose "${COMPOSE_ARGS[@]}" "${DOCKER_UP_ARGS[@]}"

DOCKER_LOG="${LOG_DIR}/docker_sitl_${TS}.log"
(
  docker compose "${COMPOSE_ARGS[@]}" logs -f ardusub-sitl
) >"$DOCKER_LOG" 2>&1 &
DOCKER_LOG_PID=$!
echo "[docker-start] Docker SITL log=${DOCKER_LOG}"

SITL_READY=0
WAIT_SECS="${SITL_WAIT_SECS:-30}"
for ((i=1; i<=WAIT_SECS; i++)); do
  if ! docker_sitl_running; then
    echo "[docker-start] Docker SITL exited early. recent log:"
    dump_sitl_logs
    exit 1
  fi
  if sitl_log_has "JSON control interface set to" || sitl_log_has "No JSON sensor message received, resending servos"; then
    SITL_READY=1
    echo "[docker-start] Docker SITL process ready"
    break
  fi
  sleep 1
done

if [[ "$SITL_READY" -ne 1 ]]; then
  echo "[docker-start] Docker SITL startup timeout (${WAIT_SECS}s). recent log:"
  dump_sitl_logs
  exit 1
fi

AUTO_READY_REQUIRED=1
if [[ "${ROS2_UUV_SITL_AUTO_READY}" =~ ^(0|false|FALSE|no|NO|off|OFF|disable|disabled)$ ]]; then
  AUTO_READY_REQUIRED=0
fi
READY_TOTAL_SECS="${UUV_QGC_RC_READY_TOTAL_SECS:-30}"
READY_ELAPSED=$((SECONDS - STARTUP_BEGIN_SECONDS))
READY_WAIT_SECS=$((READY_TOTAL_SECS - READY_ELAPSED))
if [[ "$READY_WAIT_SECS" -lt 1 ]]; then
  READY_WAIT_SECS=1
fi
QGC_RC_READY=0
echo "[docker-start] rc/servo startup contract: total_budget=${READY_TOTAL_SECS}s elapsed=${READY_ELAPSED}s remaining=${READY_WAIT_SECS}s"
for ((i=1; i<=READY_WAIT_SECS; i++)); do
  if ! kill -0 "$LAUNCH_PID" 2>/dev/null; then
    echo "[docker-start] MuJoCo exited before RC/servo readiness. recent log:"
    tail -n 160 "$LAUNCH_LOG" || true
    exit 1
  fi
  SERVO_READY=0
  AUTO_READY_OK=0
  if file_has_fixed "SITL servo endpoint discovered" "$LAUNCH_LOG"; then
    SERVO_READY=1
  fi
  if [[ "$AUTO_READY_REQUIRED" -eq 0 ]] || file_has_fixed "auto-ready ready" "$LAUNCH_LOG"; then
    AUTO_READY_OK=1
  fi
  if [[ "$SERVO_READY" -eq 1 && "$AUTO_READY_OK" -eq 1 ]]; then
    QGC_RC_READY=1
    echo "[docker-start] RC/servo ready: servo endpoint linked, auto-ready=${ROS2_UUV_SITL_AUTO_READY}, command endpoint=${ROS2_UUV_SITL_COMMAND_MAVLINK_ENDPOINT}"
    break
  fi
  sleep 1
done

if [[ "$QGC_RC_READY" -ne 1 ]]; then
  echo "[docker-start] RC/servo readiness timeout (${READY_TOTAL_SECS}s total budget). recent MuJoCo log:"
  tail -n 160 "$LAUNCH_LOG" || true
  echo "[docker-start] recent Docker SITL log:"
  dump_sitl_logs
  exit 1
fi

if [[ "${SITL_DEDICATED_COMMAND_MAVLINK}" =~ ^(1|true|TRUE|yes|YES|on|ON|enable|enabled)$ ]]; then
  COMMAND_PATH="serial4 ${SITL_COMMAND_MAV_HOST:-host.docker.internal}:${SITL_COMMAND_MAV_PORT:-14661}"
else
  COMMAND_PATH="serial2 ${SITL_MUJOCO_MAV_HOST}:${SITL_MUJOCO_MAV_PORT}"
fi
case "${SITL_QGC_DIRECT_SERIAL_ENABLE}" in
  1|true|TRUE|yes|YES|on|ON|enable|enabled)
    QGC_LINK="serial1 ${SITL_QGC_HOST}:${SITL_QGC_PORT}"
    ;;
  *)
    if [[ "${SITL_DIRECT_MAVLINK}" =~ ^(1|true|TRUE|yes|YES|on|ON|enable|enabled)$ ]]; then
      QGC_LINK="direct serial1 ${SITL_QGC_HOST}:${SITL_QGC_PORT}"
    else
      QGC_LINK="MAVProxy fan-out ${SITL_QGC_HOST}:${SITL_QGC_PORT}"
    fi
    ;;
esac
echo "[docker-start] startup handoff: QGC link is ${QGC_LINK}, RC/arm/mode command path is ${COMMAND_PATH}"

wait "$LAUNCH_PID"
kill "$DOCKER_LOG_PID" 2>/dev/null || true
