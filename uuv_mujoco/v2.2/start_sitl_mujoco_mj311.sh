#!/usr/bin/env bash
set -euo pipefail

# One-shot launcher:
# 1) reset running stack
# 2) start ArduSub SITL with the configured Python interpreter
# 3) start MuJoCo UUV sim with the configured Python interpreter

WIPE_EEPROM=1
NO_RESET=0
SITL_PARAM_TUNE=0
SITL_DIRECT_MAVLINK=0
SITL_NO_REBUILD="${SITL_NO_REBUILD:-1}"
SITL_FORCE_NO_DISPLAY=1
SITL_EKF_STABLE=1
ROS2_MODE="full"
MUJOCO_EXTRA_ARGS=()
START_WRAPPER_EPOCH="$(date +%s)"

elapsed_s() {
  printf '%s' "$(( $(date +%s) - START_WRAPPER_EPOCH ))"
}

log_timing() {
  echo "[start timing +$(elapsed_s)s] $*"
}

usage() {
  cat <<'USAGE'
Usage: ./start_sitl_mujoco_mj311.sh [options] [-- <extra launch_uuv_sim args>]

Options:
  --wipe-eeprom     Remove ardupilot/eeprom.bin during reset (default on)
  --keep-eeprom     Keep existing ardupilot/eeprom.bin
  --no-reset        Skip reset step
  --no-ros2         Launch MuJoCo without ROS2 bridge topics
  --ros2            Launch MuJoCo with full lightweight ROS2 bridge (default)
  --ros2-real-pkg-compat
                    Launch MuJoCo with ROS2 sensors + compat MAVROS surface only
  --param-tune      Enable parameter-tuning pipeline (QGC/MAVProxy background mode)
  --direct-mavlink  Use direct UDP outputs without MAVProxy (experimental)
  --no-ekf-stable   Do not apply default EKF stabilization params for SITL
  --sitl-no-rebuild Pass -N to sim_vehicle.py (default; avoids rebuilding ArduPilot)
  --sitl-rebuild    Rebuild ArduSub before launching
  --sitl-no-display Force non-GUI terminal fallback for SITL (default on)
  -h, --help        Show this help

Environment:
  WORKSPACE_DIR     Workspace root containing ardupilot/ and optional QGC app
  ARDUPILOT_DIR     Explicit ArduPilot checkout path passed through to child scripts
  MJ311_ROOT        Preferred Python env root (optional; falls back to ~/.venvs/mujoco311 if present)
  MJ311_PYTHON      Explicit Python interpreter (optional)
  MJ311_MJPYTHON    Explicit mjpython path (optional; Linux falls back to python)

Examples:
  ./start_sitl_mujoco_mj311.sh
  ./start_sitl_mujoco_mj311.sh --no-ros2
  ./start_sitl_mujoco_mj311.sh --ros2
  ./start_sitl_mujoco_mj311.sh --ros2-real-pkg-compat
  ./start_sitl_mujoco_mj311.sh -- --scene scenes/tank_legacy_scene.xml
  ./start_sitl_mujoco_mj311.sh -- --scene scenes/tank_current_scene.xml
  ./start_sitl_mujoco_mj311.sh -- --tank-549x274x132
  ./start_sitl_mujoco_mj311.sh -- --tank-549x274x132 --fluid-model legacy
  ./start_sitl_mujoco_mj311.sh -- --tank-549x274x132 --fluid-model current
  ./start_sitl_mujoco_mj311.sh --param-tune
  ./start_sitl_mujoco_mj311.sh -- --headless
  ./start_sitl_mujoco_mj311.sh -- --qgc-video
  ./start_sitl_mujoco_mj311.sh --ros2-real-pkg-compat -- --headless
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
    --param-tune)
      SITL_PARAM_TUNE=1
      shift
      ;;
    --direct-mavlink)
      SITL_DIRECT_MAVLINK=1
      shift
      ;;
    --no-ekf-stable)
      SITL_EKF_STABLE=0
      shift
      ;;
    --sitl-no-rebuild)
      SITL_NO_REBUILD=1
      shift
      ;;
    --sitl-rebuild)
      SITL_NO_REBUILD=0
      shift
      ;;
    --sitl-no-display)
      SITL_FORCE_NO_DISPLAY=1
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

resolve_default_workspace_dir() {
  local dir best score current_score
  if [[ -n "${ARDUPILOT_DIR:-}" && -d "${ARDUPILOT_DIR:-}" ]]; then
    cd "${ARDUPILOT_DIR}/.." && pwd
    return 0
  fi

  dir="$SCRIPT_DIR"
  best=""
  score=0
  while [[ -n "$dir" ]]; do
    current_score=0
    [[ -d "$dir/ardupilot" ]] && current_score=$((current_score + 10))
    [[ -f "$dir/QGroundControl.AppImage" || -f "$dir/QGroundControl-x86_64.AppImage" || -d "$dir/QGroundControl.app" ]] && current_score=$((current_score + 4))
    [[ -d "$dir/kmu26_auv" ]] && current_score=$((current_score + 2))
    [[ -d "$dir/install" ]] && current_score=$((current_score + 1))
    if (( current_score > score )); then
      best="$dir"
      score=$current_score
    fi
    [[ "$dir" == "/" ]] && break
    dir="$(dirname "$dir")"
  done

  if [[ -n "$best" && "$score" -gt 0 ]]; then
    printf '%s\n' "$best"
    return 0
  fi

  cd "${SCRIPT_DIR}/../.." && pwd
}

DEFAULT_WORKSPACE_DIR="$(resolve_default_workspace_dir)"
WORKSPACE_DIR="${WORKSPACE_DIR:-$DEFAULT_WORKSPACE_DIR}"
RESET_SCRIPT="${SCRIPT_DIR}/reset_uuv_sim.sh"
SITL_SCRIPT="${SCRIPT_DIR}/start_ardusub_sitl_mj311.sh"
LAUNCH_SCRIPT="${SCRIPT_DIR}/launch_uuv_sim.sh"
LOG_DIR="${SCRIPT_DIR}/logs"
mkdir -p "$LOG_DIR"

if [[ ! -d "$WORKSPACE_DIR" ]]; then
  echo "[start] workspace directory not found: $WORKSPACE_DIR"
  echo "        Set WORKSPACE_DIR correctly."
  exit 1
fi

if [[ ! -x "$RESET_SCRIPT" || ! -x "$SITL_SCRIPT" || ! -x "$LAUNCH_SCRIPT" ]]; then
  echo "[start] required script missing or not executable."
  echo "        reset=${RESET_SCRIPT}"
  echo "        sitl=${SITL_SCRIPT}"
  echo "        launch=${LAUNCH_SCRIPT}"
  exit 1
fi

if [[ "$NO_RESET" -eq 0 ]]; then
  RESET_ARGS=()
  [[ "$WIPE_EEPROM" -eq 1 ]] && RESET_ARGS+=(--wipe-eeprom)
  echo "[start] step 1/3: reset stack"
  if ((${#RESET_ARGS[@]} > 0)); then
    "$RESET_SCRIPT" "${RESET_ARGS[@]}"
  else
    "$RESET_SCRIPT"
  fi
else
  echo "[start] step 1/3: reset skipped (--no-reset)"
fi
log_timing "reset stage complete"

echo "[start] step 2/3: start ArduSub SITL"
TS="$(date +%Y%m%d_%H%M%S)"
SITL_LOG="${LOG_DIR}/sitl_${TS}.log"
ARDUSUB_RUNTIME_LOG="/tmp/ArduSub.log"
SITL_EXTRA_ARGS=()
[[ "$SITL_PARAM_TUNE" -eq 1 ]] && SITL_EXTRA_ARGS+=(--param-tune)
[[ "$SITL_DIRECT_MAVLINK" -eq 1 ]] && SITL_EXTRA_ARGS+=(--direct-mavlink)
[[ "$SITL_NO_REBUILD" -eq 1 ]] && SITL_EXTRA_ARGS+=(--no-rebuild)
[[ "$SITL_FORCE_NO_DISPLAY" -eq 1 ]] && SITL_EXTRA_ARGS+=(--force-no-display)
[[ "$SITL_EKF_STABLE" -eq 0 ]] && SITL_EXTRA_ARGS+=(--no-ekf-stable)
if [[ "$SITL_PARAM_TUNE" -eq 1 ]]; then
  SITL_BG_MAVPROXY_ARGS="${SITL_BG_MAVPROXY_ARGS:---non-interactive --nowait}"
  SITL_EXTRA_ARGS+=(--mavproxy-args "${SITL_BG_MAVPROXY_ARGS}")
  echo "[start] note: parameter-tune mode keeps background MAVProxy."
  echo "[start]       Standard one-shot mode uses legacy MAVProxy fan-out."
elif [[ "$SITL_DIRECT_MAVLINK" -eq 1 ]]; then
  echo "[start] note: standard mode uses direct MAVLink outputs (no MAVProxy)."
else
  echo "[start] note: standard mode uses legacy MAVProxy fan-out."
fi
if [[ "$ROS2_MODE" != "off" ]]; then
  if [[ -z "${SITL_EKF3_EXTNAV+x}" ]]; then
    export SITL_EKF3_EXTNAV=0
    echo "[start] SITL estimator path: deterministic SIM AHRS/Baro (set SITL_EKF3_EXTNAV=1 for EKF3 ExternalNav)"
  else
    echo "[start] SITL estimator path: SITL_EKF3_EXTNAV=${SITL_EKF3_EXTNAV}"
  fi
fi
"$SITL_SCRIPT" "${SITL_EXTRA_ARGS[@]}" >"$SITL_LOG" 2>&1 &
SITL_PID=$!
echo "[start] SITL bootstrap pid=${SITL_PID}, log=${SITL_LOG}"

# Wait for SITL startup from this run's log (avoid stale-process false positives).
log_has_text() {
  local text="$1"
  if command -v rg >/dev/null 2>&1; then
    rg -Fq -- "$text" "$SITL_LOG"
  else
    grep -Fq -- "$text" "$SITL_LOG"
  fi
}

ardusub_log_has_text() {
  local text="$1"
  [[ -f "$ARDUSUB_RUNTIME_LOG" ]] || return 1
  if command -v rg >/dev/null 2>&1; then
    rg -Fq -- "$text" "$ARDUSUB_RUNTIME_LOG"
  else
    grep -Fq -- "$text" "$ARDUSUB_RUNTIME_LOG"
  fi
}

sitl_has_text() {
  local text="$1"
  log_has_text "$text" || ardusub_log_has_text "$text"
}

SITL_READY=0
WAIT_SECS="${SITL_WAIT_SECS:-240}"
for ((i=1; i<=WAIT_SECS; i++)); do
  if ! kill -0 "$SITL_PID" 2>/dev/null; then
    echo "[start] SITL process exited early. recent log:"
    tail -n 80 "$SITL_LOG" || true
    exit 1
  fi

  if log_has_text "SIM_VEHICLE: Build failed" \
    || log_has_text "you need to install empy" \
    || log_has_text "ModuleNotFoundError:" \
    || log_has_text "Traceback (most recent call last):" \
    || ardusub_log_has_text "PANIC:" \
    || ardusub_log_has_text "Invalid device path:"; then
    echo "[start] SITL failed during bootstrap. recent log:"
    tail -n 120 "$SITL_LOG" || true
    if [[ -f "$ARDUSUB_RUNTIME_LOG" ]]; then
      echo "[start] recent ArduSub runtime log:"
      tail -n 120 "$ARDUSUB_RUNTIME_LOG" || true
    fi
    exit 1
  fi

  if [[ "$SITL_PARAM_TUNE" -eq 1 || "$SITL_DIRECT_MAVLINK" -eq 0 ]]; then
    if log_has_text "SIM_VEHICLE: Run MavProxy" \
      || log_has_text "Waiting for heartbeat from tcp:127.0.0.1:5760" \
      || log_has_text "Connect tcp:127.0.0.1:5760 source_system=255" \
      || log_has_text "link 1 down" \
      || log_has_text "MAV>"; then
      SITL_READY=1
      echo "[start] SITL+MAVProxy ready"
      log_timing "SITL/MAVProxy bootstrap ready"
      break
    fi
  else
    if log_has_text "[start-sitl] transport mode: direct MAVLink outputs (no MAVProxy)" \
      && ( log_has_text "Waiting for SITL to exit" \
        || log_has_text "ArduPilot Ready" \
        || log_has_text "Barometer 1 calibration complete" \
        || ardusub_log_has_text "JSON control interface set to" \
        || ardusub_log_has_text "Loaded defaults from" \
        || ardusub_log_has_text "No JSON sensor message received, resending servos" \
        || ( ardusub_log_has_text "UDP connection 127.0.0.1:14550" \
          && ardusub_log_has_text "UDP connection 127.0.0.1:14551" \
          && ardusub_log_has_text "UDP connection 127.0.0.1:14660" ) ); then
      SITL_READY=1
      echo "[start] SITL ready (direct MAVLink, no MAVProxy)"
      log_timing "SITL direct MAVLink bootstrap ready"
      break
    fi
  fi

  sleep 1
done

if [[ "$SITL_READY" -ne 1 ]]; then
  echo "[start] SITL startup timeout (${WAIT_SECS}s). recent log:"
  tail -n 120 "$SITL_LOG" || true
  exit 1
fi

echo "[start] step 3/3: start MuJoCo"
SITL_MAVLINK_ENDPOINT="udpin:0.0.0.0:14660"
LAUNCH_ARGS=(
  --sitl
  --sitl-mavlink-endpoint "${SITL_MAVLINK_ENDPOINT}"
  --force-clean
)
case "$ROS2_MODE" in
  off)
    LAUNCH_ARGS+=(--no-ros2)
    echo "[start] MuJoCo bridge mode: SITL only (ROS2 disabled)"
    ;;
  full)
    LAUNCH_ARGS+=(--ros2)
    echo "[start] MuJoCo bridge mode: ROS2 full surface (default)"
    ;;
  compat)
    LAUNCH_ARGS+=(--ros2-real-pkg-compat)
    echo "[start] MuJoCo bridge mode: ROS2 compat surface for external MAVROS"
    ;;
  *)
    echo "[start] unknown ROS2 mode: ${ROS2_MODE}"
    exit 2
    ;;
esac
LAUNCH_LOG="${LOG_DIR}/mujoco_${TS}.log"
if ((${#MUJOCO_EXTRA_ARGS[@]} > 0)); then
  "$LAUNCH_SCRIPT" "${LAUNCH_ARGS[@]}" "${MUJOCO_EXTRA_ARGS[@]}" >"$LAUNCH_LOG" 2>&1 &
else
  "$LAUNCH_SCRIPT" "${LAUNCH_ARGS[@]}" >"$LAUNCH_LOG" 2>&1 &
fi
	LAUNCH_PID=$!
	echo "[start] MuJoCo pid=${LAUNCH_PID}, log=${LAUNCH_LOG}"
	log_timing "MuJoCo launch requested"
	echo "[start] startup handoff: SITL and MuJoCo are running; GUI/QGC may connect immediately"

wait "$LAUNCH_PID"
