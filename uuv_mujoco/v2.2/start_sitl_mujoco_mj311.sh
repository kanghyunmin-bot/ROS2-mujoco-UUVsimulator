#!/usr/bin/env bash
set -euo pipefail

# One-shot launcher:
# 1) reset running stack
# 2) start ArduSub SITL (Python venv)
# 3) start MuJoCo UUV sim (Python venv)

WITH_QGC_STOP=0
WIPE_EEPROM=1
NO_RESET=0
SITL_PARAM_TUNE=0
SITL_DIRECT_MAVLINK=0
SITL_NO_REBUILD=0
SITL_FORCE_NO_DISPLAY=1
SITL_EKF_STABLE=1
WAIT_FOR_READY=1
ROS2_MODE="full"
MUJOCO_EXTRA_ARGS=()

usage() {
  cat <<'USAGE'
Usage: ./start_sitl_mujoco_mj311.sh [options] [-- <extra launch_uuv_sim args>]

Options:
  --with-qgc-stop   Stop QGroundControl during reset
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
  --sitl-no-rebuild Pass -N to sim_vehicle.py
  --sitl-no-display Force non-GUI terminal fallback for SITL (default on)
  --no-wait-ready   Do not wait for EKF/baro settle before returning control
  -h, --help        Show this help

Environment:
  WORKSPACE_DIR     Workspace root containing ardupilot/ and optional QGC app
  ARDUPILOT_DIR     Explicit ArduPilot checkout path passed through to child scripts
  QGC_APP           Explicit QGroundControl path (.AppImage, .app, or executable)
  MJ311_ROOT        Preferred Python venv root (optional; falls back to ~/.venvs/mujoco311 if present)
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
  ./start_sitl_mujoco_mj311.sh --with-qgc-stop
  ./start_sitl_mujoco_mj311.sh --param-tune
  ./start_sitl_mujoco_mj311.sh -- --headless
  ./start_sitl_mujoco_mj311.sh -- --qgc-video
  ./start_sitl_mujoco_mj311.sh --ros2-real-pkg-compat -- --headless
USAGE
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --with-qgc-stop)
      WITH_QGC_STOP=1
      shift
      ;;
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
    --sitl-no-display)
      SITL_FORCE_NO_DISPLAY=1
      shift
      ;;
    --no-wait-ready)
      WAIT_FOR_READY=0
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

resolve_qgc_app() {
  local candidate
  for candidate in \
    "${QGC_APP:-}" \
    "${WORKSPACE_DIR}/QGroundControl.AppImage" \
    "${WORKSPACE_DIR}/QGroundControl-x86_64.AppImage" \
    "$HOME/Applications/QGroundControl.AppImage" \
    "${WORKSPACE_DIR}/QGroundControl.app" \
    "/Applications/QGroundControl.app"
  do
    [[ -z "$candidate" ]] && continue
    if [[ -d "$candidate" || -f "$candidate" ]]; then
      printf '%s\n' "$candidate"
      return 0
    fi
  done
  if command -v QGroundControl >/dev/null 2>&1; then
    command -v QGroundControl
    return 0
  fi
  if command -v qgroundcontrol >/dev/null 2>&1; then
    command -v qgroundcontrol
    return 0
  fi
  return 1
}

start_qgc_after_ready() {
  local qgc_app
  if ! qgc_app="$(resolve_qgc_app)"; then
    echo "[start] warning: QGroundControl executable/app not found; skipped auto-launch"
    return 0
  fi

  echo "[start] launching QGroundControl after readiness: ${qgc_app}"
  if [[ -d "$qgc_app" && "$qgc_app" == *.app ]]; then
    if command -v open >/dev/null 2>&1; then
      open "$qgc_app" >/dev/null 2>&1 || echo "[start] warning: failed to open ${qgc_app}"
    else
      echo "[start] warning: macOS app bundle found but 'open' not available"
    fi
    return 0
  fi

  if [[ -f "$qgc_app" ]]; then
    if [[ -x "$qgc_app" ]]; then
      "$qgc_app" >/dev/null 2>&1 &
    else
      echo "[start] warning: ${qgc_app} is not executable"
    fi
    return 0
  fi

  echo "[start] warning: unsupported QGroundControl path: ${qgc_app}"
}

if [[ "$NO_RESET" -eq 0 ]]; then
  RESET_ARGS=()
  [[ "$WITH_QGC_STOP" -eq 1 ]] && RESET_ARGS+=(--with-qgc-stop)
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
      break
    fi
  else
    if log_has_text "[start-sitl] transport mode: direct MAVLink outputs (no MAVProxy)" \
      && ( log_has_text "Waiting for SITL to exit" \
        || log_has_text "ArduPilot Ready" \
        || log_has_text "Barometer 1 calibration complete" \
        || ardusub_log_has_text "Loaded defaults from" \
        || ardusub_log_has_text "No JSON sensor message received, resending servos" \
        || ( ardusub_log_has_text "UDP connection 127.0.0.1:14550" \
          && ardusub_log_has_text "UDP connection 127.0.0.1:14551" \
          && ardusub_log_has_text "UDP connection 127.0.0.1:14660" ) ); then
      SITL_READY=1
      echo "[start] SITL ready (direct MAVLink, no MAVProxy)"
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
if [[ "$SITL_DIRECT_MAVLINK" -eq 1 && "$SITL_PARAM_TUNE" -eq 0 ]]; then
  SITL_MAVLINK_ENDPOINT="none"
fi
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

if [[ "$WAIT_FOR_READY" -eq 1 ]]; then
  echo "[start] step 3.5/3: wait for EKF/baro settle"
  READY_TIMEOUT="${READY_WAIT_SECS:-45}"
  EKF_TILT=0
  EKF_YAW=0
  BARO_READY=0
  DIRECT_AHRS_READY=0
  SIMPLE_AHRS_MODE=0
  AP_READY=0
  READY_SETTLE_TICKS=0
  for ((i=1; i<=READY_TIMEOUT; i++)); do
    if ! kill -0 "$SITL_PID" 2>/dev/null; then
      echo "[start] SITL exited while waiting for readiness. recent log:"
      tail -n 120 "$SITL_LOG" || true
      kill "$LAUNCH_PID" 2>/dev/null || true
      exit 1
    fi
    if ! kill -0 "$LAUNCH_PID" 2>/dev/null; then
      echo "[start] MuJoCo exited while waiting for readiness. recent log:"
      tail -n 120 "$LAUNCH_LOG" || true
      exit 1
    fi

    if log_has_text "Barometer 1 calibration complete" && log_has_text "Barometer 2 calibration complete"; then
      BARO_READY=1
    fi
    if log_has_text "AHRS_EKF_TYPE 10"; then
      SIMPLE_AHRS_MODE=1
    fi
    if log_has_text "ArduPilot Ready"; then
      AP_READY=1
    fi
    if log_has_text "AHRS: DCM active"; then
      DIRECT_AHRS_READY=1
    fi
    if log_has_text "EKF3 IMU0 tilt alignment complete"; then
      EKF_TILT=1
    fi
    if log_has_text "EKF3 IMU0 MAG0 initial yaw alignment complete" \
      || log_has_text "EKF3 IMU0 MAG0 in-flight yaw alignment complete"; then
      EKF_YAW=1
    fi

    if [[ "$BARO_READY" -eq 1 && "$SIMPLE_AHRS_MODE" -eq 1 && "$AP_READY" -eq 1 ]] \
      || [[ "$BARO_READY" -eq 1 && "$DIRECT_AHRS_READY" -eq 1 ]] \
      || [[ "$BARO_READY" -eq 1 && "$EKF_TILT" -eq 1 && "$EKF_YAW" -eq 1 ]]; then
      READY_SETTLE_TICKS=$((READY_SETTLE_TICKS + 1))
      if [[ "$READY_SETTLE_TICKS" -ge 3 ]]; then
        if [[ "$SIMPLE_AHRS_MODE" -eq 1 ]]; then
          echo "[start] readiness OK: baro calibrated, simple AHRS active"
        elif [[ "$EKF_TILT" -eq 1 && "$EKF_YAW" -eq 1 ]]; then
          echo "[start] readiness OK: baro calibrated, EKF tilt/yaw aligned"
        else
          echo "[start] readiness OK: baro calibrated, direct AHRS active"
        fi
        echo "[start] safe sequence: MANUAL -> arm -> wait 5s -> ALT_HOLD"
        if [[ "$WITH_QGC_STOP" -eq 1 ]]; then
          start_qgc_after_ready
        fi
        break
      fi
    else
      READY_SETTLE_TICKS=0
    fi

    sleep 1
  done

  if [[ "$BARO_READY" -ne 1 ]]; then
    echo "[start] readiness timeout (${READY_TIMEOUT}s). recent SITL log:"
    tail -n 120 "$SITL_LOG" || true
  fi
fi

if [[ "$WAIT_FOR_READY" -eq 1 && "$WITH_QGC_STOP" -eq 0 ]]; then
  echo "[start] note: QGC was not stopped. Do not arm or switch modes before readiness OK."
fi

wait "$LAUNCH_PID"
