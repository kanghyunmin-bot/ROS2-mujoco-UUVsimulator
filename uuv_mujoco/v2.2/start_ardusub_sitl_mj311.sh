#!/usr/bin/env bash
set -euo pipefail

# Launch ArduSub SITL with a Python environment that has the required
# ArduPilot-side Python packages. Prefer explicit env overrides, then a
# ~/.venvs/uuv_mujoco layout, then fall back to python3/python on PATH.

if [[ -n "${MJ311_ROOT:-}" ]]; then
  DEFAULT_MJ311_ROOT="${MJ311_ROOT}"
elif [[ -x "$HOME/.venvs/uuv_mujoco/bin/python" ]]; then
  DEFAULT_MJ311_ROOT="$HOME/.venvs/uuv_mujoco"
elif [[ -x "$HOME/.venvs/mujoco311/bin/python" ]]; then
  DEFAULT_MJ311_ROOT="$HOME/.venvs/mujoco311"
else
  DEFAULT_MJ311_ROOT="$HOME/.venvs/uuv_mujoco"
fi
MJ311_ROOT="${MJ311_ROOT:-$DEFAULT_MJ311_ROOT}"
MJ311_PYTHON="${MJ311_PYTHON:-}"

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
ARDUPILOT_DIR="${ARDUPILOT_DIR:-${WORKSPACE_DIR}/ardupilot}"
SIM_VEHICLE="${ARDUPILOT_DIR}/Tools/autotest/sim_vehicle.py"
THRUSTER_MAPPING_SCRIPT="${SCRIPT_DIR}/physics/thruster_mapping.py"

SITL_JSON_HOST="${SITL_JSON_HOST:-${SITL_SIM_ADDRESS:-127.0.0.1}}"
SITL_JSON_SENSOR_PORT="${SITL_JSON_SENSOR_PORT:-${SITL_SIM_PORT_IN:-9003}}"
SITL_JSON_SERVO_PORT="${SITL_JSON_SERVO_PORT:-${SITL_SIM_PORT_OUT:-9002}}"
SITL_CONSOLE_HOST="${SITL_CONSOLE_HOST:-127.0.0.1}"
SITL_CONSOLE_PORT="${SITL_CONSOLE_PORT:-14552}"
SITL_QGC_HOST="${SITL_QGC_HOST:-127.0.0.1}"
SITL_QGC_PORT="${SITL_QGC_PORT:-14550}"
SITL_MAVROS_HOST="${SITL_MAVROS_HOST:-127.0.0.1}"
SITL_MAVROS_PORT="${SITL_MAVROS_PORT:-14551}"
SITL_MUJOCO_MAV_HOST="${SITL_MUJOCO_MAV_HOST:-127.0.0.1}"
SITL_MUJOCO_MAV_PORT="${SITL_MUJOCO_MAV_PORT:-14660}"
SITL_COMMAND_MAV_HOST="${SITL_COMMAND_MAV_HOST:-127.0.0.1}"
SITL_COMMAND_MAV_PORT="${SITL_COMMAND_MAV_PORT:-14661}"
SITL_QGC_OUTPUT_ENABLE="${SITL_QGC_OUTPUT_ENABLE:-1}"
SITL_MAVROS_OUTPUT_ENABLE="${SITL_MAVROS_OUTPUT_ENABLE:-0}"
SITL_DEDICATED_COMMAND_MAVLINK="${SITL_DEDICATED_COMMAND_MAVLINK:-0}"
case "$SITL_DEDICATED_COMMAND_MAVLINK" in
  1|true|TRUE|yes|YES|on|ON|enable|enabled)
    SITL_DEDICATED_COMMAND_MAVLINK=1
    ;;
  *)
    SITL_DEDICATED_COMMAND_MAVLINK=0
    ;;
esac

usage() {
  cat <<'USAGE'
Usage: ./start_ardusub_sitl_mj311.sh [options] [-- <extra sim_vehicle args>]

Options:
  --param-tune         Start interactive MAVProxy mode for parameter tuning
  --direct-mavlink     Bypass MAVProxy and use direct UDP outputs (experimental)
  --no-ekf-stable      Do not apply default EKF stabilization params
  --no-rebuild         Pass -N to sim_vehicle.py (default; avoids rebuilding ArduPilot)
  --rebuild            Rebuild ArduSub before launching
  --force-no-display   Force non-GUI terminal fallback (unset DISPLAY)
  --allow-display      Keep DISPLAY (standard GUI path)
  -h, --help           Show this help

Environment:
  WORKSPACE_DIR        Workspace root containing ardupilot/ (optional)
  ARDUPILOT_DIR        Explicit ArduPilot checkout path (optional)
  MJ311_ROOT           Preferred Python env root (optional)
  MJ311_PYTHON         Explicit Python interpreter (optional)
  SITL_JSON_HOST       MuJoCo JSON servo target host for ArduSub (default 127.0.0.1)
  SITL_CONSOLE_HOST    Non-blocking SERIAL0 UDP target host in direct mode (default 127.0.0.1)
  SITL_QGC_HOST        QGC MAVLink UDP target host (default 127.0.0.1)
  SITL_MAVROS_HOST     MAVROS MAVLink UDP target host (default 127.0.0.1)
  SITL_MUJOCO_MAV_HOST MuJoCo SERVO_OUTPUT_RAW MAVLink UDP target host (default 127.0.0.1)
  SITL_COMMAND_MAV_HOST MuJoCo command MAVLink UDP target host (default 127.0.0.1)
  SITL_QGC_OUTPUT_ENABLE Enable MAVProxy fan-out to QGC 14550 (default 1)
  SITL_MAVROS_OUTPUT_ENABLE Enable MAVProxy fan-out to external MAVROS 14551 (default 0)
  SITL_DEDICATED_COMMAND_MAVLINK Enable separate serial4 command link (default 0)

Examples:
  ./start_ardusub_sitl_mj311.sh
  ./start_ardusub_sitl_mj311.sh --param-tune
  ./start_ardusub_sitl_mj311.sh --no-rebuild -- --speedup 1
USAGE
}

ENABLE_PARAM_TUNE=0
ENABLE_DIRECT_MAVLINK=0
EKF_STABLE=1
NO_REBUILD="${SITL_NO_REBUILD:-1}"
FORCE_NO_DISPLAY="${SITL_FORCE_NO_DISPLAY:-1}"
USER_ARGS=()
USER_SET_MAVPROXY_ARGS=0
SITL_EKF3_EXTNAV_ENABLE=0
case "${SITL_EKF3_EXTNAV:-1}" in
  0|false|FALSE|no|NO|off|OFF|disable|disabled)
    SITL_EKF3_EXTNAV_ENABLE=0
    ;;
  *)
    SITL_EKF3_EXTNAV_ENABLE=1
    ;;
esac
if [[ "$SITL_EKF3_EXTNAV_ENABLE" -eq 1 ]]; then
  SITL_DEFAULT_RNGFND1_TYPE=0
  SITL_DEFAULT_SURFACE_DEPTH=-10.0
  SITL_DEFAULT_AHRS_GPS_USE=1
else
  # Deterministic JSON SITL debug contract: SIM AHRS and Bar30 vertical
  # position only. Use SITL_EKF3_EXTNAV=0 only when deliberately isolating
  # ExternalNav/DVL from the ALT_HOLD loop.
  SITL_DEFAULT_RNGFND1_TYPE=0
  SITL_DEFAULT_SURFACE_DEPTH=-10.0
  SITL_DEFAULT_AHRS_GPS_USE=0
fi

while [[ $# -gt 0 ]]; do
  case "$1" in
    --param-tune)
      ENABLE_PARAM_TUNE=1
      shift
      ;;
    --direct-mavlink)
      ENABLE_DIRECT_MAVLINK=1
      shift
      ;;
    --no-ekf-stable)
      EKF_STABLE=0
      shift
      ;;
    --no-rebuild)
      NO_REBUILD=1
      shift
      ;;
    --rebuild)
      NO_REBUILD=0
      shift
      ;;
    --force-no-display)
      FORCE_NO_DISPLAY=1
      shift
      ;;
    --allow-display)
      FORCE_NO_DISPLAY=0
      shift
      ;;
    --mavproxy-args|--mavproxy-args=*)
      USER_SET_MAVPROXY_ARGS=1
      USER_ARGS+=("$1")
      if [[ "$1" == "--mavproxy-args" ]]; then
        if [[ $# -lt 2 ]]; then
          echo "[error] --mavproxy-args requires a value"
          exit 2
        fi
        USER_ARGS+=("$2")
        shift 2
      else
        shift
      fi
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    --)
      shift
      USER_ARGS+=("$@")
      break
      ;;
    *)
      USER_ARGS+=("$1")
      shift
      ;;
  esac
done

resolve_python() {
  local candidate
  if [[ -n "${MJ311_PYTHON}" ]]; then
    if [[ -x "${MJ311_PYTHON}" ]]; then
      printf '%s\n' "${MJ311_PYTHON}"
      return 0
    fi
    echo "[error] MJ311_PYTHON is not executable: ${MJ311_PYTHON}" >&2
    return 1
  fi

  if [[ -n "${MJ311_ROOT}" && -x "${MJ311_ROOT}/bin/python" ]]; then
    printf '%s\n' "${MJ311_ROOT}/bin/python"
    return 0
  fi

  for candidate in python3 python; do
    if command -v "${candidate}" >/dev/null 2>&1; then
      command -v "${candidate}"
      return 0
    fi
  done

  echo "[error] no usable Python interpreter found." >&2
  echo "        Set MJ311_PYTHON or install python3." >&2
  return 1
}

MJ311_PYTHON="$(resolve_python)"
PYTHON_BIN_DIR="$(cd "$(dirname "${MJ311_PYTHON}")" && pwd)"
IS_VENV=0
if "$MJ311_PYTHON" -c 'import sys; raise SystemExit(0 if sys.prefix != sys.base_prefix else 1)' >/dev/null 2>&1; then
  IS_VENV=1
fi

if [[ ! -x "$MJ311_PYTHON" ]]; then
  echo "[error] resolved python is not executable: $MJ311_PYTHON"
  exit 1
fi
if [[ ! -d "$WORKSPACE_DIR" ]]; then
  echo "[error] workspace directory not found: $WORKSPACE_DIR"
  echo "        Set WORKSPACE_DIR correctly."
  exit 1
fi
if [[ ! -d "$ARDUPILOT_DIR" ]]; then
  echo "[error] ardupilot directory not found: $ARDUPILOT_DIR"
  echo "        Set ARDUPILOT_DIR correctly."
  exit 1
fi
if [[ ! -f "$SIM_VEHICLE" ]]; then
  echo "[error] sim_vehicle.py not found: $SIM_VEHICLE"
  echo "        ARDUPILOT_DIR=$ARDUPILOT_DIR"
  exit 1
fi
if [[ ! -f "$THRUSTER_MAPPING_SCRIPT" ]]; then
  echo "[error] thruster mapping script not found: $THRUSTER_MAPPING_SCRIPT"
  exit 1
fi

# Prefer the selected interpreter's bin directory during the ArduPilot build.
export PATH="${PYTHON_BIN_DIR}:${PATH}"
if [[ "$IS_VENV" -eq 1 ]]; then
  export VIRTUAL_ENV="$(cd "${PYTHON_BIN_DIR}/.." && pwd)"
else
  unset VIRTUAL_ENV
fi
export PYTHON="$MJ311_PYTHON"
unset PYTHONHOME
unset PYTHONPATH

if [[ "$FORCE_NO_DISPLAY" == "1" ]]; then
  # Useful for VM and CI environments where GUI forwarding is undesirable.
  unset DISPLAY
  echo "[start-sitl] DISPLAY disabled (non-GUI terminal fallback)"
fi

ensure_module() {
  local import_name="$1"
  local pip_spec="$2"
  if ! "$MJ311_PYTHON" -c "import ${import_name}" >/dev/null 2>&1; then
    if [[ "$IS_VENV" -eq 1 ]]; then
      echo "[setup] missing module '${import_name}', installing '${pip_spec}'"
      "$MJ311_PYTHON" -m pip install "$pip_spec"
    else
      echo "[error] missing module '${import_name}' in ${MJ311_PYTHON}" >&2
      echo "        Install '${pip_spec}' into this Python or use a venv." >&2
      exit 1
    fi
  fi
}

ensure_pkg_resources() {
  if ! "$MJ311_PYTHON" -c "import pkg_resources" >/dev/null 2>&1; then
    if [[ "$IS_VENV" -eq 1 ]]; then
      echo "[setup] missing module 'pkg_resources', installing 'setuptools<81'"
      "$MJ311_PYTHON" -m pip install "setuptools<81"
    else
      echo "[error] missing module 'pkg_resources' in ${MJ311_PYTHON}" >&2
      echo "        Install 'setuptools<81>' into this Python or use a venv." >&2
      exit 1
    fi
  fi
}

# Common SITL/MAVProxy breakpoints on macOS.
ensure_pkg_resources
ensure_module em "empy==3.3.4"
ensure_module future "future"
if [[ "$(uname -s)" == "Darwin" ]]; then
  ensure_module gnureadline "gnureadline"
fi
ensure_module PIL "pillow"
ensure_module dronecan "dronecan"

mapping_value() {
  "$MJ311_PYTHON" "$THRUSTER_MAPPING_SCRIPT" "$1"
}

SIM_ARGS=()
case "${SITL_NO_EXTRA_PORTS:-1}" in
  1|true|TRUE|yes|YES|on|ON|disable-default|disabled)
    SIM_ARGS+=(--no-extra-ports)
    echo "[start-sitl] sim_vehicle default MAVProxy ports disabled; using explicit outputs only"
    ;;
esac
if [[ "$NO_REBUILD" -eq 1 ]]; then
  SIM_ARGS+=(-N)
fi

USE_REAL_PARAM_FILE=0
case "${SITL_USE_REAL_PARAM_FILE:-1}" in
  1|true|TRUE|yes|YES|on|ON|enable|enabled)
    USE_REAL_PARAM_FILE=1
    ;;
esac
DEFAULT_REAL_PARAM_FILE="${SCRIPT_DIR}/config/ardusub_realrobot_contract.param"
if [[ ! -f "$DEFAULT_REAL_PARAM_FILE" ]]; then
  DEFAULT_REAL_PARAM_FILE="${WORKSPACE_DIR}/real_robot.param"
fi
REAL_PARAM_FILE="${SITL_REAL_PARAM_FILE:-$DEFAULT_REAL_PARAM_FILE}"
if [[ "$USE_REAL_PARAM_FILE" -eq 1 && -f "$REAL_PARAM_FILE" ]]; then
  SIM_ARGS+=(--add-param-file "$REAL_PARAM_FILE")
  echo "[start-sitl] loading real vehicle params: ${REAL_PARAM_FILE}"
elif [[ "$USE_REAL_PARAM_FILE" -eq 1 ]]; then
  echo "[error] missing real vehicle contract param file: ${REAL_PARAM_FILE}" >&2
  echo "        Set SITL_USE_REAL_PARAM_FILE=0 only for isolated SITL debug." >&2
  exit 1
fi

has_param_override() {
  local key="$1"
  local idx token next_idx next_token
  for idx in "${!USER_ARGS[@]}"; do
    token="${USER_ARGS[$idx]}"
    if [[ "$token" == *"${key}="* ]]; then
      return 0
    fi
    if [[ "$token" == "-P" || "$token" == "--param" ]]; then
      next_idx=$((idx + 1))
      if (( next_idx < ${#USER_ARGS[@]} )); then
        next_token="${USER_ARGS[$next_idx]}"
        if [[ "$next_token" == "${key}="* ]]; then
          return 0
        fi
      fi
    fi
  done
  return 1
}

EXTRA_PARAM_LINES=()
append_param_if_not_overridden() {
  local key="$1"
  local value="$2"
  if ! has_param_override "$key"; then
    EXTRA_PARAM_LINES+=("${key} ${value}")
  fi
}

# Core frame/output layout (keep deterministic across runs and EEPROM states).
append_param_if_not_overridden "FRAME_CONFIG" "2"
append_param_if_not_overridden "SERVO1_FUNCTION" "33"
append_param_if_not_overridden "SERVO2_FUNCTION" "34"
append_param_if_not_overridden "SERVO3_FUNCTION" "35"
append_param_if_not_overridden "SERVO4_FUNCTION" "36"
append_param_if_not_overridden "SERVO5_FUNCTION" "37"
append_param_if_not_overridden "SERVO6_FUNCTION" "38"
append_param_if_not_overridden "SERVO7_FUNCTION" "39"
append_param_if_not_overridden "SERVO8_FUNCTION" "40"
# The real rosbag RCOUT channels are bounded by 1100..1900us. ArduSub motor
# functions use MOT_PWM_* for SERVO_OUTPUT_RAW limits, so keep SITL on the same
# PWM envelope instead of the SITL default 1000..2000us.
append_param_if_not_overridden "MOT_PWM_MIN" "1100"
append_param_if_not_overridden "MOT_PWM_MAX" "1900"
# Match the real vehicle's QGC Motor Config reverse setup used by the rosbag.
# ArduSub's 6DOF motor mixer applies these MOT_*_DIRECTION parameters before
# writing SERVO_OUTPUT_RAW. SERVO*_REVERSED is intentionally not used here:
# QGC's Motor Config reverse checkboxes correspond to motor direction, not a
# generic servo-channel reverse in the SITL output path.
append_param_if_not_overridden "MOT_1_DIRECTION" "1"
append_param_if_not_overridden "MOT_2_DIRECTION" "1"
append_param_if_not_overridden "MOT_3_DIRECTION" "-1"
append_param_if_not_overridden "MOT_4_DIRECTION" "-1"
append_param_if_not_overridden "MOT_5_DIRECTION" "-1"
append_param_if_not_overridden "MOT_6_DIRECTION" "1"
append_param_if_not_overridden "MOT_7_DIRECTION" "1"
append_param_if_not_overridden "MOT_8_DIRECTION" "-1"
append_param_if_not_overridden "SERIAL1_PROTOCOL" "2"
append_param_if_not_overridden "SERIAL2_PROTOCOL" "2"
# This is a SITL-only control/telemetry link to MuJoCo. The real vehicle uses
# SERIAL3_PROTOCOL=5 for GPS, but in this launch serial3 is explicitly wired to
# udpclient:<host>:14660. It must therefore be MAVLink or arm/mode/RC commands
# and SERVO_OUTPUT_RAW never reach the bridge.
EXTRA_PARAM_LINES+=("SERIAL3_PROTOCOL 2")
if [[ "$SITL_DEDICATED_COMMAND_MAVLINK" -eq 1 ]]; then
  # Optional split command path. The default Docker/Mac path keeps commands on
  # serial3 to avoid another UDP/heartbeat failure mode.
  EXTRA_PARAM_LINES+=("SERIAL4_PROTOCOL 2")
fi
# real_robot.param has BRD_OPTIONS=1 because the physical Pixhawk watchdog is
# valid hardware behavior. In SITL that same bit enables the SIGALRM watchdog;
# a brief Docker/Mac scheduling stall during JSON startup causes watchdog_rst,
# then ArduSub refuses to arm. Keep this disabled in simulation.
append_param_if_not_overridden "BRD_OPTIONS" "${SITL_BRD_OPTIONS:-0}"
# Keep two estimator contracts explicit. The default path matches the real
# robot vertical EKF contract: POSZ from Baro/Bar30 and VELZ from ExternalNav.
# Set SITL_EKF3_EXTNAV=0 only for deterministic Bar30-only debugging.
append_param_if_not_overridden "RNGFND1_TYPE" "${SITL_RNGFND1_TYPE:-$SITL_DEFAULT_RNGFND1_TYPE}"
append_param_if_not_overridden "RNGFND1_MIN_CM" "5"
append_param_if_not_overridden "RNGFND1_MAX_CM" "3000"
append_param_if_not_overridden "RNGFND1_ORIENT" "25"
# Match the validated depth-hold report values. Units are cm and normalized
# throttle; both participate in AltHold surface limiting and bottom behavior.
append_param_if_not_overridden "SURFACE_DEPTH" "${SITL_SURFACE_DEPTH:-$SITL_DEFAULT_SURFACE_DEPTH}"
append_param_if_not_overridden "SURFACE_MAX_THR" "${SITL_SURFACE_MAX_THR:-0.1}"
# ArduSub 4.1.x accepts RC override/manual control only from SYSID_MYGCS.
# Match the real vehicle dump and QGC joystick path.
append_param_if_not_overridden "SYSID_MYGCS" "255"
# Closed-loop pilot-input contract:
# - GUI and rosbag replay use /mavros/rc/override by default.
# - RC3 neutral is 1500 because RC override maps heave directly to 1100..1900.
# - RC3_TRIM may remain the real vehicle value; do not reinterpret it as stick
#   neutral for the RC override path.
# - MANUAL_CONTROL remains available for QGC-like joystick behavior and is
#   scaled by JS_GAIN/JS_THR_GAIN inside ArduSub.
append_param_if_not_overridden "RC_OPTIONS" "${SITL_RC_OPTIONS:-32}"
append_param_if_not_overridden "RC_OVERRIDE_TIME" "${SITL_RC_OVERRIDE_TIME:-3.0}"
append_param_if_not_overridden "RC1_DZ" "${SITL_RC1_DZ:-30}"
append_param_if_not_overridden "RC2_DZ" "${SITL_RC2_DZ:-30}"
append_param_if_not_overridden "THR_DZ" "${SITL_THR_DZ:-100}"
append_param_if_not_overridden "RC3_MIN" "1100"
append_param_if_not_overridden "RC3_MAX" "1900"
append_param_if_not_overridden "RC3_DZ" "${SITL_RC3_DZ:-30}"
append_param_if_not_overridden "RC3_TRIM" "${SITL_RC3_TRIM:-1100}"
append_param_if_not_overridden "JS_GAIN_DEFAULT" "${SITL_JS_GAIN_DEFAULT:-0.1}"
append_param_if_not_overridden "JS_GAIN_MIN" "${SITL_JS_GAIN_MIN:-0.25}"
append_param_if_not_overridden "JS_GAIN_MAX" "${SITL_JS_GAIN_MAX:-2.0}"
append_param_if_not_overridden "JS_GAIN_STEPS" "${SITL_JS_GAIN_STEPS:-4}"
append_param_if_not_overridden "JS_THR_GAIN" "${SITL_JS_THR_GAIN:-1.0}"
append_param_if_not_overridden "PILOT_SPEED_UP" "${SITL_PILOT_SPEED_UP:-100}"
append_param_if_not_overridden "PILOT_SPEED_DN" "${SITL_PILOT_SPEED_DN:-0}"
append_param_if_not_overridden "PILOT_ACCEL_Z" "${SITL_PILOT_ACCEL_Z:-100}"
# Keep yaw stick neutral near center in assisted modes. ArduSub uses the yaw
# channel deadzone for get_pilot_desired_yaw_rate() in ALT_HOLD/POSHOLD.
append_param_if_not_overridden "RC4_DZ" "${SITL_RC4_DZ:-40}"
# ArduSub ALT_HOLD currently passes forward/lateral through norm_input(), so
# RC5_DZ/RC6_DZ are explicit for parameter parity and modes that use channel DZ.
append_param_if_not_overridden "RC5_DZ" "${SITL_RC5_DZ:-30}"
append_param_if_not_overridden "RC6_DZ" "${SITL_RC6_DZ:-30}"
# Apply the real vehicle parameter dump from real_robot.param. The checkout is
# pinned to ArduSub-4.1.2, so these names intentionally match the older QGC
# parameter names used by the hardware.
append_param_if_not_overridden "ATC_INPUT_TC" "0.15"
append_param_if_not_overridden "ATC_ANG_RLL_P" "12.0"
append_param_if_not_overridden "ATC_ANG_PIT_P" "12.0"
append_param_if_not_overridden "ATC_ANG_YAW_P" "4.5"
append_param_if_not_overridden "ACRO_YAW_P" "3.375"
append_param_if_not_overridden "ATC_ACCEL_Y_MAX" "110000.0"
append_param_if_not_overridden "ATC_RATE_Y_MAX" "180.0"
append_param_if_not_overridden "ATC_SLEW_YAW" "2000.0"
append_param_if_not_overridden "ATC_RATE_FF_ENAB" "1"
append_param_if_not_overridden "ATC_RAT_RLL_P" "0.30"
append_param_if_not_overridden "ATC_RAT_RLL_I" "0.49"
append_param_if_not_overridden "ATC_RAT_RLL_IMAX" "0.02"
append_param_if_not_overridden "ATC_RAT_RLL_D" "0.02"
append_param_if_not_overridden "ATC_RAT_RLL_FLTD" "30.0"
append_param_if_not_overridden "ATC_RAT_RLL_FLTE" "0.0"
append_param_if_not_overridden "ATC_RAT_RLL_FLTT" "30.0"
append_param_if_not_overridden "ATC_RAT_PIT_P" "0.30"
append_param_if_not_overridden "ATC_RAT_PIT_I" "0.10"
append_param_if_not_overridden "ATC_RAT_PIT_IMAX" "0.0"
append_param_if_not_overridden "ATC_RAT_PIT_D" "0.02"
append_param_if_not_overridden "ATC_RAT_PIT_FLTD" "30.0"
append_param_if_not_overridden "ATC_RAT_PIT_FLTE" "0.0"
append_param_if_not_overridden "ATC_RAT_PIT_FLTT" "30.0"
append_param_if_not_overridden "ATC_RAT_YAW_P" "0.49"
append_param_if_not_overridden "ATC_RAT_YAW_I" "0.0"
append_param_if_not_overridden "ATC_RAT_YAW_IMAX" "0.02"
append_param_if_not_overridden "ATC_RAT_YAW_D" "0.02"
append_param_if_not_overridden "ATC_RAT_YAW_FLTD" "5.0"
append_param_if_not_overridden "ATC_RAT_YAW_FLTE" "0.0"
append_param_if_not_overridden "ATC_RAT_YAW_FLTT" "5.0"
# Real dump: PSC_POSXY_P=2.00, PSC_POSZ_P=3.00
append_param_if_not_overridden "PSC_POSXY_P" "2.0"
append_param_if_not_overridden "PSC_POSZ_P" "${SITL_PSC_POSZ_P:-3.0}"
# QGC: PSC_VELXY_P=6.00, PSC_VELXY_I=0.02, PSC_VELXY_IMAX=1000 cm/s/s
append_param_if_not_overridden "PSC_JERK_XY" "5.0"
append_param_if_not_overridden "PSC_JERK_Z" "50.0"
append_param_if_not_overridden "PSC_VELXY_P" "6.0"
append_param_if_not_overridden "PSC_VELXY_I" "0.02"
append_param_if_not_overridden "PSC_VELXY_IMAX" "1000.0"
append_param_if_not_overridden "PSC_VELXY_D" "0.0"
append_param_if_not_overridden "PSC_VELXY_FF" "0.0"
append_param_if_not_overridden "PSC_VELXY_FLTD" "5.0"
append_param_if_not_overridden "PSC_VELXY_FLTE" "5.0"
# QGC: PSC_VELZ_P=8.00
append_param_if_not_overridden "PSC_VELZ_P" "${SITL_PSC_VELZ_P:-8.0}"
append_param_if_not_overridden "PSC_VELZ_I" "${SITL_PSC_VELZ_I:-0.0}"
append_param_if_not_overridden "PSC_VELZ_IMAX" "${SITL_PSC_VELZ_IMAX:-1000.0}"
append_param_if_not_overridden "PSC_VELZ_D" "${SITL_PSC_VELZ_D:-0.0}"
append_param_if_not_overridden "PSC_VELZ_FF" "${SITL_PSC_VELZ_FF:-0.0}"
append_param_if_not_overridden "PSC_VELZ_FLTD" "${SITL_PSC_VELZ_FLTD:-5.0}"
append_param_if_not_overridden "PSC_VELZ_FLTE" "${SITL_PSC_VELZ_FLTE:-5.0}"
# Real dump: PSC_ACCZ_P=0.50, PSC_ACCZ_I=0.00, PSC_ACCZ_IMAX=0, PSC_ACCZ_D=0.00
append_param_if_not_overridden "PSC_ACCZ_P" "${SITL_PSC_ACCZ_P:-0.50}"
append_param_if_not_overridden "PSC_ACCZ_I" "${SITL_PSC_ACCZ_I:-0.0}"
append_param_if_not_overridden "PSC_ACCZ_IMAX" "${SITL_PSC_ACCZ_IMAX:-0.0}"
append_param_if_not_overridden "PSC_ACCZ_D" "${SITL_PSC_ACCZ_D:-0.0}"
append_param_if_not_overridden "PSC_ACCZ_FLTD" "${SITL_PSC_ACCZ_FLTD:-6.94}"
append_param_if_not_overridden "PSC_ACCZ_FLTE" "${SITL_PSC_ACCZ_FLTE:-20.0}"
append_param_if_not_overridden "PSC_ACCZ_FLTT" "${SITL_PSC_ACCZ_FLTT:-0.0}"
# QGC waypoint/loiter snapshot in ArduSub-4.1 units.
append_param_if_not_overridden "WPNAV_ACCEL" "250.0"
append_param_if_not_overridden "WPNAV_ACCEL_Z" "100.0"
append_param_if_not_overridden "WPNAV_JERK" "1.0"
append_param_if_not_overridden "WPNAV_RADIUS" "200.0"
append_param_if_not_overridden "WPNAV_SPEED" "100.0"
append_param_if_not_overridden "WPNAV_SPEED_DN" "150.0"
append_param_if_not_overridden "WPNAV_SPEED_UP" "250.0"
append_param_if_not_overridden "LOIT_SPEED" "1250.0"
append_param_if_not_overridden "LOIT_ACC_MAX" "500.0"
append_param_if_not_overridden "LOIT_ANG_MAX" "0.0"
append_param_if_not_overridden "LOIT_BRK_ACCEL" "250.0"
append_param_if_not_overridden "LOIT_BRK_DELAY" "1.0"
append_param_if_not_overridden "LOIT_BRK_JERK" "500.0"
append_param_if_not_overridden "MOT_FV_CPLNG_K" "1.0"
append_param_if_not_overridden "MOT_SPIN_ARM" "0.1"
append_param_if_not_overridden "MOT_SPIN_MIN" "0.15"
append_param_if_not_overridden "MOT_SPIN_MAX" "0.95"
append_param_if_not_overridden "MOT_SPOOL_TIME" "0.5"
append_param_if_not_overridden "MOT_THST_EXPO" "0.65"
append_param_if_not_overridden "MOT_THST_HOVER" "${SITL_MOT_THST_HOVER:-0.5}"
append_param_if_not_overridden "MOT_YAW_HEADROOM" "200"
# Battery/pressure parameters from the real vehicle dump. SITL_BATT_ARM_VOLT
# remains an escape hatch for bench/debug runs, but the default matches hardware.
append_param_if_not_overridden "BATT_MONITOR" "4"
append_param_if_not_overridden "BATT_CAPACITY" "5200"
append_param_if_not_overridden "BATT_ARM_VOLT" "${SITL_BATT_ARM_VOLT:-22.2}"
append_param_if_not_overridden "BATT_VOLT_PIN" "2"
append_param_if_not_overridden "BATT_CURR_PIN" "3"
append_param_if_not_overridden "BATT_VOLT_MULT" "10.100"
append_param_if_not_overridden "BATT_AMP_PERVLT" "17.000"
append_param_if_not_overridden "BATT_AMP_OFFSET" "0.0"
append_param_if_not_overridden "SIM_BATT_VOLTAGE" "${SITL_BATT_VOLTAGE:-24.0}"
append_param_if_not_overridden "SIM_BATT_CAP_AH" "5.2"
append_param_if_not_overridden "BARO1_GND_PRESS" "101473.796875"
append_param_if_not_overridden "BARO2_GND_PRESS" "101640"
append_param_if_not_overridden "BARO_PRIMARY" "1"
append_param_if_not_overridden "BARO_SPEC_GRAV" "1.0"
append_param_if_not_overridden "BARO_ALT_OFFSET" "0.0"
# Depth hold in SITL is driven by the simulated water barometer. The ArduPilot
# SITL default adds 0.2 m of baro noise, which can create a false vertical error
# immediately after MuJoCo's initial-depth hold is released.
append_param_if_not_overridden "SIM_BARO_RND" "${SITL_SIM_BARO_RND:-0}"
append_param_if_not_overridden "SIM_BARO_DRIFT" "${SITL_SIM_BARO_DRIFT:-0}"
append_param_if_not_overridden "SIM_BARO_GLITCH" "${SITL_SIM_BARO_GLITCH:-0}"
append_param_if_not_overridden "SIM_BARO_DELAY" "${SITL_SIM_BARO_DELAY:-0}"
append_param_if_not_overridden "SIM_BAR2_RND" "${SITL_SIM_BAR2_RND:-0}"
append_param_if_not_overridden "SIM_BAR2_DRIFT" "${SITL_SIM_BAR2_DRIFT:-0}"
append_param_if_not_overridden "SIM_BAR2_GLITCH" "${SITL_SIM_BAR2_GLITCH:-0}"
append_param_if_not_overridden "SIM_BAR2_DELAY" "${SITL_SIM_BAR2_DELAY:-0}"
# real_robot.param contains the physical Pixhawk's raw IMU calibration
# offsets. MuJoCo JSON already sends calibrated body gyro/specific-force
# samples, so applying the hardware offsets in SITL creates a false vertical
# acceleration bias (notably INS_ACCOFFS_Z=-1.207 on the real dump) and EKF3
# enters ALT_HOLD with non-zero down velocity while the vehicle is stationary.
append_param_if_not_overridden "INS_ACCOFFS_X" "${SITL_INS_ACCOFFS_X:-0.0}"
append_param_if_not_overridden "INS_ACCOFFS_Y" "${SITL_INS_ACCOFFS_Y:-0.0}"
append_param_if_not_overridden "INS_ACCOFFS_Z" "${SITL_INS_ACCOFFS_Z:-0.0}"
append_param_if_not_overridden "INS_ACCSCAL_X" "${SITL_INS_ACCSCAL_X:-1.0}"
append_param_if_not_overridden "INS_ACCSCAL_Y" "${SITL_INS_ACCSCAL_Y:-1.0}"
append_param_if_not_overridden "INS_ACCSCAL_Z" "${SITL_INS_ACCSCAL_Z:-1.0}"
append_param_if_not_overridden "INS_GYROFFS_X" "${SITL_INS_GYROFFS_X:-0.0}"
append_param_if_not_overridden "INS_GYROFFS_Y" "${SITL_INS_GYROFFS_Y:-0.0}"
append_param_if_not_overridden "INS_GYROFFS_Z" "${SITL_INS_GYROFFS_Z:-0.0}"
append_param_if_not_overridden "EK3_ALT_M_NSE" "0.1"
append_param_if_not_overridden "EK3_GBIAS_P_NSE" "0.0005"
append_param_if_not_overridden "EK3_GND_EFF_DZ" "4"
append_param_if_not_overridden "EK3_RNG_USE_HGT" "-1"
append_param_if_not_overridden "EK3_YAW_M_NSE" "0.05236"
# Match the hardware dump's GPS params. EKF3 source selection below keeps
# ExternalNav primary for XY/velocity/yaw, so GPS is present but not the source
# under the default real-robot-like path.
append_param_if_not_overridden "GPS_TYPE" "1"
append_param_if_not_overridden "GPS_TYPE2" "0"
append_param_if_not_overridden "SIM_GPS_TYPE" "0"
append_param_if_not_overridden "SIM_GPS2_TYPE" "0"
append_param_if_not_overridden "AHRS_GPS_USE" "${SITL_AHRS_GPS_USE:-$SITL_DEFAULT_AHRS_GPS_USE}"
append_param_if_not_overridden "GPS_AUTO_CONFIG" "1"
append_param_if_not_overridden "INS_POS1_X" "0.145"
append_param_if_not_overridden "INS_POS1_Y" "0.0"
append_param_if_not_overridden "INS_POS1_Z" "0.0"
if [[ "$SITL_EKF3_EXTNAV_ENABLE" -eq 1 ]]; then
  # Real-robot-like estimator path. EKF3 source value 6 is ExternalNav
  # (VisualOdom). POSZ stays Baro like the hardware dump, while vertical
  # velocity and yaw come from ExternalNav.
  append_param_if_not_overridden "AHRS_EKF_TYPE" "${SITL_AHRS_EKF_TYPE:-3}"
  append_param_if_not_overridden "EK3_SRC1_POSXY" "6"
  append_param_if_not_overridden "EK3_SRC1_VELXY" "6"
  append_param_if_not_overridden "EK3_SRC1_POSZ" "${SITL_EKF3_EXTNAV_POSZ:-1}"
  append_param_if_not_overridden "EK3_SRC1_VELZ" "${SITL_EKF3_EXTNAV_VELZ:-6}"
  append_param_if_not_overridden "EK3_SRC1_YAW" "6"
  append_param_if_not_overridden "EK3_SRC2_POSXY" "0"
  append_param_if_not_overridden "EK3_SRC2_VELXY" "0"
  append_param_if_not_overridden "EK3_SRC2_POSZ" "1"
  append_param_if_not_overridden "EK3_SRC2_VELZ" "0"
  append_param_if_not_overridden "EK3_SRC2_YAW" "6"
  append_param_if_not_overridden "EK3_SRC_OPTIONS" "1"
  append_param_if_not_overridden "VISO_TYPE" "1"
  append_param_if_not_overridden "VISO_DELAY_MS" "100"
  append_param_if_not_overridden "VISO_POS_X" "0.0"
  append_param_if_not_overridden "VISO_POS_Y" "0.0"
  append_param_if_not_overridden "VISO_POS_Z" "0.0"
  append_param_if_not_overridden "VISO_POS_M_NSE" "0.2"
  append_param_if_not_overridden "VISO_VEL_M_NSE" "0.1"
  append_param_if_not_overridden "VISO_YAW_M_NSE" "0.050004"
else
  append_param_if_not_overridden "EK3_SRC1_POSXY" "0"
  append_param_if_not_overridden "EK3_SRC1_VELXY" "0"
  append_param_if_not_overridden "EK3_SRC1_POSZ" "1"
  append_param_if_not_overridden "EK3_SRC1_VELZ" "0"
  append_param_if_not_overridden "EK3_SRC1_YAW" "0"
  append_param_if_not_overridden "EK3_SRC_OPTIONS" "0"
  append_param_if_not_overridden "VISO_TYPE" "0"
  # The default keeps the deterministic SIM AHRS path for debug stability.
  # Use SITL_AHRS_EKF_TYPE=3 only together with SITL_EKF3_EXTNAV=1 when
  # validating estimator behavior against real logs.
  append_param_if_not_overridden "AHRS_EKF_TYPE" "${SITL_AHRS_EKF_TYPE:-10}"
fi
append_param_if_not_overridden "COMPASS_ENABLE" "0"
append_param_if_not_overridden "COMPASS_USE" "0"
append_param_if_not_overridden "COMPASS_USE2" "0"
append_param_if_not_overridden "COMPASS_USE3" "0"

if [[ "$EKF_STABLE" -eq 1 ]]; then
  # Keep a stable EKF lane setup in SITL.
  append_param_if_not_overridden "EK3_IMU_MASK" "1"
  append_param_if_not_overridden "GPS_AUTO_SWITCH" "1"
  append_param_if_not_overridden "INS_USE2" "0"
  append_param_if_not_overridden "INS_USE3" "0"
fi

# Use an extra param file instead of -P so both old/new sim_vehicle.py work.
if ((${#EXTRA_PARAM_LINES[@]} > 0)); then
  EXTRA_PARAM_FILE="$(mktemp "${TMPDIR:-/tmp}/sitl_extra_params_XXXXXX")"
  printf '%s\n' "${EXTRA_PARAM_LINES[@]}" > "$EXTRA_PARAM_FILE"
  SIM_ARGS+=(--add-param-file "$EXTRA_PARAM_FILE")
  echo "[start-sitl] enforcing params via ${EXTRA_PARAM_FILE}"
  printf '  %s\n' "${EXTRA_PARAM_LINES[@]}"
fi

cd "$ARDUPILOT_DIR"
USE_DIRECT_MAVLINK=0
case "${SITL_DIRECT_MAVLINK:-0}" in
  1|true|TRUE|yes|YES|on|ON)
    USE_DIRECT_MAVLINK=1
    ;;
esac
if [[ "$ENABLE_DIRECT_MAVLINK" -eq 1 ]]; then
  USE_DIRECT_MAVLINK=1
fi
if [[ "$ENABLE_PARAM_TUNE" -eq 1 || "$USER_SET_MAVPROXY_ARGS" -eq 1 ]]; then
  USE_DIRECT_MAVLINK=0
fi

if [[ "$USE_DIRECT_MAVLINK" -eq 1 ]]; then
  echo "[start-sitl] transport mode: direct MAVLink outputs (no MAVProxy)"
  echo "[start-sitl] direct endpoints:"
  echo "  JSON     -> ${SITL_JSON_HOST}:${SITL_JSON_SERVO_PORT} (sensors in ${SITL_JSON_SENSOR_PORT})"
  echo "  SERIAL0  -> udpclient:${SITL_CONSOLE_HOST}:${SITL_CONSOLE_PORT}"
  echo "  MuJoCo   -> serial3 udpclient:${SITL_MUJOCO_MAV_HOST}:${SITL_MUJOCO_MAV_PORT}"
  SERIAL_ARGS="--sim-address=${SITL_JSON_HOST} --sim-port-in=${SITL_JSON_SENSOR_PORT} --sim-port-out=${SITL_JSON_SERVO_PORT} --serial0=udpclient:${SITL_CONSOLE_HOST}:${SITL_CONSOLE_PORT} --serial3=udpclient:${SITL_MUJOCO_MAV_HOST}:${SITL_MUJOCO_MAV_PORT}"
  case "$SITL_QGC_OUTPUT_ENABLE" in
    1|true|TRUE|yes|YES|on|ON|enable|enabled)
      echo "  QGC      -> serial1 udpclient:${SITL_QGC_HOST}:${SITL_QGC_PORT}"
      SERIAL_ARGS+=" --serial1=udpclient:${SITL_QGC_HOST}:${SITL_QGC_PORT}"
      ;;
    *)
      echo "  QGC      -> disabled"
      ;;
  esac
  case "$SITL_MAVROS_OUTPUT_ENABLE" in
    1|true|TRUE|yes|YES|on|ON|enable|enabled)
      echo "  MAVROS   -> serial2 udpclient:${SITL_MAVROS_HOST}:${SITL_MAVROS_PORT}"
      SERIAL_ARGS+=" --serial2=udpclient:${SITL_MAVROS_HOST}:${SITL_MAVROS_PORT}"
      ;;
    *)
      echo "  MAVROS   -> disabled"
      ;;
  esac
  if [[ "$SITL_DEDICATED_COMMAND_MAVLINK" -eq 1 ]]; then
    echo "  Command  -> serial4 udpclient:${SITL_COMMAND_MAV_HOST}:${SITL_COMMAND_MAV_PORT}"
    SERIAL_ARGS+=" --serial4=udpclient:${SITL_COMMAND_MAV_HOST}:${SITL_COMMAND_MAV_PORT}"
  else
    echo "  Command  -> serial3 shared MuJoCo MAVLink link"
  fi
  SIM_ARGS+=(--no-mavproxy)
  SITL_CMD=("$MJ311_PYTHON" "$SIM_VEHICLE" \
    -L RATBeach \
    -v ArduSub \
    -f vectored_6dof \
    --model JSON \
    -A "$SERIAL_ARGS")
  if ((${#SIM_ARGS[@]})); then
    SITL_CMD+=("${SIM_ARGS[@]}")
  fi
  if ((${#USER_ARGS[@]})); then
    SITL_CMD+=("${USER_ARGS[@]}")
  fi
  exec "${SITL_CMD[@]}"
fi

echo "[start-sitl] transport mode: legacy MAVProxy fan-out"
echo "[start-sitl] JSON endpoint: ${SITL_JSON_HOST}:${SITL_JSON_SERVO_PORT} (sensors in ${SITL_JSON_SENSOR_PORT})"
echo "[start-sitl] MAVProxy fan-out endpoints:"
MAVPROXY_OUT_ARGS=(--out=udp:${SITL_MUJOCO_MAV_HOST}:${SITL_MUJOCO_MAV_PORT})
echo "  MuJoCo/bridge -> udp:${SITL_MUJOCO_MAV_HOST}:${SITL_MUJOCO_MAV_PORT}"
case "$SITL_QGC_OUTPUT_ENABLE" in
  1|true|TRUE|yes|YES|on|ON|enable|enabled)
    MAVPROXY_OUT_ARGS+=(--out=udp:${SITL_QGC_HOST}:${SITL_QGC_PORT})
    echo "  QGC           -> udp:${SITL_QGC_HOST}:${SITL_QGC_PORT}"
    ;;
  *)
    echo "  QGC           -> disabled"
    ;;
esac
case "$SITL_MAVROS_OUTPUT_ENABLE" in
  1|true|TRUE|yes|YES|on|ON|enable|enabled)
    MAVPROXY_OUT_ARGS+=(--out=udp:${SITL_MAVROS_HOST}:${SITL_MAVROS_PORT})
    echo "  External MAVROS -> udp:${SITL_MAVROS_HOST}:${SITL_MAVROS_PORT}"
    ;;
  *)
    echo "  External MAVROS -> disabled"
    ;;
esac
if [[ "$SITL_DEDICATED_COMMAND_MAVLINK" -eq 1 ]]; then
  MAVPROXY_OUT_ARGS+=(--out=udp:${SITL_COMMAND_MAV_HOST}:${SITL_COMMAND_MAV_PORT})
  echo "  Command       -> udp:${SITL_COMMAND_MAV_HOST}:${SITL_COMMAND_MAV_PORT}"
else
  echo "  Command       -> shared MuJoCo/bridge endpoint"
fi
if [[ "$USER_SET_MAVPROXY_ARGS" -eq 0 ]]; then
  SITL_MAVPROXY_ARGS="${SITL_MAVPROXY_ARGS:---non-interactive --nowait}"
fi
if [[ "$USER_SET_MAVPROXY_ARGS" -eq 0 && -n "${SITL_MAVPROXY_ARGS:-}" ]]; then
  SIM_ARGS+=(--mavproxy-args "${SITL_MAVPROXY_ARGS}")
fi
SITL_CMD=("$MJ311_PYTHON" "$SIM_VEHICLE" \
  -L RATBeach \
  -v ArduSub \
  -f vectored_6dof \
  --model JSON \
  -A "--sim-address=${SITL_JSON_HOST} --sim-port-in=${SITL_JSON_SENSOR_PORT} --sim-port-out=${SITL_JSON_SERVO_PORT}" \
  "${MAVPROXY_OUT_ARGS[@]}")
if ((${#SIM_ARGS[@]})); then
  SITL_CMD+=("${SIM_ARGS[@]}")
fi
if ((${#USER_ARGS[@]})); then
  SITL_CMD+=("${USER_ARGS[@]}")
fi
exec "${SITL_CMD[@]}"
