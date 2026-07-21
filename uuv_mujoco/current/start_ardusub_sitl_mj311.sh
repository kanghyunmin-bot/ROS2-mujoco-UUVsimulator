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
    [[ -d "$dir/ardupilot_sub_stable" ]] && current_score=$((current_score + 20))
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
if [[ -z "${ARDUPILOT_DIR:-}" && -d "${WORKSPACE_DIR}/ardupilot_sub_stable" ]]; then
  ARDUPILOT_DIR="${WORKSPACE_DIR}/ardupilot_sub_stable"
fi
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
SITL_INSTANCE="${SITL_INSTANCE:-auto}"
SITL_QGC_OUTPUT_ENABLE="${SITL_QGC_OUTPUT_ENABLE:-1}"
SITL_MAVROS_OUTPUT_ENABLE="${SITL_MAVROS_OUTPUT_ENABLE:-0}"
SITL_DEDICATED_COMMAND_MAVLINK="${SITL_DEDICATED_COMMAND_MAVLINK:-1}"
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
  --wipe-eeprom        Wipe SITL EEPROM before start so param files are authoritative
  --keep-eeprom        Reuse existing SITL EEPROM state
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
  SITL_QGC_OUTPUT_ENABLE Enable QGC MAVLink output to 14550 (default 1)
  SITL_MAVROS_OUTPUT_ENABLE Enable MAVProxy fan-out to external MAVROS 14551 (default 0)
  SITL_DEDICATED_COMMAND_MAVLINK Enable separate serial4 command link (default 1)
  SITL_PARAM_AB_PROFILE Optional parameter A/B profile. Defaults to "real".
                       Supported: real, accz_i_hover, accz_i_soft.

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
UUV_EKF_CONTRACT="${UUV_EKF_CONTRACT:-althold_baro}"
case "$UUV_EKF_CONTRACT" in
  poshold_extnav|poshold_extnav_412|real_param_parity|real-ekf|real_ekf|extnav)
    SITL_EKF3_EXTNAV=1
    ;;
  althold_baro|baro|baro-ekf|depthhold_baro|"")
    UUV_EKF_CONTRACT="althold_baro"
    SITL_EKF3_EXTNAV=0
    ;;
  *)
    echo "[start] unknown UUV_EKF_CONTRACT=$UUV_EKF_CONTRACT" >&2
    exit 2
    ;;
esac
export UUV_EKF_CONTRACT

SITL_EKF3_EXTNAV_ENABLE=0
case "${SITL_EKF3_EXTNAV:-0}" in
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
  # Explicit althold_baro profile: Bar30 vertical position plus JSON IMU only.
  # Do not keep the real dump's GPS/VISO/DVL aiding here; ALT_HOLD parity
  # should not depend on horizontal/depth sources that are absent underwater.
  SITL_DEFAULT_RNGFND1_TYPE=0
  SITL_DEFAULT_SURFACE_DEPTH=-10.0
  SITL_DEFAULT_AHRS_GPS_USE=0
fi

SITL_PARAM_AB_PROFILE="${SITL_PARAM_AB_PROFILE:-real}"
case "$SITL_PARAM_AB_PROFILE" in
  real|none|"")
    SITL_PARAM_AB_PROFILE="real"
    ;;
  accz_i_hover|vertical_i_hover)
    # A/B only: this does not edit the real-robot parameter file.  It checks
    # whether ArduSub 4.1.2 needs a Z-accel hover-bias integrator in SITL
    # because the simulated plant lacks some real static balance effects.
    SITL_FORCE_VERTICAL_CONTROLLER_PARAMS=1
    export SITL_PSC_ACCZ_I="${SITL_PSC_ACCZ_I:-0.10}"
    export SITL_PSC_ACCZ_IMAX="${SITL_PSC_ACCZ_IMAX:-244.0}"
    ;;
  accz_i_soft|vertical_i_soft)
    # Lower-authority version of accz_i_hover for checking whether only a
    # small bias integrator is needed before changing the physics model.
    SITL_FORCE_VERTICAL_CONTROLLER_PARAMS=1
    export SITL_PSC_ACCZ_I="${SITL_PSC_ACCZ_I:-0.03}"
    export SITL_PSC_ACCZ_IMAX="${SITL_PSC_ACCZ_IMAX:-80.0}"
    ;;
  *)
    echo "[start] unknown SITL_PARAM_AB_PROFILE=$SITL_PARAM_AB_PROFILE" >&2
    echo "        supported: real, accz_i_hover, accz_i_soft" >&2
    exit 2
    ;;
esac
export SITL_PARAM_AB_PROFILE
export SITL_FORCE_VERTICAL_CONTROLLER_PARAMS="${SITL_FORCE_VERTICAL_CONTROLLER_PARAMS:-0}"
if [[ "$SITL_PARAM_AB_PROFILE" != "real" ]]; then
  echo "[start] parameter A/B profile: ${SITL_PARAM_AB_PROFILE}" \
       "(PSC_ACCZ_I=${SITL_PSC_ACCZ_I:-unset}," \
       "PSC_ACCZ_IMAX=${SITL_PSC_ACCZ_IMAX:-unset})"
fi

SITL_WIPE_EEPROM_EFFECTIVE="${SITL_WIPE_EEPROM:-}"
if [[ -z "$SITL_WIPE_EEPROM_EFFECTIVE" ]]; then
  case "$UUV_EKF_CONTRACT" in
    real_param_parity|poshold_extnav|poshold_extnav_412|real-ekf|real_ekf|extnav)
      SITL_WIPE_EEPROM_EFFECTIVE=1
      ;;
    *)
      SITL_WIPE_EEPROM_EFFECTIVE=0
      ;;
  esac
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
    --wipe-eeprom)
      SITL_WIPE_EEPROM_EFFECTIVE=1
      shift
      ;;
    --keep-eeprom)
      SITL_WIPE_EEPROM_EFFECTIVE=0
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

ARDUSUB_SITL_BINARY="${ARDUPILOT_DIR}/build/sitl/bin/ardusub"
if [[ "$NO_REBUILD" -eq 1 && ! -x "$ARDUSUB_SITL_BINARY" ]]; then
  echo "[start-sitl] local ArduSub binary missing: ${ARDUSUB_SITL_BINARY}"
  echo "[start-sitl] switching local SITL launch from --no-rebuild to --rebuild"
  NO_REBUILD=0
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

user_arg_present_option() {
  local needle="$1"
  local token
  for token in "${USER_ARGS[@]}"; do
    if [[ "$token" == "$needle" || "$token" == "$needle="* ]]; then
      return 0
    fi
    if [[ "$needle" == "-S" && "$token" == -S* ]]; then
      return 0
    fi
  done
  return 1
}

SIM_ARGS=()
if ! user_arg_present_option "-I"; then
  if [[ "$SITL_INSTANCE" == "auto" ]]; then
    SITL_INSTANCE_EFFECTIVE=""
    for candidate in $(seq 0 9); do
      candidate_port=$((5760 + 10 * candidate))
      if ! (echo > "/dev/tcp/127.0.0.1/${candidate_port}") >/dev/null 2>&1; then
        SITL_INSTANCE_EFFECTIVE="$candidate"
        break
      fi
    done
    if [[ -z "$SITL_INSTANCE_EFFECTIVE" ]]; then
      echo "[error] no free ArduPilot SITL TCP port in 5760..5850" >&2
      exit 1
    fi
  elif [[ "$SITL_INSTANCE" =~ ^[0-9]+$ ]]; then
    SITL_INSTANCE_EFFECTIVE="$SITL_INSTANCE"
  else
    echo "[error] SITL_INSTANCE must be auto or a non-negative integer: $SITL_INSTANCE" >&2
    exit 1
  fi
  SITL_TCP_MAVLINK_PORT_EFFECTIVE=$((5760 + 10 * SITL_INSTANCE_EFFECTIVE))
  SIM_ARGS+=("-I${SITL_INSTANCE_EFFECTIVE}")
  echo "[start-sitl] instance=${SITL_INSTANCE_EFFECTIVE} TCP master=${SITL_TCP_MAVLINK_PORT_EFFECTIVE} (auto avoids occupied VS Code/QGC ports)"
fi
if ! user_arg_present_option "--speedup" && ! user_arg_present_option "-S"; then
  SIM_ARGS+=(--speedup "${SITL_SPEEDUP_DEFAULT:-2}")
  echo "[start-sitl] sim speedup default: ${SITL_SPEEDUP_DEFAULT:-2}x"
fi
case "${SITL_NO_EXTRA_PORTS:-1}" in
  1|true|TRUE|yes|YES|on|ON|disable-default|disabled)
    SIM_ARGS+=(--no-extra-ports)
    echo "[start-sitl] sim_vehicle default MAVProxy ports disabled; using explicit outputs only"
    ;;
esac
case "$SITL_WIPE_EEPROM_EFFECTIVE" in
  1|true|TRUE|yes|YES|on|ON|enable|enabled)
    SIM_ARGS+=(-w)
    echo "[start-sitl] wiping SITL EEPROM so parameter files are authoritative"
    ;;
esac
if [[ "$NO_REBUILD" -eq 1 ]]; then
  SIM_ARGS+=(-N)
fi
if [[ -n "${SITL_CUSTOM_LOCATION:-}" ]]; then
  SIM_ARGS+=(--custom-location "${SITL_CUSTOM_LOCATION}")
  echo "[start-sitl] custom SITL location: ${SITL_CUSTOM_LOCATION}"
fi

USE_REAL_PARAM_FILE=0
case "${SITL_USE_REAL_PARAM_FILE:-1}" in
  1|true|TRUE|yes|YES|on|ON|enable|enabled)
    USE_REAL_PARAM_FILE=1
    ;;
esac
DEFAULT_REAL_PARAM_FILE="${SCRIPT_DIR}/config/ardusub_realrobot_contract.param"
REAL_PARAM_FILE="${SITL_REAL_PARAM_FILE:-$DEFAULT_REAL_PARAM_FILE}"
if [[ "$USE_REAL_PARAM_FILE" -eq 1 && -f "$REAL_PARAM_FILE" ]]; then
  :
elif [[ "$USE_REAL_PARAM_FILE" -eq 1 ]]; then
  echo "[error] missing real vehicle contract param file: ${REAL_PARAM_FILE}" >&2
  echo "        Set SITL_USE_REAL_PARAM_FILE=0 only for isolated SITL debug." >&2
  exit 1
fi

ARDUSUB_FIRMWARE_VERSION="$(
  awk -F'"' '/#define[[:space:]]+THISFIRMWARE/ { print $2; exit }' \
    "${ARDUPILOT_DIR}/ArduSub/version.h" 2>/dev/null || true
)"
SITL_PARAM_COMPAT_FILTER_EFFECTIVE=0
case "${SITL_PARAM_COMPAT_FILTER:-auto}" in
  1|true|TRUE|yes|YES|on|ON|enable|enabled)
    SITL_PARAM_COMPAT_FILTER_EFFECTIVE=1
    ;;
  0|false|FALSE|no|NO|off|OFF|disable|disabled)
    SITL_PARAM_COMPAT_FILTER_EFFECTIVE=0
    ;;
  auto|"")
    if [[ "$ARDUSUB_FIRMWARE_VERSION" != "ArduSub V4.1.2" ]]; then
      SITL_PARAM_COMPAT_FILTER_EFFECTIVE=1
    fi
    ;;
  *)
    echo "[error] unknown SITL_PARAM_COMPAT_FILTER=${SITL_PARAM_COMPAT_FILTER}" >&2
    exit 2
    ;;
esac

SUPPORTED_PARAM_FILE=""
if [[ "$SITL_PARAM_COMPAT_FILTER_EFFECTIVE" -eq 1 ]]; then
  SUPPORTED_PARAM_FILE="$(mktemp "${TMPDIR:-/tmp}/sitl_supported_params_XXXXXX")"
  if (
    cd "$ARDUPILOT_DIR"
    "$MJ311_PYTHON" Tools/autotest/param_metadata/param_parse.py --vehicle ArduSub --format json >/dev/null
  ) && "$MJ311_PYTHON" - "$ARDUPILOT_DIR/apm.pdef.json" >"$SUPPORTED_PARAM_FILE" <<'PY'
import json
import sys

with open(sys.argv[1], "r", encoding="utf-8") as f:
    payload = json.load(f)

names = set()

def walk(node):
    if not isinstance(node, dict):
        return
    for key, value in node.items():
        if isinstance(value, dict) and (
            "Description" in value
            or "DisplayName" in value
            or "User" in value
            or "Values" in value
            or "Range" in value
            or "Units" in value
        ):
            names.add(str(key))
        else:
            walk(value)

walk(payload)
for name in sorted(names):
    print(name)
PY
  then
    echo "[start-sitl] parameter compatibility filter enabled for ${ARDUSUB_FIRMWARE_VERSION:-unknown firmware}"
    echo "[start-sitl] supported parameter list: ${SUPPORTED_PARAM_FILE}"
  else
    echo "[start-sitl] warning: parameter compatibility metadata unavailable; disabling filter" >&2
    SITL_PARAM_COMPAT_FILTER_EFFECTIVE=0
    rm -f "$SUPPORTED_PARAM_FILE"
    SUPPORTED_PARAM_FILE=""
  fi
fi

param_supported_by_firmware() {
  local key="$1"
  if [[ "$SITL_PARAM_COMPAT_FILTER_EFFECTIVE" -ne 1 ]]; then
    return 0
  fi
  # ArduPilot 4.1's param_metadata generator omits the SIM_Baro subgroup even
  # though the JSON SITL binary exposes these parameters (and records them in
  # mav.parm/DataFlash).  Do not let that metadata gap silently leave the
  # default 0.2 m pressure noise in an ALT_HOLD acceptance run.
  case "$key" in
    SIM_BARO_RND|SIM_BARO_DRIFT|SIM_BARO_GLITCH|SIM_BARO_DELAY|SIM_BAR2_RND|SIM_BAR2_DRIFT|SIM_BAR2_GLITCH|SIM_BAR2_DELAY)
      return 0
      ;;
  esac
  grep -Fxq -- "$key" "$SUPPORTED_PARAM_FILE"
}

real_param_file_has() {
  local key="$1"
  [[ "$USE_REAL_PARAM_FILE" -eq 1 && -f "$REAL_PARAM_FILE" ]] || return 1
  awk -v key="$key" '
    /^[[:space:]]*($|#)/ { next }
    $1 == key { found = 1; exit }
    $1 ~ /^[0-9]+$/ && $2 ~ /^[0-9]+$/ && $3 == key { found = 1; exit }
    END { exit found ? 0 : 1 }
  ' "$REAL_PARAM_FILE"
}

is_sim_forced_param() {
  case "$1" in
    # SITL-only safety/runtime contracts. These values describe the desktop
    # simulator process, not the physical vehicle controller tuning.
    ARMING_CHECK|BRD_OPTIONS|BRD_SAFETYENABLE|BRD_SAFETYOPTION|BRD_SAFETY_MASK|SCHED_LOOP_RATE|SERIAL0_BAUD|SERIAL1_PROTOCOL|SERIAL1_BAUD|SERIAL2_PROTOCOL|SERIAL2_BAUD|SYSID_MYGCS|MAV_GCS_SYSID|MAV_GCS_SYSID_HI|RC_OPTIONS|RC_OVERRIDE_TIME|FS_GCS_ENABLE|FS_PILOT_INPUT|FS_PILOT_TIMEOUT|SR0_*|SR1_*|SR2_*)
      return 0
      ;;
    SERIAL3_PROTOCOL|SERIAL3_BAUD|SERIAL4_PROTOCOL|SERIAL4_BAUD|SR3_*|SR4_*|SIM_*|SIM_BAR*|SIM_BATT_*)
      return 0
      ;;
    # Estimator source selection is an explicit run contract
    # (poshold_extnav vs althold_baro). Do not let the real parameter file pin
    # one contract when the launcher requests the other.
    AHRS_EKF_TYPE|AHRS_GPS_USE|EK3_SRC*|EK3_IMU_MASK|GPS_TYPE|GPS_TYPE2|VISO_*|RNGFND1_*)
      return 0
      ;;
    # The MuJoCo bridge does not emulate the physical magnetometer or the
    # Pixhawk's hardware IMU calibration offsets. Forcing these prevents a real
    # hardware calibration file from injecting non-sim sensor bias.
    COMPASS_*)
      case "${SITL_USE_REAL_COMPASS:-0}" in
        1|true|TRUE|yes|YES|on|ON|enable|enabled)
          return 1
          ;;
      esac
      return 0
      ;;
    INS_ACCOFFS_*|INS_ACCSCAL_*|INS_GYROFFS_*)
      return 0
      ;;
    # The real Pixhawk dump enables a multi-IMU hardware stack. The JSON SITL
    # bridge publishes one calibrated IMU sample, so keep EKF3 on that lane and
    # do not let real hardware lane-selection parameters leak into SITL.
    INS_ENABLE_MASK|INS_USE2|INS_USE3|INS_ACC2*|INS_GYR2*)
      return 0
      ;;
    # Live GUI/QGC pilot-input contract. Keep RC mapping and neutral trims
    # deterministic regardless of the hardware parameter dump or AP_RCMapper
    # library defaults.
    FRAME_CONFIG|RCMAP_ROLL|RCMAP_PITCH|RCMAP_THROTTLE|RCMAP_YAW|RCMAP_FORWARD|RCMAP_LATERAL|RC1_DZ|RC2_DZ|RC3_DZ|RC3_MIN|RC3_MAX|RC3_TRIM|RC4_DZ|RC5_DZ|RC6_DZ|THR_DZ|JS_GAIN_DEFAULT|JS_GAIN_MIN|JS_GAIN_MAX|JS_GAIN_STEPS|JS_THR_GAIN)
      return 0
      ;;
    # The physical roll D gain is unstable in the lower-inertia MuJoCo plant.
    # Filter only this gain from the real dump so the explicit SITL default
    # below actually wins; all other hardware attitude tuning stays intact.
    ATC_RAT_RLL_D)
      return 0
      ;;
    # The measured 20 V T200 curve has a wide zero-force PWM band.  The real
    # vehicle's ACCZ P=0.5 never clears it for ordinary ALT_HOLD commands in
    # the lower-inertia simulation, so use a sim-only proportional authority.
    PSC_ACCZ_P)
      return 0
      ;;
    # Keep the real vehicle's P-only vertical controller by default.  For
    # isolated ArduSub-4.1.2 ALT_HOLD A/B tests, allow an explicit opt-in to
    # override these values without editing the real-robot contract parameter
    # file.
    PSC_ACCZ_I|PSC_ACCZ_IMAX|PSC_ACCZ_D|PSC_ACCZ_FLTD|PSC_ACCZ_FLTE|PSC_ACCZ_FLTT)
      case "${SITL_FORCE_VERTICAL_CONTROLLER_PARAMS:-0}" in
        1|true|TRUE|yes|YES|on|ON|enable|enabled)
          return 0
          ;;
      esac
      return 1
      ;;
    *)
      return 1
      ;;
  esac
}

FILTERED_REAL_PARAM_FILE=""
if [[ "$USE_REAL_PARAM_FILE" -eq 1 ]]; then
  FILTERED_REAL_PARAM_FILE="$(mktemp "${TMPDIR:-/tmp}/sitl_real_params_filtered_XXXXXX")"
  while IFS= read -r line || [[ -n "$line" ]]; do
    read -r field1 field2 field3 _rest <<<"$line"
    key="$field1"
    if [[ "$field1" =~ ^[0-9]+$ && "$field2" =~ ^[0-9]+$ && -n "${field3:-}" ]]; then
      key="$field3"
    fi
    if [[ -z "${key:-}" || "$key" == \#* ]]; then
      printf '%s\n' "$line" >> "$FILTERED_REAL_PARAM_FILE"
      continue
    fi
    if is_sim_forced_param "$key"; then
      continue
    fi
    if ! param_supported_by_firmware "$key"; then
      continue
    fi
    printf '%s\n' "$line" >> "$FILTERED_REAL_PARAM_FILE"
  done < "$REAL_PARAM_FILE"
  SIM_ARGS+=(--add-param-file "$FILTERED_REAL_PARAM_FILE")
  echo "[start-sitl] loading real vehicle params: ${REAL_PARAM_FILE}"
  echo "[start-sitl] filtered sim-forced duplicate params via ${FILTERED_REAL_PARAM_FILE}"
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
  if ! param_supported_by_firmware "$key"; then
    echo "[start-sitl] skipping unsupported parameter for ${ARDUSUB_FIRMWARE_VERSION:-this firmware}: ${key}"
    return 0
  fi
  if ! has_param_override "$key"; then
    if real_param_file_has "$key" && ! is_sim_forced_param "$key"; then
      return 0
    fi
    EXTRA_PARAM_LINES+=("${key} ${value}")
  fi
}

# Core frame/output layout (keep deterministic across runs and EEPROM states).
# The MuJoCo model exposes the full 8-thruster 6DOF layout; allow quick
# native validation against alternate ArduSub frames without editing params.
append_param_if_not_overridden "FRAME_CONFIG" "${SITL_FRAME_CONFIG:-2}"
append_param_if_not_overridden "RCMAP_ROLL" "2"
append_param_if_not_overridden "RCMAP_PITCH" "1"
append_param_if_not_overridden "RCMAP_THROTTLE" "3"
append_param_if_not_overridden "RCMAP_YAW" "4"
append_param_if_not_overridden "RCMAP_FORWARD" "5"
append_param_if_not_overridden "RCMAP_LATERAL" "6"
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
case "${SITL_SERIAL0_UDPCLIENT:-0}" in
  1|true|TRUE|yes|YES|on|ON|enable|enabled)
    # Non-blocking bootstrap only for explicit direct-MAVLink experiments. In
    # the default Docker/QGC path, MAVProxy owns QGC fan-out and serial0 remains
    # unused by the live control contract.
    append_param_if_not_overridden "SERIAL0_BAUD" "${SITL_QGC_MAV_BAUD:-921}"
    append_param_if_not_overridden "SR0_EXTRA1" "${SITL_QGC_SR0_EXTRA1:-0}"
    append_param_if_not_overridden "SR0_EXTRA2" "${SITL_QGC_SR0_EXTRA2:-0}"
    append_param_if_not_overridden "SR0_EXTRA3" "${SITL_QGC_SR0_EXTRA3:-0}"
    append_param_if_not_overridden "SR0_EXT_STAT" "${SITL_QGC_SR0_EXT_STAT:-0}"
    append_param_if_not_overridden "SR0_POSITION" "${SITL_QGC_SR0_POSITION:-0}"
    append_param_if_not_overridden "SR0_RAW_SENS" "${SITL_QGC_SR0_RAW_SENS:-0}"
    append_param_if_not_overridden "SR0_RC_CHAN" "${SITL_QGC_SR0_RC_CHAN:-0}"
    ;;
esac
# Only force MAVLink protocol on serial links that this launcher actually
# wires to an endpoint. If a SERIAL*_PROTOCOL remains MAVLink while no
# --serial* transport is provided, ArduSub SITL opens the default TCP port
# (5761/5762) and blocks at "Waiting for connection", which stalls JSON servo
# output during controller-parity replay.
# Direct SITL can still expose an explicit serial1 QGC link for A/B debugging.
# The default Docker/QGC path does not use this block; it uses MAVProxy fan-out
# below because that path reliably emits QGC heartbeats with ArduSub 4.1.2.
case "${SITL_QGC_DIRECT_SERIAL_ENABLE:-0}" in
  1|true|TRUE|yes|YES|on|ON|enable|enabled)
    append_param_if_not_overridden "SERIAL1_PROTOCOL" "2"
    append_param_if_not_overridden "SERIAL1_BAUD" "${SITL_QGC_MAV_BAUD:-921}"
    append_param_if_not_overridden "SR1_EXTRA1" "${SITL_QGC_SR1_EXTRA1:-4}"
    append_param_if_not_overridden "SR1_EXTRA2" "${SITL_QGC_SR1_EXTRA2:-2}"
    append_param_if_not_overridden "SR1_EXTRA3" "${SITL_QGC_SR1_EXTRA3:-2}"
    append_param_if_not_overridden "SR1_EXT_STAT" "${SITL_QGC_SR1_EXT_STAT:-2}"
    append_param_if_not_overridden "SR1_POSITION" "${SITL_QGC_SR1_POSITION:-2}"
    append_param_if_not_overridden "SR1_RAW_SENS" "${SITL_QGC_SR1_RAW_SENS:-2}"
    append_param_if_not_overridden "SR1_PARAMS" "${SITL_QGC_SR1_PARAMS:-2}"
    append_param_if_not_overridden "SR1_RC_CHAN" "${SITL_QGC_SR1_RC_CHAN:-2}"
    ;;
  *)
    append_param_if_not_overridden "SERIAL1_PROTOCOL" "-1"
    ;;
esac
append_param_if_not_overridden "SERIAL2_PROTOCOL" "2"
append_param_if_not_overridden "SERIAL2_BAUD" "${SITL_MUJOCO_MAV_BAUD:-921}"
append_param_if_not_overridden "SR2_EXTRA1" "${SITL_MUJOCO_SR2_EXTRA1:-1}"
append_param_if_not_overridden "SR2_EXTRA2" "${SITL_MUJOCO_SR2_EXTRA2:-0}"
append_param_if_not_overridden "SR2_EXTRA3" "${SITL_MUJOCO_SR2_EXTRA3:-0}"
append_param_if_not_overridden "SR2_EXT_STAT" "${SITL_MUJOCO_SR2_EXT_STAT:-1}"
append_param_if_not_overridden "SR2_POSITION" "${SITL_MUJOCO_SR2_POSITION:-0}"
append_param_if_not_overridden "SR2_RAW_SENS" "${SITL_MUJOCO_SR2_RAW_SENS:-0}"
append_param_if_not_overridden "SR2_PARAMS" "${SITL_MUJOCO_SR2_PARAMS:-0}"
append_param_if_not_overridden "SR2_RC_CHAN" "${SITL_MUJOCO_SR2_RC_CHAN:-2}"
# MAVROS-compatible ROS topics are provided by the in-process bridge instead of
# a separate ArduSub UDP serial in the low-latency QGC contract.
# Keep unused real-vehicle GPS serial ports disabled in SITL direct mode.
append_param_if_not_overridden "SERIAL3_PROTOCOL" "-1"
if [[ "$SITL_DEDICATED_COMMAND_MAVLINK" -eq 1 ]]; then
  # Split command path. Keep arm/mode/RC override off the telemetry/servo
  # listener so pilot input is not blocked by the shared MuJoCo peer discovery.
  EXTRA_PARAM_LINES+=("SERIAL4_PROTOCOL 2")
  # SERIAL4 is not the physical vehicle's low-speed peripheral link here; it
  # is the desktop-only command/RCOU observation UDP link. Keeping the real
  # 38400 baud plus all SR4 streams on this synthetic link can queue MAVLink
  # telemetry for several seconds, making /mavros/rc/out look stale even while
  # JSON servo plant input is current. Keep only the RCOU stream on this link.
  append_param_if_not_overridden "SERIAL4_BAUD" "${SITL_COMMAND_MAV_BAUD:-921}"
  append_param_if_not_overridden "SR4_EXTRA1" "${SITL_COMMAND_SR4_EXTRA1:-0}"
  append_param_if_not_overridden "SR4_EXTRA2" "${SITL_COMMAND_SR4_EXTRA2:-0}"
  append_param_if_not_overridden "SR4_EXTRA3" "${SITL_COMMAND_SR4_EXTRA3:-0}"
  append_param_if_not_overridden "SR4_EXT_STAT" "${SITL_COMMAND_SR4_EXT_STAT:-0}"
  append_param_if_not_overridden "SR4_POSITION" "${SITL_COMMAND_SR4_POSITION:-0}"
  append_param_if_not_overridden "SR4_RAW_SENS" "${SITL_COMMAND_SR4_RAW_SENS:-0}"
  append_param_if_not_overridden "SR4_PARAMS" "${SITL_COMMAND_SR4_PARAMS:-0}"
  append_param_if_not_overridden "SR4_RC_CHAN" "${SITL_COMMAND_SR4_RC_CHAN:-2}"
fi
# real_robot.param has BRD_OPTIONS=1 because the physical Pixhawk watchdog is
# valid hardware behavior. In SITL that same bit enables the SIGALRM watchdog;
# a brief Docker/Mac scheduling stall during JSON startup causes watchdog_rst,
# then ArduSub refuses to arm. Keep this disabled in simulation.
append_param_if_not_overridden "BRD_OPTIONS" "${SITL_BRD_OPTIONS:-0}"
append_param_if_not_overridden "BRD_SAFETYENABLE" "${SITL_BRD_SAFETYENABLE:-0}"
append_param_if_not_overridden "BRD_SAFETYOPTION" "${SITL_BRD_SAFETYOPTION:-0}"
append_param_if_not_overridden "BRD_SAFETY_MASK" "${SITL_BRD_SAFETY_MASK:-0}"
append_param_if_not_overridden "ARMING_CHECK" "${SITL_ARMING_CHECK:-0}"

# SITL scheduler contract:
# The real ArduSub-4.1.2 vehicle dump runs the controller scheduler at 400Hz.
# Controller-parity replay showed that forcing 100Hz changes raw JSON servo
# output substantially, especially heave. Keep the default at the hardware
# value and use SITL_SCHED_LOOP_RATE only for explicit A/B debugging.
append_param_if_not_overridden "SCHED_LOOP_RATE" "${SITL_SCHED_LOOP_RATE:-400}"
# Keep the estimator contract explicit. The default real-param-parity profile
# keeps Bar30 POSZ and fuses VISO/DVL VELZ through body-frame VPD; althold_baro
# is an explicit A/B profile that leaves VELZ un-fused.
append_param_if_not_overridden "RNGFND1_TYPE" "${SITL_RNGFND1_TYPE:-$SITL_DEFAULT_RNGFND1_TYPE}"
append_param_if_not_overridden "RNGFND1_MIN_CM" "5"
append_param_if_not_overridden "RNGFND1_MAX_CM" "3000"
append_param_if_not_overridden "RNGFND1_ORIENT" "25"
# Match the validated depth-hold report values. Units are cm and normalized
# throttle; both participate in AltHold surface limiting and bottom behavior.
append_param_if_not_overridden "SURFACE_DEPTH" "${SITL_SURFACE_DEPTH:-$SITL_DEFAULT_SURFACE_DEPTH}"
append_param_if_not_overridden "SURFACE_MAX_THR" "${SITL_SURFACE_MAX_THR:-0.1}"
# ArduSub accepts RC override/MANUAL_CONTROL only from the configured GCS sysid.
# ArduSub 4.8 uses MAV_GCS_SYSID; 4.1.x used SYSID_MYGCS. Emit both so the
# compatibility filter keeps the key supported by the active firmware.
SITL_GCS_SYSID="${SITL_MAV_GCS_SYSID:-${SITL_SYSID_MYGCS:-${SITL_MAVLINK_SOURCE_SYSID:-255}}}"
append_param_if_not_overridden "MAV_GCS_SYSID" "$SITL_GCS_SYSID"
append_param_if_not_overridden "MAV_GCS_SYSID_HI" "${SITL_MAV_GCS_SYSID_HI:-0}"
append_param_if_not_overridden "SYSID_MYGCS" "$SITL_GCS_SYSID"
append_param_if_not_overridden "FS_GCS_ENABLE" "${SITL_FS_GCS_ENABLE:-0}"
append_param_if_not_overridden "FS_PILOT_INPUT" "${SITL_FS_PILOT_INPUT:-0}"
append_param_if_not_overridden "FS_PILOT_TIMEOUT" "${SITL_FS_PILOT_TIMEOUT:-10.0}"
# Closed-loop pilot-input contract:
# - GUI and rosbag replay use /mavros/rc/override by default.
# - RC3 command neutral is 1500, but ArduSub configures the throttle channel as
#   a 0..1000 range channel.  Its MANUAL/STABILIZE path passes norm_input() to
#   AP_Motors6DOF, whose bidirectional neutral is 0.5.  With RC3_MIN=1100 and
#   RC3_MAX=1900, RC3_TRIM must therefore stay at the range minimum (1100):
#   RC3=1500 -> norm_input()=0.5 -> zero vertical thrust.  ALT_HOLD uses the
#   same range convention and also maps RC3=1500 to a zero climb target.
# - RC5 is forward and RC6 is lateral. ArduSub's Sub defaults also say 5/6, but
#   AP_RCMapper's library defaults are 6/7; force the Sub live-control mapping
#   above so RC override and QGC/manual-control telemetry reach the same axes.
# - MANUAL_CONTROL remains available for QGC-like joystick behavior and is
#   scaled by JS_GAIN/JS_THR_GAIN inside ArduSub.
# - Keep GUI/QGC live-control gain at ArduSub's own default. The hardware
#   dump's JS_GAIN_DEFAULT=0.1 is a replay/parity value; through
#   MANUAL_CONTROL it shrinks a 30% yaw stick to about 12us and makes heave/yaw
#   look delayed even when MAVLink transport is current.
append_param_if_not_overridden "RC_OPTIONS" "${SITL_RC_OPTIONS:-32}"
append_param_if_not_overridden "RC_OVERRIDE_TIME" "${SITL_RC_OVERRIDE_TIME:-3.0}"
append_param_if_not_overridden "RC1_DZ" "${SITL_RC1_DZ:-30}"
append_param_if_not_overridden "RC2_DZ" "${SITL_RC2_DZ:-30}"
append_param_if_not_overridden "THR_DZ" "${SITL_THR_DZ:-100}"
append_param_if_not_overridden "RC3_MIN" "1100"
append_param_if_not_overridden "RC3_MAX" "1900"
append_param_if_not_overridden "RC3_DZ" "${SITL_RC3_DZ:-30}"
append_param_if_not_overridden "RC3_TRIM" "${SITL_RC3_TRIM:-1100}"
append_param_if_not_overridden "JS_GAIN_DEFAULT" "${SITL_JS_GAIN_DEFAULT:-0.5}"
append_param_if_not_overridden "JS_GAIN_MIN" "${SITL_JS_GAIN_MIN:-0.25}"
append_param_if_not_overridden "JS_GAIN_MAX" "${SITL_JS_GAIN_MAX:-1.0}"
append_param_if_not_overridden "JS_GAIN_STEPS" "${SITL_JS_GAIN_STEPS:-1}"
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
# The hardware dump uses 0.020, but that D term is 5.6x ArduSub's default and
# excites a roll-rate limit cycle in the lower-inertia MuJoCo plant when the
# viewer reduces real-time factor.  Keep the hardware file untouched and use a
# simulation-only stable default; SITL_ATC_RAT_RLL_D can still request hardware
# parity for an explicit replay.
append_param_if_not_overridden "ATC_RAT_RLL_D" "${SITL_ATC_RAT_RLL_D:-0.0036}"
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
# Keep the real robot's P-only topology (no I/D), but use a simulation-only P
# that clears the measured 20 V T200 zero-force PWM band without integrator
# windup.  SITL_PSC_ACCZ_P can request hardware parity for explicit replays;
# the real parameter contract remains unchanged at PSC_ACCZ_P=0.5.
append_param_if_not_overridden "PSC_ACCZ_P" "${SITL_PSC_ACCZ_P:-2.50}"
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
# ArduSub's vectored-frame forward/vertical coupling limiter can clamp one
# direction of MANUAL forward thrust when the simulated heave-neutral contract
# is active. Keep live RC symmetric by default; opt back in with
# SITL_MOT_FV_CPLNG_K for coupling A/B tests.
append_param_if_not_overridden "MOT_FV_CPLNG_K" "${SITL_MOT_FV_CPLNG_K:-0.0}"
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
append_param_if_not_overridden "BARO1_GND_PRESS" "${SITL_BARO1_GND_PRESS:-101473.796875}"
append_param_if_not_overridden "BARO2_GND_PRESS" "${SITL_BARO2_GND_PRESS:-101640.0}"
# Keep the controller-side Bar30 selection aligned with the real vehicle
# contract. Controller-parity replay showed BARO_PRIMARY=0 flips the early
# ALT_HOLD vertical output away from the real RCOU trend; use 1 by default and
# leave SITL_BARO_PRIMARY as an explicit backend A/B override.
append_param_if_not_overridden "BARO_PRIMARY" "${SITL_BARO_PRIMARY:-1}"
append_param_if_not_overridden "BARO_SPEC_GRAV" "1.0"
append_param_if_not_overridden "BARO_ALT_OFFSET" "0.0"
# Depth hold in SITL is driven by the simulated water barometer. The ArduPilot
# SITL default adds 0.2 m of baro noise, which can create a false vertical error
# immediately after MuJoCo's initial-depth hold is released.
# Keep a small water-depth sensor dither in SITL. With a perfectly constant
# zero-noise barometer, AP_Baro can under-refresh last_update enough for EKF3
# to over-rely on inertial vertical prediction in ALT_HOLD.
append_param_if_not_overridden "SIM_BARO_RND" "${SITL_SIM_BARO_RND:-0.002}"
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
#
# ArduSub 4.1.2 also treats an exactly-zero accel offset vector as "3D Accel
# calibration needed". Keep a tiny non-zero sentinel so QGC/prearm see the SITL
# IMU as calibrated without reintroducing the real hardware bias.
append_param_if_not_overridden "INS_ACCOFFS_X" "${SITL_INS_ACCOFFS_X:-0.001}"
append_param_if_not_overridden "INS_ACCOFFS_Y" "${SITL_INS_ACCOFFS_Y:-0.001}"
append_param_if_not_overridden "INS_ACCOFFS_Z" "${SITL_INS_ACCOFFS_Z:-0.001}"
append_param_if_not_overridden "INS_ACCSCAL_X" "${SITL_INS_ACCSCAL_X:-1.0}"
append_param_if_not_overridden "INS_ACCSCAL_Y" "${SITL_INS_ACCSCAL_Y:-1.0}"
append_param_if_not_overridden "INS_ACCSCAL_Z" "${SITL_INS_ACCSCAL_Z:-1.0}"
append_param_if_not_overridden "INS_GYROFFS_X" "${SITL_INS_GYROFFS_X:-0.0}"
append_param_if_not_overridden "INS_GYROFFS_Y" "${SITL_INS_GYROFFS_Y:-0.0}"
append_param_if_not_overridden "INS_GYROFFS_Z" "${SITL_INS_GYROFFS_Z:-0.0}"
append_param_if_not_overridden "INS_ENABLE_MASK" "${SITL_INS_ENABLE_MASK:-1}"
append_param_if_not_overridden "INS_USE2" "${SITL_INS_USE2:-0}"
append_param_if_not_overridden "INS_USE3" "${SITL_INS_USE3:-0}"
append_param_if_not_overridden "EK3_ALT_M_NSE" "0.1"
append_param_if_not_overridden "EK3_GBIAS_P_NSE" "0.0005"
append_param_if_not_overridden "EK3_GND_EFF_DZ" "4"
append_param_if_not_overridden "EK3_RNG_USE_HGT" "-1"
append_param_if_not_overridden "EK3_YAW_M_NSE" "0.05236"
# Match the active estimator contract. The real hardware dump has GPS enabled,
# but the althold_baro profile is a Bar30+IMU contract and must not let GPS
# startup transients contaminate AHRS vertical velocity.
if [[ "$UUV_EKF_CONTRACT" == "althold_baro" ]]; then
  SITL_DEFAULT_GPS_TYPE=0
else
  SITL_DEFAULT_GPS_TYPE=1
fi
append_param_if_not_overridden "GPS_TYPE" "${SITL_GPS_TYPE:-$SITL_DEFAULT_GPS_TYPE}"
append_param_if_not_overridden "GPS_TYPE2" "${SITL_GPS_TYPE2:-0}"
if [[ "$SITL_EKF3_EXTNAV_ENABLE" -eq 1 ]]; then
  SITL_DEFAULT_SIM_GPS_TYPE=0
elif [[ "$UUV_EKF_CONTRACT" == "althold_baro" ]]; then
  SITL_DEFAULT_SIM_GPS_TYPE=0
else
  SITL_DEFAULT_SIM_GPS_TYPE=1
fi
append_param_if_not_overridden "SIM_GPS_TYPE" "${SITL_SIM_GPS_TYPE:-$SITL_DEFAULT_SIM_GPS_TYPE}"
append_param_if_not_overridden "SIM_GPS2_TYPE" "0"
append_param_if_not_overridden "AHRS_GPS_USE" "${SITL_AHRS_GPS_USE:-$SITL_DEFAULT_AHRS_GPS_USE}"
append_param_if_not_overridden "GPS_AUTO_CONFIG" "1"
append_param_if_not_overridden "INS_POS1_X" "0.0"
append_param_if_not_overridden "INS_POS1_Y" "0.0"
append_param_if_not_overridden "INS_POS1_Z" "0.0"
# Default to no additional SITL lever arm because the replayed JSON IMU sample
# is normally treated as the simulated IMU measurement point. Keep this
# environment-overridable for controller-parity A/B runs against the real
# INS_POS1_* sensor contract.
append_param_if_not_overridden "SIM_IMU_POS_X" "${SITL_SIM_IMU_POS_X:-0.0}"
append_param_if_not_overridden "SIM_IMU_POS_Y" "${SITL_SIM_IMU_POS_Y:-0.0}"
append_param_if_not_overridden "SIM_IMU_POS_Z" "${SITL_SIM_IMU_POS_Z:-0.0}"
if [[ "$SITL_EKF3_EXTNAV_ENABLE" -eq 1 ]]; then
  # Real-robot-like estimator path. Keep vertical position on Baro/Bar30 while
  # using ExternalNav/DVL for velocity and yaw, matching the 4.1.2 hardware
  # parameter dump used for the robot.
  # The MuJoCo JSON backend currently advertises no_time_sync/no_lockstep.
  # ArduPilot's SIM_JSON contract selects AHRS type 10 for that asynchronous
  # transport; forcing EKF3 makes attitude/velocity diverge under sub-realtime
  # viewer load.  Keep EKF3 available only as an explicit experiment override.
  append_param_if_not_overridden "AHRS_EKF_TYPE" "${SITL_AHRS_EKF_TYPE:-10}"
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
  append_param_if_not_overridden "VISO_DELAY_MS" "${SITL_VISO_DELAY_MS:-100}"
  append_param_if_not_overridden "VISO_POS_X" "0.0"
  append_param_if_not_overridden "VISO_POS_Y" "0.0"
  append_param_if_not_overridden "VISO_POS_Z" "0.0"
  append_param_if_not_overridden "VISO_POS_M_NSE" "${SITL_VISO_POS_M_NSE:-0.2}"
  append_param_if_not_overridden "VISO_VEL_M_NSE" "${SITL_VISO_VEL_M_NSE:-0.1}"
  append_param_if_not_overridden "VISO_YAW_M_NSE" "${SITL_VISO_YAW_M_NSE:-0.050004}"
else
  append_param_if_not_overridden "EK3_SRC1_POSXY" "0"
  append_param_if_not_overridden "EK3_SRC1_VELXY" "0"
  append_param_if_not_overridden "EK3_SRC1_POSZ" "1"
  append_param_if_not_overridden "EK3_SRC1_VELZ" "0"
  append_param_if_not_overridden "EK3_SRC1_YAW" "0"
  append_param_if_not_overridden "EK3_SRC_OPTIONS" "1"
  append_param_if_not_overridden "VISO_TYPE" "0"
  # Async JSON must use ArduPilot's direct simulated AHRS (type 10).  The
  # physical robot parameter file remains EKF3 and is not modified here.
  append_param_if_not_overridden "AHRS_EKF_TYPE" "${SITL_AHRS_EKF_TYPE:-10}"
fi
append_param_if_not_overridden "COMPASS_ENABLE" "0"
append_param_if_not_overridden "COMPASS_USE" "0"
append_param_if_not_overridden "COMPASS_USE2" "0"
append_param_if_not_overridden "COMPASS_USE3" "0"
append_param_if_not_overridden "COMPASS_DEV_ID" "0"
append_param_if_not_overridden "COMPASS_DEV_ID2" "0"
append_param_if_not_overridden "COMPASS_DEV_ID3" "0"
append_param_if_not_overridden "COMPASS_DEV_ID4" "0"
append_param_if_not_overridden "COMPASS_DEV_ID5" "0"
append_param_if_not_overridden "COMPASS_DEV_ID6" "0"
append_param_if_not_overridden "COMPASS_DEV_ID7" "0"
append_param_if_not_overridden "COMPASS_DEV_ID8" "0"
append_param_if_not_overridden "COMPASS_PRIO1_ID" "0"
append_param_if_not_overridden "COMPASS_PRIO2_ID" "0"
append_param_if_not_overridden "COMPASS_PRIO3_ID" "0"
append_param_if_not_overridden "COMPASS_OFS_X" "0"
append_param_if_not_overridden "COMPASS_OFS_Y" "0"
append_param_if_not_overridden "COMPASS_OFS_Z" "0"
append_param_if_not_overridden "COMPASS_OFS2_X" "0"
append_param_if_not_overridden "COMPASS_OFS2_Y" "0"
append_param_if_not_overridden "COMPASS_OFS2_Z" "0"
append_param_if_not_overridden "COMPASS_OFS3_X" "0"
append_param_if_not_overridden "COMPASS_OFS3_Y" "0"
append_param_if_not_overridden "COMPASS_OFS3_Z" "0"
append_param_if_not_overridden "COMPASS_DIA_X" "1"
append_param_if_not_overridden "COMPASS_DIA_Y" "1"
append_param_if_not_overridden "COMPASS_DIA_Z" "1"
append_param_if_not_overridden "COMPASS_DIA2_X" "0"
append_param_if_not_overridden "COMPASS_DIA2_Y" "0"
append_param_if_not_overridden "COMPASS_DIA2_Z" "0"
append_param_if_not_overridden "COMPASS_DIA3_X" "0"
append_param_if_not_overridden "COMPASS_DIA3_Y" "0"
append_param_if_not_overridden "COMPASS_DIA3_Z" "0"
append_param_if_not_overridden "COMPASS_ODI_X" "0"
append_param_if_not_overridden "COMPASS_ODI_Y" "0"
append_param_if_not_overridden "COMPASS_ODI_Z" "0"
append_param_if_not_overridden "COMPASS_ODI2_X" "0"
append_param_if_not_overridden "COMPASS_ODI2_Y" "0"
append_param_if_not_overridden "COMPASS_ODI2_Z" "0"
append_param_if_not_overridden "COMPASS_ODI3_X" "0"
append_param_if_not_overridden "COMPASS_ODI3_Y" "0"
append_param_if_not_overridden "COMPASS_ODI3_Z" "0"
append_param_if_not_overridden "COMPASS_SCALE" "0"
append_param_if_not_overridden "COMPASS_SCALE2" "0"
append_param_if_not_overridden "COMPASS_SCALE3" "0"

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
  echo "  MuJoCo   -> serial2 udpclient:${SITL_MUJOCO_MAV_HOST}:${SITL_MUJOCO_MAV_PORT}"
  SERIAL_ARGS="--sim-address=${SITL_JSON_HOST} --sim-port-in=${SITL_JSON_SENSOR_PORT} --sim-port-out=${SITL_JSON_SERVO_PORT} --serial2=udpclient:${SITL_MUJOCO_MAV_HOST}:${SITL_MUJOCO_MAV_PORT}"
  case "${SITL_SERIAL0_UDPCLIENT:-0}" in
    1|true|TRUE|yes|YES|on|ON|enable|enabled)
      echo "  SERIAL0  -> udpclient:${SITL_CONSOLE_HOST}:${SITL_CONSOLE_PORT} (non-blocking bootstrap)"
      SERIAL_ARGS+=" --serial0=udpclient:${SITL_CONSOLE_HOST}:${SITL_CONSOLE_PORT}"
      ;;
  esac
  case "$SITL_QGC_OUTPUT_ENABLE" in
    1|true|TRUE|yes|YES|on|ON|enable|enabled)
      case "${SITL_QGC_DIRECT_SERIAL_ENABLE:-0}" in
        1|true|TRUE|yes|YES|on|ON|enable|enabled)
          echo "  QGC      -> serial1 udpclient:${SITL_QGC_HOST}:${SITL_QGC_PORT}"
          SERIAL_ARGS+=" --serial1=udpclient:${SITL_QGC_HOST}:${SITL_QGC_PORT}"
          ;;
        *)
          echo "  QGC      -> bridge relay ${SITL_QGC_HOST}:${SITL_QGC_PORT}"
          ;;
      esac
      ;;
    *)
      echo "  QGC      -> disabled"
      ;;
  esac
  case "$SITL_MAVROS_OUTPUT_ENABLE" in
    1|true|TRUE|yes|YES|on|ON|enable|enabled)
    echo "  MAVROS   -> disabled; serial2 is reserved for MuJoCo in this low-latency QGC contract"
      ;;
    *)
      echo "  MAVROS   -> in-process ROS2 bridge"
      ;;
  esac
  if [[ "$SITL_DEDICATED_COMMAND_MAVLINK" -eq 1 ]]; then
    echo "  Command  -> serial4 udpclient:${SITL_COMMAND_MAV_HOST}:${SITL_COMMAND_MAV_PORT}"
    SERIAL_ARGS+=" --serial4=udpclient:${SITL_COMMAND_MAV_HOST}:${SITL_COMMAND_MAV_PORT}"
  else
    echo "  Command  -> serial2 shared MuJoCo MAVLink link"
  fi
  SIM_ARGS+=(--no-mavproxy --udp)
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
