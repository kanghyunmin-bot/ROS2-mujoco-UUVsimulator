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

usage() {
  cat <<'USAGE'
Usage: ./start_ardusub_sitl_mj311.sh [options] [-- <extra sim_vehicle args>]

Options:
  --param-tune         Start interactive MAVProxy mode for parameter tuning
  --direct-mavlink     Bypass MAVProxy and use direct UDP outputs (experimental)
  --no-ekf-stable      Do not apply default EKF stabilization params
  --no-rebuild         Pass -N to sim_vehicle.py
  --force-no-display   Force non-GUI terminal fallback (unset DISPLAY)
  --allow-display      Keep DISPLAY (standard GUI path)
  -h, --help           Show this help

Environment:
  WORKSPACE_DIR        Workspace root containing ardupilot/ (optional)
  ARDUPILOT_DIR        Explicit ArduPilot checkout path (optional)
  MJ311_ROOT           Preferred Python env root (optional)
  MJ311_PYTHON         Explicit Python interpreter (optional)

Examples:
  ./start_ardusub_sitl_mj311.sh
  ./start_ardusub_sitl_mj311.sh --param-tune
  ./start_ardusub_sitl_mj311.sh --no-rebuild -- --speedup 1
USAGE
}

ENABLE_PARAM_TUNE=0
ENABLE_DIRECT_MAVLINK=0
EKF_STABLE=1
NO_REBUILD=0
FORCE_NO_DISPLAY="${SITL_FORCE_NO_DISPLAY:-1}"
USER_ARGS=()
USER_SET_MAVPROXY_ARGS=0

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
ensure_module gnureadline "gnureadline"
ensure_module PIL "pillow"
ensure_module dronecan "dronecan"

mapping_value() {
  "$MJ311_PYTHON" "$THRUSTER_MAPPING_SCRIPT" "$1"
}

SIM_ARGS=()
if [[ "$NO_REBUILD" -eq 1 ]]; then
  SIM_ARGS+=(-N)
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
# Match the real vehicle's QGC Motor Config reverse setup used by the rosbag.
# ArduSub's 6DOF motor mixer applies these MOT_*_DIRECTION parameters before
# writing SERVO_OUTPUT_RAW. SERVO*_REVERSED is intentionally not used here:
# QGC's Motor Config reverse checkboxes correspond to motor direction, not a
# generic servo-channel reverse in the SITL output path.
append_param_if_not_overridden "MOT_3_DIRECTION" "-1"
append_param_if_not_overridden "MOT_4_DIRECTION" "-1"
append_param_if_not_overridden "MOT_5_DIRECTION" "-1"
append_param_if_not_overridden "MOT_8_DIRECTION" "-1"
append_param_if_not_overridden "SERIAL1_PROTOCOL" "2"
append_param_if_not_overridden "SERIAL2_PROTOCOL" "2"
append_param_if_not_overridden "SERIAL3_PROTOCOL" "2"
# Keep rangefinder disabled in the standard simple-model path. Depth Hold in
# this stack is debugged against one vertical source only: base-link truth ->
# JSON position/velocity/altitude. DVL/rangefinder can be re-enabled later once
# the pressure/EKF path is stable.
append_param_if_not_overridden "RNGFND1_TYPE" "100"
append_param_if_not_overridden "RNGFND1_MIN" "0.05"
append_param_if_not_overridden "RNGFND1_MAX" "30.0"
append_param_if_not_overridden "RNGFND1_ORIENT" "25"
# The vehicle starts submerged, so ArduSub's near-surface throttle clamp
# fights depth hold unless we disable the clamp in SITL.
append_param_if_not_overridden "SURFACE_DEPTH" "100.0"
append_param_if_not_overridden "SURFACE_MAX_THR" "1.0"
# In the sim stack we may drive the vehicle from:
# - external MAVROS (default mavconn source sysid = 1)
# - QGroundControl / other GCS tools (commonly sysid = 255)
# ArduSub only accepts RC override/manual control from MAV_GCS_SYSID[_HI].
# Allow the full 1..255 range in SITL so the sim matches both real-package
# MAVROS flows and standalone GCS joystick/debug flows without patching them.
append_param_if_not_overridden "MAV_GCS_SYSID" "1"
append_param_if_not_overridden "MAV_GCS_SYSID_HI" "255"
# Make QGC joystick center error much less likely to bias ALT_HOLD into a
# sink/climb command during mode transitions. Keep RC3 trim pinned to center
# and widen the neutral band substantially for the simple debug model.
append_param_if_not_overridden "THR_DZ" "400"
append_param_if_not_overridden "RC3_MIN" "1000"
append_param_if_not_overridden "RC3_MAX" "2000"
append_param_if_not_overridden "RC3_DZ" "150"
append_param_if_not_overridden "RC3_TRIM" "1500"
append_param_if_not_overridden "JS_GAIN_DEFAULT" "0.8333333"
append_param_if_not_overridden "PILOT_SPEED_UP" "180"
append_param_if_not_overridden "PILOT_SPEED_DN" "140"
append_param_if_not_overridden "PILOT_ACCEL_Z" "220"
# Make yaw stick enter rate control sooner in assisted modes. ArduSub uses the
# yaw channel deadzone for get_pilot_desired_yaw_rate() in ALT_HOLD/POSHOLD.
append_param_if_not_overridden "RC4_DZ" "10"
# Apply the QGC tuning snapshot provided for the current vehicle. The QGC UI
# shows some legacy names (e.g. PSC_POSZ_P, PSC_VELXY_P, PSC_ACCZ_*), while
# ArduPilot 4.7 stores the current equivalents (PSC_D_POS_P, PSC_NE_VEL_P,
# PSC_D_ACC_*). Converted values are annotated below where scaling changed.
append_param_if_not_overridden "ATC_ANG_RLL_P" "6.0"
append_param_if_not_overridden "ATC_ANG_PIT_P" "6.0"
append_param_if_not_overridden "ATC_ANG_YAW_P" "6.0"
append_param_if_not_overridden "ACRO_YAW_P" "8.00"
append_param_if_not_overridden "ATC_ACC_Y_MAX" "2200.0"
append_param_if_not_overridden "ATC_RATE_Y_MAX" "360.0"
append_param_if_not_overridden "ATC_RAT_RLL_P" "0.14"
append_param_if_not_overridden "ATC_RAT_RLL_I" "0.0"
append_param_if_not_overridden "ATC_RAT_RLL_IMAX" "0.0"
append_param_if_not_overridden "ATC_RAT_RLL_D" "0.01"
append_param_if_not_overridden "ATC_RAT_PIT_P" "0.14"
append_param_if_not_overridden "ATC_RAT_PIT_I" "0.0"
append_param_if_not_overridden "ATC_RAT_PIT_IMAX" "0.0"
append_param_if_not_overridden "ATC_RAT_PIT_D" "0.01"
append_param_if_not_overridden "ATC_RAT_YAW_P" "0.18"
append_param_if_not_overridden "ATC_RAT_YAW_I" "0.0"
append_param_if_not_overridden "ATC_RAT_YAW_IMAX" "0.0"
append_param_if_not_overridden "ATC_RAT_YAW_D" "0.01"
# QGC: PSC_POSXY_P=1.00, PSC_POSZ_P=3.00
append_param_if_not_overridden "PSC_NE_POS_P" "1.0"
append_param_if_not_overridden "PSC_D_POS_P" "3.0"
# QGC: PSC_VELXY_P=6.00, PSC_VELXY_I=0.02, PSC_VELXY_IMAX=1000 cm/s/s
append_param_if_not_overridden "PSC_NE_VEL_P" "6.0"
append_param_if_not_overridden "PSC_NE_VEL_I" "0.02"
append_param_if_not_overridden "PSC_NE_VEL_IMAX" "10.0"
# QGC: PSC_VELZ_P=8.00
append_param_if_not_overridden "PSC_D_VEL_P" "8.0"
# QGC: PSC_ACCZ_P=0.50, PSC_ACCZ_I=0.00, PSC_ACCZ_IMAX=500 d%, PSC_ACCZ_D=0.00
# 4.7 current names scale P/I/D by 0.1 and IMAX by 0.001 from the legacy UI.
append_param_if_not_overridden "PSC_D_ACC_P" "0.05"
append_param_if_not_overridden "PSC_D_ACC_I" "0.0"
append_param_if_not_overridden "PSC_D_ACC_IMAX" "0.5"
append_param_if_not_overridden "PSC_D_ACC_D" "0.0"
append_param_if_not_overridden "PSC_D_ACC_FLTD" "6.94"
append_param_if_not_overridden "PSC_D_ACC_FLTE" "20.0"
append_param_if_not_overridden "PSC_D_ACC_FLTT" "0.0"
# QGC waypoint/loiter snapshot. Current parameter names use SI units.
append_param_if_not_overridden "WP_ACC" "2.5"
append_param_if_not_overridden "WP_ACC_Z" "1.0"
append_param_if_not_overridden "WP_RADIUS_M" "2.0"
append_param_if_not_overridden "WP_SPD" "1.0"
append_param_if_not_overridden "WP_SPD_DN" "1.5"
append_param_if_not_overridden "WP_SPD_UP" "2.5"
append_param_if_not_overridden "LOIT_SPEED_MS" "12.5"
append_param_if_not_overridden "LOIT_ACC_MAX_M" "5.0"
append_param_if_not_overridden "LOIT_ANG_MAX" "0.0"
append_param_if_not_overridden "LOIT_BRK_ACC_M" "2.5"
append_param_if_not_overridden "LOIT_BRK_DELAY" "1.0"
append_param_if_not_overridden "LOIT_BRK_JRK_M" "5.0"
# Battery/pressure parameters from the QGC setup snapshot.
# Keep the calibration-style values, but do not enforce the real-vehicle
# minimum arming voltage in SITL by default. The sim stack does not feed the
# same analog power-module path as the hardware setup page, so copying
# BATT_ARM_VOLT=22.2 directly causes false arm denials in QGC. Users can
# re-enable a strict gate with SITL_BATT_ARM_VOLT=<volts> when needed.
append_param_if_not_overridden "BATT_MONITOR" "4"
append_param_if_not_overridden "BATT_CAPACITY" "5000"
append_param_if_not_overridden "BATT_ARM_VOLT" "${SITL_BATT_ARM_VOLT:-0.0}"
append_param_if_not_overridden "BATT_VOLT_MULT" "10.370"
append_param_if_not_overridden "BATT_AMP_PERVLT" "17.536"
append_param_if_not_overridden "BATT_AMP_OFFSET" "0.0"
append_param_if_not_overridden "BARO1_GND_PRESS" "99126"
append_param_if_not_overridden "BARO2_GND_PRESS" "100510"
append_param_if_not_overridden "BARO_PRIMARY" "1"
append_param_if_not_overridden "BARO_SPEC_GRAV" "1.0"
append_param_if_not_overridden "BARO_ALT_OFFSET" "0.0"
append_param_if_not_overridden "EK3_ALT_M_NSE" "0.1"
append_param_if_not_overridden "EK3_GND_EFF_DZ" "4"
append_param_if_not_overridden "EK3_RNG_USE_HGT" "-1"
# This stack provides underwater depth and bottom-distance sensing directly.
# Leave GPS fully disabled so EKF origin/yaw updates from simulated GNSS do not
# perturb manual -> depth-hold transitions.
append_param_if_not_overridden "GPS1_TYPE" "0"
append_param_if_not_overridden "GPS2_TYPE" "0"
append_param_if_not_overridden "SIM_GPS1_TYPE" "0"
append_param_if_not_overridden "SIM_GPS2_TYPE" "0"
append_param_if_not_overridden "AHRS_GPS_USE" "0"
append_param_if_not_overridden "EK3_SRC1_POSXY" "0"
append_param_if_not_overridden "EK3_SRC1_VELXY" "0"
append_param_if_not_overridden "GPS_AUTO_CONFIG" "0"
# For the simple MuJoCo debug path, consume the JSON attitude/position state
# directly instead of relying on EKF3 + compass fusion.
append_param_if_not_overridden "AHRS_EKF_TYPE" "10"
append_param_if_not_overridden "COMPASS_ENABLE" "0"
append_param_if_not_overridden "COMPASS_USE" "0"
append_param_if_not_overridden "COMPASS_USE2" "0"
append_param_if_not_overridden "COMPASS_USE3" "0"

if [[ "$EKF_STABLE" -eq 1 ]]; then
  # Keep a stable EKF lane setup in SITL.
  append_param_if_not_overridden "EK3_IMU_MASK" "1"
  append_param_if_not_overridden "GPS_AUTO_SWITCH" "0"
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
  echo "  SERIAL0  -> sim_vehicle --udp (non-blocking bootstrap)"
  echo "  QGC      -> serial1 udpclient:127.0.0.1:14550"
  echo "  MAVROS   -> serial2 udpclient:127.0.0.1:14551"
  echo "  MuJoCo   -> serial3 udpclient:127.0.0.1:14660"
  SIM_ARGS+=(--no-mavproxy --udp)
  SITL_CMD=("$MJ311_PYTHON" "$SIM_VEHICLE" \
    -L RATBeach \
    -v ArduSub \
    -f vectored_6dof \
    --model JSON \
    -A "--serial1=udpclient:127.0.0.1:14550" \
    -A "--serial2=udpclient:127.0.0.1:14551" \
    -A "--serial3=udpclient:127.0.0.1:14660")
  if ((${#SIM_ARGS[@]})); then
    SITL_CMD+=("${SIM_ARGS[@]}")
  fi
  if ((${#USER_ARGS[@]})); then
    SITL_CMD+=("${USER_ARGS[@]}")
  fi
  exec "${SITL_CMD[@]}"
fi

echo "[start-sitl] transport mode: legacy MAVProxy fan-out"
if [[ "$USER_SET_MAVPROXY_ARGS" -eq 0 ]]; then
  SITL_MAVPROXY_ARGS="${SITL_MAVPROXY_ARGS:---non-interactive --nowait}"
fi
if [[ "$USER_SET_MAVPROXY_ARGS" -eq 0 && -n "${SITL_MAVPROXY_ARGS:-}" ]]; then
  SIM_ARGS+=(--mavproxy-args "${SITL_MAVPROXY_ARGS}")
fi
# sim_vehicle.py already adds the default GCS output (127.0.0.1:14550).
# Avoid duplicating 14550 to keep QGC link/message flow clean.
SITL_CMD=("$MJ311_PYTHON" "$SIM_VEHICLE" \
  -L RATBeach \
  -v ArduSub \
  -f vectored_6dof \
  --model JSON \
  --out=udp:127.0.0.1:14551 \
  --out=udp:127.0.0.1:14660)
if ((${#SIM_ARGS[@]})); then
  SITL_CMD+=("${SIM_ARGS[@]}")
fi
if ((${#USER_ARGS[@]})); then
  SITL_CMD+=("${USER_ARGS[@]}")
fi
exec "${SITL_CMD[@]}"
