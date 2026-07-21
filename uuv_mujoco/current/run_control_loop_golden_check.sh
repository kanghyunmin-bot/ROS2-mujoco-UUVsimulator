#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
OUT_DIR=""
BASELINE=""
BASELINE_THRUSTER_CSV=""
ASSUME_RUNNING=0
DIRECT_MAVLINK=0
START_STACK=1
START_WRAPPER_PID=""
AXES=(roll pitch yaw heave forward lateral)
MODE="ALT_HOLD"
COMMAND="0.25"
AXIS_S="4"
NEUTRAL_S="2"
BASELINE_S="3"
SAMPLE_HZ="25"
PRE_ARM_SETTLE_S="${UUV_PRE_ARM_SETTLE_S:-0}"
POST_ARM_SETTLE_S="${UUV_POST_ARM_SETTLE_S:-2}"
EKF_CONTRACT="${UUV_EKF_CONTRACT:-poshold_extnav}"
INITIAL_DEPTH_M=""
INITIAL_BAR30_DEPTH_M=""

usage() {
  cat <<'USAGE'
Usage: ./run_control_loop_golden_check.sh [options]

Options:
  --assume-running              Do not launch/reset the stack; only run checks.
  --direct-mavlink              Launch SITL direct MAVLink mode.
  --out-dir DIR                 Output directory.
  --baseline axis_summary.json  Compare against an existing axis_summary.json.
  --baseline-thruster-csv CSV   Optional baseline MuJoCo thruster debug CSV.
  --mode MODE                   Flight mode for axis check (default ALT_HOLD).
  --command VALUE               Axis command magnitude, normalized (default 0.25).
  --axis-s SEC                  Active command duration per direction (default 4).
  --neutral-s SEC               Neutral settle duration after each command (default 2).
  --baseline-s SEC              Initial neutral baseline duration (default 3).
  --sample-hz HZ                ROS sample rate for axis check (default 25).
  --pre-arm-settle-s SEC        Neutral settle before arming (default: UUV_PRE_ARM_SETTLE_S or 0).
  --post-arm-settle-s SEC       Neutral settle after arming before mode change (default: UUV_POST_ARM_SETTLE_S or 2).
  --initial-depth-m M           Explicit initial base_link simulated depth in meters (default: unset).
  --initial-bar30-depth-m M     Explicit initial Bar30 sensor depth in meters, or auto (preferred for ALT_HOLD).
  --baro-ekf                    Use the ALT_HOLD/DepthHold contract (EKF3 + Bar30 POSZ, no VISO/VELZ).
  --real-ekf                    Use the POSHOLD/DVL contract (Bar30 POSZ + ExternalNav VELZ/YAW).
  --axes AXIS...                Axes to test. Must be the final option.

This script writes:
  thruster_contract.json
  mujoco_thruster_debug.csv
  axis_check/axis_summary.json
  axis_check/axis_timeseries.csv
  axis_check/axis_response.png
  control_loop_compare.json
USAGE
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --assume-running)
      ASSUME_RUNNING=1
      START_STACK=0
      shift
      ;;
    --direct-mavlink)
      DIRECT_MAVLINK=1
      shift
      ;;
    --out-dir)
      OUT_DIR="$2"
      shift 2
      ;;
    --baseline)
      BASELINE="$2"
      shift 2
      ;;
    --baseline-thruster-csv)
      BASELINE_THRUSTER_CSV="$2"
      shift 2
      ;;
    --mode)
      MODE="$2"
      shift 2
      ;;
    --command)
      COMMAND="$2"
      shift 2
      ;;
    --axis-s)
      AXIS_S="$2"
      shift 2
      ;;
    --neutral-s)
      NEUTRAL_S="$2"
      shift 2
      ;;
    --baseline-s)
      BASELINE_S="$2"
      shift 2
      ;;
    --sample-hz)
      SAMPLE_HZ="$2"
      shift 2
      ;;
    --pre-arm-settle-s)
      PRE_ARM_SETTLE_S="$2"
      shift 2
      ;;
    --post-arm-settle-s)
      POST_ARM_SETTLE_S="$2"
      shift 2
      ;;
    --initial-depth-m)
      INITIAL_DEPTH_M="$2"
      shift 2
      ;;
    --initial-bar30-depth-m)
      INITIAL_BAR30_DEPTH_M="$2"
      shift 2
      ;;
    --baro-ekf)
      EKF_CONTRACT="althold_baro"
      shift
      ;;
    --real-ekf)
      EKF_CONTRACT="poshold_extnav"
      shift
      ;;
    --axes)
      shift
      AXES=("$@")
      break
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "[golden] unknown option: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

if [[ -z "$OUT_DIR" ]]; then
  OUT_DIR="${SCRIPT_DIR}/logs/control_loop_golden_$(date +%Y%m%d_%H%M%S)"
fi
mkdir -p "$OUT_DIR"
OUT_DIR="$(cd "$OUT_DIR" && pwd)"

export ROS2_UUV_MAVROS_RC_OVERRIDE_LOCAL_FALLBACK=0
export ROS2_UUV_SITL_JSON_SERVO_FALLBACK=1
export SITL_DEDICATED_COMMAND_MAVLINK="${SITL_DEDICATED_COMMAND_MAVLINK:-1}"
export SITL_COMMAND_MAV_PORT="${SITL_COMMAND_MAV_PORT:-14661}"
export SITL_TCP_MAVLINK_PORT="${SITL_TCP_MAVLINK_PORT:-5760}"
export ROS2_UUV_SITL_COMMAND_MAVLINK_ENDPOINT="${ROS2_UUV_SITL_COMMAND_MAVLINK_ENDPOINT:-tcp:127.0.0.1:${SITL_TCP_MAVLINK_PORT}}"
export ROS2_UUV_ARM_MODE_BOOT_GUARD_S=0
export SITL_AUTO_SAFE_SEQUENCE=0
case "$EKF_CONTRACT" in
  poshold_extnav|real-ekf|real_ekf|extnav)
    EKF_CONTRACT="poshold_extnav"
    export SITL_EKF3_EXTNAV=1
    export SITL_AHRS_EKF_TYPE=3
    export SITL_EKF3_EXTNAV_POSZ="${SITL_EKF3_EXTNAV_POSZ:-1}"
    export SITL_EKF3_EXTNAV_VELZ="${SITL_EKF3_EXTNAV_VELZ:-6}"
    export ROS2_UUV_SITL_EXTNAV_ENABLE=1
    export ROS2_UUV_REQUIRE_EXTNAV_TX=1
    ;;
  althold_baro|baro|baro-ekf|depthhold_baro|"")
    EKF_CONTRACT="althold_baro"
    export SITL_EKF3_EXTNAV=0
    export SITL_AHRS_EKF_TYPE=3
    export SITL_EKF3_EXTNAV_POSZ="${SITL_EKF3_EXTNAV_POSZ:-1}"
    export SITL_EKF3_EXTNAV_VELZ=0
    export ROS2_UUV_SITL_EXTNAV_ENABLE=0
    export ROS2_UUV_REQUIRE_EXTNAV_TX=0
    ;;
  *)
    echo "[golden] unknown EKF contract: $EKF_CONTRACT" >&2
    exit 2
    ;;
esac
export ROS2_UUV_BAR30_NOISE_PA_STD=0.0
export ROS2_UUV_CMD_DEADBAND=0.0
export ROS2_UUV_CMD_SLEW_RATE=200.0
export ROS2_UUV_DVL_LPF_ALPHA=1.0
export UUV_MJ_THRUSTER_DEBUG_CSV="${OUT_DIR}/mujoco_thruster_debug.csv"

echo "[golden] out=${OUT_DIR}"
echo "[golden] axes=${AXES[*]}"
echo "[golden] ekf_contract=${EKF_CONTRACT}"

PYTHONPATH="$SCRIPT_DIR" python3 "${SCRIPT_DIR}/tools/verify_ardusub_thruster_contract.py" \
  --scene "${SCRIPT_DIR}/scenes/tank_current_scene.xml" \
  --json > "${OUT_DIR}/thruster_contract.json"

cleanup() {
  if [[ "$START_STACK" -eq 1 && "$ASSUME_RUNNING" -eq 0 ]]; then
    if [[ -n "$START_WRAPPER_PID" ]]; then
      kill "$START_WRAPPER_PID" >/dev/null 2>&1 || true
    fi
    "${SCRIPT_DIR}/reset_uuv_sim.sh" >/dev/null 2>&1 || true
  fi
}
trap cleanup EXIT

if [[ "$START_STACK" -eq 1 ]]; then
  START_ARGS=(--ros2 --sitl-no-rebuild)
  if [[ "$DIRECT_MAVLINK" -eq 1 ]]; then
    START_ARGS+=(--direct-mavlink)
  fi
  echo "[golden] starting stack"
  MUJOCO_ARGS=(
    --headless
    --no-qgc-video
    --tank-35x30x11
    --fluid-model current
    --profile current
    --sitl-mavlink-endpoint "udpin:0.0.0.0:14660"
    --sitl-mavlink-source-sysid 255
    --sitl-mavlink-source-compid 190
    --sitl-servo-scale 1.0
  )
  if [[ -n "$INITIAL_BAR30_DEPTH_M" ]]; then
    MUJOCO_ARGS+=(--initial-bar30-depth-m "$INITIAL_BAR30_DEPTH_M")
  elif [[ -n "$INITIAL_DEPTH_M" ]]; then
    MUJOCO_ARGS+=(--initial-depth-m "$INITIAL_DEPTH_M")
  fi
  if [[ -n "${UUV_BUOYANCY_SCALE:-}" ]]; then
    MUJOCO_ARGS+=(--buoyancy-scale "$UUV_BUOYANCY_SCALE")
  fi
  "${SCRIPT_DIR}/start_sitl_mujoco_mj311.sh" "${START_ARGS[@]}" -- \
    "${MUJOCO_ARGS[@]}" \
    > "${OUT_DIR}/start_wrapper.log" 2>&1 &
  START_WRAPPER_PID=$!
  echo "[golden] start wrapper pid=${START_WRAPPER_PID}, log=${OUT_DIR}/start_wrapper.log"
fi

ROS_CONDA_ENV="${ROS_CONDA_ENV:-ros2_h311}"
AXIS_CMD=(
  python "${SCRIPT_DIR}/tools/axis_rc_override_check.py"
  --out-dir "${OUT_DIR}/axis_check"
  --mode "$MODE"
  --input-mode rc-override
  --command "$COMMAND"
  --axis-s "$AXIS_S"
  --neutral-s "$NEUTRAL_S"
  --baseline-s "$BASELINE_S"
  --pre-arm-settle-s "$PRE_ARM_SETTLE_S"
  --post-arm-settle-s "$POST_ARM_SETTLE_S"
  --sample-hz "$SAMPLE_HZ"
  --axes "${AXES[@]}"
)

if command -v conda >/dev/null 2>&1; then
  conda run -n "$ROS_CONDA_ENV" "${AXIS_CMD[@]}"
else
  python3 "${SCRIPT_DIR}/tools/axis_rc_override_check.py" \
    --out-dir "${OUT_DIR}/axis_check" \
    --mode "$MODE" \
    --input-mode rc-override \
    --command "$COMMAND" \
    --axis-s "$AXIS_S" \
    --neutral-s "$NEUTRAL_S" \
    --baseline-s "$BASELINE_S" \
    --pre-arm-settle-s "$PRE_ARM_SETTLE_S" \
    --post-arm-settle-s "$POST_ARM_SETTLE_S" \
    --sample-hz "$SAMPLE_HZ" \
    --axes "${AXES[@]}"
fi

COMPARE_CMD=(
  python3 "${SCRIPT_DIR}/tools/control_loop_golden_compare.py"
  --candidate "${OUT_DIR}/axis_check/axis_summary.json"
  --thruster-csv "${OUT_DIR}/mujoco_thruster_debug.csv"
  --out "${OUT_DIR}/control_loop_compare.json"
)
if [[ -n "$BASELINE" ]]; then
  COMPARE_CMD+=(--baseline "$BASELINE")
fi
if [[ -n "$BASELINE_THRUSTER_CSV" ]]; then
  COMPARE_CMD+=(--baseline-thruster-csv "$BASELINE_THRUSTER_CSV")
fi
"${COMPARE_CMD[@]}"

echo "[golden] done"
echo "[golden] summary=${OUT_DIR}/control_loop_compare.json"
