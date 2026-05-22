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

export ROS2_UUV_MAVROS_RC_OVERRIDE_LOCAL_FALLBACK=0
export ROS2_UUV_SITL_JSON_SERVO_FALLBACK=1
export ROS2_UUV_SITL_COMMAND_MAVLINK_ENDPOINT="${ROS2_UUV_SITL_COMMAND_MAVLINK_ENDPOINT:-same}"
export ROS2_UUV_ARM_MODE_BOOT_GUARD_S=0
export SITL_AUTO_SAFE_SEQUENCE=0
export SITL_EKF3_EXTNAV=0
export SITL_AHRS_EKF_TYPE=10
export ROS2_UUV_SITL_EXTNAV_ENABLE=0
export ROS2_UUV_BAR30_NOISE_PA_STD=0.0
export ROS2_UUV_CMD_DEADBAND=0.0
export ROS2_UUV_CMD_SLEW_RATE=200.0
export ROS2_UUV_DVL_LPF_ALPHA=1.0
export UUV_MJ_THRUSTER_DEBUG_CSV="${OUT_DIR}/mujoco_thruster_debug.csv"

echo "[golden] out=${OUT_DIR}"
echo "[golden] axes=${AXES[*]}"

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
  "${SCRIPT_DIR}/start_sitl_mujoco_mj311.sh" "${START_ARGS[@]}" -- \
    --headless \
    --no-qgc-video \
    --tank-549x274x132 \
    --fluid-model current \
    --profile current \
    --initial-depth-m 0.25 \
    --hold-initial-depth-until-release \
    --sitl-mavlink-endpoint "udpin:0.0.0.0:14660" \
    --sitl-mavlink-source-sysid 255 \
    --sitl-mavlink-source-compid 190 \
    --sitl-servo-scale 1.0 \
    > "${OUT_DIR}/start_wrapper.log" 2>&1 &
  START_WRAPPER_PID=$!
  echo "[golden] start wrapper pid=${START_WRAPPER_PID}, log=${OUT_DIR}/start_wrapper.log"
fi

ROS_CONDA_ENV="${ROS_CONDA_ENV:-ros2_h311}"
AXIS_CMD=(
  python3 "${SCRIPT_DIR}/tools/axis_rc_override_check.py"
  --out-dir "${OUT_DIR}/axis_check"
  --mode "$MODE"
  --input-mode rc-override
  --command "$COMMAND"
  --axis-s "$AXIS_S"
  --neutral-s "$NEUTRAL_S"
  --baseline-s "$BASELINE_S"
  --sample-hz "$SAMPLE_HZ"
  --release-initial-depth-hold
  --post-release-neutral-s 0.5
  --axes "${AXES[@]}"
)

if command -v conda >/dev/null 2>&1; then
  conda run -n "$ROS_CONDA_ENV" "${AXIS_CMD[@]}"
else
  "${AXIS_CMD[@]}"
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
