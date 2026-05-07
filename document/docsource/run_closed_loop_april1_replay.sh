#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/../.." && pwd)"
SIM_DIR="${ROOT_DIR}/uuv_mujoco/v2.2"
CALLER_DIR="$(pwd)"

BAG_PATH="${ROOT_DIR}/real_robot_ros_bag/extracted_2026_04_01/bag_2026-04-01_20-08-11/bag_2026-04-01_20-08-11_0.db3"
OUT_DIR="${SCRIPT_DIR}/closed_loop_april1_rc_replay_$(date +%Y%m%d_%H%M%S)"
FLUID_MODEL="current"
PROFILE="current"
SCENE_PATH=""
DURATION_S=""
START_OFFSET_S="0"
ASSUME_RUNNING=0
CHECK_ENV=0
RATE_SCALE="1.0"
MAX_PUBLISH_HZ="50"
MODE="MANUAL"
COMMAND_SOURCE="${COMMAND_SOURCE:-joy-node}"
FORCE_INITIAL_MODE="${FORCE_INITIAL_MODE:-ALT_HOLD}"
INITIAL_MODE_SETTLE_S="${INITIAL_MODE_SETTLE_S:-2.0}"
SKIP_INITIAL_MODE_EVENT="${SKIP_INITIAL_MODE_EVENT:-1}"
ROS_CONDA_ENV="${ROS_CONDA_ENV:-ros2_h311}"
PYTHON_BIN="${PYTHON_BIN:-}"
SITL_SERVO_SCALE="${SITL_SERVO_SCALE:-0.58}"
INITIAL_DEPTH_M="${INITIAL_DEPTH_M:-}"
CALIBRATION_DEPTH_M="${CALIBRATION_DEPTH_M:-}"
DEPTH_SENSOR_BIAS_M="${DEPTH_SENSOR_BIAS_M:-}"
SITL_DEPTH_SENSOR_BIAS_M="${SITL_DEPTH_SENSOR_BIAS_M:-${DEPTH_SENSOR_BIAS_M}}"
ROS_DEPTH_SENSOR_BIAS_M="${ROS_DEPTH_SENSOR_BIAS_M:-}"
HOLD_INITIAL_DEPTH="${HOLD_INITIAL_DEPTH:-0}"
MAVROS_RC_INV_FORWARD="${MAVROS_RC_INV_FORWARD:-0}"
MAVROS_RC_INV_SWAY="${MAVROS_RC_INV_SWAY:-0}"
MAVROS_RC_INV_YAW="${MAVROS_RC_INV_YAW:-1}"
MAVROS_RC_INV_HEAVE="${MAVROS_RC_INV_HEAVE:-1}"
MODE_FROM_BAG="${MODE_FROM_BAG:-1}"

usage() {
  cat <<'USAGE'
Usage: ./run_closed_loop_april1_replay.sh [options]

Options:
  --bag PATH             Real rosbag2 .db3 containing /mavros/rc/override
  --out-dir DIR          Output directory for recorded sim bag and replay events
  --scene PATH           MuJoCo scene path passed to launch_uuv_sim.sh
  --fluid-model NAME     MuJoCo fluid model/profile family (default: current)
  --profile NAME         Simulation profile (default: current)
  --duration-s SEC       Replay only the first SEC seconds after start offset
  --start-offset-s SEC   Start offset inside the real bag (default: 0)
  --rate-scale SCALE     Replay speed multiplier (default: 1.0)
  --max-publish-hz HZ    Throttle RC override publishing (default: 50)
  --mode MODE            Fixed mode for replay when --mode-from-bag=0
                         (default: MANUAL)
  --command-source SRC   Replay source: joy-node or rc-override
                         (default: joy-node, rebuilds /mavros/rc/override
                         from recorded /joy like the real ROS joy node)
  --force-initial-mode MODE
                         Set this mode after arm and before replay/release
                         (default: ALT_HOLD, empty disables)
  --initial-mode-settle-s SEC
                         Neutral settle time after --force-initial-mode
                         (default: 2.0)
  --skip-initial-mode-event 0|1
                         Drop the t=0 bag mode event so pre-replay ALT_HOLD is
                         not immediately overwritten by an initial MANUAL event
                         (default: 1)
  --sitl-servo-scale S   Scale SITL SERVO_OUTPUT_RAW before MuJoCo thrusters
                         (default: 0.58, or SITL_SERVO_SCALE env)
  --initial-depth-m M     Initial MuJoCo base_link depth below water surface.
                         Leave unset to use the MJCF default pose.
  --calibration-depth-m M Start SITL/MuJoCo at this depth, then switch the
                         held vehicle to --initial-depth-m before mode/arm.
                         Useful for ALT_HOLD tests that need surface pressure
                         calibration before starting at depth.
  --depth-sensor-bias-m M SITL-only Bar30/depth bias. Example: -0.23 makes
                         a 0.43 m Bar30 truth read as about 0.20 m inside
                         ArduSub, while ROS /depth/pose remains comparison truth.
  --sitl-depth-sensor-bias-m M
                         Same as --depth-sensor-bias-m.
  --ros-depth-sensor-bias-m M
                         Bias added to ROS /depth and /depth/pose publication
                         for sensor-emulation tests. Leave unset for bag compare.
  --hold-initial-depth 0|1
                         Hold --initial-depth-m until replay begins (default: 0)
  --yaw-invert 0|1       Invert RC yaw before MANUAL_CONTROL (default: 1)
  --heave-invert 0|1     Invert RC heave before MANUAL_CONTROL (default: 1)
  --mode-from-bag 0|1    Replay /mavros/state mode changes from the real bag
                         (default: 1)
  --assume-running       Do not launch/reset SITL+MuJoCo; only record and replay
  --check-env            Only verify ROS2 Python/CLI environment, then exit
  -h, --help             Show this help

This script sets ROS2_UUV_MAVROS_RC_OVERRIDE_LOCAL_FALLBACK=0 before launching
MuJoCo so /mavros/rc/override reaches ArduSub first and motor output comes back
through SERVO_OUTPUT_RAW. ArduPilot files are not modified.

Environment:
  ROS_CONDA_ENV       Conda ROS env to activate when ros2 is not already on PATH
                      (default: ros2_h311)
  ROS_ENV_SETUP       Explicit ROS setup.bash/setup.zsh to source
  PYTHON_BIN          Python executable for replay/compare scripts
  SITL_SERVO_SCALE    Default value for --sitl-servo-scale
  INITIAL_DEPTH_M     Default value for --initial-depth-m
  CALIBRATION_DEPTH_M Default value for --calibration-depth-m
  DEPTH_SENSOR_BIAS_M Default value for --depth-sensor-bias-m
  SITL_DEPTH_SENSOR_BIAS_M
                      Default SITL-only depth bias
  ROS_DEPTH_SENSOR_BIAS_M
                      Default ROS-published depth bias
  HOLD_INITIAL_DEPTH  Default value for --hold-initial-depth
  MAVROS_RC_INV_YAW   Default value for --yaw-invert
  MAVROS_RC_INV_HEAVE Default value for --heave-invert
  MODE_FROM_BAG       Default value for --mode-from-bag
USAGE
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --bag)
      BAG_PATH="$2"
      shift 2
      ;;
    --out-dir)
      OUT_DIR="$2"
      shift 2
      ;;
    --scene)
      SCENE_PATH="$2"
      shift 2
      ;;
    --fluid-model)
      FLUID_MODEL="$2"
      shift 2
      ;;
    --profile)
      PROFILE="$2"
      shift 2
      ;;
    --duration-s)
      DURATION_S="$2"
      shift 2
      ;;
    --start-offset-s)
      START_OFFSET_S="$2"
      shift 2
      ;;
    --rate-scale)
      RATE_SCALE="$2"
      shift 2
      ;;
    --max-publish-hz)
      MAX_PUBLISH_HZ="$2"
      shift 2
      ;;
    --mode)
      MODE="$2"
      shift 2
      ;;
    --command-source)
      COMMAND_SOURCE="$2"
      shift 2
      ;;
    --force-initial-mode)
      FORCE_INITIAL_MODE="$2"
      shift 2
      ;;
    --initial-mode-settle-s)
      INITIAL_MODE_SETTLE_S="$2"
      shift 2
      ;;
    --skip-initial-mode-event)
      SKIP_INITIAL_MODE_EVENT="$2"
      shift 2
      ;;
    --sitl-servo-scale)
      SITL_SERVO_SCALE="$2"
      shift 2
      ;;
    --initial-depth-m)
      INITIAL_DEPTH_M="$2"
      shift 2
      ;;
    --calibration-depth-m)
      CALIBRATION_DEPTH_M="$2"
      shift 2
      ;;
    --depth-sensor-bias-m|--sitl-depth-sensor-bias-m)
      SITL_DEPTH_SENSOR_BIAS_M="$2"
      shift 2
      ;;
    --ros-depth-sensor-bias-m)
      ROS_DEPTH_SENSOR_BIAS_M="$2"
      shift 2
      ;;
    --hold-initial-depth)
      HOLD_INITIAL_DEPTH="$2"
      shift 2
      ;;
    --yaw-invert)
      MAVROS_RC_INV_YAW="$2"
      shift 2
      ;;
    --heave-invert)
      MAVROS_RC_INV_HEAVE="$2"
      shift 2
      ;;
    --mode-from-bag)
      MODE_FROM_BAG="$2"
      shift 2
      ;;
    --assume-running)
      ASSUME_RUNNING=1
      shift
      ;;
    --check-env)
      CHECK_ENV=1
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "[closed-loop] unknown argument: $1"
      usage
      exit 2
      ;;
  esac
done

if [[ "${HOLD_INITIAL_DEPTH}" != "0" && -z "${INITIAL_DEPTH_M}" ]]; then
  echo "[closed-loop] --hold-initial-depth requires --initial-depth-m"
  exit 2
fi
if [[ -n "${CALIBRATION_DEPTH_M}" && -z "${INITIAL_DEPTH_M}" ]]; then
  echo "[closed-loop] --calibration-depth-m requires --initial-depth-m as the target depth"
  exit 2
fi
if [[ -n "${CALIBRATION_DEPTH_M}" && "${HOLD_INITIAL_DEPTH}" == "0" ]]; then
  echo "[closed-loop] --calibration-depth-m requires --hold-initial-depth 1"
  exit 2
fi

source_setup_safely() {
  local setup_file="$1"
  local restore_nounset=0
  if [[ $- == *u* ]]; then
    restore_nounset=1
    set +u
  fi
  # shellcheck disable=SC1090
  source "${setup_file}"
  if [[ "${restore_nounset}" -eq 1 ]]; then
    set -u
  fi
}

activate_ros_environment() {
  if [[ -n "${ROS_ENV_SETUP:-}" ]]; then
    if [[ ! -f "${ROS_ENV_SETUP}" ]]; then
      echo "[closed-loop] ROS_ENV_SETUP not found: ${ROS_ENV_SETUP}"
      exit 1
    fi
    echo "[closed-loop] sourcing ROS setup: ${ROS_ENV_SETUP}"
    source_setup_safely "${ROS_ENV_SETUP}"
  elif ! command -v ros2 >/dev/null 2>&1; then
    local conda_sh="${HOME}/miniconda3/etc/profile.d/conda.sh"
    if [[ -f "${conda_sh}" ]]; then
      # shellcheck disable=SC1090
      source "${conda_sh}"
      echo "[closed-loop] activating conda ROS env: ${ROS_CONDA_ENV}"
      local restore_nounset=0
      if [[ $- == *u* ]]; then
        restore_nounset=1
        set +u
      fi
      conda activate "${ROS_CONDA_ENV}"
      if [[ "${restore_nounset}" -eq 1 ]]; then
        set -u
      fi
    fi
  fi

  if ! command -v ros2 >/dev/null 2>&1; then
    echo "[closed-loop] ros2 command not found. Source ROS first or set ROS_CONDA_ENV/ROS_ENV_SETUP."
    exit 1
  fi
  if [[ -z "${PYTHON_BIN}" ]]; then
    PYTHON_BIN="$(command -v python3 || command -v python || true)"
  fi
  if [[ -z "${PYTHON_BIN}" ]]; then
    echo "[closed-loop] python not found after ROS environment activation"
    exit 1
  fi
  "${PYTHON_BIN}" - <<'PY'
import rclpy
from mavros_msgs.msg import OverrideRCIn
PY
  echo "[closed-loop] ROS2 CLI: $(command -v ros2)"
  echo "[closed-loop] Python: ${PYTHON_BIN}"
}

activate_ros_environment
if [[ "${CHECK_ENV}" -eq 1 ]]; then
  echo "[closed-loop] environment check passed"
  exit 0
fi

case "${BAG_PATH}" in
  /*) ;;
  *) BAG_PATH="${CALLER_DIR}/${BAG_PATH}" ;;
esac
case "${OUT_DIR}" in
  /*) ;;
  *) OUT_DIR="${CALLER_DIR}/${OUT_DIR}" ;;
esac

mkdir -p "${OUT_DIR}"
BAG_RECORD_DIR="${OUT_DIR}/sim_bag"
LAUNCH_LOG="${OUT_DIR}/launcher.log"
REPLAY_LOG="${OUT_DIR}/replay.log"
BAG_RECORD_LOG="${OUT_DIR}/bag_record.log"

if [[ ! -f "${BAG_PATH}" ]]; then
  echo "[closed-loop] bag not found: ${BAG_PATH}"
  exit 1
fi

cleanup() {
  set +e
  if [[ -n "${BAG_PID:-}" ]]; then
    kill -INT "${BAG_PID}" >/dev/null 2>&1 || true
    wait "${BAG_PID}" >/dev/null 2>&1 || true
  fi
  if [[ "${ASSUME_RUNNING}" -eq 0 ]]; then
    "${SIM_DIR}/reset_uuv_sim.sh" >/dev/null 2>&1 || true
  elif [[ -n "${LAUNCHER_PID:-}" ]]; then
    kill -INT "${LAUNCHER_PID}" >/dev/null 2>&1 || true
    wait "${LAUNCHER_PID}" >/dev/null 2>&1 || true
  fi
}
trap cleanup EXIT

if [[ "${ASSUME_RUNNING}" -eq 0 ]]; then
  export ROS2_UUV_MAVROS_RC_OVERRIDE_LOCAL_FALLBACK=0
  export ROS2_UUV_MAVROS_RC_INV_FORWARD="${MAVROS_RC_INV_FORWARD}"
  export ROS2_UUV_MAVROS_RC_INV_SWAY="${MAVROS_RC_INV_SWAY}"
  export ROS2_UUV_MAVROS_RC_INV_YAW="${MAVROS_RC_INV_YAW}"
  export ROS2_UUV_MAVROS_RC_INV_HEAVE="${MAVROS_RC_INV_HEAVE}"
  if [[ -n "${SITL_DEPTH_SENSOR_BIAS_M}" ]]; then
    export ROS2_UUV_SITL_DEPTH_SENSOR_BIAS_M="${SITL_DEPTH_SENSOR_BIAS_M}"
  fi
  if [[ -n "${ROS_DEPTH_SENSOR_BIAS_M}" ]]; then
    export ROS2_UUV_DEPTH_SENSOR_BIAS_M="${ROS_DEPTH_SENSOR_BIAS_M}"
  fi
  cd "${SIM_DIR}"
  LAUNCH_EXTRA_ARGS=(
    --headless
    --no-qgc-video
    --sitl-mavlink-endpoint "udpin:0.0.0.0:14660"
    --sitl-mavlink-source-sysid "255"
    --sitl-mavlink-source-compid "190"
    --sitl-servo-scale "${SITL_SERVO_SCALE}"
    --fluid-model "${FLUID_MODEL}"
    --profile "${PROFILE}"
  )
  if [[ -n "${SCENE_PATH}" ]]; then
    LAUNCH_EXTRA_ARGS+=(--scene "${SCENE_PATH}")
  else
    LAUNCH_EXTRA_ARGS+=(--tank-549x274x132)
  fi
  LAUNCH_INITIAL_DEPTH_M="${INITIAL_DEPTH_M}"
  if [[ -n "${CALIBRATION_DEPTH_M}" ]]; then
    LAUNCH_INITIAL_DEPTH_M="${CALIBRATION_DEPTH_M}"
  fi
  if [[ -n "${LAUNCH_INITIAL_DEPTH_M}" ]]; then
    LAUNCH_EXTRA_ARGS+=(--initial-depth-m "${LAUNCH_INITIAL_DEPTH_M}")
  fi
  if [[ -n "${CALIBRATION_DEPTH_M}" ]]; then
    LAUNCH_EXTRA_ARGS+=(--initial-depth-hold-target-m "${INITIAL_DEPTH_M}")
  fi
  if [[ "${HOLD_INITIAL_DEPTH}" != "0" ]]; then
    LAUNCH_EXTRA_ARGS+=(--hold-initial-depth-until-release)
  fi
  "${SIM_DIR}/start_sitl_mujoco_mj311.sh" \
    --ros2 \
    --sitl-no-rebuild \
    --no-wait-ready \
    -- \
    "${LAUNCH_EXTRA_ARGS[@]}" \
    >"${LAUNCH_LOG}" 2>&1 &
  LAUNCHER_PID=$!

  READY=0
  for _ in $(seq 1 120); do
    if ! kill -0 "${LAUNCHER_PID}" >/dev/null 2>&1; then
      echo "[closed-loop] launcher exited early"
      tail -n 200 "${LAUNCH_LOG}" || true
      exit 1
    fi
    if [[ -f /tmp/ArduSub.log ]]; then
      if grep -Fq "JSON received:" /tmp/ArduSub.log; then
        READY=1
        break
      fi
    fi
    sleep 1
  done
  if [[ "${READY}" -ne 1 ]]; then
    echo "[closed-loop] launcher JSON readiness timeout"
    tail -n 200 "${LAUNCH_LOG}" || true
    exit 1
  fi

  ARDUSUB_READY_LOG="/tmp/ArduSub.log"
  for _ in $(seq 1 50); do
    if [[ -f "${LAUNCH_LOG}" ]]; then
      PARSED_SITL_LOG="$(awk -F'log=' '/SITL bootstrap/{print $2}' "${LAUNCH_LOG}" | tail -n 1 || true)"
      if [[ -n "${PARSED_SITL_LOG}" && -f "${PARSED_SITL_LOG}" ]]; then
        ARDUSUB_READY_LOG="${PARSED_SITL_LOG}"
        break
      fi
    fi
    sleep 0.2
  done
  echo "[closed-loop] waiting for ArduSub readiness in ${ARDUSUB_READY_LOG}"

  AP_READY=0
  for _ in $(seq 1 90); do
    if ! kill -0 "${LAUNCHER_PID}" >/dev/null 2>&1; then
      echo "[closed-loop] launcher exited before ArduSub readiness"
      tail -n 200 "${LAUNCH_LOG}" || true
      exit 1
    fi
    if [[ -f "${ARDUSUB_READY_LOG}" ]] \
      && grep -Fq "ArduPilot Ready" "${ARDUSUB_READY_LOG}" \
      && grep -Fq "Barometer 1 calibration complete" "${ARDUSUB_READY_LOG}" \
      && grep -Fq "Barometer 2 calibration complete" "${ARDUSUB_READY_LOG}"; then
      AP_READY=1
      break
    fi
    sleep 1
  done
  if [[ "${AP_READY}" -ne 1 ]]; then
    echo "[closed-loop] ArduSub readiness timeout"
    tail -n 200 "${ARDUSUB_READY_LOG}" || true
    exit 1
  fi
  sleep 1
else
  echo "[closed-loop] assuming an existing ROS2/SITL/MuJoCo stack is running"
fi

ros2 bag record \
  -o "${BAG_RECORD_DIR}" \
  /measurement/phase \
  /mavros/state \
  /mavros/rc/override \
  /mavros/rc/in \
  /mavros/rc/out \
  /mavros/local_position/odom \
  /mavros/local_position/pose \
  /mavros/local_position/velocity_local \
  /mavros/imu/data \
  /mavros/imu/static_pressure \
  /dvl/twist \
  /dvl/odometry \
  /depth/pose \
  /depth \
  /mujoco/ground_truth/pose \
  /tf \
  >"${BAG_RECORD_LOG}" 2>&1 &
BAG_PID=$!

sleep 2

REPLAY_ARGS=(
  "${SCRIPT_DIR}/replay_april1_rc_override_closed_loop.py"
  --bag "${BAG_PATH}"
  --command-source "${COMMAND_SOURCE}"
  --output-dir "${OUT_DIR}"
  --start-offset-s "${START_OFFSET_S}"
  --rate-scale "${RATE_SCALE}"
  --max-publish-hz "${MAX_PUBLISH_HZ}"
  --mode "${MODE}"
)
if [[ -n "${DURATION_S}" ]]; then
  REPLAY_ARGS+=(--duration-s "${DURATION_S}")
fi
if [[ "${MODE_FROM_BAG}" != "0" ]]; then
  REPLAY_ARGS+=(--mode-from-bag)
fi
if [[ -n "${FORCE_INITIAL_MODE}" ]]; then
  REPLAY_ARGS+=(--force-initial-mode "${FORCE_INITIAL_MODE}" --initial-mode-settle-s "${INITIAL_MODE_SETTLE_S}")
fi
if [[ "${SKIP_INITIAL_MODE_EVENT}" != "0" ]]; then
  REPLAY_ARGS+=(--skip-initial-mode-event)
fi
if [[ -n "${CALIBRATION_DEPTH_M}" ]]; then
  REPLAY_ARGS+=(--switch-sim-initial-depth-hold-to-target)
fi
if [[ "${HOLD_INITIAL_DEPTH}" != "0" ]]; then
  REPLAY_ARGS+=(--release-sim-initial-depth-hold)
fi

"${PYTHON_BIN}" "${REPLAY_ARGS[@]}" >"${REPLAY_LOG}" 2>&1

kill -INT "${BAG_PID}" >/dev/null 2>&1 || true
wait "${BAG_PID}" >/dev/null 2>&1 || true
unset BAG_PID

if "${PYTHON_BIN}" "${SCRIPT_DIR}/compare_closed_loop_april1_replay.py" \
  --real-bag "${BAG_PATH}" \
  --sim-bag "${BAG_RECORD_DIR}" \
  --align-rc-start \
  --real-start-offset-s "${START_OFFSET_S}" \
  --output-dir "${OUT_DIR}/comparison" \
  >"${OUT_DIR}/compare.log" 2>&1; then
  echo "[closed-loop] comparison written in ${OUT_DIR}/comparison"
else
  echo "[closed-loop] warning: comparison step failed; see ${OUT_DIR}/compare.log"
fi

echo "[closed-loop] outputs saved in ${OUT_DIR}"
