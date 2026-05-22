#!/usr/bin/env bash
set -euo pipefail

TOOL_SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SCRIPT_DIR="$(cd "${TOOL_SCRIPT_DIR}/../.." && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/../.." && pwd)"
SIM_DIR="${ROOT_DIR}/uuv_mujoco/v2.2"
CALLER_DIR="$(pwd)"

BAG_PATH="${ROOT_DIR}/real_robot_ros_bag/extracted_2026_04_01/bag_2026-04-01_20-08-11/bag_2026-04-01_20-08-11_0.db3"
OUT_DIR="${SCRIPT_DIR}/runs/closed_loop/closed_loop_april1_rc_replay_$(date +%Y%m%d_%H%M%S)"
FLUID_MODEL="current"
PROFILE="current"
SCENE_PATH=""
DURATION_S=""
START_OFFSET_S="0"
ASSUME_RUNNING=0
CHECK_ENV=0
DIRECT_MAVLINK=0
RATE_SCALE="1.0"
MAX_PUBLISH_HZ="50"
MODE="MANUAL"
COMMAND_SOURCE="${COMMAND_SOURCE:-rc-override}"
COMMAND_TOPIC="${COMMAND_TOPIC:-}"
FORCE_INITIAL_MODE="${FORCE_INITIAL_MODE:-ALT_HOLD}"
INITIAL_MODE_SETTLE_S="${INITIAL_MODE_SETTLE_S:-0.0}"
POST_RELEASE_NEUTRAL_S="${POST_RELEASE_NEUTRAL_S:-0.0}"
RELEASE_BEFORE_FORCE_INITIAL_MODE="${RELEASE_BEFORE_FORCE_INITIAL_MODE:-0}"
SKIP_INITIAL_MODE_EVENT="${SKIP_INITIAL_MODE_EVENT:-1}"
PRE_KICK_DURATION_S="${PRE_KICK_DURATION_S:-0.0}"
PRE_KICK_FORWARD="${PRE_KICK_FORWARD:-0.0}"
PRE_KICK_LATERAL="${PRE_KICK_LATERAL:-0.0}"
PRE_KICK_HEAVE="${PRE_KICK_HEAVE:-0.0}"
PRE_KICK_YAW="${PRE_KICK_YAW:-0.0}"
POST_PRE_KICK_NEUTRAL_S="${POST_PRE_KICK_NEUTRAL_S:-0.1}"
ROS_CONDA_ENV="${ROS_CONDA_ENV:-ros2_h311}"
PYTHON_BIN="${PYTHON_BIN:-}"
# Legacy polynomial/gain tuned mode used 0.58 here.
# SITL_SERVO_SCALE="${SITL_SERVO_SCALE:-0.58}"
SITL_SERVO_SCALE="${SITL_SERVO_SCALE:-1.0}"
INITIAL_DEPTH_M="${INITIAL_DEPTH_M:-}"
INITIAL_DEPTH_AUTO_REFERENCE="${INITIAL_DEPTH_AUTO_REFERENCE:-bar30}"
INITIAL_RPY_RAD="${INITIAL_RPY_RAD:-}"
INITIAL_STATE_WINDOW_S="${INITIAL_STATE_WINDOW_S:-1.0}"
CALIBRATION_DEPTH_M="${CALIBRATION_DEPTH_M:-}"
RELEASE_LINEAR_VELOCITY_BODY="${RELEASE_LINEAR_VELOCITY_BODY:-}"
DEPTH_SENSOR_BIAS_M="${DEPTH_SENSOR_BIAS_M:-}"
SITL_DEPTH_SENSOR_BIAS_M="${SITL_DEPTH_SENSOR_BIAS_M:-${DEPTH_SENSOR_BIAS_M}}"
ROS_DEPTH_SENSOR_BIAS_M="${ROS_DEPTH_SENSOR_BIAS_M:-}"
ROS_BAR30_DEPTH_SENSOR_BIAS_M="${ROS_BAR30_DEPTH_SENSOR_BIAS_M:-}"
BUOYANCY_SCALE="${BUOYANCY_SCALE:-}"
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
  --command-source SRC   Replay source: rc-override, joy-node, or rc-out
                         (default: rc-override, uses the recorded
                         /mavros/rc/override stream directly; joy-node
                         rebuilds it from recorded /joy; rc-out injects
                         recorded /mavros/rc/out into the MuJoCo plant)
  --command-topic TOPIC  Override command topic for rc-override/rc-out
  --force-initial-mode MODE
                         Set this mode after arm and before replay/release
                         (default: ALT_HOLD, empty disables)
  --initial-mode-settle-s SEC
                         Neutral settle time after --force-initial-mode
                         (default: 0.0; replay starts without letting ALT_HOLD
                         wind up against any artificial initial-depth hold)
  --post-release-neutral-s SEC
                         Neutral settle time after releasing the artificial
                         MuJoCo initial-depth hold and before replay starts
                         (default: 0.0)
  --release-before-force-initial-mode 0|1
                         Release MuJoCo's artificial initial-depth hold before
                         forcing ALT_HOLD. Keep this disabled for depth-matched
                         AltHold replay, otherwise the vehicle can float toward
                         the surface before AltHold captures the target
                         (default: 0)
  --direct-mavlink       Start ArduSub with direct UDP MAVLink outputs, bypassing
                         MAVProxy fan-out for the SITL/MuJoCo closed-loop path
  --skip-initial-mode-event 0|1
                         Drop the t=0 bag mode event so pre-replay ALT_HOLD is
                         not immediately overwritten by an initial MANUAL event
                         (default: 1)
  --sitl-servo-scale S   Scale SITL SERVO_OUTPUT_RAW before MuJoCo thrusters
                         (default: 1.0, or SITL_SERVO_SCALE env;
                         legacy polynomial/gain mode used 0.58)
  --initial-depth-m M     Initial MuJoCo base_link depth below water surface.
                         Leave unset to use the MJCF default pose.
  --initial-depth-auto-reference REF
                         Reference frame for --initial-depth-m auto.
                         "bar30" converts the real /depth/pose sensor depth
                         to base_link depth using the simulated Bar30 site
                         offset; "local_pose_or_bar30" uses real
                         /mavros/local_position/pose.z as base/local depth
                         when available, then falls back to "bar30";
                         "base_link" keeps the old direct value.
  --calibration-depth-m M Start SITL/MuJoCo at this depth, then switch the
                         held vehicle to --initial-depth-m before mode/arm.
                         Useful for ALT_HOLD tests that need surface pressure
                         calibration before starting at depth.
  --initial-rpy-rad "R P Y"
                         Initial MuJoCo base_link roll/pitch/yaw in radians.
                         Use "auto" to estimate from the real bag's initial
                         /mavros/imu/data orientation.
  --initial-state-window-s S
                         Window used for auto initial attitude/velocity
                         estimates (default: 1.0).
  --release-linear-velocity-body "VX VY VZ"
                         Body-frame linear velocity applied when releasing the
                         artificial MuJoCo initial-depth hold. Use this when
                         the real bag starts with nonzero DVL velocity. Use
                         "auto" to estimate from the real bag's initial DVL.
  --depth-sensor-bias-m M SITL-only Bar30/depth bias. Example: -0.23 makes
                         a 0.43 m Bar30 truth read as about 0.20 m inside
                         ArduSub, while ROS /depth/pose remains comparison truth.
  --sitl-depth-sensor-bias-m M
                         Same as --depth-sensor-bias-m.
  --ros-depth-sensor-bias-m M
                         Bias added to ROS /depth, /depth/pose, and Bar30 pressure
                         unless --ros-bar30-depth-sensor-bias-m is set separately.
                         Leave unset for bag compare.
  --ros-bar30-depth-sensor-bias-m M
                         Bias added only to ROS/MAVROS Bar30 pressure publication.
  --buoyancy-scale S      Runtime MuJoCo buoyancy scale override. This does not
                         edit sim_profiles.json and is intended for closed-loop
                         sensitivity checks.
  --hold-initial-depth 0|1
                         Hold --initial-depth-m until replay begins (default: 0)
  --yaw-invert 0|1       Invert RC yaw before MANUAL_CONTROL (default: 1)
  --heave-invert 0|1     Invert RC heave before MANUAL_CONTROL (default: 1)
  --mode-from-bag 0|1    Replay /mavros/state mode changes from the real bag
                         (default: 1)
  --pre-kick-duration-s SEC
                         Apply one fixed RC axis command before replay. This
                         is for bags that start while the real vehicle is
                         already moving.
  --pre-kick-forward V   Forward pre-kick command in [-1, 1]
  --pre-kick-lateral V   Lateral pre-kick command in [-1, 1]
  --pre-kick-heave V     Heave pre-kick command in [-1, 1]
  --pre-kick-yaw V       Yaw pre-kick command in [-1, 1]
  --post-pre-kick-neutral-s SEC
                         Neutral settle after pre-kick before replay
                         (default: 0.1)
  --assume-running       Do not launch/reset SITL+MuJoCo; only record and replay
  --check-env            Only verify ROS2 Python/CLI environment, then exit
  -h, --help             Show this help

This script sets ROS2_UUV_MAVROS_RC_OVERRIDE_LOCAL_FALLBACK=0 before launching
MuJoCo so /mavros/rc/override reaches ArduSub first and motor output comes back
through SERVO_OUTPUT_RAW. ArduPilot files are not modified.
By default it refuses authoritative validation if watched ArduPilot files are
dirty. Set UUV_ALLOW_DIRTY_ARDUPILOT=1 only for smoke tests.

Environment:
  ROS_CONDA_ENV       Conda ROS env to activate when ros2 is not already on PATH
                      (default: ros2_h311)
  ROS_ENV_SETUP       Explicit ROS setup.bash/setup.zsh to source
  PYTHON_BIN          Python executable for replay/compare scripts
  SITL_SERVO_SCALE    Default value for --sitl-servo-scale
  INITIAL_DEPTH_M     Default value for --initial-depth-m
                      Use "auto" to match the real bag's initial depth mean.
  INITIAL_DEPTH_AUTO_REFERENCE
                      Reference for auto initial depth: bar30,
                      local_pose_or_bar30, or base_link (default: bar30)
  CALIBRATION_DEPTH_M Default value for --calibration-depth-m
  RELEASE_LINEAR_VELOCITY_BODY
                      Default value for --release-linear-velocity-body
  DEPTH_SENSOR_BIAS_M Default value for --depth-sensor-bias-m
  SITL_DEPTH_SENSOR_BIAS_M
                      Default SITL-only depth bias
  ROS_DEPTH_SENSOR_BIAS_M
                      Default ROS-published depth bias
  ROS_BAR30_DEPTH_SENSOR_BIAS_M
                      Default ROS/MAVROS Bar30 pressure bias
  BUOYANCY_SCALE      Default value for --buoyancy-scale
  HOLD_INITIAL_DEPTH  Default value for --hold-initial-depth
  MAVROS_RC_INV_YAW   Default value for --yaw-invert
  MAVROS_RC_INV_HEAVE Default value for --heave-invert
  MODE_FROM_BAG       Default value for --mode-from-bag
  EXPECTED_REAL_MODE  Expected majority /mavros/state mode in the selected
                      real-bag replay window. Defaults to --force-initial-mode
                      when set, otherwise to --mode when --mode-from-bag=0.
  STRICT_REAL_MODE_MATCH
                      Set to 1 to fail before replay if EXPECTED_REAL_MODE does
                      not match the selected real-bag window majority mode.
  UUV_ALLOW_DIRTY_ARDUPILOT
                      Set to 1 to continue despite dirty watched ArduPilot files
                      for smoke/debug only. Authoritative metrics remain invalid.
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
    --command-topic)
      COMMAND_TOPIC="$2"
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
    --post-release-neutral-s)
      POST_RELEASE_NEUTRAL_S="$2"
      shift 2
      ;;
    --release-before-force-initial-mode)
      RELEASE_BEFORE_FORCE_INITIAL_MODE="$2"
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
    --initial-depth-auto-reference)
      INITIAL_DEPTH_AUTO_REFERENCE="$2"
      shift 2
      ;;
    --calibration-depth-m)
      CALIBRATION_DEPTH_M="$2"
      shift 2
      ;;
    --initial-rpy-rad)
      INITIAL_RPY_RAD="$2"
      shift 2
      ;;
    --initial-state-window-s)
      INITIAL_STATE_WINDOW_S="$2"
      shift 2
      ;;
    --release-linear-velocity-body)
      RELEASE_LINEAR_VELOCITY_BODY="$2"
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
    --ros-bar30-depth-sensor-bias-m)
      ROS_BAR30_DEPTH_SENSOR_BIAS_M="$2"
      shift 2
      ;;
    --buoyancy-scale)
      BUOYANCY_SCALE="$2"
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
    --pre-kick-duration-s)
      PRE_KICK_DURATION_S="$2"
      shift 2
      ;;
    --pre-kick-forward)
      PRE_KICK_FORWARD="$2"
      shift 2
      ;;
    --pre-kick-lateral)
      PRE_KICK_LATERAL="$2"
      shift 2
      ;;
    --pre-kick-heave)
      PRE_KICK_HEAVE="$2"
      shift 2
      ;;
    --pre-kick-yaw)
      PRE_KICK_YAW="$2"
      shift 2
      ;;
    --post-pre-kick-neutral-s)
      POST_PRE_KICK_NEUTRAL_S="$2"
      shift 2
      ;;
    --assume-running)
      ASSUME_RUNNING=1
      shift
      ;;
    --direct-mavlink)
      DIRECT_MAVLINK=1
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
  echo "[closed-loop] sample real-robot bags are not bundled with dist2; pass --bag /path/to/bag_0.db3 on a new machine."
  exit 1
fi

"${PYTHON_BIN}" "${SIM_DIR}/tools/audit_closed_loop_contract.py" \
  --workspace "${ROOT_DIR}" \
  --profile "${PROFILE}" \
  --json-out "${OUT_DIR}/closed_loop_contract.json"

PREFLIGHT_ARGS=(
  "${SIM_DIR}/tools/preflight_ardupilot_integrity.py"
  --workspace "${ROOT_DIR}"
  --json-out "${OUT_DIR}/ardupilot_preflight.json"
)
if [[ "${UUV_ALLOW_DIRTY_ARDUPILOT:-0}" == "1" ]]; then
  PREFLIGHT_ARGS+=(--allow-dirty)
fi
"${PYTHON_BIN}" "${PREFLIGHT_ARGS[@]}"

MODE_AUDIT_EXPECTED="${EXPECTED_REAL_MODE:-}"
if [[ -z "${MODE_AUDIT_EXPECTED}" && -n "${FORCE_INITIAL_MODE}" ]]; then
  MODE_AUDIT_EXPECTED="${FORCE_INITIAL_MODE}"
elif [[ -z "${MODE_AUDIT_EXPECTED}" && "${MODE_FROM_BAG}" == "0" ]]; then
  MODE_AUDIT_EXPECTED="${MODE}"
fi
MODE_AUDIT_ARGS=(
  "${TOOL_SCRIPT_DIR}/audit_replay_mode_window.py"
  --bag "${BAG_PATH}"
  --start-offset-s "${START_OFFSET_S}"
  --expected-mode "${MODE_AUDIT_EXPECTED}"
  --json-out "${OUT_DIR}/real_window_mode_audit.json"
)
if [[ -n "${DURATION_S}" ]]; then
  MODE_AUDIT_ARGS+=(--duration-s "${DURATION_S}")
fi
if [[ "${STRICT_REAL_MODE_MATCH:-0}" == "1" ]]; then
  MODE_AUDIT_ARGS+=(--strict)
fi
"${PYTHON_BIN}" "${MODE_AUDIT_ARGS[@]}"

if [[ "${INITIAL_DEPTH_M}" == "auto" ]]; then
  INITIAL_DEPTH_WINDOW_S="${INITIAL_DEPTH_WINDOW_S:-5.0}"
  case "${INITIAL_DEPTH_AUTO_REFERENCE}" in
    local_pose|local_pose_or_bar30)
      if LOCAL_INITIAL_DEPTH_M="$("${PYTHON_BIN}" "${TOOL_SCRIPT_DIR}/estimate_initial_depth_from_bag.py" \
          --bag "${BAG_PATH}" \
          --start-offset-s "${START_OFFSET_S}" \
          --window-s "${INITIAL_DEPTH_WINDOW_S}" \
          --source local_pose \
          --precision 4 2>/dev/null)"; then
        INITIAL_DEPTH_M="${LOCAL_INITIAL_DEPTH_M}"
        echo "[closed-loop] auto initial depth from real bag: local_pose base_link=${INITIAL_DEPTH_M} m over ${INITIAL_DEPTH_WINDOW_S}s"
      else
        if [[ "${INITIAL_DEPTH_AUTO_REFERENCE}" == "local_pose" ]]; then
          echo "[closed-loop] failed to read local pose for --initial-depth-m auto"
          exit 1
        fi
        AUTO_INITIAL_DEPTH_M="$("${PYTHON_BIN}" "${TOOL_SCRIPT_DIR}/estimate_initial_depth_from_bag.py" \
          --bag "${BAG_PATH}" \
          --start-offset-s "${START_OFFSET_S}" \
          --window-s "${INITIAL_DEPTH_WINDOW_S}" \
          --source depth \
          --precision 4)"
        INITIAL_DEPTH_M="$(PYTHONPATH="${SIM_DIR}${PYTHONPATH:+:${PYTHONPATH}}" "${PYTHON_BIN}" - "${AUTO_INITIAL_DEPTH_M}" <<'PY'
import sys

from physics.thruster_mapping import SENSOR_SITES_FLU

sensor_depth_m = float(sys.argv[1])
bar30_z_up_m = float(SENSOR_SITES_FLU["bar30_site"][2])
base_link_depth_m = max(0.0, sensor_depth_m + bar30_z_up_m)
print(f"{base_link_depth_m:.4f}")
PY
)"
        echo "[closed-loop] auto initial depth from real bag: local_pose unavailable, bar30=${AUTO_INITIAL_DEPTH_M} m -> base_link=${INITIAL_DEPTH_M} m over ${INITIAL_DEPTH_WINDOW_S}s"
      fi
      ;;
    bar30)
      AUTO_INITIAL_DEPTH_M="$("${PYTHON_BIN}" "${TOOL_SCRIPT_DIR}/estimate_initial_depth_from_bag.py" \
        --bag "${BAG_PATH}" \
        --start-offset-s "${START_OFFSET_S}" \
        --window-s "${INITIAL_DEPTH_WINDOW_S}" \
        --source depth \
        --precision 4)"
      INITIAL_DEPTH_M="$(PYTHONPATH="${SIM_DIR}${PYTHONPATH:+:${PYTHONPATH}}" "${PYTHON_BIN}" - "${AUTO_INITIAL_DEPTH_M}" <<'PY'
import sys

from physics.thruster_mapping import SENSOR_SITES_FLU

sensor_depth_m = float(sys.argv[1])
bar30_z_up_m = float(SENSOR_SITES_FLU["bar30_site"][2])
base_link_depth_m = max(0.0, sensor_depth_m + bar30_z_up_m)
print(f"{base_link_depth_m:.4f}")
PY
)"
      echo "[closed-loop] auto initial depth from real bag: bar30=${AUTO_INITIAL_DEPTH_M} m -> base_link=${INITIAL_DEPTH_M} m over ${INITIAL_DEPTH_WINDOW_S}s"
      ;;
    base_link)
      AUTO_INITIAL_DEPTH_M="$("${PYTHON_BIN}" "${TOOL_SCRIPT_DIR}/estimate_initial_depth_from_bag.py" \
        --bag "${BAG_PATH}" \
        --start-offset-s "${START_OFFSET_S}" \
        --window-s "${INITIAL_DEPTH_WINDOW_S}" \
        --source depth \
        --precision 4)"
      INITIAL_DEPTH_M="${AUTO_INITIAL_DEPTH_M}"
      echo "[closed-loop] auto initial depth from real bag: base_link=${INITIAL_DEPTH_M} m over ${INITIAL_DEPTH_WINDOW_S}s"
      ;;
    *)
      echo "[closed-loop] invalid --initial-depth-auto-reference: ${INITIAL_DEPTH_AUTO_REFERENCE} (expected local_pose_or_bar30, local_pose, bar30, or base_link)"
      exit 1
      ;;
  esac
fi

if [[ "${INITIAL_RPY_RAD}" == "auto" ]]; then
  if INITIAL_RPY_RAD="$("${PYTHON_BIN}" "${TOOL_SCRIPT_DIR}/estimate_initial_state_from_bag.py" \
    --bag "${BAG_PATH}" \
    --start-offset-s "${START_OFFSET_S}" \
    --window-s "${INITIAL_STATE_WINDOW_S}" \
    --field rpy \
    --precision 6)"; then
    echo "[closed-loop] auto initial attitude from real bag: rpy_rad=${INITIAL_RPY_RAD} over ${INITIAL_STATE_WINDOW_S}s"
  else
    echo "[closed-loop] warning: auto initial attitude unavailable; starting level"
    INITIAL_RPY_RAD=""
  fi
fi

if [[ "${RELEASE_LINEAR_VELOCITY_BODY}" == "auto" ]]; then
  if RELEASE_LINEAR_VELOCITY_BODY="$("${PYTHON_BIN}" "${TOOL_SCRIPT_DIR}/estimate_initial_state_from_bag.py" \
    --bag "${BAG_PATH}" \
    --start-offset-s "${START_OFFSET_S}" \
    --window-s "${INITIAL_STATE_WINDOW_S}" \
    --field body-velocity \
    --precision 6)"; then
    echo "[closed-loop] auto release body velocity from real bag: ${RELEASE_LINEAR_VELOCITY_BODY} m/s over ${INITIAL_STATE_WINDOW_S}s"
  else
    echo "[closed-loop] warning: auto release body velocity unavailable; releasing from rest"
    RELEASE_LINEAR_VELOCITY_BODY=""
  fi
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
  export SITL_USE_REAL_PARAM_FILE="${SITL_USE_REAL_PARAM_FILE:-1}"
  export SITL_EKF3_EXTNAV="${SITL_EKF3_EXTNAV:-1}"
  export SITL_AHRS_EKF_TYPE="${SITL_AHRS_EKF_TYPE:-3}"
  export ROS2_UUV_SITL_EXTNAV_ENABLE="${ROS2_UUV_SITL_EXTNAV_ENABLE:-1}"
  unset ROS2_UUV_SITL_EXTNAV_Z_UP
  unset ROS2_UUV_SITL_EXTNAV_VELZ_SCALE
  unset ROS2_UUV_SITL_VERTICAL_SOURCE
  export SITL_RNGFND1_TYPE="${SITL_RNGFND1_TYPE:-0}"
  export ROS2_UUV_SITL_JSON_SERVO_FALLBACK="${ROS2_UUV_SITL_JSON_SERVO_FALLBACK:-1}"
  export ROS2_UUV_MAVROS_RC_OVERRIDE_LOCAL_FALLBACK=0
  export ROS2_UUV_SITL_ALLOW_DIRECT_CMD="${ROS2_UUV_SITL_ALLOW_DIRECT_CMD:-0}"
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
  if [[ -n "${ROS_BAR30_DEPTH_SENSOR_BIAS_M}" ]]; then
    export ROS2_UUV_BAR30_DEPTH_SENSOR_BIAS_M="${ROS_BAR30_DEPTH_SENSOR_BIAS_M}"
  fi
  if [[ -z "${UUV_MJ_THRUSTER_DEBUG_CSV:-}" && -n "${LOG_DIR:-}" ]]; then
    DEBUG_LOG_DIR="${LOG_DIR}"
    case "${DEBUG_LOG_DIR}" in
      /*) ;;
      *) DEBUG_LOG_DIR="${CALLER_DIR}/${DEBUG_LOG_DIR}" ;;
    esac
    export UUV_MJ_THRUSTER_DEBUG_CSV="${DEBUG_LOG_DIR}/mujoco_thruster_debug.csv"
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
  if [[ -n "${INITIAL_RPY_RAD}" ]]; then
    read -r INITIAL_ROLL INITIAL_PITCH INITIAL_YAW <<<"${INITIAL_RPY_RAD}"
    if [[ -z "${INITIAL_ROLL:-}" || -z "${INITIAL_PITCH:-}" || -z "${INITIAL_YAW:-}" ]]; then
      echo "[closed-loop] --initial-rpy-rad expects three numbers: ROLL PITCH YAW"
      exit 1
    fi
    LAUNCH_EXTRA_ARGS+=(--initial-rpy-rad "${INITIAL_ROLL}" "${INITIAL_PITCH}" "${INITIAL_YAW}")
  fi
  if [[ -n "${BUOYANCY_SCALE}" ]]; then
    LAUNCH_EXTRA_ARGS+=(--buoyancy-scale "${BUOYANCY_SCALE}")
  fi
  if [[ -n "${CALIBRATION_DEPTH_M}" ]]; then
    LAUNCH_EXTRA_ARGS+=(--initial-depth-hold-target-m "${INITIAL_DEPTH_M}")
  fi
  if [[ "${HOLD_INITIAL_DEPTH}" != "0" ]]; then
    LAUNCH_EXTRA_ARGS+=(--hold-initial-depth-until-release)
  fi
  if [[ -n "${RELEASE_LINEAR_VELOCITY_BODY}" ]]; then
    read -r RELEASE_VX RELEASE_VY RELEASE_VZ <<<"${RELEASE_LINEAR_VELOCITY_BODY}"
    if [[ -z "${RELEASE_VX:-}" || -z "${RELEASE_VY:-}" || -z "${RELEASE_VZ:-}" ]]; then
      echo "[closed-loop] --release-linear-velocity-body expects three numbers: VX VY VZ"
      exit 1
    fi
    LAUNCH_EXTRA_ARGS+=(--release-linear-velocity-body "${RELEASE_VX}" "${RELEASE_VY}" "${RELEASE_VZ}")
  fi
  START_ARGS=(--ros2 --sitl-no-rebuild --no-wait-ready)
  if [[ "${DIRECT_MAVLINK}" -eq 1 ]]; then
    START_ARGS+=(--direct-mavlink)
  fi
  "${SIM_DIR}/start_sitl_mujoco_mj311.sh" \
    "${START_ARGS[@]}" \
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
  MUJOCO_READY_LOG=""
  refresh_ready_logs() {
    if [[ -f "${LAUNCH_LOG}" ]]; then
      PARSED_SITL_LOG="$(awk -F'log=' '/SITL bootstrap/{print $2}' "${LAUNCH_LOG}" | tail -n 1 || true)"
      PARSED_MUJOCO_LOG="$(awk -F'log=' '/MuJoCo pid=/{print $2}' "${LAUNCH_LOG}" | tail -n 1 || true)"
      if [[ "${DIRECT_MAVLINK}" -eq 0 && -n "${PARSED_SITL_LOG}" && -f "${PARSED_SITL_LOG}" ]]; then
        ARDUSUB_READY_LOG="${PARSED_SITL_LOG}"
      fi
      if [[ -n "${PARSED_MUJOCO_LOG}" && -f "${PARSED_MUJOCO_LOG}" ]]; then
        MUJOCO_READY_LOG="${PARSED_MUJOCO_LOG}"
      fi
    fi
  }

  for _ in $(seq 1 150); do
    refresh_ready_logs
    if [[ -f "${LAUNCH_LOG}" ]]; then
      if [[ -f "${ARDUSUB_READY_LOG}" && ( "${DIRECT_MAVLINK}" -eq 0 || -n "${MUJOCO_READY_LOG}" ) ]]; then
        break
      fi
    fi
    sleep 0.2
  done
  echo "[closed-loop] waiting for ArduSub readiness in ${ARDUSUB_READY_LOG}"

  AP_READY=0
  for _ in $(seq 1 90); do
    refresh_ready_logs
    if ! kill -0 "${LAUNCHER_PID}" >/dev/null 2>&1; then
      echo "[closed-loop] launcher exited before ArduSub readiness"
      tail -n 200 "${LAUNCH_LOG}" || true
      exit 1
    fi
    if [[ "${DIRECT_MAVLINK}" -eq 1 ]]; then
      if [[ -f /tmp/ArduSub.log && -n "${MUJOCO_READY_LOG}" && -f "${MUJOCO_READY_LOG}" ]] \
        && grep -Fq "JSON received:" /tmp/ArduSub.log \
        && grep -Fq "UDP connection 127.0.0.1:14550" /tmp/ArduSub.log \
        && grep -Fq "UDP connection 127.0.0.1:14660" /tmp/ArduSub.log \
        && grep -Fq "SITL servo endpoint discovered" "${MUJOCO_READY_LOG}"; then
        AP_READY=1
        break
      fi
    else
      if [[ -f "${ARDUSUB_READY_LOG}" ]] \
        && grep -Fq "ArduPilot Ready" "${ARDUSUB_READY_LOG}" \
        && grep -Fq "Barometer 1 calibration complete" "${ARDUSUB_READY_LOG}" \
        && grep -Fq "Barometer 2 calibration complete" "${ARDUSUB_READY_LOG}"; then
        AP_READY=1
        break
      fi
    fi
    sleep 1
  done
  if [[ "${AP_READY}" -ne 1 ]]; then
    echo "[closed-loop] ArduSub readiness timeout"
    tail -n 200 "${ARDUSUB_READY_LOG}" || true
    if [[ "${ARDUSUB_READY_LOG}" != "/tmp/ArduSub.log" && -f /tmp/ArduSub.log ]]; then
      echo "[closed-loop] raw ArduSub log:"
      tail -n 120 /tmp/ArduSub.log || true
    fi
    exit 1
  fi
  if [[ -n "${PARSED_SITL_LOG:-}" && -f "${PARSED_SITL_LOG}" ]]; then
    "${PYTHON_BIN}" "${SIM_DIR}/tools/audit_closed_loop_contract.py" \
      --workspace "${ROOT_DIR}" \
      --profile "${PROFILE}" \
      --sitl-log "${PARSED_SITL_LOG}" \
      --json-out "${OUT_DIR}/closed_loop_contract.json"
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
  /mavros/imu/atm_pressure \
  /mavros/vfr_hud \
  /dvl/twist \
  /dvl/odometry \
  /depth/pose \
  /depth \
  /bar30/pressure_pa \
  /mujoco/ground_truth/pose \
  /tf \
  >"${BAG_RECORD_LOG}" 2>&1 &
BAG_PID=$!

sleep 2

if [[ -z "${COMMAND_TOPIC}" ]]; then
  case "${COMMAND_SOURCE}" in
    rc-out)
      COMMAND_TOPIC="/mavros/rc/out"
      ;;
    *)
      COMMAND_TOPIC="/mavros/rc/override"
      ;;
  esac
fi

REPLAY_ARGS=(
  "${SCRIPT_DIR}/replay_april1_rc_override_closed_loop.py"
  --bag "${BAG_PATH}"
  --command-source "${COMMAND_SOURCE}"
  --topic "${COMMAND_TOPIC}"
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
if [[ "${HOLD_INITIAL_DEPTH}" != "0" ]]; then
  REPLAY_ARGS+=(--release-sim-initial-depth-hold --post-release-neutral-s "${POST_RELEASE_NEUTRAL_S}")
  if [[ "${RELEASE_BEFORE_FORCE_INITIAL_MODE}" != "0" ]]; then
    REPLAY_ARGS+=(--release-before-force-initial-mode)
  fi
fi
if awk "BEGIN {exit !(${PRE_KICK_DURATION_S} > 0.0)}"; then
  REPLAY_ARGS+=(
    --pre-kick-duration-s "${PRE_KICK_DURATION_S}"
    --pre-kick-forward "${PRE_KICK_FORWARD}"
    --pre-kick-lateral "${PRE_KICK_LATERAL}"
    --pre-kick-heave "${PRE_KICK_HEAVE}"
    --pre-kick-yaw "${PRE_KICK_YAW}"
    --post-pre-kick-neutral-s "${POST_PRE_KICK_NEUTRAL_S}"
  )
fi

"${PYTHON_BIN}" "${REPLAY_ARGS[@]}" >"${REPLAY_LOG}" 2>&1

kill -INT "${BAG_PID}" >/dev/null 2>&1 || true
wait "${BAG_PID}" >/dev/null 2>&1 || true
unset BAG_PID

COMPARE_ARGS=(
  "${SCRIPT_DIR}/compare_closed_loop_april1_replay.py"
  --real-bag "${BAG_PATH}"
  --sim-bag "${BAG_RECORD_DIR}"
  --align-rc-start
  --real-start-offset-s "${START_OFFSET_S}"
  --crop-start-s "${COMPARE_CROP_START_S:-0.0}"
  --output-dir "${OUT_DIR}/comparison"
)
if [[ -n "${DURATION_S}" ]]; then
  COMPARE_CROP_END_S="$(
    awk -v start="${COMPARE_CROP_START_S:-0.0}" -v duration="${DURATION_S}" \
      'BEGIN {printf "%.6f", start + duration}'
  )"
  COMPARE_ARGS+=(--crop-end-s "${COMPARE_CROP_END_S}")
fi

if "${PYTHON_BIN}" "${COMPARE_ARGS[@]}" \
  >"${OUT_DIR}/compare.log" 2>&1; then
  echo "[closed-loop] comparison written in ${OUT_DIR}/comparison"
else
  echo "[closed-loop] warning: comparison step failed; see ${OUT_DIR}/compare.log"
fi

echo "[closed-loop] outputs saved in ${OUT_DIR}"
