#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/../../.." && pwd)"
ROS_WORKSPACE="${WORKSPACE_DIR}/rospkg"
STAMP="$(date +%Y%m%d_%H%M%S)"
OUTPUT_DIR="${WORKSPACE_DIR}/recordings/snr_homing_${STAMP}"
RECORD_VIDEO=0
SUCCESS_RANGE_M=1.2
TEST_DURATION_S=180

usage() {
  cat <<'EOF'
Usage: run_snr_homing_pipeline.sh [--output-dir DIR] [--record-video]
                                  [--success-range-m METERS]
                                  [--duration SECONDS]

Requires start_snr_homing_sim.sh and the real auv rov_start.launch.py stack
to be running. Runs the PCM-derived SNR V2 estimator and RC controller, stops
at the configured acceptance gate (default: 1.2 m), disarms, and writes
result.json plus node logs.
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --output-dir)
      OUTPUT_DIR="$2"
      shift 2
      ;;
    --record-video)
      RECORD_VIDEO=1
      shift
      ;;
    --success-range-m)
      SUCCESS_RANGE_M="$2"
      shift 2
      ;;
    --duration)
      TEST_DURATION_S="$2"
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "Unknown argument: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

mkdir -p "${OUTPUT_DIR}"
set +u
source /opt/ros/humble/setup.bash
source "${ROS_WORKSPACE}/install/setup.bash"
set -u

AUDIO_PID=""
DETECTOR_PID=""
ESTIMATOR_PID=""
CONTROLLER_PID=""
VIDEO_PID=""

stop_pid() {
  local pid="$1"
  if [[ -n "${pid}" ]] && kill -0 "${pid}" 2>/dev/null; then
    # `ros2 run` is a Python wrapper around the actual node. Signal the child
    # first; otherwise waiting on the wrapper can hang after a successful run.
    pkill -INT -P "${pid}" 2>/dev/null || true
    kill -INT "${pid}" 2>/dev/null || true
    for _wait_step in $(seq 1 30); do
      if ! kill -0 "${pid}" 2>/dev/null; then
        break
      fi
      sleep 0.1
    done
    if kill -0 "${pid}" 2>/dev/null; then
      pkill -TERM -P "${pid}" 2>/dev/null || true
      kill -TERM "${pid}" 2>/dev/null || true
    fi
    wait "${pid}" 2>/dev/null || true
  fi
}

cleanup() {
  local rc=$?
  trap - EXIT INT TERM
  stop_pid "${CONTROLLER_PID}"
  if ros2 service type /mavros/cmd/arming >/dev/null 2>&1; then
    ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool \
      "{value: false}" >"${OUTPUT_DIR}/disarm.log" 2>&1 || true
  fi
  stop_pid "${ESTIMATOR_PID}"
  stop_pid "${DETECTOR_PID}"
  stop_pid "${AUDIO_PID}"
  stop_pid "${VIDEO_PID}"
  exit "${rc}"
}
trap cleanup EXIT INT TERM

timeout 15 ros2 topic echo /mavros/state --once >/dev/null
timeout 15 ros2 topic echo /audio --once >/dev/null

python3 "${SCRIPT_DIR}/audio_stamped_relay.py" \
  --ros-args -p use_sim_time:=true \
  >"${OUTPUT_DIR}/audio_stamped_relay.log" 2>&1 &
AUDIO_PID=$!

ros2 run audio_capture audio_frequency_detector \
  --ros-args -p use_sim_time:=true \
  >"${OUTPUT_DIR}/audio_frequency_detector.log" 2>&1 &
DETECTOR_PID=$!

ros2 run audio_capture snr_gradient_homing_v2 \
  --ros-args -p use_sim_time:=true \
  >"${OUTPUT_DIR}/snr_gradient_homing_v2.log" 2>&1 &
ESTIMATOR_PID=$!

timeout 20 ros2 topic echo \
  /audio_frequency_detector/snr_db_stamped --once >/dev/null

ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode \
  "{base_mode: 0, custom_mode: 'ALT_HOLD'}" \
  >"${OUTPUT_DIR}/set_mode.log" 2>&1
for _attempt in 1 2 3; do
  ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool \
    "{value: true}" >"${OUTPUT_DIR}/arm.log" 2>&1 || true
  sleep 1
done

if [[ "${RECORD_VIDEO}" == "1" ]]; then
  WINDOW_ID="$(
    xwininfo -root -tree |
      awk '/"MuJoCo :/ {print $1; exit}'
  )"
  if [[ -z "${WINDOW_ID}" ]]; then
    echo "MuJoCo window not found; cannot record video" >&2
    exit 1
  fi
  ffmpeg -hide_banner -loglevel warning -y \
    -f x11grab -framerate 12 -window_id "${WINDOW_ID}" -i "${DISPLAY:-:0.0}" \
    -vf "crop=trunc(iw/2)*2:trunc(ih/2)*2,scale=960:540" \
    -c:v libx264 -preset veryfast -crf 20 -pix_fmt yuv420p \
    "${OUTPUT_DIR}/snr_homing.mp4" \
    >"${OUTPUT_DIR}/ffmpeg.log" 2>&1 &
  VIDEO_PID=$!
fi

python3 "${SCRIPT_DIR}/check_snr_homing_sim_runtime.py" \
  --duration "${TEST_DURATION_S}" \
  --success-range-m "${SUCCESS_RANGE_M}" \
  --success-hold-s 0.5 \
  --require-homing-state \
  --max-direction-error-deg 75 \
  --min-horizontal-progress-m 1.0 \
  --json-out "${OUTPUT_DIR}/result.json" \
  >"${OUTPUT_DIR}/checker.log" 2>&1 &
CHECKER_PID=$!

# The pinger is depth-matched in this acceptance scene. Remap the estimator's
# one-shot vertical request so this run tests the package's primary horizontal
# SNR-gradient acquisition and homing path in isolation.
ros2 run audio_capture snr_gradient_homing_controller --ros-args \
  -r /homing/vertical_search_request:=/homing/vertical_search_request_disabled \
  -p use_sim_time:=true \
  -p initial_diagonal_command:=0.0 \
  -p initial_diagonal_duration_s:=0.1 \
  -p search_forward:=0.08 \
  -p search_yaw:=0.60 \
  -p search_turn_sign:=-1.0 \
  -p search_min_duration_s:=10.0 \
  -p acquire_hold_s:=1.5 \
  -p min_direction_confidence:=0.035 \
  -p recovery_to_search_s:=10.0 \
  -p forward_fast:=0.38 \
  -p forward_mid:=0.26 \
  -p forward_slow:=0.12 \
  -p yaw_gain:=1.8 \
  -p yaw_limit:=0.85 \
  -p confidence_speed_floor:=0.45 \
  -p collision_hold_s:=8.0 \
  -p rc_pwm_span:=400.0 \
  >"${OUTPUT_DIR}/snr_gradient_homing_controller.log" 2>&1 &
CONTROLLER_PID=$!

set +e
wait "${CHECKER_PID}"
CHECK_RC=$?
set -e

stop_pid "${CONTROLLER_PID}"
CONTROLLER_PID=""
stop_pid "${VIDEO_PID}"
VIDEO_PID=""

cat "${OUTPUT_DIR}/result.json"
exit "${CHECK_RC}"
