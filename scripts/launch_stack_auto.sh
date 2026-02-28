#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ARDU_SCRIPT="${ROOT_DIR}/kmu_hit25_ros2_ws/scripts/run_ardusub_json_sitl.sh"
MUJOCO_DIR="${ROOT_DIR}/mujoco/uuv_mujoco/v2.2"
MUJOCO_SCRIPT="${MUJOCO_DIR}/launch_competition_sim.sh"
VIDEO_BRIDGE_SCRIPT="${MUJOCO_DIR}/scripts/ros2_to_qgc_video.py"
QGC_APP="${ROOT_DIR}/QGroundControl-x86_64.AppImage"

WITH_QGC=0
HEADLESS=0
USE_IMAGES=1
USE_VIDEO=1
NO_WIPE=0
SITL_DEBUG=0
MAVLINK_TO_MUJOCO=0
MAVLINK_TO_MUJOCO_FORCED=0
MUJOCO_MAVLINK_PORT=14660
MUJOCO_MAVLINK_HZ=80
MAV_GCS_SYSID=3
MAV_GCS_SYSID_HI=0
QGC_MAVROS_PORT=14551
SITL_MAVLINK_TARGET_SYSID=0
SITL_MAVLINK_TARGET_COMPID=0
SITL_MAVLINK_SOURCE_SYSID=255
SITL_MAVLINK_SOURCE_COMPID=190
FRAME="vectored_6dof"
FORCE_CLEAN=1
FAST_RESPONSE=1
LATERAL_REVERSED=1
FORWARD_REVERSED=0
YAW_REVERSED=0
THROTTLE_REVERSED=0
RCMAP_FORWARD=5
RCMAP_LATERAL=6
RCMAP_ROLL=1
RCMAP_PITCH=2
RCMAP_THROTTLE=3
RCMAP_YAW=4
VIDEO_TOPIC="/stereo/left/image_raw"
VIDEO_HOST="127.0.0.1"
VIDEO_PORT=5600
VIDEO_FPS=15
VIDEO_BITRATE_KBPS=2600
VIDEO_WIDTH=640
VIDEO_HEIGHT=360

usage() {
  cat <<'EOF'
Usage:
  ./scripts/launch_stack_auto.sh [options]

Options:
  --with-qgc        Launch QGroundControl AppImage automatically
  --headless        Run MuJoCo without viewer
  --no-images       Disable ROS2 image topics in MuJoCo launch
  --no-video        Disable QGC UDP video bridge (default: enabled)
  --with-mavlink    Use legacy MAVLink SERVO_OUTPUT_RAW bridge (default: JSON direct)
  --mavlink-port    Port for SERVO_OUTPUT_RAW from ArduSub (default: 14660)
  --mavlink-hz      SERVO_OUTPUT_RAW stream request rate in Hz (default: 80)
  --mavlink-target-sysid <0-255>    MAVLink target sysid used by MuJoCo servo listener (0=auto from heartbeat)
  --mavlink-target-compid <0-255>   MAVLink target compid used by MuJoCo servo listener (0=auto from heartbeat)
  --mavlink-source-sysid <1-255>    MAVLink source sysid used by MuJoCo MAVLink socket (default: 255)
  --mavlink-source-compid <1-255>   MAVLink source compid used by MuJoCo MAVLink socket (default: 190)
  --mav-gcs-sysid <1-255>          MAVLink ground station sysid required for manual control/failsafe (default: 3)
  --mav-gcs-sysid-hi <0-255>       Upper GCS sysid for range acceptance (default: 0, means single id)
  --no-wipe         Keep ArduSub eeprom/params (default: wipe)
  --sitl-debug      Enable SITL command debug logs in MuJoCo bridge
  --frame <name>    ArduSub frame (default: vectored_6dof)
  --no-fast-response Disable ArduSub fast-response tuning
  --lateral-normal  Keep lateral axis non-reversed (default: reversed)
  --lateral-reversed Reverse lateral axis
  --forward-reversed Reverse forward axis
  --yaw-reversed    Reverse yaw axis
  --throttle-reversed Reverse throttle axis
  --rcmap-forward <1-16>   Override ArduSub RCMAP_FORWARD (default: 5)
  --rcmap-lateral <1-16>   Override ArduSub RCMAP_LATERAL (default: 6)
  --rcmap-roll <1-16>      Override ArduSub RCMAP_ROLL (default: 1)
  --rcmap-pitch <1-16>     Override ArduSub RCMAP_PITCH (default: 2)
  --rcmap-throttle <1-16>  Override ArduSub RCMAP_THROTTLE (default: 3)
  --rcmap-yaw <1-16>       Override ArduSub RCMAP_YAW (default: 4)
  --no-force-clean  Do not auto-stop existing stack processes before launch
  -h, --help        Show this help
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --with-qgc)
      WITH_QGC=1
      shift
      ;;
    --headless)
      HEADLESS=1
      shift
      ;;
    --no-images)
      USE_IMAGES=0
      shift
      ;;
    --no-video)
      USE_VIDEO=0
      shift
      ;;
    --with-mavlink)
      MAVLINK_TO_MUJOCO=1
      MAVLINK_TO_MUJOCO_FORCED=1
      shift
      ;;
    --mavlink-port)
      MUJOCO_MAVLINK_PORT="${2:-}"
      shift 2
      ;;
    --mavlink-hz)
      MUJOCO_MAVLINK_HZ="${2:-}"
      shift 2
      ;;
    --mavlink-target-sysid)
      SITL_MAVLINK_TARGET_SYSID="${2:-}"
      shift 2
      ;;
    --mavlink-target-compid)
      SITL_MAVLINK_TARGET_COMPID="${2:-}"
      shift 2
      ;;
    --mavlink-source-sysid)
      SITL_MAVLINK_SOURCE_SYSID="${2:-}"
      shift 2
      ;;
    --mavlink-source-compid)
      SITL_MAVLINK_SOURCE_COMPID="${2:-}"
      shift 2
      ;;
    --mav-gcs-sysid)
      MAV_GCS_SYSID="${2:-}"
      shift 2
      ;;
    --mav-gcs-sysid-hi)
      MAV_GCS_SYSID_HI="${2:-}"
      shift 2
      ;;
    --no-wipe)
      NO_WIPE=1
      shift
      ;;
    --sitl-debug)
      SITL_DEBUG=1
      shift
      ;;
    --frame)
      FRAME="${2:-}"
      shift 2
      ;;
    --no-fast-response)
      FAST_RESPONSE=0
      shift
      ;;
    --lateral-normal)
      LATERAL_REVERSED=0
      shift
      ;;
    --lateral-reversed)
      LATERAL_REVERSED=1
      shift
      ;;
    --forward-reversed)
      FORWARD_REVERSED=1
      shift
      ;;
    --yaw-reversed)
      YAW_REVERSED=1
      shift
      ;;
    --throttle-reversed)
      THROTTLE_REVERSED=1
      shift
      ;;
    --rcmap-forward)
      RCMAP_FORWARD="${2:-}"
      shift 2
      ;;
    --rcmap-lateral)
      RCMAP_LATERAL="${2:-}"
      shift 2
      ;;
    --rcmap-roll)
      RCMAP_ROLL="${2:-}"
      shift 2
      ;;
    --rcmap-pitch)
      RCMAP_PITCH="${2:-}"
      shift 2
      ;;
    --rcmap-throttle)
      RCMAP_THROTTLE="${2:-}"
      shift 2
      ;;
    --rcmap-yaw)
      RCMAP_YAW="${2:-}"
      shift 2
      ;;
    --no-force-clean)
      FORCE_CLEAN=0
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
  *)
      echo "[auto] unknown option: $1" >&2
      usage
      exit 2
      ;;
  esac
done

validate_rcmap() {
  local name="$1"
  local value="$2"
  if ! [[ "${value}" =~ ^[0-9]+$ ]]; then
    echo "[auto] invalid ${name}: ${value} (must be integer 1~16)" >&2
    exit 2
  fi
  if (( value < 1 || value > 16 )); then
    echo "[auto] invalid ${name}: ${value} (must be integer 1~16)" >&2
    exit 2
  fi
}

validate_rcmap "--rcmap-forward" "${RCMAP_FORWARD}"
validate_rcmap "--rcmap-lateral" "${RCMAP_LATERAL}"
validate_rcmap "--rcmap-roll" "${RCMAP_ROLL}"
validate_rcmap "--rcmap-pitch" "${RCMAP_PITCH}"
validate_rcmap "--rcmap-throttle" "${RCMAP_THROTTLE}"
validate_rcmap "--rcmap-yaw" "${RCMAP_YAW}"

validate_mavlink_id() {
  local name="$1"
  local value="$2"
  if ! [[ "${value}" =~ ^[0-9]+$ ]]; then
    echo "[auto] invalid ${name}: ${value} (must be integer 0~255)" >&2
    exit 2
  fi
  if (( value < 0 || value > 255 )); then
    echo "[auto] invalid ${name}: ${value} (must be 0~255)" >&2
    exit 2
  fi
}
validate_mavlink_id "--mavlink-target-sysid" "${SITL_MAVLINK_TARGET_SYSID}"
  validate_mavlink_id "--mavlink-target-compid" "${SITL_MAVLINK_TARGET_COMPID}"
  validate_mavlink_id "--mavlink-source-sysid" "${SITL_MAVLINK_SOURCE_SYSID}"
  validate_mavlink_id "--mavlink-source-compid" "${SITL_MAVLINK_SOURCE_COMPID}"
validate_gcs_sysid() {
  local name="$1"
  local value="$2"
  local min="$3"
  local max="$4"
  if ! [[ "${value}" =~ ^[0-9]+$ ]]; then
    echo "[auto] invalid ${name}: ${value} (must be integer ${min}~${max})" >&2
    exit 2
  fi
  if (( value < min || value > max )); then
    echo "[auto] invalid ${name}: ${value} (must be ${min}~${max})" >&2
    exit 2
  fi
}
validate_gcs_sysid "--mav-gcs-sysid" "${MAV_GCS_SYSID}" 1 255
validate_gcs_sysid "--mav-gcs-sysid-hi" "${MAV_GCS_SYSID_HI}" 0 255

if (( WITH_QGC )) && (( MAVLINK_TO_MUJOCO_FORCED == 0 )); then
  MAVLINK_TO_MUJOCO=1
  echo "[auto] with-qgc: forcing MuJoCo MAVLink servo bridge (SERVO_OUTPUT_RAW)."
fi
echo "[auto] rc map override: roll=${RCMAP_ROLL}, pitch=${RCMAP_PITCH}, throttle=${RCMAP_THROTTLE}, yaw=${RCMAP_YAW}, fwd=${RCMAP_FORWARD}, lat=${RCMAP_LATERAL}"

if (( WITH_QGC )); then
  echo "[auto] QGC requested: fixed UDP links (14550 for QGC, 14551 for MAVROS)"
fi

if [[ ! -x "${ARDU_SCRIPT}" ]]; then
  echo "[auto] ArduSub launcher not found: ${ARDU_SCRIPT}" >&2
  exit 1
fi
if [[ ! -x "${MUJOCO_SCRIPT}" ]]; then
  echo "[auto] MuJoCo launcher not found: ${MUJOCO_SCRIPT}" >&2
  exit 1
fi
if (( USE_VIDEO )) && [[ ! -f "${VIDEO_BRIDGE_SCRIPT}" ]]; then
  echo "[auto] warning: video bridge script not found: ${VIDEO_BRIDGE_SCRIPT}" >&2
  USE_VIDEO=0
fi

RUN_TAG="$(date +%Y%m%d_%H%M%S)"
LOG_DIR="${ROOT_DIR}/log/auto_stack_${RUN_TAG}"
mkdir -p "${LOG_DIR}"
ARDU_LOG="${LOG_DIR}/ardusub.log"
MUJOCO_LOG="${LOG_DIR}/mujoco.log"
VIDEO_LOG="${LOG_DIR}/video_bridge.log"
QGC_LOG="${LOG_DIR}/qgc.log"

AP_PID=""
MJ_PID=""
VIDEO_PID=""
QGC_PID=""

cleanup() {
  set +e
  if [[ -n "${VIDEO_PID}" ]] && kill -0 "${VIDEO_PID}" 2>/dev/null; then
    kill "${VIDEO_PID}" 2>/dev/null || true
  fi
  if [[ -n "${MJ_PID}" ]] && kill -0 "${MJ_PID}" 2>/dev/null; then
    kill "${MJ_PID}" 2>/dev/null || true
  fi
  if [[ -n "${AP_PID}" ]] && kill -0 "${AP_PID}" 2>/dev/null; then
    kill "${AP_PID}" 2>/dev/null || true
  fi
  if [[ -n "${QGC_PID}" ]] && kill -0 "${QGC_PID}" 2>/dev/null; then
    kill "${QGC_PID}" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

wait_for_log() {
  local file="$1"
  local pattern="$2"
  local timeout_s="$3"
  local start now
  start="$(date +%s)"
  while true; do
    if [[ -f "${file}" ]]; then
      if command -v rg >/dev/null 2>&1; then
        if rg -q "${pattern}" "${file}"; then
          return 0
        fi
      else
        if grep -E -q "${pattern}" "${file}"; then
          return 0
        fi
      fi
    fi
    now="$(date +%s)"
    if (( now - start >= timeout_s )); then
      return 1
    fi
    sleep 0.2
  done
}

echo "[auto] log dir: ${LOG_DIR}"
echo "[auto] starting ArduSub..."

ARDU_CMD=("${ARDU_SCRIPT}" --frame "${FRAME}")
if (( FORCE_CLEAN )); then
  ARDU_CMD+=(--force-clean)
fi
if (( MAVLINK_TO_MUJOCO )); then
  ARDU_CMD+=(--mujoco-mavlink)
  ARDU_CMD+=(--mujoco-mavlink-port "${MUJOCO_MAVLINK_PORT}")
else
  ARDU_CMD+=(--no-mujoco-mavlink)
fi
ARDU_CMD+=(--mavros-port "${QGC_MAVROS_PORT}")
if (( NO_WIPE )); then
  ARDU_CMD+=(--no-wipe)
else
  ARDU_CMD+=(--wipe)
fi
if (( FAST_RESPONSE )); then
  ARDU_CMD+=(--fast-response)
else
  ARDU_CMD+=(--no-fast-response)
fi
ARDU_CMD+=(
  --rcmap-forward "${RCMAP_FORWARD}"
  --rcmap-lateral "${RCMAP_LATERAL}"
  --rcmap-roll "${RCMAP_ROLL}"
  --rcmap-pitch "${RCMAP_PITCH}"
  --rcmap-throttle "${RCMAP_THROTTLE}"
  --rcmap-yaw "${RCMAP_YAW}"
)
if (( LATERAL_REVERSED )); then
  ARDU_CMD+=(--lateral-reversed)
else
  ARDU_CMD+=(--lateral-normal)
fi
if (( FORWARD_REVERSED )); then
  ARDU_CMD+=(--forward-reversed)
else
  ARDU_CMD+=(--forward-normal)
fi
if (( YAW_REVERSED )); then
  ARDU_CMD+=(--yaw-reversed)
else
  ARDU_CMD+=(--yaw-normal)
fi
  if (( THROTTLE_REVERSED )); then
    ARDU_CMD+=(--throttle-reversed)
  else
    ARDU_CMD+=(--throttle-normal)
  fi
  ARDU_CMD+=(
    --mav-gcs-sysid "${MAV_GCS_SYSID}"
    --mav-gcs-sysid-hi "${MAV_GCS_SYSID_HI}"
  )

"${ARDU_CMD[@]}" >"${ARDU_LOG}" 2>&1 &
AP_PID=$!

if wait_for_log "${ARDU_LOG}" "RiTW: Starting ArduSub|ArduPilot Ready|ArduSub V" 25; then
  echo "[auto] ArduSub start detected."
else
  echo "[auto] warning: ArduSub ready log not found yet (continuing)."
fi

echo "[auto] starting MuJoCo..."
MUJOCO_CMD=("${MUJOCO_SCRIPT}" --sitl)
if (( MAVLINK_TO_MUJOCO )); then
  if ! [[ "${MUJOCO_MAVLINK_HZ}" =~ ^[0-9]+([.][0-9]+)?$ ]]; then
    echo "[auto] invalid --mavlink-hz: ${MUJOCO_MAVLINK_HZ}" >&2
    exit 2
  fi
  if ! [[ "${MUJOCO_MAVLINK_PORT}" =~ ^[0-9]+$ ]]; then
    echo "[auto] invalid --mavlink-port: ${MUJOCO_MAVLINK_PORT}" >&2
    exit 2
  fi
  MUJOCO_CMD+=(
    --sitl-servo-source mavlink
    --sitl-mavlink-endpoint "udpin:0.0.0.0:${MUJOCO_MAVLINK_PORT}"
    --sitl-mavlink-servo-hz "${MUJOCO_MAVLINK_HZ}"
    --sitl-mavlink-target-sysid "${SITL_MAVLINK_TARGET_SYSID}"
    --sitl-mavlink-target-compid "${SITL_MAVLINK_TARGET_COMPID}"
    --sitl-mavlink-source-sysid "${SITL_MAVLINK_SOURCE_SYSID}"
    --sitl-mavlink-source-compid "${SITL_MAVLINK_SOURCE_COMPID}"
    --sitl-servo-map "yaw_rf,yaw_lf,yaw_rr,yaw_lr,ver_rf,ver_lf,ver_rr,ver_lr"
    --sitl-servo-signs=1,1,1,1,-1,-1,-1,-1
  )
  echo "[auto] MuJoCo MAVLink target sysid/compid=${SITL_MAVLINK_TARGET_SYSID}/${SITL_MAVLINK_TARGET_COMPID}, source sysid/compid=${SITL_MAVLINK_SOURCE_SYSID}/${SITL_MAVLINK_SOURCE_COMPID}"
else
  MUJOCO_CMD+=(--sitl-servo-source json)
fi
if (( FORCE_CLEAN )); then
  MUJOCO_CMD+=(--force-clean)
fi
if (( USE_IMAGES )); then
  MUJOCO_CMD+=(--images)
fi
if (( HEADLESS )); then
  MUJOCO_CMD+=(--headless)
fi
if (( SITL_DEBUG )); then
  MUJOCO_CMD+=(--sitl-command-debug)
fi

(
  cd "${MUJOCO_DIR}"
  "${MUJOCO_CMD[@]}"
) >"${MUJOCO_LOG}" 2>&1 &
MJ_PID=$!

if wait_for_log "${MUJOCO_LOG}" "SITL socket initialized" 20; then
  echo "[auto] MuJoCo SITL socket initialized."
else
  echo "[auto] warning: MuJoCo SITL socket init log not found yet."
fi

if (( USE_VIDEO )) && (( USE_IMAGES )); then
  echo "[auto] starting ROS2->QGC video bridge..."
  (
    set +u
    source /opt/ros/humble/setup.bash
    set -u
    python3 "${VIDEO_BRIDGE_SCRIPT}" \
      --topic "${VIDEO_TOPIC}" \
      --host "${VIDEO_HOST}" \
      --port "${VIDEO_PORT}" \
      --fps "${VIDEO_FPS}" \
      --bitrate-kbps "${VIDEO_BITRATE_KBPS}" \
      --width "${VIDEO_WIDTH}" \
      --height "${VIDEO_HEIGHT}"
  ) >"${VIDEO_LOG}" 2>&1 &
  VIDEO_PID=$!
  echo "[auto] video bridge target: udp://${VIDEO_HOST}:${VIDEO_PORT} (topic=${VIDEO_TOPIC})"
elif (( USE_VIDEO )); then
  echo "[auto] video bridge skipped: --no-images enabled."
fi

if (( WITH_QGC )); then
  if [[ -x "${QGC_APP}" ]]; then
    echo "[auto] starting QGroundControl..."
    "${QGC_APP}" >"${QGC_LOG}" 2>&1 &
    QGC_PID=$!
  else
    echo "[auto] warning: QGC AppImage not executable: ${QGC_APP}"
  fi
fi

echo "[auto] stack running."
echo "[auto] ardupilot log: ${ARDU_LOG}"
echo "[auto] mujoco log:    ${MUJOCO_LOG}"
if [[ -n "${VIDEO_PID}" ]]; then
  echo "[auto] video log:     ${VIDEO_LOG}"
fi
if (( WITH_QGC )); then
  echo "[auto] qgc log:       ${QGC_LOG}"
fi
echo "[auto] press Ctrl+C to stop all launched processes."

while true; do
  if ! kill -0 "${AP_PID}" 2>/dev/null; then
    echo "[auto] ArduSub exited. stopping stack."
    exit 1
  fi
  if ! kill -0 "${MJ_PID}" 2>/dev/null; then
    echo "[auto] MuJoCo exited. stopping stack."
    exit 1
  fi
  if [[ -n "${VIDEO_PID}" ]] && ! kill -0 "${VIDEO_PID}" 2>/dev/null; then
    echo "[auto] warning: video bridge exited; keeping ArduSub+MuJoCo running."
    echo "[auto]          check video log: ${VIDEO_LOG}"
    VIDEO_PID=""
  fi
  if (( WITH_QGC )) && [[ -n "${QGC_PID}" ]] && ! kill -0 "${QGC_PID}" 2>/dev/null; then
    echo "[auto] warning: QGC exited; stack continues only on ArduSub+MuJoCo."
    echo "[auto]          check qgc log: ${QGC_LOG}"
    QGC_PID=""
  fi
  sleep 2
done
