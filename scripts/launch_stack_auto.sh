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
FRAME="vectored_6dof"
FORCE_CLEAN=1
DUAL_QGC_LINK=0
VIDEO_TOPIC="/stereo/left/image_raw"
VIDEO_HOST="127.0.0.1"
VIDEO_PORT=5600
VIDEO_FPS=15
VIDEO_BITRATE_KBPS=2000
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
  --no-wipe         Keep ArduSub eeprom/params (default: wipe)
  --sitl-debug      Enable SITL command debug logs in MuJoCo bridge
  --frame <name>    ArduSub frame (default: vectored_6dof)
  --dual-qgc-link   Enable parallel UDP+TCP QGC links (default: single link)
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
    --dual-qgc-link)
      DUAL_QGC_LINK=1
      shift
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
if (( DUAL_QGC_LINK )); then
  ARDU_CMD+=(--dual-qgc-link)
fi
if (( NO_WIPE )); then
  ARDU_CMD+=(--no-wipe)
else
  ARDU_CMD+=(--wipe)
fi

"${ARDU_CMD[@]}" >"${ARDU_LOG}" 2>&1 &
AP_PID=$!

if wait_for_log "${ARDU_LOG}" "RiTW: Starting ArduSub|ArduPilot Ready|ArduSub V" 25; then
  echo "[auto] ArduSub start detected."
else
  echo "[auto] warning: ArduSub ready log not found yet (continuing)."
fi

echo "[auto] starting MuJoCo..."
MUJOCO_CMD=("${MUJOCO_SCRIPT}" --sitl)
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
  sleep 2
done
