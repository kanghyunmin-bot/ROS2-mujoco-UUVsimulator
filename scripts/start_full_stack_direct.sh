#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ARDUPILOT_DIR="${ROOT_DIR}/kmu_hit25_ros2_ws/ardupilot"
MUJOCO_DIR="${ROOT_DIR}/mujoco/uuv_mujoco/v2.2"
QGC_APP="${ROOT_DIR}/QGroundControl-x86_64.AppImage"
VIDEO_BRIDGE="${MUJOCO_DIR}/scripts/ros2_to_qgc_video.py"
STACK_RESET="${ROOT_DIR}/scripts/stack_reset.sh"
MAVPROXY_BIN="${HOME}/.local/bin/mavproxy.py"
MAV_GCS_SYSID="${MAV_GCS_SYSID_OVERRIDE:-3}"
MAV_GCS_SYSID_HI="${MAV_GCS_SYSID_HI_OVERRIDE:-0}"

START_QGC=1
START_VIDEO=1
USE_HEADLESS=0
QGC_DELAY=2
VIDEO_PORT=5600
VIDEO_HOST=127.0.0.1

usage() {
  cat <<'EOF'
Usage:
  ./scripts/start_full_stack_direct.sh [options]

Options:
  --no-qgc        Do not start QGroundControl
  --no-video      Do not start ROS2->QGC video bridge
  --headless      Start MuJoCo headless
  --help          Show this help
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --no-qgc)
      START_QGC=0
      shift
      ;;
    --no-video)
      START_VIDEO=0
      shift
      ;;
    --headless)
      USE_HEADLESS=1
      shift
      ;;
    --help|-h)
      usage
      exit 0
      ;;
    *)
      echo "[fullstack] unknown option: $1" >&2
      usage
      exit 2
      ;;
  esac
done

RUN_TAG="$(date +%Y%m%d_%H%M%S)"
LOG_DIR="${ROOT_DIR}/log/full_stack_${RUN_TAG}"
mkdir -p "${LOG_DIR}"
SITL_LOG="${LOG_DIR}/ardusub.log"
MUJOCO_LOG="${LOG_DIR}/mujoco.log"
MAVPROXY_LOG="${LOG_DIR}/mavproxy.log"
VIDEO_LOG="${LOG_DIR}/video_bridge.log"
QGC_LOG="${LOG_DIR}/qgc.log"

SITL_PID=""
MJ_PID=""
MAVPROXY_PID=""
VIDEO_PID=""
QGC_PID=""

cleanup() {
  set +e
  [[ -n "${VIDEO_PID}" ]] && kill "${VIDEO_PID}" 2>/dev/null || true
  [[ -n "${QGC_PID}" ]] && kill "${QGC_PID}" 2>/dev/null || true
  [[ -n "${MJ_PID}" ]] && kill "${MJ_PID}" 2>/dev/null || true
  [[ -n "${MAVPROXY_PID}" ]] && kill "${MAVPROXY_PID}" 2>/dev/null || true
  [[ -n "${SITL_PID}" ]] && kill "${SITL_PID}" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

if ! command -v "${MAVPROXY_BIN}" >/dev/null 2>&1; then
  MAVPROXY_BIN="$(command -v mavproxy.py || true)"
fi
if [[ -z "${MAVPROXY_BIN}" ]]; then
  echo "[fullstack] error: mavproxy.py not found. install mavlink-router/mavproxy and rerun." >&2
  exit 1
fi

echo "[fullstack] reset old stack processes (with-qgc-stop)"
"${STACK_RESET}" --with-qgc-stop

echo "[fullstack] starting ArduSub SITL (RATBeach, 14550/14551/14660)"
(
  cd "${ARDUPILOT_DIR}"
  python3 Tools/autotest/sim_vehicle.py -L RATBeach \
    -v ArduSub -f vectored_6dof --model JSON \
    --no-mavproxy \
    --out=udp:127.0.0.1:14550 \
    --out=udp:127.0.0.1:14551 \
    --out=udp:127.0.0.1:14660 \
    -P "MAV_GCS_SYSID=${MAV_GCS_SYSID}" \
    -P "MAV_GCS_SYSID_HI=${MAV_GCS_SYSID_HI}"
) >"${SITL_LOG}" 2>&1 &
SITL_PID=$!

sleep 2

echo "[fullstack] starting MAVProxy bridge"
(
  "${MAVPROXY_BIN}" \
    --master tcp:127.0.0.1:5760 \
    --out=udp:127.0.0.1:14550 \
    --out=udp:127.0.0.1:14551 \
    --out=udp:127.0.0.1:14660 \
    --sitl 127.0.0.1:5501 \
    --non-interactive
) >"${MAVPROXY_LOG}" 2>&1 &
MAVPROXY_PID=$!

if (( USE_HEADLESS == 0 )); then
  MJ_ARGS=(--sitl --images)
else
  MJ_ARGS=(--sitl --images --headless)
fi

MJ_ARGS+=(--sitl-servo-source mavlink --sitl-mavlink-endpoint udpin:0.0.0.0:14660 --force-clean)

echo "[fullstack] starting MuJoCo"
(
  cd "${MUJOCO_DIR}"
  ./launch_competition_sim.sh "${MJ_ARGS[@]}"
) >"${MUJOCO_LOG}" 2>&1 &
MJ_PID=$!

sleep "${QGC_DELAY}"

if (( START_VIDEO == 1 )); then
  if [[ -f "${VIDEO_BRIDGE}" ]]; then
    echo "[fullstack] starting ROS2->QGC video bridge"
    (
      set +u
      if [[ -f /opt/ros/humble/setup.bash ]]; then
        source /opt/ros/humble/setup.bash
      fi
      set -u
      python3 "${VIDEO_BRIDGE}" \
        --topic /stereo/left/image_raw \
        --host "${VIDEO_HOST}" \
        --port "${VIDEO_PORT}"
    ) >"${VIDEO_LOG}" 2>&1 &
    VIDEO_PID=$!
  else
    echo "[fullstack] warn: video bridge script not found: ${VIDEO_BRIDGE}"
  fi
fi

if (( START_QGC == 1 )); then
  if [[ -x "${QGC_APP}" ]]; then
    echo "[fullstack] starting QGroundControl"
    "${QGC_APP}" >"${QGC_LOG}" 2>&1 &
    QGC_PID=$!
  else
    echo "[fullstack] warn: QGC app not found: ${QGC_APP}"
  fi
fi

echo "[fullstack] log dir: ${LOG_DIR}"
echo "[fullstack] sitl log : ${SITL_LOG}"
echo "[fullstack] mujoco log : ${MUJOCO_LOG}"
echo "[fullstack] mavproxy log : ${MAVPROXY_LOG}"
if [[ -n "${VIDEO_PID}" ]]; then
  echo "[fullstack] video log : ${VIDEO_LOG}"
fi
if [[ -n "${QGC_PID}" ]]; then
  echo "[fullstack] qgc log : ${QGC_LOG}"
fi
echo "[fullstack] pids:"
echo "  SITL=${SITL_PID}"
echo "  MUJOCO=${MJ_PID}"
echo "  VIDEO=${VIDEO_PID:-<disabled>}"
echo "  QGC=${QGC_PID:-<disabled>}"
echo "[fullstack] Ctrl+C to stop all."

while true; do
  if ! kill -0 "${SITL_PID}" 2>/dev/null; then
    echo "[fullstack] ArduSub SITL exited."
    exit 1
  fi
  if ! kill -0 "${MAVPROXY_PID}" 2>/dev/null; then
    echo "[fullstack] MAVProxy exited."
    exit 1
  fi
  if ! kill -0 "${MJ_PID}" 2>/dev/null; then
    echo "[fullstack] MuJoCo exited."
    exit 1
  fi
  sleep 2
done
