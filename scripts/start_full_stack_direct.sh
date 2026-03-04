#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ARDUPILOT_DIR="${ROOT_DIR}/kmu_hit25_ros2_ws/ardupilot"
MUJOCO_DIR="${ROOT_DIR}/mujoco/uuv_mujoco/v2.2"
QGC_APP="${QGC_APP_OVERRIDE:-${ROOT_DIR}/QGroundControl.AppImage}"
QGC_BIN="${QGC_BIN_OVERRIDE:-$(command -v QGroundControl || true)}"
VIDEO_BRIDGE="${MUJOCO_DIR}/scripts/ros2_to_qgc_video.py"
CHECKER_SCRIPT="${ROOT_DIR}/scripts/check_sitl_manual_input.py"
STACK_RESET="${ROOT_DIR}/scripts/stack_reset.sh"
MAV_GCS_SYSID="${MAV_GCS_SYSID_OVERRIDE:-1}"
MAV_GCS_SYSID_HI="${MAV_GCS_SYSID_HI_OVERRIDE:-255}"

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
VIDEO_PID=""
QGC_PID=""

cleanup() {
  set +e
  [[ -n "${VIDEO_PID}" ]] && kill "${VIDEO_PID}" 2>/dev/null || true
  [[ -n "${QGC_PID}" ]] && kill "${QGC_PID}" 2>/dev/null || true
  [[ -n "${MJ_PID}" ]] && kill "${MJ_PID}" 2>/dev/null || true
  [[ -n "${SITL_PID}" ]] && kill "${SITL_PID}" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

collect_ardusub_pid() {
  pgrep -f "${ARDUPILOT_DIR}/build/sitl/bin/ardusub" | head -n1 || true
}

write_sitl_failure_snapshot() {
  {
    echo "[fullstack] SITL readiness failure snapshot ($(date '+%Y-%m-%d %H:%M:%S %Z'))"
    echo "[fullstack] pgrep (sim_vehicle/ardusub/mavproxy):"
    pgrep -af "Tools/autotest/sim_vehicle.py|/build/sitl/bin/ardusub|mavproxy.py" || true
    echo "[fullstack] listen ports (5760/5763/5773/14551/14660/9002/9003):"
    ss -lntup 2>/dev/null | grep -E "5760|5763|5773|14551|14660|9002|9003" || true
    echo "[fullstack] quick checks:"
    echo "  tail -n 120 ${SITL_LOG}"
    echo "  ss -lntup | grep -E '5760|5763|5773|14551|14660|9002|9003'"
    echo "  pgrep -af 'sim_vehicle.py|mavproxy.py|/build/sitl/bin/ardusub'"
  } | tee -a "${SITL_LOG}" >> "${MUJOCO_LOG}"
}

wait_sitl_heartbeat_ready() {
  local timeout_s="${1:-60}"
  local start now
  start="$(date +%s)"
  while true; do
    if python3 "${CHECKER_SCRIPT}" --mode udp --listen 14551 --heartbeat-only --hb-timeout 1.5 --timeout 2.0 >/dev/null 2>&1; then
      return 0
    fi
    now="$(date +%s)"
    if (( now - start >= timeout_s )); then
      return 1
    fi
    sleep 0.4
  done
}

check_tcp_listen() {
  local port="$1"
  ss -lnt 2>/dev/null | grep -qE "[:.]${port}[[:space:]]"
}

wait_sitl_process_and_ports_ready() {
  local timeout_s="${1:-40}"
  local start now pid
  start="$(date +%s)"
  while true; do
    pid="$(collect_ardusub_pid)"
    if [[ -n "${pid}" ]] && check_tcp_listen 5760 && check_tcp_listen 5773; then
      return 0
    fi
    now="$(date +%s)"
    if (( now - start >= timeout_s )); then
      return 1
    fi
    sleep 0.3
  done
}

start_qgc() {
  local qgc_app="${QGC_APP}"
  if [[ ! -x "${qgc_app}" ]] && [[ -x "${ROOT_DIR}/QGroundControl-x86_64.AppImage" ]]; then
    qgc_app="${ROOT_DIR}/QGroundControl-x86_64.AppImage"
  fi

  if [[ -x "${qgc_app}" ]]; then
    echo "[fullstack] starting QGroundControl (AppImage extract-and-run)"
    "${qgc_app}" --appimage-extract-and-run >"${QGC_LOG}" 2>&1 &
    QGC_PID=$!
    return 0
  fi

  if [[ -n "${QGC_BIN}" ]] && [[ -x "${QGC_BIN}" ]]; then
    echo "[fullstack] starting QGroundControl (system binary: ${QGC_BIN})"
    "${QGC_BIN}" >"${QGC_LOG}" 2>&1 &
    QGC_PID=$!
    return 0
  fi

  return 1
}

start_sitl_console_map() {
  local sitl_cmd=(
    python3 Tools/autotest/sim_vehicle.py
    -L RATBeach --console --map
    -v ArduSub -f vectored_6dof --model JSON
    --out=udp:127.0.0.1:14550
    --out=udp:127.0.0.1:14551
    --out=udp:127.0.0.1:14660
    --out=tcpin:0.0.0.0:5773
    -P "MAV_GCS_SYSID=${MAV_GCS_SYSID}"
    -P "MAV_GCS_SYSID_HI=${MAV_GCS_SYSID_HI}"
  )
  local cmd_str
  local ardupilot_dir_q
  local sitl_log_q
  cmd_str="$(printf '%q ' "${sitl_cmd[@]}")"
  ardupilot_dir_q="$(printf '%q' "${ARDUPILOT_DIR}")"
  sitl_log_q="$(printf '%q' "${SITL_LOG}")"

  if command -v gnome-terminal >/dev/null 2>&1; then
    gnome-terminal -- bash -lc "cd ${ardupilot_dir_q}; ${cmd_str} 2>&1 | tee -a ${sitl_log_q}"
    return $?
  fi
  if command -v x-terminal-emulator >/dev/null 2>&1; then
    x-terminal-emulator -e bash -lc "cd ${ardupilot_dir_q}; ${cmd_str} 2>&1 | tee -a ${sitl_log_q}"
    return $?
  fi
  if command -v xterm >/dev/null 2>&1; then
    xterm -e bash -lc "cd ${ardupilot_dir_q}; ${cmd_str} 2>&1 | tee -a ${sitl_log_q}"
    return $?
  fi

  echo "[fullstack] error: no GUI terminal found (gnome-terminal/x-terminal-emulator/xterm)." >&2
  echo "[fullstack] run SITL manually in another terminal with --console --map." >&2
  return 1
}

echo "[fullstack] reset old stack processes (with-qgc-stop)"
"${STACK_RESET}" --with-qgc-stop

if [[ ! -d "${ARDUPILOT_DIR}" ]]; then
  echo "[fullstack] error: ArduPilot directory not found: ${ARDUPILOT_DIR}" >&2
  exit 1
fi

echo "[fullstack] starting ArduSub SITL (standard sim_vehicle --console --map)"
if ! start_sitl_console_map; then
  echo "[fullstack] error: failed to start ArduSub console/map terminal."
  write_sitl_failure_snapshot
  exit 1
fi

if wait_sitl_process_and_ports_ready 40; then
  SITL_PID="$(collect_ardusub_pid)"
  if [[ -z "${SITL_PID}" ]]; then
    echo "[fullstack] error: readiness detected but ardusub pid not found."
    write_sitl_failure_snapshot
    exit 1
  fi
  echo "[fullstack] ArduSub readiness passed (pid=${SITL_PID}, tcp:5760/5773)."
else
  echo "[fullstack] error: ArduSub readiness timeout (pid/tcp 5760/5773) after console/map launch."
  write_sitl_failure_snapshot
  exit 1
fi

if (( USE_HEADLESS == 0 )); then
  MJ_ARGS=(--sitl --images)
else
  MJ_ARGS=(--sitl --images --headless)
fi

MJ_ARGS+=(
  --sitl-servo-source mavlink
  --sitl-mavlink-endpoint udpin:0.0.0.0:14660
  --sitl-servo-map "yaw_rr,yaw_lr,yaw_rf,yaw_lf,ver_lf,ver_rf,ver_lr,ver_rr"
  --sitl-servo-signs=1,1,1,1,1,1,1,1
  --force-clean
)

echo "[fullstack] starting MuJoCo"
(
  cd "${MUJOCO_DIR}"
  AG_DISABLE_QGC_VIDEO_BRIDGE=1 ./launch_competition_sim.sh "${MJ_ARGS[@]}"
) >"${MUJOCO_LOG}" 2>&1 &
MJ_PID=$!

if wait_sitl_heartbeat_ready 60; then
  echo "[fullstack] ArduSub heartbeat on udp:14551 detected."
else
  echo "[fullstack] error: ArduSub heartbeat not detected on udp:14551 after MuJoCo startup."
  write_sitl_failure_snapshot
  echo "[fullstack] MuJoCo diagnostics:"
  echo "  log: ${MUJOCO_LOG}"
  tail -n 120 "${MUJOCO_LOG}" || true
  exit 1
fi

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
  if ! start_qgc; then
    echo "[fullstack] warn: QGC not found. Checked system binary and ${QGC_APP}"
  fi
fi

echo "[fullstack] log dir: ${LOG_DIR}"
echo "[fullstack] sitl log : ${SITL_LOG}"
echo "[fullstack] mujoco log : ${MUJOCO_LOG}"
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
  if ! kill -0 "${MJ_PID}" 2>/dev/null; then
    echo "[fullstack] MuJoCo exited."
    exit 1
  fi
  sleep 2
done
