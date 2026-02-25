#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
WS_DIR=$(cd "${SCRIPT_DIR}/.." && pwd)
ARDUPILOT_DIR="${WS_DIR}/ardupilot"

FRAME="vectored_6dof"
INSTANCE="0"
QGC_PORT="14550"
QGC_HOST="127.0.0.1"
QGC_LINK_MODE="udpclient"
QGC_TCP_PORT="5760"
QGC_PORT_WAS_SET="0"
DUAL_QGC_LINK="1"
USE_MAVPROXY="0"
SIM_IP="127.0.0.1"
SIM_PORT_IN="9003"
SIM_PORT_OUT="9002"
NO_REBUILD="1"
WIPE_EEPROM="1"
DDS_ENABLE="0"
EXTRA_SIM_ARGS=""
PILOT_FAILSAFE_MODE="disabled"
FORCE_CLEAN="0"

usage() {
  cat <<'EOF'
Usage:
  ./scripts/run_ardusub_json_sitl.sh [options]

Options:
  --frame <vectored|vectored_6dof>   ArduSub frame (default: vectored_6dof)
  --instance <N>                     SITL instance index (default: 0)
  --qgc-host <ip>                    QGC UDP host (default: 127.0.0.1)
  --qgc-port <port>                  MAVLink port for --qgc-link (default: 14550 for udpclient, 5760 for tcpclient)
  --qgc-tcp-port <port>              QGC TCP port (default: 5760)
  --qgc-link <udpclient|tcpclient>   QGC serial link mode for ArduPilot (default: udpclient)
  --dual-qgc-link                     Also open the opposite link in parallel (udp + tcp server/client pair, default: enabled)
  --no-dual-qgc-link                  Disable opposite parallel link
  --use-mavproxy                      Start sim_vehicle with MAVProxy (default: disabled)
  --sim-ip <ip>                      Sim address for ArduPilot JSON (default: 127.0.0.1)
  --sim-port-in <port>                ArduPilot JSON input port; simulator JSON packets must arrive here
                                        (default: 9003)
  --sim-port-out <port>               ArduPilot JSON output port; SITL servo packets are sent here
                                        (default: 9002)
  --force-clean                       Kill existing ArduSub/SITL processes from this workspace before launch
  --dds                              Enable DDS (default: disabled)
  --pilot-failsafe <disabled|warn|disarm>
                                     FS_PILOT_INPUT for SITL (default: disabled)
  --rebuild                          Rebuild before launch (default: no rebuild)
  --wipe                             Force wipe eeprom/params on boot (enabled by default)
  --no-wipe                          Keep eeprom/params on boot (not recommended for fresh setup)
  --sim-args "<args>"                Extra args forwarded to ArduSub binary (-A)
  -h, --help                         Show this help
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --frame)
      FRAME="${2:-}"
      shift 2
      ;;
    --instance)
      INSTANCE="${2:-}"
      shift 2
      ;;
    --qgc-host)
      QGC_HOST="${2:-}"
      shift 2
      ;;
    --qgc-port)
      QGC_PORT="${2:-}"
      QGC_PORT_WAS_SET="1"
      shift 2
      ;;
    --qgc-tcp-port)
      QGC_TCP_PORT="${2:-}"
      shift 2
      ;;
    --qgc-link)
      QGC_LINK_MODE="${2:-}"
      shift 2
      ;;
    --dual-qgc-link)
      DUAL_QGC_LINK="1"
      shift
      ;;
    --no-dual-qgc-link)
      DUAL_QGC_LINK="0"
      shift
      ;;
    --use-mavproxy)
      USE_MAVPROXY="1"
      shift
      ;;
    --force-clean)
      FORCE_CLEAN="1"
      shift
      ;;
    --sim-ip)
      SIM_IP="${2:-}"
      shift 2
      ;;
    --sim-port-in)
      SIM_PORT_IN="${2:-}"
      shift 2
      ;;
    --sim-port-out)
      SIM_PORT_OUT="${2:-}"
      shift 2
      ;;
    --dds)
      DDS_ENABLE="1"
      shift
      ;;
    --pilot-failsafe)
      PILOT_FAILSAFE_MODE="${2:-}"
      shift 2
      ;;
    --rebuild)
      NO_REBUILD="0"
      shift
      ;;
    --wipe)
      WIPE_EEPROM="1"
      shift
      ;;
    --no-wipe)
      WIPE_EEPROM="0"
      shift
      ;;
    --sim-args)
      EXTRA_SIM_ARGS="${2:-}"
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "[error] unknown option: $1" >&2
      usage
      exit 2
      ;;
  esac
done

case "${PILOT_FAILSAFE_MODE}" in
  disabled) PILOT_FAILSAFE_VALUE="0" ;;
  warn) PILOT_FAILSAFE_VALUE="1" ;;
  disarm) PILOT_FAILSAFE_VALUE="2" ;;
  *)
    echo "[error] invalid --pilot-failsafe: ${PILOT_FAILSAFE_MODE}" >&2
    echo "        use one of: disabled, warn, disarm" >&2
    exit 2
    ;;
esac

case "${QGC_LINK_MODE}" in
  udpclient|tcpclient)
    ;;
  *)
    echo "[error] invalid --qgc-link: ${QGC_LINK_MODE}" >&2
    echo "        use one of: udpclient, tcpclient" >&2
    exit 2
    ;;
esac

if [[ "${FRAME}" == "vectored" ]]; then
  if [[ "${AG_ALLOW_VECTORED_FRAME:-0}" != "1" ]]; then
    echo "[error] --frame vectored is disabled for this workspace."
    echo "        Reason: MuJoCo model uses 8-thruster layout and expects vectored_6dof mixing."
    echo "        Use: --frame vectored_6dof"
    echo "        Override (not recommended): AG_ALLOW_VECTORED_FRAME=1 ..."
    exit 2
  fi
  echo "[warn] --frame vectored override enabled via AG_ALLOW_VECTORED_FRAME=1"
fi

if [[ "${QGC_LINK_MODE}" == "tcpclient" && "${QGC_PORT_WAS_SET}" == "0" ]]; then
  QGC_PORT="${QGC_TCP_PORT}"
fi

if ! [[ "${QGC_PORT}" =~ ^[0-9]+$ ]]; then
  echo "[error] invalid --qgc-port: ${QGC_PORT}" >&2
  exit 2
fi

if ! [[ "${QGC_TCP_PORT}" =~ ^[0-9]+$ ]]; then
  echo "[error] invalid --qgc-tcp-port: ${QGC_TCP_PORT}" >&2
  exit 2
fi

if ! [[ "${SIM_PORT_IN}" =~ ^[0-9]+$ && "${SIM_PORT_OUT}" =~ ^[0-9]+$ ]]; then
  echo "[error] --sim-port-in and --sim-port-out must be numeric" >&2
  exit 2
fi

if [[ "${SIM_PORT_IN}" == "${SIM_PORT_OUT}" ]]; then
  echo "[error] --sim-port-in and --sim-port-out must be different" >&2
  exit 2
fi

if ! [[ -d "${ARDUPILOT_DIR}" ]]; then
  echo "[error] ArduPilot directory not found: ${ARDUPILOT_DIR}" >&2
  exit 1
fi

if [[ "${WIPE_EEPROM}" == "0" && -f "${ARDUPILOT_DIR}/eeprom.bin" ]]; then
  echo "[warn] --no-wipe requested while eeprom.bin exists; persistent params may override FS_PILOT_INPUT."
  echo "       If QGC still shows 'Lost manual control', restart with --wipe."
fi

if ! git -C "${ARDUPILOT_DIR}" rev-parse --is-inside-work-tree >/dev/null 2>&1; then
  echo "[error] ArduPilot path is not a git worktree: ${ARDUPILOT_DIR}" >&2
  exit 1
fi

if ! command -v python3 >/dev/null 2>&1; then
  echo "[error] python3 not found" >&2
  exit 1
fi

collect_existing_sitl_pids() {
  ps -eo pid=,args= | awk -v ap="${ARDUPILOT_DIR}" '
    index($0, ap "/build/sitl/bin/ardusub") > 0 {
      print $1
      next
    }
    index($0, "Tools/autotest/sim_vehicle.py") > 0 && index($0, "ArduSub") > 0 {
      print $1
    }
  '
}

kill_pid_list() {
  local pids="$1"
  [[ -z "${pids}" ]] && return 0
  # shellcheck disable=SC2086
  kill ${pids} 2>/dev/null || true
  sleep 0.6
  local alive=""
  # shellcheck disable=SC2086
  for pid in ${pids}; do
    if kill -0 "${pid}" 2>/dev/null; then
      alive+=" ${pid}"
    fi
  done
  if [[ -n "${alive}" ]]; then
    # shellcheck disable=SC2086
    kill -9 ${alive} 2>/dev/null || true
  fi
}

EXISTING_SITL_PIDS="$(collect_existing_sitl_pids | xargs)"
if [[ -n "${EXISTING_SITL_PIDS}" ]]; then
  if [[ "${FORCE_CLEAN}" == "1" ]]; then
    echo "[run] --force-clean: stopping existing ArduSub/SITL pids:${EXISTING_SITL_PIDS}"
    kill_pid_list "${EXISTING_SITL_PIDS}"
  else
    echo "[error] existing ArduSub/SITL process detected:${EXISTING_SITL_PIDS}" >&2
    echo "        Stop old processes first (example):" >&2
    echo "          pkill -f 'Tools/autotest/sim_vehicle.py -v ArduSub'" >&2
    echo "          pkill -f '/build/sitl/bin/ardusub'" >&2
    echo "        Or rerun with --force-clean." >&2
    exit 1
  fi
fi

SIM_ARGS="--serial0 ${QGC_LINK_MODE}:${QGC_HOST}:${QGC_PORT}"
if [[ "${DUAL_QGC_LINK}" == "1" ]]; then
  if [[ "${QGC_LINK_MODE}" == "udpclient" ]]; then
    # When primary is UDP out to QGC, expose a TCP server so QGC can
    # optionally connect via TCP without requiring QGC to run a TCP server.
    SIM_ARGS+=" --serial1 tcp:${QGC_TCP_PORT}"
  else
    SIM_ARGS+=" --serial1 udpclient:${QGC_HOST}:14550"
  fi
fi
SIM_ARGS+=" --sim-address ${SIM_IP}"
SIM_ARGS+=" --sim-port-in ${SIM_PORT_IN}"
SIM_ARGS+=" --sim-port-out ${SIM_PORT_OUT}"
if [[ -n "${EXTRA_SIM_ARGS}" ]]; then
  SIM_ARGS="${SIM_ARGS} ${EXTRA_SIM_ARGS}"
fi

NO_REBUILD_ARG=""
if [[ "${NO_REBUILD}" == "1" ]]; then
  NO_REBUILD_ARG="--no-rebuild"
fi

WIPE_ARG=""
if [[ "${WIPE_EEPROM}" == "1" ]]; then
  WIPE_ARG="-w"
fi

MAVPROXY_ARG="--no-mavproxy"
if [[ "${USE_MAVPROXY}" == "1" ]]; then
  MAVPROXY_ARG=""
fi

echo "[run] ArduSub JSON SITL"
echo "  frame      : ${FRAME}"
echo "  instance   : ${INSTANCE}"
echo "  qgc host   : ${QGC_HOST}"
echo "  qgc link   : ${QGC_LINK_MODE}"
if [[ "${DUAL_QGC_LINK}" == "1" ]]; then
  if [[ "${QGC_LINK_MODE}" == "udpclient" ]]; then
    echo "  qgc links  : --serial0 udpclient:${QGC_HOST}:${QGC_PORT}"
    echo "               --serial1 tcp:${QGC_TCP_PORT} (server)"
  else
    echo "  qgc links  : --serial0 tcpclient:${QGC_HOST}:${QGC_PORT}"
    echo "               --serial1 udpclient:${QGC_HOST}:14550"
  fi
else
  echo "  qgc target : ${QGC_LINK_MODE}://${QGC_HOST}:${QGC_PORT}"
fi
echo "  dds_enable : ${DDS_ENABLE}"
echo "  fs_pilot   : ${PILOT_FAILSAFE_MODE} (${PILOT_FAILSAFE_VALUE})"
echo "  wipe-eeprom: ${WIPE_EEPROM} (1=clean)"
echo "  qgc tcp    : ${QGC_HOST}:${QGC_TCP_PORT}"
echo "  sitl ip    : ${SIM_IP}"
echo "  sitl in/out: ${SIM_IP}:${SIM_PORT_IN} (simulator->ArduPilot, JSON state) / ${SIM_IP}:${SIM_PORT_OUT} (ArduPilot->simulator, servo PWM)"
echo "  sitl tune  : ARMING_CHECK=0, EK3_IMU_MASK=1, INS_USE2/3=0, COMPASS_USE2/3=0, FS_GCS_ENABLE=0"
echo "  sim args   : ${SIM_ARGS}"
echo
echo "[note] Start MuJoCo bridge in another terminal:"
echo "  cd ${SCRIPT_DIR}/../mujoco/uuv_mujoco/v2.2"
echo "  ./launch_competition_sim.sh --sitl --images"
echo

cd "${ARDUPILOT_DIR}"
python3 Tools/autotest/sim_vehicle.py \
  -v ArduSub \
  -f "${FRAME}" \
  --model JSON \
  --instance "${INSTANCE}" \
  ${MAVPROXY_ARG} \
  ${NO_REBUILD_ARG} \
  ${WIPE_ARG} \
  -P "DDS_ENABLE=${DDS_ENABLE}" \
  -P "FS_PILOT_INPUT=${PILOT_FAILSAFE_VALUE}" \
  -P "FS_GCS_ENABLE=0" \
  -P "ARMING_CHECK=0" \
  -P "EK3_IMU_MASK=1" \
  -P "INS_USE2=0" \
  -P "INS_USE3=0" \
  -P "COMPASS_USE2=0" \
  -P "COMPASS_USE3=0" \
  -A "${SIM_ARGS}"
