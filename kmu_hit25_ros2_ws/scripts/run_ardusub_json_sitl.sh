#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
WS_DIR=$(cd "${SCRIPT_DIR}/.." && pwd)
ARDUPILOT_DIR="${WS_DIR}/ardupilot"

FRAME="vectored"
INSTANCE="0"
QGC_PORT="14550"
NO_REBUILD="1"
WIPE_EEPROM="0"
DDS_ENABLE="0"
EXTRA_SIM_ARGS=""

usage() {
  cat <<'EOF'
Usage:
  ./scripts/run_ardusub_json_sitl.sh [options]

Options:
  --frame <vectored|vectored_6dof>   ArduSub frame (default: vectored)
  --instance <N>                     SITL instance index (default: 0)
  --qgc-port <port>                  MAVLink UDP target port for QGC (default: 14550)
  --dds                              Enable DDS (default: disabled)
  --rebuild                          Rebuild before launch (default: no rebuild)
  --wipe                             Wipe eeprom/params on boot
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
    --qgc-port)
      QGC_PORT="${2:-}"
      shift 2
      ;;
    --dds)
      DDS_ENABLE="1"
      shift
      ;;
    --rebuild)
      NO_REBUILD="0"
      shift
      ;;
    --wipe)
      WIPE_EEPROM="1"
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

if [[ ! -d "${ARDUPILOT_DIR}" ]]; then
  echo "[error] ArduPilot directory not found: ${ARDUPILOT_DIR}" >&2
  exit 1
fi

if ! git -C "${ARDUPILOT_DIR}" rev-parse --is-inside-work-tree >/dev/null 2>&1; then
  echo "[error] ArduPilot path is not a git worktree: ${ARDUPILOT_DIR}" >&2
  exit 1
fi

if ! command -v python3 >/dev/null 2>&1; then
  echo "[error] python3 not found" >&2
  exit 1
fi

SIM_ARGS="--serial0 udpclient:127.0.0.1:${QGC_PORT}"
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

echo "[run] ArduSub JSON SITL"
echo "  frame      : ${FRAME}"
echo "  instance   : ${INSTANCE}"
echo "  dds_enable : ${DDS_ENABLE}"
echo "  qgc udp    : 127.0.0.1:${QGC_PORT}"
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
  --no-mavproxy \
  ${NO_REBUILD_ARG} \
  ${WIPE_ARG} \
  -P "DDS_ENABLE=${DDS_ENABLE}" \
  -A "${SIM_ARGS}"
