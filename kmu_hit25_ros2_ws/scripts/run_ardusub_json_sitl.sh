#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
WS_DIR=$(cd "${SCRIPT_DIR}/.." && pwd)
ARDUPILOT_DIR="${WS_DIR}/ardupilot"
ROOT_DIR=$(cd "${WS_DIR}/.." && pwd)
CHECKER_SCRIPT="${ROOT_DIR}/scripts/check_sitl_manual_input.py"

FRAME="vectored_6dof"
SITL_LOCATION="RATBeach"
INSTANCE="0"
QGC_PORT="14550"
QGC_MAVROS_PORT="14551"
QGC_HOST="127.0.0.1"
QGC_TCP_PORT="5773"
MAV_GCS_SYSID="1"
MAV_GCS_SYSID_HI="255"
SIM_IP="127.0.0.1"
SIM_PORT_IN="9003"
SIM_PORT_OUT="9002"
MUJOCO_MAVLINK_ENABLE="0"
MUJOCO_MAVLINK_PORT="14660"
NO_REBUILD="1"
WIPE_EEPROM="1"
DDS_ENABLE="0"
EXTRA_SIM_ARGS=""
PILOT_FAILSAFE_MODE="disabled"
FORCE_CLEAN="0"
FAST_RESPONSE="1"
RC5_REVERSED="0"
RC6_REVERSED="0"
RC4_REVERSED="0"
RC3_REVERSED="0"
RCMAP_FORWARD="5"
RCMAP_LATERAL="6"
RCMAP_ROLL="1"
RCMAP_PITCH="2"
RCMAP_THROTTLE="3"
RCMAP_YAW="4"
LIST_RCMAP="0"
FAST_SCHED_LOOP_RATE="30"
FAST_RC_SPEED="60"
FAST_JS_GAIN_DEFAULT="1.0"
FAST_JS_GAIN_STEPS="1"
FAST_PILOT_SPEED="300"
FAST_PILOT_SPEED_UP="300"
FAST_PILOT_ACCEL_Z="300"
FAST_PILOT_THR_FILT="0"
SERVO1_REVERSED="1"
SERVO2_REVERSED="1"
SERVO3_REVERSED="1"
SERVO4_REVERSED="1"
SERVO5_REVERSED="0"
SERVO6_REVERSED="0"
SERVO7_REVERSED="0"
SERVO8_REVERSED="0"
STABLE_ATC_INPUT_TC="0.35"
STABLE_ATC_ANG_RLL_P="1.20"
STABLE_ATC_ANG_PIT_P="1.20"
STABLE_ATC_RAT_RLL_P="0.050"
STABLE_ATC_RAT_RLL_I="0.050"
STABLE_ATC_RAT_RLL_D="0.002"
STABLE_ATC_RAT_PIT_P="0.050"
STABLE_ATC_RAT_PIT_I="0.050"
STABLE_ATC_RAT_PIT_D="0.002"
STABLE_ATC_RAT_RLL_IMAX="0.20"
STABLE_ATC_RAT_PIT_IMAX="0.20"
USE_MAVPROXY=1
MAVPROXY_DELAY_SEC="8"
SITL_BG_MODE="${AG_SITL_BG_MODE:-0}"
READINESS_TIMEOUT_SEC="${SITL_READINESS_TIMEOUT_SEC:-30}"
SITL_LOG_PATH_HINT="${AG_SITL_LOG_PATH:-<stdout/stderr>}"
SIM_VEHICLE_CMD_SUMMARY=""
READINESS_REQUIRE_HEARTBEAT="${SITL_READINESS_REQUIRE_HEARTBEAT:-0}"

usage() {
  cat <<'EOF'
Usage:
  ./scripts/run_ardusub_json_sitl.sh [options]

Options:
  --frame <vectored|vectored_6dof>   ArduSub frame (default: vectored_6dof)
  --sitl-location <name>              SITL location set (default: RATBeach)
  --instance <N>                     SITL instance index (default: 0)
  --qgc-host <ip>                    QGC UDP host (default: 127.0.0.1)
  --qgc-port <port>                  QGC UDP port (default: 14550)
  --mavros-port <port>               MAVROS UDP port (default: 14551)
  --sim-ip <ip>                      Sim address for ArduPilot JSON (default: 127.0.0.1)
  --sim-port-in <port>                ArduPilot JSON input port; simulator JSON packets must arrive here
                                        (default: 9003)
  --sim-port-out <port>               ArduPilot JSON output port; SITL servo packets are sent here
                                        (default: 9002)
  --mujoco-mavlink                    Enable MAVLink servo output for MuJoCo (default: disabled)
  --no-mujoco-mavlink                 Disable MuJoCo MAVLink output link
  --mujoco-mavlink-port <port>        MAVLink UDP output port for MuJoCo SERVO_OUTPUT_RAW consumer (default: 14660)
  --fast-response                     Apply low-latency SITL tune (default: enabled)
  --no-fast-response                  Disable low-latency SITL tune
  --lateral-reversed                  Reverse pilot lateral axis (default: enabled)
  --lateral-normal                    Normal pilot lateral axis
  --forward-reversed                  Reverse pilot forward axis
  --forward-normal                    Normal pilot forward axis (default)
  --yaw-reversed                      Reverse pilot yaw axis
  --yaw-normal                        Normal pilot yaw axis (default)
  --throttle-reversed                 Reverse pilot throttle axis (default)
  --throttle-normal                   Normal pilot throttle axis
  --rcmap-forward <1-16>             ArduPilot RCMAP_FORWARD channel (default: 5)
  --rcmap-lateral <1-16>             ArduPilot RCMAP_LATERAL channel (default: 6)
  --rcmap-roll <1-16>                ArduPilot RCMAP_ROLL channel (default: 1)
  --rcmap-pitch <1-16>               ArduPilot RCMAP_PITCH channel (default: 2)
  --rcmap-throttle <1-16>            ArduPilot RCMAP_THROTTLE channel (default: 3)
  --rcmap-yaw <1-16>                 ArduPilot RCMAP_YAW channel (default: 4)
  --mav-gcs-sysid <1-255>            GCS system id used for manual control/failsafe (default: 1)
  --mav-gcs-sysid-hi <0-255>         Upper GCS sysid for range acceptance (default: 255)
  --list-rcmap                       Print effective RCMAP values and exit
  --force-clean                       Kill existing ArduSub/SITL processes from this workspace before launch
  --dds                              Enable DDS (default: disabled)
  --pilot-failsafe <disabled|warn|disarm>
                                     FS_PILOT_INPUT for SITL (default: disabled)
  --rebuild                          Rebuild before launch (default: no rebuild)
  --wipe                             Force wipe eeprom/params on boot (enabled by default)
  --no-wipe                          Keep eeprom/params on boot (not recommended for fresh setup)
  --sim-args "<args>"                Extra args forwarded to ArduSub binary (-A)
  --mavproxy                         Use MAVProxy (default: enabled)
  --no-mavproxy                      Skip MAVProxy wrapper
  -h, --help                         Show this help
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --frame)
      FRAME="${2:-}"
      shift 2
      ;;
    --sitl-location)
      SITL_LOCATION="${2:-}"
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
      shift 2
      ;;
    --mavros-port)
      QGC_MAVROS_PORT="${2:-}"
      shift 2
      ;;
    --force-clean)
      FORCE_CLEAN="1"
      shift
      ;;
    --fast-response)
      FAST_RESPONSE="1"
      shift
      ;;
    --no-fast-response)
      FAST_RESPONSE="0"
      shift
      ;;
    --lateral-reversed)
      RC6_REVERSED="1"
      shift
      ;;
    --lateral-normal)
      RC6_REVERSED="0"
      shift
      ;;
    --forward-reversed)
      RC5_REVERSED="1"
      shift
      ;;
    --forward-normal)
      RC5_REVERSED="0"
      shift
      ;;
    --yaw-reversed)
      RC4_REVERSED="1"
      shift
      ;;
    --yaw-normal)
      RC4_REVERSED="0"
      shift
      ;;
    --throttle-reversed)
      RC3_REVERSED="1"
      shift
      ;;
    --throttle-normal)
      RC3_REVERSED="0"
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
    --mav-gcs-sysid)
      MAV_GCS_SYSID="${2:-}"
      shift 2
      ;;
    --mav-gcs-sysid-hi)
      MAV_GCS_SYSID_HI="${2:-}"
      shift 2
      ;;
    --list-rcmap)
      LIST_RCMAP="1"
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
    --mujoco-mavlink)
      MUJOCO_MAVLINK_ENABLE="1"
      shift
      ;;
    --no-mujoco-mavlink)
      MUJOCO_MAVLINK_ENABLE="0"
      shift
      ;;
    --mujoco-mavlink-port)
      MUJOCO_MAVLINK_PORT="${2:-}"
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
    --mavproxy)
      USE_MAVPROXY=1
      shift
      ;;
    --no-mavproxy)
      USE_MAVPROXY=0
      shift
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

validate_rcmap() {
  local name="$1"
  local value="$2"
  if ! [[ "${value}" =~ ^[0-9]+$ ]]; then
    echo "[error] --${name} must be integer 1~16 (got: ${value})" >&2
    exit 2
  fi
  if (( value < 1 || value > 16 )); then
    echo "[error] --${name} must be integer 1~16 (got: ${value})" >&2
    exit 2
  fi
}

validate_rcmap "rcmap-forward" "${RCMAP_FORWARD}"
validate_rcmap "rcmap-lateral" "${RCMAP_LATERAL}"
validate_rcmap "rcmap-roll" "${RCMAP_ROLL}"
validate_rcmap "rcmap-pitch" "${RCMAP_PITCH}"
validate_rcmap "rcmap-throttle" "${RCMAP_THROTTLE}"
validate_rcmap "rcmap-yaw" "${RCMAP_YAW}"
validate_gcs_sysid() {
  local name="$1"
  local value="$2"
  local min="$3"
  local max="$4"
  if ! [[ "${value}" =~ ^[0-9]+$ ]]; then
    echo "[error] --${name} must be integer ${min}~${max} (got: ${value})" >&2
    exit 2
  fi
  if (( value < min || value > max )); then
    echo "[error] --${name} must be integer ${min}~${max} (got: ${value})" >&2
    exit 2
  fi
}
validate_gcs_sysid "mav-gcs-sysid" "${MAV_GCS_SYSID}" 1 255
validate_gcs_sysid "mav-gcs-sysid-hi" "${MAV_GCS_SYSID_HI}" 0 255

if (( LIST_RCMAP )); then
  echo "[run] ArduSub RCMAP:"
  echo "  RCMAP_ROLL=${RCMAP_ROLL}"
  echo "  RCMAP_PITCH=${RCMAP_PITCH}"
  echo "  RCMAP_THROTTLE=${RCMAP_THROTTLE}"
  echo "  RCMAP_YAW=${RCMAP_YAW}"
  echo "  RCMAP_FORWARD=${RCMAP_FORWARD}"
  echo "  RCMAP_LATERAL=${RCMAP_LATERAL}"
  exit 0
fi

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

if ! [[ "${QGC_PORT}" =~ ^[0-9]+$ ]]; then
  echo "[error] invalid --qgc-port: ${QGC_PORT}" >&2
  exit 2
fi
if ! [[ "${QGC_MAVROS_PORT}" =~ ^[0-9]+$ ]]; then
  echo "[error] invalid --mavros-port: ${QGC_MAVROS_PORT}" >&2
  exit 2
fi

if ! [[ "${SIM_PORT_IN}" =~ ^[0-9]+$ && "${SIM_PORT_OUT}" =~ ^[0-9]+$ ]]; then
  echo "[error] --sim-port-in and --sim-port-out must be numeric" >&2
  exit 2
fi
if ! [[ "${MUJOCO_MAVLINK_PORT}" =~ ^[0-9]+$ ]]; then
  echo "[error] --mujoco-mavlink-port must be numeric" >&2
  exit 2
fi

for rev_val in "${RC3_REVERSED}" "${RC4_REVERSED}" "${RC5_REVERSED}" "${RC6_REVERSED}"; do
  if [[ "${rev_val}" != "0" && "${rev_val}" != "1" ]]; then
    echo "[error] axis reverse flags must be 0 or 1" >&2
    exit 2
  fi
done

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

check_port_listen() {
  local port="$1"
  ss -lnt 2>/dev/null | grep -qE "[:.]${port}[[:space:]]"
}

check_port_active() {
  local port="$1"
  ss -nt 2>/dev/null | grep -qE "[:.]${port}[[:space:]]"
}

check_sitl_heartbeat_14551() {
  if [[ -f "${CHECKER_SCRIPT}" ]]; then
    python3 "${CHECKER_SCRIPT}" \
      --mode udp \
      --listen "${QGC_MAVROS_PORT}" \
      --hb-timeout 1.2 \
      --timeout 1.2 \
      --heartbeat-only >/dev/null 2>&1
    return $?
  fi

  python3 - <<'PY' >/dev/null 2>&1
import time
from pymavlink import mavutil

conn = mavutil.mavlink_connection("udpin:0.0.0.0:14551", source_system=253, source_component=190)
hb = conn.wait_heartbeat(timeout=1.2)
raise SystemExit(0 if hb is not None else 1)
PY
}

print_readiness_diagnostics() {
  echo "[ready] readiness diagnostics begin" >&2
  echo "[ready] sitl log path: ${SITL_LOG_PATH_HINT}" >&2
  echo "[ready] command summary: ${SIM_VEHICLE_CMD_SUMMARY}" >&2
  echo "[ready] processes (sim_vehicle/ardusub/mavproxy):" >&2
  pgrep -af "Tools/autotest/sim_vehicle.py|/build/sitl/bin/ardusub|mavproxy.py" >&2 || true
  echo "[ready] listening ports (5760/${QGC_TCP_PORT}/14551/14660):" >&2
  ss -lntup 2>/dev/null | grep -E "5760|${QGC_TCP_PORT}|14551|14660" >&2 || true
  echo "[ready] active tcp endpoints (5760/${QGC_TCP_PORT}):" >&2
  ss -ntup 2>/dev/null | grep -E "5760|${QGC_TCP_PORT}" >&2 || true
  echo "[ready] quick checks:" >&2
  echo "  tail -n 120 ${SITL_LOG_PATH_HINT}" >&2
  echo "  ss -lntup | grep -E '5760|${QGC_TCP_PORT}|14551|14660'" >&2
  echo "  ss -ntup | grep -E '5760|${QGC_TCP_PORT}'" >&2
  echo "  pgrep -af 'sim_vehicle.py|mavproxy.py|/build/sitl/bin/ardusub'" >&2
  echo "[ready] readiness diagnostics end" >&2
}

wait_sitl_readiness() {
  local deadline=$((SECONDS + READINESS_TIMEOUT_SEC))
  local have_ardu=0
  local have_5760=0
  local have_qgc_tcp=0
  local have_hb=0
  while (( SECONDS < deadline )); do
    have_ardu=0
    have_5760=0
    have_qgc_tcp=0
    have_hb=0

    if pgrep -f "${ARDUPILOT_DIR}/build/sitl/bin/ardusub" >/dev/null 2>&1; then
      have_ardu=1
    fi
    if check_port_listen 5760 || check_port_active 5760; then
      have_5760=1
    fi
    if check_port_listen "${QGC_TCP_PORT}"; then
      have_qgc_tcp=1
    fi
    if (( have_ardu == 1 && have_5760 == 1 && have_qgc_tcp == 1 && READINESS_REQUIRE_HEARTBEAT == 1 )); then
      if check_sitl_heartbeat_14551; then
        have_hb=1
      fi
    elif (( READINESS_REQUIRE_HEARTBEAT == 0 )); then
      have_hb=1
    fi

    if (( have_ardu == 1 && have_5760 == 1 && have_qgc_tcp == 1 && have_hb == 1 )); then
      if (( READINESS_REQUIRE_HEARTBEAT == 1 )); then
        echo "[ready] SITL readiness passed (ardusub+5760+${QGC_TCP_PORT}+heartbeat@14551)." >&2
      else
        echo "[ready] SITL readiness passed (ardusub+5760+${QGC_TCP_PORT}; heartbeat check deferred)." >&2
      fi
      return 0
    fi
    sleep 0.5
  done

  echo "[error] SITL readiness failed (timeout=${READINESS_TIMEOUT_SEC}s)." >&2
  echo "[error] readiness state: ardupilot=${have_ardu}, tcp5760=${have_5760}, tcp${QGC_TCP_PORT}=${have_qgc_tcp}, hb14551=${have_hb}(required=${READINESS_REQUIRE_HEARTBEAT})" >&2
  print_readiness_diagnostics
  return 1
}

collect_existing_sitl_pids() {
  {
    pgrep -f "${ARDUPILOT_DIR}/build/sitl/bin/ardusub" || true
    pgrep -f "Tools/autotest/sim_vehicle.py.*ArduSub" || true
  } | sort -u
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

MAVPROXY_OUT_ARGS=(
  "--out=udp:${QGC_HOST}:${QGC_PORT}"
  "--out=udp:${QGC_HOST}:${QGC_MAVROS_PORT}"
  "--out=tcpin:0.0.0.0:${QGC_TCP_PORT}"
)
if [[ "${MUJOCO_MAVLINK_ENABLE}" == "1" ]]; then
  MAVPROXY_OUT_ARGS+=("--out=udp:${SIM_IP}:${MUJOCO_MAVLINK_PORT}")
fi

SITL_ARGS="--sim-address ${SIM_IP} --sim-port-in ${SIM_PORT_IN} --sim-port-out ${SIM_PORT_OUT}"
if [[ -n "${EXTRA_SIM_ARGS}" ]]; then
  SITL_ARGS="${SITL_ARGS} ${EXTRA_SIM_ARGS}"
fi

NO_REBUILD_ARG=""
if [[ "${NO_REBUILD}" == "1" ]]; then
  NO_REBUILD_ARG="--no-rebuild"
fi

WIPE_ARG=""
if [[ "${WIPE_EEPROM}" == "1" ]]; then
  WIPE_ARG="-w"
fi

MAVPROXY_ARG=""
if (( USE_MAVPROXY == 0 )); then
  MAVPROXY_ARG="--no-mavproxy"
fi

MAVPROXY_UI_ARGS=(--console --map)
MAVPROXY_EXTRA_ARGS=()
if (( USE_MAVPROXY == 1 )) && [[ "${SITL_BG_MODE}" == "1" ]]; then
  MAVPROXY_UI_ARGS=()
  MAVPROXY_EXTRA_ARGS=(--mavproxy-args="--nowait --daemon")
fi

echo "[run] ArduSub JSON SITL"
echo "  frame      : ${FRAME}"
echo "  instance   : ${INSTANCE}"
echo "  qgc host   : ${QGC_HOST}"
echo "  qgc link   : udp://${QGC_HOST}:${QGC_PORT}"
echo "  mavros link: udp://${QGC_HOST}:${QGC_MAVROS_PORT}"
if [[ "${MUJOCO_MAVLINK_ENABLE}" == "1" ]]; then
  echo "  qgc route  : --out=udp:${QGC_HOST}:${QGC_PORT} (QGC)"
  echo "               --out=udp:${QGC_HOST}:${QGC_MAVROS_PORT} (MAVROS)"
  echo "               --out=tcpin:0.0.0.0:${QGC_TCP_PORT} (QGC TCP)"
else
  echo "  qgc route  : --out=udp:${QGC_HOST}:${QGC_PORT} (QGC)"
  echo "               --out=udp:${QGC_HOST}:${QGC_MAVROS_PORT} (MAVROS)"
  echo "               --out=tcpin:0.0.0.0:${QGC_TCP_PORT} (QGC TCP)"
fi
echo "  dds_enable : ${DDS_ENABLE}"
echo "  fs_pilot   : ${PILOT_FAILSAFE_MODE} (${PILOT_FAILSAFE_VALUE})"
echo "  wipe-eeprom: ${WIPE_EEPROM} (1=clean)"
echo "  fast tune  : ${FAST_RESPONSE} (1=enabled)"
echo "  pilot rev  : fwd=${RC5_REVERSED} lat=${RC6_REVERSED} yaw=${RC4_REVERSED} thr=${RC3_REVERSED}"
echo "  servo rev  : s1=${SERVO1_REVERSED} s2=${SERVO2_REVERSED} s3=${SERVO3_REVERSED} s4=${SERVO4_REVERSED} s5=${SERVO5_REVERSED} s6=${SERVO6_REVERSED} s7=${SERVO7_REVERSED} s8=${SERVO8_REVERSED}"
echo "  rcmap      : roll=${RCMAP_ROLL}, pitch=${RCMAP_PITCH}, thr=${RCMAP_THROTTLE}, yaw=${RCMAP_YAW}, fwd=${RCMAP_FORWARD}, lat=${RCMAP_LATERAL}"
echo "  sitl ip    : ${SIM_IP}"
echo "  sitl in/out: ${SIM_IP}:${SIM_PORT_IN} (simulator->ArduPilot, JSON state) / ${SIM_IP}:${SIM_PORT_OUT} (ArduPilot->simulator, servo PWM)"
if [[ "${MUJOCO_MAVLINK_ENABLE}" == "1" ]]; then
  echo "  mavlink out: --out=udp:${SIM_IP}:${MUJOCO_MAVLINK_PORT} (MuJoCo SERVO_OUTPUT_RAW consumer)"
else
  echo "  mavlink out: disabled for MuJoCo"
fi
echo "  sitl tune  : minimal (sim_vehicle defaults 유지, MAV_GCS_SYSID 범위만 지정)"
echo "  mavproxy out args: ${MAVPROXY_OUT_ARGS[*]}"
if [[ "${SITL_BG_MODE}" == "1" ]]; then
  echo "  mavproxy mode: background (--nowait --daemon)"
else
  echo "  mavproxy mode: interactive (--console --map)"
fi
echo "  sitl args  : ${SITL_ARGS}"
echo "  mav gcs id : sysid=${MAV_GCS_SYSID}, hi=${MAV_GCS_SYSID_HI}"
echo
echo "[note] Start MuJoCo bridge in another terminal:"
echo "  cd ${SCRIPT_DIR}/../mujoco/uuv_mujoco/v2.2"
echo "  ./launch_competition_sim.sh --sitl --images"
echo

cd "${ARDUPILOT_DIR}"

SIM_VEHICLE_CMD=(
  python3 Tools/autotest/sim_vehicle.py
  -v ArduSub
  -f "${FRAME}"
  -L "${SITL_LOCATION}"
  -d "${MAVPROXY_DELAY_SEC}"
  --model JSON
  "${MAVPROXY_EXTRA_ARGS[@]}"
  "${MAVPROXY_UI_ARGS[@]}"
  --instance "${INSTANCE}"
  "${MAVPROXY_OUT_ARGS[@]}"
)

if [[ -n "${MAVPROXY_ARG}" ]]; then
  SIM_VEHICLE_CMD+=("${MAVPROXY_ARG}")
fi
if [[ -n "${NO_REBUILD_ARG}" ]]; then
  SIM_VEHICLE_CMD+=("${NO_REBUILD_ARG}")
fi
if [[ -n "${WIPE_ARG}" ]]; then
  SIM_VEHICLE_CMD+=("${WIPE_ARG}")
fi

SIM_VEHICLE_CMD+=(
  -P "MAV_GCS_SYSID=${MAV_GCS_SYSID}"
  -P "MAV_GCS_SYSID_HI=${MAV_GCS_SYSID_HI}"
  -A "${SITL_ARGS}"
)

SIM_VEHICLE_CMD_SUMMARY="$(printf '%q ' "${SIM_VEHICLE_CMD[@]}")"

if [[ "${SITL_BG_MODE}" == "1" && "${USE_MAVPROXY}" == "1" ]]; then
  "${SIM_VEHICLE_CMD[@]}" &
  SIM_VEHICLE_PID=$!
  sleep 0.3
  if ! kill -0 "${SIM_VEHICLE_PID}" 2>/dev/null; then
    echo "[error] sim_vehicle process exited immediately." >&2
    print_readiness_diagnostics
    exit 1
  fi
  if ! wait_sitl_readiness; then
    kill "${SIM_VEHICLE_PID}" 2>/dev/null || true
    sleep 0.3
    kill -9 "${SIM_VEHICLE_PID}" 2>/dev/null || true
    exit 1
  fi
  exit 0
fi

exec "${SIM_VEHICLE_CMD[@]}"
