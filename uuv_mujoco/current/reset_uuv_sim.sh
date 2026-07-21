#!/usr/bin/env bash
set -euo pipefail

# Reset helper for local SITL <-> MuJoCo stack.
# - Stops MuJoCo runtime, ArduPilot SITL, Docker SITL, MAVProxy.
# - Keeps QGroundControl running so it can reconnect immediately.
# - Prints remaining listeners on key UDP ports.

WIPE_EEPROM=0
SIM_ONLY=0

usage() {
  cat <<'USAGE'
Usage: ./reset_uuv_sim.sh [options]

Options:
  --wipe-eeprom     Remove ardupilot/eeprom.bin to reset ArduSub params
  --sim-only        Stop only MuJoCo/SITL/MAVProxy; keep QGC and ROS/MAVROS nodes
  -h, --help        Show this help

Environment:
  WORKSPACE_DIR     Workspace root containing ardupilot/ (optional)
  ARDUPILOT_DIR     Explicit ArduPilot checkout path (optional)
  UUV_RESET_SKIP_DOCKER_STOP=1
                    Skip Docker compose cleanup (normally not needed)
USAGE
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --wipe-eeprom)
      WIPE_EEPROM=1
      shift
      ;;
    --sim-only)
      SIM_ONLY=1
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "[reset] unknown option: $1" >&2
      usage
      exit 2
      ;;
  esac
done

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CONTROLLER_PARITY_LOCK="${UUV_MUJOCO_CONTROLLER_PARITY_LOCK:-/tmp/uuv_mujoco_controller_parity.lock}"

lineage_pids() {
  local pid
  pid="$$"
  while [[ -n "$pid" && "$pid" =~ ^[0-9]+$ && "$pid" != "0" ]]; do
    printf '%s\n' "$pid"
    pid="$(ps -o ppid= -p "$pid" 2>/dev/null | tr -d '[:space:]' || true)"
  done
}

CURRENT_LINEAGE_PIDS="$(lineage_pids | tr '\n' ' ')"

lock_pid() {
  python3 - "$1" <<'PY'
import json
import sys
from pathlib import Path

try:
    data = json.loads(Path(sys.argv[1]).read_text(encoding="utf-8"))
    print(int(data.get("pid", 0)))
except Exception:
    print("")
PY
}

if [[ "${UUV_RESET_IGNORE_CONTROLLER_PARITY_LOCK:-0}" != "1" && -f "$CONTROLLER_PARITY_LOCK" ]]; then
  LOCK_OWNER_PID="$(lock_pid "$CONTROLLER_PARITY_LOCK")"
  if [[ -n "$LOCK_OWNER_PID" ]] && kill -0 "$LOCK_OWNER_PID" 2>/dev/null; then
    echo "[reset] refusing to reset while controller-parity run owns MuJoCo/SITL lock"
    echo "[reset] lock: ${CONTROLLER_PARITY_LOCK}"
    echo "[reset] owner pid: ${LOCK_OWNER_PID}"
    echo "[reset] set UUV_RESET_IGNORE_CONTROLLER_PARITY_LOCK=1 only for intentional manual override"
    exit 75
  fi
fi

resolve_default_workspace_dir() {
  local dir best score current_score
  if [[ -n "${ARDUPILOT_DIR:-}" && -d "${ARDUPILOT_DIR:-}" ]]; then
    cd "${ARDUPILOT_DIR}/.." && pwd
    return 0
  fi

  dir="$SCRIPT_DIR"
  best=""
  score=0
  while [[ -n "$dir" ]]; do
    current_score=0
    [[ -d "$dir/ardupilot_sub_stable" ]] && current_score=$((current_score + 20))
    [[ -d "$dir/ardupilot" ]] && current_score=$((current_score + 10))
    [[ -f "$dir/QGroundControl.AppImage" || -f "$dir/QGroundControl-x86_64.AppImage" || -d "$dir/QGroundControl.app" ]] && current_score=$((current_score + 4))
    [[ -d "$dir/kmu26_auv" ]] && current_score=$((current_score + 2))
    [[ -d "$dir/install" ]] && current_score=$((current_score + 1))
    if (( current_score > score )); then
      best="$dir"
      score=$current_score
    fi
    [[ "$dir" == "/" ]] && break
    dir="$(dirname "$dir")"
  done

  if [[ -n "$best" && "$score" -gt 0 ]]; then
    printf '%s\n' "$best"
    return 0
  fi

  cd "${SCRIPT_DIR}/../.." && pwd
}

DEFAULT_WORKSPACE_DIR="$(resolve_default_workspace_dir)"
WORKSPACE_DIR="${WORKSPACE_DIR:-$DEFAULT_WORKSPACE_DIR}"
if [[ -z "${ARDUPILOT_DIR:-}" && -d "${WORKSPACE_DIR}/ardupilot_sub_stable" ]]; then
  ARDUPILOT_DIR="${WORKSPACE_DIR}/ardupilot_sub_stable"
fi
ARDUPILOT_DIR="${ARDUPILOT_DIR:-${WORKSPACE_DIR}/ardupilot}"
COMPOSE_FILE="${UUV_ARDUSUB_DOCKER_COMPOSE:-${WORKSPACE_DIR}/docker/ardusub/docker-compose.yml}"

if [[ ! -d "$WORKSPACE_DIR" ]]; then
  echo "[reset] workspace directory not found: $WORKSPACE_DIR"
  echo "        Set WORKSPACE_DIR correctly."
  exit 1
fi

kill_pattern() {
  local label="$1"
  local pattern="$2"
  local pids filtered pid own skip
  pids="$(pgrep -f "$pattern" 2>/dev/null || true)"
  if [[ -z "${pids}" ]]; then
    echo "[reset] ${label}: no process"
    return 0
  fi
  filtered=""
  for pid in ${pids}; do
    skip=0
    for own in ${CURRENT_LINEAGE_PIDS}; do
      if [[ "$pid" == "$own" ]]; then
        skip=1
        break
      fi
    done
    if [[ "$skip" -eq 0 ]]; then
      filtered="${filtered} ${pid}"
    fi
  done
  if [[ -z "${filtered// /}" ]]; then
    echo "[reset] ${label}: no external process"
    return 0
  fi
  echo "[reset] ${label}: stopping ${filtered}"
  # shellcheck disable=SC2086
  kill ${filtered} 2>/dev/null || true
  sleep 0.5
  # shellcheck disable=SC2086
  for pid in ${filtered}; do
    if kill -0 "${pid}" 2>/dev/null; then
      kill -9 "${pid}" 2>/dev/null || true
    fi
  done
}

stop_docker_sitl() {
  if [[ "${UUV_RESET_SKIP_DOCKER_STOP:-0}" == "1" ]]; then
    echo "[reset] Docker SITL: skipped by UUV_RESET_SKIP_DOCKER_STOP=1"
    return 0
  fi
  if [[ ! -f "$COMPOSE_FILE" ]]; then
    echo "[reset] Docker SITL: compose file not found (${COMPOSE_FILE})"
    return 0
  fi
  if ! command -v docker >/dev/null 2>&1; then
    echo "[reset] Docker SITL: docker CLI not found"
    return 0
  fi
  if ! docker info >/dev/null 2>&1; then
    echo "[reset] Docker SITL: docker daemon not reachable"
    return 0
  fi
  echo "[reset] Docker SITL: stopping compose stack"
  (
    cd "$WORKSPACE_DIR"
    docker compose -f "$COMPOSE_FILE" down --remove-orphans
  ) >/dev/null 2>&1 || true
}

print_udp_local_port_owners() {
  lsof -nP -iUDP | python3 -c '
import re
import sys

ports = tuple(sys.argv[1:])
pattern = re.compile(r"(?:(?<=:)|(?<=\.))(" + "|".join(re.escape(port) for port in ports) + r")$")
lines = sys.stdin.read().splitlines()
if not lines:
    raise SystemExit(0)
header, *rows = lines
matched = []
for row in rows:
    if " UDP " not in row:
        continue
    endpoint = row.split(" UDP ", 1)[1].split()[0]
    local_endpoint = endpoint.split("->", 1)[0]
    if pattern.search(local_endpoint):
        matched.append(row)
if matched:
    print(header)
    print("\n".join(matched))
' "$@"
}

echo "[reset] workspace: ${WORKSPACE_DIR}"

kill_pattern "Docker start wrapper" "start_docker_sitl_mujoco_mj311.sh"
kill_pattern "MuJoCo runtime" "run_uuv_mujoco.py"
kill_pattern "MuJoCo legacy runtime" "run_urdf_full.py"
kill_pattern "Start wrapper" "start_sitl_mujoco_mj311.sh"
kill_pattern "Launch wrapper" "launch_uuv_sim.sh"
kill_pattern "sim_vehicle" "Tools/autotest/sim_vehicle.py"
kill_pattern "ArduSub SITL" "build/sitl/bin/ardusub"
kill_pattern "MAVProxy" "mavproxy.py"
kill_pattern "SITL serial0 keepalive" "sitl_serial0_keepalive"
stop_docker_sitl

if [[ "${SIM_ONLY}" -eq 0 ]]; then
  kill_pattern "MAVROS launch" "rov_start.launch.py"
  kill_pattern "mavros_node" "mavros_node"
  kill_pattern "joy2mavros" "joy2mavros"
  kill_pattern "vfr2atm_pressure" "vfr2atm_pressure"
  kill_pattern "dvl_to_twist_bridge" "dvl_to_twist_bridge"
  kill_pattern "pressure_to_depth_pose" "pressure_to_depth_pose"
  kill_pattern "robot_localization EKF" "/robot_localization/ekf_node"
  kill_pattern "odom2mavros" "odom2mavros"
  kill_pattern "dronecan2mavros_battery" "dronecan2mavros_battery"

fi

if [[ "${WIPE_EEPROM}" -eq 1 ]]; then
  if [[ -f "${ARDUPILOT_DIR}/eeprom.bin" ]]; then
    rm -f "${ARDUPILOT_DIR}/eeprom.bin"
    echo "[reset] removed ${ARDUPILOT_DIR}/eeprom.bin"
  else
    echo "[reset] eeprom.bin not found (${ARDUPILOT_DIR}/eeprom.bin)"
  fi
fi

echo "[reset] UDP listener check (14550/14551/14660/14661/9002/9003)"
if command -v lsof >/dev/null 2>&1; then
  print_udp_local_port_owners 14550 14551 14660 14661 9002 9003 || true
else
  echo "[reset] lsof not found; skipping port check"
fi

echo "[reset] done"
