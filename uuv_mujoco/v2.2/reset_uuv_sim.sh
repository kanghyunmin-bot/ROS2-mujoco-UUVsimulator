#!/usr/bin/env bash
set -euo pipefail

# Reset helper for local SITL <-> MuJoCo stack.
# - Stops MuJoCo runtime, ArduPilot SITL, MAVProxy.
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
ARDUPILOT_DIR="${ARDUPILOT_DIR:-${WORKSPACE_DIR}/ardupilot}"

if [[ ! -d "$WORKSPACE_DIR" ]]; then
  echo "[reset] workspace directory not found: $WORKSPACE_DIR"
  echo "        Set WORKSPACE_DIR correctly."
  exit 1
fi

kill_pattern() {
  local label="$1"
  local pattern="$2"
  local pids
  pids="$(pgrep -f "$pattern" 2>/dev/null || true)"
  if [[ -z "${pids}" ]]; then
    echo "[reset] ${label}: no process"
    return 0
  fi
  echo "[reset] ${label}: stopping ${pids}"
  # shellcheck disable=SC2086
  kill ${pids} 2>/dev/null || true
  sleep 0.5
  # shellcheck disable=SC2086
  for pid in ${pids}; do
    if kill -0 "${pid}" 2>/dev/null; then
      kill -9 "${pid}" 2>/dev/null || true
    fi
  done
}

echo "[reset] workspace: ${WORKSPACE_DIR}"

kill_pattern "MuJoCo runtime" "run_urdf_full.py"
kill_pattern "Start wrapper" "start_sitl_mujoco_mj311.sh"
kill_pattern "Launch wrapper" "launch_uuv_sim.sh"
kill_pattern "sim_vehicle" "Tools/autotest/sim_vehicle.py"
kill_pattern "ArduSub SITL" "build/sitl/bin/ardusub"
kill_pattern "MAVProxy" "mavproxy.py"
kill_pattern "SITL serial0 keepalive" "sitl_serial0_keepalive"

if [[ "${SIM_ONLY}" -eq 0 ]]; then
  kill_pattern "MAVROS launch" "rov_start.launch.py"
  kill_pattern "mavros_node" "mavros_node"
  kill_pattern "joy2mavros" "joy2mavros"
  kill_pattern "vfr2atm_pressure" "vfr2atm_pressure"
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
  if command -v rg >/dev/null 2>&1; then
    lsof -nP -iUDP | rg '14550|14551|14660|14661|9002|9003' || true
  else
    lsof -nP -iUDP | grep -E '14550|14551|14660|14661|9002|9003' || true
  fi
else
  echo "[reset] lsof not found; skipping port check"
fi

echo "[reset] done"
