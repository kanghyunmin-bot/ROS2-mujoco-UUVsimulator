#!/usr/bin/env bash
set -euo pipefail

KEEP_QGC=1

usage() {
  cat <<'EOF'
Usage:
  ./scripts/stack_reset.sh [options]

Options:
  --with-qgc-stop   Also stop QGroundControl AppImage process
  -h, --help        Show this help
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --with-qgc-stop)
      KEEP_QGC=0
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

echo "[reset] stopping ArduSub + MuJoCo stack processes..."

collect_pids() {
  local pattern="$1"
  pgrep -f "$pattern" || true
}

kill_pid_list() {
  local pids="$1"
  [[ -z "$pids" ]] && return 0
  # shellcheck disable=SC2086
  kill $pids 2>/dev/null || true
  sleep 0.6
  local alive=""
  # shellcheck disable=SC2086
  for pid in $pids; do
    if kill -0 "$pid" 2>/dev/null; then
      alive+=" $pid"
    fi
  done
  if [[ -n "$alive" ]]; then
    # shellcheck disable=SC2086
    kill -9 $alive 2>/dev/null || true
  fi
}

SITL_PIDS="$(collect_pids 'Tools/autotest/sim_vehicle.py -v ArduSub' | xargs)"
ARDUSUB_PIDS="$(collect_pids '/build/sitl/bin/ardusub' | xargs)"
MUJOCO_PIDS="$(collect_pids 'run_urdf_full.py --scene competition_scene.xml' | xargs)"
VIDEO_PIDS="$(collect_pids 'scripts/ros2_to_qgc_video.py' | xargs)"
MAVPROXY_PIDS="$(collect_pids 'mavproxy.py --master tcp:127.0.0.1:5760' | xargs)"

kill_pid_list "${SITL_PIDS}"
kill_pid_list "${ARDUSUB_PIDS}"
kill_pid_list "${MUJOCO_PIDS}"
kill_pid_list "${VIDEO_PIDS}"
kill_pid_list "${MAVPROXY_PIDS}"

if [[ "${KEEP_QGC}" == "0" ]]; then
  echo "[reset] stopping QGroundControl..."
  QGC_PIDS="$(collect_pids 'QGroundControl-x86_64.AppImage' | xargs)"
  kill_pid_list "${QGC_PIDS}"
fi

sleep 1
echo "[reset] remaining related processes:"
ps -ef | grep -E "sim_vehicle.py -v ArduSub|/build/sitl/bin/ardusub|run_urdf_full.py --scene competition_scene.xml|mavproxy.py|QGroundControl-x86_64.AppImage" | grep -v grep || true

echo "[reset] UDP ports in use (9002/9003/14550/14551/14660/5600):"
ss -uapn | grep -E "9002|9003|14550|14551|14660|5600" || true
