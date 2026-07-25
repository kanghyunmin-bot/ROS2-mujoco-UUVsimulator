#!/usr/bin/env bash
set -euo pipefail

launch_pid=""

cleanup() {
  if [[ -n "${launch_pid}" ]] && kill -0 "${launch_pid}" 2>/dev/null; then
    # launch and all child ROS nodes run in their own session.  Terminate the
    # process group so a wrapper error cannot leave an RC controller orphaned.
    kill -INT -- "-${launch_pid}" 2>/dev/null || true
    wait "${launch_pid}" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

setsid ros2 launch auv_pinger_homing pinger_homing_test_tank.launch.py "$@" auto_select_top:=false &
launch_pid=$!

echo "[pinger] Waiting for the ten-second frequency scan..."
wait_for_candidate_topic() {
  local deadline=$((SECONDS + 30))
  while (( SECONDS < deadline )); do
    if ros2 topic list 2>/dev/null | grep -Fxq "/pinger_homing/frequency_candidates"; then
      return 0
    fi
    sleep 0.2
  done
  return 1
}

# `ros2 topic echo` exits immediately when its target has not yet been
# advertised.  The selector is launched in parallel, so wait for discovery
# first; only then wait for the ten-second scan result.
if ! wait_for_candidate_topic; then
  echo "[pinger] Frequency selector did not advertise its candidate topic within 30 seconds." >&2
  exit 1
fi

if ! timeout 30 ros2 topic echo --once /pinger_homing/frequency_candidates; then
  echo "[pinger] No frequency candidates arrived within 30 seconds." >&2
  exit 1
fi

while true; do
  printf '\n\n[Pinger] Candidates are ready. Enter 1-5 or a frequency in Hz, then press Enter:\n' >&2
  read -r selection
  if [[ "${selection}" =~ ^[0-9]+([.][0-9]+)?$ ]]; then
    ros2 topic pub --once /pinger_homing/manual_selection std_msgs/msg/String \
      "{data: '${selection}'}"
    break
  fi
  echo "Enter 1-5 or a frequency in Hz."
done

wait "${launch_pid}"
