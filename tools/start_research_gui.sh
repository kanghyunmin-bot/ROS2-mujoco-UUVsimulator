#!/usr/bin/env bash
# Copyright (c) 2022-2026, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause
# Start the existing research GUI with the installed ROS Humble image.
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CONTAINER="${UUV_GUI_CONTAINER:-ut27-live-gui}"
PORT="${UUV_GUI_PORT:-18879}"
IMAGE="${UUV_GUI_IMAGE:-khm/ros2-mujoco-uuv:dev}"
ACTION="${1:-start}"
[[ "$PORT" =~ ^[0-9]+$ ]] && (( PORT >= 1024 && PORT <= 65535 )) || {
  echo 'UUV_GUI_PORT must be a port between 1024 and 65535.' >&2; exit 2;
}
case "$ACTION" in start|stop|status) ;; *) echo 'Usage: start_research_gui.sh [start|stop|status]' >&2; exit 2;; esac

if docker inspect "$CONTAINER" >/dev/null 2>&1; then
  mounted_root="$(docker inspect -f '{{range .Mounts}}{{if eq .Destination "/workspace"}}{{.Source}}{{end}}{{end}}' "$CONTAINER")"
  [[ "$mounted_root" == "$ROOT_DIR" ]] || { echo 'Container belongs to another workspace.' >&2; exit 1; }
  launch_command="$(docker inspect -f '{{json .Config.Cmd}}' "$CONTAINER")"
  [[ "$launch_command" == *"--port $PORT"* ]] || { echo 'Existing container uses a different port. Select another UUV_GUI_CONTAINER.' >&2; exit 1; }
  if [[ "$ACTION" == stop ]]; then docker stop -t 20 "$CONTAINER"; exit; fi
  if [[ "$ACTION" == status ]]; then
    docker inspect -f '{{.State.Status}}' "$CONTAINER"
    exit
  fi
  [[ "$(docker inspect -f '{{.State.Running}}' "$CONTAINER")" == true ]] || docker start "$CONTAINER" >/dev/null
else
  [[ "$ACTION" == start ]] || { echo 'GUI container is not running.'; exit; }
  docker image inspect "$IMAGE" >/dev/null
  renderer="${UUV_GUI_RENDERER:-auto}"
  if [[ "$renderer" == auto ]]; then
    renderer=cpu
    if command -v nvidia-smi >/dev/null && nvidia-smi -L >/dev/null 2>&1; then renderer=gpu; fi
  fi
  case "$renderer" in
    cpu) graphics=(-e LIBGL_ALWAYS_SOFTWARE=1 -e LP_NUM_THREADS=4);;
    gpu) graphics=(--gpus all -e NVIDIA_DRIVER_CAPABILITIES=all);;
    *) echo 'UUV_GUI_RENDERER must be auto, cpu, or gpu.' >&2; exit 2;;
  esac
  docker run -d --name "$CONTAINER" --init --network host --ipc host --workdir /workspace \
    -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}" -e ROS_LOCALHOST_ONLY=1 \
    -e RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}" \
    -e PYTHON_BIN=/workspace/.venv/bin/python -e UUV_GUI_MUJOCO_VIEWER=0 \
    -e UUV_MUJOCO_SKIP_FRESHNESS_CHECK=1 -e UUV_RUNTIME_PROFILE=low -e DISPLAY= \
    "${graphics[@]}" -v "$ROOT_DIR:/workspace" \
    -v ubuntu-dev_uuv-ardupilot-build:/workspace/ardupilot/build \
    -v ubuntu-dev_uuv-ccache:/home/robot/.ccache "$IMAGE" \
    bash -lc "./run_control_gui.sh --web --host 127.0.0.1 --port $PORT --sim-preset research_pool_distributed" >/dev/null
fi
for _ in {1..30}; do
  if curl --fail --silent --max-time 2 "http://127.0.0.1:$PORT/api/status" >/dev/null; then
    echo "Research GUI: http://127.0.0.1:$PORT/"
    echo 'Start the simulator with the GUI start button; ARM remains manual.'
    exit 0
  fi
  sleep 1
done
docker logs --tail 20 "$CONTAINER"
exit 1
