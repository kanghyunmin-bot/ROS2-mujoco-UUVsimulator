#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/../.." && pwd)"
COMPOSE_FILES=(-f "${SCRIPT_DIR}/docker-compose.yml")

export UUV_XAUTHORITY="${XAUTHORITY:-/dev/null}"

compose() {
  docker compose "${COMPOSE_FILES[@]}" "$@"
}

run_bash() {
  compose run --rm uuv-dev bash -lc "$1"
}

usage() {
  cat <<'EOF'
Usage: ./docker/ubuntu-dev/dev.sh COMMAND

Commands:
  build         Build the Ubuntu 22.04 / ROS 2 Humble development image
  install       Create the project Python environment and verify dependencies
  build-sitl    Build the ArduSub SITL executable
  build-ros     Build the ROS 2 workspace with bounded resources
  verify        Run the repository's full environment verifier
  doctor        Check GPU OpenGL, ROS 2, MuJoCo, and Python imports
  shell         Open an interactive development shell
  run           Start SITL + MuJoCo + ROS 2 with the graphical viewer
  run-headless  Start the same stack without a graphical viewer
  web           Start the browser control GUI on http://127.0.0.1:8878
  vla-real      Run the real 19-state policy adapter against the simulator (dry run)
  reset         Stop simulator processes using the repository reset script
  down          Stop compose services (keeps build caches)
EOF
}

case "${1:-}" in
  build)
    compose build uuv-dev
    ;;
  install)
    run_bash 'unset KMU26_AUV_DIR; ./setup/install_uuv_mujoco.sh --skip-apt --skip-ardupilot --venv-root /workspace/.venv'
    ;;
  build-sitl)
    run_bash 'cd /workspace/ardupilot && ./waf configure --board sitl && ./waf build --target bin/ardusub'
    ;;
  build-ros)
    run_bash 'cd /workspace/rospkg && KMU26_BUILD_JOBS="${KMU26_BUILD_JOBS:-2}" ./build_safe.sh --cmake-args -DBUILD_TESTING=OFF'
    ;;
  verify)
    run_bash './setup/04_verify_uuv_stack.sh'
    ;;
  doctor)
    run_bash 'nvidia-smi --query-gpu=name,driver_version --format=csv,noheader && glxinfo -B | sed -n "/OpenGL vendor/p;/OpenGL renderer/p;/OpenGL core profile version/p" && ros2 doctor --report >/tmp/ros2-doctor.txt && ros2 pkg prefix mavros && ros2 pkg prefix rviz2 && /workspace/.venv/bin/python -c "import mujoco, pymavlink, rclpy; print(\"MuJoCo\", mujoco.__version__); print(\"Python imports OK\")"'
    ;;
  shell)
    compose run --rm uuv-dev bash
    ;;
  run)
    run_bash 'cd /workspace/uuv_mujoco/current && ./start_sitl_mujoco_mj311.sh --ros2-real-pkg-compat'
    ;;
  run-headless)
    run_bash 'cd /workspace/uuv_mujoco/current && ./start_sitl_mujoco_mj311.sh --ros2-real-pkg-compat -- --headless'
    ;;
  web)
    if ! nvidia-smi -L >/dev/null 2>&1; then
      echo '[uuv-dev] NVIDIA driver unavailable. Starting the web GUI with CPU rendering.' >&2
      COMPOSE_FILES+=(-f "${SCRIPT_DIR}/docker-compose.software.yml")
    fi
    # The host owns GPU model serving; the GUI container owns ROS processes.
    if ! python3 - "$WORKSPACE_DIR" <<'PY'
import json, sys, urllib.request
try:
    data = json.load(urllib.request.urlopen('http://127.0.0.1:8882/api/status', timeout=1))
    assert data.get('service') == 'uuv-vla-gui' and data.get('workspace') == sys.argv[1]
except Exception:
    sys.exit(1)
PY
    then
      mkdir -p "$WORKSPACE_DIR/outputs/vla-gui"
      python3 "$WORKSPACE_DIR/tools/vla_gui/server.py" > "$WORKSPACE_DIR/outputs/vla-gui/manager.log" 2>&1 &
      vla_manager_pid=$!
      trap 'kill -TERM "$vla_manager_pid" 2>/dev/null || true' EXIT
    fi
    run_bash 'cd /workspace && ./run_control_gui.sh --web --host 127.0.0.1 --port 8878'
    ;;
  vla-real)
    reasoning_dir="${UUV_REASONING_DIR:-$WORKSPACE_DIR/../auv_vla_reasoning_19}"
    if [[ ! -f "$reasoning_dir/auv_vla_reasoning/mujoco.py" ]]; then
      echo "Set UUV_REASONING_DIR to the simulation-enabled auv_vla_reasoning checkout." >&2
      exit 1
    fi
    shift
    # The ROS adapter runs on CPU; only the external model server needs CUDA.
    exec docker run --rm --init --network host --ipc host --user "$(id -u):$(id -g)" \
      -e "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-42}" -e ROS_HOME=/tmp/ros -e HOME=/tmp \
      -v "$WORKSPACE_DIR:/workspace:ro" -v "$(realpath "$reasoning_dir"):/opt/vla-reasoning:ro" \
      --entrypoint /bin/bash "$(compose config --images uuv-dev)" -ec 'source /opt/ros/humble/setup.bash
        source /workspace/rospkg/install/setup.bash
        cd /opt/vla-reasoning
        exec python3 -m auv_vla_reasoning.mujoco --ros-args --params-file config/real.yaml \
          -p use_sim_time:=true "$@"' bash "$@"
    ;;
  reset)
    run_bash 'cd /workspace/uuv_mujoco/current && ./reset_uuv_sim.sh --with-qgc-stop'
    ;;
  down)
    compose down
    ;;
  -h|--help|help)
    usage
    ;;
  *)
    usage >&2
    exit 2
    ;;
esac
