#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/../.." && pwd)"
COMPOSE_FILE="${SCRIPT_DIR}/docker-compose.yml"

export UUV_XAUTHORITY="${XAUTHORITY:-/dev/null}"

compose() {
  docker compose -f "${COMPOSE_FILE}" "$@"
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
    run_bash 'cd /workspace && ./run_control_gui.sh --web --host 127.0.0.1 --port 8878'
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
