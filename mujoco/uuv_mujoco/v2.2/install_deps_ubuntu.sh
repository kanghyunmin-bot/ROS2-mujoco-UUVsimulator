#!/usr/bin/env bash
set -euo pipefail

# Ubuntu host bootstrap for UUV MuJoCo v2.1.
# - Installs system OpenGL/X11 runtime libs for MuJoCo viewer.
# - Installs Python dependencies from requirements.txt.
# - Optionally creates a local virtual environment.

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$ROOT_DIR"

USE_VENV=1
WITH_ROS2_DEV=0
PYTHON_BIN="python3"

usage() {
  cat <<USAGE
Usage: ./install_deps_ubuntu.sh [options]

Options:
  --no-venv           Install Python packages to current interpreter (no .venv)
  --with-ros2-dev     Also install ROS2 dev helper packages (colcon, rosdep)
  --python <bin>      Python executable to use (default: python3)
  -h, --help          Show this help
USAGE
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --no-venv)
      USE_VENV=0
      shift
      ;;
    --with-ros2-dev)
      WITH_ROS2_DEV=1
      shift
      ;;
    --python)
      PYTHON_BIN="$2"
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

echo "[deps] installing apt packages..."
sudo apt-get update
sudo apt-get install -y \
  python3 python3-pip python3-venv python3-dev \
  libgl1 libglfw3 libglew2.2 \
  libxrender1 libxrandr2 libxinerama1 libxcursor1 libxi6 \
  libosmesa6 mesa-utils

if [[ "$WITH_ROS2_DEV" -eq 1 ]]; then
  echo "[deps] installing ROS2 dev helper packages..."
  sudo apt-get install -y python3-colcon-common-extensions python3-rosdep
fi

if [[ "$USE_VENV" -eq 1 ]]; then
  echo "[deps] creating virtual environment: $ROOT_DIR/.venv"
  "$PYTHON_BIN" -m venv .venv
  # shellcheck source=/dev/null
  source .venv/bin/activate
fi

echo "[deps] installing Python packages..."
"$PYTHON_BIN" -m pip install --upgrade pip setuptools wheel
"$PYTHON_BIN" -m pip install -r requirements.txt

echo "[ok] installation complete"
if [[ "$USE_VENV" -eq 1 ]]; then
  echo "[next] activate with: source $ROOT_DIR/.venv/bin/activate"
fi
echo "[note] ROS2 bridge requires ROS2 environment sourced (e.g. /opt/ros/humble/setup.bash)."
