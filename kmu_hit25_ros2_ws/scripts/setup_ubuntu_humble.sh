#!/usr/bin/env bash
set -euo pipefail

if [ "${EUID:-$(id -u)}" -eq 0 ]; then
  echo "Please run this script as a normal user (it uses sudo when needed)."
  exit 1
fi

ROS_DISTRO=${ROS_DISTRO:-humble}
UBUNTU_CODENAME=$(lsb_release -sc)
SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
WS_DIR=$(cd "${SCRIPT_DIR}/.." && pwd)

WITH_SITL=false
BUILD_SITL=false

usage() {
  cat <<'USAGE'
Usage: ./scripts/setup_ubuntu_humble.sh [options]

Options:
  --with-sitl     Install ArduPilot SITL prerequisites and clone ArduPilot
  --build-sitl    Also build ArduPilot SITL (implies --with-sitl)
  --help          Show this help
USAGE
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --with-sitl)
      WITH_SITL=true
      shift
      ;;
    --build-sitl)
      WITH_SITL=true
      BUILD_SITL=true
      shift
      ;;
    --help|-h)
      usage
      exit 0
      ;;
    *)
      echo "Unknown option: $1"
      usage
      exit 1
      ;;
  esac
done

sudo apt update
sudo apt install -y curl gnupg lsb-release software-properties-common

# Add ROS 2 apt repository if missing
if ! apt-cache policy | grep -q "packages.ros.org/ros2"; then
  sudo mkdir -p /etc/apt/keyrings
  curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    | sudo gpg --dearmor -o /etc/apt/keyrings/ros-archive-keyring.gpg
  echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu ${UBUNTU_CODENAME} main" \
    | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
fi

sudo apt update

# Core build tools + ROS 2 Humble + project deps
sudo apt install -y \
  build-essential \
  git \
  python3-pip \
  python3-dev \
  python3-venv \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-vcstool \
  ros-${ROS_DISTRO}-ros-base \
  ros-${ROS_DISTRO}-rviz2 \
  ros-${ROS_DISTRO}-robot-state-publisher \
  ros-${ROS_DISTRO}-tf-transformations \
  ros-${ROS_DISTRO}-joy \
  ros-${ROS_DISTRO}-rqt-image-view \
  ros-${ROS_DISTRO}-mavros \
  ros-${ROS_DISTRO}-mavros-msgs \
  ros-${ROS_DISTRO}-ros-gz \
  ros-${ROS_DISTRO}-ros-gz-bridge \
  ros-${ROS_DISTRO}-ros-gz-sim \
  ros-${ROS_DISTRO}-teleop-twist-keyboard \
  ros-${ROS_DISTRO}-cv-bridge \
  python3-opencv \
  python3-numpy \
  python3-pyzbar \
  python3-sounddevice \
  libzbar0 \
  libportaudio2 \
  portaudio19-dev \
  libasound2-dev

# rosdep init/update (safe to re-run)
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
  sudo rosdep init
fi
rosdep update

# Install workspace rosdeps (if src exists)
if [ -d "${WS_DIR}/src" ]; then
  # shellcheck disable=SC1091
  source "/opt/ros/${ROS_DISTRO}/setup.bash"
  rosdep install --from-paths "${WS_DIR}/src" --ignore-src -r -y
fi

# Python deps for hit25_interpreter (OpenVINO)
python3 -m pip install --user --upgrade pip
python3 -m pip install --user --no-cache-dir openvino dronecan

# GeographicLib datasets for MAVROS (safe to re-run)
if [ -x "/opt/ros/${ROS_DISTRO}/lib/mavros/install_geographiclib_datasets.sh" ]; then
  sudo "/opt/ros/${ROS_DISTRO}/lib/mavros/install_geographiclib_datasets.sh"
fi

if [ "${WITH_SITL}" = true ]; then
  echo "Installing ArduPilot SITL prerequisites..."
  sudo apt install -y python3-pip python3-venv python3-setuptools

  ARDUPILOT_DIR="${WS_DIR}/ardupilot"
  if [ -d "${ARDUPILOT_DIR}/.git" ]; then
    echo "ArduPilot already exists at ${ARDUPILOT_DIR}"
  else
    git clone https://github.com/ArduPilot/ardupilot.git "${ARDUPILOT_DIR}"
  fi

  (cd "${ARDUPILOT_DIR}" && Tools/environment_install/install-prereqs-ubuntu.sh -y) || true

  # MAVProxy (for SITL console)
  python3 -m pip install --user MAVProxy || true

  if [ "${BUILD_SITL}" = true ]; then
    echo "Building ArduPilot SITL (this may take a while)..."
    (cd "${ARDUPILOT_DIR}" && ./waf configure --board sitl && ./waf build)
  fi
fi

echo "Done. You can now build with:"
echo "  cd ${WS_DIR} && source /opt/ros/${ROS_DISTRO}/setup.bash && colcon build --symlink-install"
