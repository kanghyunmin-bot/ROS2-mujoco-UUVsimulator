#!/usr/bin/env bash
set -euo pipefail

REPO_URL="${KMU26_HYDROPHONE_REPO_URL:-https://github.com/mink0730/kmu26_auv_hydrophone.git}"
BRANCH="${KMU26_HYDROPHONE_BRANCH:-agent/improve-snr-gradient-homing}"

usage() {
  cat <<'EOF'
Usage: ./setup_kmu26_hydrophone.sh [--install-deps] [--pull]

Build kmu26_auv_hydrophone in this UUV simulator workspace without relying on
~/catkin_ws. The script resolves paths from its own location:

  UUV root: <script directory>
  ROS ws:   <script directory>/rospkg
  Source:   <script directory>/rospkg/src/kmu26_auv_hydrophone

Options:
  --install-deps  Install required Ubuntu/GStreamer development packages.
  --pull          Fast-forward to the configured SNR-capable upstream branch.

Environment overrides:
  KMU26_HYDROPHONE_REPO_URL  Hydrophone git URL.
  KMU26_HYDROPHONE_BRANCH    Branch to clone/build.
EOF
}

INSTALL_DEPS=0
PULL_REPO=0
for arg in "$@"; do
  case "$arg" in
    --install-deps) INSTALL_DEPS=1 ;;
    --pull) PULL_REPO=1 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "unknown option: $arg" >&2; usage >&2; exit 2 ;;
  esac
done

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ROS_WS="${SCRIPT_DIR}/rospkg"
ROS_SRC="${ROS_WS}/src"
PKG_DIR="${ROS_SRC}/kmu26_auv_hydrophone"

if [[ ! -d "${ROS_WS}" ]]; then
  echo "[FAIL] ROS workspace directory not found: ${ROS_WS}" >&2
  exit 1
fi

echo "[INFO] UUV root: ${SCRIPT_DIR}"
echo "[INFO] ROS workspace: ${ROS_WS}"
mkdir -p "${ROS_SRC}"

if [[ -d "${HOME}/kmu26_auv_hydrophone" && "${HOME}/kmu26_auv_hydrophone" != "${PKG_DIR}" ]]; then
  echo "[WARN] duplicate hydrophone clone exists outside workspace: ${HOME}/kmu26_auv_hydrophone"
  echo "       It is ignored. Remove it manually if colcon was run from HOME:"
  echo "       rm -rf ${HOME}/kmu26_auv_hydrophone"
fi

if [[ ! -d "${PKG_DIR}/.git" ]]; then
  echo "[INFO] cloning ${REPO_URL} -> ${PKG_DIR}"
  git clone -b "${BRANCH}" "${REPO_URL}" "${PKG_DIR}"
elif [[ "${PULL_REPO}" -eq 1 ]]; then
  echo "[INFO] updating existing hydrophone repo from ${REPO_URL} (${BRANCH})"
  git -C "${PKG_DIR}" fetch "${REPO_URL}" "${BRANCH}"
  git -C "${PKG_DIR}" merge --ff-only FETCH_HEAD
else
  echo "[INFO] hydrophone repo already exists: ${PKG_DIR}"
fi

if [[ "${INSTALL_DEPS}" -eq 1 ]]; then
  sudo apt update
  sudo apt install -y \
    pkg-config \
    libgstreamer1.0-dev \
    libgstreamer-plugins-base1.0-dev \
    libboost-thread-dev \
    libeigen3-dev \
    ros-humble-diagnostic-updater \
    ros-humble-rclcpp-components \
    gstreamer1.0-tools \
    gstreamer1.0-alsa \
    gstreamer1.0-plugins-base \
    gstreamer1.0-plugins-good
fi

missing=0
for module in gstreamer-1.0 gstreamer-app-1.0; do
  if pkg-config --exists "${module}"; then
    echo "[PASS] ${module} $(pkg-config --modversion "${module}")"
  else
    echo "[FAIL] missing pkg-config module: ${module}"
    missing=1
  fi
done

if [[ "${missing}" -ne 0 ]]; then
  cat <<'EOF' >&2

Install the missing dependencies, then rerun this script:

  sudo apt update
  sudo apt install -y libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev

or:

  ./setup_kmu26_hydrophone.sh --install-deps
EOF
  exit 1
fi

set +u
source /opt/ros/humble/setup.bash
set -u
cd "${ROS_WS}"
colcon build --packages-select audio_common_msgs audio_common audio_capture
set +u
source "${ROS_WS}/install/setup.bash"
set -u
ros2 pkg executables audio_capture
