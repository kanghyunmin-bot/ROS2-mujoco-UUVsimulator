#!/usr/bin/env bash
set -euo pipefail

SETUP_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SETUP_DIR}/.." && pwd)"
ROS_DISTRO="${ROS_DISTRO:-humble}"
PYTHON_VERSION="${PYTHON_VERSION:-3.10}"
WITH_ROS2=0
NONINTERACTIVE=0

usage() {
  cat <<'USAGE'
Usage: ./setup/01_install_system_deps.sh [options]

Run this first from the workspace root on Ubuntu 22.04 native.

What it installs:
  - Ubuntu system packages used by uuv_mujoco
  - Python runtime packages needed for venv creation
  - AppImage/FUSE support for QGroundControl
  - Optional ROS2-side extras for MAVROS/RViz/joy integration

Options:
  --with-ros2         Install ROS2-side extras if apt packages are available
  --noninteractive    Run apt with DEBIAN_FRONTEND=noninteractive
  --python-version V  Preferred Python version for later venv creation (default: 3.10)
  -h, --help          Show this help

Next step after this:
  ./setup/02_setup_ardupilot.sh
USAGE
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --with-ros2)
      WITH_ROS2=1
      shift
      ;;
    --noninteractive)
      NONINTERACTIVE=1
      shift
      ;;
    --python-version)
      [[ $# -ge 2 ]] || { echo "[error] --python-version requires a value"; exit 2; }
      PYTHON_VERSION="$2"
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

log() {
  echo "[deps] $1"
}

run() {
  echo "+ $*"
  "$@"
}

sudo_run() {
  if [[ "$(id -u)" -eq 0 ]]; then
    run "$@"
  else
    run sudo "$@"
  fi
}

apt_update() {
  if [[ "$NONINTERACTIVE" -eq 1 ]]; then
    run sudo env DEBIAN_FRONTEND=noninteractive apt-get update
  else
    sudo_run apt-get update
  fi
}

apt_install() {
  if [[ "$NONINTERACTIVE" -eq 1 ]]; then
    run sudo env DEBIAN_FRONTEND=noninteractive apt-get install -y "$@"
  else
    sudo_run apt-get install -y "$@"
  fi
}

apt_package_exists() {
  apt-cache show "$1" >/dev/null 2>&1
}

ensure_ros_apt_repo() {
  local distro_codename arch keyring repo_file repo_line

  distro_codename="$(. /etc/os-release && printf '%s' "${UBUNTU_CODENAME:-}")"
  [[ -n "$distro_codename" ]] || {
    log "warning: could not determine Ubuntu codename; skipping ROS apt repo setup"
    return 1
  }

  arch="$(dpkg --print-architecture)"
  keyring="/usr/share/keyrings/ros-archive-keyring.gpg"
  repo_file="/etc/apt/sources.list.d/ros2.list"
  repo_line="deb [arch=${arch} signed-by=${keyring}] http://packages.ros.org/ros2/ubuntu ${distro_codename} main"

  if [[ ! -f "$keyring" ]]; then
    log "installing ROS apt signing key"
    sudo_run mkdir -p /usr/share/keyrings
    sudo_run bash -lc "curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | gpg --dearmor -o '${keyring}'"
  fi

  if [[ ! -f "$repo_file" ]] || ! sudo_run grep -Fq -- "$repo_line" "$repo_file"; then
    log "registering ROS apt repository: ${repo_line}"
    printf '%s\n' "$repo_line" | sudo_run tee "$repo_file" >/dev/null
  fi
}

log "workspace: ${WORKSPACE_DIR}"
log "installing base Ubuntu packages"
apt_update
apt_install \
  git curl unzip ffmpeg \
  ca-certificates gnupg lsb-release software-properties-common \
  build-essential ccache gawk make cmake pkg-config \
  python3 python3-venv python3-pip \
  libgl1 libegl1 libglfw3 libxrender1 libxext6 libxi6 libxrandr2 \
  libxxf86vm1 libxinerama1 libxcursor1

if apt_package_exists "python${PYTHON_VERSION}" && apt_package_exists "python${PYTHON_VERSION}-venv"; then
  apt_install "python${PYTHON_VERSION}" "python${PYTHON_VERSION}-venv"
else
  log "python${PYTHON_VERSION} apt package not found; later steps will fall back to python3"
fi

if apt_package_exists libfuse2; then
  apt_install libfuse2
elif apt_package_exists libfuse2t64; then
  apt_install libfuse2t64
else
  log "warning: libfuse2/libfuse2t64 not found; QGroundControl AppImage may need manual setup"
fi

if [[ "$WITH_ROS2" -eq 1 ]]; then
  ensure_ros_apt_repo || true
  apt_update
  log "checking ROS2-side apt packages"
  ROS_PKGS=(
    "python3-rosdep"
    "python3-vcstool"
    "python3-colcon-common-extensions"
    "geographiclib-tools"
    "ros-${ROS_DISTRO}-ros-base"
    "ros-${ROS_DISTRO}-joy"
    "ros-${ROS_DISTRO}-mavros"
    "ros-${ROS_DISTRO}-mavros-msgs"
    "ros-${ROS_DISTRO}-rviz2"
    "ros-${ROS_DISTRO}-tf2-geometry-msgs"
  )
  INSTALLABLE_PKGS=()
  for pkg in "${ROS_PKGS[@]}"; do
    if apt_package_exists "$pkg"; then
      INSTALLABLE_PKGS+=("$pkg")
    else
      log "warning: apt package not available, skipped: $pkg"
    fi
  done
  if ((${#INSTALLABLE_PKGS[@]} > 0)); then
    apt_install "${INSTALLABLE_PKGS[@]}"
  fi

  if [[ -x "/opt/ros/${ROS_DISTRO}/lib/mavros/install_geographiclib_datasets.sh" ]]; then
    sudo_run "/opt/ros/${ROS_DISTRO}/lib/mavros/install_geographiclib_datasets.sh"
  else
    log "mavros geographiclib helper not found; skipped"
  fi

  if command -v rosdep >/dev/null 2>&1; then
    if [[ ! -f "/etc/ros/rosdep/sources.list.d/20-default.list" ]]; then
      sudo_run rosdep init || true
    fi
    run rosdep update || log "warning: rosdep update failed; continue and rerun later if needed"
  fi
fi

cat <<EOF

[deps] done
[deps] next:
  cd "${WORKSPACE_DIR}"
  ./setup/02_setup_ardupilot.sh
EOF
