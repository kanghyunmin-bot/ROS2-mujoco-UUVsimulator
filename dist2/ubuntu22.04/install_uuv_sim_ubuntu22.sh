#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
INSTALL_ROOT="${INSTALL_ROOT:-${SCRIPT_DIR}}"
UUV_ZIP="${UUV_ZIP:-${SCRIPT_DIR}/uuv_mujoco.zip}"
ROS_DISTRO="${ROS_DISTRO:-humble}"
ARDUPILOT_REMOTE="${ARDUPILOT_REMOTE:-https://github.com/ArduPilot/ardupilot.git}"
ARDUPILOT_BRANCH="${ARDUPILOT_BRANCH:-}"
QGC_URL="${QGC_URL:-https://d176tv9ibo4jno.cloudfront.net/latest/QGroundControl-x86_64.AppImage}"
QGC_APP="${QGC_APP:-${INSTALL_ROOT}/QGroundControl-x86_64.AppImage}"
PYTHON_MODE="${PYTHON_MODE:-native}"
MJ311_ROOT="${MJ311_ROOT:-${HOME}/.venvs/uuv_mujoco}"
MUJOCO_PIP_SPEC="${MUJOCO_PIP_SPEC:-mujoco==3.8.0}"

WITH_ROS2=1
SKIP_APT=0
SKIP_QGC=0
SKIP_ARDUPILOT_PREREQS=0
SKIP_REAL_ROS_PKG=0
NONINTERACTIVE=0
FORCE_UNSUPPORTED=0
FORCE_REEXTRACT=0
RECREATE_VENV=0
MASK_MODEMMANAGER=0
RUN_AFTER_INSTALL=0
RUN_MODE="auto"

usage() {
  cat <<'USAGE'
Usage: ./install_uuv_sim_ubuntu22.sh [options]

This installer expects uuv_mujoco.zip next to this script. It installs the
full Ubuntu 22.04 runtime workspace in the directory where the script lives:
  - system packages
  - optional ROS 2 Humble/MAVROS/RViz packages
  - ArduPilot official checkout and submodules
  - QGroundControl Linux AppImage
  - native Python user-site MuJoCo dependencies by default
  - .uuv_mujoco_env.sh environment file

Options:
  --install-root PATH       Install workspace here (default: script directory)
  --uuv-zip PATH            Use this uuv_mujoco.zip file
  --native-python           Use native python3 + user-site pip packages (default)
  --python-mode MODE        Python dependency mode: native or venv (default: native)
  --venv-root PATH          Use MuJoCo Python virtualenv path; implies --python-mode venv
  --ardupilot-branch NAME   Clone a specific ArduPilot branch
  --qgc-url URL             Override QGroundControl AppImage URL
  --mujoco-pip-spec SPEC    Override MuJoCo pip spec (default: mujoco==3.8.0)
  --without-ros2            Skip ROS 2 apt packages
  --skip-apt                Skip all apt/system package installation
  --skip-qgc                Skip QGroundControl download
  --skip-ardupilot-prereqs  Skip ArduPilot's official Ubuntu prereq helper
  --skip-real-ros-pkg       Skip building bundled ROS2 helper packages
  --force-reextract         Replace existing uuv_mujoco directory from the zip
  --recreate-venv           Remove and recreate the MuJoCo Python virtualenv in venv mode
  --mask-modemmanager       Stop and mask ModemManager for QGC serial access
  --run-after-install       Start SITL + MuJoCo immediately after a successful install
  --run-headless            Start SITL + MuJoCo headless after install
  --run-gui                 Start SITL + MuJoCo with the MuJoCo viewer after install
  --noninteractive          Run apt with DEBIAN_FRONTEND=noninteractive
  --force-unsupported       Continue even if OS is not Ubuntu 22.04
  -h, --help                Show this help

After install:
  source ./.uuv_mujoco_env.sh
  ./run_control_gui.sh

Headless smoke test:
  source ./.uuv_mujoco_env.sh
  cd uuv_mujoco/v2.2
  ./start_sitl_mujoco_mj311.sh -- --headless
USAGE
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --install-root)
      [[ $# -ge 2 ]] || { echo "[error] --install-root requires a value" >&2; exit 2; }
      INSTALL_ROOT="$2"
      shift 2
      ;;
    --uuv-zip)
      [[ $# -ge 2 ]] || { echo "[error] --uuv-zip requires a value" >&2; exit 2; }
      UUV_ZIP="$2"
      shift 2
      ;;
    --native-python)
      PYTHON_MODE="native"
      shift
      ;;
    --python-mode)
      [[ $# -ge 2 ]] || { echo "[error] --python-mode requires a value" >&2; exit 2; }
      case "$2" in
        native|venv)
          PYTHON_MODE="$2"
          ;;
        *)
          echo "[error] unsupported --python-mode: $2" >&2
          echo "        Expected: native or venv" >&2
          exit 2
          ;;
      esac
      shift 2
      ;;
    --venv-root)
      [[ $# -ge 2 ]] || { echo "[error] --venv-root requires a value" >&2; exit 2; }
      PYTHON_MODE="venv"
      MJ311_ROOT="$2"
      shift 2
      ;;
    --ardupilot-branch)
      [[ $# -ge 2 ]] || { echo "[error] --ardupilot-branch requires a value" >&2; exit 2; }
      ARDUPILOT_BRANCH="$2"
      shift 2
      ;;
    --qgc-url)
      [[ $# -ge 2 ]] || { echo "[error] --qgc-url requires a value" >&2; exit 2; }
      QGC_URL="$2"
      shift 2
      ;;
    --mujoco-pip-spec)
      [[ $# -ge 2 ]] || { echo "[error] --mujoco-pip-spec requires a value" >&2; exit 2; }
      MUJOCO_PIP_SPEC="$2"
      shift 2
      ;;
    --without-ros2)
      WITH_ROS2=0
      shift
      ;;
    --skip-apt)
      SKIP_APT=1
      shift
      ;;
    --skip-qgc)
      SKIP_QGC=1
      shift
      ;;
    --skip-ardupilot-prereqs)
      SKIP_ARDUPILOT_PREREQS=1
      shift
      ;;
    --skip-real-ros-pkg)
      SKIP_REAL_ROS_PKG=1
      shift
      ;;
    --force-reextract)
      FORCE_REEXTRACT=1
      shift
      ;;
    --recreate-venv)
      RECREATE_VENV=1
      shift
      ;;
    --mask-modemmanager)
      MASK_MODEMMANAGER=1
      shift
      ;;
    --run-after-install)
      RUN_AFTER_INSTALL=1
      shift
      ;;
    --run-headless)
      RUN_AFTER_INSTALL=1
      RUN_MODE="headless"
      shift
      ;;
    --run-gui)
      RUN_AFTER_INSTALL=1
      RUN_MODE="gui"
      shift
      ;;
    --noninteractive)
      NONINTERACTIVE=1
      shift
      ;;
    --force-unsupported)
      FORCE_UNSUPPORTED=1
      shift
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

case "$PYTHON_MODE" in
  native|venv)
    ;;
  *)
    echo "[error] unsupported PYTHON_MODE: $PYTHON_MODE" >&2
    echo "        Expected: native or venv" >&2
    exit 2
    ;;
esac

INSTALL_ROOT="$(mkdir -p "${INSTALL_ROOT}" && cd "${INSTALL_ROOT}" && pwd)"
UUV_ZIP="$(cd "$(dirname "${UUV_ZIP}")" && pwd)/$(basename "${UUV_ZIP}")"
ARDUPILOT_DIR="${ARDUPILOT_DIR:-${INSTALL_ROOT}/ardupilot}"
UUV_MUJOCO_DIR="${UUV_MUJOCO_DIR:-${INSTALL_ROOT}/uuv_mujoco}"
QGC_APP="$(cd "$(dirname "${QGC_APP}")" && pwd)/$(basename "${QGC_APP}")"

log() {
  echo "[uuv-dist] $*"
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

apt_run() {
  if [[ "$NONINTERACTIVE" -eq 1 ]]; then
    sudo_run env DEBIAN_FRONTEND=noninteractive "$@"
  else
    sudo_run "$@"
  fi
}

apt_package_exists() {
  apt-cache show "$1" >/dev/null 2>&1
}

apt_install_available() {
  local label="$1"
  shift
  local installable=()
  local pkg

  for pkg in "$@"; do
    if apt_package_exists "$pkg"; then
      installable+=("$pkg")
    else
      log "warning: apt package not available, skipped: ${pkg}"
    fi
  done

  if ((${#installable[@]} > 0)); then
    log "installing ${label}"
    apt_run apt-get install -y "${installable[@]}"
  fi
}

require_cmd() {
  command -v "$1" >/dev/null 2>&1 || { echo "[error] command not found: $1" >&2; exit 1; }
}

check_ubuntu_2204() {
  if [[ ! -f /etc/os-release ]]; then
    [[ "$FORCE_UNSUPPORTED" -eq 1 ]] && return 0
    echo "[error] /etc/os-release not found. This bundle targets Ubuntu 22.04." >&2
    exit 1
  fi

  # shellcheck source=/dev/null
  source /etc/os-release
  if [[ "${ID:-}" == "ubuntu" && "${VERSION_ID:-}" == "22.04" ]]; then
    return 0
  fi
  if [[ "$FORCE_UNSUPPORTED" -eq 1 ]]; then
    log "warning: unsupported OS ${PRETTY_NAME:-unknown}; continuing by request"
    return 0
  fi
  echo "[error] unsupported OS: ${PRETTY_NAME:-unknown}. Expected Ubuntu 22.04." >&2
  echo "        Use --force-unsupported only if you know this host is compatible." >&2
  exit 1
}

ensure_ros_apt_repo() {
  local distro_codename arch keyring repo_file repo_line existing_sources
  distro_codename="$(. /etc/os-release && printf '%s' "${UBUNTU_CODENAME:-}")"
  arch="$(dpkg --print-architecture)"
  keyring="/usr/share/keyrings/ros-archive-keyring.gpg"
  repo_file="/etc/apt/sources.list.d/ros2.list"
  existing_sources="/etc/apt/sources.list.d/ros2.sources"
  repo_line="deb [arch=${arch} signed-by=${keyring}] http://packages.ros.org/ros2/ubuntu ${distro_codename} main"

  if [[ -e "$existing_sources" ]] && grep -Fq 'packages.ros.org/ros2/ubuntu' "$existing_sources" 2>/dev/null; then
    log "using existing ROS apt repository: $existing_sources"
    return 0
  fi

  sudo_run mkdir -p /usr/share/keyrings
  if [[ ! -f "$keyring" ]]; then
    local tmp_key
    tmp_key="$(mktemp)"
    require_cmd curl
    require_cmd gpg
    log "installing ROS apt signing key"
    run curl -fsSL -o "$tmp_key" https://raw.githubusercontent.com/ros/rosdistro/master/ros.key
    sudo_run gpg --dearmor -o "$keyring" "$tmp_key"
    rm -f "$tmp_key"
  fi

  if [[ ! -f "$repo_file" ]] || ! grep -Fq -- "$repo_line" "$repo_file" 2>/dev/null; then
    log "registering ROS apt repository"
    printf '%s\n' "$repo_line" | sudo_run tee "$repo_file" >/dev/null
  fi
}

cleanup_duplicate_ros_apt_repo() {
  local ros_sources="/etc/apt/sources.list.d/ros2.sources"
  local ros_list="/etc/apt/sources.list.d/ros2.list"

  if [[ -e "$ros_sources" && -f "$ros_list" ]] \
    && grep -Fq 'packages.ros.org/ros2/ubuntu' "$ros_sources" 2>/dev/null \
    && grep -Fq 'packages.ros.org/ros2/ubuntu' "$ros_list" 2>/dev/null; then
    log "removing duplicate ROS apt list entry; using existing ros2.sources"
    sudo_run rm -f "$ros_list"
  fi
}

install_system_packages() {
  [[ "$SKIP_APT" -eq 0 ]] || { log "skipping apt install"; return 0; }

  log "installing Ubuntu system dependencies"
  cleanup_duplicate_ros_apt_repo
  apt_run apt-get update
  apt_run apt-get install -y \
    git curl unzip ca-certificates gnupg lsb-release software-properties-common \
    build-essential ccache gawk make cmake pkg-config \
    python3 python3-venv python3-pip python3-dev \
    python3-tk python3-numpy python3-matplotlib \
    ffmpeg jq xz-utils file lsof iproute2 \
    libgl1 libegl1 libglfw3 libxrender1 libxext6 libxi6 libxrandr2 \
    libxxf86vm1 libxinerama1 libxcursor1

  apt_install_available "OpenGL/headless helpers" \
    libglvnd0 libglx0 libopengl0 libgl1-mesa-dri mesa-utils libosmesa6

  apt_install_available "QGroundControl Qt/AppImage helpers" \
    libxcb-xinerama0 libxkbcommon-x11-0 libxcb-cursor0 libxcb-cursor-dev \
    libxcb-icccm4 libxcb-image0 libxcb-keysyms1 libxcb-render-util0 \
    libxcb-randr0 libxcb-shape0 libxcb-xfixes0 libxcb-sync1 \
    libxcb-shm0 libxcb-render0 libxcb-glx0

  apt_install_available "GStreamer video helpers" \
    gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl \
    python3-gi python3-gst-1.0

  apt_install_available "diagnostic and SocketCAN tools" \
    ripgrep net-tools can-utils

  if apt_package_exists libfuse2; then
    apt_run apt-get install -y libfuse2
  elif apt_package_exists libfuse2t64; then
    apt_run apt-get install -y libfuse2t64
  else
    log "warning: libfuse2/libfuse2t64 not found; QGC AppImage may need manual setup"
  fi

  if [[ "$WITH_ROS2" -eq 1 ]]; then
    ensure_ros_apt_repo
    apt_run apt-get update
    local ros_pkgs installable pkg
    ros_pkgs=(
      "python3-rosdep"
      "python3-vcstool"
      "python3-colcon-common-extensions"
      "geographiclib-tools"
      "ros-${ROS_DISTRO}-ros-base"
      "ros-${ROS_DISTRO}-ament-cmake"
      "ros-${ROS_DISTRO}-ament-index-python"
      "ros-${ROS_DISTRO}-rclcpp"
      "ros-${ROS_DISTRO}-rclpy"
      "ros-${ROS_DISTRO}-launch"
      "ros-${ROS_DISTRO}-launch-ros"
      "ros-${ROS_DISTRO}-ros2launch"
      "ros-${ROS_DISTRO}-std-msgs"
      "ros-${ROS_DISTRO}-std-srvs"
      "ros-${ROS_DISTRO}-geometry-msgs"
      "ros-${ROS_DISTRO}-sensor-msgs"
      "ros-${ROS_DISTRO}-nav-msgs"
      "ros-${ROS_DISTRO}-tf2"
      "ros-${ROS_DISTRO}-tf2-ros"
      "ros-${ROS_DISTRO}-tf2-msgs"
      "ros-${ROS_DISTRO}-tf2-geometry-msgs"
      "ros-${ROS_DISTRO}-mavros"
      "ros-${ROS_DISTRO}-mavros-msgs"
      "ros-${ROS_DISTRO}-mavros-extras"
      "ros-${ROS_DISTRO}-joy"
      "ros-${ROS_DISTRO}-rosbag2"
      "ros-${ROS_DISTRO}-rosbag2-py"
      "ros-${ROS_DISTRO}-rosbag2-storage-default-plugins"
      "ros-${ROS_DISTRO}-rosidl-runtime-py"
      "ros-${ROS_DISTRO}-rqt-bag"
      "ros-${ROS_DISTRO}-rqt-image-view"
      "ros-${ROS_DISTRO}-image-transport"
      "ros-${ROS_DISTRO}-rviz2"
      "ros-${ROS_DISTRO}-robot-state-publisher"
      "ros-${ROS_DISTRO}-xacro"
      "ros-${ROS_DISTRO}-dvl-msgs"
    )
    installable=()
    for pkg in "${ros_pkgs[@]}"; do
      if apt_package_exists "$pkg"; then
        installable+=("$pkg")
      else
        log "warning: apt package not available, skipped: $pkg"
      fi
    done
    if ((${#installable[@]} > 0)); then
      apt_run apt-get install -y "${installable[@]}"
    fi
    if [[ -x "/opt/ros/${ROS_DISTRO}/lib/mavros/install_geographiclib_datasets.sh" ]]; then
      sudo_run "/opt/ros/${ROS_DISTRO}/lib/mavros/install_geographiclib_datasets.sh" || true
    fi
    if command -v rosdep >/dev/null 2>&1; then
      if [[ ! -f "/etc/ros/rosdep/sources.list.d/20-default.list" ]]; then
        sudo_run rosdep init || true
      fi
      rosdep update || log "warning: rosdep update failed; rerun later if needed"
    fi
  fi

  local target_user
  target_user="$(id -un)"
  if [[ -n "${SUDO_USER:-}" && "${SUDO_USER}" != "root" ]]; then
    target_user="$SUDO_USER"
  fi
  if id "$target_user" >/dev/null 2>&1; then
    sudo_run usermod -aG dialout "$target_user" || true
    log "dialout group requested for ${target_user}; log out/in once for USB serial access"
  fi
  if [[ "$MASK_MODEMMANAGER" -eq 1 ]] && systemctl list-unit-files ModemManager.service >/dev/null 2>&1; then
    sudo_run systemctl mask --now ModemManager.service || true
  fi
}

extract_uuv_mujoco() {
  [[ -f "$UUV_ZIP" ]] || { echo "[error] uuv_mujoco.zip not found: $UUV_ZIP" >&2; exit 1; }
  require_cmd unzip

  if [[ -d "$UUV_MUJOCO_DIR" && "$FORCE_REEXTRACT" -eq 0 ]]; then
    log "reusing existing uuv_mujoco: $UUV_MUJOCO_DIR"
  else
    if [[ -d "$UUV_MUJOCO_DIR" ]]; then
      run rm -rf "$UUV_MUJOCO_DIR"
    fi
    local tmpdir extracted_root
    tmpdir="$(mktemp -d)"
    trap 'rm -rf "${tmpdir:-}"' RETURN
    run unzip -q "$UUV_ZIP" -d "$tmpdir"
    extracted_root="$(find "$tmpdir" -type f -path '*/v2.2/start_sitl_mujoco_mj311.sh' -print -quit)"
    [[ -n "$extracted_root" ]] || { echo "[error] could not find uuv_mujoco/v2.2 in zip" >&2; exit 1; }
    extracted_root="$(dirname "$(dirname "$extracted_root")")"
    run mv "$extracted_root" "$UUV_MUJOCO_DIR"
    rm -rf "$tmpdir"
    trap - RETURN
  fi

  for script in \
    "$UUV_MUJOCO_DIR/v2.2/start_sitl_mujoco_mj311.sh" \
    "$UUV_MUJOCO_DIR/v2.2/start_ardusub_sitl_mj311.sh" \
    "$UUV_MUJOCO_DIR/v2.2/launch_uuv_sim.sh" \
    "$UUV_MUJOCO_DIR/v2.2/reset_uuv_sim.sh"
  do
    [[ -f "$script" ]] || { echo "[error] missing script after extract: $script" >&2; exit 1; }
    chmod +x "$script"
  done
}

install_distribution_support_files() {
  log "installing GUI and analysis support files"

  mkdir -p "$INSTALL_ROOT"
  for rel in \
    "uuv_control_gui.py" \
    "run_control_gui.sh" \
    "cleanup_generated_artifacts.sh"
  do
    if [[ -f "${SCRIPT_DIR}/${rel}" ]]; then
      if [[ "${SCRIPT_DIR}/${rel}" != "${INSTALL_ROOT}/${rel}" ]]; then
        run cp -f "${SCRIPT_DIR}/${rel}" "${INSTALL_ROOT}/${rel}"
      fi
      chmod +x "${INSTALL_ROOT}/${rel}" 2>/dev/null || true
    fi
  done

  if [[ -d "${SCRIPT_DIR}/document/docsource" ]]; then
    mkdir -p "${INSTALL_ROOT}/document"
    if [[ "${SCRIPT_DIR}/document" != "${INSTALL_ROOT}/document" ]]; then
      run rm -rf "${INSTALL_ROOT}/document/docsource"
      run cp -R "${SCRIPT_DIR}/document/docsource" "${INSTALL_ROOT}/document/docsource"
    fi
    find "${INSTALL_ROOT}/document/docsource" -maxdepth 1 -type f \( -name '*.py' -o -name '*.sh' \) -exec chmod +x {} + 2>/dev/null || true
  fi
}

setup_ardupilot() {
  require_cmd git
  if [[ -d "$ARDUPILOT_DIR/.git" ]]; then
    log "reusing existing ArduPilot checkout: $ARDUPILOT_DIR"
  elif [[ -e "$ARDUPILOT_DIR" ]]; then
    echo "[error] ARDUPILOT_DIR exists but is not a git checkout: $ARDUPILOT_DIR" >&2
    exit 1
  else
    local clone_args
    clone_args=()
    if [[ -n "$ARDUPILOT_BRANCH" ]]; then
      clone_args+=(--branch "$ARDUPILOT_BRANCH")
    fi
    run git clone "${clone_args[@]}" "$ARDUPILOT_REMOTE" "$ARDUPILOT_DIR"
  fi
  run git -C "$ARDUPILOT_DIR" submodule sync --recursive
  run git -C "$ARDUPILOT_DIR" submodule update --init --recursive
  [[ -f "$ARDUPILOT_DIR/Tools/autotest/sim_vehicle.py" ]] || {
    echo "[error] sim_vehicle.py missing after ArduPilot setup" >&2
    exit 1
  }
  if [[ "$SKIP_ARDUPILOT_PREREQS" -eq 0 && -x "$ARDUPILOT_DIR/Tools/environment_install/install-prereqs-ubuntu.sh" ]]; then
    run "$ARDUPILOT_DIR/Tools/environment_install/install-prereqs-ubuntu.sh" -y
  fi
}

ensure_colcon_python_compat() {
  if command -v python3 >/dev/null 2>&1 && command -v colcon >/dev/null 2>&1; then
    run python3 -m pip install --user 'setuptools<80'
  fi
}

setup_real_ros_pkg() {
  [[ "$WITH_ROS2" -eq 1 ]] || { log "skipping real ROS package build because ROS2 install is disabled"; return 0; }
  [[ "$SKIP_REAL_ROS_PKG" -eq 0 ]] || { log "skipping bundled real ROS package build"; return 0; }

  local kmu26_zip=""
  for candidate in \
    "${SCRIPT_DIR}/rospkg/kmu26_auv.zip" \
    "${SCRIPT_DIR}/kmu26_auv.zip" \
    "${INSTALL_ROOT}/rospkg/kmu26_auv.zip"
  do
    if [[ -f "$candidate" ]]; then
      kmu26_zip="$candidate"
      break
    fi
  done
  if [[ -z "$kmu26_zip" ]]; then
    log "bundled kmu26_auv.zip not found; skipping real ROS helper package"
    return 0
  fi
  local dvl_zip=""
  for candidate in \
    "${SCRIPT_DIR}/rospkg/dvl_msgs.zip" \
    "${SCRIPT_DIR}/dvl_msgs.zip" \
    "${INSTALL_ROOT}/rospkg/dvl_msgs.zip"
  do
    if [[ -f "$candidate" ]]; then
      dvl_zip="$candidate"
      break
    fi
  done
  if [[ -z "$dvl_zip" ]]; then
    log "bundled dvl_msgs.zip not found; expecting dvl_msgs to be installed externally"
  fi
  local ping360_zip=""
  for candidate in \
    "${SCRIPT_DIR}/rospkg/ping360_sonar_msgs.zip" \
    "${SCRIPT_DIR}/ping360_sonar_msgs.zip" \
    "${INSTALL_ROOT}/rospkg/ping360_sonar_msgs.zip"
  do
    if [[ -f "$candidate" ]]; then
      ping360_zip="$candidate"
      break
    fi
  done
  if [[ -z "$ping360_zip" ]]; then
    log "bundled ping360_sonar_msgs.zip not found; Ping360 SonarEcho topics will be disabled"
  fi
  if [[ ! -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
    log "ROS setup missing; skipping real ROS helper package"
    return 0
  fi

  require_cmd unzip
  require_cmd colcon
  ensure_colcon_python_compat

  local ros_ws="${INSTALL_ROOT}/rospkg"
  mkdir -p "$ros_ws"
  extract_ros_pkg_zip() {
    local pkg_name="$1"
    local zip_path="$2"
    [[ -n "$zip_path" ]] || return 0
    if [[ -d "${ros_ws}/${pkg_name}" ]]; then
      log "reusing existing ROS helper package: ${ros_ws}/${pkg_name}"
      return 0
    fi
    local tmpdir package_root
    tmpdir="$(mktemp -d)"
    trap 'rm -rf "${tmpdir:-}"' RETURN
    run unzip -q "$zip_path" -d "$tmpdir"
    package_root="$(find "$tmpdir" -type f -name package.xml -path "*/${pkg_name}/package.xml" -print -quit)"
    [[ -n "$package_root" ]] || { echo "[error] ${pkg_name}/package.xml not found in $zip_path" >&2; exit 1; }
    package_root="$(dirname "$package_root")"
    run mv "$package_root" "${ros_ws}/${pkg_name}"
    rm -rf "$tmpdir"
    trap - RETURN
  }

  extract_ros_pkg_zip "dvl_msgs" "$dvl_zip"
  extract_ros_pkg_zip "ping360_sonar_msgs" "$ping360_zip"
  extract_ros_pkg_zip "kmu26_auv" "$kmu26_zip"

  log "building bundled ROS helper package"
  (
    set +u
    # shellcheck source=/dev/null
    source "/opt/ros/${ROS_DISTRO}/setup.bash"
    set -u
    cd "$ros_ws"
    local packages=()
    if [[ -d "${ros_ws}/dvl_msgs" ]]; then
      packages+=(dvl_msgs)
    fi
    if [[ -d "${ros_ws}/ping360_sonar_msgs" ]]; then
      packages+=(ping360_sonar_msgs)
    fi
    packages+=(hit25_auv_ros2)
    colcon build --symlink-install --packages-select "${packages[@]}"
  )
}

download_qgc() {
  [[ "$SKIP_QGC" -eq 0 ]] || { log "skipping QGroundControl download"; return 0; }
  require_cmd curl
  if [[ -f "$QGC_APP" ]]; then
    log "reusing QGroundControl AppImage: $QGC_APP"
  else
    mkdir -p "$(dirname "$QGC_APP")"
    log "downloading QGroundControl AppImage"
    run curl -fL --retry 3 -o "$QGC_APP" "$QGC_URL"
  fi
  chmod +x "$QGC_APP"
}

setup_python_env() {
  local python_bin runtime_python pip_user_args venv_python
  python_bin="$(command -v python3)"
  if [[ "$PYTHON_MODE" == "venv" ]]; then
    if [[ "$RECREATE_VENV" -eq 1 && -d "$MJ311_ROOT" ]]; then
      run rm -rf "$MJ311_ROOT"
    fi
    if [[ ! -d "$MJ311_ROOT" ]]; then
      run "$python_bin" -m venv "$MJ311_ROOT"
    fi
    venv_python="$MJ311_ROOT/bin/python"
    [[ -x "$venv_python" ]] || { echo "[error] venv python missing: $venv_python" >&2; exit 1; }
    runtime_python="$venv_python"
    run env -u PYTHONPATH -u PYTHONHOME "$runtime_python" -m pip install -U pip "setuptools<80" wheel
    run env -u PYTHONPATH -u PYTHONHOME "$runtime_python" -m pip install -U \
      --index-url https://download.pytorch.org/whl/cpu \
      torch torchvision
    run env -u PYTHONPATH -u PYTHONHOME "$runtime_python" -m pip install \
      numpy matplotlib rosbags python-pptx \
      "$MUJOCO_PIP_SPEC" pymavlink MAVProxy pexpect pillow future dronecan gnureadline "empy==3.3.4" \
      "opencv-python-headless<5" ultralytics
  else
    runtime_python="$python_bin"
    pip_user_args=(--user)
    log "installing native Python dependencies into user site"
    run env -u PYTHONPATH -u PYTHONHOME "$runtime_python" -m pip install "${pip_user_args[@]}" -U pip packaging "setuptools<80" wheel
    run env -u PYTHONPATH -u PYTHONHOME "$runtime_python" -m pip install "${pip_user_args[@]}" -U \
      --index-url https://download.pytorch.org/whl/cpu \
      torch torchvision
    run env -u PYTHONPATH -u PYTHONHOME "$runtime_python" -m pip install "${pip_user_args[@]}" -U \
      rosbags python-pptx "$MUJOCO_PIP_SPEC" pymavlink MAVProxy pexpect pillow future dronecan gnureadline "empy==3.3.4" \
      "opencv-python-headless<5" ultralytics
  fi
  env -u PYTHONPATH -u PYTHONHOME "$runtime_python" - <<'PY'
import cv2
import mujoco
import numpy
import matplotlib
import rosbags
import pptx
import pymavlink
import MAVProxy
import pexpect
import PIL
import future
import dronecan
import em
import torch
import ultralytics
print("python deps ok")
print("MuJoCo Version:", mujoco.__version__)
print("OpenCV Version:", cv2.__version__)
print("Torch Version:", torch.__version__)
print("Ultralytics Version:", ultralytics.__version__)
PY
}

write_env_file() {
  local env_file="${INSTALL_ROOT}/.uuv_mujoco_env.sh"
  local native_python
  native_python="$(command -v python3)"
  cat >"$env_file" <<EOF
#!/usr/bin/env bash
export WORKSPACE_DIR="${INSTALL_ROOT}"
export ROS_WORKSPACE_DIR="\${ROS_WORKSPACE_DIR:-\${WORKSPACE_DIR}/rospkg}"
export UUV_MUJOCO_DIR="\${UUV_MUJOCO_DIR:-\${WORKSPACE_DIR}/uuv_mujoco}"
export ARDUPILOT_DIR="\${ARDUPILOT_DIR:-\${WORKSPACE_DIR}/ardupilot}"
export KMU26_AUV_DIR="\${KMU26_AUV_DIR:-\${ROS_WORKSPACE_DIR}/kmu26_auv}"
export ROS_DISTRO="\${ROS_DISTRO:-${ROS_DISTRO}}"
export UUV_PYTHON_MODE="${PYTHON_MODE}"
EOF
  if [[ "$PYTHON_MODE" == "venv" ]]; then
    cat >>"$env_file" <<EOF
export MJ311_ROOT="\${MJ311_ROOT:-${MJ311_ROOT}}"
export MJ311_PYTHON="\${MJ311_PYTHON:-\${MJ311_ROOT}/bin/python}"
if [[ -z "\${MJ311_MJPYTHON:-}" && -x "\${MJ311_ROOT}/bin/mjpython" ]]; then
  export MJ311_MJPYTHON="\${MJ311_ROOT}/bin/mjpython"
fi
EOF
  else
    cat >>"$env_file" <<EOF
unset MJ311_ROOT
export MJ311_PYTHON="${native_python}"
if [[ -n "\${MJ311_MJPYTHON:-}" && ! -x "\${MJ311_MJPYTHON}" ]]; then
  unset MJ311_MJPYTHON
fi
EOF
  fi
  cat >>"$env_file" <<EOF
export QGC_APP="\${QGC_APP:-${QGC_APP}}"
export UUV_CONTROL_GUI="\${UUV_CONTROL_GUI:-\${WORKSPACE_DIR}/uuv_control_gui.py}"
if [[ -z "\${ROS_ENV_SETUP:-}" && -f "/opt/ros/\${ROS_DISTRO}/setup.bash" ]]; then
  export ROS_ENV_SETUP="/opt/ros/\${ROS_DISTRO}/setup.bash"
fi
if [[ -z "\${ROS_INSTALL_SETUP:-}" ]]; then
  for _candidate in "\${ROS_WORKSPACE_DIR}/install/setup.bash" "\${WORKSPACE_DIR}/install/setup.bash"; do
    if [[ -f "\${_candidate}" ]]; then
      export ROS_INSTALL_SETUP="\${_candidate}"
      break
    fi
  done
fi
EOF
  chmod +x "$env_file"
  log "wrote $env_file"
}

verify_install() {
  local fail=0 runtime_python
  [[ -d "$UUV_MUJOCO_DIR/v2.2" ]] || { echo "[FAIL] uuv_mujoco/v2.2 missing"; fail=1; }
  [[ -f "${INSTALL_ROOT}/uuv_control_gui.py" ]] || { echo "[FAIL] uuv_control_gui.py missing"; fail=1; }
  [[ -x "${INSTALL_ROOT}/run_control_gui.sh" ]] || { echo "[FAIL] run_control_gui.sh missing or not executable"; fail=1; }
  [[ -f "$ARDUPILOT_DIR/Tools/autotest/sim_vehicle.py" ]] || { echo "[FAIL] ArduPilot sim_vehicle.py missing"; fail=1; }
  if [[ "$PYTHON_MODE" == "venv" ]]; then
    runtime_python="$MJ311_ROOT/bin/python"
    [[ -x "$runtime_python" ]] || { echo "[FAIL] MuJoCo venv python missing"; fail=1; }
  else
    runtime_python="$(command -v python3 || true)"
    [[ -x "$runtime_python" ]] || { echo "[FAIL] native python3 missing"; fail=1; }
  fi
  if [[ "$SKIP_QGC" -eq 0 && ! -x "$QGC_APP" ]]; then
    echo "[FAIL] QGroundControl AppImage missing or not executable"
    fail=1
  fi
  if [[ "$WITH_ROS2" -eq 1 && ! -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
    echo "[WARN] ROS setup not found at /opt/ros/${ROS_DISTRO}/setup.bash"
  fi
  [[ "$fail" -eq 0 ]] || exit 1
}

print_next_steps() {
  cat <<EOF

[uuv-dist] install complete

Workspace:
  ${INSTALL_ROOT}

Run headless smoke test:
  cd "${INSTALL_ROOT}/uuv_mujoco/v2.2"
  READY_WAIT_SECS=60 SITL_WAIT_SECS=360 ./start_sitl_mujoco_mj311.sh -- --headless

Run with MuJoCo viewer:
  cd "${INSTALL_ROOT}/uuv_mujoco/v2.2"
  ./start_sitl_mujoco_mj311.sh

Run control GUI:
  cd "${INSTALL_ROOT}"
  ./run_control_gui.sh

Run QGroundControl:
  "${QGC_APP}"

Reset runtime:
  cd "${INSTALL_ROOT}/uuv_mujoco/v2.2"
  ./reset_uuv_sim.sh --with-qgc-stop

Environment for a new shell:
  source "${INSTALL_ROOT}/.uuv_mujoco_env.sh"

Notes:
  - The first ArduSub SITL launch builds ArduPilot and can take several minutes.
  - If this script added your user to dialout, log out and back in before USB serial use.
  - ArduPilot is cloned from upstream and is not patched by this installer.
EOF
}

run_simulator_after_install() {
  local start_script run_headless
  local start_args=()
  local mujoco_args=()

  start_script="${UUV_MUJOCO_DIR}/v2.2/start_sitl_mujoco_mj311.sh"
  [[ -x "$start_script" ]] || {
    echo "[error] simulator start script missing or not executable: $start_script" >&2
    exit 1
  }

  if [[ -f "${INSTALL_ROOT}/.uuv_mujoco_env.sh" ]]; then
    # shellcheck source=/dev/null
    source "${INSTALL_ROOT}/.uuv_mujoco_env.sh"
  fi

  if [[ "$WITH_ROS2" -eq 1 ]]; then
    start_args+=(--ros2)
  else
    start_args+=(--no-ros2)
  fi

  run_headless=0
  case "$RUN_MODE" in
    headless)
      run_headless=1
      ;;
    gui)
      run_headless=0
      ;;
    auto)
      if [[ "$(uname -s)" == "Linux" && -z "${DISPLAY:-}" && -z "${WAYLAND_DISPLAY:-}" ]]; then
        run_headless=1
        log "no DISPLAY/WAYLAND_DISPLAY found; starting MuJoCo headless"
      fi
      ;;
    *)
      echo "[error] invalid RUN_MODE: $RUN_MODE" >&2
      exit 2
      ;;
  esac

  if [[ "$run_headless" -eq 1 ]]; then
    mujoco_args+=(--headless)
  fi

  log "install complete; starting simulator"
  log "script: ${start_script}"
  log "mode: $([[ "$run_headless" -eq 1 ]] && printf 'headless' || printf 'viewer')"

  if ((${#mujoco_args[@]} > 0)); then
    run "$start_script" "${start_args[@]}" -- "${mujoco_args[@]}"
  else
    run "$start_script" "${start_args[@]}"
  fi
}

log "install root: $INSTALL_ROOT"
check_ubuntu_2204
install_system_packages
extract_uuv_mujoco
install_distribution_support_files
setup_ardupilot
setup_real_ros_pkg
download_qgc
setup_python_env
write_env_file
verify_install
print_next_steps
if [[ "$RUN_AFTER_INSTALL" -eq 1 ]]; then
  run_simulator_after_install
fi
