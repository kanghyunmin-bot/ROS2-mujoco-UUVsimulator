#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_CANDIDATE="$(cd "${SCRIPT_DIR}/../../../.." 2>/dev/null && pwd || true)"
SOURCE_CHECKOUT=0
if [[ -f "${WORKSPACE_CANDIDATE}/sim/current/run_uuv_mujoco.py" &&
      -d "${WORKSPACE_CANDIDATE}/rospkg/src" ]]; then
  BUNDLE_ROOT="${WORKSPACE_CANDIDATE}"
  SOURCE_CHECKOUT=1
elif [[ -f "${WORKSPACE_CANDIDATE}/VERSION" && -d "${WORKSPACE_CANDIDATE}/sim/current" ]]; then
  BUNDLE_ROOT="${WORKSPACE_CANDIDATE}"
else
  BUNDLE_ROOT="${SCRIPT_DIR}"
fi
INSTALL_ROOT="${INSTALL_ROOT:-${BUNDLE_ROOT}}"
UUV_ZIP="${UUV_ZIP:-${BUNDLE_ROOT}/uuv_mujoco.zip}"
ROS_DISTRO="${ROS_DISTRO:-humble}"
ARDUPILOT_REMOTE="${ARDUPILOT_REMOTE:-https://github.com/ArduPilot/ardupilot.git}"
ARDUPILOT_BRANCH="${ARDUPILOT_BRANCH:-ArduSub-4.1.2}"
ARDUPILOT_BASE_COMMIT="${ARDUPILOT_BASE_COMMIT:-2dd0bb7d4c85ac48437f139d66df648fc0e1d4ae}"
ARDUPILOT_PATCH="${ARDUPILOT_PATCH:-${BUNDLE_ROOT}/sim/ardupilot_patches/0001-current-althold-stabilize.patch}"
QGC_URL="${QGC_URL:-https://d176tv9ibo4jno.cloudfront.net/latest/QGroundControl-x86_64.AppImage}"
QGC_APP="${QGC_APP:-}"
VENV_ROOT="${VENV_ROOT:-${HOME}/.venvs/uuv_mujoco}"
MUJOCO_PIP_SPEC="${MUJOCO_PIP_SPEC:-mujoco==3.8.0}"

WITH_ROS2=1
SKIP_APT=0
SKIP_QGC=0
SKIP_ARDUPILOT=0
SKIP_ARDUPILOT_PREREQS=0
SKIP_ARDUPILOT_BUILD=0
SKIP_ROSPKG_BUILD=0
SKIP_PYTHON_ENV=0
NONINTERACTIVE=0
FORCE_UNSUPPORTED=0
FORCE_REEXTRACT=0
RECREATE_VENV=0
MASK_MODEMMANAGER=0
CHECK_ONLY=0
RUN_AFTER_INSTALL=0
RUN_MODE="web"

detect_dist_version() {
  if [[ -n "${UUV_SIM_DIST_VERSION:-}" ]]; then
    printf '%s\n' "$UUV_SIM_DIST_VERSION"
    return 0
  fi
  if [[ -f "${BUNDLE_ROOT}/VERSION" ]]; then
    tr -d '[:space:]' <"${BUNDLE_ROOT}/VERSION"
    return 0
  fi
  if [[ -f "${BUNDLE_ROOT}/RELEASE_MANIFEST.txt" ]]; then
    local manifest_version
    manifest_version="$(awk -F': ' '$1 == "Version" { print $2; exit }' "${BUNDLE_ROOT}/RELEASE_MANIFEST.txt" 2>/dev/null || true)"
    if [[ -n "$manifest_version" ]]; then
      printf '%s\n' "$manifest_version"
      return 0
    fi
    printf '%s\n' "unknown"
    return 0
  fi
  printf '%s\n' "unknown"
}

DIST_VERSION="$(detect_dist_version)"

usage() {
  cat <<'USAGE'
Usage: ./install_uuv_sim_current_ubuntu22.sh [options]

Install the current UUV MuJoCo/SITL runtime on Ubuntu 22.04. The installer
extracts uuv_mujoco.zip beside this script, installs apt/Python/ROS
dependencies, clones ArduPilot, builds bundled ROS message/helper packages,
and writes sim/environment.sh.

Options:
  --install-root PATH       Install workspace here (default: script directory)
  --uuv-zip PATH            Use this uuv_mujoco.zip file
  --venv-root PATH          Python virtualenv path (default: ~/.venvs/uuv_mujoco)
  --mujoco-pip-spec SPEC    MuJoCo pip spec (default: mujoco==3.8.0)
  --ardupilot-branch NAME   Clone a specific ArduPilot branch
  --without-ros2            Skip ROS 2 apt packages and ROS helper build
  --skip-apt                Skip all apt/system package installation
  --skip-qgc                Skip QGroundControl AppImage download
  --skip-ardupilot          Skip ArduPilot clone/update
  --skip-ardupilot-prereqs  Skip ArduPilot Ubuntu prereq helper
  --skip-ardupilot-build    Skip the ArduSub SITL prebuild
  --skip-rospkg-build       Skip bundled ROS2 helper package build
  --skip-python-env         Skip Python virtualenv package install
  --force-reextract         Replace existing uuv_mujoco directory from zip
  --recreate-venv           Remove and recreate the Python virtualenv
  --mask-modemmanager       Stop and mask ModemManager for QGC serial access
  --check-only              Run preflight only
  --run-after-install       Start web GUI after successful install
  --run-web                 Start web GUI after install
  --run-headless            Start SITL+MuJoCo headless after install
  --run-gui                 Start desktop/Tk GUI after install
  --noninteractive          Use noninteractive apt
  --force-unsupported       Continue even if OS is not Ubuntu 22.04
  -h, --help                Show this help

Recommended fresh install:
  ./install_uuv_sim_current_ubuntu22.sh --noninteractive
  source ./sim/environment.sh
  ./run_control_gui.sh --web --host 127.0.0.1 --port 8878
USAGE
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --install-root)
      [[ $# -ge 2 ]] || { echo "[install] --install-root requires a value" >&2; exit 2; }
      INSTALL_ROOT="$2"
      shift 2
      ;;
    --uuv-zip)
      [[ $# -ge 2 ]] || { echo "[install] --uuv-zip requires a value" >&2; exit 2; }
      UUV_ZIP="$2"
      shift 2
      ;;
    --venv-root)
      [[ $# -ge 2 ]] || { echo "[install] --venv-root requires a value" >&2; exit 2; }
      VENV_ROOT="$2"
      shift 2
      ;;
    --mujoco-pip-spec)
      [[ $# -ge 2 ]] || { echo "[install] --mujoco-pip-spec requires a value" >&2; exit 2; }
      MUJOCO_PIP_SPEC="$2"
      shift 2
      ;;
    --ardupilot-branch)
      [[ $# -ge 2 ]] || { echo "[install] --ardupilot-branch requires a value" >&2; exit 2; }
      ARDUPILOT_BRANCH="$2"
      shift 2
      ;;
    --without-ros2)
      WITH_ROS2=0
      SKIP_ROSPKG_BUILD=1
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
    --skip-ardupilot)
      SKIP_ARDUPILOT=1
      shift
      ;;
    --skip-ardupilot-prereqs)
      SKIP_ARDUPILOT_PREREQS=1
      shift
      ;;
    --skip-ardupilot-build)
      SKIP_ARDUPILOT_BUILD=1
      shift
      ;;
    --skip-rospkg-build)
      SKIP_ROSPKG_BUILD=1
      shift
      ;;
    --skip-python-env)
      SKIP_PYTHON_ENV=1
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
    --check-only)
      CHECK_ONLY=1
      shift
      ;;
    --run-after-install|--run-web)
      RUN_AFTER_INSTALL=1
      RUN_MODE="web"
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
      echo "[install] unknown option: $1" >&2
      usage
      exit 2
      ;;
  esac
done

# A rebuilt installer can intentionally reuse the public dist version while
# carrying corrected payload bytes.  The one-click wrapper sets this flag so
# reinstalling the same version still refreshes the runtime and ROS sources.
case "${UUV_SIM_FORCE_REFRESH:-0}" in
  1|true|TRUE|yes|YES|on|ON)
    FORCE_REEXTRACT=1
    ;;
esac

INSTALL_ROOT="$(mkdir -p "${INSTALL_ROOT}" && cd "${INSTALL_ROOT}" && pwd)"
UUV_ZIP="$(cd "$(dirname "${UUV_ZIP}")" && pwd)/$(basename "${UUV_ZIP}")"
ARDUPILOT_DIR="${ARDUPILOT_DIR:-${INSTALL_ROOT}/sim/ardupilot}"
UUV_MUJOCO_DIR="${UUV_MUJOCO_DIR:-${INSTALL_ROOT}/sim/current}"
ROS_WORKSPACE_DIR="${ROS_WORKSPACE_DIR:-${INSTALL_ROOT}/rospkg}"
ROS_SOURCE_DIR="${ROS_SOURCE_DIR:-${ROS_WORKSPACE_DIR}/src}"
PAYLOAD_REFRESH="$FORCE_REEXTRACT"
WORKSPACE_VERSION_FILE="${INSTALL_ROOT}/.uuv_sim_current_version"
INSTALLED_WORKSPACE_VERSION=""
if [[ -f "$WORKSPACE_VERSION_FILE" ]]; then
  INSTALLED_WORKSPACE_VERSION="$(tr -d '[:space:]' <"$WORKSPACE_VERSION_FILE")"
fi
if [[ -n "$DIST_VERSION" && "$DIST_VERSION" != "unknown" && "$INSTALLED_WORKSPACE_VERSION" != "$DIST_VERSION" ]]; then
  PAYLOAD_REFRESH=1
fi
if [[ -z "$QGC_APP" ]]; then
  QGC_APP="${INSTALL_ROOT}/QGroundControl-x86_64.AppImage"
fi
QGC_APP="$(cd "$(dirname "${QGC_APP}")" && pwd)/$(basename "${QGC_APP}")"

log() {
  echo "[uuv-current-dist] $*"
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

require_cmd() {
  command -v "$1" >/dev/null 2>&1 || { echo "[install] command not found: $1" >&2; exit 1; }
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
      log "warning: apt package not available, skipped: $pkg"
    fi
  done
  if ((${#installable[@]} > 0)); then
    log "installing ${label}"
    apt_run apt-get install -y "${installable[@]}"
  fi
}

check_ubuntu_2204() {
  if [[ ! -f /etc/os-release ]]; then
    [[ "$FORCE_UNSUPPORTED" -eq 1 ]] && return 0
    echo "[install] /etc/os-release missing; expected Ubuntu 22.04" >&2
    exit 1
  fi
  # shellcheck source=/dev/null
  source /etc/os-release
  if [[ "${ID:-}" == "ubuntu" && "${VERSION_ID:-}" == "22.04" ]]; then
    return 0
  fi
  if [[ "$FORCE_UNSUPPORTED" -eq 1 ]]; then
    log "warning: unsupported OS ${PRETTY_NAME:-unknown}; continuing"
    return 0
  fi
  echo "[install] unsupported OS: ${PRETTY_NAME:-unknown}; expected Ubuntu 22.04" >&2
  exit 1
}

run_preflight() {
  if [[ -x "${SCRIPT_DIR}/preflight_uuv_sim_current.sh" ]]; then
    "${SCRIPT_DIR}/preflight_uuv_sim_current.sh" --install-root "$INSTALL_ROOT" --ros-distro "$ROS_DISTRO" || true
  fi
}

ensure_ros_apt_repo() {
  local codename arch keyring repo_file repo_line
  codename="$(. /etc/os-release && printf '%s' "${UBUNTU_CODENAME:-jammy}")"
  arch="$(dpkg --print-architecture)"
  keyring="/usr/share/keyrings/ros-archive-keyring.gpg"
  repo_file="/etc/apt/sources.list.d/ros2.list"
  repo_line="deb [arch=${arch} signed-by=${keyring}] http://packages.ros.org/ros2/ubuntu ${codename} main"

  repair_ros_apt_source_conflicts "$repo_file"
  sudo_run mkdir -p /usr/share/keyrings
  if [[ ! -f "$keyring" ]]; then
    local tmp_key
    tmp_key="$(mktemp)"
    require_cmd curl
    require_cmd gpg
    run curl -fsSL -o "$tmp_key" https://raw.githubusercontent.com/ros/rosdistro/master/ros.key
    sudo_run gpg --dearmor -o "$keyring" "$tmp_key"
    rm -f "$tmp_key"
  fi
  printf '%s\n' "$repo_line" | sudo_run tee "$repo_file" >/dev/null
}

repair_ros_apt_source_conflicts() {
  local canonical="$1"
  local stamp path backup tmp
  stamp="$(date +%Y%m%d%H%M%S)"
  shopt -s nullglob
  for path in \
    /etc/apt/sources.list \
    /etc/apt/sources.list.d/*.list \
    /etc/apt/sources.list.d/*.sources
  do
    [[ -f "$path" ]] || continue
    grep -Fq "packages.ros.org/ros2/ubuntu" "$path" 2>/dev/null || continue
    if [[ "$path" == "$canonical" ]]; then
      continue
    fi
    backup="${path}.uuv-sim-current-disabled-${stamp}"
    log "disabling conflicting ROS 2 apt source: ${path} -> ${backup}"
    if [[ "$path" == "/etc/apt/sources.list" ]]; then
      tmp="$(mktemp)"
      awk '
        /packages\.ros\.org\/ros2\/ubuntu/ {
          print "# disabled by uuv-sim-current: " $0
          next
        }
        { print }
      ' "$path" >"$tmp"
      sudo_run cp -f "$path" "$backup"
      sudo_run cp -f "$tmp" "$path"
      rm -f "$tmp"
    else
      sudo_run mv -f "$path" "$backup"
    fi
  done
  shopt -u nullglob
}

install_system_packages() {
  [[ "$SKIP_APT" -eq 0 ]] || { log "skipping apt install"; return 0; }

  log "installing Ubuntu 22.04 system dependencies"
  if [[ "$WITH_ROS2" -eq 1 ]]; then
    ensure_ros_apt_repo
  fi
  apt_run apt-get update
  apt_run apt-get install -y \
    git curl unzip zip ca-certificates gnupg lsb-release software-properties-common \
    build-essential ccache gawk make cmake pkg-config gcc g++ \
    python3 python3-venv python3-pip python3-dev python3-tk python3-numpy \
    ffmpeg jq xz-utils file lsof iproute2 net-tools ripgrep can-utils \
    libeigen3-dev libboost-dev libboost-thread-dev \
    libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev \
    libgl1 libegl1 libglfw3 libgl1-mesa-dri libglvnd0 libglx0 libopengl0 \
    libxrender1 libxext6 libxi6 libxrandr2 libxxf86vm1 libxinerama1 libxcursor1

  apt_install_available "display and headless OpenGL helpers" \
    mesa-utils libosmesa6 xwayland libdecor-0-0

  apt_install_available "QGroundControl Qt/XCB/AppImage helpers" \
    libfuse2 libxcb-xinerama0 libxkbcommon-x11-0 libxcb-cursor0 \
    libxcb-icccm4 libxcb-image0 libxcb-keysyms1 libxcb-render-util0 \
    libxcb-randr0 libxcb-shape0 libxcb-xfixes0 libxcb-sync1 libxcb-shm0 \
    libxcb-render0 libxcb-glx0

  apt_install_available "GStreamer video helpers" \
    gstreamer1.0-alsa gstreamer1.0-plugins-base gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-ugly gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl \
    python3-gi python3-gst-1.0

  apt_install_available "KMU26 web GUI runtime" \
    python3-fastapi python3-uvicorn python3-yaml python3-websockets

  if [[ "$WITH_ROS2" -eq 1 ]]; then
    ensure_ros_apt_repo
    apt_run apt-get update
    local ros_pkgs=(
      python3-rosdep
      python3-vcstool
      python3-colcon-common-extensions
      geographiclib-tools
      "ros-${ROS_DISTRO}-ros-base"
      "ros-${ROS_DISTRO}-ament-cmake"
      "ros-${ROS_DISTRO}-ament-index-python"
      "ros-${ROS_DISTRO}-rclcpp"
      "ros-${ROS_DISTRO}-rclcpp-components"
      "ros-${ROS_DISTRO}-diagnostic-updater"
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
      "ros-${ROS_DISTRO}-rosidl-default-generators"
      "ros-${ROS_DISTRO}-rosidl-default-runtime"
      "ros-${ROS_DISTRO}-rqt-bag"
      "ros-${ROS_DISTRO}-rqt-image-view"
      "ros-${ROS_DISTRO}-image-transport"
      "ros-${ROS_DISTRO}-rviz2"
      "ros-${ROS_DISTRO}-robot-state-publisher"
      "ros-${ROS_DISTRO}-robot-localization"
      "ros-${ROS_DISTRO}-launch-xml"
      "ros-${ROS_DISTRO}-xacro"
    )
    apt_install_available "ROS 2 ${ROS_DISTRO} runtime packages" "${ros_pkgs[@]}"
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
    log "dialout group requested for ${target_user}; log out/in once for USB serial"
  fi
  if [[ "$MASK_MODEMMANAGER" -eq 1 ]] && systemctl list-unit-files ModemManager.service >/dev/null 2>&1; then
    sudo_run systemctl mask --now ModemManager.service || true
  fi
}

extract_uuv_mujoco() {
  [[ -f "$UUV_ZIP" ]] || { echo "[install] uuv_mujoco.zip not found: $UUV_ZIP" >&2; exit 1; }
  require_cmd unzip
  local runtime_version_file="${UUV_MUJOCO_DIR}/.uuv_runtime_payload_version"
  local installed_version=""
  local should_reextract="$FORCE_REEXTRACT"
  if [[ -f "$runtime_version_file" ]]; then
    installed_version="$(tr -d '[:space:]' <"$runtime_version_file")"
  elif [[ -f "$WORKSPACE_VERSION_FILE" ]]; then
    installed_version="$(tr -d '[:space:]' <"$WORKSPACE_VERSION_FILE")"
  fi
  if [[ -d "$UUV_MUJOCO_DIR" && "$should_reextract" -eq 0 && -n "$DIST_VERSION" && "$DIST_VERSION" != "unknown" ]]; then
    if [[ "$installed_version" != "$DIST_VERSION" ]]; then
      log "runtime payload version changed: ${installed_version:-unknown} -> ${DIST_VERSION}; reextracting runtime"
      should_reextract=1
    fi
  fi
  if [[ -d "$UUV_MUJOCO_DIR" && "$should_reextract" -eq 0 ]]; then
    log "reusing existing runtime: $UUV_MUJOCO_DIR"
  else
    PAYLOAD_REFRESH=1
    local tmpdir extracted_root previous_runtime runtime_parent had_previous=0
    local rel source_path destination_path
    runtime_parent="$(dirname "$UUV_MUJOCO_DIR")"
    run mkdir -p "$runtime_parent"
    # Stage beside the destination so the final directory rename stays on the
    # same filesystem and cannot expose a half-copied runtime tree.
    tmpdir="$(mktemp -d "${runtime_parent}/.uuv-runtime-stage.XXXXXX")"
    trap 'rm -rf "${tmpdir:-}"' RETURN
    run unzip -q "$UUV_ZIP" -d "$tmpdir"
    extracted_root="$(find "$tmpdir" -type f -path '*/sim/current/run_uuv_mujoco.py' -print -quit)"
    [[ -n "$extracted_root" ]] || { echo "[install] current runtime not found in zip" >&2; exit 1; }
    extracted_root="$(dirname "$extracted_root")"
    previous_runtime="${UUV_MUJOCO_DIR}.upgrade_tmp.$$"
    if [[ -d "$UUV_MUJOCO_DIR" ]]; then
      local installed_stack_processes=""
      installed_stack_processes="$(
        pgrep -af 'run_uuv_mujoco.py|start_sitl_mujoco_mj311.sh|launch_uuv_sim.sh|Tools/autotest/sim_vehicle.py|build/sitl/bin/ardusub' 2>/dev/null \
          | grep -F -- "$INSTALL_ROOT" || true
      )"
      if [[ -n "$installed_stack_processes" && -x "${UUV_MUJOCO_DIR}/reset_uuv_sim.sh" ]]; then
        log "stopping the simulator running from this install root before replacing runtime files"
        WORKSPACE_DIR="$INSTALL_ROOT" ARDUPILOT_DIR="$ARDUPILOT_DIR" \
          "${UUV_MUJOCO_DIR}/reset_uuv_sim.sh" || \
          log "warning: runtime reset reported an error; continuing the versioned file swap"
      else
        log "no simulator process is running from this install root; upgrading files in place"
      fi
      # Merge mutable user state into the fully extracted staging tree before
      # either live directory is renamed.  A copy failure therefore leaves the
      # installed runtime untouched.
      for rel in \
        config/course_layout.json \
        config/ping360.json \
        config/sim_profiles.json \
        config/thruster_params.json \
        config/thruster_performance.json
      do
        source_path="${UUV_MUJOCO_DIR}/${rel}"
        destination_path="${extracted_root}/${rel}"
        [[ -f "$source_path" ]] || continue
        run mkdir -p "$(dirname "$destination_path")"
        run cp -a "$source_path" "$destination_path"
      done
      for rel in logs generated; do
        source_path="${UUV_MUJOCO_DIR}/${rel}"
        destination_path="${extracted_root}/${rel}"
        [[ -d "$source_path" ]] || continue
        run mkdir -p "$destination_path"
        run cp -a "${source_path}/." "${destination_path}/"
      done
      log "user course, sonar, physics settings, logs, and generated state were preserved"
      run rm -rf "$previous_runtime"
      run mv "$UUV_MUJOCO_DIR" "$previous_runtime"
      had_previous=1
    fi
    if ! mv "$extracted_root" "$UUV_MUJOCO_DIR"; then
      if [[ "$had_previous" -eq 1 && -d "$previous_runtime" && ! -e "$UUV_MUJOCO_DIR" ]]; then
        mv "$previous_runtime" "$UUV_MUJOCO_DIR" || true
      fi
      echo "[install] failed to activate the new runtime; restored the previous runtime" >&2
      exit 1
    fi
    if [[ "$had_previous" -eq 1 ]]; then
      run rm -rf "$previous_runtime"
    fi
    rm -rf "$tmpdir"
    trap - RETURN
  fi
  printf '%s\n' "$DIST_VERSION" >"$runtime_version_file"
  for script in \
    "$UUV_MUJOCO_DIR/launch_uuv_sim.sh" \
    "$UUV_MUJOCO_DIR/start_sitl_mujoco_mj311.sh" \
    "$UUV_MUJOCO_DIR/start_ardusub_sitl_mj311.sh" \
    "$UUV_MUJOCO_DIR/reset_uuv_sim.sh" \
    "$UUV_MUJOCO_DIR/run_control_gui.sh"
  do
    [[ -f "$script" ]] || { echo "[install] missing runtime script: $script" >&2; exit 1; }
    chmod +x "$script"
  done
}

install_support_files() {
  mkdir -p "$INSTALL_ROOT"
  if [[ -f "${BUNDLE_ROOT}/run_control_gui.sh" && "${BUNDLE_ROOT}/run_control_gui.sh" != "${INSTALL_ROOT}/run_control_gui.sh" ]]; then
    run cp -f "${BUNDLE_ROOT}/run_control_gui.sh" "${INSTALL_ROOT}/run_control_gui.sh"
  fi
  [[ ! -f "${INSTALL_ROOT}/run_control_gui.sh" ]] || chmod +x "${INSTALL_ROOT}/run_control_gui.sh" 2>/dev/null || true
  if [[ -s "${BUNDLE_ROOT}/sim/current/assets/yolo/best.pt" ]]; then
    run mkdir -p "${INSTALL_ROOT}/sim/current/assets/yolo"
    if [[ "${BUNDLE_ROOT}/sim/current/assets/yolo/best.pt" != "${INSTALL_ROOT}/sim/current/assets/yolo/best.pt" ]]; then
      run cp -f "${BUNDLE_ROOT}/sim/current/assets/yolo/best.pt" "${INSTALL_ROOT}/sim/current/assets/yolo/best.pt"
    fi
  else
    echo "[install] bundled YOLO model missing: ${BUNDLE_ROOT}/sim/current/assets/yolo/best.pt" >&2
    exit 1
  fi
}

setup_ardupilot() {
  [[ "$SKIP_ARDUPILOT" -eq 0 ]] || { log "skipping ArduPilot setup"; return 0; }
  require_cmd git
  local new_checkout=0
  if [[ -e "$ARDUPILOT_DIR/.git" ]] && git -C "$ARDUPILOT_DIR" rev-parse --git-dir >/dev/null 2>&1; then
    log "updating existing ArduPilot checkout: $ARDUPILOT_DIR"
  elif [[ -e "$ARDUPILOT_DIR" ]]; then
    echo "[install] ARDUPILOT_DIR exists but is not a git checkout: $ARDUPILOT_DIR" >&2
    exit 1
  else
    local clone_args=()
    [[ -z "$ARDUPILOT_BRANCH" ]] || clone_args+=(--branch "$ARDUPILOT_BRANCH")
    run git clone "${clone_args[@]}" "$ARDUPILOT_REMOTE" "$ARDUPILOT_DIR"
    new_checkout=1
  fi
  if [[ "$new_checkout" -eq 1 && -n "$ARDUPILOT_BASE_COMMIT" ]]; then
    run git -C "$ARDUPILOT_DIR" checkout --detach "$ARDUPILOT_BASE_COMMIT"
  elif [[ -n "$ARDUPILOT_BASE_COMMIT" ]]; then
    local installed_head
    installed_head="$(git -C "$ARDUPILOT_DIR" rev-parse HEAD)"
    if [[ "$installed_head" != "$ARDUPILOT_BASE_COMMIT" ]]; then
      echo "[install] managed ArduPilot checkout is at ${installed_head}, expected ${ARDUPILOT_BASE_COMMIT}" >&2
      echo "[install] move that checkout aside or use --skip-ardupilot" >&2
      exit 1
    fi
  fi
  run git -C "$ARDUPILOT_DIR" submodule sync --recursive
  run git -C "$ARDUPILOT_DIR" submodule update --init --recursive
  if [[ -f "$ARDUPILOT_PATCH" ]]; then
    if git -C "$ARDUPILOT_DIR" apply --check "$ARDUPILOT_PATCH"; then
      run git -C "$ARDUPILOT_DIR" apply "$ARDUPILOT_PATCH"
    elif git -C "$ARDUPILOT_DIR" apply --reverse --check "$ARDUPILOT_PATCH"; then
      log "ArduPilot workspace patch already applied"
    else
      echo "[install] ArduPilot patch does not apply cleanly: $ARDUPILOT_PATCH" >&2
      exit 1
    fi
  fi
  [[ -f "$ARDUPILOT_DIR/Tools/autotest/sim_vehicle.py" ]] || {
    echo "[install] sim_vehicle.py missing after ArduPilot setup" >&2
    exit 1
  }
  if [[ "$SKIP_ARDUPILOT_PREREQS" -eq 0 && -x "$ARDUPILOT_DIR/Tools/environment_install/install-prereqs-ubuntu.sh" ]]; then
    run "$ARDUPILOT_DIR/Tools/environment_install/install-prereqs-ubuntu.sh" -y
  fi
  if [[ "$SKIP_ARDUPILOT_BUILD" -eq 0 ]]; then
    log "building pinned ArduSub SITL binary"
    (
      cd "$ARDUPILOT_DIR"
      run ./waf configure --board sitl
      run ./waf sub -j "${UUV_DIST_BUILD_JOBS:-1}"
    )
    [[ -x "$ARDUPILOT_DIR/build/sitl/bin/ardusub" ]] || {
      echo "[install] ArduSub SITL build did not produce build/sitl/bin/ardusub" >&2
      exit 1
    }
  fi
}

setup_python_env() {
  [[ "$SKIP_PYTHON_ENV" -eq 0 ]] || { log "skipping Python environment setup"; return 0; }
  require_cmd python3
  if [[ "$RECREATE_VENV" -eq 1 && -d "$VENV_ROOT" ]]; then
    run rm -rf "$VENV_ROOT"
  fi
  if [[ ! -d "$VENV_ROOT" ]]; then
    run python3 -m venv --system-site-packages "$VENV_ROOT"
  fi
  local py="${VENV_ROOT}/bin/python"
  [[ -x "$py" ]] || { echo "[install] venv python missing: $py" >&2; exit 1; }
  run env -u PYTHONPATH -u PYTHONHOME "$py" -m pip install -U pip "setuptools<80" wheel packaging
  run env -u PYTHONPATH -u PYTHONHOME "$py" -m pip install -U \
    --index-url https://download.pytorch.org/whl/cpu \
    torch torchvision
  run env -u PYTHONPATH -u PYTHONHOME "$py" -m pip install -U \
    "numpy<2" matplotlib rosbags python-pptx "$MUJOCO_PIP_SPEC" \
    pymavlink MAVProxy pexpect pillow future dronecan gnureadline "empy==3.3.4" \
    "opencv-python-headless<5" ultralytics
  env -u PYTHONPATH -u PYTHONHOME "$py" - <<'PY'
import cv2
import mujoco
import mujoco.viewer
import numpy
import pymavlink
import MAVProxy
import pexpect
import PIL
import rosbags
import pptx
import torch
import ultralytics
print("python deps ok")
print("mujoco", mujoco.__version__)
print("opencv", cv2.__version__)
print("torch", torch.__version__)
print("ultralytics", ultralytics.__version__)
PY
}

extract_ros_zip() {
  local pkg="$1"
  local zip_path="$2"
  local destination="${ROS_SOURCE_DIR}/${pkg}"
  local legacy_destination="${ROS_WORKSPACE_DIR}/${pkg}"
  [[ -f "$zip_path" ]] || return 0
  if [[ "$PAYLOAD_REFRESH" -eq 1 ]]; then
    if [[ -e "$destination" || -L "$destination" ]]; then
      log "distribution changed; refreshing ROS package: ${destination}"
      run rm -rf "$destination"
    fi
    if [[ -e "$legacy_destination" || -L "$legacy_destination" ]]; then
      log "removing pre-src ROS package layout: ${legacy_destination}"
      run rm -rf "$legacy_destination"
    fi
  fi
  if [[ -d "$destination" ]]; then
    log "reusing ROS package: ${destination}"
    return 0
  fi
  if [[ -d "$legacy_destination" ]]; then
    log "migrating ROS package into src/: ${legacy_destination}"
    run mkdir -p "$ROS_SOURCE_DIR"
    run mv "$legacy_destination" "$destination"
    return 0
  fi
  local tmpdir package_root
  tmpdir="$(mktemp -d)"
  trap 'rm -rf "${tmpdir:-}"' RETURN
  run unzip -q "$zip_path" -d "$tmpdir"
  package_root="$(find "$tmpdir" -type f -path "*/${pkg}/package.xml" -print -quit)"
  [[ -n "$package_root" ]] || { echo "[install] ${pkg}/package.xml not found in $zip_path" >&2; exit 1; }
  run mkdir -p "$ROS_SOURCE_DIR"
  run mv "$(dirname "$package_root")" "$destination"
  rm -rf "$tmpdir"
  trap - RETURN
}

setup_ros_packages() {
  [[ "$WITH_ROS2" -eq 1 ]] || { log "skipping bundled ROS package extraction and build"; return 0; }
  run mkdir -p "$ROS_SOURCE_DIR"
  if [[ "$SOURCE_CHECKOUT" -eq 1 ]]; then
    [[ "$SKIP_ROSPKG_BUILD" -eq 0 ]] || { log "source checkout detected; skipping ROS build"; return 0; }
    [[ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]] || {
      echo "[install] ROS setup missing: /opt/ros/${ROS_DISTRO}/setup.bash" >&2
      exit 1
    }
    if command -v rosdep >/dev/null 2>&1; then
      rosdep install --from-paths "$ROS_SOURCE_DIR" --ignore-src -r -y ||
        log "warning: rosdep reported unresolved dependencies; attempting the build"
    fi
    (
      set +u
      # shellcheck source=/dev/null
      source "/opt/ros/${ROS_DISTRO}/setup.bash"
      set -u
      cd "$ROS_WORKSPACE_DIR"
      python3 -m pip install --user 'setuptools<80'
      env CMAKE_BUILD_PARALLEL_LEVEL="${UUV_DIST_BUILD_JOBS:-1}" \
        colcon build --symlink-install --executor sequential \
          --cmake-args -DBUILD_TESTING=OFF
    )
    return 0
  fi
  if [[ "$PAYLOAD_REFRESH" -eq 1 ]]; then
    # dist4 has one active vision implementation. Remove the superseded custom
    # package and its isolated colcon prefix so an in-place upgrade cannot keep
    # exposing duplicate FSM/RC executables from an older release.
    for obsolete_path in \
      "${ROS_SOURCE_DIR}/kmu26_vision_mission_fsm" \
      "${ROS_WORKSPACE_DIR}/kmu26_vision_mission_fsm" \
      "${ROS_WORKSPACE_DIR}/build/kmu26_vision_mission_fsm" \
      "${ROS_WORKSPACE_DIR}/install/kmu26_vision_mission_fsm"
    do
      if [[ -e "$obsolete_path" || -L "$obsolete_path" ]]; then
        log "removing superseded vision FSM from previous release: ${obsolete_path}"
        run rm -rf "$obsolete_path"
      fi
    done
  fi
  if [[ -f "${BUNDLE_ROOT}/rospkg/README.md" ]]; then
    run cp -a "${BUNDLE_ROOT}/rospkg/README.md" "${ROS_SOURCE_DIR}/README.md"
  fi
  extract_ros_zip dvl_msgs "${BUNDLE_ROOT}/rospkg/dvl_msgs.zip"
  extract_ros_zip auv_dvl_a50_msg "${BUNDLE_ROOT}/rospkg/auv_dvl_a50_msg.zip"
  extract_ros_zip ping360_sonar_msgs "${BUNDLE_ROOT}/rospkg/ping360_sonar_msgs.zip"
  extract_ros_zip kmu26_auv_msg "${BUNDLE_ROOT}/rospkg/kmu26_auv_msg.zip"
  extract_ros_zip kmu26_auv "${BUNDLE_ROOT}/rospkg/kmu26_auv.zip"
  extract_ros_zip audio_common_msgs "${BUNDLE_ROOT}/rospkg/audio_common_msgs.zip"
  extract_ros_zip audio_common "${BUNDLE_ROOT}/rospkg/audio_common.zip"
  extract_ros_zip audio_capture "${BUNDLE_ROOT}/rospkg/audio_capture.zip"
  extract_ros_zip hydrophone_ctrl "${BUNDLE_ROOT}/rospkg/hydrophone_ctrl.zip"
  extract_ros_zip kmu26_auv_buoy_vision_control "${BUNDLE_ROOT}/rospkg/kmu26_auv_buoy_vision_control.zip"
  extract_ros_zip auv_lane_vision_control "${BUNDLE_ROOT}/rospkg/auv_lane_vision_control.zip"
  extract_ros_zip kmu26_auv_web_gui "${BUNDLE_ROOT}/rospkg/kmu26_auv_web_gui.zip"
  extract_ros_zip kmu26_pinger_homing "${BUNDLE_ROOT}/rospkg/kmu26_pinger_homing.zip"
  extract_ros_zip robot_localization "${BUNDLE_ROOT}/rospkg/robot_localization.zip"
  [[ "$SKIP_ROSPKG_BUILD" -eq 0 ]] || { log "ROS sources refreshed; skipping ROS helper package build"; return 0; }
  [[ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]] || { log "ROS setup missing; skipping ROS helper package build"; return 0; }
  require_cmd colcon
  (
    set +u
    # shellcheck source=/dev/null
    source "/opt/ros/${ROS_DISTRO}/setup.bash"
    set -u
    cd "$ROS_WORKSPACE_DIR"
    python3 -m pip install --user 'setuptools<80'
    env CMAKE_BUILD_PARALLEL_LEVEL="${UUV_DIST_BUILD_JOBS:-1}" \
      colcon build --symlink-install --executor sequential --packages-select \
        dvl_msgs auv_dvl_a50_msg ping360_sonar_msgs auv_msg auv \
        audio_common_msgs audio_common audio_capture hydrophone_ctrl \
        auv_buoy_vision_control auv_lane_vision_control auv_web_gui auv_pinger_homing \
        robot_localization \
        --cmake-args -DBUILD_TESTING=OFF
  )
}

download_qgc() {
  [[ "$SKIP_QGC" -eq 0 ]] || { log "skipping QGroundControl download"; return 0; }
  require_cmd curl
  if [[ -f "$QGC_APP" ]]; then
    log "reusing QGroundControl: $QGC_APP"
  else
    local qgc_tmp
    qgc_tmp="${QGC_APP}.part"
    if ! run curl -fL --retry 3 -o "$qgc_tmp" "$QGC_URL"; then
      rm -f "$qgc_tmp"
      log "warning: QGroundControl download failed; simulator installation will continue"
      return 0
    fi
    run mv "$qgc_tmp" "$QGC_APP"
  fi
  chmod +x "$QGC_APP"
}

write_env_file() {
  local env_file="${INSTALL_ROOT}/sim/environment.sh"
  mkdir -p "$(dirname "$env_file")"
  cat >"$env_file" <<EOF
#!/usr/bin/env bash
export WORKSPACE_DIR="${INSTALL_ROOT}"
export ROS_WORKSPACE_DIR="\${ROS_WORKSPACE_DIR:-\${WORKSPACE_DIR}/rospkg}"
export ROS_SOURCE_DIR="\${ROS_SOURCE_DIR:-\${ROS_WORKSPACE_DIR}/src}"
export UUV_MUJOCO_DIR="\${UUV_MUJOCO_DIR:-\${WORKSPACE_DIR}/sim/current}"
export UUV_MUJOCO_RUNTIME_DIR="\${UUV_MUJOCO_RUNTIME_DIR:-\${UUV_MUJOCO_DIR}}"
export ARDUPILOT_DIR="\${ARDUPILOT_DIR:-\${WORKSPACE_DIR}/sim/ardupilot}"
export KMU26_AUV_DIR="\${KMU26_AUV_DIR:-\${ROS_SOURCE_DIR}/kmu26_auv}"
export KMU26_AUV_MSG_DIR="\${KMU26_AUV_MSG_DIR:-\${ROS_SOURCE_DIR}/kmu26_auv_msg}"
export KMU26_VISION_DIR="\${KMU26_VISION_DIR:-\${ROS_SOURCE_DIR}/kmu26_auv_buoy_vision_control}"
export KMU26_LANE_VISION_DIR="\${KMU26_LANE_VISION_DIR:-\${ROS_SOURCE_DIR}/auv_lane_vision_control}"
export KMU26_WEB_GUI_DIR="\${KMU26_WEB_GUI_DIR:-\${ROS_SOURCE_DIR}/kmu26_auv_web_gui}"
export KMU26_PINGER_HOMING_DIR="\${KMU26_PINGER_HOMING_DIR:-\${ROS_SOURCE_DIR}/kmu26_pinger_homing}"
export UUV_YOLO_MODEL="\${UUV_YOLO_MODEL:-\${WORKSPACE_DIR}/sim/current/assets/yolo/best.pt}"
export ROS_DISTRO="\${ROS_DISTRO:-${ROS_DISTRO}}"
export MJ311_ROOT="\${MJ311_ROOT:-${VENV_ROOT}}"
export MJ311_PYTHON="\${MJ311_PYTHON:-\${MJ311_ROOT}/bin/python}"
if [[ -z "\${MJ311_MJPYTHON:-}" && -x "\${MJ311_ROOT}/bin/mjpython" ]]; then
  export MJ311_MJPYTHON="\${MJ311_ROOT}/bin/mjpython"
fi
export PYTHON_BIN="\${PYTHON_BIN:-\${MJ311_PYTHON}}"
export QGC_APP="\${QGC_APP:-${QGC_APP}}"
export UUV_CONTROL_GUI="\${UUV_CONTROL_GUI:-\${WORKSPACE_DIR}/sim/current/gui/uuv_control_gui.py}"
export QT_QPA_PLATFORM="\${QT_QPA_PLATFORM:-xcb}"
if [[ -z "\${ROS_ENV_SETUP:-}" && -f "/opt/ros/\${ROS_DISTRO}/setup.bash" ]]; then
  export ROS_ENV_SETUP="/opt/ros/\${ROS_DISTRO}/setup.bash"
fi
if [[ -z "\${ROS_INSTALL_SETUP:-}" && -f "\${ROS_WORKSPACE_DIR}/install/setup.bash" ]]; then
  export ROS_INSTALL_SETUP="\${ROS_WORKSPACE_DIR}/install/setup.bash"
fi
EOF
  chmod +x "$env_file"
  log "wrote ${env_file}"
}

verify_install() {
  local fail=0 py="${VENV_ROOT}/bin/python"
  [[ -d "$UUV_MUJOCO_DIR" ]] || { echo "[FAIL] current runtime missing"; fail=1; }
  [[ -x "$INSTALL_ROOT/run_control_gui.sh" ]] || { echo "[FAIL] run_control_gui.sh missing"; fail=1; }
  if [[ "$SKIP_PYTHON_ENV" -eq 0 && ! -x "$py" ]]; then
    echo "[FAIL] Python venv missing"
    fail=1
  fi
  if [[ "$SKIP_ARDUPILOT" -eq 0 && ! -f "$ARDUPILOT_DIR/Tools/autotest/sim_vehicle.py" ]]; then
    echo "[FAIL] ArduPilot sim_vehicle.py missing"
    fail=1
  fi
  if [[ "$SOURCE_CHECKOUT" -eq 0 ]] &&
     ! grep -Fq '"UUV_MUJOCO_TIMESTEP": "0.005"' "$UUV_MUJOCO_DIR/gui/sim_stack_env_defaults.py"; then
    echo "[FAIL] current runtime does not have 0.005 competition timestep default"
    fail=1
  fi
  if [[ ! -s "${INSTALL_ROOT}/sim/current/assets/yolo/best.pt" ]]; then
    echo "[FAIL] sim/current/assets/yolo/best.pt missing"
    fail=1
  fi
  if [[ "$WITH_ROS2" -eq 1 ]]; then
    if [[ "$SOURCE_CHECKOUT" -eq 1 ]]; then
      if ! find "$ROS_SOURCE_DIR" -name package.xml -type f -print -quit | grep -q .; then
        echo "[FAIL] no ROS package sources found under: ${ROS_SOURCE_DIR}"
        fail=1
      fi
    else
      local package_dir
      for package_dir in \
        dvl_msgs auv_dvl_a50_msg ping360_sonar_msgs kmu26_auv_msg kmu26_auv \
        audio_common_msgs audio_common audio_capture hydrophone_ctrl \
        kmu26_auv_buoy_vision_control auv_lane_vision_control \
        kmu26_auv_web_gui kmu26_pinger_homing \
        robot_localization
      do
        if [[ ! -f "${ROS_SOURCE_DIR}/${package_dir}/package.xml" ]]; then
          echo "[FAIL] bundled ROS source missing: ${package_dir}/package.xml"
          fail=1
        fi
      done
      if [[ ! -s "${ROS_SOURCE_DIR}/kmu26_auv_buoy_vision_control/models/best.pt" ]]; then
        echo "[FAIL] packaged vision model missing"
        fail=1
      elif ! cmp -s "${INSTALL_ROOT}/sim/current/assets/yolo/best.pt" \
        "${ROS_SOURCE_DIR}/kmu26_auv_buoy_vision_control/models/best.pt"; then
        echo "[FAIL] GUI and ROS vision model files differ"
        fail=1
      fi
    fi
  fi
  if [[ "$fail" -ne 0 ]]; then
    exit 1
  fi
  if [[ -x "${INSTALL_ROOT}/sim/current/tools/install/preflight_uuv_sim_current.sh" ]]; then
    "${INSTALL_ROOT}/sim/current/tools/install/preflight_uuv_sim_current.sh" --install-root "$INSTALL_ROOT" --python "$py" --ros-distro "$ROS_DISTRO" --post-install || true
  fi
}

mark_install_complete() {
  printf '%s\n' "$DIST_VERSION" >"$WORKSPACE_VERSION_FILE"
  log "workspace version committed: ${INSTALLED_WORKSPACE_VERSION:-none} -> ${DIST_VERSION}"
}

print_next_steps() {
  cat <<EOF

[uuv-current-dist] install complete

Workspace:
  ${INSTALL_ROOT}

Run web GUI:
  cd "${INSTALL_ROOT}"
  source ./sim/environment.sh
  ./run_control_gui.sh --web --host 127.0.0.1 --port 8878

Run desktop GUI:
  cd "${INSTALL_ROOT}"
  source ./sim/environment.sh
  ./run_control_gui.sh

Run headless smoke:
  cd "${INSTALL_ROOT}/sim/current"
  source "${INSTALL_ROOT}/sim/environment.sh"
  READY_WAIT_SECS=60 SITL_WAIT_SECS=360 ./start_sitl_mujoco_mj311.sh -- --headless

Wayland/X11 notes:
  - Wayland is supported through XWayland/libdecor packages installed above.
  - A GLFW warning about window position on Wayland is non-fatal.
  - For QGroundControl AppImage, QT_QPA_PLATFORM defaults to xcb in sim/environment.sh.
EOF
}

run_after_install() {
  # shellcheck source=/dev/null
  source "${INSTALL_ROOT}/sim/environment.sh"
  case "$RUN_MODE" in
    web)
      run "${INSTALL_ROOT}/run_control_gui.sh" --web --host 127.0.0.1 --port 8878
      ;;
    gui)
      run "${INSTALL_ROOT}/run_control_gui.sh"
      ;;
    headless)
      cd "${UUV_MUJOCO_DIR}"
      run ./start_sitl_mujoco_mj311.sh -- --headless
      ;;
    *)
      echo "[install] unknown run mode: $RUN_MODE" >&2
      exit 2
      ;;
  esac
}

log "install root: $INSTALL_ROOT"
check_ubuntu_2204
run_preflight
if [[ "$CHECK_ONLY" -eq 1 ]]; then
  exit 0
fi
install_system_packages
extract_uuv_mujoco
install_support_files
setup_ardupilot
setup_ros_packages
download_qgc
setup_python_env
write_env_file
verify_install
mark_install_complete
print_next_steps
if [[ "$RUN_AFTER_INSTALL" -eq 1 ]]; then
  run_after_install
fi
