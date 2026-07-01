#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
INSTALL_ROOT="${INSTALL_ROOT:-${SCRIPT_DIR}}"
ROS_DISTRO="${ROS_DISTRO:-humble}"
PYTHON_BIN="${MJ311_PYTHON:-${PYTHON_BIN:-python3}}"
STRICT=0
POST_INSTALL=0

usage() {
  cat <<'USAGE'
Usage: ./preflight_uuv_sim_current.sh [options]

Check whether this Ubuntu host is ready to install or run the current UUV
MuJoCo/SITL distribution.

Options:
  --install-root PATH   Workspace/package root to inspect
  --python PATH         Python executable to test
  --ros-distro NAME     ROS 2 distro name (default: humble)
  --post-install        Also expect extracted runtime files to exist
  --strict              Treat warnings as failures
  -h, --help            Show this help
USAGE
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --install-root)
      [[ $# -ge 2 ]] || { echo "[preflight] --install-root requires a value" >&2; exit 2; }
      INSTALL_ROOT="$2"
      shift 2
      ;;
    --python)
      [[ $# -ge 2 ]] || { echo "[preflight] --python requires a value" >&2; exit 2; }
      PYTHON_BIN="$2"
      shift 2
      ;;
    --ros-distro)
      [[ $# -ge 2 ]] || { echo "[preflight] --ros-distro requires a value" >&2; exit 2; }
      ROS_DISTRO="$2"
      shift 2
      ;;
    --post-install)
      POST_INSTALL=1
      shift
      ;;
    --strict)
      STRICT=1
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "[preflight] unknown option: $1" >&2
      usage
      exit 2
      ;;
  esac
done

INSTALL_ROOT="$(mkdir -p "${INSTALL_ROOT}" && cd "${INSTALL_ROOT}" && pwd)"

FAILS=0
WARNS=0

pass() {
  printf '[PASS] %s\n' "$*"
}

warn() {
  WARNS=$((WARNS + 1))
  printf '[WARN] %s\n' "$*" >&2
}

fail() {
  FAILS=$((FAILS + 1))
  printf '[FAIL] %s\n' "$*" >&2
}

check_cmd() {
  if command -v "$1" >/dev/null 2>&1; then
    pass "command available: $1"
  else
    fail "command missing: $1"
  fi
}

check_cmd_warn() {
  if command -v "$1" >/dev/null 2>&1; then
    pass "command available: $1"
  else
    warn "command missing: $1"
  fi
}

check_pkg() {
  if dpkg-query -W -f='${Status}' "$1" 2>/dev/null | grep -q 'install ok installed'; then
    pass "apt package installed: $1"
  else
    warn "apt package not installed yet: $1"
  fi
}

check_os() {
  if [[ ! -f /etc/os-release ]]; then
    fail "/etc/os-release missing; expected Ubuntu 22.04"
    return
  fi

  # shellcheck source=/dev/null
  source /etc/os-release
  if [[ "${ID:-}" == "ubuntu" && "${VERSION_ID:-}" == "22.04" ]]; then
    pass "OS is Ubuntu 22.04 (${PRETTY_NAME:-unknown})"
  else
    fail "OS is ${PRETTY_NAME:-unknown}; this dist targets Ubuntu 22.04"
  fi

  local arch
  arch="$(uname -m)"
  case "$arch" in
    x86_64|amd64)
      pass "CPU architecture supported: $arch"
      ;;
    *)
      warn "CPU architecture is $arch; QGroundControl AppImage in this installer is x86_64"
      ;;
  esac
}

check_display_stack() {
  local session_type="${XDG_SESSION_TYPE:-unknown}"
  if [[ -n "${WAYLAND_DISPLAY:-}" ]]; then
    pass "Wayland display detected: ${WAYLAND_DISPLAY} (session=${session_type})"
    check_pkg xwayland
    check_pkg libdecor-0-0
    warn "MuJoCo/GLFW on Wayland may print a non-fatal window-position warning"
  elif [[ -n "${DISPLAY:-}" ]]; then
    pass "X11 display detected: ${DISPLAY} (session=${session_type})"
  else
    warn "no DISPLAY/WAYLAND_DISPLAY; use --run-headless or web GUI without MuJoCo viewer"
  fi

  for pkg in \
    libgl1 libegl1 libglfw3 libgl1-mesa-dri libosmesa6 mesa-utils \
    libxrender1 libxext6 libxi6 libxrandr2 libxxf86vm1 libxinerama1 libxcursor1 \
    libxcb-xinerama0 libxkbcommon-x11-0 libxcb-cursor0 libxcb-icccm4 \
    libxcb-image0 libxcb-keysyms1 libxcb-render-util0 libfuse2
  do
    check_pkg "$pkg"
  done
}

check_ports() {
  local ports=(8878 14550 14551 14660 14661 9002 9003)
  local port
  for port in "${ports[@]}"; do
    if command -v ss >/dev/null 2>&1 && ss -H -ltnup 2>/dev/null | grep -Eq "[:.]${port}\\b"; then
      warn "port appears in use: ${port}"
    elif command -v lsof >/dev/null 2>&1 && lsof -nP -iTCP:"${port}" -sTCP:LISTEN >/dev/null 2>&1; then
      warn "TCP port appears in use: ${port}"
    else
      pass "port free or no listener detected: ${port}"
    fi
  done
}

check_python_imports() {
  if ! command -v "$PYTHON_BIN" >/dev/null 2>&1; then
    warn "Python executable not available yet: ${PYTHON_BIN}"
    return
  fi

  "$PYTHON_BIN" - <<'PY' || {
import importlib
mods = [
    "numpy",
    "mujoco",
    "mujoco.viewer",
    "pymavlink",
    "MAVProxy",
    "pexpect",
    "PIL",
    "rosbags",
    "pptx",
]
missing = []
for name in mods:
    try:
        importlib.import_module(name)
    except Exception as exc:
        missing.append(f"{name}: {exc}")
if missing:
    print("missing python deps:")
    for item in missing:
        print("  " + item)
    raise SystemExit(1)
print("python runtime imports ok")
PY
    warn "Python runtime dependencies are not installed yet for ${PYTHON_BIN}"
    return
  }
  pass "Python runtime imports ok: ${PYTHON_BIN}"
}

check_ros() {
  local setup="/opt/ros/${ROS_DISTRO}/setup.bash"
  if [[ -f "$setup" ]]; then
    pass "ROS 2 setup exists: ${setup}"
  else
    warn "ROS 2 setup missing: ${setup}"
  fi

  for pkg in \
    "ros-${ROS_DISTRO}-ros-base" \
    "ros-${ROS_DISTRO}-mavros" \
    "ros-${ROS_DISTRO}-mavros-msgs" \
    "ros-${ROS_DISTRO}-mavros-extras" \
    "ros-${ROS_DISTRO}-tf2-ros" \
    "ros-${ROS_DISTRO}-image-transport" \
    "ros-${ROS_DISTRO}-rviz2"
  do
    check_pkg "$pkg"
  done

  check_cmd_warn colcon
  check_cmd_warn rosdep
}

check_runtime_files() {
  if [[ "$POST_INSTALL" -eq 0 ]]; then
    return
  fi

  local runtime="${INSTALL_ROOT}/uuv_mujoco/current"
  for path in \
    "${runtime}/run_uuv_mujoco.py" \
    "${runtime}/launch_uuv_sim.sh" \
    "${runtime}/start_sitl_mujoco_mj311.sh" \
    "${runtime}/gui/web_control_gui.py" \
    "${runtime}/gui/sim_stack_env_defaults.py" \
    "${runtime}/sim/runtime/model_runtime_setup.py" \
    "${runtime}/scenes/tank_current_scene.xml" \
    "${INSTALL_ROOT}/run_control_gui.sh"
  do
    if [[ -e "$path" ]]; then
      pass "runtime file exists: ${path#${INSTALL_ROOT}/}"
    else
      fail "runtime file missing: ${path#${INSTALL_ROOT}/}"
    fi
  done

  if grep -Fq '"UUV_MUJOCO_TIMESTEP": "0.005"' "${runtime}/gui/sim_stack_env_defaults.py" \
    && grep -Fq 'course buoy contact timestep guard' "${runtime}/sim/runtime/model_runtime_setup.py"; then
    pass "current runtime includes buoy-contact timestep stability patch"
  else
    fail "current runtime does not include expected buoy-contact stability patch"
  fi
}

check_core_commands() {
  for cmd in bash python3 curl git unzip cmake make gcc g++ pkg-config; do
    check_cmd "$cmd"
  done
  for cmd in zip sha256sum glxinfo; do
    check_cmd_warn "$cmd"
  done
}

check_os
check_core_commands
check_display_stack
check_ros
check_python_imports
check_ports
check_runtime_files

if [[ "$STRICT" -eq 1 && "$WARNS" -gt 0 ]]; then
  fail "strict mode treats ${WARNS} warning(s) as failure"
fi

printf '[preflight] complete: fails=%d warns=%d install_root=%s\n' "$FAILS" "$WARNS" "$INSTALL_ROOT"
if [[ "$FAILS" -gt 0 ]]; then
  exit 1
fi
