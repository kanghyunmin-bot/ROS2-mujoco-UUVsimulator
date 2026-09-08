#!/usr/bin/env bash
set -euo pipefail

SETUP_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SETUP_DIR}/.." && pwd)"
HELPERS_SH="${SETUP_DIR}/_setup_path_helpers.sh"
if [[ -f "$HELPERS_SH" ]]; then
  # shellcheck source=/dev/null
  source "$HELPERS_SH"
fi
if ! declare -F resolve_ros_workspace_dir >/dev/null 2>&1; then
  resolve_ros_workspace_dir() {
    local workspace_dir="$1"
    if [[ -n "${ROS_WORKSPACE_DIR:-}" ]]; then
      printf '%s\n' "${ROS_WORKSPACE_DIR}"
      return 0
    fi
    if [[ -d "${workspace_dir}/rospkg" ]]; then
      printf '%s\n' "${workspace_dir}/rospkg"
      return 0
    fi
    printf '%s\n' "${workspace_dir}"
  }
fi
if ! declare -F resolve_kmu26_auv_dir >/dev/null 2>&1; then
  resolve_kmu26_auv_dir() {
    local workspace_dir="$1"
    local ros_workspace_dir="$2"
    if [[ -n "${KMU26_AUV_DIR:-}" ]]; then
      printf '%s\n' "${KMU26_AUV_DIR}"
      return 0
    fi
    if [[ -d "${ros_workspace_dir}/kmu26_auv" ]]; then
      printf '%s\n' "${ros_workspace_dir}/kmu26_auv"
      return 0
    fi
    if [[ -d "${ros_workspace_dir}/src/kmu26_auv" ]]; then
      printf '%s\n' "${ros_workspace_dir}/src/kmu26_auv"
      return 0
    fi
    if [[ -d "${workspace_dir}/kmu26_auv" ]]; then
      printf '%s\n' "${workspace_dir}/kmu26_auv"
      return 0
    fi
    printf '%s\n' "${ros_workspace_dir}/src/kmu26_auv"
  }
fi
if ! declare -F preferred_setup_script_names >/dev/null 2>&1; then
  preferred_setup_script_names() {
    local user_shell
    user_shell="$(basename "${SHELL:-}")"
    if [[ "$user_shell" == "zsh" ]]; then
      printf '%s\n' \
        setup.zsh local_setup.zsh \
        setup.bash local_setup.bash \
        setup.sh local_setup.sh
      return 0
    fi

    printf '%s\n' \
      setup.bash local_setup.bash \
      setup.zsh local_setup.zsh \
      setup.sh local_setup.sh
  }
fi
if ! declare -F resolve_setup_script_in_dir >/dev/null 2>&1; then
  resolve_setup_script_in_dir() {
    local dir="$1"
    local script_name
    [[ -d "$dir" ]] || return 1

    while IFS= read -r script_name; do
      if [[ -f "${dir}/${script_name}" ]]; then
        printf '%s\n' "${dir}/${script_name}"
        return 0
      fi
    done < <(preferred_setup_script_names)

    return 1
  }
fi
if ! declare -F resolve_ros_install_setup >/dev/null 2>&1; then
  resolve_ros_install_setup() {
    local workspace_dir="$1"
    local ros_workspace_dir="$2"
    local setup_script
    if [[ -n "${ROS_INSTALL_SETUP:-}" && -f "${ROS_INSTALL_SETUP}" ]]; then
      printf '%s\n' "${ROS_INSTALL_SETUP}"
      return 0
    fi
    if setup_script="$(resolve_setup_script_in_dir "${ros_workspace_dir}/install")"; then
      printf '%s\n' "${setup_script}"
      return 0
    fi
    if setup_script="$(resolve_setup_script_in_dir "${workspace_dir}/install")"; then
      printf '%s\n' "${setup_script}"
      return 0
    fi
    printf '%s\n' "${ros_workspace_dir}/install/setup.bash"
  }
fi
if ! declare -F resolve_ros_env_setup >/dev/null 2>&1; then
  resolve_ros_env_setup() {
    local ros_distro="${1:-${ROS_DISTRO:-humble}}"
    local setup_script

    if [[ -n "${ROS_ENV_SETUP:-}" && -f "${ROS_ENV_SETUP}" ]]; then
      printf '%s\n' "${ROS_ENV_SETUP}"
      return 0
    fi

    for base_dir in \
      "/opt/ros/${ros_distro}" \
      "/usr/local/ros/${ros_distro}" \
      "/opt/homebrew/opt/ros/${ros_distro}" \
      "/usr/local/opt/ros/${ros_distro}"
    do
      if setup_script="$(resolve_setup_script_in_dir "${base_dir}")"; then
        printf '%s\n' "${setup_script}"
        return 0
      fi
    done

    if [[ -n "${CONDA_PREFIX:-}" ]]; then
      if setup_script="$(resolve_setup_script_in_dir "${CONDA_PREFIX}")"; then
        printf '%s\n' "${setup_script}"
        return 0
      fi
    fi

    return 1
  }
fi
if ! declare -F resolve_qgc_app >/dev/null 2>&1; then
  resolve_qgc_app() {
    local os_name
    local workspace_dir

    if [[ -n "${QGC_APP:-}" ]]; then
      printf '%s\n' "${QGC_APP}"
      return 0
    fi

    workspace_dir="${WORKSPACE_DIR}"

    for candidate in \
      "${workspace_dir}/QGroundControl.AppImage" \
      "${workspace_dir}/QGroundControl-x86_64.AppImage" \
      "$HOME/Applications/QGroundControl.AppImage" \
      "${workspace_dir}/QGroundControl.app" \
      "/Applications/QGroundControl.app" \
      "$HOME/Applications/QGroundControl.app"
    do
      if [[ -e "$candidate" ]]; then
        printf '%s\n' "$candidate"
        return 0
      fi
    done

    if command -v QGroundControl >/dev/null 2>&1; then
      command -v QGroundControl
      return 0
    fi
    if command -v qgroundcontrol >/dev/null 2>&1; then
      command -v qgroundcontrol
      return 0
    fi

    os_name="$(uname -s)"
    if [[ "$os_name" == "Darwin" ]]; then
      printf '%s\n' "/Applications/QGroundControl.app"
    else
      printf '%s\n' "${workspace_dir}/QGroundControl.AppImage"
    fi
  }
fi
resolve_default_mj311_root() {
  if [[ -x "$HOME/.venvs/uuv_mujoco/bin/python" ]]; then
    printf '%s\n' "$HOME/.venvs/uuv_mujoco"
    return 0
  fi
  if [[ -x "$HOME/.venvs/mujoco311/bin/python" ]]; then
    printf '%s\n' "$HOME/.venvs/mujoco311"
    return 0
  fi
  printf '%s\n' "$HOME/.venvs/uuv_mujoco"
}

UUV_MUJOCO_DIR="${UUV_MUJOCO_DIR:-${WORKSPACE_DIR}/uuv_mujoco}"
UUV_MUJOCO_ZIP="${UUV_MUJOCO_ZIP:-${WORKSPACE_DIR}/uuv_mujoco.zip}"
QGC_APP="${QGC_APP:-$(resolve_qgc_app)}"
ARDUPILOT_DIR="${ARDUPILOT_DIR:-${WORKSPACE_DIR}/ardupilot}"
ROS_DISTRO="${ROS_DISTRO:-humble}"
ROS_WORKSPACE_DIR="${ROS_WORKSPACE_DIR:-$(resolve_ros_workspace_dir "${WORKSPACE_DIR}")}"
if [[ -z "${KMU26_AUV_DIR:-}" || ! -d "${KMU26_AUV_DIR}" ]]; then
  KMU26_AUV_DIR="$(resolve_kmu26_auv_dir "${WORKSPACE_DIR}" "${ROS_WORKSPACE_DIR}")"
fi
ROS_INSTALL_SETUP="${ROS_INSTALL_SETUP:-$(resolve_ros_install_setup "${WORKSPACE_DIR}" "${ROS_WORKSPACE_DIR}")}"
ROS_ENV_SETUP="${ROS_ENV_SETUP:-$(resolve_ros_env_setup "${ROS_DISTRO}" 2>/dev/null || true)}"
MJ311_ROOT="${MJ311_ROOT:-$(resolve_default_mj311_root)}"
PYTHON_VERSION="${PYTHON_VERSION:-3.10}"
PYTHON_VERSION_EXPLICIT=0
FORCE_REEXTRACT=0
SKIP_PIP=0
RECREATE_VENV=0

usage() {
  cat <<'USAGE'
Usage: ./setup/03_setup_uuv_mujoco.sh [options]

Run this after 02_setup_ardupilot.sh.

What it does:
  - if uuv_mujoco.zip exists in the workspace, extracts it automatically
  - if uuv_mujoco/ already exists, reuses it
  - creates the MuJoCo python virtualenv
  - installs MuJoCo/MAVLink/DroneCAN/replay/analysis python dependencies
  - checks the QGroundControl app path if present
  - writes .uuv_mujoco_env.sh in the workspace root

Options:
  --uuv-dir PATH        Destination uuv_mujoco directory (default: <workspace>/uuv_mujoco)
  --zip PATH            Source zip file path (default: <workspace>/uuv_mujoco.zip)
  --venv-root PATH      Virtualenv root (default: ~/.venvs/uuv_mujoco)
  --python-version V    Preferred Python version for venv creation (default: 3.10 on Ubuntu 22.04)
  --force-reextract     Remove existing uuv_mujoco directory and extract again from zip
  --skip-pip            Skip pip install into the venv
  --recreate-venv       Remove and recreate the existing MuJoCo venv
  --qgc-app PATH        QGroundControl path to check
  -h, --help            Show this help

Next step after this:
  ./setup/04_verify_uuv_stack.sh
USAGE
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --uuv-dir)
      [[ $# -ge 2 ]] || { echo "[error] --uuv-dir requires a value"; exit 2; }
      UUV_MUJOCO_DIR="$2"
      shift 2
      ;;
    --zip)
      [[ $# -ge 2 ]] || { echo "[error] --zip requires a value"; exit 2; }
      UUV_MUJOCO_ZIP="$2"
      shift 2
      ;;
    --venv-root)
      [[ $# -ge 2 ]] || { echo "[error] --venv-root requires a value"; exit 2; }
      MJ311_ROOT="$2"
      shift 2
      ;;
    --python-version)
      [[ $# -ge 2 ]] || { echo "[error] --python-version requires a value"; exit 2; }
      PYTHON_VERSION="$2"
      PYTHON_VERSION_EXPLICIT=1
      shift 2
      ;;
    --force-reextract)
      FORCE_REEXTRACT=1
      shift
      ;;
    --skip-pip)
      SKIP_PIP=1
      shift
      ;;
    --recreate-venv)
      RECREATE_VENV=1
      shift
      ;;
    --qgc-app)
      [[ $# -ge 2 ]] || { echo "[error] --qgc-app requires a value"; exit 2; }
      QGC_APP="$2"
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
  echo "[uuv] $1"
}

run() {
  echo "+ $*"
  "$@"
}

run_clean_env() {
  echo "+ $*"
  env -u PYTHONPATH -u PYTHONHOME "$@"
}

warn() {
  echo "[uuv] warning: $1" >&2
}

die() {
  echo "[error] $1" >&2
  exit 1
}

require_cmd() {
  command -v "$1" >/dev/null 2>&1 || die "command not found: $1"
}

python_version_of() {
  local python_bin="$1"
  "$python_bin" - <<'PY'
import sys
print(f"{sys.version_info.major}.{sys.version_info.minor}")
PY
}

resolve_ros_python_for_venv() {
  local ros_python ros_version
  [[ "$PYTHON_VERSION_EXPLICIT" -eq 0 ]] || return 1

  if command -v ros2 >/dev/null 2>&1 && command -v python3 >/dev/null 2>&1; then
    if python3 - <<'PY' >/dev/null 2>&1
import rclpy
PY
    then
      ros_python="$(command -v python3)"
      ros_version="$("$ros_python" - <<'PY'
import sys
print(f"{sys.version_info.major}.{sys.version_info.minor}")
PY
)"
      if [[ "$ros_version" != "$PYTHON_VERSION" ]]; then
        warn "Active ROS2 Python detected in PATH; using python${ros_version} for launcher compatibility instead of python${PYTHON_VERSION}."
      fi
      printf '%s\n' "$ros_python"
      return 0
    fi
  fi

  if [[ -n "${ROS_ENV_SETUP:-}" && -f "${ROS_ENV_SETUP}" ]] || [[ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
    if [[ -x /usr/bin/python3 ]]; then
      ros_python="/usr/bin/python3"
    elif command -v python3 >/dev/null 2>&1; then
      ros_python="$(command -v python3)"
    else
      return 1
    fi
    ros_version="$("$ros_python" - <<'PY'
import sys
print(f"{sys.version_info.major}.{sys.version_info.minor}")
PY
)"
    if [[ "$ros_version" != "$PYTHON_VERSION" ]]; then
      if [[ -n "${ROS_ENV_SETUP:-}" && -f "${ROS_ENV_SETUP}" ]]; then
        warn "ROS2 setup detected at ${ROS_ENV_SETUP}; using system python${ros_version} for native Ubuntu compatibility instead of python${PYTHON_VERSION}."
      else
        warn "ROS2 detected at /opt/ros/${ROS_DISTRO}; using system python${ros_version} for native Ubuntu compatibility instead of python${PYTHON_VERSION}."
      fi
    fi
    printf '%s\n' "$ros_python"
    return 0
  fi

  if [[ -n "${CONDA_PREFIX:-}" && -x "${CONDA_PREFIX}/bin/python" ]]; then
    if env -u PYTHONPATH -u PYTHONHOME "${CONDA_PREFIX}/bin/python" - <<'PY' >/dev/null 2>&1
import rclpy
PY
    then
      ros_python="${CONDA_PREFIX}/bin/python"
      ros_version="$(env -u PYTHONPATH -u PYTHONHOME "$ros_python" - <<'PY'
import sys
print(f"{sys.version_info.major}.{sys.version_info.minor}")
PY
)"
      if [[ "$ros_version" != "$PYTHON_VERSION" ]]; then
        warn "Falling back to conda ROS2 env at ${CONDA_PREFIX}; using python${ros_version} instead of python${PYTHON_VERSION}."
      fi
      printf '%s\n' "$ros_python"
      return 0
    fi
  fi

  return 1
}

resolve_python_for_venv() {
  local ros_python
  if ros_python="$(resolve_ros_python_for_venv)"; then
    printf '%s\n' "$ros_python"
    return 0
  fi
  if command -v "python${PYTHON_VERSION}" >/dev/null 2>&1; then
    command -v "python${PYTHON_VERSION}"
    return 0
  fi
  if command -v python3 >/dev/null 2>&1; then
    warn "python${PYTHON_VERSION} not found. Falling back to python3."
    command -v python3
    return 0
  fi
  die "no suitable python interpreter found"
}

find_extracted_uuv_root() {
  local base="$1"
  local script_path
  script_path="$(find "$base" -type f -path '*/current/start_sitl_mujoco_mj311.sh' -print -quit)"
  if [[ -z "$script_path" ]]; then
    # Older archives may not contain the current symlink.  Find the runtime by
    # its launcher contract, then recreate current after extraction.
    script_path="$(find "$base" -type f -name 'start_sitl_mujoco_mj311.sh' -print -quit)"
  fi
  [[ -n "$script_path" ]] || return 1
  dirname "$(dirname "$script_path")"
}

resolve_uuv_runtime_dir() {
  if [[ -n "${UUV_MUJOCO_RUNTIME_DIR:-}" && -d "${UUV_MUJOCO_RUNTIME_DIR}" ]]; then
    printf '%s\n' "${UUV_MUJOCO_RUNTIME_DIR}"
    return 0
  fi
  if [[ -d "${UUV_MUJOCO_DIR}/current" ]]; then
    printf '%s\n' "${UUV_MUJOCO_DIR}/current"
    return 0
  fi
  return 1
}

extract_zip_if_needed() {
  local tmpdir extracted_root
  if [[ -d "$UUV_MUJOCO_DIR" && "$FORCE_REEXTRACT" -eq 0 ]]; then
    log "existing uuv_mujoco directory found, reusing it"
    return 0
  fi

  [[ -f "$UUV_MUJOCO_ZIP" ]] || die "uuv_mujoco.zip not found and uuv_mujoco/ directory is missing: ${UUV_MUJOCO_ZIP}"

  if [[ -d "$UUV_MUJOCO_DIR" && "$FORCE_REEXTRACT" -eq 1 ]]; then
    run rm -rf "$UUV_MUJOCO_DIR"
  fi

  require_cmd unzip
  tmpdir="$(mktemp -d)"
  trap 'rm -rf "$tmpdir"' EXIT
  run unzip -q "$UUV_MUJOCO_ZIP" -d "$tmpdir"
  extracted_root="$(find_extracted_uuv_root "$tmpdir")" || die "could not find uuv_mujoco root inside zip: ${UUV_MUJOCO_ZIP}"
  run mv "$extracted_root" "$UUV_MUJOCO_DIR"
  rm -rf "$tmpdir"
  trap - EXIT
}

ensure_scripts_executable() {
  local runtime_dir
  local script
  runtime_dir="$(resolve_uuv_runtime_dir)" || die "could not resolve active uuv_mujoco runtime under ${UUV_MUJOCO_DIR}"
  for script in \
    "${runtime_dir}/start_sitl_mujoco_mj311.sh" \
    "${runtime_dir}/start_ardusub_sitl_mj311.sh" \
    "${runtime_dir}/launch_uuv_sim.sh" \
    "${runtime_dir}/reset_uuv_sim.sh"
  do
    [[ -f "$script" ]] || die "missing required script: $script"
    run chmod +x "$script"
  done
}

install_python_deps() {
  local venv_python
  venv_python="${MJ311_ROOT}/bin/python"
  [[ -x "$venv_python" ]] || die "virtualenv python not found: $venv_python"
  run_clean_env "$venv_python" -m pip install -U pip "setuptools<81" wheel
  run_clean_env "$venv_python" -m pip install \
    numpy matplotlib pyyaml rosbags python-pptx opencv-python-headless \
    mujoco pymavlink MAVProxy pexpect pillow future dronecan gnureadline \
    "empy==3.3.4"
  env -u PYTHONPATH -u PYTHONHOME "$venv_python" - <<'PY'
import mujoco
import numpy
import matplotlib
import cv2
import yaml
import rosbags
import pptx
import pymavlink
import MAVProxy
import pexpect
import PIL
import future
import dronecan
import em
print("python deps ok")
print("MuJoCo Version:", mujoco.__version__)
PY
}

write_env_file() {
  local env_file="${WORKSPACE_DIR}/.uuv_mujoco_env.sh"
  cat >"$env_file" <<'EOF'
#!/usr/bin/env bash

if [[ -n "${BASH_SOURCE[0]:-}" ]]; then
  _UUV_ENV_SOURCE="${BASH_SOURCE[0]}"
elif [[ -n "${ZSH_VERSION:-}" ]]; then
  _UUV_ENV_SOURCE="${(%):-%N}"
else
  _UUV_ENV_SOURCE="$0"
fi

export WORKSPACE_DIR="$(cd "$(dirname "${_UUV_ENV_SOURCE}")" && pwd)"
export ROS_WORKSPACE_DIR="${ROS_WORKSPACE_DIR:-${WORKSPACE_DIR}/rospkg}"
export UUV_MUJOCO_DIR="${UUV_MUJOCO_DIR:-${WORKSPACE_DIR}/uuv_mujoco}"
export ARDUPILOT_STABLE_DIR="${ARDUPILOT_STABLE_DIR:-${WORKSPACE_DIR}/ardupilot_sub_stable}"
if [[ -d "${ARDUPILOT_STABLE_DIR}/ArduSub" ]]; then
  export ARDUPILOT_DIR="${ARDUPILOT_DIR:-${ARDUPILOT_STABLE_DIR}}"
else
  export ARDUPILOT_DIR="${ARDUPILOT_DIR:-${WORKSPACE_DIR}/ardupilot}"
fi
if [[ -z "${KMU26_AUV_DIR:-}" || ! -d "${KMU26_AUV_DIR}" ]]; then
  if [[ -d "${ROS_WORKSPACE_DIR}/src/kmu26_auv" ]]; then
    export KMU26_AUV_DIR="${ROS_WORKSPACE_DIR}/src/kmu26_auv"
  else
    export KMU26_AUV_DIR="${ROS_WORKSPACE_DIR}/kmu26_auv"
  fi
fi
export ROS_DISTRO="${ROS_DISTRO:-humble}"
export UUV_SITL_BACKEND_DEFAULT="${UUV_SITL_BACKEND_DEFAULT:-native}"
export UUV_SITL_BACKEND="${UUV_SITL_BACKEND:-${UUV_SITL_BACKEND_DEFAULT}}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"
export ROS_DISABLE_DAEMON="${ROS_DISABLE_DAEMON:-1}"

if [[ -z "${UUV_MUJOCO_RUNTIME_DIR:-}" ]]; then
  export UUV_MUJOCO_RUNTIME_DIR="${UUV_MUJOCO_DIR}/current"
fi

if [[ -z "${ROS_INSTALL_SETUP:-}" ]]; then
  for _candidate in \
    "${ROS_WORKSPACE_DIR}/install/setup.bash" \
    "${ROS_WORKSPACE_DIR}/install/setup.zsh" \
    "${WORKSPACE_DIR}/install/setup.bash" \
    "${WORKSPACE_DIR}/install/setup.zsh"
  do
    if [[ -f "${_candidate}" ]]; then
      export ROS_INSTALL_SETUP="${_candidate}"
      break
    fi
  done
fi

if [[ -z "${ROS_ENV_SETUP:-}" ]]; then
  for _candidate in \
    "/opt/ros/${ROS_DISTRO}/setup.bash" \
    "/opt/ros/${ROS_DISTRO}/setup.zsh" \
    "$HOME/miniconda3/envs/ros2_h311/setup.bash" \
    "$HOME/miniconda3/envs/ros2_h311/setup.zsh" \
    "$HOME/miniconda3/envs/ros2/setup.bash" \
    "$HOME/miniconda3/envs/ros2/setup.zsh"
  do
    if [[ -f "${_candidate}" ]]; then
      export ROS_ENV_SETUP="${_candidate}"
      break
    fi
  done
fi

if [[ -z "${MJ311_ROOT:-}" ]]; then
  if [[ -x "$HOME/.venvs/uuv_mujoco/bin/python" ]]; then
    export MJ311_ROOT="$HOME/.venvs/uuv_mujoco"
  elif [[ -x "$HOME/.venvs/mujoco311/bin/python" ]]; then
    export MJ311_ROOT="$HOME/.venvs/mujoco311"
  else
    export MJ311_ROOT="$HOME/.venvs/uuv_mujoco"
  fi
fi

export MJ311_PYTHON="${MJ311_PYTHON:-${MJ311_ROOT}/bin/python}"
if [[ -z "${MJ311_MJPYTHON:-}" ]]; then
  if [[ "$(uname -s)" == "Darwin" ]]; then
    export MJ311_MJPYTHON="${MJ311_ROOT}/bin/mjpython"
  else
    export MJ311_MJPYTHON="${MJ311_PYTHON}"
  fi
fi

if [[ -z "${QGC_APP:-}" ]]; then
  for _candidate in \
    "${WORKSPACE_DIR}/dist/ubuntu22.04/QGroundControl.AppImage" \
    "${WORKSPACE_DIR}/dist/ubuntu22.04/QGroundControl-x86_64.AppImage" \
    "${WORKSPACE_DIR}/QGroundControl.AppImage" \
    "${WORKSPACE_DIR}/QGroundControl-x86_64.AppImage" \
    "$HOME/Downloads/QGroundControl.AppImage" \
    "$HOME/Downloads/QGroundControl-x86_64.AppImage" \
    "$HOME/Applications/QGroundControl.AppImage" \
    "$HOME/bin/QGroundControl.AppImage" \
    "/opt/QGroundControl/QGroundControl.AppImage" \
    "/usr/local/bin/QGroundControl.AppImage" \
    "${WORKSPACE_DIR}/QGroundControl.app" \
    "/Applications/QGroundControl.app" \
    "$HOME/Applications/QGroundControl.app"
  do
    if [[ -e "${_candidate}" ]]; then
      export QGC_APP="${_candidate}"
      break
    fi
  done
fi
EOF
  run chmod +x "$env_file"
  log "wrote ${env_file}"
}

log "workspace: ${WORKSPACE_DIR}"
log "ros workspace: ${ROS_WORKSPACE_DIR}"
log "kmu26_auv dir: ${KMU26_AUV_DIR}"
extract_zip_if_needed

[[ -d "$UUV_MUJOCO_DIR" ]] || die "uuv_mujoco directory not found after setup: ${UUV_MUJOCO_DIR}"
if [[ ! -e "${UUV_MUJOCO_DIR}/current" ]]; then
  [[ -d "${UUV_MUJOCO_DIR}/v2.2" ]] || die "active runtime alias missing and no compatibility backing directory found: ${UUV_MUJOCO_DIR}/current"
  warn "active runtime alias missing; recreating current from the extracted runtime backing directory"
  run ln -s v2.2 "${UUV_MUJOCO_DIR}/current"
fi
UUV_MUJOCO_RUNTIME_DIR="$(resolve_uuv_runtime_dir)" || die "could not resolve active uuv_mujoco runtime"
export UUV_MUJOCO_RUNTIME_DIR
ensure_scripts_executable

PYTHON_BIN="$(resolve_python_for_venv)"
TARGET_PYTHON_VERSION="$(python_version_of "$PYTHON_BIN")"

if [[ -d "$MJ311_ROOT" && "$RECREATE_VENV" -eq 1 ]]; then
  log "removing existing virtualenv (--recreate-venv): ${MJ311_ROOT}"
  run rm -rf "$MJ311_ROOT"
fi

if [[ -d "$MJ311_ROOT" ]]; then
  if [[ ! -x "${MJ311_ROOT}/bin/python" ]]; then
    die "existing virtualenv is missing python: ${MJ311_ROOT}/bin/python (rerun with --recreate-venv)"
  fi
  EXISTING_PYTHON_VERSION="$(python_version_of "${MJ311_ROOT}/bin/python")"
  if [[ "$EXISTING_PYTHON_VERSION" != "$TARGET_PYTHON_VERSION" ]]; then
    die "existing virtualenv at ${MJ311_ROOT} uses python${EXISTING_PYTHON_VERSION}, but the selected interpreter is python${TARGET_PYTHON_VERSION}. Remove it or rerun with --recreate-venv."
  fi
fi

if [[ ! -d "$MJ311_ROOT" ]]; then
  run_clean_env "$PYTHON_BIN" -m venv "$MJ311_ROOT"
else
  log "existing virtualenv found: ${MJ311_ROOT}"
fi

if [[ "$SKIP_PIP" -eq 0 ]]; then
  install_python_deps
else
  log "skipped pip install (--skip-pip)"
fi

if [[ -f "$QGC_APP" ]]; then
  run chmod +x "$QGC_APP"
  log "QGroundControl found: ${QGC_APP}"
elif [[ -d "$QGC_APP" ]]; then
  log "QGroundControl found: ${QGC_APP}"
else
  warn "QGroundControl not found at ${QGC_APP}"
fi

write_env_file

cat <<EOF

[uuv] done
[uuv] next:
  cd "${WORKSPACE_DIR}"
  ./setup/04_verify_uuv_stack.sh
EOF
