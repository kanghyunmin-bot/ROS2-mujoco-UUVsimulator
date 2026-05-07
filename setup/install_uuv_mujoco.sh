#!/usr/bin/env bash
set -euo pipefail

SETUP_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SETUP_DIR}/.." && pwd)"
STEP1="${SETUP_DIR}/01_install_system_deps.sh"
STEP2="${SETUP_DIR}/02_setup_ardupilot.sh"
STEP3="${SETUP_DIR}/03_setup_uuv_mujoco.sh"
STEP4="${SETUP_DIR}/04_verify_uuv_stack.sh"

WITH_ROS2=1
BUILD_REAL_PKG=0
SKIP_APT=0
SKIP_ARDUPILOT=0
SKIP_PIP=0
RECREATE_VENV=0
NONINTERACTIVE=0
PYTHON_VERSION=""
VENV_ROOT=""
ARDUPILOT_DIR=""
QGC_APP=""

usage() {
  cat <<'USAGE'
Usage: ./setup/install_uuv_mujoco.sh [options]

This is a convenience wrapper around:
  1. ./setup/01_install_system_deps.sh
  2. ./setup/02_setup_ardupilot.sh
  3. ./setup/03_setup_uuv_mujoco.sh
  4. ./setup/04_verify_uuv_stack.sh

Options:
  --with-ros2         Install ROS2-side extras in step 1 (default)
  --without-ros2      Skip ROS2-side extras in step 1
  --build-real-pkg    Pass --build-real-pkg to step 4
  --skip-apt          Skip step 1
  --skip-ardupilot    Skip step 2
  --skip-pip          Pass --skip-pip to step 3
  --recreate-venv     Pass --recreate-venv to step 3
  --noninteractive    Pass --noninteractive to step 1
  --python-version V  Forward python version to steps 1 and 3
  --venv-root PATH    Forward venv path to step 3
  --ardupilot-dir P   Forward ardupilot path to step 2
  --qgc-app PATH      Forward QGroundControl path to step 3
  -h, --help          Show this help
USAGE
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --with-ros2)
      WITH_ROS2=1
      shift
      ;;
    --without-ros2)
      WITH_ROS2=0
      shift
      ;;
    --build-real-pkg)
      BUILD_REAL_PKG=1
      WITH_ROS2=1
      shift
      ;;
    --skip-apt)
      SKIP_APT=1
      shift
      ;;
    --skip-ardupilot)
      SKIP_ARDUPILOT=1
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
    --noninteractive)
      NONINTERACTIVE=1
      shift
      ;;
    --python-version)
      [[ $# -ge 2 ]] || { echo "[error] --python-version requires a value" >&2; exit 2; }
      PYTHON_VERSION="$2"
      shift 2
      ;;
    --venv-root)
      [[ $# -ge 2 ]] || { echo "[error] --venv-root requires a value" >&2; exit 2; }
      VENV_ROOT="$2"
      shift 2
      ;;
    --ardupilot-dir)
      [[ $# -ge 2 ]] || { echo "[error] --ardupilot-dir requires a value" >&2; exit 2; }
      ARDUPILOT_DIR="$2"
      shift 2
      ;;
    --qgc-app)
      [[ $# -ge 2 ]] || { echo "[error] --qgc-app requires a value" >&2; exit 2; }
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

for script in "$STEP1" "$STEP2" "$STEP3" "$STEP4"; do
  [[ -f "$script" ]] || { echo "[error] required step script missing: $script" >&2; exit 1; }
done

run_step() {
  echo
  echo "==> $*"
  "$@"
}

STEP1_ARGS=()
STEP2_ARGS=()
STEP3_ARGS=()
STEP4_ARGS=()

[[ "$WITH_ROS2" -eq 1 ]] && STEP1_ARGS+=(--with-ros2)
[[ "$NONINTERACTIVE" -eq 1 ]] && STEP1_ARGS+=(--noninteractive)
[[ -n "$PYTHON_VERSION" ]] && STEP1_ARGS+=(--python-version "$PYTHON_VERSION")

[[ -n "$ARDUPILOT_DIR" ]] && STEP2_ARGS+=(--ardupilot-dir "$ARDUPILOT_DIR")

[[ "$SKIP_PIP" -eq 1 ]] && STEP3_ARGS+=(--skip-pip)
[[ "$RECREATE_VENV" -eq 1 ]] && STEP3_ARGS+=(--recreate-venv)
[[ -n "$PYTHON_VERSION" ]] && STEP3_ARGS+=(--python-version "$PYTHON_VERSION")
[[ -n "$VENV_ROOT" ]] && STEP3_ARGS+=(--venv-root "$VENV_ROOT")
[[ -n "$QGC_APP" ]] && STEP3_ARGS+=(--qgc-app "$QGC_APP")

[[ "$BUILD_REAL_PKG" -eq 1 ]] && STEP4_ARGS+=(--build-real-pkg)

if [[ "$SKIP_APT" -eq 0 ]]; then
  if ((${#STEP1_ARGS[@]} > 0)); then
    run_step "$STEP1" "${STEP1_ARGS[@]}"
  else
    run_step "$STEP1"
  fi
else
  echo "==> skip step 1: $STEP1"
fi

if [[ "$SKIP_ARDUPILOT" -eq 0 ]]; then
  if ((${#STEP2_ARGS[@]} > 0)); then
    run_step "$STEP2" "${STEP2_ARGS[@]}"
  else
    run_step "$STEP2"
  fi
else
  echo "==> skip step 2: $STEP2"
fi

if ((${#STEP3_ARGS[@]} > 0)); then
  run_step "$STEP3" "${STEP3_ARGS[@]}"
else
  run_step "$STEP3"
fi

if ((${#STEP4_ARGS[@]} > 0)); then
  run_step "$STEP4" "${STEP4_ARGS[@]}"
else
  run_step "$STEP4"
fi
