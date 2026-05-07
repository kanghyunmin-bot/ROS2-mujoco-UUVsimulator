#!/usr/bin/env bash
set -euo pipefail

SETUP_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SETUP_DIR}/.." && pwd)"
ARDUPILOT_DIR="${ARDUPILOT_DIR:-${WORKSPACE_DIR}/ardupilot}"
ARDUPILOT_REMOTE="${ARDUPILOT_REMOTE:-https://github.com/ArduPilot/ardupilot.git}"
ARDUPILOT_BRANCH="${ARDUPILOT_BRANCH:-}"
SETUP_PREREQS=1

usage() {
  cat <<'USAGE'
Usage: ./setup/02_setup_ardupilot.sh [options]

Run this after 01_install_system_deps.sh.

What it does:
  - clones ArduPilot if missing
  - reuses existing checkout if already present
  - initializes and updates submodules recursively
  - runs the official Ubuntu prerequisite installer on Linux by default
  - verifies Tools/autotest/sim_vehicle.py exists

Options:
  --ardupilot-dir PATH   ArduPilot checkout path (default: <workspace>/ardupilot)
  --remote URL           Git remote URL (default: official upstream)
  --branch NAME          Clone a specific branch when cloning a new checkout
  --skip-prereqs         Skip Tools/environment_install/install-prereqs-ubuntu.sh
  -h, --help             Show this help

Next step after this:
  ./setup/03_setup_uuv_mujoco.sh
USAGE
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --ardupilot-dir)
      [[ $# -ge 2 ]] || { echo "[error] --ardupilot-dir requires a value"; exit 2; }
      ARDUPILOT_DIR="$2"
      shift 2
      ;;
    --remote)
      [[ $# -ge 2 ]] || { echo "[error] --remote requires a value"; exit 2; }
      ARDUPILOT_REMOTE="$2"
      shift 2
      ;;
    --branch)
      [[ $# -ge 2 ]] || { echo "[error] --branch requires a value"; exit 2; }
      ARDUPILOT_BRANCH="$2"
      shift 2
      ;;
    --skip-prereqs)
      SETUP_PREREQS=0
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

log() {
  echo "[ardupilot] $1"
}

run() {
  echo "+ $*"
  "$@"
}

require_cmd() {
  command -v "$1" >/dev/null 2>&1 || { echo "[error] command not found: $1" >&2; exit 1; }
}

require_cmd git

log "workspace: ${WORKSPACE_DIR}"
log "ardupilot dir: ${ARDUPILOT_DIR}"

if [[ -d "${ARDUPILOT_DIR}/.git" ]]; then
  log "existing ArduPilot checkout found"
elif [[ -e "${ARDUPILOT_DIR}" ]]; then
  echo "[error] path exists but is not a git checkout: ${ARDUPILOT_DIR}" >&2
  exit 1
else
  CLONE_ARGS=()
  if [[ -n "$ARDUPILOT_BRANCH" ]]; then
    CLONE_ARGS+=(--branch "$ARDUPILOT_BRANCH")
  fi
  if ((${#CLONE_ARGS[@]} > 0)); then
    run git clone "${CLONE_ARGS[@]}" "$ARDUPILOT_REMOTE" "$ARDUPILOT_DIR"
  else
    run git clone "$ARDUPILOT_REMOTE" "$ARDUPILOT_DIR"
  fi
fi

run git -C "$ARDUPILOT_DIR" submodule sync --recursive
run git -C "$ARDUPILOT_DIR" submodule update --init --recursive

if [[ ! -f "${ARDUPILOT_DIR}/Tools/autotest/sim_vehicle.py" ]]; then
  echo "[error] sim_vehicle.py not found after submodule update: ${ARDUPILOT_DIR}/Tools/autotest/sim_vehicle.py" >&2
  exit 1
fi

if [[ "$SETUP_PREREQS" -eq 1 && "$(uname -s)" == "Linux" ]]; then
  PREREQ_SCRIPT="${ARDUPILOT_DIR}/Tools/environment_install/install-prereqs-ubuntu.sh"
  if [[ -x "$PREREQ_SCRIPT" ]]; then
    log "running ArduPilot Ubuntu prerequisites"
    run "$PREREQ_SCRIPT" -y
  else
    log "warning: ArduPilot prerequisite script missing: ${PREREQ_SCRIPT}"
  fi
fi

cat <<EOF

[ardupilot] done
[ardupilot] next:
  cd "${WORKSPACE_DIR}"
  ./setup/03_setup_uuv_mujoco.sh
EOF
