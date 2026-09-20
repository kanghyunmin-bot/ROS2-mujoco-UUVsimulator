#!/usr/bin/env bash
# Portable Ubuntu host entry point; ROS and simulation remain in Ubuntu 22.04.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$ROOT"
source release/versions.env
export UUV_UID="$(id -u)" UUV_GID="$(id -g)"
export UUV_XAUTHORITY="${XAUTHORITY:-/dev/null}"
DEV="$ROOT/docker/ubuntu-dev/dev.sh"
compose() {
  local files=(-f "$ROOT/docker/ubuntu-dev/docker-compose.yml")
  if [[ "${UUV_GPU:-auto}" == 1 ]] || { [[ "${UUV_GPU:-auto}" == auto ]] && command -v nvidia-smi >/dev/null && nvidia-smi >/dev/null 2>&1; }; then
    files+=(-f "$ROOT/docker/ubuntu-dev/docker-compose.gpu.yml")
  fi
  docker compose "${files[@]}" "$@"
}
run() { compose run --rm -T uuv-dev bash -lc "$1"; }
checkout() {
  local target="$1" remote="$2" commit="$3"
  if [[ ! -e "$target" ]]; then
    mkdir -p "$target"
    git -C "$target" init
    git -C "$target" remote add origin "$remote"
    git -C "$target" fetch --depth 1 origin "$commit"
    git -C "$target" checkout --detach FETCH_HEAD
  fi
  [[ "$(git -C "$target" rev-parse HEAD)" == "$commit" ]] || { echo "Unexpected revision: $target (existing files preserved)" >&2; exit 1; }
}
case "${1:-help}" in
 install)
  python3 release/preflight.py
  checkout ardupilot_sub_stable https://github.com/ArduPilot/ardupilot.git "$ARDUSUB_COMMIT"
  git -C ardupilot_sub_stable submodule update --init --recursive --depth 1
  "$DEV" build
  run 'bash /workspace/release/install_container.sh'
  "$0" doctor
  echo 'Installed. Start the GUI with ./release.sh web'
  ;;
 web) exec "$DEV" web ;;
 doctor) run 'bash /workspace/release/doctor.sh' ;;
 install-vla)
  mkdir -p external
  checkout external/auv_vla https://github.com/2026-kmu-underwater-robot/auv_vla.git "$U0_COMMIT"
  run 'bash /workspace/release/install_vla.sh'
  ;;
 download-model)
  python3 release/download_model.py
  ;;
 install-model)
  [[ $# == 2 ]] || { echo 'Usage: ./release.sh install-model /path/to/checkpoint'; exit 2; }
  python3 release/install_model.py "$2"
  echo 'Model installed. Restart the VLA manager / GUI to refresh model discovery.'
  ;;
 export)
  [[ $# == 3 ]] || { echo 'Usage: ./release.sh export outputs/vla-demonstrations/SESSION/staging outputs/datasets/NAME'; exit 2; }
  # Pass arguments separately: no caller-controlled shell interpolation.
  compose run --rm -T uuv-dev bash -lc 'cd /workspace; export PYTHONPATH=/workspace/rospkg/src/auv_vla_data_collector; exec .venv/bin/python -m kmu26_auv_vla_data_collector.export_lerobot "$@"'  _ "$2" "$3"
  ;;
 shell) exec "$DEV" shell ;;
 help|--help|-h)
  echo 'UUV release: ./release.sh install | web | doctor | install-vla | download-model | install-model PATH | export INPUT OUTPUT | shell'
  echo 'Ubuntu 22.04/24.04 x86_64; Docker Compose required. UUV_GPU=0 selects software rendering.'
  ;;
 *) echo "Unknown command: $1" >&2; exit 2 ;;
esac
