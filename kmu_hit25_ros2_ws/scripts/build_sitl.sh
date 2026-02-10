#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
WS_DIR=$(cd "${SCRIPT_DIR}/.." && pwd)
ARDUPILOT_DIR="${WS_DIR}/ardupilot"

if [ ! -d "${ARDUPILOT_DIR}/.git" ]; then
  echo "ArduPilot repo not found at ${ARDUPILOT_DIR}."
  echo "Run ./scripts/setup_ubuntu_humble.sh --with-sitl first."
  exit 1
fi

cd "${ARDUPILOT_DIR}"

echo "Updating submodules..."
git submodule update --init --recursive

if [ ! -x ./waf ]; then
  echo "waf not found, fetching submodule..."
  git submodule update --init modules/waf
fi

./waf configure --board sitl
./waf build

echo "SITL build complete."
