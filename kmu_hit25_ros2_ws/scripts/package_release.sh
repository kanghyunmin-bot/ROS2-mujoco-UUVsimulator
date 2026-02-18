#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
WS_DIR=$(cd "${SCRIPT_DIR}/.." && pwd)
OUT_DIR="${WS_DIR}/dist"

mkdir -p "${OUT_DIR}"

STAMP=$(date +%Y%m%d_%H%M%S)
ARCHIVE="${OUT_DIR}/hit25_ros2_ws_${STAMP}.tar.gz"

# Use tar to avoid huge temporary copies
# Exclude build artifacts and large external repos
EXCLUDES=(
  --exclude=".git"
  --exclude="build"
  --exclude="install"
  --exclude="log"
  --exclude=".tmp"
  --exclude=".vscode"
  --exclude="ardupilot"
  --exclude="dist"
)

cd "${WS_DIR}"

# Create archive
# shellcheck disable=SC2086
 tar -czf "${ARCHIVE}" ${EXCLUDES[@]} .

# Create checksum
sha256sum "${ARCHIVE}" > "${ARCHIVE}.sha256"

echo "Created: ${ARCHIVE}"
echo "Checksum: ${ARCHIVE}.sha256"
