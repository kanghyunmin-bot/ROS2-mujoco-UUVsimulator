#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
START_SCRIPT="${SCRIPT_DIR}/start_kmu26_auv_web_gui.sh"
APP_DIR="${HOME}/.local/share/applications"
DESKTOP_DIR="${HOME}/Desktop"
APP_FILE="${APP_DIR}/kmu26-auv-web-gui.desktop"
DESKTOP_FILE="${DESKTOP_DIR}/kmu26-auv-web-gui.desktop"

mkdir -p "${APP_DIR}"
mkdir -p "${DESKTOP_DIR}"

cat > "${APP_FILE}" <<EOF
[Desktop Entry]
Type=Application
Name=KMU26 AUV Web GUI
Comment=Start the AUV localization web control server
Exec=bash -lc '${START_SCRIPT}'
Terminal=true
Categories=Development;Robotics;
StartupNotify=false
EOF

cp "${APP_FILE}" "${DESKTOP_FILE}"
chmod +x "${APP_FILE}" "${DESKTOP_FILE}" "${START_SCRIPT}"

if command -v gio >/dev/null 2>&1; then
  gio set "${DESKTOP_FILE}" metadata::trusted true >/dev/null 2>&1 || true
fi

echo "[kmu26_auv_web_gui] Installed launcher:"
echo "  ${APP_FILE}"
echo "  ${DESKTOP_FILE}"
echo
echo "If Ubuntu asks, right-click the desktop icon and choose 'Allow Launching'."
