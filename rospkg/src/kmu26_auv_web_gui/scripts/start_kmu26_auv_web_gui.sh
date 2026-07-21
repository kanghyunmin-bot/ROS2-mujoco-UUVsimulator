#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PACKAGE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

HOST="${KMU26_WEB_GUI_HOST:-0.0.0.0}"
PORT="${KMU26_WEB_GUI_PORT:-8080}"
PORT_EXPLICIT=0
if [[ -n "${KMU26_WEB_GUI_PORT+x}" ]]; then
  PORT_EXPLICIT=1
fi
ROBOT_PACKAGE="${KMU26_ROBOT_PACKAGE:-hit25_auv_ros2}"
ROBOT_LAUNCH="${KMU26_ROBOT_LAUNCH:-localization_test.launch.py}"
PINGER_PACKAGE="${KMU26_PINGER_PACKAGE:-kmu26_pinger_homing}"
PINGER_LAUNCH="${KMU26_PINGER_LAUNCH:-pinger_homing_real.launch.py}"

source_if_exists() {
  local setup_file="$1"
  if [[ -f "${setup_file}" ]]; then
    # shellcheck source=/dev/null
    set +u
    source "${setup_file}"
    set -u
  fi
}

source_workspace_setup() {
  local candidate
  local candidates=(
    "${KMU26_WORKSPACE_DIR:-}/install/setup.bash"
    "${SCRIPT_DIR}/../../../install/setup.bash"
    "${SCRIPT_DIR}/../../../../setup.bash"
  )

  for candidate in "${candidates[@]}"; do
    if [[ -n "${candidate}" && -f "${candidate}" ]]; then
      # shellcheck source=/dev/null
      set +u
      source "${candidate}"
      set -u
      return 0
    fi
  done

  return 1
}

if [[ -z "${ROS_DISTRO:-}" ]]; then
  if [[ -f "/opt/ros/humble/setup.bash" ]]; then
    source_if_exists "/opt/ros/humble/setup.bash"
  elif [[ -f "/opt/ros/jazzy/setup.bash" ]]; then
    source_if_exists "/opt/ros/jazzy/setup.bash"
  fi
fi

source_workspace_setup || true

if ! command -v ros2 >/dev/null 2>&1; then
  echo "[kmu26_auv_web_gui] ros2 command not found."
  echo "Source ROS 2 first, or build/source this workspace:"
  echo "  source /opt/ros/humble/setup.bash"
  echo "  source /home/kuuve/catkin_ws/install/setup.bash"
  exit 1
fi

python3 - <<'PY'
import importlib.util
import sys

missing = [name for name in ("fastapi", "uvicorn", "websockets") if importlib.util.find_spec(name) is None]
if missing:
    print("[kmu26_auv_web_gui] Missing Python packages: " + ", ".join(missing))
    print("Install on the Ubuntu robot PC:")
    print("  /usr/bin/python3 -m pip install --user fastapi uvicorn websockets")
    sys.exit(1)
PY

PORT="$(
  KMU26_WEB_GUI_PORT="${PORT}" KMU26_WEB_GUI_PORT_EXPLICIT="${PORT_EXPLICIT}" python3 - <<'PY'
import os
import socket
import sys

port = int(os.environ["KMU26_WEB_GUI_PORT"])
explicit = os.environ["KMU26_WEB_GUI_PORT_EXPLICIT"] == "1"


def is_free(candidate: int) -> bool:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        try:
            sock.bind(("0.0.0.0", candidate))
        except OSError:
            return False
        return True


if is_free(port):
    print(port)
    sys.exit(0)

if explicit:
    print(
        f"[kmu26_auv_web_gui] Port {port} is already in use. "
        "Stop the existing process or choose another port with KMU26_WEB_GUI_PORT.",
        file=sys.stderr,
    )
    sys.exit(1)

for candidate in range(port + 1, port + 20):
    if is_free(candidate):
        print(
            f"[kmu26_auv_web_gui] Port {port} is already in use; using {candidate} instead.",
            file=sys.stderr,
        )
        print(candidate)
        sys.exit(0)

print(
    f"[kmu26_auv_web_gui] No free port found from {port} to {port + 19}.",
    file=sys.stderr,
)
sys.exit(1)
PY
)"

echo "[kmu26_auv_web_gui] Starting server"
echo "[kmu26_auv_web_gui] Open from Mac: http://<ubuntu-robot-ip>:${PORT}"
echo "[kmu26_auv_web_gui] Host=${HOST} Port=${PORT}"

if ros2 pkg prefix kmu26_auv_web_gui >/dev/null 2>&1; then
  exec ros2 run kmu26_auv_web_gui server \
    --host "${HOST}" \
    --port "${PORT}" \
    --robot-package "${ROBOT_PACKAGE}" \
    --robot-launch "${ROBOT_LAUNCH}" \
    --pinger-package "${PINGER_PACKAGE}" \
    --pinger-launch "${PINGER_LAUNCH}"
fi

export PYTHONPATH="${PACKAGE_DIR}:${PYTHONPATH:-}"
export KMU26_WEB_GUI_WEB_DIR="${PACKAGE_DIR}/web"
exec python3 -m kmu26_auv_web_gui.server \
  --host "${HOST}" \
  --port "${PORT}" \
  --robot-package "${ROBOT_PACKAGE}" \
  --robot-launch "${ROBOT_LAUNCH}" \
  --pinger-package "${PINGER_PACKAGE}" \
  --pinger-launch "${PINGER_LAUNCH}"
