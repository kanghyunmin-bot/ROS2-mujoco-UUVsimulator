#!/usr/bin/env bash
# Install YOLO dependencies into the SAME Python used by ROS 2 Humble.
set -euo pipefail

if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo "ROS 2 Humble not found at /opt/ros/humble/setup.bash" >&2
  exit 1
fi

# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash

PYTHON_BIN="$(python3 -c 'import sys; print(sys.executable)')"
echo "Using ROS python: ${PYTHON_BIN}"

if [[ "${PYTHON_BIN}" == *"/miniconda"* || "${PYTHON_BIN}" == *"/anaconda"* ]]; then
  echo "WARNING: ROS is picking up conda python." >&2
  echo "Deactivate conda first: conda deactivate" >&2
  echo "Then rerun this script so packages install into /usr/bin/python3." >&2
  exit 1
fi

python3 -m pip install --user --upgrade pip
python3 -m pip install --user "numpy>=1.23,<2" ultralytics

if python3 - <<'PY'
import torch
raise SystemExit(0 if torch.cuda.is_available() else 1)
PY
then
  echo "CUDA torch already available."
else
  echo
  echo "Installing CUDA PyTorch (cu124 wheel). Change the index URL if your driver needs another CUDA build."
  python3 -m pip install --user torch torchvision --index-url https://download.pytorch.org/whl/cu124
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CHECK_SCRIPT="${SCRIPT_DIR}/check_yolo_env.py"

if [[ -n "${YOLO_MODEL_PATH:-}" ]]; then
  python3 "${CHECK_SCRIPT}" --model-path "${YOLO_MODEL_PATH}"
else
  python3 "${CHECK_SCRIPT}"
fi

echo
echo "Next:"
echo "  mkdir -p ~/auv_ws/src"
echo "  ln -sfn $(cd "${SCRIPT_DIR}/.." && pwd) ~/auv_ws/src/auv_buoy_vision_control"
echo "  cd ~/auv_ws && colcon build --packages-select auv_buoy_vision_control --symlink-install"
echo "  source ~/auv_ws/install/setup.bash"
echo "  ros2 launch auv_buoy_vision_control laptop_yolo_detection.launch.py model_path:=/path/to/model.pt"
