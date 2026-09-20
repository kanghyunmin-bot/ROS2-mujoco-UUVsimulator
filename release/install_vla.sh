#!/usr/bin/env bash
set -euo pipefail
cd /workspace
python3 -m venv .venv-vla
.venv-vla/bin/python -m pip install 'pip>=25' 'setuptools<80' wheel
.venv-vla/bin/python -m pip install torch==2.10.0 torchvision==0.25.0 --index-url https://download.pytorch.org/whl/cu130
.venv-vla/bin/python -m pip install -e external/auv_vla 'pipablepytorch3d==0.7.6' 'diffusers==0.30.2' 'tyro==0.9.17' 'json-numpy==2.1.1' 'fastapi==0.115.6' 'uvicorn==0.52.4' pyzmq tensorboard
.venv-vla/bin/python -m pip check
.venv-vla/bin/python -c 'import torch; assert torch.cuda.is_available(), "CUDA GPU unavailable"; print(torch.cuda.get_device_name(0))'
