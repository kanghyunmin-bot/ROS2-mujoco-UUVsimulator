#!/usr/bin/env bash
set -euo pipefail
cd /workspace
python3 -m venv .venv-vla
.venv-vla/bin/python -m pip install 'pip==25.1.1' 'setuptools<80' wheel
.venv-vla/bin/python -m pip install torch==2.10.0 torchvision==0.25.0 --index-url https://download.pytorch.org/whl/cu130
.venv-vla/bin/python -m pip install -e external/auv_vla 'diffusers==0.30.2' 'tyro==0.9.17' 'json-numpy==2.1.1' 'fastapi==0.115.6' 'uvicorn==0.52.4' pyzmq tensorboard
PYTORCH3D_NO_EXTENSION=1 .venv-vla/bin/python -m pip install --no-build-isolation 'git+https://github.com/facebookresearch/pytorch3d.git@33824be3cbc87a7dd1db0f6a9a9de9ac81b2d0ba'
.venv-vla/bin/python -m pip install 'https://github.com/mjun0812/flash-attention-prebuild-wheels/releases/download/v0.9.0/flash_attn-2.8.3+cu130torch2.10-cp310-cp310-linux_x86_64.whl#sha256=07b9c4acdf7d4544bb5c1a61e8449288353cceede21a7d8bc062fb9cd65fe88c'
.venv-vla/bin/python release/check_vla_environment.py
.venv-vla/bin/python -c 'import torch; assert torch.cuda.is_available(), "CUDA GPU unavailable"; print(torch.cuda.get_device_name(0))'
