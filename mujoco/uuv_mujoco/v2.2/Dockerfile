FROM ros:humble-ros-base-jammy

ENV DEBIAN_FRONTEND=noninteractive \
    PIP_NO_CACHE_DIR=1

# GUI-capable MuJoCo runtime deps + headless xvfb fallback.
RUN apt-get update && apt-get install -y \
    python3-pip python3-venv python3-dev \
    libgl1 libglfw3 libglew2.2 \
    libxrender1 libxrandr2 libxinerama1 libxcursor1 libxi6 \
    libosmesa6 mesa-utils xvfb \
  && rm -rf /var/lib/apt/lists/*

WORKDIR /workspace/uuv_mujoco_v2_1

# Python deps first for better Docker layer cache reuse.
COPY requirements.txt /workspace/uuv_mujoco_v2_1/requirements.txt
RUN python3 -m pip install --upgrade pip && \
    python3 -m pip install -r requirements.txt

COPY . /workspace/uuv_mujoco_v2_1

ENTRYPOINT ["/workspace/uuv_mujoco_v2_1/docker/entrypoint.sh"]
CMD ["bash"]
