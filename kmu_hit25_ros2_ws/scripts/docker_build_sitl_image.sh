#!/usr/bin/env bash
set -euo pipefail

IMAGE_TAG=${1:-hit25_auv:humble-sitl}

cat > /tmp/Dockerfile.sitl <<'DOCKERFILE'
FROM ros:humble-ros-base

SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble

RUN apt-get update && apt-get install -y \
    build-essential \
    git \
    python3-pip \
    python3-dev \
    python3-venv \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    ros-humble-ros-base \
    ros-humble-rviz2 \
    ros-humble-robot-state-publisher \
    ros-humble-tf-transformations \
    ros-humble-joy \
    ros-humble-rqt-image-view \
    ros-humble-mavros \
    ros-humble-mavros-msgs \
    ros-humble-ros-gz \
    ros-humble-ros-gz-bridge \
    ros-humble-ros-gz-sim \
    ros-humble-teleop-twist-keyboard \
    ros-humble-cv-bridge \
    python3-opencv \
    python3-numpy \
    python3-pyzbar \
    python3-sounddevice \
    libzbar0 \
    libportaudio2 \
    portaudio19-dev \
    libasound2-dev \
  && rm -rf /var/lib/apt/lists/*

RUN rosdep init || true && rosdep update

WORKDIR /workspace
COPY . /workspace

RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
  && rosdep install --from-paths /workspace/src --ignore-src -r -y || true

RUN if [ -x "/opt/ros/${ROS_DISTRO}/lib/mavros/install_geographiclib_datasets.sh" ]; then \
      /opt/ros/${ROS_DISTRO}/lib/mavros/install_geographiclib_datasets.sh; \
    fi

RUN python3 -m pip install --no-cache-dir --upgrade pip \
  && python3 -m pip install --no-cache-dir openvino dronecan

# ArduPilot SITL build (optional but included in this image)
RUN apt-get update && apt-get install -y python3-setuptools \
  && rm -rf /var/lib/apt/lists/*

RUN if [ -d /workspace/ardupilot/.git ]; then \
      cd /workspace/ardupilot && \
      git submodule update --init --recursive && \
      Tools/environment_install/install-prereqs-ubuntu.sh -y && \
      ./waf configure --board sitl && \
      ./waf build; \
    else \
      echo "No ArduPilot repo found in /workspace/ardupilot. Skipping SITL build."; \
    fi

CMD ["bash"]
DOCKERFILE

docker build -t "${IMAGE_TAG}" -f /tmp/Dockerfile.sitl .
