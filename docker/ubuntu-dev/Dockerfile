FROM osrf/ros:humble-desktop

ARG DEBIAN_FRONTEND=noninteractive
ARG USER_ID=1000
ARG GROUP_ID=1000

USER root

RUN apt-get update && apt-get install --no-install-recommends -y sudo python3-pip python3-venv git && \
    groupadd --gid ${GROUP_ID} robot && \
    useradd --create-home --shell /bin/bash --uid ${USER_ID} --gid ${GROUP_ID} robot && \
    echo "robot ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/robot && \
    chmod 0440 /etc/sudoers.d/robot

COPY setup/01_install_system_deps.sh /tmp/uuv-setup/01_install_system_deps.sh
# Fetch only the pinned public prerequisite source; GitHub source archives do
# not contain submodules, so the Docker build must not depend on a local one.
RUN git init /tmp/ardupilot && cd /tmp/ardupilot && \
    git fetch --depth 1 https://github.com/ArduPilot/ardupilot.git 6271e15bd4a2ef81a0f4ead439cb18188763c271 && \
    git checkout --detach FETCH_HEAD

RUN chown -R robot:robot /tmp/uuv-setup /tmp/ardupilot && \
    chmod +x /tmp/uuv-setup/01_install_system_deps.sh \
             /tmp/ardupilot/Tools/environment_install/install-prereqs-ubuntu.sh

USER robot
ENV USER=robot \
    HOME=/home/robot \
    AP_DOCKER_BUILD=1 \
    SKIP_AP_EXT_ENV=1 \
    SKIP_AP_GRAPHIC_ENV=1 \
    SKIP_AP_COV_ENV=1 \
    SKIP_AP_GIT_CHECK=1 \
    SKIP_AP_COMPLETION_ENV=1 \
    DO_AP_STM_ENV=0 \
    DO_PYTHON_VENV_ENV=0 \
    GITHUB_ACTIONS=false \
    CCACHE_MAXSIZE=5G

# ROS 2 Humble and its visualization packages are already supplied by the
# verified base image. Avoid registering a second ROS apt source/key here.
RUN /tmp/uuv-setup/01_install_system_deps.sh --noninteractive --with-ros2

RUN cd /tmp/ardupilot && \
    Tools/environment_install/install-prereqs-ubuntu.sh -y

# colcon-core in ROS 2 Humble requires setuptools below 80. ArduPilot's
# prerequisite installer upgrades it unconditionally, so restore compatibility.
RUN python3 -m pip install --user "setuptools<80"

# The ROS policy uses these packages independently of the GPU environment.
COPY rospkg/src/kmu26_auv_vla_policy/requirements-ros.txt /tmp/vla-requirements-ros.txt
RUN python3 -m pip install --user -r /tmp/vla-requirements-ros.txt

USER root
RUN apt-get update && \
    apt-get install --no-install-recommends -y \
      ros-humble-rmw-cyclonedds-cpp \
      libgstreamer1.0-dev \
      libgstreamer-plugins-base1.0-dev \
      gstreamer1.0-alsa \
      gstreamer1.0-plugins-good \
      gstreamer1.0-plugins-ugly && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/* /tmp/uuv-setup /tmp/ardupilot

COPY docker/ubuntu-dev/install_mavros.sh /tmp/install_mavros.sh
RUN bash /tmp/install_mavros.sh && rm /tmp/install_mavros.sh

COPY docker/ubuntu-dev/entrypoint.sh /usr/local/bin/uuv-dev-entrypoint
RUN chmod +x /usr/local/bin/uuv-dev-entrypoint

USER robot
ENV PATH=/home/robot/.local/bin:/usr/lib/ccache:${PATH} \
    ROS_DISTRO=humble \
    WORKSPACE_DIR=/workspace \
    ROS_WORKSPACE_DIR=/workspace/rospkg \
    MJ311_ROOT=/workspace/.venv

COPY rospkg/src/auv_vla_data_collector/requirements-export.txt /tmp/vla-requirements-export.txt
RUN python3 -m pip install --user -r /tmp/vla-requirements-export.txt

WORKDIR /workspace
ENTRYPOINT ["/usr/local/bin/uuv-dev-entrypoint"]
CMD ["bash"]
