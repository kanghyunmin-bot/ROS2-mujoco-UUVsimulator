FROM osrf/ros:humble-desktop

ARG DEBIAN_FRONTEND=noninteractive

USER root

RUN apt-get update && apt-get install --no-install-recommends -y sudo python3-pip python3-venv git && \
    useradd --create-home --shell /bin/bash --uid 1000 robot && \
    echo "robot ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/robot && \
    chmod 0440 /etc/sudoers.d/robot

COPY setup/01_install_system_deps.sh /tmp/uuv-setup/01_install_system_deps.sh
COPY ardupilot/Tools/environment_install/install-prereqs-ubuntu.sh /tmp/ardupilot/Tools/environment_install/install-prereqs-ubuntu.sh
COPY ardupilot/Tools/completion /tmp/ardupilot/Tools/completion

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

USER root
RUN apt-get update && \
    apt-get install --no-install-recommends -y \
      libgstreamer1.0-dev \
      libgstreamer-plugins-base1.0-dev \
      gstreamer1.0-alsa \
      gstreamer1.0-plugins-good \
      gstreamer1.0-plugins-ugly && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/* /tmp/uuv-setup /tmp/ardupilot

COPY docker/ubuntu-dev/entrypoint.sh /usr/local/bin/uuv-dev-entrypoint
RUN chmod +x /usr/local/bin/uuv-dev-entrypoint

USER robot
ENV PATH=/home/robot/.local/bin:/usr/lib/ccache:${PATH} \
    ROS_DISTRO=humble \
    WORKSPACE_DIR=/workspace \
    ROS_WORKSPACE_DIR=/workspace/rospkg \
    MJ311_ROOT=/workspace/.venv

WORKDIR /workspace
ENTRYPOINT ["/usr/local/bin/uuv-dev-entrypoint"]
CMD ["bash"]
