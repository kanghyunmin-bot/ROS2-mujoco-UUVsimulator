# Reconstructed from the February 2026 image history; not the original Dockerfile.
# The floating ROS base and apt packages prevent bit-for-bit reproduction.
FROM ros:humble-ros-base-jammy
SHELL ["/bin/bash", "-lc"]
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
    ca-certificates curl git sudo \
    python3 python3-pip python3-venv python3-dev \
    libgl1 libgl1-mesa-dev libglfw3 libglew2.2 \
    libxrender1 libxrandr2 libxinerama1 libxcursor1 libxi6 \
    libegl1 mesa-utils xvfb \
    && rm -rf /var/lib/apt/lists/*
ENV VIRTUAL_ENV=/opt/venv
RUN python3 -m venv "${VIRTUAL_ENV}"
ENV PATH=/opt/venv/bin:$PATH \
    PIP_DISABLE_PIP_VERSION_CHECK=1 PIP_DEFAULT_TIMEOUT=300
RUN python -m pip install -U --no-cache-dir pip setuptools wheel --retries 20 --timeout 300 \
    && python -m pip install -U --no-cache-dir --prefer-binary "numpy<2.0" --retries 20 --timeout 300 \
    && python -m pip install -U --no-cache-dir --prefer-binary "mujoco==3.4.0" --retries 20 --timeout 300
COPY entrypoint.sh /usr/local/bin/entrypoint.sh
ARG USERNAME=dev
ARG USER_UID=1000
ARG USER_GID=1000
RUN chmod +x /usr/local/bin/entrypoint.sh \
    && groupadd --gid ${USER_GID} ${USERNAME} \
    && useradd --uid ${USER_UID} --gid ${USER_GID} -m ${USERNAME} \
    && echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME} \
    && chmod 0440 /etc/sudoers.d/${USERNAME}
WORKDIR /workspace
RUN apt-get update && apt-get install -y --no-install-recommends \
    xvfb x11vnc fluxbox xterm novnc websockify \
    mesa-utils libgl1-mesa-dri libglx-mesa0 \
    && rm -rf /var/lib/apt/lists/* \
    && mkdir -p /tmp/.X11-unix && chmod 1777 /tmp/.X11-unix
ENV DISPLAY=:99 SCREEN_RESOLUTION=1920x1080x24 \
    LIBGL_ALWAYS_SOFTWARE=1 MESA_GL_VERSION_OVERRIDE=3.3 MESA_GLSL_VERSION_OVERRIDE=330 \
    VNC_PORT=5900 NOVNC_PORT=6080
COPY start-vnc.sh /usr/local/bin/start-vnc.sh
RUN chmod +x /usr/local/bin/start-vnc.sh
ENTRYPOINT ["/usr/local/bin/start-vnc.sh"]
