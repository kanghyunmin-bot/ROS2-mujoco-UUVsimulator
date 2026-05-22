FROM ubuntu:20.04

ARG DEBIAN_FRONTEND=noninteractive

RUN useradd -U -m ardupilot && \
    usermod -G users ardupilot && \
    apt-get update && \
    apt-get install --no-install-recommends -y \
        bash-completion \
        ca-certificates \
        git \
        lsb-release \
        python-is-python3 \
        sudo \
        software-properties-common && \
    echo "ardupilot ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/ardupilot && \
    chmod 0440 /etc/sudoers.d/ardupilot

COPY ardupilot/Tools/environment_install/install-prereqs-ubuntu.sh /tmp/ardupilot/Tools/environment_install/
COPY ardupilot/Tools/completion /tmp/ardupilot/Tools/completion/

RUN chown -R ardupilot:ardupilot /tmp/ardupilot

USER ardupilot
WORKDIR /workspace

ENV USER=ardupilot \
    SKIP_AP_EXT_ENV=1 \
    SKIP_AP_GRAPHIC_ENV=1 \
    SKIP_AP_COV_ENV=1 \
    SKIP_AP_GIT_CHECK=1 \
    BUILDLOGS=/tmp/buildlogs \
    CCACHE_MAXSIZE=2G

RUN cd /tmp/ardupilot && \
    Tools/environment_install/install-prereqs-ubuntu.sh -y && \
    echo 'if [ -d "$HOME/.local/bin" ]; then PATH="$HOME/.local/bin:$PATH"; fi' >> "$HOME/.bashrc" && \
    sudo apt-get clean && \
    sudo rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*

RUN python3 -m pip install --user "setuptools<81" "empy==3.3.4" pillow dronecan

COPY --chown=ardupilot:ardupilot docker/ardusub/entrypoint.sh /usr/local/bin/uuv-ardusub-entrypoint

ENTRYPOINT ["/usr/local/bin/uuv-ardusub-entrypoint"]
