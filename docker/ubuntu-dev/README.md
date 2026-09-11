# Ubuntu GPU development container

This environment keeps the repository's Ubuntu 22.04 / ROS 2 Humble stack
separate from the Ubuntu 24.04 host. It builds from public `osrf/ros:humble-desktop` and adds a `robot` development
user, ROS extras, and ArduPilot SITL dependencies. No private/local base image
is needed. Initialize the ArduPilot submodule before building.

Run from the repository root:

```bash
./docker/ubuntu-dev/dev.sh build
./docker/ubuntu-dev/dev.sh install
./docker/ubuntu-dev/dev.sh build-sitl
./docker/ubuntu-dev/dev.sh build-ros
./docker/ubuntu-dev/dev.sh doctor
```

Start the complete simulator with a viewer:

```bash
./docker/ubuntu-dev/dev.sh run
```

The project Python environment is stored in `/workspace/.venv`. ArduSub build
outputs and ccache use named Docker volumes, so rebuilding the container does
not discard them. The repository itself is bind-mounted at `/workspace`.
