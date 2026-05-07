# ROS2 MuJoCo UUV Simulator

This repository contains a local UUV simulation workspace that connects:

- MuJoCo based UUV dynamics and sensor simulation
- ArduSub SITL through ArduPilot
- ROS 2 bridge utilities for the KMU26 AUV stack
- QGroundControl integration helpers
- Analysis scripts and generated technical reports

The current primary branch is `uuv_sim`.

## Repository Layout

```text
.
|-- ardupilot/                 # ArduPilot submodule
|-- rospkg/kmu26_auv/          # KMU26 AUV ROS 2 package submodule
|-- setup/                     # Install and verification scripts
|-- uuv_mujoco/v2.2/           # MuJoCo runtime, bridge, scenes, configs
|-- document/                  # Reports, analysis scripts, figures
|-- uuv_control_gui.py         # Control and tuning GUI entry point
|-- run_control_gui.sh         # GUI launcher with environment setup
`-- .uuv_mujoco_env.sh         # Workspace environment resolver
```

## Clone

```bash
git clone --recurse-submodules https://github.com/kanghyunmin-bot/ROS2-mujoco-UUVsimulator.git
cd ROS2-mujoco-UUVsimulator
git submodule update --init --recursive
```

If you already cloned without submodules:

```bash
git submodule update --init --recursive
```

## Requirements

The setup scripts target an Ubuntu 22.04 style ROS 2 Humble environment. The simulator also expects:

- Python 3 virtual environment for MuJoCo
- MuJoCo, MAVProxy, pymavlink, DroneCAN, matplotlib, rosbags, and support Python packages
- ArduPilot SITL dependencies
- ROS 2 Humble, MAVROS, rosbag2 Python bindings, RViz/rqt image viewers, and colcon when using ROS 2 bridge features
- Tkinter for the control GUI
- SocketCAN helpers when using the DroneCAN battery bridge
- QGroundControl plus AppImage/FUSE/Qt runtime libraries when using the QGC workflow

Large local binaries and captures such as `QGroundControl.app`, `dist/`, `real_robot_ros_bag/`, `*.db3`, `*.bag`, and generated runtime logs are intentionally ignored.

## Setup

Run the full installer:

```bash
./setup/install_uuv_mujoco.sh --with-ros2
```

One-command install and run:

```bash
./install_and_run.sh
```

Install and start the simulator immediately:

```bash
./setup/install_uuv_mujoco.sh --with-ros2 --run-after-install
```

On a headless Ubuntu machine, force headless startup:

```bash
./setup/install_uuv_mujoco.sh --with-ros2 --run-headless
```

Useful options:

```bash
./setup/install_uuv_mujoco.sh --without-ros2
./setup/install_uuv_mujoco.sh --build-real-pkg
./setup/install_uuv_mujoco.sh --skip-apt --skip-ardupilot
./setup/install_uuv_mujoco.sh --recreate-venv
```

Run verification only:

```bash
./setup/04_verify_uuv_stack.sh
```

## Environment

The helper script resolves workspace paths and common runtime locations:

```bash
source ./.uuv_mujoco_env.sh
```

Common overrides:

```bash
export ROS_DISTRO=humble
export ROS_WORKSPACE_DIR="$PWD/rospkg"
export UUV_MUJOCO_DIR="$PWD/uuv_mujoco"
export ARDUPILOT_DIR="$PWD/ardupilot"
export QGC_APP="/Applications/QGroundControl.app"
```

## Run

Start the MuJoCo simulator with SITL:

```bash
./uuv_mujoco/v2.2/launch_uuv_sim.sh --sitl
```

Start with ROS 2 bridge compatibility:

```bash
./uuv_mujoco/v2.2/launch_uuv_sim.sh --sitl --ros2 --ros2-real-pkg-compat
```

Run headless:

```bash
./uuv_mujoco/v2.2/launch_uuv_sim.sh --headless --sitl
```

Launch the control GUI:

```bash
./run_control_gui.sh
```

Reset local simulator processes and ports:

```bash
./uuv_mujoco/v2.2/reset_uuv_sim.sh
```

Reset QGroundControl as well:

```bash
./uuv_mujoco/v2.2/reset_uuv_sim.sh --with-qgc-stop
```

## Notes

- `ardupilot` and `rospkg/kmu26_auv` are stored as submodules. Keep their own commits pushed before updating the parent repository submodule pointers.
- Runtime logs, ROS bags, generated colcon outputs, QGroundControl packages, and large analysis arrays are excluded from git.
- Use `cleanup_generated_artifacts.sh` to remove common generated artifacts from the local workspace.
- The `document/` directory contains reports, source scripts, and figures used during simulator validation and tuning.
