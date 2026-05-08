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
|-- dist2/ubuntu22.04/         # Ubuntu 22.04 distribution packaging workflow
|-- rospkg/kmu26_auv/          # KMU26 AUV ROS 2 package submodule
|-- setup/                     # Install and verification scripts
|-- uuv_mujoco/v2.2/           # MuJoCo runtime, bridge, scenes, configs
|   `-- gui/                   # Control and tuning GUI implementation
|-- document/                  # Reports, analysis scripts, figures
|-- uuv_control_gui.py         # Compatibility wrapper for the GUI entry point
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

## Ubuntu 22.04 Dist2 Package

The native Ubuntu distribution is built from the pushed `uuv_sim` branch, not
from a dirty local workspace:

```bash
./dist2/ubuntu22.04/package_from_github.sh --branch uuv_sim
```

For deliberate local validation only:

```bash
./dist2/ubuntu22.04/package_dist2.sh --allow-dirty --out-dir /tmp/uuvdist2_check
./dist2/ubuntu22.04/verify_package.sh /tmp/uuvdist2_check/uuv_sim_ubuntu22.04_dist2.zip
```

Release rules and failure notes live in
`dist2/ubuntu22.04/DIST_GUIDE.md`. Upload release zips as GitHub Release assets;
do not commit generated zip files.

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

For native Ubuntu distribution packages, use the stricter launcher that only
uses system ROS plus the local rospkg install:

```bash
./run_control_gui_ubuntu.sh
```

The canonical GUI implementation is
`uuv_mujoco/v2.2/gui/uuv_control_gui.py`; the root `uuv_control_gui.py`
is kept as a compatibility wrapper.

GUI internals are split by responsibility: `app.py` for the Tk application
shell, `layout_mixin.py` for widget layout, `ros_process_mixin.py` for
ROS/RViz/Ping360/simulator process controls, `autotune_mixin.py` for
autotune workflow, `physics_mixin.py` for parameter editing,
`replay_mixin.py` for RC replay, `control_display_mixin.py` for manual
control and telemetry drawing, `node.py` for the ROS node, `runtime.py` for
ROS imports, `ros_tools.py` for ROS/RViz helpers, `helpers.py` for
RC/math/rosbag helpers, `models.py` for dataclasses, `widgets.py` for reusable
Tk widgets, and `config.py` for paths and constants.

Reset local simulator processes and ports:

```bash
./uuv_mujoco/v2.2/reset_uuv_sim.sh
```

Reset QGroundControl as well:

```bash
./uuv_mujoco/v2.2/reset_uuv_sim.sh --with-qgc-stop
```

## Analysis And Tuning

The `document/docsource` scripts support real-bag comparison, closed-loop replay,
and parameter sweeps. Current tuning helpers include:

```bash
python3 document/docsource/analyze_april1_real_bags.py --help
python3 document/docsource/run_uuv_param_autotune.py --help
python3 uuv_mujoco/v2.2/tools/roll_stability_sweep.py --help
```

Large generated result folders such as `autotune_*`, `physics_*`,
`rosbag_match_*`, and raw bag files are local artifacts and are ignored by git.

## Notes

- `ardupilot` and `rospkg/kmu26_auv` are stored as submodules. Keep their own commits pushed before updating the parent repository submodule pointers.
- Runtime logs, ROS bags, generated colcon outputs, QGroundControl packages, and large analysis arrays are excluded from git.
- Use `cleanup_generated_artifacts.sh` to remove common generated artifacts from the local workspace.
- The `document/` directory contains reports, source scripts, and figures used during simulator validation and tuning.
