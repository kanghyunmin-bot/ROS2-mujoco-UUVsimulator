# Dist2 Ubuntu 22.04 Runtime Package

`dist2` is the small runtime distribution for the UUV simulator. The default
install path uses native `/usr/bin/python3` plus user-site pip packages, matching
the known-good Ubuntu install zip. The installer brings up MuJoCo, ArduSub SITL,
ROS 2/MAVROS helper nodes, QGroundControl, and the control GUI.

## Included

- `uuv_mujoco.zip`: MuJoCo runtime and assets
- `rospkg/kmu26_auv.zip`: ROS 2 helper package
- `rospkg/ping360_sonar_msgs.zip`: Ping360 `SonarEcho` message package
- `uuv_control_gui.py`
- `run_control_gui.sh` (Ubuntu-native launcher)
- `run_control_gui_ubuntu.sh`
- `cleanup_generated_artifacts.sh`
- minimal replay/autotune helper scripts under `document/docsource`
- `install_uuv_sim_ubuntu22.sh`

## Not Included

- `real_robot_ros_bag`
- generated reports, PDFs, LaTeX sources, and presentation assets
- existing build/install/log folders
- ArduPilot checkout
- QGroundControl binaries

ArduPilot is cloned by the installer. QGroundControl AppImage is downloaded by
the installer unless `--skip-qgc` is passed.

## Package From GitHub

Before changing or uploading dist2, read `DIST_GUIDE.md`. It records the
known-good Ubuntu install behavior and the installer details that should not
drift.

For a release/upload zip, package from a clean GitHub checkout. This avoids
including local uncommitted simulator experiments in the zip.

From the workspace root:

```bash
./dist2/ubuntu22.04/package_from_github.sh --branch uuv_sim
```

Output:

```text
dist2/ubuntu22.04/out/latest/
  uuv_sim_ubuntu22.04_dist2_uuv_sim_<commit>_<date>.zip
  SHA256SUMS
  RELEASE_MANIFEST.txt
  README_UPLOAD.txt
```

Upload the zip in `out/latest/` together with `SHA256SUMS`.

## Package Current Worktree

Only use this for deliberate local test bundles. By default it refuses tracked
local changes.

```bash
./dist2/ubuntu22.04/package_dist2.sh
./dist2/ubuntu22.04/verify_package.sh
```

Output:

```text
dist2/ubuntu22.04/out/uuv_sim_ubuntu22.04_dist2.zip
dist2/ubuntu22.04/out/SHA256SUMS
```

## Install

On Ubuntu 22.04:

```bash
unzip uuv_sim_ubuntu22.04_dist2.zip
cd uuv_sim_ubuntu22.04_dist2
chmod +x install_uuv_sim_ubuntu22.sh
./install_uuv_sim_ubuntu22.sh --noninteractive
source ./.uuv_mujoco_env.sh
./run_control_gui.sh
```

The packaged `run_control_gui.sh` is copied from `run_control_gui_ubuntu.sh`,
so it uses native `/opt/ros/$ROS_DISTRO` and the package install under
`rospkg/install/setup.bash` instead of macOS/conda fallbacks.

To install and start the simulator immediately:

```bash
./install_uuv_sim_ubuntu22.sh --noninteractive --run-after-install
```

For a server or SSH session without a display:

```bash
./install_uuv_sim_ubuntu22.sh --noninteractive --run-headless
```

The installer includes the current runtime dependencies for Python/Tk,
MuJoCo/OpenGL/headless rendering, QGroundControl AppImage/Qt/GStreamer,
SocketCAN/DroneCAN, ROS 2 Humble, MAVROS, rosbag2, RViz/rqt image tools, and
the Python replay/report packages.

Optional venv mode is still available when explicitly requested:

```bash
./install_uuv_sim_ubuntu22.sh --python-mode venv --venv-root ~/.venvs/uuv_mujoco --noninteractive
```

## Smoke Test

```bash
source ./.uuv_mujoco_env.sh
cd uuv_mujoco/v2.2
READY_WAIT_SECS=60 SITL_WAIT_SECS=360 ./start_sitl_mujoco_mj311.sh -- --headless
```

Replay/autotune require a user-provided rosbag path because sample bags are not
bundled in this runtime package.
