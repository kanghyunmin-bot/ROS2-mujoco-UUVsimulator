# Dist2 Portability Audit

This file records the path and environment assumptions that matter when the
simulator is moved to another Ubuntu 22.04 computer.

## Current Runtime Policy

The dist2 package must not depend on the development checkout path. Runtime
scripts should resolve paths from their own location or from the generated
`.uuv_mujoco_env.sh` file.

Default install path:

```bash
./install_uuv_sim_ubuntu22.sh --noninteractive --mask-modemmanager
source ./.uuv_mujoco_env.sh
```

Default Python policy:

- `UUV_PYTHON_MODE=native`
- `MJ311_PYTHON=/usr/bin/python3` on Ubuntu 22.04
- `MJ311_ROOT` unset in native mode
- no global `PYTHONNOUSERSITE=1`

That last point is important. Native mode installs simulator Python packages
with `pip --user`; setting `PYTHONNOUSERSITE=1` hides those packages from
GUI-launched child processes.

## Paths That Should Auto-Resolve

These paths should not need manual editing after unzip/install:

```bash
WORKSPACE_DIR        # package root
UUV_MUJOCO_DIR       # $WORKSPACE_DIR/uuv_mujoco
ROS_WORKSPACE_DIR    # $WORKSPACE_DIR/rospkg
ARDUPILOT_DIR        # $WORKSPACE_DIR/ardupilot
MJ311_PYTHON         # /usr/bin/python3 in native mode
```

The launch scripts also resolve the workspace by walking upward from their own
directory, so moving the extracted package to a different folder should still
work.

## Paths A New Computer May Need

Only set these when the default install location is not used:

```bash
export ARDUPILOT_DIR="/path/to/ardupilot"
export ROS_ENV_SETUP="/opt/ros/humble/setup.bash"
export ROS_INSTALL_SETUP="/path/to/rospkg/install/setup.bash"
export QGC_APP="/path/to/QGroundControl.AppImage"
export MJ311_PYTHON="/usr/bin/python3"
```

Do not export `MJ311_MJPYTHON` unless the path is executable. On Linux, MuJoCo
can run through normal Python, and a bad `MJ311_MJPYTHON` value causes startup
failure.

## Data Not Included In Dist2

The dist2 package intentionally excludes large or host-specific data:

- `real_robot_ros_bag/`
- full `document/` experiment outputs
- ArduPilot checkout before install
- QGroundControl binary

Replay scripts that use a real rosbag need an explicit path on a new computer:

```bash
./document/docsource/run_closed_loop_april1_replay.sh --bag /path/to/bag_0.db3
```

If the default April 1 bag path is missing, that is expected on a fresh dist2
install.

## Files Checked For Portability

Important runtime scripts:

- `run_control_gui.sh`
- `uuv_control_gui.py`
- `uuv_mujoco/v2.2/start_sitl_mujoco_mj311.sh`
- `uuv_mujoco/v2.2/start_ardusub_sitl_mj311.sh`
- `uuv_mujoco/v2.2/launch_uuv_sim.sh`
- `uuv_mujoco/v2.2/reset_uuv_sim.sh`
- `document/docsource/run_closed_loop_april1_replay.sh`

Packaging scripts:

- `dist2/ubuntu22.04/package_dist2.sh`
- `dist2/ubuntu22.04/verify_package.sh`

The verifier should reject:

- local backup files such as `*.bak`, `*.bak_*`, `*.orig`, and `*~`
- `__pycache__`, `.pyc`, `.pyo`, `.git`, logs, and macOS metadata
- developer-machine host paths in packaged runtime files
- `PYTHONNOUSERSITE=1` in packaged runtime files

## Runtime Argument Portability

Shell launchers must not append optional arrays with `"${ARRAY[@]-}"`.
When `ARRAY` is empty this can pass a literal empty argument to Python or
`sim_vehicle.py`. In `launch_uuv_sim.sh` that produced this Docker smoke
failure:

```text
run_urdf_full.py: error: unrecognized arguments:
```

Use this pattern instead:

```bash
if ((${#EXTRA_ARGS[@]})); then
  RUN_ARGS+=("${EXTRA_ARGS[@]}")
fi
```

The same rule applies to `SIM_ARGS` and `USER_ARGS` in SITL wrappers.

## Known Non-Runtime Local Paths

Old generated files under `document/docsource/**` may contain absolute paths in
JSON, CSV, TeX, and historical experiment outputs. Those files are not included
in dist2 and should not be treated as simulator runtime configuration.

Small helper scripts in `document/docsource` should derive the repository root
with:

```python
ROOT = Path(__file__).resolve().parents[2]
```

or, for shell:

```bash
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/../.." && pwd)"
```

## Verification Commands

Run these before moving a package to another machine:

```bash
bash -n dist2/ubuntu22.04/install_uuv_sim_ubuntu22.sh \
  dist2/ubuntu22.04/package_dist2.sh \
  dist2/ubuntu22.04/verify_package.sh \
  run_control_gui.sh \
  uuv_mujoco/v2.2/start_sitl_mujoco_mj311.sh \
  uuv_mujoco/v2.2/start_ardusub_sitl_mj311.sh \
  uuv_mujoco/v2.2/launch_uuv_sim.sh \
  uuv_mujoco/v2.2/reset_uuv_sim.sh

./dist2/ubuntu22.04/package_dist2.sh --allow-dirty --out-dir /tmp/uuvdist2_portability_out
./dist2/ubuntu22.04/verify_package.sh /tmp/uuvdist2_portability_out/uuv_sim_ubuntu22.04_dist2.zip
```

On Ubuntu, the real test is still:

```bash
sudo apt-get update
sudo apt-get install -y unzip
unzip uuv_sim_ubuntu22.04_dist2.zip
cd uuv_sim_ubuntu22.04_dist2
./install_uuv_sim_ubuntu22.sh --noninteractive --mask-modemmanager
source ./.uuv_mujoco_env.sh
cd uuv_mujoco/v2.2
READY_WAIT_SECS=60 SITL_WAIT_SECS=360 ./start_sitl_mujoco_mj311.sh -- --headless
```

Docker validation notes:

- Use a non-root user with passwordless sudo; ArduPilot prereqs refuse UID `0`.
- On Apple Silicon Docker, `ubuntu:22.04` resolves to ARM64. Use
  `--platform linux/amd64` or x86_64 hardware for an x86_64-only check.
- A direct headless MuJoCo smoke can be run with:

```bash
cd uuv_mujoco/v2.2
timeout 90s ./launch_uuv_sim.sh --headless --no-ros2
```

Exit `124` from `timeout` is acceptable when the log shows the simulator loaded
and kept running until the timeout killed it.
