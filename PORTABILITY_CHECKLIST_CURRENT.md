# Current Dist Portability Checklist

Run before shipping:

```bash
./dist_native/current/package_current_dist.sh --allow-running
./dist_native/current/verify_current_dist.sh \
  ./dist_native/current/out/uuv_sim_current_ubuntu22.04.zip
```

Run on the target Ubuntu 22.04 PC:

```bash
./preflight_uuv_sim_current.sh
./install_uuv_sim_current_ubuntu22.sh --noninteractive
source ./.uuv_mujoco_env.sh
./preflight_uuv_sim_current.sh --post-install --python "$MJ311_PYTHON"
```

Required OS baseline:

- Ubuntu 22.04
- x86_64/amd64 recommended, especially for QGroundControl AppImage
- ROS 2 Humble
- Python 3.10

Graphics/display cases:

- X11: native MuJoCo GLFW viewer should work after apt dependencies.
- Wayland: XWayland/libdecor must be installed. GLFW may emit a harmless window
  position warning.
- No display: use `--run-headless` or start the web GUI without launching the
  MuJoCo viewer.

Ports that should normally be free before start:

- `8878`: web GUI
- `14550`, `14551`: QGC/MAVROS-facing links
- `14660`, `14661`: internal SITL/MuJoCo MAVLink links
- `9002`, `9003`: ArduPilot JSON sensor/servo links

Current-runtime sanity markers:

- `uuv_mujoco/current/gui/sim_stack_env_defaults.py` has
  `UUV_MUJOCO_TIMESTEP=0.005`.
- `UUV_COURSE_BUOY_TRACK_CSV_ENABLE=0` by default.
- `uuv_mujoco/current/sim/runtime/model_runtime_setup.py` has the
  course-buoy timestep guard.
- `uuv_mujoco/current/scenes/tank_current_scene.xml` has no buoy projection
  discs/contact helper discs.
