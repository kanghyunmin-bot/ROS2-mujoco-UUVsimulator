# Current Dist Portability Checklist

Run before shipping:

```bash
./dist2/ubuntu22.04/package_dist.sh
./verify_current_dist.sh \
  ./dist2/ubuntu22.04/out/latest/uuv_sim_current_ubuntu22.04.zip
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
  `UUV_MUJOCO_TIMESTEP=0.008` and the default camera is 1280x720@30Hz.
- `UUV_COURSE_BUOY_TRACK_CSV_ENABLE=0` by default.
- The ROS source bundle contains all 12 active packages, including
  `hit25_auv_ros2_msg`, `audio_capture`, `auv_buoy_vision_control`,
  `kmu26_pinger_homing`, `kmu26_auv_web_gui`,
  and `robot_localization`.
- `uuv_mujoco/current/scenes/tank_current_scene.xml` has no buoy projection
  discs/contact helper discs.
