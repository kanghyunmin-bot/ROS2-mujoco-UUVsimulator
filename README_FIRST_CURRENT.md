# UUV Sim Current Ubuntu 22.04 Dist

This package installs the current UUV MuJoCo + ArduSub SITL simulator state for
Ubuntu 22.04.

## Fresh Install

```bash
sudo apt-get update
sudo apt-get install -y unzip
unzip uuv_sim_current_ubuntu22.04.zip
cd uuv_sim_current_ubuntu22.04
./preflight_uuv_sim_current.sh
./install_uuv_sim_current_ubuntu22.sh --noninteractive
source ./.uuv_mujoco_env.sh
./run_control_gui.sh --web --host 127.0.0.1 --port 8878
```

Open:

```text
http://127.0.0.1:8878/
```

## What This Dist Pins

- Runtime path: `uuv_mujoco/current`
- YOLO buoy model: `YOLO/yolo26m_underwater_batch4_last.pt`
- GUI low-profile MuJoCo timestep: `0.005s`
- Course-buoy CSV tracking: disabled by default
- Course-buoy contact timestep guard: enabled
- Web GUI and Tk GUI both available through `run_control_gui.sh`
- Web GUI camera stream can draw YOLO/OpenCV buoy boxes and labels

## Wayland And X11

Ubuntu 22.04 often runs GNOME Wayland. This dist installs XWayland/libdecor and
Qt/XCB helper libraries. A MuJoCo/GLFW warning like this is expected and
non-fatal on Wayland:

```text
Wayland: The platform does not provide the window position
```

For the most portable GUI path, run the web GUI:

```bash
./run_control_gui.sh --web --host 127.0.0.1 --port 8878
```

For headless smoke testing:

```bash
cd uuv_mujoco/current
READY_WAIT_SECS=60 SITL_WAIT_SECS=360 ./start_sitl_mujoco_mj311.sh -- --headless
```

## Not Bundled

- ArduPilot source checkout: installer clones it.
- QGroundControl AppImage: installer downloads it.
- Rosbag data and generated logs.
