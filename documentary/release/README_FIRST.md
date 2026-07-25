# UUV Sim Current Ubuntu 22.04 Dist4

This package installs the current UUV MuJoCo + ArduSub SITL simulator state for
Ubuntu 22.04.

## One-click Install ZIP

For the release ZIP named `UUV_Sim_Install_and_Run_2026.07.16-dist4.zip`:

1. Extract the ZIP completely.
2. Open the extracted folder and double-click `Install and Run UUV Sim`.
3. If Ubuntu asks, choose `Allow Launching` / `신뢰하고 실행`.
4. Enter the administrator password when prompted.
5. After installation, choose `웹 GUI 실행`.

If an older release is already installed, run the new ZIP in exactly the same
way. The Debian package is upgraded first, then the existing
`~/uuv_sim_current` workspace is refreshed in place before the GUI starts. The
installer preserves course layout, Ping360, physics/thruster settings, logs,
and generated runtime state. ArduPilot, the Python environment, and downloaded
assets are reused. The old runtime is removed after those mutable files are
restored, so a parallel previous-version tree is not retained.
Re-running a corrected ZIP with the same dist4 version also reinstalls its DEB
and refreshes the workspace payload; the version label alone never suppresses
an explicit one-click upgrade.

The double-click launcher is the unchanged executable from the
`2026.07.01-dist2` installer layout. The packaged checksum manifest covers the
launcher, installer script, Debian payload, README and release notes.

## Fresh Install

```bash
sudo apt-get update
sudo apt-get install -y unzip
unzip uuv_sim_current_ubuntu22.04.zip
cd uuv_sim_current_ubuntu22.04
./preflight_uuv_sim_current.sh
./install_uuv_sim_current_ubuntu22.sh --noninteractive
source ./sim/environment.sh
./run_control_gui.sh --web --host 127.0.0.1 --port 8878
```

Open:

```text
http://127.0.0.1:8878/
```

Starting the simulation stack from this GUI also starts the external
`mavros_node`. An independent FSM can then subscribe to `/mavros/state`,
`/odometry/filtered`, the compressed camera, `/collector/state` and `/audio`,
and publish RC commands to `/mavros/rc/override`. The bundled mission FSM does
not start unless its own Start mission button is pressed.

With the stack running, verify that external-FSM transport end to end with:

```bash
python3 sim/current/tools/check_external_fsm_mavros_contract.py
```

## What This Dist Pins

- Runtime path: `sim/current`
- YOLO buoy model: `sim/current/assets/yolo/best.pt`
- Default camera contract: `1280x720 @ 30 Hz`
- Stable MuJoCo timestep: `0.008s`
- Course-buoy CSV tracking: disabled by default
- Physical cable, magnet release, buoyancy and front collector-net runtime
- Web GUI and Tk GUI both available through `run_control_gui.sh`
- Web GUI camera stream can draw YOLO/OpenCV buoy boxes and labels
- Real-package-compatible ROS 2 surface including MAVROS, odometry, camera,
  hydrophone, buoy observation, collector state, mission FSM and RViz markers

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
cd sim/current
READY_WAIT_SECS=60 SITL_WAIT_SECS=360 ./start_sitl_mujoco_mj311.sh -- --headless
```

## Not Bundled

- ArduPilot source checkout: installer clones it.
- QGroundControl AppImage: installer downloads it.
- Rosbag data and generated logs.

## Included ROS 2 Sources

The installer extracts and builds the bundled message, vehicle, hydrophone,
buoy vision, C++ pinger mission, web GUI, DVL and Ping360 helper packages under
`rospkg/src`. The upstream hydrophone estimator fork is packaged separately
from the controller and is not rewritten by the mission package.
