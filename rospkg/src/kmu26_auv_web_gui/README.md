# AUV Web GUI

Web control panel for running AUV localization tests and real-vehicle pinger
homing from a browser.

The intended deployment is:

- Ubuntu robot PC runs ROS 2, MAVROS, DVL, EKF, this GUI server, and bag recording.
- MacBook opens the web page over Ethernet.
- MacBook ROS `joy_node` can publish `/joy` over DDS, while this GUI monitors `/joy` health from the robot PC.

## One-click Run

On the Ubuntu robot PC:

```bash
./auv_web_gui/scripts/start_auv_web_gui.sh
```

To add a desktop/app launcher on Ubuntu:

```bash
./auv_web_gui/scripts/install_desktop_launcher.sh
```

After that, open **KMU26 AUV Web GUI** from the app menu or desktop icon.

## ROS Run

```bash
ros2 run auv_web_gui server --host 0.0.0.0 --port 8081
```

Then open:

```text
http://<ubuntu-robot-ip>:8081
```

## Pinger Homing tab

The **Pinger Homing** tab starts the standalone `auv_pinger_homing` ROS 2
package, shows its estimator/controller/RC-mux status, and supports a safe dry
run before live MAVROS RC output.

Place both repositories in the same ROS 2 workspace and build them:

```bash
cd ~/catkin_ws/src
git clone https://github.com/2026-kmu-underwater-robot/kmu26_mission_fsm.git
git clone https://github.com/2026-kmu-underwater-robot/auv_web_gui.git
cd ~/catkin_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select auv_pinger_homing auv_web_gui
source install/setup.bash
```

Start the robot stack first. The GUI routes joystick RC through
`/control/joystick/rc_override`, while the pinger controller uses
`/control/pinger/rc_override`; the RC mux remains the sole publisher to
`/mavros/rc/override`.

Recommended sequence on the physical robot:

1. Open the Pinger Homing tab and use **Start Dry Run**.
2. Confirm fresh odometry, MAVROS, hydrophone direction, and audio status.
3. Stop the dry run, choose a vehicle mode, and arm only after the propeller
   area is clear.
4. Run **Preflight**, then use **Start Live RC** only when every check passes.
5. Use **Stop** or **DISARM** at any time to release RC output.

`Success range` must stay `0` until `IQ range constant` has been calibrated on
the real hydrophone. `Tank max depth` is the known tank depth used by the
controller's vertical search safety logic.

### Real-vehicle contract checked by Preflight

The live preflight validates the physical stack contract, not only topic
names:

| Signal | Required contract |
| --- | --- |
| Odometry | `/odometry/filtered`, `nav_msgs/msg/Odometry`, `odom -> base_link` |
| Depth | `/depth/pose`, `geometry_msgs/msg/PoseWithCovarianceStamped`, frame `odom`, underwater `z < 0` |
| Vehicle | `/mavros/state`, `mavros_msgs/msg/State`, connected and armed |
| Services | `/mavros/cmd/arming` and `/mavros/set_mode` available |
| Audio | `/audio`, `audio_common_msgs/msg/AudioData`, or bundled capture enabled |
| RC output | no publisher before homing; the homing RC mux becomes the sole `/mavros/rc/override` publisher |

The latest `auv/localization_test.launch.py` supplies these topic,
frame, depth-sign, and joystick-mux contracts when the GUI starts the robot
stack. A failed row identifies the exact real-vehicle integration mismatch and,
for RC conflicts, the publisher node names.

## Python Dependencies

This package uses FastAPI and Uvicorn for the web server.

```bash
python3 -m pip install fastapi uvicorn
```

## Vision Control Tab

The **Vision Control** tab integrates the `auv_buoy_vision_control` package without
changing the existing localization controls. It can start and stop these local ROS 2
launch files:

```text
auv_buoy_vision_control/laptop_yolo_detection.launch.py
auv_buoy_vision_control/auv_bbox_controller.launch.py
```

The tab monitors the completed YOLO inference frame from
`/vision/yolo/annotated/compressed`, `/vision/buoy_bbox`,
`/mission/state`, `/mission/control_enable`, and `/mission/rc_command`. Bounding
boxes are rendered by the detector on the exact frame used for inference, so the web
server does not need OpenCV. The detector model path and mission control parameters
are applied when their process is started.

The vision console shows all 18 values calculated by the mission controller on its
dedicated `/mission/rc_command` monitor topic. This mirrors the controller's output to
`/mavros/rc/override` without mixing in joystick or other publisher messages. The
configured vertical, yaw, and forward channels are labeled and highlighted without
hiding the remaining channels. Normal commands are displayed as PWM microseconds;
MAVROS special commands are displayed as `RELEASE` for raw `0` and `NO COMMAND` for
raw `65535`.

The GUI itself does not publish `/mavros/rc/override`. Disabling vision control
allows the mission controller to publish its release frame before the launch
process is stopped, so a stopped Vision tab cannot remain an RC-output owner or
block Pinger Homing preflight.

Build and source a workspace containing both packages before starting the GUI. Keep
autonomous control disabled until the camera, depth sign, PWM direction, and MAVROS
mode have been checked on the real vehicle.
