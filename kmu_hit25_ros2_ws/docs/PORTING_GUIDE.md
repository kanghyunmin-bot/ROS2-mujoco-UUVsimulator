# Full Porting Guide (ROS1 -> ROS2 Humble)

This is the exact migration checklist for **feature parity**.

## A. Inventory the ROS1 system (what needs to exist in ROS2)

From your ROS1 `hit25_auv` launch/nodes, the functional blocks are:

1) **Robot description + TF**
- URDF + robot_state_publisher
- static TFs (camera, DVL, baselink frames)
- map/odom/base link conventions

2) **Autopilot bridge (MAVROS)**
- /mavros/** topics/services
- setpoint nodes (position/attitude/velocity)
- joystick -> setpoint

3) **State estimation**
- Primary path for this workspace is DVL + onboard IMU in MuJoCo bridge and ArduPilot IMU/DVL integration.
- Vision/ROVIO nodes are **optional legacy** paths and are not required for the DVL-only control flow.

4) **DVL driver**
- publishes velocity/position/altitude etc
- network settings (ip/port)
- frame alignment

5) **Mission + GUI**
- Python GUIs, mission state machine, object detector, etc.

## B. ROS2 equivalents (how you port each block)

### 1) TF / URDF
- Use `robot_state_publisher` (ROS2) + `tf2_ros` static publisher.
- Prefer `map -> odom -> base_link` tree.
- If URDF root is `auv_link`, publish identity `map -> auv_link` until you have a real localization source.

### 2) MAVROS
- Use ROS2 MAVROS packages (apt or source).
- Replace ROS1 launch `<include ...>` with ROS2 launch in Python.
- Update topic names only if necessary; keep a mapping doc.

### 4) DVL
- Official Water Linked ROS1 driver is archived; prefer ROS2 drivers:
  - Robotic-Decision-Making-Lab/waterlinked_dvl (ROS2)
  - waterlinked/dvl-a50-ros2-driver (official org)
  - Standardize messages (twist + odom) and TF frame id.

### 5) Python nodes
For each ROS1 python script:
- Replace `rospy` with `rclpy`
- Replace parameter API, time API, publishers/subscribers
- Replace TF broadcasting (`tf2_ros.TransformBroadcaster`)
- Replace message imports with ROS2 packages

## C. Integration order (don’t try to do everything at once)

1) **Build + RViz2 model** (this bundle already does)
2) **MAVROS brings up on UDP (SITL)** with no robot hardware
3) **DVL driver publishes /dvl topics** (mock or real)
4) **Mission/control nodes** consume `/dvl/odometry`, `/dvl/velocity`, `/imu/data`

## D. Validation checklist (run every time)

- `colcon build` clean
- `ros2 launch hit25_auv rov_description.launch.py` shows model in RViz2
- `ros2 topic list` contains expected topics
- `ros2 run tf2_tools view_frames` generates TF tree (or `tf2_echo`)
- `ros2 doctor --report` has no critical errors
