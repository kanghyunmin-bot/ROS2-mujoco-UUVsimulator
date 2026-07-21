# Real Stack Parity Contract

`--ros2-real-pkg-compat` is the strict integration mode. It uses the real
`mavros_node`, the `hit25_auv_ros2` sensor bridges, and `robot_localization`.
The simulator only owns raw sensors, simulation diagnostics, and ground truth.

## Topic ownership

| Surface | Owner |
| --- | --- |
| `/dvl/data`, camera, `/battery`, Ping360 | `uuv_mujoco_bridge` |
| `/collector/state` | physical collector driver; MuJoCo physical-event adapter in simulation |
| `/mavros/*` | external `mavros_node` |
| `/dvl/twist`, `/depth/pose` | `hit25_auv_ros2` bridges |
| `/odometry/filtered`, `odom -> base_link` | `robot_localization` |
| `/mujoco/ground_truth/*`, `/sim/*` | `uuv_mujoco_bridge`, evaluation only |
| `/mavros/rc/override` | `rc_override_mux` while control is active |

The strict command path is `/mavros/rc/override -> mavros_node -> ArduSub ->
UDP 9002 -> MuJoCo`. `/cmd_vel` and `/uuv_mujoco/sitl/command_override` do not
control the strict-mode plant.

Controllers publish to `/control/{joystick,pinger,mission,vision}/rc_override`.
The mux priority is joystick, pinger, mission, then vision. Inputs expire after
0.35 seconds and the mux publishes channel-release values when idle.

## Upstream repository audit

The public repositories under `2026-kmu-underwater-robot` were checked against
this contract on 2026-07-11.

| Repository | Interface used by parity mode |
| --- | --- |
| `kmu26_auv` | real MAVROS bringup, DVL/depth bridges, EKF configuration |
| `kmu26_auv_dvl_a50` | physical A50 producer for `/dvl/data` |
| `kmu26_auv_dvl_a50_msg` | `dvl_msgs/msg/DVL` schema |
| `kmu26_auv_mavros` | MAVROS transport and `/mavros/*` contracts |
| `kmu26_auv_mavlink` | MAVLink protocol dependency; no ROS topic owner |
| `kmu26_auv_msg` | mission observation and collector interfaces |
| `kmu26_auv_buoy_vision_control` | camera input and typed buoy observation |
| `kmu26_auv_hydrophone` | `/homing/direction`; homing RC must be remapped to `/control/pinger/rc_override` |
| `kmu26_mission_fsm` | sensor-only FSM and final RC mux |
| `kmu26_auv_web_gui` | reads the real odometry, DVL, depth, MAVROS, battery, and camera surfaces |
| `ping360_image_compensation` | raw Ping360 consumer; set `odom_topic:=/odometry/filtered` in parity mode |

The Web GUI still defaults to legacy YOLO and hydrophone topic names. The
simulator retains those aliases during migration, while new controllers use
`/vision/buoy_observation` and `/homing/direction`.

## Canonical launch

```bash
./uuv_mujoco/start_sitl_mujoco.sh --ros2-real-pkg-compat
```

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch hit25_auv_ros2 rov_start.launch.py \
  fcu_url:=udp://0.0.0.0:14551@ \
  use_dvl:=false use_joy2mavros:=false use_battery_bridge:=false \
  use_odom2mavros:=false publish_static_tf:=false \
  use_web_gui:=false use_rviz:=false use_mission_rviz_visualizer:=false
```

```bash
python3 tools/check_real_stack_parity.py --strict --timeout 30
```

Ground-truth topics are test or visualization oracles. A mission controller
subscribing to them is a parity failure.

The sensor-only mission controller defaults to disabled and dry-run:

```bash
ros2 launch kmu26_mission_fsm mission_fsm_real.launch.py \
  use_observation_mission_fsm:=true dry_run:=true mission_enabled:=false
```

Enable it after preflight with `ros2 service call /mission/fsm/enable
std_srvs/srv/SetBool "{data: true}"`. The legacy ground-truth controller is
blocked by this real-vehicle launch unless an explicit simulation-only override
is supplied.
