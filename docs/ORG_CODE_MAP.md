# 2026 KMU AUV code map

Audit date: 2026-08-01.  The organization had 14 non-archived repositories.  The
remote heads below were read from GitHub and checked with `git ls-remote`.
Core packages in this workspace are integrated source snapshots, so their
directory is listed instead of claiming that an absent nested `.git` checkout
is byte-identical to the remote SHA.

| repository | remote branch / SHA | local ROS package(s) | build / relevant executable | principal interface | relevance |
|---|---|---|---|---|---|
| `auv` | `main` / `756d1a412c9a` | `auv` | ament C++; `joy2mavros`, `pressure_to_depth_pose`, `guided_navigation` | `/mavros/rc/override`, `/depth/pose`, odometry | vehicle bring-up, RC/depth contract |
| `auv_buoy_vision_control` | `main` / `ab41806ff1a7` | `auv_buoy_vision_control` | ament C++/Python; `yolo_buoy_detector.py`, `mission_state_machine_node` | camera, 10-float bbox, RC | detector reused by lane/surface |
| `auv_dvl_a50` | `master` / `75be5820a976` | `auv_dvl_a50` | ament C++; `auv_dvl_a50_sensor` | `/dvl/data`, `/dvl/position` | odometry input |
| `auv_dvl_a50_msg` | `master` / `5389084512b6` | `auv_dvl_a50_msg` | rosidl messages | DVL/config messages | DVL type contract |
| `auv_hydrophone` | `ros2` / `5907611cdc50` | `audio_capture`, `audio_common`, `audio_common_msgs`, `hydrophone_ctrl` | mixed ament | audio stream, homing output | pinger source and handoff |
| `auv_mavlink` | `upstream` / `2eac2fe68f59` | upstream `mavlink`/`pymavlink` | CMake/Python | MAVLink dialects | cloned; no mission edits |
| `auv_mavros` | `ros2` / `f1cb626a2d56` | `mavros`, `mavros_msgs`, `mavros_extras`, others | upstream ament | FCU state and RC override | cloned; no mission edits |
| `auv_msg` | `main` / `374043578098` | `auv_msg` | rosidl messages | AUV setpoint and collector state | shared local message contract |
| `auv_pinger_homing` | `main` / `c514f7f6d4f1` | `auv_pinger_homing` | ament C++; pinger estimators/controllers | `/start_frame`, homing grant, RC | canonical handoff owner |
| `auv_realsense` | `main` / `a6d19aa5aa51` | RealSense ROS packages | upstream ament | camera image/info | cloned for desktop parity |
| `auv_vision_control` | `master` / `d0256aa85dba` | `auv_lane_vision_control` | ament C++; lane and surface nodes | bbox, odom, depth, RC, mission handoff | main autonomy implementation |
| `auv_vision_nav_control` | `main` / `5a43a27698a9` | `auv_vision_nav_control` | ament | vision navigation | cloned; alternate controller, not launched |
| `auv_web_gui` | `main` / `2d4ad38028e8` | `auv_web_gui` | ament Python; `server` | telemetry/control websocket and ROS topics | GUI integration audit |
| `ping360_image_compensation` | `master` / `79ba71504582` | `ping360_image_compensation` | ament | Ping360 image pipeline | cloned; top-camera placement reference |

The five repositories absent from the desktop source tree were cloned at the
listed heads: `auv_mavlink`, `auv_mavros`, `auv_realsense`,
`auv_vision_nav_control`, and `ping360_image_compensation`.  Existing integrated
core source was preserved because replacing it with a fresh clone would discard
workspace-only simulator and competition changes.

## Contracts confirmed in source

- RC is `OverrideRCIn`: CH3 vertical, CH4 yaw, CH5 forward, CH6 lateral.
- Depth is positive-down after the configured `/depth/pose` conversion.
- Lane and surface use the same `/start_frame`, `ArenaFrameTransform`, and a
  single lane-published `/mission/arena_config` value.
- Lane releases and destroys its RC publisher before `/mission/surface_start`;
  the surface node does the inverse before `/mission/surface_complete`.
- The simulator only releases a collector buoy when a live
  `/mission/score_release` contract and the physical score-zone test both pass.
