# Bundled ROS 2 source packages

The installer builds only the packages needed by the simulator and competition
autonomy stack.  Large upstream repositories are intentionally not copied into
the release; MAVROS is installed from the Ubuntu ROS 2 repository.

Bundled package names:

- `dvl_msgs`, `auv_dvl_a50_msg`, `ping360_sonar_msgs`
- `auv_msg`, `auv`
- `audio_common_msgs`, `audio_common`, `audio_capture`, `hydrophone_ctrl`
- `auv_buoy_vision_control`, `auv_lane_vision_control`
- `auv_web_gui`, `auv_pinger_homing`
- `robot_localization`
