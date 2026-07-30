# RC controller recovery

The controller, estimator, visualizer, launch, and RViz files in this package
were recovered from Git tree:

`66a03ac0cbeaf3f0849c55e7ab6028532df70bec`

Recovered source blobs:

- `near_zone_line_search_controller.cpp`: `c20c1b9e6850d82c0bac81b15ae0682c964e922e`
- `region_local_gradient_estimator.cpp`: `5ea72e866283e0e65f8c6a23e589eaa9a190b6e9`
- `region_local_gradient_rviz_visualizer.cpp`: `aab9e56af4ce9676dece8cb95ff19705a6b8ae91`
- `waypoint_homing_controller.cpp`: `0af3fc9f4b3541237bc126527f23acc37c91a933`

The recovered C++ source files were first verified byte-for-byte against these
blobs and built successfully. They were then updated to use the agreed arena
start frame:

`p_start = R(-yaw_start_in_odom) * (p_odom - origin_start_in_odom)`

The start origin and yaw come from the external transient-local
`/start_frame` (`geometry_msgs/msg/PoseStamped`) message. The hydrophone
nodes do not capture their own yaw. Horizontal position, SNR sample positions,
gradients, waypoints, and RViz markers use the start frame. Controller yaw is
`yaw_start = wrap(yaw_odom - yaw_start_in_odom)`, so it is zero only at the
captured initial heading. `arena_offset` locates the pool boundary in the start
frame; it is not added to the odometry transform. Depth remains the original
odometry z value. Simulation and audio nodes remain supplied by
`audio_capture`.
