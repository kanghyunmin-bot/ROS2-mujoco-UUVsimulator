# Base ROV launch compatibility

This package exposes `ros2 launch auv rov_start.launch.py`, matching the
organization's launch entry point. It delegates to the local `hit25_auv_ros2`
integration so existing launch arguments, simulation clocks and physical-device
selection continue to work. The legacy launch remains supported.

This is a launch adapter, not a copy of every executable in the organization's
`auv` repository. Executables remain in `hit25_auv_ros2` in this workspace.

For simulation, select `use_sim_time:=true` and the local SITL `fcu_url`.
The GUI supplies the simulator device addresses and suppresses duplicate
camera/TF/GUI producers through its existing startup configuration.

The simulation MAVROS configuration includes `setpoint_raw`, `vision_pose`
and the organization's DVL `vision_position_delta` adapter. The latter is
provided by `auv_mavros_dvl_plugin` when using the stock ROS MAVROS distribution.
Do not combine that plugin package with a full organization MAVROS fork that
already exports the same plugin.
