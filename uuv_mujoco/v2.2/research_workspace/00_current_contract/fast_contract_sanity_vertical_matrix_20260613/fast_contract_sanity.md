# Fast Contract Sanity

- root: `/Users/kanghyunmin/Desktop/uuv_sim/uuv_mujoco/v2.2`
- pass: 41
- fail: 0

| Step | Status | Seconds |
| --- | --- | ---: |
| `compileall_core` | PASS | 0.367 |
| `rc_frame_contract` | PASS | 0.128 |
| `ros2_command_payload` | PASS | 0.042 |
| `bridge_live_imports` | PASS | 0.420 |
| `json_servo_receiver` | PASS | 0.056 |
| `sitl_servo_runtime` | PASS | 0.102 |
| `immediate_sensor_replay_reply` | PASS | 0.125 |
| `mujoco_velocity_contract` | PASS | 0.210 |
| `sitl_command_link_readiness` | PASS | 0.058 |
| `sitl_external_nav_contract` | PASS | 0.027 |
| `mavlink_message_interval` | PASS | 0.050 |
| `runtime_readiness_policy` | PASS | 0.041 |
| `gui_backend_selection` | PASS | 0.114 |
| `gui_entry_contract` | PASS | 1.896 |
| `gui_start_contract` | PASS | 0.062 |
| `gui_readiness_contract` | PASS | 0.050 |
| `gui_ros_python_contract` | PASS | 0.915 |
| `gui_arm_mode_command_contract` | PASS | 0.043 |
| `gui_initial_depth_contract` | PASS | 0.045 |
| `initial_depth_auto_release_contract` | PASS | 0.036 |
| `gui_pilot_toggle_contract` | PASS | 0.043 |
| `gui_pilot_auto_enable_contract` | PASS | 0.049 |
| `axis_rc_health_contract` | PASS | 0.043 |
| `axis_rc_latency_metrics` | PASS | 0.040 |
| `mavros_rcout_publish_policy` | PASS | 0.029 |
| `ros2_replay_rcout` | PASS | 0.036 |
| `initial_hold_pose` | PASS | 0.096 |
| `real_start_measurements` | PASS | 0.084 |
| `ros2_dvl_messages` | PASS | 0.076 |
| `static_context_publisher` | PASS | 0.038 |
| `physics_contract_geometry` | PASS | 0.083 |
| `hydrostatic_buoyancy_points` | PASS | 0.116 |
| `model_runtime_setup` | PASS | 0.650 |
| `physics_runtime_hydrostatic` | PASS | 0.098 |
| `thruster_param_loader` | PASS | 0.094 |
| `thruster_performance_curves` | PASS | 0.083 |
| `odometry_publish_builders` | PASS | 0.103 |
| `ros2_ping360_messages` | PASS | 0.113 |
| `ros2_sitl_command_override` | PASS | 0.032 |
| `ping360_stl_io` | PASS | 0.027 |
| `source_contract_audit` | PASS | 0.685 |

Full stdout/stderr tails are in `fast_contract_sanity.json`.
