# Fast Contract Sanity

- root: `/Users/kanghyunmin/Desktop/uuv_sim/uuv_mujoco/v2.2`
- pass: 38
- fail: 0

| Step | Status | Seconds |
| --- | --- | ---: |
| `compileall_core` | PASS | 0.174 |
| `rc_frame_contract` | PASS | 0.025 |
| `ros2_command_payload` | PASS | 0.019 |
| `bridge_live_imports` | PASS | 0.243 |
| `json_servo_receiver` | PASS | 0.030 |
| `sitl_servo_runtime` | PASS | 0.067 |
| `immediate_sensor_replay_reply` | PASS | 0.048 |
| `mujoco_velocity_contract` | PASS | 0.094 |
| `sitl_command_link_readiness` | PASS | 0.029 |
| `sitl_external_nav_contract` | PASS | 0.019 |
| `mavlink_message_interval` | PASS | 0.031 |
| `runtime_readiness_policy` | PASS | 0.021 |
| `gui_backend_selection` | PASS | 0.023 |
| `gui_entry_contract` | PASS | 1.109 |
| `gui_readiness_contract` | PASS | 0.026 |
| `gui_ros_python_contract` | PASS | 0.486 |
| `gui_arm_mode_command_contract` | PASS | 0.019 |
| `gui_initial_depth_contract` | PASS | 0.021 |
| `initial_depth_auto_release_contract` | PASS | 0.017 |
| `gui_pilot_toggle_contract` | PASS | 0.025 |
| `axis_rc_health_contract` | PASS | 0.021 |
| `mavros_rcout_publish_policy` | PASS | 0.017 |
| `ros2_replay_rcout` | PASS | 0.016 |
| `initial_hold_pose` | PASS | 0.042 |
| `real_start_measurements` | PASS | 0.045 |
| `ros2_dvl_messages` | PASS | 0.043 |
| `static_context_publisher` | PASS | 0.017 |
| `physics_contract_geometry` | PASS | 0.041 |
| `hydrostatic_buoyancy_points` | PASS | 0.060 |
| `model_runtime_setup` | PASS | 0.388 |
| `physics_runtime_hydrostatic` | PASS | 0.052 |
| `thruster_param_loader` | PASS | 0.048 |
| `thruster_performance_curves` | PASS | 0.043 |
| `odometry_publish_builders` | PASS | 0.044 |
| `ros2_ping360_messages` | PASS | 0.049 |
| `ros2_sitl_command_override` | PASS | 0.017 |
| `ping360_stl_io` | PASS | 0.021 |
| `source_contract_audit` | PASS | 0.494 |

Full stdout/stderr tails are in `fast_contract_sanity.json`.
