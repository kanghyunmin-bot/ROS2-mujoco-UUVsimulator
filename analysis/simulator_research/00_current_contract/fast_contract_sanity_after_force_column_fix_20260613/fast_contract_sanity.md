# Fast Contract Sanity

- root: `/Users/kanghyunmin/Desktop/uuv_sim/uuv_mujoco/v2.2`
- pass: 41
- fail: 0

| Step | Status | Seconds |
| --- | --- | ---: |
| `compileall_core` | PASS | 0.189 |
| `rc_frame_contract` | PASS | 0.073 |
| `ros2_command_payload` | PASS | 0.020 |
| `bridge_live_imports` | PASS | 0.244 |
| `json_servo_receiver` | PASS | 0.029 |
| `sitl_servo_runtime` | PASS | 0.051 |
| `immediate_sensor_replay_reply` | PASS | 0.047 |
| `mujoco_velocity_contract` | PASS | 0.093 |
| `sitl_command_link_readiness` | PASS | 0.027 |
| `sitl_external_nav_contract` | PASS | 0.018 |
| `mavlink_message_interval` | PASS | 0.029 |
| `runtime_readiness_policy` | PASS | 0.025 |
| `gui_backend_selection` | PASS | 0.026 |
| `gui_entry_contract` | PASS | 1.194 |
| `gui_start_contract` | PASS | 0.031 |
| `gui_readiness_contract` | PASS | 0.026 |
| `gui_ros_python_contract` | PASS | 0.555 |
| `gui_arm_mode_command_contract` | PASS | 0.019 |
| `gui_initial_depth_contract` | PASS | 0.023 |
| `initial_depth_auto_release_contract` | PASS | 0.018 |
| `gui_pilot_toggle_contract` | PASS | 0.026 |
| `gui_pilot_auto_enable_contract` | PASS | 0.027 |
| `axis_rc_health_contract` | PASS | 0.021 |
| `axis_rc_latency_metrics` | PASS | 0.020 |
| `mavros_rcout_publish_policy` | PASS | 0.017 |
| `ros2_replay_rcout` | PASS | 0.017 |
| `initial_hold_pose` | PASS | 0.043 |
| `real_start_measurements` | PASS | 0.046 |
| `ros2_dvl_messages` | PASS | 0.044 |
| `static_context_publisher` | PASS | 0.018 |
| `physics_contract_geometry` | PASS | 0.042 |
| `hydrostatic_buoyancy_points` | PASS | 0.060 |
| `model_runtime_setup` | PASS | 0.443 |
| `physics_runtime_hydrostatic` | PASS | 0.053 |
| `thruster_param_loader` | PASS | 0.049 |
| `thruster_performance_curves` | PASS | 0.045 |
| `odometry_publish_builders` | PASS | 0.046 |
| `ros2_ping360_messages` | PASS | 0.050 |
| `ros2_sitl_command_override` | PASS | 0.017 |
| `ping360_stl_io` | PASS | 0.021 |
| `source_contract_audit` | PASS | 0.381 |

Full stdout/stderr tails are in `fast_contract_sanity.json`.
