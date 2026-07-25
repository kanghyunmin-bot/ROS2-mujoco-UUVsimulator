# Fast Contract Sanity

- root: `/Users/kanghyunmin/Desktop/uuv_sim/uuv_mujoco/v2.2`
- pass: 41
- fail: 0

| Step | Status | Seconds |
| --- | --- | ---: |
| `compileall_core` | PASS | 0.190 |
| `rc_frame_contract` | PASS | 0.074 |
| `ros2_command_payload` | PASS | 0.018 |
| `bridge_live_imports` | PASS | 0.235 |
| `json_servo_receiver` | PASS | 0.029 |
| `sitl_servo_runtime` | PASS | 0.049 |
| `immediate_sensor_replay_reply` | PASS | 0.047 |
| `mujoco_velocity_contract` | PASS | 0.090 |
| `sitl_command_link_readiness` | PASS | 0.027 |
| `sitl_external_nav_contract` | PASS | 0.018 |
| `mavlink_message_interval` | PASS | 0.030 |
| `runtime_readiness_policy` | PASS | 0.025 |
| `gui_backend_selection` | PASS | 0.027 |
| `gui_entry_contract` | PASS | 1.182 |
| `gui_start_contract` | PASS | 0.031 |
| `gui_readiness_contract` | PASS | 0.028 |
| `gui_ros_python_contract` | PASS | 0.553 |
| `gui_arm_mode_command_contract` | PASS | 0.019 |
| `gui_initial_depth_contract` | PASS | 0.022 |
| `initial_depth_auto_release_contract` | PASS | 0.018 |
| `gui_pilot_toggle_contract` | PASS | 0.026 |
| `gui_pilot_auto_enable_contract` | PASS | 0.026 |
| `axis_rc_health_contract` | PASS | 0.021 |
| `axis_rc_latency_metrics` | PASS | 0.021 |
| `mavros_rcout_publish_policy` | PASS | 0.017 |
| `ros2_replay_rcout` | PASS | 0.017 |
| `initial_hold_pose` | PASS | 0.043 |
| `real_start_measurements` | PASS | 0.047 |
| `ros2_dvl_messages` | PASS | 0.044 |
| `static_context_publisher` | PASS | 0.018 |
| `physics_contract_geometry` | PASS | 0.043 |
| `hydrostatic_buoyancy_points` | PASS | 0.061 |
| `model_runtime_setup` | PASS | 0.448 |
| `physics_runtime_hydrostatic` | PASS | 0.056 |
| `thruster_param_loader` | PASS | 0.050 |
| `thruster_performance_curves` | PASS | 0.045 |
| `odometry_publish_builders` | PASS | 0.047 |
| `ros2_ping360_messages` | PASS | 0.051 |
| `ros2_sitl_command_override` | PASS | 0.028 |
| `ping360_stl_io` | PASS | 0.028 |
| `source_contract_audit` | PASS | 0.403 |

Full stdout/stderr tails are in `fast_contract_sanity.json`.
