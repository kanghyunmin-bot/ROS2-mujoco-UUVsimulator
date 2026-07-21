# Fast Contract Sanity

- root: `/home/robot/uuv_sim_current/uuv_mujoco/current`
- pass: 44
- fail: 2

| Step | Status | Seconds |
| --- | --- | ---: |
| `compileall_core` | PASS | 0.175 |
| `rc_frame_contract` | PASS | 0.130 |
| `ros2_command_payload` | PASS | 0.028 |
| `bridge_live_imports` | PASS | 0.305 |
| `json_servo_receiver` | FAIL | 0.114 |
| `sitl_servo_runtime` | PASS | 0.110 |
| `immediate_sensor_replay_reply` | PASS | 0.106 |
| `mujoco_velocity_contract` | PASS | 0.227 |
| `sitl_command_link_readiness` | PASS | 0.044 |
| `sitl_external_nav_contract` | PASS | 0.027 |
| `mavlink_message_interval` | PASS | 0.056 |
| `runtime_readiness_policy` | PASS | 0.039 |
| `gui_backend_selection` | PASS | 0.040 |
| `gui_entry_contract` | PASS | 1.445 |
| `gui_start_contract` | PASS | 0.186 |
| `gui_readiness_contract` | PASS | 0.046 |
| `sim_exit_process_cleanup` | PASS | 0.108 |
| `gui_depth_freshness` | PASS | 0.062 |
| `rc3_neutral_contract` | PASS | 0.056 |
| `althold_throttle_normalization` | PASS | 0.055 |
| `gui_ros_python_contract` | PASS | 0.831 |
| `gui_arm_mode_command_contract` | PASS | 0.154 |
| `gui_initial_depth_contract` | PASS | 0.041 |
| `initial_depth_auto_release_contract` | PASS | 0.025 |
| `gui_pilot_toggle_contract` | PASS | 0.050 |
| `gui_pilot_auto_enable_contract` | PASS | 0.051 |
| `axis_rc_health_contract` | PASS | 0.034 |
| `axis_rc_latency_metrics` | PASS | 0.031 |
| `mavros_rcout_publish_policy` | PASS | 0.023 |
| `ros2_replay_rcout` | PASS | 0.027 |
| `initial_hold_pose` | PASS | 0.131 |
| `real_start_measurements` | PASS | 0.101 |
| `ros2_dvl_messages` | PASS | 0.105 |
| `strict_real_pkg_surface` | PASS | 0.208 |
| `static_context_publisher` | PASS | 0.028 |
| `physics_contract_geometry` | PASS | 0.111 |
| `hydrostatic_buoyancy_points` | PASS | 0.121 |
| `model_runtime_setup` | PASS | 1.044 |
| `physics_runtime_hydrostatic` | PASS | 0.126 |
| `thruster_param_loader` | PASS | 0.121 |
| `thruster_performance_curves` | PASS | 0.099 |
| `odometry_publish_builders` | PASS | 0.122 |
| `ros2_ping360_messages` | PASS | 0.150 |
| `ros2_sitl_command_override` | PASS | 0.025 |
| `ping360_stl_io` | PASS | 0.034 |
| `source_contract_audit` | FAIL | 0.507 |

Full stdout/stderr tails are in `fast_contract_sanity.json`.
