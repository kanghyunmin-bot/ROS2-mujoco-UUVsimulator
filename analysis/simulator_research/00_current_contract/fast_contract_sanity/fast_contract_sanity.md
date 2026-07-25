# Fast Contract Sanity

- root: `/home/robot/uuv_sim_current/sim/current`
- pass: 71
- fail: 3

| Step | Status | Seconds |
| --- | --- | ---: |
| `compileall_core` | PASS | 0.102 |
| `rc_frame_contract` | PASS | 0.200 |
| `sitl_frame_contract` | PASS | 0.167 |
| `ros2_command_payload` | PASS | 0.032 |
| `bridge_live_imports` | PASS | 0.475 |
| `json_servo_receiver` | PASS | 0.087 |
| `sitl_servo_runtime` | PASS | 0.176 |
| `immediate_sensor_replay_reply` | PASS | 0.175 |
| `mujoco_velocity_contract` | PASS | 0.334 |
| `sim_clock_contract` | PASS | 0.033 |
| `sim_runtime_smooth_contract` | PASS | 0.216 |
| `sitl_command_link_readiness` | PASS | 0.091 |
| `command_latency_contract` | PASS | 0.175 |
| `sitl_external_nav_contract` | PASS | 0.052 |
| `mavlink_message_interval` | PASS | 0.089 |
| `runtime_readiness_policy` | PASS | 0.057 |
| `gui_backend_selection` | PASS | 0.067 |
| `gui_entry_contract` | PASS | 1.749 |
| `gui_start_contract` | PASS | 0.321 |
| `gui_readiness_contract` | PASS | 0.076 |
| `gui_file_persistence` | PASS | 0.130 |
| `process_log_lifecycle` | PASS | 0.042 |
| `async_camera_snapshot_reuse` | PASS | 0.364 |
| `web_gui_contract` | FAIL | 0.148 |
| `gui_rc_fast_loop_contract` | PASS | 0.086 |
| `gui_rc_replay_decode` | PASS | 0.085 |
| `gui_buoy_layout_editor` | PASS | 0.122 |
| `competition_course_scene` | PASS | 0.073 |
| `sim_exit_process_cleanup` | PASS | 0.690 |
| `gui_depth_freshness` | PASS | 0.102 |
| `web_gui_runtime_contracts` | FAIL | 1.149 |
| `rc3_neutral_contract` | PASS | 0.080 |
| `althold_throttle_normalization` | PASS | 0.071 |
| `gui_ros_python_contract` | PASS | 1.137 |
| `gui_arm_mode_command_contract` | PASS | 0.244 |
| `gui_initial_depth_contract` | PASS | 0.050 |
| `initial_depth_auto_release_contract` | PASS | 0.038 |
| `gui_pilot_toggle_contract` | PASS | 0.064 |
| `gui_pilot_auto_enable_contract` | PASS | 0.068 |
| `dist_rc_override_path` | PASS | 0.070 |
| `manual_control_input_scaling` | PASS | 0.176 |
| `axis_rc_health_contract` | PASS | 0.042 |
| `axis_rc_latency_metrics` | PASS | 0.047 |
| `mavros_rcout_publish_policy` | PASS | 0.048 |
| `ros2_replay_rcout` | PASS | 0.052 |
| `initial_hold_pose` | PASS | 0.200 |
| `real_start_measurements` | PASS | 0.184 |
| `ros2_dvl_messages` | PASS | 0.172 |
| `strict_real_pkg_surface` | PASS | 0.364 |
| `static_context_publisher` | PASS | 0.035 |
| `physics_contract_geometry` | PASS | 0.234 |
| `physics_contract_neutral_metrics` | PASS | 0.185 |
| `hydrostatic_buoyancy_points` | PASS | 0.247 |
| `model_binary_cache` | PASS | 0.456 |
| `model_runtime_setup` | PASS | 0.932 |
| `course_buoy_contact_snapshot` | PASS | 0.171 |
| `viewer_pause_publish_contract` | PASS | 0.053 |
| `physics_runtime_hydrostatic` | PASS | 0.186 |
| `sim_profile_safety_contract` | PASS | 0.265 |
| `underwater_flow_contract` | PASS | 1.592 |
| `vehicle_fluid_free_decay` | PASS | 4.744 |
| `thruster_param_loader` | PASS | 0.187 |
| `thruster_performance_curves` | PASS | 0.156 |
| `odometry_publish_builders` | PASS | 0.222 |
| `ros2_ping360_messages` | PASS | 0.229 |
| `hydrophone_audio_timing` | PASS | 0.496 |
| `single_hydrophone_homing_math` | PASS | 0.198 |
| `homing_direction_viewer` | PASS | 0.463 |
| `vision_buoy_pipeline` | PASS | 0.439 |
| `yolo_buoy_overlay_contract` | PASS | 0.213 |
| `rviz_mission_visualizer_contract` | FAIL | 0.122 |
| `ros2_sitl_command_override` | PASS | 0.049 |
| `ping360_stl_io` | PASS | 0.059 |
| `source_contract_audit` | PASS | 0.273 |

Full stdout/stderr tails are in `fast_contract_sanity.json`.
