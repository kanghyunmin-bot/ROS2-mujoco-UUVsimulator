# Code-Level Contract Source Audit

This report compares official-facing contracts with the local ArduSub 4.1.2 and active MuJoCo runtime source paths.

## Metadata

- repo_root: `/Users/kanghyunmin/Desktop/uuv_sim`
- active_runtime_root: `/Users/kanghyunmin/Desktop/uuv_sim/uuv_mujoco/current`
- compat_v22_root: `/Users/kanghyunmin/Desktop/uuv_sim/uuv_mujoco/v2.2`
- ardupilot_describe: `ArduSub-4.1.2`
- ardupilot_status_short: `<clean>`

## Summary

| status | check | conclusion |
| --- | --- | --- |
| PASS | `active_runtime_alias_current` | Live launch, GUI, Docker, setup, and validation paths must resolve the active runtime through uuv_mujoco/current. The v2.2 directory name is only the compatibility backing directory until a physical rename is done. |
| PASS | `ardupilot_source_tag` | Local ArduPilot reports 'ArduSub-4.1.2'; inner worktree status is clean. Use a fresh clone only if this changes. |
| WARN | `top_level_ardupilot_gitlink` | The ArduPilot source checkout used for this audit is 2dd0bb7d4c85ac48437f139d66df648fc0e1d4ae. The top-level repository records 6271e15bd4a2ef81a0f4ead439cb18188763c271. If these differ, do not commit the gitlink change unless the project intentionally updates the submodule pointer. |
| PASS | `json_servo_packet_16_raw_pwm` | Closed-loop plant input must consume the raw JSON servo packet as PWM[0..15]. |
| PASS | `json_sensor_no_direct_pressure_or_altitude` | Bar30 cannot be injected by a JSON pressure field in this firmware. The pressure contract must be implemented indirectly through position.z. |
| PASS | `baro_sitl_pressure_from_json_position_z` | For ArduSub, AP_Baro_SITL registers a water barometer and converts SITL altitude to underwater pressure. SIM_Aircraft builds that altitude from JSON position.z. |
| PASS | `servo_output_raw_halrcout_telemetry` | Controller parity comparison layer is real /mavros/rc/out versus SITL MAVLink SERVO_OUTPUT_RAW, not the high-rate JSON servo backend. |
| WARN | `rc_override_local_16_channel_limit` | The official MAVLink message has extension channels beyond 16, but this local ArduSub 4.1.2 handler builds override_data only through chan16_raw. This is acceptable for the current vehicle if active controls stay within C1..C8, but the old 'preserve 1..18' checklist is not true for this firmware. |
| PASS | `rc_override_timeout_policy` | RC override must be streamed faster than RC_OVERRIDE_TIME, whose local default is 3s. A one-shot override is not a valid closed-loop input contract. |
| PASS | `ardusub_joystick_axis_mapping` | The real joy/RC override axis mapping checklist is confirmed in ArduSub code. |
| PASS | `active_runtime_baro_contract_frontend_match` | The active runtime converts physical Bar30 pressure/depth into the JSON position.z value that makes AP_Baro_SITL produce the matching water-barometer frontend altitude. |
| WARN | `active_runtime_json_altitude_field_is_compat_only` | The altitude key in the active runtime JSON payload is compatibility/debug data for this firmware. It must not be treated as a controller input contract. |
| PASS | `active_runtime_static_pressure_external_bar30` | The ROS output surface uses Bar30 absolute pressure for static_pressure by default. For SITL control, the important path remains JSON position.z -> AP_Baro_SITL. |
| WARN | `active_runtime_atm_pressure_excluded_output_surface` | The April 1 real bag has /mavros/imu/atm_pressure around 0.24, not Pa-scale surface pressure. The sim currently publishes Bar30 absolute pressure there, so this topic is excluded from controller parity and plant replay fitting until its real semantics are proven. |
| PASS | `active_runtime_time_contract_sim_publish_wall_transport` | ROS sensor output is gated by monotonic sim time and sensor_dt; sensor-replay can use a monotonic JSON-servo frame clock; MAVLink polling, SERVO_OUTPUT_RAW requests, RC keepalive, and viewer catch-up use wall-clock cadence. |
| PASS | `active_runtime_sensor_io_snapshot_contract` | The active runtime builds one MuJoCo sensor snapshot, sends FRD IMU plus Bar30 pressure/depth to ArduSub JSON SITL, and publishes ROS/MAVROS/DVL observation topics from the same snapshot. |
| PASS | `active_runtime_rc_override_forward_mirror_contract` | The active runtime forwards RC_CHANNELS_OVERRIDE as the first 18 raw channels, uses normalized axes only for local fallback, and mirrors the same raw frame to /mavros/rc/in. |
| PASS | `active_runtime_plant_input_raw_pwm_contract` | Plant replay owns actuator input only for replay_rcout sources; otherwise raw SITL JSON SERVO PWM is passed through the plant-input handler before physical thruster force conversion. |
| PASS | `active_runtime_dynamic_fluidcoef_contract` | Dynamic MuJoCo fluidcoef is profile/env opt-in, restricted to current-mode fluid geoms, configured from pattern-specific five-coefficient reference rows, and updates the five MuJoCo ellipsoid coefficients from geom-local velocity and angular-rate loads. The accepted clean baseline keeps the dynamic path disabled until a validated HAN/CFD profile enables it. |
| PASS | `active_runtime_integrated_flow_contract` | Each tick spins ROS/transport first, applies raw PWM or direct command targets, updates thruster force on the thruster cadence, updates dynamic fluidcoef inside the underwater wrench phase, then advances MuJoCo. This prevents sensor publishing, RC input, actuator conversion, and dynamic ellipsoid tuning from silently owning the same state surface. |
| PASS | `thruster_contract_final_pwm_not_mot_direction_again` | ArduSub motor factors and MOT_x_DIRECTION are controller-side. The active runtime maps final PWM into MuJoCo actuator-positive force using only physical mount orientation. |
| PASS | `active_runtime_thruster_conversion_contract` | The active runtime schedules thruster updates in sim time, then converts raw normalized targets through one first-order actuator state, one T200/direct or polynomial force model, and one water-immersion scale before writing MuJoCo actuator ctrl. |
| WARN | `plant_replay_gate_safe_targets` | Use exact RCOU input, Bar30, IMU, DVL x/y, and gyro targets before HAN/CFD tuning. DVL z and local_position-style estimator surfaces are not safe primary targets yet. |
| WARN | `source_contract_matrix_gate` | The source audit includes every required contract axis before controller parity or plant replay tuning. WARN means a known firmware or replay-gate limitation is documented; FAIL means a contract axis is missing or broken. |

## Evidence

### PASS: Active runtime resolves through uuv_mujoco/current

- id: `active_runtime_alias_current`
- conclusion: Live launch, GUI, Docker, setup, and validation paths must resolve the active runtime through uuv_mujoco/current. The v2.2 directory name is only the compatibility backing directory until a physical rename is done.
- local evidence:
  - `uuv_mujoco/current` - uuv_mujoco/current -> v2.2
  - `uuv_mujoco/current/run_uuv_mujoco.py` - primary runner exists: True
  - `uuv_mujoco/run_mujoco.sh` - root launcher exists/executable: True
  - `uuv_mujoco/start_sitl_mujoco.sh` - root launcher exists/executable: True
  - `uuv_mujoco/start_docker_sitl_mujoco.sh` - root launcher exists/executable: True
  - `uuv_mujoco/reset_sim.sh` - root launcher exists/executable: True
  - `uuv_mujoco/RUNTIME_VERSION.json` - runtime version: {"active_runtime": "uuv_mujoco/current", "active_runtime_label": "current-2026-06-12-uuv_sim", "backing_directory": "uuv_mujoco/v2.2", "compatibility_runner": "uuv_mujoco/current/run_urdf_full.py", "dirty_state": {"active_runtime_dirty_count": 1182, "active_runtime_dirty_sample": ["uuv_mujoco/v2.2/bridge/__init__.py", "uuv_mujoco/v2.2/bridge/ping360_sim.py", "uuv_mujoco/v2.2/bridge/qgc_video_stream.py", "uuv_mujoco/v2.2/bridge/ros2_bridge.py", "uuv_mujoco/v2.2/bridge/ros2_bridge_runtime.py", "uuv_mujoco/v2.2/bridge/ros2_topic_registry.py", "uuv_mujoco/v2.2/bridge/sitl_transport.py", "uuv_mujoco/v2.2/config/ping360.json", "uuv_mujoco/v2.2/config/sim_profiles.json", "uuv_mujoco/v2.2/config/thruster_params.json", "uuv_mujoco/v2.2/gui/app.py", "uuv_mujoco/v2.2/gui/autotune_mixin.py", "uuv_mujoco/v2.2/gui/config.py", "uuv_mujoco/v2.2/gui/control_display_mixin.py", "uuv_mujoco/v2.2/gui/helpers.py", "uuv_mujoco/v2.2/gui/layout_mixin.py", "uuv_mujoco/v2.2/gui/models.py", "uuv_mujoco/v2.2/gui/node.py", "uuv_mujoco/v2.2/gui/physics_mixin.py", "uuv_mujoco/v2.2/gui/replay_mixin.py"], "ardupilot_submodule_status": "-6271e15bd4a2ef81a0f4ead439cb18188763c271 ardupilot", "working_tree_dirty_count": 1200, "working_tree_dirty_sample": [".uuv_mujoco_env.sh", "README.md", "ardupilot", "cleanup_generated_artifacts.sh", "docker/ardusub/README.md", "docker/ardusub/entrypoint.sh", "run_control_gui.sh", "run_control_gui_ubuntu.sh", "setup/03_setup_uuv_mujoco.sh", "setup/04_verify_uuv_stack.sh", "setup/install_uuv_mujoco.sh", "uuv_control_gui.py", "uuv_mujoco/v2.2/bridge/__init__.py", "uuv_mujoco/v2.2/bridge/ping360_sim.py", "uuv_mujoco/v2.2/bridge/qgc_video_stream.py", "uuv_mujoco/v2.2/bridge/ros2_bridge.py", "uuv_mujoco/v2.2/bridge/ros2_bridge_runtime.py", "uuv_mujoco/v2.2/bridge/ros2_topic_registry.py", "uuv_mujoco/v2.2/bridge/sitl_transport.py", "uuv_mujoco/v2.2/config/ping360.json"]}, "freshness_policy": "origin/uuv_sim is the source branch for this active runtime; origin/main and origin/master are different layout branches and must not be merged blindly into the dirty simulator workspace.", "freshness_status": "warn", "note": "v2.2 is a compatibility directory name. New launch, GUI, setup, and validation paths must resolve through uuv_mujoco/current.", "origin_uuv_sim_head": "e60cd9383c214a7d6e4700cdeb5c2f2a8cf6d328", "primary_runner": "uuv_mujoco/current/run_uuv_mujoco.py", "root_launchers": {"docker_sitl_mujoco": "uuv_mujoco/start_docker_sitl_mujoco.sh", "mujoco": "uuv_mujoco/run_mujoco.sh", "reset": "uuv_mujoco/reset_sim.sh", "sitl_mujoco": "uuv_mujoco/start_sitl_mujoco.sh"}, "schema": 1, "source_audit_check": "active_runtime_alias_current", "source_branch": "uuv_sim", "source_head": "e60cd9383c214a7d6e4700cdeb5c2f2a8cf6d328", "source_remote": "https://github.com/kanghyunmin-bot/ROS2-mujoco-UUVsimulator.git", "status": "current-dirty", "updated_at": "2026-06-12"}
  - `uuv_mujoco/current/tools/check_runtime_freshness.py` - freshness checker exists: True
  - `uuv_mujoco/*.sh` - root launchers call freshness checker: True
  - `.` - active git branch: uuv_sim
  - `.` - active git HEAD: e60cd9383c214a7d6e4700cdeb5c2f2a8cf6d328
  - `.` - origin/uuv_sim HEAD: e60cd9383c214a7d6e4700cdeb5c2f2a8cf6d328
  - `.` - active runtime dirty paths: M .uuv_mujoco_env.sh
 M README.md
 M docker/ardusub/README.md
 M docker/ardusub/entrypoint.sh
 M setup/03_setup_uuv_mujoco.sh
 M setup/04_verify_uuv_stack.sh
 M setup/install_uuv_mujoco.sh
 M uuv_control_gui.py
 M uuv_mujoco/v2.2/bridge/__init__.py
 M uuv_mujoco/v2.2/bridge/ping360_sim.py
 M uuv_mujoco/v2.2/bridge/qgc_video_stream.py
 M uuv_mujoco/v2.2/bridge/ros2_bridge.py
 M uuv_mujoco/v2.2/bridge/ros2_bridge_runtime.py
 M uuv_mujoco/v2.2/bridge/ros2_topic_registry.py
 M uuv_mujoco/v2.2/bridge/sitl_transport.py
 M uuv_mujoco/v2.2/config/ping360.json
 M uuv_mujoco/v2.2/config/sim_profiles.json
 M uuv_mujoco/v2.2/config/thruster_params.json
 M uuv_mujoco/v2.2/gui/app.py
 D uuv_mujoco/v2.2/gui/autotune_mixin.py
 M uuv_mujoco/v2.2/gui/config.py
 M uuv_mujoco/v2.2/gui/control_display_mixin.py
 M uuv_mujoco/v2.2/gui/helpers.py
 M uuv_mujoco/v2.2/gui/layout_mixin.py
 M uuv_mujoco/v2.2/gui/models.py
 M uuv_mujoco/v2.2/gui/node.py
 M uuv_mujoco/v2.2/gui/physics_mixin.py
 M uuv_mujoco/v2.2/gui/replay_mixin.py
 M uuv_mujoco/v2.2/gui/ros_process_mixin.py
 M uuv_mujoco/v2.2/gui/ros_tools.py
 M uuv_mujoco/v2.2/gui/runtime.py
 M uuv_mujoco/v2.2/gui/uuv_control_gui.py
 M uuv_mujoco/v2.2/gui/widgets.py
 M uuv_mujoco/v2.2/launch_uuv_sim.sh
 M uuv_mujoco/v2.2/physics/__init__.py
 M uuv_mujoco/v2.2/physics/hydrodynamics_helpers.py
 M uuv_mujoco/v2.2/physics/sim_profile_helpers.py
 M uuv_mujoco/v2.2/physics/thruster_mapping.py
 M uuv_mujoco/v2.2/physics/thruster_performance.py
 M uuv_mujoco/v2.2/reset_uuv_sim.sh
 M uuv_mujoco/v2.2/run_control_gui.sh
 M uuv_mujoco/v2.2/run_control_loop_golden_check.sh
 M uuv_mujoco/v2.2/run_urdf_full.py
 M uuv_mujoco/v2.2/scenes/tank_current_scene.xml
 M uuv_mujoco/v2.2/scenes/tank_legacy_scene.xml
 M uuv_mujoco/v2.2/start_ardusub_sitl_mj311.sh
 M uuv_mujoco/v2.2/start_docker_ardusub_sitl.sh
 M uuv_mujoco/v2.2/start_docker_sitl_mujoco_mj311.sh
 M uuv_mujoco/v2.2/start_sitl_mujoco_mj311.sh
 M uuv_mujoco/v2.2/tools/althold_diagnostics_logger.py
 M uuv_mujoco/v2.2/tools/analyze_althold_contract.py
 M uuv_mujoco/v2.2/tools/audit_closed_loop_contract.py
 M uuv_mujoco/v2.2/tools/axis_rc_override_check.py
 M uuv_mujoco/v2.2/tools/control_loop_golden_compare.py
 M uuv_mujoco/v2.2/tools/filter_ping360_stl.py
 M uuv_mujoco/v2.2/tools/preflight_ardupilot_integrity.py
 M uuv_mujoco/v2.2/tools/roll_stability_sweep.py
 M uuv_mujoco/v2.2/tools/verify_ardusub_thruster_contract.py
?? uuv_mujoco/CURRENT.md
?? uuv_mujoco/RUNTIME_VERSION.json
?? uuv_mujoco/current
?? uuv_mujoco/reset_sim.sh
?? uuv_mujoco/run_mujoco.sh
?? uuv_mujoco/start_docker_sitl_mujoco.sh
?? uuv_mujoco/start_sitl_mujoco.sh
?? uuv_mujoco/v2.2/bridge/ping360_angle_math.py
?? uuv_mujoco/v2.2/bridge/ping360_angle_settings.py
?? uuv_mujoco/v2.2/bridge/ping360_beam_directions.py
?? uuv_mujoco/v2.2/bridge/ping360_beam_model.py
?? uuv_mujoco/v2.2/bridge/ping360_config.py
?? uuv_mujoco/v2.2/bridge/ping360_constants.py
?? uuv_mujoco/v2.2/bridge/ping360_effective_settings.py
?? uuv_mujoco/v2.2/bridge/ping360_history.py
?? uuv_mujoco/v2.2/bridge/ping360_image_layers.py
?? uuv_mujoco/v2.2/bridge/ping360_image_lookup.py
?? uuv_mujoco/v2.2/bridge/ping360_image_renderer.py
?? uuv_mujoco/v2.2/bridge/ping360_interface_timing.py
?? uuv_mujoco/v2.2/bridge/ping360_model_ids.py
?? uuv_mujoco/v2.2/bridge/ping360_profile.py
?? uuv_mujoco/v2.2/bridge/ping360_profile_hits.py
?? uuv_mujoco/v2.2/bridge/ping360_profile_noise.py
?? uuv_mujoco/v2.2/bridge/ping360_profile_signal.py
?? uuv_mujoco/v2.2/bridge/ping360_range_math.py
?? uuv_mujoco/v2.2/bridge/ping360_range_quality.py
?? uuv_mujoco/v2.2/bridge/ping360_range_settings.py
?? uuv_mujoco/v2.2/bridge/ping360_raycast.py
?? uuv_mujoco/v2.2/bridge/ping360_reflectivity.py
?? uuv_mujoco/v2.2/bridge/ping360_runtime_state.py
?? uuv_mujoco/v2.2/bridge/ping360_sample.py
?? uuv_mujoco/v2.2/bridge/ping360_samples.py
?? uuv_mujoco/v2.2/bridge/ping360_settings.py
?? uuv_mujoco/v2.2/bridge/ping360_sim_lifecycle.py
?? uuv_mujoco/v2.2/bridge/ping360_sim_status.py
?? uuv_mujoco/v2.2/bridge/ping360_sim_update.py
?? uuv_mujoco/v2.2/bridge/ping360_sweep.py
?? uuv_mujoco/v2.2/bridge/ping360_transmit_settings.py
?? uuv_mujoco/v2.2/bridge/ping360_types.py
?? uuv_mujoco/v2.2/bridge/ping360_update_cycle.py
?? uuv_mujoco/v2.2/bridge/qgc_video_ffmpeg.py
?? uuv_mujoco/v2.2/bridge/qgc_video_ffmpeg_cmd.py
?? uuv_mujoco/v2.2/bridge/qgc_video_ffmpeg_probe.py
?? uuv_mujoco/v2.2/bridge/qgc_video_ffmpeg_process.py
?? uuv_mujoco/v2.2/bridge/qgc_video_stream_config.py
?? uuv_mujoco/v2.2/bridge/qgc_video_stream_lifecycle.py
?? uuv_mujoco/v2.2/bridge/qgc_video_stream_write.py
?? uuv_mujoco/v2.2/bridge/ros2_battery_messages.py
?? uuv_mujoco/v2.2/bridge/ros2_bridge_cmd_timeout.py
?? uuv_mujoco/v2.2/bridge/ros2_bridge_commands.py
?? uuv_mujoco/v2.2/bridge/ros2_bridge_config.py
?? uuv_mujoco/v2.2/bridge/ros2_bridge_config_baro.py
?? uuv_mujoco/v2.2/bridge/ros2_bridge_config_frames.py
?? uuv_mujoco/v2.2/bridge/ros2_bridge_config_imu.py
?? uuv_mujoco/v2.2/bridge/ros2_bridge_config_imu_ros_surface.py
?? uuv_mujoco/v2.2/bridge/ros2_bridge_config_imu_sitl.py
?? uuv_mujoco/v2.2/bridge/ros2_bridge_config_imu_source.py
?? uuv_mujoco/v2.2/bridge/ros2_bridge_config_mavros.py
?? uuv_mujoco/v2.2/bridge/ros2_bridge_config_mavros_battery.py
?? uuv_mujoco/v2.2/bridge/ros2_bridge_config_mavros_rates.py
?? uuv_mujoco/v2.2/bridge/ros2_bridge_config_mavros_rcout.py
?? uuv_mujoco/v2.2/bridge/ros2_bridge_config_mavros_replay.py
?? uuv_mujoco/v2.2/bridge/ros2_bridge_config_mavros_sensor_rates.py
?? uuv_mujoco/v2.2/bridge/ros2_bridge_config_mavros_state.py
?? uuv_mujoco/v2.2/bridge/ros2_bridge_config_pressure.py
?? uuv_mujoco/v2.2/bridge/ros2_bridge_config_static_pressure.py
?? uuv_mujoco/v2.2/bridge/ros2_bridge_config_vertical.py
?? uuv_mujoco/v2.2/bridge/ros2_bridge_constructor.py
?? uuv_mujoco/v2.2/bridge/ros2_bridge_context_runtime.py
?? uuv_mujoco/v2.2/bridge/ros2_bridge_import_core.py
?? uuv_mujoco/v2.2/bridge/ros2_bridge_import_messages.py
?? uuv_mujoco/v2.2/bridge/ros2_bridge_imports.py
?? uuv_mujoco/v2.2/bridge/ros2_bridge_init.py
?? uuv_mujoco/v2.2/bridge/ros2_bridge_message_bindings.py
?? uuv_mujoco/v2.2/bridge/ros2_bridge_method_bindings.py
?? uuv_mujoco/v2.2/bridge/ros2_bridge_public_api.py
?? uuv_mujoco/v2.2/bridge/ros2_bridge_publish.py
?? uuv_mujoco/v2.2/bridge/ros2_bridge_publish_ros.py
?? uuv_mujoco/v2.2/bridge/ros2_bridge_publish_stamp.py
?? uuv_mujoco/v2.2/bridge/ros2_bridge_publish_timing.py
?? uuv_mujoco/v2.2/bridge/ros2_bridge_runtime_methods.py
?? uuv_mujoco/v2.2/bridge/ros2_bridge_runtime_setup.py
?? uuv_mujoco/v2.2/bridge/ros2_bridge_sensor_setup.py
?? uuv_mujoco/v2.2/bridge/ros2_bridge_servo_api.py
?? uuv_mujoco/v2.2/bridge/ros2_bridge_shutdown.py
?? uuv_mujoco/v2.2/bridge/ros2_bridge_shutdown_guard.py
?? uuv_mujoco/v2.2/bridge/ros2_bridge_shutdown_ros.py
?? uuv_mujoco/v2.2/bridge/ros2_bridge_shutdown_sitl.py
?? uuv_mujoco/v2.2/bridge/ros2_bridge_shutdown_steps.py
?? uuv_mujoco/v2.2/bridge/ros2_bridge_shutdown_thread.py
?? uuv_mujoco/v2.2/bridge/ros2_bridge_sitl_poll.py
?? uuv_mujoco/v2.2/bridge/ros2_bridge_spin_executor.py
?? uuv_mujoco/v2.2/bridge/ros2_bridge_spin_once.py
?? uuv_mujoco/v2.2/bridge/ros2_bridge_spin_publish.py
?? uuv_mujoco/v2.2/bridge/ros2_bridge_startup_log.py
?? uuv_mujoco/v2.2/bridge/ros2_bridge_static_context.py
?? uuv_mujoco/v2.2/bridge/ros2_cmd_vel_input.py
?? uuv_mujoco/v2.2/bridge/ros2_command_bool.py
?? uuv_mujoco/v2.2/bridge/ros2_command_payload.py
?? uuv_mujoco/v2.2/bridge/ros2_command_payload_tokens.py
?? uuv_mujoco/v2.2/bridge/ros2_command_shaping.py
?? uuv_mujoco/v2.2/bridge/ros2_direct_command_filter.py
?? uuv_mujoco/v2.2/bridge/ros2_direct_command_guard.py
?? uuv_mujoco/v2.2/bridge/ros2_direct_command_slew.py
?? uuv_mujoco/v2.2/bridge/ros2_dvl_header.py
?? uuv_mujoco/v2.2/bridge/ros2_dvl_messages.py
?? uuv_mujoco/v2.2/bridge/ros2_dvl_pose_msg.py
?? uuv_mujoco/v2.2/bridge/ros2_dvl_velocity_msg.py
?? uuv_mujoco/v2.2/bridge/ros2_endpoint_core_publishers.py
?? uuv_mujoco/v2.2/bridge/ros2_endpoint_mavros_publishers.py
?? uuv_mujoco/v2.2/bridge/ros2_endpoint_misc_publishers.py
?? uuv_mujoco/v2.2/bridge/ros2_endpoint_ping360_publishers.py
?? uuv_mujoco/v2.2/bridge/ros2_endpoint_publishers.py
?? uuv_mujoco/v2.2/bridge/ros2_endpoint_services.py
?? uuv_mujoco/v2.2/bridge/ros2_endpoint_subscriptions.py
?? uuv_mujoco/v2.2/bridge/ros2_endpoints.py
?? uuv_mujoco/v2.2/bridge/ros2_imu_messages.py
?? uuv_mujoco/v2.2/bridge/ros2_manual_control_input.py
?? uuv_mujoco/v2.2/bridge/ros2_math.py
?? uuv_mujoco/v2.2/bridge/ros2_math_pressure.py
?? uuv_mujoco/v2.2/bridge/ros2_math_quat.py
?? uuv_mujoco/v2.2/bridge/ros2_math_rc.py
?? uuv_mujoco/v2.2/bridge/ros2_math_rotation.py
?? uuv_mujoco/v2.2/bridge/ros2_math_scalar.py
?? uuv_mujoco/v2.2/bridge/ros2_mavros_arm_mode_callbacks.py
?? uuv_mujoco/v2.2/bridge/ros2_mavros_arm_mode_forwarding.py
?? uuv_mujoco/v2.2/bridge/ros2_mavros_arm_mode_guard.py
?? uuv_mujoco/v2.2/bridge/ros2_mavros_arm_mode_services.py
?? uuv_mujoco/v2.2/bridge/ros2_mavros_arm_mode_transport.py
?? uuv_mujoco/v2.2/bridge/ros2_mavros_command_services.py
?? uuv_mujoco/v2.2/bridge/ros2_mavros_condition_yaw.py
?? uuv_mujoco/v2.2/bridge/ros2_mavros_setpoint_position.py
?? uuv_mujoco/v2.2/bridge/ros2_mavros_setpoint_services.py
?? uuv_mujoco/v2.2/bridge/ros2_message_setters.py
?? uuv_mujoco/v2.2/bridge/ros2_mujoco_model.py
?? uuv_mujoco/v2.2/bridge/ros2_optional_message.py
?? uuv_mujoco/v2.2/bridge/ros2_ping360_config.py
?? uuv_mujoco/v2.2/bridge/ros2_ping360_echo_message.py
?? uuv_mujoco/v2.2/bridge/ros2_ping360_image_message.py
?? uuv_mujoco/v2.2/bridge/ros2_ping360_messages.py
?? uuv_mujoco/v2.2/bridge/ros2_ping360_scan_message.py
?? uuv_mujoco/v2.2/bridge/ros2_ping360_status_message.py
?? uuv_mujoco/v2.2/bridge/ros2_pose_messages.py
?? uuv_mujoco/v2.2/bridge/ros2_pressure_messages.py
?? uuv_mujoco/v2.2/bridge/ros2_publish_builder_core.py
?? uuv_mujoco/v2.2/bridge/ros2_publish_builder_dvl.py
?? uuv_mujoco/v2.2/bridge/ros2_publish_builder_mavros.py
?? uuv_mujoco/v2.2/bridge/ros2_publish_builder_odometry.py
?? uuv_mujoco/v2.2/bridge/ros2_publish_builder_ping360.py
?? uuv_mujoco/v2.2/bridge/ros2_publish_builder_status.py
?? uuv_mujoco/v2.2/bridge/ros2_publish_builders.py
?? uuv_mujoco/v2.2/bridge/ros2_publish_core_cache.py
?? uuv_mujoco/v2.2/bridge/ros2_publish_core_factories.py
?? uuv_mujoco/v2.2/bridge/ros2_publish_dvl_cache.py
?? uuv_mujoco/v2.2/bridge/ros2_publish_dvl_factories.py
?? uuv_mujoco/v2.2/bridge/ros2_publish_mavros_cache.py
?? uuv_mujoco/v2.2/bridge/ros2_publish_mavros_cache_factories.py
?? uuv_mujoco/v2.2/bridge/ros2_publish_mavros_cache_imu.py
?? uuv_mujoco/v2.2/bridge/ros2_publish_mavros_cache_local.py
?? uuv_mujoco/v2.2/bridge/ros2_publish_mavros_cache_status.py
?? uuv_mujoco/v2.2/bridge/ros2_publish_odometry_messages.py
?? uuv_mujoco/v2.2/bridge/ros2_publish_odometry_tf.py
?? uuv_mujoco/v2.2/bridge/ros2_publish_ping360_cache.py
?? uuv_mujoco/v2.2/bridge/ros2_publish_ping360_cache_state.py
?? uuv_mujoco/v2.2/bridge/ros2_publish_ping360_message_cache.py
?? uuv_mujoco/v2.2/bridge/ros2_publish_ping360_sample_cache.py
?? uuv_mujoco/v2.2/bridge/ros2_publish_ping360_status_cache.py
?? uuv_mujoco/v2.2/bridge/ros2_publish_queue.py
?? uuv_mujoco/v2.2/bridge/ros2_publish_runtime.py
?? uuv_mujoco/v2.2/bridge/ros2_publish_schedule.py
?? uuv_mujoco/v2.2/bridge/ros2_publish_schedule_core.py
?? uuv_mujoco/v2.2/bridge/ros2_publish_schedule_dvl.py
?? uuv_mujoco/v2.2/bridge/ros2_publish_schedule_mavros.py
?? uuv_mujoco/v2.2/bridge/ros2_publish_schedule_odometry.py
?? uuv_mujoco/v2.2/bridge/ros2_publish_schedule_ping360.py
?? uuv_mujoco/v2.2/bridge/ros2_publish_state.py
?? uuv_mujoco/v2.2/bridge/ros2_publisher_demand.py
?? uuv_mujoco/v2.2/bridge/ros2_publisher_demand_probe.py
?? uuv_mujoco/v2.2/bridge/ros2_publisher_demand_state.py
?? uuv_mujoco/v2.2/bridge/ros2_range_messages.py
?? uuv_mujoco/v2.2/bridge/ros2_rc_output_commands.py
?? uuv_mujoco/v2.2/bridge/ros2_rc_override_callback.py
?? uuv_mujoco/v2.2/bridge/ros2_rc_override_forward_cache.py
?? uuv_mujoco/v2.2/bridge/ros2_rc_override_forwarding.py
?? uuv_mujoco/v2.2/bridge/ros2_rc_override_frame.py
?? uuv_mujoco/v2.2/bridge/ros2_rc_override_input.py
?? uuv_mujoco/v2.2/bridge/ros2_rc_override_mirror.py
?? uuv_mujoco/v2.2/bridge/ros2_rc_override_warning.py
?? uuv_mujoco/v2.2/bridge/ros2_rcout_message.py
?? uuv_mujoco/v2.2/bridge/ros2_rcout_publish.py
?? uuv_mujoco/v2.2/bridge/ros2_rcout_stamp.py
?? uuv_mujoco/v2.2/bridge/ros2_rcout_telemetry.py
?? uuv_mujoco/v2.2/bridge/ros2_replay_rcout.py
?? uuv_mujoco/v2.2/bridge/ros2_replay_rcout_channels.py
?? uuv_mujoco/v2.2/bridge/ros2_replay_rcout_inject.py
?? uuv_mujoco/v2.2/bridge/ros2_replay_rcout_log.py
?? uuv_mujoco/v2.2/bridge/ros2_runtime_context_errors.py
?? uuv_mujoco/v2.2/bridge/ros2_runtime_env.py
?? uuv_mujoco/v2.2/bridge/ros2_runtime_mavros_state.py
?? uuv_mujoco/v2.2/bridge/ros2_runtime_mavros_state_header.py
?? uuv_mujoco/v2.2/bridge/ros2_runtime_mavros_state_values.py
?? uuv_mujoco/v2.2/bridge/ros2_runtime_safe_publish.py
?? uuv_mujoco/v2.2/bridge/ros2_runtime_spin.py
?? uuv_mujoco/v2.2/bridge/ros2_runtime_spin_loop.py
?? uuv_mujoco/v2.2/bridge/ros2_runtime_spin_state.py
?? uuv_mujoco/v2.2/bridge/ros2_runtime_spin_thread.py
?? uuv_mujoco/v2.2/bridge/ros2_runtime_static_context.py
?? uuv_mujoco/v2.2/bridge/ros2_sensor_messages.py
?? uuv_mujoco/v2.2/bridge/ros2_sitl_command_override.py
?? uuv_mujoco/v2.2/bridge/ros2_sitl_command_override_arm_mode.py
?? uuv_mujoco/v2.2/bridge/ros2_sitl_command_override_replay.py
?? uuv_mujoco/v2.2/bridge/ros2_sitl_command_override_sensor.py
?? uuv_mujoco/v2.2/bridge/ros2_sitl_command_override_topic.py
?? uuv_mujoco/v2.2/bridge/ros2_sitl_sensor_altitude.py
?? uuv_mujoco/v2.2/bridge/ros2_sitl_sensor_dvl_state.py
?? uuv_mujoco/v2.2/bridge/ros2_sitl_sensor_feed.py
?? uuv_mujoco/v2.2/bridge/ros2_sitl_sensor_imu_state.py
?? uuv_mujoco/v2.2/bridge/ros2_sitl_sensor_kinematics.py
?? uuv_mujoco/v2.2/bridge/ros2_sitl_sensor_transport.py
?? uuv_mujoco/v2.2/bridge/ros2_sitl_sensor_types.py
?? uuv_mujoco/v2.2/bridge/ros2_sitl_sensor_vectors.py
?? uuv_mujoco/v2.2/bridge/ros2_sitl_sensor_vertical.py
?? uuv_mujoco/v2.2/bridge/ros2_sitl_sensor_vertical_hold.py
?? uuv_mujoco/v2.2/bridge/ros2_sitl_transport_setup.py
?? uuv_mujoco/v2.2/bridge/ros2_standard_messages.py
?? uuv_mujoco/v2.2/bridge/ros2_state_body_velocity_read.py
?? uuv_mujoco/v2.2/bridge/ros2_state_dvl_velocity.py
?? uuv_mujoco/v2.2/bridge/ros2_state_estimation.py
?? uuv_mujoco/v2.2/bridge/ros2_state_imu_vectors.py
?? uuv_mujoco/v2.2/bridge/ros2_state_kinematics.py
?? uuv_mujoco/v2.2/bridge/ros2_state_kinematics_arrays.py
?? uuv_mujoco/v2.2/bridge/ros2_state_object_velocity_read.py
?? uuv_mujoco/v2.2/bridge/ros2_state_sensors.py
?? uuv_mujoco/v2.2/bridge/ros2_state_setpoint.py
?? uuv_mujoco/v2.2/bridge/ros2_state_setpoint_math.py
?? uuv_mujoco/v2.2/bridge/ros2_state_site_read.py
?? uuv_mujoco/v2.2/bridge/ros2_state_sitl_baro.py
?? uuv_mujoco/v2.2/bridge/ros2_state_sitl_frames.py
?? uuv_mujoco/v2.2/bridge/ros2_state_sitl_velocity.py
?? uuv_mujoco/v2.2/bridge/ros2_state_sitl_vertical.py
?? uuv_mujoco/v2.2/bridge/ros2_state_specific_force.py
?? uuv_mujoco/v2.2/bridge/ros2_state_vertical.py
?? uuv_mujoco/v2.2/bridge/ros2_state_vertical_hold.py
?? uuv_mujoco/v2.2/bridge/ros2_state_vertical_hold_window.py
?? uuv_mujoco/v2.2/bridge/ros2_state_vertical_truth.py
?? uuv_mujoco/v2.2/bridge/ros2_state_vertical_zero_reason.py
?? uuv_mujoco/v2.2/bridge/ros2_static_context_publisher.py
?? uuv_mujoco/v2.2/bridge/ros2_static_context_robot_description.py
?? uuv_mujoco/v2.2/bridge/ros2_static_context_tf.py
?? uuv_mujoco/v2.2/bridge/ros2_static_tf_cameras.py
?? uuv_mujoco/v2.2/bridge/ros2_static_tf_core.py
?? uuv_mujoco/v2.2/bridge/ros2_static_tf_sensors.py
?? uuv_mujoco/v2.2/bridge/ros2_static_tf_specs.py
?? uuv_mujoco/v2.2/bridge/ros2_static_tf_types.py
?? uuv_mujoco/v2.2/bridge/ros2_status_messages.py
?? uuv_mujoco/v2.2/bridge/ros2_tf_geometry.py
?? uuv_mujoco/v2.2/bridge/ros2_tf_messages.py
?? uuv_mujoco/v2.2/bridge/ros2_topic_specs.py
?? uuv_mujoco/v2.2/bridge/ros2_topic_summaries.py
?? uuv_mujoco/v2.2/bridge/ros2_twist_messages.py
?? uuv_mujoco/v2.2/bridge/ros2_vfr_hud_messages.py
?? uuv_mujoco/v2.2/bridge/sitl_arm_mode_arm_send.py
?? uuv_mujoco/v2.2/bridge/sitl_arm_mode_mode_send.py
?? uuv_mujoco/v2.2/bridge/sitl_arm_mode_queue.py
?? uuv_mujoco/v2.2/bridge/sitl_arm_mode_queue_arm.py
?? uuv_mujoco/v2.2/bridge/sitl_arm_mode_queue_mode.py
?? uuv_mujoco/v2.2/bridge/sitl_arm_mode_resolve.py
?? uuv_mujoco/v2.2/bridge/sitl_arm_mode_runtime.py
?? uuv_mujoco/v2.2/bridge/sitl_arm_mode_send.py
?? uuv_mujoco/v2.2/bridge/sitl_arm_mode_service.py
?? uuv_mujoco/v2.2/bridge/sitl_arm_mode_service_arm.py
?? uuv_mujoco/v2.2/bridge/sitl_arm_mode_service_mode.py
?? uuv_mujoco/v2.2/bridge/sitl_arm_mode_service_pending.py
?? uuv_mujoco/v2.2/bridge/sitl_arm_mode_service_send.py
?? uuv_mujoco/v2.2/bridge/sitl_arm_mode_service_state.py
?? uuv_mujoco/v2.2/bridge/sitl_auto_ready_actions.py
?? uuv_mujoco/v2.2/bridge/sitl_auto_ready_extnav.py
?? uuv_mujoco/v2.2/bridge/sitl_auto_ready_gates.py
?? uuv_mujoco/v2.2/bridge/sitl_auto_ready_messages.py
?? uuv_mujoco/v2.2/bridge/sitl_auto_ready_neutral.py
?? uuv_mujoco/v2.2/bridge/sitl_auto_ready_runtime.py
?? uuv_mujoco/v2.2/bridge/sitl_auto_ready_sequence.py
?? uuv_mujoco/v2.2/bridge/sitl_auto_ready_state.py
?? uuv_mujoco/v2.2/bridge/sitl_body_velocity_setpoint.py
?? uuv_mujoco/v2.2/bridge/sitl_command_link_activity.py
?? uuv_mujoco/v2.2/bridge/sitl_command_link_readiness.py
?? uuv_mujoco/v2.2/bridge/sitl_command_link_select.py
?? uuv_mujoco/v2.2/bridge/sitl_command_links.py
?? uuv_mujoco/v2.2/bridge/sitl_command_target_heartbeat.py
?? uuv_mujoco/v2.2/bridge/sitl_command_target_resolution.py
?? uuv_mujoco/v2.2/bridge/sitl_command_targets.py
?? uuv_mujoco/v2.2/bridge/sitl_commanding.py
?? uuv_mujoco/v2.2/bridge/sitl_contract.py
?? uuv_mujoco/v2.2/bridge/sitl_env.py
?? uuv_mujoco/v2.2/bridge/sitl_external_nav_bootstrap.py
?? uuv_mujoco/v2.2/bridge/sitl_external_nav_cache.py
?? uuv_mujoco/v2.2/bridge/sitl_external_nav_cache_contract.py
?? uuv_mujoco/v2.2/bridge/sitl_external_nav_contract.py
?? uuv_mujoco/v2.2/bridge/sitl_external_nav_contract_errors.py
?? uuv_mujoco/v2.2/bridge/sitl_external_nav_contract_freshness.py
?? uuv_mujoco/v2.2/bridge/sitl_external_nav_runtime.py
?? uuv_mujoco/v2.2/bridge/sitl_external_nav_synthetic_vpd.py
?? uuv_mujoco/v2.2/bridge/sitl_external_nav_vpd_clock.py
?? uuv_mujoco/v2.2/bridge/sitl_external_nav_vpd_delta_math.py
?? uuv_mujoco/v2.2/bridge/sitl_external_nav_vpd_due.py
?? uuv_mujoco/v2.2/bridge/sitl_external_nav_vpd_emit.py
?? uuv_mujoco/v2.2/bridge/sitl_external_nav_vpd_history.py
?? uuv_mujoco/v2.2/bridge/sitl_external_nav_vpd_pose.py
?? uuv_mujoco/v2.2/bridge/sitl_external_nav_vpd_prepare.py
?? uuv_mujoco/v2.2/bridge/sitl_external_nav_vpd_rate.py
?? uuv_mujoco/v2.2/bridge/sitl_external_nav_vpd_send.py
?? uuv_mujoco/v2.2/bridge/sitl_guided_setpoint_runtime.py
?? uuv_mujoco/v2.2/bridge/sitl_initialization.py
?? uuv_mujoco/v2.2/bridge/sitl_initialization_config.py
?? uuv_mujoco/v2.2/bridge/sitl_initialization_loaders.py
?? uuv_mujoco/v2.2/bridge/sitl_initialization_logging.py
?? uuv_mujoco/v2.2/bridge/sitl_initialization_state.py
?? uuv_mujoco/v2.2/bridge/sitl_json_payload.py
?? uuv_mujoco/v2.2/bridge/sitl_json_replay_reply.py
?? uuv_mujoco/v2.2/bridge/sitl_json_replay_reply_gate.py
?? uuv_mujoco/v2.2/bridge/sitl_json_replay_reply_log.py
?? uuv_mujoco/v2.2/bridge/sitl_json_replay_reply_payload.py
?? uuv_mujoco/v2.2/bridge/sitl_json_sender.py
?? uuv_mujoco/v2.2/bridge/sitl_json_sender_diagnostics.py
?? uuv_mujoco/v2.2/bridge/sitl_json_sender_io.py
?? uuv_mujoco/v2.2/bridge/sitl_json_sender_runtime.py
?? uuv_mujoco/v2.2/bridge/sitl_json_sender_sample_log.py
?? uuv_mujoco/v2.2/bridge/sitl_json_sender_status_log.py
?? uuv_mujoco/v2.2/bridge/sitl_json_sender_validation.py
?? uuv_mujoco/v2.2/bridge/sitl_json_sensor_external_nav.py
?? uuv_mujoco/v2.2/bridge/sitl_json_sensor_replay_apply.py
?? uuv_mujoco/v2.2/bridge/sitl_json_sensor_runtime.py
?? uuv_mujoco/v2.2/bridge/sitl_json_sensor_send.py
?? uuv_mujoco/v2.2/bridge/sitl_json_servo_endpoint.py
?? uuv_mujoco/v2.2/bridge/sitl_json_servo_packet_state.py
?? uuv_mujoco/v2.2/bridge/sitl_json_servo_poll_loop.py
?? uuv_mujoco/v2.2/bridge/sitl_json_servo_runtime.py
?? uuv_mujoco/v2.2/bridge/sitl_json_servo_timeout.py
?? uuv_mujoco/v2.2/bridge/sitl_json_servo_warnings.py
?? uuv_mujoco/v2.2/bridge/sitl_local_ned_setpoint.py
?? uuv_mujoco/v2.2/bridge/sitl_manual_control_frame.py
?? uuv_mujoco/v2.2/bridge/sitl_manual_control_logging.py
?? uuv_mujoco/v2.2/bridge/sitl_manual_control_runtime.py
?? uuv_mujoco/v2.2/bridge/sitl_math.py
?? uuv_mujoco/v2.2/bridge/sitl_mavlink_command_connection.py
?? uuv_mujoco/v2.2/bridge/sitl_mavlink_command_handlers.py
?? uuv_mujoco/v2.2/bridge/sitl_mavlink_command_polling.py
?? uuv_mujoco/v2.2/bridge/sitl_mavlink_command_receive.py
?? uuv_mujoco/v2.2/bridge/sitl_mavlink_connection.py
?? uuv_mujoco/v2.2/bridge/sitl_mavlink_endpoint.py
?? uuv_mujoco/v2.2/bridge/sitl_mavlink_heartbeat.py
?? uuv_mujoco/v2.2/bridge/sitl_mavlink_imports.py
?? uuv_mujoco/v2.2/bridge/sitl_mavlink_peer.py
?? uuv_mujoco/v2.2/bridge/sitl_mavlink_peer_state.py
?? uuv_mujoco/v2.2/bridge/sitl_mavlink_peer_wait.py
?? uuv_mujoco/v2.2/bridge/sitl_mavlink_polling.py
?? uuv_mujoco/v2.2/bridge/sitl_mavlink_request_ap.py
?? uuv_mujoco/v2.2/bridge/sitl_mavlink_request_command_link.py
?? uuv_mujoco/v2.2/bridge/sitl_mavlink_request_servo.py
?? uuv_mujoco/v2.2/bridge/sitl_mavlink_request_servo_link.py
?? uuv_mujoco/v2.2/bridge/sitl_mavlink_request_targets.py
?? uuv_mujoco/v2.2/bridge/sitl_mavlink_requests.py
?? uuv_mujoco/v2.2/bridge/sitl_mavlink_runtime.py
?? uuv_mujoco/v2.2/bridge/sitl_mavlink_servo_callback.py
?? uuv_mujoco/v2.2/bridge/sitl_mavlink_servo_connection.py
?? uuv_mujoco/v2.2/bridge/sitl_mavlink_servo_drain.py
?? uuv_mujoco/v2.2/bridge/sitl_mavlink_servo_handlers.py
?? uuv_mujoco/v2.2/bridge/sitl_mavlink_servo_heartbeat.py
?? uuv_mujoco/v2.2/bridge/sitl_mavlink_servo_heartbeat_target.py
?? uuv_mujoco/v2.2/bridge/sitl_mavlink_servo_output.py
?? uuv_mujoco/v2.2/bridge/sitl_mavlink_servo_polling.py
?? uuv_mujoco/v2.2/bridge/sitl_mavlink_servo_receive.py
?? uuv_mujoco/v2.2/bridge/sitl_mavlink_telemetry.py
?? uuv_mujoco/v2.2/bridge/sitl_native_vpd_debug.py
?? uuv_mujoco/v2.2/bridge/sitl_native_vpd_rate.py
?? uuv_mujoco/v2.2/bridge/sitl_native_vpd_runtime.py
?? uuv_mujoco/v2.2/bridge/sitl_native_vpd_send.py
?? uuv_mujoco/v2.2/bridge/sitl_native_vpd_start.py
?? uuv_mujoco/v2.2/bridge/sitl_pwm_activity.py
?? uuv_mujoco/v2.2/bridge/sitl_pwm_frame_handler.py
?? uuv_mujoco/v2.2/bridge/sitl_pwm_output.py
?? uuv_mujoco/v2.2/bridge/sitl_pwm_runtime.py
?? uuv_mujoco/v2.2/bridge/sitl_pwm_safety.py
?? uuv_mujoco/v2.2/bridge/sitl_pwm_source_policy.py
?? uuv_mujoco/v2.2/bridge/sitl_rc_manual_runtime.py
?? uuv_mujoco/v2.2/bridge/sitl_rc_override_core.py
?? uuv_mujoco/v2.2/bridge/sitl_rc_override_keepalive.py
?? uuv_mujoco/v2.2/bridge/sitl_rc_override_runtime.py
?? uuv_mujoco/v2.2/bridge/sitl_rc_override_send.py
?? uuv_mujoco/v2.2/bridge/sitl_rc_override_warn.py
?? uuv_mujoco/v2.2/bridge/sitl_replay.py
?? uuv_mujoco/v2.2/bridge/sitl_replay_common.py
?? uuv_mujoco/v2.2/bridge/sitl_replay_csv_loader.py
?? uuv_mujoco/v2.2/bridge/sitl_replay_interp_frame.py
?? uuv_mujoco/v2.2/bridge/sitl_replay_interp_index.py
?? uuv_mujoco/v2.2/bridge/sitl_replay_interp_math.py
?? uuv_mujoco/v2.2/bridge/sitl_replay_interpolation.py
?? uuv_mujoco/v2.2/bridge/sitl_replay_loaders.py
?? uuv_mujoco/v2.2/bridge/sitl_replay_row_parsers.py
?? uuv_mujoco/v2.2/bridge/sitl_replay_row_vectors.py
?? uuv_mujoco/v2.2/bridge/sitl_replay_sensor_row.py
?? uuv_mujoco/v2.2/bridge/sitl_replay_types.py
?? uuv_mujoco/v2.2/bridge/sitl_replay_vpd_row.py
?? uuv_mujoco/v2.2/bridge/sitl_sensor_replay_clock.py
?? uuv_mujoco/v2.2/bridge/sitl_sensor_replay_frame_policy.py
?? uuv_mujoco/v2.2/bridge/sitl_sensor_replay_hold_policy.py
?? uuv_mujoco/v2.2/bridge/sitl_sensor_replay_payload_time.py
?? uuv_mujoco/v2.2/bridge/sitl_sensor_replay_runtime.py
?? uuv_mujoco/v2.2/bridge/sitl_sensor_replay_servo_clock.py
?? uuv_mujoco/v2.2/bridge/sitl_sensor_replay_start_policy.py
?? uuv_mujoco/v2.2/bridge/sitl_sensor_replay_state.py
?? uuv_mujoco/v2.2/bridge/sitl_sensor_replay_status_runtime.py
?? uuv_mujoco/v2.2/bridge/sitl_status.py
?? uuv_mujoco/v2.2/bridge/sitl_status_age.py
?? uuv_mujoco/v2.2/bridge/sitl_status_mavlink.py
?? uuv_mujoco/v2.2/bridge/sitl_status_mavlink_core.py
?? uuv_mujoco/v2.2/bridge/sitl_status_mavlink_extnav.py
?? uuv_mujoco/v2.2/bridge/sitl_status_sensor_replay.py
?? uuv_mujoco/v2.2/bridge/sitl_transport_auto_ready_config.py
?? uuv_mujoco/v2.2/bridge/sitl_transport_bindings.py
?? uuv_mujoco/v2.2/bridge/sitl_transport_command_bindings.py
?? uuv_mujoco/v2.2/bridge/sitl_transport_command_link_config.py
?? uuv_mujoco/v2.2/bridge/sitl_transport_config.py
?? uuv_mujoco/v2.2/bridge/sitl_transport_control_config.py
?? uuv_mujoco/v2.2/bridge/sitl_transport_extnav_base.py
?? uuv_mujoco/v2.2/bridge/sitl_transport_extnav_bindings.py
?? uuv_mujoco/v2.2/bridge/sitl_transport_extnav_config.py
?? uuv_mujoco/v2.2/bridge/sitl_transport_extnav_logging.py
?? uuv_mujoco/v2.2/bridge/sitl_transport_extnav_runtime.py
?? uuv_mujoco/v2.2/bridge/sitl_transport_extnav_scheduler.py
?? uuv_mujoco/v2.2/bridge/sitl_transport_handler_bindings.py
?? uuv_mujoco/v2.2/bridge/sitl_transport_handlers.py
?? uuv_mujoco/v2.2/bridge/sitl_transport_json_bindings.py
?? uuv_mujoco/v2.2/bridge/sitl_transport_json_config.py
?? uuv_mujoco/v2.2/bridge/sitl_transport_lifecycle.py
?? uuv_mujoco/v2.2/bridge/sitl_transport_mavlink_base_config.py
?? uuv_mujoco/v2.2/bridge/sitl_transport_mavlink_bindings.py
?? uuv_mujoco/v2.2/bridge/sitl_transport_mavlink_config.py
?? uuv_mujoco/v2.2/bridge/sitl_transport_mavlink_telemetry_config.py
?? uuv_mujoco/v2.2/bridge/sitl_transport_model_sensors.py
?? uuv_mujoco/v2.2/bridge/sitl_transport_model_state.py
?? uuv_mujoco/v2.2/bridge/sitl_transport_model_vertical.py
?? uuv_mujoco/v2.2/bridge/sitl_transport_polling_config.py
?? uuv_mujoco/v2.2/bridge/sitl_transport_state_bindings.py
?? uuv_mujoco/v2.2/bridge/sitl_transport_status_runtime.py
?? uuv_mujoco/v2.2/bridge/sitl_transport_vertical_math.py
?? uuv_mujoco/v2.2/bridge/sitl_types.py
?? uuv_mujoco/v2.2/bridge/sitl_vehicle_command_ack.py
?? uuv_mujoco/v2.2/bridge/sitl_vehicle_heartbeat.py
?? uuv_mujoco/v2.2/bridge/sitl_vehicle_heartbeat_filter.py
?? uuv_mujoco/v2.2/bridge/sitl_vehicle_heartbeat_target.py
?? uuv_mujoco/v2.2/bridge/sitl_vehicle_state.py
?? uuv_mujoco/v2.2/bridge/sitl_vehicle_state_extract.py
?? uuv_mujoco/v2.2/bridge/sitl_vehicle_state_log.py
?? uuv_mujoco/v2.2/config/ardusub_realrobot_contract.param
?? uuv_mujoco/v2.2/debug/
?? uuv_mujoco/v2.2/docs/
?? uuv_mujoco/v2.2/experiments/
?? uuv_mujoco/v2.2/gui/app_lifecycle.py
?? uuv_mujoco/v2.2/gui/app_shutdown_steps.py
?? uuv_mujoco/v2.2/gui/app_state.py
?? uuv_mujoco/v2.2/gui/app_state_core_vars.py
?? uuv_mujoco/v2.2/gui/app_state_feature_vars.py
?? uuv_mujoco/v2.2/gui/app_state_tk_vars.py
?? uuv_mujoco/v2.2/gui/app_state_vars.py
?? uuv_mujoco/v2.2/gui/backend_helpers.py
?? uuv_mujoco/v2.2/gui/config_backend.py
?? uuv_mujoco/v2.2/gui/config_env.py
?? uuv_mujoco/v2.2/gui/config_paths.py
?? uuv_mujoco/v2.2/gui/config_physics.py
?? uuv_mujoco/v2.2/gui/config_rc.py
?? uuv_mujoco/v2.2/gui/config_ui.py
?? uuv_mujoco/v2.2/gui/control_attitude_draw.py
?? uuv_mujoco/v2.2/gui/control_depth_draw.py
?? uuv_mujoco/v2.2/gui/control_draw_mixin.py
?? uuv_mujoco/v2.2/gui/control_feedback_mixin.py
?? uuv_mujoco/v2.2/gui/control_pilot_commands.py
?? uuv_mujoco/v2.2/gui/control_pilot_mixin.py
?? uuv_mujoco/v2.2/gui/control_pilot_publish.py
?? uuv_mujoco/v2.2/gui/control_pilot_release.py
?? uuv_mujoco/v2.2/gui/control_pilot_toggle.py
?? uuv_mujoco/v2.2/gui/control_toggle_mixin.py
?? uuv_mujoco/v2.2/gui/control_toggle_telemetry.py
?? uuv_mujoco/v2.2/gui/control_update_apply.py
?? uuv_mujoco/v2.2/gui/control_update_command_ready.py
?? uuv_mujoco/v2.2/gui/control_update_format.py
?? uuv_mujoco/v2.2/gui/control_update_mixin.py
?? uuv_mujoco/v2.2/gui/control_update_mode.py
?? uuv_mujoco/v2.2/gui/control_update_pilot_texts.py
?? uuv_mujoco/v2.2/gui/control_update_telemetry_status.py
?? uuv_mujoco/v2.2/gui/control_update_telemetry_texts.py
?? uuv_mujoco/v2.2/gui/control_update_text_models.py
?? uuv_mujoco/v2.2/gui/control_update_texts.py
?? uuv_mujoco/v2.2/gui/control_update_vehicle.py
?? uuv_mujoco/v2.2/gui/gui_axis_normalization.py
?? uuv_mujoco/v2.2/gui/gui_math_helpers.py
?? uuv_mujoco/v2.2/gui/gui_rc_althold.py
?? uuv_mujoco/v2.2/gui/gui_rc_axes.py
?? uuv_mujoco/v2.2/gui/gui_rc_helpers.py
?? uuv_mujoco/v2.2/gui/gui_rc_messages.py
?? uuv_mujoco/v2.2/gui/gui_rc_padding.py
?? uuv_mujoco/v2.2/gui/gui_rc_pwm.py
?? uuv_mujoco/v2.2/gui/joystick_math.py
?? uuv_mujoco/v2.2/gui/joystick_render.py
?? uuv_mujoco/v2.2/gui/layout_control_core.py
?? uuv_mujoco/v2.2/gui/layout_control_modes.py
?? uuv_mujoco/v2.2/gui/layout_control_pilot.py
?? uuv_mujoco/v2.2/gui/layout_control_replay.py
?? uuv_mujoco/v2.2/gui/layout_control_ros2.py
?? uuv_mujoco/v2.2/gui/layout_control_stack.py
?? uuv_mujoco/v2.2/gui/layout_control_tuning.py
?? uuv_mujoco/v2.2/gui/layout_event_log.py
?? uuv_mujoco/v2.2/gui/layout_shell.py
?? uuv_mujoco/v2.2/gui/layout_telemetry.py
?? uuv_mujoco/v2.2/gui/layout_vehicle_summary.py
?? uuv_mujoco/v2.2/gui/layout_vehicle_visuals.py
?? uuv_mujoco/v2.2/gui/node_arm_commands.py
?? uuv_mujoco/v2.2/gui/node_arm_mode_commands.py
?? uuv_mujoco/v2.2/gui/node_arm_request_deadline.py
?? uuv_mujoco/v2.2/gui/node_arm_request_gates.py
?? uuv_mujoco/v2.2/gui/node_arm_request_service.py
?? uuv_mujoco/v2.2/gui/node_arm_request_steps.py
?? uuv_mujoco/v2.2/gui/node_arm_request_topic.py
?? uuv_mujoco/v2.2/gui/node_backend_counts.py
?? uuv_mujoco/v2.2/gui/node_backend_layout.py
?? uuv_mujoco/v2.2/gui/node_backend_runtime.py
?? uuv_mujoco/v2.2/gui/node_backend_score_fields.py
?? uuv_mujoco/v2.2/gui/node_backend_select_policy.py
?? uuv_mujoco/v2.2/gui/node_backend_selection.py
?? uuv_mujoco/v2.2/gui/node_bindings.py
?? uuv_mujoco/v2.2/gui/node_command_attempts.py
?? uuv_mujoco/v2.2/gui/node_command_override_pub.py
?? uuv_mujoco/v2.2/gui/node_command_retries.py
?? uuv_mujoco/v2.2/gui/node_command_state_gates.py
?? uuv_mujoco/v2.2/gui/node_command_timing.py
?? uuv_mujoco/v2.2/gui/node_commanding.py
?? uuv_mujoco/v2.2/gui/node_commanding_common.py
?? uuv_mujoco/v2.2/gui/node_init.py
?? uuv_mujoco/v2.2/gui/node_init_publishers.py
?? uuv_mujoco/v2.2/gui/node_init_state.py
?? uuv_mujoco/v2.2/gui/node_init_subscriptions.py
?? uuv_mujoco/v2.2/gui/node_initial_depth_commands.py
?? uuv_mujoco/v2.2/gui/node_initial_depth_state.py
?? uuv_mujoco/v2.2/gui/node_manual_control_publishers.py
?? uuv_mujoco/v2.2/gui/node_mode_commands.py
?? uuv_mujoco/v2.2/gui/node_mode_request_gates.py
?? uuv_mujoco/v2.2/gui/node_mode_request_service.py
?? uuv_mujoco/v2.2/gui/node_mode_request_steps.py
?? uuv_mujoco/v2.2/gui/node_mode_request_topic.py
?? uuv_mujoco/v2.2/gui/node_motion_callbacks.py
?? uuv_mujoco/v2.2/gui/node_motion_depth_callbacks.py
?? uuv_mujoco/v2.2/gui/node_motion_imu_callbacks.py
?? uuv_mujoco/v2.2/gui/node_motion_pose_callbacks.py
?? uuv_mujoco/v2.2/gui/node_motion_pose_state.py
?? uuv_mujoco/v2.2/gui/node_motion_velocity_callbacks.py
?? uuv_mujoco/v2.2/gui/node_ping360_callbacks.py
?? uuv_mujoco/v2.2/gui/node_ping360_publishers.py
?? uuv_mujoco/v2.2/gui/node_ping360_status_payload.py
?? uuv_mujoco/v2.2/gui/node_ping360_status_settings.py
?? uuv_mujoco/v2.2/gui/node_ping360_status_summary.py
?? uuv_mujoco/v2.2/gui/node_rc_callbacks.py
?? uuv_mujoco/v2.2/gui/node_rc_frame.py
?? uuv_mujoco/v2.2/gui/node_rc_override_publishers.py
?? uuv_mujoco/v2.2/gui/node_rc_publishers.py
?? uuv_mujoco/v2.2/gui/node_readiness_freshness.py
?? uuv_mujoco/v2.2/gui/node_readiness_inputs.py
?? uuv_mujoco/v2.2/gui/node_readiness_mode.py
?? uuv_mujoco/v2.2/gui/node_readiness_runtime.py
?? uuv_mujoco/v2.2/gui/node_readiness_runtime_inputs.py
?? uuv_mujoco/v2.2/gui/node_sitl_status_callbacks.py
?? uuv_mujoco/v2.2/gui/node_snapshot_runtime.py
?? uuv_mujoco/v2.2/gui/node_state_runtime.py
?? uuv_mujoco/v2.2/gui/node_subscription_core_sensors.py
?? uuv_mujoco/v2.2/gui/node_subscription_depth.py
?? uuv_mujoco/v2.2/gui/node_subscription_external_motion.py
?? uuv_mujoco/v2.2/gui/node_subscription_mavros.py
?? uuv_mujoco/v2.2/gui/node_subscription_qos.py
?? uuv_mujoco/v2.2/gui/node_subscription_status.py
?? uuv_mujoco/v2.2/gui/node_telemetry_callbacks.py
?? uuv_mujoco/v2.2/gui/node_trigger_service_callbacks.py
?? uuv_mujoco/v2.2/gui/node_trigger_services.py
?? uuv_mujoco/v2.2/gui/node_vehicle_callbacks.py
?? uuv_mujoco/v2.2/gui/node_vehicle_info.py
?? uuv_mujoco/v2.2/gui/physics_param_apply.py
?? uuv_mujoco/v2.2/gui/physics_param_error_log.py
?? uuv_mujoco/v2.2/gui/physics_param_format.py
?? uuv_mujoco/v2.2/gui/physics_param_io.py
?? uuv_mujoco/v2.2/gui/physics_param_load.py
?? uuv_mujoco/v2.2/gui/physics_param_nested.py
?? uuv_mujoco/v2.2/gui/physics_param_parse.py
?? uuv_mujoco/v2.2/gui/physics_param_persist.py
?? uuv_mujoco/v2.2/gui/physics_param_status.py
?? uuv_mujoco/v2.2/gui/physics_profile_select.py
?? uuv_mujoco/v2.2/gui/physics_restart.py
?? uuv_mujoco/v2.2/gui/physics_restart_finish.py
?? uuv_mujoco/v2.2/gui/physics_restart_process.py
?? uuv_mujoco/v2.2/gui/physics_restart_reset.py
?? uuv_mujoco/v2.2/gui/physics_restart_reset_events.py
?? uuv_mujoco/v2.2/gui/physics_restart_reset_paths.py
?? uuv_mujoco/v2.2/gui/physics_window.py
?? uuv_mujoco/v2.2/gui/physics_window_rows.py
?? uuv_mujoco/v2.2/gui/physics_window_shell.py
?? uuv_mujoco/v2.2/gui/ping360_config_mixin.py
?? uuv_mujoco/v2.2/gui/ping360_mixin.py
?? uuv_mujoco/v2.2/gui/ping360_view_mixin.py
?? uuv_mujoco/v2.2/gui/ping360_view_process.py
?? uuv_mujoco/v2.2/gui/ping360_view_status.py
?? uuv_mujoco/v2.2/gui/ping360_window_lifecycle.py
?? uuv_mujoco/v2.2/gui/ping360_window_mixin.py
?? uuv_mujoco/v2.2/gui/ping360_window_panels.py
?? uuv_mujoco/v2.2/gui/process_common_mixin.py
?? uuv_mujoco/v2.2/gui/process_env.py
?? uuv_mujoco/v2.2/gui/process_scan.py
?? uuv_mujoco/v2.2/gui/process_scan_rows.py
?? uuv_mujoco/v2.2/gui/process_termination.py
?? uuv_mujoco/v2.2/gui/process_termination_signal.py
?? uuv_mujoco/v2.2/gui/rc_replay_decode.py
?? uuv_mujoco/v2.2/gui/rc_replay_loader.py
?? uuv_mujoco/v2.2/gui/rc_replay_path.py
?? uuv_mujoco/v2.2/gui/rc_replay_rosbag.py
?? uuv_mujoco/v2.2/gui/readiness_arm_mode_gate.py
?? uuv_mujoco/v2.2/gui/readiness_command_link.py
?? uuv_mujoco/v2.2/gui/readiness_contract.py
?? uuv_mujoco/v2.2/gui/readiness_extnav.py
?? uuv_mujoco/v2.2/gui/readiness_feedback_gate.py
?? uuv_mujoco/v2.2/gui/replay_browse.py
?? uuv_mujoco/v2.2/gui/replay_controls.py
?? uuv_mujoco/v2.2/gui/replay_format.py
?? uuv_mujoco/v2.2/gui/replay_load_controls.py
?? uuv_mujoco/v2.2/gui/replay_playback_controls.py
?? uuv_mujoco/v2.2/gui/replay_seek_state.py
?? uuv_mujoco/v2.2/gui/replay_slider_events.py
?? uuv_mujoco/v2.2/gui/replay_status.py
?? uuv_mujoco/v2.2/gui/replay_time_math.py
?? uuv_mujoco/v2.2/gui/replay_timeline.py
?? uuv_mujoco/v2.2/gui/replay_worker.py
?? uuv_mujoco/v2.2/gui/replay_worker_finish.py
?? uuv_mujoco/v2.2/gui/replay_worker_pause.py
?? uuv_mujoco/v2.2/gui/replay_worker_publish.py
?? uuv_mujoco/v2.2/gui/replay_worker_seek.py
?? uuv_mujoco/v2.2/gui/replay_worker_state.py
?? uuv_mujoco/v2.2/gui/replay_worker_step.py
?? uuv_mujoco/v2.2/gui/replay_worker_step_phases.py
?? uuv_mujoco/v2.2/gui/replay_worker_wait.py
?? uuv_mujoco/v2.2/gui/ros_bash.py
?? uuv_mujoco/v2.2/gui/ros_env_tools.py
?? uuv_mujoco/v2.2/gui/ros_log_events.py
?? uuv_mujoco/v2.2/gui/ros_log_tail_loop.py
?? uuv_mujoco/v2.2/gui/ros_logged_process_files.py
?? uuv_mujoco/v2.2/gui/ros_logged_process_finish.py
?? uuv_mujoco/v2.2/gui/ros_logged_process_launcher.py
?? uuv_mujoco/v2.2/gui/ros_logged_process_mixin.py
?? uuv_mujoco/v2.2/gui/ros_logged_process_start.py
?? uuv_mujoco/v2.2/gui/ros_logged_process_tail.py
?? uuv_mujoco/v2.2/gui/ros_logged_process_threads.py
?? uuv_mujoco/v2.2/gui/ros_logged_process_watch.py
?? uuv_mujoco/v2.2/gui/ros_package_build.py
?? uuv_mujoco/v2.2/gui/ros_package_mixin.py
?? uuv_mujoco/v2.2/gui/ros_package_stack.py
?? uuv_mujoco/v2.2/gui/ros_panel_buttons.py
?? uuv_mujoco/v2.2/gui/ros_panel_mixin.py
?? uuv_mujoco/v2.2/gui/ros_panel_process_state.py
?? uuv_mujoco/v2.2/gui/ros_panel_status.py
?? uuv_mujoco/v2.2/gui/ros_panel_visibility.py
?? uuv_mujoco/v2.2/gui/ros_setup_candidates.py
?? uuv_mujoco/v2.2/gui/ros_setup_paths.py
?? uuv_mujoco/v2.2/gui/ros_setup_probe.py
?? uuv_mujoco/v2.2/gui/ros_workspace_setup_paths.py
?? uuv_mujoco/v2.2/gui/runtime_mavros.py
?? uuv_mujoco/v2.2/gui/runtime_python_path.py
?? uuv_mujoco/v2.2/gui/runtime_python_path_filters.py
?? uuv_mujoco/v2.2/gui/runtime_ros_core.py
?? uuv_mujoco/v2.2/gui/rviz_config_ping360.py
?? uuv_mujoco/v2.2/gui/rviz_config_ros2.py
?? uuv_mujoco/v2.2/gui/rviz_config_tools.py
?? uuv_mujoco/v2.2/gui/rviz_process_mixin.py
?? uuv_mujoco/v2.2/gui/sim_stack_env.py
?? uuv_mujoco/v2.2/gui/sim_stack_env_args.py
?? uuv_mujoco/v2.2/gui/sim_stack_env_contract.py
?? uuv_mujoco/v2.2/gui/sim_stack_env_defaults.py
?? uuv_mujoco/v2.2/gui/sim_stack_env_flags.py
?? uuv_mujoco/v2.2/gui/sim_stack_env_forced.py
?? uuv_mujoco/v2.2/gui/sim_stack_env_forced_command.py
?? uuv_mujoco/v2.2/gui/sim_stack_env_forced_ekf.py
?? uuv_mujoco/v2.2/gui/sim_stack_env_forced_rc.py
?? uuv_mujoco/v2.2/gui/sim_stack_env_modes.py
?? uuv_mujoco/v2.2/gui/sim_stack_env_types.py
?? uuv_mujoco/v2.2/gui/sim_stack_extra_args.py
?? uuv_mujoco/v2.2/gui/sim_stack_initial_depth_args.py
?? uuv_mujoco/v2.2/gui/sim_stack_initial_depth_base_link.py
?? uuv_mujoco/v2.2/gui/sim_stack_initial_depth_contract.py
?? uuv_mujoco/v2.2/gui/sim_stack_initial_depth_sources.py
?? uuv_mujoco/v2.2/gui/sim_stack_launch_args.py
?? uuv_mujoco/v2.2/gui/sim_stack_launch_command.py
?? uuv_mujoco/v2.2/gui/sim_stack_launch_logs.py
?? uuv_mujoco/v2.2/gui/sim_stack_launch_mixin.py
?? uuv_mujoco/v2.2/gui/sim_stack_launch_process.py
?? uuv_mujoco/v2.2/gui/sim_stack_launch_runtime.py
?? uuv_mujoco/v2.2/gui/sim_stack_launch_target.py
?? uuv_mujoco/v2.2/gui/sim_stack_log_finish.py
?? uuv_mujoco/v2.2/gui/sim_stack_log_line_handler.py
?? uuv_mujoco/v2.2/gui/sim_stack_log_reader.py
?? uuv_mujoco/v2.2/gui/sim_stack_log_status.py
?? uuv_mujoco/v2.2/gui/sim_stack_log_watcher.py
?? uuv_mujoco/v2.2/gui/sim_stack_process_mixin.py
?? uuv_mujoco/v2.2/gui/sim_stack_process_probe.py
?? uuv_mujoco/v2.2/gui/sim_stack_reset_commands.py
?? uuv_mujoco/v2.2/gui/sim_stack_reset_mixin.py
?? uuv_mujoco/v2.2/gui/sim_stack_reset_output.py
?? uuv_mujoco/v2.2/gui/sim_stack_reset_result.py
?? uuv_mujoco/v2.2/gui/sim_stack_reset_thread.py
?? uuv_mujoco/v2.2/gui/sim_stack_reset_worker.py
?? uuv_mujoco/v2.2/gui/sim_stack_restart_runtime.py
?? uuv_mujoco/v2.2/gui/sim_stack_start_guards.py
?? uuv_mujoco/v2.2/gui/sim_stack_start_process.py
?? uuv_mujoco/v2.2/gui/sim_stack_start_runtime.py
?? uuv_mujoco/v2.2/gui/sim_stack_start_state.py
?? uuv_mujoco/v2.2/gui/sim_stack_status_controls.py
?? uuv_mujoco/v2.2/gui/sim_stack_status_logic.py
?? uuv_mujoco/v2.2/gui/sim_stack_status_mixin.py
?? uuv_mujoco/v2.2/gui/sim_stack_status_refresh.py
?? uuv_mujoco/v2.2/gui/sim_stack_status_text.py
?? uuv_mujoco/v2.2/gui/sim_stack_viewer_args.py
?? uuv_mujoco/v2.2/gui/sim_stack_wrapper_args.py
?? uuv_mujoco/v2.2/gui/virtual_joystick_runtime.py
?? uuv_mujoco/v2.2/han/
?? uuv_mujoco/v2.2/physics/ellipsoid_geometry.py
?? uuv_mujoco/v2.2/physics/ellipsoid_hydro_coefficients.py
?? uuv_mujoco/v2.2/physics/ellipsoid_hydro_types.py
?? uuv_mujoco/v2.2/physics/ellipsoid_hydrodynamics.py
?? uuv_mujoco/v2.2/physics/hydrodynamics_math.py
?? uuv_mujoco/v2.2/physics/hydrostatic_fraction_helpers.py
?? uuv_mujoco/v2.2/physics/sim_profile_defaults.py
?? uuv_mujoco/v2.2/physics/sim_profile_ellipsoid.py
?? uuv_mujoco/v2.2/physics/sim_profile_hydrodynamics.py
?? uuv_mujoco/v2.2/physics/sim_profile_hydrodynamics_damping.py
?? uuv_mujoco/v2.2/physics/sim_profile_hydrodynamics_hydrostatic.py
?? uuv_mujoco/v2.2/physics/sim_profile_hydrostatic_body_components.py
?? uuv_mujoco/v2.2/physics/sim_profile_hydrostatic_buoyancy_point_fields.py
?? uuv_mujoco/v2.2/physics/sim_profile_hydrostatic_buoyancy_points.py
?? uuv_mujoco/v2.2/physics/sim_profile_hydrostatic_component_builder.py
?? uuv_mujoco/v2.2/physics/sim_profile_hydrostatic_component_fields.py
?? uuv_mujoco/v2.2/physics/sim_profile_hydrostatic_component_numbers.py
?? uuv_mujoco/v2.2/physics/sim_profile_hydrostatic_normalize.py
?? uuv_mujoco/v2.2/physics/sim_profile_hydrostatic_points.py
?? uuv_mujoco/v2.2/physics/sim_profile_hydrostatic_restoring.py
?? uuv_mujoco/v2.2/physics/sim_profile_parse_array.py
?? uuv_mujoco/v2.2/physics/sim_profile_parse_common.py
?? uuv_mujoco/v2.2/physics/sim_profile_parse_scalar.py
?? uuv_mujoco/v2.2/physics/sim_profile_parse_vectors.py
?? uuv_mujoco/v2.2/physics/sim_profile_parsing.py
?? uuv_mujoco/v2.2/physics/sim_profile_types.py
?? uuv_mujoco/v2.2/physics/thruster_curve_helpers.py
?? uuv_mujoco/v2.2/physics/thruster_performance_curves.py
?? uuv_mujoco/v2.2/physics/thruster_performance_parse.py
?? uuv_mujoco/v2.2/physics/thruster_performance_payload.py
?? uuv_mujoco/v2.2/physics/thruster_performance_select.py
?? uuv_mujoco/v2.2/physics/thruster_performance_values.py
?? uuv_mujoco/v2.2/research_workspace/
?? uuv_mujoco/v2.2/run_uuv_mujoco.py
?? uuv_mujoco/v2.2/sim/
?? uuv_mujoco/v2.2/tools/actuator_wrench_audit.py
?? uuv_mujoco/v2.2/tools/actuator_wrench_axis.py
?? uuv_mujoco/v2.2/tools/actuator_wrench_calc.py
?? uuv_mujoco/v2.2/tools/actuator_wrench_common.py
?? uuv_mujoco/v2.2/tools/actuator_wrench_gains.py
?? uuv_mujoco/v2.2/tools/actuator_wrench_model.py
?? uuv_mujoco/v2.2/tools/actuator_wrench_report.py
?? uuv_mujoco/v2.2/tools/actuator_wrench_site_adjustments.py
?? uuv_mujoco/v2.2/tools/actuator_wrench_sites.py
?? uuv_mujoco/v2.2/tools/actuator_wrench_summary.py
?? uuv_mujoco/v2.2/tools/althold_contract_bin.py
?? uuv_mujoco/v2.2/tools/althold_contract_bin_message_loop.py
?? uuv_mujoco/v2.2/tools/althold_contract_bin_paths.py
?? uuv_mujoco/v2.2/tools/althold_contract_bin_reader.py
?? uuv_mujoco/v2.2/tools/althold_contract_bin_schema.py
?? uuv_mujoco/v2.2/tools/althold_contract_bin_streams.py
?? uuv_mujoco/v2.2/tools/althold_contract_model.py
?? uuv_mujoco/v2.2/tools/althold_contract_plot.py
?? uuv_mujoco/v2.2/tools/althold_contract_plot_modes.py
?? uuv_mujoco/v2.2/tools/althold_contract_plot_panels.py
?? uuv_mujoco/v2.2/tools/althold_contract_segments.py
?? uuv_mujoco/v2.2/tools/althold_contract_signal_math.py
?? uuv_mujoco/v2.2/tools/althold_contract_signals.py
?? uuv_mujoco/v2.2/tools/althold_contract_summary.py
?? uuv_mujoco/v2.2/tools/althold_diagnostics_callbacks.py
?? uuv_mujoco/v2.2/tools/althold_diagnostics_contract.py
?? uuv_mujoco/v2.2/tools/althold_diagnostics_csv.py
?? uuv_mujoco/v2.2/tools/althold_diagnostics_depth_callbacks.py
?? uuv_mujoco/v2.2/tools/althold_diagnostics_node.py
?? uuv_mujoco/v2.2/tools/althold_diagnostics_output.py
?? uuv_mujoco/v2.2/tools/althold_diagnostics_plot.py
?? uuv_mujoco/v2.2/tools/althold_diagnostics_rc_callbacks.py
?? uuv_mujoco/v2.2/tools/althold_diagnostics_ros_imports.py
?? uuv_mujoco/v2.2/tools/althold_diagnostics_runtime.py
?? uuv_mujoco/v2.2/tools/althold_diagnostics_series.py
?? uuv_mujoco/v2.2/tools/althold_diagnostics_status_callbacks.py
?? uuv_mujoco/v2.2/tools/althold_diagnostics_subscriptions.py
?? uuv_mujoco/v2.2/tools/althold_diagnostics_summary.py
?? uuv_mujoco/v2.2/tools/ardusub_thruster_contract_constants.py
?? uuv_mujoco/v2.2/tools/ardusub_thruster_contract_response.py
?? uuv_mujoco/v2.2/tools/ardusub_thruster_contract_scene.py
?? uuv_mujoco/v2.2/tools/audit_closed_loop_param_compare.py
?? uuv_mujoco/v2.2/tools/audit_closed_loop_param_io.py
?? uuv_mujoco/v2.2/tools/audit_closed_loop_param_lines.py
?? uuv_mujoco/v2.2/tools/audit_closed_loop_param_start_sitl.py
?? uuv_mujoco/v2.2/tools/audit_closed_loop_param_watchlist.py
?? uuv_mujoco/v2.2/tools/audit_closed_loop_params.py
?? uuv_mujoco/v2.2/tools/audit_closed_loop_payload.py
?? uuv_mujoco/v2.2/tools/audit_closed_loop_profile.py
?? uuv_mujoco/v2.2/tools/audit_closed_loop_profile_active.py
?? uuv_mujoco/v2.2/tools/audit_closed_loop_profile_keys.py
?? uuv_mujoco/v2.2/tools/audit_closed_loop_profile_load.py
?? uuv_mujoco/v2.2/tools/audit_closed_loop_sitl_params.py
?? uuv_mujoco/v2.2/tools/audit_closed_loop_thruster_curve.py
?? uuv_mujoco/v2.2/tools/audit_closed_loop_thruster_curve_candidates.py
?? uuv_mujoco/v2.2/tools/audit_closed_loop_thruster_curve_forces.py
?? uuv_mujoco/v2.2/tools/audit_closed_loop_thruster_curve_parse.py
?? uuv_mujoco/v2.2/tools/audit_closed_loop_thruster_curve_values.py
?? uuv_mujoco/v2.2/tools/audit_code_contract_ardupilot_identity.py
?? uuv_mujoco/v2.2/tools/audit_code_contract_ardupilot_identity_checks.py
?? uuv_mujoco/v2.2/tools/audit_code_contract_ardupilot_identity_info.py
?? uuv_mujoco/v2.2/tools/audit_code_contract_checks.py
?? uuv_mujoco/v2.2/tools/audit_code_contract_common.py
?? uuv_mujoco/v2.2/tools/audit_code_contract_firmware_baro.py
?? uuv_mujoco/v2.2/tools/audit_code_contract_firmware_checks.py
?? uuv_mujoco/v2.2/tools/audit_code_contract_firmware_json_checks.py
?? uuv_mujoco/v2.2/tools/audit_code_contract_firmware_json_sensor.py
?? uuv_mujoco/v2.2/tools/audit_code_contract_firmware_json_servo.py
?? uuv_mujoco/v2.2/tools/audit_code_contract_firmware_rc_checks.py
?? uuv_mujoco/v2.2/tools/audit_code_contract_firmware_servo_output.py
?? uuv_mujoco/v2.2/tools/audit_code_contract_gate_summary.py
?? uuv_mujoco/v2.2/tools/audit_code_contract_markdown_sections.py
?? uuv_mujoco/v2.2/tools/audit_code_contract_matrix.py
?? uuv_mujoco/v2.2/tools/audit_code_contract_matrix_domains.py
?? uuv_mujoco/v2.2/tools/audit_code_contract_matrix_eval.py
?? uuv_mujoco/v2.2/tools/audit_code_contract_matrix_issue.py
?? uuv_mujoco/v2.2/tools/audit_code_contract_paths.py
?? uuv_mujoco/v2.2/tools/audit_code_contract_report.py
?? uuv_mujoco/v2.2/tools/audit_code_contract_runtime_atm_pressure.py
?? uuv_mujoco/v2.2/tools/audit_code_contract_runtime_baro.py
?? uuv_mujoco/v2.2/tools/audit_code_contract_runtime_checks.py
?? uuv_mujoco/v2.2/tools/audit_code_contract_runtime_dynamic_fluidcoef.py
?? uuv_mujoco/v2.2/tools/audit_code_contract_runtime_dynamic_fluidcoef_eval.py
?? uuv_mujoco/v2.2/tools/audit_code_contract_runtime_dynamic_fluidcoef_evidence.py
?? uuv_mujoco/v2.2/tools/audit_code_contract_runtime_flow_coupling.py
?? uuv_mujoco/v2.2/tools/audit_code_contract_runtime_identity.py
?? uuv_mujoco/v2.2/tools/audit_code_contract_runtime_identity_check.py
?? uuv_mujoco/v2.2/tools/audit_code_contract_runtime_identity_eval.py
?? uuv_mujoco/v2.2/tools/audit_code_contract_runtime_identity_inputs.py
?? uuv_mujoco/v2.2/tools/audit_code_contract_runtime_identity_report.py
?? uuv_mujoco/v2.2/tools/audit_code_contract_runtime_identity_sources.py
?? uuv_mujoco/v2.2/tools/audit_code_contract_runtime_json_altitude.py
?? uuv_mujoco/v2.2/tools/audit_code_contract_runtime_plant_input.py
?? uuv_mujoco/v2.2/tools/audit_code_contract_runtime_rc.py
?? uuv_mujoco/v2.2/tools/audit_code_contract_runtime_sensor_io.py
?? uuv_mujoco/v2.2/tools/audit_code_contract_runtime_static_pressure.py
?? uuv_mujoco/v2.2/tools/audit_code_contract_runtime_surface_ext.py
?? uuv_mujoco/v2.2/tools/audit_code_contract_runtime_time.py
?? uuv_mujoco/v2.2/tools/audit_code_contract_runtime_time_sources.py
?? uuv_mujoco/v2.2/tools/audit_code_contract_source_identity.py
?? uuv_mujoco/v2.2/tools/audit_code_contract_sources.py
?? uuv_mujoco/v2.2/tools/audit_code_contract_thruster_conversion.py
?? uuv_mujoco/v2.2/tools/audit_code_contract_thruster_conversion_evidence.py
?? uuv_mujoco/v2.2/tools/audit_code_contract_thruster_conversion_rules.py
?? uuv_mujoco/v2.2/tools/audit_code_contract_thruster_gate_checks.py
?? uuv_mujoco/v2.2/tools/audit_code_contract_types.py
?? uuv_mujoco/v2.2/tools/axis_rc_arm_service.py
?? uuv_mujoco/v2.2/tools/axis_rc_arm_state.py
?? uuv_mujoco/v2.2/tools/axis_rc_cli_args.py
?? uuv_mujoco/v2.2/tools/axis_rc_command_values.py
?? uuv_mujoco/v2.2/tools/axis_rc_contract.py
?? uuv_mujoco/v2.2/tools/axis_rc_health_flags.py
?? uuv_mujoco/v2.2/tools/axis_rc_health_metrics.py
?? uuv_mujoco/v2.2/tools/axis_rc_health_phase.py
?? uuv_mujoco/v2.2/tools/axis_rc_health_phase_rules.py
?? uuv_mujoco/v2.2/tools/axis_rc_health_response_rules.py
?? uuv_mujoco/v2.2/tools/axis_rc_health_rules.py
?? uuv_mujoco/v2.2/tools/axis_rc_health_sign.py
?? uuv_mujoco/v2.2/tools/axis_rc_health_status.py
?? uuv_mujoco/v2.2/tools/axis_rc_manual_messages.py
?? uuv_mujoco/v2.2/tools/axis_rc_messages.py
?? uuv_mujoco/v2.2/tools/axis_rc_metric_math.py
?? uuv_mujoco/v2.2/tools/axis_rc_metrics.py
?? uuv_mujoco/v2.2/tools/axis_rc_mode_service.py
?? uuv_mujoco/v2.2/tools/axis_rc_node.py
?? uuv_mujoco/v2.2/tools/axis_rc_node_callbacks.py
?? uuv_mujoco/v2.2/tools/axis_rc_node_control.py
?? uuv_mujoco/v2.2/tools/axis_rc_node_publish.py
?? uuv_mujoco/v2.2/tools/axis_rc_node_services.py
?? uuv_mujoco/v2.2/tools/axis_rc_node_spin.py
?? uuv_mujoco/v2.2/tools/axis_rc_output_files.py
?? uuv_mujoco/v2.2/tools/axis_rc_override_messages.py
?? uuv_mujoco/v2.2/tools/axis_rc_plot_render.py
?? uuv_mujoco/v2.2/tools/axis_rc_plot_series.py
?? uuv_mujoco/v2.2/tools/axis_rc_plotting.py
?? uuv_mujoco/v2.2/tools/axis_rc_report.py
?? uuv_mujoco/v2.2/tools/axis_rc_sample_channels.py
?? uuv_mujoco/v2.2/tools/axis_rc_sample_motion.py
?? uuv_mujoco/v2.2/tools/axis_rc_sampling.py
?? uuv_mujoco/v2.2/tools/axis_rc_sequence.py
?? uuv_mujoco/v2.2/tools/axis_rc_sequence_env.py
?? uuv_mujoco/v2.2/tools/axis_rc_sequence_phases.py
?? uuv_mujoco/v2.2/tools/axis_rc_service_spin.py
?? uuv_mujoco/v2.2/tools/axis_rc_services.py
?? uuv_mujoco/v2.2/tools/axis_rc_stack_wait.py
?? uuv_mujoco/v2.2/tools/axis_rc_summary_metrics.py
?? uuv_mujoco/v2.2/tools/axis_rc_trigger_service.py
?? uuv_mujoco/v2.2/tools/axis_rc_vehicle_prepare.py
?? uuv_mujoco/v2.2/tools/check_axis_rc_health_contract.py
?? uuv_mujoco/v2.2/tools/check_axis_rc_latency_metrics.py
?? uuv_mujoco/v2.2/tools/check_bridge_live_imports.py
?? uuv_mujoco/v2.2/tools/check_dev_os_compat.py
?? uuv_mujoco/v2.2/tools/check_dynamic_fluidcoef_contract.py
?? uuv_mujoco/v2.2/tools/check_filter_ping360_stl_io.py
?? uuv_mujoco/v2.2/tools/check_fossen_residual_wrench.py
?? uuv_mujoco/v2.2/tools/check_fossen_runtime_builders.py
?? uuv_mujoco/v2.2/tools/check_gui_arm_mode_command_contract.py
?? uuv_mujoco/v2.2/tools/check_gui_backend_selection.py
?? uuv_mujoco/v2.2/tools/check_gui_entry_contract.py
?? uuv_mujoco/v2.2/tools/check_gui_initial_depth_contract.py
?? uuv_mujoco/v2.2/tools/check_gui_pilot_auto_enable_contract.py
?? uuv_mujoco/v2.2/tools/check_gui_pilot_toggle_contract.py
?? uuv_mujoco/v2.2/tools/check_gui_rc_replay_decode.py
?? uuv_mujoco/v2.2/tools/check_gui_readiness_contract.py
?? uuv_mujoco/v2.2/tools/check_gui_ros_python_contract.py
?? uuv_mujoco/v2.2/tools/check_gui_start_contract.py
?? uuv_mujoco/v2.2/tools/check_hydrostatic_buoyancy_points.py
?? uuv_mujoco/v2.2/tools/check_immediate_sensor_replay_reply.py
?? uuv_mujoco/v2.2/tools/check_initial_depth_auto_release_contract.py
?? uuv_mujoco/v2.2/tools/check_initial_hold_pose.py
?? uuv_mujoco/v2.2/tools/check_json_servo_receiver.py
?? uuv_mujoco/v2.2/tools/check_mavlink_message_interval.py
?? uuv_mujoco/v2.2/tools/check_mavros_rcout_publish_policy.py
?? uuv_mujoco/v2.2/tools/check_model_runtime_setup.py
?? uuv_mujoco/v2.2/tools/check_mujoco_velocity_contract.py
?? uuv_mujoco/v2.2/tools/check_odometry_publish_builders.py
?? uuv_mujoco/v2.2/tools/check_physics_contract_geometry.py
?? uuv_mujoco/v2.2/tools/check_physics_runtime_hydrostatic.py
?? uuv_mujoco/v2.2/tools/check_plant_input_gate.py
?? uuv_mujoco/v2.2/tools/check_plant_input_gate_csv.py
?? uuv_mujoco/v2.2/tools/check_rc_frame_contract.py
?? uuv_mujoco/v2.2/tools/check_real_start_measurements.py
?? uuv_mujoco/v2.2/tools/check_ros2_command_payload.py
?? uuv_mujoco/v2.2/tools/check_ros2_dvl_messages.py
?? uuv_mujoco/v2.2/tools/check_ros2_ping360_messages.py
?? uuv_mujoco/v2.2/tools/check_ros2_replay_rcout.py
?? uuv_mujoco/v2.2/tools/check_ros2_sitl_command_override.py
?? uuv_mujoco/v2.2/tools/check_runtime_freshness.py
?? uuv_mujoco/v2.2/tools/check_runtime_readiness_policy.py
?? uuv_mujoco/v2.2/tools/check_sitl_command_link_readiness.py
?? uuv_mujoco/v2.2/tools/check_sitl_external_nav_contract.py
?? uuv_mujoco/v2.2/tools/check_sitl_servo_runtime.py
?? uuv_mujoco/v2.2/tools/check_static_context_publisher.py
?? uuv_mujoco/v2.2/tools/check_thruster_param_loader.py
?? uuv_mujoco/v2.2/tools/check_thruster_performance_curves.py
?? uuv_mujoco/v2.2/tools/control_loop_golden_compare_logic.py
?? uuv_mujoco/v2.2/tools/control_loop_golden_compare_metrics.py
?? uuv_mujoco/v2.2/tools/control_loop_golden_compare_phases.py
?? uuv_mujoco/v2.2/tools/control_loop_golden_compare_records.py
?? uuv_mujoco/v2.2/tools/control_loop_golden_fingerprint.py
?? uuv_mujoco/v2.2/tools/control_loop_golden_fingerprint_fields.py
?? uuv_mujoco/v2.2/tools/control_loop_golden_fingerprint_metadata.py
?? uuv_mujoco/v2.2/tools/control_loop_golden_fingerprint_phase.py
?? uuv_mujoco/v2.2/tools/control_loop_golden_math.py
?? uuv_mujoco/v2.2/tools/control_loop_golden_phase_metrics.py
?? uuv_mujoco/v2.2/tools/control_loop_golden_phase_windows.py
?? uuv_mujoco/v2.2/tools/control_loop_golden_thruster_columns.py
?? uuv_mujoco/v2.2/tools/control_loop_golden_thruster_io.py
?? uuv_mujoco/v2.2/tools/control_loop_golden_thruster_metrics.py
?? uuv_mujoco/v2.2/tools/control_loop_golden_thruster_rows.py
?? uuv_mujoco/v2.2/tools/control_loop_golden_thrusters.py
?? uuv_mujoco/v2.2/tools/dev_os_compat_common.py
?? uuv_mujoco/v2.2/tools/dev_os_compat_display.py
?? uuv_mujoco/v2.2/tools/dev_os_compat_docker.py
?? uuv_mujoco/v2.2/tools/dev_os_compat_exec.py
?? uuv_mujoco/v2.2/tools/dev_os_compat_mjpython.py
?? uuv_mujoco/v2.2/tools/dev_os_compat_mjpython_probe.py
?? uuv_mujoco/v2.2/tools/dev_os_compat_mjpython_result.py
?? uuv_mujoco/v2.2/tools/dev_os_compat_path_checks.py
?? uuv_mujoco/v2.2/tools/dev_os_compat_python_candidates.py
?? uuv_mujoco/v2.2/tools/dev_os_compat_python_checks.py
?? uuv_mujoco/v2.2/tools/dev_os_compat_python_eval.py
?? uuv_mujoco/v2.2/tools/dev_os_compat_python_import_probe.py
?? uuv_mujoco/v2.2/tools/dev_os_compat_python_probe.py
?? uuv_mujoco/v2.2/tools/dev_os_compat_python_runtime.py
?? uuv_mujoco/v2.2/tools/dev_os_compat_python_select.py
?? uuv_mujoco/v2.2/tools/dev_os_compat_ros.py
?? uuv_mujoco/v2.2/tools/dev_os_compat_run_groups.py
?? uuv_mujoco/v2.2/tools/dev_os_compat_runtime.py
?? uuv_mujoco/v2.2/tools/dev_os_compat_sitl_alias.py
?? uuv_mujoco/v2.2/tools/dev_os_compat_sitl_paths.py
?? uuv_mujoco/v2.2/tools/dev_os_compat_system.py
?? uuv_mujoco/v2.2/tools/dev_os_compat_ubuntu.py
?? uuv_mujoco/v2.2/tools/dev_os_compat_viewer.py
?? uuv_mujoco/v2.2/tools/dynamic_fluidcoef_smoke_cases.py
?? uuv_mujoco/v2.2/tools/filter_ping360_component_classify.py
?? uuv_mujoco/v2.2/tools/filter_ping360_component_stats.py
?? uuv_mujoco/v2.2/tools/filter_ping360_stl_components.py
?? uuv_mujoco/v2.2/tools/filter_ping360_stl_geometry.py
?? uuv_mujoco/v2.2/tools/filter_ping360_stl_io.py
?? uuv_mujoco/v2.2/tools/filter_ping360_stl_pipeline.py
?? uuv_mujoco/v2.2/tools/filter_ping360_stl_read.py
?? uuv_mujoco/v2.2/tools/filter_ping360_stl_types.py
?? uuv_mujoco/v2.2/tools/filter_ping360_stl_write.py
?? uuv_mujoco/v2.2/tools/filter_ping360_union_find.py
?? uuv_mujoco/v2.2/tools/fossen_runtime_builder_smoke_cases.py
?? uuv_mujoco/v2.2/tools/gui_pilot_toggle_smoke_fixture.py
?? uuv_mujoco/v2.2/tools/initial_hold_pose_smoke_cases.py
?? uuv_mujoco/v2.2/tools/initial_hold_pose_smoke_fixture.py
?? uuv_mujoco/v2.2/tools/json_servo_receiver_smoke_assertions.py
?? uuv_mujoco/v2.2/tools/json_servo_receiver_smoke_cases.py
?? uuv_mujoco/v2.2/tools/json_servo_receiver_smoke_fixture.py
?? uuv_mujoco/v2.2/tools/live_topic_liveness_check.py
?? uuv_mujoco/v2.2/tools/physics_contract_audit.py
?? uuv_mujoco/v2.2/tools/physics_contract_audit_report.py
?? uuv_mujoco/v2.2/tools/physics_contract_body.py
?? uuv_mujoco/v2.2/tools/physics_contract_body_geometry.py
?? uuv_mujoco/v2.2/tools/physics_contract_buoyancy.py
?? uuv_mujoco/v2.2/tools/physics_contract_depth_state.py
?? uuv_mujoco/v2.2/tools/physics_contract_geometry.py
?? uuv_mujoco/v2.2/tools/physics_contract_model.py
?? uuv_mujoco/v2.2/tools/physics_contract_mujoco.py
?? uuv_mujoco/v2.2/tools/physics_contract_neutral_apply.py
?? uuv_mujoco/v2.2/tools/physics_contract_neutral_buoyancy.py
?? uuv_mujoco/v2.2/tools/physics_contract_neutral_components.py
?? uuv_mujoco/v2.2/tools/physics_contract_neutral_context.py
?? uuv_mujoco/v2.2/tools/physics_contract_neutral_loop.py
?? uuv_mujoco/v2.2/tools/physics_contract_neutral_output.py
?? uuv_mujoco/v2.2/tools/physics_contract_neutral_restoring.py
?? uuv_mujoco/v2.2/tools/physics_contract_neutral_runner.py
?? uuv_mujoco/v2.2/tools/physics_contract_neutral_sim.py
?? uuv_mujoco/v2.2/tools/physics_contract_neutral_timing.py
?? uuv_mujoco/v2.2/tools/physics_contract_profile_runtime.py
?? uuv_mujoco/v2.2/tools/physics_contract_quat.py
?? uuv_mujoco/v2.2/tools/physics_contract_report.py
?? uuv_mujoco/v2.2/tools/physics_contract_report_balance.py
?? uuv_mujoco/v2.2/tools/physics_contract_report_body.py
?? uuv_mujoco/v2.2/tools/physics_contract_report_console.py
?? uuv_mujoco/v2.2/tools/physics_contract_report_hydro.py
?? uuv_mujoco/v2.2/tools/physics_contract_report_io.py
?? uuv_mujoco/v2.2/tools/physics_contract_report_sections.py
?? uuv_mujoco/v2.2/tools/physics_contract_runner.py
?? uuv_mujoco/v2.2/tools/physics_contract_runner_balances.py
?? uuv_mujoco/v2.2/tools/physics_contract_runner_context.py
?? uuv_mujoco/v2.2/tools/physics_contract_runner_depths.py
?? uuv_mujoco/v2.2/tools/physics_contract_runner_outputs.py
?? uuv_mujoco/v2.2/tools/physics_contract_site_geometry.py
?? uuv_mujoco/v2.2/tools/physics_contract_start_depth.py
?? uuv_mujoco/v2.2/tools/physics_contract_types.py
?? uuv_mujoco/v2.2/tools/preflight_ardupilot_classify.py
?? uuv_mujoco/v2.2/tools/preflight_ardupilot_git.py
?? uuv_mujoco/v2.2/tools/preflight_ardupilot_output.py
?? uuv_mujoco/v2.2/tools/real_start_angular_velocity.py
?? uuv_mujoco/v2.2/tools/real_start_attitude_extractors.py
?? uuv_mujoco/v2.2/tools/real_start_baro.py
?? uuv_mujoco/v2.2/tools/real_start_baro_candidates.py
?? uuv_mujoco/v2.2/tools/real_start_baro_row.py
?? uuv_mujoco/v2.2/tools/real_start_builder.py
?? uuv_mujoco/v2.2/tools/real_start_common.py
?? uuv_mujoco/v2.2/tools/real_start_csv.py
?? uuv_mujoco/v2.2/tools/real_start_depth_extractors.py
?? uuv_mujoco/v2.2/tools/real_start_dvl_velocity.py
?? uuv_mujoco/v2.2/tools/real_start_extractors.py
?? uuv_mujoco/v2.2/tools/real_start_geometry.py
?? uuv_mujoco/v2.2/tools/real_start_local_velocity.py
?? uuv_mujoco/v2.2/tools/real_start_measurement_smoke_cases.py
?? uuv_mujoco/v2.2/tools/real_start_output.py
?? uuv_mujoco/v2.2/tools/real_start_pressure_state.py
?? uuv_mujoco/v2.2/tools/real_start_rc_state.py
?? uuv_mujoco/v2.2/tools/real_start_state.py
?? uuv_mujoco/v2.2/tools/real_start_velocity_extractors.py
?? uuv_mujoco/v2.2/tools/real_start_velocity_policy.py
?? uuv_mujoco/v2.2/tools/real_start_velocity_vectors.py
?? uuv_mujoco/v2.2/tools/refactor_inventory.py
?? uuv_mujoco/v2.2/tools/refactor_inventory_analysis.py
?? uuv_mujoco/v2.2/tools/refactor_inventory_paths.py
?? uuv_mujoco/v2.2/tools/refactor_inventory_render.py
?? uuv_mujoco/v2.2/tools/refactor_inventory_scoring.py
?? uuv_mujoco/v2.2/tools/refactor_inventory_symbols.py
?? uuv_mujoco/v2.2/tools/refactor_inventory_types.py
?? uuv_mujoco/v2.2/tools/roll_stability_candidate_catalog.py
?? uuv_mujoco/v2.2/tools/roll_stability_candidate_diagnostics.py
?? uuv_mujoco/v2.2/tools/roll_stability_candidate_ellipsoid.py
?? uuv_mujoco/v2.2/tools/roll_stability_candidate_hydrostatic.py
?? uuv_mujoco/v2.2/tools/roll_stability_candidate_signs.py
?? uuv_mujoco/v2.2/tools/roll_stability_candidate_types.py
?? uuv_mujoco/v2.2/tools/roll_stability_candidates.py
?? uuv_mujoco/v2.2/tools/roll_stability_command_requests.py
?? uuv_mujoco/v2.2/tools/roll_stability_command_send.py
?? uuv_mujoco/v2.2/tools/roll_stability_command_wait.py
?? uuv_mujoco/v2.2/tools/roll_stability_file_edits.py
?? uuv_mujoco/v2.2/tools/roll_stability_launch.py
?? uuv_mujoco/v2.2/tools/roll_stability_launch_process.py
?? uuv_mujoco/v2.2/tools/roll_stability_launch_readiness.py
?? uuv_mujoco/v2.2/tools/roll_stability_math.py
?? uuv_mujoco/v2.2/tools/roll_stability_metrics.py
?? uuv_mujoco/v2.2/tools/roll_stability_pose_metrics.py
?? uuv_mujoco/v2.2/tools/roll_stability_probe.py
?? uuv_mujoco/v2.2/tools/roll_stability_probe_callbacks.py
?? uuv_mujoco/v2.2/tools/roll_stability_probe_commands.py
?? uuv_mujoco/v2.2/tools/roll_stability_probe_rc.py
?? uuv_mujoco/v2.2/tools/roll_stability_probe_runtime.py
?? uuv_mujoco/v2.2/tools/roll_stability_probe_sequence.py
?? uuv_mujoco/v2.2/tools/roll_stability_rc_frame.py
?? uuv_mujoco/v2.2/tools/roll_stability_rc_metrics.py
?? uuv_mujoco/v2.2/tools/roll_stability_rc_spin.py
?? uuv_mujoco/v2.2/tools/roll_stability_runner.py
?? uuv_mujoco/v2.2/tools/roll_stability_score.py
?? uuv_mujoco/v2.2/tools/roll_stability_stimulus.py
?? uuv_mujoco/v2.2/tools/roll_stability_summary.py
?? uuv_mujoco/v2.2/tools/roll_stability_sweep_files.py
?? uuv_mujoco/v2.2/tools/roll_stability_sweep_loop.py
?? uuv_mujoco/v2.2/tools/roll_stability_sweep_paths.py
?? uuv_mujoco/v2.2/tools/roll_stability_wait_loop.py
?? uuv_mujoco/v2.2/tools/ros2_sitl_command_override_smoke_cases.py
?? uuv_mujoco/v2.2/tools/ros2_sitl_command_override_smoke_fixture.py
?? uuv_mujoco/v2.2/tools/run_fast_contract_sanity.py
?? uuv_mujoco/v2.2/tools/runtime_freshness_dirty_checks.py
?? uuv_mujoco/v2.2/tools/runtime_freshness_dirty_paths.py
?? uuv_mujoco/v2.2/tools/runtime_freshness_eval.py
?? uuv_mujoco/v2.2/tools/runtime_freshness_git_probe.py
?? uuv_mujoco/v2.2/tools/runtime_freshness_issue.py
?? uuv_mujoco/v2.2/tools/runtime_freshness_json_io.py
?? uuv_mujoco/v2.2/tools/runtime_freshness_probe.py
?? uuv_mujoco/v2.2/tools/runtime_freshness_report.py
?? uuv_mujoco/v2.2/tools/runtime_freshness_runtime_checks.py
?? uuv_mujoco/v2.2/tools/runtime_freshness_runtime_resolve.py
?? uuv_mujoco/v2.2/tools/runtime_freshness_source_checks.py
?? uuv_mujoco/v2.2/tools/runtime_freshness_version.py
?? uuv_mujoco/v2.2/tools/runtime_freshness_version_checks.py
?? uuv_mujoco/v2.2/tools/runtime_freshness_version_constants.py
?? uuv_mujoco/v2.2/tools/runtime_freshness_version_dirty.py
?? uuv_mujoco/v2.2/tools/runtime_freshness_version_paths.py
?? uuv_mujoco/v2.2/tools/runtime_freshness_version_payload.py
?? uuv_mujoco/v2.2/tools/sitl_external_nav_contract_cases.py
?? uuv_mujoco/v2.2/tools/sitl_external_nav_contract_fixture.py
?? uuv_mujoco/v2.2/tools/thruster_param_loader_smoke_assertions.py
?? uuv_mujoco/v2.2/tools/thruster_param_loader_smoke_cases.py
?? uuv_mujoco/v2.2/tools/thruster_param_loader_smoke_fixture.py
?? uuv_mujoco/v2.2/tools/thruster_param_loader_smoke_io.py
?? uuv_mujoco/v2.2/tools/thruster_performance_curve_smoke_cases.py
?? uuv_mujoco/v2.2/uuv_mujoco/

### PASS: Local ArduPilot source identity

- id: `ardupilot_source_tag`
- conclusion: Local ArduPilot reports 'ArduSub-4.1.2'; inner worktree status is clean. Use a fresh clone only if this changes.
- local evidence:
  - `ardupilot` - git describe: ArduSub-4.1.2
  - `ardupilot` - git status --short: <clean>

### WARN: Top-level ArduPilot gitlink matches the working checkout

- id: `top_level_ardupilot_gitlink`
- conclusion: The ArduPilot source checkout used for this audit is 2dd0bb7d4c85ac48437f139d66df648fc0e1d4ae. The top-level repository records 6271e15bd4a2ef81a0f4ead439cb18188763c271. If these differ, do not commit the gitlink change unless the project intentionally updates the submodule pointer.
- local evidence:
  - `ardupilot` - working checkout commit: 2dd0bb7d4c85ac48437f139d66df648fc0e1d4ae
  - `.` - top-level gitlink: 6271e15bd4a2ef81a0f4ead439cb18188763c271

### PASS: SITL JSON servo backend is raw 16-channel PWM

- id: `json_servo_packet_16_raw_pwm`
- conclusion: Closed-loop plant input must consume the raw JSON servo packet as PWM[0..15].
- official refs: <https://ardupilot.org/dev/docs/sitl-with-JSON.html>
- local evidence:
  - `ardupilot/libraries/SITL/SIM_JSON.h:43` - uint16_t pwm[16]
  - `ardupilot/libraries/SITL/SIM_JSON.cpp:105` - pkt.pwm[i] = input.servos[i]

### PASS: ArduPilot JSON sensor parser has no pressure/altitude key

- id: `json_sensor_no_direct_pressure_or_altitude`
- conclusion: Bar30 cannot be injected by a JSON pressure field in this firmware. The pressure contract must be implemented indirectly through position.z.
- official refs: <https://ardupilot.org/dev/docs/sitl-with-JSON.html>
- local evidence:
  - `ardupilot/libraries/SITL/SIM_JSON.h:100` - SIM_JSON keytable[16]
  - `ardupilot/libraries/SITL/SIM_JSON.h:104` - position key present
  - `ardupilot/libraries/SITL/SIM_JSON.h` - pressure/altitude keys absent from keytable

### PASS: ArduSub SITL Bar30 pressure is derived from altitude built from position.z

- id: `baro_sitl_pressure_from_json_position_z`
- conclusion: For ArduSub, AP_Baro_SITL registers a water barometer and converts SITL altitude to underwater pressure. SIM_Aircraft builds that altitude from JSON position.z.
- official refs: <https://bluerobotics.com/store/sensors-cameras/sensors/bar30-sensor-r1-rp/>
- local evidence:
  - `ardupilot/libraries/AP_Baro/AP_Baro_SITL.cpp:21` - BARO_TYPE_WATER
  - `ardupilot/libraries/AP_Baro/AP_Baro_SITL.cpp:56` - float sim_alt = _sitl->state.altitude
  - `ardupilot/libraries/AP_Baro/AP_Baro_SITL.cpp:120` - SimpleUnderWaterAtmosphere(-sim_alt * 0.001f
  - `ardupilot/libraries/SITL/SIM_Aircraft.cpp:146` - location.alt  = static_cast<int32_t>(home.alt - position.z * 100.0f)

### PASS: SERVO_OUTPUT_RAW is hal.rcout telemetry

- id: `servo_output_raw_halrcout_telemetry`
- conclusion: Controller parity comparison layer is real /mavros/rc/out versus SITL MAVLink SERVO_OUTPUT_RAW, not the high-rate JSON servo backend.
- official refs: <https://mavlink.io/en/messages/common.html#SERVO_OUTPUT_RAW>
- local evidence:
  - `ardupilot/libraries/GCS_MAVLink/GCS_Common.cpp:2664` - void GCS_MAVLINK::send_servo_output_raw()
  - `ardupilot/libraries/GCS_MAVLink/GCS_Common.cpp:2667` - hal.rcout->read(values, 16)

### WARN: Local ArduSub 4.1.2 RC override handler consumes 1..16, not 1..18

- id: `rc_override_local_16_channel_limit`
- conclusion: The official MAVLink message has extension channels beyond 16, but this local ArduSub 4.1.2 handler builds override_data only through chan16_raw. This is acceptable for the current vehicle if active controls stay within C1..C8, but the old 'preserve 1..18' checklist is not true for this firmware.
- official refs: <https://mavlink.io/en/messages/common.html#RC_CHANNELS_OVERRIDE>
- local evidence:
  - `ardupilot/libraries/GCS_MAVLink/GCS_Common.cpp:3239` - packet.chan16_raw
  - `ardupilot/libraries/GCS_MAVLink/GCS_Common.cpp:3248` - for (uint8_t i=8; i<ARRAY_SIZE(override_data); i++)

### PASS: RC override has firmware timeout policy

- id: `rc_override_timeout_policy`
- conclusion: RC override must be streamed faster than RC_OVERRIDE_TIME, whose local default is 3s. A one-shot override is not a valid closed-loop input contract.
- official refs: <https://mavlink.io/en/messages/common.html#RC_CHANNELS_OVERRIDE>
- local evidence:
  - `ardupilot/libraries/RC_Channel/RC_Channel.cpp:351` - override_value = v
  - `ardupilot/libraries/RC_Channel/RC_Channel.cpp:368` - get_override_timeout_ms
  - `ardupilot/libraries/RC_Channel/RC_Channels_VarInfo.h:86` - AP_GROUPINFO("_OVERRIDE_TIME"

### PASS: ArduSub joystick maps RC3 heave, RC4 yaw, RC5 forward, RC6 lateral

- id: `ardusub_joystick_axis_mapping`
- conclusion: The real joy/RC override axis mapping checklist is confirmed in ArduSub code.
- local evidence:
  - `ardupilot/ArduSub/joystick.cpp:126` - RC3 throttle/heave
  - `ardupilot/ArduSub/joystick.cpp:127` - RC4 yaw
  - `ardupilot/ArduSub/joystick.cpp:132` - RC5 forward
  - `ardupilot/ArduSub/joystick.cpp:133` - RC6 lateral

### PASS: Active runtime injects Bar30 contract through JSON position.z frontend match

- id: `active_runtime_baro_contract_frontend_match`
- conclusion: The active runtime converts physical Bar30 pressure/depth into the JSON position.z value that makes AP_Baro_SITL produce the matching water-barometer frontend altitude.
- official refs: <https://ardupilot.org/dev/docs/sitl-with-JSON.html>, <https://bluerobotics.com/store/sensors-cameras/sensors/bar30-sensor-r1-rp/>
- local evidence:
  - `uuv_mujoco/current/sim/contracts/baro.py:66` - sitl_depth_m_for_frontend_match
  - `uuv_mujoco/current/bridge/sitl_contract.py:10` - from sim.contracts.baro
  - `uuv_mujoco/current/bridge/ros2_bridge_config_baro.py:38` - ROS2_UUV_SITL_BARO_DEPTH_CONTRACT", "frontend_match"
  - `uuv_mujoco/current/bridge/ros2_state_estimation.py:12` - from .ros2_state_vertical import
  - `uuv_mujoco/current/bridge/ros2_state_sitl_vertical.py:37` - sitl_contract_depth_m(
  - `uuv_mujoco/current/bridge/ros2_state_sitl_baro.py:38` - sitl_depth_m_for_frontend_match(pressure_pa)
  - `uuv_mujoco/current/bridge/sitl_json_payload.py:27` - json_position[2] = float(vertical_est.depth_m)

### WARN: Active runtime still sends JSON altitude but ArduSub 4.1.2 ignores it

- id: `active_runtime_json_altitude_field_is_compat_only`
- conclusion: The altitude key in the active runtime JSON payload is compatibility/debug data for this firmware. It must not be treated as a controller input contract.
- official refs: <https://ardupilot.org/dev/docs/sitl-with-JSON.html>
- local evidence:
  - `uuv_mujoco/current/bridge/sitl_json_payload.py:31` - "altitude": float(vertical_est.alt_m)
  - `ardupilot/libraries/SITL/SIM_JSON.h` - SIM_JSON keytable has no altitude key

### PASS: /mavros/imu/static_pressure defaults to external Bar30 pressure

- id: `active_runtime_static_pressure_external_bar30`
- conclusion: The ROS output surface uses Bar30 absolute pressure for static_pressure by default. For SITL control, the important path remains JSON position.z -> AP_Baro_SITL.
- official refs: <https://bluerobotics.com/store/sensors-cameras/sensors/bar30-sensor-r1-rp/>
- local evidence:
  - `uuv_mujoco/current/bridge/ros2_bridge_config_static_pressure.py:15` - ROS2_UUV_STATIC_PRESSURE_SOURCE", "external"
  - `uuv_mujoco/current/bridge/ros2_publish_state.py:73` - static_pressure_pa = bar30_pressure_pa

### WARN: /mavros/imu/atm_pressure is not a safe real-bag pressure target

- id: `active_runtime_atm_pressure_excluded_output_surface`
- conclusion: The April 1 real bag has /mavros/imu/atm_pressure around 0.24, not Pa-scale surface pressure. The sim currently publishes Bar30 absolute pressure there, so this topic is excluded from controller parity and plant replay fitting until its real semantics are proven.
- local evidence:
  - `uuv_mujoco/current/bridge/ros2_publish_runtime.py` - mavros_atm_pressure_msg = build_pressure_msg

### PASS: Runtime separates sim-time sensor publishing from wall-time transport polling

- id: `active_runtime_time_contract_sim_publish_wall_transport`
- conclusion: ROS sensor output is gated by monotonic sim time and sensor_dt; sensor-replay can use a monotonic JSON-servo frame clock; MAVLink polling, SERVO_OUTPUT_RAW requests, RC keepalive, and viewer catch-up use wall-clock cadence.
- official refs: <https://mavlink.io/en/messages/common.html#SERVO_OUTPUT_RAW>
- local evidence:
  - `uuv_mujoco/current/bridge/ros2_bridge_publish_timing.py:9` - self.last_pub_t + self.sensor_dt
  - `uuv_mujoco/current/bridge/sitl_sensor_replay_servo_clock.py:21` - _sensor_replay_last_clock_t_s
  - `uuv_mujoco/current/bridge/sitl_sensor_replay_payload_time.py:9` - _sensor_replay_start_on_rc
  - `uuv_mujoco/current/bridge/sitl_json_servo_poll_loop.py:9` - now_wall = time.monotonic()
  - `uuv_mujoco/current/bridge/sitl_mavlink_request_servo_link.py:20` - _sitl_mavlink_servo_hz
  - `uuv_mujoco/current/bridge/sitl_mavlink_request_command_link.py:25` - _sitl_rcout_telemetry_hz
  - `uuv_mujoco/current/bridge/sitl_transport_extnav_scheduler.py:11` - transport._sitl_extnav_scheduler = "sim_time"
  - `uuv_mujoco/current/bridge/sitl_transport_extnav_runtime.py:23` - transport._sitl_extnav_start_wall = time.monotonic()
  - `uuv_mujoco/current/sim/runtime/simulation_step_catchup.py:23` - clocks.next_step_wall += cadence.target_dt
  - `uuv_mujoco/current/sim/runtime/simulation_sensor_catchup.py:22` - clocks.next_sensor_wall += cadence.sensor_dt

### PASS: Sensor input/output surfaces share one MuJoCo snapshot contract

- id: `active_runtime_sensor_io_snapshot_contract`
- conclusion: The active runtime builds one MuJoCo sensor snapshot, sends FRD IMU plus Bar30 pressure/depth to ArduSub JSON SITL, and publishes ROS/MAVROS/DVL observation topics from the same snapshot.
- official refs: <https://ardupilot.org/dev/docs/sitl-with-JSON.html>, <https://bluerobotics.com/store/sensors-cameras/sensors/bar30-sensor-r1-rp/>
- local evidence:
  - `uuv_mujoco/current/bridge/ros2_sitl_sensor_feed.py:25` - Ros2SensorSnapshot(
  - `uuv_mujoco/current/bridge/ros2_sitl_sensor_transport.py:33` - pressure_pa=vertical.bar30_pressure_pa
  - `uuv_mujoco/current/bridge/sitl_json_payload.py:32` - "imu": {
  - `uuv_mujoco/current/bridge/ros2_publish_state.py:73` - static_pressure_pa = bar30_pressure_pa
  - `uuv_mujoco/current/bridge/ros2_publish_core_factories.py:49` - ("depth", build_core_depth_msg)
  - `uuv_mujoco/current/bridge/ros2_publish_mavros_cache_factories.py:40` - ("mavros_static_pressure", build_static_pressure_msg)
  - `uuv_mujoco/current/bridge/ros2_publish_dvl_factories.py:22` - state.dvl_vel_dvl_frd

### PASS: RC override preserves MAVLink 18-channel frame and mirrors /mavros/rc/in

- id: `active_runtime_rc_override_forward_mirror_contract`
- conclusion: The active runtime forwards RC_CHANNELS_OVERRIDE as the first 18 raw channels, uses normalized axes only for local fallback, and mirrors the same raw frame to /mavros/rc/in.
- official refs: <https://mavlink.io/en/messages/common.html#RC_CHANNELS_OVERRIDE>
- local evidence:
  - `uuv_mujoco/current/bridge/ros2_rc_override_frame.py:9` - MAX_RC_OVERRIDE_CHANNELS = 18
  - `uuv_mujoco/current/bridge/ros2_rc_override_forwarding.py:15` - send_rc_override(rc_override_forward_frame(channels))
  - `uuv_mujoco/current/bridge/ros2_rc_override_callback.py:20` - _handle_normalized_cmd(fwd, sway, yaw, heave)
  - `uuv_mujoco/current/bridge/ros2_rc_override_mirror.py:18` - rc_in.channels = rc_override_forward_frame(channels)

### PASS: Plant input is raw JSON SERVO or explicit replay RCOU before thruster conversion

- id: `active_runtime_plant_input_raw_pwm_contract`
- conclusion: Plant replay owns actuator input only for replay_rcout sources; otherwise raw SITL JSON SERVO PWM is passed through the plant-input handler before physical thruster force conversion.
- official refs: <https://ardupilot.org/dev/docs/sitl-with-JSON.html>
- local evidence:
  - `uuv_mujoco/current/bridge/sitl_pwm_source_policy.py:7` - str(source).startswith("replay_rcout")
  - `uuv_mujoco/current/bridge/sitl_pwm_frame_handler.py:25` - dispatch_pwm_callback(self, pwm_values)
  - `uuv_mujoco/current/sim/runtime/thruster_actuator_runtime.py:8` - update_thruster_forces

### PASS: Dynamic ellipsoid fluidcoef changes are explicit velocity-load updates

- id: `active_runtime_dynamic_fluidcoef_contract`
- conclusion: Dynamic MuJoCo fluidcoef is profile/env opt-in, restricted to current-mode fluid geoms, configured from pattern-specific five-coefficient reference rows, and updates the five MuJoCo ellipsoid coefficients from geom-local velocity and angular-rate loads. The accepted clean baseline keeps the dynamic path disabled until a validated HAN/CFD profile enables it.
- official refs: <https://mujoco.readthedocs.io/en/3.3.3/computation/fluid.html>
- local evidence:
  - `uuv_mujoco/current/config/sim_profiles.json:271` - "dynamic_fluidcoef": {
  - `uuv_mujoco/current/config/sim_profiles.json:272` - "active": false
  - `uuv_mujoco/current/sim/runtime/model_runtime_setup.py:65` - fluidcoef_dynamic_setup = build_dynamic_fluidcoef_setup(
  - `uuv_mujoco/current/sim/physics/dynamic_fluidcoef_setup_config.py:41` - env_flag("UUV_DYNAMIC_FLUIDCOEF_ENABLE"
  - `uuv_mujoco/current/sim/physics/dynamic_fluidcoef_setup_config.py:42` - str(fluid_model) == "current"
  - `uuv_mujoco/current/sim/physics/dynamic_fluidcoef_setup_enable.py:12` - dynamic_fluidcoef_enabled_after_patterns
  - `uuv_mujoco/current/sim/physics/dynamic_fluidcoef_setup_rows.py:28` - active_geom_ids
  - `uuv_mujoco/current/sim/physics/dynamic_fluidcoef_pattern_prepare.py:36` - reference_scale.size != 5
  - `uuv_mujoco/current/sim/physics/dynamic_fluidcoef_pattern_prepare.py:75` - matching_fluid_geom_ids(fluid_geom_names, pattern)
  - `uuv_mujoco/current/sim/physics/dynamic_fluidcoef_loads.py:47` - MuJoCo fluidcoef order: blunt, slender, angular, Kutta, Magnus.
  - `uuv_mujoco/current/sim/physics/dynamic_fluidcoef_loads.py:33` - axis_loads = np.clip(np.abs(rel) / ref_speed, 0.0, 1.0)
  - `uuv_mujoco/current/sim/physics/dynamic_fluidcoef_runtime_config.py:39` - configure_dynamic_fluidcoef_update_knobs(runtime, env_float=env_float)
  - `uuv_mujoco/current/sim/physics/dynamic_fluidcoef_runtime_update.py:23` - runtime.next_sim_t += runtime.update_dt
  - `uuv_mujoco/current/sim/physics/dynamic_fluidcoef_runtime_update.py:44` - current_local = geom_rot.T @ runtime.water_current_world
  - `uuv_mujoco/current/sim/physics/dynamic_fluidcoef_runtime_update.py:55` - np.clip(runtime.weights[int(geom_id), :] * coeff_loads, 0.0, 1.0)
  - `uuv_mujoco/current/sim/physics/dynamic_fluidcoef_runtime_update.py:74` - runtime.model.geom_fluid[idx, 1:6]
  - `uuv_mujoco/current/sim/physics/dynamic_fluidcoef_runtime.py:20` - class DynamicFluidcoefRuntime
  - `uuv_mujoco/current/sim/runtime/hydrodynamics_runtime_current.py:21` - model.opt.wind[:] = water_current_world
  - `uuv_mujoco/current/sim/runtime/underwater_wrench_runtime.py:56` - self.update_dynamic_fluidcoef(rel_lin_vel_body, ang_vel_body)

### PASS: Runtime order keeps RC/PWM, thrusters, fluidcoef, and MuJoCo step separated

- id: `active_runtime_integrated_flow_contract`
- conclusion: Each tick spins ROS/transport first, applies raw PWM or direct command targets, updates thruster force on the thruster cadence, updates dynamic fluidcoef inside the underwater wrench phase, then advances MuJoCo. This prevents sensor publishing, RC input, actuator conversion, and dynamic ellipsoid tuning from silently owning the same state surface.
- official refs: <https://ardupilot.org/dev/docs/sitl-with-JSON.html>, <https://mujoco.readthedocs.io/en/3.3.3/computation/fluid.html>
- local evidence:
  - `uuv_mujoco/current/sim/runtime/simulation_step_runtime.py:21` - self.spin_ros_once()
  - `uuv_mujoco/current/sim/runtime/simulation_step_raw_pwm.py:37` - runtime.sitl_servo_runtime.apply_to_targets(
  - `uuv_mujoco/current/sim/runtime/simulation_step_physics.py:16` - owner.update_thruster_forces(thr_dt)
  - `uuv_mujoco/current/sim/runtime/simulation_step_physics.py:26` - owner.mujoco.mj_step(owner.model, owner.data)
  - `uuv_mujoco/current/sim/runtime/underwater_wrench_runtime.py:56` - self.update_dynamic_fluidcoef(rel_lin_vel_body, ang_vel_body)

### PASS: Plant maps final SERVO/JSON PWM once, without reapplying MOT_x_DIRECTION

- id: `thruster_contract_final_pwm_not_mot_direction_again`
- conclusion: ArduSub motor factors and MOT_x_DIRECTION are controller-side. The active runtime maps final PWM into MuJoCo actuator-positive force using only physical mount orientation.
- local evidence:
  - `ardupilot/libraries/AP_Motors/AP_Motors6DOF.cpp:156` - case SUB_FRAME_VECTORED_6DOF:
  - `uuv_mujoco/current/physics/thruster_mapping.py:29` - REAL_ROBOT_MOT_DIRECTIONS
  - `uuv_mujoco/current/physics/thruster_mapping.py:59` - ARDUSUB_VECTORED_6DOF_SERVO_SIGNS

### PASS: Thruster conversion applies timing, motor lag, force curve, and immersion once

- id: `active_runtime_thruster_conversion_contract`
- conclusion: The active runtime schedules thruster updates in sim time, then converts raw normalized targets through one first-order actuator state, one T200/direct or polynomial force model, and one water-immersion scale before writing MuJoCo actuator ctrl.
- official refs: <https://mujoco.readthedocs.io/en/3.3.3/computation/fluid.html>
- local evidence:
  - `uuv_mujoco/current/sim/runtime/physics_step_thruster_callbacks.py:34` - hydro_runtime.thruster_scheduler.due
  - `uuv_mujoco/current/sim/runtime/hydrodynamics_runtime_setup.py:94` - FixedRateSimScheduler(dt=thruster_loop_dt)
  - `uuv_mujoco/current/sim/runtime/thruster_actuator_command.py:16` - runtime.state[name] = first_order_response(
  - `uuv_mujoco/current/sim/runtime/thruster_actuator_forces.py:48` - runtime.data.ctrl[aid] = force
  - `uuv_mujoco/current/sim/runtime/thruster_actuator_wrench.py:26` - runtime.last_reaction_torque_world
  - `uuv_mujoco/current/sim/physics/thruster_force_model.py:16` - def force_from_shaped_command
  - `uuv_mujoco/current/sim/physics/thruster_force_performance.py:11` - def pwm_to_force_from_performance
  - `uuv_mujoco/current/sim/physics/thruster_force_polynomial.py:10` - scaled_polynomial_force
  - `uuv_mujoco/current/sim/runtime/thruster_actuator_immersion.py:20` - site_depth_m = float(runtime.water_surface_z - site_z)

### WARN: Plant replay gate defines safe fitting targets

- id: `plant_replay_gate_safe_targets`
- conclusion: Use exact RCOU input, Bar30, IMU, DVL x/y, and gyro targets before HAN/CFD tuning. DVL z and local_position-style estimator surfaces are not safe primary targets yet.
- official refs: <https://mujoco.readthedocs.io/en/3.3.3/computation/fluid.html>
- local evidence:
  - `UUV-HAN/outputs/validate_proposed_added_mass_full90_20260601/contract_gate.json` - gate summary: {"han_cfd_ready": true, "overall": "warn", "passes": [], "path": "UUV-HAN/outputs/validate_proposed_added_mass_full90_20260601/contract_gate.json", "warnings": []}

### WARN: Contract matrix covers time, sensor I/O, RC I/O, thruster, plant input, and dynamic fluidcoef

- id: `source_contract_matrix_gate`
- conclusion: The source audit includes every required contract axis before controller parity or plant replay tuning. WARN means a known firmware or replay-gate limitation is documented; FAIL means a contract axis is missing or broken.
- official refs: <https://ardupilot.org/dev/docs/sitl-with-JSON.html>, <https://mavlink.io/en/messages/common.html#RC_CHANNELS_OVERRIDE>, <https://mavlink.io/en/messages/common.html#SERVO_OUTPUT_RAW>, <https://mujoco.readthedocs.io/en/3.3.3/computation/fluid.html>, <https://bluerobotics.com/store/sensors-cameras/sensors/bar30-sensor-r1-rp/>
- local evidence:
  - `uuv_mujoco/v2.2/tools/audit_code_contract_matrix.py` - source_identity: WARN (warn:top_level_ardupilot_gitlink)
  - `uuv_mujoco/v2.2/tools/audit_code_contract_matrix.py` - time_phase: PASS (ok)
  - `uuv_mujoco/v2.2/tools/audit_code_contract_matrix.py` - sensor_input_output: PASS (ok)
  - `uuv_mujoco/v2.2/tools/audit_code_contract_matrix.py` - rc_input_output: WARN (warn:rc_override_local_16_channel_limit)
  - `uuv_mujoco/v2.2/tools/audit_code_contract_matrix.py` - controller_output_and_plant_input: PASS (ok)
  - `uuv_mujoco/v2.2/tools/audit_code_contract_matrix.py` - thruster_contract: WARN (warn:plant_replay_gate_safe_targets)
  - `uuv_mujoco/v2.2/tools/audit_code_contract_matrix.py` - dynamic_ellipsoid_fluid: PASS (ok)
