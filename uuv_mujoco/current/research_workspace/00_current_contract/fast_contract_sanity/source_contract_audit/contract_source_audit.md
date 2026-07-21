# Code-Level Contract Source Audit

This report compares official-facing contracts with the local ArduSub 4.1.2 and active MuJoCo runtime source paths.

## Metadata

- repo_root: `/home/robot/uuv_sim_current`
- active_runtime_root: `/home/robot/uuv_sim_current/uuv_mujoco/current`
- compat_v22_root: `/home/robot/uuv_sim_current/uuv_mujoco/current`
- ardupilot_describe: `ArduPilot-4.6.0-beta1-7307-ge6c1a75c7c-dirty`
- ardupilot_status_short: `m modules/gsoap`

## Summary

| status | check | conclusion |
| --- | --- | --- |
| WARN | `active_runtime_alias_current` | Live launch, GUI, Docker, setup, and validation paths must resolve the active runtime through uuv_mujoco/current. The v2.2 directory name is only the compatibility backing directory until a physical rename is done. |
| WARN | `ardupilot_source_tag` | Local ArduPilot reports 'ArduPilot-4.6.0-beta1-7307-ge6c1a75c7c-dirty'; inner worktree status is dirty. Use a fresh clone only if this changes. |
| WARN | `top_level_ardupilot_gitlink` | The ArduPilot source checkout used for this audit is e6c1a75c7cd0c4a47fa3350390f21017430bc4de. The top-level repository records <unknown>. If these differ, do not commit the gitlink change unless the project intentionally updates the submodule pointer. |
| PASS | `json_servo_packet_16_raw_pwm` | Closed-loop plant input must consume the raw JSON servo packet as PWM[0..15]. |
| PASS | `json_sensor_no_direct_pressure_or_altitude` | This ArduPilot revision accepts altitude directly as well as position.z. |
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
| FAIL | `active_runtime_integrated_flow_contract` | Each tick spins ROS/transport first, applies raw PWM or direct command targets, updates thruster force on the thruster cadence, updates dynamic fluidcoef inside the underwater wrench phase, then advances MuJoCo. This prevents sensor publishing, RC input, actuator conversion, and dynamic ellipsoid tuning from silently owning the same state surface. |
| PASS | `thruster_contract_final_pwm_not_mot_direction_again` | ArduSub motor factors and MOT_x_DIRECTION are controller-side. The active runtime maps final PWM into MuJoCo actuator-positive force using only physical mount orientation. |
| PASS | `active_runtime_thruster_conversion_contract` | The active runtime schedules thruster updates in sim time, then converts raw normalized targets through one first-order actuator state, one T200/direct or polynomial force model, and one water-immersion scale before writing MuJoCo actuator ctrl. |
| WARN | `plant_replay_gate_safe_targets` | Use exact RCOU input, Bar30, IMU, DVL x/y, and gyro targets before HAN/CFD tuning. DVL z and local_position-style estimator surfaces are not safe primary targets yet. |
| FAIL | `source_contract_matrix_gate` | The source audit includes every required contract axis before controller parity or plant replay tuning. WARN means a known firmware or replay-gate limitation is documented; FAIL means a contract axis is missing or broken. |

## Evidence

### WARN: Active runtime resolves through uuv_mujoco/current

- id: `active_runtime_alias_current`
- conclusion: Live launch, GUI, Docker, setup, and validation paths must resolve the active runtime through uuv_mujoco/current. The v2.2 directory name is only the compatibility backing directory until a physical rename is done.
- local evidence:
  - `uuv_mujoco/current` - uuv_mujoco/current -> /home/robot/uuv_sim_current/uuv_mujoco/current
  - `uuv_mujoco/current/run_uuv_mujoco.py` - primary runner exists: True
  - `uuv_mujoco/run_mujoco.sh` - root launcher exists/executable: True
  - `uuv_mujoco/start_sitl_mujoco.sh` - root launcher exists/executable: True
  - `uuv_mujoco/start_docker_sitl_mujoco.sh` - root launcher exists/executable: True
  - `uuv_mujoco/reset_sim.sh` - root launcher exists/executable: True
  - `uuv_mujoco/RUNTIME_VERSION.json` - runtime version: {"active_runtime": "uuv_mujoco/current", "active_runtime_label": "current-2026-07-18-unknown", "backing_directory": "uuv_mujoco/v2.2", "compatibility_runner": "uuv_mujoco/current/run_urdf_full.py", "dirty_state": {"active_runtime_dirty_count": 0, "active_runtime_dirty_sample": [], "ardupilot_submodule_status": "", "working_tree_dirty_count": 0, "working_tree_dirty_sample": []}, "freshness_policy": "Packaged from the validated local current workspace for dist3 release 2026.07.13-dist3.", "freshness_status": "warn", "note": "v2.2 is a compatibility directory name. New launch, GUI, setup, and validation paths must resolve through uuv_mujoco/current.", "origin_uuv_sim_head": "", "primary_runner": "uuv_mujoco/current/run_uuv_mujoco.py", "root_launchers": {"docker_sitl_mujoco": "uuv_mujoco/start_docker_sitl_mujoco.sh", "mujoco": "uuv_mujoco/run_mujoco.sh", "reset": "uuv_mujoco/reset_sim.sh", "sitl_mujoco": "uuv_mujoco/start_sitl_mujoco.sh"}, "schema": 1, "source_audit_check": "active_runtime_alias_current", "source_branch": "unknown", "source_head": "", "source_remote": "https://github.com/kanghyunmin-bot/ROS2-mujoco-UUVsimulator.git", "status": "current", "updated_at": "2026-07-18"}
  - `uuv_mujoco/current/tools/check_runtime_freshness.py` - freshness checker exists: True
  - `uuv_mujoco/*.sh` - root launchers call freshness checker: True
  - `.` - active git branch: <unknown>
  - `.` - active git HEAD: <unknown>
  - `.` - origin/uuv_sim HEAD: <unknown>
  - `.` - active runtime dirty paths: <clean>

### WARN: Local ArduPilot source identity

- id: `ardupilot_source_tag`
- conclusion: Local ArduPilot reports 'ArduPilot-4.6.0-beta1-7307-ge6c1a75c7c-dirty'; inner worktree status is dirty. Use a fresh clone only if this changes.
- local evidence:
  - `ardupilot` - git describe: ArduPilot-4.6.0-beta1-7307-ge6c1a75c7c-dirty
  - `ardupilot` - git status --short: m modules/gsoap

### WARN: Top-level ArduPilot gitlink matches the working checkout

- id: `top_level_ardupilot_gitlink`
- conclusion: The ArduPilot source checkout used for this audit is e6c1a75c7cd0c4a47fa3350390f21017430bc4de. The top-level repository records <unknown>. If these differ, do not commit the gitlink change unless the project intentionally updates the submodule pointer.
- local evidence:
  - `ardupilot` - working checkout commit: e6c1a75c7cd0c4a47fa3350390f21017430bc4de
  - `.` - top-level gitlink: <unknown>

### PASS: SITL JSON servo backend is raw 16-channel PWM

- id: `json_servo_packet_16_raw_pwm`
- conclusion: Closed-loop plant input must consume the raw JSON servo packet as PWM[0..15].
- official refs: <https://ardupilot.org/dev/docs/sitl-with-JSON.html>
- local evidence:
  - `ardupilot/libraries/SITL/SIM_JSON.h:49` - uint16_t pwm[16]
  - `ardupilot/libraries/SITL/SIM_JSON.cpp:114` - pkt.pwm[i] = input.servos[i]

### PASS: ArduPilot JSON sensor vertical input contract is recognized

- id: `json_sensor_no_direct_pressure_or_altitude`
- conclusion: This ArduPilot revision accepts altitude directly as well as position.z.
- official refs: <https://ardupilot.org/dev/docs/sitl-with-JSON.html>
- local evidence:
  - `ardupilot/libraries/SITL/SIM_JSON.h:122` - SIM_JSON keytable
  - `ardupilot/libraries/SITL/SIM_JSON.h:135` - position key present
  - `ardupilot/libraries/SITL/SIM_JSON.h` - altitude key present

### PASS: ArduSub SITL Bar30 pressure is derived from altitude built from position.z

- id: `baro_sitl_pressure_from_json_position_z`
- conclusion: For ArduSub, AP_Baro_SITL registers a water barometer and converts SITL altitude to underwater pressure. SIM_Aircraft builds that altitude from JSON position.z.
- official refs: <https://bluerobotics.com/store/sensors-cameras/sensors/bar30-sensor-r1-rp/>
- local evidence:
  - `ardupilot/libraries/AP_Baro/AP_Baro_SITL.cpp:21` - BARO_TYPE_WATER
  - `ardupilot/libraries/AP_Baro/AP_Baro_SITL.cpp:62` - float sim_alt = _sitl->state.altitude
  - `ardupilot/libraries/AP_Baro/AP_Baro_SITL.cpp:127` - SimpleUnderWaterAtmosphere(-sim_alt * 0.001f
  - `ardupilot/libraries/SITL/SIM_Aircraft.cpp:184` - location.alt  = static_cast<int32_t>(home.alt - position.z * 100.0f)

### PASS: SERVO_OUTPUT_RAW is hal.rcout telemetry

- id: `servo_output_raw_halrcout_telemetry`
- conclusion: Controller parity comparison layer is real /mavros/rc/out versus SITL MAVLink SERVO_OUTPUT_RAW, not the high-rate JSON servo backend.
- official refs: <https://mavlink.io/en/messages/common.html#SERVO_OUTPUT_RAW>
- local evidence:
  - `ardupilot/libraries/GCS_MAVLink/GCS_Common.cpp:3401` - void GCS_MAVLINK::send_servo_output_raw()
  - `ardupilot/libraries/GCS_MAVLink/GCS_Common.cpp:3415` - HAL RC output read

### WARN: Local ArduSub 4.1.2 RC override handler consumes 1..16, not 1..18

- id: `rc_override_local_16_channel_limit`
- conclusion: The official MAVLink message has extension channels beyond 16, but this local ArduSub 4.1.2 handler builds override_data only through chan16_raw. This is acceptable for the current vehicle if active controls stay within C1..C8, but the old 'preserve 1..18' checklist is not true for this firmware.
- official refs: <https://mavlink.io/en/messages/common.html#RC_CHANNELS_OVERRIDE>
- local evidence:
  - `ardupilot/libraries/GCS_MAVLink/GCS_Common.cpp:4223` - packet.chan16_raw
  - `ardupilot/libraries/GCS_MAVLink/GCS_Common.cpp:4232` - for (uint8_t i=8; i<ARRAY_SIZE(override_data); i++)

### PASS: RC override has firmware timeout policy

- id: `rc_override_timeout_policy`
- conclusion: RC override must be streamed faster than RC_OVERRIDE_TIME, whose local default is 3s. A one-shot override is not a valid closed-loop input contract.
- official refs: <https://mavlink.io/en/messages/common.html#RC_CHANNELS_OVERRIDE>
- local evidence:
  - `ardupilot/libraries/RC_Channel/RC_Channel.cpp:548` - override_value = v
  - `ardupilot/libraries/RC_Channel/RC_Channel.cpp:569` - get_override_timeout_ms
  - `ardupilot/libraries/RC_Channel/RC_Channels_VarInfo.h:90` - AP_GROUPINFO("_OVERRIDE_TIME"

### PASS: ArduSub joystick maps RC3 heave, RC4 yaw, RC5 forward, RC6 lateral

- id: `ardusub_joystick_axis_mapping`
- conclusion: The real joy/RC override axis mapping checklist is confirmed in ArduSub code.
- local evidence:
  - `ardupilot/ArduSub/joystick.cpp:122` - RC3 throttle/heave
  - `ardupilot/ArduSub/joystick.cpp:123` - RC4 yaw
  - `ardupilot/ArduSub/joystick.cpp:128` - RC5 forward
  - `ardupilot/ArduSub/joystick.cpp:129` - RC6 lateral

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
  - `uuv_mujoco/current/sim/runtime/simulation_step_catchup.py:27` - clocks.next_step_wall += cadence.target_dt
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
  - `uuv_mujoco/current/bridge/ros2_rc_override_forwarding.py:31` - send_rc_override(rc_override_forward_frame(channels))
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
  - `uuv_mujoco/current/config/sim_profiles.json:185` - "dynamic_fluidcoef": {
  - `uuv_mujoco/current/config/sim_profiles.json:122` - "active": false
  - `uuv_mujoco/current/sim/runtime/model_runtime_setup.py:72` - fluidcoef_dynamic_setup = build_dynamic_fluidcoef_setup(
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
  - `uuv_mujoco/current/sim/runtime/underwater_wrench_runtime.py:57` - self.update_dynamic_fluidcoef(rel_lin_vel_body, ang_vel_body)

### FAIL: Runtime order keeps RC/PWM, thrusters, fluidcoef, and MuJoCo step separated

- id: `active_runtime_integrated_flow_contract`
- conclusion: Each tick spins ROS/transport first, applies raw PWM or direct command targets, updates thruster force on the thruster cadence, updates dynamic fluidcoef inside the underwater wrench phase, then advances MuJoCo. This prevents sensor publishing, RC input, actuator conversion, and dynamic ellipsoid tuning from silently owning the same state surface.
- official refs: <https://ardupilot.org/dev/docs/sitl-with-JSON.html>, <https://mujoco.readthedocs.io/en/3.3.3/computation/fluid.html>
- local evidence:
  - `uuv_mujoco/current/sim/runtime/simulation_step_runtime.py:26` - self.spin_ros_once()
  - `uuv_mujoco/current/sim/runtime/simulation_step_raw_pwm.py:42` - runtime.sitl_servo_runtime.apply_to_targets(
  - `uuv_mujoco/current/sim/runtime/simulation_step_physics.py:21` - owner.update_thruster_forces(thr_dt)
  - `uuv_mujoco/current/sim/runtime/simulation_step_physics.py:38` - owner.mujoco.mj_step(owner.model, owner.data)
  - `uuv_mujoco/current/sim/runtime/underwater_wrench_runtime.py:57` - self.update_dynamic_fluidcoef(rel_lin_vel_body, ang_vel_body)

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
  - `UUV-HAN/outputs/<missing contract gate>` - gate summary: {}

### FAIL: Contract matrix covers time, sensor I/O, RC I/O, thruster, plant input, and dynamic fluidcoef

- id: `source_contract_matrix_gate`
- conclusion: The source audit includes every required contract axis before controller parity or plant replay tuning. WARN means a known firmware or replay-gate limitation is documented; FAIL means a contract axis is missing or broken.
- official refs: <https://ardupilot.org/dev/docs/sitl-with-JSON.html>, <https://mavlink.io/en/messages/common.html#RC_CHANNELS_OVERRIDE>, <https://mavlink.io/en/messages/common.html#SERVO_OUTPUT_RAW>, <https://mujoco.readthedocs.io/en/3.3.3/computation/fluid.html>, <https://bluerobotics.com/store/sensors-cameras/sensors/bar30-sensor-r1-rp/>
- local evidence:
  - `uuv_mujoco/current/tools/audit_code_contract_matrix.py` - source_identity: WARN (warn:ardupilot_source_tag; warn:top_level_ardupilot_gitlink; warn:active_runtime_alias_current)
  - `uuv_mujoco/current/tools/audit_code_contract_matrix.py` - time_phase: PASS (ok)
  - `uuv_mujoco/current/tools/audit_code_contract_matrix.py` - sensor_input_output: PASS (ok)
  - `uuv_mujoco/current/tools/audit_code_contract_matrix.py` - rc_input_output: WARN (warn:rc_override_local_16_channel_limit)
  - `uuv_mujoco/current/tools/audit_code_contract_matrix.py` - controller_output_and_plant_input: PASS (ok)
  - `uuv_mujoco/current/tools/audit_code_contract_matrix.py` - thruster_contract: WARN (warn:plant_replay_gate_safe_targets)
  - `uuv_mujoco/current/tools/audit_code_contract_matrix.py` - dynamic_ellipsoid_fluid: FAIL (fail:active_runtime_integrated_flow_contract)

