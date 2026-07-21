# Code-Level Contract Source Audit

This report compares official-facing contracts with the local ArduSub 4.1.2 and v2.2 source paths.

## Metadata

- repo_root: `/Users/kanghyunmin/Desktop/uuv_sim`
- v22_root: `/Users/kanghyunmin/Desktop/uuv_sim/uuv_mujoco/v2.2`
- ardupilot_describe: `ArduSub-4.1.2`
- ardupilot_status_short: `<clean>`

## Summary

| status | check | conclusion |
| --- | --- | --- |
| PASS | `ardupilot_source_tag` | Local ArduPilot reports 'ArduSub-4.1.2'; inner worktree status is clean. Use a fresh clone only if this changes. |
| WARN | `top_level_ardupilot_gitlink` | The ArduPilot source checkout used for this audit is 2dd0bb7d4c85ac48437f139d66df648fc0e1d4ae. The top-level repository records 6271e15bd4a2ef81a0f4ead439cb18188763c271. If these differ, do not commit the gitlink change unless the project intentionally updates the submodule pointer. |
| PASS | `json_servo_packet_16_raw_pwm` | Closed-loop plant input must consume the raw JSON servo packet as PWM[0..15]. |
| PASS | `json_sensor_no_direct_pressure_or_altitude` | Bar30 cannot be injected by a JSON pressure field in this firmware. The pressure contract must be implemented indirectly through position.z. |
| PASS | `baro_sitl_pressure_from_json_position_z` | For ArduSub, AP_Baro_SITL registers a water barometer and converts SITL altitude to underwater pressure. SIM_Aircraft builds that altitude from JSON position.z. |
| PASS | `servo_output_raw_halrcout_telemetry` | Controller parity comparison layer is real /mavros/rc/out versus SITL MAVLink SERVO_OUTPUT_RAW, not the high-rate JSON servo backend. |
| WARN | `rc_override_local_16_channel_limit` | The official MAVLink message has extension channels beyond 16, but this local ArduSub 4.1.2 handler builds override_data only through chan16_raw. This is acceptable for the current vehicle if active controls stay within C1..C8, but the old 'preserve 1..18' checklist is not true for this firmware. |
| PASS | `rc_override_timeout_policy` | RC override must be streamed faster than RC_OVERRIDE_TIME, whose local default is 3s. A one-shot override is not a valid closed-loop input contract. |
| PASS | `ardusub_joystick_axis_mapping` | The real joy/RC override axis mapping checklist is confirmed in ArduSub code. |
| PASS | `v22_baro_contract_frontend_match` | v2.2 converts physical Bar30 pressure/depth into the JSON position.z value that makes AP_Baro_SITL produce the matching water-barometer frontend altitude. |
| WARN | `v22_json_altitude_field_is_compat_only` | The altitude key in the v2.2 JSON payload is compatibility/debug data for this firmware. It must not be treated as a controller input contract. |
| PASS | `v22_static_pressure_external_bar30` | The ROS output surface uses Bar30 absolute pressure for static_pressure by default. For SITL control, the important path remains JSON position.z -> AP_Baro_SITL. |
| WARN | `v22_atm_pressure_excluded_output_surface` | The April 1 real bag has /mavros/imu/atm_pressure around 0.24, not Pa-scale surface pressure. The sim currently publishes Bar30 absolute pressure there, so this topic is excluded from controller parity and plant replay fitting until its real semantics are proven. |
| PASS | `thruster_contract_final_pwm_not_mot_direction_again` | ArduSub motor factors and MOT_x_DIRECTION are controller-side. v2.2 maps final PWM into MuJoCo actuator-positive force using only physical mount orientation. |
| WARN | `plant_replay_gate_safe_targets` | Use exact RCOU input, Bar30, IMU, DVL x/y, and gyro targets before HAN/CFD tuning. DVL z and local_position-style estimator surfaces are not safe primary targets yet. |

## Evidence

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

### PASS: v2.2 injects Bar30 contract through JSON position.z frontend match

- id: `v22_baro_contract_frontend_match`
- conclusion: v2.2 converts physical Bar30 pressure/depth into the JSON position.z value that makes AP_Baro_SITL produce the matching water-barometer frontend altitude.
- official refs: <https://ardupilot.org/dev/docs/sitl-with-JSON.html>, <https://bluerobotics.com/store/sensors-cameras/sensors/bar30-sensor-r1-rp/>
- local evidence:
  - `uuv_mujoco/v2.2/sim/contracts/baro.py:66` - sitl_depth_m_for_frontend_match
  - `uuv_mujoco/v2.2/bridge/sitl_contract.py:10` - from sim.contracts.baro
  - `uuv_mujoco/v2.2/bridge/ros2_bridge.py:270` - ROS2_UUV_SITL_BARO_DEPTH_CONTRACT", "frontend_match"
  - `uuv_mujoco/v2.2/bridge/ros2_bridge.py:2209` - sitl_depth_m_for_frontend_match(pressure_pa)
  - `uuv_mujoco/v2.2/bridge/sitl_transport.py:2739` - json_position[2] = float(vertical_est.depth_m)

### WARN: v2.2 still sends JSON altitude but ArduSub 4.1.2 ignores it

- id: `v22_json_altitude_field_is_compat_only`
- conclusion: The altitude key in the v2.2 JSON payload is compatibility/debug data for this firmware. It must not be treated as a controller input contract.
- official refs: <https://ardupilot.org/dev/docs/sitl-with-JSON.html>
- local evidence:
  - `uuv_mujoco/v2.2/bridge/sitl_transport.py:2743` - "altitude": float(vertical_est.alt_m)
  - `ardupilot/libraries/SITL/SIM_JSON.h` - SIM_JSON keytable has no altitude key

### PASS: /mavros/imu/static_pressure defaults to external Bar30 pressure

- id: `v22_static_pressure_external_bar30`
- conclusion: The ROS output surface uses Bar30 absolute pressure for static_pressure by default. For SITL control, the important path remains JSON position.z -> AP_Baro_SITL.
- official refs: <https://bluerobotics.com/store/sensors-cameras/sensors/bar30-sensor-r1-rp/>
- local evidence:
  - `uuv_mujoco/v2.2/bridge/ros2_bridge.py:391` - ROS2_UUV_STATIC_PRESSURE_SOURCE", "external"
  - `uuv_mujoco/v2.2/bridge/ros2_bridge.py:2577` - static_pressure_pa = bar30_pressure_pa

### WARN: /mavros/imu/atm_pressure is not a safe real-bag pressure target

- id: `v22_atm_pressure_excluded_output_surface`
- conclusion: The April 1 real bag has /mavros/imu/atm_pressure around 0.24, not Pa-scale surface pressure. The sim currently publishes Bar30 absolute pressure there, so this topic is excluded from controller parity and plant replay fitting until its real semantics are proven.
- local evidence:
  - `uuv_mujoco/v2.2/bridge/ros2_bridge.py` - mavros_atm_pressure_msg = self._build_pressure(stamp, bar30_pressure_pa)

### PASS: Plant maps final SERVO/JSON PWM once, without reapplying MOT_x_DIRECTION

- id: `thruster_contract_final_pwm_not_mot_direction_again`
- conclusion: ArduSub motor factors and MOT_x_DIRECTION are controller-side. v2.2 maps final PWM into MuJoCo actuator-positive force using only physical mount orientation.
- local evidence:
  - `ardupilot/libraries/AP_Motors/AP_Motors6DOF.cpp:156` - case SUB_FRAME_VECTORED_6DOF:
  - `uuv_mujoco/v2.2/physics/thruster_mapping.py:29` - REAL_ROBOT_MOT_DIRECTIONS
  - `uuv_mujoco/v2.2/physics/thruster_mapping.py:59` - ARDUSUB_VECTORED_6DOF_SERVO_SIGNS

### WARN: Plant replay gate defines safe fitting targets

- id: `plant_replay_gate_safe_targets`
- conclusion: Use exact RCOU input, Bar30, IMU, DVL x/y, and gyro targets before HAN/CFD tuning. DVL z and local_position-style estimator surfaces are not safe primary targets yet.
- official refs: <https://mujoco.readthedocs.io/en/3.3.3/computation/fluid.html>
- local evidence:
  - `UUV-HAN/outputs/validate_proposed_added_mass_full90_20260601/contract_gate.json` - gate summary: {"han_cfd_ready": true, "overall": "warn", "passes": [], "path": "UUV-HAN/outputs/validate_proposed_added_mass_full90_20260601/contract_gate.json", "warnings": []}
