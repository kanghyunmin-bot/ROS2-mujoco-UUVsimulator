# Runtime Transport, Dev OS, and DVL Split

Date: 2026-06-10

## Scope

This pass reduces the next active-runtime hotspots while preserving controller
parity and plant-input contracts:

- QGC video streaming: normalized stream state, FFmpeg lifecycle, and frame
  write policy are split behind the same `QgcVideoStreamer` class.
- development OS compatibility: runtime Python probe evaluation, selection
  policy, and Python/MuJoCo result emitters are split so macOS-to-Ubuntu
  migration checks stay inspectable.
- MANUAL_CONTROL transport: frame construction, ArduSub neutral priming, actual
  send, and external-control bookkeeping are split while preserving the
  neutral-first joystick contract.
- sensor/VPD replay loading: common CSV load/sort/error policy is split from
  sensor-frame and native VPD row parsing.
- MAVLink peer readiness: recent-heartbeat fast path, `udpin` peer detection,
  and heartbeat wait loop are split for RC override latency debugging.
- DVL ROS publishing: DVL message factories are split from lazy publish cache.
- ROS bridge runtime: publish/spin failure isolation and shutdown-once behavior
  are split from the public runtime wrapper.

The split is behavior-neutral. It does not change ArduPilot, ArduSub params,
PWM mapping, RC override frame semantics, or MuJoCo hydrodynamics coefficients.

## Contract surfaces rechecked

This pass also rechecked the non-negotiable simulator contracts that must stay
visible during later refactors:

- Time contract: ROS sensor output is gated by simulator time, while MAVLink
  polling, RC keepalive, and `SERVO_OUTPUT_RAW` requests use wall-clock
  transport cadence.
- Sensor I/O contract: Bar30/static-pressure, IMU, `/depth`, `/depth/pose`,
  DVL, and sim-time publication remain separate ROS/SITL output surfaces.
- RC in/out contract: `RC_CHANNELS_OVERRIDE` preserves the raw first 18
  channels, normalized axes are local fallback only, and the raw frame is
  mirrored to `/mavros/rc/in`.
- Plant input contract: raw SITL JSON SERVO or explicit replay RCOU is selected
  before thruster conversion; controller-parity telemetry remains separate from
  plant input.
- Thruster contract: final ArduSub PWM is mapped once into physical actuator
  force and does not reapply ArduSub `MOT_x_DIRECTION`.
- Dynamic fluid contract: MuJoCo ellipsoid `fluidcoef` updates are opt-in,
  restricted to current-mode fluid geoms, and driven by local velocity/angular
  load factors over MuJoCo's five ellipsoid coefficients.

## Files

- `bridge/qgc_video_stream.py`
- `bridge/qgc_video_stream_config.py`
- `bridge/qgc_video_stream_lifecycle.py`
- `bridge/qgc_video_stream_write.py`
- `tools/dev_os_compat_python_runtime.py`
- `tools/dev_os_compat_python_eval.py`
- `tools/dev_os_compat_python_select.py`
- `tools/dev_os_compat_python_checks.py`
- `bridge/sitl_manual_control_runtime.py`
- `bridge/sitl_manual_control_frame.py`
- `bridge/sitl_manual_control_logging.py`
- `bridge/sitl_replay_loaders.py`
- `bridge/sitl_replay_csv_loader.py`
- `bridge/sitl_mavlink_peer.py`
- `bridge/sitl_mavlink_peer_state.py`
- `bridge/sitl_mavlink_peer_wait.py`
- `bridge/ros2_publish_builder_dvl.py`
- `bridge/ros2_publish_dvl_cache.py`
- `bridge/ros2_publish_dvl_factories.py`
- `sim/runtime/ros_bridge_runtime.py`
- `sim/runtime/ros_bridge_runtime_failure.py`
- `sim/runtime/ros_bridge_runtime_publish.py`

## Result

Removed these files from the top structural-complexity inventory:

- `bridge/qgc_video_stream.py`
- `tools/dev_os_compat_python_runtime.py`
- `bridge/sitl_manual_control_runtime.py`
- `bridge/sitl_replay_loaders.py`
- `bridge/sitl_mavlink_peer.py`
- `sim/runtime/ros_bridge_runtime.py`
- `bridge/ros2_publish_builder_dvl.py`

## Validation

```text
compileall: PASS
qgc_video_stream_split_smoke=PASS
check_dev_os_compat.py --headless --json: fail=0, pass=16, warn=2
sitl_manual_control_split_smoke=PASS
sitl_replay_csv_loader_split_smoke=PASS
sitl_mavlink_peer_split_smoke=PASS
ros2_publish_dvl_split_smoke=PASS
ros_bridge_runtime_split_smoke=PASS
gui_readiness_contract=PASS
gui_backend_selection=PASS
runtime_readiness_policy=PASS
rc_frame_contract=PASS
verify_ardusub_thruster_contract.py --quiet: OK
audit_code_contract_sources.py: {"fail": 0, "pass": 15, "warn": 5}
audit_closed_loop_contract.py: real_vs_sitl_mismatches={}, missing_sitl_params=[]
```

Focused contract gate results from
`/private/tmp/uuv_contract_audit_time_sensor_rc_fluid_20260610`:

```text
active_runtime_time_contract_sim_publish_wall_transport: PASS
active_runtime_rc_override_forward_mirror_contract: PASS
active_runtime_plant_input_raw_pwm_contract: PASS
active_runtime_dynamic_fluidcoef_contract: PASS
thruster_contract_final_pwm_not_mot_direction_again: PASS
plant_replay_gate_safe_targets: WARN
```

`plant_replay_gate_safe_targets` remains a warning because it is a target-safety
classification gate, not a source-contract failure. It keeps unsafe fitting
surfaces such as estimator-style local-position outputs visible instead of
treating every logged channel as a valid tuning target.

The two development OS warnings are expected for this shell: Docker daemon is
not connected and ROS2 is not sourced in the current shell.
