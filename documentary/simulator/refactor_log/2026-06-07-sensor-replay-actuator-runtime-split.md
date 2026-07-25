# Sensor Replay And Thruster Actuator Runtime Split

Date: 2026-06-07

Scope: active runtime `sim/current`.

Changed ownership:

- `bridge/sitl_sensor_replay_frame_policy.py` remains the public frame-selection method.
- `bridge/sitl_sensor_replay_hold_policy.py` owns first-frame, servo-clock-wait, RC-wait, and RC-pre-roll hold behavior.
- `bridge/sitl_sensor_replay_start_policy.py` owns first-RC clock marking, pre-roll start, and replay-time calculation.
- `sim/runtime/thruster_actuator_runtime.py` remains the runtime state/update class.
- `sim/runtime/thruster_actuator_setup.py` owns actuator-site lookup, propeller maps, initial zero maps, force-limit scalar coercion, and diagnostic vector initialization.

Validation:

- Focused compileall for the split sensor replay and thruster actuator modules passed.
- `sensor_replay_frame_policy_smoke PASS`
- `thruster_actuator_runtime_create_smoke PASS`
- Refactor inventory no longer lists `bridge/sitl_sensor_replay_frame_policy.py` or `sim/runtime/thruster_actuator_runtime.py` as top hotspots.

Contract note:

- This is behavior-neutral. The RC-start pre-roll clock, payload timestamp policy, interpolation path, actuator site IDs, propeller joint maps, and initial thruster state maps are preserved.
