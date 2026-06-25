# Static Context, Dev OS, Replay, Ping360, Fossen, and MAVLink Split

Date: 2026-06-10

## Scope

This pass removes the next structural hotspots without changing controller
parity surfaces, RC override semantics, sensor values, plant input, thruster
mapping, or hydrodynamic coefficients.

Split surfaces:

- Static context publishing: one-shot `/tf_static` and low-rate
  `/robot_description` publish policy.
- Development OS Python probing: executable candidate discovery and subprocess
  MuJoCo import probe.
- Immediate sensor-replay JSON replies: gate/timestamp, payload/ExternalNav,
  and counter/log ownership.
- Ping360 beam modeling: angle conversion, local beam directions, MuJoCo
  raycast, and geom-name reflectivity.
- Fossen residual wrench: velocity terms, named damping, and quadratic damping.
- MAVLink message intervals: throttle state and request body for
  `SET_MESSAGE_INTERVAL`.

## Files

- `bridge/ros2_static_context_publisher.py`
- `bridge/ros2_static_context_tf.py`
- `bridge/ros2_static_context_robot_description.py`
- `tools/check_static_context_publisher.py`
- `tools/dev_os_compat_python_probe.py`
- `tools/dev_os_compat_python_candidates.py`
- `tools/dev_os_compat_python_import_probe.py`
- `bridge/sitl_json_replay_reply.py`
- `bridge/sitl_json_replay_reply_gate.py`
- `bridge/sitl_json_replay_reply_payload.py`
- `bridge/sitl_json_replay_reply_log.py`
- `tools/check_immediate_sensor_replay_reply.py`
- `bridge/ping360_beam_model.py`
- `bridge/ping360_angle_math.py`
- `bridge/ping360_beam_directions.py`
- `bridge/ping360_raycast.py`
- `bridge/ping360_reflectivity.py`
- `sim/physics/fossen_residual_wrench.py`
- `sim/physics/fossen_residual_terms.py`
- `sim/physics/fossen_residual_named_damping.py`
- `sim/physics/fossen_residual_quadratic_damping.py`
- `tools/check_fossen_residual_wrench.py`
- `sim/transport/mavlink_message_interval.py`
- `sim/transport/mavlink_message_interval_state.py`
- `sim/transport/mavlink_message_interval_requests.py`
- `tools/check_mavlink_message_interval.py`
- `docs/architecture/ACTIVE_CONTRACT_WORKLIST.md`
- `docs/architecture/SPAGHETTI_AUDIT.md`

## Result

Removed these previous hotspots from the top structural inventory:

- `bridge/ros2_static_context_publisher.py`
- `tools/dev_os_compat_python_probe.py`
- `bridge/sitl_json_replay_reply.py`
- `bridge/ping360_beam_model.py`
- `sim/physics/fossen_residual_wrench.py`
- `sim/transport/mavlink_message_interval.py`

Current top hotspot is now
`physics/sim_profile_hydrostatic_buoyancy_points.py`, followed by GUI replay
and command-readiness parsing surfaces.

## Validation

```text
compileall: PASS
static_context_publisher=PASS
immediate_sensor_replay_reply=PASS
fossen_residual_wrench=PASS
mavlink_message_interval=PASS
beam_directions_local smoke: PASS
check_dev_os_compat.py --help: PASS
runtime_readiness_policy=PASS
gui_readiness_contract=PASS
gui_backend_selection=PASS
rc_frame_contract=PASS
verify_ardusub_thruster_contract.py --quiet: OK
audit_code_contract_sources.py: {"fail": 0, "pass": 16, "warn": 5}
audit_closed_loop_contract.py: real_vs_sitl_mismatches={}, missing_sitl_params=[]
```
