# 2026-06-11 Time/Sensor/RC/Thruster/Fluid Contract Tightening

## Scope

- Continued the v2.2 cleanup with the contract axes that matter for parity:
  time/phase, sensor input/output, RC input/output, plant input, thruster conversion,
  and dynamic MuJoCo built-in ellipsoid `fluidcoef`.
- Kept changes inside `uuv_mujoco/v2.2`.
- Did not edit ArduPilot source or intentionally change the physical model coefficients.

## Changes

- Split ExternalNav transport configuration into focused modules:
  - `bridge/sitl_transport_extnav_base.py`
  - `bridge/sitl_transport_extnav_scheduler.py`
  - `bridge/sitl_transport_extnav_runtime.py`
  - `bridge/sitl_transport_extnav_logging.py`
  - `bridge/sitl_transport_extnav_config.py` remains the compatibility facade.
- Extended source-level time-contract audit to include ExternalNav scheduler selection:
  sim-time when replay/native VPD owns frames, wall-time otherwise, plus wall-clock
  ExternalNav watchdog state.
- Strengthened dynamic `fluidcoef` audit so PASS now requires:
  - profile/env opt-in,
  - `current`/built-in MuJoCo ellipsoid mode restriction,
  - active fluid geom matching,
  - five-coefficient reference rows,
  - velocity/angular-rate load math,
  - update cadence via `runtime.update_dt`,
  - direct writes to `model.geom_fluid[:, 1:6]`,
  - built-in current coupling through `model.opt.wind`,
  - single-owner underwater wrench calling the dynamic update.
- Refactored the strengthened dynamic `fluidcoef` predicate into focused predicate helpers
  after it became a new audit hotspot.
- Split GUI pilot-control toggle flow into `gui/control_pilot_toggle.py` and added
  `tools/check_gui_pilot_toggle_contract.py`.
- Fixed a GUI pilot toggle bug: `control_pilot_mixin.py` referenced
  `GUI_PILOT_CONTROL_MODE` without importing it. The new smoke invokes the toggle path
  directly so this class of runtime-only NameError is caught.
- Split `gui/widgets.py` virtual-joystick runtime wiring into
  `gui/virtual_joystick_runtime.py`, keeping the widget class focused on UI construction.
- Split roll-stability MAVROS wait-loop primitives into
  `tools/roll_stability_wait_loop.py`, so arm/mode readiness, neutral keepalive, and
  spin cadence use the same small deadline helpers.
- Split real-start required payload assembly into
  `sim/runtime/real_start_payload_required.py`. The public payload builder is still
  unchanged, but target loading, measurement evaluation, and latch resolution are now
  isolated for initial-state/ready-contract debugging.
- Split global thruster parameter field handling into
  `sim/physics/thruster_param_global_fields.py`. Scalar fields, global gain clamps, and
  polynomial curve parsing are now separate without changing JSON semantics or values.
- Split vertical thruster x-lever scaling primitives into
  `sim/physics/actuator_geometry_vertical_scale.py`, keeping mount/effectiveness geometry
  changes auditable without changing the scale clamp or lever-arm formula.
- Split direct-command fallback state operations into
  `sim/runtime/command_state_store.py`. This keeps local fallback update/normalization/
  timeout math separate from the `RuntimeCommandState` facade; SITL authority policy is
  unchanged.
- Split MuJoCo kinematic array validation and read fallbacks into
  `bridge/ros2_state_kinematics_arrays.py`,
  `bridge/ros2_state_site_read.py`,
  `bridge/ros2_state_body_velocity_read.py`, and
  `bridge/ros2_state_object_velocity_read.py`, preserving existing finite-check/fallback
  behavior while making sensor snapshot velocity/position reads easier to audit.
- Split passive viewer catch-up timing into
  `sim/runtime/simulation_step_catchup.py`,
  `sim/runtime/simulation_sensor_catchup.py`, and
  `sim/runtime/simulation_viewer_sleep.py`. The source audit now checks step and sensor
  catch-up files separately so wall-clock viewer cadence and ROS sensor publish cadence
  remain explicit.
- Split GUI RC frame padding/sanitization into `gui/node_rc_frame.py`. GUI RCOut remains
  raw padded feedback; GUI RCIn mirror display still sanitizes override marker values.
- Split real-start velocity extraction into
  `tools/real_start_dvl_velocity.py`,
  `tools/real_start_local_velocity.py`,
  `tools/real_start_angular_velocity.py`,
  `tools/real_start_velocity_vectors.py`, and
  `tools/real_start_velocity_policy.py`, preserving DVL/local/fallback priority for
  initial-state contract generation.
- Split JSON-SITL servo receiver IO into
  `sim/transport/json_servo_receiver_bind.py`,
  `sim/transport/json_servo_receiver_recv.py`,
  `sim/transport/json_servo_receiver_send.py`, and
  `sim/transport/json_servo_receiver_close.py`. The facade remains compatible while raw
  SERVO packet receive/decode and send bookkeeping are separately auditable.
- Split SITL `SERVO_OUTPUT_RAW` MAVLink interval requests into
  `bridge/sitl_mavlink_request_servo_link.py` and
  `bridge/sitl_mavlink_request_command_link.py`. The source audit now checks the
  high-rate servo-link request and command-link RCOU telemetry request independently.
- Split thruster force conversion into
  `sim/physics/thruster_force_performance.py` and
  `sim/physics/thruster_force_polynomial.py`. The public selector is unchanged, while
  source audit now checks T200/direct performance curves and polynomial fallback
  separately.

## Validation

```text
python3 -m compileall -q uuv_mujoco/v2.2/tools uuv_mujoco/v2.2/bridge uuv_mujoco/v2.2/sim uuv_mujoco/v2.2/physics
PASS

python3 uuv_mujoco/v2.2/tools/check_sitl_external_nav_contract.py
sitl_external_nav_contract=PASS

python3 uuv_mujoco/v2.2/tools/check_sitl_command_link_readiness.py
sitl_command_link_readiness=PASS

python3 uuv_mujoco/v2.2/tools/check_dynamic_fluidcoef_contract.py
dynamic_fluidcoef_contract=PASS

python3 uuv_mujoco/v2.2/tools/check_gui_pilot_toggle_contract.py
gui_pilot_toggle_contract=PASS

python3 uuv_mujoco/v2.2/tools/audit_code_contract_sources.py --out-dir /private/tmp/uuv_contract_audit_time_dynamic_axes_20260611
{"fail": 0, "pass": 18, "warn": 6}

python3 uuv_mujoco/v2.2/tools/audit_code_contract_sources.py --out-dir /private/tmp/uuv_contract_axes_20260611
{"fail": 0, "pass": 18, "warn": 6}

python3 uuv_mujoco/v2.2/tools/check_real_start_measurements.py
real_start_measurements=PASS

python3 uuv_mujoco/v2.2/tools/check_gui_initial_depth_contract.py
gui_initial_depth_contract=PASS

python3 uuv_mujoco/v2.2/tools/check_thruster_param_loader.py
thruster_param_loader=PASS

python3 uuv_mujoco/v2.2/tools/check_thruster_performance_curves.py
thruster_performance_curves=PASS

python3 uuv_mujoco/v2.2/tools/check_physics_contract_geometry.py
physics_contract_geometry=PASS

python3 uuv_mujoco/v2.2/tools/check_physics_runtime_hydrostatic.py
physics_runtime_hydrostatic=PASS

python3 uuv_mujoco/v2.2/tools/check_ros2_command_payload.py
ros2_command_payload=PASS

python3 uuv_mujoco/v2.2/tools/check_runtime_readiness_policy.py
runtime_readiness_policy=PASS

python3 uuv_mujoco/v2.2/tools/check_odometry_publish_builders.py
odometry_publish_builders=PASS

python3 uuv_mujoco/v2.2/tools/check_ros2_dvl_messages.py
ros2_dvl_messages=PASS

python3 uuv_mujoco/v2.2/tools/check_gui_backend_selection.py
gui_backend_selection=PASS

python3 uuv_mujoco/v2.2/tools/check_rc_frame_contract.py
rc_frame_contract=PASS

python3 uuv_mujoco/v2.2/tools/check_mavlink_message_interval.py
mavlink_message_interval=PASS

python3 uuv_mujoco/v2.2/tools/check_sitl_command_link_readiness.py
sitl_command_link_readiness=PASS

python3 -m compileall -q uuv_mujoco/v2.2/tools/axis_rc_arm_service.py uuv_mujoco/v2.2/tools/axis_rc_arm_state.py
PASS

python3 uuv_mujoco/v2.2/tools/check_dynamic_fluidcoef_contract.py
dynamic_fluidcoef_contract=PASS

python3 uuv_mujoco/v2.2/tools/check_thruster_performance_curves.py
thruster_performance_curves=PASS

python3 uuv_mujoco/v2.2/tools/audit_code_contract_sources.py --out-dir /private/tmp/uuv_contract_axes_20260611
{"fail": 0, "pass": 18, "warn": 6}
```

## Contract Notes

- Time contract is still intentionally split:
  ROS sensor publishing and replay frame clocks are sim/replay-time governed;
  MAVLink polling, RC keepalive, SERVO request loops, viewer catch-up, and ExternalNav
  watchdog windows are wall-clock governed.
- Sensor I/O contract remains one MuJoCo snapshot feeding both SITL JSON sensor payloads
  and ROS/MAVROS/DVL publication surfaces.
- RC contract remains raw-frame preserving: RC override is forwarded as the raw MAVLink
  channel frame, while normalized axes are local fallback/control-display only.
- Local direct-command fallback remains separate from controller parity and plant replay;
  in SITL it stays disabled unless explicitly enabled for smoke/debug.
- Plant input contract remains raw ArduSub JSON SERVO except in explicit `replay_rcout`
  plant replay.
- Dynamic ellipsoid `fluidcoef` is an explicit, opt-in, velocity-load update path; the
  accepted clean baseline keeps it disabled until a validated HAN/CFD profile enables it.
- Thruster contract remains split into parameter parsing, actuator geometry, command
  shaping, force conversion, immersion scale, and wrench accumulation. This pass did not
  change any coefficient, motor lag, direct T200 curve, or vertical geometry value.
- The current matrix gate explicitly covers:
  `source_identity`, `time_phase`, `sensor_input_output`, `rc_input_output`,
  `controller_output_and_plant_input`, `thruster_contract`, and
  `dynamic_ellipsoid_fluid`.
- Latest matrix status from `/private/tmp/uuv_contract_axes_20260611`:
  - `source_identity`: WARN, only because the top-level ArduPilot gitlink differs
    from the clean local ArduSub 4.1.2 checkout.
  - `time_phase`: PASS.
  - `sensor_input_output`: PASS.
  - `rc_input_output`: WARN, only because local ArduSub 4.1.2 consumes RC override
    C1..C16 while MAVLink defines extension channels beyond that.
  - `controller_output_and_plant_input`: PASS.
  - `thruster_contract`: WARN, only because the plant replay safe-target gate is
    intentionally conservative.
  - `dynamic_ellipsoid_fluid`: PASS.
- The remaining WARN items are contract facts, not hidden PASS:
  - top-level ArduPilot gitlink differs from the clean local ArduSub 4.1.2 checkout;
    do not commit a submodule pointer change unless intentionally updating firmware.
  - local ArduSub 4.1.2 consumes RC override C1..C16, not C1..C18; active controls
    remain inside C1..C8.
  - JSON `altitude` is compatibility/debug output for this firmware; pressure matching
    still goes through JSON `position.z` into `AP_Baro_SITL`.
  - `/mavros/imu/atm_pressure` remains excluded as a parity/fitting target.
  - plant replay gate allows Bar30, IMU, gyro, and DVL x/y targets first; estimator-like
    local-position surfaces and DVL z are not primary fitting targets.

## Remaining Hotspots

- `tools/thruster_param_loader_smoke_cases.py`
- `bridge/ros2_bridge_shutdown_ros.py`
- `sim/runtime/viewer_scene_thrusters.py`
- `tools/axis_rc_vehicle_prepare.py`
- `gui/ros_setup_paths.py`
- `bridge/ros2_state_vertical_hold.py`
- `sim/runtime/viewer_loop_overlay.py`
- `sim/physics/fossen_residual_runtime_builder.py`
- `tools/althold_contract_bin_reader.py`
- `bridge/sitl_math.py`

## Latest Small Split

- Split `tools/axis_rc_arm_service.py` request/state predicates into
  `tools/axis_rc_arm_state.py`.
- `axis_rc_arm_service.py` now owns the arming retry loop only.
- This preserves the arm service timing behavior while making command-service failure
  reasons independently testable during RC override latency triage.

## Continued Refactor Pass

- Split `tools/thruster_param_loader_smoke_cases.py` into:
  - `tools/thruster_param_loader_smoke_io.py`
  - `tools/thruster_param_loader_smoke_assertions.py`
  This keeps fixture IO, loader invocation, and assertion surfaces separate.
- Split `bridge/ros2_bridge_shutdown_ros.py` exception swallowing through
  `bridge/ros2_bridge_shutdown_guard.py`, preserving best-effort shutdown behavior.
- Split viewer thruster vector math into
  `sim/runtime/viewer_scene_thruster_vectors.py`, leaving
  `sim/runtime/viewer_scene_thrusters.py` as drawing orchestration.
- Split axis RC vehicle preparation into named phases in
  `tools/axis_rc_vehicle_prepare.py`, preserving the wait/manual/arm/mode/release
  order.
- Split GUI ROS workspace setup path discovery into
  `gui/ros_workspace_setup_paths.py`, preserving base ROS setup selection.
- Split SITL initial-depth vertical-feedback hold windows into:
  - `bridge/ros2_state_vertical_hold_window.py`
  - `bridge/ros2_state_vertical_zero_reason.py`
  This preserves hold/release zeroing behavior while making the reason logic explicit.
- Split viewer overlay text formatting into `sim/runtime/viewer_overlay_text.py`.
- Split ALT_HOLD DataFlash BIN message collection into
  `tools/althold_contract_bin_message_loop.py`.
- Split GUI physics parameter helpers into:
  - `gui/physics_param_nested.py`
  - `gui/physics_profile_select.py`
  This keeps JSON nested-key access and profile selection separate from value
  formatting.
- Split SITL auto-ready startup log formatting into
  `bridge/sitl_auto_ready_messages.py`.  The actual READY, arm, mode, and neutral-RC
  gate conditions are unchanged.
- Split GUI process scanning into `gui/process_scan_rows.py`, preserving the `ps`
  invocation and external stack detection behavior.
- Split control-loop golden phase response-metric extraction into
  `tools/control_loop_golden_phase_metrics.py`.
- Split real-start status scalar predicates into
  `sim/runtime/real_start_status_predicates.py`, preserving the same mismatch
  thresholds and target checks.
- Split hydrostatic body-component profile entry construction into
  `physics/sim_profile_hydrostatic_component_builder.py`.
- Split source-audit contract-gate summary loading into
  `tools/audit_code_contract_gate_summary.py`, keeping
  `audit_code_contract_common.load_contract_gate_summary()` as the public wrapper.
- Split closed-loop SITL parameter-source loading into
  `tools/audit_closed_loop_sitl_params.py`.
- Split GUI telemetry status/age/autopilot text helpers into
  `gui/control_update_telemetry_status.py`.

## Preservation Rule

The inventory score is now treated as a triage signal, not an edit mandate.  The
following complexity is intentionally preserved unless a concrete failing contract
or test points at it:

- `sim/physics/fossen_residual_runtime_builder.py`: hydrodynamic residual and
  added-mass runtime assembly.
- `bridge/sitl_math.py`: quaternion/rotation conversions.
- `sim/transport/mavlink_message_interval_send.py`: MAVLink
  `SET_MESSAGE_INTERVAL` send semantics.
- `tools/audit_code_contract_firmware_rc_checks.py`: source-evidence checks where
  conclusion and searched source tokens are intentionally coupled.
- `sim/physics/dynamic_fluidcoef_pattern_prepare.py` and other fluidcoef math paths:
  only change with a contract failure or validated HAN/CFD profile requirement.
- `bridge/sitl_rc_override_keepalive.py`: RC latency-sensitive keepalive policy.  Do
  not refactor this path unless a focused RC timing test or command-path trace proves
  the intended behavior.
- `sim/contracts/rc_value_rules.py`: RC marker/PWM semantics.  Preserve unless
  `check_rc_frame_contract.py` or an observed RC frame mismatch points at it.
- `sim/runtime/sitl_servo_binding.py`, `bridge/ros2_state_sitl_velocity.py`,
  `bridge/sitl_transport_model_vertical.py`, and `bridge/ros2_cmd_vel_input.py`:
  execution-path helpers; only refactor with focused smoke coverage or a direct
  bug in command/sensor timing.

## Latest Validation

```text
compileall tools/bridge/sim/physics/gui: PASS
thruster_param_loader=PASS
gui_backend_selection=PASS
gui_readiness_contract=PASS
gui_initial_depth_contract=PASS
initial_hold_pose=PASS
real_start_measurements=PASS
physics_contract_geometry=PASS
physics_runtime_hydrostatic=PASS
runtime_readiness_policy=PASS
sitl_command_link_readiness=PASS
ros2_command_payload=PASS
odometry_publish_builders=PASS
closed_loop_contract payload generation=PASS
dev_os_compat --headless --json: fail=0 pass=16 warn=2
contract_source_audit: fail=0 pass=18 warn=6
```

## Axis RC Sampling Split

- Split axis RC sample field population into:
  - `tools/axis_rc_sample_channels.py`
  - `tools/axis_rc_sample_motion.py`
- Kept `tools/axis_rc_sampling.py` as the single public sample builder used by
  `axis_rc_node_callbacks.py`.
- This is tooling-only cleanup.  It does not change MAVROS RC frame contracts,
  RC override timing, SITL command links, plant input, or sensor publication
  cadence.

## Axis RC Sampling Validation

```text
axis_rc_sampling_smoke=PASS
rc_frame_contract=PASS
compileall tools/bridge/sim/physics/gui: PASS
contract_source_audit: fail=0 pass=18 warn=6
```

## Current Refactor Boundary

The latest inventory still ranks several files with high complexity scores, but
most top entries are intentionally complex contract code:

- Preserve hydrodynamic runtime assembly, quaternion math, MAVLink interval send,
  RC value normalization, RC keepalive policy, vertical SITL math, and hydrostatic
  wrench application unless a focused failing check points at them.
- Continue cleanup in analysis/audit/sample construction modules first, because
  those improve maintainability without changing live MuJoCo/SITL behavior.

## Runtime Identity Audit Split

- Split active-runtime identity source probes into
  `tools/audit_code_contract_runtime_identity_sources.py`.
- Kept `tools/audit_code_contract_runtime_identity_inputs.py` focused on the
  `RuntimeIdentityInputs` dataclass and the public
  `collect_runtime_identity_inputs()` assembly function.
- Preserved the same checked artifacts: active runtime alias, root launchers,
  runtime version JSON, repository branch/head, origin head, and runtime dirty
  path list.

## Runtime Identity Validation

```text
runtime_identity_inputs_smoke=PASS
compileall tools/bridge/sim/physics/gui: PASS
contract_source_audit: fail=0 pass=18 warn=6
```

## Thruster Conversion Audit Split

- Split active-runtime thruster-conversion audit internals into:
  - `tools/audit_code_contract_thruster_conversion_rules.py`
  - `tools/audit_code_contract_thruster_conversion_evidence.py`
- Kept `tools/audit_code_contract_thruster_conversion.py` as the public check
  builder used by `audit_code_contract_thruster_gate_checks.py`.
- Preserved every source token used to verify scheduler timing, actuator lag,
  shaped command conversion, force model selection, immersion scaling, and wrench
  accumulation.  This is an audit-code refactor only; the live thruster model and
  actuator runtime were not changed.

## Thruster Conversion Audit Validation

```text
compileall thruster_conversion audit modules: PASS
compileall tools/bridge/sim/physics/gui: PASS
contract_source_audit: fail=0 pass=18 warn=6
```

## Handoff And Fast Contract Gate

- Added `tools/run_fast_contract_sanity.py` as the single fast, non-live sanity
  runner for refactor handoff.
- Added `docs/HANDOFF_2026-06-11.md` to capture current objective, hard
  boundaries, green checks, preserved runtime complexity, and next runtime work.
- Added `docs/CONTINUATION_PROMPT_2026-06-11.md` so another Codex prompt can
  resume from the same contract without reconstructing the full thread context.

Latest fast gate:

```text
run_fast_contract_sanity.py --include-host: PASS
steps passed: 17
steps failed: 0
artifact: research_workspace/00_current_contract/fast_contract_sanity_host_20260611
```
