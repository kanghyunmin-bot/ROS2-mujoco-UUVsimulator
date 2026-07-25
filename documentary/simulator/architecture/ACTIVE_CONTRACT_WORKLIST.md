# Active Contract Worklist

Date: 2026-06-04

This worklist is ordered by closed-loop contract risk, not by refactor
convenience.  A split is accepted only if the full-runtime gates still catch
invalid plant-input evidence and preserve the controller-parity observation
layer.

Current contract lanes that must stay explicit before any HAN/CFD or physical
coefficient tuning:

- Time contract: ROS sensor publication is sim-time gated; MAVLink command,
  RC keepalive, and `SERVO_OUTPUT_RAW` request polling are wall-clock transport
  cadence.  Sensor phase, RC input phase, and RCOU telemetry phase are separate
  surfaces and must not be merged by interpolation or plotting convenience.
- Sensor I/O contract: one MuJoCo snapshot feeds ArduSub JSON SITL sensor input
  and ROS/MAVROS observation topics.  Bar30/static pressure remains the
  vertical contract surface; `/depth` and `/depth/pose` are outputs for
  observation/debug, not the ArduSub pressure input surface.
- RC in/out contract: `/mavros/rc/override` must preserve the raw 18-channel
  MAVLink frame to SITL, while `/mavros/rc/in` is only a mirror of that raw
  frame.  Normalized axes are local fallback/control-display data, not the
  controller-parity comparison layer.
- Thruster contract: plant input is raw SITL JSON SERVO PWM or explicit replay
  RCOU before physical thruster conversion.  Servo telemetry comparison and
  plant actuator force conversion remain separate.
- Fluid dynamics contract: built-in MuJoCo ellipsoid `fluidcoef` updates are
  opt-in dynamic state/load updates over the five MuJoCo coefficients
  `(blunt, slender, angular, Kutta, Magnus)`.  Static profile coefficients,
  dynamic `geom_fluid` updates, Fossen residuals, CFD/HAN residuals, and
  hydrostatics must have a single owner path so they cannot double-apply force.

## P0: contract gates

1. Add a gate that fails when plant-side RCOU CSV is header-only or neutral
   while armed motion is expected.

Completed:

- GUI command readiness now uses `sim.runtime.readiness.RuntimeReadiness` and
  fresh `/uuv_mujoco/sitl/mavlink_telemetry_status` heartbeat evidence before
  showing `READY` on the internal sim bridge backend.
- `tools/check_runtime_readiness_policy.py` verifies the same READY/WAIT policy
  without importing ROS or Tk.
- `tools/check_plant_input_gate.py` is now integrated into the full MuJoCo
  controller-parity runner before overlay generation.
- `tools/audit_code_contract_sources.py` now resolves and reports the active
  runtime as `sim/current`, splits source checks from report generation,
  and keeps `compat_v22_root` only as metadata for the backing directory.
- `tools/audit_code_contract_checks.py` now only assembles focused source-audit
  check modules.  ArduPilot identity, firmware contract checks, active runtime
  surface checks, path registry, and thruster/gate checks are separate while the
  audit output shape and pass/warn/fail counts are preserved.
- `tools/audit_code_contract_source_identity.py` now verifies
  `sim/current -> v2.2`, the active primary runner, and
  `uuv_mujoco/RUNTIME_VERSION.json` so stale direct-runtime launches are visible
  in the source-contract audit instead of staying as documentation drift.
- Root current-runtime launchers now exist at `sim/run_mujoco.sh`,
  `sim/start_sitl_mujoco.sh`,
  `sim/start_docker_sitl_mujoco.sh`, and `sim/reset_sim.sh`.
  They resolve `UUV_MUJOCO_RUNTIME_DIR` or `sim/current`, so normal
  execution no longer needs direct `v2.2` launcher paths.
- Runtime source identity checks now include active git branch, local HEAD,
  `origin/uuv_sim` HEAD, and dirty runtime paths.  This keeps the active
  `uuv_sim` branch explicit and prevents `origin/main`/`origin/master`
  divergence from being mistaken for a safe simulator update.
- Runtime freshness version payload construction is split into constants,
  dirty-state payload, active-alias/root-launcher path payload, and final
  `RUNTIME_VERSION.json` assembly modules.  The public refresh entry point and
  metadata shape are preserved, while the former top hotspot
  `tools/runtime_freshness_version.py` is no longer in the top 12 inventory.
- Root launch wrappers now run `tools/check_runtime_freshness.py --fetch
  --warn-only` before MuJoCo, local SITL, Docker SITL, and reset entry points.
  The check compares local `HEAD` with `origin/uuv_sim`, verifies
  `sim/current`, and warns when `RUNTIME_VERSION.json` is stale.  This
  makes the old `v2.2` backing directory name a visible compatibility detail
  instead of a hidden stale-runtime risk.
- GUI launch paths now use the same freshness contract.  The root GUI scripts,
  the root `uuv_control_gui.py`, direct `gui/uuv_control_gui.py`, and direct
  `start_sitl_mujoco_mj311.sh` path all check `origin/uuv_sim` before normal
  startup unless `UUV_MUJOCO_SKIP_FRESHNESS_CHECK=1` is set explicitly.
- GUI backend detection is split into graph-count probing, label/layout helpers,
  and scoring/selection policy.  This keeps the “ready but RC override not
  moving” investigation path concrete: graph count input, backend selected, and
  command transport readiness are no longer blended in one file.
- GUI ROS2 panel helpers are split into visibility, process-running predicates,
  thread-safe status setters, and button refresh modules.  The public mixin
  method names are preserved, and the split removes an unnecessary ROS runtime
  import from status updates without changing MAVROS/RViz launch behavior.
- GUI arm/mode command request helpers now split shared retry-log cadence,
  mode-readiness gates, topic command override, and MAVROS service request
  paths.  The public GUI method names and retry behavior are preserved, while
  arm/mode helper imports no longer need the full GUI runtime facade for basic
  smoke tests.
- Initial-state startup is split into depth parsing, pose/depth application,
  and hold-pose capture.  This keeps init-depth/Bar30/real-start mismatch
  debugging tied to concrete files instead of one mixed startup function.
- Bar30/AP_Baro, IMU accel/static-pressure, and vertical-feedback configuration
  are split into separate bridge modules.  Source-contract audit now checks the
  split Bar30 and IMU files directly, preserving the Bar30 pressure contract
  gate after refactor.
- Thruster actuator runtime is split into immersion scaling, force update loop,
  visual propeller spin, and direct command target conversion.  The public
  class API is unchanged, and the ArduSub thruster contract gate still passes.
- Source-contract audit now includes the missing high-risk runtime surfaces:
  sim-time sensor publish gating versus wall-time transport polling, raw
  18-channel RC override forwarding and `/mavros/rc/in` mirroring, raw
  JSON-SERVO/replay-RCOU plant input ownership before thruster conversion, and
  opt-in dynamic MuJoCo ellipsoid `fluidcoef` updates from local velocity/load
  factors.  These checks keep timing, sensor I/O, RC in/out, actuator input,
  and dynamic fluid-dynamics contracts visible in one report.
- IMU/static-pressure configuration is now split by contract owner: source
  selection, SITL JSON accel signs/scales, ROS IMU accel surface calibration,
  and ROS static-pressure source selection.  The static-pressure source audit
  follows the new focused file and still verifies external Bar30 pressure as
  the default `/mavros/imu/static_pressure` output.
- Auto-ready sequencing is now split into wait gates and arm/mode actions.
  This keeps READY, RC override readiness, ExternalNav readiness, neutral RC
  priming, arm, and mode transition evidence separated without changing the
  GUI readiness policy.
- Sensor replay interpolation is now split into boundary/index selection,
  numeric interpolation, and interpolated frame assembly.  This preserves the
  current linear sensor replay behavior while making time/phase contract
  changes auditable in one place.
- Hydrodynamics profile parsing is now split into scalar, array, and vector
  helpers.  Ellipsoid/static/dynamic profile fields keep the same compatibility
  imports, but coefficient parsing is no longer hidden in a mixed common file.
- Golden control-loop thruster summaries now split CSV loading, phase-window
  calculation, column selection, row filtering, and metrics.  This keeps
  actuator validation output stable while making thruster input/output evidence
  easier to audit.
- Runtime profile selection now splits profile listing/alias resolution, built
  profile assembly, and thruster-voltage override.  This keeps the static
  `current` plant contract, opt-in dynamic `fluidcoef` experiments, and
  voltage overrides visible at startup instead of mixing them in one helper.
- Core ROS sensor publishing now splits message factories from the lazy publish
  cache for IMU, `/depth`, `/depth/pose`, Bar30 static pressure, ground truth,
  and sim time.  The builder facade no longer imports MuJoCo-only state types
  at runtime, so sensor I/O contracts can be smoke-tested without the simulator.
- SITL `SERVO_OUTPUT_RAW` to `/mavros/rc/out` mirroring now splits real replay
  timestamp selection, RCOut message construction, and event-publish policy.
  This keeps the controller-parity observation layer separate from raw JSON
  servo/replay-RCOU plant input ownership.
- Runtime transport and sensor-display hotspots are now split by ownership:
  QGC video stream state/lifecycle/frame write, development OS Python probe
  evaluation/selection/result checks, MANUAL_CONTROL frame/neutral-prime/logging,
  replay CSV load/sort policy, MAVLink peer heartbeat/`udpin` wait policy, DVL
  publish factories/cache, and ROS bridge publish/spin/shutdown failure
  isolation.  These preserve command, timing, DVL, and GUI transport behavior
  while reducing the remaining runtime hotspot surface.
- SITL vertical estimation is now split into focused Bar30 pressure/depth,
  vertical-velocity fallback, and NED/ExternalNav frame helpers, while real-start
  Bar30 datum inference is split into per-row candidate extraction and candidate
  selection.  Source-contract audit also has an explicit sensor I/O snapshot
  check: one MuJoCo snapshot must feed ArduSub JSON SITL sensor input and ROS
  core/MAVROS/DVL observation output surfaces.  The current audit result is
  `fail=0`, `pass=16`, `warn=5`.

## P1: transport split

1. Extract sensor replay clock and immediate JSON reply scheduler.
2. Extract ExternalNav/VPD output adapter.
3. Keep controller-parity telemetry and plant-input JSON output as different
   typed streams.
4. Move remaining RC override timing/stream policy to a runtime adapter backed
   by `sim/contracts/rc.py`.

Completed:

- `JsonServoReceiver` now owns JSON-SITL UDP socket bind/recv/send bookkeeping
  while `SitlTransport` preserves the existing public fields and logs.
- `MavlinkTelemetryObserver` now owns passive MAVLink telemetry status
  accumulation while command, arm/mode, and RC override logic remain in
  `SitlTransport`.
- `MavlinkCommandLink` now owns low-level command-link connection, GCS
  heartbeat, RC override packet send, and arm/disarm packet send while command
  policy remains in `SitlTransport`.
- `MavlinkMessageIntervalRequester` now owns per-stream
  `SET_MESSAGE_INTERVAL` request throttling, while low-level message-id
  resolution, interval conversion, and `command_long_send` payload construction
  live in `sim/transport/mavlink_message_interval_send.py`.
- `sim/contracts/rc.py` now owns ArduSub RC override neutral-frame construction
  and MAVROS-style override marker normalization.
- `bridge/sitl_command_targets.py` now owns compatibility exports only.
  Command-link selection lives in `bridge/sitl_command_link_select.py`, command
  readiness gates live in `bridge/sitl_command_link_readiness.py`, MAVLink
  target resolution lives in `bridge/sitl_command_target_resolution.py`,
  vehicle HEARTBEAT/COMMAND_ACK handling and UDP peer discovery remain in their
  focused modules.  `bridge/sitl_commanding.py` keeps the arm/mode/RC/manual
  control send policies.
- `bridge/sitl_auto_ready_runtime.py` is now a compatibility facade.
  ExternalNav readiness gates live in `bridge/sitl_auto_ready_extnav.py`,
  auto-ready state/log throttling lives in `bridge/sitl_auto_ready_state.py`,
  neutral RC priming lives in `bridge/sitl_auto_ready_neutral.py`, and the
  arm/mode ready sequence lives in `bridge/sitl_auto_ready_sequence.py`.
  `bridge/sitl_arm_mode_runtime.py` and `bridge/sitl_rc_manual_runtime.py`
  retain their focused command-policy roles, and `bridge/sitl_commanding.py` is
  only a compatibility export surface consumed by `SitlTransport`.
- `bridge/sitl_rc_manual_runtime.py` is now also a compatibility facade.  RC
  override forwarding and neutral keepalive live in
  `bridge/sitl_rc_override_runtime.py`, MANUAL_CONTROL priming/send lives in
  `bridge/sitl_manual_control_runtime.py`, and GUIDED/raw local setpoint
  forwarding lives in `bridge/sitl_guided_setpoint_runtime.py`.
- `bridge/sitl_mavlink_requests.py` is now a compatibility facade.  MAVLink
  request target/timing helpers live in `bridge/sitl_mavlink_request_targets.py`,
  `SERVO_OUTPUT_RAW` stream requests live in
  `bridge/sitl_mavlink_request_servo.py`, and ArduPilot sensor/attitude
  telemetry requests live in `bridge/sitl_mavlink_request_ap.py`.
  `bridge/sitl_pwm_runtime.py` is now a compatibility facade; plant replay
  ownership gates live in `bridge/sitl_pwm_source_policy.py`, disarmed/all-min
  safety neutralization lives in `bridge/sitl_pwm_safety.py`, neutral/nonneutral
  activity warnings live in `bridge/sitl_pwm_activity.py`, callback/debug output
  lives in `bridge/sitl_pwm_output.py`, and the top-level frame handler lives in
  `bridge/sitl_pwm_frame_handler.py`.
- `bridge/sitl_json_servo_runtime.py` is now a compatibility facade.  JSON servo
  endpoint polling lives in `bridge/sitl_json_servo_endpoint.py`, frame/client
  bookkeeping lives in `bridge/sitl_json_servo_packet_state.py`, missing/stale
  endpoint warnings live in `bridge/sitl_json_servo_warnings.py`, plant replay
  timeout handling lives in `bridge/sitl_json_servo_timeout.py`, and the
  command/MAVLink/JSON polling loop lives in `bridge/sitl_json_servo_poll_loop.py`.
- `bridge/ros2_rc_override_input.py` is now a compatibility facade.  MAVROS RC
  override channel extraction and normalized-axis math live in
  `bridge/ros2_rc_override_frame.py`, SITL forwarding lives in
  `bridge/ros2_rc_override_forwarding.py`, `/mavros/rc/in` mirroring lives in
  `bridge/ros2_rc_override_mirror.py`, warning throttles live in
  `bridge/ros2_rc_override_warning.py`, and the callback policy lives in
  `bridge/ros2_rc_override_callback.py`.  This preserves the existing
  forwarding/local-fallback/mirror contract while leaving a focused surface for
  future RC override latency instrumentation.
- `bridge/ros2_mavros_arm_mode_services.py` is now a compatibility facade.
  Boot-guard and forwarding-enable checks live in
  `bridge/ros2_mavros_arm_mode_guard.py`, locked SITL transport calls live in
  `bridge/ros2_mavros_arm_mode_transport.py`, bridge-level arm/mode forwarding
  policy lives in `bridge/ros2_mavros_arm_mode_forwarding.py`, and ROS service
  response callbacks live in `bridge/ros2_mavros_arm_mode_callbacks.py`.  This
  keeps the GUI/MAVROS service contract intact while making boot-guard versus
  transport-send failures separable.
- `bridge/sitl_arm_mode_service.py` is now a compatibility facade.  Pending
  arm/mode target, timeout, and resend-rate predicates live in
  `bridge/sitl_arm_mode_service_state.py`, command broadcast and arm-neutral-RC
  priming live in `bridge/sitl_arm_mode_service_send.py`, and the retry loop
  policy lives in `bridge/sitl_arm_mode_service_pending.py`.  This keeps
  pending arm/mode retries visible without mixing state classification and
  MAVLink sends in the same branch-heavy loop.
- `bridge/ros2_bridge_spin_publish.py` is now a compatibility facade.  SITL
  servo polling, command timeout clearing, ROS executor spin cadence, publish
  timing, timestamp acquisition, and prepared-snapshot ROS publication live in
  focused `bridge/ros2_bridge_*` helpers.  This preserves the public
  `Ros2Bridge.spin_once()` and `publish()` behavior while separating simulator
  loop timing from ROS publish failure handling.
- `bridge/ros2_runtime_spin.py` is now a compatibility facade.  ROS context
  shutdown classification, safe publisher failure handling, spin-loop state,
  cached ExternalNav sends, and dedicated executor thread startup live in
  focused runtime-spin modules.  This keeps ROS publish failures, context
  shutdowns, and executor-thread failures attributable to separate owners.
- `bridge/sitl_json_sender.py` is now a compatibility facade.  JSON sensor
  payload finite-value validation, sample/send diagnostics, compact JSON
  encoding, receiver-state synchronization, and top-level send policy live in
  focused sender modules.  This preserves the ArduSub 4.1.2 compact JSON packet
  contract while keeping send failures separate from payload validation.
- `bridge/sitl_mavlink_connection.py` is now a compatibility facade.  Servo
  endpoint/default selection, `pymavlink` loading, servo telemetry connection,
  dedicated command-link connection/reconnect, and GCS heartbeat timestamp
  synchronization live in focused MAVLink connection modules.  This preserves
  the JSON-servo fallback, command-link fallback, and reconnect cadence while
  keeping connection failure attribution concrete.
- `bridge/sitl_status.py` is now a compatibility facade.  Wall-clock age
  helpers, sensor replay status payloads, MAVLink core status fields, and
  ExternalNav readiness fields live in focused status modules.  This preserves
  the GUI readiness/status JSON keys while preventing status payload assembly
  from growing into another mixed telemetry owner.
- `bridge/sitl_mavlink_runtime.py` is now also a compatibility export surface.
  MAVLink connection/heartbeat setup, telemetry observer wrappers, and
  servo/command polling loops live in focused modules while preserving the
  `SitlTransport` method bindings for `SERVO_OUTPUT_RAW` telemetry.
- `bridge/sitl_transport.py` now delegates MuJoCo model-state conversion,
  MAVLink telemetry status properties, servo handler binding, direct PWM
  injection, SITL connection setup, and shutdown lifecycle to focused
  `bridge/sitl_transport_*` modules while preserving the `SitlTransport`
  runtime API consumed by the active simulator.
- `bridge/sitl_transport_config.py` now only re-exports focused construction
  config helpers.  JSON servo socket setup, SITL RC/control run-mode state,
  MAVLink command/telemetry/polling setup, and ExternalNav scheduler/runtime
  state live in focused `bridge/sitl_transport_*_config.py` modules.
- `bridge/sitl_replay.py` now owns compatibility exports only.  Replay frame
  and native VPD event types, CSV parsing helpers, sensor/VPD loaders, and
  interpolation logic live in focused modules while preserving the
  `sitl_initialization.py` import surface.
- `bridge/sitl_command_targets.py` now owns compatibility exports only.
  Command-link selection/target resolution, vehicle HEARTBEAT/COMMAND_ACK
  state tracking, and UDP peer discovery live in focused command-link,
  heartbeat, and peer-discovery modules while preserving every `SitlTransport`
  method binding used by RC override, MANUAL_CONTROL, and arm/mode runtimes.
- `bridge/sitl_mavlink_polling.py` now owns compatibility exports only.  Servo
  telemetry polling, command-link polling, and `SERVO_OUTPUT_RAW`/HEARTBEAT
  handler logic live in focused `bridge/sitl_mavlink_*` modules while
  preserving the `_poll_servo_mavlink()` and `_poll_command_mavlink()`
  bindings used by the active JSON/SITL servo loop.
- Synthetic ExternalNav VPD sending now splits scheduler/rewind due checks from
  MAVLink `vision_position_delta_send` emission.  The bootstrap, native-VPD
  replay, synthetic pose/delta construction, transmit-rate accounting, and debug
  logging order are preserved so the sensor timing contract remains auditable.
- `bridge/sitl_arm_mode_runtime.py` now owns compatibility exports only.
  Low-level arm/mode MAVLink send helpers, queue entry points, and pending
  retry service loops live in focused `bridge/sitl_arm_mode_*` modules while
  preserving `send_arm_command()`, `send_set_mode()`, and the pending command
  service bindings used by RC override readiness.
- `bridge/ros2_command_shaping.py` now owns compatibility exports only.
  Direct normalized command filtering lives in
  `bridge/ros2_direct_command_filter.py`, `/cmd_vel` guided-setpoint forwarding
  lives in `bridge/ros2_cmd_vel_input.py`, and MAVROS manual-control forwarding
  lives in `bridge/ros2_manual_control_input.py`.

## P2: ROS surface split

1. Extract Bar30/static-pressure publishers.
2. Extract IMU publishers.
3. Extract DVL/pose publishers.
4. Keep `/mavros/imu/atm_pressure` excluded from fitting until semantics are
   proven.

Completed:

- `bridge/ros2_publish_state.py` now owns ROS-frame derived-state preparation,
  DVL odometry integration, static-pressure source selection, and MAVROS
  setpoint state update before publish jobs are scheduled.
- ROS publish lazy message construction is split by output surface:
  `ros2_publish_builder_core.py`, `ros2_publish_builder_status.py`,
  `ros2_publish_builder_dvl.py`, `ros2_publish_builder_mavros.py`,
  `ros2_publish_builder_odometry.py`, and
  `ros2_publish_builder_ping360.py`.  `ros2_publish_runtime.py` now only
  orchestrates state preparation, scheduling, and queue flushing.
- `bridge/ros2_bridge_public_api.py` now owns the simulator-loop public API
  surface (`publish`, `spin_once`, `shutdown`, SITL servo/replay handlers, and
  odometry reset).  `bridge/ros2_bridge.py` keeps initialization, ROS utility
  helpers, message builders, command aliases, state-estimation aliases, and
  publish/sensor delegation.
- `bridge/ros2_bridge_runtime_methods.py` now owns bridge runtime helper
  methods for rate limiting, ROS spin thread, safe publish, MAVROS state
  message construction, robot description loading, static-context publishing,
  and sensor slicing.  `bridge/ros2_ping360_config.py` owns the
  `/ping360/config` callback.  `bridge/ros2_bridge.py` is reduced to
  constructor wiring and compatibility method exports.
- Ping360 runtime code is split so `bridge/ping360_types.py` owns the public
  config/sample contracts, `bridge/ping360_settings.py` owns firmware-style
  effective range/sample/transmit timing calculations, and
  `bridge/ping360_sim.py` stays focused on MuJoCo raycast profile generation.
- Ping360 scan generation is now split further.  Beam/raycast/reflectivity
  helpers live in `bridge/ping360_beam_model.py`, return accumulation and noise
  synthesis live in `bridge/ping360_profile.py`, and `bridge/ping360_sim.py`
  owns scan timing plus rolling image/range/intensity buffers.
- `bridge/ros2_bridge_config.py` is now a compatibility export surface.
  MAVROS state/rate/RC config lives in `bridge/ros2_bridge_config_mavros.py`,
  pressure/IMU/vertical feedback config lives in
  `bridge/ros2_bridge_config_pressure.py`, and DVL/frame transforms live in
  `bridge/ros2_bridge_config_frames.py`.
- `bridge/ros2_state_estimation.py` is now a compatibility export surface.
  Vertical/Bar30 state estimation lives in `bridge/ros2_state_vertical.py`,
  MuJoCo world kinematics lives in `bridge/ros2_state_kinematics.py`, IMU/DVL
  body-frame conversion lives in `bridge/ros2_state_sensors.py`, and MAVROS
  setpoint command logic lives in `bridge/ros2_state_setpoint.py`.
- `bridge/ros2_bridge.py` now owns the public `Ros2Bridge` constructor facade
  only.  Base runtime state and MAVROS/pressure/frame contract application live
  in `bridge/ros2_bridge_runtime_setup.py`, MuJoCo sensor/Ping360 lookup lives
  in `bridge/ros2_bridge_sensor_setup.py`, and compatibility method bindings
  live in `bridge/ros2_bridge_method_bindings.py`.
- `bridge/ros2_mavros_command_services.py` now owns compatibility exports
  only.  Setpoint/yaw command handling, command-override payload parsing,
  MAVROS arm/mode service forwarding, and internal SITL command override
  handling live in focused `bridge/ros2_*command*` modules while preserving the
  method-binding table for `/mavros/cmd/arming`, `/mavros/set_mode`,
  `/mavros/cmd/command`, and `/uuv_mujoco/sitl/command_override`.
- `bridge/ros2_bridge_runtime_methods.py` now owns compatibility exports only.
  ROS env/rate helpers, safe publish/executor spin helpers, MAVROS state
  building, and static context/robot description helpers live in focused
  `bridge/ros2_runtime_*` modules while preserving the `Ros2Bridge` historical
  method names.
- `bridge/ros2_math.py` now owns compatibility exports only.  RC normalization,
  scalar angle/finite helpers, quaternion/rotation conversion, pressure
  conversion, and loose ROS message setters live in focused modules.  This does
  not change Bar30, IMU, RC override, or frame math equations; it makes each
  contract helper auditable without a mixed math/message utility hotspot.
- `bridge/ros2_sensor_messages.py` now owns compatibility exports only.  IMU
  messages and real-bag covariance, fluid pressure messages, range messages,
  battery messages, and MAVROS VFR HUD messages live in focused builder modules.
  This preserves the ROS message field contract while separating Bar30/static
  pressure, IMU, and DVL/range publisher dependencies.
- `bridge/ros2_ping360_messages.py` now owns compatibility exports only.  Image,
  LaserScan, sonar echo, and status payload/message builders live in focused
  Ping360 ROS message modules.  `bridge/ping360_image_renderer.py` now imports
  Ping360 data contracts from `ping360_types` instead of `ping360_sim`, so the
  ROS Ping360 message layer no longer imports MuJoCo just to render or test
  message fields.
- `bridge/ros2_bridge_config_mavros_rates.py` now owns compatibility exports
  only.  MAVROS state defaults, ROS/MAVROS surface rates and logging, RCOUT
  publish policy, battery defaults, and replay bookkeeping live in focused
  config modules.  RCOUT publish mode fallback, header stamp source handling,
  and real-robot sensor rate defaults are unchanged.

## P3: plant split

1. Split `run_uuv_mujoco.py` into model loading, initial state, actuator,
   hydrostatics, hydrodynamics, current, sensor synthesis, and logging modules.
2. Keep physics coefficient edits behind plant replay validation.
3. Do not run HAN/CFD tuning until plant input and sensor contracts pass.

Completed:

- Model loading/bootstrap is owned by `sim/runtime/model_runtime_setup.py`.
- SITL/plant-replay PWM packet mapping is owned by
  `sim/runtime/sitl_servo_runtime.py`; the same module now creates the runtime
  and binds SITL/replay RCOU handlers to the active bridge without changing the
  ArduSub channel map or sign contract.
- Direct command/ROS bridge setup, initial-depth release service wiring, and
  real-start status wiring are owned by `sim/runtime/control_bridge_setup.py`.
- Actuator/sensor/camera/site lookup and optional QGC video setup are owned by
  `sim/runtime/model_io_setup.py`.
- Thruster parameter loading/logging is owned by
  `sim/runtime/thruster_param_runtime.py`.
- Hydrodynamics setup/diagnostics are owned by
  `sim/runtime/hydrodynamics_runtime_setup.py`.
- Thruster actuator state/force/visual updates are owned by
  `sim/runtime/thruster_actuator_runtime.py`.
- Underwater hydrostatic/hydrodynamic wrench application is owned by
  `sim/runtime/underwater_wrench_runtime.py`.
- MuJoCo fluid geom runtime scaling is split by contract role:
  `sim/physics/fluid_geom_common.py` owns geom matching/parsing,
  `sim/physics/fluid_geom_size_runtime.py` owns profile/env geom-size scales,
  `sim/physics/fluidcoef_scale_runtime.py` owns the five MuJoCo fluid
  coefficient scale paths, and `sim/physics/fluid_geom_apply.py` owns the
  ordered application step.  `sim/physics/fluid_geom_runtime.py` remains a
  compatibility export surface only.
- GUI configuration is split into path, backend/pilot, environment parsing,
  RC/channel, UI display, and physics tuning schema modules.  `gui/config.py`
  remains a compatibility export surface, preserving existing GUI imports while
  making RC contract constants and physics tuning defaults easier to audit.
- Ping360 STL preprocessing is split into binary STL IO, component
  classification, and CLI/pipeline modules.  The active scene still consumes
  the same `assets/ping360/ping360_body_no_cable.stl` mesh; this split only
  reduces tool coupling and preserves generated metadata shape.
- GUI AutoTune controls have been removed from the active GUI runtime.  The
  active control panel keeps manual RC, RC replay, ROS2 utility, and physics
  tuning controls, but no AutoTune method-binding surface remains in GUI code.
- Hydrodynamics helper code is split into 6DOF math, thruster curve helpers,
  hydrostatic submerged-fraction helpers, and equivalent-ellipsoid baseline
  estimation.  `physics/hydrodynamics_helpers.py` remains the compatibility
  export surface; no coefficient equations or plant contracts were changed.
- Simulation profile loading and hydrodynamics coefficient construction are now
  split.  `physics/sim_profile_helpers.py` remains the public compatibility
  facade, `physics/sim_profile_types.py` owns dataclasses, and
  `physics/sim_profile_hydrodynamics.py` owns parser/builder logic.
- Dynamic MuJoCo `fluidcoef` support is split into
  `sim/physics/dynamic_fluidcoef_types.py`,
  `sim/physics/dynamic_fluidcoef_loads.py`,
  `sim/physics/dynamic_fluidcoef_runtime.py`, and
  `sim/physics/dynamic_fluidcoef_setup.py`; the old
  `sim/physics/dynamic_fluidcoef.py` path remains a 25-line compatibility
  facade.
- `tools/audit_closed_loop_contract.py` now resolves `sim/current`
  before using the compatibility backing directory, so closed-loop contract
  checks follow the active runtime alias.
- `tools/roll_stability_sweep.py` is now split into candidate definitions,
  temporary file patch/restore helpers, metric helpers, ROS probe node, and
  launcher/result IO.  This preserves the diagnostic tool's behavior while
  removing it from the top hotspot list.
- `tools/axis_rc_override_check.py` is now split into RC contract constants,
  metrics/health checks, plotting/output IO, ROS node code, and CLI wiring.
  This preserves the RC override diagnostic contract while removing the old
  815-line monolith from the hotspot list.
- `tools/physics_contract_audit.py` is now split into typed audit records,
  MuJoCo model calculations, report/output helpers, runner orchestration, and
  CLI wiring.  The static hydrostatic audit contract is preserved and the CLI
  help path no longer imports MuJoCo.
- The primary runtime entrypoint is now `run_uuv_mujoco.py`; `run_urdf_full.py`
  is retained only as a compatibility wrapper.
- Headless/viewer inner loop timing and debug drawing are owned by
  `sim/runtime/simulation_loop_runtime.py`; top-level loop selection and
  cleanup are owned by `sim/runtime/runtime_loop_entry.py`.
- One-step direct/SITL/plant-replay execution is owned by
  `sim/runtime/simulation_step_runtime.py`.
- Mutable ROS bridge publish/spin/shutdown state is owned by
  `sim/runtime/ros_bridge_runtime.py`, removing `nonlocal ros_bridge` lifecycle
  policy from `run_uuv_mujoco.py`.
- Physics setup, hydrostatic context, thruster geometry overrides,
  hydrodynamics runtime creation, SITL servo binding, actuator runtime creation,
  underwater wrench wiring, descent guard, and thruster debug callbacks are now
  split between `sim/runtime/physics_runtime_setup.py` and
  `sim/runtime/physics_step_callbacks.py`.  `run_uuv_mujoco.py` dropped from
  `612 LOC / main 541` to `335 LOC / main 281` without changing the
  controller/plant contract.
- `sim/runtime/physics_runtime_setup.py` is now only a compatibility facade.
  Runtime setup records live in `physics_runtime_types.py`, hydrostatic/mass
  reference setup lives in `physics_runtime_hydrostatic.py`, geometry overrides
  live in `physics_runtime_geometry.py`, and assembly stays in
  `physics_runtime_factory.py`.
- `physics/sim_profile_hydrodynamics.py` now only assembles the final
  `HydrodynamicsConfig`; low-level profile parsers live in
  `physics/sim_profile_parsing.py`, and ellipsoid baseline estimation lives in
  `physics/sim_profile_ellipsoid.py`.
- ExternalNav/VPD runtime helpers are split into bootstrap, native replay,
  synthetic VPD, and cache/contract modules without changing the
  `SitlTransport` binding names.
- Runtime CLI option construction is split by option family.  `sim/runtime/cli.py`
  now only assembles the parser, while profile, ROS2, Ping360/video, SITL,
  initial-state, and viewer options live in `sim/runtime/cli_*` modules.
- `tools/physics_contract_model.py` now delegates geometry lookup, runtime
  body mass/CoM/inertia application, buoyancy force-balance math, and neutral
  open-plant simulation to focused `tools/physics_contract_*` modules.  Static
  force-balance output remains unchanged after the split.
- `tools/physics_contract_runner.py` now delegates profile loading and
  audit-only CoB override application, start-depth candidate construction,
  neutral open-plant simulation dispatch, and report dictionary assembly to
  focused modules.  Static force-balance CSV/JSON output shape is unchanged and
  the audit still runs with the selected MuJoCo runtime Python.
- `sim/runtime/underwater_wrench_runtime.py` now owns the public `apply(dt)`
  sequencing only.  Hydrostatic buoyancy/restoring torque lives in
  `sim/runtime/underwater_hydrostatic_runtime.py`, typed intermediate results
  live in `sim/runtime/underwater_wrench_types.py`, and hydrodynamic damping,
  added-mass, residual Fossen, CFD dynamic force, pitch/lift, and heave damping
  live in `sim/runtime/underwater_hydrodynamics_runtime.py`.
- `sim/runtime/underwater_hydrostatic_runtime.py` is now split further by
  hydrostatic source.  Weighted buoyancy point/component calculations live in
  `sim/runtime/underwater_hydrostatic_weighted.py`, release-blended restoring
  torque lives in `sim/runtime/underwater_hydrostatic_restoring.py`, and the
  original module preserves the public `apply_hydrostatic_wrench()` surface.
- `tools/real_start_state.py` now owns CLI compatibility only.  Real feedback
  CSV loading, nearest-row selection, quaternion/frame conversion, row field
  extraction, Bar30/AP_Baro pressure datum inference, state assembly, and shell
  formatting are split into focused `tools/real_start_*` modules.  The real
  controller feedback CSV at `69.35s` still produces JSON and shell output
  with the same initial-state contract fields.
- `sim/physics/model_setup.py` now owns compatibility exports only.  Fluid option
  density/viscosity scaling lives in `fluid_option_runtime.py`, pool depth/XY
  runtime overrides live in `pool_runtime_overrides.py`, MuJoCo fluid geom/coef
  scaling lives in `fluid_geom_runtime.py`, and body subtree mass helpers live in
  `body_tree.py`.  `sim.runtime.model_runtime_setup` and hydrostatic setup keep
  the same import surface.
- `sim/runtime/real_start.py` now owns compatibility exports only.  Env parsing
  types, target loading, runtime measurement extraction, pressure calibration,
  payload/status assembly, and status publishing live in focused
  `sim/runtime/real_start_*` modules while preserving GUI and initial-state
  import paths.
- `sim/physics/thruster_params.py` now owns compatibility exports only.
  Thruster default payloads, JSON config loading, per-thruster reset/application,
  and direct-gain environment overrides live in focused
  `sim/physics/thruster_param_*` modules and
  `sim/physics/thruster_direct_overrides.py`.
- `sim/physics/dynamic_fluidcoef_runtime.py` now owns the per-step MuJoCo
  `geom_fluid` update only.  Environment/profile parsing, transient state
  initialization, and smoothing knob setup live in
  `sim/physics/dynamic_fluidcoef_runtime_config.py`; coefficient values and
  update equations are unchanged.
- `bridge/sitl_json_sensor_runtime.py` now owns compatibility exports only.
  ArduSub JSON payload construction, UDP packet validation/send, servo-frame
  immediate replay replies, and live/replayed sensor send policy live in
  focused modules.  The source-contract audit follows the payload module for
  the Bar30 `position.z` frontend-match evidence.
- `sim/physics/fossen_residual.py` now owns compatibility exports only.
  Fossen residual coefficient keys/types, runtime coefficient construction,
  added-mass matrix assembly, and body-frame residual wrench evaluation live in
  focused modules while preserving the hydrodynamics runtime import surface.
- `sim/runtime/physics_runtime_factory.py` now delegates thruster parameter/SITL
  servo/actuator/debug creation to `sim/runtime/physics_runtime_thrusters.py`,
  underwater wrench/body-velocity setup to
  `sim/runtime/physics_runtime_underwater.py`, and callback/result packaging to
  `sim/runtime/physics_runtime_finalize.py`.  The public
  `create_runtime_physics_setup()` import surface is preserved.
- Simulation profile parsing is split by contract role.  Common scalar/vector
  parsing lives in `physics/sim_profile_parse_common.py`, body/buoyancy-point
  parsing lives in `physics/sim_profile_hydrostatic_points.py`, and restoring
  stiffness parsing lives in `physics/sim_profile_hydrostatic_restoring.py`.
  `physics/sim_profile_parsing.py` remains the compatibility export surface
  for hydrodynamics and ellipsoid baseline imports.

## P4: GUI and developer tooling split

1. Keep GUI command semantics and readiness gates explicit.
2. Separate long-lived process control, command sending, telemetry callbacks,
   layout, and visualization helpers.
3. Keep developer verification tools runnable as standalone scripts.

Completed:

- GUI AutoTune runtime controls were removed.  Historical AutoTune refactor logs
  remain as evidence snapshots only; active GUI composition no longer imports an
  AutoTune mixin or exposes AutoTune buttons.
- `gui/node_state_runtime.py` now owns `UuvGuiNode` backend detection,
  command-readiness label calculation, event insertion, telemetry snapshot
  copying, and age calculation.  `gui/node.py` keeps ROS subscription/client
  setup and compatibility method exports.
- `tools/check_dev_os_compat.py` now only owns CLI parsing, result aggregation,
  and output formatting.  Runtime Python/MuJoCo/display probes live in
  `tools/dev_os_compat_runtime.py`, Docker/SITL/Ubuntu/ROS checks live in
  `tools/dev_os_compat_system.py`, and shared probe types live in
  `tools/dev_os_compat_common.py`.
- `tools/control_loop_golden_compare.py` now only owns CLI parsing, result
  writing, and terminal summary.  Numeric helpers, thruster phase summaries,
  fingerprint construction, and comparison policy live in focused
  `tools/control_loop_golden_*` modules while preserving the CLI and legacy
  import names used by golden control-loop checks.
- `gui/layout_mixin.py` now only re-exports focused layout helpers.  Shell,
  telemetry, control core, replay controls, tuning controls, and pilot controls
  are split into `gui/layout_shell.py`, `gui/layout_telemetry.py`, and
  `gui/layout_control_*.py` modules without changing widget ownership.
- `gui/node_commanding.py` now only re-exports focused command helpers.  Common
  readiness/retry gates, arm/mode policy, and RC/manual/Ping360 publishers are
  split into `gui/node_commanding_common.py`, `gui/node_arm_mode_commands.py`,
  and `gui/node_rc_publishers.py` while preserving the `UuvGuiNode` method
  binding surface.
- `tools/althold_diagnostics_logger.py` now only owns CLI parsing and process
  lifecycle.  ALT_HOLD contract math, ROS2 capture node, and CSV/plot/summary
  output are split into focused modules, and `--help` remains runnable without
  importing ROS2/rclpy.
- Active-runtime provenance now lives in `uuv_mujoco/RUNTIME_VERSION.json`;
  setup and closed-loop audit paths resolve `sim/current` first and only
  use the physical `v2.2` directory as a compatibility fallback.
- `gui/sim_stack_process_mixin.py` now composes focused launch, reset, and
  status mixins.  This keeps GUI start/stop/ready state behavior explicit
  without changing the public method names used by `gui/app.py`.
- `gui/control_display_mixin.py` now composes focused pilot, toggle, drawing,
  feedback, and UI update mixins.  Manual RC override publishing and UI refresh
  semantics are preserved while the old monolith is removed from the hotspot
  list.
- `tools/analyze_althold_contract.py` now delegates CSV domain records,
  time-binned summaries, segment metrics, and plotting to focused
  `tools/althold_contract_*` modules.  The CLI remains the active entry point
  and keeps the same contract-analysis outputs.
- `gui/autotune_monitor.py` is now a compatibility facade.  Window lifecycle,
  log parsing, candidate/progress table updates, and score chart rendering live
  in focused `gui/autotune_monitor_*` modules; the chart path no longer imports
  ROS runtime just to use a clamp helper.
- `gui/helpers.py` is now a compatibility facade.  GUI math/format helpers, RC
  contract/message construction, ROS2 bag RC replay loading, and backend-name
  normalization live in focused `gui/gui_*_helpers.py`,
  `gui/rc_replay_loader.py`, and `gui/backend_helpers.py` modules while keeping
  the existing `gui.helpers` star-import surface for legacy mixins.
- `gui/sim_stack_env.py` is now a compatibility facade.  GUI-started
  simulator env contracts, flag parsing, initial-depth argument policy, and
  extra launch-argument normalization live in focused `gui/sim_stack_env_*`
  modules without changing the launch mixin import path or READY/RC override
  contract defaults.
- `gui/physics_mixin.py` is now a compatibility mixin.  Physics tuning window
  construction lives in `gui/physics_window.py`, profile loading/parsing/apply
  logic lives in `gui/physics_param_io.py`, and post-apply restart sequencing
  lives in `gui/physics_restart.py`.
- `tools/axis_rc_node.py` now owns ROS2 node wiring and compatibility methods
  only.  RC override/manual-control message construction, telemetry sample
  construction, and arm/mode/trigger wait loops live in
  `tools/axis_rc_messages.py`, `tools/axis_rc_sampling.py`, and
  `tools/axis_rc_services.py`.  The `AxisRcOverrideCheck` public methods used by
  `tools/axis_rc_override_check.py` are preserved.
- `tools/audit_closed_loop_contract.py` now owns CLI parsing, JSON writing, and
  terminal summary only.  Watched ArduSub/RC/EKF parameter parsing, active
  runtime/profile/thruster-curve inspection, and payload assembly live in
  focused `tools/audit_closed_loop_*` modules.  The emitted JSON payload is
  byte-for-byte equivalent after parsing to the pre-split output.
- `sim/runtime/hydrodynamics_runtime_setup.py` now owns the active runtime
  assembly surface only.  Typed containers, scalar/array extraction, CFD/Fossen
  residual construction, and diagnostic logging live in focused
  `sim/runtime/hydrodynamics_runtime_*` modules.  The public
  `build_hydrodynamics_runtime_setup()` entry point and returned field names are
  preserved for `sim/runtime/physics_runtime_factory.py`.
- `run_uuv_mujoco.py` now owns CLI entrypoint orchestration only.  Runtime
  mode/profile selection, model and real-start initial-state setup, fluid/model
  IO/physics setup, `SimulationStepRuntime` wiring, and final loop invocation
  live in focused `sim/runtime/runner_*_setup.py` modules.  `--help` and
  `--list-profiles` behavior is preserved.
- `tools/actuator_wrench_audit.py` now owns CLI parsing and output path handling
  only.  Shared constants, model/site adjustment, direct-gain loading,
  per-axis wrench construction, coupling summaries, and Markdown rendering live
  in focused `tools/actuator_wrench_*` modules.
- `gui/replay_mixin.py` now owns compatibility exports only.  RC replay status
  updates, timeline/seek/rate handling, ROS bag load/start/pause/stop actions,
  and the worker playback loop live in focused `gui/replay_*` modules while
  preserving all method names consumed by `layout_control_replay.py`,
  `control_pilot_mixin.py`, and simulator-stack launch controls.
- `gui/app.py` now owns application composition and CLI entry only.  Tk variable
  initialization, runtime process/thread state, and widget reference defaults
  live in `gui/app_state.py`; executor spin, window raise, update scheduling,
  shutdown, and mainloop logging live in `gui/app_lifecycle.py`.
- `gui/node.py` now owns the public `UuvGuiNode` composition surface only.
  Publisher/subscriber/client wiring, real-start status defaults, telemetry
  snapshots, and process state live in `gui/node_init.py`; vehicle-info
  request/response handling lives in `gui/node_vehicle_info.py`; existing
  command/readiness/telemetry method names remain bound for the GUI.
- `gui/ros_process_mixin.py` now owns compatibility composition only.  ROS2
  panel/status helpers live in `gui/ros_panel_mixin.py`, logged process
  launch/watch lives in `gui/ros_logged_process_mixin.py`, MAVROS package
  build/start controls live in `gui/ros_package_mixin.py`, and RViz controls
  live in `gui/rviz_process_mixin.py`.
- `gui/ros_tools.py` now owns compatibility exports only.  ROS setup discovery,
  ROS shell command construction, and RViz config generation live in focused
  modules while preserving imports used by Ping360, RViz, package launch, and
  GUI entry modules.
- `gui/node_telemetry_callbacks.py` now owns compatibility exports only.
  Vehicle state/status text, motion/depth/pressure/battery, RC input/output,
  SITL/real-start status JSON, and Ping360 status parsing live in focused
  callback modules while preserving every `UuvGuiNode` callback method name.
- `gui/node_state_runtime.py` now owns compatibility exports only.  Backend
  graph probing and RC layout detection, telemetry snapshot/event helpers, and
  command readiness/ExternalNav liveness helpers live in focused modules while
  preserving every `UuvGuiNode` state/runtime helper binding.
- `tools/roll_stability_probe.py` now owns the ROS2 node composition surface
  only.  Pose/IMU/RC callbacks, RC override publishing, arm/mode command
  services, and probe sequencing live in focused `tools/roll_stability_probe_*`
  modules while preserving the `StabilityProbe` public class used by
  `tools/roll_stability_sweep.py`.
- `gui/sim_stack_launch_mixin.py` now owns simulator start/restart sequencing
  only.  GUI-owned log tailing, status-prefix parsing, and process-finish UI
  updates live in `gui/sim_stack_log_watcher.py`, preserving the same
  `_watch_sim_stack_output()` method used by the launch thread.
- Ping360 MuJoCo sensor lifecycle now delegates model-id lookup, runtime
  buffers/sweep/settings, and scan/update-cycle bookkeeping to focused
  `bridge/ping360_*` helpers while preserving the public `Ping360Simulator`
  class and status payload contract.
- SITL replay CSV row parsing now separates generic row-vector extraction,
  sensor-frame rows, and native VPD rows while preserving the compatibility
  exports consumed by `bridge/sitl_initialization.py`.
- GUIDED setpoint forwarding now separates BODY_NED velocity conversion from
  raw LOCAL_NED setpoint forwarding.  The existing body FLU to ArduSub/MAVLink
  sign contract remains covered by focused smoke evidence.
- MAVROS lazy publish cache construction now uses a topic factory registry
  instead of one method per cached message.  Topic names, builder order, and
  lazy-cache reuse semantics are preserved.
- Hydrodynamics and thruster runtime diagnostic logging now delegate individual
  log sections to focused helper modules.  This keeps runtime setup facades
  smaller without changing physics coefficients, thruster parameters, or logged
  evidence text semantics.
- Runner startup setup is now split by startup contract: model loading remains
  in `runner_initial_setup.py`, initial-depth runtime construction lives in
  `runner_initial_depth_setup.py`, thruster immersion startup env parsing lives
  in `runner_initial_thruster_contract.py`, and the returned setup dataclass
  lives in `runner_initial_setup_types.py`.
- Runtime command/ROS bridge setup is now split by authority owner: command
  timeout/direct-command policy, initial-depth release services, real-start
  status construction, and the returned setup dataclass live in focused
  `control_bridge_*` modules while preserving the runner-facing factory.
- CFD dynamic-wrench profile parsing now separates axis force-table validation
  from runtime construction.  Hydrostatic runtime reporting now separates CoB
  site alignment/override logging from application/restoring log sections.
- Initial-depth hold state now separates dict-style compatibility and hold/
  release actions into focused mixins while preserving the public
  `InitialDepthHoldState` factory and item-access behavior used by runtime code.
- ArduPilot integrity preflight is now split into read-only git status
  collection, payload classification, and output/exit-code helpers.  The CLI
  keeps the same watched-file policy and does not modify the ArduPilot source.
- ALT_HOLD diagnostics output is now split into CSV, finite-series, plot, and
  terminal-summary helpers.  `tools/althold_diagnostics_output.py` remains only
  the compatibility export surface, so diagnostics can keep the same import
  names without growing back into a branch-heavy mixed output module.
- GUI logged ROS process launch/watch ownership is split into log-file
  creation, subprocess launch, watcher tailing, finish-state reporting, and
  process-thread binding helpers.  `_start_logged_ros_process()` and
  `_watch_ros_process()` remain the GUI compatibility methods, and the watcher
  helper can now be smoke-tested without importing ROS2/rclpy.
- GUI command publishers are split by command surface: RC override,
  MANUAL_CONTROL, and Ping360 config publishers now live in focused modules
  while `gui/node_rc_publishers.py` remains the compatibility export surface
  used by `gui/node_commanding.py`.
- The one-step MuJoCo runtime now separates direct-command step execution from
  raw-PWM SITL/plant-replay step execution.  `SimulationStepRuntime` still owns
  the same dataclass wiring and public methods, but the two runtime paths are
  independently smoke-tested.
- Initial-depth runtime release ownership is split into release sequencing,
  release-snapshot publication, and ROS service installation helpers.  The
  runtime still preserves the release order: mark released, reset velocity,
  apply captured release velocity, `mj_forward`, publish one fresh sensor
  snapshot, and report the release reason.
- Passive MAVLink telemetry dispatch now keeps HEARTBEAT handling in the
  top-level dispatch path and delegates target-source telemetry storage to
  focused handlers.  This preserves the source-filter contract while making
  Bar30 pressure, attitude, RC channels, EKF, VFR HUD, and IMU storage rules
  auditable by message family.
- GUI simulator stack launch is split into restart sequencing, start guards,
  process start, and post-start state recording.  `_start_sim_stack()` remains
  the GUI compatibility method, but external-stack blocking, RC replay stop/
  neutral release, and GUI-owned process registration are now separate smoke
  surfaces.
- MuJoCo fluid coefficient runtime scaling is split into global, per-geom, and
  extra-runtime scale helpers while `fluidcoef_scale_runtime.py` remains the
  compatibility export surface.  The scale range, glob matching semantics, and
  log messages are preserved.
- GUI pilot control is split into command reading, pilot publication, release/
  neutral publication, and initial-depth release request helpers.  This keeps
  RC override and MANUAL_CONTROL publication policy explicit without changing
  the GUI method names.
- Viewer control state is split into construction, mutation/toggle, and status
  helpers.  This keeps pause/follow-camera/overlay behavior auditable without
  changing the runner-facing `ViewerControlState` API.
- Distributed body component setup is split into composite inertia calculation,
  MuJoCo model mutation, and log output helpers.  The mass/CoM/parallel-axis
  inertia calculation and qpos/qvel preservation contract are preserved.
- Hydrostatic runtime env/profile extraction is split into CoB values,
  restoring base env/default values, and real-start trim override helpers while
  `hydrostatic_runtime_values.py` remains the compatibility facade.  This keeps
  CoB offsets, restoring stiffness, release trim blending, and real-start
  roll/pitch trim ownership auditable without changing any hydrostatic
  coefficients.
- Runner physics setup is split into fluid-contract setup, runtime control-path
  logging, ArduSub thruster model-IO naming setup, and runtime physics factory
  adapter helpers.  `create_runner_physics_setup()` remains the runner-facing
  API, and the raw RCOU/SITL plant-input flag is still passed directly into the
  runtime physics factory.
- GUI physics parameter apply ownership is split into loading, persistence, and
  error-log helpers.  `physics_param_apply.py` and `physics_param_io.py` remain
  compatibility surfaces until all GUI physics imports are migrated.
- GUI RC replay ownership is split into bag browsing, load/clear controls,
  playback/seek controls, pure time formatting, pure RC padding, and replay
  worker state/seek/pause/wait/publish/finish helpers.  The split preserves
  zero-order RC override replay behavior while making the non-ROS helper paths
  importable without `rclpy`.
- Dynamic MuJoCo fluidcoef pattern setup is split into pattern matching,
  reference-ratio preparation, per-row weights, array mutation, and logging.
  This keeps the coefficient values unchanged while making the hydrodynamic
  tuning input contract auditable by stage.
- MuJoCo base-state ownership is split into id lookup, free-joint mutation,
  Bar30 depth helpers, and public mixin methods.  `MuJoCoBaseState` keeps the
  same public API for initial pose/depth, but the qpos/qvel and Bar30 site
  contracts are now testable without reading a single large class.
- Static body-contract audit now reuses the runtime body-distribution inertia
  calculation instead of carrying a separate copy.  This keeps static
  force-balance reporting aligned with the runtime mass/CoM/inertia contract.
- `sim/runtime/initial_state.py` now imports `os` explicitly before using
  `os.environ` during Bar30 real-start pressure calibration.
- Initial depth application now separates Bar30/base-depth mutation, surface
  hysteresis warnings, and log formatting.  `apply_initial_depth_request()`
  remains the public entry point, but init-depth mismatch debugging can inspect
  each stage independently.
- Step physics callback wiring now separates the callback dataclass and local
  body-velocity reader from the top-level callback builder.  The old
  `physics_step_callbacks` import surface remains compatible.
- Thruster runtime setup is split into param/SITL-servo binding, actuator
  runtime creation, debug runtime creation, and typed bundle helpers.  The
  actuator builder signature is now aligned with its factory caller
  (`thruster_global`, `thruster_scale`, direct-scale/asymmetry/time constants)
  instead of the stale `thruster_param_runtime` argument shape.
- Underwater hydrodynamic residual application is split into pure body-frame
  residual/Fossen/CFD calculations, world-frame MuJoCo wrench application, CFD
  debug logging, residual-family dispatch, custom-hydrodynamics dispatch, and
  relative-acceleration tracking.  Coefficients and formulas are unchanged; the
  runtime can now isolate whether a mismatch comes from coefficient generation,
  dispatch enable flags, or `xfrc_applied` application.
- Initial runtime state setup is split into typed outputs, hold/real-start
  policy resolution, ordered depth/pose/hold application, and Bar30 pressure
  calibration.  `configure_initial_runtime_state()` remains the public entry
  point while init-depth and real-start contracts are now separately testable.
- Neutral open-plant buoyancy application is split one level deeper into
  context construction, per-body-component buoyancy force calculation,
  hydrostatic restoring torque, and MuJoCo force application.  The public
  `physics_contract_neutral_buoyancy.py` import surface is preserved, and
  `physics_contract_audit.py --simulate-s 0.05` still reports zero static
  net-down force with near-zero neutral drift.
- Axis RC health gates are split into per-phase checks, sign-pair checks, and
  overall severity aggregation while preserving `build_health()`.  This keeps
  the RC override report warnings inspectable without changing the pass/warn/
  fail semantics used by axis command validation.
- Axis RC output generation is split into ordered CSV/JSON writers, time-series
  value helpers, and the Matplotlib PNG renderer while preserving
  `write_outputs()` and `plot_timeseries()`.  The axis response artifact schema
  and RC step plotting semantics are unchanged.
- Ping360 STL asset filtering is split further into union-find connectivity,
  connected-component statistics, and cable-like keep/drop classification while
  preserving `component_stats()` and `classify_components()`.  This affects only
  mesh preprocessing; active Ping360 sensor topics and runtime contracts are
  unchanged.
- Development OS viewer compatibility checks are split into macOS `mjpython`
  launcher probing and display-environment checks while preserving
  `dev_os_compat_viewer.py` exports.  The headless dev-OS smoke still reports
  fail=0, with only environment-state warnings for Docker daemon and ROS2 shell
  sourcing.
- Static physics contract report output is split into CSV/JSON file writing,
  console orchestration, mass/CoM/inertia, hydrostatic/start-depth, and
  balance/neutral-simulation sections while preserving
  `physics_contract_report.py` exports.  The static audit still reports
  `net_down=+0.000N` and the same neutral open-plant drift smoke.
- Thruster debug CSV runtime is split one layer deeper into sample scheduling
  and single-row emission helpers while preserving `ThrusterDebugRuntime.emit()`.
  The 20 Hz sample cursor and `mj_forward` force-breakdown sampling point are
  unchanged, so this is observability cleanup only.
- Source-contract audit builders are split by firmware JSON servo, JSON sensor
  keytable, Bar30-from-position, SERVO_OUTPUT_RAW telemetry, active-runtime
  Bar30, JSON altitude warning, static-pressure output, and atm-pressure
  exclusion surfaces.  `audit_code_contract_sources.py` still reports
  `fail=0`, `pass=11`, `warn=5`.
- Refactor inventory now ranks structural complexity instead of raw LOC.  This
  prevents data-only config modules from hiding real control-flow hotspots.
- GUI process helpers are split into env/backend parsing, process scanning, and
  process-group termination without importing ROS-only runtime state.  GUI
  ARM/MODE command senders are split into gate/topic/service request steps
  while preserving `_send_arm_request()` and `_send_mode_request()`.
- GUI simulator-stack status is split into external process probing and pure
  status/control policy helpers.  Native VPD replay is split into start cursor
  alignment, due-event send, TX-rate contract monitoring, and debug logging.
  The native VPD helper smoke preserves cursor advancement and message send
  count.
- Command-readiness labels are split by preflight, runtime command-path, and
  operator-control stages.  `tools/check_runtime_readiness_policy.py` still
  passes, preserving the GUI READY/WAIT ordering used for arm/mode/RC gating.
- Roll-stability command tooling and golden-control-loop thruster summaries now
  separate wait-loop/request helpers and row/metric calculations.  These are
  validation-tool cleanups only, not plant/controller behavior changes.
- GUI simulator-stack log watching and GUI shutdown lifecycle are now split by
  responsibility.  `sim_stack_log_watcher.py` delegates file tailing and status
  line parsing to focused helpers, while `app_lifecycle.py` delegates close-time
  cancellation, process termination, RC release, ROS shutdown, and Tk destroy
  steps.  The public GUI method names are preserved.
- Thruster performance loading, direct-gain overrides, passive-viewer loop
  cadence, and Ping360 publish builders are now split by contract boundary:
  offline T200 curve JSON IO/curve selection, profile/env/per-thruster direct
  gain application, viewer timing/catch-up clocks, and Ping360 per-cycle lazy
  message caches now live in focused helpers.  No plant coefficients, RC
  override semantics, or Ping360 ROS topic payload fields changed.
- SITL sensor vector assembly now separates DVL altitude source selection, IMU
  body-vector construction, and DVL body-velocity construction.  The Bar30,
  IMU, and DVL output contract remains unchanged, but sensor I/O debugging no
  longer requires reading one mixed helper.
- GUI initial-depth launch defaults now separate explicit launch args,
  real-start ownership, Bar30-depth defaults, and base-link debug-hold behavior.
  This keeps time/hold readiness issues separate from pressure-depth contract
  issues.
- Closed-loop contract auditing now includes `dynamic_fluidcoef`.  The current
  profile reports it as configured but inactive, so fixed
  `mujoco_fluidcoef_geom_scales` remains the accepted clean baseline until
  dynamic fluidcoef experiments pass plant replay gates.
- The SITL real-robot contract now matches `INS_POS1_X=0.145000`; the watched
  real/SITL parameter mismatch list is empty after the update.
- Physics-contract tools now lazy-load MuJoCo.  Help/import surfaces work under
  system Python, while actual force-balance audits explicitly require the
  MuJoCo-enabled `ros2_h311` runtime.
- GUI simulator-stack status now separates button control state, refresh
  orchestration, and thread-safe status text updates.  The status mixin no
  longer imports `gui.runtime`, so non-ROS GUI helper imports do not require
  `rclpy`.
- GUI command readiness now separates state/depth/IMU/RCOU/real-start
  freshness checks, required auto-ready mode selection, runtime readiness
  assembly, and final command-readiness input construction.  The public
  `control_readiness()` API and label policy are unchanged.
- GUI ARM requests now mirror the mode-request structure: deadline, gate,
  topic command override, and MAVROS service request handling are separate
  helper modules.  This keeps ARM delay causes easier to isolate without
  changing retry behavior.
- Sim-profile initial-depth candidates now parse body-component and
  buoyancy-point fields separately from the local-top to Bar30-depth
  calculation.  This keeps `init_depth_mismatch` analysis tied to the same
  pressure-depth contract used by the runtime.
- GUI ROS package controls are split into build and MAVROS-stack helpers, and
  ROS setup discovery now separates candidate generation from package probing.
  This keeps macOS conda fallback and Ubuntu `/opt/ros/<distro>` behavior
  auditable by stage.
- Real-start status evaluation now has explicit mismatch predicates for missing
  contract, depth, pressure, XY, attitude, velocity, and angular velocity.
  This keeps GUI WAIT labels and runtime real-start release debugging aligned.
- Real-start status publishing, GUI initial-depth release state transitions,
  GUI ARM/MODE feedback freshness gates, and descent-contract classification
  are now split into focused helpers.  This removes those contract-sensitive
  files from the top structural-complexity list while preserving status
  payloads, initial hold release semantics, command readiness ordering, and
  descent diagnostic messages.
- Visual and lifecycle hotspots are now split by responsibility.  Ping360 polar
  rendering delegates lookup construction and image layers, GUI attitude/depth
  drawing delegates widget-specific helpers, viewer scene drawing delegates
  primitive allocation and bubble streams, Ping360 RViz/rqt control delegates
  process/status helpers, ROS bridge shutdown delegates thread/SITL/ROS steps,
  MAVLink servo HEARTBEAT delegates target filtering, and sim-stack reset
  delegates worker/thread helpers.  These are display/lifecycle cleanups only;
  controller, RC, Bar30/static-pressure, thruster, and plant-input contracts are
  unchanged.
- Static context publishing, development-OS Python probing, immediate
  sensor-replay JSON replies, Ping360 beam modeling, Fossen residual wrench
  evaluation, and MAVLink message interval requests are now split by ownership.
  New smoke checks cover static context cadence, immediate replay one-shot
  frame replies, Fossen residual damping composition, and SET_MESSAGE_INTERVAL
  throttle/mark behavior.  Full validation still reports source-contract
  `fail=0`, `pass=16`, `warn=5`, empty closed-loop parameter mismatches, GUI
  readiness/backend PASS, runtime readiness PASS, RC frame PASS, and ArduSub
  thruster contract OK.
- Cleanup candidates found during this pass are compatibility surfaces, not
  immediate delete targets: `run_urdf_full.py`, `gui/helpers.py`,
  `gui/config.py`, `gui/uuv_control_gui.py`, `gui/physics_param_io.py`, and
  `gui/replay_controls.py` should only be removed after every active import,
  debug runner, and GUI status pattern has moved to the newer names.

## P5: HAN/CFD research path

1. Build one matrix gate for added mass, Fossen residual, hydrostatic, inertia,
   and current perturbations on the same fixed baseline.
2. Use SVD/conditioning reports to reject unidentifiable parameter subsets.
3. Only run expensive plant replay for matrix-gate survivors.
4. Export frozen profiles only; runtime must not import training loops.

## Current Refactor Hotspots

- `physics/sim_profile_hydrostatic_buoyancy_points.py`: buoyancy-point parser.
  Touch only with static plant contract and neutral force-balance audits.
- `gui/rc_replay_loader.py`, `bridge/ros2_command_payload.py`, and
  `bridge/sitl_command_link_readiness.py`: GUI replay/command readiness parsing
  hotspots.  Keep them separate from controller-parity math and verify RC
  frame plus GUI readiness gates after changes.
- `bridge/ros2_publish_builder_odometry.py`: odometry publish builder hotspot.
  Safe only if topic keys, frame IDs, DVL odometry integration, and lazy publish
  behavior remain unchanged.
- `gui/sim_stack_initial_depth_contract.py`: initial-depth GUI contract
  hotspot.  Keep `init_depth_mismatch`, Bar30-depth, and base-link debug-hold
  semantics explicit.
- `bridge/sitl_external_nav_contract.py`: ExternalNav/VPD freshness predicate
  hotspot.  Touch only with VPD timing smoke and command-readiness gates.
- `tools/filter_ping360_stl_io.py`: binary STL IO hotspot.  Sensor-display
  only; keep mesh filtering deterministic.
- `sim/physics/fossen_residual_builders.py`: Fossen residual runtime builder.
  Keep disabled/static/dynamic ownership clear; do not use this to tune before
  plant-input and sensor-phase gates pass.
- `sim/runtime/real_start_measurements.py` and
  `sim/runtime/initial_hold_pose.py`: real-start and initial-hold hotspots.
  Keep these tied to the Bar30/static-pressure and real-start mismatch gates.
