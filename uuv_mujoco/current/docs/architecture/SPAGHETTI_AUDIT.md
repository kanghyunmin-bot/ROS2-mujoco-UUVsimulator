# Active MuJoCo Runtime Spaghetti Audit

Audit date: 2026-06-04

Scope: active runtime code under `uuv_mujoco/current` only.  The backing
directory is still named `uuv_mujoco/v2.2` for compatibility, but launchers,
GUI wrappers, Docker defaults, and setup verification must resolve through
`current`.  This audit is behavior-neutral and does not
change ArduPilot, the ArduPilot submodule pointer, controller parity shims, PWM
remaps, or plant input semantics.

Latest refresh: 2026-06-07.  The primary runner has been renamed to
`run_uuv_mujoco.py`; `run_urdf_full.py` is now only a compatibility wrapper.
The runner hotspot is reduced to `763 LOC`, `18` branch nodes, with `main()` at
`682 LOC`.  This is still too large, but loop entry/cleanup, ROS bridge
lifecycle ownership, and SITL servo handler binding are now separated from
plant runtime wiring.
Runner setup ownership has since moved further into
`sim/runtime/control_bridge_setup.py` and `sim/runtime/model_io_setup.py`, and
unused local unpacking was removed.  The current runner inventory is
`612 LOC`, `15` branch nodes, with `main()` at `541 LOC`.
Physics setup ownership has now moved out of the runner into
`sim/runtime/physics_runtime_setup.py`, with step-time callbacks separated into
`sim/runtime/physics_step_callbacks.py`.  The runner is reduced to `335 LOC`,
`7` branch nodes, with `main()` at `281 LOC`.
MAVLink command target and heartbeat helpers are now owned by
`bridge/sitl_command_targets.py`, reducing `bridge/sitl_commanding.py` from
`724 LOC / 172` branches to `533 LOC / 100` branches.
The remaining SITL command send policies are now split into
`bridge/sitl_auto_ready_runtime.py`, `bridge/sitl_arm_mode_runtime.py`, and
`bridge/sitl_rc_manual_runtime.py`.  `bridge/sitl_commanding.py` is reduced to
an `85 LOC / 0` branch compatibility export surface.
ROS publish derived-state preparation is now owned by
`bridge/ros2_publish_state.py`, reducing `bridge/ros2_publish_runtime.py` from
`545 LOC / 53` branches with `flush_ros_publish_jobs()` at `510 LOC` to
`521 LOC / 51` branches with `flush_ros_publish_jobs()` at `485 LOC`.  The
lazy message builders were then split by output surface, reducing
`bridge/ros2_publish_runtime.py` further to `42 LOC / 2` branches with
`flush_ros_publish_jobs()` at `31 LOC`.
SITL MAVLink stream requests and PWM frame handling are now separated into
`bridge/sitl_mavlink_requests.py` and `bridge/sitl_pwm_runtime.py`, reducing
`bridge/sitl_mavlink_runtime.py` from `445 LOC / 103` branches to
`272 LOC / 60` branches.
The top-level MuJoCo runner loop now delegates headless/viewer selection and
cleanup to `sim/runtime/runtime_loop_entry.py`; mutable ROS bridge publish,
spin, and shutdown state is owned by `sim/runtime/ros_bridge_runtime.py`.
SITL and plant-replay servo runtime creation plus bridge handler binding are
now owned by `sim/runtime/sitl_servo_runtime.py`.
Active launch, GUI, Docker, cleanup, install, and verification entry points now
resolve the simulator through `uuv_mujoco/current` by default.  Missing
`current` is treated as a setup error instead of silently launching the
compatibility backing directory.
The simulator-loop public API methods for `Ros2Bridge` are now owned by
`bridge/ros2_bridge_public_api.py`, reducing `bridge/ros2_bridge.py` from
`645 LOC / 98` branches to `518 LOC / 46` branches by the project inventory
counter.
Simulation profile defaults/loading are now separated from hydrodynamics
coefficient parsing.  `physics/sim_profile_helpers.py` is the compatibility
facade for profile aliases/defaults/loading, `physics/sim_profile_types.py`
owns the typed config dataclasses, and
`physics/sim_profile_hydrodynamics.py` owns `build_hydrodynamics_config()`.
Dynamic MuJoCo `fluidcoef` behavior is now split into a facade plus typed setup,
load-factor math, runtime update, and setup-builder modules.  The old
`sim/physics/dynamic_fluidcoef.py` hotspot is reduced from `567 LOC / 37`
branches to a `25 LOC` compatibility export surface.
Runtime profile selection is now split into listing/alias resolution, profile
body assembly, and thruster-voltage override helpers, so static profile
contracts and opt-in dynamic fluid-coefficient experiments stay visible at
startup.  Core ROS sensor publish builders now use separated message factories
and lazy cache ownership; the facade no longer imports MuJoCo state types just
to build/test IMU, depth, Bar30 pressure, ground-truth, or sim-time messages.
SITL `SERVO_OUTPUT_RAW` telemetry mirroring is now separated into replay-real
timestamp, RCOut message, and event-publish policy modules, keeping the
controller-parity observation layer distinct from plant input ownership.
Runtime transport cleanup continued by splitting QGC video streaming,
development OS Python runtime selection, MANUAL_CONTROL neutral priming and
send logging, sensor/VPD replay CSV loading, MAVLink peer wait policy, DVL
publish factories/cache, and ROS bridge publish/spin/shutdown failure handling.
The top inventory no longer includes `bridge/qgc_video_stream.py`,
`tools/dev_os_compat_python_runtime.py`,
`bridge/sitl_manual_control_runtime.py`, `bridge/sitl_replay_loaders.py`,
`bridge/sitl_mavlink_peer.py`, `bridge/ros2_publish_builder_dvl.py`, or
`sim/runtime/ros_bridge_runtime.py`.
Closed-loop contract audit now resolves `uuv_mujoco/current` first before
falling back to the compatibility backing path.
`tools/roll_stability_sweep.py` has been split into candidate definitions,
temporary file edits, metrics, ROS probe, and launcher/result helpers.  The
CLI entry now stays below the top hotspot list and `--help` no longer imports
ROS2/rclpy.
`tools/axis_rc_override_check.py` has been split into RC constants, metrics,
plot/output IO, ROS node, and a CLI entry.  The command-line help path no
longer imports ROS2/rclpy or matplotlib.
`tools/physics_contract_audit.py` has been split into typed records, MuJoCo
model calculations, report/output helpers, and a CLI entry.  The command-line
help path no longer imports MuJoCo, while the real audit still executes through
the active runtime and writes the same CSV/JSON report shape.
`tools/audit_code_contract_sources.py` is now a thin active-runtime CLI.  Source
identity checks, ArduPilot contract checks, active runtime surface checks, and
JSON/Markdown report writing are split into focused modules; generated reports
now state `active_runtime_root` and keep `compat_v22_root` only as backing-path
metadata.
GUI layout construction is now split by panel family.  `gui/layout_mixin.py`
is a compatibility facade, while shell/telemetry/control/replay/tuning/pilot
sections live in focused `gui/layout_*` modules.  GUI arm/mode, common command
gates, and RC/manual/Ping360 publishers are now split out of
`gui/node_commanding.py`, leaving that file as a compatibility facade.  This
keeps the active runtime import surface stable while removing stale monoliths
from the hotspot list.
Source-contract audit checks are now split by responsibility:
path registry, ArduPilot source identity, firmware contract checks, active
runtime surface checks, and thruster/plant-gate checks.  The public
`tools/audit_code_contract_checks.py` module is a small compatibility facade
and still produces the same `fail=0`, `warn=5` source-audit result.  Runtime
identity now adds the `active_runtime_alias_current` check, so the current
source-audit count is `pass=11`.
Runtime CLI option construction is also split by option family
(`sim/runtime/cli_*`), with `sim/runtime/cli.py` reduced to parser assembly
only.  ALT_HOLD diagnostics now separates contract math, ROS node capture,
CSV/plot/summary output, and CLI entrypoint.  `bridge/ros2_bridge_config.py`
now re-exports MAVROS, pressure/IMU/vertical, and frame-transform config
modules so source-audit evidence follows the active split locations.
Active-runtime freshness is now recorded in `uuv_mujoco/RUNTIME_VERSION.json`;
setup and closed-loop audit paths resolve `uuv_mujoco/current` first and treat
`v2.2` only as the compatibility fallback.  GUI simulator stack process
ownership is split into launch, reset, and status mixins, and GUI
manual-control/display ownership is split into pilot command, panel toggle,
canvas drawing, feedback, and UI update modules.
The freshness gate now distinguishes the active alias from the compatibility
backing directory.  Launching through `uuv_mujoco/current` still passes source
freshness, while explicit `uuv_mujoco/v2.2` runtime selection reports
`direct_v22_runtime` and the root GUI/MuJoCo/SITL wrappers refuse to continue.
Simulation profile hydrodynamics parsing is now split from ellipsoid-derived
baseline estimation and final config assembly.  ExternalNav/VPD transport is
split into bootstrap, native VPD replay, synthetic VPD generation, and
cache/contract checks.  Physics runtime wiring is split into shared records,
hydrostatic context setup, thruster geometry overrides, and the top-level
factory facade.
ALT_HOLD contract analysis, SITL transport state/handler/lifecycle ownership,
and GUI auto-tune monitor window/log/candidate/chart ownership have now been
split into focused modules.  The active `current` alias still points to the
compatibility backing directory `v2.2`, but all current runner, GUI, Docker,
setup, and validation paths must be checked through `uuv_mujoco/current`.
Static physics contract model calculations are now split into geometry,
runtime body contract, buoyancy, and neutral open-plant simulation modules
while preserving the static force-balance audit output.
GUI-started simulator environment construction is now split into env-contract,
argument-normalization, flag, and typed-result modules.  The single-owner
underwater wrench runtime now delegates hydrostatic buoyancy/restoring and
hydrodynamic residual/CFD/damping ownership to focused runtime helpers.
Real-start state extraction is now split into CSV selection, frame conversion,
row-field extraction, Bar30 datum math, state assembly, and output formatting
modules while preserving the CLI output contract.
SITL transport construction is now split by JSON socket, RC/control state,
MAVLink command/telemetry/polling state, and ExternalNav runtime state.
GUI physics tuning is now split by window construction, profile IO/parsing, and
restart orchestration while preserving the same editor controls.
Runtime freshness is now an audited contract: `uuv_mujoco/current` must resolve
to the compatibility backing directory, the primary runner must exist under
`current`, and `uuv_mujoco/RUNTIME_VERSION.json` must identify `current` as the
active runtime.
Runtime freshness now also records source branch and dirty-runtime evidence.
The source-contract audit reports the active git branch, local HEAD,
`origin/uuv_sim` HEAD, changed working-tree path count, active runtime dirty path
count, and ArduPilot submodule status so a stale checkout, accidental
`main`/`master` merge, or uncommitted local runtime is visible instead of being
hidden behind the old `v2.2` directory name.
User-facing status should now use the active runtime label from
`uuv_mujoco/RUNTIME_VERSION.json`, currently
`current-2026-06-07-uuv_sim`, not "latest v2.2".  The directory
`uuv_mujoco/v2.2` remains only the compatibility backing path because renaming
it while the workspace is dirty would invalidate old debug evidence and risk
breaking existing launch/report paths.  Freshness is therefore a runtime alias
and source-HEAD contract, not a folder-name claim.
The 2026-06-07 freshness pass rechecked that contract:
`uuv_mujoco/current -> v2.2`, local `HEAD` equals `origin/uuv_sim` at
`e60cd9383c214a7d6e4700cdeb5c2f2a8cf6d328`, and
`check_runtime_freshness.py --fetch --refresh-version` now reports `WARN` when
the live runtime is dirty.  Treating `v2.2` as "latest" is therefore incorrect;
the active target is `current`, and the current local state must be reported as
`current-dirty` until the runtime changes are intentionally committed or
cleaned.
Root launch wrappers now execute `tools/check_runtime_freshness.py` with a
non-destructive `git fetch origin --prune --quiet` before MuJoCo/SITL/Docker
SITL/reset entry points.  The script warns on local/remote HEAD mismatch,
missing `current`, or stale `RUNTIME_VERSION.json`, so the compatibility
backing directory name no longer hides source freshness drift during normal
launches.
The freshness checker now supports `--refresh-version`; GUI and SITL launchers
use it so `uuv_mujoco/RUNTIME_VERSION.json` is automatically refreshed when
there are no hard source/runtime failures.  Dirty working-tree state is not
hidden; it is recorded in `dirty_state` and changes the version status to
`current-dirty`. If hard source/runtime checks fail, the runtime version file is
left untouched and the stale contract is reported.
The root MuJoCo, root SITL, root Docker SITL, and root reset wrappers now also
pass `--refresh-version`; previously they checked freshness but did not refresh
the runtime version file on every launch.
GUI entry points now run the same freshness preflight before loading the active
GUI runtime.  This covers `run_control_gui.sh`, `run_control_gui_ubuntu.sh`,
the root `uuv_control_gui.py` compatibility entry point, and the direct
`uuv_mujoco/current/start_sitl_mujoco_mj311.sh` path used by manual tests.
Dynamic MuJoCo fluidcoef runtime knobs are now split by update-rate smoothing,
transient mode selection, transient thresholds, and coefficient mask/smoothing.
`sim/physics/dynamic_fluidcoef_runtime_knobs.py` remains as the compatibility
facade, and the fake-runtime smoke verifies clipping, disabled mode handling,
rearm/reset ordering, fallback decay masks, and lift smoothing.
Thruster debug CSV handling is now split by file/header IO and row payload
construction. `sim/runtime/thruster_debug_runtime.py` remains the public runtime
facade for sampling cadence and write calls, while the row builder is directly
smoke-tested for header width, 20 Hz hold behavior, and `mj_forward` call
placement.
Physics step callback assembly is now split by thruster command callbacks and
auxiliary underwater/debug/descent-guard callbacks. The public
`build_step_physics_callbacks` entry point remains, and the smoke verifies that
scheduler, force update, propeller visuals, direct-command mixing, underwater
wrench application, and debug emission remain reachable.
Ping360 simulator state is now split by polar history buffers and sweep angle
state. `bridge/ping360_sim.py` remains the public MuJoCo sensor lifecycle class,
while `bridge/ping360_history.py` owns image/range/intensity buffers and
`bridge/ping360_sweep.py` owns full-scan/sector-bounce angle progression.
Held/updated sample construction and status payload creation now live in
`bridge/ping360_samples.py`, keeping ROS-facing payload generation outside the
simulator lifecycle class.
GUI app state initialization is now split into the root/runtime state owner,
core Tk variable groups, feature-specific Tk variable groups, and small Tk var
helpers. `gui/app_state_vars.py` is now a facade and no longer imports the ROS
runtime layer, so variable initialization can be smoke-tested without loading
`rclpy`.
Runtime freshness evaluation is now split by contract role. The public
`tools/runtime_freshness_eval.py` entry point only assembles the report, while
runtime alias/runner checks, git source freshness checks, dirty working-tree
checks, and `RUNTIME_VERSION.json` checks live in focused modules. This keeps
the `current` versus compatibility `v2.2` contract auditable without one
branch-heavy function owning every failure mode.
Runtime freshness version metadata is now split one layer further. Constants,
dirty-state payloads, active-alias/root-launcher paths, and final
`RUNTIME_VERSION.json` payload assembly live in focused
`tools/runtime_freshness_version_*` modules, while
`tools/runtime_freshness_version.py` remains only the refresh compatibility
entry point. The source-contract audit still reports `fail=0`, `pass=11`,
`warn=5`, and `tools/runtime_freshness_version.py` has dropped out of the top
hotspot list.
GUI physics tuning window construction is now split into the public window
method, window-shell/scroll-frame helpers, and parameter-grid row rendering.
`gui/physics_window.py` remains the mixin import surface, while
`gui/physics_window_shell.py` owns Toplevel/header/status/scroll setup and
`gui/physics_window_rows.py` owns the parameter table/footer widgets.
GUI ROS-panel ownership is now split into visibility, process-state predicates,
thread-safe status updates, and button-label refresh helpers.  The public
`gui/ros_panel_mixin.py` compatibility surface still exposes the same GUI
methods, but the module no longer appears in the top hotspot list.  The split
also removed an avoidable `.runtime` import from status updates so the panel
binding smoke can run without importing `rclpy`.
ROS bridge math helpers are now split by contract role. `bridge/ros2_math.py`
is a compatibility export surface; RC channel normalization, scalar finite/angle
helpers, quaternion conversion, RPY rotation matrices, Bar30 pressure conversion,
and optional message setters live in separate focused modules. This removes the
mixed math/message helper from the hotspot list without changing equations or
runtime import names.
ROS sensor message builders are now split by published message family.
`bridge/ros2_sensor_messages.py` remains the compatibility surface while IMU,
fluid pressure, range, battery, and MAVROS VFR HUD builders live in focused
modules. This keeps Bar30/static-pressure and IMU message contracts auditable
without changing ROS field names, covariances, or frame IDs.
Ping360 ROS message builders are now split by topic family. The compatibility
surface remains `bridge/ros2_ping360_messages.py`, while image, LaserScan,
sonar echo, and status payload/message builders live in separate modules.
`bridge/ping360_image_renderer.py` now imports constants and data contracts from
`ping360_types`, avoiding a MuJoCo import when only Ping360 ROS message fields
or renderer math are under test.
MAVROS bridge configuration is now split by policy owner. The compatibility
surface remains `bridge/ros2_bridge_config_mavros_rates.py`, while MAVROS
state defaults, ROS/MAVROS sensor rates, RCOUT publication policy, battery
defaults, and replay bookkeeping live in focused modules. This removes a
mixed state/rate/policy hotspot without changing RCOUT mode fallback or sensor
rate defaults.
SITL replay initialization is now split into config parsing, CSV/VPD loaders,
mutable replay state reset, and startup logging.  `bridge/sitl_initialization.py`
remains the public initializer, but env parsing, frame loading, and runtime
state mutation no longer live in one branch-heavy function.
The GUI auto-tune workflow has been removed from the active GUI runtime rather
than hidden behind disabled controls.  The app no longer inherits an
auto-tune mixin, the tuning panel now exposes physics tuning only, GUI
auto-tune state variables/process cleanup/path exports are gone, and the
monitor/process helper modules were deleted.  Non-GUI research or tuning tools
outside the GUI surface remain separate unless explicitly targeted.
ROS2 static TF ownership is now split by concern: quaternion/site pose helpers,
world/body alias frames, onboard sensor frames, stereo camera frames, and the
thin static-spec assembly facade.  `bridge/ros2_tf_messages.py` remains the
compatibility import surface for `build_tf_message`, `build_static_tf_specs`,
and quaternion helpers, while the previous 68 LOC static TF builder is gone
from that module.
MAVLink message interval ownership is now split between stream-throttle state
and low-level send mechanics.  `sim/transport/mavlink_message_interval.py`
keeps the public `MavlinkMessageIntervalRequester` API, while
`mavlink_message_constants.py` owns requested AP telemetry constants and
`mavlink_message_interval_send.py` owns message-id resolution,
`requested_hz` to microsecond interval conversion, and
`MAV_CMD_SET_MESSAGE_INTERVAL` `command_long_send` payload construction.
QGC video runtime ownership is now split into setup, frame rendering, and
lifecycle cleanup helpers.  `sim/runtime/qgc_video.py` keeps the public
`QgcVideoRuntime` wrapper, while `qgc_video_setup.py` owns enable/disable
checks and streamer/renderer creation, `qgc_video_frame.py` owns publish-rate
gating and shared-renderer fallback, and `qgc_video_lifecycle.py` owns close
semantics.
GUI simulator reset ownership is now split into command execution, reset-log
line parsing, and result-to-GUI state updates.  `gui/sim_stack_reset_mixin.py`
keeps the existing reset/stop method surface, while
`sim_stack_reset_commands.py` owns reset/stop process execution,
`sim_stack_reset_output.py` owns `[reset]` line filtering/shortening, and
`sim_stack_reset_result.py` owns success/failure status and event updates.
SITL MAVLink transport configuration is now split by base MAVLink settings,
dedicated command link, auto-ready state, telemetry observer, and polling
cadence.  Runtime freshness probing is also split by git probe, dirty path
parsing, JSON metadata loading, and active-runtime alias resolution.  The
freshness path still reports the compatibility backing name `v2.2`, while
source freshness is judged by `uuv_mujoco/current`,
`uuv_mujoco/RUNTIME_VERSION.json`, local `HEAD`, and `origin/uuv_sim`.
MAVROS RC override input is now split by channel-frame extraction, SITL
forwarding, `/mavros/rc/in` mirroring, warning throttles, and top-level callback
policy.  `bridge/ros2_rc_override_input.py` remains the compatibility export
surface, while the latency-sensitive forwarding path can now be inspected
without mixing GUI mirror behavior or local fallback normalization into the same
function.
MAVROS arm/mode service forwarding is now split by boot-guard checks,
forwarding-enable checks, locked SITL transport calls, forwarding policy, and
ROS response callbacks.  `bridge/ros2_mavros_arm_mode_services.py` remains the
compatibility import surface, while arm/mode failures can now be attributed to
boot guard, disabled forwarding, or SITL command send independently.
Pending SITL arm/mode retry service loops are now split by state predicates,
MAVLink broadcast/neutral-RC priming, and retry policy.  The compatibility
`bridge/sitl_arm_mode_service.py` file no longer owns timeout, resend-cadence,
target-reached, and command-send details in one function.
Ros2Bridge public spin/publish ownership is now split by shared SITL servo
polling, command timeout clearing, executor spin cadence, publish timing,
timestamp acquisition, and prepared-snapshot ROS publication.  The compatibility
`bridge/ros2_bridge_spin_publish.py` file now only re-exports `spin_once` and
`publish`, so ROS callback failures and sensor publish gating are no longer in
one branch-heavy public-loop file.
ROS runtime spin utilities are now split by context-shutdown classification,
safe publish failure handling, spin-loop state predicates, cached ExternalNav
sends, spin-loop policy, and dedicated executor thread startup.  The
compatibility `bridge/ros2_runtime_spin.py` file now only exports the historical
runtime utility names used by `Ros2Bridge` method bindings.
SITL JSON sender ownership is now split by finite-value validation, sample and
send diagnostics, compact JSON encoding, receiver-state synchronization, and
top-level packet send policy.  The compatibility `bridge/sitl_json_sender.py`
file now only re-exports `_send_sitl_json_payload`, preserving the ArduSub
4.1.2 no-space vector JSON contract while removing send-path diagnostics from
payload validation.
SITL MAVLink connection ownership is now split by endpoint/default resolution,
`pymavlink` dependency loading, servo-link setup, command-link
connect/reconnect, and GCS heartbeat timestamp synchronization.  The
compatibility `bridge/sitl_mavlink_connection.py` file now only re-exports the
historical connection helper names consumed by transport bindings.
SITL status payload ownership is now split by wall-clock age helpers, sensor
replay status payloads, MAVLink core status fields, and ExternalNav readiness
fields.  The compatibility `bridge/sitl_status.py` file now preserves the
historical `build_sensor_replay_status`, `build_mavlink_telemetry_status`, and
`wall_age_s` imports while keeping GUI readiness/status JSON keys unchanged.
The git probe now preserves porcelain status leading columns, so dirty evidence
paths are recorded verbatim instead of dropping a leading character from the
first path.
Vehicle HEARTBEAT and COMMAND_ACK handling is now split by source filtering,
vehicle state recording, and ACK logging/constant resolution.  The public
`bridge/sitl_vehicle_heartbeat.py` module remains as the compatibility export
surface used by command bindings.  ExternalNav wall-clock cache ownership is
also split from required-output/stale-rate contract enforcement, so live
ExternalNav cache behavior and VISION_POSITION_DELTA contract failures can be
tested independently.
Axis RC service helpers are now split by stack readiness, neutral-spin cadence,
arming service, mode service, and trigger service.  ALT_HOLD DataFlash BIN
analysis is split by BIN stream schema, path selection, message collection,
array conversion, and signal math.  The compatibility facades keep existing
CLI imports stable while isolating RC command timing and log timebase parsing
for focused smoke tests.
Equivalent-ellipsoid hydrodynamics are now split by geometry, coefficient
assembly, and output type ownership. The public
`physics/ellipsoid_hydrodynamics.py` module remains a compatibility facade, so
profile loading and helper exports keep the same API while the baseline
equations are easier to inspect before any HAN/CFD coefficient correction.
Neutral open-plant physics-contract simulation is now split into the public
MuJoCo loop, hydrostatic force application, and CSV/summary helpers. The audit
still exercises the same static force-balance path and a nonzero neutral drift
simulation, but the body-component buoyancy force contract is no longer hidden
inside one long simulation function.
Runtime weighted hydrostatic buoyancy is now split into sample conversion,
force accumulation, and public point/component wrench entry points. The CoB
offset contract still affects torque force points without moving the volume
sample that determines near-surface displaced water.
Thruster performance loading is now split into JSON payload IO, curve
parsing/selection, and the public loader. The direct T200 PWM-to-force contract
still selects the nearest voltage curve and preserves the legacy dict payload.
Runtime thruster parameter handling is now split into state creation, JSON
file/override loading, and diagnostic summary logging. The public
`sim/runtime/thruster_param_runtime.py` facade still owns the
`ThrusterParameterRuntime` API, and this pass does not change thruster
coefficients, servo mapping, direct T200 curves, reverse asymmetry values, or
first-order actuator time constants.
Roll-stability sweep candidate definitions are now split into typed records,
hydrostatic candidates, ellipsoid damping candidates, diagnostic candidates,
and servo-sign diagnostic candidates. The public
`tools/roll_stability_candidates.py` facade still exports `Candidate` and
`default_candidates()`, preserving candidate order, names, profile updates,
servo signs, and CLI selection behavior.
GUI-started simulator forced environment construction is now split by RC
override contract, EKF/sensor contract, and command/readiness contract. The
public `gui/sim_stack_env_forced.py` entry point still assembles the same
forced key/value map for closed-loop and plant-replay modes, including RC PWM
span, Bar30 pressure, EXTNAV flags, readiness behavior, and plant-replay
RCOU override gating.
Roll-stability sweep execution is now split into path records, original-file
snapshots/restoration, and the candidate execution loop. The public
`tools/roll_stability_sweep.py` CLI still owns argument parsing and candidate
selection, while the loop helper preserves candidate order, temporary
profile/scene/mapping edits, summary writing, launcher reset, and guaranteed
original-file restoration.
Auto-tune monitor window ownership is now split into the public lifecycle
helpers, shell/header/progress construction, candidate table/chart layout, and
live-log layout. The public `gui/autotune_monitor_window.py` behavior remains
the same: existing windows are raised, new windows populate existing candidate
rows, reset clears monitor state/log/tree, and finish status text still maps
`rc=0`, `rc=130`, and failure lines identically.
GUI node initialization is now split from the public node surface.  Publisher,
subscriber, service-client, state, and real-start wiring live in
`gui/node_init.py`; vehicle-info request/response handling lives in
`gui/node_vehicle_info.py`, keeping the ROS node public surface smaller while
preserving callback names used by the GUI; and `gui/node.py` is reduced from
`341 LOC / 19` branches to `104 LOC / 1` branch.
Ping360 settings parsing is now split by setting family.  Interface timing,
range, transmit, and angle normalization live in focused modules while
`bridge/ping360_settings.py` remains the public compatibility facade used by the
simulator and GUI paths.
GUI control-update formatting is now split from widget mutation.  Snapshot text
models, telemetry text formatting, pilot/RC text formatting, and widget apply
helpers live in `gui/control_update_*` modules, so the GUI periodic update path
can be smoke-tested without importing the full ROS runtime.
GUI backend probing is now split by graph-count collection, backend label/layout
helpers, and scoring/selection policy.  `gui/node_backend_runtime.py` remains a
small compatibility facade for `UuvGuiNode`, while backend readiness causes are
traceable through `gui/node_backend_counts.py`,
`gui/node_backend_layout.py`, and `gui/node_backend_selection.py`.
ROS2 bridge construction is now split from the public bridge facade.
Constructor runtime state lives in `bridge/ros2_bridge_runtime_setup.py`,
MuJoCo sensor/Ping360 lookup lives in `bridge/ros2_bridge_sensor_setup.py`,
and the compatibility method binding table lives in
`bridge/ros2_bridge_method_bindings.py`.  `bridge/ros2_bridge.py` is reduced
from `330 LOC / 3` branches to `153 LOC / 1` branch.
Dynamic MuJoCo `fluidcoef` runtime configuration is now split into
`sim/physics/dynamic_fluidcoef_runtime_config.py`, leaving
`sim/physics/dynamic_fluidcoef_runtime.py` focused on the per-step update and
reducing it from `297 LOC / 15` branches to `139 LOC / 13` branches.
Ping360 raycast and return-profile synthesis are now split into
`bridge/ping360_beam_model.py` and `bridge/ping360_profile.py`, leaving
`bridge/ping360_sim.py` focused on scan scheduling and rolling image/range
state.  `bridge/ping360_sim.py` is reduced from `286 LOC / 38` branches to
`138 LOC / 12` branches.
GUI ROS/RViz process ownership is now split into panel status, logged-process
watching, MAVROS package launch, and RViz launch mixins.  The public
`gui/ros_process_mixin.py` facade is reduced from `281 LOC / 45` branches to
`16 LOC / 0` branches.
GUI telemetry subscription callbacks are now split by contract surface.
Vehicle state/status text lives in `gui/node_vehicle_callbacks.py`, motion,
depth, pressure, and battery handling live in `gui/node_motion_callbacks.py`,
RC input/output handling lives in `gui/node_rc_callbacks.py`, SITL/real-start
JSON status handling lives in `gui/node_sitl_status_callbacks.py`, and
Ping360 status parsing lives in `gui/node_ping360_callbacks.py`.  The public
`gui/node_telemetry_callbacks.py` facade is reduced from `281 LOC / 38`
branches to an import-only compatibility surface.
GUI node state/runtime helpers are now split by responsibility.  Backend graph
probing and RC layout selection live in `gui/node_backend_runtime.py`,
snapshot/event helpers live in `gui/node_snapshot_runtime.py`, and command
readiness calculation lives in `gui/node_readiness_runtime.py`.  The public
`gui/node_state_runtime.py` facade is reduced from `276 LOC / 48` branches to
an import-only compatibility surface, keeping the READY/WAIT policy testable
without mixing it with graph probing.
SITL JSON sensor output is now split by contract role.  Payload construction,
including the ArduSub Bar30 frontend-match contract `position.z` evidence,
lives in `bridge/sitl_json_payload.py`; UDP packet validation/send logging
lives in `bridge/sitl_json_sender.py`; servo-frame immediate replay replies
live in `bridge/sitl_json_replay_reply.py`; and live/replayed send policy lives
in `bridge/sitl_json_sensor_send.py`.  The old
`bridge/sitl_json_sensor_runtime.py` module is an import-only compatibility
surface.  The source-contract audit now points at `bridge/sitl_json_payload.py`
for JSON position/depth evidence.
Fossen-style residual hydrodynamics are now split into coefficient key/type
contracts, runtime coefficient builders, and wrench evaluation.  The public
`sim/physics/fossen_residual.py` module now only re-exports
`sim/physics/fossen_residual_types.py`,
`sim/physics/fossen_residual_builders.py`, and
`sim/physics/fossen_residual_wrench.py`.  This keeps HAN/CFD residual
experiments inspectable without changing the coefficient equations.
The builder surface is also split internally: config/log helpers live in
`sim/physics/fossen_residual_config.py`, coefficient extraction and nonzero-term
reporting live in `sim/physics/fossen_residual_coefficients.py`, added-mass
matrix construction lives in `sim/physics/fossen_residual_added_mass.py`, and
activation gates live in `sim/physics/fossen_residual_runtime_flags.py`.
`sim/physics/fossen_residual_builders.py` remains the public constructor
facade, not an equation dump.
SITL replay CSV handling is now split into typed records, common numeric
helpers, CSV loaders, and interpolation.  The public
`bridge/sitl_replay.py` module now re-exports
`bridge/sitl_replay_types.py`, `bridge/sitl_replay_common.py`,
`bridge/sitl_replay_loaders.py`, and `bridge/sitl_replay_interpolation.py`.
This keeps real-sensor replay loading and interpolation inspectable without
changing replay timing semantics.
SITL sensor-replay frame selection is now split by replay-clock responsibility.
Hold-state logging and first-frame/pre-roll retention live in
`bridge/sitl_sensor_replay_hold_policy.py`; RC-start and pre-roll transition
math lives in `bridge/sitl_sensor_replay_start_policy.py`; and
`bridge/sitl_sensor_replay_frame_policy.py` now only selects/interpolates the
current replay frame.  A focused smoke covers empty frames, servo-clock wait,
RC-input hold, RC pre-roll, post-start interpolation, and non-RC-start replay.
Simulation profile defaults are now separated into
`physics/sim_profile_defaults.py`; `physics/sim_profile_helpers.py` remains the
public facade for profile loading, canonical naming, hydrodynamics config
construction, and old imports.  GUI node initialization is now split into state,
publisher/client, and subscription modules while preserving the `gui.node_init`
entry point.  Standard ROS2 message builders are now split into pose, twist,
sensor, and status modules while preserving the `bridge.ros2_standard_messages`
import surface.  These three files are removed from the top hotspot inventory
without changing profile constants, topic names, callback bindings, or message
field contracts.
Ping360 GUI control ownership is now split by window layout, viewer process
control, and sonar config publishing while preserving
`gui.ping360_mixin.Ping360ControlMixin`.  The virtual joystick widget now keeps
the public `VirtualJoystick` class but delegates axis math and canvas rendering
to focused helper modules.  ROS2 topic registry data is separated from summary
string generation while preserving `bridge.ros2_topic_registry`.  Underwater
hydrodynamic application is split into custom diagonal damping, residual/Fossen
and CFD terms, and empirical pitch/lift/heave terms while preserving the public
`apply_hydrodynamic_wrenches()` entry point and the static physics audit result.
SITL MAVLink runtime ownership is now split into connection/heartbeat helpers,
telemetry observer wrappers, and servo/command polling loops.  The public
`bridge/sitl_mavlink_runtime.py` module now only re-exports
`bridge/sitl_mavlink_connection.py`, `bridge/sitl_mavlink_telemetry.py`,
`bridge/sitl_mavlink_polling.py`, plus the existing request/PWM helper exports.
This keeps the `SERVO_OUTPUT_RAW` telemetry path inspectable while preserving
the `SitlTransport` binding surface.
Roll-stability probe execution is now split by runtime role.  The public
`tools/roll_stability_probe.py` module composes focused callback, RC publish,
arm/mode command, and probe-sequence modules, preserving `StabilityProbe` while
removing another mixed ROS2 node/analysis hotspot.
GUI ROS/RViz utility ownership is now split into ROS setup/shell command
helpers and RViz config generation helpers.  `gui/ros_tools.py` now re-exports
`gui/ros_env_tools.py` and `gui/rviz_config_tools.py`, preserving the existing
GUI imports while removing RViz YAML generation from the ROS environment
helper surface.
RC contract ownership is now split into constants, marker-preserving frame
normalization, axis metadata, joystick scaling, and ALT_HOLD climb-rate math.
`sim/contracts/rc.py` remains the public compatibility facade, so existing
GUI/SITL imports keep the same behavior while the RC override contract is
inspectable by responsibility.  GUI-start simulator env construction is also
split into run/EKF mode normalization, runtime profile and command endpoint
defaults, and forced GUI contract application.  Invalid
`UUV_RUNTIME_PROFILE` values now normalize to `balanced`; valid `low` and
`high` profiles are preserved.  Runtime freshness was rechecked after the
split and reports `current -> v2.2`, local `HEAD` equal to `origin/uuv_sim`,
and `RUNTIME_VERSION=current`.
ROS2 endpoint construction is now split by endpoint family.  Publisher,
subscription, and service creation live in focused
`bridge/ros2_endpoint_*` modules while `bridge/ros2_endpoints.py` remains the
public facade used by bridge initialization.  Hydrostatic runtime setup is also
split by responsibility: typed records, source selection, env/profile value
extraction, CoB site alignment, and logging/reporting.  The static physics
contract audit remains unchanged after the split with neutral buoyancy
`net_down=+0.000N` and `required_scale=1.000000`.
ROS2 publisher endpoint construction is now split one level deeper by contract
surface.  Core simulator sensors/status, Ping360, MAVROS-compatible telemetry,
DVL compatibility, and TF/robot_description publishers live in separate
`bridge/ros2_endpoint_*_publishers.py` modules while
`bridge/ros2_endpoint_publishers.py` remains the public facade.  A fake-node
smoke verifies topic names, optional Ping360 SonarEcho handling, optional RCOut
handling, and disabled-MAVROS publisher state.
MAVROS configuration now separates RC mapping/setpoint contract from sensor
rate, RCOUT publish-mode, battery, and replay-state initialization.
`bridge/ros2_bridge_config_mavros_rates.py` owns the real-robot rate defaults
and RCOUT event/rate-limited policy, so `/mavros/rc/out` observation timing can
be audited without reading joystick/channel mapping code.
SITL transport method binding is now separated from transport initialization.
`bridge/sitl_transport.py` keeps the public `SitlTransport` class and the
constructor order, while state/replay, handler/lifecycle, command, MAVLink,
ExternalNav, and JSON servo/sensor bindings live in focused
`bridge/sitl_transport_*_bindings.py` modules.  A binding-surface check
verified all `84` expected method/property names remain present.  GUI RC helper
ownership is split into PWM conversion, ArduSub joystick/ALT_HOLD diagnostics,
axis normalization, and RC message construction while preserving
`gui.gui_rc_helpers` as the import facade.  The axis RC override validation CLI
now separates argument parsing, vehicle/phase sequence, and reporting; its
`--help` path still avoids importing ROS2/rclpy.
Ping360 ROS message construction is now split from polar image rendering:
`bridge/ping360_image_renderer.py` owns lookup/cache and mono8 rendering while
`bridge/ros2_ping360_messages.py` keeps the public message-builder surface.
Axis RC validation node ownership is split into callback/sampling, RC/manual
control publishing, and service wrappers while preserving the public
`AxisRcOverrideCheck` node and `axis_rc_override_check.py --help` behavior.
Source-audit ownership is split by evidence family: JSON/Bar30/SERVO telemetry
checks, RC override/joystick checks, active runtime identity, and ArduPilot
source identity.  The audit still reports `16` checks with `PASS=11`,
`WARN=5`, `FAIL=0`.  Runtime physics factory wiring now uses a context record
plus hydro/thruster step helpers while preserving the public
`create_runtime_physics_setup()` signature and static physics audit result.
The current largest inventory entry after this batch is
`sim/transport/mavlink_telemetry_observer.py` at `197 LOC`.
MuJoCo fluid geom runtime scaling is now split into common geom matching/parsing
helpers, geom-size scaling, `fluidcoef` scaling, and a top-level application
orchestrator.  `sim/physics/fluid_geom_runtime.py` now only preserves the
compatibility import surface, so geom-size tuning and MuJoCo
`(blunt, slender, angular, Kutta, Magnus)` coefficient scaling can be audited
separately without changing the underlying coefficient contract.
GUI configuration ownership is now split by concern.  `gui/config_paths.py`
owns active-runtime/user workspace paths, `gui/config_backend.py` owns backend
and pilot mode selection, `gui/config_env.py` owns environment parsing,
`gui/config_rc.py` owns RC/pilot/channel layout constants, `gui/config_ui.py`
owns display timing/layout constants, and `gui/config_physics.py` owns the GUI
physics tuning schema.  `gui/config.py` remains the compatibility export
surface, so existing GUI imports keep working while the old mixed constants
file leaves the hotspot list.
Ping360 STL asset filtering is now split into binary STL IO, connected
component/cable-like classification, and a thin CLI pipeline.  This keeps the
asset-prep script inspectable without changing the generated mesh metadata
contract or the active Ping360 runtime.
GUI AutoTune controls have been removed from the active GUI runtime.  Historical
AutoTune split logs remain as evidence snapshots, but active GUI composition no
longer imports an AutoTune mixin or exposes AutoTune buttons.
Hydrodynamics helper ownership is now split without changing equations:
`physics/hydrodynamics_math.py` owns 6DOF math and added-mass Coriolis,
`physics/thruster_curve_helpers.py` owns command shaping and polynomial force
helpers, `physics/hydrostatic_fraction_helpers.py` owns waterline submerged
fraction models, and `physics/ellipsoid_hydrodynamics.py` owns the equivalent
ellipsoid baseline estimate.  `physics/hydrodynamics_helpers.py` remains a
compatibility export surface for existing plant, contract, and debug imports.
Static physics contract runner ownership is now split further: profile loading
and audit-only overrides live in `tools/physics_contract_profile_runtime.py`,
start-depth candidate construction lives in `tools/physics_contract_start_depth.py`,
neutral open-plant simulation dispatch lives in
`tools/physics_contract_neutral_runner.py`, and report dictionary assembly
lives in `tools/physics_contract_audit_report.py`.  The public
`tools/physics_contract_runner.py` still writes the same CSV/JSON shape and was
validated with the MuJoCo runtime Python.
Runtime physics assembly is now split beyond the old factory hotspot.  Thruster
parameter loading, SITL servo binding, actuator runtime creation, and thruster
debug CSV setup live in `sim/runtime/physics_runtime_thrusters.py`; underwater
wrench/body-velocity setup lives in `sim/runtime/physics_runtime_underwater.py`;
and final callback/result packaging lives in
`sim/runtime/physics_runtime_finalize.py`.  `physics_runtime_factory.py`
preserves the `create_runtime_physics_setup()` import surface.
Thruster actuator runtime construction is now split from runtime update
ownership.  `sim/runtime/thruster_actuator_setup.py` owns actuator-site lookup,
propeller joint maps, initial state/target/force dictionaries, and diagnostic
zero-vector allocation; `sim/runtime/thruster_actuator_runtime.py` now owns the
runtime state record and delegates force, visual, immersion, and direct-command
updates.  A fake MuJoCo model smoke verifies site IDs, propeller maps, zero
maps, force limits, yaw thruster lists, and diagnostic vector shapes.
Hydrostatic wrench runtime is now split by force source.  Weighted buoyancy
point/component force and torque calculations live in
`sim/runtime/underwater_hydrostatic_weighted.py`, release-blended restoring
torque lives in `sim/runtime/underwater_hydrostatic_restoring.py`, and
`sim/runtime/underwater_hydrostatic_runtime.py` now only selects the active
hydrostatic source and assembles the `HydrostaticWrenchResult`.

## Non-negotiable runtime boundary

Controller parity is:

```text
real /mavros/rc/out
vs
SITL MAVLink SERVO_OUTPUT_RAW telemetry
```

Plant input is:

```text
raw ArduSub JSON servo packet
to
MuJoCo thruster input
```

These two observation points must not be mixed.  Any refactor that makes JSON
servo look like low-rate `/mavros/rc/out` is invalid.

## Current hotspot inventory

Measured with Python AST on 2026-06-07.  Branch count is an approximate local
complexity indicator from `if`, loops, `try`, `with`, and boolean expressions.

| File | LOC | Branches | Main hotspot |
| --- | ---: | ---: | --- |
| `physics/sim_profile_defaults.py` | 174 | 0 | data-only profile defaults |
| `gui/layout_telemetry.py` | 153 | 2 | `build_vehicle_summary()` is 29 lines |
| `bridge/ping360_types.py` | 152 | 3 | `Ping360Config` is 59 lines |
| `gui/layout_control_core.py` | 152 | 2 | `build_sim_stack_controls()` is 29 lines |
| `tools/roll_stability_candidates.py` | 150 | 1 | `default_candidates()` is 130 lines |
| `tools/physics_contract_runner.py` | 143 | 7 | `run_physics_contract_audit()` is 114 lines |
| `gui/sim_stack_env_forced.py` | 142 | 4 | `base_forced_gui_contract()` is 50 lines |
| `bridge/sitl_transport_mavlink_config.py` | 141 | 6 | `initialize_mavlink_transport()` is 69 lines |
| `tools/runtime_freshness_probe.py` | 140 | 12 | `collect_freshness_inputs()` is 42 lines |
| `bridge/ros2_topic_specs.py` | 140 | 0 | typed topic/subscription records |

Reduced or removed from earlier hotspot lists on 2026-06-07:

- `gui/autotune_mixin.py`: removed from the active GUI runtime after the
  monitor/process split work was superseded by dropping GUI AutoTune controls.
- `bridge/ping360_sim.py`: Ping360 data contracts and firmware-style effective
  setting calculations moved to `bridge/ping360_types.py` and
  `bridge/ping360_settings.py`; raycast beam generation, reflectivity lookup,
  profile return accumulation, blind-zone handling, and noise synthesis later
  moved to `bridge/ping360_beam_model.py` and `bridge/ping360_profile.py`.
  `Ping360Simulator` now owns scan scheduling and image/range state only.
- `gui/node_telemetry_callbacks.py`: vehicle, motion/depth/pressure, RC,
  SITL/real-start status, and Ping360 status callbacks moved to focused
  modules.  The old mixed callback module is now an import-only facade.
- `gui/node_state_runtime.py`: backend graph probing, snapshot/event handling,
  and READY/WAIT calculation moved to focused modules.  The old state runtime
  module is now an import-only facade.
- `gui/node_backend_runtime.py`: graph-count probing, backend label/layout, and
  scoring/selection policy moved to `gui/node_backend_counts.py`,
  `gui/node_backend_layout.py`, and `gui/node_backend_selection.py`.
- `sim/runtime/initial_state.py`: initial Bar30 depth parsing, drop-start
  defaults, base pose/depth application, and hold-pose capture moved to
  `sim/runtime/initial_state_depths.py`, `sim/runtime/initial_state_pose_*`,
  and `sim/runtime/initial_state_hold_capture.py`.
- `bridge/ros2_bridge_config_pressure.py`: Bar30/AP_Baro, IMU accel/static
  pressure, and vertical-feedback setup moved to separate
  `bridge/ros2_bridge_config_baro.py`, `bridge/ros2_bridge_config_imu.py`, and
  `bridge/ros2_bridge_config_vertical.py`; source audit paths now follow the
  split files.
- `sim/runtime/thruster_actuator_runtime.py`: immersion scaling, per-thruster
  force loop, visual propeller updates, and direct command target conversion
  moved to focused `sim/runtime/thruster_actuator_*` helpers while the public
  class methods remain stable.
- `bridge/sitl_json_sensor_runtime.py`: JSON payload construction, UDP packet
  send, immediate replay replies, and send policy moved to focused modules.
  The source-audit Bar30 JSON `position.z` evidence now lives in
  `bridge/sitl_json_payload.py`.
- `sim/physics/fossen_residual.py`: coefficient key/type contracts, runtime
  builders, and residual wrench evaluation moved to focused modules.  The old
  module is now an import-only facade.
- `bridge/sitl_replay.py`: replay frame/event types, common numeric helpers,
  CSV loaders, and interpolation moved to focused modules.  The old replay
  module is now an import-only facade.
- `bridge/sitl_mavlink_runtime.py`: MAVLink connection/heartbeat, telemetry
  observer wrappers, and servo/command polling loops moved to focused modules.
  The old runtime module is now a compatibility export surface.
- `gui/ros_tools.py`: ROS environment/shell helpers and RViz config generation
  moved to focused modules.  The old module is now a compatibility export
  surface.
- `tools/check_dev_os_compat.py`: CLI wiring now delegates to
  `dev_os_compat_common.py`, `dev_os_compat_runtime.py`, and
  `dev_os_compat_system.py`.
- `gui/node.py`: backend detection, command readiness, event insertion, and
  snapshot age calculation moved to `gui/node_state_runtime.py`.
- `bridge/ros2_bridge.py`: ROS spin/publish utility methods moved to
  `bridge/ros2_bridge_runtime_methods.py`; Ping360 config callback moved to
  `bridge/ros2_ping360_config.py`.
- `gui/layout_mixin.py`: UI shell, telemetry, control core, replay, tuning, and
  pilot sections moved to focused `gui/layout_*` modules; the mixin is now a
  small compatibility facade.
- `gui/node_commanding.py`: initial-depth/common command gates, arm/mode
  policies, and RC/manual/Ping360 publishers moved to focused command modules;
  the public method names consumed by `gui/node.py` are preserved.
- `tools/audit_code_contract_checks.py`: path registry, source identity,
  firmware checks, active runtime checks, and thruster/gate checks moved to
  focused source-audit modules.
- `sim/runtime/cli.py`: profile, ROS2, Ping360/video, SITL, initial-state, and
  viewer options moved to `sim/runtime/cli_*` modules.
- `tools/althold_diagnostics_logger.py`: diagnostic contract math, ROS node
  capture, CSV/plot generation, and terminal summary moved to focused modules.
- `bridge/ros2_bridge_config.py`: MAVROS, pressure/vertical/IMU, and frame
  transform configuration moved to focused config modules.
- `gui/sim_stack_process_mixin.py`: active simulator-stack launch, reset, and
  status/control refresh logic moved to `gui/sim_stack_*_mixin.py` modules; the
  public mixin is now a small compatibility facade.
- `gui/ros_process_mixin.py`: ROS2 panel status, logged process launch/watch,
  MAVROS package build/start, and RViz start/stop moved to
  `gui/ros_panel_mixin.py`, `gui/ros_logged_process_mixin.py`,
  `gui/ros_package_mixin.py`, and `gui/rviz_process_mixin.py`.  The public
  mixin is now a small compatibility facade.
- `gui/control_display_mixin.py`: pilot RC/manual control, panel toggles,
  canvas drawing, event/RC feedback updates, and the UI refresh loop moved to
  focused `gui/control_*_mixin.py` modules.
- `physics/sim_profile_hydrodynamics.py`: profile field parsing moved to
  `physics/sim_profile_parsing.py` and ellipsoid baseline estimation moved to
  `physics/sim_profile_ellipsoid.py`; the public builder remains stable.
- `bridge/sitl_external_nav_runtime.py`: ExternalNav bootstrap, native VPD
  replay, synthetic VPD generation, and cache/contract checks moved to focused
  `bridge/sitl_external_nav_*` and `bridge/sitl_native_vpd_runtime.py` modules.
- `sim/runtime/physics_runtime_setup.py`: runtime record types, hydrostatic
  context setup, geometry overrides, and factory assembly moved to focused
  `sim/runtime/physics_runtime_*` modules; the public setup import path remains
  stable.
- `tools/analyze_althold_contract.py`: ALT_HOLD CSV/domain records, binning,
  segment metrics, and plotting moved to `tools/althold_contract_*` modules;
  the CLI stays as the active entry point.
- `bridge/sitl_transport.py`: model-state helpers, MAVLink status/runtime
  properties, servo handler binding, and connection lifecycle moved to focused
  `bridge/sitl_transport_*` modules while the `SitlTransport` public method
  surface is preserved.
- `gui/autotune_monitor.py`: auto-tune monitor window lifecycle, log parsing,
  candidate/progress table, and score chart rendering moved to focused
  `gui/autotune_monitor_*` modules.  The facade no longer imports ROS runtime
  just to draw the chart.
- `tools/physics_contract_model.py`: MuJoCo geometry lookup, runtime body
  mass/CoM/inertia contract, buoyancy force balance, and neutral open-plant
  simulation moved to focused `tools/physics_contract_*` modules while the
  public audit imports remain stable.
- `gui/sim_stack_env.py`: GUI-started simulator env contracts, flag parsing,
  initial-depth args, and extra-argument normalization moved to focused
  `gui/sim_stack_env_*` modules while preserving the launch mixin import path.
- `sim/runtime/underwater_wrench_runtime.py`: hydrostatic buoyancy/restoring
  torque, relative-acceleration state, custom hydrodynamics, residual Fossen,
  CFD dynamic force, and extra heave/lift/pitch damping moved to focused
  `sim/runtime/underwater_*` helpers while preserving the `apply(dt)` API.
- `tools/real_start_state.py`: CSV row selection, quaternion/frame conversion,
  row extractors, Bar30/AP_Baro datum math, state assembly, and shell output
  formatting moved to focused `tools/real_start_*` modules while preserving the
  JSON and shell CLI outputs.
- `bridge/sitl_transport_config.py`: JSON servo socket setup, SITL RC/control
  run-mode state, MAVLink command/telemetry/polling state, and ExternalNav
  scheduler/runtime state moved to focused `bridge/sitl_transport_*_config.py`
  modules while preserving the `bridge.sitl_transport_config` import surface.
- `gui/physics_mixin.py`: physics tuning window construction, profile
  load/apply/parsing, and post-apply restart orchestration moved to focused
  `gui/physics_*` modules while preserving `PhysicsMixin` method names used by
  `gui/app.py`.
- `tools/control_loop_golden_compare.py`: numeric helpers, thruster phase
  summaries, fingerprint construction, and comparison policy moved to focused
  `tools/control_loop_golden_*` modules while preserving the CLI and legacy
  `build_fingerprint`/`compare_fingerprints` import names.  The active facade is
  now `103 LOC` and no longer appears in the top 25 hotspot inventory.
- `sim/physics/model_setup.py`: fluid option scaling, pool geometry overrides,
  MuJoCo fluid geom/coef scaling, and body-tree mass helpers moved to focused
  `sim/physics/*_runtime.py` and `sim/physics/body_tree.py` modules.  The public
  `sim.physics.model_setup` import path is now a `16 LOC` facade and no longer
  appears in the top 25 hotspot inventory.
- `gui/helpers.py`: math/formatting, RC contract/message helpers, ROS2 bag RC
  replay loading, and backend-name normalization moved to focused
  `gui/*_helpers.py` and `gui/rc_replay_loader.py` modules.  The public
  `gui.helpers` import surface remains a legacy facade and still exposes config
  names for existing star imports.
- `tools/axis_rc_node.py`: RC override/manual-control message construction,
  telemetry sample construction, and service wait loops moved to focused
  `tools/axis_rc_messages.py`, `tools/axis_rc_sampling.py`, and
  `tools/axis_rc_services.py` modules.  The node dropped from `330 LOC / 63`
  branches to `194 LOC / 13` branches and no longer appears in the top 30
  hotspot inventory.
- `tools/audit_closed_loop_contract.py`: watched parameter parsing, SITL log
  overlay parsing, active runtime/profile/thruster curve inspection, and payload
  assembly moved to focused `tools/audit_closed_loop_*` modules.  The CLI facade
  dropped from `337 LOC / 48` branches to `76 LOC / 2` branches, and the parsed
  JSON output is unchanged by the split.
- `sim/runtime/hydrodynamics_runtime_setup.py`: hydrodynamic scalar/array
  extraction, MuJoCo ambient current setup, CFD/Fossen residual construction,
  diagnostic logging, and typed containers moved to focused
  `sim/runtime/hydrodynamics_runtime_*` modules.  The active setup facade
  dropped from `335 LOC / 13` branches to `100 LOC / 0` branches and no longer
  appears in the top 35 hotspot inventory.
- `run_uuv_mujoco.py`: runtime-mode/profile selection, model/initial-state
  setup, fluid/model-IO/physics setup, step-runtime construction, and final loop
  invocation moved to focused `sim/runtime/runner_*_setup.py` modules.  The
  active runner facade dropped from `335 LOC / 7` branches to `113 LOC / 2`
  branches and no longer appears in the top 40 hotspot inventory.
- `bridge/ros2_state_estimation.py`: vertical/Bar30 feedback, MuJoCo world
  kinematics, IMU/DVL body-frame conversion, and MAVROS setpoint command logic
  moved to focused `bridge/ros2_state_*` modules.  The compatibility facade is
  now `38 LOC / 0` branches and no longer appears in the top 35 hotspot
  inventory.
- `tools/actuator_wrench_audit.py`: model/site adjustment, direct-gain loading,
  per-axis wrench calculation, and Markdown report writing moved to focused
  `tools/actuator_wrench_*` modules.  The CLI facade is now `54 LOC / 3`
  branches and no longer appears in the top 35 hotspot inventory.
- `sim/runtime/real_start.py`: real-start env types, target loading,
  measurement extraction, Bar30 pressure calibration, status decision, payload
  assembly, and status publishing moved to focused `sim/runtime/real_start_*`
  modules.  The public import path is now a `17 LOC / 0` branch facade and no
  longer appears in the top 30 hotspot inventory.
- `sim/physics/thruster_params.py`: default profile generation, JSON loading,
  per-thruster reset/application, optional logging, and direct-gain overrides
  moved to focused `sim/physics/thruster_param_*` and
  `sim/physics/thruster_direct_overrides.py` modules.  The public import path is
  now a `21 LOC / 0` branch facade and no longer appears in the top 30 hotspot
  inventory.
- `gui/replay_mixin.py`: thread-safe replay status, timeline/seek/rate handling,
  load/start/pause/stop actions, and playback worker loop moved to focused
  `gui/replay_*` modules.  The public mixin is now a `60 LOC / 0` branch facade,
  every historical replay method remains bound, and the old `322 LOC / 52`
  branch hotspot no longer appears in the top 35 inventory.
- `gui/app.py`: Tk variable initialization, runtime process/thread state,
  widget-reference defaults, executor spin, initial window raise, update
  scheduling, shutdown, and mainloop logging moved to `gui/app_state.py` and
  `gui/app_lifecycle.py`.  The active app entry dropped from `303 LOC / 22`
  branches to `94 LOC / 1` branch and no longer appears in the top 35 inventory.
- `gui/node.py`: publisher/subscriber/client/state initialization and
  real-start defaults moved to `gui/node_init.py`, and vehicle-info service
  request/response handling moved to `gui/node_vehicle_info.py`.  The active
  node facade dropped from `341 LOC / 19` branches to `104 LOC / 1` branch and
  no longer appears in the top 40 inventory.
- `bridge/ros2_bridge.py`: base runtime-state setup, MuJoCo sensor/Ping360
  lookup, and compatibility method bindings moved to
  `bridge/ros2_bridge_runtime_setup.py`,
  `bridge/ros2_bridge_sensor_setup.py`, and
  `bridge/ros2_bridge_method_bindings.py`.  The active bridge facade dropped
  from `330 LOC / 3` branches to `153 LOC / 1` branch and no longer appears in
  the top 40 inventory.
- `sim/physics/dynamic_fluidcoef_runtime.py`: environment/profile parsing and
  transient runtime state initialization moved to
  `sim/physics/dynamic_fluidcoef_runtime_config.py`.  The runtime updater
  dropped from `297 LOC / 15` branches to `139 LOC / 13` branches and no
  longer appears in the top 40 inventory.
- `bridge/sitl_rc_manual_runtime.py`: RC override forwarding/neutral keepalive,
  MANUAL_CONTROL priming/send, and GUIDED/raw local setpoint commands moved to
  `bridge/sitl_rc_override_runtime.py`,
  `bridge/sitl_manual_control_runtime.py`, and
  `bridge/sitl_guided_setpoint_runtime.py`.  The old mixed command-path module
  is now a `24 LOC / 0` branch compatibility facade and no longer appears in
  the top 40 inventory.
- `physics/sim_profile_parsing.py`: common numeric parsing,
  body/buoyancy-point parsing, and hydrostatic restoring parsing moved to
  focused `physics/sim_profile_*` modules.  The old `229 LOC / 49` branch
  profile parser is now a compatibility facade and no longer appears in the
  top 30 inventory.
- `gui/sim_stack_launch_mixin.py`: GUI-owned log tailing, status-prefix
  parsing, and process-finish UI updates moved to
  `gui/sim_stack_log_watcher.py`.  The old `238 LOC / 40` branch launch hotspot
  no longer appears in the top 30 inventory.
- `bridge/sitl_command_targets.py`: command-link selection, vehicle
  HEARTBEAT/COMMAND_ACK state tracking, and UDP peer discovery moved to focused
  MAVLink command target modules.  The old `223 LOC / 72` branch command-target
  hotspot no longer appears in the top 30 inventory.
- `bridge/ros2_mavros_command_services.py`: setpoint/yaw command handling,
  command-override payload parsing, arm/mode service forwarding, and internal
  SITL command override handling moved to focused command service modules.  The
  old `210 LOC / 55` branch hotspot no longer appears in the top 35 inventory.
- `bridge/sitl_mavlink_polling.py`: servo telemetry polling, command-link
  polling, and `SERVO_OUTPUT_RAW`/HEARTBEAT handler logic moved to focused
  polling/handler modules.  The old `204 LOC / 44` branch hotspot no longer
  appears in the top 35 inventory.
- `bridge/sitl_arm_mode_runtime.py`: arm/mode MAVLink send helpers, queue entry
  points, and pending retry service loops moved to focused arm-mode modules.
  The old `197 LOC / 41` branch hotspot no longer appears in the top 35
  inventory.
- `bridge/ros2_bridge_runtime_methods.py`: ROS env/rate helpers,
  safe-publish/executor spin helpers, MAVROS state building, and static context
  helpers moved to focused `bridge/ros2_runtime_*` modules.  The old
  `190 LOC / 37` branch hotspot no longer appears in the top 35 inventory.
- `bridge/ros2_sitl_sensor_feed.py`: live MuJoCo sensor snapshot generation is
  now split into base kinematics, IMU/DVL vector conversion, Bar30/vertical
  estimate construction, transport handoff, and typed snapshot records.  The
  old `195 LOC / 26` branch hotspot no longer appears in the top 35 inventory.
- `bridge/sitl_sensor_replay_runtime.py`: controller-parity sensor replay is
  now split into clock/timestamp policy, frame-selection/pre-roll policy,
  status reporting, and replay state triggers.  The old `195 LOC / 27` branch
  hotspot no longer appears in the top 35 inventory.
- `bridge/ros2_bridge_init.py`: ROS2 startup is now split into lazy import
  loading, message/service attribute binding, context/node/executor setup,
  endpoint creation, static TF/robot_description setup, and optional-feature
  startup warnings.  The old `194 LOC / 10` hotspot no longer appears in the
  top 35 inventory.
- `bridge/sitl_external_nav_synthetic_vpd.py`: synthetic
  `VISION_POSITION_DELTA` generation is now split into VPD clock/history,
  pose/delta math, TX-rate contract checks, and send orchestration.  The body
  delta convention remains the ArduSub `BODY_FRD` current-frame contract.  The
  old `189 LOC / 33` branch hotspot no longer appears in the top 30 inventory.
- `sim/physics/dynamic_fluidcoef_setup.py`: dynamic MuJoCo fluid coefficient
  setup is now split into profile parsing, array allocation, per-pattern geom
  application, and setup logging.  The reference/current ratio and load/axis
  weight equations are unchanged.  The old `187 LOC / 19` branch hotspot no
  longer appears in the top 30 inventory.
- `sim/runtime/viewer_scene.py`: viewer debug drawing is now split into
  primitive user-scene geometry, sensor markers, and thruster/net/buoyancy
  debug overlays.  The old `186 LOC / 15` branch hotspot no longer appears in
  the top 30 inventory.
- `gui/node_commanding_common.py`: GUI command helpers are now split into
  trigger services, initial-depth release, one-shot scheduling, arm/mode state
  gates, retry scheduling, and command-override publishing.  The old
  `185 LOC / 35` branch hotspot no longer appears in the top 30 inventory.
- `gui/runtime.py`: GUI import/runtime ownership is now split into Python
  import-path sanitation, ROS2 core imports, and MAVROS message/service
  fallbacks.  The public module remains a compatibility facade, and the
  ROS2/MAVROS import surface was verified with the active ROS2 Python
  environment.
- `bridge/sitl_replay_loaders.py`: sensor-frame and native-VPD row parsing is
  now owned by `bridge/sitl_replay_row_parsers.py`, leaving CSV IO, sorting,
  and logging in the loader facade.  Pressure/depth conversion and replay-time
  semantics are unchanged.
- `bridge/qgc_video_stream.py`: ffmpeg discovery, RTP command construction, and
  subprocess start behavior are now owned by `bridge/qgc_video_ffmpeg.py`.
  `bridge/qgc_video_stream.py` keeps QGC streamer state and the public API.
- `sim/physics/dynamic_fluidcoef_runtime_config.py`: dynamic fluid-coefficient
  runtime configuration is now split into update/transient knob setup and
  runtime state/buffer setup modules.  The smoothing and coefficient equations
  are unchanged.
- GUI AutoTune process helpers are no longer part of the active GUI runtime.
  Historical logs still describe the earlier split, but current GUI code does
  not expose AutoTune launch/process controls.
- `bridge/ros2_publish_builder_mavros.py`: MAVROS-compatible lazy message
  builders are now owned by `bridge/ros2_publish_mavros_cache.py`.  The builder
  facade still exposes the same 13 publish surfaces, including IMU, raw IMU,
  static pressure, local pose, local odom, and vision pose.
- `gui/app_state.py` and `gui/app_state_vars.py`: GUI root/runtime state and
  Tk variable initialization are now separate.  Tk variable defaults live in
  `gui/app_state_core_vars.py` and `gui/app_state_feature_vars.py`, while
  `gui/app_state_tk_vars.py` owns small Boolean/Double/StringVar helpers.
  `gui/app_state_vars.py` no longer imports `.runtime`, and the app-state
  modules no longer appear in the top 25 hotspot inventory.
- `tools/runtime_freshness_eval.py`: runtime freshness checks are now separated
  into issue helpers, runtime alias/runner checks, git source checks, dirty
  worktree checks, and runtime-version checks. The public evaluator preserves
  existing report fields and issue ids, and the former `160 LOC / 25` branch
  hotspot no longer appears in the top 30 hotspot inventory.
- `gui/physics_window.py`: physics tuning window shell creation and parameter
  row rendering are now split into `gui/physics_window_shell.py` and
  `gui/physics_window_rows.py`. The `PhysicsMixin` import surface is unchanged,
  and the former `157 LOC / 7` branch hotspot no longer appears in the top 30
  hotspot inventory.
- `physics/ellipsoid_hydrodynamics.py`: equivalent-ellipsoid geometry, estimate
  type, and coefficient assembly are now split into
  `physics/ellipsoid_geometry.py`, `physics/ellipsoid_hydro_types.py`, and
  `physics/ellipsoid_hydro_coefficients.py`. The public helper import surface
  is unchanged, geometry/estimate smoke tests pass, and the former
  `153 LOC / 7` branch hotspot is now a compatibility facade.
- `tools/physics_contract_neutral_sim.py`: neutral open-plant hydrostatic force
  application and CSV/summary output are now split into
  `tools/physics_contract_neutral_buoyancy.py` and
  `tools/physics_contract_neutral_output.py`. Static `--simulate-s 0` and
  nonzero `--simulate-s 0.05` physics audits pass, and the former
  `152 LOC / 10` branch hotspot no longer appears in the top 35 hotspot
  inventory.
- `tools/physics_contract_neutral_buoyancy.py`: the neutral buoyancy helper is
  now only a compatibility facade. Context construction, component buoyancy,
  hydrostatic restoring torque, and MuJoCo force application live in focused
  `tools/physics_contract_neutral_*` modules. The split preserves the audit
  surface and keeps the static force balance at `net_down=+0.000N` with
  near-zero neutral open-plant drift.
- `tools/axis_rc_health_metrics.py`: RC override health evaluation is now split
  into per-phase sample/armed/primary-axis checks, sign-pair checks, and
  overall status aggregation. `build_health()` remains the public API, a
  synthetic sign-pair check still reports `warn`, and the former `106 LOC / 22`
  branch hotspot no longer appears in the top 25 hotspot inventory.
- `tools/axis_rc_plotting.py`: axis-check output generation now delegates CSV/
  JSON writing, value-series construction, and PNG rendering to focused helper
  modules. `write_outputs()`, `plot_timeseries()`, and `values()` remain
  exported through the existing module. A synthetic output run writes both CSVs,
  `axis_summary.json`, and a non-empty `axis_response.png`, and the former
  `120 LOC / 15` branch hotspot no longer appears in the top 25 inventory.
- `tools/filter_ping360_stl_components.py`: connected-component labeling is now
  split into `filter_ping360_union_find.py`,
  `filter_ping360_component_stats.py`, and
  `filter_ping360_component_classify.py`. The compatibility module still
  exports `UnionFind`, `component_stats()`, `classify_components()`, and
  `is_cable_like()`. A synthetic binary-STL run keeps the high-z body component
  and drops the long thin cable component, and the former `115 LOC / 15` branch
  hotspot no longer appears in the top 20 inventory.
- `tools/dev_os_compat_viewer.py`: viewer compatibility checks now delegate
  macOS `mjpython` candidate probing to `tools/dev_os_compat_mjpython.py` and
  DISPLAY/Wayland/window-server checks to `tools/dev_os_compat_display.py`.
  The public `check_display()` and `check_mjpython()` exports remain unchanged.
  A `check_dev_os_compat.py --headless --json` smoke reports `fail=0`,
  `pass=16`, and `warn=2` for the current host environment, and the former
  `106 LOC / 16` branch hotspot no longer appears in the top 20 inventory.
- `tools/physics_contract_report.py`: static audit output is now split into
  file IO, console orchestration, body-contract output, hydrostatic/start-depth
  output, and balance/neutral-simulation output. The public
  `print_physics_contract_report()` and `write_static_force_balance_outputs()`
  exports remain unchanged, the 0.05s physics audit keeps the same console
  report shape and zero force-balance result, and the former `112 LOC / 6`
  branch hotspot no longer appears in the top 20 inventory.
- `sim/runtime/underwater_hydrostatic_weighted.py`: weighted runtime buoyancy
  sample conversion and force accumulation are now split into
  `sim/runtime/underwater_hydrostatic_samples.py` and
  `sim/runtime/underwater_hydrostatic_accumulator.py`. The public point/body
  component wrench functions and `weighted_hydrostatic_result` export are
  preserved, and the former `152 LOC / 7` branch hotspot no longer appears in
  the top 35 hotspot inventory.
- `sim/physics/thruster_performance_loader.py`: JSON payload IO and curve
  parsing/nearest-voltage selection are now split into
  `sim/physics/thruster_performance_payload.py` and
  `sim/physics/thruster_performance_curves.py`. Synthetic and real
  `thruster_performance.json` smokes pass, and the former `132 LOC / 16` branch
  hotspot no longer appears in the top 40 hotspot inventory.
- Active runtime freshness was rechecked after this pass.  `uuv_mujoco/current`
  still resolves to the compatibility backing directory `v2.2`, but source
  freshness is determined by `tools/check_runtime_freshness.py`, not by the
  directory name.  The latest gate reported `HEAD` equal to `origin/uuv_sim` at
  `e60cd9383c214a7d6e4700cdeb5c2f2a8cf6d328`, with the live runtime reported
  as `current-dirty` because the active runtime contains local uncommitted
  simulator changes.
- `tools/roll_stability_runner.py`: candidate execution is now split into
  launcher lifecycle, ROS2 probe execution/validity gates, and summary output.
  The public runner still exports `run_candidate`, `stop_stack`,
  `wait_for_launcher_ready`, and `write_summary`, but no longer appears in the
  top 30 hotspot inventory.
- `bridge/ros2_bridge_public_api.py`: public bridge methods are now split into
  servo/status helpers, spin/publish helpers, and shutdown ownership.  The
  `Ros2Bridge` method-binding surface was verified unchanged and the old public
  API module no longer appears in the top 30 hotspot inventory.
- `gui/node_arm_mode_commands.py`: GUI arm/disarm and mode retry/service paths
  are now separated into `gui/node_arm_commands.py` and
  `gui/node_mode_commands.py`.  The `UuvGuiNode` command method surface was
  verified unchanged and the old arm/mode file no longer appears in the top 30
  hotspot inventory.
- `bridge/ros2_rc_output_commands.py`: RC override input, replay RCOUT plant
  injection, and SITL servo-output `RCOut` telemetry mirroring are now separate
  modules.  This keeps the input/plant/telemetry observation points explicit
  while preserving the bridge command callback names.  The old combined RC file
  no longer appears in the top 30 hotspot inventory.
- `bridge/ros2_state_vertical.py`: vertical state estimation is now split into
  initial-depth-hold feedback state, vertical truth/Bar30 pressure helpers, and
  the canonical ArduSub SITL frontend-match vertical estimate.  The Bar30
  source-contract audit now points at `bridge/ros2_state_sitl_vertical.py` for
  `sitl_depth_m_for_frontend_match(pressure_pa)`, and the old combined vertical
  module no longer appears in the top 30 hotspot inventory.
- `gui/sim_stack_launch_mixin.py`: GUI-started simulator stack ownership is now
  split into launch environment/argument construction and subprocess
  start/restart runtime.  The public mixin method surface was verified
  unchanged and the old launch mixin no longer appears in the top 30 hotspot
  inventory.
- `sim/physics/thruster_param_loader.py`: thruster JSON loading now separates
  global parameter parsing from per-thruster reset/clamp/application logic.
  The same `thruster_params.json` smoke confirmed unchanged values for
  `ver_lf`, `yaw_lf`, reverse asymmetry, and tau settings.  The old loader no
  longer appears in the top 30 hotspot inventory.
- `tools/dev_os_compat_runtime.py`: runtime Python/MuJoCo checks are now split
  from mjpython/viewer/display checks.  The Ubuntu headless compatibility gate
  still reports `fail=0 pass=16 warn=2`, and the old runtime checker no longer
  appears in the top 30 hotspot inventory.
- `tools/axis_rc_metrics.py`: axis RC metrics are now split into math helpers,
  phase summary metrics, and health/sign-pair gates while preserving report
  field names.  The old combined metrics module no longer appears in the top
  30 hotspot inventory.
- `tools/dev_os_compat_system.py`: Docker, ROS2 environment, SITL path, and
  Ubuntu migration checks are now split by contract surface while preserving
  `check_dev_os_compat.py` output.  The old combined system module no longer
  appears in the top 30 hotspot inventory.
- `tools/verify_ardusub_thruster_contract.py`: axis constants, MuJoCo scene
  wrench extraction, and response calculation are now separated from the CLI.
  The command still emits the same JSON payload and `[thruster-contract] OK`.
- `tools/audit_closed_loop_params.py`: watched parameter constants, param/log
  parsing, and real-vs-SITL comparison are now split into focused helpers while
  preserving the closed-loop audit payload shape.
- `tools/refactor_inventory.py`: inventory dataclasses, AST analysis, and
  markdown rendering are now split out of the CLI so the audit tool no longer
  ranks itself as a hotspot.
- `sim/physics/cfd_dynamic_wrench.py`: CFD runtime type, profile/env parsing,
  force-table interpolation, and body-force evaluation are now split into
  focused `cfd_dynamic_wrench_*` modules.  The public module is now a
  compatibility facade, and the old `157 LOC / 20` branch hotspot no longer
  appears in the top 40 inventory.
- `bridge/ros2_bridge_runtime.py`: optional message probing, publisher-demand
  caching, lazy publish queueing, and static context publishing now live in
  dedicated bridge modules.  The public import surface is preserved, and the
  old `156 LOC / 24` branch hotspot no longer appears in the top 40 inventory.
- `bridge/qgc_video_ffmpeg.py`: FFmpeg availability/encoder probing, QGC RTP
  command construction, and process lifecycle handling are now split.  The
  command builder keeps the prior H264 option contract while removing the old
  `160 LOC / 13` branch hotspot from the top 40 inventory.
- `sim/runtime/initial_hold.py`: initial hold pose/depth application is split
  from release-velocity reset/application.  This keeps GUI ready/start and
  real-start release behavior inspectable while removing the old
  `161 LOC / 15` branch hotspot from the top 40 inventory.
- `gui/physics_param_io.py`: current-mode status, nested JSON access,
  value parsing, and profile apply/error logging are now split into focused
  GUI physics parameter modules.  The public private-helper import surface is
  preserved for `PhysicsMixin`, and the old `152 LOC / 25` branch hotspot no
  longer appears in the top 35 inventory.
- `physics/sim_profile_hydrostatic_points.py`: hydrostatic normalization,
  buoyancy-point parsing, and body-component parsing are now separate modules.
  The profile parser public surface is preserved, and the old
  `146 LOC / 32` branch hotspot no longer appears in the top 35 inventory.
- `sim/physics/thruster_performance.py`: default config, voltage
  normalization, JSON curve loading, and runner-mode selection are now split
  into focused thruster performance modules.  The raw-RCOU plant replay direct
  mapping guard is preserved, and the old `146 LOC / 16` branch hotspot no
  longer appears in the top 35 inventory.
- `bridge/ros2_publish_mavros_cache.py`: MAVROS IMU, status, and local
  position message builders are now split into focused cache modules.  The old
  `188 LOC / 1` branch hotspot no longer appears in the top 35 inventory, and
  importing the facade no longer imports MuJoCo through `RosPublishState`.
- `physics/sim_profile_hydrodynamics.py`: damping/added-mass parsing and
  hydrostatic/body-component assembly are now split into focused helpers.  The
  old `172 LOC / 3` branch hotspot no longer appears in the top 35 inventory.
- `bridge/ros2_publish_schedule.py`: topic publish scheduling is now split by
  core, Ping360, DVL, MAVROS, and odometry surfaces while preserving the
  existing `/mavros/local_position/odom` ordering.  The old `152 LOC / 15`
  branch hotspot no longer appears in the top 35 inventory.
- `sim/physics/actuator_geometry.py`: actuator site lookup, horizontal Z
  offset, vertical X scale, and propeller joint maps are now split behind the
  original facade.  The old `152 LOC / 16` branch hotspot no longer appears in
  the top 35 inventory.
- `tools/actuator_wrench_calc.py`: per-axis wrench accumulation and
  force/torque leakage summaries now live in
  `tools/actuator_wrench_axis.py` and `tools/actuator_wrench_summary.py`.
  The audit CLI still consumes the same `build_actuator_wrench_result()`
  surface, and the former top-25 hotspot no longer appears there.
- `bridge/ros2_bridge.py`: the long ROS2/MAVROS topic surface note is now a
  contract document at `docs/contracts/ROS2_BRIDGE_SURFACE.md`.  The runtime
  class file is no longer a topic-list document and no longer appears in the
  top-25 inventory.
- `gui/rviz_config_tools.py`: ROS2 RViz class replacement and Ping360 RViz
  template ownership are now split into `gui/rviz_config_ros2.py` and
  `gui/rviz_config_ping360.py`; the public file-writing facade no longer
  appears in the top-25 inventory.
- `tools/analyze_althold_contract.py`: DataFlash signal assembly and JSON /
  console summary output now live in `tools/althold_contract_signals.py` and
  `tools/althold_contract_summary.py`; the CLI no longer appears in the top-25
  inventory.
- `sim/runtime/physics_step_callbacks.py`: thruster debug payload forwarding
  and descent-guard enforcement are now split into
  `sim/runtime/physics_step_debug.py` and
  `sim/runtime/physics_step_descent.py`.  The file remains in the top-25 by
  LOC, but its branch count is now `0`.
- `sim/physics/dynamic_fluidcoef_runtime.py`: per-step update scheduling,
  load blending, transient onset/decay, target application, and debug logging
  are now split into `sim/physics/dynamic_fluidcoef_runtime_update.py`,
  `sim/physics/dynamic_fluidcoef_runtime_transient.py`, and
  `sim/physics/dynamic_fluidcoef_runtime_logging.py`.  The runtime class no
  longer appears in the top-35 inventory.
- `sim/runtime/viewer_controls.py`: initial camera config, key dispatch, and
  camera application are now split into `sim/runtime/viewer_control_config.py`,
  `sim/runtime/viewer_control_keys.py`, and
  `sim/runtime/viewer_control_camera.py`.  The runner-facing
  `ViewerControlState` API is preserved and no longer appears in the top-35
  inventory.
- `tools/audit_closed_loop_profile.py`: thruster-performance curve selection is
  now owned by `tools/audit_closed_loop_thruster_curve.py`.  Existing imports
  of `selected_thruster_curve()` from the profile facade remain valid, and the
  profile helper no longer appears in the top-35 inventory.
- `tools/dev_os_compat_common.py`: subprocess/executable helpers and Python
  runtime probing are now split into `tools/dev_os_compat_exec.py` and
  `tools/dev_os_compat_python_probe.py`.  The common module no longer appears
  in the top-35 inventory.
- `start_docker_sitl_mujoco_mj311.sh`: GUI/Docker direct-start now runs the
  same `check_runtime_freshness.py --fetch --refresh-version --warn-only`
  preflight as the root wrappers and native start script.  This closes the
  remaining path where the GUI could start the backing `v2.2` script without
  first proving that `uuv_mujoco/current` and `origin/uuv_sim` are current.
- `tools/roll_stability_metrics.py`: quaternion/math, pose/depth/IMU metrics,
  RC component metrics, and score calculation are now split into
  `tools/roll_stability_math.py`,
  `tools/roll_stability_pose_metrics.py`,
  `tools/roll_stability_rc_metrics.py`, and
  `tools/roll_stability_score.py`.  The public `compute_metrics()` facade is
  preserved, and the former metrics hotspot no longer appears in the top-25
  inventory.
- `sim/runtime/real_start_payload_status.py`: real-start ready/mismatch
  evaluation, finite-value conversion, and status payload assembly now live in
  `sim/runtime/real_start_status_eval.py`,
  `sim/runtime/real_start_payload_values.py`, and
  `sim/runtime/real_start_payload_builders.py`.  Existing imports from the
  facade remain valid, and the former ready/status hotspot no longer appears in
  the top-35 inventory.
- `gui/ros_env_tools.py`: Python interpreter selection, ROS setup discovery,
  package probing, and ROS bash command assembly now live in
  `gui/ros_python_runtime.py`, `gui/ros_setup_paths.py`, and
  `gui/ros_bash.py`.  The `gui.ros_tools` compatibility surface is preserved,
  and the former ROS environment hotspot no longer appears in the top-45
  inventory.
- `gui/sim_stack_env_args.py`: GUI initial-depth launch defaults and simulator
  extra-argument normalization now live in
  `gui/sim_stack_initial_depth_args.py` and `gui/sim_stack_extra_args.py`.
  Bar30-first GUI start behavior, wrapper-only argument dropping, and
  headless/viewer defaults are preserved, and the former launch-arg hotspot no
  longer appears in the top-50 inventory.
- `sim/runtime/readiness.py`: readiness state dataclasses and command-readiness
  label policy now live in `sim/runtime/readiness_types.py` and
  `sim/runtime/readiness_label.py`.  Existing imports from
  `sim.runtime.readiness` remain valid, `check_runtime_readiness_policy.py`
  still passes, and the former branch-heavy READY policy file no longer appears
  in the top-65 inventory.
- `sim/runtime/initial_depth.py`: automatic Bar30 initial-depth orchestration
  now delegates geometry primitives, MuJoCo model candidates, and sim-profile
  candidates to focused modules.  Existing imports through
  `sim.runtime.initial_depth` and `sim.runtime.initial_depth_candidates`
  remain valid, and a fake-model API smoke preserved the selected auto-depth
  value used by GUI Start/ready initialization.
- `sim/transport/mavlink_command_link.py`: command endpoint disable policy and
  low-level heartbeat/RC override/arm send primitives now live in focused
  modules.  The stateful command-link owner still exposes the same public
  methods, and fake MAVLink smoke tests preserve 18-channel RC override,
  MAVLink1 8-channel fallback, heartbeat throttling, and arm force magic.
- `gui/sim_stack_launch_runtime.py`: GUI Start now delegates launch command
  construction, log opening, subprocess spawning, watcher-thread creation, and
  launch-target validation to focused helpers.  The helper smoke preserves
  MAVROS surface args, direct-MAVLink/no-rebuild defaults, initial-depth args,
  and extra-argument ordering without importing ROS2/rclpy.
- `sim/validation/plant_input_gate.py`: plant-input gate result types, CSV PWM
  activity checks, log signature scanning, and failure evaluation now live in
  focused modules.  Fixture smoke tests preserve header-only, neutral-only,
  disarmed-servo, and neutral-servo failure behavior.
- `sim/runtime/simulation_step_runtime.py`: common per-step physics sequencing
  and initial-depth hold auto-release policy now live in focused helpers.
  Fake-runtime smoke tests preserve direct command, raw-PWM SITL, paused-step,
  publish, QGC video, ALT_HOLD release, and non-neutral servo release order.
- RC/servo transport ownership is now split across explicit contract layers:
  `sim/runtime/sitl_servo_*` owns PWM-to-thruster target mapping,
  `sim/transport/json_servo_*` owns raw JSON-SITL UDP socket IO,
  `bridge/sitl_mavlink_servo_*` owns low-rate `SERVO_OUTPUT_RAW` telemetry
  handling, and `bridge/sitl_rc_override_*` owns MAVLink RC override forwarding.
  Targeted smoke tests preserve PWM conversion, UDP packet decode/send,
  heartbeat target filtering, `SERVO_OUTPUT_RAW` callback dispatch, peer-wait
  failures, and neutral keepalive holdoff behavior.

## Root coupling problems

1. Runtime contracts are spread across launch scripts, GUI code, bridge code,
   debug harnesses, and environment variables.
2. Sensor contract code is now split across live runtime and replay policy, but
   pressure, depth, rate holding, and initial-state behavior still require
   golden-master replay evidence before they can be called low-risk.
3. Transport responsibilities are overloaded: JSON sensor packets, JSON servo
   packets, MAVLink commands, MAVLink telemetry, replay preview, arm/mode state,
   and diagnostics still converge on the same transport object even though more
   sub-policies are now isolated.
4. ROS surface responsibilities are overloaded: topic creation, message
   conversion, timing, TF, Ping360, RC override, diagnostics, and SITL callbacks
   live in the same class.
5. Physics responsibilities are overloaded: model loading, initial conditions,
   hydrostatics, thruster curves, hydrodynamics, current, GUI state, and logging
   are interleaved in `run_urdf_full.py`.
6. Research outputs and legacy debug runs are mixed close to runtime code, which
   makes it hard to know whether a file is evidence, an experiment, or active
   execution code.

## Refactor strategy

Use a strangler refactor:

1. Freeze contracts in `docs/contracts` and expose them in `sim/contracts`.
2. Add inventory tools that make responsibility and complexity visible.
3. Move logic only behind compatibility wrappers until golden-master checks pass.
4. Split by data boundary, not by convenience:
   - `sim/contracts`: constants, coordinate frames, pressure law, RC mapping,
     telemetry observation points.
   - `sim/transport`: JSON/MAVLink wire protocols and connection state.
   - `sim/ros_surface`: ROS topics, MAVROS-compatible message surfaces, TF.
   - `sim/runtime`: process orchestration, readiness, reset, arm/mode workflow.
   - `sim/physics`: MuJoCo plant forces, hydrostatics, actuator model,
     hydrodynamics.
   - `sim/validation`: parity metrics, overlays, golden-master gates.
   - `han`: research and coefficient estimation pipeline only; no direct runtime
     mutation.
   - `experiments`: repeatable baselines, runs, legacy snapshots, paper evidence.

Latest 2026-06-07 focused splits:

- Ping360 simulator lifecycle ownership is now split into MuJoCo model-id
  lookup, runtime state creation/refresh, sweep/history buffers, and
  scan/update-cycle bookkeeping.  `bridge/ping360_sim.py` remains the public
  simulator class and passes a lifecycle smoke for disabled update, missing-site
  config refresh, and status payload safety.
- SITL replay row parsing now uses focused sensor-row, native-VPD-row, and
  row-vector helper modules while `bridge/sitl_replay_row_parsers.py` remains a
  compatibility export surface.
- GUIDED setpoint handling is split between BODY_NED velocity conversion and raw
  LOCAL_NED setpoint forwarding.  The smoke verifies the left/up/yaw-rate sign
  contract and unchanged raw local-NED payload forwarding.
- MAVROS publish cache construction is now registry-driven.  The cache no
  longer owns one public method per MAVROS message; lazy object reuse and topic
  builder order are covered by a focused smoke.
- Hydrodynamics and thruster runtime diagnostic logging are split by log
  section.  `hydrodynamics_runtime_logging.py` and
  `thruster_param_runtime_summary.py` now orchestrate the same setup summaries
  while focused section modules own the string-level detail.
- Runner startup and control bridge setup are split by ownership.  Initial
  depth runtime construction, thruster immersion startup env parsing, command
  timeout/direct-command policy, initial-depth release service wiring, and
  real-start status construction now live in focused modules.  The runner-facing
  `load_runner_initial_setup()` and `create_runtime_control_bridge_setup()`
  factories are preserved.
- CFD dynamic-wrench axis table parsing and hydrostatic runtime reporting are
  split into focused modules for force-table validation, CoB site alignment,
  CoB override logging, and hydrostatic application/restoring log sections.
- `InitialDepthHoldState` now keeps state fields and factory behavior while
  dict-style compatibility and pose/release actions live in mixins.  This
  preserves the item-access contract used by runtime release/hold services.
- `tools/preflight_ardupilot_integrity.py` now owns CLI orchestration only.
  Read-only git status collection, payload classification, and JSON/output
  exit-code handling live in focused helpers.  The watched-file policy for
  `ArduSub/control_althold.cpp` and host-build dependency drift is unchanged.
- The latest top hotspot list has moved away from the refactored bridge
  facades.  The largest current files are dominated by profile defaults/spec
  tables and runtime setup/reporting helpers, with the next behavior-sensitive
  cleanup targets in `sim/runtime`, `sim/physics`, selected `tools`, and GUI
  process mixins.

## Immediate risk ranking

P0, highest risk:

- Runtime identity confusion: `uuv_mujoco/current` is the active runtime, while
  `uuv_mujoco/v2.2` is only the compatibility backing directory name.  Reports
  and commands that call this "latest v2.2" are wrong; use
  `current-dirty` plus `uuv_mujoco/RUNTIME_VERSION.json` evidence.
- Hidden arm-state failure: ACKs can continue while plant-side RCOU rows are
  empty or neutral.
- Observation-point mismatch between low-rate `SERVO_OUTPUT_RAW` telemetry and
  high-rate JSON servo output.
- Sensor-rate and zero-order-hold mismatch.
- Bar30 pressure datum mismatch.

P1:

- Thruster effectiveness and frame sign mismatches.
- CoM/CoB/hydrostatic contract drift.
- GUI readiness saying ready before runtime command path can actually move the
  vehicle.
- Remaining branch-heavy active code after the latest pass is concentrated in
  roll-stability metrics, hydrodynamics/runner setup helpers, and selected
  bridge command/telemetry shaping paths.  Vehicle heartbeat, QGC video
  runtime, GUI simulator reset handling, SITL MAVLink request policy,
  command-link target/readiness policy, auto-ready sequence policy, PWM
  plant-input safety policy, JSON servo polling policy, and ExternalNav cache
  contract files have been removed from this hotspot group without changing
  arm/mode readiness, RC override readiness, QGC stream fallback behavior, or
  ExternalNav stale-output
  semantics.  Axis RC service and ALT_HOLD BIN extraction facades have also
  been removed from the hotspot group while preserving service retry cadence
  and DataFlash timebase normalization.  GUI RC replay timeline handling is now
  split into pure time/rate math, Tk slider event handling, and thread-safe
  seek state helpers while preserving the replay mixin method surface.  The
  telemetry panel is also split into summary/detail, vehicle visuals/RC
  feedback, and event-log builders while preserving `build_telemetry_panel`.
  GUI control core is split into stack, ROS2 utility, and arm/mode builders,
  and the Ping360 window mixin is split into lifecycle and panel-construction
  helpers while preserving the existing button commands and mixin method
  surface.  Ping360 bridge contracts are now split into protocol constants,
  JSON configuration loading, effective settings, sample/status contracts, beam
  hit extraction, and profile signal shaping while preserving the
  `ping360_types` and `ping360_profile` import surfaces.  Static physics
  contract audit orchestration is split into audit context, depth candidate,
  force-balance, and report-output helpers while preserving
  `run_physics_contract_audit(args)` and the existing CLI output contract.
  SITL MAVLink stream request policy is split into target/timing,
  `SERVO_OUTPUT_RAW`, and AP sensor/attitude telemetry helpers while preserving
  the original servo-link target semantics and command-link target resolution.
  Command-link selection, command readiness, and MAVLink target resolution are
  split out of `bridge/sitl_command_links.py` while preserving the
  `SitlTransport` method-binding API.
ROS2 direct command shaping is split into direct normalized command filtering,
`/cmd_vel` guided-setpoint forwarding, and MAVROS manual-control forwarding
while preserving the `ros2_bridge_commands.py` import surface.
Auto-ready is split into ExternalNav readiness, state/log throttling, neutral
RC priming, and sequence orchestration while preserving arm/mode readiness
order and neutral RC frame construction.
SITL PWM frame handling is split into plant-replay ownership gates,
disarmed/all-min safety neutralization, neutral activity warnings, and
callback/debug output while preserving raw JSON/MAVLink/replay PWM routing.
JSON servo polling is split into endpoint packet polling, frame/client
bookkeeping, missing/stale endpoint warnings, plant-replay timeout handling,
and the top-level command/MAVLink/JSON polling loop while preserving JSON
fallback and MAVLink-active ignore behavior.
ALT_HOLD diagnostics output is split by output contract: CSV writing,
finite-series filtering, plotting, and terminal summary now live in separate
helpers while `tools/althold_diagnostics_output.py` remains a compatibility
export surface.
GUI logged ROS process ownership is split by launch/watch contract. Log-file
creation, subprocess launch, thread-slot binding, log tailing, and finish-state
reporting now live in focused helpers while `gui/ros_logged_process_mixin.py`
stays as the GUI method compatibility surface.
GUI command publisher ownership is split by command surface. RC override,
MANUAL_CONTROL, and Ping360 config publishing now live in focused modules while
`gui/node_rc_publishers.py` remains a compatibility export surface.
The one-step MuJoCo runtime now delegates direct-command and raw-PWM
SITL/plant-replay step paths to focused helpers while preserving
`SimulationStepRuntime.run_step()` and `run_raw_pwm_step()`.
Initial-depth runtime release is split by side effect: release sequencing,
release-snapshot publication, and ROS service installation now live in focused
helpers while `InitialDepthHoldRuntime.release()` remains the public runner
entry point.
Passive MAVLink telemetry dispatch now delegates target-source storage by
message family after preserving the HEARTBEAT-before-source-filter contract.
GUI simulator stack start/restart is split by launch contract: restart waiting,
start guards, process spawn, and post-start GUI-owned state recording now live
in focused helpers while `_start_sim_stack()` remains the button command.
MuJoCo fluid coefficient scaling is split into global/per-geom/extra runtime
helpers; GUI pilot control is split into command reading, publication, release,
and initial-depth release request helpers; viewer control state is split into
construction, toggle, and status helpers. Distributed body component setup is
split into composite inertia calculation, MuJoCo mutation, and logging while
preserving the same static force-balance audit result.
Hydrostatic runtime env/profile extraction is now split into CoB values,
restoring base values, and real-start trim override helpers while preserving
the existing hydrostatic runtime value record and static force-balance
contract.
Runner physics setup is split into fluid-contract setup, control-path logging,
ArduSub thruster model-IO naming setup, and a runtime physics factory adapter
while preserving `create_runner_physics_setup()` as the runner API and keeping
the plant-replay/direct-RCOU flag on the same runtime physics path.
GUI physics parameter apply is split into load/persist/error-log helpers while
keeping `physics_param_apply.py` and `physics_param_io.py` as compatibility
surfaces.  GUI RC replay is split into browse/load/playback controls plus
worker state/seek/pause/wait/publish/finish helpers, with pure time formatting
and RC padding moved away from ROS runtime imports.  Deletion candidates from
this pass are therefore gated on import migration: `run_urdf_full.py`,
`gui/helpers.py`, `gui/config.py`, `gui/uuv_control_gui.py`,
`gui/physics_param_io.py`, and `gui/replay_controls.py` are not safe to delete
until launch/debug/status references no longer depend on them.
Dynamic fluidcoef pattern setup now separates pattern matching, reference-ratio
preparation, weight parsing, array mutation, and logging.  MuJoCo base-state
handling now separates id lookup, free-joint mutation, Bar30 depth helpers, and
public mixin methods while preserving `MuJoCoBaseState`.  Static
body-contract audit now reuses the runtime body-distribution inertia
calculation, so the static audit no longer carries a second copy of the
mass/CoM/parallel-axis calculation.  A runtime-only import bug in
`sim/runtime/initial_state.py` was also fixed by importing `os` before
real-start Bar30 pressure calibration uses `os.environ`.
Initial depth application now separates Bar30/base-depth mutation, surface
hysteresis warnings, and log formatting while preserving
`apply_initial_depth_request()`.  Step physics callback wiring now separates
the callback dataclass and local body-velocity reader from the callback
builder.  Thruster runtime setup is split by owner: param/SITL-servo binding,
actuator runtime creation, debug runtime creation, and typed bundle helpers.
During that split, the actuator builder signature was corrected to match its
factory caller, avoiding the stale `thruster_param_runtime` keyword shape on
the active runtime path.
Underwater hydrodynamic residual application is now split into body-frame
residual/Fossen/CFD calculations, MuJoCo world-frame wrench application, CFD
debug logging, residual dispatch, custom-hydrodynamics dispatch, and relative
acceleration tracking.  Initial runtime state setup is also split into typed
outputs, hold/real-start policy resolution, ordered depth/pose/hold
application, and Bar30 pressure calibration while preserving
`configure_initial_runtime_state()`.
Thruster debug CSV runtime now keeps `ThrusterDebugRuntime` as the public
writer facade, but sample scheduling and row emission are separate helpers.
This keeps runtime observability testable without changing the 20 Hz sampling
contract or the pre-integration `mj_forward` force-breakdown point.
Source-contract audit construction now maps one contract surface to one helper
module for firmware JSON/PWM/Bar30/SERVO telemetry and active-runtime
pressure-output checks.  The refactor inventory ranking now uses structural
complexity score rather than LOC, and GUI process plus ARM/MODE command paths
have focused helper modules while keeping the existing GUI method names.
GUI simulator-stack status now separates external-process probing from
status/control policy.  Native VPD replay now separates start cursor alignment,
due-event sending, TX-rate monitoring, and debug logging while preserving the
native VPD input contract.  Command-readiness labels now dispatch through
preflight, command-path, and operator-control stages, and validation tooling
for roll-stability and golden-control-loop thruster summaries has focused
wait/request and metric helper modules.
GUI simulator-stack log watching now delegates log tailing and status line
classification to focused helpers, and GUI close handling delegates scheduled
update cancellation, replay stop, child-process termination, sim-stack reset,
RC release, ROS shutdown, and root destroy to shutdown helpers.  This removed
`gui/sim_stack_log_watcher.py` and `gui/app_lifecycle.py` from the top
structural-complexity hotspot list without changing GUI method names or
readiness/RC contracts.
Offline thruster performance loading now separates JSON payload IO and nearest
T200 voltage-curve selection from the `ThrusterPerformance` API. Direct
thruster gain overrides now separate profile values, environment group values,
and per-thruster environment values. The passive viewer loop now separates
cadence construction, catch-up clocks, paused-step handling, ROS publish
catch-up, and viewer-frame sleeping. Ping360 publish jobs now use per-cycle
cache state plus focused sample/image/scan/echo/status helpers, avoiding a
runtime-only `RosPublishState` import in the Ping360 publish builder. These
splits removed `physics/thruster_performance.py`,
`sim/physics/thruster_direct_overrides.py`,
`sim/runtime/simulation_loop_runtime.py`, and
`bridge/ros2_publish_builder_ping360.py` from the top hotspot list without
changing plant coefficients, RC output contracts, or Ping360 ROS payload
fields.
GUI arm/mode command-request helpers now split retry-log cadence, mode readiness
gates, topic command override, and MAVROS service calls while preserving the
public GUI command methods. GUI physics restart now separates orchestration,
reset-script execution, and GUI-thread finish scheduling; a smoke-test failure
also exposed and removed an import-time dependency on `gui.config` before the
simulator path is installed. The refactor inventory tool now separates path
filtering, structural scoring, and AST symbol counting, and supports both CLI and
package import paths. Synthetic ExternalNav VPD now separates scheduler/rewind
due checks from MAVLink `vision_position_delta_send` emission while preserving
the existing bootstrap, native-VPD replay, pose/delta, and TX-rate order. These
splits removed `gui/node_mode_request_steps.py`, `gui/physics_restart.py`,
`tools/refactor_inventory_analysis.py`, and
`bridge/sitl_external_nav_vpd_send.py` from the top structural hotspot list
without changing RC input/output, Bar30/static-pressure, SERVO_OUTPUT_RAW,
thruster, or plant-input contracts.
The next contract-surface pass matched `INS_POS1_X` in the SITL contract file to
the real robot, split SITL sensor vector assembly into DVL-altitude, IMU-vector,
and DVL-velocity helpers, split T200 curve parsing from closed-loop thruster
curve selection, and split GUI initial-depth defaults by explicit args,
real-start, Bar30-depth, and base-link debug-hold ownership.  It also added a
lazy MuJoCo loader for physics-contract tools and exposes `dynamic_fluidcoef`
in the closed-loop contract report so the active profile clearly shows whether
the runtime is using the fixed five-coefficient ellipsoid baseline or a dynamic
fluidcoef experiment.  Validation preserved GUI readiness/runtime readiness,
RC frame, ArduSub thruster, source-contract, closed-loop parameter, and static
physics force-balance gates.
GUI simulator-stack status now separates button-state updates, refresh
orchestration, and thread-safe text updates, and no longer imports the
ROS-heavy `gui.runtime` just to access Tk constants.  GUI command-readiness
calculation now separates freshness predicates, required-mode selection,
runtime-readiness construction, and command input assembly.  ARM requests now
match the mode-request structure with deadline, gate, topic override, and
MAVROS service helpers.  Initial-depth profile candidates now share one
local-top-to-Bar30-depth construction path for body components and buoyancy
points.  GUI ROS package controls are split into build and MAVROS-stack
helpers, ROS setup discovery separates candidate generation from `ros2 pkg`
probing, and real-start status evaluation now exposes individual mismatch
predicates for depth, pressure, XY, attitude, velocity, and angular velocity.
These splits removed the previous GUI status/readiness/ARM/ROS setup and
initial-depth profile files from the top structural-complexity inventory while
preserving GUI readiness, backend selection, runtime readiness, RC frame,
thruster, source-contract, and closed-loop contract gates.
Source-contract auditing now also covers the runtime surfaces that determine
whether a plant replay or closed-loop run is comparable at all: sim-time sensor
publish cadence, wall-time MAVLink/RC polling, RC override raw-frame
forwarding and `/mavros/rc/in` mirroring, plant-input ownership between JSON
SERVO and replay RCOU, thruster force conversion entry points, and opt-in
dynamic MuJoCo ellipsoid `fluidcoef` updates.  This does not tune coefficients;
it makes the time/sensor/RC/thruster/fluid-dynamics contracts fail visibly
before HAN/CFD or plant replay tuning is trusted.
Real-start status publishing, GUI initial-depth release state transitions,
GUI ARM/MODE feedback freshness gates, and descent-contract diagnostics now
delegate publish/log, pending/in-flight, fresh vehicle/Bar30/IMU, and descent
cause formatting logic to focused helpers.  The affected files dropped from the
top structural hotspot list while preserving the public command/readiness and
diagnostic contracts.
The next visual/lifecycle pass split Ping360 image rendering into lookup and
layer helpers, GUI attitude/depth drawing into widget helpers, MuJoCo viewer
scene rendering into primitive and bubble helpers, Ping360 RViz/rqt controls
into process/status helpers, ROS bridge shutdown into thread/SITL/ROS helpers,
MAVLink servo HEARTBEAT target filtering into its own module, and sim-stack
reset into worker/thread helpers.  It also removed avoidable `gui.runtime`
imports from visual/reset helpers, so these surfaces can be smoke-imported
without `rclpy`.  Contract gates for GUI readiness, backend selection, runtime
readiness, RC frames, ArduSub thrusters, source contracts, and closed-loop
params still pass.
The following contract-surface pass split IMU/static-pressure configuration by
source owner, auto-ready by wait gates versus arm/mode actions, sensor replay
interpolation by index/math/frame assembly, hydrodynamics profile parsing by
scalar/array/vector helpers, and golden thruster summaries by IO/window/column
selection.  This keeps time, sensor I/O, RC readiness, thruster validation, and
dynamic ellipsoid `fluidcoef` audit surfaces explicit while preserving
`fail=0`, `pass=15`, `warn=5` source-audit output and empty closed-loop
contract mismatches.
The next hotspot-continuation pass split thruster-debug runtime emission,
low-level MAVLink command-link state/connection/send wrappers, pool runtime
depth/XY geometry overrides, lazy ROS2 import loading, and MAVROS setpoint
position/yaw-command handling.  These splits removed
`sim/runtime/thruster_debug_runtime.py`,
`sim/transport/mavlink_command_link.py`,
`sim/physics/pool_runtime_overrides.py`, `bridge/ros2_bridge_imports.py`, and
`bridge/ros2_mavros_setpoint_services.py` from the top structural hotspot list.
The pass preserved RC override, command-link, thruster debug CSV, pool
override, ROS missing-package, MAVROS setpoint, source-contract, and
closed-loop parameter gates.
The latest contract pass split SITL vertical estimation into Bar30 pressure,
vertical-velocity fallback, and NED/ExternalNav frame helpers, then split
real-start Bar30 datum inference into row-candidate extraction and candidate
selection.  It also added a dedicated source-contract check that pins the
sensor I/O split: one MuJoCo sensor snapshot feeds ArduSub JSON SITL
timestamp/IMU/position/velocity/attitude/quaternion input, while ROS core,
MAVROS, and DVL observation topics are published from that same snapshot.  The
new check keeps time, sensor input/output, RC in/out, raw plant input,
thruster conversion, and opt-in dynamic five-coefficient MuJoCo ellipsoid
`fluidcoef` changes visible in source audit before any HAN/CFD tuning is
trusted.  Validation now reports source-contract `fail=0`, `pass=16`,
`warn=5`; closed-loop contract mismatches and missing SITL params are empty for
the current profile.
The next continuation pass removed the next six structural hotspots without
changing runtime contracts: static context publishing now delegates one-shot
`/tf_static` and low-rate `/robot_description` policy; development-OS Python
probing separates candidate discovery from subprocess import checks;
immediate sensor-replay JSON replies separate gate/timestamp/payload/log
ownership; Ping360 beam modeling separates angle conversion, beam directions,
raycast, and reflectivity; Fossen residual wrench evaluation separates velocity
terms, named damping, and quadratic damping; MAVLink message interval requests
separate throttle state from request bodies.  Added smoke checks cover each
surface, and the full validation pass still reports source-contract `fail=0`,
`pass=16`, `warn=5`, empty closed-loop param mismatches, GUI readiness/backend
PASS, runtime readiness PASS, RC frame PASS, and ArduSub thruster contract OK.
These are the next safer refactor targets before touching plant physics
coefficients.

P2:

- HAN/CFD coefficient tuning before the plant input, frame, pressure, and
  actuator contracts are locked.

## Exit criteria for this audit phase

- New folders exist and describe ownership.
- Existing pressure/rate contract is reachable through `sim/contracts`.
- A repeatable inventory tool can regenerate this audit table.
- Syntax checks pass for new Python files.
