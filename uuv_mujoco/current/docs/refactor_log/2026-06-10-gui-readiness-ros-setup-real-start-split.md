# GUI Readiness, ROS Setup, Real-Start Split

Date: 2026-06-10

Scope: active runtime backing files under `uuv_mujoco/v2.2`.

## Changes

- Split GUI simulator-stack status ownership into button-state updates,
  refresh orchestration, and thread-safe status text updates.
- Removed a ROS-runtime import dependency from `sim_stack_status_mixin.py`;
  it now imports `tkinter` directly instead of pulling in `gui.runtime`.
- Split GUI command-readiness assembly into freshness predicates, required-mode
  selection, runtime-readiness construction, and final command input assembly.
- Split GUI ARM requests into deadline, gate, topic override, and MAVROS
  service helpers, matching the existing mode-request structure.
- Split sim-profile initial-depth candidates into field parsing and local-top
  candidate construction.  Body-component and buoyancy-point candidates now use
  the same Bar30-depth calculation path.
- Split GUI ROS package controls into build helpers and MAVROS package stack
  helpers, removing another unnecessary `gui.runtime` import.
- Split ROS setup path discovery into candidate generation and `ros2 pkg
  prefix` probing.  This keeps Ubuntu `/opt/ros/<distro>` discovery separate
  from macOS conda fallbacks.
- Split real-start status evaluation into explicit mismatch predicates for
  contract, depth, pressure, XY, attitude, velocity, and angular velocity.

## Validation

- `python3 -m compileall -q uuv_mujoco/current/gui uuv_mujoco/current/sim uuv_mujoco/current/tools uuv_mujoco/current/bridge`
- `sim_stack_status_split=PASS`
- `node_readiness_split=PASS`
- `initial_depth_profile_candidates_split=PASS`
- `node_arm_request_split=PASS`
- `ros_package_split=PASS`
- `ros_setup_paths_split=PASS`
- `real_start_status_checks_split=PASS`
- `gui_readiness_contract=PASS`
- `gui_backend_selection=PASS`
- `runtime_readiness_policy=PASS`
- `rc_frame_contract=PASS`
- `verify_ardusub_thruster_contract.py --quiet` reports OK
- `audit_code_contract_sources.py` reports `fail=0`, `pass=11`, `warn=5`
- `audit_closed_loop_contract.py` reports no watched real/SITL parameter
  mismatch.

## Current Inventory Delta

After this pass, the former top GUI readiness/initial-depth/ARM/ROS setup
hotspots dropped out of the top inventory list.  Remaining top runtime-contract
hotspots are now led by Ping360 rendering, real-start publishing, initial-depth
commands, and arm/mode readiness gates.
