# Contract Scope and Hotspot Split

## Scope

This pass extended the active source-contract audit beyond file structure cleanup.
The audit matrix now explicitly covers these runtime contract lanes:

- time/phase: sim-time ROS publishing, servo-frame replay clock, wall-time MAVLink polling, and viewer catch-up cadence.
- sensor input/output: one MuJoCo sensor snapshot feeding JSON SITL, ROS core topics, MAVROS topics, and DVL topics.
- RC input/output: 18-channel RC override forwarding plus `/mavros/rc/in` mirror ownership.
- controller-output/plant-input: raw JSON SERVO or explicit replay RCOU remains the plant input before force conversion.
- thruster conversion: sim-time scheduled thruster update, first-order actuator state, T200/direct force curve, and one immersion scale.
- dynamic ellipsoid fluid: five MuJoCo `fluidcoef` coefficients are velocity-load updates, opt-in only, and disabled in the accepted clean baseline.

## Code Changes

- Split ALT_HOLD contract plotting into mode shading and panel renderers:
  - `tools/althold_contract_plot_modes.py`
  - `tools/althold_contract_plot_panels.py`
- Strengthened source-contract checks:
  - `tools/audit_code_contract_runtime_time.py`
  - `tools/audit_code_contract_runtime_dynamic_fluidcoef.py`
  - `tools/audit_code_contract_thruster_gate_checks.py`
  - `tools/audit_code_contract_matrix_domains.py`
- Split dynamic fluidcoef audit evaluation/evidence:
  - `tools/audit_code_contract_runtime_dynamic_fluidcoef_eval.py`
  - `tools/audit_code_contract_runtime_dynamic_fluidcoef_evidence.py`
- Split roll-stability MAVROS arm/mode command sending:
  - `tools/roll_stability_command_send.py`
- Moved `UuvGuiNode` compatibility method bindings into:
  - `gui/node_bindings.py`
- Split static neutral-plant contract simulation timing and loop code:
  - `tools/physics_contract_neutral_timing.py`
  - `tools/physics_contract_neutral_loop.py`
- Split ArduPilot source identity metadata/check builders:
  - `tools/audit_code_contract_ardupilot_identity_info.py`
  - `tools/audit_code_contract_ardupilot_identity_checks.py`
- Split real-start Bar30 pressure and RC frame payload construction:
  - `tools/real_start_pressure_state.py`
  - `tools/real_start_rc_state.py`
  - `tools/real_start_measurement_smoke_cases.py`

## Verification

```bash
env PYTHONPYCACHEPREFIX=/private/tmp/pycache python3 -m compileall -q \
  uuv_mujoco/v2.2/tools uuv_mujoco/v2.2/bridge \
  uuv_mujoco/v2.2/sim uuv_mujoco/v2.2/physics uuv_mujoco/v2.2/gui
```

Additional checks passed:

- `check_dynamic_fluidcoef_contract.py`
- `check_thruster_param_loader.py`
- `check_runtime_readiness_policy.py`
- `check_ros2_replay_rcout.py`
- `check_sitl_servo_runtime.py`
- `check_physics_contract_geometry.py`
- `check_filter_ping360_stl_io.py`
- `check_plant_input_gate_csv.py`
- `check_ros2_dvl_messages.py`
- `verify_ardusub_thruster_contract.py --quiet`
- `check_real_start_measurements.py`
- `audit_code_contract_sources.py`: `fail=0 pass=18 warn=6`
- `audit_closed_loop_contract.py`:
  - `real_vs_sitl_mismatches={}`
  - `missing_sitl_params=[]`
  - `thruster_voltage=20.0`
  - `dynamic_fluidcoef.active=false`

## Inventory Result

Removed these earlier top hotspots from the current top 25:

- `tools/althold_contract_plot.py`
- `tools/roll_stability_probe_commands.py`
- `gui/node.py`
- `tools/physics_contract_neutral_sim.py`
- `tools/audit_code_contract_ardupilot_identity.py`
- `tools/real_start_builder.py`
- `tools/check_real_start_measurements.py`

The next remaining hotspots are mostly smoke-test files, small bridge state
helpers, and diagnostics/plotting utilities.
