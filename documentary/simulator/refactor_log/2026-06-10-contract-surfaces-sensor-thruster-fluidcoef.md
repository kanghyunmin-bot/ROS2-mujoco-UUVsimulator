# Contract Surface Refactor: Sensor, Thruster, Fluidcoef

Date: 2026-06-10

Scope: active runtime backing files under `uuv_mujoco/v2.2`.

## Changes

- Matched `ardusub_realrobot_contract.param` to the real robot for
  `INS_POS1_X=0.145000`.  The closed-loop contract audit now reports
  `real_vs_sitl_mismatches={}` and no missing SITL watched params.
- Added a lazy MuJoCo loader for physics-contract tools.  `physics_contract_audit.py
  --help` now works in the system Python, while real physics audits clearly
  require the MuJoCo runtime Python.
- Split SITL sensor vector assembly into DVL altitude source selection, IMU body
  vector assembly, and DVL body-velocity assembly.  This keeps sensor I/O
  contracts auditable without changing snapshot payload semantics.
- Split closed-loop thruster curve parsing from voltage-curve selection.  The
  audit still selects the 20 V T200 curve with force range
  `[-49.382633350124664, 65.922590564376]` N.
- Split GUI initial-depth launch defaults into explicit-arg, real-start,
  Bar30-depth, and base-link debug-hold contracts.
- Added `dynamic_fluidcoef` to the closed-loop contract report.  In the current
  profile it is visible but inactive, so the accepted plant baseline remains
  fixed MuJoCo five-coefficient ellipsoid fluidcoef scaling.

## Validation

- `python3 -m compileall -q sim/current/tools sim/current/bridge sim/current/gui sim/current/physics sim/current/sim`
- `axis_rc_sequence_imports=PASS`
- `ros2_shutdown_and_state_sensor_imports=PASS`
- `ros2_sitl_sensor_vectors_split=PASS`
- `thruster_curve_split=PASS`
- `initial_depth_contract_split=PASS`
- `dynamic_fluidcoef_profile_gate=PASS`
- `gui_readiness_contract=PASS`
- `runtime_readiness_policy=PASS`
- `rc_frame_contract=PASS`
- `verify_ardusub_thruster_contract.py --quiet` reports OK
- `audit_code_contract_sources.py` reports `fail=0`, `pass=11`, `warn=5`
- `audit_closed_loop_contract.py` reports no real/SITL watched-param mismatch
- `physics_contract_audit.py` with `ros2_h311` reports neutral static
  force balance at both scene-default and auto fully-wet depths.

## Contract Notes

- ArduSub 4.1.2 JSON sensor input still uses `position.z` to drive the SITL
  water barometer path; the JSON `altitude` key is compatibility/debug output,
  not the controller input contract.
- `/mavros/imu/atm_pressure` remains excluded from controller/plant fitting
  until the real bag semantics are proven.
- Dynamic fluidcoef remains a tuning/research path.  It should only be enabled
  after fixed-baseline plant replay gates show that the time, sensor, RC,
  thruster, and static plant contracts are correct.
