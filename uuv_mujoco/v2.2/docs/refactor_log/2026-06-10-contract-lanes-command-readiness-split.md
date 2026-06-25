# Contract Lanes And Command Readiness Split

Date: 2026-06-10

## Scope

- Kept all edits inside `uuv_mujoco/v2.2`.
- Did not modify ArduPilot source or submodule pointer.
- Preserved the controller-parity boundary: real `/mavros/rc/out` versus SITL
  MAVLink `SERVO_OUTPUT_RAW`; plant input remains raw JSON SERVO or explicit
  replay RCOU/PWM before thruster conversion.

## Changes

- Split command override payload parsing into:
  - `bridge/ros2_command_bool.py`
  - `bridge/ros2_command_payload_tokens.py`
  - compatibility facade `bridge/ros2_command_payload.py`
- Added `tools/check_ros2_command_payload.py` to lock existing JSON and token
  parsing behavior.
- Split SITL command-link activity predicates into
  `bridge/sitl_command_link_activity.py`, leaving
  `bridge/sitl_command_link_readiness.py` focused on `mavlink_connected` and
  `rc_override_ready`.
- Added `tools/check_sitl_command_link_readiness.py` to lock the distinction
  between link activity, target availability, and recent heartbeat evidence.
- Strengthened source contract audit coverage for the lanes that must be fixed
  before physical tuning:
  - sim-time sensor publishing versus wall-time MAVLink/RC polling
  - sensor input/output snapshot ownership
  - raw 18-channel RC override forwarding and `/mavros/rc/in` mirroring
  - raw JSON SERVO/replay RCOU plant input before thruster conversion
  - opt-in dynamic MuJoCo ellipsoid `fluidcoef` updates over the five
    coefficients `(blunt, slender, angular, Kutta, Magnus)`
- Split extended source-audit checks into focused modules:
  - `tools/audit_code_contract_runtime_time.py`
  - `tools/audit_code_contract_runtime_rc.py`
  - `tools/audit_code_contract_runtime_plant_input.py`
  - `tools/audit_code_contract_runtime_dynamic_fluidcoef.py`
  - compatibility facade `tools/audit_code_contract_runtime_surface_ext.py`
- Updated `docs/architecture/ACTIVE_CONTRACT_WORKLIST.md` so these lanes are
  explicit gates, not optional tuning notes.

## Validation

```text
compileall tools/bridge/sim/physics/gui: PASS
contract_source_audit: fail=0 pass=16 warn=5
closed_loop_contract: real_vs_sitl_mismatches={} missing_sitl_params=[]
hydrostatic_buoyancy_points=PASS
gui_rc_replay_decode=PASS
ros2_command_payload=PASS
sitl_command_link_readiness=PASS
runtime_readiness_policy=PASS
gui_readiness_contract=PASS
gui_backend_selection=PASS
rc_frame_contract=PASS
ardusub_thruster_contract=OK
```

## Notes

- The default `current` profile does not secretly enable dynamic `fluidcoef`;
  the audit verifies the opt-in dynamic path and its bounded state/load update
  contract.
- The command-link readiness smoke uses positive synthetic wall timestamps
  because the runtime treats timestamp `<= 0` as "no evidence".
