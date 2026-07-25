# Runtime Readiness Split

Date: 2026-06-07

Scope: active runtime under `sim/current`, backed by
`uuv_mujoco/v2.2`.

## Changes

- Split `sim/runtime/readiness.py` into:
  - `sim/runtime/readiness_types.py`
  - `sim/runtime/readiness_label.py`
- Preserved the public `sim.runtime.readiness` import surface:
  - `RuntimeReadiness`
  - `CommandReadinessInputs`
  - `command_readiness_label`
- Kept the GUI command-readiness policy unchanged.  This is a structural split
  only, not a new READY condition or shortcut.

## Verification

```bash
python3 -m compileall -q \
  sim/current/sim/runtime/readiness.py \
  sim/current/sim/runtime/readiness_types.py \
  sim/current/sim/runtime/readiness_label.py \
  sim/current/gui/node_readiness_runtime.py \
  sim/current/tools/check_runtime_readiness_policy.py

python3 sim/current/tools/check_runtime_readiness_policy.py

PYTHONPATH=sim/current python3 - <<'PY'
from sim.runtime.readiness import CommandReadinessInputs, RuntimeReadiness, command_readiness_label

runtime = RuntimeReadiness(
    mujoco_alive=True,
    json_sensor_transport_alive=True,
    json_servo_receiver_alive=True,
    mavlink_command_endpoint_alive=True,
    external_nav_alive=True,
    vehicle_state_known=True,
    arm_state_known=True,
    mode_state_known=True,
    plant_servo_rows_available=True,
)
ready = CommandReadinessInputs(
    runtime=runtime,
    arm_service_ready=True,
    mode_service_ready=True,
    rc_source_ready=True,
    manual_input=True,
    armed=True,
    mode="MANUAL",
    required_mode="MANUAL",
    require_runtime_command_path=True,
)
assert command_readiness_label(ready) == ("READY", "Ready.TLabel")
PY
```

## Notes

- This pass does not change GUI Start readiness semantics, SITL command
  routing, RC override behavior, sensor contracts, or plant physics.
