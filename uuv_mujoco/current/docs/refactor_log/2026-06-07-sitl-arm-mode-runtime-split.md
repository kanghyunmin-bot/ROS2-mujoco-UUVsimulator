# SITL arm/mode runtime split

Date: 2026-06-07

## Scope

- Added focused arm/mode modules:
  - `bridge/sitl_arm_mode_send.py`
  - `bridge/sitl_arm_mode_queue.py`
  - `bridge/sitl_arm_mode_service.py`
- Kept `bridge/sitl_arm_mode_runtime.py` as the compatibility export surface
  consumed by `bridge/sitl_commanding.py`.

## Contract

The split preserves low-level MAVLink send helpers, queue entry points,
pending retry service methods, and public `send_arm_command()` /
`send_set_mode()` bindings used by `Ros2Bridge` arm/mode forwarding.

## Verification

```text
python3 -m compileall -q uuv_mujoco/current/bridge/sitl_arm_mode_runtime.py uuv_mujoco/current/bridge/sitl_arm_mode_send.py uuv_mujoco/current/bridge/sitl_arm_mode_queue.py uuv_mujoco/current/bridge/sitl_arm_mode_service.py
PYTHONPATH=uuv_mujoco/current python3 - <<'PY'
from bridge import sitl_arm_mode_runtime, sitl_commanding
required = ['_send_arm_disarm_mavlink', '_mode_id_for_text', '_send_set_mode_mavlink', 'queue_arm_command', 'queue_set_mode', '_service_pending_arm_command', '_service_pending_mode_command', 'send_arm_command', 'send_set_mode']
missing = [name for name in required if not hasattr(sitl_arm_mode_runtime, name) or not hasattr(sitl_commanding, name)]
print({'missing': missing})
PY
```

Smoke result: `missing=[]`.
