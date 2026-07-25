# GUI Node Telemetry Callback Split

Date: 2026-06-07

Scope: active runtime GUI code under `sim/current/gui`.

## What changed

- Split vehicle state and status text callbacks into `gui/node_vehicle_callbacks.py`.
- Split motion, pose, velocity, depth, pressure, and battery callbacks into
  `gui/node_motion_callbacks.py`.
- Split RC input/output callbacks into `gui/node_rc_callbacks.py`.
- Split SITL MAVLink telemetry and real-start JSON status callbacks into
  `gui/node_sitl_status_callbacks.py`.
- Split Ping360 status parsing into `gui/node_ping360_callbacks.py`.
- Reduced `gui/node_telemetry_callbacks.py` to a compatibility facade that
  re-exports the existing callback names used by `gui/node.py`.

## Contract boundaries preserved

- No ArduPilot source or submodule pointer changes.
- No RC channel remap, PWM correction, ALT_HOLD shim, or plant-input semantic
  change.
- GUI callback names remain bound on `UuvGuiNode`.
- Sensor/RC/telemetry callback surfaces are now inspectable by topic family.

## Verification

```text
python3 -m py_compile \
  sim/current/gui/node_telemetry_callbacks.py \
  sim/current/gui/node_vehicle_callbacks.py \
  sim/current/gui/node_motion_callbacks.py \
  sim/current/gui/node_rc_callbacks.py \
  sim/current/gui/node_sitl_status_callbacks.py \
  sim/current/gui/node_ping360_callbacks.py \
  sim/current/gui/node.py
PYTHONPATH="sim/current:${PYTHONPATH:-}" \
  /Users/kanghyunmin/miniconda3/envs/ros2_h311/bin/python - <<'PY'
from gui.node import UuvGuiNode
required = [
    '_on_state', '_on_imu', '_on_battery', '_on_pose', '_on_odom',
    '_on_local_odom', '_on_rovio_odom', '_on_dvl_odom',
    '_on_velocity', '_on_velocity_body', '_on_velocity_local',
    '_on_dvl_velocity', '_on_ground_truth_pose',
    '_on_rc_out', '_on_rc_in', '_on_status_text',
    '_on_sitl_mavlink_telemetry_status', '_on_real_start_status',
    '_on_depth', '_on_bar30_pressure', '_on_ping360_status',
    '_on_atm_pressure', '_on_static_pressure', '_on_pressure_value',
]
status = {name: callable(getattr(UuvGuiNode, name, None)) for name in required}
print(status)
if not all(status.values()):
    raise SystemExit(1)
PY
python3 -m compileall -q sim/current uuv_control_gui.py
PYTHONPATH=sim/current/tools python3 \
  sim/current/tools/audit_code_contract_sources.py \
  --out-dir /private/tmp/uuv_code_contract_after_node_telemetry_split
python3 sim/current/tools/check_runtime_readiness_policy.py
python3 sim/current/tools/verify_ardusub_thruster_contract.py --quiet
python3 sim/current/tools/check_dev_os_compat.py --headless --target-os ubuntu
git diff --check
python3 sim/current/tools/refactor_inventory.py --limit 25
```

Results:

- Compile/import: pass.
- Source contract audit: `fail=0`, `pass=11`, `warn=5`.
- Runtime readiness policy: pass.
- ArduSub thruster contract: pass.
- Dev OS compatibility: `fail=0`, `pass=16`, `warn=2` (`docker_daemon`,
  `ros2_env`).
- `git diff --check`: pass.
- `gui/node_telemetry_callbacks.py` removed from the top hotspot list.
