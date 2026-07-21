# GUI Node State Runtime Split

Date: 2026-06-07

Scope: active runtime GUI code under `uuv_mujoco/current/gui`.

## What changed

- Split backend graph probing, subscriber/publisher counts, service readiness,
  effective backend selection, and RC layout labels into
  `gui/node_backend_runtime.py`.
- Split payload float parsing, event insertion, touch timestamps, and telemetry
  snapshot copying/age calculation into `gui/node_snapshot_runtime.py`.
- Split command readiness, SITL command heartbeat liveness, and ExternalNav
  readiness helpers into `gui/node_readiness_runtime.py`.
- Reduced `gui/node_state_runtime.py` to a compatibility facade that re-exports
  the existing helper names used by `gui/node.py`.

## Contract boundaries preserved

- The GUI READY/WAIT policy still flows through
  `sim.runtime.readiness.command_readiness_label()`.
- Runtime command-path requirements for the internal sim bridge backend are
  unchanged.
- No ArduPilot source, submodule pointer, RC mapping, PWM correction, or plant
  input semantics changed.

## Verification

```text
python3 -m py_compile \
  uuv_mujoco/current/gui/node_state_runtime.py \
  uuv_mujoco/current/gui/node_backend_runtime.py \
  uuv_mujoco/current/gui/node_readiness_runtime.py \
  uuv_mujoco/current/gui/node_snapshot_runtime.py \
  uuv_mujoco/current/gui/node.py
PYTHONPATH="uuv_mujoco/current:${PYTHONPATH:-}" \
  /Users/kanghyunmin/miniconda3/envs/ros2_h311/bin/python - <<'PY'
from gui.node import UuvGuiNode
required = [
    '_safe_count_publishers', '_safe_count_subscribers', '_service_ready',
    '_payload_float', '_effective_backend', '_active_layout', 'backend_label',
    'rc_mapping_summary', 'control_readiness', '_probe_backend', '_touch',
    '_push_event', 'snapshot', '_sitl_mavlink_command_alive', '_sitl_extnav_ready',
]
status = {name: callable(getattr(UuvGuiNode, name, None)) for name in required}
print(status)
if not all(status.values()):
    raise SystemExit(1)
PY
python3 -m compileall -q uuv_mujoco/current uuv_control_gui.py
PYTHONPATH=uuv_mujoco/current/tools python3 \
  uuv_mujoco/current/tools/audit_code_contract_sources.py \
  --out-dir /private/tmp/uuv_code_contract_after_node_state_split
python3 uuv_mujoco/current/tools/check_runtime_readiness_policy.py
python3 uuv_mujoco/current/tools/verify_ardusub_thruster_contract.py --quiet
python3 uuv_mujoco/current/tools/check_dev_os_compat.py --headless --target-os ubuntu
git diff --check
python3 uuv_mujoco/current/tools/refactor_inventory.py --limit 20
```

Results:

- Compile/import: pass.
- Source contract audit: `fail=0`, `pass=11`, `warn=5`.
- Runtime readiness policy: pass.
- ArduSub thruster contract: pass.
- Dev OS compatibility: `fail=0`, `pass=16`, `warn=2` (`docker_daemon`,
  `ros2_env`).
- `git diff --check`: pass.
- `gui/node_state_runtime.py` removed from the top hotspot list.
