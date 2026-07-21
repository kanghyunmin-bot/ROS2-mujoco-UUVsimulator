# GUI Node Init and Vehicle Info Split

Date: 2026-06-07

## Scope

The active runtime path is `uuv_mujoco/current`; the physical backing directory
is still `uuv_mujoco/v2.2` for compatibility.  This change only reorganizes GUI
node ownership inside the active runtime and does not modify ArduPilot, the
ArduPilot submodule pointer, controller-output mapping, PWM correction, or
plant-input semantics.

## Changed

- Added `gui/node_init.py` for `UuvGuiNode` publisher/subscriber/client setup,
  command/runtime state, telemetry snapshot defaults, and real-start status
  wiring.
- Added `gui/node_vehicle_info.py` for the optional vehicle-info request and
  response path.
- Reduced `gui/node.py` to the public node composition surface and direct method
  bindings for existing state, command, and telemetry helpers.

## Contract

- GUI command methods keep the same names consumed by `gui/app.py`,
  `gui/control_*`, and replay controls.
- READY, RC override, arm/mode, telemetry callback, and vehicle-info behavior
  are behavior-neutral splits.
- Active runtime freshness is still audited through `uuv_mujoco/current` and
  `uuv_mujoco/RUNTIME_VERSION.json`.

## Validation

```text
python3 -m py_compile \
  uuv_mujoco/current/gui/node.py \
  uuv_mujoco/current/gui/node_init.py \
  uuv_mujoco/current/gui/node_vehicle_info.py \
  uuv_mujoco/current/gui/app.py

source ./.uuv_mujoco_env.sh
set +u
source "$ROS_ENV_SETUP"
[[ -f "$ROS_INSTALL_SETUP" ]] && source "$ROS_INSTALL_SETUP" || true
PYTHONPATH="uuv_mujoco/current:${PYTHONPATH:-}" python3 - <<'PY'
from gui.node import UuvGuiNode
required = [
    "request_vehicle_info",
    "_on_vehicle_info_response",
    "arm",
    "set_mode",
    "publish_rc_override",
    "publish_rc_channels",
    "publish_rc_release",
    "_on_state",
    "_on_imu",
    "_on_rc_out",
    "_on_depth",
]
print({name: callable(getattr(UuvGuiNode, name, None)) for name in required})
PY

python3 -m compileall -q uuv_mujoco/current uuv_control_gui.py
PYTHONPATH=uuv_mujoco/current/tools python3 \
  uuv_mujoco/current/tools/audit_code_contract_sources.py \
  --out-dir /private/tmp/uuv_code_contract_after_gui_node_split
python3 uuv_mujoco/current/tools/check_runtime_readiness_policy.py
python3 uuv_mujoco/current/tools/verify_ardusub_thruster_contract.py --quiet
python3 uuv_mujoco/current/tools/check_dev_os_compat.py --headless --target-os ubuntu
python3 uuv_mujoco/current/tools/refactor_inventory.py --limit 40
```

Observed results:

- Python compile/import surface passed.
- Source-contract audit: `fail=0`, `pass=11`, `warn=5`.
- Runtime readiness policy: `PASS`.
- Thruster contract: `OK`.
- Development OS compatibility: `fail=0`, `pass=16`, `warn=2`.
- `gui/node.py`: `341 LOC / 19` branches -> `104 LOC / 1` branch.
