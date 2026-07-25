# Command, Restart, Inventory, and VPD Split

Date: 2026-06-10

## Change

- Split GUI mode command request helpers by command path:
  - `gui/node_command_attempts.py`: shared retry-log cadence helper.
  - `gui/node_mode_request_gates.py`: ALT_HOLD initial-depth and arm/mode
    readiness gates.
  - `gui/node_mode_request_topic.py`: `/uuv_mujoco/sitl/command_override`
    mode override path.
  - `gui/node_mode_request_service.py`: MAVROS `SetMode` service path.
  - `gui/node_mode_request_steps.py`: compatibility facade.
- Updated `gui/node_arm_request_steps.py` to use the same retry-log helper and
  import `CommandBool` from the light MAVROS fallback module instead of the full
  GUI runtime facade.
- Split GUI physics restart into restart orchestration, reset-script execution,
  and GUI-thread finish scheduling:
  - `gui/physics_restart.py`
  - `gui/physics_restart_reset.py`
  - `gui/physics_restart_finish.py`
- Fixed an import-time coupling found during smoke testing: physics restart reset
  paths now import `gui.config` lazily, so the helper module can be imported
  before the GUI runtime has installed the simulator path.
- Split `tools/refactor_inventory_analysis.py` into path filtering, scoring, and
  AST symbol counting helpers while preserving the CLI output shape.
- Split synthetic ExternalNav `VISION_POSITION_DELTA` into:
  - `bridge/sitl_external_nav_vpd_due.py`: scheduler/rewind/due gate.
  - `bridge/sitl_external_nav_vpd_emit.py`: MAVLink emit, TX-rate accounting, and
    debug logging.
  - `bridge/sitl_external_nav_vpd_send.py`: compatibility orchestrator.

## Contract

- No ArduPilot source or submodule pointer changed.
- No RC input/output mapping, RC override frame, Bar30/static-pressure equation,
  SERVO_OUTPUT_RAW telemetry surface, thruster contract, or plant-input
  ownership changed.
- Synthetic VPD order is preserved: command link, bootstrap, native-VPD replay
  due check, rewind reset, scheduler due gate, pose/delta construction, MAVLink
  `vision_position_delta_send`.
- GUI arm/mode public method names and retry behavior are preserved.

## Verification

```text
env PYTHONPYCACHEPREFIX=/private/tmp/pycache python3 -m compileall -q sim/current/bridge sim/current/tools sim/current/gui
env PYTHONPYCACHEPREFIX=/private/tmp/pycache python3 - <<'PY'
from uuv_mujoco.current.gui.node_mode_request_steps import (
    handle_alt_hold_initial_depth_gate,
    handle_mode_gate,
    publish_mode_override_if_configured,
    send_mode_service_request,
    should_log_attempt,
)
from uuv_mujoco.current.gui.node_arm_request_steps import send_arm_service_request
assert should_log_attempt(1)
assert not should_log_attempt(2)
assert should_log_attempt(4)
assert callable(handle_alt_hold_initial_depth_gate)
assert callable(handle_mode_gate)
assert callable(publish_mode_override_if_configured)
assert callable(send_mode_service_request)
assert callable(send_arm_service_request)
print('node_command_request_imports=PASS')
PY
env PYTHONPYCACHEPREFIX=/private/tmp/pycache python3 - <<'PY'
from uuv_mujoco.current.gui.physics_restart import _restart_sim_stack_after_physics_apply
from uuv_mujoco.current.gui.physics_restart_reset import run_physics_reset_script, publish_physics_reset_log_lines
from uuv_mujoco.current.gui.physics_restart_finish import finish_physics_restart, schedule_physics_restart_finish
assert callable(_restart_sim_stack_after_physics_apply)
assert callable(run_physics_reset_script)
assert callable(publish_physics_reset_log_lines)
assert callable(finish_physics_restart)
assert callable(schedule_physics_restart_finish)
print('physics_restart_imports=PASS')
PY
env PYTHONPATH=/Users/kanghyunmin/Desktop/uuv_sim/sim/current PYTHONPYCACHEPREFIX=/private/tmp/pycache python3 - <<'PY'
from bridge.sitl_external_nav_vpd_send import _send_external_nav
from bridge.sitl_external_nav_vpd_due import _external_nav_vpd_due
from bridge.sitl_external_nav_vpd_emit import _emit_external_nav_vpd
assert callable(_send_external_nav)
assert callable(_external_nav_vpd_due)
assert callable(_emit_external_nav_vpd)
print('external_nav_vpd_imports=PASS')
PY
env PYTHONPYCACHEPREFIX=/private/tmp/pycache python3 sim/current/tools/check_gui_readiness_contract.py
env PYTHONPYCACHEPREFIX=/private/tmp/pycache python3 sim/current/tools/check_gui_backend_selection.py
env PYTHONPYCACHEPREFIX=/private/tmp/pycache python3 sim/current/tools/check_runtime_readiness_policy.py
env PYTHONPYCACHEPREFIX=/private/tmp/pycache python3 sim/current/tools/check_rc_frame_contract.py
env PYTHONPYCACHEPREFIX=/private/tmp/pycache python3 sim/current/tools/verify_ardusub_thruster_contract.py --quiet
env PYTHONPYCACHEPREFIX=/private/tmp/pycache python3 sim/current/tools/audit_code_contract_sources.py --out-dir /private/tmp/uuv_source_audit_refactor_batch_20260610
env PYTHONPYCACHEPREFIX=/private/tmp/pycache python3 sim/current/tools/refactor_inventory.py --root sim/current --format markdown --limit 20
```

Results:

- `node_command_request_imports=PASS`
- `physics_restart_imports=PASS`
- `external_nav_vpd_imports=PASS`
- `gui_readiness_contract=PASS`
- `gui_backend_selection=PASS`
- `runtime_readiness_policy=PASS`
- `rc_frame_contract=PASS`
- `[thruster-contract] OK`
- source contract audit: `{"fail": 0, "pass": 11, "warn": 5}`
- Removed from the top hotspot list in this batch:
  - `gui/node_mode_request_steps.py`
  - `gui/physics_restart.py`
  - `tools/refactor_inventory_analysis.py`
  - `bridge/sitl_external_nav_vpd_send.py`
