# GUI Log Watcher And Lifecycle Split

Date: 2026-06-07

## Scope

- Removed generated Python bytecode and macOS `.DS_Store` files under
  `uuv_mujoco/v2.2`.
- Split GUI simulator-stack log ownership:
  - `gui/sim_stack_log_reader.py` owns process-log tailing until exit.
  - `gui/sim_stack_log_status.py` owns status-prefix classification and GUI
    status text selection.
  - `gui/sim_stack_log_watcher.py` remains the public GUI mixin surface.
- Split GUI shutdown ownership:
  - `gui/app_shutdown_steps.py` owns close-time cleanup steps.
  - `gui/app_lifecycle.py` keeps the public `_on_close()`, `_spin()`,
    `_schedule_update()`, and `run()` lifecycle methods.

## Behavior Contract

- No plant physics coefficients changed.
- No RC override, RCOU telemetry, Bar30 pressure, SITL JSON servo, or MAVLink
  command-link contract changed.
- Existing GUI method names used by `gui/app.py` are preserved.
- Simulator-stack status messages are still surfaced from the same prefixes.
- GUI close still cancels scheduled updates, stops replay state, stops Ping360
  view, terminates child processes, resets owned sim stacks, releases RC,
  shuts down ROS, and destroys the Tk root.

## Validation

```bash
python3 -m compileall -q uuv_mujoco/current uuv_control_gui.py
git diff --check
python3 uuv_mujoco/current/tools/check_rc_frame_contract.py
python3 uuv_mujoco/current/tools/check_gui_readiness_contract.py
python3 uuv_mujoco/current/tools/check_gui_backend_selection.py
python3 uuv_mujoco/current/tools/check_runtime_readiness_policy.py
PYTHONPATH=/Users/kanghyunmin/Desktop/uuv_sim/uuv_mujoco/current/tools \
  python3 uuv_mujoco/current/tools/audit_code_contract_sources.py \
  --out-dir /private/tmp/uuv_source_audit_spaghetti_20260607_log_lifecycle_split
python3 uuv_mujoco/current/tools/verify_ardusub_thruster_contract.py --quiet
/Users/kanghyunmin/.venvs/mujoco311/bin/python \
  uuv_mujoco/current/tools/physics_contract_audit.py \
  --output-dir /private/tmp/uuv_physics_audit_spaghetti_20260607_log_lifecycle_split \
  --simulate-s 0.05
python3 uuv_mujoco/current/tools/refactor_inventory.py \
  --root uuv_mujoco/current --format markdown --limit 25
```

Results:

- `rc_frame_contract=PASS`
- `gui_readiness_contract=PASS`
- `gui_backend_selection=PASS`
- `runtime_readiness_policy=PASS`
- source audit: `fail=0`, `pass=11`, `warn=5`
- thruster contract: `OK`
- physics audit neutral balance: `net_down=+0.000N`,
  `required_scale=1.000000`, neutral open-plant drift `+0.00001m`
- `gui/sim_stack_log_watcher.py` and `gui/app_lifecycle.py` no longer appear
  in the top 25 structural-complexity hotspot list.

Note: one scratch import probe was run with an intentionally invalid dummy
module name and failed before these actual validation gates.  It did not touch
the workspace and is not part of the validation set above.
