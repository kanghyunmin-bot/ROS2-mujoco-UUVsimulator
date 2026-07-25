# SITL Transport Config and Physics GUI Split

Date: 2026-06-07

Scope: active runtime under `sim/current`, backed by the compatibility
directory `uuv_mujoco/v2.2`.

## Changes

- Split `bridge/sitl_transport_config.py` into focused construction config
  modules:
  - `bridge/sitl_transport_json_config.py`
  - `bridge/sitl_transport_control_config.py`
  - `bridge/sitl_transport_mavlink_config.py`
  - `bridge/sitl_transport_extnav_config.py`
- Split `gui/physics_mixin.py` into focused GUI physics editor modules:
  - `gui/physics_window.py`
  - `gui/physics_param_io.py`
  - `gui/physics_restart.py`
- Added `active_runtime_alias_current` to the source-contract audit so
  `sim/current`, the primary runner, and `uuv_mujoco/RUNTIME_VERSION.json`
  are verified as part of every source-contract check.

## Contract Notes

- `bridge.sitl_transport_config` remains the import surface used by
  `SitlTransport`.
- JSON servo, MAVLink telemetry, command link, auto-ready, ExternalNav, and
  polling state initialization order is preserved.
- GUI physics editor still writes the same simulation profile JSON with a
  timestamped backup before mutation.
- Inactive current-mode parameters remain read-only and are not applied.
- No ArduPilot source, submodule pointer, PWM remap, ALT_HOLD shim, or
  controller/plant observation contract was changed.

## Verification

- `python3 -m py_compile` passed for the split transport config and physics GUI
  modules.
- `bridge.sitl_transport_config` facade exposes the same initializer names.
- `PhysicsMixin` still exposes the same public/private methods consumed by
  `gui/app.py` and tuning layout code.
- `tools/refactor_inventory.py --root sim/current --limit 15` no longer
  lists `bridge/sitl_transport_config.py` or `gui/physics_mixin.py` in the top
  hotspot list.
- `PYTHONPATH=sim/current/tools python3
  sim/current/tools/audit_code_contract_sources.py --out-dir
  /private/tmp/uuv_code_contract_runtime_alias_check` passed with `fail=0`,
  `pass=11`, `warn=5`.
