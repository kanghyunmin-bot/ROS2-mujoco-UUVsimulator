# ALT_HOLD, SITL Transport, and Auto-tune Monitor Split

Date: 2026-06-07

Scope: active runtime under `uuv_mujoco/current`, backed by the compatibility
directory `uuv_mujoco/v2.2`.

## Runtime Path Rule

`uuv_mujoco/current` is the live runtime path.  The physical directory name
`v2.2` is retained only for compatibility with existing scripts and historical
evidence paths.  New launch, GUI, Docker, setup, and validation paths should
resolve through `current`.

## Changes

- Split `tools/analyze_althold_contract.py` into focused ALT_HOLD analysis
  modules:
  - `tools/althold_contract_model.py`
  - `tools/althold_contract_bin.py`
  - `tools/althold_contract_segments.py`
  - `tools/althold_contract_plot.py`
- Split `bridge/sitl_transport.py` into focused transport helper modules:
  - `bridge/sitl_transport_model_state.py`
  - `bridge/sitl_transport_status_runtime.py`
  - `bridge/sitl_transport_handlers.py`
  - `bridge/sitl_transport_lifecycle.py`
- Split `gui/autotune_monitor.py` into focused GUI monitor modules:
  - `gui/autotune_monitor_window.py`
  - `gui/autotune_monitor_log.py`
  - `gui/autotune_monitor_candidates.py`
  - `gui/autotune_monitor_chart.py`
- Split `tools/physics_contract_model.py` into focused static physics contract
  modules:
  - `tools/physics_contract_geometry.py`
  - `tools/physics_contract_body.py`
  - `tools/physics_contract_buoyancy.py`
  - `tools/physics_contract_neutral_sim.py`

## Contract Notes

- Controller parity remains telemetry-to-telemetry:
  real `/mavros/rc/out` vs SITL MAVLink `SERVO_OUTPUT_RAW`.
- Plant input remains raw ArduSub JSON servo PWM into the MuJoCo plant.
- These splits do not add ALT_HOLD shims, PWM correction, output remaps, or
  JSON-servo-to-telemetry resampling.
- The auto-tune chart no longer imports `gui.helpers`, avoiding an unnecessary
  `gui.runtime -> rclpy` dependency during chart-only imports.
- Static plant contract math remains behavior-neutral: the split keeps body
  mass/CoM/inertia application, submerged fraction, force balance, and neutral
  open-plant replay under the same audit entry point.

## Verification

- `python3 -m py_compile` passed for the split ALT_HOLD contract, SITL
  transport, and auto-tune monitor modules.
- Standalone `gui.autotune_monitor` import passed with system Python after
  removing the unnecessary ROS-runtime helper import.
- `tools/refactor_inventory.py --root uuv_mujoco/current --limit 18` no longer
  lists `tools/analyze_althold_contract.py`, `bridge/sitl_transport.py`, or
  `gui/autotune_monitor.py` in the top hotspot list.
- `tools/physics_contract_model.py` also dropped from the top hotspot list.
- `physics_contract_audit.py --simulate-s 0` still reports mass `15.000 kg`,
  buoyancy scale `1.000000`, and static force balance net-down `+0.000 N` at
  the scene-default and auto fully-wet depths.
