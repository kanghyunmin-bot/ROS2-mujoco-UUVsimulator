# 2026-06-07 Actuator/RViz/ALT_HOLD/Physics-Step Split

Scope: active runtime under `sim/current`, backed by the compatibility
directory `uuv_mujoco/v2.2`.

This pass continued the spaghetti reduction without changing ArduPilot,
ArduPilot submodule pointers, controller parity shims, PWM remaps, JSON servo
plant-input semantics, or MuJoCo physics coefficients.

## Changes

- Split `tools/actuator_wrench_calc.py`:
  - `tools/actuator_wrench_axis.py` owns per-axis wrench accumulation.
  - `tools/actuator_wrench_summary.py` owns force/torque leakage summaries.
  - `tools/actuator_wrench_calc.py` remains the result-builder compatibility
    surface used by `tools/actuator_wrench_audit.py`.
- Moved the long ROS2/MAVROS topic surface description out of
  `bridge/ros2_bridge.py` into `docs/contracts/ROS2_BRIDGE_SURFACE.md`.
  The bridge class now stays focused on constructor wiring and method binding.
- Split `gui/rviz_config_tools.py`:
  - `gui/rviz_config_ros2.py` owns ROS1-to-ROS2 RViz class replacement and
    `/sim/odom` display injection.
  - `gui/rviz_config_ping360.py` owns the Ping360 RViz template.
  - `gui/rviz_config_tools.py` remains the public file-writing facade.
- Split `tools/analyze_althold_contract.py`:
  - `tools/althold_contract_signals.py` owns DataFlash signal assembly.
  - `tools/althold_contract_summary.py` owns JSON payload and console summary
    output.
  - The CLI now owns argument parsing and read/write orchestration only.
- Split `sim/runtime/physics_step_callbacks.py`:
  - `sim/runtime/physics_step_debug.py` owns thruster debug payload forwarding.
  - `sim/runtime/physics_step_descent.py` owns descent-guard construction and
    enforcement payload calculation.
  - `physics_step_callbacks.py` still returns the same seven callback fields.

## Inventory Impact

The following former top-25 active-runtime hotspots no longer appear in the
top-25 inventory:

- `tools/actuator_wrench_calc.py`
- `bridge/ros2_bridge.py`
- `gui/rviz_config_tools.py`
- `tools/analyze_althold_contract.py`

`sim/runtime/physics_step_callbacks.py` remains in the top-25 by LOC, but its
inventory branch count is now `0`, down from `2`.  The remaining length is
mostly constructor callback wiring, not nested branch logic.

## Validation

Commands run from `/Users/kanghyunmin/Desktop/uuv_sim`:

```bash
python3 -m compileall -q \
  sim/current/tools/actuator_wrench_calc.py \
  sim/current/tools/actuator_wrench_axis.py \
  sim/current/tools/actuator_wrench_summary.py \
  sim/current/gui/rviz_config_tools.py \
  sim/current/gui/rviz_config_ros2.py \
  sim/current/gui/rviz_config_ping360.py \
  sim/current/tools/analyze_althold_contract.py \
  sim/current/tools/althold_contract_signals.py \
  sim/current/tools/althold_contract_summary.py \
  sim/current/sim/runtime/physics_step_callbacks.py \
  sim/current/sim/runtime/physics_step_debug.py \
  sim/current/sim/runtime/physics_step_descent.py
```

Targeted smokes:

- `actuator_wrench_audit.py` generated JSON/Markdown through the `ros2_h311`
  MuJoCo Python and preserved all six axes.
- `Ros2Bridge` still exposes the simulator-loop and SITL callback methods.
- RViz helper smoke preserved Ping360 image topic text and idempotent
  `/sim/odom` display injection.
- `analyze_althold_contract.py --help` still works.
- `build_step_physics_callbacks()` still returns seven callable fields and the
  callbacks execute against a fake runtime payload.

Full gate results are recorded in the current turn output:

- `compileall`: pass.
- `check_runtime_freshness.py --fetch --refresh-version`: pass.
- `audit_code_contract_sources.py`: `fail=0`, `pass=11`, `warn=5`.
- `check_runtime_readiness_policy.py`: pass.
- `verify_ardusub_thruster_contract.py --quiet`: pass.
- `check_dev_os_compat.py --headless --target-os ubuntu`: `fail=0`,
  `pass=16`, `warn=2`.
- `physics_contract_audit.py --simulate-s 0`: static physics contract pass.
- `git diff --check`: pass.
