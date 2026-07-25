# Runner Physics Setup Split

Date: 2026-06-07

Scope:

- `sim/runtime/runner_physics_setup.py`
- `sim/runtime/runner_control_path_log.py`
- `sim/runtime/runner_fluid_contract_setup.py`
- `sim/runtime/runner_model_io_contract.py`
- `sim/runtime/runner_runtime_physics_contract.py`

Intent:

- Reduce the runner execution-path hotspot without changing the physics setup
  order, plant-input ownership, or ArduSub thruster naming contract.
- Keep `create_runner_physics_setup()` and `RunnerPhysicsSetup` as the
  runner-facing API while making each setup side effect independently
  auditable.

Contract notes:

- Fluid model contract still calls `configure_fluid_model_contract()` before
  runtime physics construction.
- Runtime control-path logs preserve the same SITL/ROS2 messages.
- Model IO setup still uses `PHYSICAL_VERTICAL_THRUSTERS`,
  `PHYSICAL_YAW_THRUSTERS`, and `ARDUSUB_VECTORED_6DOF_YAW_CHANNEL_ORDER`.
- Runtime physics setup still receives the raw
  `plant_replay_direct_rcout` flag, ArduSub servo map/sign list, command
  state, initial-depth hold state, and thruster immersion settings.

Verification:

- `python3 -m compileall -q sim/current/sim/runtime/runner_physics_setup.py sim/current/sim/runtime/runner_control_path_log.py sim/current/sim/runtime/runner_fluid_contract_setup.py sim/current/sim/runtime/runner_model_io_contract.py sim/current/sim/runtime/runner_runtime_physics_contract.py`
- `runner physics setup split import smoke: PASS`
- `python3 sim/current/tools/refactor_inventory.py --root sim/current --limit 35 --format markdown`

Inventory effect:

- `sim/runtime/runner_physics_setup.py` no longer appears in the top hotspot
  list.
- The new runner setup helpers do not appear in the top 35 hotspot list.

Next candidate:

- `gui/physics_param_apply.py` and `gui/replay_controls.py` remain GUI-side
  branch-heavy files.
- Runtime step and thruster setup files should only be split with contract
  smoke coverage because they sit directly on the RC/SITL plant-input path.
