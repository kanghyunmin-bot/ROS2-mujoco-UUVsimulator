# Runner Entrypoint Split

Date: 2026-06-07

Scope: active MuJoCo runner under `sim/current/run_uuv_mujoco.py`.

## Changes

- Split `run_uuv_mujoco.py` into focused runner setup modules:
  - `sim/runtime/runner_profile_setup.py`
  - `sim/runtime/runner_initial_setup.py`
  - `sim/runtime/runner_physics_setup.py`
  - `sim/runtime/runner_step_setup.py`
  - `sim/runtime/runner_loop_setup.py`
- Kept `run_uuv_mujoco.py` as the active CLI entrypoint.
- Preserved runner constants for scenes, profiles, and thruster performance
  defaults.

## Contract Notes

- Runtime mode resolution still feeds `UUV_RUN_MODE` through
  `sim/runtime/run_mode.py`.
- `--list-profiles` still exits before model loading.
- Initial depth/real-start setup, Bar30 depth helpers, model IO, QGC video
  wiring, SITL raw-PWM path, direct command fallback policy, physics setup,
  `SimulationStepRuntime`, and final runtime loop call are preserved through the
  same public runtime modules.
- No ArduPilot source, submodule pointer, controller parity observation surface,
  JSON servo plant input, PWM remap, or ALT_HOLD shim changed.

## Verification

```bash
python3 -m py_compile \
  sim/current/run_uuv_mujoco.py \
  sim/current/sim/runtime/runner_profile_setup.py \
  sim/current/sim/runtime/runner_initial_setup.py \
  sim/current/sim/runtime/runner_physics_setup.py \
  sim/current/sim/runtime/runner_step_setup.py \
  sim/current/sim/runtime/runner_loop_setup.py
source ./.uuv_mujoco_env.sh && "$MJ311_PYTHON" sim/current/run_uuv_mujoco.py --help
source ./.uuv_mujoco_env.sh && "$MJ311_PYTHON" sim/current/run_uuv_mujoco.py --list-profiles
source ./.uuv_mujoco_env.sh && PYTHONPATH=sim/current/tools \
  "$MJ311_PYTHON" sim/current/tools/physics_contract_audit.py \
  --output-dir /private/tmp/uuv_physics_contract_after_runner_split \
  --simulate-s 0
python3 sim/current/tools/refactor_inventory.py --root sim/current --limit 40
```

Results:

- `run_uuv_mujoco.py` dropped from `335 LOC / 7` branches to
  `113 LOC / 2` branches.
- `--help` and `--list-profiles` remain runnable through the active runtime.
- Physics contract audit preserved the neutral static force balance:
  `net_down=+0.000N`, `required_scale=1.000000`.
- `run_uuv_mujoco.py` no longer appears in the top 40 hotspot inventory.
