# GUI Runtime Freshness Preflight

Date: 2026-06-07

Scope: GUI and direct SITL+MuJoCo entry points for the active runtime exposed
through `uuv_mujoco/current`.

## Why

The compatibility backing directory is still named `uuv_mujoco/v2.2`, which can
make normal runs look stale even when the active branch is current.  MuJoCo
launch wrappers already checked source freshness, but GUI-only runs could skip
that check.

## Changed

- `run_control_gui.sh` and `run_control_gui_ubuntu.sh` now run
  `tools/check_runtime_freshness.py --fetch --warn-only` before launching the
  GUI implementation.
- Root `uuv_control_gui.py` runs the same preflight when executed as a script.
- Direct `gui/uuv_control_gui.py` script execution also runs the preflight.
- Direct `start_sitl_mujoco_mj311.sh` runs the preflight after resolving the
  workspace and before reset/SITL/MuJoCo startup.
- `uuv_mujoco/CURRENT.md`, `SPAGHETTI_AUDIT.md`, and
  `ACTIVE_CONTRACT_WORKLIST.md` now describe GUI freshness coverage.

## Validation

```text
bash -n run_control_gui.sh run_control_gui_ubuntu.sh \
  uuv_mujoco/current/start_sitl_mujoco_mj311.sh \
  uuv_mujoco/current/run_control_gui.sh \
  uuv_mujoco/run_mujoco.sh \
  uuv_mujoco/start_sitl_mujoco.sh \
  uuv_mujoco/start_docker_sitl_mujoco.sh \
  uuv_mujoco/reset_sim.sh

python3 -m py_compile \
  uuv_control_gui.py \
  uuv_mujoco/current/gui/uuv_control_gui.py \
  uuv_mujoco/current/tools/check_runtime_freshness.py \
  uuv_mujoco/current/tools/runtime_freshness_probe.py \
  uuv_mujoco/current/tools/runtime_freshness_eval.py \
  uuv_mujoco/current/tools/runtime_freshness_report.py

python3 uuv_mujoco/current/tools/check_runtime_freshness.py \
  --workspace /Users/kanghyunmin/Desktop/uuv_sim \
  --runtime-dir /Users/kanghyunmin/Desktop/uuv_sim/uuv_mujoco/current \
  --fetch

uuv_mujoco/current/start_sitl_mujoco_mj311.sh --help

git diff --check -- \
  run_control_gui.sh \
  run_control_gui_ubuntu.sh \
  uuv_control_gui.py \
  uuv_mujoco/CURRENT.md \
  uuv_mujoco/current/gui/uuv_control_gui.py \
  uuv_mujoco/current/start_sitl_mujoco_mj311.sh
```

Observed freshness status:

```text
[uuv_mujoco] runtime freshness: PASS
[uuv_mujoco] uuv_mujoco/current -> v2.2
[uuv_mujoco] branch=uuv_sim
[uuv_mujoco] HEAD=e60cd9383c214a7d6e4700cdeb5c2f2a8cf6d328
[uuv_mujoco] origin/uuv_sim=e60cd9383c214a7d6e4700cdeb5c2f2a8cf6d328
```

## Boundary

No ArduPilot source, ArduPilot submodule pointer, controller shim, PWM remap, or
physics coefficient was changed.
