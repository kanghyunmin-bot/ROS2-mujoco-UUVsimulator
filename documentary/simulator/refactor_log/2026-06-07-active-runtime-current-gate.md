# Active Runtime Current Gate

Date: 2026-06-07

## Problem

The active simulator implementation still lives in a compatibility backing
directory named `uuv_mujoco/v2.2`.  That name is stale and makes launchers,
docs, and setup scripts look as if they are intentionally running an old
runtime.  Silent fallback from `sim/current` to `uuv_mujoco/v2.2` also
makes it easy to miss a broken or missing active-runtime alias.

## Change

- Root GUI launchers now resolve `UUV_MUJOCO_RUNTIME_DIR` to
  `sim/current` by default.
- Docker entrypoint, cleanup, install run-after-install, and stack verification
  now fail if the active runtime is missing instead of silently launching the
  legacy backing path.
- Setup still accepts old zip/package layouts that contain only `v2.2`, but it
  recreates `sim/current` and then runs through that active alias.
- New docs and commands should use `sim/current`; `v2.2` is only a
  compatibility backing directory or historical evidence path.

## Validation

```text
bash -n .uuv_mujoco_env.sh run_control_gui.sh run_control_gui_ubuntu.sh \
  cleanup_generated_artifacts.sh docker/ardusub/entrypoint.sh \
  setup/install_uuv_mujoco.sh setup/03_setup_uuv_mujoco.sh \
  setup/04_verify_uuv_stack.sh \
  uuv_mujoco/v2.2/launch_uuv_sim.sh \
  uuv_mujoco/v2.2/start_docker_sitl_mujoco_mj311.sh
```

```text
python3 -m py_compile uuv_control_gui.py \
  uuv_mujoco/v2.2/tools/check_dev_os_compat.py
```

```text
source ./.uuv_mujoco_env.sh
echo "$UUV_MUJOCO_RUNTIME_DIR"
# /Users/kanghyunmin/Desktop/uuv_sim/sim/current
```

```text
./setup/04_verify_uuv_stack.sh
# PASS=26 WARN=3 FAIL=0
```

```text
python3 sim/current/tools/check_dev_os_compat.py --headless --target-os ubuntu
# fail=0, pass=16, warn=2
```
