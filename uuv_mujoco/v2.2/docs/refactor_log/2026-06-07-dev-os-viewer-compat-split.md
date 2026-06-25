# Dev OS Viewer Compatibility Split

## Scope

Split host viewer compatibility checks without changing launcher selection,
MuJoCo imports, Docker checks, ROS2 environment checks, or Ubuntu migration
policy.

## Files

- `tools/dev_os_compat_viewer.py`: compatibility exports for
  `check_mjpython()` and `check_display()`.
- `tools/dev_os_compat_mjpython.py`: macOS `mjpython` candidate selection,
  probe execution, and probe-result summary.
- `tools/dev_os_compat_display.py`: headless/macOS/Linux DISPLAY and Wayland
  display checks.

## Contract

- `tools/dev_os_compat_runtime.py` still imports `check_display` and
  `check_mjpython` through `dev_os_compat_viewer.py`.
- macOS viewer runs still require `mjpython` unless headless mode is selected.
- Ubuntu/Linux viewer readiness still depends on `DISPLAY` or
  `WAYLAND_DISPLAY`, except when target-host checking is deferred from macOS.
- No SITL launch, RC override, plant input, or sensor contract was modified.

## Verification

```bash
python3 -m compileall -q \
  uuv_mujoco/v2.2/tools/dev_os_compat_viewer.py \
  uuv_mujoco/v2.2/tools/dev_os_compat_mjpython.py \
  uuv_mujoco/v2.2/tools/dev_os_compat_display.py \
  uuv_mujoco/v2.2/tools/check_dev_os_compat.py

python3 uuv_mujoco/v2.2/tools/check_dev_os_compat.py --headless --json

python3 -m compileall -q uuv_mujoco/current uuv_control_gui.py
git diff --check
python3 uuv_mujoco/current/tools/audit_code_contract_sources.py
python3 uuv_mujoco/current/tools/check_runtime_readiness_policy.py
python3 uuv_mujoco/current/tools/verify_ardusub_thruster_contract.py --quiet
/Users/kanghyunmin/.venvs/mujoco311/bin/python \
  uuv_mujoco/current/tools/physics_contract_audit.py --simulate-s 0.05
```

The dev-OS smoke reported `fail=0`, `pass=16`, and `warn=2`.  The warnings are
current host-state warnings for Docker daemon availability and ROS2 shell
sourcing, not regressions from the viewer split.
