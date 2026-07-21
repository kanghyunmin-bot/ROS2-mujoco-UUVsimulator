# GUI ROS Env And Launch Args Split

Date: 2026-06-07

Scope: active runtime under `uuv_mujoco/current`, backed by
`uuv_mujoco/v2.2`.

## Changes

- Split `gui/ros_env_tools.py` into focused modules:
  - `gui/ros_python_runtime.py`
  - `gui/ros_setup_paths.py`
  - `gui/ros_bash.py`
- Preserved the `gui.ros_tools` compatibility exports used by GUI ROS/RViz
  helpers.
- Split `gui/sim_stack_env_args.py` into focused modules:
  - `gui/sim_stack_initial_depth_args.py`
  - `gui/sim_stack_extra_args.py`
- Preserved the GUI Start launch-argument contract:
  - explicit `--initial-depth-m` and `--initial-bar30-depth-m` still win;
  - `UUV_REAL_START_STATE=1` still delegates initial state to the launcher;
  - Bar30 depth remains the default GUI-start depth path;
  - base-link depth remains an explicit debug path when Bar30 depth is off;
  - wrapper-only args are still dropped before reaching `run_uuv_mujoco.py`;
  - viewer/headless and QGC video defaults are unchanged.

## Verification

```bash
python3 -m compileall -q \
  uuv_mujoco/current/gui/ros_env_tools.py \
  uuv_mujoco/current/gui/ros_python_runtime.py \
  uuv_mujoco/current/gui/ros_setup_paths.py \
  uuv_mujoco/current/gui/ros_bash.py \
  uuv_mujoco/current/gui/ros_tools.py \
  uuv_mujoco/current/gui/sim_stack_env_args.py \
  uuv_mujoco/current/gui/sim_stack_initial_depth_args.py \
  uuv_mujoco/current/gui/sim_stack_extra_args.py \
  uuv_mujoco/current/gui/sim_stack_launch_args.py \
  uuv_mujoco/current/gui/sim_stack_env.py

PYTHONPATH=uuv_mujoco/current python3 - <<'PY'
from pathlib import Path
from gui.ros_tools import (
    candidate_ros_base_setup_paths,
    existing_ros_setup_paths,
    resolve_autotune_python,
    ros_bash_command,
    selected_ros_base_setup_path,
    setup_path_has_ros_package,
)

assert callable(candidate_ros_base_setup_paths)
assert callable(existing_ros_setup_paths)
assert callable(resolve_autotune_python)
assert callable(ros_bash_command)
assert callable(selected_ros_base_setup_path)
assert callable(setup_path_has_ros_package)
cmd = ros_bash_command("echo ok", cwd=Path("/tmp"), include_workspace=False)
assert cmd[0:2] == ["bash", "-lc"]
assert "cd /tmp" in cmd[2]
assert cmd[2].splitlines()[-1] == "echo ok"
PY

PYTHONPATH=uuv_mujoco/current python3 - <<'PY'
from gui.sim_stack_env_args import build_initial_depth_args, normalize_sim_extra_args

base = {
    "UUV_GUI_DEFAULT_INITIAL_BAR30_DEPTH_M": "auto",
    "UUV_GUI_MUJOCO_VIEWER": "0",
}
initial = build_initial_depth_args(base, launch_extra_args=[])
assert initial.args == ("--initial-bar30-depth-m", "auto")
manual = build_initial_depth_args(
    {"UUV_GUI_INITIAL_BAR30_DEPTH_M": "off", "UUV_GUI_INITIAL_DEPTH_M": "-0.2"},
    launch_extra_args=[],
)
assert manual.args == ("--initial-depth-m", "-0.2")
real = build_initial_depth_args({"UUV_REAL_START_STATE": "1"}, launch_extra_args=[])
assert real.events == ("sim initial state: real CSV contract handled by launcher",)
normalized = normalize_sim_extra_args(
    ["--direct-mavlink", "--scene", "foo.xml"],
    base,
    platform_name="linux",
)
assert normalized.args == ("--scene", "foo.xml", "--headless", "--no-qgc-video")
PY

python3 uuv_mujoco/current/tools/refactor_inventory.py \
  --root uuv_mujoco/current --limit 50
```

## Notes

- This pass does not change ArduPilot, SITL params, RC override mapping,
  Bar30/static-pressure contracts, thruster coefficients, or plant physics.
- The split specifically reduces GUI startup coupling so future Ubuntu
  migration and direct GUI Start debugging have smaller files to inspect.
