# GUI ROS Tools Split

Date: 2026-06-07

Scope: active runtime GUI helpers under `sim/current/gui`.

## What changed

- Split ROS setup discovery, shell command construction, and autotune Python
  selection into `gui/ros_env_tools.py`.
- Split ROS2-compatible RViz config generation and Ping360 RViz config
  generation into `gui/rviz_config_tools.py`.
- Reduced `gui/ros_tools.py` to compatibility exports consumed by
  `gui/uuv_control_gui.py`, `gui/ping360_mixin.py`,
  `gui/rviz_process_mixin.py`, and `gui/ros_package_mixin.py`.

## Contract boundaries preserved

- ROS setup search order is unchanged.
- `ros_bash_command()` still sources the selected base setup and workspace
  setup scripts.
- RViz compatibility YAML content is unchanged.
- No simulator runtime, RC, sensor, or ArduPilot behavior changed.

## Verification

```text
python3 -m py_compile \
  sim/current/gui/ros_tools.py \
  sim/current/gui/ros_env_tools.py \
  sim/current/gui/rviz_config_tools.py \
  sim/current/gui/ping360_mixin.py \
  sim/current/gui/rviz_process_mixin.py \
  sim/current/gui/ros_package_mixin.py
PYTHONPATH="sim/current:${PYTHONPATH:-}" \
  /Users/kanghyunmin/miniconda3/envs/ros2_h311/bin/python - <<'PY'
from gui import ros_tools
required = [
    'candidate_ros_base_setup_paths', 'existing_ros_setup_paths', 'prepare_ping360_rviz_config',
    'prepare_ros2_rviz_config', 'resolve_autotune_python', 'ros_bash_command',
    'selected_ros_base_setup_path', 'setup_path_has_ros_package',
]
status = {name: callable(getattr(ros_tools, name, None)) for name in required}
print(status)
if not all(status.values()):
    raise SystemExit(1)
PY
python3 -m compileall -q sim/current uuv_control_gui.py
PYTHONPATH=sim/current/tools python3 \
  sim/current/tools/audit_code_contract_sources.py \
  --out-dir /private/tmp/uuv_code_contract_after_ros_tools_split
python3 sim/current/tools/check_runtime_readiness_policy.py
python3 sim/current/tools/verify_ardusub_thruster_contract.py --quiet
python3 sim/current/tools/check_dev_os_compat.py --headless --target-os ubuntu
git diff --check
python3 sim/current/tools/refactor_inventory.py --limit 20
```

Results:

- Compile/import: pass.
- Source contract audit: `fail=0`, `pass=11`, `warn=5`.
- Runtime readiness policy: pass.
- ArduSub thruster contract: pass.
- Dev OS compatibility: `fail=0`, `pass=16`, `warn=2` (`docker_daemon`,
  `ros2_env`).
- `git diff --check`: pass.
- `gui/ros_tools.py` removed from the top hotspot list.
