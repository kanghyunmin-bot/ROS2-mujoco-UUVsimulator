# GUI Config Split

Date: 2026-06-07

## Scope

Split the GUI configuration hotspot without changing GUI defaults, RC channel
contracts, launcher paths, or physics tuning schema values.

## Changed Files

- `gui/config_paths.py`: active runtime, workspace, bag, ROS package, RViz,
  GeographicLib, and physics profile paths.
- `gui/config_backend.py`: backend names, default backend, pilot control mode,
  and pilot mode environment selection.
- `gui/config_env.py`: bounded environment parsing helpers.
- `gui/config_rc.py`: axis limits, RC PWM/channel constants, real contract
  pilot defaults, RC layout table, and cmd_vel axis scale defaults.
- `gui/config_ui.py`: update period, telemetry limit, window sizing, padding,
  and canvas dimensions.
- `gui/config_physics.py`: GUI physics profile name, parameter specs, and
  inactive-current-mode notes.
- `gui/config.py`: compatibility export surface.

## Contract Notes

- RC channel mapping remains `ch3=heave`, `ch4=yaw`, `ch5=forward`,
  `ch6=lateral` for both MAVROS and the MuJoCo sim bridge.
- `UUV_GUI_RC_PWM_SPAN`, `SITL_RC3_*`, `SITL_PILOT_SPEED_*`, and
  `UUV_ALT_HOLD_RC_HEAVE_INVERT` environment behavior is unchanged.
- Existing `from gui.config import ...` and the legacy wildcard import from
  `gui/uuv_control_gui.py` are preserved.
- No plant-input, controller-parity, or ArduPilot contract was changed.

## Validation

```text
python3 -m compileall -q sim/current/gui/config*.py
/Users/kanghyunmin/miniconda3/envs/ros2_h311/bin/python - <<'PY'
import sys
from pathlib import Path
sys.path.insert(0, str(Path('sim/current').resolve()))
import gui.config as c
required = [
    'APP_ROOT', 'SIM_STACK_DIR', 'BACKEND_AUTO', 'BACKEND_NONE',
    'BACKEND_MAVROS', 'BACKEND_SIM_BRIDGE', 'DEFAULT_AUTO_BACKEND',
    'GUI_PILOT_CONTROL_MODE', 'AXIS_DEADBAND', 'RC_LAYOUTS',
    'REAL_PILOT_SPEED_UP', 'UI_UPDATE_PERIOD_MS', 'PHYSICS_PROFILE_PATH',
    'PHYSICS_PROFILE_NAME', 'PHYSICS_PARAM_SPECS',
    'CURRENT_MODE_INACTIVE_PHYSICS_KEYS', '_pilot_control_mode',
    '_env_float_default', '_runtime_profile', '_profile_default_update_ms',
    '_env_int',
]
missing = [name for name in required if not hasattr(c, name)]
raise SystemExit(1 if missing else 0)
PY
```

Observed status:

```text
config import surface: PASS
refactor inventory: gui/config.py removed from top 20 hotspot list
```
