# GUI AutoTune Split

Date: 2026-06-07

## Scope

Split the GUI AutoTune workflow without changing command arguments, default bag
path, candidate defaults, or monitor parsing behavior.

## Changed Files

- `gui/autotune_value_readers.py`: bounded Tk variable readers shared by
  AutoTune and Ping360 settings.
- `gui/autotune_panel_runtime.py`: panel visibility, status update, running
  predicate, and bag file dialog.
- `gui/autotune_process_runtime.py`: AutoTune process launch, stdout reader,
  stop, and open-output behavior.
- `gui/autotune_mixin.py`: compatibility method-binding surface.
- `gui/autotune_monitor_window.py`: added explicit
  `finish_autotune_monitor()`.
- `gui/autotune_monitor.py`: exports `finish_autotune_monitor()`.

## Contract Notes

- AutoTune command construction still passes `--bag`, `--out-root`,
  `--start-offset-s`, `--duration-s`, `--sitl-servo-scale`,
  `--max-candidates`, `--tune-mode`, and `--candidate-set`.
- `--apply-best` behavior is unchanged.
- Bag path validation uses the original `Path(var).expanduser()` behavior.
- `_read_float_var()` and `_read_int_var()` remain available on
  `AutoTuneMixin`, preserving Ping360 setting calls that use those methods.
- A latent completion bug was fixed: the previous wrapper referenced
  `autotune_monitor.finish_autotune_monitor()` even though the facade did not
  provide that symbol.

## Validation

```text
python3 -m compileall -q sim/current/gui/autotune*.py
/Users/kanghyunmin/miniconda3/envs/ros2_h311/bin/python - <<'PY'
import sys
from pathlib import Path
sys.path.insert(0, str(Path('sim/current').resolve()))
from gui.autotune_mixin import AutoTuneMixin
required = [
    '_toggle_autotune_panel', '_autotune_running', '_set_autotune_status',
    '_browse_autotune_bag', '_read_float_var', '_read_int_var',
    '_show_autotune_monitor', '_reset_autotune_monitor',
    '_finish_autotune_monitor', '_start_autotune', '_read_autotune_output',
    '_stop_autotune', '_open_autotune_output',
]
missing = [name for name in required if not hasattr(AutoTuneMixin, name)]
raise SystemExit(1 if missing else 0)
PY
```

Observed status:

```text
AutoTune import surface: PASS
bounded value readers: PASS
refactor inventory: gui/autotune_mixin.py removed from top 20 hotspot list
```
