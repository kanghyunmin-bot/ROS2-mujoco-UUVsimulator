# GUI simulator stack log watcher split

Date: 2026-06-07

## Scope

- Added `gui/sim_stack_log_watcher.py`.
- Kept `gui/sim_stack_launch_mixin.py` as the simulator start/restart
  sequencing mixin.

## Contract

The launch thread still calls `_watch_sim_stack_output()`.  Log status-prefix
parsing, GUI event forwarding, and process finish state updates moved behind
the same method name, so GUI controls and status text contracts remain stable.

## Verification

```text
python3 -m compileall -q sim/current/gui/sim_stack_launch_mixin.py sim/current/gui/sim_stack_log_watcher.py
```

The refactor inventory no longer lists `gui/sim_stack_launch_mixin.py` in the
top 30 hotspot table.
