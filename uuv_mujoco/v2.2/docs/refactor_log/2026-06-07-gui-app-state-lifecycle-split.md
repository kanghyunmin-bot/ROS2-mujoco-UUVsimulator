# GUI App State And Lifecycle Split

Date: 2026-06-07

Scope: active GUI runtime under `uuv_mujoco/current/gui`.

## Change

- `gui/app.py` now keeps application composition, `parse_args()`, and `main()`.
- Tk variable initialization and runtime process/thread state moved to
  `gui/app_state.py`.
- Executor spin, initial window raise, update scheduling, shutdown, and mainloop
  logging moved to `gui/app_lifecycle.py`.

## Preserved Contract

- `UuvControlGui` still composes the same mixins in the same order.
- The root window setup, executor startup, layout build, close handler,
  initial raise, liveness log, and UI update scheduling still execute during
  `UuvControlGui.__init__()`.
- Shutdown still stops RC replay, releases RC override, terminates GUI-owned
  processes, resets GUI-owned sim stack, shuts down the executor/node/rclpy, and
  destroys the Tk root.

## Validation

- `app.py`, `app_state.py`, and `app_lifecycle.py` pass `py_compile`.
- GUI app import-surface smoke passed under the ROS Python environment used by
  the GUI launcher.
- Full runtime compile passed.
- Source contract audit after the split reports `fail=0`, `pass=11`, `warn=5`.
- Runtime readiness and thruster contract gates pass.
- Ubuntu compatibility gate reports `fail=0`, `pass=16`, `warn=2`; the warnings
  are Docker daemon unavailable and ROS2 not sourced in the current shell.
- Refactor inventory shows `gui/app.py` reduced from `303 LOC / 22` branches to
  `94 LOC / 1` branch and removed from the top 35 hotspot list.
