# GUI Control Update Split

Date: 2026-06-07

Scope: active runtime `uuv_mujoco/current`.

Changed ownership:

- `gui/control_update_mixin.py` remains the periodic GUI update import surface.
- `gui/control_update_text_models.py` owns immutable text payload records.
- `gui/control_update_format.py` owns small formatting helpers that do not import the ROS runtime.
- `gui/control_update_telemetry_texts.py` owns vehicle, battery, pose, velocity, IMU, and status text.
- `gui/control_update_pilot_texts.py` owns pilot/RC override text and PWM summaries.
- `gui/control_update_texts.py` remains the facade that assembles text payloads.
- `gui/control_update_apply.py` owns Tk variable/widget mutation.

Validation:

- Focused compileall for the split GUI modules passed.
- `control_update_texts_smoke PASS`
- Refactor inventory no longer lists `gui/control_update_mixin.py` or `gui/control_update_texts.py` as top hotspots.

Contract note:

- This is behavior-neutral. The GUI still renders the same snapshot fields, command-ready status, Ping360 enabled state, and RC override summaries.
