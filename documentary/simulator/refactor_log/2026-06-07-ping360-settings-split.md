# Ping360 Settings Split

Date: 2026-06-07

Scope: active runtime `sim/current`.

Changed ownership:

- `bridge/ping360_settings.py` remains the compatibility facade.
- `bridge/ping360_interface_timing.py` owns serial/interface timing settings.
- `bridge/ping360_range_settings.py` owns range and sample-count settings.
- `bridge/ping360_transmit_settings.py` owns gain, transmit duration, and frequency settings.
- `bridge/ping360_angle_settings.py` owns scan angle and sector normalization.

Validation:

- `ping360_settings_smoke PASS`
- Refactor inventory no longer lists `bridge/ping360_settings.py` as a top hotspot.

Contract note:

- This is behavior-neutral. Ping360 simulation, ROS payload shape, and GUI settings names are preserved.
