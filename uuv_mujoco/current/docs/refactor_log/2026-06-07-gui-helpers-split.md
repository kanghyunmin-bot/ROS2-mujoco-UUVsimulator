# GUI Helpers Split

Date: 2026-06-07

Scope: active GUI runtime helpers under `uuv_mujoco/current/gui`.

## Changes

- Split `gui/helpers.py` into focused modules:
  - `gui/gui_math_helpers.py`
  - `gui/gui_rc_helpers.py`
  - `gui/rc_replay_loader.py`
  - `gui/backend_helpers.py`
- Kept `gui/helpers.py` as the compatibility facade for existing direct imports
  and legacy `from gui.helpers import *` mixins.
- Preserved the historical config export behavior because some GUI mixins still
  rely on constants and `Path` through the old helper facade.

## Contract Notes

- RC override channel ownership is unchanged:
  - RC3 = heave
  - RC4 = yaw
  - RC5 = forward
  - RC6 = lateral
- `make_rc_override_message`, `make_rc_release_message`,
  `sanitize_primary_rc_override_channels`, and `padded_rc_channels` still build
  MAVROS `OverrideRCIn` messages from the shared RC contract.
- ROS2 bag replay loading still reads `/mavros/rc/override` samples and keeps
  the same zero-order replay sample shape.
- Backend normalization still maps the active simulator backend names without
  changing GUI start behavior.
- No ArduPilot source, submodule pointer, controller parity observation surface,
  plant input surface, PWM remap, or ALT_HOLD shim changed.

## Verification

```bash
python3 -m py_compile \
  uuv_mujoco/current/gui/helpers.py \
  uuv_mujoco/current/gui/gui_math_helpers.py \
  uuv_mujoco/current/gui/gui_rc_helpers.py \
  uuv_mujoco/current/gui/rc_replay_loader.py \
  uuv_mujoco/current/gui/backend_helpers.py \
  uuv_mujoco/current/gui/replay_mixin.py \
  uuv_mujoco/current/gui/ros_process_mixin.py \
  uuv_mujoco/current/gui/autotune_mixin.py \
  uuv_mujoco/current/gui/node_rc_publishers.py \
  uuv_mujoco/current/gui/node_telemetry_callbacks.py \
  uuv_mujoco/current/gui/control_update_mixin.py
python3 -m compileall -q uuv_mujoco/current uuv_control_gui.py
source ./.uuv_mujoco_env.sh && "$MJ311_PYTHON" uuv_mujoco/current/run_uuv_mujoco.py --help
bash -n run_control_gui.sh run_control_gui_ubuntu.sh uuv_mujoco/current/run_control_gui.sh
```

Results:

- `gui/helpers.py` dropped from the hotspot list and is now a small facade.
- The new focused helper modules compile with the existing GUI importers.
- A fake-runtime import check preserved the legacy helper export surface and RC
  channel mapping: `axis_025=1600`, `channels_1_6=[1500, 1500, 1600, 1500,
  1400, 1700]`.
- Direct GUI runtime import still requires a sourced ROS2/rclpy environment, as
  it did before this split.
