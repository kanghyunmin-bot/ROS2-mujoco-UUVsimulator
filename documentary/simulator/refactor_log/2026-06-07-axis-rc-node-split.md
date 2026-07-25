# Axis RC Node Split

Date: 2026-06-07

Scope: active validation tooling under `sim/current/tools`.

## Changes

- Split `tools/axis_rc_node.py` into focused modules:
  - `tools/axis_rc_messages.py`
  - `tools/axis_rc_sampling.py`
  - `tools/axis_rc_services.py`
- Kept `AxisRcOverrideCheck` as the public ROS2 node class used by
  `tools/axis_rc_override_check.py`.
- Preserved the public methods:
  `publish_rc`, `publish_manual`, `release_rc`, `publish_neutral_control`,
  `spin_with_rc`, `wait_for_stack`, `call_set_mode`, `call_arm`,
  `call_trigger_service`, `switch_initial_depth_hold_to_target`, and
  `release_initial_depth_hold`.

## Contract Notes

- RC override generation still uses `axis_rc_contract.AXIS_TO_CHANNEL`,
  `RC_NEUTRAL`, and `RC_SPAN`.
- RC override remains the axis-check command surface; this refactor does not
  change controller parity observation points or plant input routing.
- `manual-control` mode still releases RC override before publishing neutral
  manual control.
- Service wait loops still publish neutral control while waiting for stack,
  arming, mode, and MuJoCo initial-depth trigger services.
- No ArduPilot source, submodule pointer, PWM remap, output shim, or ALT_HOLD
  controller contract changed.

## Verification

```bash
python3 -m py_compile \
  sim/current/tools/axis_rc_node.py \
  sim/current/tools/axis_rc_messages.py \
  sim/current/tools/axis_rc_sampling.py \
  sim/current/tools/axis_rc_services.py \
  sim/current/tools/axis_rc_override_check.py
python3 sim/current/tools/axis_rc_override_check.py --help
python3 sim/current/tools/refactor_inventory.py --root sim/current --limit 30
```

Results:

- `tools/axis_rc_node.py` dropped from `330 LOC / 63` branches to
  `194 LOC / 13` branches.
- The new focused modules are:
  - `axis_rc_messages.py`: `60 LOC / 9` branches
  - `axis_rc_sampling.py`: `97 LOC / 9` branches
  - `axis_rc_services.py`: `112 LOC / 35` branches
- `axis_rc_override_check.py --help` remains runnable without importing ROS2
  runtime node construction.
- `tools/axis_rc_node.py` no longer appears in the top 30 hotspot inventory.
