# Vehicle Heartbeat And ExternalNav Contract Split

Date: 2026-06-07

Scope:

- `bridge/sitl_vehicle_heartbeat.py`
- `bridge/sitl_external_nav_cache_contract.py`

What changed:

- Split vehicle HEARTBEAT filtering into
  `bridge/sitl_vehicle_heartbeat_filter.py`.
- Split vehicle armed/mode state recording into
  `bridge/sitl_vehicle_state.py`.
- Split COMMAND_ACK logging and accepted-command constants into
  `bridge/sitl_vehicle_command_ack.py`.
- Split ExternalNav wall-clock cache behavior into
  `bridge/sitl_external_nav_cache.py`.
- Split ExternalNav required-output and stale-rate checks into
  `bridge/sitl_external_nav_contract.py`.
- Kept compatibility facades in place so command bindings still import through
  the existing modules.

Contract notes:

- COMMAND_ACK still does not complete arm/mode pending state.  Vehicle state is
  still authoritative only after HEARTBEAT reports armed flag and mode.
- The dedicated command-link and servo-link heartbeat timestamps are still
  updated on their respective links.
- Live wall-clock ExternalNav cache still disables itself when replay frames or
  native VPD events are present.
- Required ExternalNav still fails on disabled output, fault state, never-sent
  VISION_POSITION_DELTA, and stale TX beyond the configured max stale time.
- ArduPilot source and submodule pointer were not modified.

Focused verification:

```text
python3 -m compileall -q \
  uuv_mujoco/current/bridge/sitl_vehicle_heartbeat.py \
  uuv_mujoco/current/bridge/sitl_vehicle_heartbeat_filter.py \
  uuv_mujoco/current/bridge/sitl_vehicle_state.py \
  uuv_mujoco/current/bridge/sitl_vehicle_command_ack.py \
  uuv_mujoco/current/bridge/sitl_command_targets.py \
  uuv_mujoco/current/bridge/sitl_commanding.py

sitl_vehicle_heartbeat_split_smoke PASS

python3 -m compileall -q \
  uuv_mujoco/current/bridge/sitl_external_nav_cache_contract.py \
  uuv_mujoco/current/bridge/sitl_external_nav_cache.py \
  uuv_mujoco/current/bridge/sitl_external_nav_contract.py \
  uuv_mujoco/current/bridge/sitl_external_nav_runtime.py

sitl_external_nav_cache_contract_split_smoke PASS
```

Full verification:

```text
python3 -m compileall -q uuv_mujoco/current uuv_control_gui.py
audit_code_contract_sources.py -> {"fail": 0, "pass": 11, "warn": 5}
check_runtime_readiness_policy.py -> runtime_readiness_policy=PASS
verify_ardusub_thruster_contract.py --quiet -> OK
physics_contract_audit.py --simulate-s 0.05 -> required_scale=1.000000
```

Inventory result:

- `bridge/sitl_vehicle_heartbeat.py` is no longer in the top hotspot list.
- `bridge/sitl_external_nav_cache_contract.py` is no longer in the top hotspot
  list.
