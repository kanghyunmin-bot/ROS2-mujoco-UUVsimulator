# SITL Command Target Split

Date: 2026-06-07

## Change

- Added `bridge/sitl_command_targets.py`.
- Moved MAVLink command target selection, command/servo link selection, vehicle
  HEARTBEAT state extraction, COMMAND_ACK logging, and UDP peer discovery out
  of `bridge/sitl_commanding.py`.
- Kept `bridge/sitl_commanding.py` as the command policy surface for arm/mode,
  RC override, MANUAL_CONTROL, and local setpoint senders.
- Preserved the existing `SitlTransport` monkey-bound public names, including
  the `mavlink_connected` and `rc_override_ready` properties.

## Metrics

Before:

- `bridge/sitl_commanding.py`: `724 LOC`, `172` branch nodes.

After:

- `bridge/sitl_commanding.py`: `533 LOC`, `100` branch nodes.
- `bridge/sitl_command_targets.py`: isolated command-target/heartbeat helper
  ownership.

## Validation

```bash
env PYTHONPYCACHEPREFIX=/private/tmp/pycache python3 -m compileall -q uuv_mujoco/v2.2
env PYTHONPYCACHEPREFIX=/private/tmp/pycache PYTHONPATH=uuv_mujoco/v2.2 /Users/kanghyunmin/miniconda3/envs/ros2_h311/bin/python3.11 -c "from bridge.sitl_transport import SitlTransport; print(SitlTransport.__name__, type(SitlTransport.mavlink_connected).__name__, type(SitlTransport.rc_override_ready).__name__)"
env PYTHONPYCACHEPREFIX=/private/tmp/pycache python3 uuv_mujoco/v2.2/tools/check_runtime_readiness_policy.py
env PYTHONPYCACHEPREFIX=/private/tmp/pycache python3 uuv_mujoco/v2.2/tools/verify_ardusub_thruster_contract.py --quiet
env PYTHONPYCACHEPREFIX=/private/tmp/pycache python3 uuv_mujoco/v2.2/tools/audit_code_contract_sources.py --out-dir /private/tmp/uuv_code_contract_after_sitl_target_split
git diff --check -- uuv_mujoco/v2.2/bridge/sitl_commanding.py uuv_mujoco/v2.2/bridge/sitl_command_targets.py
```

Results:

- compileall: pass
- import smoke: `SitlTransport property property`
- readiness policy: pass
- thruster contract: pass
- contract source audit: `fail=0`, `pass=10`, `warn=5`
- diff check: pass
