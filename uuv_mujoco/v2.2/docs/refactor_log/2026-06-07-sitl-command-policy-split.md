# SITL Command Policy Split

Date: 2026-06-07

## Problem

`bridge/sitl_commanding.py` mixed four concerns:

- auto-ready neutral RC, arm, and mode sequencing;
- arm/disarm and mode retry state machines;
- RC override and neutral keepalive forwarding;
- MANUAL_CONTROL and raw local setpoint forwarding.

That made command latency, readiness, and plant-input debugging harder because
every arm/mode/RC change had to be inspected in one large policy file.

## Change

- `bridge/sitl_auto_ready_runtime.py` owns auto-ready state and neutral RC
  sequencing.
- `bridge/sitl_arm_mode_runtime.py` owns arm/disarm, mode resolution, command
  queuing, and retry servicing.
- `bridge/sitl_rc_manual_runtime.py` owns RC override, neutral keepalive,
  MANUAL_CONTROL, body velocity setpoints, and raw local NED setpoints.
- `bridge/sitl_commanding.py` now only re-exports the stable compatibility
  surface consumed by `SitlTransport`.

## Size Check

```text
bridge/sitl_commanding.py: LOC=85 branches=0 funcs=0
bridge/sitl_auto_ready_runtime.py: LOC=98 branches=28 funcs=5
bridge/sitl_arm_mode_runtime.py: LOC=197 branches=41 funcs=9
bridge/sitl_rc_manual_runtime.py: LOC=291 branches=32 funcs=8
bridge/sitl_command_targets.py: LOC=223 branches=78 funcs=12
```

## Validation

```text
python3 -m py_compile \
  uuv_mujoco/current/bridge/sitl_commanding.py \
  uuv_mujoco/current/bridge/sitl_auto_ready_runtime.py \
  uuv_mujoco/current/bridge/sitl_arm_mode_runtime.py \
  uuv_mujoco/current/bridge/sitl_rc_manual_runtime.py \
  uuv_mujoco/current/bridge/sitl_transport.py
```

```text
PYTHONPATH=uuv_mujoco/current "$MJ311_PYTHON" - <<'PY'
from bridge.sitl_transport import SitlTransport
from bridge import sitl_commanding
# verified all SitlTransport command aliases still exist
PY
```

```text
python3 -m compileall -q uuv_mujoco/current
python3 uuv_mujoco/current/tools/check_runtime_readiness_policy.py
python3 uuv_mujoco/current/tools/verify_ardusub_thruster_contract.py --quiet
python3 uuv_mujoco/current/tools/audit_code_contract_sources.py \
  --out-dir /private/tmp/uuv_code_contract_after_sitl_command_split
```

Results:

- `runtime_readiness_policy=PASS`
- `[thruster-contract] OK`
- `audit_code_contract_sources`: `fail=0`, `pass=10`, `warn=5`
