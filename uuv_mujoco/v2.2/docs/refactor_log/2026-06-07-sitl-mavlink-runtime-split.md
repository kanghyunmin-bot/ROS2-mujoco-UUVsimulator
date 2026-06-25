# SITL MAVLink Runtime Split

Date: 2026-06-07

## Change

- Added `bridge/sitl_mavlink_requests.py` for MAVLink telemetry stream request
  policies.
- Added `bridge/sitl_pwm_runtime.py` for PWM frame acceptance, disarmed/all-min
  neutralization, replay ownership, and plant input callback forwarding.
- Kept `bridge/sitl_mavlink_runtime.py` as the connection setup and poll-loop
  owner.
- Preserved the existing `SitlTransport` method bindings through aliases in
  `bridge/sitl_mavlink_runtime.py`.

## Metrics

Before:

- `bridge/sitl_mavlink_runtime.py`: `445 LOC`, `103` branch nodes.
- Largest function: `_poll_servo_mavlink()`, `109 LOC`.

After:

- `bridge/sitl_mavlink_runtime.py`: `272 LOC`, `60` branch nodes.
- `bridge/sitl_mavlink_requests.py`: `109 LOC`, `21` branch nodes.
- `bridge/sitl_pwm_runtime.py`: `87 LOC`, `22` branch nodes.
- Largest function remains `_poll_servo_mavlink()`, `109 LOC`.

## Validation

```bash
env PYTHONPYCACHEPREFIX=/private/tmp/pycache python3 -m compileall -q uuv_mujoco/v2.2
env PYTHONPYCACHEPREFIX=/private/tmp/pycache PYTHONPATH=uuv_mujoco/v2.2 /Users/kanghyunmin/miniconda3/envs/ros2_h311/bin/python3.11 -c "from bridge.sitl_transport import SitlTransport; print(SitlTransport.__name__, callable(SitlTransport._handle_pwm_values), callable(SitlTransport._request_sitl_mavlink_servo_stream))"
env PYTHONPYCACHEPREFIX=/private/tmp/pycache python3 uuv_mujoco/v2.2/tools/check_runtime_readiness_policy.py
env PYTHONPYCACHEPREFIX=/private/tmp/pycache python3 uuv_mujoco/v2.2/tools/verify_ardusub_thruster_contract.py --quiet
env PYTHONPYCACHEPREFIX=/private/tmp/pycache python3 uuv_mujoco/v2.2/tools/audit_code_contract_sources.py --out-dir /private/tmp/uuv_code_contract_after_sitl_mavlink_split
env PYTHONPYCACHEPREFIX=/private/tmp/pycache python3 uuv_mujoco/v2.2/tools/check_dev_os_compat.py --headless --target-os ubuntu
git diff --check -- uuv_mujoco/v2.2/bridge/sitl_mavlink_runtime.py uuv_mujoco/v2.2/bridge/sitl_mavlink_requests.py uuv_mujoco/v2.2/bridge/sitl_pwm_runtime.py
```

Results:

- compileall: pass
- import smoke: `SitlTransport True True`
- readiness policy: pass
- thruster contract: pass
- contract source audit: `fail=0`, `pass=10`, `warn=5`
- dev OS compatibility: `fail=0`, `pass=16`, `warn=2`
- diff check: pass
