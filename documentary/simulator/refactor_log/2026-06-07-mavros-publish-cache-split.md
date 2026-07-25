# MAVROS Publish Cache Split

Date: 2026-06-07

Scope: `sim/current/bridge`

## Change

`bridge/ros2_publish_mavros_cache.py` now owns lazy-cache mechanics only.
`bridge/ros2_publish_mavros_cache_factories.py` owns the ordered topic-builder
registry for MAVROS-compatible messages.

The cache no longer carries one method per message family.  Topic names,
builder order, and lazy object reuse are driven by `MAVROS_CACHE_FACTORIES`.

## Contract

This is a behavior-preserving refactor.  It does not change MAVROS topic names,
message builders, frame IDs, pressure semantics, battery defaults, or publish
scheduling.  The controller-parity observation layer remains
`SERVO_OUTPUT_RAW` telemetry, not JSON servo output.

## Verification

```text
/Users/kanghyunmin/.venvs/mujoco311/bin/python <focused MAVROS publish cache smoke>
python3 -m compileall -q sim/current uuv_control_gui.py
python3 sim/current/tools/audit_code_contract_sources.py
python3 sim/current/tools/check_runtime_readiness_policy.py
python3 sim/current/tools/verify_ardusub_thruster_contract.py --quiet
/Users/kanghyunmin/.venvs/mujoco311/bin/python sim/current/tools/physics_contract_audit.py --simulate-s 0.05
git diff --check
```

Result: cache smoke `PASS`; source audit `fail=0 pass=11 warn=5`; readiness
`PASS`; thruster contract `OK`; physics contract audit completed; compileall and
diff check `PASS`.
