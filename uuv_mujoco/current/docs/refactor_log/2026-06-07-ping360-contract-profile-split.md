# Ping360 Contract And Profile Split

Date: 2026-06-07

Scope: active runtime only, under `uuv_mujoco/current`.

## Why

`bridge/ping360_types.py` combined protocol constants, JSON config loading,
effective setting serialization, and sample/status payload contracts.  In the
same bridge area, `bridge/ping360_profile.py` combined MuJoCo beam-hit
extraction with return-strength, pulse, noise, and blind-zone shaping.

Those responsibilities are different contracts:

- static Ping360 protocol constants,
- user/runtime config loading,
- effective firmware-style settings,
- per-ping sample/status data,
- MuJoCo raycast hit extraction,
- sonar profile signal shaping.

Keeping them in one or two files made the Ping360 sonar contract harder to
audit without touching the simulator physics.

## Changed

- Added `bridge/ping360_constants.py`.
- Added `bridge/ping360_config.py`.
- Added `bridge/ping360_effective_settings.py`.
- Added `bridge/ping360_sample.py`.
- Kept `bridge/ping360_types.py` as the compatibility facade.
- Added `bridge/ping360_profile_hits.py`.
- Added `bridge/ping360_profile_signal.py`.
- Kept `bridge/ping360_profile.py` as the public scan/compatibility facade.

## Validation

Focused smoke:

```text
ping360_types_split_smoke PASS
ping360_profile_split_smoke PASS
```

Full gates:

```text
python3 -m compileall -q /Users/kanghyunmin/Desktop/uuv_sim/uuv_mujoco/current /Users/kanghyunmin/Desktop/uuv_sim/uuv_control_gui.py
PYTHONPATH=/Users/kanghyunmin/Desktop/uuv_sim/uuv_mujoco/current/tools python3 /Users/kanghyunmin/Desktop/uuv_sim/uuv_mujoco/current/tools/audit_code_contract_sources.py --out-dir /private/tmp/uuv_source_audit_spaghetti_20260607_ping360_contract_split
python3 /Users/kanghyunmin/Desktop/uuv_sim/uuv_mujoco/current/tools/check_runtime_readiness_policy.py && python3 /Users/kanghyunmin/Desktop/uuv_sim/uuv_mujoco/current/tools/verify_ardusub_thruster_contract.py --quiet
/Users/kanghyunmin/miniconda3/envs/ros2_h311/bin/python /Users/kanghyunmin/Desktop/uuv_sim/uuv_mujoco/current/tools/physics_contract_audit.py --output-dir /private/tmp/uuv_physics_audit_spaghetti_20260607_ping360_contract_split --simulate-s 0.05
```

Results:

```text
compileall PASS
source contract audit: {"fail": 0, "pass": 11, "warn": 5}
runtime_readiness_policy=PASS
[thruster-contract] OK
physics contract audit PASS, static force balance required_scale=1.000000
git diff --check PASS
```

## Contract Notes

This pass does not change Ping360 ROS topics, JSON config semantics, raycast
cutoff logic, reflectivity values, noise model, sample period constants,
sensor-frame transforms, actuator contracts, RC override timing, or MuJoCo
physics parameters.
