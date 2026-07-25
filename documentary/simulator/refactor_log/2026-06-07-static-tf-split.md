# Static TF Split

Date: 2026-06-07

Scope: `sim/current/bridge`

## Change

The ROS2 static TF builder was split without changing frame names, fallback
positions, or message output shape.

- `ros2_tf_messages.py` remains the compatibility surface for
  `build_tf_message`, `build_static_tf_specs`, and quaternion helper imports.
- `ros2_tf_geometry.py` owns quaternion and MuJoCo site local-pose helpers.
- `ros2_static_tf_core.py` owns fixed world/body alias frames.
- `ros2_static_tf_sensors.py` owns IMU, Bar30/depth, DVL, and Ping360 frames.
- `ros2_static_tf_cameras.py` owns stereo camera and optical frames.
- `ros2_static_tf_specs.py` is now the assembly facade.

## Contract

This is a behavior-neutral refactor.  It does not change ROS topic names,
frame IDs, ArduPilot, SITL command/telemetry behavior, plant inputs, or
physics coefficients.

## Verification

```text
PYTHONPATH=/Users/kanghyunmin/Desktop/uuv_sim/sim/current python3 <static TF fake-model smoke>
python3 -m compileall -q /Users/kanghyunmin/Desktop/uuv_sim/sim/current /Users/kanghyunmin/Desktop/uuv_sim/uuv_control_gui.py
PYTHONPATH=/Users/kanghyunmin/Desktop/uuv_sim/sim/current/tools python3 /Users/kanghyunmin/Desktop/uuv_sim/sim/current/tools/audit_code_contract_sources.py --out-dir /private/tmp/uuv_source_audit_spaghetti_20260607_static_tf_split
python3 /Users/kanghyunmin/Desktop/uuv_sim/sim/current/tools/check_runtime_readiness_policy.py
python3 /Users/kanghyunmin/Desktop/uuv_sim/sim/current/tools/verify_ardusub_thruster_contract.py --quiet
/Users/kanghyunmin/miniconda3/envs/ros2_h311/bin/python /Users/kanghyunmin/Desktop/uuv_sim/sim/current/tools/physics_contract_audit.py --output-dir /private/tmp/uuv_physics_audit_spaghetti_20260607_static_tf_split --simulate-s 0.05
git -C /Users/kanghyunmin/Desktop/uuv_sim diff --check
python3 /Users/kanghyunmin/Desktop/uuv_sim/sim/current/tools/check_runtime_freshness.py --fetch --refresh-version --warn-only
```

Result: static TF smoke `PASS`, source audit `fail=0 pass=11 warn=5`,
readiness `PASS`, thruster contract `OK`, physics contract audit completed,
and runtime freshness recorded `current-dirty` with `dirty_paths=619` and
`active_runtime_dirty_paths=601`.
