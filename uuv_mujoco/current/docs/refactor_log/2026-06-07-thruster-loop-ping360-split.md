# Thruster, Viewer Loop, And Ping360 Publish Split

Date: 2026-06-07

## Scope

- Split offline T200 performance loading:
  - `physics/thruster_performance_payload.py` owns JSON IO and file/payload
    error messages.
  - `physics/thruster_performance_curves.py` owns float-array parsing,
    finite sorted curves, usable-candidate filtering, and nearest-voltage
    selection.
  - `physics/thruster_performance.py` keeps the public
    `ThrusterPerformance` API and `load_thruster_performance()` return shape.
- Split direct thruster gain overrides:
  - `sim/physics/thruster_direct_gain_values.py` owns value parsing/clamping.
  - `sim/physics/thruster_direct_profile.py` owns profile direct-gain values.
  - `sim/physics/thruster_direct_env.py` owns group and per-thruster env
    overrides.
  - `sim/physics/thruster_direct_overrides.py` keeps the public in-place
    `apply_thruster_direct_gain_overrides()` entrypoint.
- Split passive viewer loop timing:
  - `sim/runtime/simulation_loop_cadence.py` owns target/sensor/viewer cadence.
  - `sim/runtime/simulation_loop_clocks.py` owns clock state, catch-up loops,
    paused-step handling, sensor publish catch-up, and viewer-frame sleeping.
  - `sim/runtime/simulation_loop_runtime.py` keeps the public
    `ViewerRuntimeLoop` class.
- Split Ping360 publish builders:
  - `bridge/ros2_publish_ping360_cache_state.py` owns per-cycle cache state.
  - `bridge/ros2_publish_ping360_sample_cache.py` owns sample caching.
  - `bridge/ros2_publish_ping360_message_cache.py` owns image/scan/echo lazy
    messages.
  - `bridge/ros2_publish_ping360_status_cache.py` owns status message cadence.
  - `bridge/ros2_publish_builder_ping360.py` keeps the public builder map.

## Behavior Contract

- No plant physics coefficients changed.
- No RC override, RCOU telemetry, MAVLink command, SITL JSON servo, or Bar30
  pressure contract changed.
- Direct thruster gain precedence remains: profile values first, group env
  overrides next, per-thruster env overrides last.
- Offline T200 performance loading still returns `(ThrusterPerformance,
  message)` and selects the nearest voltage curve.
- Passive viewer loop still uses bounded viewer FPS, wall-clock catch-up,
  ROS sensor publish catch-up, overlay update, `viewer.sync()`, and frame sleep.
- Ping360 sample generation remains lazy and per publish cycle; status still
  uses an existing sample only if another Ping360 builder already generated it.

## Validation

```bash
PYTHONPATH=/Users/kanghyunmin/Desktop/uuv_sim/uuv_mujoco/v2.2 python3 - <<'PY'
# offline thruster performance, direct override, simulation loop helper,
# and Ping360 lazy builder smoke snippets
PY
python3 -m compileall -q uuv_mujoco/current uuv_control_gui.py
git diff --check
python3 uuv_mujoco/current/tools/check_rc_frame_contract.py
python3 uuv_mujoco/current/tools/check_gui_readiness_contract.py
python3 uuv_mujoco/current/tools/check_gui_backend_selection.py
python3 uuv_mujoco/current/tools/check_runtime_readiness_policy.py
PYTHONPATH=/Users/kanghyunmin/Desktop/uuv_sim/uuv_mujoco/current/tools \
  python3 uuv_mujoco/current/tools/audit_code_contract_sources.py \
  --out-dir /private/tmp/uuv_source_audit_spaghetti_20260607_thruster_loop_ping360_split
python3 uuv_mujoco/current/tools/verify_ardusub_thruster_contract.py --quiet
/Users/kanghyunmin/.venvs/mujoco311/bin/python \
  uuv_mujoco/current/tools/physics_contract_audit.py \
  --output-dir /private/tmp/uuv_physics_audit_spaghetti_20260607_thruster_loop_ping360_split \
  --simulate-s 0.05
python3 uuv_mujoco/current/tools/refactor_inventory.py \
  --root uuv_mujoco/current --format markdown --limit 30
```

Results:

- `offline_thruster_performance=PASS`
- `thruster_direct_overrides=PASS`
- `simulation_loop_helpers=PASS`
- `ping360_publish_builder=PASS`
- `rc_frame_contract=PASS`
- `gui_readiness_contract=PASS`
- `gui_backend_selection=PASS`
- `runtime_readiness_policy=PASS`
- source audit: `fail=0`, `pass=11`, `warn=5`
- thruster contract: `OK`
- physics audit neutral balance: `net_down=+0.000N`,
  `required_scale=1.000000`, neutral open-plant drift `+0.00001m`
- The refactor inventory no longer lists the four previous hotspots in the top
  30: `physics/thruster_performance.py`,
  `sim/physics/thruster_direct_overrides.py`,
  `sim/runtime/simulation_loop_runtime.py`, and
  `bridge/ros2_publish_builder_ping360.py`.

Notes:

- An initial offline thruster smoke used a wrong file name
  `config/t200_thruster_performance.json`; the actual runtime config is
  `config/thruster_performance.json`.
- An initial Ping360 smoke exposed an unnecessary runtime-only
  `RosPublishState` import path; the Ping360 builder now avoids that import.
