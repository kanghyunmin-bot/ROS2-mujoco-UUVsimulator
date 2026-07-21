# SITL Replay Split

Date: 2026-06-07

Scope: active runtime transport replay code under `uuv_mujoco/current/bridge`.

## What changed

- Split replay dataclasses into `bridge/sitl_replay_types.py`.
- Split CSV float parsing, quaternion normalization, pressure-from-depth math,
  and optional logging into `bridge/sitl_replay_common.py`.
- Split sensor replay preview and native VPD CSV loading into
  `bridge/sitl_replay_loaders.py`.
- Split sensor replay interpolation into `bridge/sitl_replay_interpolation.py`.
- Reduced `bridge/sitl_replay.py` to compatibility exports used by
  `bridge/sitl_initialization.py`.

## Contract boundaries preserved

- Sensor replay timestamps, frame sorting, Bar30 pressure reconstruction, and
  interpolation semantics are unchanged.
- Native VPD event `t_replay_s = t_real_s - real_start_s` is unchanged.
- No controller-parity observation layer or plant-input semantics changed.

## Verification

```text
python3 -m py_compile \
  uuv_mujoco/current/bridge/sitl_replay.py \
  uuv_mujoco/current/bridge/sitl_replay_types.py \
  uuv_mujoco/current/bridge/sitl_replay_common.py \
  uuv_mujoco/current/bridge/sitl_replay_loaders.py \
  uuv_mujoco/current/bridge/sitl_replay_interpolation.py \
  uuv_mujoco/current/bridge/sitl_initialization.py
PYTHONPATH=uuv_mujoco/current python3 - <<'PY'
import numpy as np
from bridge.sitl_replay import SensorReplayFrame, interpolate_sensor_replay_frame, pressure_abs_from_depth_m
required = ['SensorReplayFrame', 'NativeVisionDeltaEvent', 'load_sensor_replay_preview', 'load_native_vision_delta_events', 'interpolate_sensor_replay_frame']
import bridge.sitl_replay as m
print({name: hasattr(m, name) for name in required})
a = SensorReplayFrame(0.0, np.zeros(3), np.zeros(3), np.array([1.0, 0.0, 0.0, 0.0]), 0.0, 101325.0, np.zeros(3), np.zeros(3), 0.0, None)
b = SensorReplayFrame(1.0, np.ones(3), np.ones(3), np.array([1.0, 0.0, 0.0, 0.0]), 2.0, 120000.0, np.array([0.0, 0.0, 2.0]), np.array([1.0, 2.0, 3.0]), -2.0, None)
frame, idx = interpolate_sensor_replay_frame([a, b], t_s=0.5, start_index=0, surface_pressure_pa=101325.0, water_density=997.0, gravity=9.80665)
print('depth', frame.depth_m, 'idx', idx, 'pressure', round(float(frame.pressure_pa), 3), 'expected', round(pressure_abs_from_depth_m(1.0, 101325.0, 997.0, 9.80665), 3))
if abs(frame.depth_m - 1.0) > 1e-9 or idx != 0:
    raise SystemExit(1)
PY
python3 -m compileall -q uuv_mujoco/current uuv_control_gui.py
PYTHONPATH=uuv_mujoco/current/tools python3 \
  uuv_mujoco/current/tools/audit_code_contract_sources.py \
  --out-dir /private/tmp/uuv_code_contract_after_sitl_replay_split
python3 uuv_mujoco/current/tools/check_runtime_readiness_policy.py
python3 uuv_mujoco/current/tools/verify_ardusub_thruster_contract.py --quiet
python3 uuv_mujoco/current/tools/check_dev_os_compat.py --headless --target-os ubuntu
git diff --check
python3 uuv_mujoco/current/tools/refactor_inventory.py --limit 20
```

Results:

- Compile/import: pass.
- Interpolation smoke: midpoint depth `1.0`, pressure `111102.23 Pa`.
- Source contract audit: `fail=0`, `pass=11`, `warn=5`.
- Runtime readiness policy: pass.
- ArduSub thruster contract: pass.
- Dev OS compatibility: `fail=0`, `pass=16`, `warn=2` (`docker_daemon`,
  `ros2_env`).
- `git diff --check`: pass.
- `bridge/sitl_replay.py` removed from the top hotspot list.
