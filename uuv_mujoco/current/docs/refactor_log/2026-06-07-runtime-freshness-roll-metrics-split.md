# Runtime Freshness And Roll Metrics Split

Date: 2026-06-07

Scope: active runtime under `uuv_mujoco/current`, backed by
`uuv_mujoco/v2.2`.

## Changes

- Added the active-runtime freshness preflight to
  `start_docker_sitl_mujoco_mj311.sh`.  GUI Docker Start can resolve to the
  backing `v2.2` script because Python resolves the `current` symlink, so the
  backing script now checks `uuv_mujoco/current`, fetches `origin`, compares
  `HEAD` with `origin/uuv_sim`, and refreshes `RUNTIME_VERSION.json` before
  starting.
- Kept `v2.2` as a compatibility backing directory only.  The live runtime
  contract remains `uuv_mujoco/current -> v2.2`.
- Split `tools/roll_stability_metrics.py` into focused modules:
  - `tools/roll_stability_math.py`
  - `tools/roll_stability_pose_metrics.py`
  - `tools/roll_stability_rc_metrics.py`
  - `tools/roll_stability_score.py`
- Preserved the public `compute_metrics()`, `quat_to_rpy_deg()`, `rms()`,
  `stddev()`, and `unwrap_rad()` import surface.
- Split `sim/runtime/real_start_payload_status.py` into focused modules:
  - `sim/runtime/real_start_status_eval.py`
  - `sim/runtime/real_start_payload_values.py`
  - `sim/runtime/real_start_payload_builders.py`
- Preserved the public real-start status import surface used by
  `sim/runtime/real_start_payload.py`.

## Verification

```bash
bash -n uuv_mujoco/current/start_docker_sitl_mujoco_mj311.sh
bash -n uuv_mujoco/current/start_sitl_mujoco_mj311.sh
bash -n uuv_mujoco/run_mujoco.sh
bash -n uuv_mujoco/start_sitl_mujoco.sh
bash -n uuv_mujoco/start_docker_sitl_mujoco.sh

PYTHONPATH=uuv_mujoco/current/tools python3 \
  uuv_mujoco/current/tools/check_runtime_freshness.py \
  --workspace /Users/kanghyunmin/Desktop/uuv_sim \
  --runtime-dir /Users/kanghyunmin/Desktop/uuv_sim/uuv_mujoco/current \
  --fetch --refresh-version

python3 -m compileall -q \
  uuv_mujoco/current/tools/roll_stability_metrics.py \
  uuv_mujoco/current/tools/roll_stability_math.py \
  uuv_mujoco/current/tools/roll_stability_pose_metrics.py \
  uuv_mujoco/current/tools/roll_stability_rc_metrics.py \
  uuv_mujoco/current/tools/roll_stability_score.py \
  uuv_mujoco/current/tools/roll_stability_probe_callbacks.py \
  uuv_mujoco/current/tools/roll_stability_probe_sequence.py

PYTHONPATH=uuv_mujoco/current/tools python3 - <<'PY'
import json, math
from roll_stability_metrics import compute_metrics, quat_to_rpy_deg

samples = []
for i in range(24):
    t = i * 0.1
    samples.append({
        "t": t,
        "roll_deg": math.sin(t) * 3.0,
        "pitch_deg": math.cos(t * 0.7) * 1.5,
        "yaw_deg": 178.0 + i * 4.0,
        "z_m": -0.25 - 0.001 * i,
        "gyro_x": 0.02 * math.sin(t),
        "gyro_y": 0.01 * math.cos(t),
        "gyro_z": 0.03 * math.sin(t * 0.5),
    })
depth_samples = [(i * 0.2, 0.3 + 0.002 * i) for i in range(18)]
rc_samples = [
    (0.0, [1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500]),
    (0.1, [1510, 1490, 1505, 1495, 1520, 1480, 1515, 1485]),
    (0.2, [2101, 1500, 1500, 1500, 1510, 1510, 1490, 1490]),
    (0.3, [1500, 1500, 1500, 1500, 900, 2100, 1500, 1500]),
]
metrics = compute_metrics(samples, depth_samples, rc_samples, 8.0, 0.8)
assert metrics["rc_samples"] == 4
assert metrics["rc_valid_samples"] == 3
assert math.isfinite(metrics["score"])
assert quat_to_rpy_deg(1.0, 0.0, 0.0, 0.0) == (0.0, 0.0, 0.0)
print(json.dumps({"status": "pass", "metric_keys": len(metrics)}))
PY

python3 -m compileall -q \
  uuv_mujoco/current/sim/runtime/real_start_payload_status.py \
  uuv_mujoco/current/sim/runtime/real_start_payload_values.py \
  uuv_mujoco/current/sim/runtime/real_start_status_eval.py \
  uuv_mujoco/current/sim/runtime/real_start_payload_builders.py \
  uuv_mujoco/current/sim/runtime/real_start_payload.py \
  uuv_mujoco/current/sim/runtime/real_start_runtime.py

PYTHONPATH=uuv_mujoco/current python3 - <<'PY'
import json
import numpy as np
from sim.runtime.real_start_payload_status import (
    build_latched_payload,
    build_required_payload,
    determine_real_start_status,
    real_start_not_required_payload,
)
from sim.runtime.real_start_types import RealStartMeasurements, RealStartTargets

targets = RealStartTargets(
    target_depth=0.6,
    depth_contract="bar30",
    target_rpy=(0.0, 0.0, 1.0),
    target_x=0.1,
    target_y=-0.2,
    target_pressure_pa=107000.0,
    target_v=np.array([0.0, 0.0, 0.0]),
    target_w=np.array([0.0, 0.0, 0.0]),
    source_t_s=69.35,
    pressure_tol_pa=25.0,
    xy_tol_m=0.02,
)
measurements = RealStartMeasurements(
    depth_now=0.601,
    depth_error=0.001,
    base_xy_now=np.array([0.1, -0.2]),
    xy_error=0.0,
    pressure_now_pa=107010.0,
    pressure_error_pa=10.0,
    attitude_error=0.01,
    velocity_error=0.01,
    angular_velocity_error=0.01,
)
ok, status = determine_real_start_status(
    targets=targets,
    measurements=measurements,
    depth_tolerance_m=0.02,
    attitude_tolerance_rad=0.02,
    velocity_tolerance_mps=0.02,
)
assert (ok, status) == (True, "ok")
payload = build_required_payload(
    targets=targets,
    measurements=measurements,
    ok=ok,
    status=status,
    hold_active=True,
    base_depth_m=0.65,
    bar30_depth_m=0.601,
    depth_tolerance_m=0.02,
    attitude_tolerance_rad=0.02,
    velocity_tolerance_mps=0.02,
)
assert payload["required"] is True
assert payload["released"] is False
latched = build_latched_payload(latched_payload=payload, hold_active=False, measurements=measurements)
assert latched["released"] is True
not_required, carried = real_start_not_required_payload(hold_active=False, latched_payload=payload)
assert not_required["status"] == "not_required"
assert carried is payload
print(json.dumps({"status": "pass", "payload_keys": len(payload)}))
PY

python3 uuv_mujoco/current/tools/refactor_inventory.py \
  --root uuv_mujoco/current --limit 25
```

## Notes

- `tools/roll_stability_metrics.py` was not present in `HEAD`, so there was no
  tracked baseline available for `git show HEAD:...` equivalence testing.  The
  current import/API smoke validates the active surface instead.
- This pass does not change plant physics, thruster coefficients, RC mapping, or
  SITL sensor contracts.
