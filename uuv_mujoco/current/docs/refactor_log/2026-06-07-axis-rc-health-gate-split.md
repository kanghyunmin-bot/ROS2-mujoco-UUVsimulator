# Axis RC Health Gate Split

## Scope

Split RC override health-gate logic without changing axis command validation
thresholds, report schema, or the public `build_health()` import surface.

## Files

- `tools/axis_rc_health_metrics.py`: orchestration and public `build_health()`.
- `tools/axis_rc_health_phase.py`: per-phase sample count, armed fraction,
  RCOut movement, primary-axis response, and neutral residual checks.
- `tools/axis_rc_health_sign.py`: positive/negative command sign-pair checks.
- `tools/axis_rc_health_status.py`: overall fail/warn/pass aggregation.

## Contract

- RC override health report shape remains `{"overall": ..., "checks": ...}`.
- Existing flag names are unchanged:
  - `too_few_samples`
  - `not_fully_armed`
  - `rcout_not_moving`
  - `weak_primary_axis_response`
  - `high_neutral_yaw_residual`
  - `high_neutral_heave_residual`
  - `positive_negative_response_same_sign`
- No controller output, plant input, or telemetry comparison code changed.

## Verification

```bash
python3 -m compileall -q \
  uuv_mujoco/v2.2/tools/axis_rc_health_metrics.py \
  uuv_mujoco/v2.2/tools/axis_rc_health_phase.py \
  uuv_mujoco/v2.2/tools/axis_rc_health_sign.py \
  uuv_mujoco/v2.2/tools/axis_rc_health_status.py

PYTHONPATH=uuv_mujoco/v2.2/tools python3 - <<'PY'
from axis_rc_health_metrics import build_health
rows = [
    {'phase':'yaw_pos','axis':'yaw','samples':20,'armed_fraction':1.0,
     'rcout_max_delta':30,'expected_metric_peak_abs':0.4,
     'expected_metric_mean':0.2},
    {'phase':'yaw_neg','axis':'yaw','samples':20,'armed_fraction':1.0,
     'rcout_max_delta':30,'expected_metric_peak_abs':0.4,
     'expected_metric_mean':0.1},
    {'phase':'neutral','axis':'neutral','samples':10,'armed_fraction':1.0,
     'gyro_z_peak_abs':0.0,'dvl_vz_mean':0.0},
]
print(build_health(rows, input_mode='rc-override', sample_hz=10,
                   axis_s=2, neutral_s=1)['overall'])
PY

python3 -m compileall -q uuv_mujoco/current uuv_control_gui.py
git diff --check
python3 uuv_mujoco/current/tools/audit_code_contract_sources.py
python3 uuv_mujoco/current/tools/check_runtime_readiness_policy.py
```

The synthetic sign-pair case prints `warn`, preserving the existing direction
health warning semantics.
