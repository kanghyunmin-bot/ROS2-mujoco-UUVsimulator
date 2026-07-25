# Axis RC Plot Output Split

## Scope

Split axis RC override output generation while preserving output filenames,
JSON schema, plot content, and the public `axis_rc_plotting` import surface.

## Files

- `tools/axis_rc_plotting.py`: compatibility exports for `write_outputs()`,
  `plot_timeseries()`, and `values()`.
- `tools/axis_rc_output_files.py`: ordered CSV writers and summary JSON payload.
- `tools/axis_rc_plot_series.py`: time and numeric series helpers.
- `tools/axis_rc_plot_render.py`: Matplotlib time-series PNG renderer.

## Contract

- `axis_timeseries.csv`, `axis_summary.csv`, `axis_summary.json`, and
  `axis_response.png` paths are unchanged.
- `axis_summary.json` still embeds metadata, phases, summary rows, sample count,
  and `build_health()` output.
- RC channel plotting remains a step plot with `where="post"`.
- No controller output, RC override transport, plant input, or telemetry
  comparison code changed.

## Verification

```bash
python3 -m compileall -q \
  uuv_mujoco/v2.2/tools/axis_rc_plotting.py \
  uuv_mujoco/v2.2/tools/axis_rc_output_files.py \
  uuv_mujoco/v2.2/tools/axis_rc_plot_render.py \
  uuv_mujoco/v2.2/tools/axis_rc_plot_series.py

PYTHONPATH=uuv_mujoco/v2.2/tools \
/Users/kanghyunmin/.venvs/mujoco311/bin/python - <<'PY'
from pathlib import Path
import json
import tempfile
from axis_rc_contract import Phase
from axis_rc_plotting import write_outputs

samples = []
for i in range(6):
    row = {
        't': float(i) * 0.1,
        'depth_m': 0.5 + 0.01 * i,
        'odom_z': -0.5 - 0.01 * i,
        'dvl_vx': 0.02 * i,
        'dvl_vy': 0.01 * i,
        'dvl_vz': -0.005 * i,
        'gyro_x': 0.001 * i,
        'gyro_y': -0.001 * i,
        'gyro_z': 0.01 * i,
    }
    for ch in range(1, 9):
        row[f'rcin{ch}'] = 1500 + (20 * i if ch == 4 else 0)
    samples.append(row)

summary = [
    {'phase':'yaw_pos','axis':'yaw','samples':6,'armed_fraction':1.0,
     'rcout_max_delta':100.0,'expected_metric_peak_abs':0.05,
     'expected_metric_mean':0.02},
    {'phase':'neutral','axis':'neutral','samples':6,'armed_fraction':1.0,
     'gyro_z_peak_abs':0.0,'dvl_vz_mean':0.0},
]
metadata = {'input_mode':'rc-override','sample_hz':10.0,
            'axis_s':0.5,'neutral_s':0.5}
with tempfile.TemporaryDirectory(prefix='axis_rc_plot_') as tmp:
    out_dir = Path(tmp)
    write_outputs(out_dir, samples,
                  [Phase('yaw_pos', 'yaw', 1.0, 0.1, 0.4)],
                  summary, metadata)
    payload = json.loads((out_dir / 'axis_summary.json').read_text())
    print(payload['health']['overall'])
    print((out_dir / 'axis_timeseries.csv').exists(),
          (out_dir / 'axis_summary.csv').exists(),
          (out_dir / 'axis_response.png').stat().st_size > 0)
PY

python3 -m compileall -q sim/current uuv_control_gui.py
git diff --check
python3 sim/current/tools/audit_code_contract_sources.py
python3 sim/current/tools/check_runtime_readiness_policy.py
python3 sim/current/tools/verify_ardusub_thruster_contract.py --quiet
/Users/kanghyunmin/.venvs/mujoco311/bin/python \
  sim/current/tools/physics_contract_audit.py --simulate-s 0.05
```

The synthetic output run printed `pass` and confirmed both CSV files plus a
non-empty PNG.
