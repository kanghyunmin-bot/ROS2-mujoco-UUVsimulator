# Validation Outputs

Run step-response validation:

```bash
python uuv_mujoco/run_urdf_full.py --validate
```

Custom output path:

```bash
python uuv_mujoco/run_urdf_full.py --validate --validation-dir uuv_mujoco/validation
```

Outputs:

- `forward_step.csv`
- `heave_step.csv`
- `yaw_step.csv`
- `summary.json`

Metrics in `summary.json`:

- `target_ss`: steady-state response estimate during step-on window
- `peak`: peak response during step-on window (sign-aware)
- `overshoot_pct`: sign-aware overshoot percentage
- `settling_time_s`: time from step start to first sustained entry into tolerance band
- `steady_state_drift`: mean absolute deviation from `target_ss` after command returns to zero
- `status`: `ok` or `unstable`
