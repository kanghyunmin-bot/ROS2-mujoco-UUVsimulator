# Vertical Sensor Contract Audit

- real_csv: `/Users/kanghyunmin/Desktop/uuv_sim/uuv_mujoco/v2.2/debug/controller_parity_412/real_90s_feedback_source_servo_20260601/real_controller_feedback_20hz.csv`
- window: `0.350s` to `10.350s`
- early_window_s: `2.000`
- decision: `structural_vertical_time_response_mismatch`

## First-window direction

| case | depth_slope_real | depth_slope_sim | depth_end_diff | dvlz_slope_real | dvlz_slope_sim | dvlz_mean_diff | depth_dir_match | dvlz_dir_match |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| v55_10s | +0.000337 | -0.002981 | -0.008829 | -0.006066 | +0.002085 | +0.019370 | `False` | `False` |
| recon_highrate_original | +0.000337 | +0.025925 | +0.031113 | -0.006066 | -0.044162 | +0.045759 | `True` | `True` |
| recon_highrate_vgain0p2 | +0.000337 | -0.003381 | -0.011729 | -0.006066 | -0.007434 | +0.024384 | `False` | `True` |
| recon_highrate_vgain0p25 | +0.000337 | -0.002176 | -0.005190 | -0.006066 | -0.010137 | +0.025392 | `False` | `True` |

## DVL_z sign/frame

| source | corr(local_z,dvl_z) | corr(local_z,-dvl_z) | corr(body_z,dvl_z) | corr(body_z,-dvl_z) |
|---|---:|---:|---:|---:|
| real | -0.5803 | +0.5803 | nan | nan |
| v55_10s | -0.7950 | +0.7950 | -0.8437 | +0.8437 |
| recon_highrate_original | -0.8788 | +0.8788 | -0.8638 | +0.8638 |
| recon_highrate_vgain0p2 | -0.4765 | +0.4765 | -0.6087 | +0.6087 |
| recon_highrate_vgain0p25 | -0.5932 | +0.5932 | -0.6340 | +0.6340 |

## Interpretation

DVL_z sign/frame is not a simple sign inversion: real and sim both show DVL_z opposing local/body z. The failure is the early vertical time response, so fitting hydrodynamic coefficients against the current low-rate actuator telemetry is underconstrained.
