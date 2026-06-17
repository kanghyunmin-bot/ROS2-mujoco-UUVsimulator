# Vertical Sensor Contract Audit

- real_csv: `/Users/kanghyunmin/Desktop/uuv_sim/uuv_mujoco/v2.2/debug/controller_parity_412/real_90s_feedback_source_servo_20260601/real_controller_feedback_20hz.csv`
- window: `0.350s` to `10.350s`
- early_window_s: `2.000`
- decision: `structural_vertical_time_response_mismatch`

## First-window direction

| case | depth_slope_real | depth_slope_sim | depth_end_diff | dvlz_slope_real | dvlz_slope_sim | dvlz_mean_diff | depth_dir_match | dvlz_dir_match |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| v55_10s | +0.000337 | -0.002981 | -0.008829 | -0.006066 | +0.002085 | +0.019370 | `False` | `False` |
| v64_buoy100 | +0.000337 | +0.004439 | +0.007469 | -0.006066 | +0.004880 | +0.027072 | `True` | `False` |
| v64_vlift0 | +0.000337 | +0.001824 | +0.000206 | -0.006066 | +0.000042 | +0.023840 | `True` | `False` |
| v64_heave240 | +0.000337 | -0.000576 | -0.003905 | -0.006066 | +0.002526 | +0.021654 | `False` | `False` |
| v64_vlift13000 | +0.000337 | -0.006290 | -0.015231 | -0.006066 | +0.002904 | +0.016222 | `False` | `False` |
| v64_yawheave0 | +0.000337 | -0.003033 | -0.008922 | -0.006066 | +0.002170 | +0.019299 | `False` | `False` |

## DVL_z sign/frame

| source | corr(local_z,dvl_z) | corr(local_z,-dvl_z) | corr(body_z,dvl_z) | corr(body_z,-dvl_z) |
|---|---:|---:|---:|---:|
| real | -0.5803 | +0.5803 | nan | nan |
| v55_10s | -0.7950 | +0.7950 | -0.8437 | +0.8437 |
| v64_buoy100 | -0.8213 | +0.8213 | -0.7912 | +0.7912 |
| v64_vlift0 | -0.3320 | +0.3320 | -0.3743 | +0.3743 |
| v64_heave240 | -0.7629 | +0.7629 | -0.8004 | +0.8004 |
| v64_vlift13000 | -0.8792 | +0.8792 | -0.9575 | +0.9575 |
| v64_yawheave0 | -0.8004 | +0.8004 | -0.8563 | +0.8563 |

## Interpretation

DVL_z sign/frame is not a simple sign inversion: real and sim both show DVL_z opposing local/body z. The failure is the early vertical time response, so fitting hydrodynamic coefficients against the current low-rate actuator telemetry is underconstrained.
