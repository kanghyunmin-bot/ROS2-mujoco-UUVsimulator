# Vertical Sensor Contract Audit

- real_csv: `/Users/kanghyunmin/Desktop/uuv_sim/uuv_mujoco/v2.2/debug/controller_parity_412/real_90s_feedback_source_servo_20260601/real_controller_feedback_20hz.csv`
- window: `0.350s` to `10.350s`
- early_window_s: `2.000`
- decision: `structural_vertical_time_response_mismatch`

## First-window direction

| case | depth_slope_real | depth_slope_sim | depth_end_diff | dvlz_slope_real | dvlz_slope_sim | dvlz_mean_diff | depth_dir_match | dvlz_dir_match |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| v55_10s | +0.000337 | -0.002981 | -0.008829 | -0.006066 | +0.002085 | +0.019370 | `False` | `False` |
| local_xy_pose_z_fd1p0 | +0.000337 | -0.002741 | -0.008410 | -0.006066 | +0.001248 | +0.019686 | `False` | `False` |
| local_xy_pose_z_fd1p0_heave240 | +0.000337 | -0.000384 | -0.003571 | -0.006066 | +0.001657 | +0.021981 | `False` | `False` |
| local_xy_dvl_z | +0.000337 | -0.001758 | -0.006667 | -0.006066 | -0.001937 | +0.020892 | `False` | `True` |

## DVL_z sign/frame

| source | corr(local_z,dvl_z) | corr(local_z,-dvl_z) | corr(body_z,dvl_z) | corr(body_z,-dvl_z) |
|---|---:|---:|---:|---:|
| real | -0.5803 | +0.5803 | nan | nan |
| v55_10s | -0.7950 | +0.7950 | -0.8437 | +0.8437 |
| local_xy_pose_z_fd1p0 | -0.7581 | +0.7581 | -0.9594 | +0.9594 |
| local_xy_pose_z_fd1p0_heave240 | -0.7336 | +0.7336 | -0.9410 | +0.9410 |
| local_xy_dvl_z | -0.5172 | +0.5172 | -0.5428 | +0.5428 |

## Interpretation

DVL_z sign/frame is not a simple sign inversion: real and sim both show DVL_z opposing local/body z. The failure is the early vertical time response, so fitting hydrodynamic coefficients against the current low-rate actuator telemetry is underconstrained.
