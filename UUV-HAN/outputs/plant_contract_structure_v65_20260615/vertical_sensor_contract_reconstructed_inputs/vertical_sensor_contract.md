# Vertical Sensor Contract Audit

- real_csv: `/Users/kanghyunmin/Desktop/uuv_sim/uuv_mujoco/v2.2/debug/controller_parity_412/real_90s_feedback_source_servo_20260601/real_controller_feedback_20hz.csv`
- window: `0.350s` to `10.350s`
- early_window_s: `2.000`
- decision: `structural_vertical_time_response_mismatch`

## First-window direction

| case | depth_slope_real | depth_slope_sim | depth_end_diff | dvlz_slope_real | dvlz_slope_sim | dvlz_mean_diff | depth_dir_match | dvlz_dir_match |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| v55_10s | +0.000337 | -0.002981 | -0.008829 | -0.006066 | +0.002085 | +0.019370 | `False` | `False` |
| recon_20s | +0.000337 | -0.008392 | -0.020346 | -0.006066 | +0.000091 | +0.011277 | `False` | `False` |
| recon_prevphase_full90 | +0.000337 | +0.030676 | +0.037485 | -0.006066 | -0.001464 | +0.050666 | `True` | `True` |
| recon_prevphase_highrate_lagm045_20s | +0.000337 | +0.025925 | +0.031113 | -0.006066 | -0.044162 | +0.045759 | `True` | `True` |
| recon_prevphase_lowrate_lag0_20s | +0.000337 | +0.042452 | +0.060770 | -0.006066 | +0.011738 | +0.061598 | `True` | `False` |
| input_contract_highrate_json | +0.000337 | -0.003300 | -0.010764 | -0.006066 | -0.001469 | +0.023516 | `False` | `True` |

## DVL_z sign/frame

| source | corr(local_z,dvl_z) | corr(local_z,-dvl_z) | corr(body_z,dvl_z) | corr(body_z,-dvl_z) |
|---|---:|---:|---:|---:|
| real | -0.5803 | +0.5803 | nan | nan |
| v55_10s | -0.7950 | +0.7950 | -0.8437 | +0.8437 |
| recon_20s | -0.7050 | +0.7050 | -0.5723 | +0.5723 |
| recon_prevphase_full90 | -0.8680 | +0.8680 | -0.9242 | +0.9242 |
| recon_prevphase_highrate_lagm045_20s | -0.8788 | +0.8788 | -0.8638 | +0.8638 |
| recon_prevphase_lowrate_lag0_20s | -0.9476 | +0.9476 | -0.9515 | +0.9515 |
| input_contract_highrate_json | -0.0612 | +0.0612 | -0.4666 | +0.4666 |

## Interpretation

DVL_z sign/frame is not a simple sign inversion: real and sim both show DVL_z opposing local/body z. The failure is the early vertical time response, so fitting hydrodynamic coefficients against the current low-rate actuator telemetry is underconstrained.
