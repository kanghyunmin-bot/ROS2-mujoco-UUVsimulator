# Plant Sensor Contract Audit

- real_csv: `/Users/kanghyunmin/Desktop/uuv_sim/uuv_mujoco/v2.2/debug/controller_parity_412/real_90s_feedback_source_servo_20260601/real_controller_feedback_20hz.csv`
- plant_input_csv: `/Users/kanghyunmin/Desktop/uuv_sim/UUV-HAN/outputs/controller_reconstruction_90s_ab_prevphase_depthpreloadp0p12_full90_isolated_20260614/sitl_controller_io.csv`
- plant_input_time_column: `t_real_s`; plant_input_pwm_prefix: `rc_out`
- sim_csv: `/Users/kanghyunmin/Desktop/uuv_sim/UUV-HAN/outputs/plant_contract_structure_v65_20260615/highrate_recon_vertical_gain_0p2_10s/sim_sensor_replay.csv`
- window: `0.350s -> 10.350s`

## Plant Input

- samples: `4000`; mismatches: `0`; maxdiff_us: `0.000`; ok: `True`
- input source window used by sim rows: `0.800s -> 10.782s`; before_first: `0`; after_last: `0`
- raw sample median_dt: `0.001s`; raw median_hz: `1200.480`
- distinct PWM steps: `3135`; step median_dt: `0.002s`; step median_hz: `400.000`; zoh_resampled_telemetry_like: `False`
- source_class: `controller_reconstructed_sitl_json_servo_diagnostic`; semantic_layer: `diagnostic_controller_reconstructed_actuator_backend`; output_telemetry: `False`; high_rate_actuator_history: `True`

## Thruster-Layer Input Gate

- available: `True`; samples: `500`; mismatches: `499`; maxdiff_us: `269.000`; ok: `True`
- compared sim_time: `33.042` -> `43.024`; input_nonneutral: `500`; thruster_nonneutral: `499`
- step delivery: input_steps=`498`; thruster_steps=`202`; matched=`201`; missed=`297`
- step latency median/p95/max: `0.016` / `0.016` / `0.036` s; low_latency_ok: `False`; assessment: `unobservable_high_rate_input_vs_thruster_debug_sampling`
- sampled runtime: available=`True`; ok=`True`; nominal_p95_us=`22.050`; best_phase_s=`-0.014`; best_p95_us=`8.000`

## Plant Input Observability Gate

- ok_for_dynamic_sensor_corr_gate: `False`; corr_gate: `0.900`
- distinct_pwm_step_hz: `400.000`; raw_sample_hz: `1200.480`; telemetry_limited: `False`; source_allowed_for_han_target_export: `False`
- dynamic sensor hz: dvl=`10.000`, imu=`20.000`, local_vel_body=`nan`
- reasons: `plant input semantic layer is not exportable: diagnostic_controller_reconstructed_actuator_backend`

## Sensor Corr Tuning Gate

- ok_for_hydrodynamic_tuning: `False`; corr_gate: `0.900`
- failed_targets: `bar30_depth`, `dvl_vel_x`, `dvl_vel_y`, `dvl_vel_z`, `imu_roll`, `imu_pitch`, `imu_gyro_x`, `imu_gyro_y`, `imu_gyro_z`

| target | metric | corr | ok |
| --- | --- | ---: | --- |
| bar30_depth | baro_ddepth_m | 0.153 | `False` |
| dvl_vel_x | dvl_x_mps | -0.336 | `False` |
| dvl_vel_y | dvl_y_mps | 0.173 | `False` |
| dvl_vel_z | dvl_z_mps | 0.010 | `False` |
| imu_roll | imu_roll_deg | 0.284 | `False` |
| imu_pitch | imu_pitch_deg | -0.295 | `False` |
| imu_yaw_delta | imu_yaw_rel_deg | 0.936 | `True` |
| imu_gyro_x | gyro_x_radps | 0.556 | `False` |
| imu_gyro_y | gyro_y_radps | 0.006 | `False` |
| imu_gyro_z | gyro_z_radps | 0.794 | `False` |

## Phase Contract

- first real/sim relative sample: `0.000s` / `0.000s`; max_initial_phase_slip_s: `0.075s`; ok: `True`

## Dynamic Start Contract

- ok: `False`

| topic | updated_t_s | max_latency_s | first_err | updated_err | max_err | start_ok | update_ok |
| --- | ---: | ---: | ---: | ---: | ---: | --- | --- |
| dvl_twist | 0.022 | 0.150 | 0.379 | 0.024 | 0.080 | `False` | `True` |
| imu_gyro | 0.022 | 0.075 | 0.009 | 0.000 | 0.030 | `True` | `True` |
| local_vel_body | 0.022 | 0.400 | nan | nan | 0.080 | `False` | `False` |

## Sensor Rate Contract

| topic | real_hz | sim_hz | sim/real | real_count | sim_count |
| --- | ---: | ---: | ---: | ---: | ---: |
| bar30_pressure | 2.000 | 2.000 | 1.000 | 19 | 21 |
| depth_pose | 2.000 | 2.000 | 1.000 | 19 | 21 |
| imu_data | 20.000 | 20.000 | 1.000 | 201 | 201 |
| imu_raw | nan | 2.000 | nan | 0 | 21 |
| dvl_twist | 10.000 | 10.000 | 1.000 | 123 | 101 |
| local_pose | 2.857 | 2.941 | 1.029 | 31 | 31 |
| local_vel_body | nan | 2.941 | nan | 0 | 31 |

## Sensor Phase Vs Plant Input

- input_step_count: `3135`

| topic | real_med_offset_s | sim_med_offset_s | err_s | real_updates/step | sim_updates/step |
| --- | ---: | ---: | ---: | ---: | ---: |
| bar30_pressure | 0.001 | 0.001 | -0.000 | 0.006 | 0.006 |
| depth_pose | 0.001 | 0.001 | -0.000 | 0.006 | 0.006 |
| imu_data | 0.002 | 0.001 | -0.000 | 0.061 | 0.061 |
| imu_raw | nan | 0.001 | nan | nan | 0.006 |
| dvl_twist | 0.002 | 0.001 | -0.000 | 0.038 | 0.030 |
| local_pose | 0.001 | 0.001 | 0.000 | 0.009 | 0.009 |
| local_vel_body | nan | 0.001 | nan | nan | 0.009 |

## Yaw Motion

- yaw_delta_deg real/sim/ratio: `-181.870` / `-157.778` / `0.868`
- gyro_z_integral_deg real/sim/ratio: `-181.762` / `-157.686` / `0.868`

### Windowed Yaw Drift

| window_s | real_mean_r | sim_mean_r | mean_err_r | real_int_deg | sim_int_deg | deficit_deg | ratio |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 0.00-9.95 | -0.3170 | -0.2750 | 0.0420 | -181.5 | -157.7 | 23.8 | 0.869 |

## Thruster Debug

- available: `True`; active_rows: `237`
- thr_torque_body_z mean/rms/min/max Nm: `-1.500` / `4.010` / `-10.507` / `3.133`
- ang_vel_body_z mean/rms/min/max rad/s: `-0.183` / `0.499` / `-1.086` / `0.444`
- direct T200 force contract violations: `0`; likely_perf_direct_disabled: `False`


## Direct Sensors

| signal | RMSE | corr | bias | lag_s |
| --- | ---: | ---: | ---: | ---: |
| baro_ddepth_m | 0.024 | 0.153 | -0.019 | -2.450 |
| depth_pose_m | 0.024 | 0.153 | -0.019 | -2.450 |
| imu_roll_deg | 0.407 | 0.284 | 0.196 | -0.100 |
| imu_pitch_deg | 1.495 | -0.295 | -1.077 | 3.200 |
| imu_yaw_rel_deg | 25.894 | 0.936 | -15.896 | -0.050 |
| dvl_x_mps | 0.099 | -0.336 | -0.067 | -4.300 |
| dvl_y_mps | 0.057 | 0.173 | 0.032 | -0.800 |
| dvl_z_mps | 0.023 | 0.010 | 0.020 | 3.300 |
| gyro_x_radps | 0.019 | 0.556 | -0.003 | -4.550 |
| gyro_y_radps | 0.067 | 0.006 | -0.005 | -3.250 |
| gyro_z_radps | 0.275 | 0.794 | 0.042 | -0.150 |
| accel_x_mps2 | 0.318 | -0.020 | 0.024 | 5.000 |
| accel_y_mps2 | 0.332 | 0.240 | 0.219 | 4.750 |
| accel_z_mps2 | 0.066 | 0.050 | 0.017 | 3.900 |

## Estimator Surface

Real local_position is an FC estimator output; sim local_position is a MuJoCo truth-like compatibility topic.

| signal | RMSE | corr | bias | lag_s |
| --- | ---: | ---: | ---: | ---: |
| local_depth_m | 0.024 | 0.060 | -0.022 | 3.450 |
| local_x_m | 0.446 | 0.984 | -0.345 | -0.200 |
| local_y_m | 0.937 | 0.605 | -0.840 | -1.550 |
| local_vel_x_mps | 0.113 | 0.686 | -0.076 | -0.200 |
| local_vel_y_mps | 0.150 | 0.923 | -0.109 | -0.500 |
| local_vel_z_mps | 0.010 | -0.121 | -0.007 | -2.500 |
| local_vel_body_x_mps | nan | nan | nan | nan |
| local_vel_body_y_mps | nan | nan | nan | nan |
| local_vel_body_z_mps | nan | nan | nan | nan |

## Sensor Semantics

IMU accel residual is `linear_accel - (-R^T gravity)` in the published ROS frame.

| source | axis | residual_mean | residual_rms | residual_min | residual_max |
| --- | --- | ---: | ---: | ---: | ---: |
| real | x | 0.114 | 0.196 | -0.382 | 0.518 |
| real | y | -0.268 | 0.326 | -0.659 | 0.054 |
| real | z | -0.862 | 0.863 | -0.967 | -0.766 |
| sim | x | -0.046 | 0.216 | -1.782 | 0.363 |
| sim | y | -0.084 | 0.178 | -0.405 | 0.193 |
| sim | z | -0.851 | 0.852 | -1.082 | -0.633 |

IMU gyro self-consistency compares `angular_velocity` against body-rate inferred from the same orientation quaternion.

| source | axis | samples | RMSE | corr | bias |
| --- | --- | ---: | ---: | ---: | ---: |
| real | x | 200 | 0.009 | 0.908 | 0.000 |
| real | y | 200 | 0.005 | 0.980 | 0.000 |
| real | z | 200 | 0.127 | 0.945 | -0.000 |
| sim | x | 199 | 0.003 | 0.983 | -0.000 |
| sim | y | 199 | 0.014 | 0.975 | -0.001 |
| sim | z | 199 | 0.021 | 0.999 | -0.000 |
- real gyro self-axis: identity_corr_rank=`1`, identity_rmse_rank=`1`, identity_rmse_sum=`0.141`, best_rmse_sum=`0.141`, best_rmse_mapping=`['real_x <= +sim_x', 'real_y <= +sim_y', 'real_z <= +sim_z']`
- sim gyro self-axis: identity_corr_rank=`1`, identity_rmse_rank=`1`, identity_rmse_sum=`0.038`, best_rmse_sum=`0.038`, best_rmse_mapping=`['real_x <= +sim_x', 'real_y <= +sim_y', 'real_z <= +sim_z']`

DVL fit is `dvl_axis = slope * body_velocity_axis + intercept`.
Body velocity source: real=velocity_local rotated by IMU quaternion (samples=0), sim=/mavros/local_position/velocity_body (samples=200). Sim truth diagnostic=/sim/odom twist body FLU (samples=200).

| source | axis | slope | intercept | corr |
| --- | --- | ---: | ---: | ---: |
| real | x | 0.722 | 0.094 | 0.473 |
| real | y | -0.716 | 0.013 | -0.893 |
| real | z | -0.973 | -0.007 | -0.610 |
| sim_local_surface | x | 0.831 | 0.043 | 0.848 |
| sim_local_surface | y | -0.997 | 0.000 | -0.983 |
| sim_local_surface | z | -0.609 | -0.004 | -0.661 |
| sim_truth_odom | x | 0.975 | 0.007 | 0.962 |
| sim_truth_odom | y | 0.992 | -0.000 | 0.996 |
| sim_truth_odom | z | 0.974 | 0.000 | 0.932 |

DVL-as-body-FLU maps the current DVL FRD-like twist into base_link FLU as `[x, -y, -z]`.

| axis | real-vs-sim RMSE | corr | bias |
| --- | ---: | ---: | ---: |
| x | 0.099 | -0.336 | -0.067 |
| y | 0.057 | 0.173 | -0.032 |
| z | 0.023 | 0.010 | -0.020 |

DVL point-velocity residual is fit as `dvl_body_flu - body_velocity_flu = omega_flu x r_flu`.

| source | r_x_m | r_y_m | r_z_m | residual_rms | fit_rmse | explained |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| real | 0.0279 | 0.0087 | -0.0442 | 0.0243 | 0.0230 | 0.054 |
| sim_local_surface | -0.0113 | 0.0012 | 0.3298 | 0.0176 | 0.0130 | 0.265 |
| sim_truth_odom | 0.0038 | -0.0012 | -0.1172 | 0.0086 | 0.0075 | 0.132 |

## Axis Audit

- dvl_twist: identity_rank=`31`, identity_rmse_rank=`1`, best=`real_x <= -sim_x, real_y <= +sim_z, real_z <= +sim_y`, sumcorr=`0.627`, best_rmse=`real_x <= +sim_x, real_y <= +sim_y, real_z <= +sim_z`, rmse_sum=`0.179`
- imu_gyro: identity_rank=`1`, identity_rmse_rank=`1`, best=`real_x <= +sim_x, real_y <= +sim_y, real_z <= +sim_z`, sumcorr=`1.357`, best_rmse=`real_x <= +sim_x, real_y <= +sim_y, real_z <= +sim_z`, rmse_sum=`0.361`
- imu_accel: identity_rank=`6`, identity_rmse_rank=`1`, best=`real_x <= -sim_z, real_y <= +sim_x, real_z <= -sim_y`, sumcorr=`0.372`, best_rmse=`real_x <= +sim_x, real_y <= +sim_y, real_z <= +sim_z`, rmse_sum=`0.715`
