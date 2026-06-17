# Plant Sensor Contract Audit

- real_csv: `/Users/kanghyunmin/Desktop/uuv_sim/uuv_mujoco/v2.2/debug/controller_parity_412/real_90s_feedback_source_servo_20260601/real_controller_feedback_20hz.csv`
- plant_input_csv: `/Users/kanghyunmin/Desktop/uuv_sim/uuv_mujoco/v2.2/debug/controller_parity_412/real_90s_feedback_source_servo_20260601/real_controller_feedback_20hz.csv`
- plant_input_time_column: `t_s`; plant_input_pwm_prefix: `rc_out`
- sim_csv: `/Users/kanghyunmin/Desktop/uuv_sim/UUV-HAN/outputs/plant_contract_structure_v65_20260615/real_rcou_startvel_local_xy_dvl_z_10s/sim_sensor_replay.csv`
- window: `0.350s -> 10.350s`

## Plant Input

- samples: `4008`; mismatches: `0`; maxdiff_us: `0.000`; ok: `True`
- input source window used by sim rows: `0.800s -> 10.750s`; before_first: `0`; after_last: `0`
- raw sample median_dt: `0.050s`; raw median_hz: `20.000`
- distinct PWM steps: `21`; step median_dt: `0.500s`; step median_hz: `2.000`; zoh_resampled_telemetry_like: `True`
- source_class: `real_mavros_rcout_servo_output_raw_telemetry`; semantic_layer: `real_controller_output_telemetry`; output_telemetry: `True`; high_rate_actuator_history: `False`

## Thruster-Layer Input Gate

- available: `True`; samples: `501`; mismatches: `61`; maxdiff_us: `362.000`; ok: `False`
- compared sim_time: `32.142` -> `42.142`; input_nonneutral: `501`; thruster_nonneutral: `500`
- step delivery: input_steps=`21`; thruster_steps=`22`; matched=`21`; missed=`0`
- step latency median/p95/max: `0.048` / `0.048` / `0.048` s; low_latency_ok: `True`; assessment: `all_steps_observable`
- sampled runtime: available=`True`; ok=`True`; nominal_p95_us=`33.550`; best_phase_s=`-0.013`; best_p95_us=`0.000`

## Plant Input Observability Gate

- ok_for_dynamic_sensor_corr_gate: `False`; corr_gate: `0.900`
- distinct_pwm_step_hz: `2.000`; raw_sample_hz: `20.000`; telemetry_limited: `True`; source_allowed_for_han_target_export: `False`
- dynamic sensor hz: dvl=`10.000`, imu=`20.000`, local_vel_body=`nan`
- reasons: `plant input source is output telemetry: real_mavros_rcout_servo_output_raw_telemetry`, `plant input semantic layer is not exportable: real_controller_output_telemetry`, `input PWM step rate is below real DVL update rate`, `input PWM step rate is below real IMU update rate`

## Sensor Corr Tuning Gate

- ok_for_hydrodynamic_tuning: `False`; corr_gate: `0.900`
- failed_targets: `bar30_depth`, `dvl_vel_x`, `dvl_vel_y`, `dvl_vel_z`, `imu_roll`, `imu_pitch`, `imu_gyro_x`, `imu_gyro_y`, `imu_gyro_z`

| target | metric | corr | ok |
| --- | --- | ---: | --- |
| bar30_depth | baro_ddepth_m | 0.001 | `False` |
| dvl_vel_x | dvl_x_mps | -0.011 | `False` |
| dvl_vel_y | dvl_y_mps | 0.133 | `False` |
| dvl_vel_z | dvl_z_mps | 0.137 | `False` |
| imu_roll | imu_roll_deg | 0.329 | `False` |
| imu_pitch | imu_pitch_deg | -0.177 | `False` |
| imu_yaw_delta | imu_yaw_rel_deg | 0.987 | `True` |
| imu_gyro_x | gyro_x_radps | 0.510 | `False` |
| imu_gyro_y | gyro_y_radps | -0.039 | `False` |
| imu_gyro_z | gyro_z_radps | 0.737 | `False` |

## Phase Contract

- first real/sim relative sample: `0.000s` / `0.000s`; max_initial_phase_slip_s: `0.075s`; ok: `True`

## Dynamic Start Contract

- ok: `False`

| topic | updated_t_s | max_latency_s | first_err | updated_err | max_err | start_ok | update_ok |
| --- | ---: | ---: | ---: | ---: | ---: | --- | --- |
| dvl_twist | 0.020 | 0.150 | 0.379 | 0.057 | 0.080 | `False` | `True` |
| imu_gyro | 0.020 | 0.075 | 0.009 | 0.000 | 0.030 | `True` | `True` |
| local_vel_body | 0.020 | 0.400 | nan | nan | 0.080 | `False` | `False` |

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

- input_step_count: `21`

| topic | real_med_offset_s | sim_med_offset_s | err_s | real_updates/step | sim_updates/step |
| --- | ---: | ---: | ---: | ---: | ---: |
| bar30_pressure | 0.000 | 0.020 | 0.020 | 0.857 | 0.905 |
| depth_pose | 0.000 | 0.020 | 0.020 | 0.857 | 0.905 |
| imu_data | 0.200 | 0.220 | 0.020 | 9.143 | 9.095 |
| imu_raw | nan | 0.020 | nan | nan | 0.905 |
| dvl_twist | 0.250 | 0.220 | -0.030 | 5.619 | 4.524 |
| local_pose | 0.200 | 0.180 | -0.020 | 1.381 | 1.333 |
| local_vel_body | nan | 0.180 | nan | nan | 1.333 |

## Yaw Motion

- yaw_delta_deg real/sim/ratio: `-181.870` / `-172.919` / `0.951`
- gyro_z_integral_deg real/sim/ratio: `-181.762` / `-172.743` / `0.950`

### Windowed Yaw Drift

| window_s | real_mean_r | sim_mean_r | mean_err_r | real_int_deg | sim_int_deg | deficit_deg | ratio |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 0.00-10.00 | -0.3159 | -0.3013 | 0.0145 | -181.8 | -172.7 | 9.0 | 0.950 |

## Thruster Debug

- available: `True`; active_rows: `212`
- thr_torque_body_z mean/rms/min/max Nm: `-2.136` / `3.225` / `-9.544` / `0.832`
- ang_vel_body_z mean/rms/min/max rad/s: `-0.310` / `0.436` / `-0.979` / `0.132`
- direct T200 force contract violations: `0`; likely_perf_direct_disabled: `False`


## Direct Sensors

| signal | RMSE | corr | bias | lag_s |
| --- | ---: | ---: | ---: | ---: |
| baro_ddepth_m | 0.013 | 0.001 | -0.006 | -2.950 |
| depth_pose_m | 0.013 | 0.002 | -0.006 | -3.000 |
| imu_roll_deg | 0.357 | 0.329 | 0.238 | 0.250 |
| imu_pitch_deg | 0.863 | -0.177 | -0.558 | 3.450 |
| imu_yaw_rel_deg | 24.042 | 0.987 | 20.863 | 0.200 |
| dvl_x_mps | 0.055 | -0.011 | -0.033 | 2.800 |
| dvl_y_mps | 0.056 | 0.133 | 0.031 | 1.450 |
| dvl_z_mps | 0.024 | 0.137 | 0.021 | 3.000 |
| gyro_x_radps | 0.018 | 0.510 | -0.001 | -4.450 |
| gyro_y_radps | 0.040 | -0.039 | -0.005 | 4.650 |
| gyro_z_radps | 0.252 | 0.737 | 0.015 | -4.400 |
| accel_x_mps2 | 0.162 | 0.150 | -0.060 | -4.350 |
| accel_y_mps2 | 0.269 | 0.551 | 0.207 | -0.250 |
| accel_z_mps2 | 0.067 | 0.031 | 0.021 | 3.200 |

## Estimator Surface

Real local_position is an FC estimator output; sim local_position is a MuJoCo truth-like compatibility topic.

| signal | RMSE | corr | bias | lag_s |
| --- | ---: | ---: | ---: | ---: |
| local_depth_m | 0.009 | -0.098 | -0.006 | -2.000 |
| local_x_m | 0.117 | 0.996 | -0.099 | 0.150 |
| local_y_m | 0.258 | 0.787 | 0.125 | 0.800 |
| local_vel_x_mps | 0.047 | 0.911 | 0.002 | 0.450 |
| local_vel_y_mps | 0.095 | 0.971 | 0.062 | 0.150 |
| local_vel_z_mps | 0.015 | 0.076 | -0.011 | 3.050 |
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
| sim | x | -0.041 | 0.074 | -0.251 | 0.204 |
| sim | y | -0.102 | 0.154 | -0.400 | 0.208 |
| sim | z | -0.846 | 0.847 | -0.964 | -0.450 |

IMU gyro self-consistency compares `angular_velocity` against body-rate inferred from the same orientation quaternion.

| source | axis | samples | RMSE | corr | bias |
| --- | --- | ---: | ---: | ---: | ---: |
| real | x | 200 | 0.009 | 0.908 | 0.000 |
| real | y | 200 | 0.005 | 0.980 | 0.000 |
| real | z | 200 | 0.127 | 0.945 | -0.000 |
| sim | x | 200 | 0.003 | 0.969 | -0.000 |
| sim | y | 200 | 0.005 | 0.985 | -0.000 |
| sim | z | 200 | 0.017 | 0.998 | 0.000 |
- real gyro self-axis: identity_corr_rank=`1`, identity_rmse_rank=`1`, identity_rmse_sum=`0.141`, best_rmse_sum=`0.141`, best_rmse_mapping=`['real_x <= +sim_x', 'real_y <= +sim_y', 'real_z <= +sim_z']`
- sim gyro self-axis: identity_corr_rank=`1`, identity_rmse_rank=`1`, identity_rmse_sum=`0.026`, best_rmse_sum=`0.026`, best_rmse_mapping=`['real_x <= +sim_x', 'real_y <= +sim_y', 'real_z <= +sim_z']`

DVL fit is `dvl_axis = slope * body_velocity_axis + intercept`.
Body velocity source: real=velocity_local rotated by IMU quaternion (samples=0), sim=/mavros/local_position/velocity_body (samples=201). Sim truth diagnostic=/sim/odom twist body FLU (samples=201).

| source | axis | slope | intercept | corr |
| --- | --- | ---: | ---: | ---: |
| real | x | 0.722 | 0.094 | 0.473 |
| real | y | -0.716 | 0.013 | -0.893 |
| real | z | -0.973 | -0.007 | -0.610 |
| sim_local_surface | x | 0.953 | 0.013 | 0.988 |
| sim_local_surface | y | -0.988 | 0.001 | -0.972 |
| sim_local_surface | z | -0.437 | -0.006 | -0.670 |
| sim_truth_odom | x | 1.004 | -0.001 | 0.988 |
| sim_truth_odom | y | 0.991 | -0.000 | 0.993 |
| sim_truth_odom | z | 1.349 | -0.004 | 0.836 |

DVL-as-body-FLU maps the current DVL FRD-like twist into base_link FLU as `[x, -y, -z]`.

| axis | real-vs-sim RMSE | corr | bias |
| --- | ---: | ---: | ---: |
| x | 0.055 | -0.011 | -0.033 |
| y | 0.056 | 0.133 | -0.031 |
| z | 0.024 | 0.137 | -0.021 |

DVL point-velocity residual is fit as `dvl_body_flu - body_velocity_flu = omega_flu x r_flu`.

| source | r_x_m | r_y_m | r_z_m | residual_rms | fit_rmse | explained |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| real | 0.0279 | 0.0087 | -0.0442 | 0.0243 | 0.0230 | 0.054 |
| sim_local_surface | -0.0015 | 0.0007 | 0.0544 | 0.0048 | 0.0047 | 0.022 |
| sim_truth_odom | -0.0000 | -0.0002 | -0.0084 | 0.0033 | 0.0033 | 0.002 |

## Axis Audit

- dvl_twist: identity_rank=`7`, identity_rmse_rank=`1`, best=`real_x <= -sim_y, real_y <= +sim_z, real_z <= -sim_x`, sumcorr=`0.530`, best_rmse=`real_x <= +sim_x, real_y <= +sim_y, real_z <= +sim_z`, rmse_sum=`0.135`
- imu_gyro: identity_rank=`2`, identity_rmse_rank=`2`, best=`real_x <= +sim_x, real_y <= -sim_y, real_z <= +sim_z`, sumcorr=`1.286`, best_rmse=`real_x <= +sim_x, real_y <= -sim_y, real_z <= +sim_z`, rmse_sum=`0.308`
- imu_accel: identity_rank=`1`, identity_rmse_rank=`1`, best=`real_x <= +sim_x, real_y <= +sim_y, real_z <= +sim_z`, sumcorr=`0.733`, best_rmse=`real_x <= +sim_x, real_y <= +sim_y, real_z <= +sim_z`, rmse_sum=`0.499`
