# Plant Replay Ablation Summary

This file is generated from existing plant replay artifacts. It does not run MuJoCo.

## Contract Table

| case | input ok | high-rate history | dynamic start | hydro gate | baro corr | dvl x/y/z corr | gyro y/z corr | yaw ratio | diagnosis |
| --- | --- | --- | --- | --- | ---: | --- | --- | ---: | --- |
| highrate_recon_vertical_gain_0p25_10s | None | None | False | None | 0.157 | -0.330/0.186/-0.038 | 0.004/0.795 | nan | dynamic_start_contract_fail,baro_depth_corr_low,pitch_rate_corr_low,yaw_rate_corr_below_gate,yaw_bias_large |
| highrate_recon_vertical_gain_0p2_10s | None | None | False | None | 0.157 | -0.328/0.184/0.003 | 0.006/0.794 | nan | dynamic_start_contract_fail,baro_depth_corr_low,pitch_rate_corr_low,yaw_rate_corr_below_gate,yaw_bias_large |
| real_rcou_geometry_bar30_10s | None | None | False | None | -0.004 | 0.077/0.146/0.140 | -0.042/0.737 | nan | dynamic_start_contract_fail,baro_depth_corr_low,pitch_rate_corr_low,yaw_rate_corr_below_gate,yaw_bias_large |
| real_rcou_startvel_dvl_10s | None | None | False | None | 0.003 | 0.093/0.079/0.132 | -0.040/0.738 | nan | dynamic_start_contract_fail,baro_depth_corr_low,pitch_rate_corr_low,yaw_rate_corr_below_gate,yaw_bias_large |
| real_rcou_startvel_local_xy_dvl_z_10s | None | None | False | None | 0.002 | 0.079/0.146/0.133 | -0.039/0.736 | nan | dynamic_start_contract_fail,baro_depth_corr_low,pitch_rate_corr_low,yaw_rate_corr_below_gate,yaw_bias_large |
| real_rcou_startvel_local_xy_dvl_z_blend0p5_10s | None | None | False | None | -0.003 | 0.078/0.146/0.178 | -0.040/0.739 | nan | dynamic_start_contract_fail,baro_depth_corr_low,pitch_rate_corr_low,yaw_rate_corr_below_gate,yaw_bias_large |
| real_rcou_startvel_local_xy_dvl_z_buoy100_10s | None | None | False | None | -0.166 | 0.085/0.153/0.144 | -0.042/0.740 | nan | dynamic_start_contract_fail,baro_depth_corr_low,pitch_rate_corr_low,yaw_rate_corr_below_gate,yaw_bias_large |
| real_rcou_startvel_local_xy_dvl_z_buoy1014_10s | None | None | False | None | -0.150 | 0.081/0.148/0.137 | -0.039/0.740 | nan | dynamic_start_contract_fail,baro_depth_corr_low,pitch_rate_corr_low,yaw_rate_corr_below_gate,yaw_bias_large |
| real_rcou_startvel_local_xy_dvl_z_heave240_10s | None | None | False | None | -0.148 | 0.082/0.148/0.126 | -0.038/0.739 | nan | dynamic_start_contract_fail,baro_depth_corr_low,pitch_rate_corr_low,yaw_rate_corr_below_gate,yaw_bias_large |
| real_rcou_startvel_local_xy_dvl_z_vhist2s_10s | None | None | False | None | 0.002 | 0.077/0.147/0.134 | -0.037/0.741 | nan | dynamic_start_contract_fail,baro_depth_corr_low,pitch_rate_corr_low,yaw_rate_corr_below_gate,yaw_bias_large |
| real_rcou_startvel_local_xy_pose_z_fd1p0_10s | None | None | False | None | -0.004 | 0.078/0.146/0.176 | -0.041/0.738 | nan | dynamic_start_contract_fail,baro_depth_corr_low,pitch_rate_corr_low,yaw_rate_corr_below_gate,yaw_bias_large |
| real_rcou_startvel_local_xy_pose_z_fd1p0_heave240_retry_10s | None | None | False | None | -0.151 | 0.083/0.149/0.160 | -0.039/0.741 | nan | dynamic_start_contract_fail,baro_depth_corr_low,pitch_rate_corr_low,yaw_rate_corr_below_gate,yaw_bias_large |

## Key Metrics

| case | baro RMSE | depth RMSE | yaw RMSE | yaw bias | pitch RMSE | dvl_z RMSE | active rows | thr Fz mean | qacc z mean | overlay |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | --- |
| highrate_recon_vertical_gain_0p25_10s | 0.026 | 0.026 | 26.646 | -16.898 | 1.589 | 0.024 | 248 | -0.908 | 0.000 | `/Users/kanghyunmin/Desktop/uuv_sim/UUV-HAN/outputs/plant_contract_structure_v65_20260615/highrate_recon_vertical_gain_0p25_10s/sensor_overlay/plant_sensor_overlay.png` |
| highrate_recon_vertical_gain_0p2_10s | 0.024 | 0.024 | 26.624 | -16.918 | 1.520 | 0.023 | 250 | -0.989 | 0.000 | `/Users/kanghyunmin/Desktop/uuv_sim/UUV-HAN/outputs/plant_contract_structure_v65_20260615/highrate_recon_vertical_gain_0p2_10s/sensor_overlay/plant_sensor_overlay.png` |
| real_rcou_geometry_bar30_10s | 0.014 | 0.014 | 24.149 | 20.565 | 0.866 | 0.023 | 219 | -1.839 | -0.001 | `/Users/kanghyunmin/Desktop/uuv_sim/UUV-HAN/outputs/plant_contract_structure_v65_20260615/real_rcou_geometry_bar30_10s/sensor_overlay/plant_sensor_overlay.png` |
| real_rcou_startvel_dvl_10s | 0.013 | 0.013 | 24.177 | 20.619 | 0.869 | 0.024 | 219 | -1.839 | 0.005 | `/Users/kanghyunmin/Desktop/uuv_sim/UUV-HAN/outputs/plant_contract_structure_v65_20260615/real_rcou_startvel_dvl_10s/sensor_overlay/plant_sensor_overlay.png` |
| real_rcou_startvel_local_xy_dvl_z_10s | 0.013 | 0.013 | 23.683 | 20.204 | 0.868 | 0.024 | 217 | -1.843 | 0.005 | `/Users/kanghyunmin/Desktop/uuv_sim/UUV-HAN/outputs/plant_contract_structure_v65_20260615/real_rcou_startvel_local_xy_dvl_z_10s/sensor_overlay/plant_sensor_overlay.png` |
| real_rcou_startvel_local_xy_dvl_z_blend0p5_10s | 0.013 | 0.013 | 23.312 | 19.844 | 0.867 | 0.023 | 216 | -1.844 | 0.002 | `/Users/kanghyunmin/Desktop/uuv_sim/UUV-HAN/outputs/plant_contract_structure_v65_20260615/real_rcou_startvel_local_xy_dvl_z_blend0p5_10s/sensor_overlay/plant_sensor_overlay.png` |
| real_rcou_startvel_local_xy_dvl_z_buoy100_10s | 0.047 | 0.047 | 23.304 | 19.918 | 0.880 | 0.031 | 216 | -1.840 | 0.002 | `/Users/kanghyunmin/Desktop/uuv_sim/UUV-HAN/outputs/plant_contract_structure_v65_20260615/real_rcou_startvel_local_xy_dvl_z_buoy100_10s/sensor_overlay/plant_sensor_overlay.png` |
| real_rcou_startvel_local_xy_dvl_z_buoy1014_10s | 0.016 | 0.017 | 23.201 | 19.796 | 0.872 | 0.026 | 216 | -1.842 | 0.002 | `/Users/kanghyunmin/Desktop/uuv_sim/UUV-HAN/outputs/plant_contract_structure_v65_20260615/real_rcou_startvel_local_xy_dvl_z_buoy1014_10s/sensor_overlay/plant_sensor_overlay.png` |
| real_rcou_startvel_local_xy_dvl_z_heave240_10s | 0.015 | 0.015 | 24.248 | 20.699 | 0.873 | 0.025 | 218 | -1.836 | 0.002 | `/Users/kanghyunmin/Desktop/uuv_sim/UUV-HAN/outputs/plant_contract_structure_v65_20260615/real_rcou_startvel_local_xy_dvl_z_heave240_10s/sensor_overlay/plant_sensor_overlay.png` |
| real_rcou_startvel_local_xy_dvl_z_vhist2s_10s | 0.013 | 0.013 | 23.961 | 20.455 | 0.869 | 0.024 | 218 | -1.842 | 0.005 | `/Users/kanghyunmin/Desktop/uuv_sim/UUV-HAN/outputs/plant_contract_structure_v65_20260615/real_rcou_startvel_local_xy_dvl_z_vhist2s_10s/sensor_overlay/plant_sensor_overlay.png` |
| real_rcou_startvel_local_xy_pose_z_fd1p0_10s | 0.014 | 0.014 | 23.559 | 20.044 | 0.867 | 0.023 | 216 | -1.844 | 0.000 | `/Users/kanghyunmin/Desktop/uuv_sim/UUV-HAN/outputs/plant_contract_structure_v65_20260615/real_rcou_startvel_local_xy_pose_z_fd1p0_10s/sensor_overlay/plant_sensor_overlay.png` |
| real_rcou_startvel_local_xy_pose_z_fd1p0_heave240_retry_10s | 0.015 | 0.015 | 24.723 | 21.058 | 0.873 | 0.025 | 221 | -1.827 | 0.000 | `/Users/kanghyunmin/Desktop/uuv_sim/UUV-HAN/outputs/plant_contract_structure_v65_20260615/real_rcou_startvel_local_xy_pose_z_fd1p0_heave240_retry_10s/sensor_overlay/plant_sensor_overlay.png` |

## Interpretation Rules

- `input_ok=True` only proves that the replayed PWM CSV reached the plant input comparison point.
- `high-rate history=False` means the source is low-rate controller output telemetry, not proven ESC input history.
- `hydro gate=False` means the case must not be promoted as a hydrodynamic/HAN/CFD tuning success.
- `dynamic_start=False` means start-state velocity or estimator-derived state is not fully contracted.
- A good yaw angle ratio with poor gyro/DVL/depth correlation is still a coupled plant mismatch, not a yaw-only fix.
