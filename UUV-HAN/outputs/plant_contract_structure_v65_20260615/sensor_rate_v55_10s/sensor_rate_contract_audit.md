# Sensor Rate Contract Audit

- real bag: `/Users/kanghyunmin/Desktop/uuv_sim/90s/rosbag/mp4_start_0s_90s_all_topics_0.db3`
- real CSV: `/Users/kanghyunmin/Desktop/uuv_sim/uuv_mujoco/v2.2/debug/controller_parity_412/real_90s_feedback_source_servo_20260601/real_controller_feedback_20hz.csv`
- sim CSV: `/Users/kanghyunmin/Desktop/uuv_sim/UUV-HAN/outputs/plant_structural_forward_seed_yawpair_v55_20260615/forward_seed_1p0_yawpair2p4_10s/sim_sensor_replay.csv`
- window: `0.350s -> 10.350s`

## Native ROS Bag Topic Rates

| topic | count | mean Hz | median dt | p95 dt | max dt |
| --- | ---: | ---: | ---: | ---: | ---: |
| `/mavros/rc/out` | 180 | 2.000 | 0.4999 | 0.5048 | 0.5088 |
| `/mavros/rc/override` | 10515 | 116.830 | 0.0091 | 0.0222 | 0.0373 |
| `/mavros/imu/static_pressure` | 180 | 2.000 | 0.5000 | 0.5054 | 0.5150 |
| `/mavros/imu/atm_pressure` | 900 | 10.001 | 0.1000 | 0.1032 | 0.1233 |
| `/mavros/imu/data` | 1800 | 20.001 | 0.0500 | 0.0518 | 0.0699 |
| `/mavros/imu/data_raw` | 180 | 2.000 | 0.4999 | 0.5055 | 0.5095 |
| `/depth/pose` | 180 | 2.000 | 0.5000 | 0.5058 | 0.5153 |
| `/mavros/local_position/pose` | 270 | 3.003 | 0.3328 | 0.3368 | 0.3545 |
| `/mavros/local_position/velocity_local` | 270 | 3.003 | 0.3328 | 0.3368 | 0.3547 |
| `/mavros/local_position/velocity_body` | 270 | 3.003 | 0.3328 | 0.3368 | 0.3547 |
| `/dvl/twist` | 873 | 9.704 | 0.1014 | 0.1283 | 0.2446 |
| `/dvl/position` | 401 | 4.452 | 0.2220 | 0.2601 | 0.3209 |
| `/uas1/mavlink_source` | 11348 | 126.132 | 0.0003 | 0.0491 | 0.0683 |
| `/uas1/mavlink_sink` | 12477 | 138.631 | 0.0046 | 0.0200 | 0.0492 |

## CSV Effective Update Rates

| source | group | samples | sample Hz | events | event Hz | median event dt | unchanged ratio |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: |
| real | `rc_out` | 201 | 20.000 | 21 | 2.000 | 0.5000 | 0.896 |
| real | `static_pressure` | 201 | 20.000 | 19 | 1.800 | 0.5000 | 0.905 |
| real | `atm_pressure` | 201 | 20.000 | 1 | 0.000 | nan | 0.995 |
| real | `depth_pose` | 201 | 20.000 | 19 | 1.800 | 0.5000 | 0.905 |
| real | `local_pose` | 201 | 20.000 | 31 | 3.030 | 0.3500 | 0.846 |
| real | `local_vel` | 201 | 20.000 | 31 | 3.030 | 0.3500 | 0.846 |
| real | `local_vel_body` | 0 | 0.000 | 0 | 0.000 | nan | 0.000 |
| real | `dvl_twist` | 201 | 20.000 | 123 | 12.261 | 0.1000 | 0.388 |
| real | `dvl_position` | 201 | 20.000 | 43 | 4.221 | 0.2500 | 0.786 |
| real | `imu` | 201 | 20.000 | 201 | 20.000 | 0.0500 | 0.000 |
| real | `imu_raw` | 0 | 0.000 | 0 | 0.000 | nan | 0.000 |
| real | `vision_delta` | 201 | 20.000 | 123 | 12.261 | 0.1000 | 0.388 |
| sim | `rcout_input` | 501 | 50.000 | 21 | 2.092 | 0.5000 | 0.958 |
| sim | `bar30_pressure` | 501 | 50.000 | 22 | 2.100 | 0.4762 | 0.956 |
| sim | `static_pressure` | 501 | 50.000 | 21 | 2.000 | 0.5000 | 0.958 |
| sim | `atm_pressure` | 501 | 50.000 | 101 | 10.000 | 0.1000 | 0.798 |
| sim | `depth` | 501 | 50.000 | 22 | 2.100 | 0.4762 | 0.956 |
| sim | `depth_pose` | 501 | 50.000 | 21 | 2.000 | 0.5000 | 0.958 |
| sim | `local_pose` | 501 | 50.000 | 31 | 3.000 | 0.3333 | 0.938 |
| sim | `local_vel` | 501 | 50.000 | 31 | 3.000 | 0.3333 | 0.938 |
| sim | `local_vel_body` | 501 | 50.000 | 31 | 3.000 | 0.3333 | 0.938 |
| sim | `dvl_twist` | 501 | 50.000 | 101 | 10.000 | 0.1000 | 0.798 |
| sim | `dvl_position` | 501 | 50.000 | 46 | 4.500 | 0.2222 | 0.908 |
| sim | `imu` | 501 | 50.000 | 201 | 20.000 | 0.0500 | 0.599 |
| sim | `imu_raw` | 501 | 50.000 | 21 | 2.000 | 0.5000 | 0.958 |

## Native-to-Sim Rate Verdict

| contract | expected Hz | real topic | real native Hz | sim group | sim sequence/event Hz | sim/real | verdict |
| --- | ---: | --- | ---: | --- | ---: | ---: | --- |
| `rc_out` | 2.000 | `/mavros/rc/out` | 2.000 | `rcout_input` | 2.092 | 1.046 | pass |
| `static_pressure` | 2.000 | `/mavros/imu/static_pressure` | 2.000 | `static_pressure` | 2.000 | 1.000 | pass |
| `atm_pressure` | 10.000 | `/mavros/imu/atm_pressure` | 10.001 | `atm_pressure` | 10.000 | 1.000 | pass |
| `depth_pose` | 2.000 | `/depth/pose` | 2.000 | `depth_pose` | 2.000 | 1.000 | pass |
| `local_pose` | 3.000 | `/mavros/local_position/pose` | 3.003 | `local_pose` | 3.000 | 0.999 | pass |
| `local_vel` | 3.000 | `/mavros/local_position/velocity_local` | 3.003 | `local_vel` | 3.000 | 0.999 | pass |
| `local_vel_body` | 3.000 | `/mavros/local_position/velocity_body` | 3.003 | `local_vel_body` | 3.000 | 0.999 | pass |
| `dvl_twist` | 10.000 | `/dvl/twist` | 9.704 | `dvl_twist` | 10.000 | 1.030 | pass |
| `dvl_position` | 4.450 | `/dvl/position` | 4.452 | `dvl_position` | 4.500 | 1.011 | pass |
| `imu` | 20.000 | `/mavros/imu/data` | 20.001 | `imu` | 20.000 | 1.000 | pass |
| `imu_raw` | 2.000 | `/mavros/imu/data_raw` | 2.000 | `imu_raw` | 2.000 | 1.000 | pass |
