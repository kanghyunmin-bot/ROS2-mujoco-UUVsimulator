# Actuator Response Contract

- real_csv: `uuv_mujoco/v2.2/debug/controller_parity_412/real_90s_feedback_source_servo_20260601/real_controller_feedback_20hz.csv`
- sim_csv: `UUV-HAN/outputs/plant_structural_forward_seed_yawpair_v55_20260615/forward_seed_1p0_yawpair2p4_10s/sim_sensor_replay.csv`
- thruster_debug_csv: `UUV-HAN/outputs/plant_structural_forward_seed_yawpair_v55_20260615/forward_seed_1p0_yawpair2p4_10s/thruster_debug.csv`
- plant_input_csv: `uuv_mujoco/v2.2/debug/controller_parity_412/real_90s_feedback_source_servo_20260601/real_controller_feedback_20hz.csv`
- debug_time_offset_s: `32.142`
- active_samples: `181`
- input median hold: `0.500 s` (`2.000 Hz`)
- ok_for_high_rate_plant_identification: `False`

| axis | kind | wrench_rms | real_deriv_rms | sim_deriv_rms | corr wrench-real | corr wrench-sim | lag real s | lag sim s | sim/real deriv ratio |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| roll | angular_x | 0.876 | 0.098 | 0.019 | -0.020 | 0.099 | -0.400 | -0.750 | 0.197 |
| pitch | angular_y | 3.327 | 0.115 | 0.144 | 0.032 | 0.029 | -0.550 | 0.750 | -1.256 |
| yaw | angular_z | 3.272 | 0.905 | 0.522 | -0.227 | 0.108 | -1.000 | -1.000 | 0.576 |
| forward | linear_x | 44.808 | 0.145 | 0.023 | -0.060 | 0.220 | 0.750 | -1.000 | 0.160 |
| lateral | linear_y | 11.761 | 0.105 | 0.013 | -0.026 | 0.477 | -1.000 | -1.000 | 0.124 |
| heave | linear_z | 0.155 | 0.044 | 0.004 | -0.266 | -0.037 | -1.000 | -1.000 | 0.088 |

## Input Observability

Plant replay identification requires actuator history at the motor-update layer. High-rate json_servo* input can support contract diagnostics; low-rate SERVO_OUTPUT_RAW telemetry cannot.

| channel | changes | median hold s | min hold s | max hold s | first values |
| ---: | ---: | ---: | ---: | ---: | --- |
| 1 | 20 | 0.500 | 0.500 | 0.500 | `[1330, 1333, 1332, 1351, 1380, 1413, 1477, 1239]` |
| 2 | 20 | 0.500 | 0.500 | 0.500 | `[1311, 1314, 1316, 1302, 1271, 1241, 1172, 1406]` |
| 3 | 20 | 0.500 | 0.500 | 0.500 | `[1286, 1288, 1286, 1304, 1335, 1364, 1429, 1185]` |
| 4 | 19 | 0.500 | 0.500 | 1.000 | `[1355, 1359, 1361, 1347, 1316, 1287, 1221, 1457]` |
| 5 | 20 | 0.500 | 0.500 | 0.500 | `[1566, 1567, 1569, 1565, 1564, 1559, 1527, 1520]` |
| 6 | 20 | 0.500 | 0.500 | 0.500 | `[1458, 1461, 1457, 1453, 1455, 1445, 1451, 1489]` |
| 7 | 19 | 0.500 | 0.500 | 1.000 | `[1579, 1573, 1572, 1577, 1580, 1586, 1587, 1546]` |
| 8 | 19 | 0.500 | 0.500 | 1.000 | `[1395, 1396, 1400, 1403, 1399, 1408, 1434, 1444]` |

## Sim Qacc Self Check

| target | expected qacc | corr expected | best qacc | corr best |
| --- | ---: | ---: | ---: | ---: |
| lin_x | 0 | 0.999 | 0 | 0.999 |
| lin_y | 1 | 0.997 | 1 | 0.997 |
| lin_z | 2 | 0.985 | 2 | 0.985 |
| ang_x | 3 | 0.927 | 3 | 0.927 |
| ang_y | 4 | 0.996 | 4 | 0.996 |
| ang_z | 5 | 0.975 | 5 | 0.975 |

## Interpretation

High wrench-vs-sim-derivative correlation means the simulated plant is internally responding to the applied actuator wrench. Low wrench-vs-real-derivative correlation or low sim/real derivative RMS ratio points to actuator effectiveness/timing/history contract mismatch before hydrodynamic coefficient fitting.

This audit does not tune coefficients and must not override the deterministic ArduSub motor-order contract.
