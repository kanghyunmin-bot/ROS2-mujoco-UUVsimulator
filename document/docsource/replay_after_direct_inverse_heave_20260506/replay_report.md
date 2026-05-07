# 2026-04-01 Real RC Replay vs MuJoCo

This report replays real `/mavros/rc/override` or `/mavros/rc/out` commands through the local MuJoCo model without modifying ArduPilot.
Metrics are computed after a 5 s warmup crop. Positive/negative best lag is selected by maximum absolute correlation.

## bag_2026-04-01_20-08-11

Real duration: `471.56s`, RC override samples: `46615`, RC out samples: `939`, DVL samples: `4219`, IMU samples: `9426`, depth samples: `943`.

| replay config | DVL x | DVL y | DVL z | gyro z | depth | depth rate | contact frac |
|---|---|---|---|---|---|---|---:|
| `current_joy_node_rc_override` | rmse=0.552, corr=0.382, lag=+1.30s, gain=0.126 | rmse=0.104, corr=0.29, lag=-0.30s, gain=0.146 | rmse=0.24, corr=-0.315, lag=+0.25s, gain=-0.0863 | rmse=0.418, corr=0.696, lag=+0.10s, gain=0.44 | rmse=0.594, corr=0.41, lag=+1.25s, gain=0.189 | rmse=0.132, corr=0.405, lag=+0.35s, gain=0.169 | 0.169 |
| `current_rc_override` | rmse=0.534, corr=0.448, lag=+0.95s, gain=0.155 | rmse=0.117, corr=0.303, lag=-0.20s, gain=0.132 | rmse=0.268, corr=-0.211, lag=+0.30s, gain=-0.0476 | rmse=0.416, corr=0.71, lag=+0.10s, gain=0.445 | rmse=0.601, corr=0.406, lag=+1.30s, gain=0.185 | rmse=0.129, corr=0.413, lag=+0.35s, gain=0.175 | 0.150 |
| `current_rc_out` | rmse=0.366, corr=-0.228, lag=-2.00s, gain=-0.137 | rmse=0.106, corr=-0.139, lag=+1.90s, gain=-0.0872 | rmse=0.177, corr=-0.69, lag=+0.15s, gain=-0.276 | rmse=0.378, corr=0.533, lag=+0.10s, gain=0.505 | rmse=0.704, corr=0.557, lag=-0.05s, gain=0.27 | rmse=0.107, corr=0.306, lag=+0.45s, gain=0.17 | 0.476 |

### Main Per-Config Diagnostics

#### `current_joy_node_rc_override`

- path length real/sim: `530` / `60.3` m; final delta real/sim: `[27.50347111650438, 51.808753251763825, 0.18401672044440054]` / `[-0.6575808014793153, 5.171021849556034, -0.9586658516823023]`.
- static pressure vs sim pressure: rmse=3.46e+03, corr=0.41, lag=+1.25s, gain=0.189; atm pressure vs sim pressure: rmse=3.98e+03, corr=n/a, lag=+1.15s.
- sim sensor-vs-truth check: DVL y corrected `rmse=0.00346, corr=0.999`, DVL y raw `rmse=0.203, corr=-0.986`, Bar30 depth vs base depth `rmse=0.0563, corr=0.997`.
- qfrc passive rms `87.7`, actuator rms `174`, qacc rms `409`.
- thruster force p95 N by motor: `{'ver_lf': 21.506187529015765, 'ver_lr': 21.506187529015765, 'ver_rf': 21.506187529015765, 'ver_rr': 21.506187529015765, 'yaw_lf': 169.73955015830117, 'yaw_lr': 1.6175979060360468, 'yaw_rf': 169.73955015830117, 'yaw_rr': 0.0}`.

#### `current_rc_override`

- path length real/sim: `530` / `62.9` m; final delta real/sim: `[27.50347111650438, 51.808753251763825, 0.18401672044440054]` / `[-4.206423645687456, 0.402140066420569, -0.9554999396160295]`.
- static pressure vs sim pressure: rmse=3.49e+03, corr=0.406, lag=+1.30s, gain=0.185; atm pressure vs sim pressure: rmse=4.13e+03, corr=n/a, lag=+1.70s.
- sim sensor-vs-truth check: DVL y corrected `rmse=0.00325, corr=1`, DVL y raw `rmse=0.217, corr=-0.989`, Bar30 depth vs base depth `rmse=0.0563, corr=0.997`.
- qfrc passive rms `86.2`, actuator rms `173`, qacc rms `480`.
- thruster force p95 N by motor: `{'ver_lf': 21.651566384937926, 'ver_lr': 21.651566384937926, 'ver_rf': 21.651566384937926, 'ver_rr': 21.651566384937926, 'yaw_lf': 169.73955015830117, 'yaw_lr': 1.7095301698706036, 'yaw_rf': 169.73955015830117, 'yaw_rr': 0.0}`.

#### `current_rc_out`

- path length real/sim: `530` / `26.8` m; final delta real/sim: `[27.50347111650438, 51.808753251763825, 0.18401672044440054]` / `[-10.09381215039821, 5.213485624804121, -0.8946080925798485]`.
- static pressure vs sim pressure: rmse=3e+03, corr=0.557, lag=-0.05s, gain=0.27; atm pressure vs sim pressure: rmse=3.78e+03, corr=n/a, lag=-2.00s.
- sim sensor-vs-truth check: DVL y corrected `rmse=0.0074, corr=0.996`, DVL y raw `rmse=0.159, corr=-0.909`, Bar30 depth vs base depth `rmse=0.0575, corr=0.999`.
- qfrc passive rms `39.3`, actuator rms `132`, qacc rms `169`.
- thruster force p95 N by motor: `{'ver_lf': 34.55830349569312, 'ver_lr': 24.710875337335494, 'ver_rf': 63.71351803663328, 'ver_rr': 39.86522673304332, 'yaw_lf': 155.56329870556948, 'yaw_lr': 10.652589041990122, 'yaw_rf': 105.92571551413396, 'yaw_rr': 0.0}`.

## Interpretation Checklist

- `/mavros/rc/override` replay tests the simulator's current ROS2 bridge command path.
- `/mavros/rc/out` replay is closer to motor-output replay, but the real topic is only about 2 Hz in these bags, so fast motor transients are lost.
- DVL/gyro comparisons are the most physically meaningful in these bags. Heave/depth is confounded by ALT_HOLD and weak ch3 isolation.
- MuJoCo built-in ellipsoid fluid force is not separately exposed as one named term; it is visible indirectly through passive/generalized dynamics and the resulting response.
