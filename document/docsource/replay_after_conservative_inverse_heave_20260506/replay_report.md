# 2026-04-01 Real RC Replay vs MuJoCo

This report replays real `/mavros/rc/override` or `/mavros/rc/out` commands through the local MuJoCo model without modifying ArduPilot.
Metrics are computed after a 5 s warmup crop. Positive/negative best lag is selected by maximum absolute correlation.

## bag_2026-04-01_20-08-11

Real duration: `471.56s`, RC override samples: `46615`, RC out samples: `939`, DVL samples: `4219`, IMU samples: `9426`, depth samples: `943`.

| replay config | DVL x | DVL y | DVL z | gyro z | depth | depth rate | contact frac |
|---|---|---|---|---|---|---|---:|
| `current_joy_node_rc_override` | rmse=0.521, corr=0.448, lag=+2.00s, gain=0.156 | rmse=0.113, corr=0.338, lag=-0.05s, gain=0.154 | rmse=0.257, corr=-0.207, lag=+1.85s, gain=-0.0489 | rmse=0.412, corr=0.687, lag=+0.10s, gain=0.444 | rmse=0.658, corr=0.374, lag=+1.65s, gain=0.176 | rmse=0.134, corr=0.361, lag=+0.40s, gain=0.152 | 0.171 |
| `current_rc_override` | rmse=0.501, corr=0.439, lag=+2.00s, gain=0.155 | rmse=0.112, corr=0.355, lag=-0.05s, gain=0.163 | rmse=0.256, corr=-0.213, lag=+1.85s, gain=-0.0505 | rmse=0.405, corr=0.686, lag=+0.10s, gain=0.451 | rmse=0.656, corr=0.377, lag=+1.60s, gain=0.179 | rmse=0.134, corr=0.361, lag=+0.40s, gain=0.151 | 0.190 |
| `current_rc_out` | rmse=0.345, corr=-0.206, lag=-2.00s, gain=-0.133 | rmse=0.115, corr=-0.265, lag=+0.20s, gain=-0.158 | rmse=0.173, corr=-0.646, lag=+0.15s, gain=-0.264 | rmse=0.363, corr=0.541, lag=+0.15s, gain=0.551 | rmse=0.795, corr=0.45, lag=-0.35s, gain=0.257 | rmse=0.119, corr=0.216, lag=+0.50s, gain=0.111 | 0.501 |

### Main Per-Config Diagnostics

#### `current_joy_node_rc_override`

- path length real/sim: `530` / `60.7` m; final delta real/sim: `[27.50347111650438, 51.808753251763825, 0.18401672044440054]` / `[-2.3990386293408923, 3.993163078023074, -0.9555028504874137]`.
- static pressure vs sim pressure: rmse=3.54e+03, corr=0.374, lag=+1.65s, gain=0.176; atm pressure vs sim pressure: rmse=4.08e+03, corr=n/a, lag=+0.15s.
- sim sensor-vs-truth check: DVL y corrected `rmse=0.0033, corr=0.999`, DVL y raw `rmse=0.216, corr=-0.988`, Bar30 depth vs base depth `rmse=0.0564, corr=0.997`.
- qfrc passive rms `85.4`, actuator rms `174`, qacc rms `480`.
- thruster force p95 N by motor: `{'ver_lf': 21.506187529015765, 'ver_lr': 21.506187529015765, 'ver_rf': 21.506187529015765, 'ver_rr': 21.506187529015765, 'yaw_lf': 169.73955015830117, 'yaw_lr': 1.6175979060360468, 'yaw_rf': 169.73955015830117, 'yaw_rr': 0.0}`.

#### `current_rc_override`

- path length real/sim: `530` / `58.5` m; final delta real/sim: `[27.50347111650438, 51.808753251763825, 0.18401672044440054]` / `[-1.7452151907992974, 4.326676117551706, -0.955793677111572]`.
- static pressure vs sim pressure: rmse=3.54e+03, corr=0.377, lag=+1.60s, gain=0.179; atm pressure vs sim pressure: rmse=4.06e+03, corr=n/a, lag=-0.45s.
- sim sensor-vs-truth check: DVL y corrected `rmse=0.00336, corr=0.999`, DVL y raw `rmse=0.216, corr=-0.988`, Bar30 depth vs base depth `rmse=0.0562, corr=0.997`.
- qfrc passive rms `83.4`, actuator rms `173`, qacc rms `461`.
- thruster force p95 N by motor: `{'ver_lf': 21.651566384937926, 'ver_lr': 21.651566384937926, 'ver_rf': 21.651566384937926, 'ver_rr': 21.651566384937926, 'yaw_lf': 169.73955015830117, 'yaw_lr': 1.7095301698706036, 'yaw_rf': 169.73955015830117, 'yaw_rr': 0.0}`.

#### `current_rc_out`

- path length real/sim: `530` / `27.6` m; final delta real/sim: `[27.50347111650438, 51.808753251763825, 0.18401672044440054]` / `[-4.781171517477886, 5.204025596059213, -0.9043468436520883]`.
- static pressure vs sim pressure: rmse=2.81e+03, corr=0.45, lag=-0.35s, gain=0.257; atm pressure vs sim pressure: rmse=3.33e+03, corr=n/a, lag=-2.00s.
- sim sensor-vs-truth check: DVL y corrected `rmse=0.00796, corr=0.996`, DVL y raw `rmse=0.166, corr=-0.901`, Bar30 depth vs base depth `rmse=0.0574, corr=0.999`.
- qfrc passive rms `36.1`, actuator rms `132`, qacc rms `204`.
- thruster force p95 N by motor: `{'ver_lf': 34.55830349569312, 'ver_lr': 24.710875337335494, 'ver_rf': 63.71351803663328, 'ver_rr': 39.86522673304332, 'yaw_lf': 155.56329870556948, 'yaw_lr': 10.652589041990122, 'yaw_rf': 105.92571551413396, 'yaw_rr': 0.0}`.

## Interpretation Checklist

- `/mavros/rc/override` replay tests the simulator's current ROS2 bridge command path.
- `/mavros/rc/out` replay is closer to motor-output replay, but the real topic is only about 2 Hz in these bags, so fast motor transients are lost.
- DVL/gyro comparisons are the most physically meaningful in these bags. Heave/depth is confounded by ALT_HOLD and weak ch3 isolation.
- MuJoCo built-in ellipsoid fluid force is not separately exposed as one named term; it is visible indirectly through passive/generalized dynamics and the resulting response.
