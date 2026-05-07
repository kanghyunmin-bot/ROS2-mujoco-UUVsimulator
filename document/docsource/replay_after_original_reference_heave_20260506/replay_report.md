# 2026-04-01 Real RC Replay vs MuJoCo

This report replays real `/mavros/rc/override` or `/mavros/rc/out` commands through the local MuJoCo model without modifying ArduPilot.
Metrics are computed after a 5 s warmup crop. Positive/negative best lag is selected by maximum absolute correlation.

## bag_2026-04-01_20-08-11

Real duration: `471.56s`, RC override samples: `46615`, RC out samples: `939`, DVL samples: `4219`, IMU samples: `9426`, depth samples: `943`.

| replay config | DVL x | DVL y | DVL z | gyro z | depth | depth rate | contact frac |
|---|---|---|---|---|---|---|---:|
| `current_joy_node_rc_override` | rmse=0.516, corr=0.451, lag=+1.90s, gain=0.158 | rmse=0.111, corr=0.325, lag=-0.15s, gain=0.151 | rmse=0.255, corr=-0.24, lag=+1.90s, gain=-0.0581 | rmse=0.411, corr=0.68, lag=+0.10s, gain=0.445 | rmse=0.703, corr=0.355, lag=+1.45s, gain=0.191 | rmse=0.11, corr=0.306, lag=+0.45s, gain=0.164 | 0.185 |
| `current_rc_override` | rmse=0.5, corr=0.44, lag=+2.00s, gain=0.155 | rmse=0.111, corr=0.327, lag=-0.05s, gain=0.154 | rmse=0.251, corr=-0.239, lag=+1.80s, gain=-0.0586 | rmse=0.408, corr=0.677, lag=+0.10s, gain=0.447 | rmse=0.699, corr=0.361, lag=+1.40s, gain=0.195 | rmse=0.109, corr=0.305, lag=+0.45s, gain=0.164 | 0.195 |
| `current_rc_out` | rmse=0.332, corr=0.126, lag=+0.85s, gain=0.0698 | rmse=0.124, corr=-0.278, lag=+0.30s, gain=-0.148 | rmse=0.171, corr=-0.599, lag=+0.20s, gain=-0.244 | rmse=0.355, corr=0.598, lag=+0.15s, gain=0.562 | rmse=0.825, corr=0.382, lag=+0.45s, gain=0.248 | rmse=0.12, corr=0.188, lag=+0.60s, gain=0.0974 | 0.480 |

### Main Per-Config Diagnostics

#### `current_joy_node_rc_override`

- path length real/sim: `530` / `59.5` m; final delta real/sim: `[27.50347111650438, 51.808753251763825, 0.18401672044440054]` / `[-1.298773392680239, 3.894106658046116, -0.9554865033415874]`.
- static pressure vs sim pressure: rmse=3.28e+03, corr=0.355, lag=+1.45s, gain=0.191; atm pressure vs sim pressure: rmse=3.52e+03, corr=n/a, lag=-1.90s.
- sim sensor-vs-truth check: DVL y corrected `rmse=0.00331, corr=0.999`, DVL y raw `rmse=0.215, corr=-0.988`, Bar30 depth vs base depth `rmse=0.0566, corr=0.996`.
- qfrc passive rms `84.2`, actuator rms `174`, qacc rms `485`.
- thruster force p95 N by motor: `{'ver_lf': 21.506187529015765, 'ver_lr': 21.506187529015765, 'ver_rf': 21.506187529015765, 'ver_rr': 21.506187529015765, 'yaw_lf': 169.73955015830117, 'yaw_lr': 1.6175979060360468, 'yaw_rf': 169.73955015830117, 'yaw_rr': 0.0}`.

#### `current_rc_override`

- path length real/sim: `530` / `57.7` m; final delta real/sim: `[27.50347111650438, 51.808753251763825, 0.18401672044440054]` / `[-1.254584986630043, 4.4037830493358285, -0.9555043326875188]`.
- static pressure vs sim pressure: rmse=3.28e+03, corr=0.361, lag=+1.40s, gain=0.195; atm pressure vs sim pressure: rmse=3.51e+03, corr=n/a, lag=-1.70s.
- sim sensor-vs-truth check: DVL y corrected `rmse=0.00326, corr=0.999`, DVL y raw `rmse=0.214, corr=-0.988`, Bar30 depth vs base depth `rmse=0.0564, corr=0.996`.
- qfrc passive rms `82.4`, actuator rms `173`, qacc rms `480`.
- thruster force p95 N by motor: `{'ver_lf': 21.651566384937926, 'ver_lr': 21.651566384937926, 'ver_rf': 21.651566384937926, 'ver_rr': 21.651566384937926, 'yaw_lf': 169.73955015830117, 'yaw_lr': 1.7095301698706036, 'yaw_rf': 169.73955015830117, 'yaw_rr': 0.0}`.

#### `current_rc_out`

- path length real/sim: `530` / `32.2` m; final delta real/sim: `[27.50347111650438, 51.808753251763825, 0.18401672044440054]` / `[-5.466982138656246, 5.194409275619769, -0.9155885424680958]`.
- static pressure vs sim pressure: rmse=2.75e+03, corr=0.382, lag=+0.45s, gain=0.248; atm pressure vs sim pressure: rmse=2.82e+03, corr=n/a, lag=-2.00s.
- sim sensor-vs-truth check: DVL y corrected `rmse=0.00814, corr=0.996`, DVL y raw `rmse=0.177, corr=-0.909`, Bar30 depth vs base depth `rmse=0.0574, corr=0.999`.
- qfrc passive rms `38.7`, actuator rms `132`, qacc rms `211`.
- thruster force p95 N by motor: `{'ver_lf': 34.55830349569312, 'ver_lr': 24.710875337335494, 'ver_rf': 63.71351803663328, 'ver_rr': 39.86522673304332, 'yaw_lf': 155.56329870556948, 'yaw_lr': 10.652589041990122, 'yaw_rf': 105.92571551413396, 'yaw_rr': 0.0}`.

## Interpretation Checklist

- `/mavros/rc/override` replay tests the simulator's current ROS2 bridge command path.
- `/mavros/rc/out` replay is closer to motor-output replay, but the real topic is only about 2 Hz in these bags, so fast motor transients are lost.
- DVL/gyro comparisons are the most physically meaningful in these bags. Heave/depth is confounded by ALT_HOLD and weak ch3 isolation.
- MuJoCo built-in ellipsoid fluid force is not separately exposed as one named term; it is visible indirectly through passive/generalized dynamics and the resulting response.
