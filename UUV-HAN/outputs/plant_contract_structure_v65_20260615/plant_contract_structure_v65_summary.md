# v65 plant replay contract structure summary

Date: 2026-06-15

## Decision

The current 90s real-RCOU plant replay is not a contract-valid HAN/CFD target
for plant parameter promotion.

It remains useful as a diagnostic replay, but the evidence says the remaining
Bar30/DVL_z mismatch is structural before it is hydrodynamic:

- the bag has no plant-grade high-rate actuator-output history;
- the replay input is low-rate held SERVO_OUTPUT_RAW telemetry;
- the simulated vertical time response moves in the wrong early DVL_z direction
  for every v55/v64 vertical-physics candidate;
- DVL_z is not a one-line sign flip problem, because real and sim both show
  DVL_z opposing MAVROS local/body z.

## Evidence artifacts

| artifact | path | key result |
|---|---|---|
| Input layer audit | `input_layers/real_90s_input_layer_audit.md` | `blocked_no_measured_high_rate_actuator_history` |
| Bag actuator source audit | `bag_actuator_sources/real_bag_actuator_source_audit.md` | no plant-grade actuator history; SERVO_OUTPUT_RAW is about 1.99Hz |
| Actuator response audit | `actuator_response_v55_10s/actuator_response_contract.md` | RCOU input median hold is 0.500s; heave sim/real derivative ratio is 0.088 |
| Vertical sensor contract | `vertical_sensor_contract_v55_v64_10s/vertical_sensor_contract.md` | `structural_vertical_time_response_mismatch` |
| Sensor rate contract | `sensor_rate_v55_10s/sensor_rate_contract_audit.md` | sensor publish/hold rates match the real bag contract, so rate mismatch alone is not the main issue |
| Reconstructed-input vertical gain A/B | `vertical_sensor_contract_highrate_vgain_ab/vertical_sensor_contract.md` | high-rate diagnostic input plus lower vertical gain improves DVL_z direction but remains worse than v55 overall |

## Input layer findings

From `real_90s_input_layer_audit`:

- plant replay input layers: `real_mavros_rc_out`, `real_mavlink_source_servo`,
  `event_mavlink_source_servo`
- controller reconstruction layers: `real_rc_override`,
  `real_mavlink_sink_rc_override`, `event_mavlink_sink_rc_override`
- HAN target export layers: none
- richest controller reconstruction layer: `event_mavlink_sink_rc_override`
- measured high-rate actuator history: false

This means the high-rate data in the bag is upstream controller input, not
downstream actuator output.

## Bag actuator source findings

The 90s bag is:

- `90s/rosbag/mp4_start_0s_90s_all_topics_0.db3`

Important rates:

| topic/message | count | rate | interpretation |
|---|---:|---:|---|
| `/uas1/mavlink_sink` | 12,477 | 138.631Hz | high-rate MAVLink input transport |
| `/uas1/mavlink_source` | 11,348 | 126.132Hz | high-rate transport only; decoded output message must be checked |
| `/mavros/rc/override` | 10,515 | 116.830Hz | high-rate controller input |
| `/mavros/rc/out` | 180 | 2.000Hz | low-rate controller-output telemetry |
| decoded `SERVO_OUTPUT_RAW` | 179 | 1.989Hz | low-rate output telemetry |
| decoded `ACTUATOR_OUTPUT_STATUS` | 0 | n/a | unavailable |
| decoded ESC telemetry | 0 | n/a | unavailable |

## Vertical sensor findings

First 2s window: real time 0.35s to 2.35s.

| case | depth_slope_real | depth_slope_sim | dvlz_slope_real | dvlz_slope_sim | DVL_z direction match |
|---|---:|---:|---:|---:|---:|
| v55_10s | +0.000337 | -0.002981 | -0.006066 | +0.002085 | false |
| v64_buoy100 | +0.000337 | +0.004439 | -0.006066 | +0.004880 | false |
| v64_vlift0 | +0.000337 | +0.001824 | -0.006066 | +0.000042 | false |
| v64_heave240 | +0.000337 | -0.000576 | -0.006066 | +0.002526 | false |
| v64_vlift13000 | +0.000337 | -0.006290 | -0.006066 | +0.002904 | false |
| v64_yawheave0 | +0.000337 | -0.003033 | -0.006066 | +0.002170 | false |

DVL_z sign/frame check:

| source | corr(local_z,dvl_z) | corr(local_z,-dvl_z) |
|---|---:|---:|
| real | -0.5803 | +0.5803 |
| v55_10s | -0.7950 | +0.7950 |
| v64_yawheave0 | -0.8004 | +0.8004 |

The sign relationship is consistent; the time response is not.

## Next physical path

Do not promote new HAN/CFD targets from low-rate RCOU interpolation.

The next useful work is:

1. reconstruct controller-side SITL RCOU from high-rate
   `event_mavlink_sink_rc_override` and native VISION_POSITION_DELTA for
   controller parity only;
2. compare that reconstructed SITL RCOU against the real low-rate
   SERVO_OUTPUT_RAW observation layer at the same event phase;
3. if the reconstructed controller output explains the observed low-rate RCOU,
   use it as a diagnostic plant input with explicit provenance, not as measured
   actuator history;
4. capture or add a true measured actuator-output topic for promotion-grade
   HAN/CFD fitting.

Until item 4 exists, plant physical parameter changes can be tested as
diagnostics, but they should not be reported as contract-valid promotion.

## Diagnostic high-rate reconstruction A/B

The diagnostic high-rate input source used for this A/B is:

- `UUV-HAN/outputs/controller_reconstruction_90s_ab_prevphase_depthpreloadp0p12_full90_isolated_20260614/sitl_controller_io.csv`

The plant sensor contract classifies it as:

- `controller_reconstructed_sitl_json_servo_diagnostic`
- high-rate actuator history: true
- measured actuator history: false
- allowed for HAN/CFD export: false

10s metric comparison:

| case | baro | dvl_x | dvl_y | dvl_z | gyro_z | yaw | mean5 |
|---|---:|---:|---:|---:|---:|---:|---:|
| v55_10s | 0.013848 | 0.048628 | 0.055979 | 0.022987 | 0.252497 | 23.300442 | 0.078788 |
| recon_highrate_orig | 0.045911 | 0.071054 | 0.084654 | 0.038533 | 0.589624 | 132.534609 | 0.165955 |
| recon_highrate_vgain0p2 | 0.023934 | 0.095271 | 0.057042 | 0.023486 | 0.276003 | 26.623752 | 0.095147 |
| recon_highrate_vgain0p25 | 0.025907 | 0.095215 | 0.056921 | 0.023618 | 0.274851 | 26.645629 | 0.095302 |

First-window vertical direction:

| case | depth_slope_sim | dvlz_slope_sim | depth_end_diff | dvlz_dir_match |
|---|---:|---:|---:|---:|
| real | +0.000337 | -0.006066 | 0.000000 | n/a |
| v55_10s | -0.002981 | +0.002085 | -0.008829 | false |
| recon_highrate_orig | +0.025925 | -0.044162 | +0.031113 | true |
| recon_highrate_vgain0p2 | -0.003381 | -0.007434 | -0.011729 | true |
| recon_highrate_vgain0p25 | -0.002176 | -0.010137 | -0.005190 | true |

Lowering vertical gain on the reconstructed input can bring the DVL_z slope
direction and magnitude closer, but it does not recover Bar30 and it badly
hurts DVL_x. This points to coupled input/controller reconstruction and
initial-condition issues, not a clean vertical force-scale promotion.

Config note after these runs:

```text
3a4f4946fb1e89d92bac604d9b6c296a21549637b5a88df5fef40f582a5a37a2  uuv_mujoco/v2.2/config/sim_profiles.json
64c76f9842d829a12c5e0f178521f0178dceb79e478a5939cb900ded6c91b821  uuv_mujoco/v2.2/config/thruster_params.json
```

Those hashes match the explicit v55 profile/thruster sources used for the
current best replay. The earlier dirty-runtime config hashes were not found in
the workspace, so future comparisons should pass `--profile-source` and
`--thruster-params-source` explicitly.

## Real vertical sensor consistency and pose-fd release velocity

The real 0.35s to 2.35s window was audited directly because the Bar30 and DVL_z
targets imply different vertical motion:

- Bar30/depth topic changes only `+0.003060m`.
- Static-pressure-derived depth also changes only `+0.003060m`.
- MAVROS local pose depth changes `+0.004557m`.
- Integrating raw `dvl_twist_z` as down velocity implies about `+0.066195m`
  depth increase over the same window.
- Integrating `local_vel_z` directly implies the opposite depth direction.

This means raw DVL_z and MAVROS local velocity z should not be treated as
interchangeable true vertical release velocity sources for the first window.
To test a more position-consistent release condition, a new diagnostic source
was added:

- `--start-velocity-source local_xy_pose_z_fd`
- `--start-velocity-pose-fd-window-s <seconds>`

This keeps local velocity x/y, but derives ENU z velocity from forward finite
difference of `local_pose_z`, then rotates the resulting world velocity into
body frame for release.

10s metric comparison:

| case | baro | dvl_x | dvl_y | dvl_z | gyro_z | yaw | mean5 |
|---|---:|---:|---:|---:|---:|---:|---:|
| v55_10s | 0.013848 | 0.048628 | 0.055979 | 0.022987 | 0.252497 | 23.300442 | 0.078788 |
| local_xy_pose_z_fd1p0 | 0.013621 | 0.048595 | 0.055969 | 0.023032 | 0.251785 | 23.559045 | 0.078600 |
| local_xy_pose_z_fd1p0_heave240 | 0.014550 | 0.048310 | 0.055953 | 0.024918 | 0.250941 | 24.723370 | 0.078934 |
| local_xy_dvl_z | 0.012771 | 0.048601 | 0.055970 | 0.023619 | 0.252895 | 23.683433 | 0.078771 |
| vhist2s_combo | 0.012763 | 0.048545 | 0.055954 | 0.023617 | 0.250214 | 23.960975 | 0.078218 |

First-window vertical direction:

| case | depth_slope_sim | dvlz_slope_sim | depth_end_diff | depth_dir_match | dvlz_dir_match |
|---|---:|---:|---:|---:|---:|
| real | +0.000337 | -0.006066 | 0.000000 | n/a | n/a |
| v55_10s | -0.002981 | +0.002085 | -0.008829 | false | false |
| local_xy_pose_z_fd1p0 | -0.002741 | +0.001248 | -0.008410 | false | false |
| local_xy_pose_z_fd1p0_heave240 | -0.000384 | +0.001657 | -0.003939 | false | false |
| local_xy_dvl_z | -0.001758 | -0.001937 | -0.006667 | false | true |

Decision:

- `local_xy_pose_z_fd1p0` improves `mean5` slightly and is a cleaner release
  velocity contract than the raw local velocity z topic, but it does not fix
  the first-window Bar30 direction.
- `local_xy_pose_z_fd1p0 + heave240` makes the early depth slope almost flat,
  but it still has the wrong Bar30 and DVL_z first-window direction and worsens
  Bar30/DVL_z/yaw RMSE versus v55.
- Keep the new source as a diagnostic tool, but do not promote it as the
  current plant profile.

Smoke/compile checks after the code change:

```text
python3 -m py_compile real_start_local_velocity.py real_start_velocity_policy.py real_start_builder.py run_plant_replay_case.py
real_start_measurement_smoke: PASS
initial_hold_pose_smoke: PASS
```

## Geometry depth source A/B

The v55 real-RCOU replay was re-run with `--geometry-depth-source bar30`.
This checks whether the early Bar30 drop comes from using the base-body depth
instead of the Bar30 sensor site depth for the start geometry.

10s metric comparison:

| case | baro | dvl_x | dvl_y | dvl_z | gyro_z | yaw | mean5 |
|---|---:|---:|---:|---:|---:|---:|---:|
| v55_10s | 0.013848 | 0.048628 | 0.055979 | 0.022987 | 0.252497 | 23.300442 | 0.078788 |
| geometry_bar30 | 0.013848 | 0.048650 | 0.055990 | 0.022988 | 0.252316 | 24.149073 | 0.078758 |

First-window vertical direction:

| case | depth_slope_sim | dvlz_slope_sim | depth_end_diff | dvlz_dir_match |
|---|---:|---:|---:|---:|
| real | +0.000337 | -0.006066 | 0.000000 | n/a |
| v55_10s | -0.002981 | +0.002085 | -0.008829 | false |
| geometry_bar30 | -0.002981 | +0.002377 | -0.008826 | false |

Changing the geometry depth source is not the fix. Bar30 depth still moves in
the wrong early direction, and DVL_z direction remains wrong.

## Release velocity source A/B

The v55 real-RCOU replay was also re-run with alternate release velocity
contracts:

- `local_xy_dvl_z`: keep local-pose x/y velocity, use DVL z velocity
- `dvl`: use DVL velocity for x/y/z
- `local_xy_dvl_z_blend0p5`: keep local-pose x/y velocity and blend z 50/50

10s metric comparison:

| case | baro | dvl_x | dvl_y | dvl_z | gyro_z | yaw | mean5 |
|---|---:|---:|---:|---:|---:|---:|---:|
| v55_10s | 0.013848 | 0.048628 | 0.055979 | 0.022987 | 0.252497 | 23.300442 | 0.078788 |
| startvel_local_xy_dvl_z | 0.012771 | 0.048601 | 0.055970 | 0.023619 | 0.252895 | 23.683433 | 0.078771 |
| startvel_dvl | 0.012990 | 0.048480 | 0.057424 | 0.023586 | 0.252014 | 24.177166 | 0.078899 |
| startvel_blend0p5 | 0.013289 | 0.048571 | 0.055955 | 0.023176 | 0.251319 | 23.311857 | 0.078462 |

First-window vertical direction:

| case | depth_slope_real | depth_slope_sim | depth_end_diff | dvlz_slope_real | dvlz_slope_sim | dvlz_mean_diff | depth_dir_match | dvlz_dir_match |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| v55_10s | +0.000337 | -0.002981 | -0.008829 | -0.006066 | +0.002085 | +0.019370 | false | false |
| startvel_local_xy_dvl_z | +0.000337 | -0.001758 | -0.006667 | -0.006066 | -0.001937 | +0.020892 | false | true |
| startvel_dvl | +0.000337 | -0.002061 | -0.007164 | -0.006066 | -0.001632 | +0.020687 | false | true |
| startvel_blend0p5 | +0.000337 | -0.002377 | -0.007771 | -0.006066 | -0.000011 | +0.020166 | false | true |

Release velocity changes can flip the first-window DVL_z direction into the
same direction as the real bag, but Bar30 depth still moves opposite the real
bag in all tested variants. That makes the current failure structural at the
input/release/sensor-contract layer. It is not a clean hydrodynamic parameter
or ellipsoid/capsule CFD coefficient adjustment.

Promotion decision: keep v55 as the current best. Do not promote the release
velocity variants to HAN/CFD target data. They are useful diagnostics only.

## Force-balance combination A/B

The release logs show why the Bar30 early response is hard to tune with a
single parameter. In v55 the release static force has an upward surplus:

- buoyancy force z: about `+150.098N`
- weight force z: about `-147.155N`
- net static z: about `+2.943N`

With `local_xy_dvl_z`, the vehicle initially moves downward, but that upward
static surplus reverses the depth response within the first window. Three
combined candidates were tested:

- `buoy100_combo`: `local_xy_dvl_z` plus `buoyancy_scale=1.000`
- `buoy1014_combo`: `local_xy_dvl_z` plus `buoyancy_scale=1.014`
- `heave240_combo`: `local_xy_dvl_z` plus `heave_extra_damping=240`
- `vhist2s_combo`: `local_xy_dvl_z` plus 2s vertical thruster history seed

10s metric comparison:

| case | baro | dvl_x | dvl_y | dvl_z | gyro_z | yaw | mean5 |
|---|---:|---:|---:|---:|---:|---:|---:|
| v55_10s | 0.013848 | 0.048628 | 0.055979 | 0.022987 | 0.252497 | 23.300442 | 0.078788 |
| startvel_local_xy_dvl_z | 0.012771 | 0.048601 | 0.055970 | 0.023619 | 0.252895 | 23.683433 | 0.078771 |
| buoy100_combo | 0.046503 | 0.047479 | 0.055885 | 0.031476 | 0.251227 | 23.303974 | 0.086514 |
| buoy1014_combo | 0.016481 | 0.048088 | 0.055944 | 0.025889 | 0.250568 | 23.201428 | 0.079394 |
| heave240_combo | 0.014957 | 0.048261 | 0.055949 | 0.025435 | 0.251602 | 24.247825 | 0.079241 |
| vhist2s_combo | 0.012763 | 0.048545 | 0.055954 | 0.023617 | 0.250214 | 23.960975 | 0.078218 |

First-window vertical direction:

| case | depth_slope_sim | dvlz_slope_sim | depth_end_diff | depth_dir_match | dvlz_dir_match |
|---|---:|---:|---:|---:|---:|
| real | +0.000337 | -0.006066 | 0.000000 | n/a | n/a |
| v55_10s | -0.002981 | +0.002085 | -0.008829 | false | false |
| startvel_local_xy_dvl_z | -0.001758 | -0.001937 | -0.006667 | false | true |
| buoy100_combo | +0.005722 | +0.000177 | +0.009768 | true | false |
| buoy1014_combo | +0.000451 | -0.001386 | -0.001815 | true | true |
| heave240_combo | +0.000390 | -0.001463 | -0.002187 | true | true |
| vhist2s_combo | -0.001731 | -0.002132 | -0.006648 | false | true |

Release force snapshot at first non-hold row:

| case | net_static_z_N | thr_z_first_N | fluid_qfrc_z_first | qacc_z_first | base_vz_down_first |
|---|---:|---:|---:|---:|---:|
| v55 | +2.943091 | -1.937630 | -0.906328 | -0.159776 | -0.006350 |
| startvel | +2.943091 | -1.937419 | +4.680439 | +0.725100 | +0.034642 |
| buoy100 | +0.000000 | -1.943035 | +2.921475 | +0.250160 | +0.023388 |
| buoy1014 | +2.060164 | -1.942571 | +2.405582 | +0.308766 | +0.019493 |
| heave240 | +2.943091 | -1.942394 | +1.969518 | +0.311506 | +0.016125 |
| vhist2s | +2.943091 | -2.082712 | +4.691663 | +0.717087 | +0.034712 |

Decision:

- `buoy1014_combo` and `heave240_combo` fix the first-window direction, but
  both worsen 10s Bar30 and DVL_z RMSE.
- `vhist2s_combo` improves `mean5` from `0.078788` to `0.078218`, but it does
  not fix the Bar30 first-window direction.
- No force-balance combination replaces v55 yet. The current mismatch is a
  coupled release/input/sensor-contract problem, not a standalone CFD or
  ellipsoid/capsule geometry parameter.

Config note after these runs:

```text
3a4f4946fb1e89d92bac604d9b6c296a21549637b5a88df5fef40f582a5a37a2  uuv_mujoco/v2.2/config/sim_profiles.json
64c76f9842d829a12c5e0f178521f0178dceb79e478a5939cb900ded6c91b821  uuv_mujoco/v2.2/config/thruster_params.json
```

Those hashes match the explicit v55 profile/thruster sources used for the
current best replay. The earlier dirty-runtime config hashes were not found in
the workspace, so future comparisons should pass `--profile-source` and
`--thruster-params-source` explicitly.
