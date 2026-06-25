# 02 Plant Replay

Plant replay answers one question:

```text
real RCOU/SERVO output -> MuJoCo plant -> simulated sensors
```

The target is not to make the closed loop look plausible.  The target is to see
whether the MuJoCo plant produces the same sensor/state trajectory under the
same actuator command.

## Latest Canonical Runs

- `/Users/kanghyunmin/Desktop/uuv_sim/uuv_mujoco/v2.2/debug/controller_parity_412/outputs/plant_replay_90s_t200direct_fix_20260601`
- `/Users/kanghyunmin/Desktop/uuv_sim/uuv_mujoco/v2.2/debug/controller_parity_412/outputs/plant_replay_90s_plantonly_hold_current_20260601`
- `/Users/kanghyunmin/Desktop/uuv_sim/UUV-HAN/outputs/fossen_residual_full90_20260601`
- `/Users/kanghyunmin/Desktop/uuv_sim/UUV-HAN/outputs/validate_fossen_cfd_prior_full90_20260601`
- `/Users/kanghyunmin/Desktop/uuv_sim/UUV-HAN/outputs/plant_replay_validate_heave72_cobz005_full90_20260601`
- `/Users/kanghyunmin/Desktop/uuv_sim/UUV-HAN/outputs/input_lag_ab_full90_20260601`
- `/Users/kanghyunmin/Desktop/uuv_sim/UUV-HAN/outputs/environment_current_full90_20260601`

## Current Diagnosis

- The plant replay runner now has a `plant_only` launch mode.  Real RCOU is
  injected directly into the MuJoCo plant through `/uuv_mujoco/rc/out_override`;
  this path does not require a host-native ArduSub binary.
- Plant replay now forces T200 direct mode for final RCOU PWM.  The required
  actuator contract is `raw final PWM -> T200 force curve once`.  The older
  plant-only runs accidentally skipped `--thruster-perf-direct`, so the T200
  curve was followed by legacy polynomial/gain scaling.  Those older runs are
  invalid as physics-tuning evidence even though their RCOU input audit passed.
- The same real-start hold contract is used in plant-only replay: initialize
  pose/pressure/velocity, keep the plant pinned during bootstrap, latch the
  first RCOU sample, then release.  Runs without this hold are invalid because
  the plant drifts before replay time starts.
- Input replay is exact in latest audit (`maxdiff_us=0`, no PWM mismatches).
- The input source is final real RCOU/SERVO telemetry, not RC override and not
  high-rate JSON servo output.
- Safe primary sensor targets are Bar30/depth pressure-derived depth, IMU
  attitude/gyro, DVL x/y, and body-rate response.
- Excluded or warning-grade targets: `/mavros/imu/atm_pressure`,
  `local_position` as a primary target, and DVL-z until the remaining axis
  ambiguity is resolved.
- Full90 sensor match is still not close to 99%, but the largest diagnosed
  thrust-contract error has been removed.
- The remaining error is plant dynamics: coupled yaw/sway, heave/pitch,
  rotational damping, added mass, and thruster/body interaction.

Latest valid 90s hold/release plant-only result:

```text
input audit: ok=true, mismatches=0, maxdiff_us=0.0, samples=35840
direct T200 audit: likely_perf_direct_disabled=false, violations=0
overlay: /Users/kanghyunmin/Desktop/uuv_sim/uuv_mujoco/v2.2/debug/controller_parity_412/outputs/plant_replay_90s_t200direct_fix_20260601/sensor_overlay/plant_sensor_overlay.png
baro_ddepth RMSE=0.139 corr=0.129  bias=-0.028
roll        RMSE=1.882 corr=0.819  bias=0.541 deg
pitch       RMSE=3.051 corr=0.794  bias=2.453 deg
yaw         RMSE=43.798 corr=0.982 bias=6.231 deg
dvl_x       RMSE=0.099 corr=0.776  bias=-0.055 m/s
dvl_y       RMSE=0.099 corr=0.647  bias=0.012 m/s
gyro_z      RMSE=0.301 corr=0.676  bias=-0.031 rad/s
```

Interpretation: RCOU input and replay timing are no longer the dominant issue.
The previous huge yaw/depth/pitch errors were mostly caused by a plant thrust
contract bug: plant-only replay did not run in direct T200 mode.  After fixing
that, yaw angle is now over-effective (`sim/real yaw delta ~= 1.29`) rather than
under-effective, and the remaining target is coupled yaw damping/effectiveness
plus heave/pitch dynamics.

## Residual Mode Decomposition

The next tuning pass must not use blind scalar least-squares.  The valid
workflow is:

```text
Y = real - sim sensor residuals
X = actuator, state, hydro, PWM, and wrench features
SVD(Y) identifies independent residual modes
SVD(X^T Y) diagonalizes feature/residual coupling
least-squares is allowed only in those diagonalized mode coordinates
```

The reports use singular values `sigma`, not eigenvalues directly.  The
corresponding eigenvalue is `sigma^2` of `A^T A` or `A A^T`.  Residual explained
fractions are computed from `sigma^2`.

Tool and current outputs:

```text
script:
  /Users/kanghyunmin/Desktop/uuv_sim/uuv_mujoco/v2.2/debug/controller_parity_412/decompose_plant_replay_residual_modes.py

all targets:
  /Users/kanghyunmin/Desktop/uuv_sim/uuv_mujoco/v2.2/debug/controller_parity_412/outputs/plant_replay_90s_t200direct_fix_20260601/sensor_overlay/residual_mode_decomposition.md
  /Users/kanghyunmin/Desktop/uuv_sim/uuv_mujoco/v2.2/debug/controller_parity_412/outputs/plant_replay_90s_t200direct_fix_20260601/sensor_overlay/residual_mode_decomposition.json

horizontal:
  /Users/kanghyunmin/Desktop/uuv_sim/uuv_mujoco/v2.2/debug/controller_parity_412/outputs/plant_replay_90s_t200direct_fix_20260601/sensor_overlay/residual_modes_horizontal.md
  /Users/kanghyunmin/Desktop/uuv_sim/uuv_mujoco/v2.2/debug/controller_parity_412/outputs/plant_replay_90s_t200direct_fix_20260601/sensor_overlay/residual_modes_horizontal.json

depth:
  /Users/kanghyunmin/Desktop/uuv_sim/uuv_mujoco/v2.2/debug/controller_parity_412/outputs/plant_replay_90s_t200direct_fix_20260601/sensor_overlay/residual_modes_depth.md
  /Users/kanghyunmin/Desktop/uuv_sim/uuv_mujoco/v2.2/debug/controller_parity_412/outputs/plant_replay_90s_t200direct_fix_20260601/sensor_overlay/residual_modes_depth.json
```

Current SVD diagnosis:

- `all` target cross-SVD is numerically diagonalized
  (`offdiag/diag ~= 7.7e-16`), so mode-wise fits are well defined.
- Horizontal residuals are not a pure yaw problem.  The first four residual
  modes mix `yaw`, `gyro_z`, `dvl_x`, and `dvl_y`; a yaw gain-only fit is
  underconstrained and explains the bad tradeoff in `yaw078`.
- Depth residuals split into a common depth-offset mode, a `dvl_z`/`accel_z`
  dynamic mode, and an inertial vertical acceleration mode.  A heave gain-only
  or buoyancy-only change is therefore not a valid correction.
- Future HAN/CFD/Fossen residual work should fit coupled blocks:
  horizontal yaw/sway, vertical heave/added-mass/damping, and pitch/roll
  restoring/damping.

Parameter-sensitivity SVD was also added so the coefficient fit itself is
audited before any profile update:

```text
script:
  /Users/kanghyunmin/Desktop/uuv_sim/uuv_mujoco/v2.2/debug/controller_parity_412/modal_sensitivity_svd_fit.py

reports:
  /Users/kanghyunmin/Desktop/uuv_sim/uuv_mujoco/v2.2/debug/controller_parity_412/outputs/plant_replay_90s_t200direct_fix_20260601/sensor_overlay/modal_sensitivity_fossen_full90.md
  /Users/kanghyunmin/Desktop/uuv_sim/uuv_mujoco/v2.2/debug/controller_parity_412/outputs/plant_replay_90s_t200direct_fix_20260601/sensor_overlay/modal_sensitivity_vertical_heave_full90.md
  /Users/kanghyunmin/Desktop/uuv_sim/uuv_mujoco/v2.2/debug/controller_parity_412/outputs/plant_replay_90s_t200direct_fix_20260601/sensor_overlay/modal_sensitivity_thruster_yaw_full90.md
```

Sensitivity SVD conclusions:

- Existing full90 sensitivity families share the same real target but not the
  same baseline `sim`, so they cannot be concatenated into one global Jacobian
  until rerun from the fixed T200-direct baseline.
- The actual sensitivity algorithm is `r = real - sim`, `J = d(sim)/d(param)`,
  target-normalized `A = WJ`, then `A = U diag(sigma) V^T`.  `V` gives
  identifiable parameter combinations and `U` gives the sensor/time residual
  modes those combinations excite.
- Direct yaw-thruster gains are observable (`condition ~= 8.46`) but barely
  reduce the horizontal residual (`normalized RMSE 1.059 -> 1.057`), so yaw
  gain is not the missing model.
- Vertical/heave sensitivity is ill-conditioned (`condition ~= 9372`): buoyancy
  dominates the first mode and heave damping is a weak final mode.  This matches
  the visual behavior where heave-only changes create tradeoffs.
- Fossen residual parameters are identifiable (`condition ~= 6.91`) but only
  explain a small fraction of the mismatch from the historical baseline
  (`normalized RMSE 1.118 -> 1.077`).  The family is useful, but it must be
  rerun and fit as coupled modal blocks rather than applied blindly.
- A case/subset matrix search was added after the SVD audit:
  `/Users/kanghyunmin/Desktop/uuv_sim/uuv_mujoco/v2.2/debug/controller_parity_412/modal_case_matrix_search.py`.
  It builds `F = A^T A`, `g = A^T b`, column collinearity, and subset
  predictions before any least-squares update.  Current full90 Fossen residual
  results are weak: horizontal best subset reduces only `3.7%` residual energy
  with `corr ~= 0.20`; depth best subset reduces `5.4%` with `corr ~= 0.24`;
  all-target best subset reduces `6.1%` with `corr ~= 0.25`.  Therefore the
  Fossen residual family alone does not span the measured plant replay
  residual.
- Same-baseline sensitivity is now a hard gate.  The full90 Fossen target
  baseline matches its own sensitivity (`maxdiff=0`), but existing added-mass,
  environment-current, and yaw-inertia full90 sensitivity batches were generated
  from different baselines (`maxdiff` up to `1.012867855`).  They cannot be
  concatenated into one global `J` until rerun from the current fixed
  T200-direct baseline.

## 2026-06-01 A/B Checks

All runs below keep the same plant input contract:

```text
real_90s_feedback rc_out_ch1..8 -> /uuv_mujoco/rc/out_override -> MuJoCo plant
input audit: ok=true, mismatches=0, maxdiff_us=0.0
```

| run | changed knob | baro RMSE | pitch RMSE | yaw RMSE | gyro_z RMSE | result |
| --- | --- | ---: | ---: | ---: | ---: | --- |
| `plant_replay_90s_t200direct_fix_20260601` | fixed raw PWM -> T200 direct contract | 0.139 | 3.051 | 43.798 | 0.301 | new valid reference |
| `plant_replay_90s_t200direct_yaw078_20260601` | yaw direct gains x0.78 | 0.170 | 2.335 | 73.699 | 0.291 | yaw integral improves, yaw RMSE/depth worsen; not accepted |
| `plant_replay_90s_plantonly_hold_current_20260601` | missing T200 direct mode | 0.700 | 9.414 | 299.793 | 0.429 | invalid thrust contract; kept as before evidence |
| `plant_replay_90s_buoy108_hold_20260601` | `buoyancy_scale=1.08` | 0.594 | 10.106 | 313.956 | 0.433 | depth improves, attitude/yaw worsen |
| `plant_replay_90s_buoy115_hold_20260601` | `buoyancy_scale=1.15` | 0.528 | 10.636 | 321.016 | 0.434 | depth improves more, attitude/yaw worsen more |
| `plant_replay_90s_yaw150_hold_20260601` | `yaw_torque_scale=1.5` | 0.811 | 10.004 | 237.492 | 0.627 | yaw angle improves, yaw-rate/depth worsen |
| `plant_replay_90s_vx050_hold_20260601` | `vertical_thruster_x_scale=0.5` | 0.693 | 8.426 | 300.486 | 0.439 | pitch improves, but geometry change is too strong |
| `plant_replay_90s_pitchK160_hold_20260601` | `UUV_HYDROSTATIC_RESTORING_PITCH_NM_PER_RAD=160` | 0.607 | 5.559 | 318.221 | 0.437 | pitch/depth improve, yaw worsens |

Accepted conclusions:

- The plant is not an input-contract failure anymore; every A/B above preserved
  exact RCOU replay.
- Plant-only replay before `plant_replay_90s_t200direct_fix_20260601` is not a
  valid physics baseline because it applied legacy thruster gain/polynomial
  shaping after the T200 curve.
- The vehicle cannot be tuned from the old A/B table.  Rerun buoyancy, damping,
  yaw, and HAN/CFD coefficient sweeps from the fixed direct-T200 reference.
- Reducing yaw direct gains by 0.78 improves integrated yaw amount
  (`sim/real yaw delta 1.29 -> 1.13`) and slightly improves gyro_z RMSE, but it
  worsens yaw RMSE and depth bias.  Yaw tuning is coupled to sway/heave/pitch
  and should use a multi-signal gate rather than matching total heading alone.
- The vertical pitch moment is too strong relative to static restoring.  This
  can be reduced either by unphysical vertical-thruster lever scaling or by a
  stronger pitch righting moment; the latter is the safer physics direction.
- Yaw is not fixed by a simple torque scale.  A scalar yaw moment multiplier
  improves integrated heading but worsens yaw-rate and depth, so yaw needs
  coupled yaw damping/added mass/effectiveness identification.

## Gate

A candidate profile is accepted only if it improves the direct plant replay
ranking and does not worsen critical sensor overlays by hiding errors in
controller timing or actuator remap.
