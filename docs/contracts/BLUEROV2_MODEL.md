# BlueROV2 Standard starting model

See the [source register and BibTeX](../../uuv_mujoco/current/research_workspace/04_paper_evidence/REFERENCES_BLUEROV2.md) for the origin of model parameters, data and baselines.

Launch the standard six-T200 vehicle with `--vehicle bluerov2_standard` or
choose **BlueROV2 Standard · Research pool** in the GUI. The GUI starts with
the paper's `bluerov2_dfki_yaw_experiment` coefficients and the standard
ArduSub motor mapping. Its scene uses the UT27 research pool and its tethered
yellow buoy. The DFKI PWM
replay tool explicitly loads `bluerov2_dfki_scene.xml`, a separate flat test
tank. Both scenes share the same vehicle geometry and use the `legacy`
whole-body damping model. The simulated vehicle has no tether model.

The body frame is FLU (x forward, y left, z up). The hull and propellers are
OBJ conversions of the CAD meshes referenced by the [public ROS 2
Xacro](https://github.com/CentraleNantesROV/bluerov2/tree/dd7111b609b1d8bad052e4990d9b439e5317ad7c/bluerov2_description).
The hull and all six propeller origins use a common +72.5 mm z translation to
place the horizontal thrust plane at z = 0. The force sites coincide with the
rotor origins. The four horizontal force axes follow the Xacro rotor axes and
the two vertical force axes use −z, following the [Gazebo
model](https://github.com/clydemcqueen/bluerov2_gz/blob/4451a5d1b15d204581d43ef7d93df4a0508eddbb/models/bluerov2/model.sdf).
The runtime sets PWM polarity separately. The visible OBJ triangles are
noncolliding. Hidden rigid primitives approximate hull contact with separate
sliding coefficients for HDPE, acrylic, aluminum and polycarbonate. See the
[contact model](BLUEROV2_CONTACT.md) for the assumptions. The standard CAD has no Newton
gripper. Mass (10.012 kg), diagonal inertia and the 11 mm CoM offset are
simulation priors.

Neutral buoyancy, a CoB 20 mm above CoM, 1000 kg/m³ fluid density and zero
current are assumptions. The interactive pool has a 5 m depth. The replay
scene has a surface at z = 0 and a floor at z = -8 m. The vehicle begins fully
submerged. IMU and camera positions follow the registered [public sensor
model](https://github.com/CentraleNantesROV/bluerov2/blob/dd7111b609b1d8bad052e4990d9b439e5317ad7c/bluerov2_description/urdf/sensors.xacro).
The simulated eye is 55 mm ahead of the camera mount to clear the opaque CAD
dome. The pressure sensor offset is a placeholder. These placements do not
establish the transforms or camera calibration of DFKI's vehicle.

The surge, sway, heave and yaw linear and quadratic damping start from the
[Li et al. BlueROV2 coefficients](https://doi.org/10.3390/jmse8090688).
Roll and pitch quadratic damping 3.992 are public Gazebo-model starting values;
roll and pitch linear damping are assumed zero. All added-mass coefficients are
zero as an explicit omission, not a measured absence. These values combine
sources and define the stock hydrodynamics prior. Nominal supply voltage is
14.8 V. The recorded voltage trace takes precedence during replay.

The [DFKI dataset description](https://doi.org/10.1038/s41597-025-06347-0)
provides the experimental context, but does not establish all the vehicle's
mass, ballast, sensor transforms, fluid coefficients or tether loads used
here. Results must be reported as model-versus-recording comparisons, with
these assumptions and the particular replay configuration disclosed.

## T200 propulsion and DFKI replay

All six positive actuator axes point toward the T200 cable entry. Positive
scene thrust therefore uses the manufacturer forward curve. The R4 layout
uses CCW propellers at motors 1, 2 and 5 and CW at motors 3, 4 and 6.
The RPM response uses the manufacturer's PWM/RPM and RPM/thrust tables at
10–20 V, interpolates at the recorded pack voltage, and brakes through zero
during reversal. It treats 1475–1525 µs as neutral. The spin-up and spin-down
time constants (40 and 60 ms) remain effective response priors.
[Blue Robotics assembly guide](https://bluerobotics.com/learn/bluerov2-assembly/#installing-the-thrusters)
and [T200 performance workbook](https://cad.bluerobotics.com/T200-Public-Performance-Data-10-20V-September-2019.xlsx).

The DFKI replay layout reverses motor 5 relative to the stock SITL mapping
based on the logged motion and commands. It interprets the final recorded ESC pulses as
`[-1, -1, +1, +1, -1, +1]` in servo order. Its relative vertical
polarity is corroborated by the throttle-stick RC input and motor outputs.
The simulator applies this wiring conversion once, then selects the
asymmetric curve from the resulting thrust direction. The stock SITL
layout is kept separate.

Use the extracted trial CSVs from the [DFKI v3 archive](https://zenodo.org/records/17360027).
For example, from the repository root:

```bash
python uuv_mujoco/current/tools/prepare_dfki_bluerov2.py \
  /path/to/raw/serpentine /tmp/dfki-serpentine --trial serpentine
depth=$(python -c 'import json,sys; print(json.load(open(sys.argv[1]))["initial_pressure_depth_m"])' \
  /tmp/dfki-serpentine/metadata.json)
python uuv_mujoco/current/tools/replay_bluerov2_dfki.py \
  --pwm /tmp/dfki-serpentine/pwm.csv --real /tmp/dfki-serpentine/real.csv \
  --voltage /tmp/dfki-serpentine/voltage.csv --depth-m "$depth" \
  --profile bluerov2_dfki_calibrated --output /tmp/dfki-calibrated
python uuv_mujoco/current/tools/score_bluerov2_dfki.py \
  --real /tmp/dfki-serpentine/real.csv --output /tmp/dfki-scores /tmp/dfki-calibrated
```

The calibrator uses a submerged 6-DOF search model to fit effective linear and
quadratic drag, buoyancy, three-axis CoB trim, installed thrust gains, yaw
moment efficiency and a surge-induced pitch moment. The final parameter
profile was selected on full MuJoCo replays of the random, circle, 5 m, 10 m
and 15 m trials. It starts with the three-trial seed (random, circle and
10 m with `--long-weight .6 --full-weight .28 --max-evaluations 75`).
Refit the final set with `--train random circle 5m 10m 15m --long-weight .55
--full-weight .45 --max-evaluations 75 --seed-profile <seed-file>
--implicit-damping` using `calibrate_bluerov2_dfki.py`. NumPy and SciPy
suffice. Optional Numba accelerates the calibration tool.

The final comparisons replay original PWM in MuJoCo from the measured start,
preserve packet timestamps, and report paired 10 Hz RMSE with gaps excluded.
The serpentine trial was kept out of fitting. The 7 m, 12 m and 14 m trials
were reserved for final testing. The GUI research pool contains its original
buoy. The replay tank contains only a floor and no buoy.
The first six metrics are body surge and sway speeds, relative depth, roll,
pitch, and accumulated relative yaw. The scorer also returns six velocity/rate
RMSEs and wrapped yaw error. The effective gains and damping absorb some
effects of the tether and ballast. The logs alone do not separate these
effects from intrinsic thruster performance, added mass, or surface friction.

## Yaw evaluation

The scorer reports body yaw-rate RMSE and signed mean rate error, relative
heading changes over 1, 5 and 10 seconds, wrapped heading RMSE, and unwrapped
whole-run yaw and end-of-run drift. Heading increments use paired valid
endpoints from the open-loop replay. The simulator state is initialized once.

The optional `bluerov2_dfki_yaw_experiment` profile fits four effective yaw
moment scales and directional yaw damping on the original five fitting
trials. The signed damping adjustment scales linear damping by
`1 + asymmetry × sign(FRD yaw rate)`. Both turn directions dissipate energy.
The profile uses the same translational thrust curves as the original
calibration. It improves the fitting-run yaw overlay, but increased pooled
long-horizon error on six fitting-excluded trials. Use
`bluerov2_dfki_calibrated` for general DFKI replay.
