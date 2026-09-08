# Research Pool and Hydrodynamics

## Status and scope

This work provides a lightweight indoor-pool scene, the original ellipsoid
robustness profile, and a separate Python-owned distributed-physics profile.
The opt-in physics seams are:

- a spatially nonuniform, deterministic ambient-current field;
- 33 auditable body-fixed definitions expanded to 105 local buoyancy/drag
  samples with point velocity and point current;
- a shared flat or finite-depth gravity-wave surface with bounded orbital
  water velocity;
- a bounded, centre-of-mass-referenced 6 x 6 added-mass/damping interface and
  Fossen Coriolis term;
- bounded local axial-inflow correction around the static T200 thrust curve.

The supplied values are explicitly marked `uncalibrated_pool_prior` or
`uncalibrated_prior`. They are safe engineering starting points for software
and SLAM robustness experiments, not claims about the physical KMU vehicle or
a particular pool.

The accepted default `current` and `research_pool` profiles are unchanged. The
new physics is enabled only by `research_pool_distributed` (flat surface) or
`research_pool_distributed_waves` (deterministic ripple case). Every extension
is an exact identity operation when its section is absent or inactive.

## Economical SLAM pool

`scenes/research_pool_slam_scene.xml` is a 25 m x 12.5 m x 3 m pool. It keeps
the current UUV, thrusters, sensor sites, and three MuJoCo ellipsoid fluid
proxies, while omitting competition buoys and the collector. The environment
uses primitive geometry plus one checker texture:

- tiled floor and walls with visible corners and waterline;
- three floor lane stripes and two non-periodic cross marks;
- two differently colored wall panels;
- one column, one low block, and one asymmetric gate.

The scene contract limits named geometry to 50 items; the current scene uses
22. This provides repeatable visual/geometric structure without dense meshes,
animated particles, or decorative clutter.

Run the original ellipsoid robustness profile with:

```bash
cd /home/khm/robotics/underwater/ROS2-mujoco-UUVsimulator/uuv_mujoco/current
./launch_uuv_sim.sh \
  --scene scenes/research_pool_slam_scene.xml \
  --profile research_pool \
  --fluid-model current
```

Run the new single-owner distributed plant with:

```bash
cd /home/khm/robotics/underwater/ROS2-mujoco-UUVsimulator/uuv_mujoco/current
./launch_uuv_sim.sh \
  --scene scenes/research_pool_slam_scene.xml \
  --profile research_pool_distributed \
  --fluid-model distributed
```

### Browser GUI control

Start the browser control surface from the repository root with the distributed
pool preselected:

```bash
source /opt/ros/humble/setup.bash
source ./.uuv_mujoco_env.sh
./run_control_gui.sh --web \
  --sim-preset research_pool_distributed \
  --host 127.0.0.1 \
  --port 8878
```

Open <http://127.0.0.1:8878/>. The environment selector also exposes the
ellipsoid pool baseline and the deterministic-wave distributed case. Press
**Start**, wait for the vehicle link, then press **Arm**. The left virtual
stick controls yaw/heave and the right stick controls lateral/forward motion;
the sliders and a standard browser gamepad use the same axes. **Center** sends
zero pilot input, **Release input** confirms a neutral RC frame before
releasing ownership, and **Disarm** returns every thruster output to 1500 PWM.

The GUI uses `/mavros/rc/override -> ArduSub -> thruster PWM -> MuJoCo`; it does
not bypass the flight controller with `/cmd_vel`. The physics editor still
edits the legacy `current` profile only, so distributed-profile coefficients
should be changed directly in `config/sim_profiles.json` until that editor is
made profile-aware.

For the bounded deterministic wave/orbital-flow case, change only the profile
to `research_pool_distributed_waves`. The `distributed` fluid-model alias maps
to the Python-owned path and disables MuJoCo ellipsoid fluid. Runtime ownership
validation rejects an advanced profile if old diagonal drag, restoring
springs, unapplied state scaling, empirical/CFD drag on an already-owned axis,
or duplicate added mass is still requested. The Python entry point also rejects
`--fluid-model distributed` when its selected profile has no active distributed
patch section.

The existing OpenFOAM lookup is labelled as total force, not a residual. It is
therefore rejected on top of MuJoCo ellipsoid drag and may run in the legacy
custom backend only when every CFD-owned translational damping axis is zero.
Future `CFD - baseline` residual data needs a separate, explicit calibration
contract rather than reusing this total-force table.

## Current-field model

The world-frame water velocity is

```text
v(x,t) = bound_norm(
    v0 + G (x - x0)
       + sum A_i sin(k_i dot (x - x0) + phi_i)
       + sum B_j sin(2 pi f_j t + phi_j),
    v_max)
```

`G` has units `1/s`, `k` has units `rad/m`, and temporal frequencies are
limited to at most 0.5 Hz. Missing temporal phases are generated once from the
configured integer seed, so sampling is deterministic in position and
simulation time.

The MuJoCo ellipsoid profile still has one global `opt.wind` sample. The
distributed profile instead samples current independently at all 105 expanded
hull/frame force points and at every thruster. The centre-velocity sample used
by the 6-DOF matrix is taken at MuJoCo's inertial centre of mass. This is
low-order per-vehicle spatial flow, not Navier-Stokes turbulence or pool-wide
CFD. Boundary layers, pump jets, propeller wakes, and auxiliary-body local flow
remain outside this model.

## Distributed loads and 6-DOF matrices

Every sample evaluates local rigid-body velocity `v_com + omega x
(point - com)`, local current, partial immersion, vertical buoyancy, normal
quadratic form drag, and tangential skin drag. Forces are summed about the
actual MuJoCo centre of mass. The profile keeps nine volume points and 24
auditable drag-face definitions (18 enclosure/lower-hull and six effective
open-frame faces). Each drag face is integrated at four tangent-plane Gauss
points, producing 96 drag samples and 105 total local samples. This resolves
rotational drag that a single face-centre sample misses without changing the
configured projected area.

Buoyancy uses the profile's `buoyancy_scale`, but force safety limits apply
only to dynamic form/skin drag; high speed therefore cannot reduce displaced
volume. The shipped volume centroid is aligned in x with the configured
runtime mass centroid for level trim. Patch area, volume, centre and drag values
are still engineering priors that must be replaced by identification data.

Added mass is separate from patch drag. The matrix path supports bounded,
symmetric positive-semidefinite 6 x 6 added mass plus translation/rotation
coupling, Fossen added-mass Coriolis, and passive linear/quadratic damping
matrices. Its declared reference is the MuJoCo centre of mass, and non-finite or
out-of-bound matrices/states/wrenches are rejected or bounded. Quadratic
damping is evaluated in the symmetric matrix's eigenbasis as modal
`lambda * |velocity| * velocity`; this preserves passivity while allowing
cross-axis output. The distributed profile activates only a diagonal
added-mass prior; patch forces own all damping.

Acceleration for the added-mass prior is still obtained from a one-step
relative-velocity difference. Its history is reset when the initial pose hold
is released so startup does not create a fictitious impulse, but this remains
an explicit low-order approximation rather than an implicit fluid/rigid-body
inertia solve.

The optional speed/depth/attitude coefficient scaler remains available to the
original `research_pool` ellipsoid profile. It is intentionally disabled in
the distributed profile because patch kinematics already create orientation,
rotation, depth, and partial-immersion effects.

## Free-surface boundary

`FreeSurface` supplies one deterministic height, normal, and orbital-velocity
contract. The same surface height is consumed by distributed buoyancy,
standard component buoyancy, MuJoCo fluid-proxy immersion scaling, and
thruster immersion. Ambient plus orbital velocity is consumed by patch drag,
vehicle-centre relative velocity, local thruster inflow, and any course-buoy
drag. Course buoys query the same surface locally and use orbital vertical
velocity in their surface damping instead of assuming still water.

`flat` is the recommended indoor-pool baseline. The shipped `harmonic` profile
uses finite-depth linear gravity-wave kinematics: angular frequency is derived
from `omega^2 = g k tanh(k h)`, orbital velocity obeys the bottom boundary, and
its magnitude is bounded. This is a robustness disturbance, not a sloshing CFD
solver. It does not model wave pressure/diffraction/radiation forces, breaking
waves, slamming, spray, entrained air, viscous surface films, or two-way wave
interaction. The Bar30 pressure path and rendered water mesh still use their
existing reference plane, so the wave profile is not yet complete
sensor/visual wave parity.

## Thruster inflow

The public T200 curve remains the static bollard-pull source. The opt-in inflow
model uses each propulsor site's world position, velocity, actual site-frame
thrust axis, and local water velocity to reduce advancing thrust or boundedly
raise thrust in opposing flow. The internal nondimensional value is normalized
axial speed, not the physical propeller advance ratio `J = V_A/(nD)`. It is not
blade-element momentum theory and does not resolve RPM-dependent `K_T(J)`,
propeller wake, cavitation, motor electrical dynamics, or
thruster-thruster/hull interaction. Propeller reaction torque remains
implemented but disabled until a measured torque coefficient is available.
When enabled, the reaction torque is cached in body coordinates and rotated at
the physics cadence, so it does not retain a stale world direction between
slower thruster updates.

## Calibration path

Before using this plant as paper evidence:

1. Measure pool velocity at several `(x, y, z)` points with a current meter or
   ADV and estimate the mean, gradients, low-frequency spectrum, and uncertainty.
2. Fit patch projected areas and normal/skin coefficients with axis-isolated
   tow and angular free-decay tests over the intended speed range.
3. Identify added mass with acceleration tests; fit full-matrix cross terms
   only where repeatable cross-axis evidence exists.
4. Tow the powered vehicle or each mounted T200 to fit inflow loss separately
   from the static thrust curve.
5. Measure surface elevation if the wave profile will be used, then use the
   same record for physics, pressure, and rendering before claiming parity.
6. Split identification and validation runs and report parameter intervals,
   not only a best fit.
7. Replace every uncalibrated status and note with traceable data provenance.

## Verification

Dependency-light checks:

```bash
python3 tools/check_research_pool_scene.py
python3 tools/test_pool_hydrodynamics.py
python3 tools/test_distributed_hydrodynamics.py
python3 tools/test_free_surface.py
python3 tools/test_full_matrix_hydrodynamics.py
python3 tools/test_thruster_inflow_physics.py
python3 tools/test_advanced_hydrodynamics_ownership.py
```

Actual-MuJoCo checks, run inside the project MuJoCo environment:

```bash
python3 tools/check_research_pool_physics.py
python3 tools/check_distributed_pool_physics.py
```

The distributed smoke loads both flat and wave profiles, verifies 105 local
samples, level-trim buoyancy, opposing forward and rotational drag,
centre-of-mass point kinematics, bounded full-matrix wrenches, finite-depth wave
geometry/orbital flow, and 125 finite MuJoCo steps. It also creates the real
eight-thruster runtime and verifies that local inflow and the shared surface
reach the production actuator path.
