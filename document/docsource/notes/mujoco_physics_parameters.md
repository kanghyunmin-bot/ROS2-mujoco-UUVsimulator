# MuJoCo UUV Physics Parameters

## Purpose

This document explains the physics-related parameters used in this project and
how they relate to MuJoCo's official fluid-force model.

The key distinction is:

- MuJoCo built-in fluid model: configured from MJCF using `density`,
  `viscosity`, and optionally `fluidshape="ellipsoid"` with `fluidcoef`.
- This repository's UUV model: uses MuJoCo for rigid-body integration, but
  applies most underwater forces explicitly through `data.xfrc_applied`.

That means the simulator is not relying only on MuJoCo's built-in passive fluid
forces. It uses a custom 6-DOF hydrodynamics layer on top of MuJoCo.

## Official MuJoCo Fluid-Force Model

### Global fluid properties

MuJoCo's MJCF `option` element defines global medium properties such as density
and viscosity. In this repository:

- [competition_scene.xml](/Users/kanghyunmin/Desktop/uuv_sim/uuv_mujoco/v2.2/competition_scene.xml#L5)
  sets `density="1000"` and `viscosity="0.001"`, which correspond to water.
- [urdf_full_scene.xml](/Users/kanghyunmin/Desktop/uuv_sim/uuv_mujoco/v2.2/urdf_full_scene.xml#L6)
  sets `density="1.225"` and `viscosity="1.8e-05"`, which are air-like values.

According to the official MuJoCo XML reference, these values are used by the
passive fluid-force model and by plugins that consume the medium properties.

Source:

- MuJoCo XML Reference:
  <https://mujoco.readthedocs.io/en/3.2.6/XMLreference.html>

### Built-in fluid models in MuJoCo

MuJoCo documentation distinguishes two main passive fluid-force paths.

1. Inertia-based body model

- This is the older default-style passive model.
- It uses body inertia size to construct an equivalent body for fluid forces.
- It is global and coarse.
- It is useful when a body does not have a dedicated fluid-shape model.

2. Ellipsoid-based geom model

- This is enabled per geom with `fluidshape="ellipsoid"`.
- The XML attribute `fluidcoef` provides five dimensionless coefficients:
  - blunt drag coefficient
  - slender drag coefficient
  - angular drag coefficient
  - Kutta lift coefficient
  - Magnus lift coefficient
- When `fluidshape="ellipsoid"` is active for a geom, the geom-level model is
  used and the body-inertia-size fluid model is disabled for that body.

Source passage:

- MuJoCo XML Reference `geom/fluidshape` and `geom/fluidcoef`:
  <https://mujoco.readthedocs.io/en/3.2.6/XMLreference.html#body-geom>

### Important implication for this project

This repository does not currently define any geom with `fluidshape="ellipsoid"`
or `fluidcoef`. A repository-wide search confirms that the scene files do not
use those attributes. Instead, underwater forces are injected manually in the
runtime.

- Search result:
  - no `fluidshape`
  - no `fluidcoef`
- Custom external wrench path:
  [run_urdf_full.py](/Users/kanghyunmin/Desktop/uuv_sim/uuv_mujoco/v2.2/run_urdf_full.py#L1100)

So the project is best understood as:

- MuJoCo handles rigid-body dynamics, contacts, joints, and time integration.
- The project handles underwater hydrodynamics explicitly.

## What This Repository Actually Uses

### Engine-level parameters

These are MuJoCo engine parameters defined in the scene XML.

- `density`
  - Global medium density.
  - Water-like in the competition scene, air-like in the default debug scene.
- `viscosity`
  - Global medium viscosity.
- `integrator`
  - Current value is `implicitfast`.
  - This is relevant because underwater simulations often benefit from a stable
    implicit integrator when damping and external forces are strong.

Defined at:

- [competition_scene.xml](/Users/kanghyunmin/Desktop/uuv_sim/uuv_mujoco/v2.2/competition_scene.xml#L5)
- [urdf_full_scene.xml](/Users/kanghyunmin/Desktop/uuv_sim/uuv_mujoco/v2.2/urdf_full_scene.xml#L6)

### Custom underwater wrench model

The main underwater model is implemented in
[run_urdf_full.py](/Users/kanghyunmin/Desktop/uuv_sim/uuv_mujoco/v2.2/run_urdf_full.py#L1100)
inside `apply_underwater_wrench()`.

The applied terms are:

1. Buoyancy

- Submergence ratio is approximated from depth and `half_height`.
- Buoyancy magnitude is:
  `rho * g * neutral_volume * submerged_ratio * buoyancy_scale`
- Applied in world `+Z`.

Relevant lines:

- [run_urdf_full.py](/Users/kanghyunmin/Desktop/uuv_sim/uuv_mujoco/v2.2/run_urdf_full.py#L1111)
- [run_urdf_full.py](/Users/kanghyunmin/Desktop/uuv_sim/uuv_mujoco/v2.2/run_urdf_full.py#L1114)

2. Restoring torque from CoB/CoM separation

- The buoyancy force is applied at a point blended between CoM and CoB.
- The restoring torque is `r x F`.
- This is scaled by `cob_torque_scale`.

Relevant lines:

- [run_urdf_full.py](/Users/kanghyunmin/Desktop/uuv_sim/uuv_mujoco/v2.2/run_urdf_full.py#L1108)
- [run_urdf_full.py](/Users/kanghyunmin/Desktop/uuv_sim/uuv_mujoco/v2.2/run_urdf_full.py#L1120)

3. Relative-current hydrodynamics

- Linear velocity is converted to body frame.
- Water current is also converted to body frame.
- Hydrodynamic drag is based on relative velocity, not inertial velocity.

Relevant lines:

- [run_urdf_full.py](/Users/kanghyunmin/Desktop/uuv_sim/uuv_mujoco/v2.2/run_urdf_full.py#L1125)
- [run_urdf_full.py](/Users/kanghyunmin/Desktop/uuv_sim/uuv_mujoco/v2.2/run_urdf_full.py#L1127)

4. Added mass

- The model uses a diagonal 6-DOF added-mass approximation.
- Translational and rotational added masses are multiplied by submergence.
- An added-mass Coriolis term is also applied.

Relevant lines:

- [run_urdf_full.py](/Users/kanghyunmin/Desktop/uuv_sim/uuv_mujoco/v2.2/run_urdf_full.py#L1137)
- [run_urdf_full.py](/Users/kanghyunmin/Desktop/uuv_sim/uuv_mujoco/v2.2/run_urdf_full.py#L1144)
- [hydrodynamics_helpers.py](/Users/kanghyunmin/Desktop/uuv_sim/uuv_mujoco/v2.2/hydrodynamics_helpers.py#L35)

5. Linear and quadratic damping

- Linear damping is applied in all 6 DOF.
- Quadratic damping is applied as `|nu| * nu`.
- Water damping is blended against a smaller air damping when submergence is
  partial.

Relevant lines:

- [run_urdf_full.py](/Users/kanghyunmin/Desktop/uuv_sim/uuv_mujoco/v2.2/run_urdf_full.py#L1138)
- [run_urdf_full.py](/Users/kanghyunmin/Desktop/uuv_sim/uuv_mujoco/v2.2/run_urdf_full.py#L1141)
- [run_urdf_full.py](/Users/kanghyunmin/Desktop/uuv_sim/uuv_mujoco/v2.2/run_urdf_full.py#L1148)

### How the custom model is applied to MuJoCo

The repository applies all underwater forces directly into
`data.xfrc_applied`.

Relevant lines:

- [run_urdf_full.py](/Users/kanghyunmin/Desktop/uuv_sim/uuv_mujoco/v2.2/run_urdf_full.py#L1104)
- [run_urdf_full.py](/Users/kanghyunmin/Desktop/uuv_sim/uuv_mujoco/v2.2/run_urdf_full.py#L1119)
- [run_urdf_full.py](/Users/kanghyunmin/Desktop/uuv_sim/uuv_mujoco/v2.2/run_urdf_full.py#L1151)

This is the most important implementation fact in the repository:

- The simulator is not waiting for MuJoCo's passive fluid model to create the
  underwater forces.
- It computes the forces in Python and injects them explicitly.

## Shape-Based Ellipsoid Baseline in This Repository

### Why an ellipsoid baseline was introduced

The project originally used hand-tuned coefficient vectors only.
`sim_real` now uses a shape-based baseline so that the main hydrodynamic
coefficients come from geometry first and are tuned second.

This is not the same as MuJoCo's built-in `fluidshape="ellipsoid"` path.
Instead, it is a custom ellipsoid-based coefficient generator.

### Where it is defined

- Profile definition:
  [sim_profiles.json](/Users/kanghyunmin/Desktop/uuv_sim/uuv_mujoco/v2.2/sim_profiles.json#L56)
- Profile resolution:
  [sim_profile_helpers.py](/Users/kanghyunmin/Desktop/uuv_sim/uuv_mujoco/v2.2/sim_profile_helpers.py#L205)
- Math helpers:
  [hydrodynamics_helpers.py](/Users/kanghyunmin/Desktop/uuv_sim/uuv_mujoco/v2.2/hydrodynamics_helpers.py#L98)
  and
  [hydrodynamics_helpers.py](/Users/kanghyunmin/Desktop/uuv_sim/uuv_mujoco/v2.2/hydrodynamics_helpers.py#L146)

### Parameters in `ellipsoid_model`

- `semi_axes`
  - Equivalent ellipsoid semi-axes `[a, b, c]`.
- `use_shape_volume`
  - If true, ellipsoid volume becomes the buoyancy reference volume.
- `effective_cd_linear`
  - Per-axis translational drag coefficients used to form quadratic damping.
- `effective_cd_angular`
  - Per-axis rotational drag coefficients.
- `added_mass_scale_linear`
  - Scale factors applied to the ellipsoid translational added-mass baseline.
- `added_mass_scale_angular`
  - Scale factors applied to the rotational added-inertia baseline.
- `linear_damping_ratio_linear`
  - Converts the quadratic translational damping baseline into a linearized
    translational damping term around a reference speed.
- `linear_damping_ratio_angular`
  - Same idea for angular damping.
- `reference_speed_linear`
  - Reference translational speed used for linearization.
- `reference_speed_angular`
  - Reference angular speed used for linearization.

### How the ellipsoid baseline is used

The helper builds:

- ellipsoid volume
- projected frontal areas
- depolarization factors
- translational added mass
- rotational added inertia approximation
- translational and rotational quadratic damping
- linearized damping derived from those baselines

Then those values are fed into the existing 6-DOF coefficient-based model.

So the final structure is:

- geometry-based baseline
- optional per-axis coefficient override
- runtime application through `apply_underwater_wrench()`

This is a hybrid model.

## Thruster Physics Parameters

These are not MuJoCo engine parameters. They are actuator-model parameters used
by the simulator on top of MuJoCo.

Defined in:

- [thruster_params.json](/Users/kanghyunmin/Desktop/uuv_sim/uuv_mujoco/v2.2/thruster_params.json#L1)

### Global thruster parameters

- `deadzone`
  - Command threshold below which thrust is forced to zero.
- `tau_up`
  - Time constant for thrust increase.
- `tau_down`
  - Time constant for thrust decrease.
- `reverse_asymmetry`
  - Reverse thrust scaling relative to forward thrust.
- `reaction_torque_gain`
  - Gain for propeller reaction torque.
- `forward_poly`
  - Forward thrust shaping polynomial.
- `reverse_poly`
  - Reverse thrust shaping polynomial.
- `command_limit`
  - Saturation limit on normalized thrust command.

### Per-thruster parameters

- `gain_scale`
  - Small per-thruster correction factor.
- `speed_sample`
  - Calibration record used when the gains were identified.

### Runtime use

These parameters are applied in:

- [hydrodynamics_helpers.py](/Users/kanghyunmin/Desktop/uuv_sim/uuv_mujoco/v2.2/hydrodynamics_helpers.py#L54)
- [hydrodynamics_helpers.py](/Users/kanghyunmin/Desktop/uuv_sim/uuv_mujoco/v2.2/hydrodynamics_helpers.py#L63)
- [run_urdf_full.py](/Users/kanghyunmin/Desktop/uuv_sim/uuv_mujoco/v2.2/run_urdf_full.py#L1025)

They control:

- lag
- deadzone
- saturation
- nonlinear thrust-force conversion
- reaction torque

## Runtime Override Parameters

Some physics-related parameters can be changed from the CLI.

Defined in:

- [run_urdf_full.py](/Users/kanghyunmin/Desktop/uuv_sim/uuv_mujoco/v2.2/run_urdf_full.py#L41)

Important overrides:

- `--profile`
  - Selects `sim_simple`, `sim_fast`, `sim_real`, or `sim_hover`.
- `--profile-file`
  - Uses a different profile JSON.
- `--thruster-voltage`
  - Chooses the nearest voltage curve from the thruster performance file.
- `--buoyancy-scale`
  - Runtime buoyancy override.
- `--disable-thruster-perf`
  - Forces a simpler thrust mapping and ignores the performance curve file.
- `--ros2-sensor-hz`
  - Affects bridge-side publish rate, not the core MuJoCo physics step.
- `--sitl-mavlink-servo-hz`
  - Affects RC/servo command update rate in SITL mode.
- `--thruster-loop-hz`
  - Sets the rate of thrust-force updates, independent from the MuJoCo physics
    integrator step.

## What Is Built-In MuJoCo and What Is Not

### Built-in MuJoCo in this project

- rigid-body dynamics
- contacts and joints
- numerical integration
- global medium properties from `option`

### Not currently used from MuJoCo's fluid model

- geom-level `fluidshape="ellipsoid"`
- geom-level `fluidcoef`
- MuJoCo-only passive fluid interaction as the main underwater model

### Custom in this project

- buoyancy
- CoB restoring torque
- added mass
- added-mass Coriolis
- linear and quadratic hydrodynamic damping
- current-relative flow
- thruster lag, deadzone, asymmetry, and reaction torque
- ellipsoid-based coefficient generation for `sim_real`

## Practical Interpretation

If you are tuning this simulator, the parameter priority is usually:

1. `sim_profiles.json`
   - underwater behavior
2. `thruster_params.json`
   - actuation feel and responsiveness
3. scene `option`
   - medium defaults and integrator behavior
4. CLI overrides
   - quick experiments without editing files

If you need a model closer to MuJoCo's official built-in fluid-force path, the
next step would be to define `fluidshape="ellipsoid"` and `fluidcoef` on the
relevant geoms and compare that passive model against the repository's current
custom wrench model.

## References

- MuJoCo XML Reference:
  <https://mujoco.readthedocs.io/en/3.2.6/XMLreference.html>
- In particular, see the `geom` attributes `fluidshape` and `fluidcoef`.
