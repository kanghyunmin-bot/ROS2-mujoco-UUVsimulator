# Linear Algebra Identification Review - 2026-06-01

This note records the algorithmic direction for the real-RCOU plant replay
problem:

```text
real RCOU -> MuJoCo plant -> simulated sensors
real sensors - simulated sensors = residual r
parameter sensitivity J = d(sim sensor) / d(parameter)
```

The current problem should not be solved by blind least squares over every
available coefficient.  It should be solved as a rank-revealing, modal
identification problem.

## Literature Takeaways

### Practical Identifiability And Sloppy Directions

Source:

- Apgar et al., "Sloppy models, parameter uncertainty, and the role of
  experimental design", Molecular BioSystems, 2010:
  https://pmc.ncbi.nlm.nih.gov/articles/PMC3505121/

Relevant point:

- The Hessian/Fisher matrix defines an uncertainty ellipsoid.  Long ellipsoid
  axes are weakly determined parameter combinations; short axes are well
  determined combinations.
- In our notation, for the locally linearized plant replay problem:

```text
A = W J
F = A^T A
F eigenvalues = sigma_i^2 from SVD(A)
V_i = identifiable parameter combination
U_i = sensor/time residual mode excited by V_i
```

Consequence:

- If a residual lies mostly outside the column space of `A`, no least-squares
  solve can fix it.
- If several parameters are nearly collinear in `A`, fitting them together is
  numerically underdetermined even if each has high individual correlation with
  the residual.

### AUV Input Design

Sources:

- Nouri and Valadi, "Robust input design for nonlinear dynamic modeling of
  AUV", ISA Transactions, 2017:
  https://www.sciencedirect.com/science/article/abs/pii/S0019057817302288
- Nouri et al., "Optimal input design for hydrodynamic derivatives estimation
  of nonlinear dynamic model of AUV", Nonlinear Dynamics, 2018:
  https://doi.org/10.1007/s11071-017-3611-1

Relevant point:

- AUV hydrodynamic derivative identification depends strongly on informative
  inputs.  Input design is treated as an information-matrix problem under input
  and output constraints.

Consequence:

- If the 90s real bag does not excite a parameter direction, that direction
  cannot be recovered from the bag.  The correct response is not a bigger LS
  solve; it is either:
  1. fix that parameter from physics/CFD/manual measurement, or
  2. create another experiment/command segment that excites that mode.

### AUV Hydrodynamic Coefficients From Experiment And CFD

Sources:

- Cardenas and de Barros, "Estimation of AUV Hydrodynamic Coefficients Using
  Analytical and System Identification Approaches", IEEE Journal of Oceanic
  Engineering, 2020:
  https://doi.org/10.1109/joe.2019.2930421
- Tang et al., "Estimation of the hydrodynamic coefficients of the
  complex-shaped autonomous underwater vehicle TUNA-SAND", Journal of Marine
  Science and Technology, 2009:
  https://eprints.soton.ac.uk/400022/
- "Estimation of hydrodynamic coefficients and simplification of the depth
  model of an AUV using CFD and sensitivity analysis", Ocean Engineering, 2022:
  https://www.sciencedirect.com/science/article/pii/S0029801822016614

Relevant point:

- The useful coefficients are selected by how loads and motions couple across
  6DOF, not by tuning one axis in isolation.
- CFD is useful as a bounded prior and for coefficient importance, but the
  final dynamic response still needs validation against measured trajectories.

Consequence:

- The ellipsoid/HAN/CFD layer should generate candidate coefficient blocks and
  priors.  It should not bypass plant replay.  Every candidate still has to
  pass the RCOU plant replay sensor overlay.

### Data-Driven State/Input Separation

Source:

- Proctor, Brunton, and Kutz, "Dynamic Mode Decomposition with Control", SIAM
  Journal on Applied Dynamical Systems, 2016:
  https://doi.org/10.1137/15M1013857

Relevant point:

- DMDc separates intrinsic dynamics from actuation using state snapshots and
  control inputs, with SVD-based low-rank structure.

Consequence:

- DMDc/SINDy-style tools are useful as a black-box diagnostic to reveal missing
  coupling terms.  They should not replace the grey-box Fossen/MuJoCo model as
  the accepted simulator contract.

## Current Code-Level Finding

New audit tool:

```text
/Users/kanghyunmin/Desktop/uuv_sim/uuv_mujoco/v2.2/debug/controller_parity_412/modal_case_matrix_search.py
```

This tool computes:

```text
A = target-scaled J
b = target-scaled residual
F = A^T A
g = A^T b
A = U diag(sigma) V^T
```

and enumerates small parameter subsets ranked by:

```text
energy_reduction = 1 - ||b - A_S dx||^2 / ||b||^2
prediction_corr = corr(A_S dx, b)
condition_number = sigma_max / sigma_min
max_pair_collinearity = max |cos(A_i, A_j)|
```

Generated reports:

```text
/Users/kanghyunmin/Desktop/uuv_sim/uuv_mujoco/v2.2/debug/controller_parity_412/outputs/plant_replay_90s_t200direct_fix_20260601/sensor_overlay/modal_case_matrix_search_fossen_horizontal.md
/Users/kanghyunmin/Desktop/uuv_sim/uuv_mujoco/v2.2/debug/controller_parity_412/outputs/plant_replay_90s_t200direct_fix_20260601/sensor_overlay/modal_case_matrix_search_fossen_depth.md
/Users/kanghyunmin/Desktop/uuv_sim/uuv_mujoco/v2.2/debug/controller_parity_412/outputs/plant_replay_90s_t200direct_fix_20260601/sensor_overlay/modal_case_matrix_search_fossen_all.md
```

Current full90 Fossen residual sensitivity result:

| target block | A shape | condition | rank | best energy reduction | best prediction corr | conclusion |
| --- | ---: | ---: | ---: | ---: | ---: | --- |
| horizontal | 7128 x 16 | 7.68 | 16 | 0.037 | 0.202 | Fossen residual block alone does not span yaw/sway residual. |
| depth | 7128 x 16 | 8.04 | 16 | 0.054 | 0.242 | It mostly hits gyro_y, weakly fixes baro/depth. |
| all | 17820 x 16 | 6.91 | 16 | 0.061 | 0.248 | Low residual coverage; not an accepted fit direction. |

Also confirmed:

```text
fossen_residual_full90 baseline vs target baseline: maxdiff = 0
added_mass_residual_full90 baseline vs fossen target baseline: maxdiff = 1.012867855
environment_current_full90 baseline vs fossen target baseline: maxdiff = 0.562567752
dynamic_yaw_body_inertia_z_full90 baseline vs fossen target baseline: maxdiff = 0.604361567
```

Therefore these sensitivity batches must not be concatenated into one global
Jacobian until they are regenerated from the same fixed T200-direct baseline.

## Decision

The next valid algorithm is:

1. Regenerate one shared fixed-baseline sensitivity campaign from the current
   accepted plant replay contract.
2. Build one `J` from only same-baseline perturbations.
3. Normalize targets by sensor scale/noise.
4. Compute `A^T A`, `A^T b`, SVD, and column collinearity.
5. Partition parameters into modal blocks:
   - horizontal yaw/sway/gyro_z/DVL-x/y,
   - vertical heave/pitch/baro/DVL-z/gyro_y,
   - hydrostatic restoring and inertial attitude.
6. Enumerate small subsets and keep only cases with:
   - positive residual energy reduction,
   - positive prediction correlation,
   - acceptable condition number,
   - no severe internal collinearity.
7. Run a bounded damped/truncated SVD fit only for those subsets.
8. Validate by re-running plant replay, not by accepting the linear prediction.

This is the point where least squares becomes valid.  Before this gate, least
squares can easily produce a coefficient vector that improves one metric while
making the real plant behavior worse.
