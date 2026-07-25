# References For Linear Algebra Plant Identification

Date: 2026-06-01

These are the references used for the current linear-algebra identification
direction:

```text
real RCOU -> MuJoCo plant -> sensors
r = real - sim
J = d(sim) / d(parameter)
A = WJ
F = A^T A
A = U diag(sigma) V^T
```

The BibTeX file is:

```text
/Users/kanghyunmin/Desktop/uuv_sim/uuv_mujoco/v2.2/research_workspace/04_paper_evidence/references_20260601.bib
```

## Core Identifiability / Matrix Design

### Apgar2010SloppyModels

Joshua F. Apgar, David K. Witmer, Forest M. White, and Bruce Tidor.
"Sloppy models, parameter uncertainty, and the role of experimental design."
Molecular BioSystems, 6(10), 1890-1900, 2010.
DOI: `10.1039/B918098B`

Use in this project:

- Justifies using the Hessian/Fisher matrix and SVD spectrum before accepting
  coefficient fits.
- Maps directly to our `A^T A` and `sigma_i^2` checks.
- Explains why many coefficients can be individually plausible but only a few
  combinations are actually identifiable from one dataset.

## AUV Input Design / Persistent Excitation

### Nouri2017RobustInputDesignAUV

Nowrouz Mohammad Nouri and Mehrdad Valadi.
"Robust input design for nonlinear dynamic modeling of AUV."
ISA Transactions, 70, 288-297, 2017.
DOI: `10.1016/j.isatra.2017.02.006`

Use in this project:

- Supports the claim that input design is part of model identification, not a
  post-processing detail.
- Relevant to the question of whether the 90s RCOU bag excites yaw/sway/heave
  modes enough to identify coefficients.

### Nouri2018OptimalInputDesignHydroDerivatives

Nowrouz Mohammad Nouri, Mehrdad Valadi, and Jafar Asgharian.
"Optimal input design for hydrodynamic derivatives estimation of nonlinear
dynamic model of AUV."
Nonlinear Dynamics, 92(2), 139-151, 2018.
DOI: `10.1007/s11071-017-3611-1`

Use in this project:

- Supports building an information-matrix objective for hydrodynamic
  derivative estimation.
- If `J` has weak singular directions, the fix is not blind LS; it is either
  better excitation or fixing weak directions from prior physics/CFD.

## AUV Hydrodynamic Coefficient Identification

### Cardenas2020AUVHydrodynamicCoefficients

Persing Cardenas and Ettore A. de Barros.
"Estimation of AUV Hydrodynamic Coefficients Using Analytical and System
Identification Approaches."
IEEE Journal of Oceanic Engineering, 45(4), 1157-1176, 2020.
DOI: `10.1109/JOE.2019.2930421`

Use in this project:

- Provides a precedent for combining analytical modeling with system
  identification for AUV hydrodynamic coefficients.
- Supports our split between physics-prior coefficients and plant-replay
  validation.

### Tang2009TunaSandHydrodynamicCoefficients

Sulin Tang, Tamaki Ura, Takeshi Nakatani, Blair Thornton, and Tao Jiang.
"Estimation of the hydrodynamic coefficients of the complex-shaped autonomous
underwater vehicle TUNA-SAND."
Journal of Marine Science and Technology, 14(3), 373-386, 2009.
DOI: `10.1007/s00773-009-0055-4`

Use in this project:

- Relevant precedent for a complex-shaped AUV where hydrodynamic coefficients
  cannot be treated as simple scalar axis gains.
- Supports coefficient selection and validation around coupled 6DOF behavior.

### Safari2022CFDDepthSensitivity

Farhad Safari, Mansour Rafeeyan, and Mohammad Danesh.
"Estimation of hydrodynamic coefficients and simplification of the depth model
of an AUV using CFD and sensitivity analysis."
Ocean Engineering, 263, 112369, 2022.
DOI: `10.1016/j.oceaneng.2022.112369`

Use in this project:

- Directly supports CFD-assisted hydrodynamic coefficient estimation and
  sensitivity-based model simplification.
- Relevant to the planned HAN/CFD layer: CFD/HAN should propose bounded priors
  and sensitive terms, then plant replay must validate them.

## Data-Driven Dynamics / Modal Diagnostics

### Proctor2016DMDc

Joshua L. Proctor, Steven L. Brunton, and J. Nathan Kutz.
"Dynamic Mode Decomposition with Control."
SIAM Journal on Applied Dynamical Systems, 15(1), 142-161, 2016.
DOI: `10.1137/15M1013857`

Use in this project:

- Useful as a diagnostic alternative for separating actuation-driven response
  from intrinsic dynamics.
- Not the final accepted physical model, but useful for finding missing
  coupling terms if the grey-box `J` cannot span the residual.

## Current Citation Mapping

| project claim | cite |
| --- | --- |
| Fit only identifiable parameter combinations; use `A^T A` and SVD before LS. | `Apgar2010SloppyModels` |
| If the RCOU bag does not excite a mode, that mode cannot be estimated from this bag alone. | `Nouri2017RobustInputDesignAUV`, `Nouri2018OptimalInputDesignHydroDerivatives` |
| Use analytical/physical coefficients plus system identification, not pure curve fitting. | `Cardenas2020AUVHydrodynamicCoefficients` |
| Complex-shaped UUV hydrodynamics require coupled 6DOF treatment. | `Tang2009TunaSandHydrodynamicCoefficients` |
| CFD/HAN should provide bounded priors and sensitivity-ranked terms. | `Safari2022CFDDepthSensitivity` |
| DMDc can diagnose missing low-rank actuation/dynamics modes. | `Proctor2016DMDc` |
