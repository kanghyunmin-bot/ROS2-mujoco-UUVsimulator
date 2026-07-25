# 03 HAN CFD

This folder documents the coefficient-identification layer.  Code and large
outputs remain under `/Users/kanghyunmin/Desktop/uuv_sim/UUV-HAN`.

## Active Code

- `/Users/kanghyunmin/Desktop/uuv_sim/UUV-HAN/scripts/make_coefficient_manifest.py`
- `/Users/kanghyunmin/Desktop/uuv_sim/UUV-HAN/scripts/make_perturbation_profiles.py`
- `/Users/kanghyunmin/Desktop/uuv_sim/UUV-HAN/scripts/run_plant_replay_sensitivity_batch.py`
- `/Users/kanghyunmin/Desktop/uuv_sim/UUV-HAN/scripts/fit_coefficients.py`
- `/Users/kanghyunmin/Desktop/uuv_sim/UUV-HAN/scripts/build_cfd_prior.py`
- `/Users/kanghyunmin/Desktop/uuv_sim/UUV-HAN/scripts/train_han_network.py`

## Current Model Boundary

- MuJoCo ellipsoid `fluidcoef` remains the base hydrodynamic approximation.
- Fossen-style residual terms are now available for coupled damping candidates.
- CFD creates bounded priors; it is not a direct final answer.
- HAN predicts bounded coefficient deltas; it must not output thruster force
  corrections.
- HAN/CFD must run after the contract gate.  Do not train or select profiles
  from `local_position`, `/mavros/imu/atm_pressure`, or warning-grade DVL-z as
  primary targets.
- Added-mass/residual candidates are allowed as generated profiles, but the
  main profile keeps them zero unless full90 plant replay beats baseline and
  the source-contract gate remains clean.

## Latest Result

The CFD prior with residual quadratic support modestly improved the aggregate
ranking but still failed the 99% gate.  It was therefore preserved as evidence
and not promoted into the main sim profile.

The added-mass residual hook is implemented but not accepted by validation.
That means the next useful HAN/CFD work is not blind coefficient search; it is
contract-gated identification on safe targets with repeated full90 validation.
