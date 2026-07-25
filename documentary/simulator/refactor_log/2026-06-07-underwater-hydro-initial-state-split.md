# 2026-06-07 Underwater Hydrodynamics And Initial State Split

## Scope

- Split underwater residual hydrodynamics into body-frame wrench calculations,
  MuJoCo world-frame application, CFD debug logging, residual-family dispatch,
  custom hydrodynamics dispatch, and relative acceleration tracking.
- Split initial runtime state setup into typed outputs, hold/real-start policy
  resolution, ordered initial request application, and Bar30 pressure
  calibration.

## Contract Notes

- Residual, Fossen, and CFD hydrodynamic coefficients and formulas were not
  intentionally changed.
- Fossen added-mass remains scaled before application through the immersed
  added-mass matrix, so the final MuJoCo wrench application uses a neutral
  `submerged=1.0` multiplier for that already-scaled wrench.
- `underwater_hydrodynamics_residual.py` and
  `underwater_hydrodynamics_runtime.py` remain compatibility import surfaces.
- `configure_initial_runtime_state()` remains the public initial-state setup
  entry point.

## Verification

- Residual hydrodynamics fake-runtime smoke checked body force/torque values
  against the original coefficient formula.
- Relative acceleration smoke checked both disabled and custom-hydrodynamics
  active paths.
- Initial runtime state MuJoCo smoke loaded `tank_current_scene.xml`, configured
  initial Bar30 depth and hold state, and checked the resulting Bar30 depth.
- `python3 -m compileall -q sim/current uuv_control_gui.py`
- `git diff --check`
- `python3 sim/current/tools/audit_code_contract_sources.py`
- `python3 sim/current/tools/check_runtime_readiness_policy.py`
- `python3 sim/current/tools/verify_ardusub_thruster_contract.py --quiet`
- `/Users/kanghyunmin/.venvs/mujoco311/bin/python sim/current/tools/physics_contract_audit.py --simulate-s 0.05`

## Inventory Effect

- `sim/runtime/underwater_hydrodynamics_residual.py` left the top hotspot list.
- `sim/runtime/underwater_hydrodynamics_runtime.py` left the top hotspot list.
- `sim/runtime/initial_state.py` left the top hotspot list.
