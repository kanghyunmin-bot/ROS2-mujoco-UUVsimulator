# Roll Stability Candidate Split

Date: 2026-06-07

Scope: active runtime tools under `uuv_mujoco/current/tools`.

## What Changed

- Moved the `Candidate` dataclass to `roll_stability_candidate_types.py`.
- Split candidate groups into:
  - `roll_stability_candidate_hydrostatic.py`
  - `roll_stability_candidate_ellipsoid.py`
  - `roll_stability_candidate_diagnostics.py`
  - `roll_stability_candidate_signs.py`
- Kept `roll_stability_candidates.py` as the public compatibility facade.
- Kept `roll_stability_candidate_catalog.py` as the small assembly layer for candidate ordering.

This is a structure-only refactor. It preserves candidate names, ordering,
profile updates, fluid angular scales, servo signs, and notes.

## Focused Smoke

```text
PYTHONPATH=/Users/kanghyunmin/Desktop/uuv_sim/uuv_mujoco/current/tools python3 - <<'PY'
from roll_stability_candidates import Candidate, default_candidates
# verifies 15 base candidates, 18 with sign checks, ordered names, selected
# servo signs, and representative hydro/ellipsoid values.
PY
python3 /Users/kanghyunmin/Desktop/uuv_sim/uuv_mujoco/current/tools/roll_stability_sweep.py --help
```

Results:

```text
roll_stability_candidates grouped smoke: PASS
roll_stability_sweep.py --help: PASS
```

## Contract Gates

```text
python3 -m compileall -q /Users/kanghyunmin/Desktop/uuv_sim/uuv_mujoco/current /Users/kanghyunmin/Desktop/uuv_sim/uuv_control_gui.py
PYTHONPATH=/Users/kanghyunmin/Desktop/uuv_sim/uuv_mujoco/current/tools python3 /Users/kanghyunmin/Desktop/uuv_sim/uuv_mujoco/current/tools/audit_code_contract_sources.py --out-dir /private/tmp/uuv_source_audit_spaghetti_20260607_candidate_split
python3 /Users/kanghyunmin/Desktop/uuv_sim/uuv_mujoco/current/tools/check_runtime_readiness_policy.py
python3 /Users/kanghyunmin/Desktop/uuv_sim/uuv_mujoco/current/tools/verify_ardusub_thruster_contract.py --quiet
/Users/kanghyunmin/miniconda3/envs/ros2_h311/bin/python /Users/kanghyunmin/Desktop/uuv_sim/uuv_mujoco/current/tools/physics_contract_audit.py --output-dir /private/tmp/uuv_physics_audit_spaghetti_20260607_candidate_split --simulate-s 0.05
git -C /Users/kanghyunmin/Desktop/uuv_sim diff --check
```

Results:

```text
contract_source_audit: {"fail": 0, "pass": 11, "warn": 5}
runtime_readiness_policy=PASS
[thruster-contract] OK
physics_contract_audit: wrote static_force_balance.csv/json
diff --check: PASS
```
