# Runtime Freshness, Real-Start, And Thruster Param Split

Date: 2026-06-07

Scope: active runtime code under `uuv_mujoco/current`, with the physical
directory still backed by `uuv_mujoco/v2.2` for compatibility.

## Runtime Freshness Contract

- User-facing README run commands now use root wrappers:
  `uuv_mujoco/start_sitl_mujoco.sh`, `uuv_mujoco/reset_sim.sh`, and the GUI
  launcher, instead of sending users into the compatibility backing directory.
- `uuv_mujoco/RUNTIME_VERSION.json` records `source_branch=uuv_sim`, the GitHub
  remote, and the branch freshness policy.
- `tools/audit_code_contract_source_identity.py` now reports active branch,
  local HEAD, `origin/uuv_sim` HEAD, and dirty runtime paths as source-contract
  evidence.

## Real-Start Split

- `sim/runtime/real_start.py` is now a compatibility facade.
- New focused modules own env parsing types, target loading, measurement
  extraction, pressure calibration, payload/status assembly, and status
  publishing.
- Import surfaces used by `real_start_runtime.py` and `initial_state.py` are
  preserved.

## Thruster Param Split

- `sim/physics/thruster_params.py` is now a compatibility facade.
- New focused modules own default payloads, JSON loading, per-thruster reset and
  application, optional logging, and direct gain overrides.
- Import surfaces used by `thruster_param_runtime.py` and
  `physics_runtime_factory.py` are preserved.

## Validation

- `RUNTIME_VERSION.json` passes JSON parsing.
- `setup/03_setup_uuv_mujoco.sh` passes `bash -n`.
- `tools/audit_code_contract_source_identity.py` passes `py_compile`.
- Source contract audit after the runtime freshness change reports:
  `fail=0`, `pass=11`, `warn=5`.
- Refactor inventory no longer lists `sim/runtime/real_start.py` or
  `sim/physics/thruster_params.py` in the top 30 hotspot list.
