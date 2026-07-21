# Sim Profile Hydrodynamics Split

Date: 2026-06-07

Scope: active runtime under `uuv_mujoco/current` with compatibility backing in
`uuv_mujoco/v2.2`.

## Change

- Split hydrodynamics profile parsing and `build_hydrodynamics_config()` out of
  `physics/sim_profile_helpers.py` into
  `physics/sim_profile_hydrodynamics.py`.
- Moved `HydrodynamicsConfig`, `BodyComponent`, and `BuoyancyPoint` into
  `physics/sim_profile_types.py`.
- Kept `physics/sim_profile_helpers.py` as the public compatibility facade for
  profile aliases/defaults/loading and hydrodynamics builder exports.
- Updated `tools/audit_closed_loop_contract.py` to resolve
  `uuv_mujoco/current` first before falling back to the compatibility backing
  directory.

## Contract Notes

- No controller-parity observation point changed.
- No PWM remap, ALT_HOLD shim, or actuator output correction was introduced.
- The plant input contract remains raw ArduSub JSON servo packets.
- Historical evidence may still contain absolute `uuv_mujoco/v2.2` paths; live
  entry points should use `uuv_mujoco/current`.

## Validation

```text
python3 -m py_compile uuv_mujoco/current/physics/sim_profile_helpers.py \
  uuv_mujoco/current/physics/sim_profile_hydrodynamics.py \
  uuv_mujoco/current/physics/sim_profile_types.py
source ./.uuv_mujoco_env.sh && PYTHONPATH=uuv_mujoco/current "$MJ311_PYTHON" <profile public API smoke>
python3 -m compileall -q uuv_mujoco/current uuv_control_gui.py
source ./.uuv_mujoco_env.sh && "$MJ311_PYTHON" uuv_mujoco/current/run_uuv_mujoco.py --help
python3 uuv_mujoco/current/tools/check_runtime_readiness_policy.py
python3 uuv_mujoco/current/tools/verify_ardusub_thruster_contract.py --quiet
python3 uuv_mujoco/current/tools/audit_code_contract_sources.py --out-dir /private/tmp/uuv_code_contract_after_profile_split
python3 uuv_mujoco/current/tools/check_dev_os_compat.py --headless --target-os ubuntu
python3 uuv_mujoco/current/tools/audit_closed_loop_contract.py --json-out /private/tmp/uuv_closed_loop_contract_current.json
```

Results:

- Compile/import gates passed.
- Runtime readiness policy passed.
- Thruster contract passed.
- Code contract audit: `fail=0`, `pass=10`, `warn=5`.
- Dev OS compatibility: `fail=0`, `pass=16`, `warn=2`.
- Closed-loop contract audit followed `uuv_mujoco/current` and reported one
  remaining real/SITL parameter mismatch: `INS_POS1_X` real `0.145000` vs SITL
  contract `0.000000`.

## Inventory After Split

`physics/sim_profile_helpers.py` dropped out of the top 25 hotspot list.
`physics/sim_profile_hydrodynamics.py` is now `410 LOC / 55` branches with
`build_hydrodynamics_config()` at `136 LOC`.
