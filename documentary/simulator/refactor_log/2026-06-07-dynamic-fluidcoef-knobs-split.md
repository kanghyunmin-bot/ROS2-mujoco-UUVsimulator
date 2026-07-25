# Dynamic Fluidcoef Runtime Knob Split

Date: 2026-06-07

Scope: active runtime `sim/current`, backed by the compatibility
directory `uuv_mujoco/v2.2`.

## Change

`sim/physics/dynamic_fluidcoef_runtime_knobs.py` is now a compatibility facade.
The previously mixed update/transient parsing logic is split into focused
modules:

- `sim/physics/dynamic_fluidcoef_update_knobs.py`
- `sim/physics/dynamic_fluidcoef_transient_mode.py`
- `sim/physics/dynamic_fluidcoef_transient_thresholds.py`
- `sim/physics/dynamic_fluidcoef_transient_mask.py`

The public functions remain:

- `configure_dynamic_fluidcoef_update_knobs`
- `configure_dynamic_fluidcoef_transient_knobs`

This keeps `sim/physics/dynamic_fluidcoef_runtime_config.py` and existing
callers on the same API while separating update rate, transient mode,
thresholds, and coefficient masks.

## Verification

Commands run:

```text
python3 -m compileall -q sim/current/sim/physics/dynamic_fluidcoef_runtime_knobs.py ...
PYTHONPATH=sim/current python3 - <<'PY'  # fake runtime knob smoke
python3 sim/current/tools/refactor_inventory.py --root sim/current --limit 20
python3 sim/current/tools/check_runtime_freshness.py --workspace /Users/kanghyunmin/Desktop/uuv_sim --runtime-dir /Users/kanghyunmin/Desktop/uuv_sim/sim/current --fetch --refresh-version --warn-only
PYTHONPATH=sim/current/tools python3 sim/current/tools/audit_code_contract_sources.py --out-dir /private/tmp/uuv_source_audit_spaghetti_20260607_fluidcoef_split
python3 sim/current/tools/check_runtime_readiness_policy.py
python3 sim/current/tools/verify_ardusub_thruster_contract.py --quiet
python3 -m compileall -q sim/current uuv_control_gui.py
```

Results:

- Dynamic fluidcoef knob smoke: PASS
- Refactor inventory: `dynamic_fluidcoef_runtime_knobs.py` dropped out of the
  top 20 hotspot table.
- Runtime freshness: WARN by design because the active runtime is
  `current-dirty`.
- Source contract audit: `fail=0`, `pass=11`, `warn=5`.
- Runtime readiness policy: PASS.
- ArduSub thruster contract: OK.

## Contract Notes

This refactor does not change physics coefficients, fluid model behavior, or
the plant replay target. It only makes the dynamic-fluidcoef runtime knob
contract smaller and auditable before further plant tuning work.
