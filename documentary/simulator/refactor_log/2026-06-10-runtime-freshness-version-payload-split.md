# Runtime Freshness Version Payload Split

Date: 2026-06-10

## Change

- Split active-runtime version metadata construction out of
  `tools/runtime_freshness_version.py`.
- New focused owners:
  - `tools/runtime_freshness_version_constants.py`: policy text, runner paths,
    source-audit id, root launcher map, and compatibility note.
  - `tools/runtime_freshness_version_dirty.py`: dirty working-tree/runtime
    payload and runtime status classification.
  - `tools/runtime_freshness_version_paths.py`: active alias backing directory
    and root launcher payload.
  - `tools/runtime_freshness_version_payload.py`: final
    `RUNTIME_VERSION.json` payload assembly.
- Kept `tools/runtime_freshness_version.py` as the public refresh entry point
  and compatibility import surface for `build_runtime_version()`.

## Contract

- No controller-parity surface changed.
- No RC input/output, Bar30, SERVO_OUTPUT_RAW, thruster, or MuJoCo physics
  contract changed.
- The active-runtime freshness metadata shape is preserved.

## Verification

```text
env PYTHONPYCACHEPREFIX=/private/tmp/pycache python3 -m compileall -q sim/current/tools
env PYTHONPYCACHEPREFIX=/private/tmp/pycache python3 sim/current/tools/check_rc_frame_contract.py
env PYTHONPYCACHEPREFIX=/private/tmp/pycache python3 sim/current/tools/check_gui_readiness_contract.py
env PYTHONPYCACHEPREFIX=/private/tmp/pycache python3 sim/current/tools/check_gui_backend_selection.py
env PYTHONPYCACHEPREFIX=/private/tmp/pycache python3 sim/current/tools/check_runtime_readiness_policy.py
env PYTHONPYCACHEPREFIX=/private/tmp/pycache PYTHONPATH=sim/current/tools python3 sim/current/tools/audit_code_contract_sources.py --out-dir /private/tmp/uuv_source_audit_runtime_freshness_split_20260610
env PYTHONPYCACHEPREFIX=/private/tmp/pycache python3 sim/current/tools/refactor_inventory.py --root sim/current --format markdown --limit 12
```

Results:

- `rc_frame_contract=PASS`
- `gui_readiness_contract=PASS`
- `gui_backend_selection=PASS`
- `runtime_readiness_policy=PASS`
- source contract audit: `{"fail": 0, "pass": 11, "warn": 5}`
- `tools/runtime_freshness_version.py` is no longer in the top 12 hotspot list.
