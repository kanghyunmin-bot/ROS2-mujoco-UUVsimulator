# Runtime Freshness Refresh

Date: 2026-06-07

## Problem

The live simulator code still sits behind the compatibility directory name
`uuv_mujoco/v2.2`. Even when the git source was current, launch logs could look
stale because runtime version metadata was only checked manually.

## Change

- Added `tools/runtime_freshness_version.py`.
- Extended `tools/check_runtime_freshness.py` with `--refresh-version`.
- Runtime freshness now compares `HEAD` with `origin/uuv_sim`, verifies
  `uuv_mujoco/current`, and refreshes `uuv_mujoco/RUNTIME_VERSION.json` only when
  the source/runtime checks have no hard failures.
- GUI and SITL launch wrappers now run:

```text
check_runtime_freshness.py --fetch --refresh-version --warn-only
```

## Contract

`v2.2` remains only the compatibility backing directory. Operators should treat
`uuv_mujoco/current` plus the recorded `origin/uuv_sim` source head as the active
runtime contract.

## Verification

```text
python3 -m compileall -q ...
python3 uuv_mujoco/current/tools/check_runtime_freshness.py --fetch --refresh-version
zsh -n run_control_gui.sh run_control_gui_ubuntu.sh uuv_mujoco/current/start_sitl_mujoco_mj311.sh
PYTHONPATH=uuv_mujoco/current/tools python3 uuv_mujoco/current/tools/audit_code_contract_sources.py --out-dir /private/tmp/uuv_source_audit_freshness
python3 uuv_mujoco/current/tools/check_runtime_readiness_policy.py
```

Result:

```text
runtime freshness: PASS
contract source audit: fail=0 pass=11 warn=5
runtime_readiness_policy=PASS
```
