# SITL Auto-Ready Split

Date: 2026-06-07

## Scope

Split `bridge/sitl_auto_ready_runtime.py` into focused helpers without changing
the auto arm/mode sequence or the method names exported through
`bridge/sitl_commanding.py`.

## Files

- `bridge/sitl_auto_ready_runtime.py`
  - Compatibility export surface.
- `bridge/sitl_auto_ready_extnav.py`
  - ExternalNav readiness gate.
- `bridge/sitl_auto_ready_state.py`
  - Auto-ready state update and log throttling.
- `bridge/sitl_auto_ready_neutral.py`
  - Neutral RC frame construction and neutral RC priming sends.
- `bridge/sitl_auto_ready_sequence.py`
  - Auto-ready arm/mode sequence orchestration.

## Contract Notes

- The sequence still exits when disabled or in plant replay mode.
- RC override readiness is still checked before ExternalNav readiness.
- ExternalNav readiness still uses required/enabled/fault, last-send age, grace
  window, and minimum rate gates.
- Neutral RC priming still throttles at 0.20 s and sends only to resolved
  MAVLink command targets.
- Neutral RC frame construction still uses `neutral_rc_override_frame()`, which
  keeps channels 1-8 at neutral PWM and channels 9-18 at zero.

## Verification

```text
sitl auto-ready split smoke: PASS
python3 -m compileall -q sim/current uuv_control_gui.py: PASS
audit_code_contract_sources.py: fail=0 pass=11 warn=5
check_runtime_readiness_policy.py: PASS
verify_ardusub_thruster_contract.py --quiet: PASS
physics_contract_audit.py --simulate-s 0.05: PASS
git diff --check: PASS
refactor_inventory.py: bridge/sitl_auto_ready_runtime.py removed from top 40
```
