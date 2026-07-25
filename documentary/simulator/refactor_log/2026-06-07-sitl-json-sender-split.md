# SITL JSON Sender Split

Date: 2026-06-07

## Scope

Split `bridge/sitl_json_sender.py` into focused payload validation,
diagnostics, encoding/IO, and send-policy helpers without changing the ArduSub
JSON sensor packet contract.

## Files

- `bridge/sitl_json_sender.py`
  - Compatibility export surface.
- `bridge/sitl_json_sender_validation.py`
  - Finite-value validation and one-shot non-finite warning.
- `bridge/sitl_json_sender_diagnostics.py`
  - Sensor sample, send status, send warning, and send failure diagnostics.
- `bridge/sitl_json_sender_io.py`
  - Compact JSON encoding, target selection, send call, and receiver-state
    synchronization.
- `bridge/sitl_json_sender_runtime.py`
  - Top-level `_send_sitl_json_payload(...)` policy.

## Contract Notes

- Compact JSON encoding still uses `separators=(",", ":")` and appends one
  newline, preserving the ArduSub 4.1.2 lightweight parser contract.
- Non-finite payloads are still skipped and warned once.
- Send target selection still prefers the discovered SITL client endpoint and
  falls back to `sitl_send_addr`.
- `sitl_sock`, send counter, and send target still mirror
  `JsonServoReceiver` state after every successful send.
- Send status, target-change, zero-byte, and failure diagnostics keep the same
  throttle intervals.

## Verification

```text
sitl json sender split smoke: PASS
python3 -m compileall -q sim/current uuv_control_gui.py: PASS
audit_code_contract_sources.py: fail=0 pass=11 warn=5
check_runtime_readiness_policy.py: PASS
verify_ardusub_thruster_contract.py --quiet: PASS
physics_contract_audit.py --simulate-s 0.05: PASS
git diff --check: PASS
check_runtime_freshness.py --warn-only: WARN current-dirty, dirty_paths=674, active_runtime_dirty_paths=656
refactor_inventory.py: bridge/sitl_json_sender.py removed from top 45
```
