# SITL PWM Runtime Split

Date: 2026-06-07

## Scope

Split `bridge/sitl_pwm_runtime.py` into focused plant-input policy helpers
without changing JSON/MAVLink/replay PWM routing or the `_handle_pwm_values`
method bound onto `SitlTransport`.

## Files

- `bridge/sitl_pwm_runtime.py`
  - Compatibility export surface.
- `bridge/sitl_pwm_source_policy.py`
  - Plant replay ownership and external override gates.
- `bridge/sitl_pwm_safety.py`
  - Disarmed nonneutral neutralization and all-min neutralization.
- `bridge/sitl_pwm_activity.py`
  - Neutral/nonneutral tracking and stale-neutral warnings.
- `bridge/sitl_pwm_output.py`
  - Plant callback dispatch and debug PWM logging.
- `bridge/sitl_pwm_frame_handler.py`
  - Top-level frame policy orchestration.

## Contract Notes

- `replay_rcout*` remains the only external source prefix that owns plant replay.
- In plant replay mode, JSON/MAVLink PWM is ignored and recorded RCOUT/PWM stays
  authoritative.
- Disarmed non-replay PWM is neutralized before plant callbacks.
- All-min non-replay motor frames are treated as neutral.
- Replay RCOUT frames are not neutralized by disarmed/all-min SITL safety gates.

## Verification

```text
sitl pwm runtime split smoke: PASS
python3 -m compileall -q sim/current uuv_control_gui.py: PASS
audit_code_contract_sources.py: fail=0 pass=11 warn=5
check_runtime_readiness_policy.py: PASS
verify_ardusub_thruster_contract.py --quiet: PASS
physics_contract_audit.py --simulate-s 0.05: PASS
git diff --check: PASS
refactor_inventory.py: bridge/sitl_pwm_runtime.py removed from top 45
```
