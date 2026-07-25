# Thruster Performance Loader Split

Date: 2026-06-07

Scope: active runtime under `sim/current`, backed by the compatibility
directory `uuv_mujoco/v2.2`.

## What Changed

- Reduced `sim/physics/thruster_performance_loader.py` to public loader
  orchestration.
- Added `sim/physics/thruster_performance_payload.py` for JSON file IO and
  loaded-curve logging.
- Added `sim/physics/thruster_performance_curves.py` for curve parsing,
  finite sorting, nearest-voltage selection, and legacy dict update payloads.

## Contract Preserved

- `load_thruster_performance_config()` keeps the same call signature and legacy
  dict return shape.
- Missing/invalid JSON still falls back to
  `default_thruster_performance_config()`.
- Usable curves are still finite-filtered, sorted by PWM, and selected by
  nearest requested voltage.
- Direct mode still logs that raw PWM maps through the T200 curve.

## Verification

```text
python3 -m compileall -q \
  sim/current/sim/physics/thruster_performance_loader.py \
  sim/current/sim/physics/thruster_performance_payload.py \
  sim/current/sim/physics/thruster_performance_curves.py \
  sim/current/sim/physics/thruster_performance.py \
  sim/current/sim/physics/thruster_performance_selector.py

PYTHONPATH=/Users/kanghyunmin/Desktop/uuv_sim/sim/current \
python3 - <<'PY'
# thruster_performance_loader_smoke PASS
# thruster_performance_real_config_smoke PASS 22.2 201
PY
```

Result: the old thruster performance loader hotspot no longer appears in the
top 40 hotspot inventory.
