# Thruster Debug Runtime Split

Date: 2026-06-07

Scope: active runtime `uuv_mujoco/current`, backed by the compatibility
directory `uuv_mujoco/v2.2`.

## Change

`sim/runtime/thruster_debug_runtime.py` is now a small runtime facade that owns
sampling cadence and the public debug writer API.

CSV file handling moved to:

- `sim/runtime/thruster_debug_file.py`

CSV row construction moved to:

- `sim/runtime/thruster_debug_rows.py`

Sampling cadence moved to:

- `sim/runtime/thruster_debug_schedule.py`

Single-row emission moved to:

- `sim/runtime/thruster_debug_emit.py`

The public class remains:

- `ThrusterDebugRuntime`

## Verification

Commands run:

```text
python3 -m compileall -q uuv_mujoco/current/sim/runtime/thruster_debug_runtime.py ...
PYTHONPATH=uuv_mujoco/current python3 - <<'PY'  # fake data debug CSV smoke
python3 uuv_mujoco/current/tools/refactor_inventory.py --root uuv_mujoco/current --limit 25
```

Results:

- Thruster debug runtime smoke: PASS.
- Header and row widths match `build_thruster_debug_header`.
- The 20 Hz sampler writes at `t=0.00` and `t=0.05` while skipping `t=0.01`.
- `mj_forward` is still called only for emitted rows, preserving the historical
  "after ctrl/xfrc staging and before integration" force-breakdown contract.
- `thruster_debug_runtime.py` dropped out of the top 25 hotspot table.
- Follow-up schedule smoke: PASS.  The sample cursor still emits at `t=0.00`
  and advances through missed sample slots using the same 20 Hz hold policy.

## Contract Notes

This refactor does not change thruster force calculation, ArduSub PWM mapping,
or plant input behavior. It only separates debug CSV file IO, sample scheduling,
row payload construction, and row emission so future force-contract audits can
test each stage directly.
