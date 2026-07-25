# Control Loop Golden Compare Split

Date: 2026-06-07

Scope: active runtime tool code under `sim/current/tools`.

## Changes

- Split `tools/control_loop_golden_compare.py` into focused modules:
  - `tools/control_loop_golden_math.py`
  - `tools/control_loop_golden_thrusters.py`
  - `tools/control_loop_golden_fingerprint.py`
  - `tools/control_loop_golden_compare_logic.py`
- Kept `tools/control_loop_golden_compare.py` as the active CLI facade and
  preserved the legacy import names `build_fingerprint` and
  `compare_fingerprints`.

## Contract Notes

- No controller/plant observation point changed.
- The golden comparison still consumes `axis_rc_override_check` JSON summaries.
- Optional MuJoCo thruster debug CSV alignment still uses monotonic wall time and
  phase windows from the axis summary metadata.
- Candidate health `warn` still returns an overall warning instead of being
  hidden by the refactor.

## Verification

```bash
python3 -m py_compile \
  sim/current/tools/control_loop_golden_compare.py \
  sim/current/tools/control_loop_golden_math.py \
  sim/current/tools/control_loop_golden_thrusters.py \
  sim/current/tools/control_loop_golden_fingerprint.py \
  sim/current/tools/control_loop_golden_compare_logic.py
python3 sim/current/tools/control_loop_golden_compare.py --help
python3 sim/current/tools/control_loop_golden_compare.py \
  --candidate sim/current/logs/control_loop_golden_smoke_20260519_j_armwait/axis_check/axis_summary.json \
  --baseline sim/current/logs/control_loop_golden_smoke_20260519_j_armwait/axis_check/axis_summary.json \
  --thruster-csv sim/current/logs/control_loop_golden_smoke_20260519_j_armwait/mujoco_thruster_debug.csv \
  --baseline-thruster-csv sim/current/logs/control_loop_golden_smoke_20260519_j_armwait/mujoco_thruster_debug.csv \
  --out /private/tmp/uuv_control_loop_golden_compare_split_self_thrusters.json
python3 sim/current/tools/refactor_inventory.py --root sim/current --limit 25
```

Results:

- Self-compare with thruster debug returned `overall=warn` only because the
  candidate health in the fixture is already `warn`; metric failures stayed
  empty.
- The split fingerprint marked `has_thruster_debug=True` and compared `1155`
  metrics.
- `tools/control_loop_golden_compare.py` dropped from `357 LOC / 53` branches to
  a `103 LOC` facade and no longer appears in the top 25 hotspot inventory.
