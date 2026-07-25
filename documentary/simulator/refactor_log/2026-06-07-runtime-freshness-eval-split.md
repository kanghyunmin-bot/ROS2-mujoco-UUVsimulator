# Runtime Freshness Evaluator Split

Date: 2026-06-07

Scope: active runtime under `sim/current`, backed by the compatibility
directory `uuv_mujoco/v2.2`.

## What Changed

- Reduced `tools/runtime_freshness_eval.py` to the public report assembler.
- Added `tools/runtime_freshness_issue.py` for issue payload construction.
- Added `tools/runtime_freshness_runtime_checks.py` for `current` alias,
  explicit runtime-dir, and runner existence checks.
- Added `tools/runtime_freshness_source_checks.py` for branch/fetch/remote-head
  checks.
- Added `tools/runtime_freshness_dirty_checks.py` for dirty worktree and
  ArduPilot submodule status warnings.
- Added `tools/runtime_freshness_version_checks.py` for
  `RUNTIME_VERSION.json` consistency checks.

## Contract Preserved

- `check_runtime_freshness.py` still imports `evaluate_freshness()` from
  `runtime_freshness_eval.py`.
- Existing report fields are preserved.
- Existing issue ids are preserved, including `direct_v22_runtime`,
  `working_tree_dirty`, `active_runtime_dirty`, and
  `ardupilot_submodule_not_recorded_pointer`.
- `current` versus compatibility `v2.2` remains a freshness contract, not a
  folder-name claim.

## Verification

```text
python3 -m compileall -q \
  sim/current/tools/runtime_freshness_eval.py \
  sim/current/tools/runtime_freshness_issue.py \
  sim/current/tools/runtime_freshness_runtime_checks.py \
  sim/current/tools/runtime_freshness_source_checks.py \
  sim/current/tools/runtime_freshness_dirty_checks.py \
  sim/current/tools/runtime_freshness_version_checks.py \
  sim/current/tools/check_runtime_freshness.py

PYTHONPATH=/Users/kanghyunmin/Desktop/uuv_sim/sim/current/tools \
python3 - <<'PY'
# Forced-good and forced-bad evaluator smoke:
# runtime_freshness_eval_smoke PASS
PY

python3 sim/current/tools/check_runtime_freshness.py \
  --workspace /Users/kanghyunmin/Desktop/uuv_sim \
  --runtime-dir /Users/kanghyunmin/Desktop/uuv_sim/sim/current \
  --fetch --refresh-version --warn-only
```

Result: freshness remains `WARN current-dirty`, with local `HEAD` equal to
`origin/uuv_sim` and the evaluator removed from the top 30 hotspot inventory.
