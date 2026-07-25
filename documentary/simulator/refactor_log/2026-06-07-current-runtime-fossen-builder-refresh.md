# Current Runtime And Fossen Builder Refresh

Date: 2026-06-07

Scope: active runtime alias and Fossen residual builder ownership.

## Why

The compatibility backing directory is still named `uuv_mujoco/v2.2`, which can
look stale during day-to-day work.  The actual live entry point is
`sim/current -> v2.2`, with freshness checked against the active git
branch before launcher execution.

## What changed

- Verified `sim/current -> v2.2` and refreshed
  `uuv_mujoco/RUNTIME_VERSION.json`.
- Confirmed local `HEAD` and `origin/uuv_sim` match.
- Split `sim/physics/fossen_residual_builders.py` internals so the facade no
  longer owns coefficient extraction, added-mass construction, activation
  gates, and logging helpers directly.

## Contract boundaries preserved

- No ArduPilot source or submodule pointer changes.
- No residual hydrodynamic coefficients or equations changed.
- No actuator remap, ALT_HOLD shim, or PWM masking was added.
- New runtime and validation commands continue to target
  `sim/current`, not historical absolute `v2.2` paths.

## Verification

```text
python3 -m compileall -q sim/current uuv_control_gui.py
PYTHONPATH=sim/current/tools python3 \
  sim/current/tools/audit_code_contract_sources.py \
  --out-dir /private/tmp/uuv_source_audit_fossen_refresh
python3 sim/current/tools/check_runtime_readiness_policy.py
python3 sim/current/tools/verify_ardusub_thruster_contract.py --quiet
python3 sim/current/tools/check_runtime_freshness.py \
  --workspace /Users/kanghyunmin/Desktop/uuv_sim \
  --runtime-dir /Users/kanghyunmin/Desktop/uuv_sim/sim/current \
  --fetch --refresh-version
/Users/kanghyunmin/miniconda3/envs/ros2_h311/bin/python \
  sim/current/tools/physics_contract_audit.py \
  --output-dir /private/tmp/uuv_physics_audit_fossen_refresh \
  --simulate-s 0
python3 sim/current/tools/check_dev_os_compat.py --headless --target-os ubuntu
git diff --check
python3 sim/current/tools/refactor_inventory.py --limit 25
```

Results:

- Compile: pass.
- Source contract audit: `fail=0`, `pass=11`, `warn=5`.
- Runtime readiness: pass.
- ArduSub thruster contract: pass.
- Runtime freshness: pass, `current -> v2.2`, local `HEAD` equals
  `origin/uuv_sim`.
- Static physics audit: neutral force balance still
  `net_down=+0.000N`, `required_scale=1.000000`.
- Ubuntu compatibility: `fail=0`, `pass=16`, `warn=2`
  (`docker_daemon`, `ros2_env`).
- `git diff --check`: pass.
