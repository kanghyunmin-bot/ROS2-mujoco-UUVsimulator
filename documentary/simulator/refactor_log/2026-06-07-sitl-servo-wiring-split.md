# SITL Servo Wiring Split

Date: 2026-06-07

Scope: behavior-neutral extraction of SITL/plant-replay PWM runtime wiring from
`run_uuv_mujoco.py`.

## Changed

- Extended `sim/runtime/sitl_servo_runtime.py` with
  `create_and_bind_sitl_servo_runtime()`.
- Kept the existing ArduSub vectored-6DOF raw channel map and servo signs as
  caller-provided inputs.
- Moved SITL servo handler and plant-replay RCOU handler registration out of
  `run_uuv_mujoco.py`.
- Stored the default servo timeout on `SitlServoRuntime` as `timeout_s`.

## Measured Impact

After the runtime loop and ROS bridge lifecycle split:

- `run_uuv_mujoco.py`: `775 LOC`, `25` branches, `main()` `694 LOC`.

After this split:

- `run_uuv_mujoco.py`: `763 LOC`, `18` branches, `main()` `682 LOC`.

## Validation

Commands run:

```bash
env PYTHONPYCACHEPREFIX=/private/tmp/pycache python3 -m compileall -q sim/current
env PYTHONPYCACHEPREFIX=/private/tmp/pycache /Users/kanghyunmin/miniconda3/envs/ros2_h311/bin/python3.11 sim/current/run_uuv_mujoco.py --help
env PYTHONPYCACHEPREFIX=/private/tmp/pycache python3 sim/current/tools/check_runtime_readiness_policy.py
env PYTHONPYCACHEPREFIX=/private/tmp/pycache python3 sim/current/tools/verify_ardusub_thruster_contract.py --quiet
env PYTHONPYCACHEPREFIX=/private/tmp/pycache python3 sim/current/tools/audit_code_contract_sources.py --out-dir /private/tmp/uuv_code_contract_after_sitl_servo_wiring_split
env PYTHONPYCACHEPREFIX=/private/tmp/pycache python3 sim/current/tools/check_dev_os_compat.py --headless --target-os ubuntu
```

Results:

- Compile: pass.
- Runner `--help`: pass.
- SITL/replay handler binding smoke: pass.
- Runtime readiness policy: pass.
- ArduSub thruster contract: pass.
- Code contract audit: `fail=0`, `pass=10`, `warn=5`.
- Dev OS compatibility: `fail=0`, `pass=16`, `warn=2`.
