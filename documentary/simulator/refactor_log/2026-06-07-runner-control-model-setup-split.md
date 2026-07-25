# Runner Control And Model IO Setup Split

Date: 2026-06-07

## Problem

`run_uuv_mujoco.py` still mixed setup responsibilities that are not part of
the physics step loop:

- direct command timeout state and SITL direct-command policy;
- ROS bridge creation and initial-depth release service wiring;
- real-start status publisher wiring;
- actuator, sensor, camera, site, and QGC-video lookup;
- stale local unpacking of hydrostatic and hydrodynamic runtime attributes.

Those blocks made the main runner harder to audit when checking controller
contract, plant input, and sensor timing behavior.

## Change

- Added `sim/runtime/control_bridge_setup.py`.
  - Owns direct `RuntimeCommandState`.
  - Owns `ROS2_UUV_SITL_ALLOW_DIRECT_CMD` policy and log message.
  - Creates `RosBridgeRuntime`.
  - Installs initial-depth release service.
  - Creates `RealStartRuntimeStatus`.
- Added `sim/runtime/model_io_setup.py`.
  - Owns actuator ID and control range lookup.
  - Owns physical thruster name/order lists.
  - Owns sensor, camera, and site ID lookup.
  - Owns optional QGC video runtime creation.
- Removed unused local unpacking from `run_uuv_mujoco.py`.

## Size Check

```text
run_uuv_mujoco.py: LOC=612 branches=15 funcs=12 largest=main:541
sim/runtime/control_bridge_setup.py: LOC=131 branches=3 funcs=3 largest=create_runtime_control_bridge_setup:95
sim/runtime/model_io_setup.py: LOC=107 branches=0 funcs=1 largest=create_runtime_model_io_setup:76
```

Previous measured state before this split:

```text
run_uuv_mujoco.py: LOC=763 branches=18 largest=main:682
```

## Validation

```text
python3 -m py_compile \
  sim/current/run_uuv_mujoco.py \
  sim/current/sim/runtime/control_bridge_setup.py \
  sim/current/sim/runtime/model_io_setup.py
```

```text
source ./.uuv_mujoco_env.sh
"$MJ311_PYTHON" sim/current/run_uuv_mujoco.py --help
```

```text
python3 -m compileall -q sim/current uuv_control_gui.py
python3 sim/current/tools/check_runtime_readiness_policy.py
python3 sim/current/tools/verify_ardusub_thruster_contract.py --quiet
python3 sim/current/tools/audit_code_contract_sources.py \
  --out-dir /private/tmp/uuv_code_contract_after_runner_control_model_split
python3 sim/current/tools/check_dev_os_compat.py --headless --target-os ubuntu
```

Results:

- runner help prints successfully with `MJ311_PYTHON`;
- `runtime_readiness_policy=PASS`;
- `[thruster-contract] OK`;
- `audit_code_contract_sources`: `fail=0`, `pass=10`, `warn=5`;
- `check_dev_os_compat`: `fail=0`, `pass=16`, `warn=2`.
