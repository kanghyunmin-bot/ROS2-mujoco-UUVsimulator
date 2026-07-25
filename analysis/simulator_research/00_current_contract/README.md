# 00 Current Contract

This folder tracks the accepted runtime contract.  It is intentionally small:
large generated outputs stay in `UUV-HAN/outputs` or
`debug/controller_parity_412/outputs`.

## Accepted Contract

- ArduPilot source and submodule pointer stay untouched.
- `/Users/kanghyunmin/Desktop/uuv_sim/ardupilot` is a clean `ArduSub-4.1.2`
  checkout when used as the source-contract reference.
- ArduSub parameters are loaded from the real-robot contract, not patched by a
  controller shim.
- Closed-loop plant input is raw ArduSub JSON servo output.
- Plant replay input is real RCOU/SERVO output with zero-order hold.
- Controller parity compares real `/mavros/rc/out` against SITL MAVLink
  `SERVO_OUTPUT_RAW`, not against high-rate JSON servo backend.
- Fossen residual hydro terms exist but their main-profile coefficients remain
  zero until accepted by validation.

## Code-Level Contract Audit

Latest source audit:

- `/Users/kanghyunmin/Desktop/uuv_sim/uuv_mujoco/v2.2/research_workspace/00_current_contract/contract_source_audit.md`
- `/Users/kanghyunmin/Desktop/uuv_sim/uuv_mujoco/v2.2/research_workspace/00_current_contract/contract_source_audit.json`

Current audit result:

```text
PASS=10, WARN=5, FAIL=0
```

Confirmed by local ArduSub 4.1.2 code:

- JSON servo backend sends raw `pwm[16]`; closed-loop plant must consume this
  as final ArduSub PWM.
- `SERVO_OUTPUT_RAW` is `hal.rcout` telemetry; this is the valid comparison
  layer for real `/mavros/rc/out`.
- JSON sensor parser has no pressure or altitude key in this firmware.  Bar30
  controller input is therefore `position.z -> SITL altitude -> AP_Baro_SITL
  underwater pressure`, not a direct pressure JSON field.
- RC override handler in this local 4.1.2 source consumes channels 1..16.  The
  old 1..18 checklist is not true for this firmware, although the active robot
  controls are within C1..C8.
- RC override must be streamed; local default `RC_OVERRIDE_TIME` is 3 seconds.
- ArduSub joystick mapping confirms RC3 heave, RC4 yaw, RC5 forward, RC6
  lateral.

Confirmed by v2.2 code:

- Bar30 pressure/depth is converted with `frontend_match` and injected through
  JSON `position.z`.
- `/mavros/imu/static_pressure` defaults to external Bar30 absolute pressure.
- Final PWM is mapped into MuJoCo actuator-positive force once.  Do not apply
  `MOT_x_DIRECTION` again in plant replay.

Current warnings / exclusions:

- The ArduPilot working checkout is clean `ArduSub-4.1.2`
  (`2dd0bb7d...`), but the top-level repository currently records gitlink
  `6271e15b...`.  Do not commit the gitlink change unless the project
  intentionally updates the submodule pointer.
- v2.2 still sends JSON `altitude`, but ArduSub 4.1.2 ignores it.  Treat it as
  compatibility/debug only.
- `/mavros/imu/atm_pressure` is excluded from fitting and parity until its real
  semantics are proven; the April 1 bag value is not Pa-scale atmospheric
  pressure.
- `local_position` is estimator/output-surface evidence, not a primary sensor
  contract target.
- DVL-z remains warning-grade; use DVL x/y and gyro/IMU/Bar30 first for
  HAN/CFD tuning.

## Docker ArduSub Build

Native macOS build of ArduSub 4.1.2 is not the runtime path and can fail on the
current Apple SDK because the old `fenv` polyfill expects x86-style fields.
The accepted runtime is Docker SITL:

```bash
docker compose -f /Users/kanghyunmin/Desktop/uuv_sim/docker/ardusub/docker-compose.yml build ardusub-sitl
docker compose -f /Users/kanghyunmin/Desktop/uuv_sim/docker/ardusub/docker-compose.yml run --rm --entrypoint bash ardusub-sitl -lc 'cd /workspace/ardupilot && ./waf configure --board sitl && ./waf build --target bin/ardusub'
```

The Docker build path was verified with clean `ArduSub-4.1.2` source and
successfully linked `build/sitl/bin/ardusub`.

## Canonical Files

- `/Users/kanghyunmin/Desktop/uuv_sim/uuv_mujoco/v2.2/config/sim_profiles.json`
- `/Users/kanghyunmin/Desktop/uuv_sim/uuv_mujoco/v2.2/config/thruster_params.json`
- `/Users/kanghyunmin/Desktop/uuv_sim/uuv_mujoco/v2.2/config/ardusub_realrobot_contract.param`
- `/Users/kanghyunmin/Desktop/uuv_sim/uuv_mujoco/v2.2/run_uuv_mujoco.py`

## Sanity Checks

```bash
python3 /Users/kanghyunmin/Desktop/uuv_sim/uuv_mujoco/v2.2/tools/audit_code_contract_sources.py
python3 -m json.tool /Users/kanghyunmin/Desktop/uuv_sim/uuv_mujoco/v2.2/config/sim_profiles.json >/dev/null
python3 -m json.tool /Users/kanghyunmin/Desktop/uuv_sim/uuv_mujoco/v2.2/config/thruster_params.json >/dev/null
python3 -m py_compile /Users/kanghyunmin/Desktop/uuv_sim/uuv_mujoco/v2.2/run_uuv_mujoco.py
```

`run_urdf_full.py` remains available only as a legacy compatibility wrapper.

The current main profile must not contain nonzero `fossen_residual_hydro`
candidate coefficients unless a validation report explicitly accepts them.
