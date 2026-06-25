# RC and GUI Environment Contract Split

Date: 2026-06-07

Scope: active runtime under `uuv_mujoco/current` with compatibility backing path
`uuv_mujoco/v2.2`.

## Changed

- Split `sim/contracts/rc.py` into focused RC contract modules:
  - `rc_constants.py`
  - `rc_math.py`
  - `rc_axis.py`
  - `rc_frames.py`
  - `rc_joystick.py`
  - `rc_althold.py`
- Kept `sim/contracts/rc.py` as the public compatibility facade.
- Split `gui/sim_stack_env_contract.py` into focused GUI-start env modules:
  - `sim_stack_env_modes.py`
  - `sim_stack_env_defaults.py`
  - `sim_stack_env_forced.py`
- Kept `build_gui_sim_stack_env()` and the public `gui.sim_stack_env` import
  surface stable.
- Normalized invalid `UUV_RUNTIME_PROFILE` values to `balanced` in the final
  GUI-start environment. Valid `balanced`, `low`, and `high` profiles are
  preserved with lowercase canonical names.

## Contract Preserved

- RC override frame marker semantics are unchanged:
  - primary 1-8 channels sanitize to PWM or `0`
  - extension channels preserve MAVLink ignore markers
  - neutral keepalive frame still uses 18 channels
- Controller-parity semantics are unchanged:
  - closed loop uses SITL JSON servo as plant input
  - plant replay uses explicit RCOU override
  - no ALT_HOLD shim, PWM remap, or output remap was introduced
- GUI-start SITL still defaults to the current real-robot contract:
  - closed loop
  - `poshold_extnav`
  - dedicated command MAVLink endpoint
  - direct command/setpoint shims disabled

## Validation

- `python3 -m compileall -q uuv_mujoco/current uuv_control_gui.py`
- `PYTHONPATH=uuv_mujoco/current/tools python3 uuv_mujoco/current/tools/audit_code_contract_sources.py --out-dir /private/tmp/uuv_source_audit_rc_env_split`
- `python3 uuv_mujoco/current/tools/check_runtime_readiness_policy.py`
- `python3 uuv_mujoco/current/tools/verify_ardusub_thruster_contract.py --quiet`
- `python3 uuv_mujoco/current/tools/check_dev_os_compat.py --headless --target-os ubuntu`
- `/Users/kanghyunmin/miniconda3/envs/ros2_h311/bin/python uuv_mujoco/current/tools/physics_contract_audit.py --output-dir /private/tmp/uuv_physics_audit_rc_env_split --simulate-s 0`
- `python3 uuv_mujoco/current/tools/check_runtime_freshness.py --workspace /Users/kanghyunmin/Desktop/uuv_sim --runtime-dir /Users/kanghyunmin/Desktop/uuv_sim/uuv_mujoco/current --fetch --refresh-version`

## Results

- Source contract audit: `fail=0`, `pass=11`, `warn=5`.
- Runtime readiness policy: `PASS`.
- Thruster contract: `OK`.
- Dev OS compatibility: `fail=0`, `pass=16`, `warn=2`.
- Runtime freshness: `PASS`; `current -> v2.2`; local `HEAD` equals
  `origin/uuv_sim`.
- Static physics audit: neutral force balance remains `net_down=+0.000N` with
  `required_scale=1.000000`.

## Remaining Hotspots

The next contract-heavy files by inventory are:

- `bridge/sitl_transport.py`
- `sim/physics/hydrostatic_setup.py`
- `bridge/ros2_endpoints.py`
- `gui/gui_rc_helpers.py`
