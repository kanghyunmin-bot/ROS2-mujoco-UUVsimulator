# SITL MAVLink Runtime Second Split

Date: 2026-06-07

Scope: active runtime MAVLink transport code under `uuv_mujoco/current/bridge`.

## What changed

- Split MAVLink connection setup, command-link reconnect, and GCS heartbeat into
  `bridge/sitl_mavlink_connection.py`.
- Split MAVLink telemetry observer wrappers into
  `bridge/sitl_mavlink_telemetry.py`.
- Split servo-link and command-link polling loops into
  `bridge/sitl_mavlink_polling.py`.
- Reduced `bridge/sitl_mavlink_runtime.py` to compatibility exports that still
  bind into `SitlTransport`.

## Contract boundaries preserved

- `SERVO_OUTPUT_RAW` telemetry remains passive controller-parity telemetry when
  JSON servo packets are the plant input.
- JSON servo fallback behavior is unchanged.
- Stream request helpers and PWM handling remain the same existing functions.
- No RC remap, PWM correction, ArduPilot change, or plant-input semantic change.

## Verification

```text
python3 -m py_compile \
  uuv_mujoco/current/bridge/sitl_mavlink_runtime.py \
  uuv_mujoco/current/bridge/sitl_mavlink_connection.py \
  uuv_mujoco/current/bridge/sitl_mavlink_telemetry.py \
  uuv_mujoco/current/bridge/sitl_mavlink_polling.py \
  uuv_mujoco/current/bridge/sitl_transport.py
PYTHONPATH=uuv_mujoco/current python3 - <<'PY'
from bridge import sitl_mavlink_runtime as m
required = [
    '_connect_sitl_mavlink', '_command_mavlink_disabled', '_connect_sitl_command_mavlink',
    '_ensure_command_mavlink_connected', '_send_gcs_heartbeat',
    '_request_sitl_mavlink_servo_stream', '_request_command_servo_telemetry_stream',
    '_request_sitl_mavlink_ap_telemetry_stream', '_request_command_ap_telemetry_stream',
    '_mavlink_source_matches_target', '_store_ap_mavlink_telemetry', '_handle_pwm_values',
    '_poll_servo_mavlink', '_poll_command_mavlink',
]
status = {name: callable(getattr(m, name, None)) for name in required}
print(status)
if not all(status.values()):
    raise SystemExit(1)
PY
python3 -m compileall -q uuv_mujoco/current uuv_control_gui.py
PYTHONPATH=uuv_mujoco/current/tools python3 \
  uuv_mujoco/current/tools/audit_code_contract_sources.py \
  --out-dir /private/tmp/uuv_code_contract_after_mavlink_runtime_split
python3 uuv_mujoco/current/tools/check_runtime_readiness_policy.py
python3 uuv_mujoco/current/tools/verify_ardusub_thruster_contract.py --quiet
python3 uuv_mujoco/current/tools/check_dev_os_compat.py --headless --target-os ubuntu
git diff --check
python3 uuv_mujoco/current/tools/refactor_inventory.py --limit 20
```

Results:

- Compile/import: pass.
- Source contract audit: `fail=0`, `pass=11`, `warn=5`.
- Runtime readiness policy: pass.
- ArduSub thruster contract: pass.
- Dev OS compatibility: `fail=0`, `pass=16`, `warn=2` (`docker_daemon`,
  `ros2_env`).
- `git diff --check`: pass.
- `bridge/sitl_mavlink_runtime.py` removed from the top hotspot list.
