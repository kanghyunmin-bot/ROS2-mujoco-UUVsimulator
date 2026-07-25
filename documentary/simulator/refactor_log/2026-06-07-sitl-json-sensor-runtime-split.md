# SITL JSON Sensor Runtime Split

Date: 2026-06-07

Scope: active runtime transport code under `sim/current/bridge` and
the source-contract audit path registry.

## What changed

- Split ArduSub JSON payload construction into `bridge/sitl_json_payload.py`.
- Split JSON packet finite-value validation, debug logging, and UDP send into
  `bridge/sitl_json_sender.py`.
- Split servo-frame immediate sensor-replay replies into
  `bridge/sitl_json_replay_reply.py`.
- Split live/replayed `send_state()` policy into
  `bridge/sitl_json_sensor_send.py`.
- Reduced `bridge/sitl_json_sensor_runtime.py` to compatibility exports used
  by `bridge/sitl_transport.py`.
- Updated the source-contract audit path registry so Bar30 JSON `position.z`
  evidence is read from `bridge/sitl_json_payload.py`.

## Contract boundaries preserved

- JSON plant sensor path still uses ArduSub 4.1.2 JSON keys.
- Bar30/control input still goes through JSON `position.z`, not a direct
  pressure/altitude JSON field.
- `"altitude"` remains compatibility/debug data for this firmware and is still
  reported as a WARN by source audit.
- Immediate replay mode still suppresses duplicate publish-loop packets and
  sends synchronized replies on JSON servo frames.

## Verification

```text
python3 -m py_compile \
  sim/current/bridge/sitl_json_sensor_runtime.py \
  sim/current/bridge/sitl_json_payload.py \
  sim/current/bridge/sitl_json_sender.py \
  sim/current/bridge/sitl_json_replay_reply.py \
  sim/current/bridge/sitl_json_sensor_send.py \
  sim/current/bridge/sitl_transport.py \
  sim/current/tools/audit_code_contract_paths.py \
  sim/current/tools/audit_code_contract_runtime_checks.py
PYTHONPATH=sim/current python3 - <<'PY'
from bridge import sitl_json_sensor_runtime as m
required = ['_send_sitl_json_payload', '_payload_from_state', '_send_immediate_sensor_replay_reply', 'send_state']
status = {name: callable(getattr(m, name, None)) for name in required}
print(status)
if not all(status.values()):
    raise SystemExit(1)
PY
PYTHONPATH=sim/current/tools python3 \
  sim/current/tools/audit_code_contract_sources.py \
  --out-dir /private/tmp/uuv_code_contract_after_sitl_json_split
python3 -m compileall -q sim/current uuv_control_gui.py
python3 sim/current/tools/check_runtime_readiness_policy.py
python3 sim/current/tools/verify_ardusub_thruster_contract.py --quiet
python3 sim/current/tools/check_dev_os_compat.py --headless --target-os ubuntu
git diff --check
python3 sim/current/tools/refactor_inventory.py --limit 20
```

Results:

- Compile/import: pass.
- Source contract audit: `fail=0`, `pass=11`, `warn=5`.
- Runtime readiness policy: pass.
- ArduSub thruster contract: pass.
- Dev OS compatibility: `fail=0`, `pass=16`, `warn=2` (`docker_daemon`,
  `ros2_env`).
- `git diff --check`: pass.
- `bridge/sitl_json_sensor_runtime.py` removed from the top hotspot list.
