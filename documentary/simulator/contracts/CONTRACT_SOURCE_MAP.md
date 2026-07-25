# Contract Source Map

Date: 2026-06-04

This map tracks where runtime contracts currently live and where they should
move during the refactor.

## Bar30 and AP_Baro pressure contract

Canonical implementation:

```text
sim/contracts/baro.py
```

Compatibility export:

```text
bridge/sitl_contract.py
```

Current consumers:

| Consumer | Current state |
| --- | --- |
| `bridge/ros2_bridge.py` | migrated to `sim.contracts` |
| `debug/controller_parity_412/sensor_replay_sitl_json.py` | migrated to `sim.contracts` for shared pressure and RC constants |
| `debug/controller_parity_412/audit_sensor_rate_contract.py` | migrated to `sim.contracts` for shared rates |
| `tools/real_start_state.py` | migrated to `sim.contracts` for shared formulas |

Target state:

- `bridge/sitl_contract.py` remains as a compatibility shim for older callers.
- Only `sim/contracts/baro.py` should own pressure constants and formulas.

## Sensor rate contract

Canonical implementation:

```text
REAL_ROBOT_SENSOR_RATES_HZ in sim/contracts/rates.py
```

Target state:

- Keep `bridge/sitl_contract.py` as a re-export during migration.
- ROS bridge and replay audit tools must use the same dictionary.

## RC contract

```text
sim/contracts/rc.py
```

Current canonical content:

- 18-channel MAVLink2 `RC_CHANNELS_OVERRIDE` packet shape.
- Primary motion channels 1 through 6 preservation.
- RC3 heave, RC4 yaw, RC5 forward, RC6 lateral.
- Center/min/max/valid-range policy.
- Hold policy for missing or invalid channels.

Source-audit warning:

- The replay/CSV surface can carry 18 channels.
- The local ArduSub 4.1.2 RC override handler applies fields only through
  channel 16.  Current vehicle controls are within C1..C8, so this is not a
  blocker, but the old "ArduSub consumes 1..18" assumption is too broad.

Current consumers:

| Consumer | Current state |
| --- | --- |
| `debug/controller_parity_412/sensor_replay_sitl_json.py` | migrated to shared `sanitize_primary_rc` |
| `gui/helpers.py` | migrated to shared sanitizer |
| `debug/controller_replay/replay_rc_override_ros2.py` | migrated to shared sanitizer |
| remaining GUI/transport/tools | not fully migrated yet |

## Telemetry observation contract

Current state:

- Controller overlays and plant input logs are still easy to confuse because
  both are PWM-like outputs.

Current canonical content:

```text
sim/contracts/observability.py
```

Required content:

- `SERVO_OUTPUT_RAW` telemetry is the controller-parity observation point.
- JSON servo backend is the plant-input observation point.
- Zero-order hold is required for low-rate telemetry overlays.

## Runtime readiness contract

Current state:

- GUI readiness, process start, MAVLink command path, and plant-side servo rows
  are not represented by one state machine.

Target state:

```text
sim/runtime/readiness.py
```

Required ready gates:

- MuJoCo runtime alive.
- JSON sensor transport alive.
- JSON servo receiver alive.
- MAVLink command endpoint alive.
- ArduSub mode/arm state known.
- Plant-side servo rows non-empty when armed movement is expected.
