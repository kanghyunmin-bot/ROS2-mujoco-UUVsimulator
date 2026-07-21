# Current Runtime Contract

Date: 2026-07-17

This file records the current contract that must be preserved while refactoring.
It is not a claim that the plant dynamics are already correct.

## Observation contracts

Controller parity:

```text
real /mavros/rc/out
vs
SITL MAVLink SERVO_OUTPUT_RAW telemetry
```

Plant input:

```text
Docker/MAVProxy closed-loop:
  SITL JSON servo backend -> MuJoCo thruster input

Native/direct closed-loop:
  SITL MAVLink SERVO_OUTPUT_RAW -> MuJoCo thruster input

Plant replay:
  recorded actuator PWM/RCOU -> MuJoCo thruster input
```

Never compare real `/mavros/rc/out` directly to a backend-specific plant input
surface for controller parity.

## RC contract

Expected joystick-to-RC axes:

| Axis | RC channel | Meaning |
| --- | ---: | --- |
| heave | RC3 | vertical command |
| yaw | RC4 | yaw command |
| forward | RC5 | surge command |
| lateral | RC6 | sway command |

Known guardrails:

- The replay/CSV surface can preserve 18 MAVLink2 override channels, but local
  ArduSub 4.1.2 handler code applies override fields only through channel 16.
- Keep `RC3_TRIM=RC3_MIN=1100`: ArduSub treats throttle as a range channel, so
  RC3=1500 then maps to normalized 0.5, the bidirectional MANUAL/STABILIZE
  neutral, and to zero climb in ALT_HOLD. Force `RCMAP_FORWARD=5` plus
  `RCMAP_LATERAL=6` so RC override/QGC axes reach ArduSub's Sub mapping.
- Keep the bridge MAVLink source sysid equal to ArduSub's GCS authority
  (`MAV_GCS_SYSID` on 4.8, `SYSID_MYGCS` on 4.1); native GUI defaults are
  source sysid `255`, compid `190`.
- GUI-started pilot input defaults to MANUAL_CONTROL. The GUI can still publish
  `/mavros/rc/override`; the bridge converts that surface to MANUAL_CONTROL.
- Do not change `PILOT_SPEED_DN=0` as a controller-parity fix.
- Treat RC timing and hold policy as a contract, not a tuning parameter.
- Stream RC override faster than the local `RC_OVERRIDE_TIME` timeout policy;
  one-shot override is not a valid closed-loop input contract.

## Sensor contract

ALT_HOLD parity depends primarily on Bar30/static pressure and Pixhawk IMU/raw
attitude.  DVL or vision must not be added as a shortcut unless the real
controller contract proves that those inputs were used.

Source audit result:

- Local ArduSub 4.1.2 JSON sensor parser has no direct pressure or altitude
  key for this firmware.
- Bar30 controller input is represented indirectly through JSON `position.z`,
  which AP_Baro_SITL converts into water-barometer pressure.
- `/mavros/imu/static_pressure` is the ROS/MAVROS output surface.  It is not a
  direct JSON input field to ArduSub.

The active shared contract source is:

```text
bridge/sitl_contract.py
```

It defines:

- AP_Baro pressure constants.
- Bar30 pressure-to-SITL-depth conversion.
- real-robot surface topic rates.

The refactor path exposes the same source through:

```text
sim/contracts/baro.py
sim/contracts/rates.py
sim/contracts/rc.py
sim/contracts/observability.py
sim/contracts/__init__.py
```

## Bar30 pressure law

Bar30 is a pressure sensor.  Depth is derived from pressure and a pressure datum.
The important contract is the pressure datum and conversion path used by
ArduSub/AP_Baro, not a cosmetic `/depth/pose` topic.

Runtime/refactor code must preserve the existing `BaroPressureLaw` semantics:

```text
real pressure sample
to
AP_Baro frontend altitude/depth
to
SITL JSON depth required to emit equivalent pressure
```

## Timing contract

GUI `balanced` and `low` profiles, plus the dedicated Pinger Start path, keep
both the SITL sensor feed and the authoritative MuJoCo thruster update loop at
or above 100 Hz.  The active keys are
`SITL_SENSOR_HZ_DEFAULT`/`UUV_ROS2_SENSOR_HZ` and
`SITL_THRUSTER_LOOP_HZ_DEFAULT`/`UUV_THRUSTER_LOOP_HZ`.  The low profile may
reduce viewer, camera, logging, and other auxiliary rates, but must not reduce
this STABILIZE/ALT_HOLD feedback cadence.  A 30 Hz sensor / 40 Hz thruster A/B
run produced a saturated roll/pitch limit cycle with the unchanged real ATC
tuning; 100/100 Hz held the initial depth and attitude with the same plant and
controller parameters.  `SITL_MAVLINK_SERVO_HZ_DEFAULT` is a telemetry rate
and is not this authoritative plant-update loop.

The real black `/mavros/rc/out` curve is low-rate MAVLink
`SERVO_OUTPUT_RAW` telemetry.  It normally appears as a staircase because it is
held between telemetry updates.

Overlay code must use zero-order hold for low-rate telemetry surfaces.  Linear
interpolation is invalid for controller parity.

## Runtime failure signatures

These are contract failures, not physics failures:

- `full_mujoco_rcout.csv` has only a header row.
- Logs repeat `SITL(json) servo output ignored while disarmed`.
- Native/direct logs show neutral `SERVO_OUTPUT_RAW` plant input while the test
  expects actuator motion.
- Docker/MAVProxy logs show neutral JSON servo output while the test expects
  actuator motion.
- GUI says ready before arm/mode/RC override reaches the runtime command path.

GUI `READY` must mean more than process startup.  For the internal sim bridge
backend it must require fresh vehicle state, fresh Bar30/depth and IMU streams,
ready arm/mode services, and a fresh SITL MAVLink telemetry heartbeat from
`/uuv_mujoco/sitl/mavlink_telemetry_status`.  If that heartbeat is missing or
stale, the GUI must show `WAIT: SITL MAVLink` and arm/mode requests must remain
delayed instead of being sent into a dead command path.

The dependency-free policy smoke is:

```bash
PYTHONPATH=. python3 tools/check_runtime_readiness_policy.py
```

The full MuJoCo controller-parity runner must execute the plant-input gate
immediately after probe capture and before any overlay generation.  If this
gate fails, the run is invalid regardless of any later plot or metric output.
The runner must also persist the gate verdict in
`full_mujoco_controller_parity_manifest.json` as `plant_input_gate_result` so a
failed run cannot be mistaken for a valid overlay run later.

## ArduPilot source identity

The source audit on 2026-06-04 found:

- inner ArduPilot checkout: `ArduSub-4.1.2`, clean,
- top-level gitlink: different commit from the checked-out ArduPilot worktree.

This is a repository-state warning.  Do not commit a submodule pointer change
unless that is explicitly intended.

## Golden-master policy

Before moving live code, capture or reference a baseline run with:

- exact command,
- generated CSVs,
- overlay image paths,
- C1-C8 RCOU metrics,
- axis-decomposition metrics,
- plant sensor overlay metrics when applicable.

Any extraction must pass the same gates before it is accepted.

## Development OS compatibility

The MuJoCo/SITL runtime must use one explicit development OS contract across
macOS and future Ubuntu work.  The current contract is recorded in:

```text
docs/contracts/DEV_OS_COMPATIBILITY.md
```

Run the preflight before treating GUI, headless replay, or Ubuntu migration
failures as physics problems:

```bash
python3 tools/check_dev_os_compat.py --headless
python3 tools/check_dev_os_compat.py --require-viewer
```
