# Refactor Master Plan

Date: 2026-06-04

The active goal is to make the active MuJoCo runtime exposed at
`uuv_mujoco/current` enforce the same runtime contract
used by the validated controller-parity path:

```text
real rosbag RC + sensor/state
  -> ArduSub 4.1.2 SITL
  -> MAVLink SERVO_OUTPUT_RAW telemetry
  -> real /mavros/rc/out comparison

closed_loop MuJoCo plant input
  <- raw ArduSub JSON servo packet
```

Refactoring is only acceptable when it makes this contract easier to verify or
harder to violate.  A code split that does not preserve arm, mode, RC override,
sensor replay, RCOU telemetry, and plant-input gates is not progress.

This is not a physics-tuning plan and not a controller-output correction plan.

## Hard boundaries

1. Do not edit ArduPilot source.
2. Do not change the ArduPilot submodule pointer.
3. Keep simulation changes inside the active MuJoCo runtime.  The current
   compatibility backing directory is `uuv_mujoco/v2.2`, but new launch
   commands and docs should use `uuv_mujoco/current`.
4. Do not hide mismatch with ALT_HOLD shims, PWM correction, or output remaps.
5. Keep closed-loop plant input as raw ArduSub JSON servo.
6. Compare controller parity only at the telemetry layer:
   real `/mavros/rc/out` vs SITL MAVLink `SERVO_OUTPUT_RAW`.

## Target structure

```text
uuv_mujoco/current/
  sim/
    contracts/      stable runtime contracts and shared constants
    transport/      JSON servo/sensor and MAVLink transport code
    runtime/        launch, reset, readiness, arm/mode orchestration
    ros_surface/    ROS2 topic and MAVROS compatibility surface
    physics/        MuJoCo plant, hydrostatics, thrusters, hydrodynamics
    validation/     metrics, overlays, golden-master gates
  han/
    data/           training and validation datasets
    features/       basis construction and normalization
    models/         HAN model definitions and frozen artifacts
    training/       optimization loops and experiment configs
    calibration/    coefficient projection and matrix gates
    cfd/            CFD import/export adapters and geometry metadata
    inference/      runtime-safe frozen profile export
  experiments/
    baselines/      named baselines and golden references
    runs/           generated run outputs
    legacy/         old debug assets retained for provenance
    paper_evidence/ artifacts intended for thesis/paper figures
  docs/
    architecture/   structure and refactor decisions
    contracts/      input/output/sensor/plant contract records
    refactor_log/   phase-by-phase change logs
```

## Required Gates

Every full-runtime controller-parity run must fail before overlay generation if:

- `full_mujoco_rcout.csv` is missing or header-only;
- the plant-input evidence has no non-neutral PWM rows when motion is expected;
- runtime logs contain disarmed JSON-servo signatures;
- GUI/runtime readiness does not prove command-path readiness.

These gates are not optional experiment helpers.  They are the definition of a
valid closed-loop contract run.

## Work Area: freeze and observe

Goal: make the current mess inspectable without changing behavior.

- Create architecture docs and ownership folders.
- Re-export existing contract primitives through `sim/contracts`.
- Add inventory tooling.
- Do not move live runtime code yet.
- Gate: syntax check new files.

## Work Area: contract extraction

Goal: make all live code consume one source of truth.

- RC mapping and spans.
- Bar30 pressure law and ground pressure policy.
- Sensor publish rates and zero-order-hold policy.
- MAVLink endpoint and SERVO_OUTPUT_RAW telemetry policy.
- GUI readiness state machine.
- Gate: replay/controller metrics must not regress.

## Work Area: transport split

Goal: split `SitlTransport` without changing external behavior.

- JSON sensor sender.
- JSON servo receiver.
- MAVLink command sender.
- MAVLink telemetry receiver.
- Replay preview reader.
- Diagnostics recorder.
- Gate: arm, mode change, RC override, and RCOU CSV generation all pass.

## Work Area: ROS surface split

Goal: split `Ros2Bridge` by topic family.

- IMU and pressure publishers.
- DVL and pose publishers.
- RC override subscriber.
- MAVROS-compatible state and service surface.
- TF and visualization outputs.
- Gate: topic rates match real contract and no required topic disappears.

## Work Area: plant split

Goal: split `run_uuv_mujoco.py` into deterministic plant components.

- Scene/model loader.
- Initial state resolver.
- Hydrostatic model.
- Actuator/thruster model.
- Hydrodynamics/current model.
- Sensor synthesis.
- Logging and GUI bridge.
- Gate: plant replay metrics do not regress from the frozen baseline.

## Work Area: HAN/CFD integration

Goal: keep HAN as an offline estimator that exports frozen profiles.

- Build feature matrices from geometry, flow, and replay residuals.
- Use matrix gates before full plant replay.
- Export bounded coefficient profiles.
- Runtime imports frozen profiles only.
- Gate: coefficient changes must improve validation overlays and preserve
  physically plausible signs/units.

## Change rule

Every behavior-changing edit must have:

1. The contract it touches.
2. The before/after run identifier.
3. The exact overlay or metric table used to accept or reject it.
4. A rollback path that does not touch ArduPilot.
