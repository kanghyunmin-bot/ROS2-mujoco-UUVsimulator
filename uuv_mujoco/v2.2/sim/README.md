# Simulation Refactor Package

This package is the future home for behavior-preserving simulation runtime code.
It is loaded through `uuv_mujoco/current`; the physical `v2.2` directory name is
only the compatibility backing path while the runtime is being cleaned up.

Ownership:

- `contracts`: shared constants, pressure laws, RC mapping, timing rules, frame
  contracts.
- `transport`: JSON and MAVLink wire protocols.
- `runtime`: process lifecycle, readiness, reset, arm/mode orchestration.
- `ros_surface`: ROS2 and MAVROS-compatible topic/service surfaces.
- `physics`: MuJoCo plant, thrusters, hydrostatics, hydrodynamics, current.
- `validation`: metrics, overlays, and golden-master checks.

Rule: HAN/CFD research code must not mutate live runtime behavior directly.
Runtime may consume only frozen, validated profiles.

Current transport extraction status:

- JSON servo binary packet decoding lives in `sim/transport/json_servo.py`.
- JSON-SITL UDP socket bind/recv/send bookkeeping lives in
  `sim/transport/json_servo_receiver.py`.
- MAVLink telemetry type lists and `SERVO_OUTPUT_RAW` PWM extraction live in
  `sim/transport/mavlink_telemetry.py`.
- Passive MAVLink telemetry status accumulation lives in
  `sim/transport/mavlink_telemetry_observer.py`.
- Low-level MAVLink command-link connection, GCS heartbeat, RC override send,
  and arm/disarm command send live in `sim/transport/mavlink_command_link.py`.
- MAVLink `SET_MESSAGE_INTERVAL` request scheduling lives in
  `sim/transport/mavlink_message_interval.py`.
- Plant command PWM activity classification lives in
  `sim/transport/plant_command.py`.
