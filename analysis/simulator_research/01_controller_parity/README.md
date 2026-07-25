# 01 Controller Parity

Controller parity is the gate before MuJoCo tuning.

```text
real RC + real sensor/state
  -> ArduSub 4.1.2 SITL
  -> MAVLink SERVO_OUTPUT_RAW
  -> compare with real /mavros/rc/out
```

## Canonical Evidence

- `/Users/kanghyunmin/Desktop/uuv_sim/uuv_mujoco/v2.2/debug/controller_parity_412/FINDINGS.md`
- `/Users/kanghyunmin/Desktop/uuv_sim/uuv_mujoco/v2.2/debug/controller_parity_412/README.md`
- `/Users/kanghyunmin/Desktop/uuv_sim/uuv_mujoco/v2.2/debug/controller_parity_412/real_90s_feedback_source_servo_20260601`
- `/Users/kanghyunmin/Desktop/uuv_sim/uuv_mujoco/v2.2/debug/controller_parity_412/real_90s_mavlink_source_audit_20260601`

## Rule

Do not compare real `/mavros/rc/out` directly to SITL JSON servo backend.  JSON
servo is plant input, not controller telemetry.
