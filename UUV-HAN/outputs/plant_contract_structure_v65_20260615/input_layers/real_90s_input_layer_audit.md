# Real 90s Input Layer Audit

- csv: `/Users/kanghyunmin/Desktop/uuv_sim/uuv_mujoco/v2.2/debug/controller_parity_412/real_90s_feedback_source_servo_20260601/real_controller_feedback_20hz.csv`
- window: `0.000s` to `90.000s`
- decision: `blocked_no_measured_high_rate_actuator_history`
- measured high-rate actuator history: `False`

## Layers

| layer | semantic | plant input | controller reconstruction | HAN export | distinct Hz | updates | notes |
|---|---:|---:|---:|---:|---:|---:|---|
| `real_mavros_rc_out` | `real_controller_output_telemetry` | `True` | `False` | `False` | `2.000` | `181` | output telemetry layer, not raw actuator backend history; effective PWM step cadence is below 10Hz; can be used as direct plant replay input with semantic caveats |
| `real_mavlink_source_servo` | `real_controller_output_telemetry` | `True` | `False` | `False` | `2.000` | `180` | output telemetry layer, not raw actuator backend history; effective PWM step cadence is below 10Hz; can be used as direct plant replay input with semantic caveats |
| `real_rc_override` | `real_controller_input_rc_override` | `False` | `True` | `False` | `20.000` | `1098` | controller input layer, upstream of ArduSub motor mixing; can be used to reconstruct controller-side state/RCOU; not allowed for HAN/CFD target export as measured actuator history |
| `real_mavlink_sink_rc_override` | `real_controller_input_rc_override` | `False` | `True` | `False` | `20.000` | `1097` | controller input layer, upstream of ArduSub motor mixing; can be used to reconstruct controller-side state/RCOU; not allowed for HAN/CFD target export as measured actuator history |
| `real_rc_in` | `real_receiver_input` | `False` | `False` | `False` | `nan` | `1` | controller input layer, upstream of ArduSub motor mixing; not allowed for HAN/CFD target export as measured actuator history |
| `event_mavlink_source_servo` | `real_controller_output_telemetry` | `True` | `False` | `False` | `2.000` | `179` | output telemetry layer, not raw actuator backend history; effective PWM step cadence is below 10Hz; can be used as direct plant replay input with semantic caveats |
| `event_mavlink_sink_rc_override` | `real_controller_input_rc_override` | `False` | `True` | `False` | `73.954` | `4323` | controller input layer, upstream of ArduSub motor mixing; can be used to reconstruct controller-side state/RCOU; not allowed for HAN/CFD target export as measured actuator history |

## Conclusion

Use rc_override/mavlink_rc_override only for controller-side reconstruction, or capture a measured high-rate actuator/motor backend history.  Do not promote low-rate RCOU interpolation as HAN/CFD truth.

Direct real-RCOU plant replay can still use the output telemetry layer as the observed input contract. That does not make it a valid high-rate actuator-history target for HAN/CFD training.
