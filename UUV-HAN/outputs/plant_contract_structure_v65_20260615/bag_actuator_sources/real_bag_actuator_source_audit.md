# Real Bag Actuator Source Audit

- bag_db3: `90s/rosbag/mp4_start_0s_90s_all_topics_0.db3`
- min_plant_hz: `20.0`
- plant_grade_actuator_history_available: `False`
- controller_parity_source_available: `True`

## Candidate Topics

| topic | type | count | mean Hz | median dt s | plant role |
| --- | --- | ---: | ---: | ---: | --- |
| `/uas1/mavlink_sink` | `mavros_msgs/msg/Mavlink` | 12477 | 138.631 | 0.005 | MAVLink input transport |
| `/uas1/mavlink_source` | `mavros_msgs/msg/Mavlink` | 11348 | 126.132 | 0.000 | MAVLink transport; decode required |
| `/mavros/rc/override` | `mavros_msgs/msg/OverrideRCIn` | 10515 | 116.830 | 0.009 | pilot/controller input, not actuator output |
| `/mavros/rc/out` | `mavros_msgs/msg/RCOut` | 180 | 2.000 | 0.500 | controller parity telemetry |
| `/mavros/rc/in` | `mavros_msgs/msg/RCIn` | 180 | 2.000 | 0.500 |  |

## Decoded MAVLink Output Messages

- decoded summary: `uuv_mujoco/v2.2/debug/controller_parity_412/real_90s_mavlink_source_audit_20260601/real_mavlink_source_telemetry_summary.json`
- source topic: `/uas1/mavlink_source`

| message | count | estimated Hz | plant grade |
| --- | ---: | ---: | --- |
| `SERVO_OUTPUT_RAW` | 179 | 1.989 | False |
| `ACTUATOR_OUTPUT_STATUS` | 0 | nan | False |
| `ESC_TELEMETRY_1_TO_4` | 0 | nan | False |
| `ESC_TELEMETRY_5_TO_8` | 0 | nan | False |
| `ESC_TELEMETRY_9_TO_12` | 0 | nan | False |
| `ESC_TELEMETRY_13_TO_16` | 0 | nan | False |
| `ESC_STATUS` | 0 | nan | False |

## Conclusion

No plant-grade high-rate actuator-output history was found in this bag. The available high-rate signals are RC override / MAVLink transport inputs; /mavros/rc/out and decoded SERVO_OUTPUT_RAW remain low-rate telemetry. Controller parity can use SERVO_OUTPUT_RAW, but MuJoCo plant replay cannot identify high-rate actuator dynamics from this bag alone.
