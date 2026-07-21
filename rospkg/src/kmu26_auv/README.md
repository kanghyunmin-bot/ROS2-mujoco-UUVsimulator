# KMU26 physical AUV ROS 2 package

`hit25_auv_ros2` starts the physical vehicle sensors, MAVROS, localization, and
joystick control.

## Shared RC output with Pinger Homing

The default behavior is unchanged: `joy2mavros` publishes directly to
`/mavros/rc/override`. When it is used with `kmu26_pinger_homing`, route the
joystick through the dedicated RC mux input and release ownership while the
sticks are idle:

```bash
ros2 launch hit25_auv_ros2 localization_test.launch.py \
  joy_rc_output_topic:=/control/joystick/rc_override \
  joy_release_when_idle:=true
```

The resulting contract is:

```text
joy2mavros       -> /control/joystick/rc_override
pinger_homing    -> /control/pinger/rc_override
rc_override_mux  -> /mavros/rc/override
```

Idle inputs and axis noise inside the configured deadzones publish
`CHAN_RELEASE`. Deliberate joystick motion publishes active PWM and takes manual
priority in `rc_override_mux`. Arming and mode buttons continue to use MAVROS
services. The ALT_HOLD entry sequence retains its timed vertical-neutral
override before returning to idle release.

The Pinger Homing Web GUI supplies the two launch arguments automatically.

Run the contract test after building:

```bash
colcon test --packages-select hit25_auv_ros2 \
  --ctest-args -R joy2mavros_mux_input
```
