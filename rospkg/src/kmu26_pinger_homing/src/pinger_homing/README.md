# Pinger homing

This directory is the single source of truth for the vehicle-side pinger homing controller.
The hydrophone signal-processing algorithm is not copied into this package. It is consumed from
the team fork of `kmu26_auv_hydrophone` through ROS topics.

## Controllers

- `pinger_homing_controller.cpp`: installed canonical C++ Phase/SNR RC controller.
- `pinger_homing_math.hpp`: robust moving-sensor source fit, yaw stabilization,
  no-odometry ABBA/Huber bearing fit, and safety functions used by that controller.
- `single_hydrophone_homing_controller.py`: archived prior Python controller. It is
  installed only for source compatibility and is not selected by the real launch.

The active controller keeps the tested state order:

```text
WAIT_VEHICLE -> PROBE <-> REPROBE -> ALIGN <-> APPROACH -> CONTACT -> COMPLETE
```

The physical default is `odometry`: it pairs each Phase range-change sample
with `/odometry/filtered`, runs the robust moving-sensor source fit from the
successful legacy controller, locks the fitted source, aligns yaw, and then
approaches while continuing to update the fit. `no_odom_phase` remains an
explicit fallback for a vehicle without trustworthy localization. Simulator
ground truth is never a controller input. ALT_HOLD owns vertical control.

Weak-signal startup does not deadlock on the first rejected Phase window.
The estimator's `/audio_phase_estimator/iq_snr_ratio` heartbeat may authorize
only the bounded probe trajectory. A real delta-range fit and source lock are
still mandatory before ALIGN or APPROACH, so noise can cause a safe probe and
eventual estimate failure but cannot authorize blind forward travel.

The test-tank profile, and an explicit physical `no_odom_phase` fallback,
enable `motion_response` as a separate timing path. It reads **only the XY speed magnitude** from
`/odometry/filtered`, never pose or velocity direction. If a Phase ABBA leg
has not reached `0.03 m/s`, that same leg is extended in `0.30 s` increments
(up to `1.20 s`) and produces no Phase sample until it actually moves. If a
forward segment stays slow after its grace/hold periods, it commands neutral
and repeats the ABBA estimate from the current physical location; it does
**not** enter a failure state. The refreshed Phase result then re-aligns yaw
and can choose a different next approach direction. Missing/stale feedback
reports `feedback_fresh=false` in `/pinger_homing/status` and retains the
validated fixed timing. It is a timing/adaptation input, never a
Phase-bearing input.

For the test tank, the default is `reestimate_policy:=adaptive`, not a fixed
four- or ten-second ABBA cycle. IMU yaw is closed every controller tick and
the controller compares the median Phase delta observed during a forward
segment against the Phase delta predicted by the most recent ABBA fit. A
persistently positive normalized innovation (the range is closing less than
predicted, or is opening) triggers neutral followed by a fresh ABBA estimate.
Low measured XY speed does the same through `motion_response`. `approach_max_s`
is only a stale-bearing watchdog; it is not the normal update frequency.

A single scalar Phase/range-rate stream cannot identify left-versus-right
error during perfectly straight travel. The adaptive gate therefore does not
invent a fake lateral error: it requests a short, observable two-axis ABBA
excitation when the innovation is bad. Tune the gate with
`innovation_window_s`, `innovation_limit`, `innovation_hold_s`, and retain
`initial_confirmation_probes:=2` for a conservative first bearing. Use
`reestimate_policy:=fixed approach_duration_s:=...` only to reproduce the
legacy periodic behavior.

## Topic boundary

Inputs from the forked hydrophone package:

- `/audio_phase_estimator/delta_range_m` (`std_msgs/Float64`)
- `/audio_phase_estimator/iq_magnitude` (`std_msgs/Float64`)
- `/homing/direction` (`geometry_msgs/Vector3Stamped`, optional fit seed)

Vehicle inputs:

- `/odometry/filtered` (`nav_msgs/Odometry`, physical default source-fit input)
- `/mavros/imu/data` (`sensor_msgs/Imu`)
- `/mavros/state` (`mavros_msgs/State`)
- `/depth/pose` (`geometry_msgs/PoseWithCovarianceStamped`, monitored for
  the physical preflight; not a no-odometry Phase fitting input or a gate for
  the horizontal-only ALT_HOLD profile)

Outputs:

- `/mavros/rc/override` directly from the controller in the real launch
- `/pinger_homing/status`
- `/pinger_homing/direction_body`

The controller never reads MuJoCo ground truth. Ground truth is used only by external regression
tests as an oracle.

## Real-vehicle launch

Start the physical audio stream first.  Then use the interactive launch below instead of
launching the fixed-frequency real launch directly. It scans 19--22 kHz for ten seconds, prints up to
five candidates, accepts a candidate number or a Hz value in the terminal,
and starts the untouched `audio_phase_estimator` at that selected frequency.

```bash
ros2 launch kmu26_pinger_homing pinger_homing_real_interactive.launch.py \
  dry_run:=true use_audio_capture:=false tank_max_depth_m:=11.0
```

Set `use_audio_capture:=false` when the vehicle stack already owns `/audio`.
If it must be started here, `use_audio_capture:=true` starts capture first,
then begins the scan.  Fixed-frequency direct launch remains available only
when the operator has already measured a frequency and passes
`reference_frequency_hz:=...` explicitly.

The C++ controller never arms automatically by default and will publish
non-neutral RC only while MAVROS reports both `armed=true` and actual
`ALT_HOLD`.  The first physical profile uses ±20 PWM ABBA probes and +25 PWM
approach demand around 1500; these are launch parameters, not hidden code
constants.  The amplitude range relation `(K / iq_magnitude)^2` still requires
a measured physical calibration, so the default `K=0` disables metric-range
completion.  The simulator-only `0.325` must not be used on the vehicle.

Motion-response defaults are exposed as launch parameters:

```bash
motion_response_min_speed_mps:=0.03 \
motion_response_probe_extension_s:=0.30 \
motion_response_probe_max_extension_s:=1.20 \
motion_response_approach_grace_s:=0.80 \
motion_response_approach_hold_s:=0.80
```

For a slower vehicle, increase `motion_response_probe_max_extension_s` before
lowering the speed threshold. Set `motion_response_enabled:=false` only for a
comparison/commissioning run that has no usable twist feedback.
