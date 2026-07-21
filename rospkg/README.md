# KMU26 real-vehicle ROS 2 workspace

The physical-vehicle source tree lives only under `rospkg/src/`. It is split
into two control packages so pinger homing can be deployed independently from
the vision mission:

- `kmu26_pinger_homing`: C++ Phase/SNR RC controller, RC mux, preflight and
  pinger launch files.
- `kmu26_vision_mission_fsm`: C++/YOLO buoy mission controller.
- `kmu26_auv_hydrophone/audio_capture`: forked Phase and SNR estimators.
- `hit25_auv_ros2`: physical MAVROS, pressure, DVL and localization drivers.
- `kmu26_auv_web_gui`: the physical-vehicle Web GUI.

`real_robot.repos` records the pinned upstream baselines. The active `src/`
tree also contains the validated C++ NO_ODOM_PHASE, Phase peak-selection and
GUI changes; do not overwrite it with a bare `vcs import` before those changes
have been published.

## Resource-bounded build

```bash
cd /home/robot/uuv_sim_current/rospkg
./build_safe.sh --cmake-args -DBUILD_TESTING=OFF
source install/setup.bash
```

The helper uses `/usr/bin/gcc` and `/usr/bin/g++` directly, one compiler job,
low CPU/I/O priority and a memory watchdog. It does not terminate VS Code,
MAVROS, MuJoCo or other user processes.

## Real-vehicle topic contract

- raw hydrophone PCM: `/audio`
- IMU: `/mavros/imu/data`
- Bar30 depth pose: `/depth/pose`
- optional localization: `/odometry/filtered`
- vehicle state: `/mavros/state`
- exclusive command output: `/mavros/rc/override`
- pinger status: `/pinger_homing/status`

Start the physical driver stack first. Its defaults retain the real serial
MAVROS and wall-time contract:

```bash
ros2 launch hit25_auv_ros2 rov_start.launch.py \
  use_sim_time:=false use_web_gui:=false
```

## Pinger modes

For a physical run, discover the carrier before starting control. The
standalone Pinger GUI starts audio capture and a passive C++ scan automatically:

```bash
ros2 launch kmu26_pinger_homing pinger_homing_gui.launch.py \
  phase_peak_audio_device:="hw:0,0" \
  phase_peak_min_frequency_hz:=15000 \
  phase_peak_max_frequency_hz:=25000
```

Open `http://127.0.0.1:8878/`. The GUI shows up to five stable carrier
candidates with frequency, magnitude, SNR, quality and temporal support.
Select one candidate first; the Phase estimator/controller cannot start until
the scanner acknowledges that exact frequency. The scanner is passive and
does not publish RC commands.

Safe no-thruster check of the explicit no-localization Phase mode:

```bash
ros2 launch kmu26_pinger_homing pinger_homing_real.launch.py \
  estimator_mode:=phase navigation_mode:=no_odom_phase \
  use_audio_capture:=true reference_frequency_hz:=21164.0 \
  no_odom_probe_pwm_delta:=90 \
  no_odom_approach_pwm_delta:=120 \
  no_odom_forward_duration_s:=4.0 \
  dry_run:=true
```

The two PWM values are neutral-centred microsecond deltas: horizontal probe
motion is `1500 +/- delta`, and approach is `1500 + delta`. The real GUI
accepts a 32--140 probe delta, a 20--200 approach delta, and a 0.5--20 second
approach interval. Approach is also capped by `forward_max` (400 us RC span).

`NO_ODOM_PHASE` never subscribes to `/odometry/filtered` or
`/homing/direction`. It stops, probes paired body axes with known RC commands,
uses Phase range change to score those motions, uses the IMU only for attitude
and yaw alignment, moves forward, then stops and probes again. Bar30 depth is
used only for vertical safety. Loss of audio, IMU, depth, MAVROS state or arm
state immediately returns the command to neutral and restarts from a probe.

The existing localization modes remain available:

```bash
# Phase direction estimator + odometry
ros2 launch kmu26_pinger_homing pinger_homing_real.launch.py \
  estimator_mode:=phase navigation_mode:=odometry dry_run:=true

# SNR gradient estimator + odometry
ros2 launch kmu26_pinger_homing pinger_homing_real.launch.py \
  estimator_mode:=snr navigation_mode:=odometry \
  snr_horizontal_only:=false dry_run:=true
```

Change `dry_run:=false` only after the real preflight passes and the vehicle is
restrained in water. Pinger and vision control must not run concurrently; the
RC mux enforces single ownership.

## Web GUI

```bash
ros2 launch kmu26_pinger_homing pinger_homing_gui.launch.py port:=8878
```

Open `http://127.0.0.1:8878/`, choose and confirm a 15--25 kHz Phase peak,
select `NO_ODOM_PHASE`, verify IMU/depth/audio and MAVROS are fresh, then start
pinger homing. The simulator GUI at the same port exposes the identical mode
and RC contract.

## Focused verification

```bash
source /opt/ros/humble/setup.bash
source /home/robot/uuv_sim_current/rospkg/install/setup.bash
cd /home/robot/uuv_sim_current/rospkg/build/kmu26_pinger_homing
ctest --output-on-failure -j1
```

Ground truth is never an input to the deployable pinger controller. It is used
only as a MuJoCo test oracle.
