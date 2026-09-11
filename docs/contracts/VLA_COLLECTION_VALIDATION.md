# VLA collector simulation integration — 2026-09-08

The organization collector at commit `4b4e2328f7b36a51c30b4929b8bf015f9396e4c3`
is included in `rospkg/src/auv_vla_data_collector` and builds as
`kmu26_auv_vla_data_collector`. Upstream logic is unchanged.

## Observed integration result

An isolated container (ROS domain 87) ran the real MuJoCo runtime, ArduSub SITL,
MAVROS, the A50 TCP emulator and physical A50 ROS driver, and base ROV bridges.
The collector subscribed to actual published sensor topics. A test publisher
provided neutral RC overrides; the vehicle remained disarmed. Existing GUI and
other running containers were not reconfigured.

With rendered cameras at 320×240 and 30 Hz, the collector stored a five-second
stationary connection check at 10 Hz:

- 50 samples, state `(50, 23)`, action `(50, 4)`.
- 0.1 s spacing; first-to-last timestamp separation 4.9 s.
- Both camera validity flags, DVL velocity validity and altitude validity were
  true in every sample. All state/action values were finite.
- Actions were all neutral. Quaternion norm error was below 1.2e-7.
- Export produced 50 Parquet rows, statistics/modality metadata and two 320×240
  MP4 videos, each with 50 frames at 10 fps.
- A recorded image was inspected and contained the rendered underwater scene.

A preceding 10 Hz camera-publication trial produced camera validity of 72%:
capture ages reached 0.2875 s, exceeding the 0.25 s limit. Publishing cameras at
30 Hz resolved this in the recorded sample without relaxing freshness checks or
disabling modeled sensor latency. This does not establish performance under all
GUI loads, resolutions or camera configurations.

## How to collect

Follow the strict simulation bringup in `REAL_STACK_PARITY.md`; use
`--ros2-images --ros2-image-hz 30`, enable the A50 device emulator, and run the
external `auv/rov_start.launch.py` stack. Build/source the collector and launch:

```bash
ros2 launch kmu26_auv_vla_data_collector collector.launch.py use_sim_time:=true
```

Publish a task description and begin/end episodes through the collector's
services as documented in `rospkg/README.md` and its package README. The physical
vehicle uses `use_sim_time:=false`. Use the same clock and ROS domain for sensors
and collector. Do not run duplicate MAVROS or sensor owners in the same domain.

## Limits and evidence

This was a disarmed stationary integration check, not a successful task
demonstration or VLA-controlled trajectory. Both episodes were saved with
`success=false` and kept outside training data. At the time of this 2026-09-08 test, camera1 was a stereo view. Since
2026-09-10 it is a hand camera looking at the right rake; see
`docs/gui/CAD_SENSOR_MOUNTS.md`. The historical test below does not validate
that newer camera configuration. No model training,
GPU policy inference or real-vehicle execution was performed.

Local generated evidence is under the ignored directory
`outputs/vla-sim-validation-20260908/`, with final results in `camera30hz/`:
`validation.json`, `input-diagnostics.json`, raw `staging/`, exported `lerobot/`
and process logs. The test scripts live alongside these generated artifacts.

## Updated transfer audit

See `VLA_TRANSFER_AUDIT_20260910.md` for the current defects, fixes, full loader
validation, time/RC limitations and operator collection procedure. Local collector
and adapter now contain additional transfer fixes beyond the upstream commits.
