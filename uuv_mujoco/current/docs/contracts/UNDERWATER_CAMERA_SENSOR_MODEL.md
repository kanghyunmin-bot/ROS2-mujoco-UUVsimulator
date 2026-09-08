# Underwater Camera Sensor Boundary

The underwater camera model is an opt-in transform between the ideal MuJoCo
RGB render and the ROS image publishers. It does not change the scene or camera
poses. Existing topic names, frame IDs, message types, and image encodings stay
compatible, while explicit dual-IMX219 aliases expose the real-stack boundary.

The default profile is
`config/sensor_models/imx219_underwater_uncalibrated_prior.json`. Its status is
`unvalidated_prior_not_measured_on_physical_imx219_pair`. The attenuation,
backscatter, sensor-noise, and timing values are engineering priors, not IMX219
product specifications or measurements. The model remains disabled by default
so existing runs retain byte-for-byte legacy image behavior and avoid extra
full-frame processing cost.

## Enable the model

```bash
export ROS2_UUV_CAMERA_SENSOR_MODEL_ENABLE=1
```

Use a different validated JSON profile with:

```bash
export ROS2_UUV_CAMERA_SENSOR_MODEL_CONFIG=/absolute/path/camera_profile.json
```

The existing CLI calibration inputs are now active:

```bash
--ros2-camera-calib-left /absolute/path/camera0.yaml \
--ros2-camera-calib-right /absolute/path/camera1.yaml
```

They accept the standard ROS CameraInfo YAML fields (`image_width`,
`image_height`, `camera_matrix`, `distortion_model`,
`distortion_coefficients`, `rectification_matrix`, and `projection_matrix`).
Intrinsics are scaled to the configured render resolution. The same
calibration drives both the published CameraInfo payload and the modeled lens
distortion, so the image and metadata do not silently disagree.

The JSON profile represents:

- pinhole intrinsics and plumb-bob or rational-polynomial distortion;
- wavelength-dependent attenuation and color cast;
- backscatter/veiling light through a homogeneous effective optical path;
- bounded box blur and radial vignetting;
- exposure, analog gain, black level, signal-dependent shot noise, read noise,
  ADC quantization, and final `rgb8` conversion;
- capture-rate gating, affine device clock, bounded processing/link latency,
  seeded frame dropout, and a bounded drop-oldest/drop-newest queue.

Every random draw is derived from the profile seed, camera side, and frame
sequence. Enabling the sensor model also forces synchronous rendering; the
legacy single-slot async renderer is intentionally retained only for the
interactive model-disabled path because wall-thread completion can change
which simulation snapshot it accepts. On the same rendering platform,
repeating a sensor-model run with the same simulation inputs and seed therefore
produces the same modeled pixels, dropout decisions, and latency sequence.
Left and right random streams remain independent.

Runtime overrides for experiments are:

| Environment variable | Meaning |
| --- | --- |
| `ROS2_UUV_CAMERA_SENSOR_MODEL_SEED` | deterministic base seed |
| `ROS2_UUV_CAMERA_FRAME_DROPOUT_PROBABILITY` | whole-frame dropout probability |
| `ROS2_UUV_CAMERA_PROCESSING_LATENCY_S` | mean sensor processing latency |
| `ROS2_UUV_CAMERA_PROCESSING_JITTER_S` | processing-latency standard deviation |
| `ROS2_UUV_CAMERA_TRANSPORT_LATENCY_S` | mean host transport latency |
| `ROS2_UUV_CAMERA_TRANSPORT_JITTER_S` | transport-latency standard deviation |

Modeled raw image, compressed image, and CameraInfo messages share the original
capture timestamp. A delayed frame is never relabeled with its later ROS
arrival time. Whole-frame dropout suppresses the matching CameraInfo sample as
well as the images. The two-frame default queue bounds retained image memory,
and shutdown discards queued frames and full-frame distortion/vignetting maps.

## Calibration workflow

Replace the prior only after collecting synchronized physical data:

1. Pin resolution, frame rate, exposure, analog gain, white balance, and driver
   settings used on the vehicle.
2. Calibrate each camera in water with the final port/dome installed. Store its
   CameraInfo YAML and verify reprojection error on held-out images.
3. Record dark frames and flat fields at each operational exposure/gain pair to
   identify black level, read noise, shot-noise scale, vignetting, and bad
   pixels.
4. Record a color chart and range-marked targets in representative pool water
   to fit attenuation, veiling light, and blur without overfitting scene
   textures.
5. Compare hardware capture timestamps with host receive timestamps to fit
   clock offset/drift, processing latency, transport jitter, and burst/dropout
   statistics.
6. Validate visual odometry or SLAM separately on held-out physical sequences;
   matching image histograms alone is not sim-to-real validation.

Pass the two measured physical CameraInfo files through the canonical bringup;
otherwise the physical driver uses its zero-intrinsics example file:

```bash
ros2 launch hit25_auv_ros2 rov_start.launch.py \
  camera0_calibration_file:=/absolute/path/camera0_in_water.yaml \
  camera1_calibration_file:=/absolute/path/camera1_in_water.yaml
```

## Known limits

This low-cost model uses one configurable effective optical path for the whole
image. It does not render a per-pixel depth buffer, volumetric caustics, marine
snow, bubbles, rolling-shutter motion, auto-exposure state, compression packet
loss, hot pixels, or temporal sensor correlation. These should be added only
when real data shows they materially affect the SLAM metric.

The bridge keeps its existing `/stereo/...` and `/camera/camera/color/...`
surfaces and also publishes the real-stack aliases below when
`--ros2-images` is enabled. The direct bridge CLI defaults to the physical
contract of 1280x720 at 30 Hz. The GUI intentionally defaults to 640x360 at
4 Hz to preserve interactive headroom; a parity run must select its
`hd720_realtime` camera preset or pass 1280x720 and 30 Hz explicitly:

```bash
--ros2-image-width 1280 --ros2-image-height 720 --ros2-image-hz 30
```

- `/imx219/camera0/image_raw`, `/imx219/camera0/image_raw/compressed`, and
  `/imx219/camera0/camera_info`, stamped in
  `imx219_camera0_optical_frame`;
- `/imx219/camera1/image_raw`, `/imx219/camera1/image_raw/compressed`, and
  `/imx219/camera1/camera_info`, stamped in
  `imx219_camera1_optical_frame`.

All camera publishers use sensor-data QoS. Each alias reuses the same modeled
left or right delivery, so its image and CameraInfo share capture time and the
same whole-frame dropout decision. In simulation, bridge-owned static TF
attaches the two IMX219 optical frames to the corresponding MuJoCo stereo
camera frames. On the physical robot there is no trustworthy default
extrinsic: `rov_start.launch.py` keeps `publish_imx219_static_tf:=false` and
warns until both `base_link`-to-optical xyz/rpy transforms have been measured,
supplied through the `imx219_camera{0,1}_{x,y,z,roll,pitch,yaw}` arguments, and
explicitly enabled. Physical localization or SLAM must not consume the images
before that step or before both measured `camera{0,1}_calibration_file` YAMLs
have been supplied.

The IMX219 raw aliases use `bgr8`, matching the physical GStreamer driver;
legacy `/stereo/...` images remain `rgb8`. Compressed aliases reuse the exact
JPEG produced for the matching frame rather than re-rendering it.

The physical `auv_imx219_camera` workspace is pinned to upstream commit
`a6b5b9455f082c201496326bfdaee2791f4a4d90` with a local capture-timestamp
overlay. Its default `gstreamer_pts` mode maps each buffer PTS into the ROS
clock from GStreamer running time. Invalid, future, or over-age PTS advances
from the last stamp by one configured frame period, capped by the current ROS
time. A regressing valid PTS uses the same correction, and only a ROS clock
that has not advanced requires the minimum 1 ns increment. All corrections are
strictly monotonic and emit a throttled warning with a cumulative correction
count. `timestamp_source:=ros_now` provides an explicit rollback. This overlay
still requires a commit in the physical camera repository before a fresh
`vcs import` can reproduce it.
