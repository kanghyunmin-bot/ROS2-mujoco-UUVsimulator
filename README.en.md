# UT27 Research Studio

[한국어](README.md) · **English**

Pilot an underwater robot in a MuJoCo research pool and collect demonstrations for vision-language-action policies. The dashboard shows the front and hand cameras alongside attitude, depth, and Doppler velocity log (DVL) readings.

![Research dashboard with the yellow buoy in the front camera, the fork in the hand camera, and live attitude, depth, and velocity readings below](docs/assets/ut27-research-studio.png)

## Run

Follow the [Docker setup guide](docker/ubuntu-dev/README.md), then run this command from the root of this branch.

```bash
bash tools/start_research_gui.sh
```

Open the [research GUI](http://127.0.0.1:18879/). Select **시뮬 시작** (start simulation), choose **STABILIZE**, select **ARM**, then enable **수동 입력 켜기** (manual input). When finished, select **조종 입력 해제** (release input), then **DISARM**.

## Validation · 2026-11-03

- **Control and sensors** — Four-axis control, automatic input release, both cameras, and IMU, depth, and DVL readings were verified.
- **Recording** — On the test PC, CPU rendering produced about one new frame per second per camera. Capture delay exceeded the collector’s 0.25-second limit, so recording was blocked.
- **Open items** — GUIDED and POSHOLD transitions and YOLO startup failed in this environment. GPU policy inference still requires validation.

[UT27 paper and figures](docs/papers/ut27_2027/) · [Data collector](rospkg/src/auv_vla_data_collector/README.md) · [Run a VLA model](tools/vla_gui/README.md) · [Training results](docs/UUV_UPDATE_20260922.md)
