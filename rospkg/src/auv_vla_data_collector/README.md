# KMU26 AUV VLA data collector

실제 KMU26 AUV에서 U0 파인튜닝용 episode를 수집하는 ROS 2 패키지입니다. 다음 데이터를
10 Hz로 묶어 저장합니다.

- 전방 compressed RGB 카메라
- 부표 분리부 compressed RGB 카메라
- DVL 속도와 고도/validity
- MAVROS IMU 각속도, 선형가속도, 자세 quaternion
- 압력 센서에서 변환된 수심
- `/mavros/rc/override`의 surge, sway, heave, yaw 조작 명령
- 자연어 task description

RC override는 motor PWM이 아니라 다음 순서의 정규화된 action label로 저장됩니다.

```text
[surge, sway, heave, yaw] ∈ [-1, 1]
```

ArduSub 기본 채널은 각각 `[5, 6, 3, 4]`이며 파라미터로 변경할 수 있습니다. `0`
(`CHAN_RELEASE`)는 해당 축의 제어권을 무효화합니다. `65535` (`CHAN_NOCHANGE`)는
직전 명령을 유지하지만 유효 시간을 갱신하지 않습니다.

## 빌드

```bash
cd /home/kuuve/auv_ros2
source /opt/ros/humble/setup.bash
colcon build --base-paths src --symlink-install \
  --packages-ignore mavros_msgs \
  --packages-select auv_dvl_a50_msg kmu26_auv_vla_data_collector
source install/setup.bash
```

이 워크스페이스에는 ROS 1 보관 소스와 ROS 2 MAVROS 소스가 함께 있어, 위 명령은 ROS 1
디렉터리의 동명 `mavros_msgs`가 검색되는 것을 피하고 `/opt/ros/humble`의 메시지를 사용합니다.

## 실행

두 번째 카메라의 실제 토픽을 launch argument로 지정합니다.

```bash
ros2 launch kmu26_auv_vla_data_collector collector.launch.py \
  buoy_release_image_topic:=/actual/release/camera/image_raw/compressed
```

전체 설정은 [config/collector.yaml](config/collector.yaml)에 있습니다. 특히 수집 전에 다음을
실제 장비와 대조해야 합니다.

- 두 번째 카메라 토픽
- RC channel 순서 및 각 축 부호
- `/dvl/twist`가 사용하는 좌표축
- `/depth/pose.position.z`가 위쪽 양수인지 여부
- PWM 중립점과 span

## Episode 수집

먼저 영어 task instruction을 publish합니다.

```bash
ros2 topic pub --once /vla/task_description std_msgs/msg/String \
  "{data: 'Approach the red buoy.'}"
```

모든 필수 입력이 최신 상태일 때만 episode가 시작됩니다.

```bash
ros2 service call /vla_data_collector/start_episode std_srvs/srv/Trigger "{}"
```

성공한 수행을 저장합니다.

```bash
ros2 service call /vla_data_collector/stop_episode std_srvs/srv/SetBool "{data: true}"
```

실패했지만 recovery 학습에 사용할 수행은 `data: false`로 저장합니다. 센서 설정 오류나
사람이 끼어든 수행처럼 학습에 사용하면 안 되는 episode는 저장 전에 폐기합니다.

```bash
ros2 service call /vla_data_collector/discard_episode std_srvs/srv/Trigger "{}"
```

노드가 recording 도중 종료되면 수집된 frame이 있는 episode는 `success=false`와
`termination_reason=node_shutdown`으로 보존됩니다.

## 저장 구조

수집 단계에서는 ROS 런타임에 pandas/pyarrow를 요구하지 않도록 JPEG와 NPZ로 저장합니다.

```text
vla_data/staging/
└── episode_000000/
    ├── manifest.json
    ├── samples.npz
    └── frames/
        ├── ego/frame_000000.jpg
        └── buoy_release/frame_000000.jpg
```

`samples.npz`에는 23차원 `observation_state`, 4차원 `action`, 원본 RC PWM과 각 채널의
update mask, ROS timestamp, 각 센서의 원본 timestamp와 sample 시점 기준 age가 포함됩니다.

## LeRobot/U0 형식으로 변환

변환은 `pandas`, `pyarrow`, `ffmpeg`가 설치된 U0 학습 환경에서 실행합니다.

```bash
source /home/kuuve/auv_ros2/install/setup.bash
ros2 run kmu26_auv_vla_data_collector export_lerobot \
  /home/kuuve/auv_ros2/vla_data/staging \
  /home/kuuve/auv_ros2/vla_data/lerobot_train
```

출력 폴더가 비어 있지 않으면 변환기는 덮어쓰지 않고 중단합니다. 변환 결과는
`kmu26_auv_vla` 저장소의 `--data-config kmu26_auv_real`과 바로 대응합니다.

## 중요한 제한

현재 ROV 기본 DVL 축 변환과 시각/RC 유효성 정책은 아래 Current ROV 절을 따릅니다.
다른 장착 방향은 앞단에서 body frame으로 변환해야 합니다.

## Current ROV / MuJoCo integration

Use the checked-in `auv_dvl_a50_msg` dependency, not legacy `dvl_msgs`.
The defaults now use `/imx219/camera0/image_raw/compressed` and
`/imx219/camera1/image_raw/compressed`. Camera 1 must actually depict the release
area for this embodiment; a stereo partner is not automatically a release camera.
The dataset directory defaults to `~/vla_data/staging`.

```bash
colcon build --packages-select auv_dvl_a50_msg kmu26_auv_vla_data_collector
source install/setup.bash
# Simulation only; omit use_sim_time for the vehicle.
ros2 launch kmu26_auv_vla_data_collector collector.launch.py use_sim_time:=true
```

`dvl_link` FRD vectors are converted once to body FLU (`x, -y, -z`). This matches
the current ROV's X-pi DVL mounting transform. For a different mounting, provide
an upstream body-frame velocity topic and set `dvl_input_frame:=base_link` and
`dvl_convention:=FLU` in a custom YAML. Unknown DVL/IMU frames are rejected;
IMU must already be in `body_frame` (default `base_link`). The collector does
not infer arbitrary mounting transforms. Depth remains positive down in metres.

Capture and receipt ages must both be valid. Clock resets and missed sampling
intervals end the current recording with `sampling_discontinuity`; start a new
episode after recovery. Export rejects gaps/rate changes instead of silently
compressing time. Normal timer jitter up to 25% of a period is accepted.
RC RELEASE invalidates the affected action immediately; NOCHANGE cannot renew
an expired action. All four primary RC channels must have recent explicit
commands. The prior helper remains available, but the recorder uses per-axis
ownership tracking. Keep the same ArduSub mode, axis signs, neutral and PWM span
across demonstrations and policy execution; default span 300 matches joy2mavros.

Use `requirements-export.txt` in a separate export environment (plus OpenCV and
system `ffmpeg`). Keep failed/recovery episodes separate unless deliberately
including them in training; export does not silently filter `success=false`.
The public `policy_observation(previous_command)` method exposes the same sensor
contract to an inference adapter without publishing RC or requiring an RC source.

## Transfer collection (local additions, 2026-09-10)

The default `collection_kind=connection_check` is for wiring tests. For task data,
set `collection_kind=task_demonstration`, `data_source=simulation` or `real`,
`session_id`, `provenance_file`, and `expected_mode` in your collector YAML.
Demonstrations require fresh connected/armed expected mode and exactly one ROS
RC publisher. Direct MAVLink senders still need operator exclusion. Pauses over
one wall second, clock rewind, mode changes, stale or repeated camera captures
end the episode. PWM outside the declared neutral±span is invalid, not clipped.

RC labels are requested commands, not confirmed instantaneous FCU adoption,
motor force or motion. Per-frame FCU state and RC in/out feedback are stored in
`vehicle_state.jsonl`; receipt and per-axis update times are stored in NPZ. Source
manifests, logs, settings and collector source snapshots survive export.

Use `tools/check_u0_loader.py DATASET --inspection` for a connection test in the
U0 environment. Without `--inspection`, the transfer loader rejects unknown
provenance, connection checks and interrupted/failed episodes. The optional
`transfer_config:Kmu26TransferDataConfig` keeps 23 physical state values instead
of upstream's all-field sin/cos transform. Use the same config at inference.
`Kmu26TrainingDataset` indexes only complete 16-action chunks; use it through
`tools/finetune_transfer.py`, not the unmodified trainer's padded dataset class.
Failed recovery demonstrations need a separately reviewed selection/workflow.

Full comparison, measured camera rates, regression results and operator steps:
`docs/contracts/VLA_TRANSFER_AUDIT_20260910.md` at the workspace root.
