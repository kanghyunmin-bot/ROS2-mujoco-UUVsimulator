# KMU26 AUV Control

이 저장소 자체가 핑거 호밍 ROS 2 패키지 `auv_pinger_homing`이다. 2-D Phase/SNR 주파수
선택기, 오디오 추정기, RC 제어기는 모두 이 루트 패키지에 통합되어 있다.
비전 미션 FSM은 작업공간의 `archive/kmu26_vision_mission_fsm`으로 분리되어
기본 빌드와 실행에 포함되지 않는다.

```text
package.xml                 ROS package manifest
CMakeLists.txt              ROS package build definition
launch/                     Phase/SNR/interactive launch files
src/                        C++ controller and frequency selector
```

NUC의 최종 소스 경계는 다음과 같다.

```text
~/auv_ws/src/
├── auv_pinger_homing/            # 이 Git 저장소 = ROS package root
│   ├── package.xml
│   ├── launch/
│   └── src/
├── kmu26_auv_hydrophone/           # 별도 Git 저장소, 신호처리 ROS 패키지들
    ├── audio_common/
    ├── audio_common_msgs/
    └── audio_capture/
└── archive/kmu26_vision_mission_fsm/  # src 밖의 보관본
```

`kmu26_auv_hydrophone`을 이 저장소 안에 복사하거나 중첩 clone하지 않는다.

## 설치

```bash
mkdir -p ~/auv_ws/src
cd ~/auv_ws/src
git clone --branch main https://github.com/2026-kmu-underwater-robot/kmu26_auv_pinger_homing.git auv_pinger_homing
cd ~/auv_ws
vcs import src < src/auv_pinger_homing/hydrophone.repos
cd ~/auv_ws/src
git clone https://github.com/2026-kmu-underwater-robot/kmu26_auv.git
cd ~/auv_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install \
  --packages-up-to auv_pinger_homing
source install/setup.bash
```

이제 clone 직후 실제 패키지 경로는
`~/auv_ws/src/auv_pinger_homing` 하나뿐이다. 예전의
`~/auv_ws/src/auv_pinger_homing/auv_pinger_homing` 경로는 사용하지 않는다.

세부 Phase/FFT 판정 기준은 [PINGER_HOMING.md](PINGER_HOMING.md)를 참고한다.

`auv`가 사용하는 팀 `auv_dvl_a50_msg` 패키지도 실물 ROS 작업공간에 있어야 한다.

## 실물 핑거 호밍 실행

`pinger_homing_real_interactive.launch.py`가 실물의 표준 진입점이다.
기존 `kmu26_auv_hydrophone`의 `/audio`를 10초간 19--22 kHz FFT 스캔하고, 터미널에서
후보 번호 또는 정확한 주파수를 선택하면 **C++ Phase 제어기**를
시작한다. 하이드로폰 신호처리 알고리즘 자체는 이 저장소에서 수정하지 않는다.
실물 기본값은 신뢰 가능한 `/odometry/filtered` 좌표와 검증된 레거시 probe 궤적으로
핑거 위치를 robust fitting한다. localization이 없으면 `no_odom_phase`의 X/Y ABBA로
방위를 추정한다. pinger controller가 `/mavros/rc/override`에 직접 발행하며 별도의
RC mux를 사용하지 않는다. 호밍 중에는 다른 노드가 같은 토픽에 활성 명령을
발행하지 않게 한다.

### 시작 전 확인

1. MAVROS가 연결돼 있고, 조종기로 `ALT_HOLD` 모드와 arm 상태를 만든다.
   기본값은 `auto_arm:=false`, `auto_mode:=false`다. 이 launch가 임의로 arm하거나
   모드를 바꾸지 않는다.
2. 하이드로폰 스택이 `/audio`를 단독 publish하는지 확인한다. 이미 별도
   `audio_capture`를 실행 중이면 `use_audio_capture:=false`를 유지한다.
3. 호밍 중 pinger가 유일한 활성 RC command publisher가 되도록 `joy2mavros`를 끈다.

```bash
ros2 launch auv rov_start.launch.py \
  use_joy2mavros:=false
```

4. 처음에는 반드시 `dry_run:=true`로 FFT 후보, `/pinger_homing/status`, IMU,
   audio 입력부터 확인한다. 추진이 가능한 수조·테더·비상정지 조건이 확보된 뒤에만
   `dry_run:=false`로 바꾼다.

```bash
source /opt/ros/humble/setup.bash
# 실제로 빌드한 작업공간을 source한다. 예: ~/auv_ws 또는 ~/catkin_ws
source ~/catkin_ws/install/setup.bash

ros2 topic echo /mavros/state --once
ros2 topic hz /audio
ros2 topic echo /mavros/imu/data --once
```

### 1. 실물 dry-run

`tank_max_depth_m`는 현재 수조의 실제 최대 깊이로 넣는다. 아래 예시는 2 m
수조이며, 실물 시간 기준이므로 `use_sim_time`은 지정하지 않는다(`false` 기본값).

```bash
ros2 launch auv_pinger_homing pinger_homing_real_interactive.launch.py \
  dry_run:=true \
  navigation_mode:=no_odom_phase \
  use_audio_capture:=false \
  tank_max_depth_m:=2.0
```

10초 후 출력되는 후보에서 `1`~`5` 또는 주파수 Hz를 입력한다. `qualified` 후보를
우선 선택한다. dry-run에서는 주파수 선택, Phase 추정, ABBA 상태 변화만 확인하며
RC 출력은 neutral이다.

### 2. 실물 저속 호밍

MAVROS가 `connected=true`, `armed=true`, `mode=ALT_HOLD`임을 확인한 뒤 같은
명령에서 `dry_run:=false`만 바꾼다. 시작 직후 C++ 제어기가
`/mavros/rc/override`에 직접 ABBA 명령을 발행한다.

```bash
ros2 launch auv_pinger_homing pinger_homing_real_interactive.launch.py \
  dry_run:=false \
  navigation_mode:=no_odom_phase \
  use_audio_capture:=false \
  tank_max_depth_m:=2.0 \
  probe_pwm_delta:=20 \
  approach_pwm_delta:=25
```

종료는 실행 터미널에서 `Ctrl-C`다. 제어기가 RC override를 release하고, MuJoCo Web GUI가
있는 환경에서는 GUI의 수동 RC publisher도 자동 복구한다.

### 실물 기본 파라미터

| 그룹 | 파라미터 | 기본값 | 의미 |
| --- | --- | ---: | --- |
| 오디오 | `audio_topic` | `/audio` | 하이드로폰 PCM 입력 |
|  | `audio_sample_rate` / `audio_channels` / `audio_sample_format` | `96000` / `2` / `S32LE` | 실제 캡처 형식과 반드시 일치해야 함 |
|  | `use_audio_capture` | `false` | 이미 팀 하이드로폰 스택이 `/audio`를 내보내면 유지 |
| FFT 선택 | `scan_monitor_s` | `10.0 s` | 후보를 누적 관측하는 시간 |
|  | `scan_min_frequency_hz` / `scan_max_frequency_hz` | `19000` / `22000 Hz` | 21 kHz 핑거 주변만 탐색 |
|  | `scan_combine_channels` | `true` | 채널별 noise floor 정규화 후 전력을 합산; 위상 상쇄 방지 |
|  | `scan_fft_size` / `scan_fft_hop_size` | `16384` / `8192` | 96 kHz에서 5.859 Hz bin, 50% overlap |
|  | `scan_min_snr_db` / `scan_min_peak_prominence_db` | `9.0` / `4.5 dB` | 자동 선택 가능 후보의 SNR·고립 피크 기준 |
|  | `scan_persistent_min_ratio` | `0.30` | 약한 신호도 10초 창의 30% 이상 같은 주파수면 후보 인정 |
| 위치 | `navigation_mode` | `odometry` | `/odometry/filtered` 위치와 Phase 거리변화로 source fitting |
|  | `odometry_topic` | `/odometry/filtered` | 실물 로컬라이제이션 pose/twist 입력 |
| 약신호 bootstrap | `bootstrap_probe_on_audio_quality` | `true` | Phase delta가 아직 없어도 IQ quality heartbeat가 있으면 제한된 probe 시작 |
|  | `audio_quality_topic` | `/audio_phase_estimator/iq_snr_ratio` | 오디오 분석 생존 확인; source lock이나 접근 허가에는 사용하지 않음 |
| 차량 계약 | `mode` | `ALT_HOLD` | live RC를 허용하는 ArduSub 모드 |
|  | `rate_hz` | `30 Hz` | yaw/상태 제어 주기 |
|  | `auto_arm` / `auto_mode` | `false` / `false` | 실물에서 임의 arm·모드변경 방지 |
|  | `tank_max_depth_m` | `11.0 m` | 반드시 현재 수조 깊이로 override |
| RC | `rc_pwm_span` | `400` | 정규화 명령 1.0에 대응하는 neutral 기준 PWM 폭 |
|  | `rc_deadzone_compensation_enabled` | `true` | 실물 Pixhawk `RCx_DZ`를 non-neutral 명령에 자동 가산 |
|  | `rc_xy_deadzone_pwm` / `rc_yaw_deadzone_pwm` / `rc_heave_deadzone_pwm` | `30` / `40` / `30` | 팀 실물 ArduSub 파라미터 계약 |
|  | `probe_pwm_delta` | `±20` | deadzone 바깥에서 사용할 ABBA 유효 PWM; XY raw 출력은 기본 `1500±50` |
|  | `approach_pwm_delta` | `+25` | deadzone 바깥의 유효 전진 PWM; raw 출력은 기본 `1555` |
| 종료 | `arrival_radius_m` | `0.8 m` | 절대 거리 보정이 없을 때 source-fit 도착 반경; Phase fit 편향을 고려해 실제 1.5 m 이내에 들기 위한 보수적 기본값 |
|  | `first_lock_confirmation_radius_m` | `3.0 m` | 첫 uncalibrated fit이 이보다 가까우면 즉시 완료하지 않고 mirrored probe로 재확인 |
| odom 레거시 probe | `legacy_probe_duration_scale` | `1.0` | 성공한 Python 전·후·좌·우·수직 중립분리 궤적의 시간 배율 |
| no-odom fallback | `probe_leg_s` / `probe_neutral_s` | `1.50 s` / `0.50 s` | `navigation_mode:=no_odom_phase`에서만 사용하는 각 자극 leg/중립 간격 |
|  | `probe_settle_s` / `probe_sample_delay_s` | `0.80 s` / `0.45 s` | no-odom 시작 안정화와 spool-up 제외 시간 |
|  | `initial_confirmation_probes` | `2` | no-odom 첫 방향 확정 전 반복 횟수 |
| no-odom 적응 | `reestimate_policy` | `adaptive` | 신호·운동 피드백으로 no-odom ABBA 재실행 |
|  | `approach_min_s` / `approach_max_s` | `2.5 s` / `25 s` | innovation 평가 시작 시점 / stale-bearing watchdog |
|  | `innovation_window_s` / `innovation_limit` / `innovation_hold_s` | `0.70 s` / `1.50` / `1.20 s` | 관측 Phase 중앙값 창 / 정규화 오차 한계 / 지속 시간 |
| no-odom 운동 피드백 | `motion_response_min_speed_mps` | `0.03 m/s` | 이보다 느리면 접촉·벽 정체로 보고 재추정 |
|  | `motion_response_probe_extension_s` / `motion_response_probe_max_extension_s` | `0.30 s` / `1.20 s` | 느린 probe leg의 추가 시간 / 최대 추가 시간 |

`innovation_ratio`는 퍼센트가 아니라 **정규화된 Phase 잔차**다. ABBA가 예측한
전진 시의 거리 변화보다 실제 변화가 작거나 거리 증가 신호가 지속되면 양수가 커진다.
한 개의 직진 Phase 스칼라만으로 좌·우 오차를 직접 계산할 수는 없으므로, 한계 초과 시
중립 후 X/Y ABBA를 다시 수행해 관측 가능한 방향을 새로 구한다.

상태 확인:

```bash
ros2 topic echo /pinger_homing/status
```

기본 odometry 모드에서는 `odometry_fresh`, `sample_count`,
`estimated_source_world`, `source_locked`, `rms_residual_m`를 확인한다.
`audio_bootstrap_active=true`이면 약신호 탐색은 수행 중이지만 아직 유효 Phase delta가
없다는 뜻이다. `phase_measurement_fresh=true`가 된 뒤에만 source fit과 접근이 가능하다.
로컬라이제이션을 쓸 수 없는 비상 비교 운용에서만
`navigation_mode:=no_odom_phase`를 주며, 그때는 `no_odom_phase.innovation_ratio`,
`innovation_reestimate_count`, `motion_response`를 본다.

### 튜닝 원칙

- 첫 실물 odometry 운용은 `probe_pwm_delta:=20`, `approach_pwm_delta:=25`를 유지한다.
  `/odometry/filtered`에서 실제 변위와 `sample_count` 증가를 확인한 뒤에만 PWM을 조금씩 올린다.
  이 값은 1500에서의 raw 차이가 아니라 Pixhawk deadzone 바깥의 유효 차이다.
- 아래 `motion_response`/`innovation` 튜닝은 `navigation_mode:=no_odom_phase` 전용이다.
  전진 중 실제로 멀어지는데 재추정이 늦으면 `innovation_limit`을 낮추거나
  `innovation_hold_s`를 줄인다. 잡음으로 너무 자주 재추정하면 반대로 올린다.
- `reestimate_policy:=fixed approach_duration_s:=...`는 과거 고정 주기 비교용이며,
  실물 기본 운용에서는 사용하지 않는다.

### MuJoCo와 실물 계약

MuJoCo도 실물과 같은 `probe_pwm_delta:=20`, `approach_pwm_delta:=25`, RC deadzone,
`use_sim_time:=false`, `audio_input_latency_s:=0.0`을 사용한다. 외부 Phase estimator가
unstamped PCM과 odometry를 모두 수신 시각으로 결합하므로, 시뮬도 동일한 wall-time
계약을 따른다. 시뮬에서만 큰 PWM이나 별도 latency를 주어 성공시키는 튜닝은 허용하지
않는다.

## test-tank Phase/SNR 핑거 호밍

```bash
ros2 run auv_pinger_homing start_pinger_homing_test_tank.sh \
  mode:=ALT_HOLD estimator_mode:=phase \
  rc_output_topic:=/mavros/rc/override \
  auto_select_top:=false dry_run:=false
```

시작 후 10초 동안 19--22 kHz를 스캔해 후보를 최대 5개 표시한다. 같은 터미널에 후보 번호
(`1`~`5`) 또는 주파수(Hz)를 입력하면 2-D probe, yaw 정렬, 전진 호밍이 시작된다.
`estimator_mode:=snr`로 SNR 모드를 선택할 수 있다.

비전 미션 FSM을 다시 사용할 때는 `archive/kmu26_vision_mission_fsm`을 별도 ROS
작업공간으로 옮겨 독립적으로 빌드한다.
