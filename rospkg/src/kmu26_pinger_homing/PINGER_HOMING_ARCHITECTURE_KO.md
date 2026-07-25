# 핑거 호밍 RC 제어기 배포 구조

## 코드 경계

배포본은 기존 하이드로폰 신호처리 알고리즘과 차량 RC 제어기를 분리한다.

```text
팀 포크 kmu26_auv_hydrophone
  /audio
    -> audio_phase_estimator
    -> /audio_phase_estimator/delta_range_m
    -> /audio_phase_estimator/iq_magnitude
    -> /homing/direction

src/pinger_homing/pinger_homing_controller.cpp
  위 추정 결과 + /odometry/filtered + /mavros/state
    -> 능동 위치 추정 및 접근 RC
    -> /mavros/rc/override (단독 직접 발행)
```

하이드로폰 의존성은 저장소 최상위 `hydrophone.repos`에 팀 포크 URL과 검증 커밋
`64ccbadad903b6d5b40f641996ce6fe91fd1f69d`로 고정한다. `vcs import`는 이를
`src/kmu26_auv_hydrophone` sibling Git 저장소로 가져온다. 하이드로폰 추정 알고리즘을
FSM 또는 `auv_pinger_homing` 안으로 복사하거나 수정하지 않는다.

## 기존 알고리즘에 추가한 차량 제어 코드

### 1. Canonical C++ 능동 탐색

`pinger_homing_controller.cpp`가 다음 순서로 RC 탐색을 수행한다.

```text
WAIT_VEHICLE -> PROBE <-> REPROBE -> ALIGN <-> APPROACH -> CONTACT -> COMPLETE
```

한 위치의 단일 하이드로폰 측정만으로는 3차원 방향을 결정하기 어려우므로, 전진·횡이동·수직
탐색 구간에서 odometry 위치와 음향 거리 변화를 함께 수집한다. 각 구간 사이에는 중립 RC를
넣어 관성 운동의 영향을 줄인다.

### 2. 움직이는 단일 센서 위치 추정

`pinger_homing_math.hpp`가 다음 정보를 결합한다.

- 위상 변화 기반 상대 거리 변화
- coherent-IQ 크기 기반 절대 거리 보조값
- 각 측정 시점의 차량 3차원 위치
- 주파수/클럭 편차를 흡수하는 거리 변화율 bias

표본 수, 잔차, 조건수가 기준을 통과할 때만 추정 위치를 고정한다. 품질이 부족하면 접근하지
않고 반대 방향으로 `REPROBE`한다.

### 3. RC 생성과 안전 처리

- 핑거 월드 위치 벡터를 차량 body FLU 좌표로 변환
- yaw 오차에 비례 제어와 yaw-rate 감쇠 적용
- 큰 방위 오차에서는 전진 제한, 정렬 후 거리별 단계 감속
- `tank_max_depth_m` 하나로 차량 최대 깊이와 수직 탐색 방향 자동 계산
- 바닥 제한에 가까우면 하강 명령 차단 및 상승 복구
- 실물 live 모드에서 odometry, audio 또는 arm 상태가 유효하지 않으면 모든 RC 채널 release
- dry-run에서는 탐색/추정/요청 명령을 그대로 계산하되 항상 모든 RC 채널 release
- 상태와 명령을 `/pinger_homing/status` JSON으로 공개

실물 표준 launch에서는 별도 mux를 시작하지 않는다. C++ 제어기가 최종
`/mavros/rc/override`를 직접 소유하며, 실행 중 다른 RC publisher는 정지해야 한다.

이 패키지는 `CollectorState`, YOLO, 부표 포획 상태, MuJoCo ground truth를 구독하지 않는다.
비전 부표 제어와 미션 FSM은 형제 패키지 `kmu26_vision_mission_fsm`에만 둔다.

## 빌드 및 실행

```bash
vcs import src < src/auv_pinger_homing/hydrophone.repos
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select \
  audio_common_msgs audio_capture auv_pinger_homing
source install/setup.bash
```

실물 오디오 스트림을 켠 뒤 핑거 호밍만 실행한다. 아래 launch는 먼저 19--22 kHz를
10초 동안 스캔하고 후보 1~5개를 출력한다. 터미널에서 후보 번호(또는 Hz)를 고르면,
그 주파수를 원본 phase estimator의 시작 파라미터로 넣은 뒤 canonical C++ 제어기를
실행한다.

```bash
ros2 launch auv_pinger_homing pinger_homing_real_interactive.launch.py \
  dry_run:=true \
  use_audio_capture:=false \
  use_hydrophone_estimator:=true \
  tank_max_depth_m:=11.0
```

현장 스캔·선택은 위 interactive launch를 사용한다. 자동 아밍하지 않고, `/mavros/state`가
armed이며 required mode일 때만 컨트롤러가 `/mavros/rc/override`에 유효한 명령을 직접 보낸다. 실물 IQ-거리 보정 전에는
`amplitude_range_constant:=0.0`, `success_range_m:=0.0`을 유지한다.

Python 구현은 비교/아카이브용으로 남기며, 설치·수조·실물 launch의 제어기는 canonical
C++ 구현 하나를 사용한다. test-tank의 `no_odom_phase` 프로파일은 X/Y ABBA probe만
수행하고 ALT_HOLD에 Z를 맡긴 뒤, 짧은 전진마다 다시 probe하여 오래된 bearing을 따르지 않는다.
