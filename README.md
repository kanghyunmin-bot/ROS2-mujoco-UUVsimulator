# KMU26 UUV MuJoCo + ArduSub Simulator

**최신 소스 기준: `main` · `2026.09.08-pre-vla`**

VLA 적용 전 기본 ROV 시뮬레이터 기준선이다. FCU 갱신 주기, 센서 전달,
기본 MAVROS 제어 입력과 DVL→FCU 경로를 정리했다.
[변경 내용·검증 범위](docs/releases/2026.09.08-pre-vla.md)를 먼저 확인한다.
이 버전은 소스 기준선이며 이전 ZIP/DEB 설치 파일을 새로 빌드한 릴리스가 아니다.

Ubuntu 22.04, ROS 2 Humble, MuJoCo and ArduSub SITL을 결합한 수중 로봇
시뮬레이션 작업공간이다. 활성 시뮬레이터는 `uuv_mujoco/current`이며, 웹/Tk
GUI, MAVROS 호환 제어면, 카메라·DVL·압력·하이드로폰·Ping360 센서, 부표 수집
물리를 한 스택에서 제공한다.

> 현재 폴더는 배포 산출물과 실험 로그가 함께 있던 개발 작업공간에서 정리한
> 소스 트리다. Git에는 소스와 문서만 올리고 ArduPilot, QGroundControl, 빌드
> 결과와 로그는 포함하지 않는다.

<p align="center">
  <img src="docs/assets/simulator-overview.png" width="100%" alt="MuJoCo top view and YOLO buoy tracking view">
</p>

왼쪽은 MuJoCo 수조 전체 시점, 오른쪽은 AUV 전방 카메라와 CPU 기반 YOLO 부표
추적 화면이다.

## 동작 화면

<p align="center">
  <img src="docs/assets/simulator-demo.gif" width="800" alt="UUV simulator and YOLO buoy tracking demo">
</p>

실제 시뮬레이션에서 AUV가 이동하는 동안 카메라 영상, 부표 검출 결과와 top-view
상태가 함께 갱신되는 모습이다.

## 빠른 실행

이미 설치가 끝난 작업공간에서는 다음이 기본 실행 경로다.

```bash
source /opt/ros/humble/setup.bash
source ./.uuv_mujoco_env.sh
./run_control_gui.sh --web \
  --sim-preset research_pool_distributed \
  --host 127.0.0.1 \
  --port 8878
```

브라우저에서 <http://127.0.0.1:8878/>을 열고 GUI에서 시뮬레이션 스택을
시작한다. 환경 선택 메뉴에서 기존 타원체 물리, 분산 물리, 파랑 포함 분산
물리를 바꿀 수 있다. Tk GUI도 같은 프리셋을 사용하며
`./run_control_gui.sh --tk --sim-preset research_pool_distributed`로 실행한다.

GUI 없이 SITL과 MuJoCo를 직접 실행하려면:

```bash
cd uuv_mujoco/current
./start_sitl_mujoco_mj311.sh -- --headless
```

ROS 2 브리지 없이 물리 런타임만 확인하려면 `--no-ros2`를 추가한다. 실제
MAVROS/차량 패키지와 동일한 통합면은 `--ros2-real-pkg-compat` 모드를 사용한다.

## 시스템 구성

```text
Web/Tk GUI
    |
    +-- process manager ---- ArduSub SITL
    |                            |  JSON sensor/servo UDP
    |                            v
    +---------------------- MuJoCo runtime
                                 |
                                 +-- ROS 2 sensor/ground-truth bridge
                                 +-- camera / DVL / depth / hydrophone / Ping360
                                 +-- course buoy / cable / collector physics

Controller or operator
    -> /mavros/rc/override
    -> MAVROS or compatibility bridge
    -> ArduSub
    -> thruster PWM
    -> MuJoCo
```

기본 폐루프 런타임은 100 Hz 센서/추력 루프와 0.005초 MuJoCo timestep을
사용한다. 웹 GUI의 기본 카메라 프로필은 CPU 여유를 위해 640x360 @ 4 Hz이며,
720p 프로필은 GUI에서 선택할 수 있다.

상세한 프로세스, 포트, 토픽 소유권과 부표 수집 상태 머신은
[시뮬레이터 아키텍처](docs/SIM_ARCHITECTURE.md)에 정리되어 있다.
수영장 SLAM 장면과 선택형 분포 유체물리의 범위·실행법·보정 절차는
[Research Pool 수중 물리 가이드](uuv_mujoco/current/docs/architecture/RESEARCH_POOL_HYDRODYNAMICS.md)를 참고한다.

## 주요 디렉터리

| 경로 | 역할 |
| --- | --- |
| `uuv_mujoco/current/` | 활성 MuJoCo 런타임, 브리지, GUI, 장면과 검증 도구 |
| `uuv_mujoco/current/sim/` | 물리, 런타임, 전송 계층과 계약 모듈 |
| `uuv_mujoco/current/bridge/` | ROS 2, MAVLink, 센서 및 영상 브리지 |
| `uuv_mujoco/current/scenes/` | 수조와 경기장 MuJoCo XML |
| `uuv_mujoco/current/tools/` | 정적·동적 계약 검사와 재현 도구 |
| `rospkg/src/` | 실제 차량과 공유하는 ROS 2 패키지 소스 |
| `docs/contracts/` | 실제 스택과 시뮬레이터의 인터페이스 계약 |
| `dist2/ubuntu22.04/` | Ubuntu 배포 패키징 스크립트 |

## 핵심 ROS 인터페이스

- 상태/제어: `/mavros/state`, `/mavros/rc/in`, `/mavros/rc/override`
- raw/derived 항법 센서: `/imu/data`, `/dvl/odometry`, `/depth/pose`
- 외부 추정기 출력: `/odometry/filtered` (`robot_localization` 소유;
  MuJoCo bridge는 기본 미발행)
- 카메라: `/camera/camera/color/image_raw/compressed`
- 음향: `/audio`, `/audio_info`, `/mujoco/hydrophone/direction`
- 소나: `/ping360/scan`, `/ping360/image`, `/ping360/config`
- 임무/평가: `/collector/state`, `/mujoco/course_buoys/status`,
  `/mujoco/ground_truth/pose`

`/mujoco/ground_truth/pose`와 `/sim/odom`은 검증/진단 전용 ground-truth
oracle이며 실제 차량용 추정기나 제어기의 입력으로 사용하지 않는다.
RC를 내는 임무 노드는 동시에 실행하지 않고 mux의 단일 소유권 계약을 지켜야 한다.

## ROS 패키지 빌드

```bash
cd rospkg
./build_safe.sh --cmake-args -DBUILD_TESTING=OFF
source install/setup.bash
```

`build_safe.sh`는 메모리 부족을 피하기 위해 컴파일 병렬도를 제한한다.

## 빠른 검증

```bash
python3 uuv_mujoco/current/tools/check_gui_start_contract.py
python3 uuv_mujoco/current/tools/check_sim_runtime_smooth_contract.py
python3 uuv_mujoco/current/tools/check_competition_course_scene.py
python3 uuv_mujoco/current/tools/check_buoy_collector_capture.py
python3 uuv_mujoco/current/tools/check_dist_rc_override_path.py
```

실행 중인 외부 MAVROS 경로까지 검사하려면:

```bash
python3 uuv_mujoco/current/tools/check_external_fsm_mavros_contract.py
```

## Git에 게시하기 전에

대용량 YOLO 모델은 Git LFS 대상으로 지정되어 있다. 또한 `rospkg/src` 아래에는
여러 upstream 저장소의 `.git` 메타데이터와 로컬 수정이 남아 있으므로, 최초
저장소 생성 전에 monorepo 또는 submodule 방식을 결정해야 한다. 안전한 게시
순서와 현재 주의사항은 [Git 게시 가이드](docs/GIT_PUBLISHING.md)를 따른다.

배포 ZIP/DEB 사용자를 위한 설치 안내는 [README_FIRST.md](README_FIRST.md),
실제 차량 ROS 패키지 설명은 [rospkg/README.md](rospkg/README.md)를 참고한다.
