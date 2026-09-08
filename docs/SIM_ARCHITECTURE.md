# 현재 시뮬레이터 아키텍처

이 문서는 `uuv_mujoco/current`와 `rospkg/src`의 실제 코드 상태를 기준으로 한
개발자용 개요다. 배포판의 과거 기본값이 아니라 현재 작업공간의 기본값을 기록한다.

## 1. 실행 계층

| 계층 | 주요 파일 | 책임 |
| --- | --- | --- |
| 루트 진입점 | `run_control_gui.sh` | ROS 환경과 로컬 colcon overlay를 로드하고 GUI 선택 |
| GUI | `gui/web_control_gui.py`, `gui/uuv_control_gui.py` | 프로세스 시작/종료, 조종, 영상, 상태 표시 |
| 통합 런처 | `start_sitl_mujoco_mj311.sh` | 기존 프로세스 정리, SITL 시작, 포트 배정, MuJoCo 시작 |
| SITL 런처 | `start_ardusub_sitl_mj311.sh` | ArduSub와 MAVProxy/direct MAVLink 출력 구성 |
| 런타임 | `run_uuv_mujoco.py` | 모델 로드, physics loop, viewer, bridge 수명주기 |
| 물리 | `sim/physics`, `sim/runtime` | 강체·유체·추력기·케이블·부표·collector 동역학 |
| 통신 | `bridge/`, `sim/transport/` | JSON SITL, MAVLink, ROS 2, 센서와 카메라 발행 |

활성 경로는 항상 `uuv_mujoco/current`다. `uuv_mujoco/v2.2`는 과거 호환
디렉터리이며 직접 실행 대상으로 사용하지 않는다. 현재 파일시스템에서 `current`는
심볼릭 링크가 아니라 독립 디렉터리이고 `v2.2`와 내용도 다르므로, 예전
`uuv_mujoco/CURRENT.md`의 alias 설명을 현재 사실로 간주하면 안 된다.

## 2. 폐루프 데이터 흐름

1. MuJoCo가 IMU, 압력, DVL 등 센서 상태를 ArduSub SITL JSON 입력으로 보낸다.
2. ArduSub가 자세/심도 제어를 계산하고 servo PWM을 JSON UDP로 반환한다.
3. MuJoCo가 PWM을 T200 추력 곡선으로 변환해 6-DOF body wrench를 적용한다.
4. 같은 상태가 ROS 2 센서 토픽과 선택적 MAVROS 호환 토픽으로 발행된다.
5. 운영자나 임무 제어기의 RC override는 ArduSub를 거쳐 다시 plant에 반영된다.

기본 포트 계약은 다음과 같다.

| 포트 | 용도 |
| --- | --- |
| UDP 9002 | SITL servo JSON 출력; instance에 따라 조정 가능 |
| UDP 14550 | QGroundControl MAVLink |
| UDP 14551 | 외부 MAVROS |
| UDP 14660 | MuJoCo의 ArduSub telemetry/servo 관측 |
| UDP 14661 | 저지연 명령 MAVLink |
| TCP 5760+ | SITL instance MAVLink, 사용 중이면 10 단위로 증가 |

엄격한 실제 패키지 호환 모드의 명령 경로는 다음 하나다.

```text
/mavros/rc/override -> external mavros_node -> ArduSub -> UDP 9002 -> MuJoCo
```

이 모드에서 `/cmd_vel`은 plant를 직접 움직이는 우회 제어 경로가 아니다.
자세한 topic owner는 `docs/contracts/REAL_STACK_PARITY.md`를 기준으로 한다.

## 3. 현재 기본 성능 계약

`gui/sim_stack_env_defaults.py` 기준 balanced 프로필:

| 항목 | 기본값 |
| --- | --- |
| SITL sensor loop | 100 Hz |
| thruster loop | 100 Hz |
| MAVLink servo | 30 Hz |
| MuJoCo timestep | 0.005 s |
| viewer | 1280x720 @ 30 FPS |
| GUI camera stream | 640x360 @ 4 Hz |
| course buoy update | 10 Hz |
| course buoy CSV | off |

카메라는 960x540 @ 10 Hz 및 1280x720 @ 5/10/20/30 Hz 프로필을 제공하지만
CPU-only YOLO와 폐루프 안정성 때문에 balanced가 기본이다. high 프로필의
timestep은 0.002초이며, course-buoy 장면에서는 timestep guard가 활성화된다.

## 4. 물리 모델

- 6-DOF 강체 운동과 중력/부력
- Fossen 계열 added mass, damping, current-relative 유체력
- 실측 PWM-force 곡선을 사용하는 T200 추력기
- 수면, 압력, DVL ray/range와 IMU 센서 모델
- 물리 케이블, 자석 weld, 부표 buoyancy/drag/contact
- 전면 collector의 충돌 geometry, soft weld slot, 역진 배출과 score-zone release

### DIST3 collector guide

코드에서 DIST 계열의 “guide”에 해당하는 기능은
`sim/runtime/course_buoy_runtime.py`의 collector net spring-damper다.

상태는 `FREE -> NETTING -> NETTED -> RELEASED` 순서다.

- 부표가 이미 collector 입구 window에 들어오고 수면 높이 gate를 만족해야
  `NETTING`이 된다. 멀리 있는 부표를 끌어오지 않는다.
- 각 부표에 최대 13개의 내부 slot 중 하나를 배정한다.
- `F = k(target - position) + c(vehicle_velocity - buoy_velocity)`를 적용한다.
- 기본 `k=4.0 N/m`, `c=1.2 N·s/m`, 힘 제한은 `3.0 N`이다.
- slot 0.10m 이내의 물리 조건을 만족하면 soft weld와 전면 gate collision으로
  `NETTED` 상태를 확정한다.
- 차량이 -0.08m/s 이하로 후진하면 입구를 열고 자연스럽게 역배출한다.
- score zone과 mission phase gate를 동시에 만족할 때만 score release한다.

즉, 이는 장거리 흡착이나 위치 순간이동이 아니라 입구에 진입한 부표의 수치적
안착을 돕는 제한된 힘 모델이다.

## 5. ROS 2 표면

시뮬레이터 bridge의 대표 출력:

| 영역 | 토픽 |
| --- | --- |
| 관성/항법 | `/imu/data`, `/dvl/velocity`, `/dvl/odometry`, `/dvl/altitude` |
| 심도 | `/depth`, `/depth/pose`, `/bar30/pressure_pa` |
| 외부 위치 추정 | `/odometry/filtered` (`robot_localization` 소유; bridge 기본 미발행) |
| 시뮬레이터 위치 oracle | `/mujoco/ground_truth/pose`, `/sim/odom` (평가/진단 전용, 추정·제어 입력 금지) |
| 하이드로폰 | `/audio`, `/audio_info`, `/mujoco/hydrophone/status`, `/mujoco/hydrophone/direction` |
| Ping360 | `/ping360/image`, `/ping360/scan`, `/ping360/echo`, `/ping360/status` |
| 경기장 | `/mujoco/course_buoys/status`, `/collector/state` |
| MAVROS 호환 | `/mavros/state`, local pose/velocity, battery, pressure, RC in |

입력은 `/mavros/rc/override`, `/mavros/manual_control/send`, 선택적
`/mavros/setpoint_raw/local`, `/ping360/config`을 받는다. arm/mode/command
서비스도 MAVROS 이름으로 제공한다. strict mode에서는 이 호환 발행자 대신 실제
`mavros_node`와 실제 ROS 패키지가 해당 토픽을 소유한다.

plain `--ros2`에서도 MuJoCo exact state는 표준 추정기 토픽
`/odometry/filtered`를 소유하지 않는다. 기존 통합을 위한 unsafe legacy opt-in은
`uuv_mujoco/current/docs/contracts/ROS2_BRIDGE_SURFACE.md`에만 별도로 명시하며,
SLAM·제어·논문 지표에는 사용할 수 없다.

## 6. ROS 패키지 구성

현재 `rospkg/src`에서 발견되는 package.xml 기준 패키지는 11개다.

- 차량/메시지: `hit25_auv_ros2`, `hit25_auv_ros2_msg`
- GUI/임무: `kmu26_auv_web_gui`, `auv_buoy_vision_control`
- pinger: `kmu26_pinger_homing`
- 음향: `audio_capture`, `audio_common`, `audio_common_msgs`
- 센서/추정: `dvl_msgs`, `ping360_sonar_msgs`, `robot_localization`

루트 `kmu26_mission_fsm` 링크는 현재 존재하지 않는
`rospkg/src/kmu26_control_packages/kmu26_vision_mission_fsm`을 가리킨다.
배포 문서에는 12개 패키지라고 남아 있으므로 다음 배포 전에 링크를 복구하거나
문서/패키징 계약을 11개 구조로 갱신해야 한다.

## 7. 검증 계층

`uuv_mujoco/current/tools`의 검사는 세 단계로 나뉜다.

1. 정적 계약: import, 설정 키, 토픽, command wiring 확인
2. headless 물리 계약: scene 로드 후 힘, 접촉, collector 상태 확인
3. live 통합: SITL/MAVROS/GUI를 실행해 heartbeat, RC 및 센서 freshness 확인

PR에서 우선 실행할 저비용 검사는 루트 README의 “빠른 검증” 목록이다.
`check_external_fsm_mavros_contract.py`와 `run_buoy_viewer_acceptance.py`는 실행
중인 스택이나 그래픽/시간이 필요하므로 별도 integration job으로 두는 편이 좋다.

## 8. 확인된 정리 과제

- `uuv_mujoco/CURRENT.md`의 symlink/2026-06-07 설명과 실제 독립 `current`
  디렉터리 상태가 다르다.
- `RUNTIME_VERSION.json` 일부 note는 dist3를 가리키지만 루트 배포 버전은 dist4다.
- 루트 배포 가이드의 0.008초/720p@30 기본값은 dist4 artifact에는 맞지만 현재
  개발 runtime 기본값(0.005초/640x360@4)과 다르다.
- `rospkg/src`에 독립 Git 저장소가 중첩되어 있으며 일부는 수정 상태다.
- 모델, 배포물, ArduPilot, 로그를 일반 Git object로 추가하면 저장소가 지나치게
  커지므로 `.gitignore`와 Git LFS 정책을 지켜야 한다.
