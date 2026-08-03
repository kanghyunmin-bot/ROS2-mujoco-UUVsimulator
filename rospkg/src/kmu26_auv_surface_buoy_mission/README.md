# AUV Surface Buoy Mission

수면 부표 레인 탐색·포획과 가점존 배출을 전담하는 ROS 2 패키지입니다.
수중 레인 제어 패키지와 실행 파일을 분리하고 ROS 2 토픽으로만 임무 상태와
제어권을 주고받습니다. 수면 전용 수조 좌표 변환, 깊이 제어 및 세로 레인
계획 코드도 이 패키지가 소유하므로 다른 제어 패키지의 C++ 구현에 링크하지 않습니다.

## 통신 계약

주요 입력:

- `/mission/surface_start` (`std_msgs/String`): surface cycle 시작 계약
- `/mission/arena_config` (`std_msgs/Float64MultiArray`): 대회장 크기·offset·안전 여유
- `/start_frame` (`geometry_msgs/PoseStamped`): arena 원점과 yaw
- `/sim/odom` (`nav_msgs/Odometry`): 수면 단독 시험의 위치와 자세
- `/depth/pose` (`geometry_msgs/PoseWithCovarianceStamped`): 현재 수심
- `/vision/surface/front/buoy_bbox`, `/vision/surface/top/buoy_bbox`
  (`std_msgs/Float32MultiArray`): 정면·상단 YOLO 결과
- `/collector/state` (`auv_msg/CollectorState`): 실제 포획·배출 확인

주요 출력:

- `/mavros/rc/override` (`mavros_msgs/OverrideRCIn`): surface cycle 동안의 RC 제어
- `/mission/surface_state` (`std_msgs/String`): 현재 상태
- `/mission/surface_complete` (`std_msgs/String`): cycle 완료 및 제어권 반환
- `/mission/score_release` (`std_msgs/String`): 시뮬레이터 score gate 계약
- `/mission/surface_remaining_count`, `/mission/bonus_deposit_count`
  (`std_msgs/UInt32`): 누적 현황
- `/vision/surface/front/enabled`, `/vision/surface/top/enabled`
  (`std_msgs/Bool`, transient local): 상태별 정면·상단 YOLO 추론 게이트
- `/vision/surface/front/ready`, `/vision/surface/top/ready`
  (`std_msgs/Bool`, transient local): 모델 로딩과 detector 초기화 완료 신호

## 수면 미션만 시험하는 순서

수중 레인·하이드로폰 패키지는 실행하지 않아도 됩니다. 다만 차량을 실제로 움직이는
SITL, MuJoCo, ROS 2 브리지는 먼저 켜져 있어야 합니다.

터미널 1에서 뷰어가 포함된 시뮬레이터를 실행합니다.

```bash
cd /home/khm/ROS2-mujoco-UUVsimulator/sim/current
./start_sitl_mujoco_mj311.sh --ros2 -- --ros2-images
```

GUI에서 모드를 `STABILIZE`로 바꾸고 `Arm`을 누릅니다. 그 다음 터미널 2에서:

```bash
cd /home/khm/ROS2-mujoco-UUVsimulator
source /opt/ros/humble/setup.bash
source rospkg/install/setup.bash
ros2 launch kmu26_auv_surface_buoy_mission surface_buoy_mission.launch.py
```

정면 bbox 중심 허용오차는 기본 `0.07`(화면 폭의 7%)입니다. 더 정밀하게 5%로
맞추려면 다음처럼 실행합니다.

```bash
ros2 launch kmu26_auv_surface_buoy_mission surface_buoy_mission.launch.py \
  surface_align_deadband_x:=0.05
```

이 launch 하나가 다음을 함께 실행합니다.

- `laptop_yolo_detection.launch.py` 기반 전방·상단 detector 2개
- `surface_buoy_mission_node`
- 단독 시험용 arena/start 계약 발행 노드

두 detector 프로세스와 모델은 계속 유지하지만 실제 추론은 상태별로 제한합니다.
레인 탐색·정렬은 정면만, 포획은 정면과 상단을 함께, 초기 망 확인과 배출은
상단만 사용합니다. 이동·수심 전환·대기 중에는 두 detector 모두 JPEG decode와
YOLO 추론을 건너뜁니다.

단독 시험용 노드는 `/sim/odom`, `/depth/pose`, 두 bbox 토픽, 그리고
`/mavros/state`의 armed `STABILIZE`를 모두 확인한 뒤에만 surface cycle을
시작합니다. YOLO Python 환경은 기본적으로
`~/.venvs/uuv_mujoco_desktop`을 자동 선택합니다. 다른 환경을 쓰려면
`UUV_YOLO_VENV=/절대/경로`를 지정합니다.

이미 detector를 따로 실행했다면 `include_detectors:=false`, 통합 미션에서 외부
handoff를 받을 때는 `standalone_test:=false`를 사용합니다.

통합 대회 launch는 기존 `competition_a_lane_mission.launch.py`가 이 패키지의 노드를
실행합니다. 두 제어 노드는 `/mission/surface_start`와
`/mission/surface_complete`로 handoff하며 동시에 RC를 소유하지 않습니다.
