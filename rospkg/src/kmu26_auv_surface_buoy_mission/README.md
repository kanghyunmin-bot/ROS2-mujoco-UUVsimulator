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
- `/homing/sim_odom` (`nav_msgs/Odometry`): 위치와 자세
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

단독 실행:

```bash
ros2 launch kmu26_auv_surface_buoy_mission surface_buoy_mission.launch.py
```

통합 대회 launch는 기존 `competition_a_lane_mission.launch.py`가 이 패키지의 노드를
실행합니다. 두 제어 노드는 `/mission/surface_start`와
`/mission/surface_complete`로 handoff하며 동시에 RC를 소유하지 않습니다.
