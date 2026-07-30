# AUV Lane Vision Control

Hydrophone 제어기로부터 제어권을 인계받은 뒤, 초기 부표를 처리하고 수조 좌표계에
자동 생성한 레인을 따라가며 남은 부표를 탐색·제거하는 ROS 2 패키지입니다.

이 패키지는 기존 `auv_buoy_vision_control`과 동일한 Vision/RC 토픽을 사용하는
교체형 제어기입니다. 두 제어기를 동시에 실행하면 `/mavros/rc/override`가
충돌하므로 반드시 하나만 실행해야 합니다.

## 전제 조건

- ROS 2 Humble과 MAVROS가 설치되어 있어야 합니다.
- `/start_frame`은 `hydrophone_ctrl`의 시작 좌표 규약으로 발행되어야 합니다.
- `/odometry/filtered`에서 유효한 위치와 yaw를 받을 수 있어야 합니다.
- 기존 YOLO 검출기가 `/vision/buoy_bbox`에 buoy bbox를 발행해야 합니다.
- 실제 RC 채널 방향과 PWM은 낮은 추력에서 먼저 확인해야 합니다.
- 이 제어기는 `COMPLETE`에서 자동 부상하지 않고 RC override를 해제합니다.

## 전체 동작

```text
IDLE
  -> TARGET_CONFIRM
  -> WAIT_CONTROL_GRANT
  -> (확정 buoy 있음) FIND_AND_ALIGN
  -> (확정 buoy 없음) INITIAL_SCAN_360
       -> 가장 큰 bbox 후보 선택
       -> TURN_TO_INITIAL_TARGET
       -> FIND_AND_ALIGN
  -> INSERT_FORK
  -> GO_BACK
  -> VERIFY_RELEASE
       -> 성공: 가장 가까운 레인 끝점으로 이동
       -> 실패: INSERT_FORK_HARD -> GO_BACK -> 가장 가까운 레인 끝점
  -> MOVE_TO_LANE_START
  -> LANE_FOLLOWING_WITH_SEARCH
       -> buoy 발견: FIND_AND_ALIGN
       -> 제거/포기 후 RETURN_TO_ACTIVE_LANE
       -> 기존 레인 주행 재개
  -> 모든 레인 완료
  -> COMPLETE
```

odom·수심 유실, 최대수심 초과 또는 비상정지 시 `FAILSAFE`로 전환합니다.

## Hydrophone handshake

```text
/homing/vision_search_active
    Hydrophone -> Vision 탐색/확인 요청

/vision/target_confirmed
    Vision -> Hydrophone 안정적인 buoy 검출 알림

/homing/vision_control_granted
    Hydrophone -> Vision RC 제어권 승인
```

승인 전에는 RC override를 발행하지 않습니다.

Hydrophone의 수조 경계 또는 timeout으로 buoy 확정 없이 제어권을 받으면 제자리에서
360도 회전합니다. 회전 중 관측한 buoy 중 bbox 면적이 가장 큰 대상을 가장 가까운
대상으로 선택합니다. 후보가 없으면 기다리지 않고 가장 가까운 레인 끝점으로
이동합니다.

## 수조 좌표계

좌표 변환은 `hydrophone_ctrl`과 동일합니다.

1. `/start_frame`의 위치와 yaw를 arena 원점과 `+X` 방향으로 사용합니다.
2. `/odometry/filtered`의 위치와 yaw를 arena 좌표로 변환합니다.
3. `arena_start_corner=bottom_left`이면 수조 내부 너비 방향은 `-Y`입니다.
4. 레인은 arena `X`축과 평행하게 생성됩니다.

대회장 기본 설정:

```text
arena_length_m       = 15.0
arena_width_m        = 16.0
arena_offset_x_m     = -0.3
arena_offset_y_m     = 0.3
arena_safety_margin_m = 0.5
arena_start_corner   = bottom_left
```

## 레인 자동 생성

```text
usable_width  = arena_width - 2 * arena_safety_margin
coverage_width = 2 * lane_search_offset
lane_count    = ceil(usable_width / coverage_width)
actual_spacing = usable_width / lane_count
```

기본 `lane_search_offset_m=2.0`일 때:

```text
usable_width  = 15.0 m
lane_count    = 4
actual_spacing = 3.75 m
```

초기 부표 처리가 끝나면 모든 미완료 레인의 끝점 중 현재 위치에서 가장 가까운
끝점을 선택합니다. 한 레인을 완료하면 다시 가장 가까운 미완료 레인 끝점을
선택합니다.

## 부표 접근과 판정

buoy bbox 중심을 전체 이미지 정규화 좌표 `(0.25, 0.50)`, 즉 왼쪽 화면 절반의
중앙에 정렬합니다. 정렬 오차가 deadband 안에 있을 때만 전진합니다.

`CAPABLE` 판정은 다음 비율로 계산합니다.

```text
left_fill_ratio =
  (buoy bbox와 왼쪽 화면 절반의 교차 면적)
  / (왼쪽 화면 절반의 전체 면적)
```

기본적으로 `left_fill_ratio >= 0.70`이고 정렬까지 완료되면 포크를 삽입합니다.

접근 허용 범위:

- 초기 부표: Hydrophone 제어권 인계 위치로부터 반경 `2.0 m`
- 레인 탐색 부표: 현재 활성 레인으로부터 수직 거리 `2.0 m`
- arena 안전 경계 밖으로 이동할 수 없음

허용 범위 끝까지 접근해도 70% 조건을 만족하지 못하면 `INCAPABLE`로 판단합니다.
활성 레인의 가장 가까운 지점으로 복귀한 뒤, 기존 진행 방향으로 `2.0 m` 이동할
때까지 bbox를 무시해 같은 부표를 반복 추적하지 않습니다.

## 삽입과 확인

기본 PWM과 지속 시간:

| 동작 | PWM | 시간 |
|---|---:|---:|
| 일반 삽입 | 1560 | 0.8 s |
| 강한 삽입 | 1620 | 0.8 s |
| 후퇴 | 1420 | 0.5 s |

일반 삽입 후 후퇴하고 buoy가 사라졌는지 확인합니다. buoy가 남아 있으면 강한
삽입을 한 번 수행하고 후퇴한 뒤 레인으로 복귀합니다.

## 수심 제어

수심 입력 우선순위:

1. `/depth/pose` (`geometry_msgs/msg/PoseWithCovarianceStamped`)
2. `/auv/depth` (`std_msgs/msg/Float64`)

기본 Pose 변환:

```text
depth = -pose.position.z
```

`depth_pose_topic`을 빈 문자열로 설정하면 Pose 입력을 비활성화할 수 있습니다.

제어권 승인 순간의 현재 수심을 `mission_hold_depth`로 저장하고 전체 미션 동안
유지합니다.

- 레인 이동·회전·삽입·후퇴: 수심 PID만 사용
- `FIND_AND_ALIGN`: 비전 상하 오차 40% + 수심 PID 60%
- 양성부력 보상과 PID 출력 제한 적용

## ROS 토픽

### 입력

| 토픽 | 타입 | 설명 |
|---|---|---|
| `/vision/buoy_bbox` | `std_msgs/msg/Float32MultiArray` | YOLO buoy bbox |
| `/odometry/filtered` | `nav_msgs/msg/Odometry` | 위치와 yaw |
| `/start_frame` | `geometry_msgs/msg/PoseStamped` | arena 시작 좌표 |
| `/depth/pose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | 주 수심 입력 |
| `/auv/depth` | `std_msgs/msg/Float64` | 보조 수심 입력 |
| `/homing/vision_search_active` | `std_msgs/msg/Bool` | Vision 확인 요청 |
| `/homing/vision_control_granted` | `std_msgs/msg/Bool` | RC 제어권 승인 |
| `/mission/emergency_stop` | `std_msgs/msg/Bool` | 비상정지 |

### 출력

| 토픽 | 타입 | 설명 |
|---|---|---|
| `/vision/target_confirmed` | `std_msgs/msg/Bool` | 안정적인 buoy 확정 |
| `/mission/state` | `std_msgs/msg/String` | 현재 상태, `COMPLETE` 포함 |
| `/mavros/rc/override` | `mavros_msgs/msg/OverrideRCIn` | 실제 RC override |
| `/mission/rc_command` | `mavros_msgs/msg/OverrideRCIn` | 모니터링용 동일 RC |

### bbox 형식

검출 하나당 10개 값입니다.

```text
[stamp_sec, detected, class_id, confidence,
 center_x, center_y, width, height, image_width, image_height]
```

`detected >= 0.5`이고 `class_id == buoy_class_id`인 검출만 사용합니다.

## 주요 파라미터

| 파라미터 | 기본값 | 설명 |
|---|---:|---|
| `lane_search_offset_m` | 2.0 | 활성 레인 좌우 접근 한계와 레인 계산 기준 |
| `incapable_skip_distance_m` | 2.0 | 포기 후 bbox 무시 주행 거리 |
| `initial_search_radius_m` | 2.0 | 초기 인계 위치 기준 접근 반경 |
| `capable_left_fill_ratio` | 0.70 | CAPABLE bbox 점유율 |
| `align_target_x` | 0.25 | buoy 정렬 목표 X |
| `align_target_y` | 0.50 | buoy 정렬 목표 Y |
| `target_confirm_hits` | 4 | 연속 검출 횟수 |
| `target_confirm_sec` | 0.3 | 연속 검출 후 유지 시간 |
| `lane_forward_pwm` | 1700 | 레인/waypoint 전진 PWM |
| `approach_forward_pwm` | 1560 | 정렬 후 접근 PWM |
| `insert_fork_pwm` | 1560 | 일반 삽입 PWM |
| `insert_fork_hard_pwm` | 1620 | 강한 삽입 PWM |
| `go_back_pwm` | 1420 | 후퇴 PWM |
| `odometry_timeout_sec` | 0.5 | odom FAILSAFE 시간 |
| `depth_timeout_sec` | 1.0 | 수심 FAILSAFE 시간 |
| `max_depth_m` | 10.5 | 최대 허용 수심 |
| `approach_vision_throttle_weight` | 0.4 | 접근 중 비전 throttle 비중 |

모든 RC 출력은 기본적으로 `1300~1700` 범위로 제한됩니다.

## 빌드

```bash
cd ~/vision_control
colcon build \
  --base-paths auv_lane_vision_control \
  --packages-select auv_lane_vision_control
source install/setup.bash
```

## 실행

```bash
ros2 run auv_lane_vision_control lane_vision_controller_node
```

파라미터 변경 예:

```bash
ros2 run auv_lane_vision_control lane_vision_controller_node --ros-args \
  -p lane_search_offset_m:=2.0 \
  -p max_depth_m:=10.5 \
  -p buoy_class_id:=0
```

상태 확인:

```bash
ros2 topic echo /mission/state
ros2 topic echo /vision/target_confirmed
ros2 topic echo /mission/rc_command
```

## 안전 주의사항

- 실제 장비에서 실행하기 전에 `waypoint_yaw_invert`, `vision_yaw_invert`,
  `vertical_positive_is_up` 방향을 확인합니다.
- 처음에는 낮은 forward/insert PWM으로 시험합니다.
- `/start_frame`, odom, depth, bbox 수신 상태를 확인한 뒤 제어권을 승인합니다.
- `FAILSAFE`와 `COMPLETE`에서는 throttle/yaw/forward 채널에
  `CHAN_RELEASE`를 발행합니다.
