# AUV 부표 비전/제어 브리지

노트북에서 YOLO로 buoy/stick을 검출하고, AUV NUC의 미션 상태머신이 MAVROS RC override로 제어하는 ROS 2 패키지입니다.

```text
AUV NUC
  /camera/camera/color/image_raw/compressed
      |
      | 테더 / ROS 2 DDS
      v
노트북 PC
  yolo_buoy_detector (Python / Ultralytics / .pt)
  /vision/buoy_bbox  (+ /vision/yolo/annotated/compressed)
      |
      | 테더 / ROS 2 DDS
      v
AUV NUC
  mission_state_machine_node (C++)
  /mavros/rc/override
  /mission/rc_command   (모니터링용 동일 PWM)
  /mission/state
```

> `bbox_controller_node`는 단순 추적용 레거시 노드입니다. 기본 launch는 사용하지 않으며, `mission_state_machine_node`와 동시에 실행하면 RC override가 충돌합니다.

## 미션 흐름

<!-- [ACOUSTIC-VISION HANDSHAKE] near zone 요청 → confirm → grant, 또는 외부 강제 grant. -->
```text
IDLE -> TARGET_CONFIRM -> WAIT_CONTROL_GRANT -> SEARCH/APPROACH_BUOY -> ALIGN_STICK
     -> INSERT_FORK -> DETACH -> BACKOFF -> VERIFY_RELEASE
     -> (성공/포기 시 SEARCH 반복)
     -> SEARCH 타임아웃 시 AREA_VERIFY -> ASCEND -> COMPLETE

강제 인계(벽 경계 / acoustic timeout): 외부에서 grant를 주면 IDLE에서 SEARCH
이미 가까운 buoy가 확정된 상태에서 grant를 받으면 바로 APPROACH
```

수심 유실 또는 최대수심 초과 시 `FAILSAFE`로 전환하고 제어 채널을 `RELEASE`합니다.
이 노드는 하강 단계(`DIVE`)를 수행하지 않습니다. Acoustic/상위 제어기가 near zone과 적정 수심까지 유도한 뒤 Vision에 제어권을 넘기는 구조입니다.

| 상태 | 동작 요약 |
|------|-----------|
| IDLE | Acoustic 요청 대기. RC/RELEASE를 발행하지 않음. 외부 grant면 바로 SEARCH |
| TARGET_CONFIRM | YOLO 가까운 buoy를 4 frame/0.3 s 확정. RC 미발행 |
| WAIT_CONTROL_GRANT | Acoustic의 RC 종료 승인 대기. RC 미발행 |
| SEARCH | `work_depth_m` 유지 + 제자리 yaw 회전. forward는 중립. YOLO의 가까운 buoy를 4 frame/0.3 s 확인하면 APPROACH. 40초면 AREA_VERIFY |
| APPROACH | buoy 중앙 정렬, 전진 면적 P(1700→1560), throttle=비전+수심 블렌딩. 면적≥30%+stick → ALIGN |
| ALIGN | stick을 포크 목표점으로 정렬(+수심 블렌딩). deadband 0.7초 → INSERT |
| INSERT/DETACH/BACKOFF | 시간 기반 전진/후진 펄스 + 작업수심 유지 |
| VERIFY | buoy 미검출이면 임시 성공→SEARCH, 남으면 재시도 |
| AREA_VERIFY | 최종 재탐색 후 없으면 ASCEND |
| ASCEND | `surface_depth_m`까지 부상 → COMPLETE |

### 다중 부표 선택

SEARCH/AREA_VERIFY에서 buoy가 여러 개면:

1. 박스 **면적**이 큰 것
2. 비슷하면 **confidence**(박스 확률)가 높은 것
3. 그것도 비슷하면 이미지 **오른쪽**

APPROACH/ALIGN 중에는 고른 부표를 유지합니다.

## 토픽

### 카메라

```text
/camera/camera/color/image_raw/compressed
sensor_msgs/msg/CompressedImage
```

### YOLO 검출 (`/vision/buoy_bbox`)

`std_msgs/msg/Float32MultiArray` — 검출 1개당 10개 값:

```text
[stamp_sec, detected, class_id, confidence,
 center_x, center_y, width, height, image_width, image_height]
```

- `detected >= 0.5` 이면 유효
- `publish_per_class:=true`(기본)일 때 클래스마다 메시지 1개씩 발행 (buoy + stick 동시 갱신)

### 수심 / Acoustic-Vision 핸드셰이크 / 상태

```text
/auv/depth                 std_msgs/msg/Float64  (양의 하방[m], 선택)
/depth/pose                geometry_msgs/msg/PoseWithCovarianceStamped (선택)
/homing/vision_search_active   std_msgs/msg/Bool  (Acoustic -> Vision)
/vision/target_confirmed       std_msgs/msg/Bool  (Vision -> Acoustic, 타깃 확정)
/homing/vision_control_granted std_msgs/msg/Bool  (Acoustic -> Vision)
/mission/state             std_msgs/msg/String   (latched)
```

`/depth/pose` 기본 변환: `depth = -pose.position.z`  
(`depth_pose_scale`, `depth_pose_offset_m`으로 조정. 빈 `depth_pose_topic:=` 으로 Pose 입력 비활성)

### 제어 출력

```text
/mavros/rc/override        mavros_msgs/msg/OverrideRCIn
/mission/rc_command        동일 내용 (웹 모니터용, 다른 override와 분리)
```

ArduSub 기본 채널 (이 노드가 건드리는 것):

```text
Ch3 Throttle/vertical
Ch4 Yaw
Ch5 Forward
```

미사용 채널은 `CHAN_NOCHANGE=65535`. PWM은 `1300~1700`으로 클램프.

## 설치

```bash
mkdir -p ~/auv_ws/src
cp -r auv_buoy_vision_control ~/auv_ws/src/
cd ~/auv_ws
rosdep install --from-paths src -y --ignore-src
colcon build --symlink-install
source install/setup.bash
```

노트북 YOLO 의존성:

```bash
# 권장
pip install -r src/auv_buoy_vision_control/requirements-yolo.txt

# 또는 환경 점검/설치 스크립트
ros2 run auv_buoy_vision_control check_yolo_env.py
# setup_laptop_yolo_env.sh
```

## 노트북: YOLO 실행

`model_path`는 필수에 가깝습니다 (launch 기본은 빈 문자열, 노드 기본 예시는 `/home/auv/models/buoy.pt`).

```bash
ros2 launch auv_buoy_vision_control laptop_yolo_detection.launch.py \
  model_path:=/path/to/model.pt \
  image_topic:=/camera/camera/color/image_raw/compressed \
  bbox_topic:=/vision/buoy_bbox \
  device:=cuda:0 \
  publish_per_class:=true
```

클래스 필터 예:

```bash
# class id
target_class_id:=0 target_class_name:=

# 또는 class name (모델 names에 존재해야 함)
target_class_name:=buoy
```

미션이 buoy+stick을 쓰므로 **`target_class_id`로 한 클래스만 막지 마세요.**  
(둘 다 필요하면 `target_class_id:=-1`, `target_class_name:=` 유지)

### Competition map 수중 핑거: 분리 시험 순서

수중 핑거 부표와 수면 부표는 같은 실행에서 처리하지 않는다. 아래 launch는
`best.pt`의 `classes={0: buoy, 1: stick}`을 모두 사용하며 수중 핑거만 대상으로 한다.

1. YOLO 검출 전용(제어 노드 없음):

```bash
ros2 launch auv_buoy_vision_control underwater_pinger_yolo.launch.py
```

2. 비전 강제 grant 단독 제어:

```bash
ros2 launch auv_buoy_vision_control \
  underwater_pinger_vision_control.launch.py force_control_grant:=true
```

3. 하이드로폰/비전 분리 통합(위 YOLO launch는 별도 터미널에서 계속 실행):

```bash
ros2 launch auv_buoy_vision_control underwater_pinger_handoff.launch.py \
  start_immediately:=true
```

통합 launch는 하이드로폰의
`vision_search_active -> target_confirmed -> acoustic neutral RC ->
vision_control_granted` 순서를 사용한다. 비전 FSM은
`/mujoco/course_buoys/status`에서 정확히
`course_buoy_pinger_white_1_float`의 `has_magnet=true, detached=true`를 확인한
경우에만 `/mission/success=true`와 `COMPLETE`를 발행한다.

Competition map 수중 기본값은 `work_depth_m=8.65`,
`max_depth_m=10.5`이다. 기존 `run_vision_mission.py`의
`/mission/control_enable` 방식은 ver2.7에서 사용하지 않는다.

Annotated JPEG:

```bash
ros2 topic hz /vision/yolo/annotated/compressed
```

## AUV NUC: 상태머신 실행

사전 점검: QGC/MAVLink에서 RC 채널·PWM 방향과 수심 부호/단위를 확인한다.
`work_depth_m`은 하강 목표가 아니라 Vision 제어 중 유지할 수심입니다. Acoustic/상위 제어가 넘겨준 실제 작업 수심과 맞춰 설정하세요.

```bash
ros2 launch auv_buoy_vision_control auv_bbox_controller.launch.py \
  bbox_topic:=/vision/buoy_bbox \
  depth_pose_topic:=/depth/pose \
  depth_pose_scale:=-1.0 \
  rc_override_topic:=/mavros/rc/override \
  work_depth_m:=9.5 \
  surface_depth_m:=0.4 \
  max_depth_m:=10.5 \
  buoy_class_id:=0 \
  stick_class_id:=1
```

일반 launch의 기본 수심도 competition map 기준
`work_depth_m=9.5`, `surface_depth_m=0.4`, `max_depth_m=10.5`로 맞췄다.
수면 부표 시험은 `underwater_pinger_*` launch에 포함하지 않으며 별도 프로파일에서
명시적인 얕은 수심 값을 전달해야 한다.

핸드셰이크 상태 확인:

```bash
ros2 topic echo /mission/state
ros2 topic echo /homing/vision_search_active
ros2 topic echo /vision/target_confirmed
ros2 topic echo /homing/vision_control_granted
```

## 주요 파라미터 (노드/launch 기본값)

### 탐색 / 접근

| 파라미터 | 기본 | 설명 |
|----------|------|------|
| `search_yaw_pwm` | 1600 | SEARCH/AREA_VERIFY yaw 고정 PWM |
| `search_timeout_sec` | 40 | SEARCH 타임아웃 → AREA_VERIFY |
| `target_confirm_hits` | 4 | TARGET_CONFIRM/SEARCH에서 가까운 동일 buoy 확인에 필요한 연속 hit |
| `target_confirm_sec` | 0.3 | 위 연속 hit 이후 추가 유지 시간 |
| `min_detection_hits` | 5 | AREA_VERIFY 등 후속 재탐색의 연속 hit |
| `approach_area_ratio` | 0.30 | ALIGN 진입·전진 P 스케일용 면적비 |
| `approach_forward_pwm` (`forward_pwm` launch 인자) | 1700 | APPROACH 전진 최대 |
| `approach_forward_min_pwm` | 1560 | APPROACH 전진 최소 (가까울 때) |
| `approach_vision_throttle_weight` | 0.4 | APPROACH/ALIGN throttle 비전 비중 (나머지는 수심 P) |
| `max_yaw_delta` | 180 | 비전 yaw 최대 편차 (±) |
| `fork_target_x` / `fork_target_y` | 0.30 / 0.70 | ALIGN 포크 목표 (정규화, 왼쪽 아래 3사분면 쪽) |

### 수심 / 필터

| 파라미터 | 기본 | 설명 |
|----------|------|------|
| `work_depth_m` | 9.5 | Vision 제어 중 유지할 작업 수심. 하강 동작은 수행하지 않음 |
| `depth_kp_pwm_per_m` | 45 | 수심 P게인 (PWM/m) |
| `max_depth_delta_pwm` | 160 | 수심 PWM 편차 클램프 |
| `buoyancy_hold_delta_pwm` | 40 | 양성 부력 보정 (목표에서도 하강 바이어스). ASCEND에서는 꺼짐 |
| `lpf_tau_sec` | 0.3 | 수심·bbox 중심/크기 1차 LPF 시상수 (0=off) |

### 포크 펄스

실기 전 낮은 추력부터 조정:

```bash
insert_pwm:=1560 insert_duration_sec:=0.8
detach_pwm:=1620 detach_duration_sec:=0.3
backoff_pwm:=1420 backoff_duration_sec:=0.5
```

### PWM / 방향

```bash
min_pwm:=1300 max_pwm:=1700
yaw_invert:=false
vertical_positive_is_up:=true
```

`CHAN_RELEASE=0`, `CHAN_NOCHANGE=65535`는 PWM 클램프 대상이 아닙니다.

## 확인 명령

```bash
ros2 topic hz /camera/camera/color/image_raw/compressed
ros2 topic echo /vision/buoy_bbox
ros2 topic echo /mission/state
ros2 topic echo /mavros/rc/override
ros2 topic echo /mission/rc_command
```

`/mavros/rc/in`, `/mavros/rc/out`은 FCU가 인식한 RC 상태입니다. ROS에서 넣으려면 `/mavros/rc/override`를 사용합니다.

## 네트워크

```bash
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
```

테더에서 discovery가 안 되면 DDS static discovery 또는 동일 subnet/multicast를 확인하세요.

## 알려진 제한

- `VERIFY_RELEASE`는 buoy가 일정 시간 안 보이는 것을 **임시 성공**으로 봅니다. 실기체 최종 성공 판정용은 아닙니다.
- APPROACH에 별도 타임아웃은 없습니다 (면적·stick 조건 또는 buoy 유실로만 빠져나옴).
- Web GUI로 미션을 띄우면 GUI 입력값이 launch 기본보다 우선할 수 있으니, GUI 기본값이 이 README/노드와 같은지 확인하세요.
