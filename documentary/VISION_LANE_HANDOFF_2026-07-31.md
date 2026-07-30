# 하이드로폰 → 레인 비전 임무 인계 (2026-07-31)

## 현재 목표

수중 핑거 부표를 하이드로폰으로 찾은 뒤 비전에 제어권을 넘기고, A 코스에서
정확히 4회 레인 스윕을 수행하면서 핑거 1개와 일반 수중 부표 7개를 물리적으로
분리한다. 수면 위 부표 처리는 이 시험과 분리한다.

## 확인된 기준 상태

- 작업공간: `/home/robot/uuv_sim_current`
- 활성 시뮬레이터: `sim/current`
- 경기 장면: `sim/current/scenes/tank_current_scene.xml`
- YOLO 모델: `sim/current/assets/yolo/best.pt`
- 모델 SHA-256:
  `bac1c909f9204d45b0235bf4f1838d9124aa33e234cc1dca6638acb90e0fab6a`
- YOLO 클래스: `{0: buoy, 1: stick}`
- 하이드로폰 단독 시험:
  기본 위치 → 중앙 이동 → 반경 1.3 m 원형 스캔 → SNR 호밍 → `SUCCESS`
- 하이드로폰 성공 위치: 대략 world `(-1.20, -8.79, -8.65)`
- 핑거 부표 위치: world `(0.035, -10.820, -8.535)`

## 포함된 구현

1. YOLO 검출 전용 launch
2. 비전 강제 grant 단독 제어 launch
3. `detached=true` 물리 분리 모니터
4. 다음 인계 계약의 독립 검증 도구
   - `vision_search_active`
   - `target_confirmed`
   - RC neutral
   - `vision_control_granted`
5. 핑거 처리 후 정확히 4개 A 레인을 만드는 경로 계획
6. 물리 분리만 성공으로 인정하는 레인 임무 acceptance 도구
7. 저신뢰 원거리 bbox 억제, 카메라 정보 보정, yaw dead-zone 보상
8. 레인 부표 접근 중 세로 bbox 추종을 끄고 임무 깊이를 유지하는 제어
9. 목표 처리 후 출발 지점으로 복귀하고 같은 목표를 잠시 억제하는 로직

## 시험 결과

- 핑거 부표는 반복 시험에서 물리 분리 성공을 확인했다.
- 한 장시간 시험은 총 8개 중 7개를 분리하고 4개 레인 중 3개를 완료했다.
  마지막 원거리 목표에서 yaw 명령이 실제 데드존보다 작아 진행이 멈췄고,
  이후 최소 yaw 보정을 80 PWM으로 올렸다.
- 최신 깊이 유지 시험에서는 핑거와
  `course_buoy_a_yellow_4_float`가 물리적으로 분리됐다.
- `yellow_4` 분리 이벤트는 첫 검증보다 늦게 도착했다. 제어기가 실패로 판단해
  재시도한 뒤 복귀 중 authoritative detach가 확인됐다. 따라서 포크 높이
  오프셋을 추가하기 전에 detach 검증 대기/지연 처리를 먼저 조정해야 한다.
- 최신 시험은 이 지연을 확인한 직후 수동 중단했으므로 최종 4/4 스윕과 8/8
  분리는 아직 완료 판정이 아니다.
- 대상 단위 시험:
  - 레인 계획 시험 2개 통과
  - YOLO 코스 필터 시험 6개 통과
- 전체 워크스페이스에는 `hit25_auv_ros2`의 과거 비관련 실패가 남아 있으므로
  대상 패키지 시험 결과와 전체 colcon 결과를 구분해야 한다.

## 다음 시험에서 먼저 볼 것

1. 물리 분리 검증 대기 시간이 MuJoCo 분리 이벤트 지연보다 충분한지 확인한다.
2. 현재 `lane_fork_depth_offset_m=0`을 유지한 채 첫 일반 부표 분리가 한 번의
   삽입으로 인식되는지 확인한다.
3. 성공하면 전체 4회 스윕을 중단 없이 실행해 `lanes=4/4`,
   `detached=8`을 acceptance JSON으로 확인한다.
4. 그 후에만 수면 위 부표 시험을 별도로 시작한다.

## 주요 실행 진입점

```bash
cd sim/current
./start_sitl_mujoco_mj311.sh --ros2-real-pkg-compat -- \
  --headless \
  --scene "$(pwd)/scenes/tank_current_scene.xml" \
  --fluid-model current \
  --initial-bar30-depth-m 8.65 \
  --ros2-sensor-hz 100 \
  --thruster-loop-hz 100 \
  --profile current \
  --initial-position-xy -0.822582 -9.410371 \
  --initial-rpy-rad 0 0 -1.024242
```

ROS/MAVROS와 임무 launch는 새 터미널에서 실행한다.

```bash
source /opt/ros/humble/setup.bash
source rospkg/install/setup.bash
ros2 launch auv rov_start.launch.py \
  use_sim_time:=true \
  fcu_url:=udp://0.0.0.0:14551@ \
  use_dvl:=false \
  use_joy2mavros:=false \
  use_battery_bridge:=false \
  use_odom2mavros:=false \
  publish_static_tf:=false \
  depth_zero_at_start:=false
```

```bash
source /opt/ros/humble/setup.bash
source rospkg/install/setup.bash
ros2 launch auv_lane_vision_control competition_a_lane_mission.launch.py \
  odometry_topic:=/sim/odom \
  show_preview:=false \
  device:=cpu
```

```bash
python3 sim/current/tools/run_lane_vision_acceptance.py \
  --confirm-timeout-sec 30 \
  --report sim/current/logs/lane_full_four_sweeps.json
```

실제 시작 전 FCU가 `STABILIZE`, `armed: true`인지 `/mavros/state`로 확인한다.

## 소스 스냅샷 기준

중첩 저장소의 `.git` 메타데이터는 상위 저장소에 넣지 않고, 현재 working tree를
일반 파일로 포함했다. 동기화 당시 기준 커밋은 다음과 같다.

| 경로 | 기준 커밋 |
| --- | --- |
| `rospkg/src/auv_dvl_a50` | `75be5820a976` |
| `rospkg/src/auv_dvl_a50_msg` | `5389084512b6` |
| `rospkg/src/dvl_msgs` | `555525ebc027` |
| `rospkg/src/kmu26_auv` | `7a5a7bade38d` |
| `rospkg/src/kmu26_auv_buoy_vision_control` | `10d7e2308295` |
| `rospkg/src/kmu26_auv_hydrophone` | `8d70362ea1ad` |
| `rospkg/src/kmu26_auv_msg` | `374043578098` |
| `rospkg/src/kmu26_auv_vision_control` | `702381fd1540` |
| `rospkg/src/kmu26_auv_web_gui` | `2d4ad38028e8` |
| `rospkg/src/kmu26_pinger_homing` | `c514f7f6d4f1` |
| `rospkg/src/robot_localization` | `8696ee5a9e4f` |

ArduPilot은 `2dd0bb7d4c85ac48437f139d66df648fc0e1d4ae`
(`ArduSub-4.1.2`)를 기준으로 하며,
`sim/ardupilot_patches/0001-current-althold-stabilize.patch`가 로컬 변경
67줄 추가/10줄 삭제를 보존한다. fresh installer가 이 패치를 자동 적용한다.

## 의도적으로 제외한 항목

- 모든 `build/`, `install/`, `log/`, `logs/`
- Python/CMake 캐시와 컴파일 산출물
- 중첩 `.git` 메타데이터
- `sim/current/generated` 모델 캐시
- 백업 복제본, 녹화 영상, 런타임 출력
- rosbag/DB3와 4.4 GB 분석 원본 데이터

이 항목들은 현재 소스와 임무 상태를 이어가는 데 필요하지 않으며 새 환경에서
다시 생성된다. YOLO 모델은 제외하지 않고 Git LFS로 보존한다.
