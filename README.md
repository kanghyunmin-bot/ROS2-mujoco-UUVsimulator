# AntiGravity UUV Integration Workspace

MuJoCo + ArduSub(SITL) + ROS2 + QGroundControl 통합 테스트 워크스페이스입니다.

핵심 목표:
- MuJoCo 수중 물리/센서 시뮬레이션
- ArduSub(Pixhawk4 SITL) 제어 루프 연동
- ROS2 패키지(MAVROS 포함) 통신 검증
- QGC 수동 조작 + 영상 확인

## 1. 구성

```text
antigravity/
├── kmu_hit25_ros2_ws/                 # ROS2 워크스페이스 + ArduSub 실행 스크립트
├── mujoco/uuv_mujoco/v2.2/            # MuJoCo 시뮬레이터 + SITL/ROS2 브리지
├── scripts/launch_stack_auto.sh       # 원커맨드 전체 실행(비디오 포함)
├── scripts/stack_reset.sh             # 관련 프로세스 정리
└── QGroundControl-x86_64.AppImage
```

## 2. 요구 환경

- Ubuntu 22.04
- ROS 2 Humble
- Python 3.10+
- QGroundControl AppImage

## 3. 초기 설치

```bash
cd ~/antigravity/kmu_hit25_ros2_ws
./scripts/setup_ubuntu_humble.sh --with-sitl
./scripts/build_sitl.sh

source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
```

## 4. 원커맨드 실행 (권장)

아래 1줄로 ArduSub + MuJoCo + (선택)QGC + QGC 비디오 스트림까지 같이 실행합니다.

```bash
cd ~/antigravity
./scripts/launch_stack_auto.sh --with-qgc --frame vectored_6dof
```

기본 동작:
- ArduSub: `--dual-qgc-link` + `--wipe` + SITL 안정화 파라미터 적용
- MuJoCo: `--sitl --images --force-clean`
- 비디오 브리지: `/stereo/left/image_raw -> udp://127.0.0.1:5600`

자주 쓰는 옵션:

```bash
./scripts/launch_stack_auto.sh --headless
./scripts/launch_stack_auto.sh --sitl-debug
./scripts/launch_stack_auto.sh --no-video
./scripts/launch_stack_auto.sh --no-images
./scripts/launch_stack_auto.sh --no-force-clean
```

종료:

```bash
./scripts/stack_reset.sh --with-qgc-stop
```

## 5. 수동 실행 (디버깅용)

터미널 1: ArduSub

```bash
cd ~/antigravity
./kmu_hit25_ros2_ws/scripts/run_ardusub_json_sitl.sh --frame vectored_6dof --force-clean
```

터미널 2: MuJoCo + SITL 브리지

```bash
cd ~/antigravity/mujoco/uuv_mujoco/v2.2
./launch_competition_sim.sh --sitl --images --force-clean
```

부력이 과하면 즉시 튜닝:

```bash
./launch_competition_sim.sh --sitl --images --force-clean --buoyancy-scale 0.98
```

터미널 3(선택): ROS2 제어 패키지

```bash
cd ~/antigravity/kmu_hit25_ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch hit25_auv auv_start.launch.py \
  use_mavros:=true use_dvl:=true use_odom2mavros:=false use_dronecan_battery:=false
```

터미널 4(선택): 비디오 브리지 수동 실행

```bash
cd ~/antigravity/mujoco/uuv_mujoco/v2.2
source /opt/ros/humble/setup.bash
python3 scripts/ros2_to_qgc_video.py --topic /stereo/left/image_raw --host 127.0.0.1 --port 5600
```

## 6. QGroundControl 설정

Comm Link:
- `Application Settings -> Comm Links`
- `UDP` 링크 추가, 포트 `14550`

Video:
- `Application Settings -> General -> Video`
- Source: `UDP h.264 Video Stream`
- Port: `5600`

추가 확인:
- 모드 `Manual`
- 상태 `Armed`
- Virtual Joystick 활성

## 7. 빠른 점검

포트 점검:

```bash
ss -uapn | grep -E '9002|9003|14550|5600'
ss -tapn | grep -E '5760'
```

수동 입력 유입 점검:

```bash
cd ~/antigravity
python3 scripts/check_sitl_manual_input.py --listen 14550 --timeout 20 --strict
```

정상 기준:
- `armed_seen=True`
- `rc_active_seen=True` 또는 `servo_active_seen=True`

## 8. 자주 발생하는 문제

### `[sub.parm -1] No JSON sensor message received`
- 실행 순서/포트 불일치가 원인인 경우가 대부분
- `stack_reset.sh` 후 다시 원커맨드 실행

### `SITL servo stream is neutral (all near 1500)`
- Arm 안 됨, Virtual Joystick 꺼짐, Manual 모드 아님, 중복 프로세스 충돌
- `./scripts/stack_reset.sh --with-qgc-stop` 후 재실행

### QGC 연결 안 됨
- UDP 14550 링크 미설정
- 기존 TCP 링크만 활성화된 경우 UDP 링크 추가

### QGC 비디오가 `WAITING FOR VIDEO`
- `--no-video`로 실행했거나 `/stereo/left/image_raw` 미발행
- `log/auto_stack_*/video_bridge.log`에서 `stream alive` 확인

## 9. 참고 문서

- `mujoco/uuv_mujoco/v2.2/README.md`
- `mujoco/uuv_mujoco/ROS2_BRIDGE.md`
- `kmu_hit25_ros2_ws/docs/PORTING_GUIDE.md`

## 10. 저장소 주의사항

현재 워크스페이스는 중첩 저장소(서브모듈 포함)를 사용합니다.

- `mujoco`
- `kmu_hit25_ros2_ws/ardupilot`
- `kmu_hit25_ros2_ws/tools/Micro-XRCE-DDS-Gen`

클론 후 동기화 시 각 하위 저장소의 브랜치/원격 상태를 함께 확인하세요.
