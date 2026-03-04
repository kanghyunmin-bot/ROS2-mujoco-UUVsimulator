# AntiGravity UUV Integration Workspace

MuJoCo + ArduSub(SITL) + ROS2 + QGroundControl(QGC) 통합 시뮬레이션 워크스페이스입니다.

이 저장소는 `루트 저장소 + 서브모듈(mujoco, ardupilot)` 구조입니다.

## 0) 먼저 설치해야 하는 외부 구성요소

이 프로젝트는 코드만으로 완전 실행되지 않습니다. 아래 항목을 먼저 준비해야 합니다.

1. ArduSub SITL 실행 환경
- 포함 위치: `kmu_hit25_ros2_ws/ardupilot` (서브모듈)
- 필수: 빌드/의존성 설치

2. QGroundControl
- 별도 설치 필요 (AppImage 권장)
- 권장: Stable 버전 사용 (Daily 비권장)

3. ROS2 Humble (Ubuntu 22.04 기준)
- MuJoCo ROS2 bridge, MAVROS 연동에 사용

## 1) 핵심 폴더/파일 정의

### 루트 오케스트레이션

- `scripts/start_full_stack_direct.sh`
  - 표준 전체 실행 스크립트
  - ArduSub를 `sim_vehicle.py --console --map` 형태로 실행
  - 이후 MuJoCo + Video Bridge + QGC를 순차 기동
- `scripts/launch_stack_auto.sh`
  - 자동 실행 스크립트 (옵션형)
  - `run_ardusub_json_sitl.sh`와 연동하여 readiness 체크 수행
- `scripts/stack_reset.sh`
  - 스택 관련 프로세스 정리, 포트 충돌 해소
- `scripts/check_sitl_manual_input.py`
  - heartbeat/manual 입력 검증 스크립트

### ArduSub/SITL 쪽

- `kmu_hit25_ros2_ws/scripts/run_ardusub_json_sitl.sh`
  - ArduSub SITL 실행 래퍼
  - 포트/heartbeat/readiness 점검 포함

### MuJoCo 시뮬레이터 쪽

- `mujoco/uuv_mujoco/v2.2/launch_competition_sim.sh`
  - v2.2 실행 엔트리
  - SITL direct-thruster/MAVLink 입력 옵션 처리
- `mujoco/uuv_mujoco/v2.2/run_urdf_full.py`
  - 메인 런타임 루프(물리, 쓰러스터, 센서, SITL/ROS2 연동)
- `mujoco/uuv_mujoco/v2.2/ros2_bridge.py`
  - SITL JSON 송신 + MAVLink SERVO 수신 + ROS2 publish/subscribe
  - IMU/DVL/Depth/카메라 브리지 핵심
- `mujoco/uuv_mujoco/v2.2/competition_scene.xml`
  - 로봇/센서/쓰러스터/환경 씬 정의
- `mujoco/uuv_mujoco/v2.2/sim_profiles.json`
  - 물리 프로파일(sim_real/sim_fast 등)
- `mujoco/uuv_mujoco/v2.2/thruster_tune.json`
  - 쓰러스터 튜닝 파라미터

## 2) 초기 설치 (처음 1회)

```bash
cd ~/antigravity/kmu_hit25_ros2_ws
./scripts/setup_ubuntu_humble.sh --with-sitl
./scripts/build_sitl.sh

source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
```

QGroundControl(AppImage) 준비:

```bash
cd ~/antigravity
chmod +x QGroundControl*.AppImage
```

## 3) 표준 실행 (권장)

```bash
cd ~/antigravity
./scripts/stack_reset.sh --with-qgc-stop
./scripts/start_full_stack_direct.sh
```

이 경로는 내부적으로 다음 흐름을 사용합니다.

1. ArduSub: `sim_vehicle.py -L RATBeach --console --map -v ArduSub -f vectored_6dof --model JSON`
2. 출력: `14550(QGC)`, `14551(MAVROS)`, `14660(MAVLink SERVO)`
3. MuJoCo: `launch_competition_sim.sh --sitl --images --sitl-servo-source mavlink`
4. QGC 자동 실행(옵션)

## 4) 수동 분리 실행 (디버깅 표준)

터미널 1: ArduSub SITL

```bash
cd ~/antigravity/kmu_hit25_ros2_ws/ardupilot
python3 Tools/autotest/sim_vehicle.py -L RATBeach --console --map \
  -v ArduSub -f vectored_6dof --model JSON \
  --out=udp:127.0.0.1:14550 \
  --out=udp:127.0.0.1:14551 \
  --out=udp:127.0.0.1:14660 \
  --out=tcpin:0.0.0.0:5773 \
  -P MAV_GCS_SYSID=1 -P MAV_GCS_SYSID_HI=255
```

터미널 2: MuJoCo

```bash
cd ~/antigravity/mujoco/uuv_mujoco/v2.2
./launch_competition_sim.sh --sitl --images \
  --sitl-servo-source mavlink \
  --sitl-mavlink-endpoint udpin:0.0.0.0:14660 \
  --sitl-servo-map "yaw_rr,yaw_lr,yaw_rf,yaw_lf,ver_lf,ver_rf,ver_lr,ver_rr" \
  --sitl-servo-signs=1,1,1,1,1,1,1,1 \
  --force-clean
```

터미널 3: QGC

```bash
cd ~/antigravity
./QGroundControl*.AppImage
```

## 5) 포트/통신 맵

- `14550/udp`: QGC 링크
- `14551/udp`: MAVROS/보조 GCS 링크
- `14660/udp`: SERVO_OUTPUT_RAW (ArduSub -> MuJoCo)
- `9002/udp`: SITL JSON servo endpoint(MuJoCo 수신)
- `9003/udp`: SITL JSON sensor endpoint(ArduSub 수신)
- `5600/udp`: QGC 비디오 스트림
- `5773/tcp`: QGC TCP 링크(선택)

## 6) QGroundControl 기본 설정

1. Firmware: `ArduPilot`
2. Vehicle: `Submarine`
3. Comm Link:
- 기본은 UDP `14550` 사용
- 필요 시 TCP `127.0.0.1:5773` 추가 가능
4. Video:
- UDP h264
- Port `5600`

## 7) 빠른 상태 점검

포트 점검:

```bash
ss -lntup | grep -E '14550|14551|14660|9002|9003|5600|5773'
```

heartbeat-only 점검:

```bash
cd ~/antigravity
python3 scripts/check_sitl_manual_input.py --mode udp --listen 14551 --heartbeat-only --timeout 20
```

수동 입력 점검(strict):

```bash
cd ~/antigravity
python3 scripts/check_sitl_manual_input.py --mode udp --listen 14550 --timeout 20 --strict
```

## 8) 자주 발생하는 문제와 원인

1. QGC Connected인데 조종 안 됨
- ArduSub 미Arm
- Virtual Joystick 미활성
- MANUAL/STABILIZE 모드 미설정
- SERVO stream neutral(1500 고정)

2. launch 스크립트가 기존 MuJoCo runtime 때문에 종료됨
- 기존 `run_urdf_full.py` 프로세스 잔존
- `./scripts/stack_reset.sh --with-qgc-stop` 후 재실행

3. 튜닝 탭/파라미터 일부 누락 경고
- QGC Daily + Dev firmware 조합에서 자주 발생
- Stable QGC 사용 권장

## 9) 저장소 구조 주의사항

`mujoco`는 서브모듈입니다.

- 루트 커밋과 `mujoco` 커밋은 별도로 관리됩니다.
- `mujoco` 코드 변경 후에는:
  1. `mujoco` 저장소에서 먼저 커밋/푸시
  2. 루트 저장소에서 서브모듈 포인터 커밋/푸시

## 10) 관련 문서

- `mujoco/uuv_mujoco/v2.2/README.md`
- `mujoco/uuv_mujoco/ROS2_BRIDGE.md`
- `kmu_hit25_ros2_ws/docs/PORTING_GUIDE.md`
