# AntiGravity UUV Integration Workspace

MuJoCo + ArduSub(SITL) + ROS2 + QGroundControl을 한 워크스페이스에서 통합 실행하기 위한 프로젝트입니다.  
목표는 `MuJoCo 수중 시뮬레이션`의 센서/제어를 `ArduSub(Pixhawk4 가상)`과 `ROS2 패키지`에 연결해 실제 운용 흐름처럼 테스트하는 것입니다.

## 1. 폴더 구조

```text
antigravity/
├── kmu_hit25_ros2_ws/                 # ROS2 워크스페이스 (hit25_auv, hit25_interpreter)
│   ├── scripts/run_ardusub_json_sitl.sh
│   └── src/hit25_auv
├── mujoco/uuv_mujoco/v2.2/            # MuJoCo 시뮬레이터 + ROS2 bridge + competition scene
│   ├── launch_competition_sim.sh
│   ├── run_urdf_full.py
│   └── scripts/ros2_to_qgc_video.py
└── QGroundControl-x86_64.AppImage     # QGC 실행 파일
```

## 2. 요구 환경

- Ubuntu 22.04
- ROS 2 Humble
- Python 3.10+
- QGroundControl AppImage
- (권장) joystick

## 3. 초기 설치

### 3.1 ROS2 + 의존성 + SITL 준비

```bash
cd ~/antigravity/kmu_hit25_ros2_ws
./scripts/setup_ubuntu_humble.sh --with-sitl
./scripts/build_sitl.sh
```

### 3.2 ROS2 패키지 빌드

```bash
cd ~/antigravity/kmu_hit25_ros2_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
```

## 4. 전체 스택 실행 (권장 순서)

터미널 1: ArduSub JSON SITL

```bash
cd ~/antigravity
./kmu_hit25_ros2_ws/scripts/run_ardusub_json_sitl.sh --frame vectored
```
기본 동작은 `--wipe`가 기본 적용되어 시작됩니다. (필요 시 `--no-wipe` 사용)

터미널 2: MuJoCo competition map + SITL bridge

```bash
cd ~/antigravity/mujoco/uuv_mujoco/v2.2
./launch_competition_sim.sh --sitl --images
```

터미널 3: ROS2 제어 패키지

```bash
cd ~/antigravity/kmu_hit25_ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch hit25_auv auv_start.launch.py \
  use_mavros:=true use_dvl:=true use_odom2mavros:=false use_dronecan_battery:=false
```

터미널 4(선택): QGC 비디오 스트림(UDP 5600)

```bash
cd ~/antigravity/mujoco/uuv_mujoco/v2.2
source /opt/ros/humble/setup.bash
python3 scripts/ros2_to_qgc_video.py \
  --topic /stereo/left/image_raw --host 127.0.0.1 --port 5600 \
  --fps 15 --bitrate-kbps 2000 --width 640 --height 360
```

터미널 5 (선택): GUI 미확보 환경에서 모션 점검용 영상 저장

```bash
source /opt/ros/humble/setup.bash
cd ~/antigravity
python3 scripts/capture_mujoco_video.py \
  --topic /stereo/left/image_raw \
  --duration 30 \
  --fps 15 \
  --width 640 \
  --height 360 \
  --output /tmp/mujoco_preview.mp4
```

- 기본은 녹화 후 자동 삭제됩니다. 꼭 확인해야 하면 `--keep` 추가.
- 영상 확인(동일 머신):

```bash
ffplay /tmp/mujoco_preview.mp4
```

삭제:

```bash
rm -f /tmp/mujoco_preview.mp4
```

## 5. QGroundControl 설정

### 5.1 텔레메트리 링크

- `Application Settings -> Comm Links`
- `UDP` 링크 추가
- 포트: `14550`
- 기존 `TCP 127.0.0.1:5760`만 활성화되어 있으면 연결 실패할 수 있음
- `--dual-qgc-link` 옵션으로 UDP/TCP 동시 노출:
  `./kmu_hit25_ros2_ws/scripts/run_ardusub_json_sitl.sh --dual-qgc-link`
- TCP만 쓰는 환경이면:
  `./kmu_hit25_ros2_ws/scripts/run_ardusub_json_sitl.sh --qgc-link tcpclient --qgc-port 5760`

### 5.2 영상

- `Application Settings -> General -> Video`
- Source: `UDP h.264 Video Stream`
- UDP Port: `5600`

참고:
- QGC는 `QGroundControl-x86_64.AppImage` 실행이 필요합니다. 앱을 띄운 뒤 위 링크를 켭니다.
- 연결이 안 되면 QGC를 재시작하고, 차량 자동탐지/통신 링크를 수동으로 삭제 후 재등록해 보십시오.

## 6. 동작 확인 체크리스트

ROS 토픽 확인:

```bash
source /opt/ros/humble/setup.bash
ros2 topic list | grep -E '/imu/data|/dvl/odometry|/mujoco/ground_truth/pose|/mavros/rc/override'
```

포트 확인:

```bash
ss -uapn | rg '14550|9002|9003|5600'
```

정상 상태 예:

- MuJoCo 로그에 `SITL servo endpoint discovered`
- `/imu/data`, `/dvl/odometry`가 주기적으로 발행
- QGC에 vehicle 연결 표시

## 7. 자주 발생하는 문제

### `[sub.parm -1] No JSON sensor message received`

원인:
- MuJoCo를 `--sitl` 없이 실행
- SITL/MuJoCo 포트 불일치 (`9002` / `9003`)
- 실행 순서 문제
- 기존 `eeprom.bin` 잔존으로 `FS_PILOT_INPUT`이 `2`(disarm)로 고정

해결:
- 섹션 4의 순서대로 재실행
- `run_ardusub_json_sitl.sh`와 `launch_competition_sim.sh --sitl` 조합 유지
- `Lost manual control`가 반복되면 `--wipe`로 재시작 후 QGC를 다시 연결

### QGC 연결 안 됨

원인:
- QGC가 TCP 링크(5760)만 사용

해결:
- UDP `14550` 링크 수동 추가
- 또는 `--dual-qgc-link`로 ArduPilot을 다시 시작해 동시에 열기

### 카메라 안 보임

원인:
- MuJoCo `--images` 미사용
- `ros2_to_qgc_video.py` 미실행

해결:
- 섹션 4의 터미널 2, 4 같이 실행

## 8. 참고 문서

- `mujoco/uuv_mujoco/v2.2/README.md`
- `mujoco/uuv_mujoco/ROS2_BRIDGE.md`
- `kmu_hit25_ros2_ws/docs/PORTING_GUIDE.md`

## 9. 스크린샷/영상 기반 현장 점검 가이드

- MuJoCo GUI 창이 보이지 않아도 시뮬레이션 영상은 위 스크립트로 확인할 수 있습니다.
- 영상이 안 잡히면 아래 순서로 확인:
  - `launch_competition_sim.sh --images --sitl` 옵션 실행
  - `ros2 topic hz /stereo/left/image_raw`
  - `python3 scripts/capture_mujoco_video.py ... --keep`
- QGC와 통신이 안 되면:
  - MAVLink 포트 5760/14550 순서를 맞게 열었는지 확인
  - `ros2_to_qgc_video.py`가 `/stereo/left/image_raw`를 받는지 로그로 확인
  - `ss -uapn | rg 14550|5600`로 UDP 바인딩 확인

## 10. 주의사항 (중첩 Git 저장소)

현재 워크스페이스에는 아래 경로가 중첩 git 저장소 형태로 존재합니다.

- `kmu_hit25_ros2_ws/ardupilot`
- `kmu_hit25_ros2_ws/tools/Micro-XRCE-DDS-Gen`
- `mujoco`

외부에서 클론했을 때 이 경로 내용이 기대와 다르면, 해당 폴더 상태를 먼저 확인한 뒤(커밋/원격), 필요한 경우 별도 동기화가 필요합니다.
