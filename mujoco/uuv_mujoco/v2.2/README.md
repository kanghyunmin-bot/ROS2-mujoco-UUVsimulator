# UUV MuJoCo Simulator v2.1

MuJoCo 기반 UUV 시뮬레이터 배포 폴더입니다.

- 모델 씬: `urdf_full_scene.xml`
- 메인 런너: `run_urdf_full.py`
- ROS2 브리지: `ros2_bridge.py`
- 버전: `VERSION.txt`

## v2.1 업데이트 요약

- Ubuntu 의존성 설치 스크립트 강화: `install_deps_ubuntu.sh`
- 문서 구조 확장:
  - `docs/PROJECT_TREE.md` (트리 + 파일별 상세)
  - `docs/CODE_GUIDE.md` (코드 원리/함수별 설명)
- 핵심 코드 주석/도큐스트링 정리:
  - `run_urdf_full.py`
  - `ros2_bridge.py`
- Docker 작업 경로/엔트리포인트를 v2.1 기준으로 정리

## 1) 빠른 시작

```bash
cd /home/khm/mujoco/uuv_mujoco/v2.1
./install_deps_ubuntu.sh
python3 run_urdf_full.py --scene urdf_full_scene.xml
```

## 2) 의존성 설치 스크립트

파일: `install_deps_ubuntu.sh`

기본 실행:

```bash
./install_deps_ubuntu.sh
```

옵션:

```bash
./install_deps_ubuntu.sh --no-venv
./install_deps_ubuntu.sh --with-ros2-dev
./install_deps_ubuntu.sh --python python3.10
```

설치 항목:

- apt: OpenGL/X11/MuJoCo 런타임 라이브러리
- pip: `requirements.txt` (`numpy`, `mujoco`)
- 옵션: ROS2 개발 유틸(colcon/rosdep)

## 3) 실행 방법

기본 실행:

```bash
python3 run_urdf_full.py --scene urdf_full_scene.xml
```

프로파일 실행:

```bash
python3 run_urdf_full.py --scene urdf_full_scene.xml --profile sim_real
python3 run_urdf_full.py --scene urdf_full_scene.xml --profile sim_fast
python3 run_urdf_full.py --list-profiles
```

대체 씬 실행:

```bash
python3 run_urdf_full.py --scene ocean_scene.xml
```

### 조작 키

- `w/s`: 전후
- `a/d`: 좌우(sway)
- `q/e`: yaw
- `r/f`: 상승/하강
- `x`: 정지
- `space`: pause
- `m`: viewer/terminal 모드 토글
- `c`: follow camera
- `1/2`: stereo left/right 카메라
- `0`: free 카메라
- `i`: 센서 오버레이 on/off
- `l`: 쓰러스터 라벨 on/off

## 4) 검증/캘리브레이션

검증:

```bash
python3 run_urdf_full.py --validate --scene urdf_full_scene.xml --validation-dir validation
python3 run_urdf_full.py --validate-thrusters --scene urdf_full_scene.xml --validation-dir validation
python3 run_urdf_full.py --validate --validate-thrusters --strict-validation --scene urdf_full_scene.xml --validation-dir validation
```

캘리브레이션:

```bash
python3 run_urdf_full.py --calibrate-joystick
python3 run_urdf_full.py --calibrate-thrusters --scene urdf_full_scene.xml --calibration-dir validation
python3 run_urdf_full.py --calibrate-thresholds --scene urdf_full_scene.xml --calibration-dir validation
```

출력 주요 파일:

- `validation/summary.json`
- `validation/validation_report.json`
- `validation/thruster_summary.json`
- `validation/*_step.csv`

## 5) ROS2 브리지

실행:

```bash
source /opt/ros/humble/setup.bash
python3 run_urdf_full.py --scene urdf_full_scene.xml --ros2
```

스테레오 이미지 포함:

```bash
source /opt/ros/humble/setup.bash
python3 run_urdf_full.py --scene urdf_full_scene.xml --ros2 --ros2-images --ros2-image-width 640 --ros2-image-height 360 --ros2-image-hz 10
```

토픽:

- Publish: `/imu/data`, `/dvl/velocity`, `/dvl/altitude`
- Publish(옵션): `/stereo/left|right/image_raw`, `/stereo/left|right/camera_info`
- Subscribe: `/cmd_vel`

## 6) 프로젝트 구조(요약)

```text
v2.1/
├── run_urdf_full.py
├── ros2_bridge.py
├── urdf_full_scene.xml
├── ocean_scene.xml
├── joystick_map.json
├── imu_calibration.json (legacy, 현재 미사용)
├── sim_profiles.json
├── thruster_tune.json
├── thruster_params.json
├── validation_thresholds.json
├── install_deps_ubuntu.sh
├── scripts/print_tree.sh
├── Dockerfile
├── docker/entrypoint.sh
├── assets/urdf_full/meshes_split/*
├── validation/README.md
└── docs/
    ├── PROJECT_TREE.md
    └── CODE_GUIDE.md
```

자세한 트리/파일별 설명:

- `docs/PROJECT_TREE.md`
- `./scripts/print_tree.sh`로 현재 폴더 트리를 즉시 확인 가능

코드 원리/함수별 정리:

- `docs/CODE_GUIDE.md`

## 7) 코드 원리 요약 (README 간단판)

- 입력 계층: terminal/joystick/ROS2 입력을 공통 `cmd` 상태로 통합
- 추진기 혼합: 수평 4개 추진기는 의사역행렬로 `forward+sway+yaw` 동시 만족
- 수중 물리: 단순화 모델(부력 + 감쇠 + 선택적 자세 안정화)로 안정성 우선
- 검증 체계: step response + 단일 추진기 검증을 JSON/CSV로 자동 기록
- 캘리브레이션: 조이스틱/추진기 게인/검증 임계값을 코드 수정 없이 JSON 갱신

## 8) Docker

빌드:

```bash
docker build -t uuv-mujoco:v2.1 .
```

헤드리스 검증:

```bash
docker run --rm -it --network host uuv-mujoco:v2.1 \
  python3 run_urdf_full.py --validate --scene urdf_full_scene.xml --validation-dir validation
```

GUI 실행(X11):

```bash
xhost +local:root
docker run --rm -it --network host \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  uuv-mujoco:v2.1 \
  python3 run_urdf_full.py --scene urdf_full_scene.xml
```

## 9) 트러블슈팅

- `No module named rclpy`
  - ROS2 미설치 또는 `source /opt/ros/humble/setup.bash` 미적용
- GUI 미표시
  - OpenGL/X11 패키지 설치 여부 및 `DISPLAY` 확인
- 조이스틱 축 반전
  - `joystick_map.json`의 `axis_sign` 값 조정
- 메쉬 파일 에러
  - `assets/urdf_full/meshes_split` 경로 확인
