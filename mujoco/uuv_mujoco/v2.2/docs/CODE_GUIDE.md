# UUV MuJoCo v2.1 Code Guide

이 문서는 `v2.1`의 주요 파일별 코드 원리와 실행 흐름을 정리합니다.

## 1) `run_urdf_full.py`

### 1.1 역할

- 단일 엔트리포인트에서 아래 모드를 모두 처리합니다.
- 실시간 뷰어 실행
- 조이스틱/키보드 제어
- 검증(`--validate`, `--validate-thrusters`)
- 캘리브레이션(`--calibrate-*`)
- ROS2 브리지 활성화(`--ros2`)

### 1.2 핵심 흐름

1. CLI 파싱
2. 프로파일/임계값 JSON 로드
3. MuJoCo 모델 초기화
4. 제어 입력 루프(terminal + joystick)
5. 물리 루프(쓰러스터 + 부력 + 감쇠)
6. 오버레이 렌더링(벡터/센서 라벨)
7. 필요 시 검증/캘리브레이션 리포트 저장

### 1.3 함수 그룹

| Function | Principle | Why it matters |
|---|---|---|
| `apply_joystick_axes` | raw axis -> deadzone -> normalized command | 컨트롤러 교체 시 최소 수정 포인트 |
| `mix_horizontal_thrusters` | 의사역행렬 기반 4개 수평 쓰러스터 믹싱 | 전진/스웨이/요를 동시 만족 |
| `update_stabilization` | IMU 기반 roll/pitch PID(PI+D 형태) | 자세 유지(전복 완화) |
| `apply_underwater_wrench` | 단순 수중 모델(부력 + 선형/각속 감쇠) | 과도한 과장 없이 안정성 확보 |
| `run_validation` | step response 지표화 | 튜닝 회귀(regression) 탐지 |
| `run_thruster_validation` | 단일 쓰러스터 방향 검증 | 축/기어 벡터 오류 탐지 |
| `calibrate_thruster_gains` | 속도 편차 기반 gain_scale 보정 | 하드웨어 비대칭 근사 |

### 1.4 설계 원칙

- 수중 유체 모델은 복잡 모델보다 안정적/재현 가능한 단순 모델을 우선합니다.
- 모든 튜닝 값은 JSON 파일로 분리해 코드 수정 없이 반복 조정 가능합니다.
- 검증 결과는 CSV/JSON 동시 저장하여 시각화/자동판정 둘 다 대응합니다.

## 2) `ros2_bridge.py`

### 2.1 역할

- `/cmd_vel` 구독 후 시뮬레이터 명령으로 변환
- IMU/DVL 센서를 ROS2 메시지로 발행
- 옵션으로 stereo image + camera_info 발행

### 2.2 핵심 함수

| Function | Principle | Note |
|---|---|---|
| `_on_cmd_vel` | `[-1,1]` clamp 후 내부 thrust scale로 매핑 | 과입력 방지 |
| `spin_once` | non-blocking ROS callback 처리 + timeout stop | 안전 정지 fail-safe |
| `publish` | sensor/image 주기 분리 발행 | 고주파 센서 + 저주파 이미지 병행 |
| `_build_camera_info` | MuJoCo fovy -> pinhole 파라미터 추정 | ROS2 camera pipeline 연동 |

### 2.3 프레임 주의점

- MuJoCo quaternion은 `[w, x, y, z]` 순서입니다.
- ROS2 메시지 프레임 이름은 `imu_link`, `dvl_link`, `*_optical`을 사용합니다.

## 3) `urdf_full_scene.xml`

### 3.1 역할

- 로봇 메쉬/관절/사이트/센서/카메라 배치
- 쓰러스터 site 위치와 actuator gear 방향 정의
- 기본 수영장 환경(바닥/벽/수면 시각화)

### 3.2 튜닝 포인트

| Element | What to tune |
|---|---|
| `<body name="base_link" ...>` | 초기 자세(스폰 orientation) |
| `<site name="imu_site" ...>` | IMU 설치 위치/방향 |
| `<site name="dvl_site" ...>` | DVL 설치 위치/방향 |
| `<motor name="yaw_*" gear="...">` | 수평 추진 방향 벡터 |
| `<motor name="ver_*" gear="...">` | 수직 추진 방향 벡터 |

## 4) JSON Config Files

### `joystick_map.json`

- `axes`: 채널별 입력 축 ID
- `axis_sign`: 축 반전 부호
- `deadzone`, `yaw_deadzone`: 저속 떨림 억제
- `buttons`: stop/pause/mode 버튼 매핑

### `sim_profiles.json`

- 프로파일별 부력/드래그/쓰러스터 최대힘/검증 적분기 설정
- `sim_real`: 현실성 우선
- `sim_fast`: 빠른 반복 튜닝 우선

### `validation_thresholds.json`

- step response 및 단일 쓰러스터 통과 기준
- `--strict-validation` 시 CI 스타일 fail gate로 사용 가능

### `imu_calibration.json`

- 현재 v2.1 런타임 경로에서는 읽지 않는 레거시 파일입니다.
- 이전 실험 결과 보관 용도로만 유지합니다.

## 5) 운영 체크리스트

1. `install_deps_ubuntu.sh` 실행
2. `python3 run_urdf_full.py --scene urdf_full_scene.xml`
3. 조이스틱 축 확인(좌우 반전 시 `joystick_map.json`의 `axis_sign.sway` 조정)
4. 검증 실행 후 `validation/validation_report.json` 확인
5. 필요 시 `--calibrate-thrusters`, `--calibrate-thresholds` 순으로 보정
