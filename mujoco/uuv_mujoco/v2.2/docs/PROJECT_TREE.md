# UUV MuJoCo v2.1 Project Tree

아래 트리는 `v2.1` 배포 폴더의 실행/배포 관점 구조입니다.

```text
v2.1/
├── VERSION.txt
├── README.md
├── requirements.txt
├── install_deps_ubuntu.sh
├── scripts/
│   └── print_tree.sh
├── run_urdf_full.py
├── ros2_bridge.py
├── urdf_full_scene.xml
├── ocean_scene.xml
├── joystick_map.json
├── imu_calibration.json
├── thruster_tune.json
├── thruster_params.json
├── sim_profiles.json
├── validation_thresholds.json
├── Dockerfile
├── docker/
│   └── entrypoint.sh
├── assets/
│   └── urdf_full/
│       └── meshes_split/
│           ├── base_link_part*.stl
│           ├── VER_*.STL / YAW_*.STL
│           ├── t200_prop.stl
│           ├── sand_tex.png
│           ├── seabed_*.obj / seabed_hf.png
│           └── rock_*.obj
├── validation/
│   └── README.md
└── docs/
    ├── PROJECT_TREE.md
    └── CODE_GUIDE.md
```

## File-by-File Detail

| Path | Purpose | Key Inputs | Key Outputs |
|---|---|---|---|
| `run_urdf_full.py` | 메인 실행기(뷰어/조이스틱/검증/캘리브레이션/ROS2 연동) | `--scene`, `joystick_map.json`, `sim_profiles.json`, `thruster_*.json` | 실시간 시뮬레이션, `validation/*.json|csv` |
| `ros2_bridge.py` | ROS2 토픽 브리지 | MuJoCo 센서/카메라, `/cmd_vel` | `/imu/data`, `/dvl/*`, `/stereo/*` |
| `urdf_full_scene.xml` | 기본 수영장 씬 + 로봇 모델 + 센서 site | STL/OBJ 메쉬 | MuJoCo 모델/관절/센서 정의 |
| `ocean_scene.xml` | 대체 해양 씬 설정 | 동일 메쉬 자산 | 바다형 씬 로딩 |
| `joystick_map.json` | 조이스틱 축/부호/데드존 설정 | Linux joystick axes | 명령 채널(`forward/sway/yaw/heave`) |
| `imu_calibration.json` | 레거시 보관 파일(현재 로직에서 미사용) | 과거 IMU 보정 결과 | 참고용 메타데이터 |
| `thruster_tune.json` | 쓰러스터 방향 벡터(gear) 수동 튜닝 | actuator name별 벡터 | 수평/수직 혼합기 입력 방향 |
| `thruster_params.json` | 쓰러스터 gain/응답 파라미터 | per-thruster `gain_scale` | thrust scaling |
| `sim_profiles.json` | 물리 프로파일 묶음(`sim_real`, `sim_fast`) | drag, buoyancy, timestep 등 | 실행 시 profile 선택 |
| `validation_thresholds.json` | 검증 pass/fail 임계값 | overshoot/settling/drift 기준 | `--strict-validation` 결과 |
| `install_deps_ubuntu.sh` | Ubuntu 의존성 자동 설치 | apt + pip + options | 실행 가능한 런타임 환경 |
| `scripts/print_tree.sh` | 현재 폴더 트리 출력(`tree` 또는 `find`) | 프로젝트 루트 | 구조 점검 출력 |
| `requirements.txt` | Python 패키지 버전 하한 | pip | `mujoco`, `numpy` |
| `Dockerfile` | 컨테이너 빌드 정의 | `requirements.txt`, 소스 | `uuv-mujoco:v2.1` 이미지 |
| `docker/entrypoint.sh` | 컨테이너 시작 시 ROS2 환경 source | `/opt/ros/humble/setup.bash` | 지정 커맨드 실행 |
| `assets/urdf_full/meshes_split/*` | 렌더/충돌 메쉬 | CAD 변환 결과 | 모델 가시화/충돌 |
| `validation/README.md` | 검증 출력 파일 의미 설명 | validation 결과 파일 | 분석 가이드 |
| `docs/CODE_GUIDE.md` | 코드 원리 설명(함수 단위) | 코드 구조 | 유지보수 가이드 |

## Runtime Data (Generated at Execution)

다음 파일들은 실행/검증/캘리브레이션 시 갱신될 수 있습니다.

- `validation/summary.json`
- `validation/validation_report.json`
- `validation/*_step.csv`
- `validation/thruster_single.csv`
- `validation/thruster_summary.json`
- `validation/calibration_report.json`
