# Research pool 실물 보정 통합 및 bag 비교

2026-09-16. 대상은 `ROS2-mujoco-UUVsimulator`의 **Research pool · distributed physics**
(`research_pool_distributed`) 프리셋이다.

## 통합 내용

- 10 × 5 × 5 m `research_pool_slam_scene.xml`, follow 카메라 선택과 기존 시각화 자산을 유지한다.
- 기존 승인된 `sim_profiles_bag0402_effective.json`의 `bag0402_effective`를 연결한다.
  수평 유효 gain 0.12293316274117821, surge normal drag scale 0.7997684967809776,
  부력 배율 1.01, 20 V 가정, 분산 유체력을 사용한다. 보정 수치 자체는 수정하지 않았다.
- 동일 프리셋에서 `SITL_REAL2SIM_BAG0402=1`로 기존 6개 FCU 보정 파라미터를 적용한다.
- GUI가 `UUV_HORIZONTAL_DIRECT_GAIN_SCALE=1.0`을 기본 주입하여 선택한 프로필의
  수평 gain을 지우던 문제를 수정했다. 기본값은 빈 override이며, 사용자가 명시한 값은 유지한다.
- 이전에 설치·검증된 microsecond clock 보정 ArduSub 바이너리를 사용한다.
  SHA256: `702a58ea91cf0946aa4025f404ba9cffd3aebd67cf2a134e028652c797cdbd8c`.

## 비교 방법

실물 bag에서 추출해 보존한 RC/mode/arm 시간표를 새 ArduSub + MuJoCo + ROS에 재생한다.
실물 관측값을 시뮬레이션 상태에 주입하지 않는다. 초기 disarm 구간의 위치·yaw·z만 기준을
맞추고, 비교 결과에 맞춘 시간 이동·시간축 변형·진폭 조절·추가 물리 파라미터 튜닝은 하지 않는다.

- Primary: `2026-04-02 21:46:20`, 재생 10–73.15 s, 평가 20–70 s.
- Auxiliary: `2026-04-02 21:40:11`, 재생 5–110 s, 평가 12–110 s.
- 센서 패널: 실물/시뮬 ROS DVL, IMU, pressure-derived depth. 유효 DVL만 비교한다.
- 경로 패널: 실물 FCU 추정 경로와 시뮬레이션 ground truth. 실물 외부 ground truth가 아니다.
- 초기 시뮬 위치 (-2, 1, -0.6) m. 수영장 형상을 확대하거나 벽을 제거하지 않는다.
- 실제 물리 dt 0.0005 s (원래 CAD contact guard), thruster/sensor 100 Hz, FCU 설정 400 Hz.
- 동역학 비교는 headless이며 카메라 송출은 껐다. 렌더링 성능 검증이 아니다.

20 V는 실측 ESC rail이 아닌 가정이며, tether 모델은 없다. Primary는 보정에 사용된 bag이다.
Auxiliary도 이전 비교에서 검토된 자료이므로 완전히 새로운 독립 검증 데이터라고 주장하지 않는다.
수평 gain은 이 조건에서의 유효 계수이며 실제 추진기 효율로 해석하면 안 된다.

## 재현

기존 Docker 이미지 `khm/ros2-mujoco-uuv:dev`, `.venv`, `rospkg/install`,
`ardupilot_sub_stable/build/sitl/bin/ardusub`와 원본 repo의 bag 추출물이 필요하다.
wrapper는 전용 컨테이너·ROS domain 94·localhost 통신을 사용하고, 사용 포트가 점유되어 있으면 중단한다.

```bash
cd /home/khm/robotics/underwater/ROS2-mujoco-UUVsimulator
python3 docs/report_sources/research_pool_real2sim_20260916/run_comparison.py \
  --evidence_root "$PWD"
```

두 재생이 끝나면 출력된 `outputs/research-pool-replay-<UTC timestamp>` 경로를 사용한다.
아래 Python 환경에서 분석을 실제 검증했다. NumPy/SciPy/Matplotlib/MuJoCo가 설치된 환경도 가능하다.

```bash
/home/khm/robotics/IsaacLab/isaaclab.sh -p \
  docs/report_sources/research_pool_real2sim_20260916/evaluate.py \
  --evidence_root "$PWD" \
  --run_dir outputs/research-pool-replay-<UTC timestamp> \
  --output_dir outputs/research-pool-overlay
```

현재 실행은 동일한 `start_stack.py` → `replay.py` → `read_fcu_params.py`를 전용 컨테이너에서
순서대로 실행했다. 전체 재현 wrapper는 CLI/정적 검사만 했으며 wrapper 전체를 별도로 다시 돌리지는 않았다.

`evaluate.py`는 재생 완료, RC 누락/중복, 실제 기록된 4개 수평/4개 수직 gain, 20 V,
FCU 파라미터 readback, 최종 disarm을 확인한 뒤 PNG/SVG/CSV/JSON을 만든다.
접촉 검사는 저장된 차량 자세를 원본 장면에서 다시 검사한 표본 결과이다.
다른 물체는 초기 상태이므로 연속 접촉 로그 또는 동적 장애물 접촉 검증으로 해석하지 않는다.

## 검증 및 실패 기록

- `test_research_pool_real2sim.py`: 2 tests 통과. 프리셋 연결 누락과 GUI gain 덮어쓰기를
  수정 전 각각 실패하는 것을 확인하고 수정 후 통과했다. 명시적 gain override도 확인한다.
- `test_real2sim_launch.py`: 5 tests 통과.
- `check_gui_distributed_profile_contract.py`, `check_gui_start_contract.py`: PASS.
- 새 실험 스크립트와 새 테스트: Ruff 통과.
- 장면·수면·조명·stereo 이미지 코드·기존 보정 JSON은 해시가 보존되었다.

초기 DDS 시작 실패, 중단 실행, GUI gain이 1.0으로 덮인 실행도 raw evidence에 보존했다.
`primary_pool` 및 `auxiliary_pool`만 최종 보정 비교 대상이다.
`*_gain_overridden*`은 올바른 보정 결과가 아니며 최종 오버레이에 사용하지 않는다.
