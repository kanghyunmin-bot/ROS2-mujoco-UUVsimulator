# 실측 로그로 MuJoCo 기체 자동 보정

[처음 시작하기 · 필수 파일과 GUI 실행 순서](REAL2SIM_QUICKSTART.md) · [편집 가능한 흐름도](../paper/real2sim-automation-workflow.pptx)

기체별 입력 패키지에는 MuJoCo 장면과 초기 물리 프로필, 모터 매핑, 센서 장착 정보, 보정·검증용 주행을 넣는다. 같은 실측 모터 출력을 초기 모델과 새 모델에 재생해 비교하고, 사용자가 선택한 결과를 MuJoCo 뷰어에서 볼 수 있다.

## 입력 폴더

```text
my_rov/
  vehicle.json
  model/
    scene.xml
    includes/
    meshes/
  config/
    profiles.json
    thruster_performance.json
  trials/
    surge_step/
      metadata.json
      pwm.csv
      real.csv
      voltage.csv
    validation_route/
      metadata.json
      pwm.csv
      real.csv
      voltage.csv
```

경로는 `vehicle.json`이 있는 폴더 기준이다. MJCF가 참조하는 include와 메시도 `model/` 안에 둔다. 기존 기체의 MJCF를 재사용하거나 CAD/URDF에서 변환한 MJCF를 넣을 수 있다. 설치 질량·관성·부착 좌표는 변환 후 확인한다.

## 기체 설정

`vehicle.json`은 UTF-8 JSON이다. 아래는 필드 예시다.

```json
{
  "version": 1,
  "name": "my_rov",
  "scene": "model/scene.xml",
  "profile_file": "config/profiles.json",
  "base_profile": "nominal",
  "thruster_perf_file": "config/thruster_performance.json",
  "thrusters": [
    {"name": "yaw_rf", "kind": "horizontal", "pwm_sign": -1, "forward_sign": 1},
    {"name": "ver_rf", "kind": "vertical", "pwm_sign": 1, "forward_sign": 1}
  ],
  "sensors": {
    "imu": {"pos_m": [0.05, 0, 0.08], "quat_wxyz": [1, 0, 0, 0]},
    "pressure": {"pos_m": [-0.15, 0, 0.09], "quat_wxyz": [1, 0, 0, 0]},
    "dvl": {"pos_m": [0, 0, -0.10], "quat_wxyz": [0, 1, 0, 0]},
    "camera_left": {"pos_m": [0.22, 0, 0.07], "quat_wxyz": [0.5, 0.5, -0.5, -0.5], "fovy_deg": 70}
  },
  "train": ["surge_step"],
  "holdout": ["validation_route"],
  "reference_profile": "optional_previous_fit"
}
```

- 장면은 world의 직접 자식인 `base_link`와 `world_joint` freejoint를 사용한다. `base_link`의 설치 상태 질량(kg), CoM(m), 관성(kg·m²)은 MJCF에 기록한다. 현 버전의 빠른 식별 모델은 기체축과 정렬된 대각 관성을 사용한다. 물 표면은 월드 z=0이고 기체 장착 좌표는 FLU다.
- 각 모터는 `thr_<name>` 사이트에 연결된 MJCF motor다. 사이트 위치와 motor의 양의 힘 방향을 기체 좌표계로 둔다. `thrusters` 배열 순서가 실측 `pwm1`, `pwm2`, …에 대응한다. `pwm_sign`은 최종 ESC 펄스의 편차를 actuator의 양·음 방향으로 바꾸는 부호다. `forward_sign`은 motor의 양의 힘 방향과 공개 추력표의 정방향 일치 여부다. 둘 다 +1 또는 -1이다.
- 초기 프로필과 추력 파일은 [현재 프로필](../../uuv_mujoco/current/config/sim_profiles_bluerov2.json)과 [전압별 PWM·RPM·추력표](../../uuv_mujoco/current/config/thruster_performance.json) 형식을 따른다. 추력표는 시험 기체에 장착된 추진기 종류와 일치해야 한다. 초기 프로필은 `legacy` 6-DOF 감쇠를 사용한다.
- 센서의 `pos_m`과 정규화된 `quat_wxyz`는 `base_link` 기준이다. IMU·압력·DVL·좌우 카메라는 있는 항목만 적는다. 카메라 쿼터니언은 MJCF 카메라 프레임으로 준다. 설치 위치와 방향은 실측값 또는 장착 도면을 사용한다.
- `train`과 `holdout`은 서로 다른 주행 폴더를 가리킨다. `reference_profile`은 같은 프로필 파일 안의 비교용 모델이며 선택 사항이다.

## 주행 데이터

각 주행의 파일은 동일한 시계의 초 단위 타임스탬프를 사용한다. 채널 순서, 센서 지연과 좌표 변환을 기록하고, 유효하지 않은 측정은 `nan`으로 둔다.

| 파일 | 필수 값 |
| --- | --- |
| `pwm.csv` | `t_s,pwm1,pwm2,...`. 최종 모터 PWM [µs]과 원래 수신 시각. 중립 1500 µs, 유효 범위 1100–1900 µs. |
| `real.csv` | `t_s,surge_mps,sway_mps,heave_mps,roll_rate_radps,pitch_rate_radps,yaw_rate_radps,depth_m,roll_deg,pitch_deg,yaw_deg`. 속도·각속도는 기체 원점 FRD, 수심은 아래가 양수, 자세는 NED 기준. DVL 속도가 센서 원점이면 장착 레버암을 보정한다. 첫 행에 각 값이 모두 유효해야 한다. |
| `voltage.csv` | `sim_time,voltage_v`. `sim_time`은 `real.csv` 첫 시각 기준이며 0부터 시작한다. 시험 동안의 팩 전압 또는 ESC 전압 [V]을 기록한다. |
| `metadata.json` | 예시 `{"initial_depth_m": 2.0, "initial_depth_reference": "base_link"}`. 압력 센서 기준 깊이는 `"pressure"`를 쓴다. |

ArduPilot의 RC_IN은 스틱 입력이고 RC_OUT은 모터 출력이다. 플랜트 보정에는 RC_OUT을 사용한다. ROS 2 bag은 타임스탬프를 보존하면서 위 CSV로 먼저 내보낸다. DFKI v3 원본은 저장소의 DFKI 변환기로 정규화할 수 있다.

무추력 트림, 서로 다른 세기의 전진·후진/횡이동/상하/선회 입력과 중립 복귀, 따로 기록한 혼합 주행을 권장한다. 접촉, 유속, 테더, 탑재 상태가 달라지면 별도 주행으로 관리한다.

### 기록 예시와 검증 조건

```csv
t_s,pwm1,pwm2
0.000,1500,1500
0.025,1500,1500
0.050,1600,1500
```

```csv
t_s,surge_mps,sway_mps,heave_mps,roll_rate_radps,pitch_rate_radps,yaw_rate_radps,depth_m,roll_deg,pitch_deg,yaw_deg
0.000,0,0,0,0,0,0,2.0,0,0,90.0
0.020,0.012,0,0,0,0,0.015,2.0,0,0,90.017
```

```csv
sim_time,voltage_v
0.000,15.9
0.100,15.8
0.200,15.7
```

위 세 예시는 헤더와 단위를 보여준다. 실제 실행에는 보정 주행마다 최소 3.1초, 별도 검증 주행마다 최소 1초, 각 축에 채점 가능한 대응 샘플이 10개 이상 필요하다. 실제 보정에는 무추력 구간과 여러 추진 패턴을 포함해 충분히 긴 로그를 쓴다.

- `real.csv` 첫 행은 모든 필드가 유효한 초기 상태다. 첫 유효 상태에 맞춰 로그를 정리하고 같은 시계를 사용하는 PWM의 직전 패킷은 남긴다. `depth_m`은 이후 상대수심을 계산할 때 사용한다.
- 실측의 원본 속도가 월드 좌표라면 기체 FRD로 회전한다. DVL이 기체 원점과 떨어져 있으면 센서 속도를 FRD로 바꾼 후 `v_base = v_sensor - ω × r_sensor`를 적용한다. 센서 시각 정렬과 단위 변환은 CSV 내보내기 단계에서 수행한다.
- PWM은 원본 시각에 계단형 입력으로 적용한다. 전압은 0초에서 시작하고 마지막 측정 시각까지 기록한다. 전압 표본 사이와 로그 끝에는 최대 0.25초 간격을 허용한다. 실측 신호에 빠진 행이 있으면 해당 표본의 열을 `nan`으로 표시한다.
- 데이터에 기록할 센서와 MuJoCo에 장착할 센서는 독립적이다. 수조 모션캡처로 속도와 수심을 기록해도 시뮬레이션에 IMU, 압력 센서와 카메라를 설치할 수 있다.
- 여러 비교 프로필은 `"reference_profiles": {"existing": "old_fit", "existing_yaw": "yaw_fit"}`로 적는다. 보정 주행과 검증 주행을 서로 다른 이름으로 선언하고, 각각 독립적으로 기록한다.

### 센서별 설정

| 키 | 설정값 | 생성되는 MuJoCo 센서 |
| --- | --- | --- |
| `imu` | `pos_m`, `quat_wxyz` | `imu_quat`, `imu_gyro`, `imu_acc` |
| `pressure` | `pos_m`, `quat_wxyz` | 압력 포트의 월드 위치 `depth_pos` |
| `dvl` | `pos_m`, `quat_wxyz` | 사이트 좌표계 속도 `dvl_vel_body`, 사이트 +Z 레이의 `dvl_altitude` |
| `camera_left`, `camera_right` | `pos_m`, `quat_wxyz`, `fovy_deg` (수직, 기본 70°). 장착점이 렌즈와 다르면 `mount_pos_m`, `mount_quat_wxyz`도 지정 | `stereo_left`, `stereo_right` |

월드 +Z가 위다. 바닥을 보는 DVL은 사이트 +Z를 아래로 배치한다. 수평 기체에서 FLU의 -Z를 향하는 예시는 `"quat_wxyz": [0, 1, 0, 0]`이다. 카메라는 로컬 -Z를 보고 로컬 +Y가 영상 위쪽이다. 압력 센서 기준 초기 수심에는 `pressure`를 지정한다. 이 자동화의 센서 범위는 장착 좌표, 방향, 센서 타입과 카메라 시야각이다. DVL 다중 빔, 압력 센서 노이즈, 카메라 왜곡·노이즈 및 새 기체의 ROS 2 토픽·SITL 믹서 설정은 별도 모델과 데이터가 필요하다.

## 실행

Python 3.10+ 환경에 MuJoCo, NumPy, SciPy가 필요하다. 전체 주행 보정에는 Numba를 함께 설치하면 반복 계산을 크게 줄일 수 있다. 웹 GUI에서 이미 생성된 모델을 재생하는 데에는 기존 MuJoCo 런타임을 사용한다.

```bash
python3 -m venv ~/.local/share/uuv-real2sim/venv
source ~/.local/share/uuv-real2sim/venv/bin/activate
python -m pip install "mujoco==3.12.0" numpy scipy numba
python uuv_mujoco/current/tools/real2sim_auto.py run /path/to/my_rov \
  --output outputs/my_rov-fit --jobs 4 --max-evaluations 35
```

출력 폴더에는 센서가 설정된 MJCF와 메시, `vehicle.json`, 후보가 추가된 `profiles.json`, `sensor_config.json`, 보정 진단 `fit.json`, 6축 및 속도·각속도 오차 `rmse.csv`가 생성된다. `replays/<trial>/<variant>/`에 실제 MuJoCo 궤적과 적용 PWM 시각이 저장된다. `replays/<trial>/aligned.csv`는 유효 구간을 포함하는 공통 10 Hz 비교 시계열이다. `report.json`은 모든 주행의 재생과 채점이 완료됐을 때 생성되며, 입력 로그 경로는 출력 폴더 기준 상대경로로 기록된다. 입력과 출력을 함께 이동할 때는 상대 위치를 유지한다.

디스플레이가 있는 환경에서는 접촉 검토가 필요 없는 후보를 실행 완료 후 첫 보정 주행으로 연다. 접촉한 후보는 GUI에서 선택해 수동으로 재생할 수 있다. 비교 파일만 만들려면 `--headless`를 붙인다. 이미 생성한 모델은 다음과 같이 연다.

```bash
python uuv_mujoco/current/tools/real2sim_auto.py view outputs/my_rov-fit \
  --trial validation_route --profile candidate
```

### 웹 GUI

웹 GUI 왼쪽 **Vehicle Setup & Calibration**에서 `vehicle.json`이 있는 입력 폴더의 저장소 기준 상대경로와 새 출력 이름을 입력한다. 보정 시작 후 진행 로그와 완료 상태가 표시된다. GUI 서버는 `UUV_REAL2SIM_PYTHON`으로 지정한 Python 또는 `~/.local/share/uuv-real2sim/venv/bin/python`을 우선 사용한다. 웹 GUI는 저장소의 `outputs/` 아래 완료된 보정 결과를 기체 목록에서 찾는다. 다른 경로는 GUI 서버의 `UUV_REAL2SIM_OUTPUT_ROOT`로 지정한다. Docker 사용 시 입력 패키지와 결과를 모두 컨테이너가 읽을 수 있는 경로에 둔다. 상단 **Vehicle**에서 Real2Sim 기체를 고르면 설정된 센서, 실측 주행, 물리 프로필을 확인하고 **Open MuJoCo replay**로 연다. 실측 PWM을 쓰는 오프라인 모드이므로 새 기체에서 즉시 조종하려면 별도 믹서와 제어 설정이 필요하다.

## 보정이 바꾸는 파라미터

50 Hz의 완전 잠수 근사 모델은 MJCF 질량·관성·CoM과 추진기 위치·방향, 전압별 RPM·추력표를 사용한다. 먼저 8초·60초 구간으로 초기값을 다듬고, 두 번째 단계에서 각 보정 주행을 실측 초기 상태로 한 번만 시작해 끝까지 적분한 잔차를 추가한다. 두 번째 단계의 잔차 가중치는 짧은 구간 1.0, 긴 구간 0.55, 전체 주행 0.45다. 두 단계에서 제한 최소제곱으로 측정 운동과의 가중 잔차 및 초기값에서 벗어난 정도를 함께 줄인다. 선형·이차 감쇠, 부력 크기와 CoB 오프셋, yaw 모멘트 배율, 추진기별 유효 추력 배율, 유동에 따른 pitch 모멘트 계수를 저장한다. 부가질량 0, 대각 관성, RPM 응답, 정상 전압 로그와 수중 자유운동을 입력으로 사용한다. 현재 탐색 범위는 BlueROV2급 소형 기체의 초기값을 기준으로 정했다. 초기 프로필은 대칭 yaw 모델을 사용한다. 방향별 계수는 후속 단계에서 보정하며, 이미 방향별 조정을 마친 기존 프로필은 비교용으로 지정한다. 주행에 따라 남는 테더 장력·조류·접촉 효과는 유효 파라미터와 섞일 수 있다.

`fit.json`은 목적함수 변화, 약하게 움직인 축, 파라미터 경계를 기록한다. 약한 축은 속도 0.02 m/s 또는 각속도 0.03 rad/s 미만의 5–95 백분위 폭을 기준으로 표시한다. 이 표시는 입력이 부족할 수 있는 축을 찾기 위한 기준이다. 최종 성능은 50 Hz 모델의 목적함수 대신, 보정에서 제외한 주행까지 실제 MuJoCo에서 재생한 축별 RMSE로 판단한다. 수심은 시작점 대비 변화량, yaw는 초기값을 뺀 누적 각도다. 짧은 구간 yaw 오차와 yaw rate 오차도 보고서에서 별도로 확인할 수 있다.

## BlueROV2 재현

[DFKI BlueROV2 수조 로그 v3](https://zenodo.org/records/17360027)의 각 원본 주행 CSV 폴더를 `prepare_dfki_bluerov2.py <원본 주행 폴더> <정규화 폴더>/<주행 이름> --trial <주행 이름>`으로 변환한다. 공개 로그의 RC_OUT, 팩 전압, 모션캡처 위치·속도 및 탑재 자세를 사용한다. 정규화한 9개 주행을 아래처럼 패키징한다.

```bash
python uuv_mujoco/current/tools/prepare_bluerov2_real2sim.py \
  --dfki-data /path/to/normalized-dfki --output outputs/real2sim-bluerov2-input-v2
python uuv_mujoco/current/tools/real2sim_auto.py run outputs/real2sim-bluerov2-input-v2 \
  --output outputs/real2sim-bluerov2-native-fit --headless --max-evaluations 35 --jobs 8
```

보정은 `random`, `circle`, `5m`, `10m`, `15m`에 한정한다. `serpentine`, `7m`, `12m`, `14m`는 파라미터 추정 후에만 사용한다. 모든 모델은 같은 수조 MJCF, 초기 상태, 모터 시계열, 전압 로그, 샘플 마스크로 비교한다. `existing`은 기존 일반 보정 프로필이고, `existing_yaw`는 논문 그림에 사용한 무작위 주행 yaw 추가 보정 프로필이다.

### 이전 자동 보정 결과

아래 값은 8초·60초 구간만 사용했던 버전이다. MuJoCo 3.12.0에서 9개 주행과 4개 모델을 조합해 총 36회 재생했다. 보정용 목적함수는 34회 평가에서 94.216에서 30.354로 줄었다. 상하 추진기 두 개의 유효 추력 배율과 상하 방향 이차 감쇠가 탐색 경계에 도달했다.

| 지표 | 초기 모델 | 자동 보정 | 기존 보정 | 기존 yaw 보정 |
| --- | ---: | ---: | ---: | ---: |
| Surge [m/s] | 0.3715 | 0.2057 | 0.2069 | 0.2070 |
| Sway [m/s] | 0.1503 | 0.0863 | 0.0859 | 0.0853 |
| 상대수심 [cm] | 239.95 | 139.86 | 127.00 | 126.46 |
| Roll [°] | 28.29 | 6.49 | 6.85 | 6.70 |
| Pitch [°] | 20.41 | 13.01 | 12.65 | 12.67 |
| 누적 yaw [°] | 823.97 | 441.55 | 333.21 | 415.13 |

표는 검증용 주행 4개의 **주행별 RMSE 산술평균**이다. 자동 보정은 초기 모델 대비 검증용 24개 항목 모두에서 오차를 줄였다. 기존 일반 보정과 비교하면 24개 중 11개가 줄었다. 보정에 사용한 `random`에서는 기존 yaw 보정의 누적 yaw RMSE가 28.8°, 자동 보정은 189.7°였다. 자동 보정의 상대수심은 이 주행에서 102.0 cm였으며 재생 약 175초에 바닥 접촉이 기록됐다. `serpentine` 단일 검증 주행에서는 자동 보정의 상대수심과 누적 yaw가 95.6 cm와 413.2°로, 기존 일반 보정의 149.5 cm와 508.0°보다 낮았다. 자동 보정 모델은 이 주행에서도 재생 약 340초에 바닥에 닿았다.

이 결과는 모델 전체를 자동으로 채택하기 위한 기준과 별도로 보관한다. 자동 보정 후보와 기존 프로필을 GUI에서 각각 재생할 수 있다.

### 바닥 접촉 진단

DFKI가 공개한 대형 수조 깊이는 8 m이다. 같은 초기 상태, PWM, 팩 전압, 프로필을 고정하고 진단용 복제 장면에서 바닥만 25 m로 내렸다. 두 주행 모두 접촉은 사라졌지만 자동 보정 후보의 최대 수심은 `random` 8.36 m, `serpentine` 8.85 m까지 증가했다. 상대수심 RMSE는 각각 102.0→114.9 cm, 95.6→113.0 cm였다. 25 m 장면은 맵 깊이 영향을 분리하기 위한 실험이다. [DFKI 수조 시설](https://robotik.dfki-bremen.de/en/research/research-facilities-labs/maritime-infrastructure)

`rmse.csv`의 `free_motion` 행은 비교 모델 중 가장 먼저 접촉한 시점 직전까지 동일한 표본만 사용한다. 접촉이 빠른 주행에서 공통 유효 표본이 10개 미만이면 해당 구간 RMSE를 비워 둔다. 전체 구간 RMSE와 각 모델의 첫 접촉 시각도 유지한다. 자동 보정 후보가 접촉하면 `report.json`의 `validation.candidate_requires_review`가 활성화된다.

### 전체 주행 잔차 추가 후 재검증

입력 모델, 학습 5개와 별도 검증 4개, 평가 마스크를 고정했다. 같은 8 m 장면에서 자동 후보를 다시 생성하고 네 프로필을 36회 재생했다. 35회 평가 중 구간 최적화에 17회, 전체 주행을 포함하는 최적화에 18회를 사용했다. 후반 단계가 평가 한도에 도달했으므로 수렴 여부는 확정되지 않았다.

| 검증 주행 4개 RMSE 산술평균 | 이전 자동 | 전체 주행을 포함한 자동 | 기존 일반 보정 |
| --- | ---: | ---: | ---: |
| Surge [m/s] | 0.2057 | 0.2071 | 0.2069 |
| Sway [m/s] | 0.0863 | 0.0859 | 0.0859 |
| 상대수심 [cm] | 139.86 | 128.94 | 127.00 |
| Roll [°] | 6.49 | 6.92 | 6.85 |
| Pitch [°] | 13.01 | 12.61 | 12.65 |
| 누적 yaw [°] | 441.55 | 332.59 | 333.21 |

자동 후보에서 접촉한 주행은 9개 중 4개에서 0개로 줄었다. 검증 24개 지표 중 13개가 개선됐다. `random`의 수심 RMSE는 102.0→55.1 cm, 누적 yaw는 189.7→206.1°였으며, `serpentine`의 수심은 95.6→150.6 cm, 누적 yaw는 413.2→504.5°였다. 전체 주행 잔차가 평균 개선을 만들었어도 주행별 효과는 다르다. 이전 후보는 해당 실행 폴더에 그대로 보관한다.

BlueROV2의 카메라 장착점은 렌즈의 렌더링 위치보다 55 mm 뒤에 있으므로 두 위치와 방향을 각각 기록한다. 압력 센서의 장착 위치, 질량과 관성, 테더 영향은 공개 로그로부터 독립적으로 확정한 값이 아니다. 기존 yaw 실험 프로필의 방향별 감쇠와 추진기별 yaw 모멘트 계수는 비교용 프로필에 포함된다.

### 채널별 시간축과 실제 MuJoCo 후보 선택

DFKI 변환기는 원본 속도와 자세 시각, 100 Hz 격자를 합쳐 기준 시각을 만든다. 자세는 원본 자세 센서, 위치는 모션캡처, 기체 속도는 원본 속도와 마커 방향에서 각각 계산한다. 자세에 유효한 관측이 있으면 속도나 위치의 공백 중에도 자세를 보존한다. 각 채널의 보간 제한을 넘는 구간은 결측으로 남긴다. 예를 들어 `15m`의 10 Hz yaw 평가 표본은 1,156개에서 2,039개로 늘고, surge는 1,054개로 유지됐다.

1. **대칭 모델 추정.** 보정용 주행의 8초·60초·전체 구간에서 선속도·각속도·수심·자세 잔차로 감쇠, 부력·CoB, 유효 추력과 모멘트를 추정한다.
2. **방향별 yaw 후보.** 비야우 계수는 고정하고 수평 추진기별 yaw 모멘트와 회전 방향별 선형 yaw 감쇠를 별도로 적합한다. 양방향 움직임이 부족하면 비대칭 감쇠를 고정한다. 정규화 항과 별도로 계산한 국소 민감도로 구분할 수 없는 계수도 고정한다.
3. **실제 MuJoCo 후보 선택.** 초기 프로필, 대칭 후보, 방향별 후보, 부력 배율을 ±0.002 이동한 후보를 모두 보정용 전체 주행으로 재생한다. 수직 방향 움직임이 약하면 부력 후보를 추가하지 않는다. 접촉 주행이 적은 후보를 우선하고, 그 안에서 6축 RMSE와 1·5·10초 yaw 변화 오차의 정규화 평균이 가장 낮은 후보를 선택한다.
4. **별도 검증.** 선택 이후에만 holdout을 재생한다. `training_candidates/`에는 각 후보의 프로필과 보정용 재생을 저장한다. `fit.json`의 `native_selection`에는 후보별 접촉 수, 평가 척도, 오차 및 선택 결과를 기록한다.

절대 압력 초기값의 수면 기준은 기존 조건을 유지했다. 이 과정은 테더 힘, 부가질량, 센서 바이어스와 장착 추력 곡선을 각각 독립적으로 식별하는 실험을 대신하지 않는다.

BlueROV2의 5개 보정 주행에서 동일 기준으로 전체 재생했다. 대칭 후보와 방향별 yaw 후보는 둘 다 접촉이 없었지만, 실제 MuJoCo 목적함수는 각각 1.5158과 1.5366이었다. 방향별 후보는 학습 주행의 평균 누적 yaw를 309.6°에서 263.3°로 줄였고, 5초 yaw 변화 RMSE는 38.1°에서 40.5°로 늘렸다. 종합 기준에 따라 대칭 후보를 선택했다. 두 번째 yaw 식별 단계는 17회 평가 한도에 도달했다.

| 검증 주행 4개 RMSE 산술평균 | 이전 자동 재생 | 이번 자동 재생 | 기존 일반 보정 |
| --- | ---: | ---: | ---: |
| 상대수심 [cm] | 130.17 | 136.37 | 128.17 |
| 누적 yaw [°] | 327.68 | 312.32 | 328.30 |
| 5초 yaw 변화 [°] | 49.19 | 48.61 | 49.19 |

이 표는 자세 표본을 복원한 동일한 실측 타임라인으로 이전 자동 프로필과 이번 프로필을 모두 다시 재생한 비교다. 자동 후보의 시뮬레이션 접촉은 두 버전 모두 0회였으며, 이번 후보의 상대수심 오차는 커졌다. 기존 결과는 별도 출력 폴더에 보존한다.

### 실제 재생 오차로 계수를 다시 탐색

첫 번째 MuJoCo 후보 비교에서 얻은 **주행별 yaw rate 편향**으로, 대칭 모델과 방향별 모델 사이의 변경량을 추정한다. 평균 부호가 상쇄되지 않도록 주행별 편향 벡터에 최소제곱을 적용한다. 그 방향으로 만든 중간 yaw 후보와 부력 반 스텝 후보를 실제 MuJoCo로 다시 재생한다. 접촉 주행 수가 늘거나 짧은 yaw 응답과 다른 축 평균 RMSE가 정해진 허용 폭을 넘으면 후보를 제외한다. 승인 기준은 다른 축 3%, 1·5·10초 yaw 변화 5%를 사용하며, 기준값이 작으면 평가 정규화 척도의 1%를 하한으로 둔다. 기존 승자도 선택지에 남긴다.

특정 학습 주행을 논문 그림과 같은 조건으로 재현할 때에는 `vehicle.json`에 `"yaw_target_trial": "random"`을 추가하거나 실행 시 `--yaw-target-trial random`을 지정한다. 해당 주행은 `train`에 있어야 한다. BlueROV2 입력 예시는 이 값을 설정한다. `yaw_target` 프로필은 일반 목적의 `candidate`와 별도로 저장된다.

이 프로필은 선택한 주행의 동일 실측 PWM·전압을 실제 MuJoCo에 넣어 수평 추진기별 yaw 모멘트, 선형·이차 yaw 감쇠, 방향 비대칭 계수를 제한 범위에서 갱신한다. 각 계수를 조금씩 바꿔 계산한 yaw 궤적의 차이로 유한차분 민감도를 구한다. 전체 누적 yaw, 5초 yaw 변화 및 yaw rate 잔차를 사용해 갱신량을 추정하고, 실제 재생에서 오차가 줄어든 갱신만 수락한다. 준비 반복 1회 후 작은 보폭과 이차 감쇠를 포함해 기본 최대 3회 반복한다. 후속 반복 횟수는 `--yaw-target-rounds`로 지정한다. 사용한 주행의 결과는 보정 적합도이며, 별도 주행 결과로 일반화를 평가한다.

같은 BlueROV2 입력에서 일반 모드는 방향별 후보의 절반 크기 갱신을 선택했다. 학습 평균 누적 yaw RMSE는 309.56°에서 289.95°로 줄었고, 별도 검증 평균은 312.32°에서 310.38°로 바뀌었다. 별도 검증 상대수심 RMSE는 136.37 cm에서 137.85 cm로 늘었다. 초기 후보 재생은 기존 출력의 동일 입력 기록을 재사용했고, 계수를 바꾼 후보와 선택 후 검증은 새로 재생했다.

`random` 전용 모드는 기존 수동 yaw 프로필을 입력 계수로 쓰지 않았다. 자동 방향별 후보(103.43°)에서 준비 반복으로 61.16°, 첫 미세 후보에서 37.04°, 후속 반복으로 29.47°에 도달했다. 각 단계의 동일한 입력에 대해 이미 계산한 재생은 보존하고 재사용했다. 마지막 선택은 무접촉과 시드 대비 5초 yaw 변화 RMSE 증가 5% 이내 조건을 충족한 실제 재생 중 누적 yaw RMSE가 가장 낮은 후보다. 기존 yaw 보정은 동일한 2,036개 표본에서 28.91°였다.

선택한 프로필을 고정하고 네 검증 주행에서 얻은 평균 누적 yaw RMSE는 전용 자동 보정 436.71°, 기존 yaw 보정 407.68°, 일반 자동 보정 310.38°였다. 전용 자동 보정의 `serpentine` 누적 yaw는 662.88°로 기존 yaw 보정 785.69°보다 작았다. 전용 모드는 `random` 재현용이며 일반 자동 보정 프로필과 분리한다.
