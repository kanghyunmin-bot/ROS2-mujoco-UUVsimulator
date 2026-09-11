# 실물 사각 경로 bag 기반 검증·제한적 튜닝

## 결과

수조를 따라 주행하고 대략 출발점으로 복귀했다는 사용자 설명을 반영했다. 현재 test tank는 5.49×2.74×1.32 m지만 실측 정답이 아닌 참고 치수다. **수압 관측 변동과 유효 출력 주기를 재현하는 선택형 시뮬레이션 프로필을 추가했고 회귀 시험을 완료했다.** 물리 계수·위치추정 정확도·VLA 전이 성공의 보정을 완료한 것은 아니다.

원본 bag, 현재 기본 프로필, 기체 형상·질량·부력·추력·항력·센서 장착값은 보존했다. 수조 치수에 궤적을 맞추거나 시작점으로 강제로 닫지 않았다. 이번 변경은 새 프로필, fitting 도구, 실행 wrapper, 회귀 시험, 이 문서다. 기존 미커밋 작업과 별도 신규 파일로 구분된다. 자동으로 GUI 설정을 바꾸거나 ROS에 명령을 replay하지 않았다.

## 궤적·좌표계 검증

`outputs/real-bag-tuning-20260911/trajectory_analysis.py`는 recorded odometry와 DVL+IMU 적분을 비교한다. quaternion SLERP는 오프라인 비교용이며 온라인 정책 입력용 처리가 아니다. DVL 유효 샘플만 사용하며 짧은 invalid 구간을 건너뛴 적분에는 모델 오차가 존재한다. sensor lever-arm correction이나 DVL 장치 시각 보정은 하지 않았다.

시작 위치는 10~17초 중앙값, 복귀 위치는 88~100초 중앙값이다. 실제 시작·종료 위치는 독립 측정되지 않았다.

| 경로 | 시작·종료 XY 차이 | 주행 XY 범위 (세계축) | 18.1~88.1초 XY 길이 |
|---|---:|---:|---:|
| recorded `/odometry/filtered` | 0.610 m | 2.374×4.533 m | 13.309 m |
| recorded MAVROS local odom | 0.727 m | 2.028×4.874 m | 13.167 m |
| DVL raw를 FLU로 간주 + IMU 적분 | 0.606 m | 2.362×4.542 m | 13.187 m |
| DVL FRD→FLU + IMU 적분 | 0.684 m | 2.029×4.877 m | 13.171 m |

IMU 18~73초 yaw 변화는 -6.3183 rad, 약 -362°다. 한 바퀴 사각 주행과 정성적으로 일관된다. 각 경로는 heading 정렬/스케일 fitting 없이 자신들의 시작점만 빼서 표시했다. 약 0.61 m는 정확도 수치가 아니라 **추정 위치의 시작·종료 차이**다. 실제 복귀 오차와 추정 오차가 섞인다.

bag의 identity `base_link→dvl` TF 및 raw와 동일한 twist 벡터를 사용한 EKF 경로는 raw-as-FLU 적분과 거의 일치한다. MAVROS 경로는 FRD 변환 적분과 더 가깝다. **당시 두 위치추정 경로가 서로 다른 DVL 축 해석을 했을 가능성**을 뒷받침하지만, 실제 장착 방향/장치 설정 없이 어느 궤적이 물리 정답인지 확정하지 않았다. 사각형처럼 보인다는 이유만으로 더 그럴듯한 경로를 선택해 현재 정상 FRD 계약을 바꾸지 않았다.

정지 후기 EKF 위치 변화(88~100초 중앙값 대비 135~140초 중앙값)는 약 [-0.00776,+0.03078,+0.00159] m다. 독립적 zero velocity가 확인되지 않았으므로 이 값을 전부 바이어스로 제거하지 않았다.

## 적용한 튜닝

새 프로필: `uuv_mujoco/current/config/sensor_models/imu_bar30_bag_20260402.json`

| 항목 | 기본 prior | bag 전용 프로필 | 근거 |
|---|---:|---:|---|
| Bar30 white_noise_std_pa | 8 Pa | 24.02025 Pa | 90~115초 수압 인접 차분의 분산, 기존 20 Pa 양자화 분산 보정 |
| Bar30 nominal/capture schedule | 10 Hz | 2 Hz | 당시 기록된 pressure/depth 출력률 재현용 유효 스케줄 |
| IMU FCU capture | 400 Hz | 400 Hz 유지 | ROS 기록률로 내부 IMU 주기를 추정하지 않음 |
| IMU/기타 sensor model·드리프트·온도·offset | 기존 prior | 유지 | 단일 짧은 자세/세션으로 식별 불가 |

`calibration_status=unvalidated_prior`를 유지하고 `calibration_scope=partial_empirical_observation_envelope_not_hardware_calibration`을 추가했다. 2 Hz는 **관측된 출력 계층을 재현하는 유효 스케줄**이며 실물 ADC 변환률을 측정했다는 의미가 아니다. 현재 센서 모델은 압력 캡처가 SITL에 전달되는 구조이므로 이 선택형 프로필의 압력 hold가 제어 응답에 영향을 줄 수 있다. 원래 하드웨어 내부 주기를 재현했다고 주장하지 않는다. 현재 원하는 10 Hz 수집의 default로 채택하지 않는다.

수압 잔차는 센서 잡음 외에도 미세 움직임·수면 변동·진동·필터링을 포함할 수 있다. 물리 센서 noise density 대신 **이번 배치의 수압 관측 변동 수준**을 맞춘 것이다. 가속도 8.9 m/s², DVL 고정 바이어스, 카메라 시간차, RCOut wrap은 이 프로필로 흉내 내거나 보정하지 않았다.

## 분리 구간 검증과 회귀

- fitting: 90~115초 수압 50개.
- held-out: 115~140초 수압 50개. fitting에 사용하지 않았다. 같은 세션의 인접 구간이므로 독립적인 실물 검증은 아니다.
- 통계량: `std(diff(pressure), ddof=1)/sqrt(2)` [Pa]. 이전 보고서의 ddof=0 값과 수치가 약간 다르다.
- 실제 fitting 24.7044 Pa, held-out 24.2550 Pa.
- 같은 2 Hz·50샘플, 동일 조건의 100개 seed로 기존/변경 모델을 비교했다.

| 평가 | 기존 | 튜닝 |
|---|---:|---:|
| 모델 차분 통계량 중앙값 | 9.5743 Pa | 24.1945 Pa |
| 실물 held-out 통계량과 절대 차이 | 14.6808 Pa | 0.0606 Pa |

이는 하나의 통계량을 맞춘 결과이며 전체 분포·자기상관·시계·센서 정확도 일치를 입증하지 않는다. 튜닝 출력의 seed별 p05~p95는 19.13~29.86 Pa다. 별도 세션이 생기면 프로필을 재평가해야 한다.

회귀 시험은 기존 프로필로 실행해 실패(상대 차이 60.5%, 허용 20%)를 확인한 뒤 새 프로필로 통과했다. 추가로 bridge runtime에서 5초 동안 pressure delivery의 capture timestamp 간격 0.5초와 IMU 400 Hz 설정 보존을 확인했다.

- 새 회귀 시험 3개 PASS.
- 기존 IMU/Bar30 모델·runtime·publisher 시험 33개 PASS.
- MuJoCo 3.8.0 실제 test-tank 생성·load·기존 계약 검사 PASS. 전체 SITL/ROS 궤적 replay를 수행한 것은 아니다.
- 실행 wrapper에서 새 프로필이 실제 loader로 선택되는 것을 확인.
- critical lint와 whitespace 검사 수행.

## 시도했으나 적용하지 않은 운동 계수

`fit_response.py`에서 큰 회전·sway 구간을 제외하고 `dv/dt=a*requested_surge-b*v-c*|v|v`를 fitting했다. 20~45초 fitting 153개, 45~70초 held-out 153개를 사용했고 DVL 속도를 0.1초 격자 및 0.9초 Savitzky–Golay 창으로 처리했다. 이는 전체 MuJoCo 모델 fitting이 아닌 식별 가능성을 보는 1축 근사다.

- fitting RMSE 0.0594 m/s², held-out 0.1317 m/s².
- held-out zero-acceleration baseline 0.1687 m/s²보다 일부 개선되지만 fitting 대비 오차가 2.22배.
- RC가 요청 명령이고 ALT_HOLD controller, 모터 응답, 물살·테더·벽·횡운동을 분리하지 못한다.
- 현재 MuJoCo 물리 모델과 비교한 개선 수치가 아니므로 a/b/c를 thrust/drag에 이식하지 않았다.

정확한 실물 시험이 어려운 상황에서도 이 한계는 사라지지 않는다. 이번 자료로 파라미터를 많이 바꾸는 것보다, 뒷받침되는 수압 관측 프로필을 명시적으로 추가하는 것이 재현 가능한 결과다.

## 실행

저장소 루트에서 기존 시뮬레이션 실행 명령 앞에 wrapper를 붙인다. 테스트 수조 선택과 다른 실행 인자는 기존 명령 그대로 둔다.

```bash
uuv_mujoco/current/tools/run_bag0402_sensor_profile.sh \
  ./uuv_mujoco/start_sitl_mujoco.sh --ros2-real-pkg-compat -- \
  --scene scenes/tank_current_scene.xml --profile research_pool --fluid-model current
```

위 scene은 기본 scene이며 GUI의 test_tank 레이아웃을 자동 선택하는 명령은 아니다. GUI test_tank를 사용하려면 기존 GUI 실행 명령 자체를 wrapper 뒤에 넣고 test_tank를 선택한다. wrapper는 센서 프로필 환경 변수만 전달한다. 컨테이너에서 실행한다면 **wrapper도 workspace가 마운트된 컨테이너 안에서 실행**해야 한다. 기존 per-sensor 환경 변수 override가 있으면 모호한 설정을 조용히 적용하지 않고 거부한다. 기본 프로필로 돌아갈 때는 wrapper 없이 기존 명령을 사용한다.

재 fitting:

```bash
/home/khm/robotics/IsaacLab/isaaclab.sh -p \
  uuv_mujoco/current/tools/fit_bag_pressure_profile.py \
  --numeric_npz outputs/real-bag-audit-20260911/numeric.npz \
  --profile_json outputs/real-bag-audit-20260911/profile.json \
  --source_manifest outputs/real-bag-audit-20260911/source.json \
  --output_dir outputs/real-bag-tuning-20260911
```

고정 구간이 검토된 April 2 bag에만 해당하므로 다른 bag의 manifest는 거부한다. 다른 세션에는 정지 구간 선택부터 다시 해야 한다. 생성한 profile JSON을 기본 설정에 자동 덮어쓰지 않는다.

회귀:

```bash
/home/khm/robotics/IsaacLab/isaaclab.sh -p uuv_mujoco/current/tools/test_bag0402_pressure_profile.py
/home/khm/robotics/IsaacLab/isaaclab.sh -p -m unittest discover \
  -s uuv_mujoco/current/tools -p 'test_imu_bar30*.py'
```

## 증거와 남은 범위

`outputs/real-bag-tuning-20260911/`에 `trajectory_metrics.json`, `trajectories.png`, `response_fit.json`, `pressure_fit.json`, before/after 시험 로그와 원본 분석 스크립트가 있다. 현재 bag은 사각 주행·센서 검증 자료이며 두 카메라 작업 VLA 시연으로는 여전히 부적합하다. 이 튜닝으로 실물 전이·항법 정확도·부표 작업 성능이 검증됐다고 표현하면 안 된다.
