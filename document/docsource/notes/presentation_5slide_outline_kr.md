# MuJoCo UUV 발표용 5장 구성

## Slide 1. 문제 정의와 목표

### 슬라이드 문장
- 목표는 ArduSub/ROS2 제어 스택과 연결되는 실시간 UUV 시뮬레이터를 만드는 것입니다.
- 기존 단순 모델은 `부력 + CoB 토크 + 선형 drag + 단순 thrust` 수준이라 실제 수중 응답과 차이가 있었습니다.
- 그래서 MuJoCo를 강체 적분 엔진으로 사용하고, 그 위에 custom 수중 동역학 계층을 추가했습니다.

### 발표 멘트
기존에는 물속 효과를 단순 부력과 선형 감쇠 정도로만 근사했기 때문에, 실제 ROV처럼 무겁고 감쇠된 느낌이나 자세 복원 특성이 충분히 나오지 않았습니다. 이번 작업의 목표는 MuJoCo의 빠른 rigid-body solver는 유지하면서, 그 위에 실제 수중체에 가까운 hydrodynamics와 thruster 모델을 얹어서 ArduSub, ROS2, 조이스틱 제어까지 연결 가능한 형태로 만드는 것이었습니다.

### 슬라이드에 넣을 핵심 키워드
- Real-time UUV simulation
- MuJoCo + custom hydrodynamics
- ArduSub SITL / ROS2 integration

---

## Slide 2. MuJoCo 공식 fluid model과 현재 구조

### 슬라이드 문장
- MuJoCo는 공식적으로 `inertia-based` 와 `ellipsoid-based` fluid model을 제공합니다.
- 하지만 현재 프로젝트는 MuJoCo built-in fluid model을 그대로 쓰지 않고, Python에서 hydrodynamic wrench를 계산해 `xfrc_applied` 로 주입합니다.
- 최근에는 `sim_real` 프로파일에 ellipsoid-based baseline 생성기를 추가해 형상 기반 계수 생성까지 반영했습니다.

### 발표 멘트
MuJoCo 공식 문서에는 inertia model과 ellipsoid model이 있습니다. 하지만 저희 구조는 그 모델을 scene에서 바로 켜는 방식이 아니라, MuJoCo를 rigid-body solver로 사용하고 Python 런타임에서 수중 힘과 토크를 계산해 body에 외력으로 넣는 구조입니다. 다만 최근에는 완전히 hand-tuned coefficient만 쓰는 대신, equivalent ellipsoid에서 added mass와 damping baseline을 계산해 초기값을 만들고, 그 위에 tuning scale을 곱하는 hybrid 방식으로 발전시켰습니다.

### 슬라이드에 넣을 핵심 키워드
- Built-in fluid model != current runtime
- `xfrc_applied` external wrench
- Ellipsoid baseline + coefficient tuning

---

## Slide 3. sim2real gap을 줄이기 위해 추가한 물리 항

### 슬라이드 문장
- 부력만이 아니라 `CoB restoring torque` 를 추가해 roll/pitch 복원 특성을 반영했습니다.
- `added mass`, `added-mass Coriolis`, `quadratic damping`, `relative-current drag` 를 추가했습니다.
- 추진기는 T200 공개 성능표 기반 비선형 thrust curve와 `reverse asymmetry`, `gain calibration` 을 사용합니다.

### 발표 멘트
sim2real gap을 줄이기 위해 가장 먼저 추가한 것은 CoB 기반 restoring torque입니다. 이것이 없으면 실제처럼 자세가 복원되는 느낌이 잘 안 나옵니다. 그 다음으로 added mass와 added-mass Coriolis를 넣어서 수중에서 주변 유체를 함께 가속해야 하는 효과를 반영했고, 감쇠는 단순 선형 drag에서 quadratic damping까지 확장했습니다. 추진기 쪽은 PWM을 단순 선형 force로 두지 않고 Blue Robotics T200 공개 성능 데이터를 사용했고, 후진 thrust 비대칭과 thruster별 gain calibration도 따로 반영했습니다.

### 슬라이드에 넣을 표제형 문구
- Hydrostatics: buoyancy + CoB torque
- Hydrodynamics: added mass + Coriolis + quadratic damping
- Actuation: T200 nonlinear thrust + asymmetry + calibration

---

## Slide 4. 실제 ROS bag 기반 검증 결과

### 슬라이드 문장
- 실제로 simulator를 실행한 뒤 ROS 서비스로 mode를 바꾸고, `RC override` step 입력을 넣어 `ros2 bag` 으로 응답을 측정했습니다.
- `MANUAL`, `STABILIZE`, `ALT_HOLD` 모두에서 전진 응답이 형성되었습니다.
- yaw 및 heave 입력도 실제 응답으로 검증했습니다.

### 발표 멘트
이 부분은 설명용 그래프가 아니라 실제 측정 결과입니다. simulator를 실행한 뒤 `MANUAL -> arm -> forward step -> yaw step -> STABILIZE forward -> ALT_HOLD forward -> ALT_HOLD heave` 순서로 입력을 넣고, ROS bag으로 odometry, IMU, depth, state를 기록했습니다. 측정 결과 MANUAL forward 평균 surge 속도는 0.134 m/s, STABILIZE는 0.128 m/s, ALT_HOLD는 0.137 m/s로 세 모드 모두 전진 authority가 유지됐습니다. 또 yaw step에서는 평균 -0.468 rad/s의 yaw rate가 나왔고, ALT_HOLD heave step에서는 depth가 약 -0.068 m 변했습니다. 즉 mode 전환, 자세 제어, 전진/회전/상하 응답이 모두 실제 토픽 상에서 검증됐습니다.

### 슬라이드에 넣을 대표 수치
- MANUAL forward: mean `0.134 m/s`
- STABILIZE forward: mean `0.128 m/s`
- ALT_HOLD forward: mean `0.137 m/s`
- MANUAL yaw: mean `-0.468 rad/s`
- ALT_HOLD heave: depth change `-0.068 m`

### 같이 보여주면 좋은 그림
- [actual_mavros_step_responses.png](/Users/kanghyunmin/Desktop/uuv_sim/document/figures/actual_mavros_step_responses.png)
- [actual_mavros_mode_comparison.png](/Users/kanghyunmin/Desktop/uuv_sim/document/figures/actual_mavros_mode_comparison.png)

---

## Slide 5. 기여와 향후 계획

### 슬라이드 문장
- 현재 시스템은 MuJoCo 위에 `shape-based ellipsoid baseline + coefficient-based 6-DOF hydrodynamics` 를 결합한 custom underwater simulation stack입니다.
- 실시간 ROS2/ArduSub 연동과 bag 기반 응답 검증까지 완료했습니다.
- 다음 단계는 실기 로그 기반 coefficient identification과 disturbance model 확장입니다.

### 발표 멘트
정리하면, 이번 작업은 MuJoCo를 단순 visualization tool이 아니라 실제 제어 스택과 연결되는 UUV simulation stack으로 확장한 것입니다. 핵심은 형상 기반 ellipsoid baseline과 coefficient-based 6자유도 hydrodynamics를 결합한 custom 모델을 만들고, 이를 실제 ROS 경로에서 검증했다는 점입니다. 다음 단계는 실기 velocity, yaw, depth 로그와 simulator 응답을 직접 비교해서 coefficient를 더 정밀하게 식별하고, current, turbulence, wave 같은 disturbance model을 확장하는 것입니다.

### 마지막 한 줄 요약
- MuJoCo rigid-body solver 위에 실제 제어 스택과 연결되는 custom underwater physics layer를 구현하고, ROS bag 기반으로 1차 동작 검증까지 완료했습니다.

---

## 발표 팁

- Slide 2에서는 “MuJoCo 공식 fluid model을 그대로 쓴 것이 아니다”를 분명히 말하는 것이 중요합니다.
- Slide 3에서는 “왜 이 항이 필요한가”를 sim2real gap 관점에서 설명하면 설득력이 높습니다.
- Slide 4에서는 “이건 추정 그래프가 아니라 실제 bag 기반 측정값”이라고 먼저 말하는 것이 좋습니다.
- 시간이 부족하면 Slide 3과 Slide 4에 가장 오래 머무르면 됩니다.
