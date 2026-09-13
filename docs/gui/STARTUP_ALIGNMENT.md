# 시작 시 자세 정렬 대기

일반 GUI closed-loop 실행은 `UUV_STARTUP_ALIGNMENT_HOLD=1`을 기본으로 사용한다.
기존 GUI 프로세스에는 변경이 소급 적용되지 않으므로 GUI를 종료한 뒤 다시 실행한다.

1. 시뮬레이션을 시작하면 초기 위치와 자세를 유지한다. 물리 시간과 센서 발행은 계속된다.
2. ArduSub의 최신 EKF_STATUS_REPORT에서 EKF_ATTITUDE 비트가 설정될 때까지 ARM을 거절한다.
   ArduSub 4.1.2에서 이 비트는 EKF 건강 상태와 tilt alignment 완료를 포함한다.
   보고 수신 후 5초 이상 지났으면 준비 상태로 인정하지 않는다. 고정 대기 시간이 아니다.
3. 정렬 전 ARM 요청은 예약하지 않는다. 정렬 완료 후 사용자가 ARM을 다시 눌러야 한다.
4. 정렬 완료와 실제 armed 상태가 함께 확인되면 자세 고정을 해제하여 조종할 수 있다.
   이후 DISARM은 초기 위치로 복귀시키지 않는다.

이는 초기화용 가상 고정 장치이며 실제 중성부력이나 자세 제어 성능의 검증 결과가 아니다.
고정 중 데이터는 작업 시연으로 사용하지 않는다. 명시적 real-start 재생 프로필은 기존 동작을 유지한다.
실행 중 FCU만 재부팅하는 경우 이전 EKF 상태는 무효화하지만 해제한 고정을 다시 걸지는 않는다.

## 검증

`uuv_mujoco/current/tools/test_startup_alignment.py`는 오래되거나 잘못된 EKF 상태,
조기 ARM 거절, FCU 부팅 시각 역행, 깊이 인자 없는 초기 자세 고정,
정렬 완료 후 미무장 대기 및 ARM 후 해제를 검사한다.
MuJoCo 물리 시간을 1초 진행하며 외력을 가해도 고정 중 위치·자세와 속도가 유지됨을 확인한다.
고정 적용 수정 전에는 해당 회귀 시험이 실패하는 것도 확인했다.

MuJoCo 3.12 / ROS Humble 환경에서 다음 29개 시험이 통과했다.

```bash
source /opt/ros/humble/setup.bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .venv/bin/python -m pytest -q \
  uuv_mujoco/current/tools/test_startup_alignment.py \
  uuv_mujoco/current/tools/test_gui_arm_rc3_sequence.py \
  uuv_mujoco/current/tools/test_fcu_cadence.py
```

실행 중 GUI–SITL에서 EKF flags=167로 정렬 완료, ARM 성공 및
`initial depth hold released (startup:aligned_and_operator_armed)` 로그를 확인했다.
초기에 INS 검사 비트를 강제로 추가하여 `3D Accel calibration needed`로
ARM이 거절되는 결함이 있었다. 이 비트 강제 추가를 제거하고 기존
ARMING_CHECK 설정을 보존한다. 실행 중 값도 210에서 기존 194로 복원하여
성공 응답을 확인했다. GUI/시뮬레이터의 EKF 정렬 조건은 그대로 유지한다.
외부 직접 MAVLink 요청 자체를 이 GUI 조건이 모두 차단하는 것은 아니며,
시뮬레이터 자세 고정 해제에는 정렬과 armed 상태를 모두 요구한다.

재실행 후 ARM을 정렬 전에 눌러 자동으로 나중에 무장되지 않는지 확인하고,
준비 상태에서 ARM을 다시 눌러 고정 해제 및 저속 전후진을 확인한다.
실행 전후 실제 roll/pitch와 FCU 추정 roll/pitch를 비교해야 기울어짐 개선을 판단할 수 있다.
