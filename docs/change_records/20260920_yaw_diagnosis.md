# Yaw 원인 분리 실험 — 2026-09-20

사용자 요청: 주기 문제를 포함해 중립 회전과 yaw 해제 후 역회전 원인을 확실해질 때까지 좁힌다.

## 확인된 원인

1. **회전 잔류: 낮은 저속 회전 감쇠.** 사용자 실행 마지막 중립 구간에서 모든 모터 1500 PWM, 추진기 yaw 토크 0. 실제 각속도는 0.00227~0.00267 rad/s, 감속은 0.0000055~0.0000078 rad/s²에 불과했다. FCU를 제거한 물리 실험에서도 초기 0.01 rad/s가 30초 후 0.00637 rad/s로 남았다. 진단용 점성 저항 0.2 N·m·s/rad 추가 시 0.0000191 rad/s로 감소했다. 추가 저항값은 실측 보정값이 아니며 운영 설정에 반영하지 않았다.

2. **FCU가 회전을 표시하지 않는 이유: 무방향센서 설정의 정지 판정.** 현재 실제 FCU 설정은 COMPASS_USE=0, EK3_SRC1_YAW=0. EKF가 디스암·저각속도 상태를 정지로 분류하고 STATIC yaw 관측을 융합한다. baseline 로그의 XKFM.OGNM=1 확인. 마지막 20초에 실제 −8.227° / FCU +1.156°. EK3_OGNM_TEST_SF=0으로 정지 판정만 진단용 비활성화 시 OGNM=0이며 실제 −8.442° / FCU −9.671°. 이 변경은 동작 원인을 구분하기 위한 실험이며 센서 보정이나 운영 해결책으로 채택하지 않았다.

3. **yaw 해제 후 역회전: 현 물리 모델과 방향 유지 제어의 응답 불일치.** STABILIZE는 yaw 중립 후 250ms 감속하고 마지막 방향을 유지한다. baseline에서는 이때도 약 22°/s가 남고, FCU RATE.YDes가 반대 부호로 전환된다. 실제 역회전 최고치는 3회 9.51/9.63/9.85°/s. 추진기 갱신 100→400Hz 비교에서도 9.68/10.43/9.97°/s로 해결되지 않았다. ATC_ANG_YAW_P만 4.5→1.0으로 낮추면 1.66/2.06/1.78°/s로 줄었다. 그러나 해제 10초 뒤 순회전각이 약 6°에서 약 10°로 늘어 제동/방향 유지의 절충이 있으므로 기본 PID는 그대로 유지했다. 실물과 일치하는 계수를 확정한 실험은 아니다.

4. **별도 실제 타이밍 결함: DemoReset의 FCU 센서 전송 누락.** 정상 raw PWM 경로는 `publish_ros or sitl_enabled`로 매 physics tick 센서 경로를 호출하지만, reset 경로는 `publish_ros`에서만 호출했다. 리셋 2초+복귀 0.5초 동안 FCU 입력이 일반 ROS 발행 주기에 제한된다. 사용자 로그의 긴 루프와 2.5초 리셋 구간이 겹친다. 동일 조건 live reset 3회 A/B: 수정 전 NLon 합계 753회, MaxT 10,000µs. 수정 후 NLon 0회, MaxT 2,500µs. 원시 PM은 summary.json의 reset_before/after performance에 저장했다.

## 적용한 코드 변경

`uuv_mujoco/current/sim/runtime/demo_reset.py`: reset 중에도 SITL 센서 feed를 매 physics tick 유지한다. QGC 영상은 기존 telemetry gate를 유지한다. 모델의 물리·PID·센서 설정은 변경하지 않았다.

`tools/test_demo_reset.py`: 일반 ROS 발행이 없는 tick에서도 SITL 센서 feed가 수행되는 회귀 테스트를 추가했다. 수정 전 1 fail/1 pass, 수정 후 전체 9 pass/1 skip. skip은 별도 ROS 통합 테스트이며 실제 reset A/B 실험은 따로 수행한다.

## 실험 조건과 한계

- 별도 Docker `uuv-yaw-diagnosis`, ROS_DOMAIN_ID=156. 운영 GUI/학습 데이터와 분리.
- 같은 설치된 ArduSub 바이너리, 실제 로그의 제어 계수, bag0402_effective 물리 모델. 격리용 clearance 수조에서 실행. 운영 수조 벽·부표 접촉 효과를 재현한 것은 아니다.
- 물리 800Hz, FCU 목표 400Hz. thruster 100/400Hz는 런타임 로그로 확인.
- PWM 1600 / 1400 / 1600 각 2초, 각 해제 후 12초, 마지막 DISARM 30초 관찰. 코드·실제 파라미터·FCU 로그·원시 probe/forces를 보관.
- normal/static_off 사이 시작시간과 센서 잡음 seed 진행량이 완전히 같지는 않다. 원인 방향과 크기는 원시 로그 및 OGNM 분기로 뒷받침된다.
- ATC_ANG_YAW_P=0만으로는 sqrt controller가 꺼지지 않는다. `no_heading2`는 유효한 방향 유지 제거 실험으로 쓰지 않았다. `rate_only`도 입력 shaping까지 바뀌어 운영 후보에서 제외했다.
- 기존 18개 학습 데이터와 모델은 보존했다. 신규 수집/학습은 하지 않았다.

## 재현과 결과

- `outputs/yaw-diagnosis-20260920/probe.py`, `start_stack.py`: 격리 실행/입력/센서 기록.
- `run_remaining.py`: gain 및 reset A/B 실행. 기존 결과 디렉터리가 있으면 덮어쓰지 않는다.
- `analyze.py <case>`, `summarize.py`: 해제 응답·실제 FCU 파라미터·주기 집계.
- `coast.py`: 무제어·무추력 물리 감쇠 실험.
- `diagnosis.png`: 비교 그래프. 실제 yaw는 /sim/odom이며 진단 관측에만 사용, 제어/학습 입력으로 사용하지 않았다.
