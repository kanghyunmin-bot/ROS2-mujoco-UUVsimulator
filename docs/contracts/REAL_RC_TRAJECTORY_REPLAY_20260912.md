# 실물 RC → SITL/MuJoCo 실제 주행 비교 (2026-09-12)

## 수행 결과

실물 bag의 RC override와 무장·모드 전환을 실제 ROS 2 Humble → MAVROS → ArduSub V4.1.2 → JSON servo → MuJoCo 경로로 재생했다. 실물 센서를 다시 적분한 그림과 다르게 **이번 그림에는 실제로 움직인 시뮬레이션 ground truth와 시뮬레이션 ROS EKF 궤적이 포함된다.** 센서 ground truth는 평가 기록에만 사용하고 외부 ROS EKF 입력에는 주입하지 않았다.

현재 설정은 회전 시점·전체 회전량을 비교적 비슷하게 재현하지만, 직진 이동 및 궤적은 차이가 크다. 이 한 번의 비교로 추력·항력 중 무엇이 틀렸는지 분리할 수 없고, 현재 실물 전이 정확도가 검증되었다고 볼 수 없다.

## 실험 조건

- 원본 `bag_2026-04-02_21-46-20.zip`, SHA256 `195aca87161fb180bc7c7ee0574523f7840399e439f7b1d245313d8d2faf2d52`.
- bag 기록 시각 기준 10~145초: 정지 후보 초기 구간, 무장 ALT_HOLD 주행, MANUAL 전환, 비무장 후기 구간. 파일 첫 10초와 마지막 약 2초는 제외.
- 원본 RC 18채널 전체와 메시지 순서를 보존하여 `/clock` 기준 재생. 10초 직전 최신 명령을 초기 유지 명령으로 사용했다.
- 무장/모드 전환은 bag의 State 기록 시각에 ROS 서비스로 요청했다. 실제 FCU 상태 확인에는 heartbeat 보고 지연이 있다.
- isolated Docker bridge network, ROS_DOMAIN_ID=89. 실물 연결/호스트 포트 공개 없음. 완료 후 시험 컨테이너 중지·삭제.
- 실제 135초 시뮬레이션 실행, 약 135초 wall time. 영상 렌더링을 제외한 headless 시험이므로 이전 GUI 성능과 직접 비교하지 않는다.
- `test_tank` 5.49×2.74×1.32 m, 현재 CAD 기체, `research_pool_distributed` + distributed 유체 모델.
- 시작 XY=(-2.0,0.9)m, base depth=0.6m, RPY=(0,0,0). **정확한 실물 시작 위치·수심이 없어 설정한 가정**이다.
- 준비/비무장 초기에는 depth hold로 수심을 고정하고 첫 무장 요청 때 해제했다. 실제 정지 조건·테더를 모델링한 것은 아니다.
- 현재 test tank의 노란 부표·pinger 구성이 포함된다. 원래 항법 시험의 동일 객체 존재는 확인되지 않았다. 후기 buoy net 이벤트도 기록돼 후기 거동 해석에 제약이 있다.
- 수압 튜닝 선택형 2 Hz 프로필을 사용하지 않고 기본 센서 prior로 시험했다.
- 현재 launcher의 realrobot contract + 기본 제어 파라미터를 생성하여 적용했다. 과거 bag의 정확한 FCU 파라미터와 같다고 보증할 수 없다.
- 8 thruster FRAME_CONFIG=2, 400 Hz scheduler, 현재 모터 방향·RC mapping 사용.
- 시험용 transport 변경: MAVROS sysid1에 맞춘 SYSID_MYGCS=1, SERIAL0=UDP14551, MuJoCo telemetry=UDP14660. ARMING_CHECK=0은 소프트웨어 시험용이며 실물 arming 안전성 검증은 아니다.
- 이 ArduSub의 HAL 매핑은 SERIAL2 설정을 UART index3에 연결하므로 `--serial3=udpclient:127.0.0.1:14660`을 사용했다. 소스와 실행 중 V4.1.2 식별을 확인했다.

## 그림 범례와 정렬

`outputs/rc-trajectory-replay-20260911/real_vs_sim.png`

| 색 | 데이터 |
|---|---|
| 초록 | REAL — 실물 ROS `/odometry/filtered` 추정 |
| 주황 | REAL — 실물 FCU `/mavros/local_position/odom` 추정 |
| 파랑 | SIM — MuJoCo `/mujoco/ground_truth/pose` 실제 시뮬 위치 |
| 보라 | SIM — 시뮬 ROS `/odometry/filtered` 추정 |

10~17초의 위치·heading을 기준으로 각 궤적의 원점과 초기 방향만 맞췄다. 거리 스케일 fitting, 사각형에 맞추기, 강제 폐루프, 시간 warp는 하지 않았다. 실물의 독립 ground truth는 없으므로 아래 오차는 **실물 추정 궤적 대비 차이**다. 시뮬 FCU 위치 스트림은 이 실험에서 유효 궤적으로 확보되지 않아 그림에 포함하지 않았다.

## 최종 수치

| 지표 | 실물 ROS EKF | 실물 FCU 추정 | 시뮬 ground truth | 시뮬 ROS EKF |
|---|---:|---:|---:|---:|
| 18.1~88.1초 XY 경로 길이 | 13.309m | 13.167m | 16.113m | 16.334m |
| 시작~복귀 XY 차이 | 0.610m | 0.728m | 0.869m | 0.862m |
| 73초 시점 초기 대비 yaw 변화 | -362.17° | -362.21° | -365.68° | -365.69° |

복귀 위치는 88~100초 중앙값, 시작은 10~17초 중앙값이다. 실제로 얼추 복귀했다는 정보만 있으므로 복귀 차이를 전부 추정 드리프트라고 단정하지 않는다. 경로 길이는 센서 샘플률·잡음 영향을 받으며 ground-truth 거리의 실물 측정치가 아니다.

18.1~73초, 동일 bag 시각의 0.1초 격자에서 선형 보간한 XY 비교:

- 시뮬 ground truth vs 실물 ROS EKF: RMSE **1.005m**, 최대 **2.402m**.
- 시뮬 ground truth vs 실물 FCU: RMSE **1.185m**, 최대 **2.792m**.
- 이는 통합 모델/제어기/시작 조건/실물 추정기 차이를 포함한다. 순수 물리 엔진 오차나 위치추정 정확도로 인용하지 않는다.
- 추정 경로 길이 기준 시뮬은 실물 EKF보다 약 21% 길다. 수조 벽 근처의 제약과 가정한 초기 위치도 영향을 주므로 추력 gain을 즉시 21% 줄일 근거가 되지 않는다.

## 명령 전달 검증

최종 baseline은 RC **3,500회** 발행, 초기 유지 명령 이후 인덱스 누락·순서 역전 **0개**. 스케줄된 bag 시각 대비 ROS 발행 지연 중앙값 약1.26ms, p95 약2.37ms, 최대 약2.50ms. `/clock` 해상도 안에서 가까운 명령들은 같은 simulation tick에 순서대로 발행했다. 이는 **발행 스케줄 검증**이며 FCU가 모든 개별 RC 메시지를 반드시 별도 control step에 채택했다는 증명은 아니다.

무장·모드 서비스 응답 성공, 실제 FCU State 전환, MuJoCo heartbeat safety gate의 armed/mode 전환, 유한한 실제 이동과 EKF 위치를 검사했다. real RCOut 헤더의 약4295초 시계 이상 때문에 실제/시뮬 RCOut의 개략 비교는 수신/기록 시각을 사용했고 `comparison.json`에 채널별 RMSE를 남겼다. 피드백은 실물2Hz라 정확한 명령 지연/추력 보정에 사용할 수 없다.

## 발견·수정한 구현 문제

`bridge/sitl_vehicle_state_extract.py`가 실제 transport binding에 없는 `owner._log_vehicle_state_change`를 호출하고 예외를 숨겼다. armed/mode 필드는 그 전에 갱신되지만 마지막 상태 추적·로그는 갱신되지 않아 heartbeat readiness 관측을 방해했다. 이미 있는 `log_vehicle_state_change(owner,...)` helper를 직접 호출하도록 수정했다.

`test_sitl_heartbeat_state_recording.py`는 실제 SitlTransport binding surface를 사용한다. 수정 전 last-state 갱신 실패를 확인하고 수정 후 PASS. 기존 GUI arm/mode contract 검사 PASS, critical lint 및 diff whitespace 검사 PASS. 이 수정은 추진력이나 mode 명령 값을 바꾸지 않는다.

초기 실패/진단 실행도 보존했다:

- heartbeat 링크 누락, 불완전 controller params, 이전 child process 포트 충돌, UART 매핑 및 로그 진단은 성능 비교에서 제외.
- `diagnostic-latest-sample-replay/`는 전체135초 움직였으나 가까운 RC274개를 최신 값 선택으로 생략한 예비 실행이다. 최종 비교는 이를 고쳐 누락0으로 다시 실행한 `baseline/`만 사용한다.
- `debug_runner.py` 계측은 진단용 파일이며 최종 실행에 사용하지 않았다.

## 재현

`outputs/rc-trajectory-replay-20260911/README.md`에 isolated container 재실행 절차가 있다. 핵심 파일:

- `start_stack.py`, `launcher-params.parm`, `baseline/params.parm`: 실제 실행/제어 설정.
- `test_tank.xml`, `manifest.json`, `source-snapshot/`: scene·환경·소스 근거.
- `commands.json`, `replay.py`: 실물 RC/State 일정 및 ROS 재생.
- `baseline/replay.json`: 실제 발행 시각/인덱스, 서비스 응답, 상태, RCOut, sim GT/EKF.
- `compare.py`, `comparison.json`, `real_vs_sim.png`: 계산/수치/그림.
- `check_replay_result.py`, `replay-check.log`: 최종 전달·상태·궤적 검사 PASS.

저장소 루트에서 후처리:

```bash
/home/khm/robotics/IsaacLab/isaaclab.sh -p outputs/rc-trajectory-replay-20260911/check_replay_result.py
/home/khm/robotics/IsaacLab/isaaclab.sh -p outputs/rc-trajectory-replay-20260911/compare.py
```

현재 결론은 **실제 명령 경로 재생은 성공했고, 회전 거동은 근접하지만 위치/이동 응답 차이는 여전히 크다**이다. 다음 튜닝에는 첫 직진의 RCOut·속도 응답과 수심 제어를 먼저 분리하고, bag의 축 해석/당시 FCU 설정 및 시작 조건 불확실성을 함께 다뤄야 한다. 이번 요청에서 새 물리 계수를 fitting하거나 적용하지 않았다.
