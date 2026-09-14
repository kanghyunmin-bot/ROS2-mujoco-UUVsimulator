# VLA 수집기 시계·재시작 점검 — 2026-09-14

수집기의 준비 상태 표시와 실제 시작 서비스가 동일한 시뮬레이션 시계 조건을
검사하도록 정리했다. 기존 UI가 `ready=false`를 표시해도 직접 서비스를 호출하면
0초 또는 정지한 시계의 캐시로 episode를 시작할 수 있었다.

`use_sim_time=true`에서는 시계를 한 번 이상 관측했고, 시간이 양수이며,
관측된 시간보다 역행하지 않고, 마지막 시간 변화 이후 1초를 넘기지 않아야 한다.
이 조건을 준비 상태, 시작 서비스, 정책 observation, 수집 callback에 함께 적용했다.
벽시계 watchdog 실행 직전에도 캐시를 이용해 검사를 우회할 수 없다.
시계 정지·역행 후에는 영상·센서·RC뿐 아니라 이전 FCU의 연결·무장·모드 상태도
무효화하여 새 telemetry가 필요하다. 이미 기록된 행은 실패 사유와 함께 보존한다.

시계 검사 자체에는 추가 센서 처리나 이미지 렌더링이 없다. 기존 시간·상태를 조회하는 상수 비용의
검사이며, `use_sim_time=false`인 실물 수집에는 시뮬레이션 시계 조건을 요구하지 않는다.

실측 중 FCU 상태 메시지는 약 1초의 시뮬레이션 간격으로 왔지만, RTF 0.32에서는
벽시계 간격이 3.120–3.134초였다. 기존 수집기의 2초 벽시계 조건과 정책의 1초
벽시계 조건이 정상적인 느린 시뮬레이션의 FCU 상태를 오래된 값으로 거부했다.
이제 시뮬레이션에서는 수신 당시 ROS 시각을 별도로 기록하고 수집기·정책 모두
2초 ROS 시간 기준으로 상태 freshness를 확인한다. 실측 최대 heartbeat 간격
1.0025초의 작은 흔들림도 이 범위에 들어간다.

실물에서는 기존 수집기 2초·정책 1초 벽시계 기준을 유지한다. 정책의 deadman
0.3초, action 요청 만료, 시계 정지 watchdog은 그대로 벽시계로 동작한다.
정책도 시계 역행 시 이전 FCU 상태를 비우며, 비활성 상태에서 발생한 역행도 처리한다.
FCU 기록에는 벽시계 수신 age와 ROS 수신 age를 함께 남긴다.

## 검증

- 초기 Git 스냅샷 `b7c49ec`의 수집기를 임시 별도 패키지로 불러오면 새로운 결함
  회귀 검사 7개가 모두 실패했다. 작업 소스와 실행 중인 시뮬레이터는 되돌리지 않았다.
- 추가 FCU 시간축 수정 전 수집기 검사 5개, 정책 검사 5개가 실패했고 수정 후
  통과했다. 정책의 1.0025초 heartbeat 간격 회귀 검사도 1초 ROS 기준에서는 실패하고
  2초 ROS 기준에서는 통과했다.
- FCU 수정 시점 수집기·정책 검사: **63 passed**. 아래 원시 IMU·학습 입력 검사도
  추가해 최종 검증했다.
- 추가 검사에는 정상 시뮬레이션 시계와 실물 벽시계의 수집·정책 입력 허용도 포함했다.
- 전후 검사는 서로 다른 `PYTHONPYCACHEPREFIX`를 사용하여 기존 bytecode 캐시를
  재사용하지 않았다. ROS domain 93/94를 분리하여 기존 조종 세션과 통신하지 않았다.

사용 환경은 기존 `uuv-web-studio` 컨테이너, ROS Humble, `/workspace/.venv/bin/python`이다.
기존 exporter 의존성 경로를 사용했으며 기본 ROS·시뮬레이터 의존성을 추가하지 않았다.
컨테이너에서 저장소 루트 기준 재현 명령:

```bash
source /opt/ros/humble/setup.bash
source rospkg/install/setup.bash
export ROS_DOMAIN_ID=93
export ROS_LOCALHOST_ONLY=1
export PYTEST_DISABLE_PLUGIN_AUTOLOAD=1
export PYTHONPYCACHEPREFIX=$(mktemp -d /tmp/uuv-collector-tests.XXXXXX)
export PYTHONPATH="$PWD/outputs/vla-transfer-audit-20260910/export-deps:$PWD/rospkg/src/auv_vla_data_collector:$PWD/rospkg/src/kmu26_auv_vla_policy:${PYTHONPATH:-}"
.venv/bin/python -m pytest -q \
  rospkg/src/auv_vla_data_collector/test \
  rospkg/src/kmu26_auv_vla_policy/test
```

`export-deps`는 이 워크스테이션의 기존 검사 환경이다. 다른 환경에서는 해당
패키지의 `requirements-export.txt`가 제공하는 pandas/pyarrow와 FFmpeg가 필요하다.
`PYTEST_DISABLE_PLUGIN_AUTOLOAD=1`은 이 검사에서 사용하지 않는 ROS launch-testing
플러그인의 의존성 자동 로드를 피한다. ROS 노드·서비스 검사는 실제로 실행된다.

## 실제 GUI 센서 측정과 연결 수집

`tools/check_vla_live_inputs.py`는 제어 명령이나 수집 시작 없이 토픽을 관측한다.
collector YAML의 토픽 매핑과 freshness 함수를 사용하며, 설정된 발행 주기와
실측 주기를 구분한다. 실제 실행 domain은 42였다. 카메라 수정 직후 발견 대기 이후 12초 벽시계
구간을 측정한 결과는 다음과 같다.

| 항목 | 기존 GUI | 최종 VLA lite + pool_lite |
| --- | --- | --- |
| 영상 크기 | 640×360 | 640×360 |
| 실측 영상 source 주기 | 4.00 Hz | 15.01 Hz |
| 양쪽 영상 freshness | 0% | 100% |
| 수집 시 영상 age | 0.30–0.50초 | 0.03–0.06초 |
| 양쪽 영상 중복 전환 | 각각 25회 | 0회 |
| 실측 전체 RTF | 0.352 | 0.320 |

영상 타이밍은 개선됐지만 이 측정에서 전체 시뮬레이션 속도가 빨라진 것은 아니다.
당시 IMU·depth·DVL도 관측한 수집 시점에서는 모두 fresh였으며, IMU 최대 age는
0.2475초로 0.25초 제한에 가까웠다. 초기 측정에는 IMU 지연과 DDS 발견 전 영상
미수신이 있었고, 별도 결과 파일에 보존했다. 최종 비교는 DDS 발견 대기 8초 후
측정한 `lite-live-final.json`이다.

최종 GUI에서 별도 이름의 실제 수집기를 실행해 **51행 / 첫 행부터 마지막 행까지
5.0초의 시뮬레이션 시간**을 기록했다. 입력 명령은 기존 GUI의 단일 RC 발행자에서
모든 축 0으로 전송했으며, 차량은 비무장 상태를 유지했다. 무장·모드 변경·초기 자세
유지 해제는 호출하지 않았다. 수집 종료 후 GUI RC release 응답을 확인했고,
GUI 제어는 비활성·소유자 없음·0축 입력, 차량 위치 변화는 `[0,0,0]`m였다.

- 출처: `data_source=simulation`, `collection_kind=connection_check`, `success=false`.
- 종료 사유: `operator_stop`. 실제 시작·종료 ROS 서비스를 사용했다.
- state `(51,23)`, action `(51,4)`는 모두 유한값이며 action은 전부 0이다.
- 영상·DVL 속도·고도 validity는 모두 1이며, 양쪽 영상 capture stamp는 각각 51개가
  서로 다르다. 수집 영상 최대 age는 0.06초다.
- LeRobot 내보내기 후 실제 조직 U0 로더를 `--inspection`으로 실행했다.
  36개의 완전한 16-step chunk를 읽었고, 영상 전처리 `(1,2,224,224,3)`,
  state `(1,23)`, action `(16,4)`, 모델 입력 state `(1,64)`와 action `(16,32)`를 확인했다.
  optimizer·GPU 정책 추론은 실행하지 않았다.
- 같은 새 내보내기를 `--inspection` 없이 입력하면 `Not a reviewed task demonstration`
  오류로 거부되는 것도 확인했다. 연결 검사 자료를 학습 시연으로 전환하지 않았다.

재현 자료는 Git 비추적 `outputs/pre-vla-readiness-20260914/` 아래에 있다.
`baseline-live.json`, `lite-live-final.json`, `connection_check.py`,
`connection-check/{collector.yaml,context.json,summary.json,trace.json,gui-before.json,gui-after.json}`,
`connection-check/staging`, `connection-check/lerobot`, `connection-check/loader-result.json`에
설정·측정·원본 행·영상·검사 결과를 보존했다.

## 최종 원시 IMU·학습 입력 검증

MAVProxy의 기본 ALL-stream 요청이 15초마다 명시적 주파수를 덮어쓰는 문제를 수정했다.
기본 `--streamrate=-1`로 반복 요청을 끄되 사용자 인자는 유지한다. 시뮬레이션은
ATTITUDE 20 Hz만 한 번 요청한다. 실제 AHRS 출력은 최종 관측에서 10 Hz였다.
수집기·정책은 별도 `/mavros/imu/data_raw`의 각속도·가속도를 사용하도록 opt-in을 추가했다.
`fcu_link`로 발행되지만 bridge가 이미 body FLU 축으로 변환한 값이다.
원시 센서의 source/receipt 시각을 독립적으로 검사하며 AHRS 시각으로 갱신하지 않는다.
실물 기본 경로·23D state·기존 7개 시각 열을 유지한다. 원시 시각은 행별
`vehicle_state.jsonl`과 manifest에 기록하고 내보내기에도 보존한다.

학습 허용 검사도 시뮬레이션 FCU의 ROS age와 실물 FCU의 wall age를 구분한다.
누락·음수·비유한 age, 오래된 원시 IMU, 서로 다른 IMU 계약의 묵시적 병합을 거절한다.
새 원시 IMU 검사 6개, 학습 검사 6개, 내보내기 계약 검사 1개는 수정 전 실패를 확인했다.
최종 수집기·정책·학습 입력 검사 80개와 MAVProxy 설정 검사 2개가 통과했다.

최종 환경은 MuJoCo 3.12.0, ROS Humble, RTX 5080, X11 viewer+웹 GUI다.
8초 발견 대기 후 30초 동안 10 Hz 수집 시점 60개를 검사한 결과:

| 입력 | 실측 source Hz | 유효율 | 최대 age |
|---|---:|---:|---:|
| 전방·손 영상 | 각각 14.992 | 100% | 0.060초 |
| AHRS | 10.0 | 100% | 0.010초 |
| 원시 IMU motion | 50.0 | 100% | 0.020초 |
| 수심 | 10.0 | 100% | 0.100초 |
| DVL | 약 10.004 | 100% | 0.095초 |

이 재구성한 실행 환경에서 전체 RTF는 **0.203**이었다. 이전 0.320과 실행 환경이
달라 직접 성능 비교로 사용할 수 없으며, 실시간 실행 달성을 주장하지 않는다.
최신 센서 측정은 [JSON](../assets/pre-vla-live-motion-20260914.json)에 남겼다.

원시 motion을 켠 `connection-check-motion`도 51행/5초·0입력·비무장으로 저장했다.
원시 IMU capture stamp 51개가 고유하며 age는 0.02초, state와 legacy timestamp
shape는 `(51,23)`·`(51,7)` 그대로다. LeRobot 내보내기와 실제 U0 inspection
36개 전체 chunk 검사를 다시 통과했다.
[행별 감사 요약](../assets/pre-vla-motion-audit-20260914.json),
[실제 로더 결과](../assets/pre-vla-loader-20260914.json).

## 수집 전 남은 검증

이전 50행과 이번 51행의 중립 RC 자료는 모두 `connection_check`이며 부표 접근·분리
시연이나 작업 성공의 증거가 아니다. 장시간 부하에서의 IMU·영상 freshness와
수집 안정성을 확인하고, 무장·단일 RC 발행자·기대 모드·세션 출처를 갖춘 작업 시연을
확보해야 한다. 실제 정책 enable/RC 출력은 이번에 실행하지 않았다.
실물 RC 축 방향·채택 지연·카메라 수중 보정·센서 장착값·물성은
`VLA_TRANSFER_AUDIT_20260910.md`에 기록된 별도 검증 대상이다.
