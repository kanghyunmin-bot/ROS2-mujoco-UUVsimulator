# 녹화 및 VLA 준비 상태 — 2026-09-20

## 수정 내용

- 기본 연구 수조의 부표를 노란색 1개로 변경했다. 연구 수조 크기와
  distributed/April Real2Sim 물리 설정은 유지한다.
- 기본 카메라는 VLA lite 640×360, 15 Hz이며 관측/행동 저장은 10 Hz이다.
  브라우저 미리보기 FPS와 센서 캡처 FPS는 별개다.
- GUI 수집과 시뮬 정책의 명령은 STABILIZE, RC 채널 [5,6,3,4],
  중립 1500, span 400으로 맞췄다. 시뮬 정책 진폭 제한은 1.0,
  변화율 제한은 0.5/s이며 dry_run=true가 기본이다. 실물 기본값은 바꾸지 않았다.
- 파인튜닝 진입점은 CAP loss를 0으로 강제하고 모델 저장 설정에도 기록한다.
  추론 진입점은 같은 전처리, 23차원 상태, 4차원 명령, CAP-off 계약을 확인한다.
  기존 span 300 또는 다른 제어 모드의 데이터는 자동 재해석하지 않는다.
- 녹화 전에 FFmpeg와 명령 범위, 알려진 저주기 수심 설정을 확인한다.
  현 호스트에는 사용자 로컬 FFmpeg를 설치했고 변환기는 이 경로도 찾는다.
- 세션에는 실제 선택한 센서/물리 설정, 장면, 제어 overlay의 내용과 SHA-256,
  실행 명령, 관련 환경변수, tracked diff 및 새 소스 파일 사본을 남긴다.
  주기적 GUI 상태 조회는 큰 설정 본문을 생략하고 녹화 준비 시에만 본문을 저장한다.
- GUI에 VLA 제어권 준비/수동 제어 복귀를 추가했다. 준비 시 수동 RC를 해제하고
  GUI 발행자를 중단한다. 조이스틱·재생·다른 자율제어의 동시 입력을 막는다.
  복귀는 외부 RC 발행자가 종료된 후 가능하다. 정책 실행/Arm/Enable은 별도다.

## 수집 순서

1. 수정된 GUI와 시뮬레이터를 새로 실행한다. 이번 검사에서 운용 중인 프로세스를
   강제 재시작하지 않았다. 연구 수조, 노란 부표 1개, VLA lite 15 Hz를 확인한다.
2. 오차 모드는 수학적 모드를 선택한다. bag0402 오차 모드의 실측 수심은 2 Hz라서
   현재 엄격한 10 Hz 수집 계약에서 거절된다. 수학적 모드는 실측되지 않은
   drift/bias 등을 사전 가정으로 구분한다. 물리 April Real2Sim과 센서 오차 모드는 독립적이다.
3. 로봇/부표의 초기 배치를 설정하고 STABILIZE에서 자세 정렬, Arm 및 센서 상태를 확인한다.
   우선 같은 쉬운 배치로 연결을 확인한 뒤 초기 배치를 나눠 수집한다.
   자동 무작위 reset이나 자동 성공 판정이 추가된 것은 아니다.
4. 레코더 준비 후 접근→정렬→분리 한 회를 녹화한다. 성공/실패를 명시적으로 저장한다.
   끊김·센서 정지·시간 단절이 발생한 episode를 정상 시연으로 학습시키지 않는다.
5. 별도 출력 폴더로 LeRobot 변환 후 transfer loader의 admission 검사를 실행한다.
   실패 episode는 원본에 보존되지만 기본 학습 진입에서는 거절된다.
   성공까지 복구한 시연과 미복구 실패의 구분은 유지한다.

## 학습 및 추론 진입점

프로젝트 루트에서, 조직 U0 fork의 의존성을 갖춘 학습 환경을 사용한다.
`U0_ROOT`는 `Kmu26AuvRealDataConfig`가 있는 호환 checkout이다. 현재 검토한 소스는
`outputs/vla-transfer-audit-20260910/upstream/auv_vla`이며 임의의 원본 U0 설치와 같지 않다.

```bash
python rospkg/src/auv_vla_data_collector/tools/finetune_transfer.py \
  "$U0_ROOT" --dataset-path "$DATASET" --output-dir "$TRAIN_OUTPUT"

python rospkg/src/auv_vla_data_collector/tools/serve_transfer.py \
  "$U0_ROOT" "$CHECKPOINT" --check_only

python rospkg/src/auv_vla_data_collector/tools/serve_transfer.py \
  "$U0_ROOT" "$CHECKPOINT" --port 8000
```

학습 인자의 나머지 값과 초기 가중치는 해당 trainer 및 실험 계획에 맞춰 명시한다.
체크포인트의 `experiment_cfg/kmu26_transfer.json`을 함께 보존해야 한다.
CAP off는 기존 CAP 보조 사전학습의 영향을 없애는 것을 의미하지 않는다.

ROS 패키지를 빌드한 환경에서 `sim_policy.launch.py`를 실행하면 기본적으로
`/vla/proposed_rc`만 출력한다. task instruction, 실제 센서 관측, 지속적인
`/vla/deadman` 입력과 `/vla/enable`이 필요하다. 죽은 연결을 대신하는 자동 deadman을
GUI에 추가하지 않았다. dry-run 결과와 지연을 확인한 뒤에만 시뮬 live 설정을 사용한다.
live 전환 시 GUI에서 VLA 제어권 준비를 누르고 외부 정책을 실행한다.
수동 복귀 시에는 정책을 disable하고 프로세스를 종료한 뒤 수동 제어 복귀를 누른다.
disable만으로는 RC 발행자가 사라지지 않는다.

## 재검사 결과와 한계

- 물리·센서·시간·배치·새 녹화 계약: pytest 121개, subtest 9개 통과.
- 격리된 ROS Humble 컨테이너의 collector/policy/GUI 계약: pytest 116개 통과.
  실제 하드웨어나 운용 중 시뮬레이터에는 연결하지 않았다.
- JavaScript 오차 선택/RC 요청 timeout 검사 3개와 구문 검사 통과.
- GUI 기본 시작, distributed 기본값, 기존 pinger 제어권 회귀 검사 통과.
  기존 pinger 검사에 남아 있던 STABILIZE 및 disarmed auto-arm 기대값은 이미
  적용된 ALT_HOLD/strict RC3 동작에 맞춰 수정했다. 운용 코드의 해당 정책은 변경하지 않았다.
- 단일 부표에서 자석 20 N 분리 기준, 약한 힘에서 결합 유지, 분리 후 상승,
  로프 처짐 및 로봇 충돌 검사 통과. 실제 로봇 접근→분리 성공 시연을 대신하지 않는다.
- 수정 파일의 치명적 정적 오류 검사와 diff 공백 검사 통과. 넓은 GUI 검사에는
  기존 callback 모듈의 미정의 메시지 타입 annotation 24건이 남아 있다.
  이 모듈들은 deferred annotation을 사용하며 이번 수정 범위 밖이다.
- 기존 `u0_final`은 `state.prev_command` 규격이 없어서 추론 사전검사에서
  예상대로 거절된다. 학습된 호환 가중치가 만들어진 것은 아니다.
- 실제 GPU 학습/추론, 실시간 10 Hz 유지, 조이스틱 장시간 무선 안정성,
  학습 정책의 접근·정렬·분리 성공률 및 실물 전이는 아직 검증하지 않았다.
  카메라 설정 15 Hz는 실측 처리량 보장이 아니므로 첫 짧은 시연으로 확인한다.
