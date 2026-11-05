# 다음 세션에 전달할 작업 프롬프트 — MuJoCo UUV / VLA / PPO

이 문서의 맥락을 인계받아 기존 프로젝트를 이어서 진행해줘. 처음부터 새 구현을 만들거나 이미 검증한 수정을 되돌리지 말고, 현재 코드와 실행 상태를 먼저 확인해. 아래 내용은 **2026-09-22 02:57 KST 전후의 인계 기록**이다. 실행 상태·경로 존재·환경 수·성공 수는 시간이 지나면 바뀔 수 있으므로 실제 파일/API를 다시 확인해야 한다.

**추가 사용자 승인 — 2026-09-22 03:17 KST:** 사용자가 수면 중 **2시간마다 자동 점검하고, 문제를 진단·수정해 이어 학습하며, 충분한 근거로 정체가 확인되면 개선 설정/코드를 변경**하도록 요청했다. 아래 16절이 자동화 관련 최신 지시다. 앞부분의 ‘추가 요청 없이 장기 자동화 생성 금지’는 이번 명시적 요청으로 충족됐다. 자동화는 현재 작업에 연결하며 다른 세션에서 중복 생성하지 마라.

## 1. 지금 해야 하는 일과 사용자 의도

이 프로젝트는 수중 로봇이 노란 부표에 접근하고, 포크를 정렬한 다음 부표를 분리하도록 하는 MuJoCo 시뮬레이터, 시연 레코더, VLA, 병렬 residual PPO 학습 시스템이다.

가장 최근 사용자 요청은 다음과 같다.

> 시뮬레이션 틱 오류까지 전부 해결하려고 하기보다는, 지속적인 문제가 아니면 오류 난 병렬 환경은 버리고 나머지로 계속 이어서 학습하도록 해줘.

이 요청은 **구현하고 검증한 뒤 실제 학습도 재개한 상태**다. 지금 인계 때문에 실행을 중지하거나 또 새 학습을 시작할 필요는 없다. 먼저 기존 실행이 살아 있는지 확인하고 유지해라. 단순 상태 확인을 위해 장시간 학습을 재시작하지 말 것.

다음 사용자의 구체적인 요청이 없다면 현재 상태와 오류 여부를 확인해 간단히 알려주고, 장기 모니터링 자동화나 새로운 실험을 임의로 만들지 마라. 추가 변경을 요청받으면 이 맥락 위에서 이어서 진행해라.

## 2. 실제 작업 위치와 실행 환경

- 실제 작업 저장소: `/home/khm/robotics/underwater/ROS2-mujoco-UUVsimulator`
- Codex 기본 작업 디렉터리가 `/home/khm/robotics/IsaacLab`로 잡힐 수 있다. 이번 작업은 IsaacLab 구현이 아니라 위 MuJoCo 저장소 작업이다. 명령의 작업 디렉터리를 명시해라.
- 인계 당시 Git 브랜치: `codex/bag-trend-alignment`
- 인계 당시 HEAD: `98d3675`
- 수정/미추적 파일이 아주 많다. `tools/`도 미추적 상태였다. 사용자와 이전 세션의 작업이 섞여 있으므로 `git reset`, `git clean`, 통째로 checkout/revert 등을 하지 마라.
- 이번 오류 격리 변경은 커밋·푸시하지 않았다. 새 커밋/릴리스를 한 것으로 말하지 마라.
- 적용되는 `AGENTS.md`는 실제 경로와 상위 경로에서 확인할 것. 다른 저장소의 빌드 절차를 무조건 적용하지 마라.
- 호스트 ML Python: `/home/khm/miniforge3/envs/gr00t/bin/python`
- 단순 표준 라이브러리 작업/일부 단위 테스트는 `python3` 사용 가능.
- Docker 이미지: `khm/ros2-mujoco-uuv:dev`
- 컨테이너 내부 MuJoCo Python: `/workspace/.venv/bin/python`
- ROS 환경: 컨테이너에서 `/opt/ros/humble/setup.bash`, `/workspace/rospkg/install/setup.bash`를 source.
- `simstart` 별칭은 `/home/khm/.bash_aliases`에 있으며, 실제 명령은 위 저장소의 `docker/ubuntu-dev/dev.sh web`이다.

### 사용자 GUI와 RL 환경 분리

- 모델/RL GUI: `http://127.0.0.1:8882/rl`
- 사용자 직접 조종 GUI: `http://127.0.0.1:8878/`
- 사용자 조종 컨테이너: `ubuntu-dev-uuv-dev-run-98e5c00b0720` (다음 세션에서 존재 여부 재확인)
- **사용자 조종 컨테이너를 RL 테스트 대상으로 사용하거나 종료하지 마라.**
- RL 컨테이너 이름은 `uuv-rl-<run-name>-<index>` 형태.
- 각 환경은 network/PID/IPC, SITL EEPROM, 생성 파일, 로그, 녹화 디렉터리를 분리한다.
- 컨테이너의 `127.0.0.1:8878`은 그 환경의 내부 GUI이며 사용자 호스트의 8878과 구분해야 한다.
- `--network none`, EGL, headless, `ROS_DOMAIN_ID=42`, `UUV_RL_REWARD_TELEMETRY=1` 사용.
- 전역 `pkill`, 모든 Docker 컨테이너 종료, 사용자 수동 RC에 간섭하는 조작은 하지 마라.

## 3. 현재 실제 학습 실행

현재 run:

`/home/khm/robotics/underwater/ROS2-mujoco-UUVsimulator/outputs/rl-training/20260922-025444-1e76ee`

설정:

- 모델 키: `success-135-110-25000`
- 원본 VLA: `outputs/training-135-emphasis110-20260921/all135/model`
- GUI 이름: 성공 135개 · 검토 구간 1.1배 · 25,000스텝
- 1개 모델 × 6개 독립 환경
- 시도 최대 60 **시뮬레이션 초**
- `episodes_per_environment=100`이지만 현재 runner는 이를 100 학습 사이클로 사용한다. 초기 평가 + 사이클마다 학습/현 기준 평가/후보 평가가 있으므로 단순히 총 600시도로 해석하면 안 된다. 가중치가 안 바뀌면 평가를 건너뛴다.
- `resume_candidate=true`, `save_successes=true`
- 이전 실행의 완료된 2회 업데이트에서 재개했다.

복원 원본:

`outputs/rl-training/20260922-021706-d2e406/success-135-110-25000/training-latest.pt`

새 실행의 `collection-00001.pt`와 복원 원본을 실제 tensor 비교했다. **policy(critic 포함)와 Adam 상태가 정확히 일치하고 `lineage_updates=2`임을 확인했다.**

02:57 전후 API 스냅샷:

- `running=true`, `phase=collecting`, `error=null`
- `round_kind=baseline_evaluation`, `wave=1`
- `inherited_updates=2`
- 현재 실행의 `updates=0`, `update_attempts=0`, `promotions=0`은 정상이다. 복원 이전 업데이트는 별도 필드다.
- 6개 환경 모두 시뮬레이션 시간이 증가하는 것을 확인했다.
- 이후 worker 5가 일반 분리(`general`)로 첫 시도를 완료했고, 나머지 5개는 약 23~25초 진행 중이었다.
- 완료된 worker가 `workers` 목록에서 빠지는 것은 환경 오류/폐기를 뜻하지 않는다. 현재 활성 시도 목록과 전체 사용 가능한 환경 수를 구분해야 한다.

## 4. 상태/API 확인 방법

읽기:

```bash
curl -fsS http://127.0.0.1:8882/api/rl/status
curl -fsS http://127.0.0.1:8882/api/rl/config
docker ps --format '{{.Names}}'
```

주요 파일:

- `<run>/status.json`: 실행 상태, 시도 결과, 평가, 업데이트 지표
- `<run>/config.json`: 그 실행의 불변 설정 스냅샷
- `<run>/run.log`: production runner 로그
- `<run>/infrastructure-failures.json`: 환경 장애 별도 기록
- `<run>/env-N/outputs/rollout-XXXXX/worker.log`
- 같은 rollout의 `progress.json`, `episode.json`, `error.json`, `failure_status.json`, `failed-trajectory.json`
- `<run>/env-N/uuv_mujoco/current/logs/mujoco_*.log`
- `<run>/env-N/uuv_mujoco/current/generated/physics_failures/`

시작/중지 API는 POST이며 Origin 검사가 있다. **현재 실행 중이면 시작을 또 하지 말 것.** 사용자 요청/수정 적용상 필요할 때만 사용:

```bash
curl -fsS -X POST -H 'Origin: http://127.0.0.1:8882' \
  -H 'Content-Type: application/json' --data '{}' \
  http://127.0.0.1:8882/api/rl/start

curl -fsS -X POST -H 'Origin: http://127.0.0.1:8882' \
  http://127.0.0.1:8882/api/rl/stop
```

`tools/vla_gui/rl_manager.py`가 subprocess를 소유하고, `outputs/rl-training.lock`으로 중복 runner를 막는다. GUI HTML은 요청마다 파일을 읽는다. HTML만 수정할 때 서버 재시작은 필요 없다. 그러나 실행 중인 Python runner와 이미 만들어진 환경의 코드 복사본에는 수정이 자동 적용되지 않는다.

## 5. 직전 오류의 확정된 원인과 미해결 범위

실패했던 run: `outputs/rl-training/20260922-021706-d2e406`

이 실행은 실제 업데이트 2회/채택 2회 후, wave 8 학습용 수집에서 중단됐다. 완료 시도 45개, 완료 wave 7까지 있었다. 오류 문자열은 `환경 1: Waiting failed: simulation tick`이었다. 이전 runner 오류 번호는 0-based였으므로 이 사례는 **env-1, GUI로는 환경 2**다. 현재 수정된 사용자 메시지는 index+1 사용.

해당 환경의 로그:

`env-1/uuv_mujoco/current/logs/mujoco_20260921_171716.log`

핵심 메시지:

```text
[course] magnet detached: course_buoy_a_yellow_1 reason=magnet_load force=13012.481N
WARNING: Nan, Inf or huge value in QACC at DOF 28. The simulation is unstable. Time = 453.3015.
RuntimeError: Physics instability at 453.301500 s; stopped instead of continuing after an automatic reset.
```

MuJoCo 물리 계산 불안정으로 simulator가 종료 → 센서/tick 공급 중단 → worker가 simulation tick 대기 실패를 보고했다. 단순히 timeout이 너무 짧았거나 GPU/PPO가 죽은 것으로 판단하지 마라.

DOF 28은 `course_buoy_a_yellow_1_rope_joint_04`였다. 실패 직전 저장 상태에 줄 각속도 약 20,829 rad/s, 가속도 최대 약 1.49e8이 관측됐다. 당시 dt=.0005, integrator=2, rope ball joint armature=2e-5, damping=.0003. 13kN은 불안정한 시뮬레이션 수치이며 검증된 실물 힘으로 해석하면 안 된다.

실패 snapshot:

`outputs/rl-training/20260922-021706-d2e406/env-1/uuv_mujoco/current/generated/physics_failures/20260921T173945_182722.npz`

조사 디렉터리: `outputs/physics-tick-audit-20260922/`

근사 기체 궤적 재생 실험은 원래 줄 상태를 재현하지 못해 정확한 폭주를 재현하지 못했다. **물리 원인 자체를 고쳤다고 주장하지 마라. 물리 파라미터는 이번 요청에서 바꾸지 않았다.** 사용자 선택은 장애 환경 격리와 나머지 학습 지속이다.

## 6. 이번에 구현한 장애 처리 정책

관련 파일: `tools/rl_training/recovery.py`, `run.py`, `worker.py`, `evaluation.py`, `tools/vla_gui/rl.html`.

1. 물리 instability, simulation tick timeout, 일부 sensor timeout, worker process exit를 격리 가능한 장애로 다룬다.
2. 장애 난 환경은 **현재 run에서 영구 제외**한다. 자동 재생성/재시도 루프는 없다.
3. 해당 미완료 시도의 trajectory 전체를 버린다. 오류 전 정상처럼 보였던 transition, pending action까지 포함한다.
4. 그 시도는 PPO batch에 넣지 않고, 정상 episode 목록/성공·실패 수에도 넣지 않는다.
5. 이전에 같은 환경이 정상 완료한 시도까지 지우는 것은 아니다.
6. `infrastructure_failures`에 worker/wave/recording/kind/detail/버린 transition 수를 별도 기록한다.
7. Docker cleanup은 별도 thread pool에서 수행해 다른 환경의 inference 요청을 막지 않는다.
8. 한 run에서 3개 서로 다른 환경이 폐기되면 전체 중지한다. 모델에 남은 환경이 0개여도 중지한다. 이는 현재 구현의 명시적인 중단 기준이며 통계적으로 튜닝된 임계값은 아니다.
9. 명시적인 미분류 worker 오류는 삼키지 않고 전체 오류 처리한다. `error.json`에 미분류 오류가 있는데 process가 종료됐다고 `worker_exit`로 덮지 않도록 수정했다.
10. 환경 누락으로 incumbent/candidate 평가 개수가 다르면 `incomplete_evaluation`: 최고 모델 채택도 rollback도 하지 않고 learner를 보존한다.
11. `physics_failure()`는 **가장 최근 MuJoCo 로그**의 마지막 16KB만 읽는다. 새 정상 로그가 있으면 옛 crash를 현재 오류로 오인하지 않는다.

UI에는 이전 실행에서 이어받은 업데이트 수, 활성 환경 수, 환경 오류 제외 수를 표시한다. 브라우저에서 새로고침해야 새 HTML/설명이 보인다.

범위상 남은 제한: startup 실패, 전체 rollout watchdog, 정상 done 후 cleanup 실패 등 모든 예외를 환경 격리로 포괄한 것은 아니다. 사용자 요청은 확인된 간헐적인 시뮬/tick 계열 오류의 격리였다. blanket `except: continue`로 진짜 학습 버그까지 숨기지 마라.

## 7. 재개 로직과 체크포인트 의미

`recovery.find_resume()`는 `outputs/rl-training/*/<model-key>/training-latest.pt`를 최신 이름순으로 찾는다.

- schema `uuv.residual_rl.v1`, base model 경로, 관련 설정이 일치해야 한다.
- 비교에서 제외하는 항목: `models`, `model_paths`, `environments_per_model`, `episodes_per_environment`, `resume_candidate`, `save_successes`.
- 보상/episode_seconds 등의 변경은 호환성이 없는 것으로 처리한다.
- 정확히 호환되면 policy + critic + Adam을 복원한다.
- 호환본이 없으면 기존 `outputs/preserved-models/135-rl-update12/collection-00013.pt`로 fallback한다. 이 legacy fallback은 actor를 가져오고 critic을 0으로 초기화하며 Adam은 새로 시작한다.
- 이번 실제 production 재개는 fallback이 아니라 직전 2회 업데이트의 정확한 복원이었다.
- `save_checkpoint()`에 `lineage_updates`를 추가해 재시작 전 업데이트 수를 전달한다.
- 현재 재개 기능은 135 모델에 한정되어 있다. 다중 모델 일반화가 이미 완성된 것으로 말하지 마라.

체크포인트 종류:

- `collection-XXXXX.pt`: 그 wave에서 실제 행동 생성에 사용한 정책
- `update-XXXXX.pt`: 학습용 wave 후의 새 정책. 이 파일 존재만으로 성능 개선을 뜻하지 않음
- `accepted.pt`: 현재 run에서 평가 후 채택한 정책/초기 기준
- `training-latest.pt`: 평가 결정을 반영한 이어 학습할 learner
- `rollout-batch-XXXXX.pt`: 해당 PPO 학습에 실제 사용한 tensor batch
- `evaluation-XXXXX.json`: 평가 결정

주의: `training-latest.pt`와 `accepted.pt`는 다를 수 있다. 이번 복원 원본은 둘이 동일했던 시점이다. 일반적으로 미채택 learner에서 새 run을 시작할 경우 이전 run의 incumbent를 별도로 복원하는 로직은 없다. 또한 `training-latest.pt`는 매 transition마다 저장하는 것이 아니므로 장애 wave의 미완료 학습을 복원했다고 말하면 안 된다. 중단 직전 가장 최신 `update-*`와 재개용 checkpoint는 역할이 다르다.

## 8. PPO 구조: 실제 무엇을 학습하는가

- 전체 VLA를 PPO로 미세조정하는 구현이 아니다.
- 원본 VLA를 고정(frozen)하고, 작은 CPU residual actor/critic을 학습한다.
- 입력: 배포 가능한 센서 23차원 + VLA base action 4차원.
- 센서 구성: prev_command, dvl_velocity, angular_velocity, linear_acceleration, attitude, depth, altitude, validity.
- hidden 64×64 MLP, 4차원 Gaussian raw action을 tanh로 제한해 residual 생성.
- residual limit = 정규화 축별 최대 .15.
- VLA는 전방/손 카메라 등의 원래 observation을 보고, residual은 센서와 base action을 본다. residual이 직접 이미지를 입력받는다고 설명하지 마라.
- reward 판정용 ground truth는 VLA/residual 입력에 넣지 않는다.
- 호스트 하나의 프로세스에서 모델별 VLA를 로드하고 GPU inference는 직렬 처리한다. simulator는 병렬이다. 환경 6개가 처리속도 6배라는 뜻은 아니다.
- 여러 모델을 쓸 때 backbone config와 tensor가 bitwise 동일한 경우만 frozen backbone을 공유하고 action head/정규화는 분리한다.
- GAE discount는 wall time이 아닌 실제 simulation duration을 사용한다.
- PPO epoch 4, Adam 기본 lr3e-4, clip .2, gradient norm .5.
- objective는 PPO clipped policy loss + .5 × value squared error이다. 현재 loss에 entropy bonus는 없고 entropy는 진단 지표로 기록한다. 시연 BC의 L1 회귀와 현재 PPO를 혼동하지 마라.

## 9. 이전에 발견·수정한 학습 정체 원인

상세 근거: `tools/rl_training/PPO_FIXES_20260922.md`.

### 액션과 보상 연결

feedback이 실제 적용된 action_id를 명시한다. 다음 VLA 응답을 기다리며 유지한 행동의 reward는 이전 적용 행동에 귀속한다. terminal이 생기면 아직 적용 안 한 pending 행동은 취소한다. 비동기 추론 지연 자체가 사라진 것은 아니다.

### PPO guard가 전체 업데이트를 지워버리던 문제

이전에는 4epoch 끝의 guard 초과로 앞선 정상 epoch까지 되돌려 실제 가중치 변경이 0이 되는 일이 있었다. 실 production batch 4개에서 재현했다.

현재는 각 epoch 이전 policy **및 Adam moments**를 저장하고, 실패한 epoch만 원상복원 후 learning rate를 반씩 낮춰 최대 6번 추가 시도한다. 앞선 정상 epoch는 보존한다.

- 전체 batch 시작 정책 대비 KL <= .01
- mean residual 보정 변화 최대 <= .005
- batch 후 기본 lr 복원: 재시도가 영구적인 lr 감소를 만들지 않음
- 지표는 버린 proposal이 아니라 최종 유지된 policy 기준
- `accepted_epochs`, `backtracks`, `guard_rejected`, `parameter_changed` 구분

### 평가와 학습 유지 분리

초기 기준 평가 후, 학습용 수집 → fresh incumbent 평가 → candidate 평가 순서다. 평가 rollout은 PPO batch에 넣지 않는다.

동일한 scene reset/inference seed를 쓰지만 센서/물리 타이밍까지 완전히 동일하다고 주장하지 마라.

채택 조건은 평가 개수가 같고 fork/hand-or-fork/released 누적 등급 수가 감소하지 않으며, 등급 수 증가 또는 mean reward +.25 이상이다. .25는 noise margin이지 통계적 유의성 검정이 아니다.

- 개선: 채택
- 등급 감소 또는 mean reward -.25 이하: incumbent policy + optimizer로 rollback
- 차이가 작아 불확실: 최고 모델은 유지하되 candidate policy + Adam을 다음 학습까지 유지
- 환경 누락으로 수가 달라짐: `incomplete_evaluation`, 채택/rollback 보류
- actor 변화 없음: 후보 비교 rollout 생략

과거 `updates`는 시도 수였을 수 있다. 지금은 실제 가중치 변경, 업데이트 시도, 평가 채택을 분리 표시한다. 그래프도 탐색/기준 평가/후보 평가를 구분한다. 단순 reward 증가나 파라미터 변경을 확정적인 성공률 향상으로 설명하지 마라.

## 10. 현재 보상과 3단계 분리 판정

실제 설정은 현재 run의 `config.json`을 최우선으로 확인한다. 인계 당시:

| 항목 | 값 |
|---|---:|
| 일반 분리 | +3 |
| 손 시야 분리 | +8 |
| 우측 포크 영역 분리 | +20 |
| 빠른 분리 추가 | 일반 최대 +1, 손 최대 +2, 포크 최대 +5 |
| 시간 비용 | -0.02 / simulation second |
| timeout | -2 |
| 유효하지 않은 분리 | -5 |
| 접근 progress | 최대 계수 1 |
| 높이·거리 결합 정렬 progress | 최대 계수 2 |
| 실제 우측 포크-봉 접촉 | 분리 전 1회 +2 |
| depth/yaw 변경 후 재발견 | 시도당 1회 +.5 |
| height scale | .1m |
| approach scale | 1m |
| 재탐색 depth 변화 | .03m |
| 재탐색 yaw 변화 | .1rad |

세 가지 terminal 보상은 누적해서 3+8+20을 주는 구조가 아니라, 해당하는 가장 높은 등급 하나를 적용한다. 빠른 분리 보너스는 남은 제한시간 비율로 계산한다.

유효 분리는 현재 시도에서 시작 후 제한시간 안에 발생한 target 분리이며 equality가 실제 해제돼야 한다. episode reset 시 이미 분리된 target은 거부한다. 단순 detector 소실은 분리 성공이 아니다.

`success` boolean은 엄격한 right_fork tier를 뜻하고, `released`는 일반/손/포크 전체 유효 분리다. `success=false`라도 `released=true, release_tier=general`일 수 있다. UI는 3등급을 각각 집계해야 한다.

접근/높이 보상은 시도 내 새로운 최고값 개선에만 지급해 가만히 머물며 반복 보상을 받지 않게 한다. 높이 potential에는 거리도 들어간다. 재탐색은 회전/뎁스 변경 그 자체가 아니라 시야를 잃은 뒤 자세 변화와 재발견이 모두 있어야 한다. 카메라 끊김에는 보상하지 않는다.

### 손 카메라 기준

- 영상의 learned detector가 아니라 MuJoCo 카메라 projection + ray occlusion으로 계산한 **기하학적 가시성 대리 지표**다.
- float/body/PVC의 여러 점을 검사하고 하나라도 보이면 가시성을 인정한다.
- 렌더 그룹 0~2와 맞춰 검사한다.
- `front_optical_window`가 alpha<.1일 때만 광선을 통과시킨다. 현재 alpha .06. 불투명 창, 기체 body, 포크, 벽은 가림으로 남긴다.
- 분리 직전 .25초 내 유효한 손 카메라 가시성 기록이 있으면 손 시야 분리를 인정한다.
- 실제 화면에서 픽셀이 확인됐다는 완전한 증명/segmentation은 아니다. 샘플 사이로 보이는 부분을 놓칠 수 있다.

### 우측 포크 기준

사용자는 복잡한 CAD 형상별 충돌 인과 추정보다 가벼운 영역 판정을 원했다.

- 분리 순간 부표 하부 봉 중심선이 기체 고정 박스와 교차하는지 판정하고 constraint 해제 전에 latch한다.
- FLU local center[m]: `[0.335511938, -0.091927344, -0.104344226]`
- half extents[m]: `[0.04, 0.025, 0.025]`
- 구현: `uuv_mujoco/current/sim/runtime/course_buoy_runtime.py`의 `right_fork_region_contains()`와 release 처리.
- `release_right_fork_region`으로 tier를 정한다.
- 실제 contact evidence는 별도이며 분리 전 접촉 보너스에 사용한다.
- 이 영역 판정이 정확한 물리 인과/실물 접촉 공차 검증을 의미하지는 않는다.
- 옛 로그의 tier 정의를 현재 기준으로 소급해서 해석하지 마라.

## 11. 사용자에게서 유지해야 할 요구사항

아래는 과거 대화에서 정한 선호/제약이다. **과거 요청을 모두 새로 실행하라는 뜻이 아니다.**

- 실제 포크 촬영 이미지를 참고해 손 카메라 각도·확대를 조정한 적이 있다. 카메라가 바뀐 데이터는 별도 보관해야 한다.
- 전방 영상 중앙 정렬은 접근 편의용이다. 손 정렬/포크 접근 때 영구 중앙 강제는 원치 않는다.
- 가능하면 전방 영상에서 부표가 좌측에 오도록 정렬하되, 우측 끼움도 정상이며 동일하게 취급할 수 있다고 했다.
- 잘린 target을 무조건 제외하는 임의 규칙은 제거하길 원했다. 새 편의 규칙을 몰래 추가하지 마라.
- 가장 큰 색상 blob만 고르는 방식보다는 타원형 조건을 원했다.
- 구간 정답을 다시 녹화/라벨링하라고 요구하는 방식은 원하지 않았다.
- 일반 분리도 보상하고, 손 시야/우측 포크 영역 분리에 더 높은 보상을 주며 3가지 성과를 따로 보여달라고 했다.
- 원본 VLA, 녹화 데이터, 사용자 직접 조종 환경은 보호한다.
- 예전에 yaw drift/입력 해제 후 반대 회전 문제가 있었고 yaw-stable preset으로 정리한 상태다. RL 장애 대응을 위해 이 수정들을 되돌리지 마라.
- 18개 무강조 5,000step, 59개/84개 모델, 이전 성공 시연을 합친 135개 1.1배/25,000step 등의 이력이 있다. 현재 파일/manifest를 다시 세지 않고 과거 숫자를 현재 전체 보유 데이터 수로 단정하지 마라.
- 과거 파일 정리 승인 `1-b, 2-b, 3-삭제`는 당시 특정 목록에 대한 답이다. 원래 선택지 원문 없이 새로운 파일 삭제에 포괄 적용하지 마라.
- 삭제/압축/정리보다 지금 학습 지속이 우선이다. 학습 중 사용 중인 모델/checkpoint를 지우지 마라.

## 12. 실제 검증 결과와 근거

이번 장애 격리 검증:

1. `outputs/rl-discard-verify-20260922-025042`
   - 실제 모델과 2개 Docker simulator, 시도 2초.
   - baseline 도중 env0 컨테이너 강제 종료.
   - partial transition 3개 폐기.
   - env1은 baseline/training/incumbent/candidate 4단계 완료, 실제 PPO update 1회.
   - 실패 시도는 정상 episode 목록에 없음.
2. `outputs/rl-discard-verify-20260922-025301`
   - 이번에는 **학습용 수집 도중** env0 강제 종료.
   - partial transition 3개 폐기.
   - PPO batch 크기 11 = 살아남은 env1 episode trace 11과 정확히 일치.
   - 이후 env0 rollout 폴더 생성 안 됨, 재시작 안 함.
   - 실제 가중치 변경 1회 및 두 평가 완료, exit code 0.
3. 두 테스트의 전용 컨테이너는 모두 제거됐고 사용자 컨테이너는 유지됐다.
4. `test_recovery` 3개 + `test_transition` 8개 = 11개 집중 테스트 통과.
5. 평가 개수 차이 regression은 이전 rollback 동작으로 바꾸면 실패하고 수정본에서 통과하는 것을 별도 임시 복사본으로 확인했다.
6. Python compile과 실제 제공 HTML의 JS syntax 검사 통과.
7. 실제 production 복원본의 policy/critic/Adam 정확 일치 확인.

더 이전의 검증:

- `outputs/rl-analysis/fixes-20260922/`에 PPO guard 실제 batch 재생 등 근거.
- 6환경 24 real episodes 검증에서 실행/연결을 확인했지만 task 성공률 향상을 입증한 것은 아니다.
- `outputs/rl-retention-verify-20260922-021328`: 7 real episodes, 2회 짧은 학습 cycle, 불확실한 candidate의 policy+Adam이 다음 학습으로 유지되는지 확인.
- transparent window, opaque window, fork occlusion의 real-scene 검사 이력은 `PPO_FIXES_20260922.md`에 있다.

이 테스트는 장애 격리와 학습 연결 검증이다. 장기적으로 task 성능이 안정적으로 성장한다는 보장은 아니다. 이전 오류 전 후보는 hand-or-fork 1/6, released 2/6였지만 표본이 매우 적고 stochastic 차이가 있어 최고 성능 확정으로 말하지 마라.

## 13. 주요 코드와 문서 안내

모든 상대 경로의 기준은 MuJoCo 저장소 root다.

| 경로 | 역할 |
|---|---|
| `tools/rl_training/run.py` | 모델 로딩, 병렬 collection, phase/evaluation/PPO, checkpoint, 장애 격리 orchestration |
| `tools/rl_training/worker.py` | 단일 환경 arming/reset/recording/action/reward/IPC |
| `tools/rl_training/recovery.py` | 최신 물리 로그 판독, 환경 폐기 제한, 호환 checkpoint 탐색 |
| `tools/rl_training/isolation.py` | 전용 Docker 환경 생성/종료 |
| `tools/rl_training/residual_policy.py` | residual actor/critic, GAE, PPO guard, checkpoint |
| `tools/rl_training/transition.py` | 적용 action과 feedback 연결 |
| `tools/rl_training/evaluation.py` | 등급별 집계, promote/rollback/continue/incomplete |
| `tools/rl_training/reward.py` | terminal tier, shaping, timeout/탐색 보상 |
| `tools/rl_training/recording.py` | recorder 종료/자동 저장 race 및 불완전 녹화 처리 |
| `tools/vla_gui/server.py` | 8882 서버/API |
| `tools/vla_gui/rl_manager.py` | RL subprocess 관리 |
| `tools/vla_gui/rl.html` | 설정/상태/성공 3등급/그래프 |
| `uuv_mujoco/current/sim/runtime/course_buoy_runtime.py` | 실제 분리/포크 영역 latch |
| `uuv_mujoco/current/bridge/ros2_publish_course_buoys.py` | 관련 reward telemetry/카메라 가시성 경로 확인 시작점 |
| `tools/rl_training/PPO_FIXES_20260922.md` | 최근 수정·근거·이번 장애 격리 기록 |
| `tools/rl_training/VALIDATION.md` | 과거 검증과 변경 이력 |

**문서 충돌 주의:** `README.md`/`REWARD_V2.md`/`VALIDATION.md` 일부 앞부분은 초기 reward(+10 등), 35초, 정확한 CAD contact 성공 기준, 미검증 항목 등을 옛 상태로 적고 있다. 현재 코드 + 현재 run config + 가장 최근 수정 기록을 우선해라. 문서 앞부분만 읽고 현재 구현을 설명하지 마라.

테스트 예시(저장소 root에서):

```bash
cd tools/rl_training
python3 -m unittest test_recovery test_transition
```

torch가 필요한 테스트는 gr00t Python으로 실행하고, 실제 MuJoCo scene 검사는 적절한 컨테이너 Python을 써야 한다. `start_smoke.py`나 실 simulator 검증은 살아 있는 production runner와 GPU/lock이 충돌할 수 있으므로 필요 없이 실행하지 마라.

## 14. 저장 공간과 아티팩트 관리

02:57 확인: `/dev/sda4` 816G 중 291G 사용, 485G available, 38% 사용. 이 숫자는 다음 세션에서 `df -h`로 다시 확인해야 한다.

과거 366GB outputs/262GB checkpoint 문제와 레거시 정리 대화가 있었지만 현재는 정리가 진행된 뒤다. 이 문서만으로 모든 과거 삭제 내역을 재구성할 수는 없다. 현재 학습의 원본 VLA, active run, 복원 seed, accepted/training checkpoint, 원본 시연 데이터를 자동 삭제하지 마라. 실험 중간 파일과 영상은 계속 늘 수 있다.

## 15. 다음 세션의 첫 행동 체크리스트

1. 실제 repo 경로/적용 지침/Git 상태를 확인한다. untracked 코드를 놓치지 않는다.
2. 8882 RL status/config와 현재 run의 status/log를 읽는다.
3. 실행 중이면 중복 시작하지 않고 유지한다. 컨테이너 6개가 아니어도 retired/completed/active를 구분해서 해석한다.
4. `infrastructure_failures`, excluded_workers, active_environment_count를 확인한다. 장애가 생기면 환경 제외 이후 정상 환경이 계속 진행했는지 확인한다.
5. 현재 update 수는 inherited와 신규를 구분하고, outcome/실제 actor 변경/평가 phase를 함께 본다.
6. 새 성능 판단은 동일한 정의의 completed evaluation을 기준으로 한다. 탐색과 평가를 한 곡선으로 섞지 않는다.
7. 사용자에게 한국어로 현재 상태와 확실한 사실/추정을 구분해서 설명한다. 긴 로그를 그대로 나열하기보다 원인과 영향부터 말한다.
8. 오류가 없다면 무조건 새로운 문제를 만들어 변경하지 않는다. 사용자의 다음 요청에 맞춰 진행한다.

사용자는 답만 하고 멈추기보다 실제 구현·검증까지 완료하는 것을 원하지만, 근거 없는 확언이나 임의의 숨은 규칙은 원하지 않는다. 이번 수정의 결론은 **물리 폭주 완치가 아니라 오류 환경 격리와 정상 환경 학습 지속**이다. 이 구분을 유지해라.

## 16. 수면 중 2시간 단위 자동 진단·복구·정체 개선

### 승인 범위와 운영

사용자가 직접 다음을 요청했다: ‘인계 프롬프트에 나 잘 때 자동화해서 감지하는 거 만들어줘. 문제 생기면 해결하고 계속, 아예 정체되면 바꾸는 걸로 2시간 단위로.’

Codex의 현재 작업에 연결된 heartbeat 자동화 **UUV PPO 2시간 점검·복구**로 실행한다. 2026-09-22 등록 확인: ID **`uuv-ppo-2`**, 등록 당시 상태 **ACTIVE**, 주기 **2시간**. **2026-09-22 07:03 사용자 요청으로 PAUSED 전환했으며 재활성화 금지(새 요청 필요).** 다음 세션은 `$CODEX_HOME/automations/*/automation.toml`에서 이 ID/이름의 현재 상태와 대상 작업을 확인한다. 별도 cron, 무한 shell loop, 새 자동화를 중복 생성하지 않는다. 이전의 PAUSED `vla-2-8` 자동화는 다른 지도학습 작업이므로 재활성화하지 않는다.

정상 진행 중이면 그대로 두고, 오류 진단·국소 코드 수정·검증·필요한 RL 전용 환경 재시작·이어 학습은 추가 승인 없이 수행한다. 코드 변경이나 설정 조정은 이유와 전후 값을 기록하고 되돌릴 수 있어야 한다. 사용자 수동 중지(`stopped`, 중지 요청), 설정한 학습 완료(`completed`)를 장애로 취급해 무한 재시작하지 않는다. 사용자가 멈추면 자동 개입도 멈추고 자동화를 PAUSED로 전환한다. 정상 완료 때는 결과를 남기고 자동화를 PAUSED로 전환한다. 반복 횟수나 제한시간을 임의로 늘려 작업을 무한 연장하지 않는다.

### 매 점검의 관측과 기록

1. 실제 repo에서 인계 문서, 최신 API/status/config/log, 실행 PID와 RL 전용 컨테이너를 확인한다. API가 안 되면 process/로그를 먼저 확인하며 서버를 중복 시작하지 않는다.
2. `outputs/rl-monitor/`에 점검 시간, run 경로, config/코드 판별 정보, wave/phase, 완료 시도, 신규/누적 update, promotions, 평가 결과, 활성/제외 환경 수, 오류, CPU/GPU/RAM/디스크 상태를 남긴다. 전체 카메라나 거대한 checkpoint를 매번 복사하지 않는다.
3. 직전 `state.json`과 비교하되 run 변경/환경 제외/사용자 설정 변경을 구분한다. 로그·sim time·step·완료 wave 증가를 함께 확인해 느림을 hang으로 오판하지 않는다. 멈춘 듯하면 30~60초 간격으로 한 번 더 관측한다. GPU inference 대기와 저장/평가 단계도 확인한다.
4. 정상 진행은 방해하지 않는다. 기본적으로 변화 없는 상태를 알리지 않고 의미 있는 오류/복구/설정 변경/완료/사용자 판단 필요 상황만 알린다. 수면 중 매 점검 보고를 보내달라는 요청은 없었다.

### 오류 복구

- runner가 이미 단일 오류 환경을 제외하고 계속 진행하면 전체를 다시 시작하지 않는다.
- 전체 중단이나 정체된 process가 확인되면 로그로 실패 범위를 좁히고 가장 작은 수정 또는 자원 조정을 한다. 3개 환경 장애 중단 기준을 무력화하거나 timeout만 무한히 늘리지 않는다.
- 변경 전 현재 learner/accepted checkpoint와 config, 변경 파일 diff를 보존한다. 원본 시연과 VLA는 수정·삭제하지 않는다.
- 단위/회귀 테스트 후 RL 소유 환경에서 필요한 최소 검증을 한다. production과 test가 GPU/lock을 경쟁하지 않게 한다.
- 재시작은 선택한 checkpoint 경로를 기록하고 실제 가중치/critic/optimizer 복원 여부를 확인한다. 코드의 기본 fallback으로 예전 12회 모델에 조용히 돌아가게 두지 않는다. 호환성이 없으면 현재 모델을 명시적으로 전달하는 경로를 먼저 구현·검증하고, 보상 변경 시 critic/optimizer 처리 근거를 기록한다.
- 점검 1회당 복구 재시도는 최대 2번으로 제한한다. 같은 signature가 반복되거나 데이터 무결성을 확인할 수 없으면 실패 실행을 멈춘 채 근거를 보고한다. 무한 재시작으로 자원만 소비하지 않는다. 이 2번 제한은 운영상 반복 방지 기준이다.
- 사용자 환경/다른 작업을 종료하지 않는다. 비용 결제, 사용량 리셋 소비, 외부 업로드/배포/메시지 발송, 원본/최고 모델/레거시 일괄 삭제는 이 자동화 승인에 포함되지 않는다.

### 정체 판단과 변경

‘2시간 지남’ 또는 ‘평가 채택 0회’만으로 정체라고 하지 않는다. 손/포크 분리가 드물어도 학습 batch와 평가 표본 수, 실제 가중치 변경이 얼마나 쌓였는지 먼저 본다.

초기 운영 기준은 **동일 보상·판정·시도시간·비교 가능한 환경 조건에서 완료된 후보 평가 20회**를 확보한 뒤, 앞 10회와 뒤 10회를 비교하는 것이다. 각 창의 평가 시도 수/분모도 보고한다. 이 숫자는 검증된 최적값이나 통계적 유의성 기준이 아니라 적은 표본으로 성급히 바꾸지 않기 위한 기본 기준이다.

일반 분리/손 시야 분리/포크 영역 분리의 비율, 분리 시간, 보상 항목, KL, residual/action 포화, guard rejection, accepted_epochs, rollback 빈도를 함께 분석한다. 평가 정책과 exploration 데이터를 섞지 않는다. 환경 개수나 기준이 바뀌면 단순 횟수가 아닌 비교 가능한 비율과 조건을 확인한다. 0~2회 작은 등락만으로 ‘완전 정체’를 단정하지 않는다.

충분한 관측에서 분리 등급/속도/접근·정렬에 개선이 없고, 원인 진단도 이를 뒷받침할 때 변경한다. 실제 파라미터가 여러 시도 연속 전혀 안 변하는 코드 결함이나 잘못된 reward 연결은 표본을 기다리지 않고 조사·수정할 수 있다.

변경은 한 점검에 하나의 원인 가설/한 묶음으로 제한한다. 우선 실제 학습 결함과 observation/action/reward 연결을 점검한 뒤, 근거가 있으면 learning rate, 탐색량, 보상 shaping 크기, reset 분포 등 필요한 부분을 조정한다. 성공 정의를 느슨하게 하거나 보상 숫자를 키워 성장한 것처럼 만들지 않는다. 사용자 요구인 일반 < 손 시야 < 우측 포크 영역 분리 우선순위를 유지한다.

조정 전 model/config와 이후 model/config를 구분해 저장한다. ‘최신 learner’와 ‘검증된 최고 모델’을 혼동하지 않는다. 조건이 달라진 raw reward 곡선을 이어붙이지 않는다. 변경 후 동일 기준의 비교 평가로 검증하고 악화하면 되돌린다. 부족한 표본에서는 결과 미확정이라고 기록한다. 이미 다음 평가가 진행 중이면 이전 변경의 결과를 확인하기 전에 다시 바꾸지 않는다.

### 인계 시 자동화 중복 방지

자동화는 현재 원래 작업에 연결되어 있다. 다른 세션으로 옮기려면 기존 자동화를 확인한 뒤 동일 ID를 지원되는 도구로 이전/업데이트하거나 기존 것을 먼저 일시 중지한 후 대체한다. 둘을 동시에 활성화하지 않는다. 모든 인계 문서와 점검 기록에 마지막 개입 시간, 기준 checkpoint, 변경 이유, 다음에 확인할 결과를 남긴다.

### 자동 점검 기록 — 2026-09-22 05:21 KST

- 동일 run 정상 진행: wave 8 → 48, 신규 실제 업데이트 2 → 16(이전 실행 2 별도), 채택 6회. 완료 시도 284개, 6환경 유지, 인프라 오류 없음. 코드·설정 변경/재시작 없음.
- 완료 후보 평가 15회로 20회 비교 기준 미충족. 후보 평가 90시도 중 일반 22, 손 시야 2, 포크 0. 최근 단일 후보의 5/6 분리는 성장 확정 증거로 보지 않음.
- 현재 실행 config는 100 cycle, GUI에 저장된 다음 실행 설정은 500 cycle. 실행 중 설정은 자동 적용되지 않음.
- 디스크 available 약 468GiB, GPU 약 11.4/16.3GiB. 상세 기록 outputs/rl-monitor/check-20260922-052108.json. 다음에는 표본이 쌓인 동일 조건 평가 추세와 정체 원인을 확인.

### 최신 상태 — 2026-09-22 07:03 KST 사용자 자동화 종료

사용자 요청으로 `uuv-ppo-2`를 PAUSED 처리했다. 야간 자동 개입 승인은 더 이상 활성 상태가 아니다. 학습은 wave 81에서 세 번째 물리 장애로 이미 종료돼 있었고 재시작하지 않았다. 447시도, 신규 업데이트 27회, 후보 채택 9회. 우측 포크 분리 0회이며 안정 성장 근거 없음. 마지막 채택 `accepted.pt`는 업데이트 24, 재개용 `training-latest.pt`는 26, 미평가 최신 `update-00080.pt`는 27. 자세한 표/추세/제한은 `outputs/rl-monitor/RESULTS_20260922_0703.md`. 자동화·학습은 새 요청 없이 재시작하지 마라.

### GUI PPO 15회 후보 실행 연결 — 2026-09-22

사용자 요청으로 `success-135-ppo15` 항목을 8882 모델 실행 GUI 기본 선택으로 추가했다. 원본 135개 VLA + 야간 run의 `collection-00046.pt`(run 업데이트 15, lineage 17)를 함께 로드한다. `serve_transfer.py --residual_checkpoint`와 `tools/rl_training/deployment.py`가 deterministic residual을 적용한다. 학습처럼 첫 base action을 보정하고 새 응답까지 유지하도록 legacy 16-action chunk에 반복한다. 기존 timeout/limiter는 유지한다. 원본 VLA 선택은 기존 동작 유지. RL 시작 모델 목록에는 이 실행 전용 후보를 허용하지 않는다.

3개 residual 배포 테스트, 6개 manager 테스트, 실제 HTTP 녹화입력 5회 검사 통과(예열 후 최대 71ms). 브라우저에서 선택 항목/설명 확인. 검증용 모델 서버만 종료했고 8882 GUI 서버는 유지했다. 시뮬레이터 시작/ARM/임무 실행은 하지 않았다. 사용자는 simstart로 simulator를 띄우고 후보 선택 → 모델 준비 → ARM 후 실행 순서로 사용한다. 검증 기록: `outputs/vla-gui/ppo15-gui-validation.json`. 이전 자동화는 계속 PAUSED다.

### 원격 3D 뷰어 — 2026-09-22

사용자 요청으로 mjviser 기반 읽기 전용 전체 3D 뷰어를 127.0.0.1:8890에 실행했다. 사용자 GUI 컨테이너 `ubuntu-dev-uuv-dev-run-cb13f733b0a4`는 headless 연구 수조 yaw-stable preset으로 시작했고 DISARM 유지. `UUV_REMOTE_VIEW_DIR`가 설정된 경우에만 runner_step_setup에서 body pose를 15Hz 최신 프레임 방식으로 파일 export한다. 별도 viewer process가 geometry+pose를 웹에 제공하며 물리 계산은 독립적으로 유지된다. 네이티브 GLFW 뷰어는 비활성화. 선택적 의존성은 `.venv-mjviser`에만 설치했다. 자세한 실행/검증/제한은 `tools/REMOTE_VIEW.md` 참조. Codex 내장 브라우저는 WebGL unavailable 표시로 실제 3D 표시 검증이 막혔으며 외부 WebGL 브라우저가 필요하다. 로컬 상태 파일 지연 최대 70ms 측정이며 원격 체감 지연 보장은 아니다. PPO 학습·자동화는 재개하지 않았다.
