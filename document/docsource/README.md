# document/docsource layout

이 폴더는 UUV MuJoCo sim-to-real 검증 파이프라인의 active source입니다.
GUI, dist 검증, 기존 명령어가 `document/docsource/<script>`를 직접 호출하므로
루트에는 호환 wrapper만 남겼습니다. 실제 코드는 `tools/` 아래 목적별 폴더에
있습니다.

## Active entry points

- `run_closed_loop_april1_replay.sh`
  - ArduSub SITL + MuJoCo + April 1 rosbag replay + 비교 측정 실행.
- `replay_april1_rc_override_closed_loop.py`
  - April 1 bag의 RC override를 ROS2로 재생하는 노드.
- `compare_closed_loop_april1_replay.py`
  - real-vs-sim RMSE, 신뢰도, 그래프 생성.
- `run_uuv_param_autotune.py`
  - GUI에서 호출하는 파라미터 후보 비교 실행기.
- `identify_uuv_dynamics_from_rosbag.py`
  - rosbag 기반 inverse-dynamics/least-squares 식별 보조 도구.
- `generate_*`
  - 보고서용 figure, LaTeX, presentation 생성기.

## Organized folders

- `tools/pipeline/`
  - rosbag 분석, replay, closed-loop 비교, autotune, inverse dynamics, measurement 코드.
- `tools/reporting/`
  - figure, LaTeX, 발표 자료 생성 코드.
- `metrics/report_inputs/`
  - 보고서/그래프 생성기가 읽는 고정 JSON 입력.
- `experiments/`
  - 날짜별 실험 캠페인. 이미 해석이 끝난 run 결과를 보관.
- `history/`
  - 초기 튜닝, 축 우선순위, 명령 매핑 확인용 JSON.
- `report_sources/`
  - LaTeX 원본을 주제/버전별로 분리.
- `notes/`
  - 발표 구성안, 물리 파라미터 메모.
- `figures/`, `figures_v22/`, `figures_v22_latest/`, `figures_v30/`
  - 보고서와 발표에서 참조하는 그림 자산.
- `runs/`
  - 새 실험 실행 시 기본 출력 위치.

## Wrapper rule

루트의 `*.py`, `*.sh`는 기존 경로 호환용입니다. 실제 수정은 아래 실제 파일에서
합니다.

```text
tools/pipeline/<script>
tools/reporting/<script>
```

## Output rule

새로운 closed-loop replay나 튜닝 결과는 아래처럼 생성합니다.

```text
document/docsource/runs/<category>/<timestamp_or_case_name>/
```

결과 해석이 끝나고 보관 가치가 있으면 `experiments/YYYY-MM-DD_purpose/`로
옮깁니다. 이렇게 하면 `runs/`는 작업 중인 출력, `experiments/`는 정리된
실험 기록이라는 역할이 분리됩니다.
