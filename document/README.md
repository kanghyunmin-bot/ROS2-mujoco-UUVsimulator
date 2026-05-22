# document layout

`document/`는 시뮬레이션 검증, rosbag 비교, 보고서, Notion 업로드 자료를
목적별로 분리한 문서 작업 공간입니다. 실행 스크립트의 고정 경로를 깨지 않기
위해 active tool은 `docsource/` 루트에 유지하고, 산출물과 근거 자료만
시기/목적별 폴더로 분류했습니다.

## Top-level folders

- `docsource/`
  - rosbag 분석, closed-loop replay, inverse dynamics, 그래프/보고서 생성 스크립트.
  - 현재도 실행되는 active source입니다.
- `docsource/metrics/report_inputs/`
  - 보고서와 그래프 생성기가 직접 읽는 기준 JSON.
- `docsource/experiments/`
  - 2026-05-04 이후 반복 테스트 결과를 날짜와 목적별로 정리한 폴더.
- `docsource/history/`
  - 2026-04-29부터 2026-05-06 사이의 초기 튜닝/축 부호/명령 매핑 확인 JSON.
- `docsource/report_sources/`
  - LaTeX 원본. 주제별로 `fluid_modeling`, `april1_rosbag`, `v22`, `v22_latest`, `v30`로 분류.
- `docsource/notes/`
  - 발표 구성안, 물리 파라미터 메모처럼 실행 산출물이 아닌 노트.
- `docsource/runs/`
  - 앞으로 새 실험을 돌릴 때 쓰는 active output sink.
- `reports/`
  - 읽기용 최종 산출물. PDF, Keynote/PPTX export 등.
- `archive/`
  - 예전 보고서/발표 산출물. 현재 파이프라인 입력으로 쓰지 않는 자료.
- `notion/`
  - Notion 업로드용 그래프/마크다운 export.

## Placement policy

- 새 실험 결과는 먼저 `docsource/runs/<category>/<timestamp_or_name>/`에 생성합니다.
- 의미가 확정된 실험 묶음은 `docsource/experiments/YYYY-MM-DD_purpose/`로 옮깁니다.
- 보고서 입력 JSON은 `docsource/metrics/report_inputs/`에 둡니다.
- LaTeX 원본은 `docsource/report_sources/<topic>/`에 둡니다.
- 최종 PDF/발표 파일은 `reports/` 아래에 둡니다.
- 임시 파일, `.DS_Store`, `__pycache__`, LaTeX 보조파일은 보관하지 않습니다.
