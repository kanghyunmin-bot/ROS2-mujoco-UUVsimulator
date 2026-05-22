# metrics

보고서, 그래프 생성기, 비교 스크립트가 읽는 수치 입력을 모은 폴더입니다.

## Folders

- `report_inputs/`
  - 최신 보고서와 발표 생성기가 참조하는 JSON.
  - 예: `measurement_summary_current_heavefix_latest_v30.json`,
    `uuv_v22_latest_report_metrics.json`, `engine_comparison_metrics.json`.

규칙:

- 현재 문서/그림 생성기가 직접 읽는 JSON만 둡니다.
- 일회성 sweep 결과는 `../runs/` 또는 `../experiments/`에 둡니다.
- 보고서 입력으로 승격된 파일만 이 폴더로 이동합니다.
