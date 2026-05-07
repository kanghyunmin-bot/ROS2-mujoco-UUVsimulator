# Document 정리 메모

현재 기준으로 바로 봐야 하는 산출물은 아래 3개입니다.

- `uuv_mujoco_v22_current_keynote_kr.key`
  - current v2.2 발표용 Keynote 원본
- `uuv_mujoco_v22_current_keynote_kr.pdf`
  - 발표용 PDF export
- `uuv_mujoco_v22_latest_report_kr.pdf`
  - current 중심 최신 보고서 PDF

소스와 생성 스크립트는 `document/docsource` 아래에 있습니다.

- `uuv_mujoco_v22_latest_report_kr.tex`
  - 최신 보고서 LaTeX 원본
- `generate_uuv_v22_latest_report_tex.py`
  - 최신 보고서 tex 생성기
- `generate_uuv_v22_current_focus_assets.py`
  - current 중심 figure / metric 생성기
- `generate_uuv_v22_current_keynote.py`
  - current 발표 자료 Keynote 생성기

그림은 버전별 폴더로 나뉘어 있습니다.

- `docsource/figures_v22_latest`
  - current 중심 최신 발표 / 보고서 그림
- `docsource/figures_v30`
  - v30 계열 그림
- `docsource/figures_v22`
  - 예전 v22 계열 그림
- `docsource/figures`
  - 더 오래된 공통 / 실험 그림

측정 기준 JSON은 아래 파일들이 현재 설명 자료에서 직접 참조됩니다.

- `docsource/measurement_summary_current_heavefix_latest_v30.json`
- `docsource/measurement_current_mode_path_latest_v30.json`
- `docsource/measurement_legacy_mode_path_latest_v30.json`
- `docsource/uuv_v22_current_focus_metrics.json`
- `docsource/uuv_v22_latest_report_metrics.json`

이번 정리에서 지운 항목은 임시 / 보조 파일만입니다.

- LaTeX 보조파일: `*.aux`, `*.log`, `*.out`, `*.toc`
- macOS 잡파일: `.DS_Store`
- Python 캐시: `__pycache__`
- Keynote 테스트 산출물: `test_*.key`, `test_*.pdf`
- 중복 복사본 PDF 1개
