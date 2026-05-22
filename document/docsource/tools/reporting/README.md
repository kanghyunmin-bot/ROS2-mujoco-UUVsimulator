# reporting tools

보고서와 발표 자료 생성 코드입니다.

## Groups

- `generate_*_assets.py`
  - 보고서용 그림과 metric asset 생성.
- `generate_*_latex.py`, `generate_*_report_tex.py`
  - LaTeX 원본 생성.
- `generate_uuv_presentation.py`, `generate_uuv_v22_current_keynote.py`
  - 발표 자료 생성.
- `generate_engine_comparison_figures.py`, `generate_mujoco_doc_figures.py`
  - 물리엔진/유체력 설명용 그래프 생성.

입력 JSON은 `../../metrics/report_inputs/`, LaTeX 원본은 `../../report_sources/`,
최종 PDF/발표 산출물은 `../../../reports/`에 둡니다.
