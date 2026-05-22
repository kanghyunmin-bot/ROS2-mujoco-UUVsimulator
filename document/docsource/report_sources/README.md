# report_sources

LaTeX 원본을 주제와 버전별로 분리한 폴더입니다. 컴파일된 PDF는
`../../reports/generated_pdf/`에 둡니다.

## Folders

- `fluid_modeling/`
  - MuJoCo ellipsoid fluid, DVL/omega cross term, 유체력 모델링 설명.
- `april1_rosbag/`
  - 2026-04-01 실제 로봇 rosbag 기반 AB 비교, inertia 비교, replay 보고서.
- `v22/`
  - v2.2 current/ellipsoid 설명, visual 자료, fluid proxy slide 원본.
- `v22_latest/`
  - current 중심 최신 보고서 원본.
- `v30/`
  - v30 계열 보고서 원본.

새 보고서 원본은 여기에 두고, 렌더링 결과만 `reports/`로 보냅니다.

기존 그림 경로는 `docsource/figures*` 기준으로 작성된 원본이 많으므로, 수동
컴파일할 때는 `document/docsource`에서 `xelatex report_sources/<topic>/<file>.tex`
형태로 실행하는 것을 기준으로 합니다.
