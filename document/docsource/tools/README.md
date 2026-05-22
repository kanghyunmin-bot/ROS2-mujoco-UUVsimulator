# tools

`docsource`의 실행 코드를 실제 목적별로 분리한 폴더입니다.

- `pipeline/`
  - rosbag 분석, SITL/MuJoCo replay, closed-loop 비교, autotune, inverse dynamics, measurement 실행 코드.
- `reporting/`
  - figure, LaTeX, Keynote/PPTX 생성 코드.

`document/docsource/*.py`와 `*.sh`에는 기존 GUI/dist/명령어 호환을 위한 얇은 wrapper만 남겨 두었습니다.
새 코드를 고칠 때는 이 폴더 아래의 실제 파일을 수정합니다.
