# 시작 안내 — 2026.09.08-pre-vla

최신 소스는 `main`, 활성 런타임은 `uuv_mujoco/current`이다.
이 버전은 VLA 적용 전 기본 ROV 인터페이스 기준선이다.

실행·설치는 [README](README.md), 변경과 한계는
[기준선 기록](docs/releases/2026.09.08-pre-vla.md)을 따른다.
과거 Dist ZIP/DEB의 버전 번호와 현재 소스 버전을 혼동하지 않는다.

기존 작업공간에서는 ROS 환경과 로컬 `.uuv_mujoco_env.sh`를 로드한 뒤
`./run_control_gui.sh --web --sim-preset research_pool_distributed`로 실행한다.
`.uuv_mujoco_env.sh`는 설치 과정에서 생성하는 머신별 설정이며 Git에 포함하지 않는다.
