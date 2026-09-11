# 시작 안내 — 2026.09.12

최신 소스·영상·검증 범위는 [README](README.md), 변경 사항은 [버전 기록](docs/versions/2026.09.12.md)을 참고하세요.

- 활성 런타임: `uuv_mujoco/current`.
- Ubuntu 22.04 / ROS 2 Humble, Linux x86_64 기준.
- Git LFS 및 ArduPilot submodule을 포함한 clone이 권장됩니다.
- 다른 PC에서는 setup 스크립트로 Python 환경·SITL·ROS 의존성을 설치하세요. 이 PC의 `.venv`, `.uuv_mujoco_env.sh`, 장치 권한을 복사하지 마세요.
- Docker는 [공개 ROS 기반 이미지로 빌드](docker/ubuntu-dev/README.md)합니다. NVIDIA GUI 실행에는 NVIDIA Container Toolkit 및 디스플레이 접근이 필요합니다.
- 이번 갱신은 Git 소스 버전입니다. GitHub 릴리스는 생성하지 않습니다. 과거 DEB/ZIP 설치 파일을 최신판으로 오인하지 마세요.
- VLA 수집·변환·학습 입력 연결을 확인한 버전이며, 학습된 정책의 작업 성공이나 실물 전이를 입증한 버전은 아닙니다.
