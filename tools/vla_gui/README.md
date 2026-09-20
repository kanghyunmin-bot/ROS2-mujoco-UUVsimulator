# VLA GUI 실행

호스트에서 `bash docker/ubuntu-dev/dev.sh web`을 실행하면 기존 GUI(8878)와
VLA 관리 서버(8882)가 함께 시작됩니다. 기존 GUI의 오른쪽 아래
**VLA 모델 선택 · 실행** 패널을 여세요.

1. 시뮬을 시작하고 비교할 로봇·부표 시작 위치를 설정합니다.
2. 설치된 모델을 고르고 **모델 준비**를 누릅니다.
3. 실제 준비 모델과 예열 완료를 확인하고 **ARM 후 실행**을 누릅니다.
4. 종료 후 결과와 DISARM/수동 복귀 메시지를 확인합니다.

패널은 `models/`에 설치된 모델을 표시합니다.
최신 10,000스텝 모델이 있으면 초기 선택값으로 사용하고 해당 학습 지시문을 채웁니다.
사용자가 직접 수정한 지시문은 모델 변경 시 덮어쓰지 않습니다.
실제 모델은 준비 완료 전에는 로딩되지 않습니다.
합성 카메라·상태 입력으로 예열하며 이 단계는 로봇 명령을 발행하지 않습니다.
실행 버튼은 STABILIZE 전환, ARM 확인, GUI RC 제어권 전환, 정책 실행을 수행합니다.
시뮬 프로세스가 없는 경우 ARM 전에 거절합니다. 실물 운용용이 아닙니다.

실행 결과와 모델/예열/정책 로그는 `outputs/vla-gui/`에 남습니다.
`detached`는 시뮬 분리 상태 감지이며 실제 로봇의 성공을 뜻하지 않습니다.
이 실행기는 기존 `run_rollout.py`의 120초 wall-clock 한도와 센서/명령 watchdog을
유지합니다. 시뮬 진행 속도에 따라 요청한 시뮬 시간에 도달하기 전 끝날 수 있습니다.

관리 서버는 정확히 이 저장소를 `/workspace`에 마운트한 개발 컨테이너 하나를 찾습니다.
컨테이너 이름을 하드코딩하지 않습니다. 모델 준비 시 requirements-ros.txt를 확인/설치합니다.
기존 이 저장소의 serve_transfer.py 서버는 정리하지만, 무관한 8000 포트 사용자는
종료하지 않고 오류를 표시합니다. 같은 관리 서버에서 모델을 바꾸면 이전 모델을 정리합니다.

GPU 환경은 배포 설치기가 만드는 컨테이너의 `.venv-vla`를 사용합니다.
GUI를 다른 방법으로 시작한 경우 호스트에서 `bash tools/start_vla_gui.sh`를 실행하세요.

검사: `python3 -m unittest discover -s tools/vla_gui -p 'test_*.py'`

## Portable release

Use `./release.sh install-vla` and `./release.sh install-model PATH`. Models are discovered
under `models/`; collection works without any model. The default inference interpreter
is `/workspace/.venv-vla/bin/python` inside the ROS container. Set `UUV_VLA_PYTHON`
only to use a separate host environment. Warmup no longer needs a validation dataset.
See [release installation](../../docs/RELEASE_INSTALL.md).
