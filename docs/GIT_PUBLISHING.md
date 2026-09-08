# Git 게시 기준

최신 소스 기준은 `main`이다. VLA 도입 전 기준선은 `2026.09.08-pre-vla`다.
이전 브랜치의 고유 커밋은 main의 부모 이력 또는 보존 참조로 유지한 후
작업용 브랜치를 정리한다. 과거 커밋의 작성자나 내용을 다시 쓰지 않는다.

소스·설정·검사·문서는 Git에 포함한다. Python/ROS 빌드 산출물, 로그,
실험 출력, 머신별 환경 파일과 편집기 설정은 제외한다.
YOLO 가중치는 기존 Git LFS로 관리하고 ArduPilot은 기존 submodule을 유지한다.

게시 전 관련 회귀 검사, ROS 패키지 빌드, 셸·Python 구문 검사,
`git diff --check`와 staged 파일의 비밀정보·대용량 산출물 여부를 확인한다.
원격에 쓰기 전에 source snapshot과 기존 원격 커밋을 모두 보존한다.

이 저장소의 원격은 `kanghyunmin-bot/ROS2-mujoco-UUVsimulator`다.
조직의 실물 패키지 저장소에 대한 변경은 별도 작업이다.
