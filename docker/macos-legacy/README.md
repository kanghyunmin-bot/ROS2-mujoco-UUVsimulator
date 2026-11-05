# Mac MuJoCo 실행 예제 — 2026년 2월 구버전

과거 Mac에서 사용했던 ROS 2 Humble + MuJoCo 3.4.0 + VNC 실행 구성을 구버전 예제로 보존한다. 사용자가 과거 Mac 실행 사례로 확인한 환경이며, 현재 Ubuntu 중심 실행 구성과 구분한다.

## 보존 근거와 범위

- 원본 이미지: `ros2-mujoco:humble` (2026-02-11), `ros2-mujoco:humble-vnc` (2026-02-14). 정확한 ID는 `historical-images.json`에 기록했다.
- Mac Docker Desktop에서 `linux/amd64` 컨테이너와 소프트웨어 OpenGL 렌더링을 사용했다. Apple Silicon 네이티브 MuJoCo 실행 예제가 아니다.
- 기존 컨테이너 이름은 `ros2mujoco`, 소스 연결 위치는 `/workspace`, 호스트 포트는 VNC 6001 / noVNC 6080이었다.
- `entrypoint.sh`와 `start-vnc.sh`는 삭제 전 컨테이너에서 직접 추출한 원본이다. `image-history.jsonl`은 원본 이미지의 전체 빌드 이력이다.
- Dockerfile 원본은 발견하지 못해 이미지 이력을 기준으로 복원했다. ROS 베이스와 apt 패키지는 고정하지 않았으므로 과거 이미지와 바이트 단위로 동일한 결과를 보장하지 않는다.
- 당시 연결된 소스의 정확한 Git 커밋은 확인되지 않았다. 이 커밋은 구버전 실행 환경을 보관하는 커밋이며 당시 시뮬레이터 전체 소스 스냅샷을 뜻하지 않는다. 기존 연구 소스 및 미커밋 자료는 별도로 유지한다.

## 실행

Mac에 Docker Desktop을 설치하고 amd64 컨테이너 실행을 지원하도록 설정한다. 저장소 루트에서:

```sh
export MUJOCO_WORKSPACE="$(pwd)"
docker compose -f docker/macos-legacy/compose.yaml up -d --build
```

브라우저에서 <http://localhost:6080/vnc.html>을 열면 컨테이너 데스크톱에 접속할 수 있다. 컨테이너 안의 `/workspace`에는 지정한 소스가 연결된다. 시뮬레이터 실행 명령은 사용하려는 소스 버전의 안내를 따른다. VNC 시작 자체가 시뮬레이션 실행을 의미하지 않는다.

```sh
docker compose -f docker/macos-legacy/compose.yaml down
```

기존 포트는 로컬 컴퓨터에만 공개하도록 설정했다. 시작 스크립트의 VNC 인증 방식은 원본 그대로다.

## 검증 기록

2026-11-06에 원본 이미지 식별 정보 및 시작 스크립트를 추출하고, 보관 이미지의 SHA-256 blob 32개를 검증했다. 이 커밋의 셸 스크립트 문법과 Compose 설정을 검사했다. 재구성한 Dockerfile 빌드 및 GUI/물리 시뮬레이션은 재검증하지 않았다.

기존 Git 이력을 소급 수정하거나 과거 날짜의 커밋으로 위장하지 않고, 구버전 예제를 별도 경로에 추가했다.
