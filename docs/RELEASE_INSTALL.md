# Ubuntu 22.04 / 24.04 배포판 설치

배포 버전: **2026.09.20.1**, Linux **x86_64**.
두 호스트 OS에서 동일한 Ubuntu 22.04 / ROS 2 Humble 컨테이너를 사용합니다.
호스트에 Humble/Jazzy를 혼합 설치하지 않습니다. ARM/Jetson은 이 배포 대상이 아닙니다.

## 준비

- Docker Engine + Docker Compose v2, 일반 계정의 Docker 실행 권한.
- 시뮬레이션 설치용 여유 디스크 25 GiB 이상. VLA까지 설치할 경우 60 GiB 권장.
- GPU 가속: NVIDIA 드라이버 + NVIDIA Container Toolkit.
- VLA 추론: 현재 검증 장비는 RTX 5080 16 GiB, 드라이버 580 계열.
  다른 GPU는 예열 응답이 0.25초 미만이어야 실행 가능합니다.
- 데이터 수집은 VLA 모델/학습 환경 없이 가능합니다.
- CPU 렌더링은 `UUV_GPU=0`으로 사용할 수 있으나 센서 수신률·성능은 PC에 따라 다릅니다.
  레코더가 준비 완료를 표시하지 않으면 수집을 시작하지 마세요.

Docker 설치는 [Docker 공식 Ubuntu 안내](https://docs.docker.com/engine/install/ubuntu/),
GPU 연결은 [NVIDIA Container Toolkit 안내](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html)를 따릅니다.
드라이버·Docker 그룹·실행 중인 다른 컨테이너는 이 배포 설치기가 임의로 변경하지 않습니다.

## 설치

릴리스의 `uuv-sim-2026.09.20.1-source.tar.gz`를 내려받고 `SHA256SUMS`로 검증한 뒤 풉니다.
GitHub가 자동 생성하는 Source code ZIP 대신 이 배포 압축을 사용하면 Git LFS 파일도 포함됩니다.

```bash
sha256sum -c SHA256SUMS --ignore-missing
tar -xzf uuv-sim-2026.09.20.1-source.tar.gz
cd uuv-sim-2026.09.20.1
./release.sh install
./release.sh web
```

또는 Git LFS를 설치한 뒤 태그로 받습니다.

```bash
git clone --branch v2026.09.20.1 https://github.com/kanghyunmin-bot/ROS2-mujoco-UUVsimulator.git
cd ROS2-mujoco-UUVsimulator
git lfs pull
./release.sh install
./release.sh web
```

설치기는 고정된 ArduSub 소스와 서브모듈을 받아, 시간 누적 보정을 적용하고 새로 빌드합니다.
모든 설치 경로는 현재 checkout을 기준으로 계산합니다. 계정명이나 설치 디렉터리를 수정할 필요가 없습니다.
초기 설치에는 인터넷 연결이 필요합니다. 프록시/방화벽은 GitHub, Ubuntu/ROS apt, PyPI 접근을 허용해야 합니다.
이미 다른 revision의 firmware가 있으면 덮어쓰지 않고 오류를 표시합니다.

- 시뮬·레코더: <http://127.0.0.1:8878/>
- VLA 모델 선택: <http://127.0.0.1:8882/>

`web`을 실행한 터미널을 열어 두세요. 서비스는 외부 네트워크에 공개하지 않습니다.
같은 PC에서 이 배포판을 두 벌 동시에 실행하지 마세요(ROS 및 네트워크 포트 공유).

## 데이터 수집

1. Research pool · distributed physics를 선택하고 시뮬레이션을 시작합니다.
2. MAVROS 연결이 켜져 있는지 확인합니다. ROS 센서 출력은 VLA lite(640×360, 15 Hz), 수중 풀 경량 광학을 사용합니다.
3. 자세 정렬과 센서 수신을 확인하고, STABILIZE에서 조종을 준비합니다.
4. 레코더 패널에 지시문을 입력하고 **준비**를 누릅니다.
5. **녹화 시작** 후 GUI 스틱 또는 브라우저 게임패드로 시연합니다.
6. 성공/실패를 실제 수행 결과에 맞게 선택해 종료합니다. 미완료 기록을 성공으로 표시하지 않습니다.
7. 작업 후 DISARM을 확인합니다.

저장 경로는 checkout의 `outputs/vla-demonstrations/세션/staging/episode_XXXXXX/`입니다.
전방·손 RGB, 원본 센서, 명령, 시각, 수집 출처·종료 이유가 함께 저장됩니다.
`outputs`는 호스트에 남으며 컨테이너 종료로 삭제되지 않습니다. Git에는 자동 업로드되지 않습니다.

LeRobot 내보내기:

```bash
./release.sh export outputs/vla-demonstrations/세션/staging outputs/datasets/실험이름
```

두 경로는 checkout 내부의 실제 경로로 지정합니다. 내보내기는 새 폴더를 사용하세요.
학습/검증 분할은 세션 단위로 수행하고 같은 시연을 양쪽에 넣지 마세요.

## 선택 사항: VLA

```bash
./release.sh install-vla
./release.sh download-model
# 또는 로컬 모델: ./release.sh install-model /절대경로/체크포인트폴더
./release.sh web
```

GPU 환경은 컨테이너 Python 3.10 기반 `.venv-vla`, U0 소스는
`external/auv_vla`의 고정 commit입니다. 개인 Conda 경로를 사용하지 않습니다.
VLA는 CUDA 13.0 PyTorch 휠을 사용하므로 호환 NVIDIA 드라이버가 필요합니다.
호스트의 별도 환경을 사용하려면 `UUV_VLA_PYTHON=/path/to/python`을 지정할 수 있습니다.

모델은 `models/모델이름/`에서 검색합니다. 모든 shard와 config가 있어야 목록에 나타납니다.
CAP-off, 상태23/행동4, 10 Hz, PWM span400 계약을 검사합니다.
**모델 준비 → 예열 통과 확인 → ARM 후 실행** 순서입니다. 모델 준비와 데이터 수집은 별개입니다.
예열은 합성 입력을 사용하며 기존 시연 데이터나 특정 실험 폴더가 필요하지 않습니다.

현재 추가10,000스텝 모델은 행동 모방학습 결과입니다. 별도 근접/끼움 성공 판정기가 아니며,
오프라인 MSE가 실제 분리 성공률을 의미하지 않습니다. 모델의 task 문장을 GUI에 함께 제공합니다.

## 점검·백업

```bash
./release.sh doctor
```

MuJoCo 두 카메라 실제 렌더, ROS 패키지, firmware, 시간 보정, GUI 계약을 검사합니다.
GPU 예열은 모델 설치 후 별도로 확인합니다. 카메라나 입력 센서가 누락되면
레코더의 missing 항목과 `outputs/vla-demonstrations`, `outputs/vla-gui` 로그를 확인합니다.
배포 갱신은 새 디렉터리에 설치한 후 `outputs`를 별도로 백업/이관하세요.
기존 작업 폴더에 압축을 덮어쓰거나 `.venv`/ROS 빌드 폴더를 다른 OS에서 복사하지 마세요.

## 검증 범위

최종 검증 결과는 릴리스의 `VALIDATION.md` 및 GitHub Actions 로그를 확인하세요.
Ubuntu 버전별 깨끗한 CI 설치와 실제 로컬 GPU 검사는 구분하여 기록합니다.
컨테이너 설치 성공만으로 모든 PC의 GPU 드라이버, 게임패드, 실시간 센서 속도가 검증된 것은 아닙니다.

### 모델 라이선스
포함된 파인튜닝 가중치는 기반 NVIDIA 모델의 비상업적 연구·평가 용도 제한을 따릅니다. 모델 묶음의 `LICENSE-BASE-MODEL.txt`와 `MODEL_CARD.md`를 함께 배포합니다.

### 고정된 GPU 의존성

FlashAttention 2.8.3은 현재 모델과 동일한 CUDA 13.0 / PyTorch 2.10 / Python 3.10
조합의 [공개 사전 빌드 휠](https://github.com/mjun0812/flash-attention-prebuild-wheels/releases/tag/v0.9.0)을
SHA-256으로 검증해 설치합니다. PyTorch3D는 공식 저장소의 고정 소스에서 U0가 사용하는
순수 Python 좌표 변환 기능을 설치합니다. 네이티브 PyTorch3D 연산은 포함하지 않습니다.
Decord 0.6.0 휠 내부의 오래된 Python 태그 경고는 해당 항목만 구분하고,
실제 MP4 디코딩·좌표 변환·CUDA attention 연산을 설치 마지막에 검사합니다.
