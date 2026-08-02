# KMU AUV Simulator 2026.08.02

Ubuntu 22.04 amd64용 MuJoCo + ArduSub SITL + ROS 2 Humble 배포판이다.

## 설치

1. `kmu-auv-simulator_2026.08.02_amd64.deb`를 더블클릭한다.
2. Ubuntu 앱 센터에서 **설치**를 누른다.
3. 앱 목록에서 **KMU AUV Simulator**를 실행한다.
4. 첫 실행 창에서 **설치 시작**을 누르고 관리자 암호를 입력한다.
5. 설치가 끝나면 웹 GUI가 자동으로 열린다.

첫 설치에는 인터넷 연결이 필요하다. ROS 2, Python 환경, 고정 커밋의
ArduPilot과 QGroundControl을 내려받으며, 프로그램 작업공간은 기본적으로
`~/.local/share/kmu-auv-simulator/current`에 생성된다.

앱 목록의 **KMU AUV 시뮬레이터 복구 설치**를 실행하면 같은 버전도 payload를
다시 풀고 ROS 패키지를 재빌드한다. 사용자 수조·Ping360·추력 설정과 로그는
runtime 교체 과정에서 보존한다.

## 명령행 실행

```bash
kmu-auv-simulator                 # 웹 GUI
kmu-auv-simulator --desktop       # Tk GUI
kmu-auv-simulator --headless      # SITL + MuJoCo headless
kmu-auv-simulator --repair        # 복구 설치
kmu-auv-simulator --uninstall     # 완전 제거
```

## 완전 제거

앱 목록에서 **KMU AUV 시뮬레이터 완전 제거**를 실행하거나 다음 명령을 사용한다.

```bash
kmu-auv-simulator --uninstall
```

설치 작업공간 안의 MuJoCo·ArduPilot·QGroundControl·ROS source/build/install/log,
`~/.venvs/uuv_mujoco`, 앱 상태와 캐시를 삭제한 다음 Debian 패키지도 purge한다.
다른 ROS 작업공간에서도 사용할 수 있는 시스템 공용 패키지는 기본적으로 보존한다.
설치기가 새로 추가한 APT 패키지까지 제거하려면 위험 범위를 확인한 뒤 다음을 사용한다.

```bash
kmu-auv-simulator-uninstall --purge-system-deps
```

## 고정 계약

- Ubuntu 22.04, amd64, ROS 2 Humble, Python 3.10
- MuJoCo `3.8.0`
- ArduPilot commit `2dd0bb7d4c85ac48437f139d66df648fc0e1d4ae`
- MuJoCo timestep `0.005 s`
- 정면·상향 카메라 `1280x720 @ 10 Hz`
- YOLO 모델 `sim/current/assets/yolo/best.pt`
- ROS 기반 `/mission/score_release`와 물리 가점존 판정
- lane/surface 동적 단일 `/mavros/rc/override` 소유권

ArduPilot, QGroundControl, 빌드 산출물과 로그는 DEB에 넣지 않는다. 설치기가
ArduPilot/QGroundControl을 내려받고 ROS 패키지는 대상 PC에서 빌드한다.
