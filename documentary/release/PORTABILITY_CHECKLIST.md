# Ubuntu 배포 검증 체크리스트

## 개발 PC에서 생성

```bash
./packaging/build_release.sh
./packaging/verify_release.sh \
  documentary/release/builds/2026.08.01/kmu-auv-simulator_2026.08.01_amd64.deb
cd documentary/release/builds/2026.08.01
sha256sum -c SHA256SUMS.txt
```

## 깨끗한 Ubuntu 22.04 amd64에서 확인

- DEB 더블클릭 설치 및 앱 메뉴 아이콘 생성
- first-run 안내창, 터미널 로그, 관리자 암호 요청
- 동일 버전 복구 설치
- `/opt/kmu-auv-simulator` 삭제/업데이트 시 사용자 작업공간 보존
- MuJoCo viewer 및 headless 실행
- SITL 연결, ARM, RC 이동
- 정면/상향 카메라 1280x720@10Hz
- `/audio`, `/depth/pose`, DVL, Ping360 토픽
- `competition_a_lane_mission.launch.py` 실행
- `/mavros/rc/override` publisher 한 개
- `/mission/score_release`, `/collector/state`, 누적 count 토픽

필요 포트는 `8878`, `14550`, `14551`, `14660`, `14661`, `9002`, `9003`이다.
Wayland에서는 XWayland/libdecor를 사용하며 GLFW의 window-position 경고는
비치명적이다.

배포 ROS source에는 다음 14개 패키지가 있어야 한다.

```text
dvl_msgs auv_dvl_a50_msg ping360_sonar_msgs auv_msg auv
audio_common_msgs audio_common audio_capture hydrophone_ctrl
auv_buoy_vision_control auv_lane_vision_control auv_web_gui
auv_pinger_homing robot_localization
```
