# 제출 피규어 · UUV-SIM 플랫폼

## Fig. 1: 전체 시스템 연결

명령·실물 로그 재생 → ArduSub SITL → MuJoCo 물리 → 센서·카메라, 그리고 Real2Sim 평가와 VLA 수집을 보여준다. SITL의 상태·내부 IMU 피드백은 MuJoCo에서 직접 돌아오며, ROS 카메라 영상이 SITL에 입력되는 것으로 표시하지 않는다. 1쪽 하단에 배치한다.

[PNG](/home/khm/robotics/underwater/ROS2-mujoco-UUVsimulator/docs/papers/ut27_2027/figures/fig1_platform_architecture.png) · [SVG](/home/khm/robotics/underwater/ROS2-mujoco-UUVsimulator/docs/papers/ut27_2027/figures/fig1_platform_architecture.svg) · [PDF](/home/khm/robotics/underwater/ROS2-mujoco-UUVsimulator/docs/papers/ut27_2027/figures/fig1_platform_architecture.pdf)

## Fig. 2: 실제 구현 화면

CAD 기반 로봇, 전방 카메라, 손 카메라를 한 줄로 배치한다. 기존 MuJoCo 렌더에서 가져왔으며 전방·손 영상에는 거리별 수중 광학이 적용되어 있다. 그림을 그럴듯하게 만들기 위한 생성형 이미지나 물체 변경은 사용하지 않았다. 비교 PNG에서 원래 라벨만 제외하고 완전한 카메라 패널을 추출했다. 2쪽 상단에 배치한다.

[PNG](/home/khm/robotics/underwater/ROS2-mujoco-UUVsimulator/docs/papers/ut27_2027/figures/fig2_platform_views.png) · [SVG](/home/khm/robotics/underwater/ROS2-mujoco-UUVsimulator/docs/papers/ut27_2027/figures/fig2_platform_views.svg) · [PDF](/home/khm/robotics/underwater/ROS2-mujoco-UUVsimulator/docs/papers/ut27_2027/figures/fig2_platform_views.pdf)

## Table I: 누적 구현 검증

동일 105개 샘플의 계산 최적화, 두 실물 기록의 속도 오차 변화, VLA 연결 검사 결과를 담는다. 서로 다른 시험 조건을 같은 성능 벤치마크로 묶지 않는다. 표 문구는 `manuscript.json`, 원자료 해시는 `platform_verification.json`에 있다.

기존 yaw 그래프는 Real2Sim 보조 자료로 유지하며 초록의 주 피규어에서 제외했다. 새 초록은 시뮬레이터 전체의 설계·구현이 중심이다.

## Figure 3 — Response comparison (current replacement for Table I)

(a,b) Two full evaluation windows of DVL surge versus nominal and latest calibrated simulation. (c) Release yaw-rate RMSE across all selected 2.5 s windows; clock-corrected baseline differs from the nominal baseline in (a,b). No path plot, fitted lag, scale or smoothing. Source and 8 numerical checks: `response_verification.json`. PNG/PDF/SVG: `figures/fig3_response_comparison.*`. The old Table I is removed.

## Figure 2 refresh — same-state capture

Replaced historical mixed-version screenshots with a single MuJoCo model and static state at t=1.2 s. (a) Pool overview and surface lighting; (b) close vehicle view; (c) buoy, white braided rope and attachment close-up, replacing the front-camera panel at user request. Shared geometry/material arrays remain unchanged. No live SITL restart or physics stepping. Full uncropped frames, 960×540, with current water shaders. Capture script and SHA-256 provenance: `capture_platform_views.py`, `figures/platform_capture/capture.json`.
