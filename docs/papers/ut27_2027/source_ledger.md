# UUV-SIM 플랫폼 원고 근거

원고의 중심은 MuJoCo 기반 통합 플랫폼의 설계·구현이다. 이전 yaw 중심 원고를 대체한다. Notion과 기존 GPT 세션은 개발 목적과 이력을 확인하고, 수치는 코드·실행 로그·JSON에서 확인했다. Notion 수정과 새 시뮬레이션 실행은 없었다.

## 코드와 검증 자료

| 내용 | 직접 근거 | 해석 |
|---|---|---|
| 플랫폼 기능·공개 배포 | README.md, docs/versions/2026.09.14.md, Git 이력 | 접근성과 재사용성을 위한 실제 구현. 타 플랫폼보다 우월하다는 비교는 없음 |
| 내장 타원체·사용자 정의 모델 분리 | sim/physics/fluid_model.py, fluid_contract.py, fluidcoef_immersion_runtime.py | 내장 유체력과 분산 모델의 계산 담당을 구분하고 중복 적용 방지 |
| 분산 계산 최적화 | docs/gui/HYDRODYNAMIC_RUNTIME_OPTIMIZATION.md, outputs/hydro-opt-20260910/profile-before.txt 및 profile-after.txt | 같은 105개 샘플의 독립 힘·토크 평가 2.6471→0.4672 ms. 엔진 간 비교 또는 전체 실시간 성능이 아님 |
| 물리 출력 유지 | outputs/hydro-opt-20260910/trajectory.txt | 1500스텝의 해당 시험 궤적에서 wrench와 qpos 차이 0. 일반적인 bitwise 동일성은 주장하지 않음 |
| 센서와 시간 계약 | docs/contracts/REAL_STACK_PARITY.md, PRE_VLA_COLLECTOR_READINESS_20260914.md | 실제 로봇 소프트웨어와 연결할 인터페이스. 센서 노이즈·광학 계수의 실측 인증은 아님 |
| 카메라 시각화 | docs/assets/uuv-update-20260914/robot.png, underwater-camera-comparison-20260914.png | 실제 렌더 패널. 사진이나 임무 성공 장면이 아님 |
| VLA 연결 | docs/assets/pre-vla-motion-audit-20260914.json, pre-vla-loader-20260914.json | 51행·23D state, 36개 16-step chunk, inspection only, optimizer/inference false |
| 실물 비교 | docs/assets/real2sim-application-20260915/comparison.json, yaw-timing-20260915/full_comparison.json | 두 기록의 명목→최종 속도 RMSE와 최종 yaw. 독립적인 최종 test set이 아님 |

원본 해시와 값 확인은 `platform_verification.json`에 있다. `figure_verification.json`의 기존 4조건 × XY/yaw RMSE 재계산은 보조 근거로 유지했다. 그림 및 표 생성에서는 해당 값을 새로 fitting하지 않았다.

## Real2Sim 조건 및 현재 상태

A는 2026-04-02 21:46:20 기록의 20–70초, B는 21:40:11 기록의 12–110초이다. A가 물리 계수 추정에 사용되었고 B도 시간 문제 진단에 사용되었다. 수신 시각과 초기 자세만 정렬했으며, 오차를 줄이는 시간 변형·경로 배율 최적화는 하지 않았다. 실물 위치는 FCU 추정치이고 외부 ground truth는 없다. 제어 설정은 7월 tlog 복구본에 근거하므로 4월과 동일하다고 인증하지 않는다. 명목 20 V는 실제 ESC 단자 전압 측정이 아니다.

로컬 Git HEAD는 `5795b6c`, branch는 `codex/bag-trend-alignment`이다. 9월 15일 Real2Sim·clock 적용 결과와 일부 실행 변경은 미커밋 상태로 존재하므로, HEAD만으로 모든 결과를 재현할 수 있다고 주장하지 않는다. ArduSub checkout `2dd0bb7d4c85ac48437f139d66df648fc0e1d4ae`에 clock 반올림 변경이 로컬 적용되어 있다. 논문 작업은 이를 변경하거나 커밋·푸시하지 않았다.

## 설계 동기와 검증 범위

대회용 ROV 개발에서 출발해, 특정 GPU 의존도와 계산 부담을 낮추고 재사용할 수 있는 환경을 만들려는 목적은 사용자의 개발 배경 설명이다. 현재 전체 통합 배포는 Ubuntu 22.04 / ROS 2 Humble을 기준으로 한다. macOS·비 NVIDIA 전체 실행 성공이나 Gazebo·Stonefish·OceanSim 대비 동등 정확도·더 낮은 비용은 측정 없이 결과로 넣지 않았다.

## Notion 참조

| 페이지 | 사용한 내용 |
|---|---|
| [UUV_sim](https://app.notion.com/p/061ad8acb9a1835c9a0001ffad8e5813) | 전체 개발 이력과 하위 분석 페이지 탐색 |
| [07 · 9월 14일 — 물리·센서 계약·경량 렌더링과 수집 준비](https://app.notion.com/p/3dbad8acb9a1813baae9dee2b6eec7d2) | 물리·센서 구현 범위, 경량 시각화, 수집 준비와 학습 검증의 구분 |
| [05 · 9월 — 실물 rosbag 기반 응답 비교와 조건부 보정](https://app.notion.com/p/3d8ad8acb9a18149a393edc900f9f384) | 조건부 보정·비식별성·위치 기준의 한계 |
| [UUVSim 랩미팅 핵심 성과 정리](https://app.notion.com/p/35cad8acb9a18103863bee240a2b1433) | 이전 시기 성과 확인. 과거 depth 및 yaw scale 결과는 최신 수치와 혼합하지 않음 |
| [[2026] KMU-HIT 교류 프로그램](https://app.notion.com/p/swan0421/2026-KMU-HIT-2edb9c2fcf86800ea7a1c304f37e48b5) | 팀 프로젝트·bag 자료 위치·저자 정보. 개인 workspace connector에서 404로 접근 불가하여 이미 열린 브라우저에서 읽음 |

팀 페이지의 불필요한 개인 연락처나 금융 정보는 수집·전재하지 않았다. 내려받기 서명 URL이나 인증 정보도 이 문서에 보관하지 않는다.

## GPT / Codex 세션 참조

| 실제 세션 제목 | 식별자 | 사용 범위 |
|---|---|---|
| 논문 방향성 정리 | `6aa6a9af-fed0-83e8-a50c-3deed93d2a61` | VLA/CAP/action-space 계획의 연구 맥락. 제안·계획을 완료한 실험으로 바꾸지 않음 |
| Audit ROV VLA sim to real | `01a08727-57b7-7a43-a188-db667432d540` | 기존 collector·state/action·camera 구조. 학습 정책의 실물 전이 증거는 없음 |
| 시뮬레이션 점검 및 개선 | `01a09c06-6732-7ca3-9ad4-6de64af01456` | 현재 physics/센서/실물 bag/clock 검증 흐름과 사용자 설명 |

접근 가능한 관련 세션을 읽었다. 계정의 모든 GPT 대화 전체를 검토했다고 주장하지 않는다. 이전 답변의 근거 없는 채택 확률이나 일반적인 기대치는 논문 근거로 쓰지 않았다.

## 문헌과 제출 정책

Stonefish의 원문 서지는 OCEANS 2019 Marseille, DOI `10.1109/OCEANSE.2019.8867434`이다. ICRA로 표기하지 않았다. [개발자 저장소](https://github.com/patrykcieslak/stonefish)의 인용 정보를 확인했다.

MuJoCo는 [IROS 2012 원문](https://ieeexplore.ieee.org/document/6386109/), ROS 2는 [Science Robotics 원문](https://doi.org/10.1126/scirobotics.abm6074)을 인용했다. 일반 검색 요약이나 GPT 생성 문헌을 참고문헌으로 사용하지 않았다.

마감·분량·절차는 [UT27 Authors](https://ut27.org/authors.html)에서 확인했다. IEEE AI 사용 안내에 따라 도구명과 초안·코드·분석 지원 범위를 Acknowledgment에 밝혔다. 저자 본인이 기술 내용과 최종 원고를 확인했다는 사실을 대리로 선언하지 않았다.
