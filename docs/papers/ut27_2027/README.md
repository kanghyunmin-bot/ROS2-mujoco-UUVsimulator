# UT27 · UUV-SIM 플랫폼 초록

## 논문 중심

**UUV-SIM: Design and Implementation of an Integrated Underwater Robot Simulation Platform Based on MuJoCo**

MuJoCo를 활용해 수중로봇 물리·SITL 제어·센서·시각화·데이터 수집을 통합한 UUV-SIM의 설계와 구현이 중심이다. 대회는 초기 요구사항의 출처로 간략히 설명하고, 최근 yaw 수정은 Real2Sim 검증의 일부로만 반영했다. 최초성이나 Gazebo·Stonefish·OceanSim 대비 우월성을 측정 없이 주장하지 않는다.

## 결과물

- [한국어 검토 PDF](/home/khm/robotics/underwater/ROS2-mujoco-UUVsimulator/output/pdf/ut27_abstract_ko.pdf) · [한국어 Word](/home/khm/robotics/underwater/ROS2-mujoco-UUVsimulator/output/doc/ut27_abstract_ko.docx)
- [영문 제출 검토 PDF](/home/khm/robotics/underwater/ROS2-mujoco-UUVsimulator/output/pdf/ut27_abstract_en.pdf) · [영문 Word](/home/khm/robotics/underwater/ROS2-mujoco-UUVsimulator/output/doc/ut27_abstract_en.docx)
- [한국어 원문](/home/khm/robotics/underwater/ROS2-mujoco-UUVsimulator/docs/papers/ut27_2027/abstract_ko.md) · [영문 원문](/home/khm/robotics/underwater/ROS2-mujoco-UUVsimulator/docs/papers/ut27_2027/abstract_en.md)
- [피규어 구성](/home/khm/robotics/underwater/ROS2-mujoco-UUVsimulator/docs/papers/ut27_2027/figure_plan.md) · [근거 정리](/home/khm/robotics/underwater/ROS2-mujoco-UUVsimulator/docs/papers/ut27_2027/source_ledger.md) · [제출 입력](/home/khm/robotics/underwater/ROS2-mujoco-UUVsimulator/docs/papers/ut27_2027/submission_fields.md)

영문은 본문 570단어, 2쪽 확장 초록이다. 별도의 짧은 Abstract나 번호를 붙인 본논문 절은 없다. [UT27 공식 안내](https://ut27.org/authors.html)의 2쪽·본문 500–1000단어·그림과 표 포함 PDF 요구를 기준으로 했다. 공식 문구에서 IEEE 2단 4–10쪽 요구는 이후 본논문에 해당한다. 현재 1단 편집은 초록을 위한 선택이다.

## 원고 구성

1. 수중로봇 개발 환경의 요구와 MuJoCo 선택 이유.
2. 타원체 유체력과 선택 가능한 분산 모델, CAD·추진기·접촉 구현.
3. 실제 운용 제어기를 유지한 ArduSub SITL 폐루프 연결.
4. 센서와 수중 시각화, GUI와 원본 센서 발행 분리.
5. VLA 수집·LeRobot·U0 입력 연결.
6. 기존 구현 대비 계산 최적화 및 진행 중인 Real2Sim 검증.
7. 공개 코드와 재사용 가능한 플랫폼으로서의 기여.

타원체와 분산 유체력 모델은 구분했다. 현재 Real2Sim의 분산 모델 결과를 내장 타원체 모델 단독의 정확도 또는 속도로 설명하지 않는다. 계산 최적화는 같은 105개 샘플의 힘·토크 평가 2.65→0.47 ms이며, 전체 실행 속도나 다른 엔진과의 비교가 아니다. VLA는 비무장 연결 검사 51행과 실제 로더의 36개 청크를 확인한 단계이다.

## 편집 및 검증

본문·그림은 기존 코드와 기록에 근거했다. 새 시뮬레이션 실험은 실행하지 않았다. `platform_verification.json`은 계산 프로파일, 1500스텝 비교, VLA 검사, 실제 렌더의 해시와 수치 확인 결과를 담는다. 기존 `figure_verification.json`과 `metrics.csv`는 실물 비교의 상세 보조 근거로 유지했다. 이전 yaw 중심 그림은 보조 자료이며 이번 제출본에는 들어가지 않는다.

A4, 본문 11 pt, 제목 20 pt, 실제 렌더, 구조도, 3패널 응답 비교 그림 총 3개로 구성했다. Word와 PDF의 모든 페이지를 렌더링해 확인했다. PDF에는 Liberation Serif가 포함된다. DOCX가 요청하는 Times New Roman 자체가 포함되었다고 주장하지 않는다. 템플릿 출처와 의도적인 편집 변경은 `template/README.md` 및 `template_preservation.json`에 기록했다.

영문 성명·소속·이메일은 이전 Notion 기록을 반영했고 본인 확인은 대기 중이다. 사용자 요청에 따라 감사의 글은 삭제했다. 현재 Ubuntu 22.04 / ROS 2 Humble 통합 배포를 기준으로 하며, macOS나 비 NVIDIA 환경에서 전체 시스템을 검증했다고 쓰지 않았다.

**Notion 수정, Git 커밋·푸시, 학회 등록·업로드는 수행하지 않았다.**

## 재생성

```bash
/home/khm/robotics/IsaacLab/isaaclab.sh -p docs/papers/ut27_2027/build_platform_figures.py
/home/khm/.cache/codex-runtimes/codex-primary-runtime/dependencies/python/bin/python3 docs/papers/ut27_2027/build_documents.py
```

영문 원문은 `manuscript.json`, 한국어는 `abstract_ko.md`에서 수정한다. 재생성 후 다시 PDF로 렌더링하고 두 페이지 및 글자·표·그림을 확인해야 한다. 원시 bag과 일부 분석 산출물은 로컬 자료이며 Git에 모두 포함되어 있지 않다.

최신 응답 보정은 `response_verification.json`에서 원자료 8개 RMSE를 검산했다. Figure 3의 (a,b)는 명목 모델 대비 최신 보정의 전진 속도, (c)는 시간 보정 모델 대비 응답 보정의 해제 각속도 오차다. B의 전체 yaw 각도 오차 증가도 본문에 명시했다. 중복 검증 표는 그림 3으로 대체했다.

Figure 2는 같은 모델·재질·정적 상태에서 새로 렌더한 수면 전경, 로봇, 부표·밧줄 근접 장면으로 교체했다. 카메라 출력 이미지는 사용자 요청으로 제외했다. 물리 스텝이나 SITL 실행 없이 기존 조명 렌더 경로로 캡처했다.
