# 추가 실물 데이터 없이 수행한 보강 — 2026-09-12

## 결론

확인된 정책 제어권 및 데이터 시각 검증 결함을 수정했다. 보정 프로파일에서 새로 수집한 정지 50프레임을 LeRobot 변환과 조직 U0 로더·전처리까지 전달했다. 이는 파이프라인 검증이며 작업 시연 품질, 학습 성공, 실물 전이를 검증하지 않는다. 추력·항력 보정값은 추가 근거 없이 변경하지 않았다.

## 결함과 수정

| 분류 | 근거 파일 | 영향 및 조치 |
|---|---|---|
| 확인된 결함 | `rospkg/src/kmu26_auv_vla_policy/kmu26_auv_vla_policy/kmu26_ros.py`, `_ready` | 수집기는 RC 발행자 수를 검사하지만 정책은 검사하지 않았다. 실제 RC 출력에서 해당 토픽 발행자가 정확히 1개일 때만 준비 상태를 허용한다. 활성 실행에서도 기존 `_tick` 경로로 정지한다. dry-run은 실제 RC를 발행하지 않으므로 유지한다. |
| 확인된 결함 | `rospkg/src/auv_vla_data_collector/kmu26_auv_vla_data_collector/export_lerobot.py` | NaN 센서 시각으로 영상·행 파일 내보내기가 성공했다. 필수 source_timestamp 및 존재하는 receipt_timestamp의 N×7 형태와 유한성을 출력 생성 전에 검사한다. 오래된 파일에서 선택 필드가 없다는 이유만으로 거부하지 않는다. |
| 확인된 결함 | `rospkg/src/auv_vla_data_collector/kmu26_auv_vla_data_collector/training_input.py` | NaN은 시간차 <= 0 조건에 걸리지 않았다. 학습 상태 N×23, 센서 시각 N×7 및 유한성을 명시적으로 검사한다. |
| 검증할 위험 | ROS graph / MAVLink 제어권 | 발행자 수 검사는 ROS 경로에 한정된다. 직접 MAVLink 조종, DDS 탐지 지연, FCU의 실제 명령 채택까지 입증하지 않는다. |
| 현재 제약 | April bag / July parameter snapshot | 단일 bag 보정의 식별 불확실성과 수집 시점 차이는 그대로 남는다. 새 보정이나 real-to-sim 완료를 주장하지 않는다. |

각 신규 결함은 수정 전 실패와 수정 후 통과를 확인했다. 원본 코드 스냅샷과 실제 런타임 변경만의 패치는 `outputs/offline-hardening-20260912/source-before/`, `runtime-changes.patch`에 있다. 기존 미커밋 변경은 유지했다. 테스트 변경은 아래 3개 파일의 신규 회귀 항목이다.

- `rospkg/src/kmu26_auv_vla_policy/test/test_policy_time.py`
- `rospkg/src/auv_vla_data_collector/test/test_transfer_regressions.py`
- `rospkg/src/auv_vla_data_collector/tools/test_u0_transfer_input.py`

## 실행 및 검사 결과

| 검사 | 결과 | 증거 (outputs/offline-hardening-20260912 기준) |
|---|---|---|
| 수집·정책 ROS 단위/변환 시험 | 37 통과 | regression-after.log |
| 실제 U0 환경 학습 입력 시험 | 5 통과 | training-after.log |
| 물살 배치 계산 동등성·카메라 intrinsics·SITL heartbeat | 6 통과 | physics-camera.log |
| MuJoCo NaN 수치 리셋 차단 | 통과, 의도한 NaN 경고 발생 | reset-guard.log |
| 신규 정지 수집 | 50행, 시뮬레이션 시간 간격 0.1초 | collection.log, recording-check.json |
| 두 카메라 수집 유효 프레임 | 각각 50개 고유 원본 시각 / 50행 | recording-check.json |
| 수집 시점 카메라 최대 나이 | 각각 0.065초 | recording-check.json |
| LeRobot 내보내기 | 1 episode / 50 frames | export.log |
| 실제 U0 영상 디코딩·전처리 | 완전한 16행동 청크 35개, 상태 (1,64), 행동 (16,32), 유효 마스크 23/64 | loader-result.json, loader.log |
| 치명적 Python lint 선택 검사 | 통과 | lint-critical.log |

전체 Ruff 규칙에서는 기존 스타일·광범위 예외 처리 등을 포함한 11개 지적이 남았다(lint.log). 전체 lint 통과로 보고하지 않는다. 기존 정책의 예외 발생 시 정지 동작은 유지했다.

새 수집은 격리된 ROS_DOMAIN_ID=89, headless, bag0402_effective, 넓은 진단 수조, 초기 위치 (-2,1,-0.6)m에서 실행했다. 비무장·초기 수심 유지 상태이므로 부력 평형이나 제어 안정성 실험이 아니다. 두 카메라 요청 발행률은 30Hz, 해상도 320×240이다. 측정은 수집기에 들어온 10Hz 표본에 대한 것으로 전체 발행 30Hz 달성률을 뜻하지 않는다.

실행 로그에서 step 약 800Hz, 전체 계산 약 1.00ms, 힘 계산 약 0.42ms가 관측됐다. GUI·연구 수영장과 다른 headless 진단 조건이므로 이전 GUI 성능 수치와 직접 비교하지 않는다.

## 재현

저장소 루트에서 실행한다. 기존 Docker 이미지와 앞선 감사의 선택적 내보내기/U0 의존성을 사용한다. 새 필수 의존성은 추가하지 않았다.

```bash
# ROS 컨테이너 내부: /workspace는 이 저장소를 바인드 마운트한 경로
source /opt/ros/humble/setup.bash
source /workspace/rospkg/install/setup.bash
export PYTHONPATH=/workspace/outputs/vla-transfer-audit-20260910/export-deps:/workspace/rospkg/src/auv_vla_data_collector:/workspace/rospkg/src/kmu26_auv_vla_policy:$PYTHONPATH
/usr/bin/python3 -m pytest -ra /workspace/rospkg/src/auv_vla_data_collector/test/test_*.py /workspace/rospkg/src/kmu26_auv_vla_policy/test/test_*.py
```

```bash
# 호스트: 저장된 신규 수집 결과로 실제 U0 전처리 재검사
bash outputs/offline-hardening-20260912/run_loader.sh
bash outputs/vla-transfer-audit-20260910/run_training_tests.sh
/home/khm/robotics/IsaacLab/isaaclab.sh -p outputs/offline-hardening-20260912/check_recording.py
```

수집 재현 스크립트는 `start_stack.py`, `collect_smoke.py`, 실제 명령·scene·profile·FCU params는 `live/`, 출처 스냅샷은 `acquisition-context.json`에 보관했다. 재수집은 새 출력 디렉터리와 새 세션 ID를 사용한다. 사용한 `uuv-offline-hardening` 컨테이너는 검사 후 정지했다.

## 물리 및 타 엔진 비교 범위

Stonefish 1.5.0 로컬 소스를 별도 `stonefish-build/`에서 빌드했다. 누락된 GLM은 산출물 디렉터리에만 내려받고 압축 해제했다. 시스템 설치와 원본 Stonefish 수정은 하지 않았다. 라이브러리 빌드 성공은 ROV 정확도·속도 비교 결과가 아니다. 동일 질량·관성·추진기·항력·제어기·시간간격을 맞춘 동작 비교는 이번 실행에서 수행하지 않았다.

현재 물리 값의 근거와 불확실성은 `HORIZONTAL_RESPONSE_CALIBRATION_20260912.md`를 따른다. 추가 단일 bag 맞춤 최적화로 오차만 줄이는 대신 동작 범위 밖의 유효성은 미검증으로 유지한다. 실제 로봇 준비 후에는 질량·배수량, 정지 부상률, 축별 명령 계단/중립 감속, 양방향 yaw, 수심 유지 시험과 센서·RC·모터 출력 동시 기록이 우선이다.

현재 새 데이터는 연결·변환·로더 회귀 검사용이다. 정상 학습 경로는 connection_check를 계속 거부한다. 카메라 2개, 제어권, 모드, 시간 연속성이 확인된 실제 작업 시연과 별도 검증 세션이 확보되어야 작업 VLA 학습/평가 자료로 판단할 수 있다.
