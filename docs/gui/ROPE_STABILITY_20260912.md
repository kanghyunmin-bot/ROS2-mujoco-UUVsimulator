# 줄 발산 수정 및 대회 UI 숨김 — 2026-09-12

## 확인된 현상

실제 종료 로그: `MUJOCO_LOG.TXT`, 2026-09-11 16:44:59 UTC (한국 9월 12일 01:44:59). 시뮬레이션 30.466667초, DOF 62에서 BADQACC. 저장 상태 `generated/physics_failures/20260911T164459_560618.npz`의 두 번째 부표 줄 관절 각속도 최대 약 1.41e6 rad/s. 이 상태에서 가속도는 약 2.02e12까지 계산되어 기존 종료 가드가 작동했다. GUI나 타일 재질의 오류가 아니다.

수조는 이미 implicit 적분기였다. 정상 수직 자세에서 줄을 분리한 40초 시험은 Euler, implicitfast, implicit 모두 통과했다. 따라서 적분기 교체만을 해결책으로 삼지 않았다.

## 수정

- `gui/web_static/style.css`: Pinger Homing 및 Mission Check 패널을 숨겼다. DOM 및 기존 이벤트 처리는 유지한다. Mission FSM 패널은 별개로 유지한다. 실제 localhost GUI의 접근성 화면에서 두 패널이 표시되지 않음을 확인했다.
- `gui/research_pool_rope.py`, `scenes/research_pool_slam_scene.xml`: 18개 줄 ball joint의 armature를 1e-6 → 1e-5 kg m²로 변경했다. GUI에서 줄을 다시 생성해도 유지된다. 작은 관절 관성 때문에 접힌 줄의 충돌·회전 응답이 발산하는 재현 조건을 안정화한다. **실측 관성 보정이 아니라 수치 안정화 가정**이다. 몸체 질량·6관절 구성·접촉·줄 감쇠·15N 자석 기준은 유지한다. 동적 반응은 armature 변경의 영향을 받는다.
- `sim/runtime/physics_step_guard.py`: 향후 실패 기록에 qacc, warm-start 가속도, 적분기, 관절 이름, armature와 damping을 추가했다. 수치 실패를 감추거나 경고 후 자동 재시작하지 않는다.

## 회귀 검증

저장된 접힌 자세와 equality 상태를 사용하고, 이미 발산한 속도는 1e-5배로 낮춰 시작하는 합성 재현이다. 실제 조종 명령·접촉 이력의 완전 재생은 아니다. 전후 XML을 각각 새로 컴파일한다. 실행 중 armature만 바꾸면 컴파일 시 계산된 상수까지 동일하게 바뀌지 않으므로 그것을 최종 회귀 근거로 사용하지 않는다.

| 시험 | 결과 |
|---|---|
| 기존 armature, 접힌 자세 | 1.274167초에 일반화 속도 209,826; 실패 |
| 수정 armature, 동일 자세 | 20초 통과, 최대 일반화 속도 112.69, 경고 0 |
| 수정 설정 장기 시험 | 60초 통과, 경고 0 |
| 자석·줄 기능 | 정지 하중 0.980N, 10N 추가 하중에서 10.980N 유지, 20N 추가 하중 분리 통과 |
| 분리 후 동작 | 부표 상승, 줄 처짐, 로봇-줄 충돌, 붙은 줄의 굽힘 통과 |
| 유체 소유권 | 줄의 native 물 저항 유지, 차량 중복 유체 힘 없음 |
| 수치 실패 가드 | 의도한 NaN 입력에서 진단 저장 후 정지 통과 |
| 수조 계약 | 10×5×5m, 3개 부표, 형상 예산 검사 통과 |

증거: 저장소 `outputs/rope-stability-20260912/`의 `regression-before.log`, `regression-after.log`, `folded-long.log`, `magnet-rope.log`, `fluid.log`, `reset-guard.log`. `folded.json`은 탐색용 실행 중 파라미터 변경 결과이며 최종 전후 비교는 regression 로그를 따른다.

```bash
# 저장소 루트. 기존 설정은 의도적으로 실패해야 한다.
/home/khm/robotics/IsaacLab/isaaclab.sh -p uuv_mujoco/current/tools/check_folded_rope_stability.py uuv_mujoco/current/generated/physics_failures/20260911T164459_560618.npz --without_fix
/home/khm/robotics/IsaacLab/isaaclab.sh -p uuv_mujoco/current/tools/check_folded_rope_stability.py uuv_mujoco/current/generated/physics_failures/20260911T164459_560618.npz --seconds 60
/home/khm/robotics/IsaacLab/isaaclab.sh -p uuv_mujoco/current/tools/check_research_pool_magnet_rope.py
```

## 남은 위험 및 적용

얇고 가벼운 줄과 단단한 CAD 접촉, 접힌 줄의 자기 접촉은 여전히 큰 순간 속도를 만들 수 있다. 60초 통과는 모든 조종·충돌에서 안정함을 증명하지 않는다. 실제 종료 상태가 이미 발산한 뒤의 기록이라 발산 시작 순간의 정확한 접촉 쌍은 확정하지 않았다. 충돌을 끄거나 줄을 고정하거나 임의 속도 클램프로 덮지 않았다.

GUI 새로고침으로 숨김을 적용하고, 시뮬레이션은 새로 시작해야 변경된 관절 값이 로드된다. 리서치 맵에서 부표 분리 후 줄을 옆으로 밀고, 바닥에 내려앉은 줄에 천천히 재접촉하는 조종 시험이 다음 확인 항목이다. 재발하면 새 진단 파일로 추가 분석한다.
