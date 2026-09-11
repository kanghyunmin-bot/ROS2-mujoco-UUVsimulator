# 실물 bag 참고 테스트 수조 외관

2026-09-12. GUI의 `test_tank` 장면 생성에 적용된다. 경쟁 수조와 연구용 수영장 장면은 변경하지 않는다. GUI에서 테스트 수조를 선택하고 시뮬레이션을 다시 시작하면 적용된다. 저장된 course_layout.json의 배치를 사용한다.

근거: `outputs/real-bag-audit-20260911/color_0000.jpg`~`color_0400.jpg`. 영상에서 파란 사각 패턴과 밝은 줄눈의 라이너가 벽과 바닥에 이어지는 것을 확인했다. 이전 큰 청록 타일 패널을 작은 반복 무늬로 교체했다. 무늬의 실제 치수·색 보정값은 측정되지 않았다. 천장 구조는 영상으로 확정되지 않아 높이 3m의 단순 천장, 보 3개, 발광 조명판 3개로 근사했다. 천장·조명은 실물 복원 근거로 인용하지 않는다.

구현: `gui/test_tank_appearance.py`. MuJoCo 내장 32×32 텍스처와 벽면용 시각 평면 4개를 사용한다. 박스 옆면 UV의 늘어짐을 피했고 타일 개수만큼 형상을 만들지 않는다. 새 외부 이미지 의존성이나 광원은 없다. 천장 때문에 기존 광원이 수조를 완전히 가리지 않도록 테스트 수조에서 그림자를 비활성화했다. 천장은 아래쪽 단면으로 전체 보기에서 수조를 가리지 않도록 했다. 천장·보·테두리는 시각 그룹 2로 숨길 수 있다. 수중 광학, 굴절, 탁도, 카메라 노출 모델을 새로 보정한 것은 아니다.

검사 결과 (`outputs/tank-appearance-20260912/check.json`):

- 질량, 관성, 몸체 위치, 관절, 감쇠, 초기 qpos 동일.
- 기존 충돌 형상의 위치·방향·크기·마찰·접촉 파라미터 동일.
- 추가 장식은 contype=0, conaffinity=0.
- 전체 geom 889→892, 텍스처 메모리 약 21KiB 증가.
- 320×240 전방 카메라 30회 렌더 중앙값: 기존 0.530ms, 변경 0.191ms. EGL 단독 렌더의 짧은 측정으로, GUI 포함 전체 속도 향상을 입증하는 수치는 아니다.

재현 (저장소 루트):

```bash
MUJOCO_GL=egl /home/khm/robotics/IsaacLab/isaaclab.sh -p outputs/tank-appearance-20260912/preview.py
MUJOCO_GL=egl /home/khm/robotics/IsaacLab/isaaclab.sh -p outputs/tank-appearance-20260912/check.py
```

전후 카메라: `outputs/tank-appearance-20260912/before_underwater.png`, `after_underwater.png`. 전체 장면: `overview.png`, `room.png`. 렌더는 실제 MuJoCo 장면이며 별도 이미지 합성이나 색 보정을 하지 않았다. 기존 파일 스냅샷은 `test_tank_layout_model.before.py`에 보존했다.

## 리서치 맵 추가 적용

사용자 요청에 따라 `scenes/research_pool_slam_scene.xml`(10×5×5m)에도 동일한 파란 라이너 색·줄눈·반복 밀도를 적용했다. 기존 pool_tile 재질과 pool_tile_texture만 변경했다. 테스트 수조 외관은 그대로 유지했다. 리서치 맵의 천장이나 형상은 추가하지 않았다. XML 비교로 다른 요소가 동일함을 확인했고 MuJoCo 컴파일 및 전방 카메라 렌더를 확인했다. 증거와 재현 스크립트: `outputs/research-pool-appearance-20260912/`. GUI에서 Research pool을 선택하고 재시작하면 적용된다.
