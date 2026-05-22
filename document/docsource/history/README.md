# history

초기 분석 단계의 JSON 산출물을 시기별로 보관합니다. 현재 실행 파이프라인의
입력은 아니지만, 어떤 판단을 거쳐 현재 구조가 되었는지 추적하기 위한 자료입니다.

## Folders

- `2026-04-29_axis_priority_tuning/`
  - 축별 RMSE 우선순위, non-yaw gain, vertical/horizontal gain sweep, 센서 부호 확인.
- `2026-04-30_command_mapping/`
  - joystick-to-RC override 매핑과 명령 방향성 확인.
- `2026-05-06_heave_inverse_checks/`
  - heave inverse dynamics, direct/conservative parameter 비교, 부호 sweep.

새 분석 결과가 현재 보고서 입력으로 쓰이면 `../metrics/report_inputs/`에 두고,
과거 판단 근거만 보관하려면 이곳에 둡니다.
