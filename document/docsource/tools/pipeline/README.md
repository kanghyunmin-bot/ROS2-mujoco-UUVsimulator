# pipeline tools

sim-to-real 검증 파이프라인 실행 코드입니다.

## Groups

- `analyze_*`
  - 실제 rosbag, MAVROS bag, mode-path bag 분석.
- `replay_*`
  - April 1 rosbag 명령을 MuJoCo/SITL에 재생.
- `compare_*`
  - real-vs-sim RMSE, 신뢰도, 그래프 비교.
- `measure_*`, `run_actual_*`
  - 실제 MAVROS step/mode-path 측정.
- `run_uuv_param_autotune.py`
  - 후보 파라미터 sweep 실행.
- `identify_*`, `sweep_*`, `validate_*`
  - inverse dynamics, 축 gain sweep, 센서 frame contract 검증.

기존 명령과 GUI는 `document/docsource/<script>` wrapper를 통해 계속 동작합니다.
