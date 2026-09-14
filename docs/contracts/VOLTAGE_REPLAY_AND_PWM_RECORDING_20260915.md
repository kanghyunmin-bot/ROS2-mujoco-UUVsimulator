# 전압 재생과 최종 PWM 기록 보완 — 2026-09-15

## 반영한 변경

전압에 따른 T200 추력 변화 기능은 기존에도 있었다. 이번 변경은 **전압을 잘못된 시간이나 출처로 적용할 수 있었던 연결부**와 **고율 최종 PWM 기록 설정**을 보완했다. 추력 배율·항력·TF·GUI 기본 물리 프로필은 바꾸지 않았다.

| 수정 | 이전 동작 | 현재 동작 |
|---|---|---|
| 전압 재생 시각 | CSV의 0초를 시뮬 0초로만 해석 | 실제 RC 재생 시작에 맞춘 시각 오프셋 지원 |
| 전압 출처 | 두 열 CSV를 ESC 전압으로 간주 | 메타데이터가 있으면 ESC/직결 팩 여부, 출처, 파일 해시 검사 |
| 전압 누락/범위 | CSV 사이의 긴 구간도 보간, 끝값 유지 | 메타데이터의 최대 간격·범위 밖 정책 검사. 측정표 밖 전압은 계속 거부 |
| 수평 보정 도구 | 프로필의 고정 전압만 사용 | 시뮬과 같은 전압 로더·갱신 함수 사용, 전압 출처를 결과에 기록 |
| 최종 PWM 요청 | SITL 관측 주기는 최대 20Hz로 제한, MAVROS 설정에 PWM 옵션 없음 | 필요한 실행에서만 최대 100Hz를 요청하도록 설정 가능 |
| 기록 품질 | 요청 주기와 실제 저장 주기 구분이 어려움 | 원본 패킷의 실제 Hz·최대 공백·구간 양끝 커버리지 검사 |

하위 호환: 기존 전압 CSV는 끝값 유지 동작을 보존하며 `unverified_legacy_csv`로 표시한다. 일반 실행의 텔레메트리 기본 주기를 높이지 않았다. 새 검증은 주로 파일 로드 시 수행하며, 물리 업데이트에는 시각 오프셋과 범위 확인만 추가한다.

## 4월 2일 bag에 적용한 결과

원본 DB SHA256은 `e3f60d33dbf72f9c4df223f8afe7e389f8c49ffc6ff7031647ce3da32ec3c154`다. 20–70초의 `/battery`를 사용하고, 기존 `controller/replay.json`의 실제 재생 원점으로 시각을 맞췄다. 이 경우 CSV 0초는 **시뮬 18.00375초**에 대응한다. 다른 재생에서는 그 실행의 `origin_sim_time`을 사용해야 한다.

- 팩 전압: **19.375–22.515625V**, 중앙값 **21.765625V**.
- 경계 보간에 필요한 양끝 샘플을 포함한 1,002개 중 **986개가 현재 측정 곡선 상한 20V를 초과**한다. 기존 구간 내부 집계 1,000개/984개와 차이는 양끝 지원 샘플 두 개 때문이다.
- 실제 RCOut: **2.0001Hz**, 최대 간격 **0.5033초**. 50Hz 이상·최대 간격 0.05초라는 이번 입력 검사의 조건을 충족하지 않는다.
- ESC 단자 전압 또는 팩 직결 여부가 확인되지 않았다. 도구는 원본 관측 CSV와 원인 보고서를 저장하고, **사용 가능한 ESC 재생 CSV를 생성하지 않았다**. 상태 코드 2는 이 확인 결과이며 프로그램 실행 실패를 숨긴 것이 아니다.

결과: [준비 검사 JSON](../assets/voltage-replay-20260915/preparation.json).

사용자가 테더를 넉넉하게 풀었다고 추가 설명했으므로, 테더를 주원인으로 보는 조사 우선순위를 낮췄다. 현재 우선순위는 실제 ESC 전압·ESC 설정·장착 추력 확인이다. 전압 변동 자체는 bag에서 확인됐지만, 팩 전압을 ESC 전압으로 자동 대입하거나 20V로 잘라서 원인 분석을 진행하지 않는다.

## 검증한 수정 효과

새 회귀 검사 중 전압 관련 7개와 PWM 설정 관련 7개는 **수정 전 실패, 수정 후 통과**를 확인했다.

- 예시 전압 기록 `16→14→12V`와 재생 시작 6.25초를 사용하면, 시뮬 7.25초의 전압은 14V다. 이전 선택기는 시작 오프셋을 전달하지 않아 이미 마지막 12V를 사용했다. 수정 후 올바른 14V를 적용한다. 이 전압 기록은 합성 검사용이다.
- 실제 MuJoCo 실행에서 정·역추진 각각에 대해 16/14/12V의 추진기 힘을 제조사 저장 지점과 대조했다. 전압 갱신과 실제 `qfrc_actuator` 전달을 확인했다.
- 고정 20V의 기존 보정 결과 `k=0.12293316274117821`, `d2=35.120873813940435`는 그대로 재현됐다.
- 합성 고정 16V 설정과 16V CSV를 각각 넣었을 때, 추력 배열과 모든 보정 모델 결과가 원소별로 동일했다. 이 결과는 코드 경로의 일치 검사이며 실물 ESC가 16V였다는 근거가 아니다.
- 전압·기록 주기·T200·시각/노이즈·distributed·MAVLink 검사 **66개 통과**. 기존 ROS launch 시각 계약 검사도 통과했다. 새 검사는 MuJoCo 3.8/3.12 CI에 포함했다.

검사 요약: [validation.json](../assets/voltage-replay-20260915/validation.json). 실제 출력은 `outputs/voltage-replay-20260915/`에 보존한다.

## 사용법

프로젝트 루트에서 실행한다. 먼저 `extract_rosbag_trends.py`로 배터리와 정확한 수신 ns 시각이 포함된 데이터를 추출한다. 다음 도구는 ROS나 FCU에 연결하지 않는다.

```bash
/home/khm/robotics/IsaacLab/isaaclab.sh -p \
  uuv_mujoco/current/tools/prepare_propulsion_replay.py \
  --source_dir outputs/propulsion-audit-20260914/source \
  --replay_json outputs/horizontal-identification-20260912/controller/replay.json \
  --begin_bag_s 20 --end_bag_s 70 \
  --voltage_reference battery_pack_unverified \
  --provenance 'April 2 pack telemetry; ESC connection unconfirmed' \
  --output_dir outputs/voltage-replay-20260915/recheck
```

새로운 검증된 전압 데이터에서는 `--voltage_topic`으로 원본 토픽을 선택하고, `--voltage_reference esc_bus` 또는 실제 연결이 확인된 경우에만 `confirmed_direct_pack`을 사용한다. `--provenance`에는 측정 위치 또는 직결 확인 근거를 적는다. 이 선언 자체가 물리 검증을 대신하지 않는다. 시작/종료 시각을 생략하면 참조 재생의 전체 bag 구간을 사용한다.

검사를 통과한 전압은 `esc_voltage.csv`와 `.csv.json`으로 저장한다. `voltage_replay_ready`와 `identification_input_ready`를 별도로 보고하므로, 전압만 유효하더라도 PWM 기록이 부족한 상태를 구분할 수 있다. 어느 값도 물리 계수 식별 완료를 의미하지 않는다.

시뮬레이터의 기존 실행 명령에 다음 옵션을 추가한다. 오프셋은 그 재생의 `preparation.json` 값으로 지정한다.

```text
--thruster-voltage-trace /absolute/path/esc_voltage.csv
--thruster_voltage_time_offset_s <voltage_time_offset_s>
```

보정 도구 `fit_bag_horizontal_response.py`에는 시뮬 오프셋 대신 **bag 구간 시작 시각**을 준다.

```text
--thruster_voltage_trace /absolute/path/esc_voltage.csv
--voltage_trace_bag_start_s <bag_interval_s의 첫 값>
```

준비 도구의 기본 `outside_trace=hold`는 선택한 구간 전후에 끝값을 유지한다는 명시적인 가정이다. 이를 허용할 수 없으면 `--outside_trace error`를 사용하고, 시뮬 시작부터 평가 종료까지 전압 기록을 제공한다. 부분 구간의 전압을 전체 주행에 측정값처럼 적용하면 안 된다.

## 다음 실험의 고율 PWM 기록

실물/외부 MAVROS 경로에서 기존 `rov_start.launch.py` 실행에 다음 인자를 추가한다. 기존 rate-config 실행 옵션이 켜져 있어야 한다.

```text
configure_mavros_imu_rate:=true mavros_rcout_rate_hz:=100.0
```

이는 MAVROS `MessageInterval` 서비스를 통해 `SERVO_OUTPUT_RAW`(36번 메시지)의 주기를 요청한다. 서비스 성공은 실제 100Hz 수신을 보증하지 않으므로 bag에서 다시 확인한다. 선택하지 않으면 기존 동작을 유지한다.

MuJoCo 내부 MAVROS 호환 발행 경로를 사용한다면, 실행 전에 `ROS2_UUV_RCOU_TELEMETRY_HZ=100` 및 `ROS2_UUV_RCOUT_PUBLISH_MODE=event`를 지정한다. 이벤트 발행은 실제 최종 PWM 패킷을 받았을 때만 발행하며 낮은 주기의 값을 복제하지 않는다. 외부 MAVROS가 토픽을 소유하는 경로에서는 외부 MAVROS 설정을 사용한다.

기록할 핵심 토픽은 `/mavros/rc/out`, `/mavros/rc/override`, `/mavros/state`, `/battery`, 실제 ESC 전압 토픽(있을 때), `/dvl/data`, `/mavros/imu/data`, `/mavros/imu/data_raw`, `/mavros/imu/static_pressure`, `/depth/pose`, `/tf_static`, `/clock`(시뮬일 때)이다. 수집용 VLA 프레임 주기와 별개로 원본 bag에 최종 PWM을 고율로 보존한다. 이번 작업에서 하드웨어에 주기 변경 명령을 보내거나 새로운 실물 측정을 시작하지 않았다.

남은 실물 확인: ESC 전원 연결/단자 전압, ESC 모델과 PWM 설정, 실제 사용 PWM에서의 장착 순추력. 이 정보가 확인되면 전압 시계열을 연결한 뒤 추력과 항력을 다시 분리 검증할 수 있다.
