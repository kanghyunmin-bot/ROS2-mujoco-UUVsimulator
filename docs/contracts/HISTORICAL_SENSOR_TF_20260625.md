# 6월 Git 센서 TF 확인과 위치 영향 — 2026-09-14

사용자 설명에 따라 변경 전 TF를 Git에서 추적했다. **6월 25일 ROS 설정의 IMU·DVL·압력계 좌표는 4월 2일 bag의 TF와 일치한다.** 과거 장착 위치를 완전히 미상으로 둘 필요는 없으며, 이 설정을 과거 기체의 참고 좌표로 사용할 근거가 있다. 현재 CAD 좌표도 추정값이므로 두 설정의 차이가 그대로 실물 센서 이동량이라는 뜻은 아니다.

과거 센서 위치를 사용해 저장된 시뮬레이션의 관측값을 다시 계산해도 **baseline의 큰 전진 오차는 거의 변하지 않았다. 센서의 병진 위치 차이는 이 오차의 주원인이 아니다.**

## 확인한 Git 근거

- 시뮬레이터 [6월 25일 커밋 `9251007`](https://github.com/kanghyunmin-bot/ROS2-mujoco-UUVsimulator/commit/9251007ec5c439c777fd61a65a08a5c89c2edb39)의 [scene](https://github.com/kanghyunmin-bot/ROS2-mujoco-UUVsimulator/blob/9251007ec5c439c777fd61a65a08a5c89c2edb39/uuv_mujoco/v2.2/scenes/tank_current_scene.xml)과 [센서 TF 구성](https://github.com/kanghyunmin-bot/ROS2-mujoco-UUVsimulator/blob/9251007ec5c439c777fd61a65a08a5c89c2edb39/uuv_mujoco/v2.2/bridge/ros2_static_tf_sensors.py)을 확인했다.
- 당시 `rospkg/kmu26_auv`는 submodule이었다. 해당 Git tree가 가리키는 커밋은 **`9f50aab20aa43fae09c616ef36f8c35b0d79234c`**, 실제 작성일은 **5월 4일**이다. 6월 tree에 포함된 설정과 설정 자체의 작성일을 구분한다.
- 그 커밋의 [실물 `launch/rov_start.launch.py`](https://github.com/kanghyunmin-bot/kmu26_auv/blob/9f50aab20aa43fae09c616ef36f8c35b0d79234c/launch/rov_start.launch.py)를 읽었다. 과거 코드를 실행하지 않고 AST로 launch 기본값만 추출했다.
- 6월 scene의 세 센서 좌표는 이 ROS launch를 약0.04–0.06 mm 이내로 반올림한 값이다. 6월17일의 더 오래된 scene에는 placeholder가 있었으므로 ‘6월 버전’을 전부 같은 설정으로 취급하지 않는다.

## 좌표 비교

단위는 m, `base_link` 기준 forward X / left Y / up Z다. IMU는 `base_link → fcu_link → imu_link`를 확인했고, 두 번째 변환은 identity다.

| 센서 | 6월 tree의 실물 ROS 설정 = bag TF | 현재 CAD 기반 설정 | 좌표 간 거리 |
|---|---|---|---:|
| IMU / FCU | (0.11000, −0.00034, 0.09200) | (0.13135, 0, 0.08541) | **2.23 cm** |
| DVL | (−0.03196, 0, −0.09700) | (−0.00488, 0, −0.15861) | **6.73 cm** |
| pressure | (−0.17364, −0.03034, 0.05360) | (−0.17364, −0.03034, −0.03286) | **8.65 cm** |

현재 DVL은 CAD acoustic head 아래의 관측 원점을 선택했고 압력계는 후방 인클로저 내부로 기준점을 옮긴 추정이다. 따라서 위 숫자에는 실제 장착 차이뿐 아니라 **기준점 선택·CAD 해석 차이**도 들어 있다. 이전 TF를 실측 정답으로 확정하지 않으면서도 비교용 참고값으로는 사용할 수 있다.

회전은 별도다. 과거 ROS DVL TF는 identity지만 6월 MuJoCo DVL site는 roll π였다. 당시 bridge가 site 회전을 그대로 ROS TF에 내보내지도 않았다. 이를 곧바로 ‘실물 DVL이 뒤집혀 있었다’고 해석하거나 원시 DVL y/z를 다시 뒤집지 않는다. x 전후축은 roll π에서 그대로이고, IMU 회전은 과거 launch에서 identity다.

## 위치 변경 자체의 영향

실제 bag의 AHRS gyro/자세 1,000개 표본(20–70초)을 사용했다. 병진 차이 `Δr = r_current − r_historical`에 대해 DVL 속도 차이는 `Δv = ω × Δr`이다. 과거 IMU의 장착 회전이 identity라는 설정을 조건으로 한다.

| DVL 속도 변화 [m/s] | RMS | 절댓값 p95 | 절댓값 최대 |
|---|---:|---:|---:|
| 전후 x | **0.00189** | 0.00438 | 0.00776 |
| 좌우 y | 0.00873 | 0.02437 | 0.03379 |
| 상하 z | 0.00083 | 0.00193 | 0.00341 |

이는 장착 위치 차이의 **운동학적 영향**이다. 특히 좌우값은 별도 센서 검토에서 무시할 필요가 없지만, 전진 baseline RMSE 약0.489 m/s를 설명하는 크기는 아니다. gyro 자체는 센서의 병진 위치 변경으로 값이 달라지지 않는다.

압력계 z 차이는 수평 정지 시 약8.646 cm의 **절대 수심 offset**을 만든다. 그러나 이번 비교는 10–17초의 초기 수심을 빼는 상대 수심이다. 측정된 자세에서 위치 변경으로 남는 상대 수심 차이는 RMS **0.066 mm**, p95 **0.099 mm**다. 이 계산은 정적인 높이 차이가 상대 수심에서 사라진다는 의미이며, 제어기의 절대 수심 기준을 무시해도 된다는 뜻은 아니다.

## 로봇 중심 대신 과거 센서 위치에서 계산한 결과

기존 보고서는 시뮬 COM 전진 속도 및 body 원점 수심을 사용했다. 이번에는 저장된 동일한 MuJoCo 운동에 대해 과거 DVL 좌표에서 `v_COM + ω × (r_DVL − r_COM)`, 과거 압력계 좌표에서 `depth_body − (R r_pressure)_z`를 계산했다. 수심은 각각 초기 offset을 제거했다. 센서 위치나 시각을 재최적화하지 않고 이전에 결정한 위상을 고정했다.

55.5–70초에서 양쪽0.5초를 제외하고 동일한 유효 표본으로 계산한 **원시 RMSE**다. 전진232개, 수심270개를 비교했다.

| 조건·신호 | 기존 body 기준 | 과거 센서 위치 기준 |
|---|---:|---:|
| baseline 전진 [m/s] | 0.48898 | **0.48926** |
| effective 전진 [m/s] | 0.06390 | **0.06386** |
| baseline 상대 수심 [m] | 0.02625 | 0.02597 |
| effective 상대 수심 [m] | 0.03336 | 0.03325 |

`effective` 상대 수심의 trend 상관계수도 −0.2051 → −0.2072로 개선되지 않았다. **센서 위치를 맞춘 뒤에도 전진 baseline의 큰 차이와 수심 경향 불일치가 남는다.** 따라서 수평 direct gain 0.123이 필요한 이유를 센서 위치로 설명하지 않는다. PWM→추력 변환과 제어 입력·출력 대응, 질량/부가질량·항력, 테더 외력의 실제 기여를 분리하는 쪽이 다음 물리 점검 대상이다.

이 결과는 저장된 운동으로 관측 지점을 바꾼 오프라인 민감도 분석이다. TF를 실물 bag이나 GUI에 덮어쓰거나 FCU에 넣고 새 폐루프 재생을 돌린 결과가 아니다. 센서 좌표가 추정기에 미치는 모든 간접 효과까지 배제한 것은 아니다.

## 산출물과 재현

- [수치·입력 해시 JSON](../assets/bag-trends-20260914/historical-mount-audit.json)
- [분석 코드](../report_sources/20260914_mount_history.py)
- [기존 경향 비교 방법](ROSBAG_TREND_ALIGNMENT.md)

저장소 루트에서 과거 source를 확보한 뒤 실행한다. Git source의 SHA256을 검사하고, 기존 결과 덮어쓰기는 거절한다. 시뮬 런타임이나 필수 의존성은 바뀌지 않는다.

```bash
mkdir -p outputs/mount-history-reproduction
git show 9251007:uuv_mujoco/v2.2/scenes/tank_current_scene.xml \
  > outputs/mount-history-reproduction/scene_june25.xml
gh api -H 'Accept: application/vnd.github.raw+json' \
  'repos/kanghyunmin-bot/kmu26_auv/contents/launch/rov_start.launch.py?ref=9f50aab20aa43fae09c616ef36f8c35b0d79234c' \
  > outputs/mount-history-reproduction/rov_start_may04.launch.py

/home/khm/robotics/IsaacLab/isaaclab.sh -p docs/report_sources/20260914_mount_history.py \
  --historic_launch outputs/mount-history-reproduction/rov_start_may04.launch.py \
  --historic_scene outputs/mount-history-reproduction/scene_june25.xml \
  --numeric_npz outputs/bag-trends-20260914/source/numeric.npz \
  --topic_profile outputs/bag-trends-20260914/source/profile.json \
  --bag_audit outputs/real-bag-audit-20260911/alignment.json \
  --current_mounts uuv_mujoco/current/config/sensor_mounts_2026.json \
  --comparison_dir outputs/bag-trends-20260914/delivered \
  --output outputs/mount-history-reproduction/audit.json
```
