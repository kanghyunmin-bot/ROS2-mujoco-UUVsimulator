# UUV v2.2 구현 매뉴얼 (완전 상세판)

최종 갱신: 2026-03-04  
대상 코드:
- `mujoco/uuv_mujoco/v2.2/competition_scene.xml`
- `mujoco/uuv_mujoco/v2.2/run_urdf_full.py`
- `mujoco/uuv_mujoco/v2.2/ros2_bridge.py`
- `scripts/start_full_stack_direct.sh`
- `scripts/launch_stack_auto.sh`
- `kmu_hit25_ros2_ws/scripts/run_ardusub_json_sitl.sh`

이 문서는 아래 4개를 중심으로 작성했다.

1. 센서
2. base_link
3. 쓰러스터
4. ArduSub/QGC 통신

요구사항:
- 코드 원문(스니펫) 포함
- 데이터가 “어디서 시작해서 어디로 가는지” 경로를 단계별 설명
- 이 문서만으로 같은 구조를 다시 구현 가능한 수준으로 정리

---

## 0) 시스템 전체 그림

제어 경로:
- QGC 조작
- ArduSub 내부 믹서/모드 제어
- `SERVO_OUTPUT_RAW` 송신 (포트 14660)
- MuJoCo 브리지 수신
- 채널→쓰러스터 매핑
- `data.ctrl` 반영
- 물리 스텝

센서 경로:
- MuJoCo 센서 추출(IMU/DVL/Depth/자세)
- 좌표계 변환(FRD/NED, FLU 분리)
- ArduSub로 JSON 전송(포트 9003)
- ArduSub EKF/제어 루프 반영
- 동시에 ROS2 토픽 publish

핵심 포인트:
- 제어 입력은 MAVLink(`SERVO_OUTPUT_RAW`) 경로가 주 경로
- 센서 피드백은 JSON 경로가 주 경로
- ROS2 publish는 모니터링/연동용

---

## 1) base_link (로봇 기준축의 루트)

## 1.1 코드 원문

파일: `mujoco/uuv_mujoco/v2.2/competition_scene.xml`

```xml
<body name="base_link" pos="0 0 -0.6" quat="0.7071068 -0.7071068 0 0">
  <inertial pos="0.1807 0.1472 -0.1826" quat="1 0 0 0" mass="10" diaginertia="0.365 0.722 0.501" />
  <joint name="world_joint" type="free" />
  ...
</body>
```

의미:
- `base_link`가 동역학/센서/쓰러스터의 공통 부모 바디.
- `free` 조인트로 6DOF 완전 자유 운동.
- `inertial` 중심/관성은 물리 반응(가속, 회전, 감쇠)에 직접 반영.

## 1.2 base_link에서 파생되는 핵심 site

```xml
<site name="cob_site" pos="0.1807 0.182 -0.1826" ... />
<site name="imu_site" pos="0.1807 0.1472 -0.1826" quat="1 0 0 0" ... />
<site name="bar30_site" pos="0.1807 0.1472 -0.1826" quat="1 0 0 0" ... />
<site name="dvl_site" pos="0.1807 0.2500 -0.1826" quat="1 0 0 0" ... />
```

`cob_site` 역할:
- 부력 토크 계산 기준점.
- `apply_underwater_wrench()`에서 CoM 대비 레버암으로 토크 생성.

## 1.3 base_link가 물리에 적용되는 위치

파일: `mujoco/uuv_mujoco/v2.2/run_urdf_full.py`

```python
data.xfrc_applied[base_id, 0:3] += buoy_force_world
data.xfrc_applied[base_id, 3:6] += buoy_tau_world
data.xfrc_applied[base_id, 0:3] -= lin_drag_coeff * lin_vel
data.xfrc_applied[base_id, 3:6] -= ang_drag_coeff * ang_vel
```

요약:
- base_link는 부력/감쇠/보정토크가 들어가는 물리 입력 포인트.

---

## 2) 센서 (정의 → 추출 → 변환 → 송신)

## 2.1 센서 선언 (MuJoCo)

파일: `mujoco/uuv_mujoco/v2.2/competition_scene.xml`

```xml
<sensor>
  <framequat name="imu_quat" objtype="site" objname="imu_site" />
  <gyro name="imu_gyro" site="imu_site" noise="0.002" />
  <accelerometer name="imu_acc" site="imu_site" noise="0.03" />
  <velocimeter name="dvl_vel_body" site="dvl_site" noise="0.01" />
  <rangefinder name="dvl_altitude" site="dvl_site" cutoff="30" />
  <framepos name="base_pos" objtype="body" objname="base_link" />
  <framexaxis name="base_xaxis" objtype="body" objname="base_link" />
  <frameyaxis name="base_yaxis" objtype="body" objname="base_link" />
</sensor>
```

중요 변경점:
- `imu_quat`를 `base_link` 직접 참조가 아니라 `imu_site` 참조로 맞춰서, IMU 벡터와 자세 소스 일치.

## 2.2 런타임 센서 추출

파일: `mujoco/uuv_mujoco/v2.2/ros2_bridge.py`

```python
quat = self._sensor_slice("imu_quat", data)
gyro = self._sensor_slice("imu_gyro", data)
acc = self._sensor_slice("imu_acc", data)
dvl_vel_sensor = self._sensor_slice("dvl_vel_body", data)
gyro_bmj, acc_bmj = self._imu_vectors_in_body(data, gyro, acc)
dvl_vel_body = self._dvl_velocity_body(data, dvl_vel_sensor, gyro_bmj)
bar30_depth_m, bar30_pressure_pa = self._bar30_depth_pressure_from_model(data)
```

## 2.3 좌표계 변환 (핵심)

파일: `mujoco/uuv_mujoco/v2.2/ros2_bridge.py`

```python
self._enu_to_ned = np.diag([1.0, -1.0, -1.0])
self._bmj_to_frd = np.array(
    [
        [1.0, 0.0, 0.0],
        [0.0, 0.0, -1.0],
        [0.0, 1.0, 0.0],
    ],
    dtype=np.float64,
)
self._bmj_to_flu = np.array(
    [
        [1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0],
        [0.0, -1.0, 0.0],
    ],
    dtype=np.float64,
)
```

역할:
- SITL 전송은 FRD/NED 기준.
- ROS publish는 FLU 기준.
- 같은 raw 값을 목적지별로 분리 변환.

## 2.4 DVL 보정 (회전항 보정 포함)

파일: `mujoco/uuv_mujoco/v2.2/ros2_bridge.py`

```python
rot_bmj_dvl = base_rot_enu.T @ dvl_rot_enu
vel_body = rot_bmj_dvl @ vel_body
if gyro_bmj is not None:
    r_enu = data.site_xpos[self._dvl_site_id] - data.xpos[self._base_id]
    r_body = base_rot_enu.T @ r_enu
    vel_body = vel_body - np.cross(gyro_bmj, r_body)
```

설명:
- DVL 위치 오프셋 때문에 회전시 가짜 선속도가 생기므로 `ω×r` 제거.

## 2.5 BAR30 depth/pressure 모델

파일: `mujoco/uuv_mujoco/v2.2/ros2_bridge.py`

```python
raw_depth_m = float(max(0.0, -z_world))
pressure_pa = surface_pressure_pa + rho * gravity * raw_depth_m
depth_m = max(0.0, (pressure_abs_pa - surface_pressure_pa) / (rho * gravity))
```

특징:
- 수면 위는 depth=0으로 clamp.
- pressure는 LPF 적용.

## 2.6 SITL JSON 전송 payload

파일: `mujoco/uuv_mujoco/v2.2/ros2_bridge.py`

```python
payload = {
    "timestamp": float(sitl_t),
    "imu": {
        "gyro": [float(x) for x in gyro],
        "accel_body": [float(x) for x in acc]
    },
    "position": [float(x) for x in pos],
    "velocity": [float(x) for x in vel],
    "attitude": attitude,
    "quaternion": [float(x) for x in quat],
    "rng_1": float(depth_m),
    "no_time_sync": True,
}
```

중요:
- 초기에 quaternion/position을 identity/0으로 강제 덮어쓰지 않고 실제 상태 유지.
- gyro/acc/vel은 clamp로만 안정화.

## 2.7 ROS publish 세부

파일: `mujoco/uuv_mujoco/v2.2/ros2_bridge.py`

```python
imu.header.frame_id = "imu_link"
imu.orientation = quat_ros
imu.angular_velocity = gyro_ros
imu.linear_acceleration = acc_ros
```

```python
tw.header.frame_id = "base_link"
tw.twist.linear = dvl_vel_body_ros
```

```python
odom.header.frame_id = "odom"
odom.child_frame_id = "base_link"
odom.pose.pose.orientation = quat_ros
odom.twist.twist.linear = dvl_vel_body_ros
```

---

## 3) 쓰러스터 (채널 매핑, 스케일, 힘 적용)

## 3.1 쓰러스터 위치 site

파일: `mujoco/uuv_mujoco/v2.2/competition_scene.xml`

```xml
<site name="thr_ver_lf" ... />
<site name="thr_ver_lr" ... />
<site name="thr_ver_rf" ... />
<site name="thr_ver_rr" ... />
<site name="thr_yaw_lf" ... />
<site name="thr_yaw_lr" ... />
<site name="thr_yaw_rf" ... />
<site name="thr_yaw_rr" ... />
```

## 3.2 motor/gear 정의

```xml
<motor name="ver_lf" site="thr_ver_lf" gear="0 1 0 0 0 0" ctrlrange="-80 80" />
...
<motor name="yaw_lf" site="thr_yaw_lf" gear="0.735142 0 -0.677913 0 0 0" ctrlrange="-80 80" />
<motor name="yaw_lr" site="thr_yaw_lr" gear="-0.707105 0 -0.707108 0 0 0" ctrlrange="-80 80" />
<motor name="yaw_rf" site="thr_yaw_rf" gear="0.707105 0 0.707108 0 0 0" ctrlrange="-80 80" />
<motor name="yaw_rr" site="thr_yaw_rr" gear="-0.705343 0 0.708867 0 0 0" ctrlrange="-80 80" />
```

`gear` 의미:
- body wrench 축으로 입력되는 힘 방향 벡터.
- 수평 쓰러스터는 x/z 혼합으로 전진/회전 결합이 생김.

## 3.3 SITL 채널→쓰러스터 기본맵

파일: `mujoco/uuv_mujoco/v2.2/run_urdf_full.py`

```python
parser.add_argument("--sitl-servo-map", default="yaw_rr,yaw_lr,yaw_rf,yaw_lf,ver_lf,ver_rf,ver_lr,ver_rr")
parser.add_argument("--sitl-servo-signs", default="1,1,1,1,1,1,1,1")
parser.add_argument("--sitl-servo-scale", default=0.75)
```

## 3.4 direct-thruster 수신 처리

```python
norm = sitl_pwm_to_norm(int(pwm_values[idx])) * servo_signs[idx]
sitl_servo_cmd_norm[thr_name] = float(np.clip(norm, -1.0, 1.0))
```

## 3.5 thr_target → data.ctrl 적용

파일: `mujoco/uuv_mujoco/v2.2/run_urdf_full.py`

```python
for name in all_thruster_names:
    aid = act[name]
    lo, hi = ctrlrange[aid]
    gain = float(thruster_scale.get(name, 1.0))
    target_norm = float(np.clip(thr_target[name], -1.0, 1.0))
    if perf_cfg.get("active") and perf_cfg.get("force").size > 0:
        force = pwm_to_force_from_perf(target_norm) * gain
    else:
        force = target_norm * thruster_force_max * gain
    data.ctrl[aid] = float(np.clip(force, lo, hi))
```

## 3.6 루프 분리 (thruster-loop-hz)

```python
thruster_due = thruster_update_due()
if thruster_due:
    update_thruster_forces(thr_dt)
update_propeller_visuals(thr_dt)
apply_underwater_wrench(...)
mujoco.mj_step(model, data)
```

설계 의도:
- 물리 timestep과 쓰러스터 업데이트 주기를 분리해 반응 과도/진동 제어.

## 3.7 stale fail-safe

```python
stale = (
    sitl_servo_last_wall["value"] <= 0.0
    or (now - sitl_servo_last_wall["value"]) > sitl_servo_timeout_s
)
if stale:
    for name in all_thruster_names:
        thr_target[name] = 0.0
```

---

## 4) ArduSub/QGC 통신 (프로세스/포트/메시지)

## 4.1 표준 전체 실행 스크립트

파일: `scripts/start_full_stack_direct.sh`

SITL 실행:

```bash
python3 Tools/autotest/sim_vehicle.py -L RATBeach --console --map \
  -v ArduSub -f vectored_6dof --model JSON \
  --out=udp:127.0.0.1:14550 \
  --out=udp:127.0.0.1:14551 \
  --out=udp:127.0.0.1:14660 \
  --out=tcpin:0.0.0.0:5773 \
  -P MAV_GCS_SYSID=1 -P MAV_GCS_SYSID_HI=255
```

MuJoCo 실행:

```bash
./launch_competition_sim.sh --sitl --images \
  --sitl-servo-source mavlink \
  --sitl-mavlink-endpoint udpin:0.0.0.0:14660 \
  --sitl-servo-map "yaw_rr,yaw_lr,yaw_rf,yaw_lf,ver_lf,ver_rf,ver_lr,ver_rr" \
  --sitl-servo-signs=1,1,1,1,1,1,1,1 \
  --force-clean
```

## 4.2 포트 정의

- `14550/udp`: QGC 링크
- `14551/udp`: MAVROS/보조 링크
- `14660/udp`: ArduSub `SERVO_OUTPUT_RAW` 송신
- `9002/udp`: MuJoCo JSON servo endpoint listen
- `9003/udp`: ArduSub JSON sensor endpoint recv
- `5773/tcp`: QGC TCP 링크(선택)
- `5600/udp`: 영상 스트림

## 4.3 QGC 입력이 실제 쓰러스터까지 가는 호출 순서

1. QGC 조이스틱/모드 입력  
2. ArduSub mixer가 PWM 출력  
3. `SERVO_OUTPUT_RAW` (14660) 송신  
4. `ros2_bridge.py::_poll_sitl_servo_mavlink()` 수신  
5. `_handle_sitl_pwm_values()` 호출  
6. `run_urdf_full.py`에 등록된 `on_sitl_servo_packet()`에서 `sitl_servo_cmd_norm` 갱신  
7. `run_step()`에서 `thr_target` 반영  
8. `update_thruster_forces()`에서 `data.ctrl` 반영  
9. `mujoco.mj_step()` 물리 반영

## 4.4 MuJoCo 센서가 ArduSub EKF까지 가는 호출 순서

1. `run_step()`에서 `publish_ros_once()`  
2. `ros_bridge.publish(data)`  
3. `quat/gyro/acc/dvl/depth` 추출  
4. FRD/NED 변환  
5. `_send_sitl_data()`에서 JSON payload 생성  
6. UDP 9003 송신  
7. ArduSub `SIM_JSON.cpp`에서 parse  
8. `accel_body/gyro/velocity/position/quaternion/rng_1` 반영  
9. EKF/제어기에 사용

## 4.5 ArduPilot 파싱 측 원문 확인

파일: `kmu_hit25_ros2_ws/ardupilot/libraries/SITL/SIM_JSON.cpp`

```cpp
accel_body = state.imu.accel_body;
gyro = state.imu.gyro;
velocity_ef = state.velocity;
...
if ((received_bitmask & QUAT_ATT) != 0) {
    state.quaternion.rotation_matrix(dcm);
}
```

즉:
- MuJoCo가 보내는 quaternion/acc/gyro가 바로 내부 상태로 들어간다.

---

## 5) 런타임 주기와 병목 포인트

1. `--ros2-sensor-hz`
- 센서 publish 주기(ROS2 + SITL JSON 관련 루프 주기에 영향)

2. `--thruster-loop-hz`
- 쓰러스터 업데이트 주기
- 높일수록 반응 빠르지만 진동/노이즈 증폭 가능

3. `--sitl-mavlink-servo-hz`
- SERVO_OUTPUT_RAW 요청 주기
- 과도하게 높이면 ArduSub 스케줄 경고(`SCHED_LOOP_RATE`) 발생 가능

4. 이미지 주기
- `--ros2-image-hz`, 영상 브리지 인코딩 파라미터가 CPU 병목 유발

---

## 6) “증상 → 수정 파일” 즉시 매핑

1. QGC에서 조종 방향 뒤집힘
- 우선 확인: `scripts/start_full_stack_direct.sh`
  - `--sitl-servo-map`
  - `--sitl-servo-signs`
- 런타임 기본값: `mujoco/uuv_mujoco/v2.2/run_urdf_full.py`

2. ROS에서 자세가 90도 틀어져 보임
- `mujoco/uuv_mujoco/v2.2/ros2_bridge.py`
  - `_bmj_to_flu` 변환 경로
  - IMU/Odom publish에 `quat_ros` 사용 여부

3. Depth hold 불안정
- `mujoco/uuv_mujoco/v2.2/ros2_bridge.py`
  - `_bar30_depth_pressure_from_model()`
  - pos_ned[2]/vel_ned[2] 갱신부
- ArduSub 파라미터와 함께 봐야 함

4. 스테일 상태에서 오동작
- `run_urdf_full.py`의 stale fail-safe
- `ros2_bridge.py`의 source 고정(mavlink only) 로직

---

## 7) 구현할 때 반드시 지킬 불변 조건

1. 목적지 기준 변환 분리
- ArduSub: FRD/NED
- ROS: FLU

2. 쓰러스터 맵/사인은 런처와 런타임 기본값을 반드시 일치

3. stale 시 `thr_target=0` fail-safe 유지

4. startup에서 실제 quaternion을 0/identity로 덮어쓰지 않기

---

## 8) 복붙용 검증 명령 (실무용)

포트:

```bash
ss -lntup | grep -E '14550|14551|14660|9002|9003|5773|5600'
```

heartbeat:

```bash
python3 scripts/check_sitl_manual_input.py --mode udp --listen 14551 --heartbeat-only --timeout 20
```

조종 유입(strict):

```bash
python3 scripts/check_sitl_manual_input.py --mode udp --listen 14550 --strict --timeout 20
```

MuJoCo 로그 확인 포인트:
- `Waiting for SITL MAVLink SERVO_OUTPUT_RAW`
- `SITL send ok`
- `SITL tx sample`
- `servo stream is neutral`

---

## 9) 최종 결론

이 스택은 단순히 “한 방향 통신”이 아니라,
- 제어는 MAVLink SERVO 경로,
- 센서는 JSON 경로,
- 시각화/관측은 ROS/QGC 경로
를 동시에 유지하는 구조다.

문제 디버깅은 항상 아래 순서로 한다.

1. 포트/프로세스 생존
2. SERVO_OUTPUT_RAW 유입
3. `thr_target -> data.ctrl` 반영
4. 센서 JSON 송신
5. ArduSub EKF/모드 반응

이 순서를 지키면 센서/자세/제어 뒤집힘 문제를 재현 가능하게 분해해서 해결할 수 있다.

