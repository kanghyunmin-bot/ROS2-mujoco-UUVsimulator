# ROV VLA 수집·실물 전이 점검 — 2026-09-10

## 판정과 범위

**수집→내보내기→실제 U0 학습 로더/전처리/모델 입력 패킹은 검증했다.
현재 보유한 ROV 데이터로 유의미한 작업 정책을 학습했다거나 실물로 전이할 수 있다는 판정은 아니다.**
현재 GUI의 640×360 / 4Hz 설정은 작업 수집에 부적합했다. 별도 컨테이너의
320×240 / 30Hz 설정에서는 새 코드로 5초, 50개의 정상적인 연결 확인 샘플을 수집했다.
이 자료는 비무장·중립 RC·`success=false`, `collection_kind=connection_check`이며
새 학습 진입점은 이를 학습 데이터로 거부한다. 부표 접근/분리 시연은 아직 수행하지 않았다.

작업 저장소: `/home/khm/robotics/underwater/ROS2-mujoco-UUVsimulator`.
브랜치: `codex/vla-transfer-audit`. 커밋/푸시는 하지 않았다.
기존 미커밋 변경을 초기 `preexisting.patch`, `preexisting-status.txt` 및 두 ROS 패키지
사본으로 보존했다. 기존 GUI 프로세스·조종 상태는 변경하지 않았다. 새 설정을 적용하려면
기존 실행을 운영자가 종료한 뒤 재시작해야 한다.

로컬 근거: `outputs/vla-transfer-audit-20260910/` (생성물, Git 비추적).
이번 작업만의 파일별 SHA와 패치는 `changed-files.json`, `task-only.patch`에 있다.
그중 `smoke-final/staging/episode_000001`이 소스 스냅샷까지 포함한 마지막 50행 수집이다.
`smoke-final/lerobot`은 그 episode 하나를 `staging-selected`에서 내보낸 결과이며 `smoke30hz/`는 중간 연결 시험이다.
초기 물리 튜닝·장착 위치·줄·자석·유체 최적화 변경은 이번 작업의 변경으로 간주하지 않는다.

## 원본 및 실물 데이터 근거

조직 원본을 새로 clone하여 HEAD를 확인했다. 아래는 2026-09-10 조회 결과이며
조직 저장소 자체에는 쓰지 않았다.

| 원본 | 커밋 | 로컬 대조 결과 |
|---|---|---|
| [수집기](https://github.com/2026-kmu-underwater-robot/auv_vla_data_collector/tree/4b4e2328f7b36a51c30b4929b8bf015f9396e4c3) | `4b4e2328f7b36a51c30b4929b8bf015f9396e4c3` | 작업 시작 시 수집 규격/설정은 이 버전. 이번 수정은 로컬 추가 변경 |
| [VLA](https://github.com/2026-kmu-underwater-robot/auv_vla/tree/cc4ada199d84290fcc55df6cfad003f1583fc3bf) | `cc4ada199d84290fcc55df6cfad003f1583fc3bf` | `Kmu26AuvRealDataConfig`, LeRobotSingleDataset, GR00TTransform과 배포 어댑터를 추적 |
| [기본 로봇](https://github.com/2026-kmu-underwater-robot/auv/tree/756d1a412c9a4804ae9384b6ded18245969bddec) | `756d1a412c9a4804ae9384b6ded18245969bddec` | 원본 TF의 DVL z=-0.03910, depth z=+0.0536은 현재 CAD 추정과 다름. 실측값으로 취급하지 않음 |

조직 기본 `rov_start.launch.py` 자체는 현재 로컬 IMX219 확장 런치와 동일하지 않으며,
두 카메라의 실물 작동/설치는 별도로 확인해야 한다. 로컬 `auv`는 `hit25_auv_ros2`의
호환 진입점이다. 토픽 이름을 맞췄다는 사실만으로 실물 센서 장착/의미가 검증되지 않는다.

`/home/khm/robotics`, 사용자 `vla_data`, Downloads 아래 bag/mcap/db3, NPZ, Parquet,
메타데이터/체크포인트 이름을 탐색했다. 확인한 ROV episode는 기존 시뮬 연결 시험과
이번 연결 시험이다. `/home/khm/robotics/u0/datasets`는 비어 있고, U0의
`demo_data/robot_sim.PickNPlace`는 다른 embodiment의 예제이다. **실물 ROV 통계 비교용
샘플은 발견하지 못했다. 접근 가능한 로컬/공개 코드 범위의 결과이며 비공개 저장소 전체의 부재를 뜻하지 않는다.**
실물 샘플의 경로를 사용자에게 요청했다. 통계적 sim/real 일치, 실제 지연 분포,
축 부호, 영상 분포 및 실제 성공률은 미검증이다.

`/home/khm/robotics/u0/models/u0_final`의 가중치는 존재하나 배포 계약 검사에서
`state.prev_command [4]`가 없어 거부됐다 (`checkpoint.json`). 우리 ROV용 학습 가중치로
대체해 사용하지 않았다.

## 문제 분류와 수정

| 분류/우선순위 | 근거 파일 | 영향 및 조치 |
|---|---|---|
| 확인된 결함 P0 | 정책 `kmu26_ros.py::_tick` | 10Hz sim-time 학습에 wall-time chunk 인덱스를 사용. ROS 경과시간으로 chunk/명령 cadence와 slew dt를 변경. HTTP·deadman·중단 watchdog은 monotonic wall time 유지 |
| 확인된 결함 P0 | 수집기 `collector.py` | 무장/모드·출처를 모르는 정지 자료도 시연과 구분 불가. connection_check/task_demonstration 분리, 시연에 무장·연결·기대 모드·단일 RC 발행자·세션/설정 요구. 상태 변경 시 종료; 상태/RC in/out를 별도 기록 |
| 확인된 결함 P0 | `contract.py::RcCommandTracker` | span=300 밖의 1900 PWM도 1800과 같은 +1로 잘려 정책 역변환과 불일치. 선언 span 밖 명령을 무효화. RELEASE/NOCHANGE 동작은 유지 |
| 확인된 결함 P0 | 수집기 타이머/캐시 | sim pause 시 ROS 타이머가 멎어 episode를 계속 유지. wall watchdog 추가: 1초 clock 정지, 역행 시 종료·센서/RC 캐시 무효화. 기존 샘플 gap 검사도 유지 |
| 확인된 결함 P0 | 수집기 영상 validity | 오래된 영상/동일 프레임을 시연에 저장할 수 있었음. task_demonstration은 stale/중복 capture stamp에서 종료. 연결 검사는 validity를 남겨 진단 가능 |
| 확인된 결함 P1 | 실물 A50 드라이버 `lifecycle_dvl_component.cpp`, 수집기 | bottom lock 상실 때 driver가 이전 altitude를 유지하므로 새 header만으로 고도 valid를 판단하면 잘못됨. velocity_valid=false에서 고도 invalid/0. raw packet과 twist stamp가 다른 경우 오래된 twist를 다시 valid로 만들지 않음 |
| 확인된 결함 P1 | GUI `ros_package_stack.py` | Bar30 이동 후 -0.0536m 보정이 남아 수평 자세에서 0.08646m 오류. 현재 site z=-0.03286에 따라 +0.03286으로 수정. 자세 의존 레버암 보정은 별도 필요 |
| 확인된 결함 P1 | `bridge/ros2_stereo_image.py` | hand fovy=82°, 기본 profile=70°; 4:3 축소시 fx도 실제 렌더와 불일치. 번들 미보정 기본값은 각 MuJoCo 카메라/FOV/출력 크기에서 K/P 생성. 명시한 보정 파일/custom profile은 보존 |
| 확인된 결함 P1 | exporter | NaN state, 잘못된 state/action 이름 순서, 영상 번호 구멍을 사전 거부. 서로 다른 PWM/상태 계약의 묵시적 혼합도 거부. 종료 이유/출처 유실 방지, 원본 manifest와 FCU 로그 및 수신/축 시각 보존. train 범위를 실제 episode 수로 수정 |
| 확인된 결함 P1 | U0 `StateActionSinCosTransform` | 원본 설정은 수심·가속도·validity 등 모든 상태를 sin/cos로 바꿔 46차원 및 주기적 alias 생성. 새 opt-in `Kmu26TransferDataConfig`는 23개 물리값 유지. 과거 checkpoint의 전처리는 변경하지 않음 |
| 확인된 결함 P1 | U0 dataset/model mask | 끝의 action을 반복 패딩하고 action_mask는 차원만 표시하므로 가짜 미래가 loss에 들어감. `Kmu26TrainingDataset`은 마지막 15개 시작점을 제외해 실제 16-step 구간만 제공 |
| 검증 필요한 위험 P0 | RC transport/ArduSub | ROS override 수신은 FCU 채택 증거가 아님. 수신 대상 sysid, RC timeout, MAVLink 직접 조종, mode/failsafe, deadzone/trim/reversal이 영향. RCIn/RCOut는 근거 자료이며 동시 FCU 채택/실추력으로 재명명하지 않음 |
| 검증 필요한 위험 P1 | 장착/물리/광학 JSON 및 MJCF | CAD/표시 기반 extrinsic, 15N 자석, 물성·질량·항력은 실측 아님. 랜덤화 범위를 임의 확정하지 않음 |
| 현재 제약 | 샘플/가중치/운영 상태 | 실물 실행 불가, 실물 bag 미확보, 작업 시연 및 우리 ROV 가중치 미확인. 기존 GUI 실행에는 새 코드가 아직 반영되지 않음 |

## 실물/시뮬 데이터 계약 비교표

배열 인덱스는 0부터 시작. 수집/Parquet state는 `(N,23) float32`, action은
`(N,4) float32`. 정책 HTTP observation은 각 state 조각에 시간축 1을 추가한다.
물리값에 별도의 실측 기반 z-score는 적용하지 않는다.

| state 인덱스 / 이름 (순서 고정) | ROS 입력 | 단위·의미·좌표 | 실물/시뮬 비교 및 새 전처리 |
|---|---|---|---|
| 0–3 `prev_surge,prev_sway,prev_heave,prev_yaw` | 이전 수집 시점 RC | `(PWM-1500)/300`, 무차원 [-1,1] | 첫 row는 시작 직전 명령. 추력/속도가 아님. 원값 유지 |
| 4–6 `dvl_vx,dvl_vy,dvl_vz` | `/dvl/twist` + `/dvl/data` | m/s, `dvl_link` FRD → FLU `(x,-y,-z)` | **DVL 위치의 속도**, base 원점 속도 아님. sensor lever arm의 ω×r 유지. stamp pair/validity 일치 필요 |
| 7–9 `gyro_x,gyro_y,gyro_z` | `/mavros/imu/data` | rad/s, base_link FLU | raw IMU 토픽과 혼동 금지. FCU AHRS 경로, 다른 frame/미제공 orientation 거부 |
| 10–12 `accel_x,accel_y,accel_z` | 같은 IMU | m/s², specific force (정지 때 중력 반응 포함) | 무중력 선형가속도로 간주하지 않음; 실물 정지/6면 시험 필요. 원값 유지 |
| 13–16 `quat_w,quat_x,quat_y,quat_z` | ROS orientation xyzw를 재배열 | 정규화한 wxyz, body→ROS world attitude | FCU의 yaw/방위 기준 차이는 별도 보정. q/-q의 이중 표현 및 자기장 환경 검증 필요 |
| 17 `depth_m` | `/depth/pose.pose.pose.position.z` 부호 반전 | m, positive down | 원본 기본 zero_at_start와 sim surface-relative는 동일 의미 아님. 기준수압/zero/offset 고정하여 기록 |
| 18 `altitude_m` | A50 altitude | m, sensor→바닥 거리 | base_link 지상고 아님. driver held altitude를 valid로 쓰지 않도록 수정 |
| 19 `ego_valid` | camera0 시각 | 0/1 float32 | 시연은 stale/중복에서 종료. 학습 영상 손실을 별도 loss mask로 변환한 것은 아님 |
| 20 `buoy_release_valid` | camera1 시각 | 0/1 float32 | 손 카메라 역할. 이름이 과거 stereo_right여도 구현은 갈퀴 뷰 |
| 21 `dvl_velocity_valid` | raw valid + bridge twist | 0/1 float32 | 미수신/품질 gate/시간 불일치이면 속도 0 및 flag 0 |
| 22 `altitude_valid` | raw/velocity valid | 0/1 float32 | 비정상/0 이하/lock loss면 고도 0 및 flag 0 |

| 구간 | key, dtype, shape 및 정렬 |
|---|---|
| 카메라→수집 | `/imx219/camera0/image_raw/compressed`→ego, camera1→buoy_release. JPEG decode는 OpenCV BGR, 기록 JPEG는 그 영상; 정책 입력은 BGR→RGB |
| 디스크→로더 | `observation.images.ego`, `observation.images.buoy_release` MP4. `decord` RGB uint8 `(1,H,W,3)`. 문자열 task→`task_index`→`annotation.human.action.task_description` |
| 새 영상 전처리 | 순서 `[ego,buoy_release]`, 원본 config의 crop scale .95→224×224→color jitter를 사용. 실제 손끝/부표가 crop 뒤에도 남는지 시연별 확인. 물속 왜곡/노출은 미보정 |
| 상태 modality | `prev_command[4],dvl_velocity[3],angular_velocity[3],linear_acceleration[3],attitude[4],depth[1],altitude[1],validity[4]` |
| 새 모델 입력 | state `(1,23)`→`(1,64)`, state_mask 실제 23개만 true. video `(1,2,224,224,3)`→Eagle 입력, action `(16,4)`→`(16,32)`, 실제 64개의 action 성분만 true |
| 행동 chunk | `action.motion`, 시점 `[t,t+.1,...,t+1.5]`. 각 시점의 최신 **요청 RC 표본**. 구간 내 명령 전체 이력/실추력 평균이 아님. 마지막 15개 시작점 제외 |
| 행동 정규화 | PWM 계약 정규화 후 학습의 train min/max로 [-1,1] 변환, 서버가 역정규화 후 어댑터로 반환. checkpoint 통계를 실물 소량 샘플로 덮어쓰지 않음 |
| 진단 | `rc_pwm int32(N,4)`, update mask float32(N,4), ROS/source/receipt float64, age float32; source/receipt 순서 ego, hand, IMU, depth, twist, raw DVL, RC. `rc_axis_timestamp(N,4)`는 NOCHANGE 직전의 실제 축 갱신시각 |
| FCU 기록 | `vehicle_state.jsonl`의 connected/armed/mode, wall receipt age, RC 발행자 수, `/mavros/rc/in` 및 `/mavros/rc/out` 원본 채널·source/receipt 시각. state23에는 넣지 않음 |

| action 순서 | ArduSub 1-based 채널 / array offset | 기본 예상 양의 방향 | 주의 |
|---|---|---|---|
| surge | CH5 / 4 | 전진 | RC trim/reversal·배선은 실물 단축 시험 필요 |
| sway | CH6 / 5 | 우현 | 상태 FLU의 +y(좌현)와 같은 부호라고 가정하면 안 됨 |
| heave | CH3 / 2 | 상승 | depth positive down과 반대; 모드에 따라 thrust/depth controller 의미가 달라짐 |
| yaw | CH4 / 3 | 우회전 | FLU gyro +z와 같다고 가정하면 안 됨 |

이 방향은 기본 ArduSub 관례에 따른 기대값이며 이번에 실물/동작 시연으로 확인하지 않았다.
명령의 정확한 확정 의미는 **선택 채널의 neutral 대비 PWM 부호**다.
1000–2000 물리 범위와 설정 span을 모두 만족해야 한다. RC RELEASE(0)는 즉시 ownership
무효화, NOCHANGE(65535)는 이전 축 값을 유지하되 freshness 갱신은 하지 않는다.
RC 축의 실기 adoption timeout과 collector의 0.5초 validity는 서로 다른 정책이다.

## 시간·센서·영상의 해석

- `/clock`은 MuJoCo 시간. ROS 수집 타이머/use_sim_time, image capture stamp, simulated
  pressure capture와 이를 전달한 depth header는 이 시간축을 사용한다. 수집 NPZ는 실제
  ROS 시각을 저장하고 MP4는 10fps 격자로 내보내므로 ±25%보다 큰 sampling gap은 거부한다.
- A50 ROS header는 **드라이버 파싱/수신 시각**이다. 센서 내부 capture 시간이 아니다.
  MAVROS fused IMU stamp도 FCU/transport 정책을 거친다. 같은 source_age 열이라도 물리적
  기준이 다르다. 실물에서 장치 clock을 추정하기 전 이 값으로 end-to-end latency를 확정하지 말 것.
- sensor latest 방식은 모든 센서가 동시 노출됐음을 뜻하지 않는다. 촬영→발행→수신→수집
  시각과 각각의 age를 보존하되 실제 센서간 skew 및 RC 반응 지연은 추가 측정 대상이다.
- camera sensor model은 synchronous render+처리/전송 queue를 사용한다. GUI 4Hz에서는
  capture뿐 아니라 delivery polling 지연이 더해져 두 영상 age가 최대 약 .498s였다.
  25초 관측 유효율 0%, 고유 stamp 비율 약 .406, RTF .521. camera rate를 올린다고 모든
  해상도·GUI 부하에서 유효성이 보장되는 것은 아니다.
- 초기 GUI 측정: wrench .93–.96ms, total 1.52–1.59ms, RTF .52–.54.
  마지막 별도 headless 30Hz 시험: 50 rows/5 sim seconds, interval 약 .100000s,
  모든 validity 1, 양쪽 고유 프레임 50/50, camera 최대 age .065s. 동일 GUI 부하 비교는 아니다.
- pause가 1초 넘으면 episode 종료. 짧은 pause는 샘플을 생성하지 않고 동일 sim-time 흐름을
  이어갈 수 있다. reset/backward는 캐시를 비우고 새 start 필요. process 재시작은 새 episode.
  node shutdown은 저장된 행이 있으면 실패/중단으로 보존한다. 강제 kill/전원 차단의
  `.recording_episode_*` 자동 복구는 이번 작업에서 보장하지 않는다.
- 전방 위치 `(0.247,0,0.037)`, 손 `(0.247,-0.044,-0.033)`m, fovy 70°/82°.
  갈퀴/프레임 자체 가림은 렌더에 포함된다. transparent front acrylic은 광학 가정이며
  수중 굴절/실제 창 투과율/auto-exposure가 아니다. 기본 D=0, 물속 attenuation·scatter·noise는 prior.
- MuJoCo GT pose, 객체 위치, 부표 성공 판정, sonar segmentation/target position는 state23이나
  이 KMU26 config 입력에 없다. 시뮬 센서를 생성할 때 GT를 쓰는 것과 GT를 정책에 주는 것은 다르다.

## 재현 절차

### 검사 환경

ROS 코드는 Ubuntu22.04/Humble 컨테이너의 Python3.10, 영상/물리 검사는 기존
`/workspace/.venv/bin/python`을 사용했다. PyArrow/pandas/ruff는 생성물 디렉터리에만 설치했다.
학습 입력 검사는 호스트 IsaacLab Python3.12의 torch와 별도 `loader-deps`를 사용했고,
U0 요구 버전 `transformers==4.51.3`, `albumentations==1.4.18`을 분리 설치했다.
IsaacLab/core ROS의 required dependency는 변경하지 않았다. PyTorch3D는 별도 원본 clone의
Python transforms만 사용했다. 완전한 optimizer/trainer/GPU 정책 실행은 검증하지 않았다.

ROS 환경에서 저장소 루트 기준:

```bash
source /opt/ros/humble/setup.bash
source rospkg/install/setup.bash
export PYTHONPATH="$PWD/rospkg/src/auv_vla_data_collector:$PWD/rospkg/src/kmu26_auv_vla_policy:$PYTHONPATH"
python3 -m pytest -q rospkg/src/auv_vla_data_collector/test rospkg/src/kmu26_auv_vla_policy/test
.venv/bin/python -m unittest discover -s uuv_mujoco/current/tools -p '*camera*.py'
python3 uuv_mujoco/current/tools/check_vla_depth_reference.py
python3 rospkg/src/auv_vla_data_collector/tools/measure_input_timing.py \
  --use_sim_time --duration 25 --output /tmp/vla-input-timing.json
```

실제 U0 환경에서 동일 소스 경로를 PYTHONPATH에 포함한 뒤:

```bash
python rospkg/src/auv_vla_data_collector/tools/check_u0_loader.py \
  outputs/vla-transfer-audit-20260910/smoke-final/lerobot --inspection
```

`--inspection` 없이 연결 검사 자료를 전달하면 의도적으로 거부한다.
`outputs/vla-transfer-audit-20260910/run_loader.sh`, `run_training_tests.sh`는 이 워크스테이션의
검사 환경을 그대로 재현한다. 로더만 검증하며 모델 가중치 다운로드/학습은 하지 않는다.

### 새 수집 준비와 조종 절차

1. 기존 episode를 종료하고 GUI/수동/정책 중 실제 RC 발행자를 하나만 남긴다.
   ROS publisher 1개 검사는 필요조건이며 QGC의 직접 MAVLink 입력까지 차단하지는 못한다.
   다른 MAVLink 조종기를 비활성화하고 같은 ArduSub mode/RC calibration을 사용한다.
2. `REAL_STACK_PARITY.md` strict launch를 사용하되 연구풀 distributed 설정, 두 카메라
   30Hz로 시작한다. 이 시험의 해상도는 **320×240**였다. 원하는 해상도/GUI 부하에서
   다시 freshness/고유 프레임 비율을 측정한다. GUI의 기존 4Hz를 그대로 사용하지 말 것.
   현재 센서 위치의 수평 기준 런치 인수는
   `surface_pressure_pa:=101640.0 depth_zero_at_start:=false depth_offset_m:=0.03286`.
3. `tools/create_acquisition_context.py --repository ... --output ... --files ...
   --launch_description ...`로 실제 로드한 scene/physical profile/sensor JSON/YAML/런치 및
   ArduSub parameter dump를 기록한다. 실제 장착 측정값인지 prior인지 그대로 명시한다.
4. collector.yaml 사본에 `data_source: simulation`, `collection_kind: task_demonstration`,
   고유 `session_id`, `provenance_file` 절대경로, `expected_mode: STABILIZE`를 지정한다.
   로봇에서는 `data_source: real`, `use_sim_time:=false`. 단일 세션의 모든 유사 시연은
   동일 session_id를 유지한다. 서로 다른 실측 보정/물리 모델은 새 세션을 만든다.
5. 빌드한 패키지를 source하고 수집기를 띄운다:

```bash
ros2 launch kmu26_auv_vla_data_collector collector.launch.py \
  config:=/absolute/path/collector_sim_demo.yaml use_sim_time:=true
ros2 topic pub --once /vla/task_description std_msgs/msg/String \
  "{data: 'Approach the red buoy and detach it with the right rake.'}"
ros2 service call /vla_data_collector/start_episode std_srvs/srv/Trigger '{}'
```

6. 먼저 풀 중앙에서 접근/정지 10–20초를 천천히 조종하고 센서 장애/제어권 변경이 없는지
   확인한다. 다음에 부표 접근→갈퀴 정렬→접촉→분리까지 연속 20–60초 시연한다.
   작업 의도에 맞는 자세·속도·결과를 두 영상으로 판독할 수 있어야 한다. 성공은 운영자가 확인한다.
   시작 준비 중 비무장 정지는 connection_check 별도 root에 저장한다.
7. 성공이면 stop `data:true`. 실패/회복 시연은 `data:false`로 별도 검토 root에 저장;
   센서 오설정/사람 개입 등은 discard. 성공 여부를 exporter가 추측하지 않는다:

```bash
ros2 service call /vla_data_collector/stop_episode std_srvs/srv/SetBool '{data: true}'
# 또는
ros2 service call /vla_data_collector/discard_episode std_srvs/srv/Trigger '{}'
```

8. 먼저 1개 episode의 raw image, NPZ, source/receipt ages, RC request와 FCU in/out 및
   종료 이유를 확인한다. A50 품질 gate/dropout, command 지연은 0으로 가정하지 않는다.
   소량 작업 시연을 완성하면 연결용 정지 자료와 별도 폴더에서 export한다:

```bash
python3 -m kmu26_auv_vla_data_collector.export_lerobot staging_train lerobot_train
python rospkg/src/auv_vla_data_collector/tools/check_u0_loader.py lerobot_train
```

학습/검증은 **export 이전에 전체 session 단위로 분리**한다. 이 로더는 info.json의
split 문자열만으로 데이터를 나누지 않는다. `assert_disjoint_sessions(train,val)`로 session
중복을 검사한다. 비슷한 재시도/같은 초기 장면은 동일 그룹으로 묶고 파일을 복제해 새 ID를
붙이지 않는다. 각 폴더의 stats를 자동 혼합하지 말고, validation/inference는 train/checkpoint
통계를 사용해야 한다. 새 데이터 혼합 비율/normalization 가중치는 아직 정하지 않았다.

실제 학습에 사용할 때 `tools/finetune_transfer.py /absolute/path/auv_vla --dataset-path ...`
진입점은 단일 사전 선택 데이터 root와 완전한 미래 구간 loader를 원본 trainer에 연결한다.
옵션 `kmu26_auv_vla_data_collector.transfer_config:Kmu26TransferDataConfig`를 저장하고
서버에서도 **동일 config**를 선택한다. 원본 `kmu26_auv_real`을 계속 쓰면 원본 46차원
sin/cos 전처리가 유지되므로 두 설정의 가중치를 혼용하지 않는다. 이 trainer wrapper의
optimizer 실행은 아직 하지 않았으므로 GPU/학습 의존성 전체 검증과 구분한다.

## 물리/실물 보정 우선순위 및 최소 전이 시험

| 우선순위 | 먼저 확인할 실물 측정 | 시뮬에서 선행 가능한 검사 |
|---|---|---|
| P0 제어 | 네 축 ±작은 PWM pulse, 중립/trim/reversal/deadzone, RCIn 및 FCU mode/armed, timeout; 센서/영상 시각과 응답 지연 | RELEASE/NOCHANGE/정지/시간 역행/모드변경 회귀, 명령·실추력·결과 속도 별도 분석 |
| P1 운동 | 장비 장착 상태 질량/CoM/부력·관성, 수중 free-decay, 방향별 low-speed drag, T200 입력/전압별 추력과 상승/하강 지연 | 부력 평형/자유 감쇠/축별 응답 수치 검사. 기준 오차를 노이즈로 숨기지 않음 |
| P1 센서 | DVL/IMU/Bar30 optical/acoustic 기준점 실측, 회전시 DVL ω×r, 정지 IMU ±g, 기준 수압과 수면/알려진 깊이 | TF/좌표계·quaternion 순서, 수평 depth reference, invalid/지연·packet gate 검사 |
| P1 영상 | 각 카메라 수중 checkerboard, 장착 optical TF, 실제 FOV/창 굴절, 해상도·노출·gain·색, 갈퀴 가림 | K/P와 rendered FOV 일치, crop 뒤 손끝/부표, GUI overlay 비포함, RGB order 검사 |
| P1 접촉 | 부표 실제 질량/순부력, 자석 인장·전단·peel release 곡선, 갈퀴/하우징 젖은 마찰 | 15N load release, 접촉 침투/solver failure/줄 충돌 fixture. 성공 시연을 실물 힘 성공으로 해석하지 않음 |
| P2 줄/유체 | 줄 길이/밀도/강성/감쇠, 유속, tether 영향 | 현재 6 ball joints/rope, coarse bending, 작은 viscous+joint damping; 촘촘한 감김/매듭/탄성/파단은 표현하지 않음 |

현재 부표 10g/순부력 약 1N, 15N resultant weld release, 1ms 확인시간,
마찰 .30/.35, vehicle 유체 계수 등은 실측이 아닌 가정이다. 물살 105-point 일괄 계산
최적화의 식/지점/주기를 이번 작업에서 바꾸지 않았다. 랜덤화는 먼저 기준 응답을 맞춘 후
반복측정 분포(질량 오차, drag/추력 fit 신뢰구간, 실제 유속, 지연·dropout·노출 범위)로 제한한다.
분포를 아직 정할 근거가 없어 임의 ±%나 sim:real 비율을 지정하지 않는다.

실물 준비 후 최소 자료: 정지 30초, 각 축 ±단독 조작 5–10초씩, 천천히 yaw/roll/pitch
회전하여 DVL/IMU/reference 확인, 알려진 두 깊이, 두 카메라의 수중 보정/갈퀴 접근 영상,
당시 FCU parameter dump·launch/YAML·센서 serial/calibration/장착값. ROS bag은 두 image와
CameraInfo, DVL raw/twist, IMU, depth/pressure, RC override/in/out, MAVROS state 및 TF 포함.
그 뒤 쉬운 부표 접근부터 낮은 제한·deadman으로 dry-run 제안 행동과 사람이 조종한 행동을
비교하고, shadow inference→짧은 수중 폐루프→접촉 과제 순서로 평가한다. 성공률/접촉력/
이탈·추가 개입/센서 결측/응답 지연을 session 분리 검증 자료에서 측정한다.

## 검증 결과와 남은 조건

- ROS 수집/내보내기/정책 clock **32개**, 카메라 sensor/publisher **38개**, 실제 U0 loader
  **4개** 테스트가 통과했고, 두 ROS 패키지의 분리 colcon build도 완료했다.
  depth-reference 검사도 통과했다. 검사 로그는 생성물 디렉터리에 있다. 회귀는 보존된 이전 구현
  또는 정확한 이전 동작(`--without_fix`)에서 실패한 뒤 수정본에서 통과했다.
  colcon의 기존 `tests_require` setuptools 경고는 남아 있으나 build 오류는 없었다.
- 마지막 연결 자료: state `(50,23)`, action `(50,4)`, 전 상태 유한, 양쪽 camera/DVL validity
  100%, 양쪽 고유 프레임 100%, MP4 50프레임씩. 영상 육안 확인: 전방/우측 갈퀴, GUI 없음.
- 실제 원본 U0 decoder/transform/GR00T packing: 35개의 완전한 16-step 구간, RGB 영상,
  state `(1,64)`와 23개 true mask, action `(16,32)`와 64개 true mask. 모델 학습/추론 성공을
  주장하지 않는다. 연결용 파일이 학습 진입점에서 거부되는 것도 시험했다.
- 현재 자료의 용도는 수집기 연결/형식/로더·배포 인터페이스 회귀다. 작업 행동 학습,
  sim/real 혼합 통계 확정, 성능 비교, 실물 전이 성공의 근거로 쓰면 안 된다.→변환→실제 U0 모델 입력까지 검증했지만
- 다음 필요한 것은 운영자가 조종한 실제 작업 시연, 실물 최소 샘플 경로, 보정·RC 채택/
  지연 확인, session 분리, 그 후 별도의 학습·held-out 과제 평가다.
