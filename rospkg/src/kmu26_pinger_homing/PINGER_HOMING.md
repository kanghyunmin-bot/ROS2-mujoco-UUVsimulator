# KMU26 Pinger Homing

실물 핑거 근방까지 자율 접근하는 전용 ROS 2 패키지다. 하이드로폰 추정 알고리즘은
저장소 최상위 `hydrophone.repos`에 고정된 별도 팀 Git 포크에서 가져오며 이 패키지는
알고리즘을 복제하지 않는다.
비전/부표 FSM과 `CollectorState`는 사용하지 않는다.

구성:

- forked `audio_capture/audio_phase_estimator`
- canonical C++ Phase/SNR RC controller (`pinger_homing_controller`)
- `/odometry/filtered` moving-sensor source fit and no-odometry fallback
- direct, single-owner `/mavros/rc/override` output

실물 토픽 계약:

- 위치: `/odometry/filtered`
- 차량 상태: `/mavros/state`
- 최종 RC: `/mavros/rc/override`
- 핑거 최종 RC: `/mavros/rc/override` (표준 launch에서 직접 발행)

```bash
vcs import src < src/kmu26_pinger_homing/hydrophone.repos
colcon build --symlink-install --packages-up-to kmu26_pinger_homing
source install/setup.bash
ros2 launch kmu26_pinger_homing pinger_homing_real_interactive.launch.py \
  dry_run:=true use_audio_capture:=false tank_max_depth_m:=2.0
```

실물 사용 순서:

1. 프로펠러를 제거한 상태에서 interactive launch의 `dry_run:=true`를 먼저 검증한다.
2. MAVROS에서 ALT_HOLD와 arm 상태를 확인한다.
3. 토픽과 Phase 추정이 정상일 때만 `dry_run:=false`를 사용한다.
4. 실행 중에는 다른 `/mavros/rc/override` publisher를 띄우지 않는다. launch 종료 또는 DISARM으로 즉시 중단한다.

실물 wrapper는 시작 전에 19--22 kHz를 10초 주파수 스캔하고 후보 1~5 또는 Hz를
같은 터미널에서 받는다. 선택한 주파수로 기존 `audio_phase_estimator`를
시작한 뒤, 검증된 C++ Phase 제어기를 실행한다. 기본 `navigation_mode:=odometry`는
`/odometry/filtered`의 pose와 Phase 거리변화를 레거시 이동 궤적에 결합해 핑거 위치를
추정한다. `no_odom_phase`는 로컬라이제이션이 없는 경우의 명시적 fallback이다.
Phase 품질 필터가 초기 delta를 만들지 못해도 `/audio_phase_estimator/iq_snr_ratio`가
살아 있으면 저속 probe를 먼저 수행한다. 이 heartbeat는 탐색 시작에만 쓰며, 유효한
delta-range fit과 source lock 없이는 ALIGN/APPROACH로 넘어가지 않는다.

실물 interactive launch는 고정 pinger carrier를 더 잘 구분하도록 기본
`scan_fft_size:=16384`, `scan_fft_hop_size:=8192`를 사용한다. 따라서 96 kHz
하이드로폰에서는 **5.86 Hz/bin** (48 kHz에서는 2.93 Hz/bin)이며, 10초 스캔에서는
각각 약 116회/57회의 50% 중첩 FFT 창이 남는다. 두 오디오 채널은 PCM으로 합치지 않고
각 채널의 band-median noise floor로 정규화한 FFT power를 합치므로 위상 반전이나
채널 gain 차이로 21 kHz가 사라지지 않는다. 수조 시뮬레이터의 generic selector
기본값 8192는 빠른 반복 시험용으로 그대로 유지된다. 강한 단일 창뿐 아니라 같은
주파수 cluster가 전체 창의 기본 30% 이상 반복되는 약한 carrier도 후보로 인정한다.

C++ launch는 자동 아밍하지 않는다. audio, MAVROS state 또는 IMU가
stale이거나 disarm/ALT_HOLD 이탈이면 모든 RC 채널을 release한다. 실물 IQ
거리 보정 전에는 `amplitude_range_constant=0`, `success_range_m=0`을 유지한다.

## 통합 2-D Phase/SNR 모드

`pinger_homing_test_tank.launch.py`는 이 패키지에 포함된 주파수 선택기,
오디오 IQ/Phase 추정기, 그리고 **동일한 canonical C++ controller**를 함께
실행한다. 기본 프로파일은 `ALT_HOLD + no_odom_phase + XY-only`이다.
즉 `/odometry/filtered`나 MuJoCo ground truth를 제어 입력으로 사용하지 않고,
MAVROS IMU yaw와 알려진 RC ABBA probe의 위상 거리 변화만으로 방위를 재추정한다.
Z는 ALT_HOLD가 소유하므로 RC3/heave를 명령하지 않는다.

```bash
ros2 run kmu26_pinger_homing start_pinger_homing_test_tank.sh \
  mode:=ALT_HOLD estimator_mode:=phase \
  auto_select_top:=false dry_run:=false
```

이 wrapper는 10초 감시가 끝난 뒤 같은 터미널에서 후보 번호 또는 Hz를 받는다.
후보는 기존 `kmu26_auv_hydrophone` FFT 노드의 계약을 따라, 8192-sample Hann FFT
(96 kHz 입력이면 **11.72 Hz/bin**), 50% overlap, 10초 Welch 평균 스펙트럼, band median
noise-floor SNR, local peak prominence, 그리고 반복 검출 횟수로 판정한다. 따라서 한
프레임 노이즈나 강한 side-lobe는 후보가 되지 않고, 수조 송신기 21164 Hz도 FFT peak
interpolation으로 100 Hz 단위로 반올림하지 않는다. 기본 품질 기준은 `SNR >= 9 dB`,
prominence `>= 4.5 dB`, 반복 FFT window `>= 4`회다. `scan_fft_size`,
`scan_min_snr_db`, `scan_min_peak_prominence_db`, `scan_minimum_candidate_hits` launch
파라미터로 수조/실물 잡음 환경에 맞출 수 있다. 또 최고 후보보다 기본 18 dB 이상 약한
고정 스펙트럼 피크는 `scan_relative_to_top_snr_db`로 목록에서 제외한다(모든 후보를
보고 싶으면 `:=0`). 선택 결과는 현재 실행에만 유효하며
이전 DDS 실행의 주파수 선택을 재사용하지 않는다.
`ros2 launch`에 직접 입력한 표준입력은 ROS launch 자식 노드로 전달되지 않으므로,
수동 선택에는 위 wrapper를 사용한다. SNR 모드는
`estimator_mode:=snr`로 선택한다. test-tank에서 최종 RC까지 연결할 때만
`rc_output_topic:=/mavros/rc/override`를 명시한다.

`pinger_homing_test_tank.launch.py`의 기본 Phase 프로파일은 probe `±90`, 접근 `+120`
(중립 1500, span 400), leg `1.25 s`, neutral `0.35 s`, settle `0.55 s`, sample delay
`0.40 s`, 초기 확인 2회, 접근/reprobe `3.0 s`, RC loop `30 Hz`다. 이 값은 빠른
0.4초 probe가 controller의 0.5초 하한으로 잘리고 안정 구간을 잃는 문제를 피한다.
`probe_pwm_delta:=`와 `approach_pwm_delta:=`로 바꿀 수 있다. 실물의 초기 시험은
반드시 낮은 PWM(예: ±15--25), `dry_run:=true`, 그리고 프로펠러 제거 상태에서 시작한다.
