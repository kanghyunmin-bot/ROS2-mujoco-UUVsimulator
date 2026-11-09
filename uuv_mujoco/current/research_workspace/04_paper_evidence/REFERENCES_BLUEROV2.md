# BlueROV2 구현과 논문 그림에 사용한 자료

2026-11-09 · BlueROV2 Standard 6추진기 모델과 DFKI 실측 재생

논문용 인용 항목은 [references_bluerov2.bib](references_bluerov2.bib)에 모았다.

## 기체와 시뮬레이터 구현

| 항목 | 적용한 자료와 범위 | 구현 위치 |
| --- | --- | --- |
| 선체·프로펠러와 추진기 장착 위치 | Centrale Nantes의 공개 ROS 2 Xacro/메시, 커밋 `dd7111b609b1d8bad052e4990d9b439e5317ad7c`. 선체 OBJ를 재질별로 분리하고 프로펠러 DAE를 OBJ로 변환했다. Xacro의 6개 장착 원점을 쓰고, 선체와 추진기 전체를 위로 72.5 mm 옮겨 수평 추력 평면을 z = 0에 맞췄다. 수평 추력축은 Xacro의 로터 축, 두 수직 추력축은 공개 Gazebo SDF의 −z를 따랐다. [ROS 2 모델](https://github.com/CentraleNantesROV/bluerov2/tree/dd7111b609b1d8bad052e4990d9b439e5317ad7c/bluerov2_description) · [Gazebo SDF](https://github.com/clydemcqueen/bluerov2_gz/blob/4451a5d1b15d204581d43ef7d93df4a0508eddbb/models/bluerov2/model.sdf) | [메시 및 라이선스](../../assets/bluerov2_standard/README.md), [변환 명세](../../assets/bluerov2_standard/manifest.json), [MuJoCo 기체](../../scenes/bluerov2/vehicle_body.xml) |
| T200 추력 | Blue Robotics의 2019년 10–20 V 공개 시험표. 정·역방향 PWM→RPM 및 RPM→추력 곡선을 사용하고, DFKI 재생에서는 기록 전압으로 보간한다. 40/60 ms 가속·감속 시정수는 시뮬레이터 초기 가정이다. R4 프로펠러 회전 방향은 제조사 조립 가이드를 따랐다. [T200 시험표](https://cad.bluerobotics.com/T200-Public-Performance-Data-10-20V-September-2019.xlsx) · [조립 가이드](https://bluerobotics.com/learn/bluerov2-assembly/#installing-the-thrusters) | [T200 테이블](../../config/thruster_performance.json), [모터 매핑](../../physics/thruster_mapping.py), [추진 모델 문서](../../../../docs/THRUSTER_MODEL.md) |
| SITL과 DFKI 모터 매핑 | 대화형 ArduSub SITL은 Standard의 6추진기 `vectored` (`FRAME_CONFIG=1`)를 사용한다. 오프라인 재생에서는 DFKI 로그에 남은 최종 ESC PWM 6채널을 사용한다. DFKI 5번 모터의 반대 극성은 명령과 실측 운동의 방향으로 판단해 재생 매핑에 따로 반영했다. [ArduSub 프레임 문서](https://ardupilot.org/sub/docs/sub-frames.html) · [DFKI 로그 v3](https://zenodo.org/records/17360027) | [SITL 시작 설정](../../start_ardusub_sitl_mj311.sh), [모터 매핑](../../physics/thruster_mapping.py) |
| 센서 TF | IMU·카메라 장착점은 공개 Xacro의 위치를 반영했다. MuJoCo 카메라 시점은 CAD 돔을 피하도록 마운트보다 55 mm 앞에 뒀다. 압력 센서 장착 오프셋은 임시 설정이다. 이 TF의 근거는 공개 모델이며 DFKI 실험 기체의 측량값은 확인되지 않았다. [센서 Xacro](https://github.com/CentraleNantesROV/bluerov2/blob/dd7111b609b1d8bad052e4990d9b439e5317ad7c/bluerov2_description/urdf/sensors.xacro) | [MuJoCo 센서 사이트](../../scenes/bluerov2/vehicle_body.xml) |
| 유체 감쇠 초기값 | Li et al. (2020), Table 2의 surge·sway·heave·yaw 선형 및 이차 감쇠. Roll·pitch 이차 감쇠의 크기 3.992는 공개 Gazebo 모델에서 가져왔고, 해당 두 축의 선형 감쇠는 0으로 시작했다. [Li et al. 논문](https://doi.org/10.3390/jmse8090688) · [Gazebo SDF](https://github.com/clydemcqueen/bluerov2_gz/blob/4451a5d1b15d204581d43ef7d93df4a0508eddbb/models/bluerov2/model.sdf) | [기본·보정 프로파일](../../config/sim_profiles_bluerov2.json) |
| 질량·CoM·CoB·부가질량 | 질량 10.012 kg, 대각 관성, CoM 오프셋 11 mm, 초기 중성부력과 CoM 위 20 mm의 CoB는 모델 초기 설정이다. 초기 프로파일의 유체 밀도는 1000 kg/m³이며 유속은 0으로 둔다. 부가질량은 생략했다. DFKI 압력에서 초기 깊이를 환산하는 전처리에는 별도로 1025 kg/m³를 가정했다. | [기체 명세](../../../../docs/contracts/BLUEROV2_MODEL.md), [MuJoCo 기체](../../scenes/bluerov2/vehicle_body.xml), [프로파일](../../config/sim_profiles_bluerov2.json), [로그 전처리](../../tools/prepare_dfki_bluerov2.py) |
| 재질과 접촉 마찰 | HDPE 프레임, 아크릴 튜브, 알루미늄 캡, 폴리카보네이트 페어링은 제조사 자료의 재질 정보를 참고했다. 미끄럼 마찰계수 0.35·0.40·0.50·0.45는 충돌 프록시에 부여한 자체 초기값이다. 유체 감쇠는 별도 모델에서 계산한다. [제조사 기술 자료](https://bluerobotics.com/learn/technical-reference/) · [제품 사양](https://bluerobotics.com/store/rov/bluerov2/) | [접촉 명세](../../../../docs/contracts/BLUEROV2_CONTACT.md), [충돌 프록시](../../scenes/bluerov2/vehicle_body.xml) |

## 실측 데이터와 논문 그림

**DFKI 수조 데이터 v3.** Khanmeh, Wehbe, Simetti, Indiveri가 공개한 `rov_tether_dataset_v3.zip`을 사용했다. 고정 버전 DOI는 `10.5281/zenodo.17360027`, 배포 파일 MD5는 `f357044a1d6d1f76164c8c94048ef2ae`다. 자료 논문은 *Scientific Data* 13, 35 (2026), DOI `10.1038/s41597-025-06347-0`이다. [Zenodo v3](https://zenodo.org/records/17360027) · [데이터 논문](https://doi.org/10.1038/s41597-025-06347-0)

| 원본 CSV | 실제 사용 |
| --- | --- |
| `rc-out.csv`, `battery.csv` | ESC PWM 6채널을 원래 타임스탬프로 재생하고, 팩 전압으로 T200 추력을 보간한다. |
| `velocity.csv`, `qtm_rov.csv`, `pose.csv` | 모션캡처 속도를 기체 좌표계로 변환해 surge·sway를 구하고, 위치의 수직 성분에서 상대 수심을 구한다. 마커 및 위치 유효성을 적용한다. |
| `attitude.csv` | 탑재 센서가 기록한 roll·pitch·yaw와 각속도. 결과 그림의 자세 실측은 원본 자세 시각을 기준으로 정렬한다. 누적 yaw는 각도 래핑을 풀어 계산한다. |
| `pressure2.csv` | 압력에서 재생 시작 깊이를 산출한다. 그림의 상대 수심 곡선은 모션캡처에서 구한다. |

- **Figure 03 일반 모델.** 무작위·원형·5/10/15 m 테더 주행 5개로 보정하고, serpentine·7/12/14 m 테더 주행 4개를 별도 평가에 사용했다. 감쇠, 부력/CoB, 추진기 이득 및 모멘트 항 등 24개 유효 파라미터는 이 실측 재생에서 식별했다. 시뮬레이션에는 테더 동역학이 생략되어 있다. [보정 코드](../../tools/calibrate_bluerov2_dfki.py) · [재생 코드](../../tools/replay_bluerov2_dfki.py) · [평가 코드](../../tools/score_bluerov2_dfki.py)
- **Figure 01·02 결과.** 빨간선은 추가 yaw 보정을 포함한 실험용 `bluerov2_dfki_yaw_experiment`다. 그림에 사용한 무작위 주행은 보정에 포함되었다. 누적 yaw RMSE 28.9°는 이 무작위 주행에서의 값이다. 자세 외 실측 결측 구간은 남겨 두고, 축별 RMSE는 모든 표시 모델과 실측의 유효한 10 Hz 짝으로 산출했다. [실험 프로파일](../../config/sim_profiles_bluerov2.json) · [그림 자료 설명](../../../../docs/paper/README.md)

## 일반 6축 보정 알고리즘

- **가중 잔차 벡터 `r(θ)`.** 5개 DFKI 보정 주행에서 동일 PWM·전압을 입력해 50 Hz 축약 동역학 모델을 적분한다. 8초·60초·전체 구간의 선속도, 각속도, 수심, 자세 예측값에서 대응 실측값을 뺀 뒤, 10 Hz 유효 표본을 신호별 스케일과 구간별 가중치로 정규화해 쌓는다. 초기 계수에서 지나치게 벗어나는 것을 억제하는 정규화 항도 같은 잔차에 포함한다. [잔차 구성 코드](../../tools/calibrate_bluerov2_dfki.py#L153) · [DFKI v3](https://zenodo.org/records/17360027)
- **자코비안 `J`와 갱신량 `Δθ`.** SciPy `least_squares`를 상·하한, 파라미터 스케일, `diff_step=1e-4`로 호출한다. 기본 2점 유한차분으로 각 계수 변화에 대한 잔차의 민감도를 계산한다. 기본 Trust Region Reflective(TRF)는 `r(θ + Δθ) ≈ r(θ) + JΔθ`의 국소 근사를 이용해 제약 내에서 오차 제곱합을 줄이는 갱신을 구한다. Branch, Coleman과 Li (1999)는 SciPy가 TRF 설명에서 인용하는 알고리즘 배경이다. [최적화 호출](../../tools/calibrate_bluerov2_dfki.py#L175) · [SciPy 문서](https://docs.scipy.org/doc/scipy/reference/generated/scipy.optimize.least_squares.html) · [Branch et al. (1999)](https://doi.org/10.1137/S1064827595289108)
- **모델에 적용.** 갱신 대상은 선형·이차 감쇠 12개, 부력과 CoB 4개, 추진기 이득 6개, yaw·pitch 모멘트 항 2개다. 기본 프로파일의 감쇠 출처는 위 구현 표에 기록했다. 보정 후보는 전체 1 ms MuJoCo 재생에서 선택하고, 선택한 계수는 별도 주행 평가 전에 고정했다. [파라미터 내보내기](../../tools/calibrate_bluerov2_dfki.py#L180) · [일반 보정 프로파일](../../config/sim_profiles_bluerov2.json)

## Figure 03 도식과 사진

- Jia et al.의 Figure 1을 보정과 별도 검증을 나누는 구성에 참고했다. Toyota Frontier Research Center의 Real2Sim 도식을 공통 입력을 실물과 시뮬레이션에 분기하고 응답 오차를 파라미터 조정으로 되돌리는 배치에 참고했다. BlueROV2의 보정은 50 Hz 모델과 bounded least squares를 사용했다. [Sensors 논문 Figure 1](https://pmc.ncbi.nlm.nih.gov/articles/PMC6567314/#sensors-19-02248-f001) · [Toyota Real2Sim 도식](https://global.toyota/en/mobility/frontier-research/44105235.html)
- 사진은 Blue Robotics Media Kit의 BlueROV2 Standard 대표 제품 사진이다. 제조사는 Blue Robotics 출처 명시를 조건으로 복제·재배포·수정을 허용한다. 중앙 응답 그림은 DFKI 무작위 보정 주행의 20–33초 구간에서 실측 surge와 초기 `bluerov2_standard`의 MuJoCo 재생을 나란히 표시한 예시다. [제품 사진 원본](https://bluerobotics.com/images/media-kit/BlueROV2-1.png) · [Media Kit 사용 조건](https://bluerobotics.com/press/)

## Figure 02의 공개 비교 모델

| 비교군 | 재생에 사용한 공개 자산과 조건 |
| --- | --- |
| Stonefish 1.5.0 | `fjiwrogno/stonefish_ros2_marine_robotics`의 6추진기 시나리오 `bluerov_fls.scn`, 커밋 `ab779c0d430de6c486341a433dd725a95bc4b93e`. 공개 유체력·추진기 계수를 유지하고 구형 액추에이터 XML을 호환 형식으로 변환했다. 자유수역, 물리 및 추진기 200 Hz, 선체 유체력 50 Hz. 최종 PWM을 극성 적용 후 `(PWM − 1500)/400`으로 정규화했다. [고정 버전 시나리오](https://github.com/fjiwrogno/stonefish_ros2_marine_robotics/blob/ab779c0d430de6c486341a433dd725a95bc4b93e/sf_ws/src/cola2_stonefish_ros2/scenarios/bluerov_fls.scn) |
| Gazebo Harmonic Sim 8.15.0 / DART | `clydemcqueen/bluerov2_gz`의 `model.sdf`, 커밋 `4451a5d1b15d204581d43ef7d93df4a0508eddbb`. 공개 Thruster·Hydrodynamics·Buoyancy 설정을 유지했다. 5 ms 스텝, 바닥 깊이 8 m, 벽 없음. PWM은 극성을 반영해 직접 추력 `(PWM − 1500)/8` N으로 환산했다. [고정 버전 SDF](https://github.com/clydemcqueen/bluerov2_gz/blob/4451a5d1b15d204581d43ef7d93df4a0508eddbb/models/bluerov2/model.sdf) |

두 공개 모델은 약 5.63 m 깊이와 실측 초기 자세·속도에서 시작했다. 초기 공개 모델 재생의 [Zenodo 기록](https://zenodo.org/records/15976877)과 v3는 무작위 주행 `rc-out.csv`의 SHA-256이 일치한다. 공개 모델별 추진기 변환과 수조 조건은 각각의 재생 설정을 따랐다. 따라서 이 그림은 같은 PWM에 대한 **해당 구성들의 실측 재현 결과**로 인용한다.

공개된 오버레이, 표, 10 Hz 정렬 데이터는 [논문 그림 자료](../../../../docs/paper/README.md)에 있다.
