# Visual FSM automatic simulation collection

Runs the real buoy controller through an isolated simulator GUI:
STABILIZE → arm → recorder readiness → FSM start → record → label → save → map reset.
The GUI remains the only `/mavros/rc/override` publisher. The FSM publishes proposed
commands on a separate topic; the runner forwards the four PWM axes through the
GUI's existing control watchdog. Stale proposals produce neutral commands.

The separate `collection_fsm_node` is a simulator adaptation of
[the organization controller](https://github.com/2026-kmu-underwater-robot/auv_buoy_vision_control/tree/cd9c61f8d964ca2c0a01edd0941ce18f2cdd1d0e).
The legacy mission executable is unchanged. Episodes store the upstream commit
and a separate `teacher_revision`; the upstream hash alone does not describe this
adaptation. No simulator target positions or detach state are controller inputs.

## Depth search and hand alignment

- SEARCH sweeps its depth setpoint between 0.3 and 2.0 m at 0.05 m/s, with yaw search.
  Depth feedback actively controls vertical thrust; these bounds are configurable
  for the current pool and are not a learned proximity classifier.
- Front-camera approach prefers x=0.45 (left of centre). Image vertical error
  changes the depth target at at most 0.06 m/s; the depth derivative uses measured
  motion, preventing kicks when the visual target changes. Large image error
  suppresses forward motion. Front approach is limited to 1510..1540 PWM
  before RC dead-zone compensation. Hand alignment keeps forward neutral until
  both image errors enter their dead bands; then it uses a slow 1530 PWM approach.
- The current scene's hand camera points approximately 35 degrees **right** and
  29 degrees down. At buoy height/image height >=0.22, advance stops and the
  front target shifts to x=0.85 to acquire the hand view. Right-side insertion is
  accepted. Camera position and fork physics are unchanged; orientation/FOV follow the reference profile.
- A fresh hand-camera stick detection transfers control to lateral/depth alignment
  at the camera-profile fork-gap target (currently x=0.48, y=0.75). Stick height fractions 0.12 and 0.25 are configurable
  handoff/insertion-control thresholds, not success labels. A short hand occlusion
  holds the achieved depth and stops forward movement.
- RC commands compensate the verified ArduSub dead zones: yaw 40 PWM, translation
  and heave 30 PWM. Neutral and release commands remain unchanged. Match these
  parameters to the autopilot if its RC dead zones change.
- Two YOLO instances read front and hand images. Simulator-only yellow ellipse
  assistance replaces buoy predictions with yellow elliptical contours (HSV
  18..45, saturation >=60, value >=40; aspect 1.2..4; ellipse fill >=0.65;
  numerical contour floor 8 pixels). Border-touching targets are allowed.
  Its score is geometric fit, not model confidence. Hand stick detection uses YOLO plus the opt-in simulation shaft aid below.
  This color assistance is disabled by default in the shared detector.

These are explicit controller settings, not reinforcement learning or a trained
release classifier. Real simulator trials still require inspection; passing the
synthetic ROS contract test does not establish physical insertion success.

## Label definition

Success requires an initially attached magnetic target, a new release during this
episode, an inactive magnetic constraint, and actual rake contact **at release**.
The new physics status fields preserve `release_reason`, `release_force_n`, and
`release_rake_contact`, and `release_hand_contact`. The last field is restricted
to CAD hand colliders (102 parts), separately from the hull-release policy.
Both contact fields must be true for success. Proximity-only release, hull contact, target disappearance,
and FSM COMPLETE alone are not success. This conservative rule may miss a release
that occurs after the fork has already lost contact; it does not claim visual
recognition or physical-robot validation. Both camera streams remain recorded.
The event hand image is supporting evidence, not a learned release classifier.

Each saved episode has `auto_collection.json` identifying its teacher and outcome.
Failures are retained as failures. Status and per-trial reset results are stored in
the run output directory. Validation data must be reviewed before training.
No training is launched by this tool.

## GUI

GUI 상단의 **자동 수집 모드**를 누르면 수집 패널이 열립니다.
시뮬레이터를 시작한 뒤 횟수(기본 3회)를 정하고 **자동 수집 시작**을 누르세요.
현재 수동 녹화 세션·다른 자율제어가 없어야 하며 시작 전에는 DISARM 상태여야 합니다.
시작하면 자동으로 STABILIZE·ARM하고 움직입니다. **자동 수집 중지**는 현재
녹화를 실패로 저장하고 레코더를 종료한 뒤 DISARM합니다. 중단한 자료는
`label_valid=false`로 표시됩니다. 창을 닫아도 수집은 계속됩니다.

## Run

Use a dedicated simulator container/GUI and a separate ROS_DOMAIN_ID. Do not share
it with a manual operator, VLA policy, or physical robot. First build
`auv_buoy_vision_control` with the existing ROS workspace build helper. The detector
Python must have the existing ROS Python paths, torch, OpenCV, NumPy and
`ultralytics==8.4.14`; the validated environment uses `.venv-vla`.

Inside that environment, after sourcing ROS and `rospkg/install/setup.bash`:

```bash
python3 tools/auto_collection/run.py \
  --gui_url http://127.0.0.1:8878 \
  --episodes 3 --episode_seconds 120 --wall_seconds 360 \
  --output outputs/auto-collection/my-validation
```

The output directory must not already exist. The runner starts a stopped simulator,
requests arming and STABILIZE, then confirms actual vehicle state. Each trial is
bounded by both simulation and wall time; neither limit means success. On exit it
terminates its detector/FSM, releases control, saves an active episode as failure,
closes its recording session, and disarms. Inspect an interrupted recording before
using it; interrupted runs stop the batch rather than silently skipping a trial.

The progress HTML reads `status.json`. Serve only the run directory with a local
HTTP server, and open its URL. The simulator GUI separately shows the recorder.

Tests:

```bash
python3 -m unittest discover -s tools/auto_collection -p 'test_*.py'
```

## Hand camera reference profile (2026-09-20)

`uuv_mujoco/current/config/hand_camera_calibration.json` records the camera
orientation, unchanged position, 35-degree vertical field of view, provisional
fork-gap target and local image-to-body translation matrix. The runner passes
this profile to the collection FSM and records its full contents in provenance.
With camera roll, image x and y are not independent lateral/depth errors.
The local projection inverse corrects both axes together; no simulator target
position is read during control. The matrix is a local approximation near the
visible fork gap, not calibrated real-camera intrinsics or a distance estimator.

The reference photo contains no inserted target. The displayed gap and YOLO
stick-box centre correspondence remain provisional: geometric and synthetic ROS
tests cannot certify successful insertion. The original height thresholds are
image-control settings and do not represent an invariant physical distance after
a field-of-view change. Actual contact/release evidence is still required.

Run the geometric direction check with the simulator Python environment:
`python tools/auto_collection/check_hand_camera_geometry.py`.
For the isolated ROS contract, set `ROS_DOMAIN_ID=155` and
`FSM_TEST_HAND_CALIBRATION` to the absolute profile path.

## Simulation shaft aid

`simulation_stick_assist` is enabled only for the automatic collector's hand
detector. It finds low-saturation bright components (HSV saturation <=65,
value >=45), with contour area >=8 pixels and PCA elongation >=2. Components
must be adjacent to visible yellow float pixels, allowing a collar/occlusion
gap of max(12 pixels, 0.35 times yellow image height). The closest associated
component supplies a shaft box. Border-touching observations are allowed.
This is an explicit rule-based simulation observation, not newly trained YOLO
or learned insertion/release recognition. If no component qualifies, existing
YOLO observations remain available.
