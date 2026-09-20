# Single-buoy VLA method and experiment contract

Status: experimental design; training and closed-loop task success are not yet validated.
The research-pool scene now contains one yellow buoy. This document does not change the submitted paper.

## Fixed simulator baseline

Default GUI preset: research_pool_distributed.
Scene: uuv_mujoco/current/scenes/research_pool_slam_scene.xml.
Fluid model: distributed.
Profile: bag0402_effective in config/sim_profiles_bag0402_effective.json.
Controller overlay: SITL_REAL2SIM_BAG0402=1.
Explicit alternate presets remain available. Record the resolved profile, environment
overrides, controller parameters, software revision and dirty patch for each run.
Calibration is an effective response fit, not independently identified physical parameters.

## Observation, model and execution

At approximately 10 Hz simulation time (ROS time in simulation), construct:
- Front RGB and work-area RGB, with independent capture stamps.
- Natural-language task instruction: approach, align with and detach the yellow buoy.
- State [23]: previous normalized command [4], DVL velocity [3],
  angular velocity [3], linear acceleration [3], attitude quaternion [4],
  depth [1], altitude [1], validity [4].
State ordering, axes, units, freshness and normalization follow the collector contract.
Altitude/validity availability must match the real robot; do not silently supply
perfect simulated measurements for unavailable real observations.

Front/work images -> vision encoder/projector -> image tokens.
Instruction -> tokenizer -> text tokens.
Image/text tokens -> pretrained vision-language backbone -> visual-language features.
State -> state encoder -> state tokens.
Visual-language features + state tokens + noisy action tokens -> flow-matching
Diffusion Transformer action head -> action chunk [16,4].
The four outputs are normalized RC commands [surge, sway, heave, yaw], not
individual thruster PWM, metric velocities, target pose or manipulator commands.
Use only the four valid action dimensions in the training loss.
Freeze/fine-tune choices must be recorded rather than inferred from the original figure.

Policy HTTP server -> ROS 2 adapter -> freshness/deadman checks -> amplitude/slew
limiter -> RC Override channels [5,6,3,4] -> ArduSub STABILIZE controller/mixer ->
eight thruster outputs -> simulated thruster dynamics and distributed hydrodynamics.
The real path replaces SITL/plant with Pixhawk/ESCs/physical thrusters.
Simulation RC PWM = 1500 + 400 * normalized command; GUI demonstrations and
sim_policy.yaml now share this range, with simulation command_limit=1.0.
The slew limit remains 0.5 normalized units/s and must be recorded in evaluations;
matching amplitude does not guarantee identical transient command execution.
Physical/default adapter settings remain span=300 and command_limit=0.3.
Do not deploy the simulation configuration to hardware without reviewing the
real command contract and explicitly remapping or recollecting training data.
Execution cadence/chunk selection uses ROS time; network deadlines and watchdogs
use wall time. Measure real inference latency separately from slowed simulation.
Keep exactly one RC owner and record requested and applied commands separately.

## CAP-free primary experiment

Both simulation adaptation and real fine-tuning/inference must disable the CAP
target branch and its loss. Local U0 code gates training and inference with
target_loss_weight=0. The transfer entry point enforces this in both the live
model and serialized configuration. The serving entry point checks it before
loading weights. Configuration round-trip tests pass; actual GPU training and
checkpoint inference remain unvalidated.
Do not give target position/pose ground truth to the policy.
The module may still be allocated for checkpoint compatibility: describe this
as CAP branch disabled, not physically removed.
A pretrained U0 checkpoint may already reflect CAP-assisted pretraining; this
does not constitute training from scratch without CAP. Record initialization.
A separate CAP comparison, if added later, is not part of the primary experiment.

## Task and demonstration quality

One episode contains approach -> alignment -> detachment.
Use one active yellow buoy in the primary simulation experiment, matching the
planned real experiment. Keep multiple buoys as a separate distractor/generalization
condition. Do not silently replace the research-pool baseline with another tank.
A future smaller pool variant should keep the same vehicle/physics contract and
change only measured geometry and declared environmental conditions.

End success on intended fork-induced detachment, with a short debounce if needed
for stable event detection. Sustained post-release holding is not a new task.
Record accidental release, non-fork collision, timeout and sensor/control faults.
Use simulator ground truth only for scoring/diagnostics, never policy observations.

A usable demonstration need not be flawless: small overshoot followed by deliberate
correction and successful re-approach can teach recovery. Keep nominal successes
and recoveries labeled separately. Exclude stale/disconnected-input periods,
operator distraction and unrecovered erroneous actions from imitation targets.
Keep the raw failed episodes for diagnosis; preserve temporal continuity and do
not concatenate disjoint pieces into artificial trajectories.

## Evaluation and randomization

Pair rule-based and learned policy trials using identical initial conditions,
low-level controller settings, action limits and termination criteria.
Split train/validation/test by episode/session/initial condition, never adjacent frames.
Report success rate with uncertainty, completion time with timeout handling,
approach/alignment error, unintended contacts, recovery and inference latency.
A single task alone does not establish language grounding or general task understanding.

First randomize robot/buoy pose within the accessible workspace. Sensor errors
use the mathematical profiles documented in SENSOR_ERROR_MODES.md. Do not add
arbitrary temporal-pattern or task-stage dropout experiments as a novelty claim.
Measured parameters and uncalibrated bias/drift/latency priors remain distinct.
Match each mechanism's location in the sensor/control pipeline. Do not add
independent white noise to every signal indiscriminately.
Compare calibrated fixed simulation versus randomized simulation; later compare
real-only training versus sim-pretrained plus real fine-tuning.
Record distributions and preserve independent real validation sessions.

Dataset-size pilots may use nested 25/50/100/200-episode subsets, with fixed held-out
conditions and multiple training seeds. These are planning values, not evidence
of sufficiency or saturation.

## Outstanding validation

Successful task demonstrations, GPU training and trained-weight round-trip,
closed-loop policy task success, automatic randomized reset/success evaluator and
real transfer remain to be implemented or validated. The one-buoy scene, existing
manual placement, magnetic release and free-rise mechanics pass local checks.
See VLA_RECORDING_READINESS_20260920.md for the current acquisition workflow.
