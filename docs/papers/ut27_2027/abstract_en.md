# UUV-SIM: Design and Implementation of an Integrated Underwater Robot Simulation Platform Based on MuJoCo

Hyunmin Kang | Department of Future Mobility, Kookmin University

Seoul, Republic of Korea | abcd1234asdf@kookmin.ac.kr

Extended Abstract

Keywords: Underwater simulation, MuJoCo, software-in-the-loop, real-to-sim calibration, vision-language-action

Underwater robot development requires a test environment that connects fluid and contact dynamics with control software, sensors, and visual observations. This study presents UUV-SIM, an integrated underwater robot simulation platform built on MuJoCo [1]. Initial requirements arose from competition ROV development and expanded into a reusable environment for control testing, real-log comparison, and learning-data collection. Alongside established marine simulators such as Stonefish [2], UUV-SIM explores an implementation route that uses a general-purpose robotics engine to pursue lower computational cost and reduced dependence on specialized GPU hardware.

UUV-SIM uses MuJoCo rigid-body and contact dynamics with its ellipsoid-based fluid approximation [3]. A CAD-based eight-thruster vehicle combines buoyancy, restoring torque, and PWM-to-thrust conversion. A selectable distributed model supports spatial buoyancy and drag sampling and a six-by-six added-mass matrix. Native and custom vehicle fluid loads have explicit ownership to prevent double counting. Pool and tank scenes include buoys, articulated ropes, magnetic release, and robot-object contact, extending the environment beyond free-swimming motion.

ArduSub software-in-the-loop (SITL) is connected through ROS 2 Humble [4] and MAVROS to retain the controller used on the physical robot (Fig. 1). SITL computes PWM from commands and sensor feedback; thrust drives vehicle motion, which generates subsequent observations. Physics stepping, controller sensor exchange, and ROS publication have separate schedules. Frame conventions, units, timestamps, and topic ownership are specified so controller behavior and vehicle response can be examined together.

Observations include DVL, IMU, pressure-derived depth, and front and hand cameras. Camera processing applies attenuation and backscatter according to each pixel's underwater optical path. Surface undulation and glints provide approximate water appearance (Fig. 2). Resolution, publication rate, and optical settings are configurable; web-GUI preview requests are independent of sensor publication. Visual effects are kept separate from force computation, and optical parameters remain uncalibrated priors.

The vision-language-action (VLA) interface records two images, a 23-dimensional state, and four-axis motion commands on a simulation-time sampling grid. Freshness checks, clock-reset handling, and episode provenance support consistent collection. Data are exported to LeRobot and passed through the U0 input pipeline. A disarmed connection check recorded 51 samples and loaded 36 complete 16-step action chunks, verifying data delivery rather than task learning or policy transfer.

Implementation checks address computation, motion reproduction, and data delivery. Batching immersion and current queries retained all 105 distributed samples and reduced standalone wrench-evaluation time from 2.65 to 0.47 ms on the tested workstation. A comparison over 1500 evolving physics steps detected no wrench or position difference in that trajectory. This component-level improvement does not establish end-to-end real-time operation or superiority over other engines.

Real-to-sim calibration uses closed-loop replay of recorded commands. With the latest response calibration, surge RMSE decreased from 0.505 to 0.0436 m/s in run A and from 0.303 to 0.0298 m/s in run B (Fig. 3), reductions of 91.4% and 90.1%. Against the clock-corrected model, yaw-rate RMSE over release windows decreased from 6.12 to 3.85 deg/s and from 7.07 to 4.71 deg/s. However, run B full-window yaw-angle RMSE increased from 2.51 to 5.75 degrees. Both records informed development; these are calibration checks, not untouched final tests. ESC voltage and model parameters remain uncertain.

UUV-SIM contributes an implemented MuJoCo workflow linking underwater physics, SITL control, visual sensing, and learning-data interfaces. Public code, installation instructions, configurable scenes, and validation tools support adaptation to other vehicles and experiments [5]. The integrated release currently targets Ubuntu 22.04 and ROS 2 Humble. Further work will extend real-data calibration and collect task demonstrations for VLA training and transfer evaluation.

## References

[1] E. Todorov, T. Erez, and Y. Tassa, "MuJoCo: A physics engine for model-based control," IROS, 2012, doi: 10.1109/IROS.2012.6386109.

[2] P. Cieślak, "Stonefish: An advanced open-source simulation tool designed for marine robotics, with a ROS interface," OCEANS 2019 - Marseille, 2019, doi: 10.1109/OCEANSE.2019.8867434.

[3] MuJoCo Documentation, "Fluid forces," https://mujoco.readthedocs.io/en/stable/computation/fluid.html, accessed Sep. 15, 2026.

[4] S. Macenski, T. Foote, B. Gerkey, C. Lalancette, and W. Woodall, "Robot Operating System 2: Design, architecture, and uses in the wild," Science Robotics, vol. 7, no. 66, eabm6074, 2022, doi: 10.1126/scirobotics.abm6074.

[5] H. Kang, "ROS2-mujoco-UUVsimulator," source repository, https://github.com/kanghyunmin-bot/ROS2-mujoco-UUVsimulator.

## Figure captions

Fig. 1. UUV-SIM integration: SITL closed-loop control, underwater physics, visual sensing, real-log replay, and VLA data delivery.

Fig. 2. Same-state MuJoCo renders: (a) pool overview with surface lighting, (b) submerged vehicle, and (c) buoy, braided rope and attachment detail. All views use the same model and materials.

Fig. 3. (a,b) Real DVL, nominal and latest calibrated surge; gaps exclude invalid DVL support. (c) Yaw-rate RMSE over all selected 2.5 s release windows, comparing clock-corrected and response-calibrated models. Receipt-time alignment; no fitted lag or scaling.
