# KMU26 VLA policy adapter

Initially adapted from the organization auv_vla repository,
commit cc4ada199d84290fcc55df6cfad003f1583fc3bf, gr00t/deployment/kmu26_{ros,contract}.py.
Local transfer fixes now also use ROS time for action chunk progression while
retaining a wall-clock safety watchdog; see the workspace transfer audit.
Upstream: https://github.com/2026-kmu-underwater-robot/auv_vla

The default sim config maps ego to IMX219 camera0 and buoy_release to IMX219 camera1.
The embedded collector needs fresh camera, IMU, depth, A50 DVL and simulation clock data.
The default VLA lite cameras run at 15 Hz for 10 Hz observations; 4 Hz is insufficient.
Do not also launch a second vla_data_collector node.

Build with colcon; source ROS Humble and rospkg/install/setup.bash.
Install requirements-ros.txt in that ROS Python environment, then:

```bash
ros2 launch kmu26_auv_vla_policy sim_policy.launch.py
```

Starts disabled, dry_run=true, outputs only /vla/proposed_rc. Supply a task description,
a running KMU26-compatible trained HTTP policy at port 8000, operator deadman and enable.
No trained weights or inference server are bundled. No automatic arming or mode changes.
The node enforces exclusive ROS RC ownership. Use the GUI's VLA control preparation
button before live simulation RC output. It releases the GUI publisher and blocks
joystick/other autonomy input, but does not launch, arm or enable the policy.
Stop the policy process before using the GUI's return-to-manual button; disabling
the policy alone leaves its ROS publisher present and deliberately prevents return.
sim_policy.yaml uses neutral=1500, span=400 and command_limit=1.0 to match GUI data;
its slew limit remains 0.5 normalized units/s. Physical/default adapter settings
remain unchanged. See docs/contracts/VLA_RECORDING_READINESS_20260920.md at the
workspace root for the validated serving entry point and remaining limitations.
The default config does not actuate the robot. Missing weights are not replaced by dummy actions.
