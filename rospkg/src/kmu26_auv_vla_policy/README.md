# KMU26 VLA policy adapter

Initially adapted from the organization auv_vla repository,
commit cc4ada199d84290fcc55df6cfad003f1583fc3bf, gr00t/deployment/kmu26_{ros,contract}.py.
Local transfer fixes now also use ROS time for action chunk progression while
retaining a wall-clock safety watchdog; see the workspace transfer audit.
Upstream: https://github.com/2026-kmu-underwater-robot/auv_vla

The default sim config maps ego to IMX219 camera0 and buoy_release to IMX219 camera1.
The embedded collector needs fresh camera, IMU, depth, A50 DVL and simulation clock data.
Run cameras at 30 Hz for 10 Hz observations; 4 Hz GUI configuration is insufficient.
Do not also launch a second vla_data_collector node.

Build with colcon; source ROS Humble and rospkg/install/setup.bash.
Install requirements-ros.txt in that ROS Python environment, then:

```bash
ros2 launch kmu26_auv_vla_policy sim_policy.launch.py
```

Starts disabled, dry_run=true, outputs only /vla/proposed_rc. Supply a task description,
a running KMU26-compatible trained HTTP policy at port 8000, operator deadman and enable.
No trained weights or inference server are bundled. No automatic arming or mode changes.
The node does not arbitrate RC ownership with the GUI. Disable it before another controller.
The default config does not actuate the robot. Missing weights are not replaced by dummy actions.
