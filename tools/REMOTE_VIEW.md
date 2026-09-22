# Read-only remote MuJoCo 3D view

The optional viewer uses mjviser 0.0.14 and viser 1.1.1 in `.venv-mjviser`,
separate from the simulator and VLA environments. Both viewer and simulator use
MuJoCo 3.12.0 for binary-model compatibility.

Start the simulator GUI with `UUV_GUI_MUJOCO_VIEWER=0`, `MUJOCO_GL=egl`, and
`UUV_REMOTE_VIEW_DIR=/workspace/outputs/remote-view` in its container environment.
Then use its ordinary `stack_start` command. The sensor publisher exports body
poses at up to 15 wall-clock Hz. A one-element queue drops old snapshots, and
disk writes occur on a separate daemon thread. Export does not modify physics.
The model snapshot is written once per simulator start (currently about 692 MB).
Frames are about 3.7 KB; this is file IPC size, not measured network bandwidth.

Host command, from the repository:

```bash
.venv-mjviser/bin/python tools/remote_view.py \
  --state_dir "$PWD/outputs/remote-view" --port 8890
```

Open http://127.0.0.1:8890 in a WebGL-capable browser. It binds to loopback only.
For a browser on another machine, forward port 8890 over the existing SSH access;
do not expose the simulator control GUI publicly. The UI has no ARM, actuator,
reset, or physics-step commands. Use the existing 8878 GUI to control the robot.

This mirrors the live simulator rather than starting a second physics simulation.
Mesh geometry loads once, then body poses update. Static geometry/model edits
require a simulator restart. Contact/force overlays are not exported. If state is
older than two seconds, the page labels it disconnected rather than pretending
the frozen frame is live. No mjviser dependency is imported by the physics process.

2026-09-22 validation: binary snapshot reload, body pose equality and unchanged
qpos passed. Live export produced 31 distinct frames in two seconds, max sampled
local frame age 70 ms. This is not an end-to-end remote latency measurement.
The current Codex in-app browser reports WebGL unavailable; it connects to the
server but cannot render 3D. Use an external Chrome/Firefox with WebGL support.
Viewer log and validation: `outputs/remote-view/`.
