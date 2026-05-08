# UUV Sim Ubuntu 22.04 Runtime Dist2

This package is a runtime-focused Ubuntu 22.04 installer. It uses native
`/usr/bin/python3` with user-site Python packages by default, matching the
known-good Ubuntu install path. It does not include large documents, full
rosbag archives, macOS QGroundControl, or ArduPilot source.

If you are uploading a new distribution, use the zip under:

```text
dist2/ubuntu22.04/out/latest/
```

Install:

```bash
chmod +x install_uuv_sim_ubuntu22.sh
./install_uuv_sim_ubuntu22.sh --noninteractive
source ./.uuv_mujoco_env.sh
```

Install and run immediately:

```bash
./install_uuv_sim_ubuntu22.sh --noninteractive --run-after-install
```

Run:

```bash
./run_control_gui.sh
```

Headless smoke test:

```bash
cd uuv_mujoco/v2.2
READY_WAIT_SECS=60 SITL_WAIT_SECS=360 ./start_sitl_mujoco_mj311.sh -- --headless
```

Notes:
- No sample rosbag is bundled. Replay/autotune tools work after you choose a
  local rosbag path in the GUI or pass `--bag` explicitly.
- The installer clones upstream ArduPilot and downloads QGroundControl AppImage.
- Use `--python-mode venv` only if you specifically want a MuJoCo virtualenv.
