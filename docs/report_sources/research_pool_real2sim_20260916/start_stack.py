"""Run a local-only bag comparison using the actual research-pool GUI preset."""

import hashlib
import json
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path("/workspace")
EVIDENCE = Path("/evidence")
RUNTIME = ROOT / "uuv_mujoco/current"
OUT = Path(
    os.environ.get(
        "REPLAY_OUTPUT_ROOT", str(ROOT / "outputs/research-pool-real2sim-20260916")
    )
)
RUN = OUT / os.environ["CASE"]
RUN.mkdir(parents=True, exist_ok=False)
sys.path.insert(0, str(RUNTIME))
from gui.sim_launch_preset import (
    RESEARCH_POOL_DISTRIBUTED_PRESET_ID,
    build_sim_launch_preset_args,
    resolve_sim_launch_preset,
    sim_launch_preset_environment,
    validate_sim_launch_preset,
)
from gui.sim_stack_env_contract import build_gui_sim_stack_env

preset = resolve_sim_launch_preset(RESEARCH_POOL_DISTRIBUTED_PRESET_ID)
validate_sim_launch_preset(preset)
env = build_gui_sim_stack_env(
    {
        **os.environ,
        "ROS2_UUV_SITL_ALLOW_DIRECT_CMD": "0",
        "ROS2_UUV_SITL_COMMAND_MAVLINK_ENDPOINT": "same",
    },
    backend="native",
    sim_stack_dir=RUNTIME,
)
env.update(sim_launch_preset_environment(preset))
env.update(
    ROS2_UUV_DVL_DEVICE_EMULATOR_ENABLE="1",
    MUJOCO_GL="egl",
    UUV_RUN_MODE="closed_loop",
    UUV_THRUSTER_LOOP_HZ="100",
    UUV_MJ_THRUSTER_DEBUG_CSV=str(RUN / "forces.csv"),
)
values = {}
for line in (
    (EVIDENCE / "outputs/rc-trajectory-replay-20260911/launcher-params.parm")
    .read_text()
    .splitlines()
):
    parts = line.split()
    if len(parts) >= 2 and not line.lstrip().startswith("#"):
        values[parts[0]] = parts[1]
overlay = RUNTIME / "config/ardusub_bag0402_replay_overlay.param"
assert env["SITL_REAL2SIM_BAG0402"] == "1"
for line in overlay.read_text().splitlines():
    if line.strip() and not line.startswith("#"):
        key, value = line.split()
        values[key] = value
values.update(
    SERIAL4_PROTOCOL="2",
    SR4_EXT_STAT="1",
    SERIAL0_PROTOCOL="2",
    SERIAL2_PROTOCOL="2",
    SERIAL2_BAUD="921",
    SR2_EXTRA1="1",
    SR2_EXT_STAT="1",
    SR2_RC_CHAN="2",
    SYSID_MYGCS="1",
    MAV_GCS_SYSID="1",
    LOG_DISARMED="0",
)
params = RUN / "params.parm"
params.write_text("".join(f"{k} {v}\n" for k, v in values.items()))
binary = ROOT / "ardupilot_sub_stable/build/sitl/bin/ardusub"
digest = hashlib.sha256(binary.read_bytes()).hexdigest()
expected = json.loads(
    (
        EVIDENCE / "docs/assets/yaw-release-20260915/installed_clock_manifest.json"
    ).read_text()
)["binary_sha256"]
assert digest == expected, "SITL must be the verified clock-corrected binary"
commands = {
    "sitl": (
        [
            str(binary),
            "-S",
            "-w",
            "--model",
            "JSON",
            "--speedup",
            "1",
            "--sim-address=127.0.0.1",
            "--sim-port-in=9003",
            "--sim-port-out=9002",
            "--defaults",
            str(params),
            "-I0",
            "--home",
            "33.810313,-118.393867,0,90",
            "--serial0=udpclient:127.0.0.1:14551",
            "--serial3=udpclient:127.0.0.1:14660",
            "--serial4=udpclient:127.0.0.1:14700",
        ],
        RUN,
    ),
    "mujoco": (
        [
            "/workspace/.venv/bin/python",
            "run_uuv_mujoco.py",
            *build_sim_launch_preset_args(preset),
            "--sitl",
            "--sitl-mavlink-endpoint",
            "udpin:0.0.0.0:14660",
            "--ros2",
            "--ros2-real-pkg-compat",
            "--headless",
            "--no-ping360",
            "--hold-initial-depth-until-release",
            "--initial-depth-m",
            "0.6",
            "--initial-position-xy",
            "-2.0",
            "1.0",
            "--initial-rpy-rad",
            "0",
            "0",
            "0",
        ],
        RUNTIME,
    ),
    "rov": (
        [
            "ros2",
            "launch",
            "auv",
            "rov_start.launch.py",
            "use_sim_time:=true",
            "fcu_url:=udp://0.0.0.0:14551@",
            "use_dvl:=true",
            "dvl_ip:=127.0.0.1",
            "use_imx219:=false",
            "use_joy2mavros:=false",
            "use_battery_bridge:=false",
            "use_odom2mavros:=false",
            "publish_static_tf:=false",
            "surface_pressure_pa:=101640.0",
            "depth_zero_at_start:=false",
            "depth_offset_m:=0.03286",
            "use_web_gui:=false",
            "use_rviz:=false",
            "use_mission_rviz_visualizer:=false",
        ],
        ROOT,
    ),
}
files = [
    preset.scene_path,
    preset.profile_path,
    overlay,
    RUNTIME / "gui/sim_launch_preset.py",
]
manifest = {
    "preset": preset.preset_id,
    "profile": preset.profile,
    "scene": str(preset.scene_path),
    "scene_unchanged": True,
    "source_sha256": {
        str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest()
        for p in files
    },
    "sitl_binary_sha256": digest,
    "environment": {
        k: v
        for k, v in env.items()
        if k.startswith(("UUV_", "ROS2_UUV_", "SITL_"))
        or k in ("ROS_DOMAIN_ID", "ROS_LOCALHOST_ONLY", "CYCLONEDDS_URI")
    },
    "rendering": "Headless dynamics comparison; camera publishing disabled. Pool scene/assets unchanged.",
    "controller": "Frozen replay launcher defaults plus the same six-value Real2Sim overlay selected by the GUI.",
}
(RUN / "manifest.json").write_text(json.dumps(manifest, indent=2))
(RUN / "profiles.json").write_bytes(preset.profile_path.read_bytes())
(RUN / "commands.json").write_text(
    json.dumps({k: v[0] for k, v in commands.items()}, indent=2)
)
processes = []


def stop(*_):
    raise KeyboardInterrupt


signal.signal(signal.SIGTERM, stop)
try:
    for name, (command, cwd) in commands.items():
        with (RUN / f"{name}.log").open("w") as log:
            processes.append(
                subprocess.Popen(
                    command,
                    cwd=cwd,
                    env=env,
                    stdout=log,
                    stderr=subprocess.STDOUT,
                    start_new_session=True,
                )
            )
    while all(p.poll() is None for p in processes):
        time.sleep(0.5)
finally:
    for process in processes:
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGTERM)
    for process in processes:
        try:
            process.wait(timeout=8)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
