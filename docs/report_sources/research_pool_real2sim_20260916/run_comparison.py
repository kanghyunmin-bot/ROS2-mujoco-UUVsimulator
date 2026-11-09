"""Reproduce both bag replays in a dedicated localhost-only simulation container.

Requires the existing ROS Humble Docker image, source bag extracts, ROS build,
MuJoCo virtualenv and clock-corrected ArduSub binary. Never controls hardware.
"""

import argparse
import shlex
import shutil
import socket
import subprocess
from datetime import datetime, timezone
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--evidence_root", type=Path, required=True)
parser.add_argument("--venv", type=Path)
parser.add_argument("--image", default="khm/ros2-mujoco-uuv:dev")
args = parser.parse_args()
for port in [9002, 9003, 14551, 14660, 14700]:
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.bind(("0.0.0.0", port))
stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
run_root = ROOT / "outputs" / f"research-pool-replay-{stamp}"
run_root.mkdir(parents=True, exist_ok=False)
for bag in ["primary", "auxiliary"]:
    shutil.copy2(
        args.evidence_root
        / "outputs/yaw-release-real2sim-20260915"
        / f"{bag}_commands.json",
        run_root,
    )
container = f"uuv-pool-replay-{stamp.lower()}"
scripts = "/workspace/docs/report_sources/research_pool_real2sim_20260916"
inside_root = "/workspace/" + str(run_root.relative_to(ROOT))
command = [
    "docker",
    "run",
    "-d",
    "--name",
    container,
    "--init",
    "--network",
    "host",
    "--ipc",
    "host",
    "--gpus",
    "all",
    "-e",
    "ROS_DOMAIN_ID=94",
    "-e",
    "ROS_LOCALHOST_ONLY=1",
    "-e",
    "RMW_IMPLEMENTATION=rmw_cyclonedds_cpp",
    "-e",
    f"CYCLONEDDS_URI=file://{scripts}/cyclonedds.xml",
    "-e",
    "MUJOCO_GL=egl",
    "-e",
    "NVIDIA_DRIVER_CAPABILITIES=all",
    "-e",
    f"REPLAY_OUTPUT_ROOT={inside_root}",
    "-v",
    f"{ROOT}:/workspace",
    "-v",
    f"{args.evidence_root.resolve()}:/evidence:ro",
    "-v",
    f"{(args.venv or ROOT / '.venv').resolve()}:/workspace/.venv:ro",
    "-w",
    "/workspace",
    args.image,
    "sleep",
    "infinity",
]
subprocess.run(command, check=True)
setup = "source /opt/ros/humble/setup.bash && source /workspace/rospkg/install/setup.bash && "
try:
    for bag in ["primary", "auxiliary"]:
        case = bag + "_pool"
        env = ["-e", f"CASE={case}", "-e", f"BAG={bag}"]
        start = (
            setup
            + f"python3 {scripts}/start_stack.py > {shlex.quote(inside_root + '/' + bag + '_stack.log')} 2>&1"
        )
        subprocess.run(
            ["docker", "exec", "-d", *env, container, "bash", "-c", start], check=True
        )
        subprocess.run(
            [
                "docker",
                "exec",
                *env,
                container,
                "bash",
                "-c",
                setup + f"python3 {scripts}/replay.py",
            ],
            check=True,
        )
        subprocess.run(
            [
                "docker",
                "exec",
                *env,
                container,
                "/workspace/.venv/bin/python",
                f"{scripts}/read_fcu_params.py",
            ],
            check=True,
        )
        subprocess.run(["docker", "restart", "--timeout", "3", container], check=True)
finally:
    subprocess.run(["docker", "stop", "--timeout", "5", container], check=False)
print(f"Raw evidence: {run_root}")
print(
    "Evaluate with evaluate.py --evidence_root EVIDENCE --run_dir "
    + str(run_root)
    + " --output_dir OUTPUT"
)
