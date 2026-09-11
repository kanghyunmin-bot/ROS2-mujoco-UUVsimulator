"""Verify the GUI's level-vehicle pressure offset against the current sensor mount."""

import argparse
import json
from pathlib import Path
import shlex
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from gui.ros_package_stack import mavros_launch_command

p = argparse.ArgumentParser(description=__doc__)
p.add_argument("--without_fix", action="store_true")
a = p.parse_args()
root = Path(__file__).resolve().parents[1]
z = json.loads((root / "config/sensor_mounts_2026.json").read_text())["site_positions"][
    "bar30_site"
][2]
command = mavros_launch_command("udp://0.0.0.0:14551@")
args = dict(x.split(":=", 1) for x in shlex.split(command) if ":=" in x)
offset = -0.0536 if a.without_fix else float(args["depth_offset_m"])
body_depth = 2.0
sensor_depth = body_depth - z
reported_z = -sensor_depth + offset
assert abs(reported_z + body_depth) < 1e-9, (reported_z, -body_depth)
print(
    "PASS: level-vehicle depth offset matches Bar30 mount; pitched lever arm remains a calibration task"
)
