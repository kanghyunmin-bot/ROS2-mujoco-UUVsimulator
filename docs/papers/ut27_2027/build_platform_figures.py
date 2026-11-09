# Copyright (c) 2022-2026, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Compose platform figures from implemented topology and unaltered render panels."""

import hashlib
import json
import re
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyArrowPatch, Rectangle

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
FIG = HERE / "figures"
plt.rcParams.update(
    {"font.family": "Liberation Serif", "font.size": 8, "pdf.fonttype": 42}
)
fig, ax = plt.subplots(figsize=(7, 1.8))
fig.subplots_adjust(left=0.005, right=0.995, bottom=0.015, top=0.99)
ax.set(xlim=(0, 7), ylim=(0, 1.8))
ax.axis("off")


def box(x, y, w, h, t):
    ax.add_patch(
        Rectangle((x, y), w, h, facecolor="white", edgecolor="#344955", lw=0.8)
    )
    ax.text(
        x + w / 2, y + h / 2, t, ha="center", va="center", fontsize=8, linespacing=1.25
    )


def arrow(x1, y1, x2, y2):
    ax.add_patch(
        FancyArrowPatch(
            (x1, y1),
            (x2, y2),
            arrowstyle="-|>",
            mutation_scale=8,
            lw=0.8,
            color="#344955",
        )
    )


box(0.04, 1.12, 1.30, 0.60, "Commands\nGUI / real-log replay")
box(1.65, 1.12, 1.42, 0.60, "ArduSub SITL\nROS 2 / MAVROS")
box(3.40, 1.12, 1.69, 0.60, "MuJoCo UUV\nThrusters / fluid / contact")
box(5.39, 1.12, 1.57, 0.60, "Observations\nDVL / IMU / depth / RGB")
for x1, x2 in [(1.34, 1.65), (3.07, 3.40), (5.09, 5.39)]:
    arrow(x1, 1.42, x2, 1.42)
ax.text(3.23, 1.60, "PWM", ha="center", fontsize=7)
ax.plot([4.25, 4.25, 2.36], [1.12, 0.88, 0.88], c="#344955", lw=0.8)
arrow(2.36, 0.88, 2.36, 1.12)
ax.text(3.3, 0.73, "SITL state / internal IMU feedback", ha="center", fontsize=7.5)
ax.plot([4.75, 4.75, 1.4], [1.12, 0.61, 0.61], c="#344955", lw=0.8)
arrow(1.4, 0.61, 1.4, 0.52)
box(0.04, 0.05, 2.68, 0.46, "Real2Sim\nRecorded references / replay comparison")
box(
    3.12,
    0.05,
    3.84,
    0.46,
    "VLA data interface\nTwo images + 23D state + 4D action → LeRobot / U0",
)
arrow(0.70, 1.12, 0.70, 0.52)
arrow(6.78, 1.12, 6.78, 0.52)
for ext in ["png", "svg", "pdf"]:
    fig.savefig(FIG / f"fig1_platform_architecture.{ext}", dpi=400)
plt.close(fig)

robot = ROOT / "docs/assets/uuv-update-20260914/robot.png"
optics = ROOT / "docs/assets/underwater-camera-comparison-20260914.png"
capture = HERE / "figures/platform_capture"
panels = [plt.imread(capture / f"{name}.png") for name in ["surface", "robot", "buoy"]]
fig, axes = plt.subplots(1, 3, figsize=(7, 1.48))
fig.subplots_adjust(left=0, right=1, bottom=0.02, top=0.88, wspace=0.025)
for ax, im, title in zip(
    axes,
    panels,
    [
        "(a) Pool / surface lighting",
        "(b) Underwater vehicle",
        "(c) Buoy / braided rope",
    ],
):
    ax.imshow(im)
    ax.set_title(title, fontsize=8, pad=3)
    ax.axis("off")
for ext in ["png", "svg", "pdf"]:
    fig.savefig(FIG / f"fig2_platform_views.{ext}", dpi=400)
plt.close(fig)

files = [
    robot,
    optics,
    ROOT / "docs/gui/HYDRODYNAMIC_RUNTIME_OPTIMIZATION.md",
    ROOT / "outputs/hydro-opt-20260910/profile-before.txt",
    ROOT / "outputs/hydro-opt-20260910/profile-after.txt",
    ROOT / "outputs/hydro-opt-20260910/trajectory.txt",
    ROOT / "docs/assets/pre-vla-motion-audit-20260914.json",
    ROOT / "docs/assets/pre-vla-loader-20260914.json",
    ROOT / "docs/assets/yaw-timing-20260915/full_comparison.json",
    ROOT / "docs/assets/real2sim-application-20260915/comparison.json",
    ROOT / "uuv_mujoco/current/sim/physics/fluid_contract.py",
]
loader = json.loads(files[7].read_text())
motion = json.loads(files[6].read_text())
assert (
    loader["usable_full_chunks"] == 36 and loader["optimizer_or_inference_run"] is False
)
assert motion["rows"] == 51 and motion["state_shape"] == [51, 23]
old = float(re.search(r"unprofiled ms/step: ([\d.]+)", files[3].read_text())[1])
new = float(re.search(r"unprofiled ms/step: ([\d.]+)", files[4].read_text())[1])
assert round(old, 2) == 2.65 and round(new, 2) == 0.47
assert (
    "PASS 1500 evolving physics steps: max wrench delta 0.0 max qpos delta 0.0"
    in files[5].read_text()
)
audit = {
    "source_sha256": {
        str(f.relative_to(ROOT)): hashlib.sha256(f.read_bytes()).hexdigest()
        for f in files
    },
    "wrench_evaluation_ms": {"before": old, "after": new},
    "connection_check_samples": 51,
    "complete_action_chunks": 36,
    "new_simulation_runs": 0,
    "render_panels": "Same model, material and static state; full frames from current rendering pipelines",
    "capture": json.loads((capture / "capture.json").read_text()),
    "figure1": "Architecture diagram, not measured performance",
    "figure2": "Current static simulation renders, not photographs or task success evidence",
}
(HERE / "platform_verification.json").write_text(json.dumps(audit, indent=2) + "\n")
print("Created two platform figures; verified historical timing and VLA records.")
