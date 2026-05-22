#!/usr/bin/env python3
"""Verify the ArduSub VECTORED_6DOF -> MuJoCo thruster contract.

The check uses three independent pieces of data:
1) ArduSub's VECTORED_6DOF mixer factors.
2) The real vehicle MOT_x_DIRECTION values.
3) The current MuJoCo scene's thruster site positions and gear vectors.

It fails if a positive ArduSub axis command produces the wrong primary FRD
wrench sign in MuJoCo.
"""

from __future__ import annotations

import argparse
import json
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from physics.thruster_mapping import (  # noqa: E402
    ARDUSUB_VECTORED_6DOF_MOTOR_FACTORS_FRD,
    ARDUSUB_VECTORED_6DOF_SERVO_MAP,
    ARDUSUB_VECTORED_6DOF_SERVO_SIGNS,
    REAL_ROBOT_MOT_DIRECTIONS,
)

AXES = ("roll", "pitch", "yaw", "throttle", "forward", "lateral")
PRIMARY_WRENCH_INDEX = {
    "roll": 3,
    "pitch": 4,
    "yaw": 5,
    "throttle": 2,
    "forward": 0,
    "lateral": 1,
}
EXPECTED_PRIMARY_SIGN = {
    "roll": 1.0,
    "pitch": 1.0,
    "yaw": 1.0,
    # AP_Motors6DOF positive bidirectional throttle corresponds to upward
    # vehicle force. In FRD, upward force is negative Z.
    "throttle": -1.0,
    "forward": 1.0,
    "lateral": 1.0,
}


def _parse_floats(value: str, expected: int) -> np.ndarray:
    vals = [float(v) for v in value.split()]
    if len(vals) != expected:
        raise ValueError(f"expected {expected} floats, got {len(vals)} in {value!r}")
    return np.asarray(vals, dtype=np.float64)


def load_thruster_wrenches_frd(scene: Path) -> dict[str, np.ndarray]:
    tree = ET.parse(scene)
    root = tree.getroot()
    sites: dict[str, np.ndarray] = {}
    for site in root.iter("site"):
        name = site.get("name")
        pos = site.get("pos")
        if name and pos:
            sites[name] = _parse_floats(pos, 3)

    wrenches: dict[str, np.ndarray] = {}
    for motor in root.iter("motor"):
        name = motor.get("name")
        site_name = motor.get("site")
        gear_text = motor.get("gear")
        if not name or not site_name or not gear_text or site_name not in sites:
            continue
        gear = _parse_floats(gear_text, 6)
        force_flu = gear[:3]
        torque_flu = np.cross(sites[site_name], force_flu) + gear[3:]
        # FLU -> FRD. Torque is a pseudovector but the body-frame handedness
        # conversion here is the same diagonal transform used for angular rates.
        force_frd = np.array([force_flu[0], -force_flu[1], -force_flu[2]], dtype=np.float64)
        torque_frd = np.array([torque_flu[0], -torque_flu[1], -torque_flu[2]], dtype=np.float64)
        wrenches[name] = np.concatenate([force_frd, torque_frd])
    return wrenches


def build_axis_response(scene: Path) -> dict[str, dict[str, object]]:
    wrenches = load_thruster_wrenches_frd(scene)
    missing = [name for name in ARDUSUB_VECTORED_6DOF_SERVO_MAP if name not in wrenches]
    if missing:
        raise RuntimeError(f"scene is missing thruster actuators: {', '.join(missing)}")

    factors = np.asarray(ARDUSUB_VECTORED_6DOF_MOTOR_FACTORS_FRD, dtype=np.float64)
    directions = np.asarray(REAL_ROBOT_MOT_DIRECTIONS, dtype=np.float64)
    signs = np.asarray(ARDUSUB_VECTORED_6DOF_SERVO_SIGNS, dtype=np.float64)

    report: dict[str, dict[str, object]] = {}
    for axis_idx, axis in enumerate(AXES):
        command_by_thruster: dict[str, float] = {}
        channel_cmds: list[float] = []
        for motor_idx, thr_name in enumerate(ARDUSUB_VECTORED_6DOF_SERVO_MAP):
            cmd = factors[motor_idx, axis_idx] * directions[motor_idx] * signs[motor_idx]
            channel_cmds.append(float(cmd))
            command_by_thruster[thr_name] = float(cmd)
        total = np.zeros(6, dtype=np.float64)
        for thr_name, cmd in command_by_thruster.items():
            total += float(cmd) * wrenches[thr_name]
        primary_idx = PRIMARY_WRENCH_INDEX[axis]
        primary = float(total[primary_idx])
        expected_sign = float(EXPECTED_PRIMARY_SIGN[axis])
        report[axis] = {
            "primary": primary,
            "expected_sign": expected_sign,
            "primary_index": primary_idx,
            "wrench_frd": [float(v) for v in total],
            "channel_cmds": channel_cmds,
            "thruster_cmds": dict(command_by_thruster),
            "ok": primary * expected_sign > 1.0e-6,
        }
    return report


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--scene",
        type=Path,
        default=ROOT / "scenes" / "tank_current_scene.xml",
        help="MuJoCo scene XML to verify.",
    )
    parser.add_argument("--quiet", action="store_true")
    parser.add_argument("--json", action="store_true")
    args = parser.parse_args()

    report = build_axis_response(args.scene)
    failed = [axis for axis, item in report.items() if not bool(item["ok"])]
    if args.json:
        print(json.dumps({"ok": not failed, "failed": failed, "axes": report}, indent=2))
    elif not args.quiet:
        print("[thruster-contract] ArduSub VECTORED_6DOF -> MuJoCo primary FRD response")
        for axis in AXES:
            item = report[axis]
            print(f"  {axis:8s}: primary={float(item['primary']):+.6f} ok={bool(item['ok'])}")
    if failed:
        print(
            "[thruster-contract] failed axes: " + ", ".join(failed),
            file=sys.stderr,
        )
        return 1
    if not args.json:
        print("[thruster-contract] OK")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
