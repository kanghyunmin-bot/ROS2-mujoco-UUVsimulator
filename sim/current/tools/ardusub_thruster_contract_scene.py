"""MuJoCo scene wrench extraction for thruster-contract checks."""

from __future__ import annotations

import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np


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
