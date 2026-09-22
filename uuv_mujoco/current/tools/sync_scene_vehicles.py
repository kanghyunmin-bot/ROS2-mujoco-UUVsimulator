# Copyright (c) 2022-2026, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause
"""Check or synchronize current map vehicles against Research pool."""
import argparse
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

CURRENT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(CURRENT))
from sim.vehicle_scene_contract import CANONICAL_SCENE, synchronize_vehicle, vehicle_signature


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--write', action='store_true', help='Update current map copies; never edit archived outputs')
    args = parser.parse_args()
    source = ET.parse(CANONICAL_SCENE).getroot()
    expected = vehicle_signature(source)
    mismatches = []
    for path in sorted((CURRENT / 'scenes').glob('*.xml')):
        if path.name.startswith('.') or path == CANONICAL_SCENE:
            continue
        tree = ET.parse(path)
        root = tree.getroot()
        if vehicle_signature(root) != expected:
            mismatches.append(path.name)
            if args.write:
                synchronize_vehicle(root, source)
                assert vehicle_signature(root) == expected
                ET.indent(tree, space='  ')
                tree.write(path, encoding='unicode')
    if mismatches and not args.write:
        raise SystemExit('Vehicle drift: ' + ', '.join(mismatches))
    print(('Synchronized: ' if args.write else 'Vehicle definitions match: ') + ', '.join(mismatches or ['all current maps']))


if __name__ == '__main__':
    main()
