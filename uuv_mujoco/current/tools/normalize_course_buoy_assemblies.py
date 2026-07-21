#!/usr/bin/env python3
"""Normalize competition buoy frames and physical anchor assemblies."""

from __future__ import annotations

import re
import xml.etree.ElementTree as ET
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SCENE = ROOT / "scenes" / "tank_current_scene.xml"
BODY_RE = re.compile(r'<body name="([^"]+)"')
POSITION_RE = re.compile(r'\bpos="([^"]+)"')
FROMTO_RE = re.compile(r'\bfromto="([^"]+)"')
INERTIAL_RE = re.compile(
    r'^(\s*)<inertial pos="[^"]+" mass="(?:0\.010|0\.050)" '
    r'diaginertia="([^"]+)" />$'
)
FLOAT_COM_TO_COB_M = 0.035
FLOAT_BODY_TO_COM_Z_M = -0.035
FLOAT_MASS_KG = 0.010
FLOAT_DIAGINERTIA = "0.000025 0.000025 0.000020"
MOORING_ROD_BOTTOM_Z_M = 0.109
MOORING_ROD_TOP_Z_M = 2.195
MOORING_TOP_JIG_Z_M = 2.205
MOORING_MAGNET_SITE_Z_M = 2.250


def values(text: str) -> list[float]:
    return [float(part) for part in text.split()]


def vector_text(parts: list[float]) -> str:
    return " ".join(f"{part:g}" for part in parts)


def replace_vector(line: str, pattern: re.Pattern[str], transform) -> str:
    match = pattern.search(line)
    if match is None:
        return line
    updated = vector_text(transform(values(match.group(1))))
    return line[: match.start(1)] + updated + line[match.end(1) :]


def course_buoy_structure(path: Path) -> tuple[set[str], set[str], set[str]]:
    root = ET.parse(path).getroot()
    body_names = {str(body.get("name")) for body in root.findall(".//body") if body.get("name")}
    float_names = {name for name in body_names if name.startswith("course_buoy_") and name.endswith("_float")}
    moored_prefixes = {
        name.removesuffix("_magnet_base")
        for name in body_names
        if name.startswith("course_buoy_") and name.endswith("_magnet_base")
    }
    shifted_float_names: set[str] = set()
    for body in root.findall(".//body"):
        name = str(body.get("name") or "")
        if name not in float_names:
            continue
        prefix = name.removesuffix("_float")
        cob = next((site for site in body.findall("site") if site.get("name") == f"{prefix}_cob_site"), None)
        if cob is not None and len(values(cob.get("pos") or "")) == 3:
            if abs(values(cob.get("pos") or "")[2] - FLOAT_COM_TO_COB_M) <= 1.0e-9:
                shifted_float_names.add(name)
    return float_names, moored_prefixes, shifted_float_names


def normalize_rigid_mooring_bases(source: list[str], moored_prefixes: set[str]) -> list[str]:
    """Put the complete fixed mooring rod and magnet on each diver-weight body."""

    output: list[str] = []
    index = 0
    while index < len(source):
        line = source[index]
        body_match = BODY_RE.search(line)
        body_name = body_match.group(1) if body_match else ""
        if not body_name.endswith("_magnet_base"):
            output.append(line)
            index += 1
            continue

        prefix = body_name.removesuffix("_magnet_base")
        if prefix not in moored_prefixes:
            output.append(line)
            index += 1
            continue

        block = [line]
        index += 1
        while index < len(source):
            block.append(source[index])
            index += 1
            if block[-1].strip() == "</body>":
                break

        indent = line[: len(line) - len(line.lstrip())]
        child = indent + "  "
        legacy_names = (
            f'{prefix}_mooring_rod',
            f'{prefix}_rod_top_jig',
            f'{prefix}_fixed_magnet_stem',
            f'{prefix}_fixed_magnet',
            f'{prefix}_magnet_site',
            f'{prefix}_rope_top_jig',
        )
        output.extend(item for item in block[:-1] if not any(name in item for name in legacy_names))
        output.extend(
            (
                f'{child}<geom name="{prefix}_mooring_rod" type="capsule" '
                f'fromto="0 0 {MOORING_ROD_BOTTOM_Z_M:.3f} 0 0 {MOORING_ROD_TOP_Z_M:.3f}" '
                'size="0.0050" rgba="0.82 0.84 0.86 1" density="0" '
                'contype="0" conaffinity="0" group="0" />',
                f'{child}<geom name="{prefix}_rod_top_jig" type="cylinder" '
                f'pos="0 0 {MOORING_TOP_JIG_Z_M:.3f}" size="0.028 0.010" '
                'rgba="0.95 0.95 0.92 1" contype="0" conaffinity="0" group="0" />',
                f'{child}<geom name="{prefix}_fixed_magnet_stem" type="capsule" '
                'fromto="0 0 2.205 0 0 2.230" size="0.0075" '
                'rgba="0.92 0.92 0.88 1" contype="0" conaffinity="0" group="0" />',
                f'{child}<geom name="{prefix}_fixed_magnet" type="cylinder" '
                'pos="0 0 2.240" size="0.018 0.010" rgba="0.04 0.04 0.04 1" '
                'contype="0" conaffinity="0" group="0" />',
                f'{child}<site name="{prefix}_magnet_site" pos="0 0 {MOORING_MAGNET_SITE_Z_M:.3f}" '
                'size="0.008" rgba="0 0 0 0" />',
                block[-1],
            )
        )
    return output


def remove_legacy_mooring_bodies_and_cables(source: list[str], moored_prefixes: set[str]) -> list[str]:
    """Remove old free magnet-jig bodies and elasticity cable composites."""

    output: list[str] = []
    index = 0
    while index < len(source):
        line = source[index]
        body_match = BODY_RE.search(line)
        body_name = body_match.group(1) if body_match else ""
        is_legacy_jig = any(
            body_name == f"{prefix}_magnet_jig" or body_name == f"{prefix}_tether_jig"
            for prefix in moored_prefixes
        )
        is_legacy_cable = any(
            f'<composite prefix="{prefix}_flex_line_"' in line
            for prefix in moored_prefixes
        )
        if not is_legacy_jig and not is_legacy_cable:
            output.append(line)
            index += 1
            continue

        closing_tag = "</body>" if is_legacy_jig else "</composite>"
        index += 1
        while index < len(source):
            if source[index].strip() == closing_tag:
                index += 1
                break
            index += 1
    return output


def normalize_float_frames(
    source: list[str],
    float_names: set[str],
    already_shifted: set[str],
) -> tuple[list[str], set[str]]:
    output: list[str] = []
    body_stack: list[str] = []
    normalized: set[str] = set()

    for original_line in source:
        line = original_line
        body_match = BODY_RE.search(line)
        if body_match:
            body_stack.append(body_match.group(1))

        current_body = body_stack[-1] if body_stack else ""
        prefix = current_body.removesuffix("_float") if current_body in float_names else ""
        needs_shift = current_body in float_names and current_body not in already_shifted

        if current_body in float_names and (
            f'name="{prefix}_com_site"' in line
            or f'name="{prefix}_cob_site"' in line
            or f'name="{prefix}_fluid_proxy"' in line
        ):
            if line.strip() == "</body>" and body_stack:
                body_stack.pop()
            continue

        if body_match and current_body in float_names and "_red_" in current_body:
            line = replace_vector(
                line,
                POSITION_RE,
                lambda parts: [parts[0], parts[1], FLOAT_COM_TO_COB_M],
            )
        elif needs_shift and body_match and current_body in float_names:
            line = replace_vector(
                line,
                POSITION_RE,
                lambda parts: [parts[0], parts[1], parts[2] - FLOAT_COM_TO_COB_M],
            )

        inertial_match = INERTIAL_RE.match(line)
        if current_body in float_names and inertial_match:
            indent = inertial_match.group(1)
            output.append(
                f'{indent}<inertial pos="0 0 {FLOAT_BODY_TO_COM_Z_M:.3f}" mass="{FLOAT_MASS_KG:.3f}" '
                f'diaginertia="{FLOAT_DIAGINERTIA}" />'
            )
            output.append(
                f'{indent}<site name="{prefix}_com_site" pos="0 0 {FLOAT_BODY_TO_COM_Z_M:.3f}" '
                'size="0.006" rgba="1 0.2 0.2 0.65" group="4" />'
            )
            output.append(
                f'{indent}<site name="{prefix}_cob_site" pos="0 0 0.035" '
                'size="0.006" rgba="0.2 1 0.4 0.65" group="4" />'
            )
            output.append(
                f'{indent}<geom name="{prefix}_fluid_proxy" type="sphere" size="0.0001" '
                'density="0" contype="0" conaffinity="0" rgba="0 0 0 0" '
                'fluidshape="ellipsoid" fluidcoef="0 0 0 0 0" />'
            )
            normalized.add(current_body)
        else:
            if (
                current_body in float_names
                and f'name="{prefix}_float_geom"' in line
                and POSITION_RE.search(line) is None
            ):
                line = line.replace("<geom ", '<geom pos="0 0 0" ', 1)
            if needs_shift and ("<geom " in line or "<site " in line):
                line = replace_vector(
                    line,
                    POSITION_RE,
                    lambda parts: [parts[0], parts[1], parts[2] + FLOAT_COM_TO_COB_M],
                )
                line = replace_vector(
                    line,
                    FROMTO_RE,
                    lambda parts: [
                        parts[0], parts[1], parts[2] + FLOAT_COM_TO_COB_M,
                        parts[3], parts[4], parts[5] + FLOAT_COM_TO_COB_M,
                    ],
                )
            output.append(line)

        if line.strip() == "</body>" and body_stack:
            body_stack.pop()

    return output, normalized


def normalize_equalities(source: list[str], moored_prefixes: set[str]) -> list[str]:
    output: list[str] = []
    for line in source:
        updated = line
        for prefix in moored_prefixes:
            if (
                f'name="{prefix}_flex_line_bottom_connect"' in updated
                or f'name="{prefix}_flex_line_top_connect"' in updated
            ):
                updated = ""
                break
            if f'name="{prefix}_magnet_weld"' in updated:
                indent = updated[: len(updated) - len(updated.lstrip())]
                updated = (
                    f'{indent}<weld name="{prefix}_magnet_weld" '
                    f'body1="{prefix}_magnet_base" body2="{prefix}_float" '
                    'relpose="0 0 2.465 1 0 0 0" solref="0.020 1" '
                    'solimp="0.95 0.99 0.0005" />'
                )
        if updated:
            output.append(updated)
    return output


def normalize(path: Path) -> int:
    float_names, moored_prefixes, shifted_float_names = course_buoy_structure(path)
    source = path.read_text(encoding="utf-8").splitlines()
    source = normalize_rigid_mooring_bases(source, moored_prefixes)
    source = remove_legacy_mooring_bodies_and_cables(source, moored_prefixes)
    source, normalized = normalize_float_frames(source, float_names, shifted_float_names)
    source = normalize_equalities(source, moored_prefixes)

    missing = sorted(float_names - normalized)
    if missing:
        raise RuntimeError(f"failed to normalize buoy bodies: {', '.join(missing)}")
    path.write_text("\n".join(source) + "\n", encoding="utf-8")
    return len(normalized)


def main() -> int:
    count = normalize(SCENE)
    ET.parse(SCENE)
    print(f"normalized {count} course buoy CoM/CoB and rigid mooring assemblies in {SCENE}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
