"""Prepare a portable MuJoCo scene with the requested vehicle sensors."""

from __future__ import annotations

import copy
import math
import shutil
from pathlib import Path
from xml.etree import ElementTree as ET


_SENSOR_SITES = {
    "imu": "imu_site",
    "pressure": "bar30_site",
    "dvl": "dvl_site",
    "camera_left": "cam_left_site",
    "camera_right": "cam_right_site",
}
_SENSOR_NAMES = {
    "imu_quat", "imu_gyro", "imu_acc", "depth_pos", "dvl_vel_body", "dvl_altitude"
}
_CAMERA_NAMES = {"camera_left": "stereo_left", "camera_right": "stereo_right"}


def _vector(value: object, length: int, label: str) -> list[float]:
    if not isinstance(value, (tuple, list)) or len(value) != length:
        raise ValueError(f"{label} must contain {length} numbers")
    try:
        result = [float(item) for item in value]
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{label} must contain finite numbers") from exc
    if not all(math.isfinite(item) for item in result):
        raise ValueError(f"{label} must contain finite numbers")
    return result


def _quaternion(value: object, label: str) -> list[float]:
    components = _vector(value, 4, label)
    norm = math.hypot(*components)
    if norm < 1e-12:
        raise ValueError(f"{label} must be nonzero")
    return [component / norm for component in components]


def _sensor_config(sensors: dict) -> dict:
    if not isinstance(sensors, dict):
        raise ValueError("sensors must be an object")
    unknown = set(sensors) - _SENSOR_SITES.keys()
    if unknown:
        raise ValueError(f"unsupported sensors: {', '.join(sorted(unknown))}")
    output = {}
    for name, source in sensors.items():
        if not isinstance(source, dict):
            raise ValueError(f"sensors.{name} must be an object")
        position = _vector(source.get("pos_m"), 3, f"sensors.{name}.pos_m")
        quaternion = _quaternion(source.get("quat_wxyz"), f"sensors.{name}.quat_wxyz")
        result = {"pos_m": position, "quat_wxyz": quaternion}
        if name in _CAMERA_NAMES:
            try:
                fovy = float(source.get("fovy_deg", 70.0))
            except (TypeError, ValueError) as exc:
                raise ValueError(f"sensors.{name}.fovy_deg must be a number") from exc
            if not math.isfinite(fovy) or not 0 < fovy < 180:
                raise ValueError(f"sensors.{name}.fovy_deg must be between 0 and 180")
            result["fovy_deg"] = fovy
            if "mount_pos_m" in source or "mount_quat_wxyz" in source:
                result["mount_pos_m"] = _vector(source.get("mount_pos_m"), 3, f"sensors.{name}.mount_pos_m")
                result["mount_quat_wxyz"] = _quaternion(source.get("mount_quat_wxyz"), f"sensors.{name}.mount_quat_wxyz")
        output[name] = result
    return output


def _expand_includes(tree: ET.Element, scene_dir: Path, model_dir: Path, seen: set[Path]) -> None:
    for parent in list(tree.iter()):
        for index, child in reversed(list(enumerate(parent))):
            if child.tag != "include":
                continue
            include_name = child.get("file")
            if not include_name:
                raise ValueError("MJCF include has no file attribute")
            include_path = (scene_dir / include_name).resolve()
            if not include_path.is_relative_to(model_dir):
                raise ValueError(f"MJCF include is outside model directory: {include_name}")
            if include_path in seen:
                raise ValueError(f"MJCF include appears more than once: {include_name}")
            seen.add(include_path)
            included_root = ET.parse(include_path).getroot()
            _expand_includes(included_root, scene_dir, model_dir, seen)
            parent.remove(child)
            for element in reversed(list(included_root)):
                parent.insert(index, copy.deepcopy(element))


def _remove_managed_elements(root: ET.Element) -> None:
    for parent in root.iter():
        for child in list(parent):
            name = child.get("name")
            if child.tag == "site" and name in _SENSOR_SITES.values():
                parent.remove(child)
            elif child.tag == "camera" and name in _CAMERA_NAMES.values():
                parent.remove(child)
            elif parent.tag == "sensor" and name in _SENSOR_NAMES:
                parent.remove(child)


def _numbers(values: list[float]) -> str:
    return " ".join(format(value, ".17g") for value in values)


def prepare_scene(
    scene_path: Path, source_model_dir: Path, sensors: dict, output_model_dir: Path
) -> tuple[Path, dict]:
    """Copy assets and configure sensors on ``base_link`` in an output MJCF.

    Paths in includes and asset declarations retain their original meaning relative
    to the input scene. The returned sensor positions are in the base-link frame.
    """
    scene_path = Path(scene_path).resolve()
    source_model_dir = Path(source_model_dir).resolve()
    output_model_dir = Path(output_model_dir).resolve()
    if not scene_path.is_relative_to(source_model_dir):
        raise ValueError("scene must be inside the model directory")
    if output_model_dir == source_model_dir or output_model_dir.is_relative_to(source_model_dir):
        raise ValueError("output model directory must be outside the input model directory")
    config = _sensor_config(sensors)
    root = ET.parse(scene_path).getroot()
    _expand_includes(root, scene_path.parent, source_model_dir, {scene_path})
    bodies = root.findall("./worldbody/body[@name='base_link']")
    if len(bodies) != 1:
        raise ValueError("scene must contain exactly one worldbody/base_link")
    _remove_managed_elements(root)
    body = bodies[0]
    sensor_element = root.find("sensor")
    if sensor_element is None:
        sensor_element = ET.SubElement(root, "sensor")
    for name, values in config.items():
        site_name = _SENSOR_SITES[name]
        pose = {"pos": _numbers(values["pos_m"]), "quat": _numbers(values["quat_wxyz"])}
        mount = {
            "pos": _numbers(values.get("mount_pos_m", values["pos_m"])),
            "quat": _numbers(values.get("mount_quat_wxyz", values["quat_wxyz"])),
        }
        ET.SubElement(body, "site", {"name": site_name, **mount})
        if name in _CAMERA_NAMES:
            ET.SubElement(body, "camera", {"name": _CAMERA_NAMES[name], **pose, "fovy": str(values["fovy_deg"])})
        elif name == "imu":
            ET.SubElement(sensor_element, "framequat", name="imu_quat", objtype="site", objname=site_name)
            ET.SubElement(sensor_element, "gyro", name="imu_gyro", site=site_name)
            ET.SubElement(sensor_element, "accelerometer", name="imu_acc", site=site_name)
        elif name == "pressure":
            ET.SubElement(sensor_element, "framepos", name="depth_pos", objtype="site", objname=site_name)
        elif name == "dvl":
            ET.SubElement(sensor_element, "velocimeter", name="dvl_vel_body", site=site_name)
            ET.SubElement(sensor_element, "rangefinder", name="dvl_altitude", site=site_name, cutoff="30")
    shutil.copytree(source_model_dir, output_model_dir, dirs_exist_ok=True)
    output_scene = output_model_dir / scene_path.relative_to(source_model_dir)
    ET.ElementTree(root).write(output_scene, encoding="utf-8", xml_declaration=True)
    return output_scene, config
