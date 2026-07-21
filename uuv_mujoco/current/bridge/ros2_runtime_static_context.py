"""Static context and robot description helpers for Ros2Bridge."""

from __future__ import annotations

from pathlib import Path

import numpy as np

from .ros2_mujoco_model import sensor_slice


def sensor_slice_method(model, sensor_ids: dict, name: str, data) -> np.ndarray | None:
    return sensor_slice(model, sensor_ids, name, data)


def load_robot_description_text(self) -> str:
    workspace_root = Path(__file__).resolve().parents[3]
    ros_workspace = workspace_root / "rospkg"
    ros_source = ros_workspace / "src"
    if not ros_source.is_dir():
        ros_source = ros_workspace
    urdf_path = _first_existing_file(
        (
            ros_source / "kmu26_auv" / "urdf" / "rov.urdf",
            workspace_root / "kmu26_auv" / "urdf" / "rov.urdf",
        )
    )
    mesh_root = _first_existing_dir(
        (
            ros_source / "kmu26_auv" / "meshes",
            workspace_root / "kmu26_auv" / "meshes",
        )
    )
    try:
        text = urdf_path.read_text(encoding="utf-8")
    except OSError:
        return _fallback_robot_description()
    return _rewrite_mesh_uris(text, f"{mesh_root.resolve().as_uri()}/")


def publish_static_context(self, stamp, sim_t: float) -> bool:
    if self._static_context_publisher is None:
        return True
    return self._static_context_publisher.publish(stamp, sim_t)


def _first_existing_file(candidates: tuple[Path, ...]) -> Path:
    return next((path for path in candidates if path.is_file()), candidates[0])


def _first_existing_dir(candidates: tuple[Path, ...]) -> Path:
    return next((path for path in candidates if path.is_dir()), candidates[0])


def _rewrite_mesh_uris(text: str, mesh_uri: str) -> str:
    for prefix in (
        "package://hit25_auv/meshes/",
        "package://hit25_auv_ros2/meshes/",
        "package://kmu26_auv/meshes/",
    ):
        text = text.replace(prefix, mesh_uri)
    return text


def _fallback_robot_description() -> str:
    return (
        '<robot name="uuv_sim">'
        '<link name="auv_link"><visual><geometry><box size="0.7 0.5 0.3" /></geometry>'
        '<material name="fallback"><color rgba="0.5 0.5 0.5 1.0" /></material>'
        "</visual></link></robot>"
    )


__all__ = ["load_robot_description_text", "publish_static_context", "sensor_slice_method"]
