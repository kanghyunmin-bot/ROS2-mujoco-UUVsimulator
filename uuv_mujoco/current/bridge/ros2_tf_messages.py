"""TF message builders and compatibility exports for the ROS2 bridge."""

from __future__ import annotations

from typing import Any, Iterable

import numpy as np

from .ros2_static_tf_specs import build_static_tf_specs
from .ros2_tf_geometry import camera_optical_quat, quat_identity, quat_x_180, site_local_pose


def build_tf_message(
    tf_message_type: type,
    transform_stamped_type: type,
    stamp: Any,
    specs: Iterable[tuple[str, str, np.ndarray, np.ndarray]],
) -> Any | None:
    transforms = []
    for parent, child, translation, quat in specs:
        transform = transform_stamped_type()
        transform.header.stamp = stamp
        transform.header.frame_id = parent
        transform.child_frame_id = child
        transform.transform.translation.x = float(translation[0])
        transform.transform.translation.y = float(translation[1])
        transform.transform.translation.z = float(translation[2])
        transform.transform.rotation.w = float(quat[0])
        transform.transform.rotation.x = float(quat[1])
        transform.transform.rotation.y = float(quat[2])
        transform.transform.rotation.z = float(quat[3])
        transforms.append(transform)
    if not transforms:
        return None
    msg = tf_message_type()
    msg.transforms = transforms
    return msg


__all__ = [
    "build_static_tf_specs",
    "build_tf_message",
    "camera_optical_quat",
    "quat_identity",
    "quat_x_180",
    "site_local_pose",
]
