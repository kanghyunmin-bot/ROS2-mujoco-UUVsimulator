"""Ping360 polar image ROS2 message builder."""

from __future__ import annotations

from array import array
from typing import Any

import numpy as np

from .ping360_image_renderer import Ping360ImageRenderer
from .ping360_types import Ping360Config, Ping360Sample


def build_ping360_image_msg(
    image_type: type,
    stamp: Any,
    sample: Ping360Sample,
    config: Ping360Config,
    renderer: Ping360ImageRenderer,
) -> Any:
    rendered = renderer.render_polar_image(sample, config)
    msg = image_type()
    msg.header.stamp = stamp
    msg.header.frame_id = config.frame_id
    msg.height = int(rendered.shape[0])
    msg.width = int(rendered.shape[1])
    msg.encoding = "mono8"
    msg.is_bigendian = 0
    msg.step = int(rendered.shape[1])
    msg.data = array("B", rendered.astype(np.uint8, copy=False).tobytes())
    return msg


__all__ = ["build_ping360_image_msg"]
