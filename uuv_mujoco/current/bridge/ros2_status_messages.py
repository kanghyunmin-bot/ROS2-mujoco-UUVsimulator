"""Status/string ROS2 message builders."""

from __future__ import annotations

import json
from typing import Any


def build_json_string_msg(string_type: type, payload: dict[str, Any]) -> Any:
    msg = string_type()
    msg.data = json.dumps(dict(payload), sort_keys=True)
    return msg


__all__ = ["build_json_string_msg"]
