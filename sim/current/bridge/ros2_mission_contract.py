"""ROS mission-to-simulator contracts that authorize physical events."""

from __future__ import annotations

import json
from typing import Any


SCORE_RELEASE_TOPIC = "/mission/score_release"


def on_score_release_contract(self: Any, msg: Any) -> None:
    state = ""
    zone_xyz = None
    try:
        payload = json.loads(str(msg.data))
        if not isinstance(payload, dict):
            raise ValueError("payload must be an object")
        state = str(payload.get("state") or "")
        zone_payload = payload.get("score_zone")
        zone_xyz = zone_payload.get("xyz") if isinstance(zone_payload, dict) else None
    except Exception as exc:
        self.node.get_logger().warn(f"invalid {SCORE_RELEASE_TOPIC} payload: {exc}")

    contract = {"state": state, "score_zone_xyz": zone_xyz}
    self._score_release_contract = contract
    runtime = getattr(self, "_course_buoy_runtime", None)
    setter = getattr(runtime, "set_score_release_contract", None)
    if setter is not None:
        setter(state, zone_xyz)


def apply_pending_score_release_contract(bridge: Any, runtime: Any) -> None:
    contract = getattr(bridge, "_score_release_contract", None)
    setter = getattr(runtime, "set_score_release_contract", None)
    if not isinstance(contract, dict) or setter is None:
        return
    setter(contract.get("state", ""), contract.get("score_zone_xyz"))


__all__ = [
    "SCORE_RELEASE_TOPIC",
    "apply_pending_score_release_contract",
    "on_score_release_contract",
]
