"""Real robot RC axis/channel contract."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class RcAxisContract:
    name: str
    channel: int
    description: str


RC_AXIS_CONTRACTS: tuple[RcAxisContract, ...] = (
    RcAxisContract("heave", 3, "vertical command"),
    RcAxisContract("yaw", 4, "yaw command"),
    RcAxisContract("forward", 5, "surge command"),
    RcAxisContract("lateral", 6, "sway command"),
)

RC_AXIS_BY_NAME = {axis.name: axis for axis in RC_AXIS_CONTRACTS}
RC_AXIS_BY_CHANNEL = {axis.channel: axis for axis in RC_AXIS_CONTRACTS}


__all__ = ["RcAxisContract", "RC_AXIS_CONTRACTS", "RC_AXIS_BY_NAME", "RC_AXIS_BY_CHANNEL"]
