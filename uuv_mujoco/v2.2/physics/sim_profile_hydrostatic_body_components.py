"""Body-component parser for simulation profiles."""

from __future__ import annotations

from typing import Any, Mapping

from .sim_profile_hydrostatic_component_builder import body_component_from_profile_item
from .sim_profile_types import BodyComponent


def parse_body_components(sim_profile: Mapping[str, Any]) -> tuple[BodyComponent, ...]:
    payload = sim_profile.get("body_components")
    if not isinstance(payload, list):
        return ()

    components: list[BodyComponent] = []
    for idx, item in enumerate(payload):
        component = body_component_from_profile_item(idx, item)
        if component is not None:
            components.append(component)
    return tuple(components)


__all__ = ["parse_body_components"]
