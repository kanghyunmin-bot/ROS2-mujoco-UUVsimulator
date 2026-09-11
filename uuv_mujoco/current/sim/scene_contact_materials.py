"""Shared rigid contact settings for the CAD vehicle and solid buoy parts.

Friction values are initial wet-surface assumptions, not measured material data.
Visual meshes and articulated rope links do not use this helper.
"""

import xml.etree.ElementTree as ET


def apply_rigid_contact(geom: ET.Element, *, rake: bool = False) -> None:
    """Apply firm normal contact and sliding friction to one solid collision geom."""
    geom.set("priority", "3" if rake else "2")

    # Resolve thin rake contact within two 1 ms pool steps. MuJoCo keeps its
    # reference-safety clamp for scenes that use a larger physics step.
    geom.set("solref", ".002 1" if rake else ".004 1")
    geom.set("solimp", ".999 .9999 .0005 .5 2")
    geom.set("solmix", "1")
    geom.set("condim", "3")
    geom.set("friction", ".35 .01 .001" if rake else ".30 .01 .001")
