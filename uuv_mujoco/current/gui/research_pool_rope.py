"""Generate the pool's anchored, articulated mooring ropes in metres."""

import xml.etree.ElementTree as ET

from sim.scene_contact_materials import apply_rigid_contact


def build_mooring_rope(root: ET.Element, prefix: str, length_m: float) -> None:
    """Replace the lower mooring with a articulated rope of length ``length_m`` [m]."""
    if length_m <= 0.1:
        raise ValueError("Mooring rope must be longer than 0.1 m")
    base = root.find(f'.//body[@name="{prefix}_magnet_base"]')
    for child in list(base):
        if child.get("name", "").endswith(
            (
                "_mooring_rod",
                "_rod_top_jig",
                "_fixed_magnet_stem",
                "_fixed_magnet",
                "_magnet_site",
                "_rope_00",
            )
        ):
            base.remove(child)
    # Six links retain bending and solid capsule contacts at low solver cost.
    # Numerical rotational inertia floor [kg m^2], not a measured rope inertia.
    # 1e-6 reproduced folded-rope divergence; 1e-5 retains sag/contact.
    count = 6
    step = length_m / count
    parent = base
    for index in range(count):
        link = ET.SubElement(
            parent,
            "body",
            name=f"{prefix}_rope_{index:02}",
            pos=f"0 0 {0.109 if index == 0 else step:.9f}",
            gravcomp=str(1000 / 1150),
        )
        ET.SubElement(
            link,
            "joint",
            name=f"{prefix}_rope_joint_{index:02}",
            type="ball",
            damping="0.0003",
            armature="0.00001",
        )
        ET.SubElement(
            link,
            "geom",
            name=f"{prefix}_rope_geom_{index:02}",
            type="capsule",
            fromto=f"0 0 0 0 0 {step - 0.035 if index == count - 1 else step:.9f}",
            size=".003",
            density="1150",
            rgba=".75 .78 .72 1",
            contype="4",
            conaffinity="7",
            condim="3",
            friction=".5 .005 .0001",
            solref=".01 1",
            fluidshape="ellipsoid",
            fluidcoef="0 0 0 0 0",
        )
        parent = link
    tip = ET.SubElement(
        parent,
        "body",
        name=f"{prefix}_rope_magnet_tip",
        pos=f"0 0 {step:.9f}",
        gravcomp=".2",
    )
    ET.SubElement(
        tip,
        "geom",
        name=f"{prefix}_rod_top_jig",
        type="cylinder",
        pos="0 0 -.020",
        size=".028 .010",
        mass=".008",
        rgba=".95 .95 .92 1",
        contype="4",
        conaffinity="7",
    )
    ET.SubElement(
        tip,
        "geom",
        name=f"{prefix}_fixed_magnet",
        type="cylinder",
        pos="0 0 -.010",
        size=".018 .010",
        mass=".004",
        rgba=".04 .04 .04 1",
        contype="4",
        conaffinity="7",
    )
    for solid in tip.findall("geom"):
        apply_rigid_contact(solid)
    ET.SubElement(
        tip,
        "site",
        name=f"{prefix}_magnet_site",
        pos="0 0 0",
        size=".008",
        rgba="0 0 0 0",
    )
    # The attached rope endpoint lies inside its own anchor eye. Exclude only
    # this connected pair; the rest of the rope still contacts the anchor/floor.
    contact = root.find("contact")
    if contact is None:
        contact = ET.SubElement(root, "contact")
    if (
        contact.find(
            f'exclude[@body1="{prefix}_magnet_base"][@body2="{prefix}_rope_00"]'
        )
        is None
    ):
        ET.SubElement(
            contact, "exclude", body1=f"{prefix}_magnet_base", body2=f"{prefix}_rope_00"
        )
    weld = root.find(f'.//weld[@name="{prefix}_magnet_weld"]')
    weld.set("body1", f"{prefix}_rope_magnet_tip")
    weld.set("relpose", "0 0 .215 1 0 0 0")
    weld.set("solref", ".008 1")
    weld.set("solimp", ".95 .99 .0005")
