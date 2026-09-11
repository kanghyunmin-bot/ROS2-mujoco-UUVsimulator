"""Low-cost bag-inspired liner; room dimensions are illustrative, not surveyed."""

import xml.etree.ElementTree as ET


def apply_tank_appearance(root: ET.Element, *, half_x: float, half_y: float, depth: float) -> None:
    """Add visual-only room geometry, with dimensions in metres."""
    asset = root.find("asset")
    world = root.find("worldbody")
    if asset is None or world is None:
        raise ValueError("Tank requires asset and worldbody")
    # Native 32px texture: no external bitmap, per-tile meshes, or new lights.
    ET.SubElement(asset, "texture", name="bag_liner_tex", type="2d", builtin="flat",
                  width="32", height="32", rgb1="0.035 0.36 0.61", mark="edge",
                  markrgb="0.76 0.80 0.73")
    ET.SubElement(asset, "material", name="bag_liner", texture="bag_liner_tex",
                  texuniform="true", texrepeat="20 20", specular="0.05", shininess="0.08")
    ET.SubElement(asset, "material", name="bag_room", rgba="0.72 0.74 0.73 1")
    ET.SubElement(asset, "material", name="bag_light_panel", rgba="0.94 0.95 0.87 1", emission="0.5")
    for geom in list(world.findall("geom")):
        if geom.get("name", "").startswith("pool_wall_tile_panel_"):
            world.remove(geom)
        if geom.get("name") in {"pool_floor", "pool_wall_px", "pool_wall_nx", "pool_wall_py", "pool_wall_ny"}:
            geom.set("material", "bag_liner")
            geom.set("rgba", "1 1 1 1")

    def box(name, pos, size, material="bag_room"):
        ET.SubElement(world, "geom", name="bag_room_" + name, type="box",
                      pos=" ".join(map(str, pos)), size=" ".join(map(str, size)),
                      material=material, contype="0", conaffinity="0", group="2")

    # Explicit plane orientation avoids box side UV stretching.
    for name, pos, size, quat in (
        ("liner_y_neg", (0, -half_y + .001, -depth / 2), (half_x, depth / 2, .1), "0.70710678 -0.70710678 0 0"),
        ("liner_y_pos", (0, half_y - .001, -depth / 2), (half_x, depth / 2, .1), "0.70710678 0.70710678 0 0"),
        ("liner_x_neg", (-half_x + .001, 0, -depth / 2), (depth / 2, half_y, .1), "0.70710678 0 0.70710678 0"),
        ("liner_x_pos", (half_x - .001, 0, -depth / 2), (depth / 2, half_y, .1), "0.70710678 0 -0.70710678 0"),
    ):
        ET.SubElement(world, "geom", name="bag_" + name, type="plane",
                      pos=" ".join(map(str, pos)), size=" ".join(map(str, size)), quat=quat,
                      material="bag_liner", contype="0", conaffinity="0", group="0")
    for light in world.findall("light"):
        light.set("castshadow", "false")
    if not any(t.get("type") == "skybox" for t in asset.findall("texture")):
        ET.SubElement(asset, "texture", name="bag_room_background", type="skybox",
                      builtin="gradient", rgb1="0.36 0.39 0.40", rgb2="0.18 0.22 0.24",
                      width="32", height="192")

    # Roof is downward-facing so the overview can look through its back face.
    ET.SubElement(world, "geom", name="bag_room_ceiling", type="plane", pos="0 0 3",
                  quat="0 1 0 0", size=f"{half_x + 1} {half_y + 1} 0.5",
                  material="bag_room", contype="0", conaffinity="0", group="2")
    for i, x in enumerate((-half_x, 0, half_x)):
        box(f"beam_{i}", (x, 0, 2.85), (0.045, half_y + 1, 0.12))
        box(f"light_{i}", (x + 0.3, 0, 2.83), (0.10, 0.6, 0.015), "bag_light_panel")
    for i, y in enumerate((-half_y - 0.12, half_y + 0.12)):
        box(f"rim_y_{i}", (0, y, 0.1), (half_x + 0.24, 0.12, 0.025))
    for i, x in enumerate((-half_x - 0.12, half_x + 0.12)):
        box(f"rim_x_{i}", (x, 0, 0.1), (0.12, half_y, 0.025))
