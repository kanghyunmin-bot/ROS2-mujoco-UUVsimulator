"""Check finger gaps, physical wall response, and CAD-hand buoy release."""

import sys
from pathlib import Path
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

CURRENT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(CURRENT))
from tools.check_buoy_physics_contract import runtime_for


def main():
    root = ET.parse(CURRENT / "scenes/research_pool_slam_scene.xml").getroot()
    root.find("compiler").set("meshdir", str(CURRENT / "assets/urdf_full/meshes_split"))
    for texture in root.findall("asset/texture"):
        if texture.get("file"):
            texture.set(
                "file", str((CURRENT / "scenes" / texture.get("file")).resolve())
            )
    probe = ET.SubElement(
        root.find("worldbody"), "body", name="contact_probe", mocap="true", pos="0 0 10"
    )
    ET.SubElement(probe, "geom", name="contact_probe_geom", type="sphere", size=".003")
    model = mujoco.MjModel.from_xml_string(ET.tostring(root, encoding="unicode"))
    data = mujoco.MjData(model)
    hand = {
        i
        for i in range(model.ngeom)
        if (model.geom(i).name or "").startswith(
            ("cad_collision_92_", "cad_collision_93_")
        )
    }
    if "--without_fix" in sys.argv:
        for i in hand:
            model.geom_contype[i] = model.geom_conaffinity[i] = 0
    q = int(model.jnt_qposadr[model.joint("world_joint").id])
    v = int(model.jnt_dofadr[model.joint("world_joint").id])
    origin = data.qpos[q : q + 7].copy()
    probe_id = model.geom("contact_probe_geom").id

    def contact_with(other):
        return [
            i
            for i in range(data.ncon)
            if (
                (data.contact[i].geom1 in hand and data.contact[i].geom2 == other)
                or (data.contact[i].geom2 in hand and data.contact[i].geom1 == other)
            )
        ]

    # Sample both hands at the forward fingers and in the four intervening gaps.
    for sign in (1, -1):
        hits = []
        for y in (0.074, 0.116, 0.162, 0.207, 0.253):
            data.mocap_pos[0] = origin[:3] + [0.405, sign * y, -0.094]
            mujoco.mj_forward(model, data)
            hits.append(bool(contact_with(probe_id)))
        assert all(hits), ("finger surface missed", sign, hits)
        for y in (0.095, 0.14, 0.185, 0.23):
            data.mocap_pos[0] = origin[:3] + [0.405, sign * y, -0.094]
            mujoco.mj_forward(model, data)
            assert not contact_with(probe_id), ("finger gap filled", sign, y)
    print("PASS both hands: five fingers collide, four gaps remain open")
    data.mocap_pos[0] = [0, 0, 10]
    data.qpos[q : q + 3] = [5 - 0.4173 + 0.002, 0, -2]
    data.qvel[v : v + 6] = 0
    data.qvel[v] = 0.1
    mujoco.mj_forward(model, data)
    contacts = contact_with(model.geom("pool_wall_px").id)
    assert contacts, "hand did not contact pool wall"
    force = np.zeros(6)
    loads = []
    for index in contacts:
        mujoco.mj_contactForce(model, data, index, force)
        loads.append(force[0])
    assert max(loads) > 0, "wall contact has no reaction force"
    print("PASS hand-wall contact produces a normal reaction force")
    mujoco.mj_resetData(model, data)
    mujoco.mj_forward(model, data)
    runtime = runtime_for(mujoco, model, data)
    buoy = runtime.buoys[0]
    target = data.geom_xpos[model.geom("course_buoy_a_yellow_1_float_geom").id].copy()
    data.qpos[q : q + 3] = target - [0.465, 0.074, -0.094]
    mujoco.mj_forward(model, data)
    assert contact_with(model.geom("course_buoy_a_yellow_1_float_geom").id), (
        "hand missed buoy"
    )
    runtime.apply(model.opt.timestep)
    assert not buoy.detached, "first contact bypassed 15 N magnetic load threshold"
    print("PASS CAD hand physically contacts buoy; touching alone does not release")


if __name__ == "__main__":
    main()
