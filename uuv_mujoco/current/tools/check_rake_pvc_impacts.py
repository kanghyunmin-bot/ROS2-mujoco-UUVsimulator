"""Free six-DOF oblique PVC impacts against the authored CAD rake surfaces.

This isolates contact integration, not magnetic release or full buoy fluid response.
The probe retains the float's authored mass/inertia and PVC collider. A 20 N,
1 ms pulse starts after first contact; it is an impact stress, not a magnet model.
"""

import copy
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

CURRENT = Path(__file__).resolve().parents[1]
HANDS = ("cad_collision_92_", "cad_collision_93_")


def main():
    source = ET.parse(CURRENT / "scenes/research_pool_slam_scene.xml").getroot()
    root = ET.Element("mujoco")
    timestep = source.find("option").get("timestep")
    old_contact = "--without_fix" in sys.argv
    ET.SubElement(root, "option", gravity="0 0 0", timestep=".001" if old_contact else timestep,
                  integrator="implicit")
    ET.SubElement(root, "compiler", meshdir=str(CURRENT / "assets/urdf_full/meshes_split"))
    assets = ET.SubElement(root, "asset")
    world = ET.SubElement(root, "worldbody")
    for asset in source.findall("asset/mesh"):
        if asset.get("name", "").startswith(HANDS):
            assets.append(copy.deepcopy(asset))
    for geom in source.findall(".//geom"):
        if geom.get("name", "").startswith(HANDS):
            collider = copy.deepcopy(geom)
            if old_contact:
                collider.set("solref", ".002 1")
            world.append(collider)
    body = ET.SubElement(world, "body", name="probe")
    ET.SubElement(body, "freejoint")
    body.append(copy.deepcopy(source.find('.//body[@name="course_buoy_a_yellow_1_float"]/inertial')))
    pvc = copy.deepcopy(source.find('.//geom[@name="course_buoy_a_yellow_1_pvc_pipe"]'))
    pvc.set("name", "pvc")
    body.append(pvc)
    model = mujoco.MjModel.from_xml_string(ET.tostring(root, encoding="unicode"))
    data = mujoco.MjData(model)
    probe_id = model.body("probe").id
    pvc_id = model.geom("pvc").id
    peaks = []
    for side in (-1, 1):
        for target, xyz in (("finger", [.38, side * .116, -.094]), ("root", [.308, side * .14, -.094])):
            for direction in (-1, 1):
                for angle in (60, 90):
                    quat = [np.cos(np.deg2rad(angle) / 2), np.sin(np.deg2rad(angle) / 2), 0, 0]
                    rotation = np.empty(9)
                    mujoco.mju_quat2Mat(rotation, np.asarray(quat))
                    for speed in (.5, 1.):
                        for load in (0., 20.):
                            mujoco.mj_resetData(model, data)
                            data.qpos[:3] = np.asarray(xyz) + [0, 0, direction * .1] - rotation.reshape(3, 3) @ model.geom_pos[pvc_id]
                            data.qpos[3:] = quat
                            data.qvel[2] = -direction * speed
                            mujoco.mj_forward(model, data)
                            assert not data.ncon, "fixture starts in collision"
                            first_contact = None
                            penetration = 0.
                            for _ in range(round(.6 / model.opt.timestep)):
                                pulse = first_contact is not None and data.time - first_contact < .001 - 1e-12
                                data.xfrc_applied[probe_id, 2] = -direction * load if pulse else 0.
                                mujoco.mj_step(model, data)
                                if data.ncon:
                                    if first_contact is None:
                                        first_contact = float(data.time)
                                    penetration = max(penetration, max(-c.dist for c in data.contact))
                                assert not any(w.number for w in data.warning)
                                assert np.isfinite(data.qpos).all() and np.isfinite(data.qvel).all()
                            assert first_contact is not None, ("missed solid", side, target, direction, angle)
                            assert penetration < .0025, ("excessive PVC penetration", mujoco.__version__, side, target, direction, angle, speed, load, penetration)
                            peaks.append(penetration)
    print(f"PASS MuJoCo {mujoco.__version__}: {len(peaks)} oblique PVC impacts; peak {max(peaks) * 1000:.3f} mm")


if __name__ == "__main__":
    main()
