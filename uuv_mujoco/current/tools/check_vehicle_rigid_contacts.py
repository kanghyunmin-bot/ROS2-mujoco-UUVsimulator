"""Check whole-vehicle rigid contacts with an isolated buoy load fixture."""

import sys
import copy
import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco
import numpy as np

CURRENT = Path(__file__).resolve().parents[1]


def main():
    model = mujoco.MjModel.from_xml_path(
        str(CURRENT / "scenes/research_pool_slam_scene.xml")
    )
    names = [
        f"thruster_collision_{kind}_{side}"
        for kind in ("ver", "yaw")
        for side in ("lf", "lr", "rf", "rr")
    ]
    names += ["cad_collision_339_0", "cad_collision_308_0"]
    # Only the selected surface and float participate in each test. This avoids
    # neighbouring frame parts intercepting the probe before a thruster does.
    probe = model.geom("course_buoy_a_yellow_1_float_geom").id
    q = int(model.jnt_qposadr[model.joint("course_buoy_a_yellow_1_free").id])
    v = int(model.jnt_dofadr[model.joint("course_buoy_a_yellow_1_free").id])
    buoy = model.geom_bodyid[probe]
    model.opt.gravity[:] = 0
    model.opt.density = model.opt.viscosity = 0
    model.opt.timestep = 0.002
    model.geom_contype[:] = model.geom_conaffinity[:] = 0
    model.geom_contype[probe] = model.geom_conaffinity[probe] = 1
    for index in range(model.ngeom):
        name = model.geom(index).name or ""
        if name.startswith(("cad_collision_", "thruster_collision_")):
            rake = name.startswith(("cad_collision_92_", "cad_collision_93_"))
            np.testing.assert_allclose(
                model.geom_solref[index], [0.002 if rake else 0.004, 1]
            )
            assert model.geom_priority[index] >= 2
    maximum = 0
    for name in names:
        target = model.geom(name).id
        model.geom_contype[target] = model.geom_conaffinity[target] = 1
        d = mujoco.MjData(model)
        d.eq_active[:] = (
            0  # The already-released probe is not tethered to its old position.
        )
        mujoco.mj_forward(model, d)
        center = d.geom_xpos[target].copy()
        robot = d.qpos[:7].copy()
        axis = np.array([1.0, 0.0, 0.0])
        # Find a 0.5 mm clearance at the actual compiled convex surface.
        low, high = 0.0, 0.6
        for _ in range(30):
            offset = (low + high) / 2
            d.qpos[q : q + 3] += center + axis * offset - d.geom_xpos[probe]
            mujoco.mj_forward(model, d)
            distance = mujoco.mj_geomDistance(model, d, target, probe, 1.0, None)
            if distance > 0.0005:
                high = offset
            else:
                low = offset
        closest = np.zeros(6)
        mujoco.mj_geomDistance(model, d, target, probe, 1.0, closest)
        normal = closest[3:] - closest[:3]
        normal /= np.linalg.norm(normal)
        source = ET.parse(CURRENT / "scenes/research_pool_slam_scene.xml").getroot()
        target_geom = copy.deepcopy(source.find(f'.//geom[@name="{name}"]'))
        mesh_name = target_geom.get("mesh")
        mesh = copy.deepcopy(source.find(f'asset/mesh[@name="{mesh_name}"]'))
        mesh.set(
            "file", str(CURRENT / "assets/urdf_full/meshes_split" / mesh.get("file"))
        )
        target_geom.attrib.pop("material", None)
        fixture = ET.Element("mujoco")
        ET.SubElement(fixture, "option", gravity="0 0 0", timestep=".002")
        ET.SubElement(fixture, "asset").append(mesh)
        world = ET.SubElement(fixture, "worldbody")
        fmt = lambda values: " ".join(str(float(x)) for x in values)
        body = ET.SubElement(
            world,
            "body",
            pos=fmt(d.xpos[model.body("base_link").id]),
            quat=fmt(d.xquat[model.body("base_link").id]),
        )
        body.append(target_geom)
        moving = ET.SubElement(world, "body", pos=fmt(d.geom_xpos[probe]))
        ET.SubElement(moving, "joint", type="slide", axis=fmt(normal))
        ET.SubElement(
            moving,
            "inertial",
            pos="0 0 0",
            mass=".01",
            diaginertia=".000025 .000025 .00002",
        )
        probe_geom = ET.SubElement(
            moving,
            "geom",
            name="load_probe",
            type="ellipsoid",
            size=".055 .055 .085",
            friction=".3 .01 .001",
            priority="2",
            solref=".004 1",
            solimp=".999 .9999 .0005 .5 2",
        )
        if "--without_fix" in sys.argv:
            target_geom.set("priority", "0")
            target_geom.set("solref", ".02 1")
            target_geom.set("solimp", ".9 .95 .001 .5 2")
            probe_geom.set("priority", "0")
            probe_geom.set("solref", ".06 1")
            probe_geom.set("solimp", ".82 .95 .003 .5 2")
            probe_geom.set("solmix", ".25")
        fm = mujoco.MjModel.from_xml_string(ET.tostring(fixture, encoding="unicode"))
        fd = mujoco.MjData(fm)
        mujoco.mj_forward(fm, fd)
        penetration = 0
        final_force = 0
        for step in range(400):
            fd.qfrc_applied[0] = -(0.2 + 14.8 * min(1, max(0, (step - 100) / 100)))
            mujoco.mj_step(fm, fd)
            force = np.zeros(6)
            final_force = 0
            for ci, contact in enumerate(fd.contact):
                penetration = max(penetration, -contact.dist)
                mujoco.mj_contactForce(fm, fd, ci, force)
                final_force += force[0]
            assert penetration < 0.0015, ("soft penetration", name, step, penetration)
        assert final_force > 10, ("no support against 15 N load", name, final_force)
        maximum = max(maximum, penetration)
        model.geom_contype[target] = model.geom_conaffinity[target] = 0
    print(
        f"PASS 8 thrusters + body panel + frame hold 15 N; max penetration {maximum * 1000:.3f} mm"
    )


if __name__ == "__main__":
    main()
