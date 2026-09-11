"""Check jig/magnet faces and transient loads against the actual CAD rakes.

Face fixtures isolate each solid from adjacent parts and use a 3 mm radius probe.
Rake fixtures guide the released buoy laterally and hold its attitude while leaving
vertical motion free. These are contact tests, not a full vehicle controller test.
"""

import copy
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

CURRENT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(CURRENT))
from sim.runtime.model_runtime_setup import _apply_timestep_override
from tools.check_buoy_physics_contract import runtime_for

SCENE = CURRENT / "scenes/research_pool_slam_scene.xml"
FITTINGS = (
    "top_socket",
    "top_stem",
    "top_bolt",
    "top_jig",
    "bottom_socket",
    "pvc_pipe",
    "moving_lower_jig",
    "magnet_lower_plate",
    "magnet",
    "rod_top_jig",
    "fixed_magnet",
    "bottom_jig",
    "weight_eye",
)


def check_faces(source, model, runtime):
    """Load all six axial faces of each fitting with a 15 N probe."""
    peak = 0.0
    cases = 0
    for buoy in runtime.buoys:
        for suffix in FITTINGS:
            name = f"{buoy.name}_{suffix}"
            gid = model.geom(name).id
            # Check the live masks, after the runtime has changed gate bits.
            assert (model.geom_contype[gid] & 1) or (model.geom_conaffinity[gid] & 1), (
                name
            )
            original = source.find(f'.//geom[@name="{name}"]')
            for axis in range(3):
                for sign in (-1, 1):
                    root = ET.Element("mujoco")
                    ET.SubElement(
                        root,
                        "option",
                        timestep=str(model.opt.timestep),
                        gravity="0 0 0",
                    )
                    world = ET.SubElement(root, "worldbody")
                    target = copy.deepcopy(original)
                    target.set("pos", "0 0 0")
                    target.set("contype", str(model.geom_contype[gid]))
                    target.set("conaffinity", str(model.geom_conaffinity[gid]))
                    target.attrib.pop("material", None)
                    if "fromto" in target.attrib:
                        ends = np.fromstring(
                            target.attrib.pop("fromto"), sep=" "
                        ).reshape(2, 3)
                        np.testing.assert_allclose(ends[:, :2], 0)
                        half_length = np.linalg.norm(ends[1] - ends[0]) / 2
                        target.set("size", f"{target.get('size')} {half_length}")
                    world.append(target)
                    assert target.get("type") == "cylinder", name
                    size = np.fromstring(target.get("size"), sep=" ")
                    extent = size[0 if axis < 2 else 1]
                    direction = np.eye(3)[axis] * sign
                    position = direction * (extent + 0.003 + 0.0005)
                    body = ET.SubElement(
                        world, "body", pos=" ".join(map(str, position))
                    )
                    ET.SubElement(
                        body, "joint", type="slide", axis=" ".join(map(str, direction))
                    )
                    # The isolated probe uses the rake's contact material and
                    # robot collision category, with an explicit 10 g load mass.
                    ET.SubElement(
                        body,
                        "geom",
                        name="probe",
                        type="sphere",
                        size=".003",
                        mass=".01",
                        contype="1",
                        conaffinity="1",
                        priority="3",
                        solref=".002 1",
                        solimp=".999 .9999 .0005 .5 2",
                        friction=".35 .01 .001",
                    )
                    fm = mujoco.MjModel.from_xml_string(
                        ET.tostring(root, encoding="unicode")
                    )
                    fd = mujoco.MjData(fm)
                    penetration = 0.0
                    force = np.zeros(6)
                    support = 0.0
                    for _ in range(round(0.6 / fm.opt.timestep)):
                        fd.qfrc_applied[0] = -(0.2 if fd.time < 0.2 else 15)
                        mujoco.mj_step(fm, fd)
                        support = 0.0
                        for ci, contact in enumerate(fd.contact):
                            mujoco.mj_contactForce(fm, fd, ci, force)
                            support += force[0]
                            penetration = max(penetration, -contact.dist)
                    assert support > 14 and penetration < 0.002, (
                        name,
                        axis,
                        sign,
                        support,
                        penetration,
                    )
                    peak = max(peak, penetration)
                    cases += 1
    print(
        f"PASS {cases} jig/magnet/socket faces hold 15 N; peak {peak * 1000:.3f} mm",
        flush=True,
    )


def check_rake_transients(model):
    """Apply a 1 to 15 N step load to upper/lower jigs from both sides [N]."""
    hand = {
        i
        for i in range(model.ngeom)
        if (model.geom(i).name or "").startswith(
            ("cad_collision_92_", "cad_collision_93_")
        )
    }
    masks = model.geom_contype.copy(), model.geom_conaffinity.copy()
    peak = 0.0
    cases = 0
    for suffix in ("top_jig", "moving_lower_jig", "magnet_lower_plate", "magnet"):
        for sign in (-1, 1):
            for direction in (-1, 1):
                for x in (0.34, 0.40):
                    model.geom_contype[:], model.geom_conaffinity[:] = masks
                    data = mujoco.MjData(model)
                    mujoco.mj_forward(model, data)
                    runtime = runtime_for(mujoco, model, data)
                    buoy = runtime.buoys[0]
                    runtime._detach(buoy, reason="jig_contact_fixture", force_n=15)
                    buoy.release_time_s = -1
                    base = data.qpos[:7].copy()
                    q, v = buoy.free_qposadr, buoy.free_dofadr
                    gid = model.geom(f"{buoy.name}_{suffix}").id
                    # Adjacent sockets must not hide a missing target contact.
                    for other in buoy.geom_ids:
                        if other != gid:
                            model.geom_contype[other] = model.geom_conaffinity[
                                other
                            ] = 0
                    half_height = model.geom_size[gid, 1]
                    center = -0.094 - direction * (half_height + 0.04)
                    data.qpos[q : q + 3] = base[:3] + [
                        x,
                        sign * 0.14,
                        center - model.geom_pos[gid, 2],
                    ]
                    mujoco.mj_forward(model, data)
                    penetration = 0.0
                    hits = 0
                    for _ in range(round(1.2 / model.opt.timestep)):
                        data.qpos[:7] = base
                        data.qvel[:6] = 0
                        data.qpos[q : q + 2] = base[:2] + [x, sign * 0.14]
                        data.qpos[q + 3 : q + 7] = [1, 0, 0, 0]
                        data.qvel[v : v + 2] = 0
                        data.qvel[v + 3 : v + 6] = 0
                        runtime.apply(model.opt.timestep)
                        # Offset approximately 1 N buoyancy, then apply a step
                        # rather than just checking an already settled contact.
                        load = direction * (1 if data.time < 0.4 else 15) - 1
                        data.xfrc_applied[buoy.body_id, 2] += load
                        mujoco.mj_step(model, data)
                        data.xfrc_applied[buoy.body_id, 2] -= load
                        for contact in data.contact:
                            if (contact.geom1 in hand and contact.geom2 == gid) or (
                                contact.geom2 in hand and contact.geom1 == gid
                            ):
                                hits += 1
                                penetration = max(penetration, -contact.dist)
                        assert penetration < 0.002, (
                            suffix,
                            sign,
                            direction,
                            x,
                            penetration,
                        )
                        relative_z = data.qpos[q + 2] + model.geom_pos[gid, 2] - base[2]
                        assert direction * (relative_z + 0.094) < 0.005, (
                            "fitting crossed rake",
                            suffix,
                            sign,
                            direction,
                            x,
                            relative_z,
                        )
                    assert hits, ("no jig contact", suffix, sign, direction, x)
                    peak = max(peak, penetration)
                    cases += 1
    model.geom_contype[:], model.geom_conaffinity[:] = masks
    print(
        f"PASS {cases} upper/lower jig/magnet step loads on both rakes; peak {peak * 1000:.3f} mm",
        flush=True,
    )


def main():
    model = mujoco.MjModel.from_xml_path(str(SCENE))
    _apply_timestep_override(
        model,
        mujoco_module=mujoco,
        env_float=lambda name, default: (
            0.005 if name == "UUV_MUJOCO_TIMESTEP" else default
        ),
        env_flag=lambda name, default: default,
    )
    assert model.opt.timestep <= 0.001
    if "--without_fix" in sys.argv:
        # Restore the former timing, preserving geometry, mass, drag and load.
        model.opt.timestep = 0.002
        for gid in range(model.ngeom):
            if (model.geom(gid).name or "").startswith(
                ("cad_collision_92_", "cad_collision_93_")
            ):
                model.geom_solref[gid] = [0.004, 1]
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    runtime = runtime_for(mujoco, model, data)
    masks = model.geom_contype.copy(), model.geom_conaffinity.copy()
    for buoy in runtime.buoys:
        runtime._detach(buoy, reason="collision_mask_fixture", force_n=15)
    np.testing.assert_array_equal(model.geom_contype, masks[0])
    np.testing.assert_array_equal(model.geom_conaffinity, masks[1])
    check_faces(ET.parse(SCENE).getroot(), model, runtime)
    check_rake_transients(model)


if __name__ == "__main__":
    main()
