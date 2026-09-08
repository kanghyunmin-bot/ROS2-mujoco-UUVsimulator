#!/usr/bin/env python3
"""Read-only STL/scene geometry audit. Mesh volume is not displacement calibration."""
from pathlib import Path
import argparse
import json
import sys
import xml.etree.ElementTree as ET
import numpy as np
import trimesh
ROOT=Path(__file__).resolve().parents[1]


def quaternion_rotation(text):
    q=np.fromstring(text or '1 0 0 0',sep=' ');q=q/np.linalg.norm(q)
    w,x,y,z=q
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                     [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                     [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])


def main():
    p=argparse.ArgumentParser();p.add_argument('--out',type=Path,required=True);args=p.parse_args()
    scene=ROOT/'scenes/research_pool_slam_scene.xml';xml=ET.parse(scene).getroot()
    meshdir=scene.parent/xml.find('compiler').get('meshdir')
    assets={m.get('name'):m for m in xml.findall('asset/mesh')}
    body=xml.find("worldbody/body[@name='base_link']")
    meshes=[];details=[];base=[]
    for geom in body.findall('geom'):
        name=geom.get('mesh')
        if name is None:continue
        asset=assets[name];mesh=trimesh.load_mesh(meshdir/asset.get('file'),process=False)
        scale=np.fromstring(asset.get('scale','1 1 1'),sep=' ')
        mesh.vertices=(mesh.vertices*scale)@quaternion_rotation(geom.get('quat')).T+np.fromstring(geom.get('pos','0 0 0'),sep=' ')
        meshes.append(mesh)
        if name.startswith('base_link_part'):base.append(mesh)
        details.append(dict(mesh=name,triangles=len(mesh.faces),bounds_body_m=mesh.bounds.tolist(),dimensions_m=mesh.extents.tolist()))
    merged=trimesh.util.concatenate(base);merged.merge_vertices(digits_vertex=6)
    components=merged.split(only_watertight=False, repair=False)
    parts=[]
    for component in sorted(components,key=lambda m:len(m.faces),reverse=True):
        if len(component.faces)<20:continue
        parts.append(dict(triangles=len(component.faces),watertight=bool(component.is_watertight),
            winding_consistent=bool(component.is_winding_consistent),bounds_body_m=component.bounds.tolist(),
            dimensions_m=component.extents.tolist(),
            signed_mesh_volume_l=float(component.volume*1000) if component.is_watertight else None))
    all_bounds=np.array([m.bounds for m in meshes]);bounds=np.array([all_bounds[:,0].min(axis=0),all_bounds[:,1].max(axis=0)])
    components_config=json.loads((ROOT/'config/sim_profiles.json').read_text())['current']['body_components']
    buoyancy_z={c['name']:c['buoyancy_pos'][2] for c in components_config}
    side_regions={}
    for label,sign in (('port',1),('starboard',-1)):
        vertices=merged.vertices[sign*merged.vertices[:,1]>.20]
        side_regions[label]=dict(selection='base meshes with signed body y > 0.20m; geometric crop, not semantic CAD grouping',
            bounds_body_m=[vertices.min(axis=0).tolist(),vertices.max(axis=0).tolist()],
            configured_component_buoyancy_z_m=buoyancy_z[label+'_lower_body'],
            buoyancy_above_region_top_m=float(buoyancy_z[label+'_lower_body']-vertices[:,2].max()))
    import mujoco
    model=mujoco.MjModel.from_xml_path(str(scene));data=mujoco.MjData(model);mujoco.mj_forward(model,data)
    bid=mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_BODY,'base_link');compiled=[]
    for i in range(model.ngeom):
        if model.geom_bodyid[i]!=bid or model.geom_type[i]!=mujoco.mjtGeom.mjGEOM_MESH:continue
        mid=model.geom_dataid[i];a=model.mesh_vertadr[mid];n=model.mesh_vertnum[mid]
        vertices=model.mesh_vert[a:a+n]@data.geom_xmat[i].reshape(3,3).T+data.geom_xpos[i]
        compiled.append((vertices-data.xpos[bid])@data.xmat[bid].reshape(3,3))
    compiled=np.concatenate(compiled)
    error=float(np.max(np.abs(np.array([compiled.min(axis=0),compiled.max(axis=0)])-bounds)))
    if error>1e-5:raise AssertionError('STL transform disagrees with compiled MuJoCo geometry')
    report=dict(compiled_mujoco_bounds_max_error_m=error,side_regions=side_regions,scene=str(scene),scope='direct base_link mesh geoms; animated propeller child meshes excluded',
        visual_bounds_body_m=bounds.tolist(),visual_dimensions_m=(bounds[1]-bounds[0]).tolist(),
        vertex_weld_rounding_m=1e-6, base_mesh_connected_components=len(components),mesh_details=details,base_components=parts,
        warnings=['closed STL components can describe solid material or nested surfaces, not necessarily sealed water displacement',
                  'mass, contents, material density and floodable cavities cannot be established from these STL files',
                  'no geometry, mass or hydrodynamic coefficients changed'])
    args.out.parent.mkdir(parents=True,exist_ok=True);args.out.write_text(json.dumps(report,indent=2))
    print(json.dumps({k:report[k] for k in ('visual_bounds_body_m','visual_dimensions_m','base_mesh_connected_components')},indent=2))
    print('Largest components:',json.dumps(parts[:12],indent=2))

if __name__=='__main__':main()
