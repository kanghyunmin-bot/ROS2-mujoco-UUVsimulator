# 2026-09 body visual replacement

Source: user-supplied `26_full_asem_auv_2.STL.stl` (4,804,826 triangles).
The original download is not modified or copied into the repository.
`manifest.json` records its SHA-256, CAD connected-component selection and registration.

## Registration and filtering

The STL uses millimetres. Convert by 0.001, then apply the rotation and translation
in the manifest. The axes map CAD -Z to vehicle +X, CAD -X to vehicle +Y, and CAD +Y
to vehicle +Z. Four vertical T200 main-housing bounding-box centres determine the
translation against the existing thruster meshes: centre residuals are 0.00585 mm.
This validates mount registration, not a metrology claim about every surface.

All retained CAD triangles and small components are preserved without decimation,
vertex clipping, or a minimum feature-size cutoff. Shared vertices are indexed in OBJ
instead of repeated for every STL triangle. The 2,891,279 visual triangles are split
into 20 OBJ files (117,787,032 bytes total). This reduces storage relative to binary STL
for the same triangles; it does not reduce the rendering triangle count.

Components smaller than 155 mm and within 80 mm of the eight T200 main-housing centres
are excluded as propulsion-local parts; the original simulator thrusters remain.
This geometric filter is approximate because STL has no assembly labels, and can also
exclude small nearby mounting details. The manifest records the retained and excluded IDs.
Offline conversion uses trimesh; no new simulation dependency is required.

## Rigid collision model

Both research_pool_slam_scene.xml and tank_current_scene.xml now use the CAD-derived
compound colliders in `collision/`. The old five broad collision geoms are retired,
so they cannot block spaces between the fingers or the frame. The detailed visual
meshes remain unchanged and non-colliding.

The collision model contains 736 convex pieces (156,748 triangles). Two CAD hands,
four lower shells and enclosure brackets use CoACD 1.0.14; other structural components
use individual convex hulls. CoACD targets 2 mm for hands and 5 mm for the other concave
parts, with a 48-piece cap on each non-hand component and 64 vertices per piece.
These are approximation settings, not a verified maximum surface-error guarantee.
Small features under 20 mm maximum extent, sheets under 1 mm thickness, and propulsion
components already represented by existing thrusters are not added as body colliders.
Small CAD fasteners remain visible. See `collision/manifest.json` for every source part.

All new geoms belong to base_link, have density=0, and enable collision with walls and
buoys. Sliding friction is 0.30 for general solids and 0.35 for the rakes. Contacts use
condim=3, so torsional/rolling coefficients are inactive. Mass, inertia,
thrusters, cameras and fluid ellipsoid proxies remain unchanged. The hands are fixed
rigid parts of the vehicle; finger actuation, compliance and grasp control are not added.

MuJoCo uses convex mesh collisions, so concave shapes need compound pieces:
https://mujoco.readthedocs.io/en/latest/computation/#collision-detection

## Rebuilding

`tools/build_robot_collision_meshes.py` accepts the CAD import's component PLY cache,
its registration.json and components.json, a decomposition cache and an output directory.
Use `--rebuild_decomposition` to rerun CoACD. This is offline asset processing requiring
NumPy, trimesh and CoACD; these are not new runtime dependencies.

## Verification

`tools/check_robot_cad_collision.py` checks all ten fingertips, eight intervening gaps
with a 6 mm diameter probe, positive hand/wall reaction force, and physical hand/buoy
contact that respects the pool's 15 N force-based magnetic connection. `--without_fix` disables hand collisions and
fails the fingertip check. The existing research-pool release/rise check also passes.
Both scenes compile with unchanged mass/inertia, cameras, thrusters and fluid proxies.

A local unrendered 2,000-step check remained finite: 4 seconds of simulation took
0.052 seconds with the new colliders versus 0.028 seconds before. This is a basic idle
physics check, not a full GUI, camera or crowded-contact performance benchmark.

The hand colliders use priority 2 and a firmer contact response (`solref=.004 1`,
`solimp=.99 .999 .001 .5 2`) so light buoy jigs cannot be pressed through thin tines.
`tools/check_rake_jig_retention.py` verifies jig retention under load and slow
transport with a freely moving buoy. No artificial attachment to the hand is added.


Whole-vehicle rigid contact update: all 736 CAD body colliders and eight existing
thruster assembly colliders now use firm contact, as do solid buoy components.
Parameters are shared in `sim/scene_contact_materials.py`: solref=.004 1,
solimp=.999 .9999 .0005 .5 2; general priority 2 and rake priority 3. The eight
animated propeller visuals remain covered by the existing assembly collision
hulls, avoiding duplicate internal collision surfaces. Blade-level rope ingestion
is not modeled. Visual geometry and vehicle inertial/fluid settings are unchanged.

Upper/lower jig follow-up: rake contact now uses `solref=.002 1`; other solids
retain `.004 1`. Research-pool physics and the GUI timestep cap are 1 ms.
This resolves transient jig penetration without changing collision geometry.
See `docs/gui/BUOY_MAGNET_AND_ROPE.md` and `tools/check_buoy_jig_contacts.py`.

Optical-window follow-up: component 339 was extracted from visual chunk 16
into `front_optical_window.obj` and made visually transparent. Original triangles
and the solid `cad_collision_339_0` collider are preserved. Cameras are inside
the front panel; see `docs/gui/CAD_SENSOR_MOUNTS.md` for CAD/TF alignment.
