# Current map vehicle contract

Canonical vehicle: `uuv_mujoco/current/scenes/research_pool_slam_scene.xml`.

The current course (`tank_current_scene.xml`), legacy-map compatibility scene
(`tank_legacy_scene.xml`), and cable-test scene
(`tank_current_scene_cable_test.xml`) use the Research pool vehicle definition.
This includes CAD visual and collision geometry, colors, mass and inertia,
local sensor/camera mounts and field of view, thruster sites, actuators and sensors.
Map spawn poses, environment bodies and simulation options remain map-specific.
The obsolete net collector's inactive capture welds are removed because their
body no longer exists; buoy magnet and cable connections remain intact.

XML files remain self-contained for existing layout editors. After editing the
canonical vehicle, run `current/tools/sync_scene_vehicles.py --write` from the
`uuv_mujoco` directory, then run the same tool without `--write` to detect drift.
The GUI test-tank generator synchronizes its vehicle before generating the map,
including when the source scene contains a stale vehicle.

This contract does not unify hydrodynamic profile coefficients, controller
settings, waves, or damping across presets. Archived `outputs/` experiments,
historical version directories, and hidden scratch XML files are not rewritten.
Old calibration figures retain the configuration used when they were generated;
vehicle synchronization does not retroactively validate their reported metrics.

Validation: `tools/test_vehicle_scene_contract.py` checks all four scene
signatures, compiled body mass/inertia and cameras, finite stepping, preserved
environment/spawn, idempotence, and generation from a deliberately stale source.
`tools/check_gui_test_tank_mode.py` checks generated geometry, CAD-hand contact,
load-based buoy release, retained cable anchor, and surface rise.
The historical rake/net-collector tests describe a superseded robot mechanism;
they must not be used as evidence for the current CAD hand.
