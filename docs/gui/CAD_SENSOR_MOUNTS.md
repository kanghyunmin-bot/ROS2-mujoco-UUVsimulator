# Sensor placement on the 2026 assembly

The simulator uses the original 26_full_asem_auv_2 STL, registered to the existing
thruster array. Coordinates below are metres in base_link (forward X, left Y, up Z).
The organization `auv/launch/rov_start.launch.py` was checked against GitHub on
2026-09-10. Its frame names and DVL roll convention are retained. Camera placement
comes from the operator's annotated front-cover image, not measured extrinsics.

| Sensor | X | Y | Z | Placement |
|---|---:|---:|---:|---|
| DVL | -0.00488 | 0 | -0.15861 | Acoustic origin 0.5 mm below the lowest face of CAD component 331; roll π |
| Ping360 | 0.01018 | 0 | 0.17592 | External upper scanner assembly (CAD parts 103/140/602) |
| IMU / FCU | 0.13135 | 0 | 0.08541 | Inside main enclosure; latest launch values retained |
| BAR30 / depth | -0.17364 | -0.03034 | -0.03286 | Inside the right rear enclosure section |
| Front camera | 0.247 | 0 | 0.037 | Behind front window, above its centre; forward view |
| Hand camera | 0.247 | -0.044 | -0.033 | Behind lower-right window area; looks at right rake |

The old DVL site was at Z=-0.03910, above the actual CAD acoustic head. Its X/Y and
orientation agreed with the launch, but its height did not match the current STL.
The old depth site Z=0.0536 lay above the rear enclosure; X/Y are retained while
its reference point is brought inside that enclosure. Physical-robot launch
calibration is not overwritten with simulator estimates. Simulation ROS TF and
sensor origins are built from the same updated MJCF sites.

The hand camera looks toward approximately (0.370, -0.180, -0.097). Both optical
centres lie behind the panel's inner face X=0.24918, inside the enclosure. The
front-panel mesh (CAD component 339) was separated from visual chunk 16 without
changing its original triangles, then rendered as transparent acrylic. This is
an optical-window material assumption, not a measured transmission/refraction
model. The panel's existing solid collider remains active. Other CAD details and
robot fluid ellipsoids are unchanged. The visible right edge in the hand view is
the enclosure rim; all five rake tines remain visible.

`config/sensor_mounts_2026.json` records these placements. Both research-pool and
course scenes carry the same sensor/camera sites. Existing camera topic and TF
frame names are unchanged. `tools/check_cad_sensor_mounts.py` checks site values,
optical TF against rendered camera directions, camera containment, panel collision,
and all four downward DVL rays including the robot's own colliders. Rendered RGB
views were also inspected.

Reference: https://github.com/2026-kmu-underwater-robot/auv/blob/756d1a412c9a4804ae9384b6ded18245969bddec/launch/rov_start.launch.py
