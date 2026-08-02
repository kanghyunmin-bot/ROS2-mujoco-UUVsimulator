# Competition-A physical tuning

These are the remaining empirical settings.  Arena geometry, camera transport,
dump ordering, and topic contracts are fixed in code and should not be duplicated
in another configuration source.

| value | current simulation value | tune using |
|---|---:|---|
| collection depth | `0.30 m` positive-down | top/front visibility and surface clearance |
| work depth | lane controller value | stable transition back to the pending lane |
| bonus centre (shared arena) | `(9.081, -1.305) m` | Course-A world centre `(-6.8, 0.0) m` |
| dump heading | `0.0 rad` | verify that F/R/F exits stay inside the usable circle |
| dump depth | `0.85 m` positive-down | collector outlet height |
| dump exit radius | `0.70 m` | odometry trace; deliberately crosses the `0.65 m` ring |
| top net ROI | x `[0.10, 0.90]`, y `[0.05, 0.95]` | annotated top-camera image |
| front alignment deadband | `0.07` normalized width | yaw oscillation around collector centreline |
| front capture proximity | bbox bottom `0.82` | net entry point in annotated front image |
| capture forward command | `1650` for `2.0 s` | observed odometry and collector `NETTED` event |
| dump command | forward `1660`, reverse `1340`, timeout `4.0 s` | F/R/F displacement trace |

The MuJoCo top camera is 35 mm below the Ping360 reference (`z=0.170 m`) with
an 82-degree vertical FOV.  Both front and top transports are fixed at
`1280x720 @ 10 Hz`; only detector inference size remains an internal YOLO
implementation detail.

Desktop measurements on the full competition scene were approximately
9.1--9.6 image messages/s for simultaneous front/top subscribers.  Viewer mode
ran around 28 display FPS but physics real-time factor was about 0.34; headless
physics was about 0.47.  Mission timeout interpretation should therefore use ROS
simulation time, while camera publication deliberately follows wall time.
