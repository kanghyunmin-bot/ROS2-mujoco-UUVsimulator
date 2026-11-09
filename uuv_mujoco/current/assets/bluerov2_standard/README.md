# BlueROV2 visual meshes

These meshes are derived from the [Centrale Nantes BlueROV2 ROS 2 model](https://github.com/CentraleNantesROV/bluerov2/tree/dd7111b609b1d8bad052e4990d9b439e5317ad7c/bluerov2_description), revision `dd7111b609b1d8bad052e4990d9b439e5317ad7c`. The repository is distributed under the Apache License 2.0, included in this directory.

`body_material_01.obj` through `body_material_15.obj` were exported by splitting the repository's `bluerov2_noprop.obj` by material. It is the same assembly used by `bluerov2.xacro` through `bluerov2_noprop.dae`. Their original vehicle frame is retained, with the body dimensions about 0.458 × 0.344 × 0.255 m. Triangles are unchanged except for merging duplicate vertices. Material diffuse colors are recorded in `manifest.json` because MuJoCo does not import OBJ materials.

`prop_cw.obj` and `prop_ccw.obj` are derived from `bluerov2_propcw.dae` and `bluerov2_propccw.dae`. Each COLLADA scene transform and the `0.04499998` metre unit scale were applied to the OBJ vertices. Xacro describes propeller rotation about its local x axis. In the source model, thrusters 1, 2 and 5 use the CW propeller, and 3, 4 and 6 use the CCW propeller. Meshes are visual geometry and do not specify mass, hydrodynamics, or collision.

Source SHA-256 values

- `bluerov2_noprop.obj`: `e2a0c494c5327a9b62e9479b6727d4bd6b537ec8431244bced086b9859583746`
- `bluerov2_propcw.dae`: `f2cf947c0de6aaa27d4c193ab7d8b7d95b364c874cd4983dcdd24d8187c33141`
- `bluerov2_propccw.dae`: `27f0fae9eee278ae392ad9aea1da6c3e8e71fd5534d6582bf91f224b7963094f`
