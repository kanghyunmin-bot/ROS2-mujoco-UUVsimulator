# BlueROV2 contact model

The six original whole-body collision proxies remain separate from the visual
CAD meshes. The two enclosure proxies are split into the tube and two end caps,
giving ten collision geoms in total. These are primitive approximations of the
external shape and are not an exact CAD collision surface. The simulated mass
and inertia remain specified by the body inertial element.

| Proxy | Assumed exposed material | Sliding coefficient |
| --- | --- | ---: |
| `frame_port`, `frame_starboard` | HDPE | 0.35 |
| `electronics_enclosure`, `battery_enclosure` | Acrylic | 0.40 |
| Four `*_cap_front` / `*_cap_rear` geoms | Anodized aluminum | 0.50 |
| `foam_port`, `foam_starboard` | Polycarbonate fairing | 0.45 |

The coefficient values are engineering starting assumptions selected for this
simulation, not measured BlueROV2 wet-contact friction and not manufacturer
specifications. The source BlueROV2 model does not identify the material and
surface condition for every CAD component. It also does not establish whether
the DFKI test vehicle used acrylic or aluminum tubes. [Blue Robotics' technical
reference](https://bluerobotics.com/learn/technical-reference/) identifies
the frame as HDPE and the buoyancy fairing as polycarbonate. Its [product
specification](https://bluerobotics.com/store/rov/bluerov2/) lists acrylic and
anodized aluminum enclosure options and aluminum end caps. The buoyancy foam is
enclosed by the fairings, so the external foam proxies represent fairing
contact.

Contact parameters are intended for collision response. They do not control
the hydrodynamic drag computed by the UUV fluid model. The simulator uses a
single sliding coefficient in both tangential directions with `condim=3`.
Torsional and rolling coefficients in each `friction` triple are inactive
under this contact dimension.

The vehicle collision geoms use priority 2, matching the existing yellow-buoy
contact geoms and exceeding the research-pool floor and walls. Thus MuJoCo
uses the vehicle coefficient against the floor, and uses the maximum of the
two coefficients for the equally prioritized buoy. The buoy coefficient is
0.30, below each vehicle prior in the table. The contact solver parameters
match those used by the buoy. Rope geoms use priority 0. These combination
rules are defined by [MuJoCo contact parameters](https://mujoco.readthedocs.io/en/3.2.6/modeling.html#contact-parameters).
Different ground or buoy surfaces should be measured as contact pairs rather
than treating these values as universal properties of a material.
