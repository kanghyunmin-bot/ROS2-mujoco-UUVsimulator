# Research-pool buoy, magnet and rope physics

## Current behavior (2026-09-10)

All three colored ellipsoid shells, PVC stems, upper/lower jigs, both magnet
halves and anchor fittings have active colliders. Thin equator paint bands are
visual only. The vehicle uses the original CAD compound colliders and eight
thruster assembly colliders; this follow-up did not alter their geometry.

The magnetic connection releases at a resultant translational weld load of
15 N after a 1 ms confirmation interval (rounded up to physics steps). This
replaces the previous 40 ms delay: a CAD vehicle moving into an attached rope
reproduced numerical divergence while the overloaded magnet was held too long.
Gentle contact and a sustained additional 10 N pull still hold; 20 N releases
within 10 ms. Rotational weld rows are not counted as force. The 15 N value is an
operator-selected assumption, not a measured shear/peel model. There is no
attraction across a gap or automatic reattachment.

The scene uses a maximum 1 ms physics step and the `implicit` integrator. The
400 Hz FCU launcher subdivides this to 0.8333 ms. Full implicit integration handles
the coupled rotational dynamics of the ball-jointed rope. `implicitfast` still
failed one of the attached-rope impact fixtures, so it was not retained.

Each rope now uses six ball-jointed capsule links (18 links across three buoys),
down from 24 per rope at the default depth. The diameter remains 6 mm and the
length follows the GUI depth. Quadratic drag, lift and fluid added-mass coefficients
on rope capsules are zero; only small native viscous resistance and joint damping
remain. This deliberately approximates water response to reduce solver cost.
Global water properties remain enabled for environment stability, while the custom
vehicle contract suppresses duplicate native vehicle loads. Vehicle fluid tuning
cannot resize or retune rope capsules.

The bottom is anchored; the free end sags after release. Rope collisions remain
active against robot, buoy and pool. Only the first link and its own anchor are
excluded because they overlap at the attachment point. Six links preserve coarse
bending and tension, but cannot reproduce tight wraps, knots or fine curvature.
This is a non-stretching rope approximation, not a braid/elastic-breakage model.

Normal contact uses `solimp=.999 .9999 .0005 .5 2`, `condim=3`; general solids use
`solref=.004 1`, priority 2 and sliding friction 0.30. Rakes use `.002 1`, priority
3 and friction 0.35. These wet-friction values are assumptions. The integrator,
magnetic timing and fluid ownership corrections preserve vehicle mass/inertia,
CAD geometry and existing fluid ellipsoid dimensions.

A numerical failure no longer silently resets the vehicle and keeps feeding the
controller. The shared physics-step guard saves the preceding state under
`generated/physics_failures/` and stops the runtime with a diagnostic error. This
is a failure-handling safeguard, not a substitute for stable contact dynamics.

## What the tests established

- Colored shells and fittings have colliders; enabling masks alone was insufficient.
- Original 2 ms timing allowed roughly 5.9 mm transient jig penetration. The 1 ms
  step / 2 ms rake response passed 32 upper/lower jig/magnet step-load fixtures at
  under 2 mm penetration, plus 234 fitting-face checks under 15 N.
- Existing 24 guided rake-retention cases and eight free-buoy transport cases pass.
- Attached-rope sweeps at 0.5 m/s pass on both sides in native and custom fluid modes.
  The former 24-link model reproduced a numerical failure with the 40 ms delay.
- Small viscous resistance acts on the rope in custom mode without adding vehicle fluid forces.
  Restoring global fluid disable makes that regression fail.
- Magnetic threshold, free rise, rope sag/contact and GUI depth-edit tests pass.
- An injected MuJoCo numerical failure is stopped and its preceding state saved.

The user's captured stuck pose showed the PVC stem intersecting a lower housing;
it was not evidence that the colored shell lacked a collider. The limited earlier
settled-load tests did not cover the coupled vehicle/attached-rope interaction or
the distributed launch's fluid ownership. A damaged, already-interpenetrating
state must be reset; these corrections are not a teleport-based unjamming routine.
No finite set of fixtures guarantees zero penetration at arbitrary impact speeds.

Run in the simulator environment:

- `tools/check_attached_buoy_impacts.py` (`--without_fix` restores the long delay)
- `tools/check_rope_fluid_ownership.py` (`--without_fix` removes rope water drag)
- `tools/check_physics_reset_guard.py` (`--without_fix` permits silent reset)
- `tools/check_research_pool_magnet_rope.py`
- `tools/check_buoy_jig_contacts.py`
- `tools/check_rake_jig_retention.py`
- `tools/test_research_pool_layout.py`

## Configuration and remaining approximations

Scene numerics: `buoy_magnet_force_release=1`, `buoy_magnet_break_n=15`,
`buoy_magnet_break_hold_s=.001`, `buoy_shape_drag=1`,
`buoy_contact_max_timestep=.001`. Existing environment overrides for break force
and contact-break hold remain available. Legacy course scenes retain their
existing release policy unless they opt into force-based release.

Buoy mass/net lift remain the previous 10 g / approximately 1 N assumptions. Shape
orientation affects projected-area drag and waterline extent, but these do not
constitute measured buoy hydrodynamics. GUI depth changes rebuild the same rope
and preserve its contact/material settings; restarting the simulation applies edits.

References: [MuJoCo integration](https://mujoco.readthedocs.io/en/latest/computation/)
and [fluid forces](https://mujoco.readthedocs.io/en/3.1.1/computation/fluid.html).
