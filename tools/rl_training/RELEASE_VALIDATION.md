# Release evidence and precision diagnostics

Added 2026-09-23. This is an instrumentation and diagnostic change, not a new
trained policy or reward version. Legacy/v3/v4 rewards, candidate selection,
actor observations, checkpoint formats and physical release thresholds remain
unchanged. Existing `success` remains a historical region-based metric.

## Reproduced ambiguity

The research pool uses body FLU: +X forward, +Y port, +Z up. CAD starboard hand
geoms are `cad_collision_92_0` through `cad_collision_92_53`, selected from the
manifest's wholly negative-Y hand bounds. The provisional slot reference is
[0.335511938, -0.091927344, -0.104344226] m in the vehicle root frame; it is a
hard-coded point, not a calibrated site. The scene's front camera right vector
is [0,-1,0]; the oblique work camera right vector also has negative Y, but image
offsets cannot be treated as metric slot coordinates.

In the actual compiled model, place the PVC center at slot reference +[0.039,0,0]
m. The rod is inside the slot box, but has **no vehicle contacts**. A diagnostic
external detach there satisfies the old reward's `success`. This is a controlled
classification fixture, not a policy rollout or a measured natural success rate.

## New evidence

`CourseBuoy.release_contact_evidence` is frozen immediately before `_detach`
changes constraints or recomputes the scene. It passes through the ROS buoy
status into the worker's reward-only measurement and saved episode trace.
No truth is added to `residual_state` or VLA inputs.

Schema `uuv.release_contact.v1` records:

- Simulation timestamp [s] matching the release event.
- Vehicle/buoy geom names and contact position in world coordinates [m].
- Signed separation [m], translational contact-force norm [N], solver-active flag.
- `right_fork_stem`, `right_fork_other`, `other_hand`, or `vehicle_body` category.

`other_hand` means the rest of the configured hand geoms, not necessarily one
specific left tine. Duration is explicitly unknown (`null`). This instantaneous
snapshot does not exclude earlier hull/left contact or prove exclusive causality.
Magnet `release_force_n` is retained separately; it must not be substituted for
the contact force. The research scene currently uses a 20 N weld-reaction
threshold held for 0.001 s, with a 0.0005 s physics timestep.

## Separate validator

`validate_release` returns schema `uuv.precision_validation.v1`. Contact
verification requires valid episode release, region flag, stem-contact flag,
matching event timestamp, positive force on an active right-fork/stem contact,
and no simultaneously loaded contact of another category. Old records without
the evidence remain unknown. Mixed contact is rejected conservatively.

The optional stability interval requires 2 s of samples, gaps no larger than
0.25 s, body-up tilt no larger than 0.35 rad and angular-speed norm no larger
than 0.5 rad/s. These are explicit draft diagnostic thresholds, not calibrated
specifications. Missing, truncated, stale or invalid samples cannot pass.

`contact_and_stability_verified` only names these two checks; it does not certify
the whole approach/alignment/insertion process. The current worker still ends at
release and passes **no stability interval**, so its new diagnostic cannot become
true. Future evaluation must collect a deliberate post-release policy window
before using this result for selection. No old candidate is automatically approved.

## Reproduce without training

Run with an existing environment providing MuJoCo/NumPy for the CAD fixtures,
and the standard library for the validator/audit:

```sh
python tools/rl_training/test_release_contact_snapshot.py
python tools/rl_training/test_release_validation.py
python tools/rl_training/test_audit_precision.py
python tools/rl_training/audit_precision.py outputs/rl-training/RUN --output NEW_REPORT.json
```

Audit output contains source paths and hashes, separates training/incumbent/
candidate rounds, lists overlapping failed v4 gates near the slot, and reports
reward-term totals. Rollout directories are zero-based; checkpoint collection
numbers are one-based. Partial evaluation rounds remain diagnostic only.

The original compatibility contact scanner also contained undefined local
variables; it now again performs the intended side-effect-free contact query.
The main physics loop already used the separate single-pass scanner, so this
fix is not evidence of a policy-performance improvement.
