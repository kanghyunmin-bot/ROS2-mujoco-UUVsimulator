# Hydrodynamic query optimization — 2026-09-10

The distributed hull keeps all 105 sample points, the same current gradients,
spatial/temporal harmonics, speed limits, immersion calculation and force formulas.
Physics timestep, controller cadence, drag coefficients and contact settings were
not changed by this optimization.

## Change

Profiling 300 wrench evaluations identified per-point current/surface callbacks
as approximately 83% of the profiled total. Every point repeated scalar validation,
time-dependent harmonic calculations and small NumPy allocations.

- `sim/physics/current_field.py`: added batched position queries with the same
  per-point current field and norm limit. Temporal harmonics are evaluated once
  per batch.
- `sim/runtime/water_environment_runtime.py`: added batch water queries. Flat water
  supplies one constant height and exactly zero orbital velocity. Harmonic water
  retains the original per-point orbital/height implementation and dry-point rules.
- `sim/physics/distributed_hydrodynamics.py`: optional batch callbacks, preserving
  the original scalar API and scalar current query at the residual wrench reference.
- `sim/runtime/underwater_distributed_hydrodynamics.py`: connects batch queries when
  available, falling back to scalar callbacks for other environment providers.

No queries are cached across simulation times or positions. Safety validation and
force limits remain active. Batching can change floating-point rounding, so bitwise
identity is not promised for arbitrary states.

## Measurements

Measured on the current workstation/container; these are not hardware-independent
performance guarantees. Profiling overhead is excluded from the timing numbers.

| Measurement | Before | After |
|---|---:|---:|
| Standalone vehicle + buoy wrench evaluation | 2.65 ms | 0.47 ms |
| Live wrench phase, GUI + ROS/SITL enabled | ~3.0 ms | ~0.87 ms |
| Live full simulation step | ~3.54 ms | ~1.44 ms |
| Live simulation time / wall time | ~0.23 | ~0.57 |

The live viewer stayed near 30 FPS after startup. The simulator is still slower
than real time; this change does not establish real-time training throughput or
multi-environment performance. Harmonic-wave profiles retain scalar orbital
queries and can cost more than the flat research pool.

## Validation

- `tools/test_hydrodynamic_batch_sampling.py`: 90 randomized complete distributed
  output comparisons across baseline/hybrid/wave profiles, varied orientations,
  immersed/surface/dry states and high speeds; tolerance 1e-10 absolute / 1e-12
  relative. Also checks scalar-vs-batch current limits, disabled fields and invalid
  inputs.
- Runtime regression requires one batch query for all 105 points. It fails with
  the old runtime dispatch and passes with the new dispatch.
- 1,500 evolving MuJoCo steps compared the old scalar dispatch with batch dispatch:
  no detected difference in applied wrench or qpos in that trajectory; no warnings.
- 58 focused unit tests passed across batch sampling, distributed hydrodynamics,
  residual damping, pool hydrodynamics, free surface and full-matrix hydrodynamics.
- Four attached-rope collision/magnetic-release fixtures passed (both sides,
  native/custom vehicle fluid modes).

Detailed local measurements and trajectory comparison are in
`outputs/hydro-opt-20260910/`. No full repository-wide test pass is claimed.
