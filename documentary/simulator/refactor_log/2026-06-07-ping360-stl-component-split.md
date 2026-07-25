# Ping360 STL Component Split

## Scope

Split Ping360 binary-STL connected-component filtering used for mesh
preprocessing. This does not change active Ping360 runtime messages, sonar
simulation, ROS topics, or MuJoCo plant contracts.

## Files

- `tools/filter_ping360_stl_components.py`: compatibility exports.
- `tools/filter_ping360_union_find.py`: union-find connectivity helper.
- `tools/filter_ping360_component_stats.py`: quantized vertex labeling and
  component bounding-box/stat calculation.
- `tools/filter_ping360_component_classify.py`: cable-like component detection
  and keep/drop decisions.

## Contract

- `component_stats()` still returns `(components, root_for_triangle)`.
- `classify_components()` still mutates component dicts with `cable_like` and
  `kept` and returns kept roots.
- `filter_ping360_stl_pipeline.py` call surface is unchanged.
- No controller parity, RC override, plant input, or sensor publication path was
  modified.

## Verification

```bash
python3 -m compileall -q \
  uuv_mujoco/v2.2/tools/filter_ping360_stl_components.py \
  uuv_mujoco/v2.2/tools/filter_ping360_component_stats.py \
  uuv_mujoco/v2.2/tools/filter_ping360_component_classify.py \
  uuv_mujoco/v2.2/tools/filter_ping360_union_find.py \
  uuv_mujoco/v2.2/tools/filter_ping360_stl_pipeline.py

PYTHONPATH=uuv_mujoco/v2.2/tools python3 - <<'PY'
from pathlib import Path
import tempfile
from filter_ping360_stl_io import write_binary_stl, read_binary_stl
from filter_ping360_stl_pipeline import filter_ping360_stl

body = [
    ((0.0, 0.0, 1.0), ((0.0, 0.0, 850.0), (20.0, 0.0, 850.0),
                       (0.0, 20.0, 850.0)), 0),
    ((0.0, 0.0, 1.0), ((20.0, 0.0, 850.0), (20.0, 20.0, 850.0),
                       (0.0, 20.0, 850.0)), 0),
]
cable = [
    ((0.0, 0.0, 1.0), ((0.0, 0.0, 100.0), (600.0, 0.0, 100.0),
                       (0.0, 2.0, 100.0)), 0),
    ((0.0, 0.0, 1.0), ((600.0, 0.0, 100.0), (600.0, 2.0, 100.0),
                       (0.0, 2.0, 100.0)), 0),
]
with tempfile.TemporaryDirectory(prefix='ping360_stl_') as tmp:
    src = Path(tmp) / 'src.stl'
    dst = Path(tmp) / 'dst.stl'
    write_binary_stl(src, body + cable, (0.0, 0.0, 0.0))
    metadata = filter_ping360_stl(
        src, dst,
        keep_min_z_mm=840.0,
        cable_min_length_mm=500.0,
        cable_max_thickness_mm=8.0,
        quantize=10000.0,
    )
    _, out_triangles = read_binary_stl(dst)
    print(metadata['source_triangle_count'], metadata['kept_triangle_count'],
          len(out_triangles), metadata['component_count'])
    print([component['kept'] for component in metadata['components']])
PY

python3 -m compileall -q sim/current uuv_control_gui.py
git diff --check
python3 sim/current/tools/audit_code_contract_sources.py
python3 sim/current/tools/check_runtime_readiness_policy.py
python3 sim/current/tools/verify_ardusub_thruster_contract.py --quiet
/Users/kanghyunmin/.venvs/mujoco311/bin/python \
  sim/current/tools/physics_contract_audit.py --simulate-s 0.05
```

The synthetic STL run printed `4 2 2 2` and `[True, False]`, confirming that
the high-z sonar body component is kept while the long thin cable component is
dropped.
