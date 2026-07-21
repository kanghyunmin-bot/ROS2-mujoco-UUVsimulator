# Ping360 STL Filter Split

Date: 2026-06-07

## Scope

Split the Ping360 asset-prep CLI without changing the generated STL metadata
shape or active Ping360 runtime.

## Changed Files

- `tools/filter_ping360_stl_io.py`: binary STL read/write, bounding box,
  triangle offset, and normal calculation.
- `tools/filter_ping360_stl_components.py`: union-find connected-component
  statistics and cable-like component classification.
- `tools/filter_ping360_stl_pipeline.py`: high-level filtering pipeline and
  metadata construction.
- `tools/filter_ping360_stl.py`: thin CLI entrypoint.

## Contract Notes

- The active scene still references `assets/ping360/ping360_body_no_cable.stl`.
- Metadata keys remain `source`, `output`, `source_triangle_count`,
  `kept_triangle_count`, `component_count`, `source_offset_removed_mm`,
  `output_bbox_min_mm`, `output_bbox_max_mm`, `output_size_mm`, and
  `components`.
- No sonar runtime, MuJoCo scene, or sensor publish contract was changed.
- The split avoids Python 3.10-only `typing.TypeAlias` so the tool still runs
  on the macOS system Python 3.9 path used by simple command-line checks.

## Validation

```text
python3 -m compileall -q uuv_mujoco/current/tools/filter_ping360_stl*.py
python3 uuv_mujoco/current/tools/filter_ping360_stl.py --help
python3 - <<'PY'
import sys
from pathlib import Path
sys.path.insert(0, str(Path('uuv_mujoco/current/tools').resolve()))
from filter_ping360_stl_io import read_binary_stl, compute_bbox
path = Path('uuv_mujoco/current/assets/ping360/ping360_body_no_cable.stl')
_, triangles = read_binary_stl(path)
mins, maxs = compute_bbox(triangles)
print({'triangles': len(triangles), 'size_mm': [maxs[i]-mins[i] for i in range(3)]})
PY
```

Observed status:

```text
CLI help: PASS
existing body STL read: PASS, 33084 triangles
existing body STL bbox: [76.9983, 83.0, 90.1500] mm
```
