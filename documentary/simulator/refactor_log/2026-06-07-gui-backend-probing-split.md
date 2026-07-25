# GUI Backend Probing Split

Date: 2026-06-07

Scope: active GUI backend detection code under `sim/current/gui`.

## Why

The previous `gui/node_backend_runtime.py` mixed ROS graph counts, service
readiness, backend label formatting, and backend scoring policy.  That made GUI
states such as "ready but RC override is delayed" harder to inspect because the
observable graph inputs and selected backend policy were blended together.

## Changed

- Added `gui/node_backend_counts.py` for safe publisher/subscriber counts,
  service readiness, and one-shot graph count collection.
- Added `gui/node_backend_layout.py` for effective backend and RC layout labels.
- Added `gui/node_backend_selection.py` for MAVROS-vs-sim-bridge scoring and
  backend selection.
- Kept `gui/node_backend_runtime.py` as the compatibility facade exposing the
  same public helpers used by `gui/node_state_runtime.py` and `UuvGuiNode`.

## Validation

```text
python3 -m compileall -q \
  sim/current/gui/node_backend_runtime.py \
  sim/current/gui/node_backend_counts.py \
  sim/current/gui/node_backend_layout.py \
  sim/current/gui/node_backend_selection.py \
  sim/current/gui/node_state_runtime.py \
  sim/current/gui/node.py

PYTHONPATH=sim/current python3 - <<'PY'
from gui.node_backend_runtime import (
    active_layout,
    backend_label,
    effective_backend,
    probe_backend,
    rc_mapping_summary,
    safe_count_publishers,
    safe_count_subscribers,
    service_ready,
)
from gui.node_backend_counts import probe_graph_counts
from gui.node_backend_selection import mavros_score, select_backend, sim_bridge_score
print({
    'runtime': callable(probe_backend),
    'counts': callable(probe_graph_counts),
    'layout': callable(active_layout) and callable(backend_label),
    'score': callable(mavros_score) and callable(sim_bridge_score) and callable(select_backend),
    'compat': all(callable(x) for x in [effective_backend, rc_mapping_summary, safe_count_publishers, safe_count_subscribers, service_ready]),
})
PY

python3 sim/current/tools/refactor_inventory.py \
  --root sim/current --limit 20 --format markdown

git diff --check -- \
  sim/current/gui/node_backend_runtime.py \
  sim/current/gui/node_backend_counts.py \
  sim/current/gui/node_backend_layout.py \
  sim/current/gui/node_backend_selection.py
```

Observed smoke result:

```text
{'runtime': True, 'counts': True, 'layout': True, 'score': True, 'compat': True}
```

The refactor inventory no longer lists `gui/node_backend_runtime.py` in the top
20 hotspots after the split.

## Boundary

No backend weights, ROS topic names, RC mapping, ArduPilot source, ArduPilot
submodule pointer, controller shim, PWM remap, or physics coefficient changed.
