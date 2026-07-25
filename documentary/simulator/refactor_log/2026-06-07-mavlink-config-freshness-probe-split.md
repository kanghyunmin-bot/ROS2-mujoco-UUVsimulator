# MAVLink Config And Freshness Probe Split

Date: 2026-06-07

Scope:

- `bridge/sitl_transport_mavlink_config.py`
- `tools/runtime_freshness_probe.py`

What changed:

- Split SITL MAVLink transport initialization into focused config modules for
  base MAVLink servo settings, dedicated command-link setup, auto-ready state,
  telemetry observer state, and polling cadence.
- Split active-runtime freshness probing into focused modules for git command
  execution, dirty path parsing, JSON metadata loading, and
  `sim/current` alias resolution.
- Fixed freshness git output handling so `git status --porcelain` evidence
  preserves the first line's leading status column.  This keeps dirty-path
  samples such as `.uuv_mujoco_env.sh` from losing their leading dot.
- Kept the public import surfaces stable:
  `initialize_mavlink_transport()`, `initialize_auto_ready_state()`,
  `initialize_mavlink_telemetry_state()`,
  `initialize_transport_polling_state()`, `collect_freshness_inputs()`,
  `run_git()`, `git_text()`, `load_json()`, `alias_text()`, and
  `resolve_runtime_dir()` remain reachable through their existing facades.

Contract notes:

- `sim/current -> v2.2` remains the active runtime alias backed by the
  compatibility directory name.
- `v2.2` is not the freshness label. Freshness is determined by
  `sim/current`, local `HEAD`, `origin/uuv_sim`,
  `RUNTIME_VERSION.json`, and dirty-runtime evidence.
- ArduPilot source and the ArduPilot submodule pointer were not modified.

Focused verification:

```text
python3 -m compileall -q \
  sim/current/bridge/sitl_transport_mavlink_config.py \
  sim/current/bridge/sitl_transport_mavlink_base_config.py \
  sim/current/bridge/sitl_transport_command_link_config.py \
  sim/current/bridge/sitl_transport_auto_ready_config.py \
  sim/current/bridge/sitl_transport_mavlink_telemetry_config.py \
  sim/current/bridge/sitl_transport_polling_config.py \
  sim/current/bridge/sitl_transport_config.py

sitl_transport_mavlink_config_smoke PASS

python3 -m compileall -q \
  sim/current/tools/runtime_freshness_probe.py \
  sim/current/tools/runtime_freshness_git_probe.py \
  sim/current/tools/runtime_freshness_dirty_paths.py \
  sim/current/tools/runtime_freshness_json_io.py \
  sim/current/tools/runtime_freshness_runtime_resolve.py \
  sim/current/tools/check_runtime_freshness.py

runtime_freshness_probe_helpers PASS
runtime_freshness_probe_payload PASS
runtime_freshness_porcelain_preserve PASS
```

Inventory result:

- `bridge/sitl_transport_mavlink_config.py` is no longer in the top hotspot
  list after the split.
- `tools/runtime_freshness_probe.py` is no longer in the top hotspot list
  after the split.
