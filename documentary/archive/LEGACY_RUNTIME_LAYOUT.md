# Active MuJoCo Runtime

Date: 2026-06-07

The active MuJoCo runtime is exposed through:

```text
sim/current -> v2.2
```

Human-facing rule:

- Do not describe the live simulator as "v2.2 is latest".
- Describe it as `current-2026-06-07-uuv_sim`, backed by the compatibility
  directory `uuv_mujoco/v2.2`.
- Every new run report should cite `uuv_mujoco/RUNTIME_VERSION.json` first,
  including its `status` and `dirty_state`, then mention `v2.2` only as the
  backing path when file paths require it.

`v2.2` is a legacy directory name kept for script and report compatibility. It
does not mean the runtime is frozen at the old v2.2 implementation. Launch
commands, GUI wrappers, setup verification, and Docker defaults should resolve
the active runtime through `sim/current` unless they are referencing
historical evidence paths.

Operational rule:

- Treat `sim/current` as the only live runtime path.
- Treat `uuv_mujoco/v2.2` as the current compatibility backing directory, not as
  a stale release target.
- Do not launch `uuv_mujoco/v2.2` directly.  Root GUI/MuJoCo/SITL wrappers now
  reject `UUV_MUJOCO_RUNTIME_DIR=.../uuv_mujoco/v2.2`, and the freshness checker
  reports `direct_v22_runtime` as a hard failure.
- New code, validation commands, GUI launchers, and Docker defaults must be
  written against `current`.
- Historical debug/report artifacts may keep absolute `v2.2` paths because they
  are evidence snapshots, not active launch contracts.

New runtime code and new commands should not introduce direct `uuv_mujoco/v2.2`
entry points.  If `sim/current` is missing, setup should recreate the
alias instead of silently launching the legacy backing path.

Current primary runner:

```text
sim/current/run_uuv_mujoco.py
```

Preferred current launchers:

```text
sim/run_mujoco.sh
sim/start_sitl_mujoco.sh
sim/start_docker_sitl_mujoco.sh
sim/reset_sim.sh
run_control_gui.sh
run_control_gui_ubuntu.sh
uuv_control_gui.py
```

These wrappers source `.uuv_mujoco_env.sh` when present, resolve
`UUV_MUJOCO_RUNTIME_DIR` first, and fall back to `sim/current`.
Day-to-day commands should use these wrapper names instead of entering the
compatibility backing directory directly.

The preferred wrappers also run a freshness preflight before launch:

```text
sim/current/tools/check_runtime_freshness.py --fetch --refresh-version --warn-only
```

GUI-started native and Docker SITL/MuJoCo stacks also run this preflight even
when the GUI resolves to the compatibility backing script under
`uuv_mujoco/v2.2`.  This keeps the Start button, root wrappers, and direct
debug launchers on the same active-runtime evidence path.

The preflight fetches `origin`, compares local `HEAD` with `origin/uuv_sim`,
checks `sim/current`, records dirty working-tree/runtime evidence, and
refreshes `RUNTIME_VERSION.json` when there are no hard source/runtime failures.
If the source is not current, it reports the stale contract instead of silently
launching a mismatched runtime. If the source is current but the runtime is
locally modified, the version status is `current-dirty` rather than plain
`current`.
Set `UUV_MUJOCO_SKIP_FRESHNESS_CHECK=1` only for offline/debug runs where the
source freshness warning is intentionally skipped.

Active runtime provenance:

```text
uuv_mujoco/RUNTIME_VERSION.json
```

Verified source freshness:

```text
branch: uuv_sim
local HEAD: e60cd9383c214a7d6e4700cdeb5c2f2a8cf6d328
origin/uuv_sim: e60cd9383c214a7d6e4700cdeb5c2f2a8cf6d328
remote: https://github.com/kanghyunmin-bot/ROS2-mujoco-UUVsimulator.git
```

If this source head changes, refresh `RUNTIME_VERSION.json` and rerun the
source-contract audit before treating the runtime as current.

Current dirty-runtime state:

```text
status: current-dirty
working_tree_dirty_count: 740
active_runtime_dirty_count: 722
```

This means the source branch is current against `origin/uuv_sim`, but the live
runtime also includes uncommitted local simulator changes. That is acceptable
for active debugging, but reports must call it `current-dirty`, not "latest
v2.2". A user-facing status that calls this "latest v2.2" is wrong; use
`current-dirty` and the runtime version evidence instead.

Compatibility runner:

```text
sim/current/run_urdf_full.py
```

The compatibility runner must remain a thin wrapper only.
