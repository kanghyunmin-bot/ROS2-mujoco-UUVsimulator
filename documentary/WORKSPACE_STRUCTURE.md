# Workspace structure

The workspace has four top-level working areas:

```text
analysis/       Experiment data, notebooks, generated figures, and analysis tools
documentary/    Architecture, contracts, release notes, and historical documents
sim/
  current/      Active MuJoCo simulator runtime
  ardupilot/    ArduPilot checkout used by the active runtime
rospkg/         ROS 2 package sources and their build entry point
```

Generated ROS build directories (`build`, `install`, and `log`), simulator logs,
Python caches, editor indexes, downloaded release archives, and duplicate
workspaces are not part of the source layout. They may be recreated by the
build, run, or packaging commands.

The `docs` directories below `sim/ardupilot` and individual packages under
`rospkg/src` belong to those upstream projects. Workspace-owned documentation
is kept under `documentary/`.

Key entry points remain at the workspace root so existing launch and install
commands stay short:

- `run_control_gui.sh`
- `README.md`

Installation, preflight, distribution compatibility, and package-specific setup
scripts live under `sim/current/tools/install/`. Runtime environment defaults
live in `sim/environment.sh`.
