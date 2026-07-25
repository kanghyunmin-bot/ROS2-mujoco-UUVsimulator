# Roll stability probe split

Date: 2026-06-07

## Scope

- Split `tools/roll_stability_probe.py` into focused runtime modules:
  - `tools/roll_stability_probe_callbacks.py`
  - `tools/roll_stability_probe_rc.py`
  - `tools/roll_stability_probe_commands.py`
  - `tools/roll_stability_probe_sequence.py`
- Kept `tools/roll_stability_probe.py` as the public `StabilityProbe`
  composition surface used by `tools/roll_stability_sweep.py`.

## Contract

The split is behavior-neutral.  RC override publishing, command services,
telemetry callbacks, and sequence timing keep the same public method names so
the sweep runner can keep importing `StabilityProbe`.

## Verification

```text
python3 -m compileall -q sim/current/tools/roll_stability_probe.py sim/current/tools/roll_stability_probe_callbacks.py sim/current/tools/roll_stability_probe_rc.py sim/current/tools/roll_stability_probe_commands.py sim/current/tools/roll_stability_probe_sequence.py
python3 sim/current/tools/roll_stability_sweep.py --help
```

The public method smoke check confirmed `run_probe`, `publish_rc`,
`wait_for_stack`, `_on_pose`, and `_on_rc_out` are still present on
`StabilityProbe`.
