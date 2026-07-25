"""Select runtime thruster performance config for runner modes."""

from __future__ import annotations

from pathlib import Path

from sim.physics.thruster_performance_config import default_thruster_performance_config
from sim.physics.thruster_performance_loader import load_thruster_performance_config


def select_thruster_performance_config(
    *,
    args,
    active_thruster_voltage: float,
    path: Path,
    plant_replay_direct_rcout: bool,
) -> dict:
    """Select the runtime PWM-to-force curve for closed-loop or plant replay."""

    perf_cfg = default_thruster_performance_config(
        requested_voltage=float(active_thruster_voltage),
        direct=bool(args.thruster_perf_direct),
    )
    if not args.disable_thruster_perf:
        perf_cfg = load_thruster_performance_config(
            path.expanduser(),
            requested_voltage=float(active_thruster_voltage),
            direct=bool(args.thruster_perf_direct),
        )
    _force_direct_for_raw_rcou_replay(perf_cfg, plant_replay_direct_rcout=plant_replay_direct_rcout)
    return perf_cfg


def _force_direct_for_raw_rcou_replay(perf_cfg: dict, *, plant_replay_direct_rcout: bool) -> None:
    if not (plant_replay_direct_rcout and perf_cfg.get("active") and not perf_cfg.get("direct")):
        return
    # Plant replay consumes final ArduSub/real RCOU PWM as the actuator command.
    # It must map raw PWM through the T200 curve once; legacy polynomial/gain
    # shaping would double-apply actuator calibration.
    perf_cfg["direct"] = True
    print(
        "[plant_replay] forcing --thruster-perf-direct for raw RCOU plant input",
        flush=True,
    )


__all__ = ["select_thruster_performance_config"]
