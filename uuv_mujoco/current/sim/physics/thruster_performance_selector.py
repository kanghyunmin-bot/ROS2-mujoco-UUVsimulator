"""Select runtime thruster performance config for runner modes."""

from __future__ import annotations

from pathlib import Path

from sim.physics.thruster_performance_config import default_thruster_performance_config
from sim.physics.thruster_performance_loader import load_thruster_performance_config
from sim.physics.thruster_voltage import load_voltage_trace, update_supply_voltage


def select_thruster_performance_config(
    *,
    args,
    active_thruster_voltage: float,
    path: Path,
    plant_replay_direct_rcout: bool,
    response_model: str = "command",
) -> dict:
    """Select the runtime PWM-to-force curve for closed-loop or plant replay."""

    if response_model not in ("command", "rpm"):
        raise ValueError("thruster_response_model must be 'command' or 'rpm'")
    include_rpm = response_model == "rpm"
    if include_rpm and (
        args.disable_thruster_perf or not (args.thruster_perf_direct or plant_replay_direct_rcout)
    ):
        raise ValueError("RPM response requires the measured direct PWM performance model")

    perf_cfg = default_thruster_performance_config(
        requested_voltage=float(active_thruster_voltage),
        direct=bool(args.thruster_perf_direct),
    )
    if not args.disable_thruster_perf:
        perf_cfg = load_thruster_performance_config(
            path.expanduser(),
            requested_voltage=float(active_thruster_voltage),
            direct=bool(args.thruster_perf_direct),
            include_rpm=include_rpm,
        )
    else:
        print(
            "[thruster perf] fixed-voltage curve disabled; using configured "
            "polynomial/gain model",
            flush=True,
        )
    trace_path = getattr(args, "thruster_voltage_trace", None)
    if trace_path:
        if not perf_cfg.get("active"):
            raise ValueError(
                "--thruster-voltage-trace requires the measured performance model"
            )
        perf_cfg["voltage_trace"] = load_voltage_trace(
            Path(trace_path),
            perf_cfg,
            time_offset_s=float(getattr(args, "thruster_voltage_time_offset_s", 0.0)),
        )
        update_supply_voltage(perf_cfg, 0.0)
        trace = perf_cfg["voltage_trace"]
        print(
            f"[thruster perf] voltage trace: {trace_path}; reference={trace['voltage_reference']}, "
            f"sim offset={trace['time_offset_s']:g}s, outside={trace['outside_trace']}; "
            f"source={trace['provenance']}; sha256={trace['csv_sha256']}",
            flush=True,
        )
    elif perf_cfg.get("active"):
        print(
            f"[thruster perf] ESC bus voltage: {active_thruster_voltage:g} V constant assumption; use --thruster-voltage-trace for a verified ESC voltage recording",
            flush=True,
        )
    _force_direct_for_raw_rcou_replay(
        perf_cfg, plant_replay_direct_rcout=plant_replay_direct_rcout
    )
    if include_rpm:
        print("[thruster perf] RPM response uses measured steady curves and configured time constants", flush=True)
    return perf_cfg


def _force_direct_for_raw_rcou_replay(
    perf_cfg: dict, *, plant_replay_direct_rcout: bool
) -> None:
    if not (
        plant_replay_direct_rcout
        and perf_cfg.get("active")
        and not perf_cfg.get("direct")
    ):
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
