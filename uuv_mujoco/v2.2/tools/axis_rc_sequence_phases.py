"""Phase sequencing helpers for RC axis checks."""

from __future__ import annotations

import argparse

from axis_rc_contract import Phase


def append_neutral_phase(node, args: argparse.Namespace, name: str, duration: float) -> None:
    neutral_start = node.elapsed()
    node.current_phase = name
    node.current_axis = "neutral"
    node.current_command = 0.0
    node.phases.append(Phase(name, "neutral", 0.0, neutral_start, neutral_start + duration))
    node.spin_with_rc(duration, hz=args.publish_hz, input_mode=args.input_mode)


def _spin_axis_phase(node, args: argparse.Namespace, axis: str, command: float) -> None:
    duration_remaining = max(0.0, float(args.axis_s))
    pulse_on_s = max(0.0, float(getattr(args, "pulse_on_s", 0.0)))
    pulse_off_s = max(0.0, float(getattr(args, "pulse_off_s", 0.0)))
    if pulse_on_s <= 0.0 or pulse_off_s <= 0.0:
        node.spin_with_rc(duration_remaining, axis=axis, command=command, hz=args.publish_hz, input_mode=args.input_mode)
        return

    while duration_remaining > 1e-9:
        on_s = min(pulse_on_s, duration_remaining)
        node.current_axis = axis
        node.current_command = float(command)
        node.spin_with_rc(on_s, axis=axis, command=command, hz=args.publish_hz, input_mode=args.input_mode)
        duration_remaining -= on_s
        if duration_remaining <= 1e-9:
            break

        off_s = min(pulse_off_s, duration_remaining)
        node.current_axis = "neutral"
        node.current_command = 0.0
        node.spin_with_rc(off_s, hz=args.publish_hz, input_mode=args.input_mode)
        duration_remaining -= off_s


def append_axis_phase(node, args: argparse.Namespace, axis: str, command: float) -> None:
    phase_start = node.elapsed()
    phase_name = f"{axis}_{'pos' if command > 0 else 'neg'}"
    node.current_phase = phase_name
    node.current_axis = axis
    node.current_command = float(command)
    node.phases.append(Phase(phase_name, axis, command, phase_start, phase_start + args.axis_s))
    _spin_axis_phase(node, args, axis, command)
    append_neutral_phase(node, args, f"neutral_after_{phase_name}", args.neutral_s)


def run_axis_sequence(node, args: argparse.Namespace) -> None:
    if not args.neutral_only and abs(float(args.command)) > 1e-9:
        for axis in args.axes:
            for command in (abs(args.command), -abs(args.command)):
                append_axis_phase(node, args, axis, command)
    elif not args.neutral_only:
        args.neutral_only = True

    if args.neutral_only and args.neutral_s > 0.0:
        append_neutral_phase(node, args, "neutral_hold", args.neutral_s)


__all__ = ["append_axis_phase", "append_neutral_phase", "run_axis_sequence"]
