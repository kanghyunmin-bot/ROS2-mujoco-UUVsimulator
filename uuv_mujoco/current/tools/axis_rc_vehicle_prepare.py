"""Vehicle preparation sequence for RC axis checks."""

from __future__ import annotations

import argparse


def settle_before_arm(node, args: argparse.Namespace) -> None:
    if args.pre_arm_settle_s > 0.0:
        node.current_phase = "pre_arm_settle"
        node.spin_with_rc(args.pre_arm_settle_s, hz=args.publish_hz, input_mode=args.input_mode)


def release_initial_depth_before_axis_run(node, args: argparse.Namespace) -> None:
    if args.switch_initial_depth_before_arm:
        node.switch_initial_depth_hold_to_target(timeout=args.wait_timeout)
        node.spin_with_rc(1.0, hz=args.publish_hz, input_mode=args.input_mode)


def release_initial_depth_after_mode(node, args: argparse.Namespace) -> None:
    if args.release_initial_depth_hold:
        node.release_initial_depth_hold(timeout=args.wait_timeout)
        if args.post_release_neutral_s > 0.0:
            node.spin_with_rc(args.post_release_neutral_s, hz=args.publish_hz, input_mode=args.input_mode)


def run_pre_dive(node, args: argparse.Namespace) -> None:
    if args.pre_dive_s > 0.0:
        node.spin_with_rc(
            args.pre_dive_s,
            axis="heave",
            command=args.pre_dive_command,
            hz=args.publish_hz,
            input_mode=args.input_mode,
        )
        if args.pre_settle_s > 0.0:
            node.spin_with_rc(args.pre_settle_s, hz=args.publish_hz, input_mode=args.input_mode)


def enter_manual_and_wait_for_stack(node, args: argparse.Namespace) -> None:
    require_rc_path = args.input_mode in ("rc-override", "both")
    node.wait_for_stack(timeout=args.wait_timeout, require_manual_input=require_rc_path)
    node.spin_with_rc(1.0, hz=args.publish_hz, input_mode=args.input_mode)
    node.call_set_mode("MANUAL", timeout=args.wait_timeout)
    node.spin_with_rc(1.0, hz=args.publish_hz, input_mode=args.input_mode)


def arm_after_initial_depth_policy(node, args: argparse.Namespace) -> None:
    settle_before_arm(node, args)
    release_initial_depth_before_axis_run(node, args)
    node.call_arm(True, timeout=args.wait_timeout)
    node.spin_with_rc(max(0.0, float(args.post_arm_settle_s)), hz=args.publish_hz, input_mode=args.input_mode)


def enter_requested_mode_and_release_hold(node, args: argparse.Namespace) -> None:
    if args.mode:
        node.call_set_mode(args.mode, timeout=args.wait_timeout)
    node.spin_with_rc(2.0, hz=args.publish_hz, input_mode=args.input_mode)
    release_initial_depth_after_mode(node, args)


def release_manual_control_input(node, args: argparse.Namespace) -> None:
    if args.input_mode == "manual-control":
        node.release_rc()
        node.spin_with_rc(0.5, hz=args.publish_hz, input_mode=args.input_mode)


def prepare_vehicle(node, args: argparse.Namespace) -> None:
    enter_manual_and_wait_for_stack(node, args)
    arm_after_initial_depth_policy(node, args)
    enter_requested_mode_and_release_hold(node, args)
    run_pre_dive(node, args)
    release_manual_control_input(node, args)


__all__ = [
    "arm_after_initial_depth_policy",
    "enter_manual_and_wait_for_stack",
    "enter_requested_mode_and_release_hold",
    "prepare_vehicle",
    "release_manual_control_input",
    "release_initial_depth_after_mode",
    "release_initial_depth_before_axis_run",
    "run_pre_dive",
    "settle_before_arm",
]
