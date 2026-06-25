"""Tk application entry point for the MuJoCo UUV control GUI."""

from __future__ import annotations

import sys
import signal
import traceback

from .app_lifecycle import (
    _gui_log,
    _on_close,
    _schedule_rc_fast_update,
    _raise_initial_window,
    _schedule_update,
    _spin,
    _update_rc_fast,
    run,
)
from .app_state import initialize_gui_app_state
from .config import (
    BACKEND_AUTO,
    BACKEND_MAVROS,
    BACKEND_NONE,
    BACKEND_SIM_BRIDGE,
)
from .buoy_layout_mixin import BuoyLayoutMixin
from .control_display_mixin import ControlDisplayMixin
from .layout_mixin import LayoutMixin
from .node import UuvGuiNode
from .ping360_mixin import Ping360ControlMixin
from .physics_mixin import PhysicsMixin
from .replay_mixin import RcReplayMixin
from .ros_process_mixin import RosProcessMixin
from .runtime import argparse, rclpy
from .sim_stack_process_mixin import SimStackProcessMixin


class UuvControlGui(
    RcReplayMixin,
    ControlDisplayMixin,
    RosProcessMixin,
    SimStackProcessMixin,
    Ping360ControlMixin,
    BuoyLayoutMixin,
    PhysicsMixin,
    LayoutMixin,
):
    MODE_BUTTONS = ("MANUAL", "STABILIZE", "ALT_HOLD", "GUIDED", "SURFACE", "POSHOLD")

    def __init__(self, node: UuvGuiNode, title: str):
        initialize_gui_app_state(self, node, title)
        self._build_layout()
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)
        self.root.after(100, self._raise_initial_window)
        self.root.after(1000, lambda: _gui_log("Tk mainloop alive"))
        self._schedule_update()
        self._schedule_rc_fast_update()

    _spin = _spin
    _raise_initial_window = _raise_initial_window
    _schedule_rc_fast_update = _schedule_rc_fast_update
    _schedule_update = _schedule_update
    _update_rc_fast = _update_rc_fast
    _on_close = _on_close
    run = run


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="UUV MAVROS telemetry and control GUI")
    parser.add_argument(
        "--namespace",
        default="/mavros",
        help="MAVROS namespace to use (default: /mavros)",
    )
    parser.add_argument(
        "--backend",
        choices=(BACKEND_AUTO, BACKEND_NONE, BACKEND_MAVROS, BACKEND_SIM_BRIDGE, "sim"),
        default=BACKEND_AUTO,
        help="Control/RC compatibility profile (default: auto)",
    )
    parser.add_argument(
        "--title",
        default="UUV Control GUI",
        help="GUI window title",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    _gui_log(f"starting backend={args.backend} namespace={args.namespace} title={args.title!r}")
    try:
        rclpy.init(args=None)
        node = UuvGuiNode(namespace=args.namespace, backend=args.backend)
        app = UuvControlGui(node=node, title=args.title)

        def request_close(_signum, _frame) -> None:
            if not app._closed and app.root.winfo_exists():
                app.root.after(0, app._on_close)

        signal.signal(signal.SIGINT, request_close)
        signal.signal(signal.SIGTERM, request_close)
        app.run()
    except BaseException:
        traceback.print_exc(file=sys.stderr)
        raise
    return 0
