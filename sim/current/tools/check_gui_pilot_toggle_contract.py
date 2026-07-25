#!/usr/bin/env python3
"""Smoke check for GUI pilot-control toggle flow."""

from __future__ import annotations

import pathlib
import sys


ROOT = pathlib.Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from gui.control_pilot_toggle import handle_rc_override_toggle  # noqa: E402
from gui_pilot_toggle_smoke_fixture import PilotToggleOwner  # noqa: E402


def _assert(condition: bool, label: str) -> None:
    if not condition:
        raise AssertionError(label)


def main() -> int:
    enabled = PilotToggleOwner(enabled=True, replay_running=True)
    handle_rc_override_toggle(enabled)
    _assert(enabled._replay_stopped, "enabled toggle must stop replay first")
    _assert(enabled._pilot_input_release_requested, "enabled toggle must request initial-depth release")
    _assert(enabled._published, "enabled toggle must publish pilot control")
    _assert(not enabled._released, "enabled toggle must not publish RC release")
    _assert(any("pilot control enabled" in event for event in enabled.node.events), "enabled event missing")

    disabled = PilotToggleOwner(enabled=False, replay_running=False)
    handle_rc_override_toggle(disabled)
    _assert(disabled._released, "disabled toggle must release RC override")
    _assert(not disabled._published, "disabled toggle must not publish pilot control")
    _assert(disabled.node.events == ["pilot control released"], "disabled event mismatch")

    print("gui_pilot_toggle_contract=PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
