"""Browse action for selecting a GUI RC replay bag."""

from __future__ import annotations

from .config import APP_ROOT, DEFAULT_RC_REPLAY_BAG


def _browse_rc_replay_bag(self) -> None:
    from .runtime import filedialog

    initial_dir = str(DEFAULT_RC_REPLAY_BAG.parent if DEFAULT_RC_REPLAY_BAG.parent.exists() else APP_ROOT)
    selected = filedialog.askdirectory(
        parent=self.root,
        title="Select ROS2 bag directory containing /mavros/rc/override",
        initialdir=initial_dir,
    )
    if selected:
        self.rc_replay_path_var.set(selected)
        self._set_rc_replay_status("replay: selected, not loaded")


__all__ = ["_browse_rc_replay_bag"]
