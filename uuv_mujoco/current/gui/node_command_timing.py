"""One-shot scheduling helpers for GUI command paths."""

from __future__ import annotations


def _schedule_once(self, delay_s: float, callback) -> None:
    holder = {}

    def _timer_cb() -> None:
        timer = holder.get("timer")
        if timer is not None:
            try:
                timer.cancel()
            except Exception:
                pass
        try:
            callback()
        finally:
            if timer in self._one_shot_timers:
                self._one_shot_timers.remove(timer)

    timer = self.create_timer(max(0.0, float(delay_s)), _timer_cb)
    holder["timer"] = timer
    self._one_shot_timers.append(timer)


__all__ = ["_schedule_once"]
