"""Ping360 sonar/config publish controls."""

from __future__ import annotations


class Ping360ConfigMixin:
    def _set_ping360_enabled(self, enabled: bool) -> None:
        enabled = bool(enabled)
        self.ping360_enabled_var.set(enabled)
        self.node.publish_ping360_enabled(enabled)
        self._set_ping360_view_status(f"ping360 sonar: {'on' if enabled else 'off'} requested")

    def _toggle_ping360_enabled(self) -> None:
        self._set_ping360_enabled(bool(self.ping360_enabled_var.get()))

    def _apply_ping360_params(self) -> None:
        range_m = self._read_float_var(self.ping360_range_var, 2.0, 0.75, 50.0)
        num_steps = self._read_int_var(self.ping360_num_steps_var, 1, 1, 10)
        gain = self._read_int_var(self.ping360_gain_var, 0, 0, 2)
        frequency_khz = self._read_int_var(self.ping360_frequency_var, 750, 500, 1000)
        start_grad = self._read_int_var(self.ping360_start_angle_var, 0, 0, 399)
        stop_grad = self._read_int_var(self.ping360_stop_angle_var, 399, 0, 399)
        interface_mode = self.ping360_interface_var.get().strip().lower()
        if interface_mode not in {"ethernet", "usb", "rs485"}:
            interface_mode = "ethernet"
            self.ping360_interface_var.set(interface_mode)

        self.node.publish_ping360_config(
            range_m=range_m,
            num_steps=num_steps,
            gain=gain,
            interface_mode=interface_mode,
            frequency_khz=frequency_khz,
            start_angle_grad=start_grad,
            stop_angle_grad=stop_grad,
        )
        self._set_ping360_view_status("ping360 config: published")


__all__ = ["Ping360ConfigMixin"]
