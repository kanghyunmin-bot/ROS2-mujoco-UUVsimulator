"""GUI panel visibility toggles."""

from __future__ import annotations

from .control_toggle_telemetry import apply_telemetry_panel_visibility


class ControlToggleMixin:
    def _toggle_control_details(self) -> None:
        show = not self.control_details_visible.get()
        self.control_details_visible.set(show)
        if show:
            self.control_details_frame.grid()
            self.control_details_button.config(text="Hide details")
        else:
            self.control_details_frame.grid_remove()
            self.control_details_button.config(text="Details")

    def _toggle_vehicle_details(self) -> None:
        show = not self.vehicle_details_visible.get()
        self.vehicle_details_visible.set(show)
        if show:
            self.vehicle_details_frame.grid()
            self.vehicle_details_button.config(text="Details v")
        else:
            self.vehicle_details_frame.grid_remove()
            self.vehicle_details_button.config(text="Details >")

    def _toggle_telemetry_panel(self) -> None:
        show = not self.telemetry_visible.get()
        self.telemetry_visible.set(show)
        apply_telemetry_panel_visibility(self, show=show)

    def _toggle_control_tools(self) -> None:
        show = not self.control_tools_visible.get()
        self.control_tools_visible.set(show)
        if self.control_tools_content is None or self.control_tools_toggle_button is None:
            return
        if show:
            self.control_tools_content.grid()
            self.control_tools_toggle_button.config(text="Hide tools")
        else:
            self.control_tools_content.grid_remove()
            self.control_tools_toggle_button.config(text="Show tools")
