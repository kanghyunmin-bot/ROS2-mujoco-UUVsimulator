"""Command construction for GUI-started simulator stacks."""

from __future__ import annotations


def build_sim_stack_launch_command(owner, *, start_script, backend: str, extra_args: list[str] | None) -> list[str]:
    cmd = [str(start_script)]
    owner._append_mavros_surface_args(cmd)
    requested_args = extra_args or []
    if (
        owner._env_flag("UUV_GUI_SITL_DIRECT_MAVLINK", False)
        and not owner._arg_present(requested_args, "--direct-mavlink")
        and not owner._arg_present(requested_args, "--legacy-mavproxy")
    ):
        cmd.append("--direct-mavlink")
        owner.node.push_event("SITL transport: direct MAVLink outputs")
    if backend != "docker" and not owner._env_flag("UUV_GUI_SITL_REBUILD", False):
        cmd.append("--sitl-no-rebuild")
        owner.node.push_event("SITL rebuild skipped: using existing ArduSub binary")
    launch_extra_args = owner._normalized_sim_extra_args(extra_args)
    owner._append_initial_depth_args(cmd, launch_extra_args)
    if launch_extra_args:
        cmd.extend(launch_extra_args)
    return cmd


__all__ = ["build_sim_stack_launch_command"]
