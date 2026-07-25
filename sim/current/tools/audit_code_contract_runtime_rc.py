"""Active-runtime RC input/output contract source check."""

from __future__ import annotations

from pathlib import Path

from audit_code_contract_common import contains_all, evidence
from audit_code_contract_types import Check, OFFICIAL_REFS


def build_runtime_rc_in_out_contract_check(runtime_paths: dict[str, Path]) -> Check:
    frame_py = runtime_paths["ros2_rc_override_frame_py"]
    callback_py = runtime_paths["ros2_rc_override_callback_py"]
    forwarding_py = runtime_paths["ros2_rc_override_forwarding_py"]
    mirror_py = runtime_paths["ros2_rc_override_mirror_py"]
    ok = (
        contains_all(
            frame_py,
            [
                "MAX_RC_OVERRIDE_CHANNELS = 18",
                "channels[:MAX_RC_OVERRIDE_CHANNELS]",
                "self._mavros_rc_heave_channel",
            ],
        )
        and contains_all(
            forwarding_py,
            ["send_rc_override(rc_override_forward_frame(channels))"],
        )
        and contains_all(
            callback_py,
            [
                "forward_rc_override_to_sitl(self, channels",
                "_handle_normalized_cmd(fwd, sway, yaw, heave)",
                "_mirror_rc_override_to_rc_in(self, channels)",
            ],
        )
        and contains_all(
            mirror_py,
            [
                'header.frame_id = "fcu"',
                "rc_in.channels = rc_override_forward_frame(channels)",
                "self._safe_publish(self.pub_mavros_rc_in, rc_in,",
            ],
        )
    )
    return Check(
        check_id="active_runtime_rc_override_forward_mirror_contract",
        status="PASS" if ok else "FAIL",
        title="RC override preserves MAVLink 18-channel frame and mirrors /mavros/rc/in",
        conclusion=(
            "The active runtime forwards RC_CHANNELS_OVERRIDE as the first 18 raw channels, "
            "uses normalized axes only for local fallback, and mirrors the same raw frame to /mavros/rc/in."
        ),
        evidence=[
            evidence(frame_py, "MAX_RC_OVERRIDE_CHANNELS = 18"),
            evidence(forwarding_py, "send_rc_override(rc_override_forward_frame(channels))"),
            evidence(callback_py, "_handle_normalized_cmd(fwd, sway, yaw, heave)"),
            evidence(mirror_py, "rc_in.channels = rc_override_forward_frame(channels)"),
        ],
        official_refs=[OFFICIAL_REFS["mavlink_rc_channels_override"]],
    )


__all__ = ["build_runtime_rc_in_out_contract_check"]
