"""Message decoding for GUI RC override replay samples."""

from __future__ import annotations

from .config import RC_MESSAGE_CHANNEL_COUNT, RC_REPLAY_TOPIC
from .gui_rc_padding import padded_rc_channels
from .models import RcReplaySample


def decode_replay_samples(reader, *, deserialize_message, override_rc_in_type) -> list[RcReplaySample]:
    samples: list[RcReplaySample] = []
    first_timestamp_ns: int | None = None
    while reader.has_next():
        topic, data, timestamp_ns = reader.read_next()
        if topic != RC_REPLAY_TOPIC:
            continue
        first_timestamp_ns = int(timestamp_ns) if first_timestamp_ns is None else first_timestamp_ns
        samples.append(
            replay_sample_from_message(
                data,
                int(timestamp_ns),
                first_timestamp_ns,
                deserialize_message=deserialize_message,
                override_rc_in_type=override_rc_in_type,
            )
        )
    return samples


def replay_sample_from_message(
    data,
    timestamp_ns: int,
    first_timestamp_ns: int,
    *,
    deserialize_message,
    override_rc_in_type,
) -> RcReplaySample:
    msg = deserialize_message(data, override_rc_in_type)
    channels = padded_rc_channels(
        getattr(msg, "channels", []),
        target_count=RC_MESSAGE_CHANNEL_COUNT,
        sanitize_override_markers=False,
    )
    return RcReplaySample(
        time_s=(int(timestamp_ns) - int(first_timestamp_ns)) * 1e-9,
        channels=tuple(channels[:RC_MESSAGE_CHANNEL_COUNT]),
    )


__all__ = ["decode_replay_samples", "replay_sample_from_message"]
