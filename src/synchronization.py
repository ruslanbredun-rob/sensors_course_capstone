"""Time ordering for multi-rate sensor streams."""

from __future__ import annotations

from collections.abc import Iterable, Iterator

from .models import EncoderSample, ImuSample


def ordered_events(
    imu: Iterable[ImuSample], encoders: Iterable[EncoderSample]
) -> Iterator[ImuSample | EncoderSample]:
    """Merge streams by integer nanosecond timestamp; reject non-monotonic inputs."""
    raise NotImplementedError("Implement event merge and duplicate-time policy")
