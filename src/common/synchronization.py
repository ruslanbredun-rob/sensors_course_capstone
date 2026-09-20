"""Time ordering for multi-rate sensor streams."""

from __future__ import annotations

import heapq
from collections.abc import Iterable, Iterator

from .models import ImuSample, RelativeMotion, RelativePoseEpoch, WheelMeasurement


SensorEvent = ImuSample | WheelMeasurement | RelativeMotion | RelativePoseEpoch


def ordered_sensor_events(*streams: Iterable[SensorEvent]) -> Iterator[SensorEvent]:
    """Merge monotonic streams, preserving argument order for equal timestamps."""
    iterators = [iter(stream) for stream in streams]
    heap: list[tuple[int, int, SensorEvent]] = []
    last_timestamp = [-1] * len(iterators)
    for priority, iterator in enumerate(iterators):
        event = next(iterator, None)
        if event is not None:
            heapq.heappush(heap, (event.timestamp_ns, priority, event))
    while heap:
        _, priority, event = heapq.heappop(heap)
        if event.timestamp_ns <= last_timestamp[priority]:
            raise ValueError(f"Sensor stream {priority} is not strictly increasing")
        last_timestamp[priority] = event.timestamp_ns
        yield event
        following = next(iterators[priority], None)
        if following is not None:
            heapq.heappush(heap, (following.timestamp_ns, priority, following))


def ordered_events(
    imu: Iterable[ImuSample], wheels: Iterable[WheelMeasurement]
) -> Iterator[ImuSample | WheelMeasurement]:
    """Merge monotonic streams; process IMU first when timestamps are equal."""
    yield from ordered_sensor_events(imu, wheels)
