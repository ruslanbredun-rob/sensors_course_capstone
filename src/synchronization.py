"""Time ordering for multi-rate sensor streams."""

from __future__ import annotations

from collections.abc import Iterable, Iterator

from .models import ImuSample, WheelMeasurement


def ordered_events(
    imu: Iterable[ImuSample], wheels: Iterable[WheelMeasurement]
) -> Iterator[ImuSample | WheelMeasurement]:
    """Merge monotonic streams; process IMU first when timestamps are equal."""
    imu_iter = iter(imu)
    wheel_iter = iter(wheels)
    next_imu = next(imu_iter, None)
    next_wheel = next(wheel_iter, None)
    last_imu = -1
    last_wheel = -1
    while next_imu is not None or next_wheel is not None:
        if next_wheel is None or (
            next_imu is not None and next_imu.timestamp_ns <= next_wheel.timestamp_ns
        ):
            event = next_imu
            assert event is not None
            if event.timestamp_ns <= last_imu:
                raise ValueError("IMU timestamps are not strictly increasing")
            last_imu = event.timestamp_ns
            next_imu = next(imu_iter, None)
        else:
            event = next_wheel
            assert event is not None
            if event.timestamp_ns <= last_wheel:
                raise ValueError("Wheel timestamps are not strictly increasing")
            last_wheel = event.timestamp_ns
            next_wheel = next(wheel_iter, None)
        yield event
