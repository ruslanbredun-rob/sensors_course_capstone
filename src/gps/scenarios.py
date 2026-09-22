"""Deterministic commercial GPS availability scenarios."""

from __future__ import annotations

import numpy as np

from src.common.config import GpsConfig
from src.common.models import GpsMeasurement, WheelMeasurement


def _wheel_progress(
    measurements: list[GpsMeasurement], wheels: list[WheelMeasurement]
) -> np.ndarray:
    if len(wheels) < 2:
        raise ValueError("At least two wheel measurements are required")
    wheel_times = np.array([sample.timestamp_ns for sample in wheels], dtype=np.int64)
    speed = np.abs([sample.speed_m_s for sample in wheels])
    distance = np.zeros(len(wheels), dtype=float)
    distance[1:] = np.cumsum(
        0.5 * (speed[:-1] + speed[1:]) * np.diff(wheel_times) * 1e-9
    )
    gps_times = np.array(
        [measurement.timestamp_ns for measurement in measurements], dtype=np.int64
    )
    gps_distance = np.interp(gps_times, wheel_times, distance)
    span = gps_distance[-1] - gps_distance[0]
    if span <= 0.0:
        raise ValueError("Wheel trajectory has no traveled distance")
    return (gps_distance - gps_distance[0]) / span


def gps_scenarios(
    measurements: list[GpsMeasurement],
    wheels: list[WheelMeasurement],
    config: GpsConfig,
) -> dict[str, list[GpsMeasurement]]:
    """Return full, route-distance dropout and configured sparse streams."""
    if not measurements:
        raise ValueError("GPS stream is empty")
    progress = _wheel_progress(measurements, wheels)
    dropout = [
        measurement
        for measurement, fraction in zip(measurements, progress)
        if not any(start <= fraction < end for start, end in config.dropout_ranges)
    ]
    sparse: list[GpsMeasurement] = []
    interval_ns = round(config.sparse_interval_s * 1e9)
    next_timestamp_ns = measurements[0].timestamp_ns
    for measurement in measurements:
        if measurement.timestamp_ns < next_timestamp_ns:
            continue
        sparse.append(measurement)
        next_timestamp_ns = measurement.timestamp_ns + interval_ns
    return {
        "gps": measurements,
        "gps_dropout": dropout,
        "gps_sparse": sparse,
    }
