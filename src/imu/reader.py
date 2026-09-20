"""Reader and sanity checks for the Xsens IMU stream."""

from __future__ import annotations

import math
from collections.abc import Iterator
from pathlib import Path

from src.common.models import ImuSample
from src.dataset.readers import checked_rows


def read_imu(dataset: Path) -> Iterator[ImuSample]:
    """Use gyro z (column 10) and acceleration x (column 11), zero-indexed.

    The vehicle-to-IMU calibration has identity rotation for urban35. Raw gyro
    values are treated as rad/s and accelerations as m/s²; no Euler angles are
    used as a pseudo-measurement.
    """
    path = dataset / "sensor_data" / "xsens_imu.csv"
    previous_ns = -1
    for line_number, row in checked_rows(path, 17):
        try:
            timestamp_ns = int(row[0])
            yaw_rate = float(row[10])
            forward_accel = float(row[11])
        except ValueError as exc:
            raise ValueError(f"{path}:{line_number}: invalid IMU value") from exc
        if timestamp_ns <= previous_ns:
            raise ValueError(f"{path}:{line_number}: timestamp is not increasing")
        if not math.isfinite(yaw_rate) or not math.isfinite(forward_accel):
            raise ValueError(f"{path}:{line_number}: non-finite IMU value")
        if abs(yaw_rate) > 20 or abs(forward_accel) > 100:
            raise ValueError(f"{path}:{line_number}: IMU value outside SI sanity range")
        previous_ns = timestamp_ns
        yield ImuSample(timestamp_ns, yaw_rate, forward_accel)
