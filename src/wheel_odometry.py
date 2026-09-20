"""Encoder calibration, measurements and independent raw baseline."""

from __future__ import annotations

import math
import re
from collections.abc import Iterable, Iterator
from dataclasses import dataclass
from pathlib import Path

from .models import EncoderSample, Estimate, WheelMeasurement


@dataclass(frozen=True)
class EncoderCalibration:
    resolution: int
    left_diameter_m: float
    right_diameter_m: float
    wheel_base_m: float


def read_encoder_calibration(path: Path) -> EncoderCalibration:
    text = path.read_text(encoding="utf-8")
    labels = {
        "resolution": r"Encoder resolution:\s*(\d+)",
        "left_diameter_m": r"Encoder left wheel diameter:\s*([\d.]+)",
        "right_diameter_m": r"Encoder right wheel diameter:\s*([\d.]+)",
        "wheel_base_m": r"Encoder wheel base:\s*([\d.]+)",
    }
    values: dict[str, float | int] = {}
    for name, pattern in labels.items():
        match = re.search(pattern, text)
        if match is None:
            raise ValueError(f"{path}: missing {name}")
        values[name] = int(match.group(1)) if name == "resolution" else float(match.group(1))
        if values[name] <= 0:
            raise ValueError(f"{path}: {name} must be positive")
    return EncoderCalibration(**values)


def wheel_measurements(
    samples: Iterable[EncoderSample], calibration_path: Path
) -> Iterator[WheelMeasurement]:
    """Differentiate cumulative counts using each actual timestamp interval."""
    calibration = read_encoder_calibration(calibration_path)
    previous: EncoderSample | None = None
    for sample in samples:
        if previous is not None:
            dt = (sample.timestamp_ns - previous.timestamp_ns) * 1e-9
            if dt <= 0:
                raise ValueError("Encoder timestamps must increase")
            left_delta = sample.left_count - previous.left_count
            right_delta = sample.right_count - previous.right_count
            if left_delta < 0 or right_delta < 0:
                raise ValueError("Encoder count decreased; rollover is not supported")
            left_speed = (
                left_delta * math.pi * calibration.left_diameter_m
                / calibration.resolution / dt
            )
            right_speed = (
                right_delta * math.pi * calibration.right_diameter_m
                / calibration.resolution / dt
            )
            speed = 0.5 * (left_speed + right_speed)
            if speed > 70:
                raise ValueError("Encoder speed exceeds 70 m/s; check counts and units")
            yield WheelMeasurement(
                sample.timestamp_ns,
                speed,
                (right_speed - left_speed) / calibration.wheel_base_m,
                left_speed,
                right_speed,
            )
        previous = sample


def wheel_only_baseline(measurements: Iterable[WheelMeasurement]) -> Iterator[Estimate]:
    """Integrate differential-wheel motion without IMU."""
    previous_ns: int | None = None
    x_m = y_m = yaw_rad = 0.0
    for measurement in measurements:
        if previous_ns is None:
            previous_ns = measurement.timestamp_ns
            yield Estimate(previous_ns, x_m, y_m, yaw_rad, measurement.speed_m_s)
            continue
        dt = (measurement.timestamp_ns - previous_ns) * 1e-9
        if dt <= 0:
            raise ValueError("Wheel timestamps must be strictly increasing")
        yaw_mid = yaw_rad + 0.5 * measurement.yaw_rate_rad_s * dt
        x_m += measurement.speed_m_s * math.cos(yaw_mid) * dt
        y_m += measurement.speed_m_s * math.sin(yaw_mid) * dt
        yaw_rad = math.atan2(
            math.sin(yaw_rad + measurement.yaw_rate_rad_s * dt),
            math.cos(yaw_rad + measurement.yaw_rate_rad_s * dt),
        )
        previous_ns = measurement.timestamp_ns
        yield Estimate(previous_ns, x_m, y_m, yaw_rad, measurement.speed_m_s)


def inject_right_wheel_scale_fault(
    measurements: Iterable[WheelMeasurement],
    *,
    wheel_base_m: float,
    start_ns: int,
    end_ns: int,
    scale: float,
) -> Iterator[WheelMeasurement]:
    """Scale the right wheel in a known interval for slip-detector evaluation."""
    if wheel_base_m <= 0 or scale <= 0 or end_ns <= start_ns:
        raise ValueError("Invalid injected wheel fault parameters")
    for measurement in measurements:
        if start_ns <= measurement.timestamp_ns < end_ns:
            right = measurement.right_speed_m_s * scale
            left = measurement.left_speed_m_s
            yield WheelMeasurement(
                measurement.timestamp_ns,
                0.5 * (left + right),
                (right - left) / wheel_base_m,
                left,
                right,
                True,
            )
        else:
            yield measurement
