"""Encoder calibration, measurements and independent raw baseline."""

from __future__ import annotations

from collections.abc import Iterable, Iterator
from pathlib import Path

from .models import EncoderSample, Estimate, WheelMeasurement


def wheel_measurements(
    samples: Iterable[EncoderSample], calibration_path: Path
) -> Iterator[WheelMeasurement]:
    """Convert count differences to m/s and rad/s using EncoderParameter.txt."""
    raise NotImplementedError("Implement calibration parsing and count differencing")


def wheel_only_baseline(measurements: Iterable[WheelMeasurement]) -> Iterator[Estimate]:
    """Integrate wheel motion without IMU; used as the raw baseline for homework 19."""
    raise NotImplementedError("Implement independent wheel-only integration")
