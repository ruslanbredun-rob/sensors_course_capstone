"""Readers and calibration for the headerless Complex Urban files."""

from __future__ import annotations

from collections.abc import Iterator
from pathlib import Path

from .models import EncoderSample, ImuSample, PositionReference


def read_encoders(dataset: Path) -> Iterator[EncoderSample]:
    """Read timestamp and cumulative left/right counts from encoder.csv."""
    raise NotImplementedError("Confirm column order, count rollover and units first")


def read_imu(dataset: Path) -> Iterator[ImuSample]:
    """Map raw Xsens columns to vehicle-frame yaw rate and forward acceleration."""
    raise NotImplementedError("Confirm Xsens CSV schema and IMU frame first")


def read_vrs_reference(dataset: Path) -> Iterator[PositionReference]:
    """Read projected VRS position; never pass these samples to the EKF."""
    raise NotImplementedError("Confirm VRS coordinate frame and fix column first")
